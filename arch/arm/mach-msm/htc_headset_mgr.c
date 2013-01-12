/*
 *
 * /arch/arm/mach-msm/htc_headset_mgr.c
 *
 *  HTC headset manager driver.
 *
 *  Copyright (C) 2010 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <asm/atomic.h>
#include <asm/mach-types.h>

#include <mach/board.h>
#include <mach/vreg.h>
#include <mach/board-htcleo-microp.h>

#include <mach/htc_headset_mgr.h>

#define DRIVER_NAME "HS_MGR"

/* #define CONFIG_DEBUG_H2W */

/*Delay 200ms when 11pin device plug in*/
#define H2W_DETECT_DELAY	msecs_to_jiffies(200)
#define BUTTON_H2W_DELAY	msecs_to_jiffies(10)

#define H2WI(fmt, arg...) \
	printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#define H2WE(fmt, arg...) \
	printk(KERN_ERR "[H2W] %s " fmt "\r\n", __func__, ## arg)

#ifdef CONFIG_DEBUG_H2W
#define H2W_DBG(fmt, arg...) \
	printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#else
#define H2W_DBG(fmt, arg...) do {} while (0)
#endif

static struct workqueue_struct *detect_wq;

static void insert_35mm_do_work(struct work_struct *work);
static DECLARE_WORK(insert_35mm_work, insert_35mm_do_work);
static void remove_35mm_do_work(struct work_struct *work);
static DECLARE_WORK(remove_35mm_work, remove_35mm_do_work);

static struct workqueue_struct *button_wq;

static void button_35mm_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(button_35mm_work, button_35mm_do_work);

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len);

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_RPC_SERVER_PROG,
	.vers		= HS_RPC_SERVER_VERS,
	.rpc_call	= hs_mgr_rpc_call,
};

struct button_work {
	struct delayed_work key_work;
	int key_code;
};

static struct h2w_info *hi;
static struct hs_notifier_func hs_mgr_notifier;

void hs_notify_hpin_irq(void)
{
	hi->hpin_jiffies = jiffies;
	SYS_MSG("HPIN IRQ");
}

int hs_hpin_stable(void)
{
	unsigned long last_hpin_jiffies = 0;
	unsigned long unstable_jiffies = 1.2 * HZ;

	last_hpin_jiffies = hi->hpin_jiffies;

	if (time_before_eq(jiffies, last_hpin_jiffies + unstable_jiffies))
		return 0;

	return 1;
}

int headset_notifier_register(struct headset_notifier *notifier)
{
	if (!notifier->func) {
		SYS_MSG("NULL register function");
		return 0;
	}

	switch (notifier->id) {
	case HEADSET_REG_REMOTE_ADC:
		SYS_MSG("Register REMOTE_ADC notifier");
		hs_mgr_notifier.remote_adc = notifier->func;
		break;
	case HEADSET_REG_RPC_KEY:
		SYS_MSG("Register RPC_KEY notifier");
		hs_mgr_notifier.rpc_key = notifier->func;
		break;
	case HEADSET_REG_MIC_STATUS:
		SYS_MSG("Register MIC_STATUS notifier");
		hs_mgr_notifier.mic_status = notifier->func;
		break;
	case HEADSET_REG_MIC_BIAS:
		SYS_MSG("Register MIC_BIAS notifier");
		hs_mgr_notifier.mic_bias_enable = notifier->func;
		break;
	case HEADSET_REG_MIC_SELECT:
		SYS_MSG("Register MIC_SELECT notifier");
		hs_mgr_notifier.mic_select = notifier->func;
		break;
	case HEADSET_REG_KEY_INT_ENABLE:
		SYS_MSG("Register KEY_INT_ENABLE notifier");
		hs_mgr_notifier.key_int_enable = notifier->func;
		break;
	case HEADSET_REG_KEY_ENABLE:
		SYS_MSG("Register KEY_ENABLE notifier");
		hs_mgr_notifier.key_enable = notifier->func;
		break;
	default:
		SYS_MSG("Unknown register ID");
		return 0;
	}

	return 1;
}

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len)
{
	struct hs_rpc_server_args_key *args_key;

	DBG_MSG("");

	switch (req->procedure) {
	case HS_RPC_SERVER_PROC_NULL:
		SYS_MSG("RPC_SERVER_NULL");
		break;
	case HS_RPC_SERVER_PROC_KEY:
		args_key = (struct hs_rpc_server_args_key *)(req + 1);
		args_key->adc = be32_to_cpu(args_key->adc);
		SYS_MSG("RPC_SERVER_KEY ADC = %u (0x%X)",
			args_key->adc, args_key->adc);
		if (hs_mgr_notifier.rpc_key)
			hs_mgr_notifier.rpc_key(args_key->adc);
		else
			SYS_MSG("RPC_KEY notify function doesn't exist");
		break;
	default:
		SYS_MSG("Unknown RPC procedure");
		return -EINVAL;
	}

	return 0;
}

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

void button_pressed(int type)
{
	printk(KERN_INFO "[H2W] button_pressed %d\n", type);
	atomic_set(&hi->btn_state, type);
	input_report_key(hi->input, type, 1);
	input_sync(hi->input);
}

void button_released(int type)
{
	printk(KERN_INFO "[H2W] button_released %d\n", type);
	atomic_set(&hi->btn_state, 0);
	input_report_key(hi->input, type, 0);
	input_sync(hi->input);
}

void headset_button_event(int is_press, int type)
{
	if (!hs_hpin_stable()) {
		H2WI("The HPIN is unstable, SKIP THE BUTTON EVENT.");
		return;
	}

	if (!is_press) {
		if (hi->ignore_btn)
			hi->ignore_btn = 0;
		else
			button_released(type);
	} else {
		if (!hi->ignore_btn && !atomic_read(&hi->btn_state))
			button_pressed(type);
	}
}

static void set_35mm_hw_state(int state)
{
	if (hi->mic_bias_state != state && hs_mgr_notifier.mic_bias_enable) {
		hs_mgr_notifier.mic_bias_enable(state);
		hi->mic_bias_state = state;
		if (state) /* Wait for MIC bias stable */
			msleep(HS_DELAY_MIC_BIAS);
	}

	if (hs_mgr_notifier.mic_select)
		hs_mgr_notifier.mic_select(state);

	if (hs_mgr_notifier.key_enable)
		hs_mgr_notifier.key_enable(state);

	if (hs_mgr_notifier.key_int_enable)
		hs_mgr_notifier.key_int_enable(state);
}

static void insert_11pin_35mm(int *state)
{
	int mic = HEADSET_NO_MIC;

	SYS_MSG("Insert USB 3.5mm headset");
	set_35mm_hw_state(1);

	if (hs_mgr_notifier.mic_status) {
		mic = hs_mgr_notifier.mic_status();
	}

	if (mic == HEADSET_NO_MIC) {
		/* without microphone */
		*state |= BIT_HEADSET_NO_MIC;
		hi->h2w_35mm_status = HTC_35MM_NO_MIC;
		printk(KERN_INFO "11pin_3.5mm without microphone\n");
	} else { /* with microphone */
		*state |= BIT_HEADSET;
		hi->h2w_35mm_status = HTC_35MM_MIC;
		printk(KERN_INFO "11pin_3.5mm with microphone\n");
	}
}

static void remove_11pin_35mm(void)
{
	SYS_MSG("Remove USB 3.5mm headset");

	set_35mm_hw_state(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->h2w_35mm_status = HTC_35MM_UNPLUG;
}

static void button_35mm_do_work(struct work_struct *w)
{
	int key;
	int pressed;
	struct button_work *work;

	work = container_of(w, struct button_work, key_work.work);
	hi->key_level_flag = work->key_code;

	if (!hi->is_ext_insert && !hi->h2w_35mm_status) {
		kfree(work);
		H2WI("3.5mm headset is plugged out, skip report key event");
		return;
	}

	if (hi->key_level_flag) {
		pressed = (hi->key_level_flag & 0x80) ? 0:1;
		switch (hi->key_level_flag & 0x7f) {
		case 1:
			H2WI("3.5mm RC: Play(%d)", pressed);
			key = HS_MGR_KEYCODE_MEDIA;
			break;
		case 2:
			H2WI("3.5mm RC: BACKWARD(%d)", pressed);
			key = HS_MGR_KEYCODE_BACKWARD;
			break;
		case 3:
			H2WI("3.5mm RC: FORWARD(%d)", pressed);
			key = HS_MGR_KEYCODE_FORWARD;
			break;
		default:
			H2WI("3.5mm RC: WRONG Button Pressed (%d)", hi->key_level_flag);
			return;
		}
		headset_button_event(pressed, key);
	}
	wake_lock_timeout(&hi->headset_wake_lock, 1.5 * HZ);

	kfree(work);
}

static void enable_metrico_headset(int enable)
{
	if (enable && !hi->metrico_status) {
#if 0
		enable_mos_test(1);
#endif
		hi->metrico_status = 1;
		printk(KERN_INFO "Enable metrico headset\n");
	}

	if (!enable && hi->metrico_status) {
#if 0
		enable_mos_test(0);
#endif
		hi->metrico_status = 0;
		printk(KERN_INFO "Disable metrico headset\n");
	}
}

static void remove_35mm_do_work(struct work_struct *work)
{
	int state;

	wake_lock_timeout(&hi->headset_wake_lock, 2.5 * HZ);

	H2W_DBG("");
	/*To solve the insert, remove, insert headset problem*/
	if (time_before_eq(jiffies, hi->insert_jiffies))
		msleep(800);
	if (hi->is_ext_insert) {
		H2WI("Skip 3.5mm headset plug out!!!");
		return;
	}

	SYS_MSG("Remove 3.5mm headset");
	set_35mm_hw_state(0);

	/* For HW Metrico lab test */
	if (hi->metrico_status)
		enable_metrico_headset(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->ext_35mm_status = HTC_35MM_UNPLUG;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);

	if (hi->usb_dev_type == USB_HEADSET) {
		hi->usb_dev_status = STATUS_CONNECTED_ENABLED;
		state &= ~(BIT_35MM_HEADSET | BIT_HEADSET);
		state |= BIT_HEADSET_NO_MIC;
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
	} else if (hi->usb_dev_type == H2W_TVOUT) {
		state &= ~(BIT_HEADSET | BIT_35MM_HEADSET);
		state |= BIT_HEADSET_NO_MIC;
		switch_set_state(&hi->sdev, state);
#if 0
	} else if (hi->cable_in1 && !gpio_get_value(hi->cable_in1)) {
		state &= ~BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
		queue_delayed_work(detect_wq, &detect_h2w_work,
				   HS_DELAY_ZERO_JIFFIES);
#endif
	} else {
		state &= ~(BIT_HEADSET | BIT_HEADSET_NO_MIC |
			BIT_35MM_HEADSET);
		switch_set_state(&hi->sdev, state);
	}

	mutex_unlock(&hi->mutex_lock);
}

static void insert_35mm_do_work(struct work_struct *work)
{
	int state;
	int i, mic1, mic2;

	H2W_DBG("");
	hi->insert_jiffies = jiffies + HZ;

	wake_lock_timeout(&hi->headset_wake_lock, 1.5 * HZ);

#if 0
	if (hi->usb_dev_type && hi->is_ext_insert &&
	    hi->usb_dev_type != H2W_TVOUT && hi->usb_dev_type != USB_HEADSET)
		remove_headset();
	else if (hi->usb_dev_type == USB_HEADSET)
		hi->usb_dev_status = STATUS_CONNECTED_DISABLED;
#endif

	if (hi->usb_dev_type == USB_HEADSET)
		hi->usb_dev_status = STATUS_CONNECTED_DISABLED;

	if (hi->is_ext_insert) {
		SYS_MSG("Insert 3.5mm headset");
		set_35mm_hw_state(1);
		hi->ignore_btn = 0;

		mic1 = mic2 = HEADSET_NO_MIC;
		if (hs_mgr_notifier.mic_status) {
			if (hi->ext_35mm_status == HTC_35MM_NO_MIC ||
				hi->h2w_35mm_status == HTC_35MM_NO_MIC)
				for (i = 0; i < 10; i++) {
					mic1 = hs_mgr_notifier.mic_status();
					msleep(HS_DELAY_MIC_DETECT);
					mic2 = hs_mgr_notifier.mic_status();
					if (mic1 == mic2)
						break;
				}
			else
				mic1 = mic2 = hs_mgr_notifier.mic_status();
		}

		/* For HW Metrico lab test */
		if (mic2 == HEADSET_METRICO && !hi->metrico_status)
			enable_metrico_headset(1);

		mutex_lock(&hi->mutex_lock);
		state = switch_get_state(&hi->sdev);
		state &= ~(BIT_HEADSET | BIT_HEADSET_NO_MIC);
		if (mic2 == HEADSET_NO_MIC || mic1 != mic2) {
			state |= BIT_HEADSET_NO_MIC;
			printk(KERN_INFO "3.5mm_headset without microphone\n");
		} else {
			state |= BIT_HEADSET;
			printk(KERN_INFO "3.5mm_headset with microphone\n");
		}

		state |= BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
		if (state & BIT_HEADSET_NO_MIC)
			hi->ext_35mm_status = HTC_35MM_NO_MIC;
		else
			hi->ext_35mm_status = HTC_35MM_MIC;
		mutex_unlock(&hi->mutex_lock);
	}
}

int htc_35mm_remote_notify_insert_ext_headset(int insert)
{
	if (hi) {
		mutex_lock(&hi->mutex_lock);
		hi->is_ext_insert = insert;
		mutex_unlock(&hi->mutex_lock);

		H2WI(" %d", hi->is_ext_insert);
		if (!hi->is_ext_insert)
			queue_work(detect_wq, &remove_35mm_work);
		else
			queue_work(detect_wq, &insert_35mm_work);
	}
	return 1;
}

int htc_35mm_remote_notify_microp_ready(void)
{
	if (hi) {
		if (hi->is_ext_insert)
			queue_work(detect_wq, &insert_35mm_work);
#if 0
		if (hi->h2w_35mm_status)
			insert_headset(NORMAL_HEARPHONE);
#endif
	}
	return 1;
}

int htc_35mm_remote_notify_button_status(int key_level)
{
	struct button_work *work;

	if (hi->ext_35mm_status == HTC_35MM_NO_MIC ||
		hi->h2w_35mm_status == HTC_35MM_NO_MIC) {
		SYS_MSG("MIC re-detection");
		msleep(HS_DELAY_MIC_DETECT);
		queue_work(detect_wq, &insert_35mm_work);
	} else if (!hs_hpin_stable()) {
		H2WI("The HPIN is unstable, SKIP THE BUTTON EVENT.");
		return 1;
	} else {
		work = kzalloc(sizeof(struct button_work), GFP_KERNEL);
		if (!work) {
			printk(KERN_INFO "Failed to allocate button memory\n");
			return 1;
		}
		work->key_code = key_level;
		INIT_DELAYED_WORK(&work->key_work, button_35mm_do_work);
		queue_delayed_work(button_wq, &work->key_work,
				   HS_JIFFIES_BUTTON);
	}

	return 1;
}

static void usb_headset_detect(int type)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);

	switch (type) {
	case NO_DEVICE:
		if (hi->usb_dev_type == USB_HEADSET) {
			printk(KERN_INFO "Remove USB headset\n");
			hi->usb_dev_type = NO_DEVICE;
			hi->usb_dev_status = STATUS_DISCONNECTED;
			state &= ~BIT_USB_HEADSET;
			if (!hi->is_ext_insert)
				state &= ~BIT_HEADSET_NO_MIC;
		}
		break;
	case USB_HEADSET:
		printk(KERN_INFO "Insert USB headset\n");
		hi->usb_dev_type = USB_HEADSET;
		if (hi->is_ext_insert) {
			printk(KERN_INFO "Disable USB headset\n");
			hi->usb_dev_status = STATUS_CONNECTED_DISABLED;
			state |= BIT_USB_HEADSET;
		} else {
			printk(KERN_INFO "Enable USB headset\n");
			hi->usb_dev_status = STATUS_CONNECTED_ENABLED;
			state |= (BIT_USB_HEADSET | BIT_HEADSET_NO_MIC);
		}
		break;
	default:
		printk(KERN_INFO "Unknown headset type\n");
	}

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
}

void headset_ext_detect(int type)
{
	switch (type) {
	case NO_DEVICE:
		if (hi->usb_dev_type == USB_HEADSET)
			usb_headset_detect(type);
		break;
	case USB_HEADSET:
		usb_headset_detect(type);
		break;
	default:
		printk(KERN_INFO "Unknown headset type\n");
	}
}

int switch_send_event(unsigned int bit, int on)
{
	unsigned long state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(bit);

	if (on)
		state |= bit;

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static ssize_t tty_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	mutex_lock(&hi->mutex_lock);
	s += sprintf(s, "%d\n", hi->tty_enable_flag);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t tty_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_TTY_FULL | BIT_TTY_VCO | BIT_TTY_HCO);

	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		hi->tty_enable_flag = 1;
		switch_set_state(&hi->sdev, state | BIT_TTY_FULL);
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY FULL\n");
		return count;
	}
	if (count == (strlen("vco_enable") + 1) &&
	   strncmp(buf, "vco_enable", strlen("vco_enable")) == 0) {
		hi->tty_enable_flag = 2;
		switch_set_state(&hi->sdev, state | BIT_TTY_VCO);
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY VCO\n");
		return count;
	}
	if (count == (strlen("hco_enable") + 1) &&
	   strncmp(buf, "hco_enable", strlen("hco_enable")) == 0) {
		hi->tty_enable_flag = 3;
		switch_set_state(&hi->sdev, state | BIT_TTY_HCO);
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY HCO\n");
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->tty_enable_flag = 0;
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Disable TTY\n");
		return count;
	}
	printk(KERN_ERR "tty_enable_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(tty, 0666, tty_flag_show, tty_flag_store);

static ssize_t fm_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_FM_HEADSET | BIT_FM_SPEAKER);

	if (count == (strlen("fm_headset") + 1) &&
	   strncmp(buf, "fm_headset", strlen("fm_headset")) == 0) {
		hi->fm_flag = 1;
		state |= BIT_FM_HEADSET;
		printk(KERN_INFO "Enable FM HEADSET\n");
	} else if (count == (strlen("fm_speaker") + 1) &&
	   strncmp(buf, "fm_speaker", strlen("fm_speaker")) == 0) {
		hi->fm_flag = 2;
		state |= BIT_FM_SPEAKER;
		printk(KERN_INFO "Enable FM SPEAKER\n");
	} else if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->fm_flag = 0 ;
		printk(KERN_INFO "Disable FM\n");
	} else {
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_ERR "fm_enable_flag_store: invalid argument\n");
		return -EINVAL;
	}
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return count;
}

static ssize_t fm_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	mutex_lock(&hi->mutex_lock);
	if (hi->fm_flag == 0)
		show_str = "disable";
	if (hi->fm_flag == 1)
		show_str = "fm_headset";
	if (hi->fm_flag == 2)
		show_str = "fm_speaker";

	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}
static DEVICE_ACCESSORY_ATTR(fm, 0666, fm_flag_show, fm_flag_store);

static int register_common_headset(struct h2w_info *h2w, int create_attr)
{
	int ret = 0;
	hi = h2w;

	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		goto err_create_class;
	}

	hi->tty_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		goto err_create_tty_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret)
		goto err_create_tty_device_file;

	hi->fm_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		goto err_create_fm_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret)
		goto err_create_fm_device_file;

	return 0;

err_create_fm_device_file:
	device_unregister(hi->fm_dev);
err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);
err_create_tty_device_file:
	device_unregister(hi->tty_dev);
err_create_tty_device:
	class_destroy(hi->htc_accessory_class);
err_create_class:

	return ret;
}

static void unregister_common_headset(struct h2w_info *h2w)
{
	hi = h2w;
	device_remove_file(hi->tty_dev, &dev_attr_tty);
	device_unregister(hi->tty_dev);
	device_remove_file(hi->fm_dev, &dev_attr_fm);
	device_unregister(hi->fm_dev);
	class_destroy(hi->htc_accessory_class);
}

static int htc_35mm_probe(struct platform_device *pdev)
{
	int ret;

	struct htc_headset_mgr_platform_data *pdata = pdev->dev.platform_data;

	SYS_MSG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	hi->driver_flag = pdata->driver_flag;
	hi->hpin_jiffies = jiffies;

	hi->ext_35mm_status = 0;
	hi->h2w_35mm_status = 0;
	hi->is_ext_insert = 0;
	hi->mic_bias_state = 0;
	hi->key_level_flag = -1;

	atomic_set(&hi->btn_state, 0);
	hi->ignore_btn = 0;
	hi->usb_dev_type = NO_DEVICE;
	hi->tty_enable_flag = 0;
	hi->fm_flag = 0;
	hi->mic_switch_flag = 1;
	hi->rc_flag = 0;

	hi->insert_11pin_35mm = insert_11pin_35mm;
	hi->remove_11pin_35mm = remove_11pin_35mm;

	mutex_init(&hi->mutex_lock);
	mutex_init(&hi->mutex_rc_lock);

	wake_lock_init(&hi->headset_wake_lock, WAKE_LOCK_SUSPEND, "headset");

	hi->sdev.name = "h2w";
	hi->sdev.print_name = h2w_print_name;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	detect_wq = create_workqueue("detection");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		goto err_create_detect_work_queue;
	}
	button_wq = create_workqueue("button");
	if (button_wq  == NULL) {
			ret = -ENOMEM;
			goto err_create_button_work_queue;
	}

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "h2w headset";
	set_bit(EV_SYN, hi->input->evbit);
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_END, hi->input->keybit);
	set_bit(KEY_MUTE, hi->input->keybit);
	set_bit(KEY_VOLUMEDOWN, hi->input->keybit);
	set_bit(KEY_VOLUMEUP, hi->input->keybit);
	set_bit(KEY_NEXTSONG, hi->input->keybit);
	set_bit(KEY_PLAYPAUSE, hi->input->keybit);
	set_bit(KEY_PREVIOUSSONG, hi->input->keybit);
	set_bit(KEY_MEDIA, hi->input->keybit);
	set_bit(KEY_SEND, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
	goto err_register_input_dev;

	ret = register_common_headset(hi, 0);
	if (ret)
		goto err_register_common_headset;

	if (hi->driver_flag & DRIVER_HS_MGR_RPC_SERVER) {
		/* Create RPC server */
		ret = msm_rpc_create_server(&hs_rpc_server);
		if (ret < 0) {
			SYS_MSG("Failed to create RPC server");
			goto err_create_rpc_server;
		}
		SYS_MSG("Create RPC server successfully");
	}

	SYS_MSG("--------------------");

	return 0;

err_create_rpc_server:

err_register_common_headset:
	input_unregister_device(hi->input);

err_register_input_dev:
	input_free_device(hi->input);

err_request_input_dev:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	switch_dev_unregister(&hi->sdev);

err_switch_dev_register:

	printk(KERN_ERR "H2W: Failed to register driver\n");

	return ret;
}

static int htc_35mm_remove(struct platform_device *pdev)
{
	H2W_DBG("");

#if 0
	if ((switch_get_state(&hi->sdev) &
		(BIT_HEADSET | BIT_HEADSET_NO_MIC)) != 0)
		remove_headset();
#endif

	unregister_common_headset(hi);
	input_unregister_device(hi->input);
	destroy_workqueue(detect_wq);
	destroy_workqueue(button_wq);
	switch_dev_unregister(&hi->sdev);

	return 0;
}

static struct platform_driver htc_35mm_driver = {
	.probe		= htc_35mm_probe,
	.remove		= htc_35mm_remove,
	.driver		= {
		.name		= "HTC_HEADSET_MGR",
		.owner		= THIS_MODULE,
	},
};


static int __init htc_35mm_init(void)
{
	H2W_DBG("");
	return platform_driver_register(&htc_35mm_driver);
}

static void __exit htc_35mm_exit(void)
{
	platform_driver_unregister(&htc_35mm_driver);
}

module_init(htc_35mm_init);
module_exit(htc_35mm_exit);

MODULE_DESCRIPTION("HTC headset manager driver");
MODULE_LICENSE("GPL");
