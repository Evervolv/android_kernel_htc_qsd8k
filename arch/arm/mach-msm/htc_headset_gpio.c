/*
 *
 * /arch/arm/mach-msm/htc_headset_gpio.c
 *
 *  HTC GPIO headset detection driver.
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

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <mach/board-htcleo-microp.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>

#define DRIVER_NAME "HS_GPIO"

/* #define DEBUG */

#ifdef DEBUG
#define AJ_DBG(fmt, arg...) \
	printk(KERN_INFO "[Audio Jack] %s " fmt "\r\n", __func__, ## arg)
#else
#define AJ_DBG(fmt, arg...) do {} while (0)
#endif

int microp_get_remote_adc(uint32_t *val);
int microp_set_adc_req(uint8_t channel);

struct audio_jack_info {
	unsigned int irq_jack;
	unsigned int irq_mic;
	int audio_jack_detect;
	int key_enable_gpio;
	int mic_select_gpio;
	int audio_jack_flag;
	int mic_detect;
	int last_pressed_key;
	int microp_channel;

	struct hrtimer detection_timer;
	ktime_t debounce_time;

	struct work_struct work;
	struct work_struct mic_work;
	spinlock_t spin_lock;

	struct wake_lock audiojack_wake_lock;
};

static struct audio_jack_info *pjack_info;

int microp_set_adc_req(uint8_t value)
{
	int ret;
	uint8_t cmd[1];

	cmd[0] = value; //value; TODO finish code... now only keys ADC
	ret = microp_i2c_write(MICROP_I2C_WCMD_ADC_REQ, cmd, 1);
	if (ret < 0) 
	{
		pr_err("%s: request adc fail\n", __func__);
		return -EIO;
	}

	return 0;
}

int microp_get_remote_adc(uint32_t *val)
{
	int ret;
	uint8_t data[4];

	if (!val)
		return -EIO; 

	ret = microp_i2c_read(MICROP_I2C_RCMD_ADC_VALUE, data, 2);
	if (ret < 0) 
	{
		pr_err("%s: request adc fail\n", __func__);
		return -EIO;
	}

//	printk("%x %x\n", data[0], data[1]);
	*val = data[1] | (data[0] << 8);
	printk("remote adc %d\n", *val);
	return 0;
}

static int hs_gpio_get_mic(void)
{
	int value;
	value = !gpio_get_value(pjack_info->mic_detect);
	
	printk("hs_gpio_get_mic: %d\n", value);
	return value;
}

static int hs_enable_key_irq(int status)
{
	printk("hs_enable_key_irq: %d\n", status);
	if(status)
		enable_irq(pjack_info->irq_mic);
	else 
		disable_irq(pjack_info->irq_mic);
	return 0;
}

void hs_gpio_key_enable(int enable)
{
	DBG_MSG();

	if (pjack_info->key_enable_gpio)
		gpio_set_value(pjack_info->key_enable_gpio, enable);
}

void hs_gpio_mic_select(int enable)
{
	DBG_MSG();

	if (pjack_info->mic_select_gpio)
		gpio_set_value(pjack_info->mic_select_gpio, enable);
}

static int get_remote_keycode(int *keycode)
{
	uint32_t val;
	uint32_t btn = 0;

	microp_set_adc_req(pjack_info->microp_channel);
	if (microp_get_remote_adc(&val))
	{
		// failed. who know why? ignore
		*keycode = 0;
		return 1;
	}

	if((val >= 0) && (val <= 33))
	{
		btn = 1;
	}
	else if((val >= 38) && (val <= 82))
	{
		btn = 2;
	}
	else if((val >= 95) && (val <= 200))
	{
		btn = 3;
	}
	else if(val > 200)
	{   
		// check previous key
		if (pjack_info->last_pressed_key)
		{
			*keycode = pjack_info->last_pressed_key | 0x80;
			pjack_info->last_pressed_key = 0;
			return 0;
		}
		*keycode = 0;
		return 1;
	}

	pjack_info->last_pressed_key = btn;
	*keycode = btn;
	return 0;
}

static irqreturn_t mic_irq_handler(int irq, void *dev_id)
{
	pr_info("MIC IRQ Handler\n");
	int value1, value2;
	int retry_limit = 10;

	AJ_DBG("");

	do {
		value1 = gpio_get_value(pjack_info->mic_detect);
		irq_set_irq_type(pjack_info->irq_mic, value1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(pjack_info->mic_detect);
	} while (value1 != value2 && retry_limit-- > 0);

	AJ_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	schedule_work(&pjack_info->mic_work);
	return IRQ_HANDLED;
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	pr_info("DET IRQ Handler\n");
	int value1, value2;
	int retry_limit = 10;

	hs_notify_hpin_irq();

	AJ_DBG("");

	do {
		value1 = gpio_get_value(pjack_info->audio_jack_detect);
		irq_set_irq_type(pjack_info->irq_jack, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(pjack_info->audio_jack_detect);
	} while (value1 != value2 && retry_limit-- > 0);

	AJ_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	if ((pjack_info->audio_jack_flag == 0) ^ value2) {
		wake_lock_timeout(&pjack_info->audiojack_wake_lock, 4*HZ);

		/* Do the rest of the work in timer context */
		hrtimer_start(&pjack_info->detection_timer,
		pjack_info->debounce_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart detect_35mm_event_timer_func(struct hrtimer *data)
{
	int state;

	AJ_DBG("");
	state = !gpio_get_value(pjack_info->audio_jack_detect);
	if (pjack_info->audio_jack_flag != state) {
		pjack_info->audio_jack_flag = state;
		schedule_work(&pjack_info->work);
	}

	return HRTIMER_NORESTART;
}

static void mic_work_func(struct work_struct *work)
{
	pr_info("MIC Schedule Work\n");
	int keycode = 0;

	printk("mic_intr_work_func\n");
	if (get_remote_keycode(&keycode) == 0) 
	{
		printk("keycode %d\n", keycode);
		htc_35mm_remote_notify_button_status(keycode);
	}
	else
		printk("mic error keycode\n");
	  
}

static void audiojack_work_func(struct work_struct *work)
{
	int is_insert;
	pr_info("DET Schedule Work\n");
	unsigned long flags = 0;

	spin_lock_irqsave(&pjack_info->spin_lock, flags);
	is_insert = pjack_info->audio_jack_flag;
	spin_unlock_irqrestore(&pjack_info->spin_lock, flags);

	htc_35mm_remote_notify_insert_ext_headset(is_insert);

	if (is_insert)
		pjack_info->debounce_time = ktime_set(0, 200000000);
	else
		pjack_info->debounce_time = ktime_set(0, 500000000);
}

static void hs_gpio_register(void)
{
	struct headset_notifier notifier;

	if (pjack_info->mic_select_gpio) {
		notifier.id = HEADSET_REG_MIC_SELECT;
		notifier.func = hs_gpio_mic_select;
		headset_notifier_register(&notifier);
	}

	if (pjack_info->key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_gpio_key_enable;
		headset_notifier_register(&notifier);
	}
	
	if (pjack_info->mic_detect) {
		notifier.id = HEADSET_REG_MIC_STATUS;
		notifier.func = hs_gpio_get_mic;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_KEY_INT_ENABLE;
		notifier.func = hs_enable_key_irq;
		headset_notifier_register(&notifier);
	}
}

static int audiojack_probe(struct platform_device *pdev)
{
	int ret;
	struct htc_headset_gpio_platform_data *pdata = pdev->dev.platform_data;

	SYS_MSG("++++++++++++++++++++");

	pjack_info = kzalloc(sizeof(struct audio_jack_info), GFP_KERNEL);
	if (!pjack_info)
		return -ENOMEM;

	pjack_info->audio_jack_detect = pdata->hpin_gpio;
	pjack_info->key_enable_gpio = pdata->key_enable_gpio;
	pjack_info->mic_select_gpio = pdata->mic_select_gpio;
	pjack_info->mic_detect = pdata->mic_detect_gpio;
	pjack_info->microp_channel = pdata->microp_channel;
	pjack_info->audio_jack_flag = 0;
	pjack_info->last_pressed_key = 0;

	pjack_info->debounce_time = ktime_set(0, 500000000);
	hrtimer_init(&pjack_info->detection_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pjack_info->detection_timer.function = detect_35mm_event_timer_func;

	INIT_WORK(&pjack_info->work, audiojack_work_func);
	INIT_WORK(&pjack_info->mic_work, mic_work_func);

	spin_lock_init(&pjack_info->spin_lock);
	wake_lock_init(&pjack_info->audiojack_wake_lock,
		WAKE_LOCK_SUSPEND, "audiojack");

	if (pjack_info->audio_jack_detect) {
		ret = gpio_request(pjack_info->audio_jack_detect,
				   "3.5mm_detect");
		if (ret < 0)
			goto err_request_detect_gpio;

		ret = gpio_direction_input(pjack_info->audio_jack_detect);
		if (ret < 0)
			goto err_set_detect_gpio;

		pjack_info->irq_jack =
			gpio_to_irq(pjack_info->audio_jack_detect);
		if (pjack_info->irq_jack < 0) {
			ret = pjack_info->irq_jack;
			goto err_request_detect_irq;
		}

		ret = request_irq(pjack_info->irq_jack,
				  detect_irq_handler,
				  IRQF_TRIGGER_LOW, "35mm_headset", NULL);
		if (ret < 0)
			goto err_request_detect_irq;

		ret = irq_set_irq_wake(pjack_info->irq_jack, 1);
		if (ret < 0)
			goto err_set_irq_wake;
		pr_info("DET IRQ Registered!");
	}


	if (pjack_info->mic_detect) {
		ret = gpio_request(pjack_info->mic_detect,
				   "mic_detect");
		if (ret < 0)
			goto err_request_detect_gpio;

		ret = gpio_direction_input(pjack_info->mic_detect);
		if (ret < 0)
			goto err_set_detect_gpio;

		pjack_info->irq_mic =
			gpio_to_irq(pjack_info->mic_detect);
		if (pjack_info->irq_mic < 0) {
			ret = pjack_info->irq_mic;
			goto err_request_detect_irq;
		}

		ret = request_irq(pjack_info->irq_mic,
				  mic_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_LOW, "mic_headset", NULL);
		if (ret < 0)
			goto err_request_detect_irq;

		disable_irq(pjack_info->irq_mic);
		pr_info("MIC IRQ Registered!");
	}
	hs_gpio_register();

	SYS_MSG("--------------------");

	return 0;

err_set_irq_wake:
	if (pjack_info->audio_jack_detect)
		free_irq(pjack_info->irq_jack, 0);
err_request_detect_irq:
err_set_detect_gpio:
	if (pjack_info->audio_jack_detect)
		gpio_free(pjack_info->audio_jack_detect);
err_request_detect_gpio:
	printk(KERN_ERR "Audiojack: Failed in audiojack_probe\n");

	return ret;
}

static int audiojack_remove(struct platform_device *pdev)
{
	if (pjack_info->audio_jack_detect)
		free_irq(pjack_info->irq_jack, 0);

	if (pjack_info->audio_jack_detect)
		free_irq(pjack_info->irq_mic, 0);

	if (pjack_info->audio_jack_detect)
		gpio_free(pjack_info->audio_jack_detect);

	if (pjack_info->audio_jack_detect)
		gpio_free(pjack_info->mic_detect);

	return 0;
}

static struct platform_driver audiojack_driver = {
	.probe		= audiojack_probe,
	.remove		= audiojack_remove,
	.driver		= {
		.name		= "HTC_HEADSET_GPIO",
		.owner		= THIS_MODULE,
	},
};

static int __init audiojack_init(void)
{
	return platform_driver_register(&audiojack_driver);
}

static void __exit audiojack_exit(void)
{
	platform_driver_unregister(&audiojack_driver);
}

module_init(audiojack_init);
module_exit(audiojack_exit);

MODULE_DESCRIPTION("HTC GPIO headset detection driver");
MODULE_LICENSE("GPL");
