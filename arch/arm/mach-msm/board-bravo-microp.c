/* board-bravo-microp.c
 *
 * Copyright (C) 2009 Google.
 * Copyright (C) 2009 HTC Corporation.
 * Copyright (C) 2010 Giulio Cervera <giulio.cervera@gmail.com>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
 *
 * The Microp on bravo is an i2c device that supports
 * the following functions
 *   - LEDs (Green, Amber, Blue, Button-backlight)
 *   - Lightsensor
 *   - Headset & Remotekeys
 *   - G-sensor
 *   - Interrupts
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/htc_pwrsink.h>
#include <linux/earlysuspend.h>
#include <linux/bma150.h>
#include <linux/lightsensor.h>
#include <mach/mmc.h>
#include <mach/htc_35mm_jack.h>
//#include <asm/setup.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/irq.h>

#include <mach/board-bravo-microp-common.h>
#include "board-bravo.h"

#define READ_GPI_STATE_HPIN	(1<<2)
#define READ_GPI_STATE_SDCARD	(1<<0)

/*#define DEBUG_BMA150  */
#ifdef DEBUG_BMA150
/* Debug logging of accelleration data */
#define GSENSOR_LOG_MAX 2048  /* needs to be power of 2 */
#define GSENSOR_LOG_MASK (GSENSOR_LOG_MAX - 1)

struct gsensor_log {
	ktime_t timestamp;
	short x;
	short y;
	short z;
};

static DEFINE_MUTEX(gsensor_log_lock);
static struct gsensor_log gsensor_log[GSENSOR_LOG_MAX];
static unsigned gsensor_log_head;
static unsigned gsensor_log_tail;

void gsensor_log_status(ktime_t time, short x, short y, short z)
{
	unsigned n;
	mutex_lock(&gsensor_log_lock);
	n = gsensor_log_head;
	gsensor_log[n].timestamp = time;
	gsensor_log[n].x = x;
	gsensor_log[n].y = y;
	gsensor_log[n].z = z;
	n = (n + 1) & GSENSOR_LOG_MASK;
	if (n == gsensor_log_tail)
		gsensor_log_tail = (gsensor_log_tail + 1) & GSENSOR_LOG_MASK;
	gsensor_log_head = n;
	mutex_unlock(&gsensor_log_lock);
}

static int gsensor_log_print(struct seq_file *sf, void *private)
{
	unsigned n;

	mutex_lock(&gsensor_log_lock);
	seq_printf(sf, "timestamp                  X      Y      Z\n");
	for (n = gsensor_log_tail;
	     n != gsensor_log_head;
	     n = (n + 1) & GSENSOR_LOG_MASK) {
		seq_printf(sf, "%10d.%010d %6d %6d %6d\n",
			   gsensor_log[n].timestamp.tv.sec,
			   gsensor_log[n].timestamp.tv.nsec,
			   gsensor_log[n].x, gsensor_log[n].y,
			   gsensor_log[n].z);
	}
	mutex_unlock(&gsensor_log_lock);
	return 0;
}

static int gsensor_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, gsensor_log_print, NULL);
}

static struct file_operations gsensor_log_fops = {
	.open = gsensor_log_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif /* def DEBUG_BMA150 */

static struct mutex gsensor_RW_mutex;
static struct mutex gsensor_set_mode_mutex;

static int microp_headset_has_mic(void);
static int microp_enable_headset_plug_event(void);
static int microp_enable_key_event(void);
static int microp_disable_key_event(void);

static struct h35mm_platform_data bravo_h35mm_data = {
	.plug_event_enable = microp_enable_headset_plug_event,
	.headset_has_mic = microp_headset_has_mic,
	.key_event_enable = microp_enable_key_event,
	.key_event_disable = microp_disable_key_event,
};

static struct platform_device bravo_h35mm = {
	.name	= "htc_headset",
	.id	= -1,
	.dev	= {
		.platform_data = &bravo_h35mm_data,
	},
};

enum led_type {
	GREEN_LED,
	AMBER_LED,
	BLUE_LED,
	BUTTONS_LED,
	NUM_LEDS,
};

static uint16_t remote_key_adc_table[6] = {
	0, 33, 43, 110, 129, 220
};

static struct wake_lock microp_i2c_wakelock;

static struct i2c_client *private_microp_client;

struct microp_int_pin {
	uint16_t int_gsensor;
	uint16_t int_lsensor;
	uint16_t int_reset;
	uint16_t int_simcard;
	uint16_t int_hpin;
	uint16_t int_remotekey;
};

struct microp_led_data {
	int type;
	struct led_classdev ldev;
	struct mutex led_data_mutex;
	struct work_struct brightness_work;
	spinlock_t brightness_lock;
	enum led_brightness brightness;
	uint8_t mode;
	uint8_t blink;
};

struct microp_i2c_work {
	struct work_struct work;
	struct i2c_client *client;
	int (*intr_debounce)(uint8_t *pin_status);
	void (*intr_function)(uint8_t *pin_status);
};

struct microp_i2c_client_data {
	struct microp_led_data leds[NUM_LEDS];
	uint16_t version;
	struct microp_i2c_work work;
	struct delayed_work hpin_debounce_work;
	struct delayed_work ls_read_work;
	struct early_suspend early_suspend;
	uint8_t enable_early_suspend;
	uint8_t enable_reset_button;
	int microp_is_suspend;
	int auto_backlight_enabled;
	uint8_t button_led_value;
	int headset_is_in;
	int is_hpin_pin_stable;
	uint32_t spi_devices_vote;
	uint32_t spi_devices;
	struct mutex microp_i2c_rw_mutex;
	struct mutex microp_adc_mutex;
	struct hrtimer gen_irq_timer;
	uint16_t intr_status;
};

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

#define I2C_READ_RETRY_TIMES  10
#define I2C_WRITE_RETRY_TIMES 10
#define MICROP_I2C_WRITE_BLOCK_SIZE 80

static int i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	struct microp_i2c_client_data *cdata;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->microp_i2c_rw_mutex);
	mdelay(1);
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		msleep(10);
	}
	mutex_unlock(&cdata->microp_i2c_rw_mutex);
	dev_dbg(&client->dev, "R [%02X] = %s\n",
			addr, hex2string(data, length));

	if (retry > I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct microp_i2c_client_data *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->microp_i2c_rw_mutex);
	mdelay(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}
	if (retry > I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->microp_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->microp_i2c_rw_mutex);

	return 0;
}

struct i2c_client *get_microp_client(void)
{
	return private_microp_client;
}

int microp_i2c_read(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_read_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_read);

int microp_i2c_write(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_write_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_write);

int microp_read_adc(uint8_t channel, uint16_t *value)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret;
	uint8_t cmd[2], data[2];

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);
	cmd[0] = 0;
	cmd[1] = channel;
	mutex_lock(&cdata->microp_adc_mutex);
	ret = i2c_write_block(client, MICROP_I2C_WCMD_READ_ADC_VALUE_REQ,
			      cmd, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: request adc fail\n", __func__);
		mutex_unlock(&cdata->microp_adc_mutex);
		return -EIO;
	}

	ret = i2c_read_block(client, MICROP_I2C_RCMD_ADC_VALUE, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
		mutex_unlock(&cdata->microp_adc_mutex);
		return -EIO;
	}

	*value = data[0] << 8 | data[1];
	mutex_unlock(&cdata->microp_adc_mutex);
	return 0;
}
EXPORT_SYMBOL(microp_read_adc);

static int microp_read_gpi_status(struct i2c_client *client, uint16_t *status)
{
	uint8_t data[3];
	int ret;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GPI_STATUS, data, 3);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read failed\n", __func__);
		return -EIO;
	}
	*status = (data[0] << 16 | data[1] << 8 | data[2]);
	return 0;
}

static int microp_interrupt_enable(struct i2c_client *client,
				   uint16_t interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	data[0] = interrupt_mask >> 8;
	data[1] = interrupt_mask & 0xFF;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_CTL_EN, data, 2);

	if (ret < 0)
		dev_err(&client->dev, "%s: enable 0x%x interrupt failed\n",
			__func__, interrupt_mask);
	return ret;
}

static int microp_interrupt_disable(struct i2c_client *client,
				    uint16_t interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	data[0] = interrupt_mask >> 8;
	data[1] = interrupt_mask & 0xFF;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_CTL_DIS, data, 2);

	if (ret < 0)
		dev_err(&client->dev, "%s: disable 0x%x interrupt failed\n",
			__func__, interrupt_mask);
	return ret;
}

int microp_write_interrupt(struct i2c_client *client,
		uint16_t interrupt, uint8_t enable)
{
	int ret;

	if (enable) {
		ret = microp_interrupt_enable(client, interrupt);
		printk("%s: microp_interrupt_enable called\n", __func__ );
	} else {
		ret = microp_interrupt_disable(client, interrupt);
		printk("%s: microp_interrupt_disable called\n", __func__ );
	}

	return ret;
}
EXPORT_SYMBOL(microp_write_interrupt);

/*
 * SD slot card-detect support
 */
static unsigned int sdslot_cd = 0;
static void (*sdslot_status_cb)(int card_present, void *dev_id);
static void *sdslot_mmc_dev;

int bravo_microp_sdslot_status_register(
		void (*cb)(int card_present, void *dev_id),
		void *dev_id)
{
	if (sdslot_status_cb)
		return -EBUSY;
	sdslot_status_cb = cb;
	sdslot_mmc_dev = dev_id;
	return 0;
}

unsigned int bravo_microp_sdslot_status(struct device *dev)
{
	return sdslot_cd;
}

static void bravo_microp_sdslot_update_status(int status)
{
	sdslot_cd = !(status & READ_GPI_STATE_SDCARD);
	if (sdslot_status_cb)
		sdslot_status_cb(sdslot_cd, sdslot_mmc_dev);
}

/*
 *Headset Support
*/
static void hpin_debounce_do_work(struct work_struct *work)
{
	uint16_t gpi_status = 0;
	struct microp_i2c_client_data *cdata;
	int insert = 0;
	struct i2c_client *client;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	microp_read_gpi_status(client, &gpi_status);
	insert = (gpi_status & READ_GPI_STATE_HPIN) ? 0 : 1;
	if (insert != cdata->headset_is_in) {
		cdata->headset_is_in = insert;
		pr_debug("headset %s\n", insert ? "inserted" : "removed");
		htc_35mm_jack_plug_event(cdata->headset_is_in,
					 &cdata->is_hpin_pin_stable);
	}
}

static int microp_enable_headset_plug_event(void)
{
	int ret;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint16_t stat;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	/* enable microp interrupt to detect changes */
	ret = microp_interrupt_enable(client, IRQ_HEADSETIN);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to enable irqs\n",
			__func__);
		return 0;
	}
	/* see if headset state has changed */
	microp_read_gpi_status(client, &stat);
	stat = !(stat & READ_GPI_STATE_HPIN);
	if(cdata->headset_is_in != stat) {
		cdata->headset_is_in = stat;
		pr_debug("Headset state changed\n");
		htc_35mm_jack_plug_event(stat, &cdata->is_hpin_pin_stable);
	}

	return 1;
}

static int microp_headset_detect_mic(void)
{
	uint16_t data;

	microp_read_adc(MICROP_REMOTE_KEY_ADC_CHAN, &data);
	if (data >= 200)
		return 1;
	else
		return 0;
}

static int microp_headset_has_mic(void)
{
	int mic1 = -1;
	int mic2 = -1;
	int count = 0;

	mic2 = microp_headset_detect_mic();

	/* debounce the detection wait until 2 consecutive read are equal */
	while ((mic1 != mic2) && (count < 10)) {
		mic1 = mic2;
		msleep(600);
		mic2 = microp_headset_detect_mic();
		count++;
	}

	pr_info("%s: microphone (%d) %s\n", __func__, count,
		mic1 ? "present" : "not present");

	return mic1;
}

static int microp_enable_key_event(void)
{
	int ret;
	struct i2c_client *client;

	client = private_microp_client;

	if (!is_cdma_version(system_rev)) 
		gpio_set_value(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 1);

	/* turn on key interrupt */
	/* enable microp interrupt to detect changes */
	ret = microp_interrupt_enable(client, IRQ_REMOTEKEY);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to enable irqs\n",
			__func__);
		return ret;
	}
	return 0;
}

static int microp_disable_key_event(void)
{
	int ret;
	struct i2c_client *client;

	client = private_microp_client;

	/* shutdown key interrupt */
	if (!is_cdma_version(system_rev))
		gpio_set_value(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0);

	/* disable microp interrupt to detect changes */
	ret = microp_interrupt_disable(client, IRQ_REMOTEKEY);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to disable irqs\n",
			__func__);
		return ret;
	}
	return 0;
}

static int get_remote_keycode(int *keycode)
{
	struct i2c_client *client = private_microp_client;
	int ret;
	uint8_t data[2];

	ret = i2c_read_block(client, MICROP_I2C_RCMD_REMOTE_KEYCODE, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read remote keycode fail\n",
			 __func__);
		return -EIO;
	}
	pr_debug("%s: key = 0x%x\n", __func__, data[1]);
	if (!data[1]) {
		*keycode = 0;
		return 1;		/* no keycode */
	} else {
		*keycode = data[1];
	}
	return 0;
}

static ssize_t microp_i2c_remotekey_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	uint16_t value;
	int i, button = 0;
	int ret;

	client = to_i2c_client(dev);

	microp_read_adc(MICROP_REMOTE_KEY_ADC_CHAN, &value);

	for (i = 0; i < 3; i++) {
		if ((value >= remote_key_adc_table[2 * i]) &&
		    (value <= remote_key_adc_table[2 * i + 1])) {
			button = i + 1;
		}

	}

	ret = sprintf(buf, "Remote Key[0x%03X] => button %d\n",
		      value, button);

	return ret;
}

static DEVICE_ATTR(key_adc, 0644, microp_i2c_remotekey_adc_show, NULL);

/*
 * LED support
*/
static int microp_i2c_write_led_mode(struct i2c_client *client,
				struct led_classdev *led_cdev,
				uint8_t mode, uint16_t off_timer)
{
	struct microp_i2c_client_data *cdata;
	struct microp_led_data *ldata;
	uint8_t data[7];
	int ret;

	cdata = i2c_get_clientdata(client);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	if (ldata->type == GREEN_LED) {
		data[0] = 0x01;
		data[1] = mode;
		data[2] = off_timer >> 8;
		data[3] = off_timer & 0xFF;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
	} else if (ldata->type == AMBER_LED) {
		data[0] = 0x02;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = mode;
		data[5] = off_timer >> 8;
		data[6] = off_timer & 0xFF;
	} else if (ldata->type == BLUE_LED) {
		data[0] = 0x04;
		data[1] = mode;
		data[2] = off_timer >> 8;
		data[3] = off_timer & 0xFF;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
	}

	ret = i2c_write_block(client, MICROP_I2C_WCMD_LED_MODE, data, 7);
	if (ret == 0) {
		mutex_lock(&ldata->led_data_mutex);
		if (mode > 1)
			ldata->blink = mode;
		else
			ldata->mode = mode;
		mutex_unlock(&ldata->led_data_mutex);
	}
	return ret;
}

static ssize_t microp_i2c_led_blink_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	int ret;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	mutex_lock(&ldata->led_data_mutex);
	ret = sprintf(buf, "%d\n", ldata->blink ? ldata->blink - 1 : 0);
	mutex_unlock(&ldata->led_data_mutex);

	return ret;
}

static ssize_t microp_i2c_led_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	int val, ret;
	uint8_t mode;

	val = -1;
	sscanf(buf, "%u", &val);

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	mutex_lock(&ldata->led_data_mutex);
	switch (val) {
	case 0: /* stop flashing */
		mode = ldata->mode;
		ldata->blink = 0;
		break;
	case 1:
	case 2:
	case 3:
		mode = val + 1;
		break;

	default:
		mutex_unlock(&ldata->led_data_mutex);
		return -EINVAL;
	}
	mutex_unlock(&ldata->led_data_mutex);

	ret = microp_i2c_write_led_mode(client, led_cdev, mode, 0xffff);
	if (ret)
		dev_err(&client->dev, "%s set blink failed\n", led_cdev->name);

	return count;
}

static DEVICE_ATTR(blink, 0644, microp_i2c_led_blink_show,
				microp_i2c_led_blink_store);

static ssize_t microp_i2c_led_off_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_i2c_client_data *cdata;
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	uint8_t data[2];
	int ret, offtime;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);
	cdata = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "Getting %s remaining time\n", led_cdev->name);

	if (ldata->type == GREEN_LED) {
		ret = i2c_read_block(client,
				MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME, data, 2);
	} else if (ldata->type == AMBER_LED) {
		ret = i2c_read_block(client,
				MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME,
				data, 2);
	} else if (ldata->type == BLUE_LED) {
		ret = i2c_read_block(client,
				MICROP_I2C_RCMD_BLUE_LED_REMAIN_TIME, data, 2);
	} else {
		dev_err(&client->dev, "Unknown led %s\n", ldata->ldev.name);
		return -EINVAL;
	}

	if (ret) {
		dev_err(&client->dev,
			"%s get off_timer failed\n", led_cdev->name);
	}
	offtime = (int)((data[1] | data[0] << 8) * 2);

	ret = sprintf(buf, "Time remains %d:%d\n", offtime / 60, offtime % 60);
	return ret;
}

static ssize_t microp_i2c_led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	int min, sec, ret;
	uint16_t off_timer;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	dev_dbg(&client->dev, "Setting %s off_timer to %d min %d sec\n",
			led_cdev->name, min, sec);

	if (!min && !sec)
		off_timer = 0xFFFF;
	else
		off_timer = (min * 60 + sec) / 2;

	ret = microp_i2c_write_led_mode(client, led_cdev,
					ldata->mode, off_timer);
	if (ret) {
		dev_err(&client->dev,
			"%s set off_timer %d min %d sec failed\n",
			led_cdev->name, min, sec);
	}
	return count;
}

static DEVICE_ATTR(off_timer, 0644, microp_i2c_led_off_timer_show,
			microp_i2c_led_off_timer_store);

static void microp_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	unsigned long flags;
	struct i2c_client *client = to_i2c_client(led_cdev->dev->parent);
	struct microp_led_data *ldata =
		container_of(led_cdev, struct microp_led_data, ldev);

	dev_dbg(&client->dev, "Setting %s brightness current %d new %d\n",
			led_cdev->name, led_cdev->brightness, brightness);

	if (brightness > 255)
		brightness = 255;
	led_cdev->brightness = brightness;

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	ldata->brightness = brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	schedule_work(&ldata->brightness_work);
}

static void microp_led_brightness_set_work(struct work_struct *work)
{
	unsigned long flags;
	struct microp_led_data *ldata =
		container_of(work, struct microp_led_data, brightness_work);
	struct led_classdev *led_cdev = &ldata->ldev;

	struct i2c_client *client = to_i2c_client(led_cdev->dev->parent);

	enum led_brightness brightness;
	int ret;
	uint8_t mode;

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	brightness = ldata->brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	if (brightness)
		mode = 1;
	else
		mode = 0;

	ret = microp_i2c_write_led_mode(client, led_cdev, mode, 0xffff);
	if (ret) {
		dev_err(&client->dev,
			 "led_brightness_set failed to set mode\n");
	}
}

static void microp_led_brightness_gpo_set_work(struct work_struct *work)
{
	unsigned long flags;
	struct microp_led_data *ldata =
		container_of(work, struct microp_led_data, brightness_work);

	enum led_brightness brightness;
	int ret;
	uint8_t addr, data[3] = {0x00,0x02,0x00}, enable;

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	brightness = ldata->brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	enable = brightness ? 1 : 0;
	if (enable)
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_EN;
	else
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_DIS;

	ret = microp_i2c_write (addr, data, 3);
	if (ret < 0)
		pr_err("%s failed on set gpo led mode:%d\n", __func__, brightness);
}

struct device_attribute *green_amber_attrs[] = {
	&dev_attr_blink,
	&dev_attr_off_timer,
};

static void microp_led_buttons_brightness_set_work(struct work_struct *work)
{

	unsigned long flags;
	struct microp_led_data *ldata =
		container_of(work, struct microp_led_data, brightness_work);
	struct led_classdev *led_cdev = &ldata->ldev;

	struct i2c_client *client = to_i2c_client(led_cdev->dev->parent);
	struct microp_i2c_client_data *cdata = i2c_get_clientdata(client);

	uint8_t data[4] = {0, 0, 0};
	int ret = 0;
	enum led_brightness brightness;
	uint8_t value;

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	brightness = ldata->brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	value = brightness >= 255 ? 0xFF : 0;

	/* avoid a flicker that can occur when writing the same value */
	if (cdata->button_led_value == value)
		return;
	cdata->button_led_value = value;

	/* in 40ms */
	data[0] = 0x05;
	/* duty cycle 0-255 */
	data[1] = value;
	/* bit2 == change brightness */
	data[3] = 0x04;

	ret = i2c_write_block(client, MICROP_I2C_WCMD_BUTTONS_LED_CTRL,
			      data, 4);
	if (ret < 0)
		dev_err(&client->dev, "%s failed on set buttons\n", __func__);
}

static int microp_oj_interrupt_mode(struct i2c_client *client, uint8_t enable)
{
	int ret;

	if (enable) {
		ret = microp_interrupt_enable(client, IRQ_OJ);
		printk("%s: microp_interrupt_enable called\n", __func__ );
	} else {
		ret = microp_interrupt_disable(client, IRQ_OJ);
		printk("%s: microp_interrupt_disable called\n", __func__ );
	}

	return ret;
}

static int microp_spi_enable(uint8_t on)
{
	struct i2c_client *client;
	int ret;

	client = private_microp_client;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_SPI_EN, &on, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
		return ret;
	}

	msleep(10);
	return ret;
}

/* Lookup active SPI devices and only turn it off when no device
 * is using it
 * */
int microp_spi_vote_enable(int spi_device, uint8_t enable) {
	//XXX need to check that all that crap in the HTC kernel is needed
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata = i2c_get_clientdata(client);
	uint8_t data[2] = {0, 0};
	int ret = 0;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (spi_device == SPI_OJ) {
		microp_oj_interrupt_mode(client, enable);
		printk(KERN_ERR "%s: Changing OJ interrupt mode [%d]\n", __func__, enable);
	}
	
	mutex_lock(&cdata->microp_adc_mutex);
	/* Add/remove it from the poll */
	if (enable)
		cdata->spi_devices_vote |= spi_device;
	else
		cdata->spi_devices_vote &= ~spi_device;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_SPI_BL_STATUS, data, 2);
	if (ret != 0) {
		printk(KERN_ERR "%s: read SPI/BL status fail\n", __func__);
		mutex_unlock(&cdata->microp_adc_mutex);
		return ret;
	}

	if ((data[1] & 0x01) ==
		((cdata->spi_devices & cdata->spi_devices_vote) ? 1 : 0)) {
			printk(KERN_ERR "%s: already in voted state, [spi_device %d,enable %d], [spi_status %d, spi_devices_vote %d]\n", __func__, spi_device, enable, data[1]&0x01, cdata->spi_devices_vote);
			mutex_unlock(&cdata->microp_adc_mutex);
			return ret;
	}

	if (cdata->spi_devices & cdata->spi_devices_vote)
		enable = 1;
	else
		enable = 0;

	printk(KERN_ERR "%s: Changing SPI [%d]\n", __func__, enable);

	mutex_unlock(&cdata->microp_adc_mutex);
	ret = microp_spi_enable(enable);
	return ret;
}
EXPORT_SYMBOL(microp_spi_vote_enable);

/*
 * G-sensor
 */
static int gsensor_read_reg(uint8_t reg, uint8_t *data)
{
	struct i2c_client *client = private_microp_client;
	int ret;
	uint8_t tmp[2];

	mutex_lock(&gsensor_RW_mutex);

	ret = i2c_write_block(client, MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ,
			      &reg, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}
	msleep(10);

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GSENSOR_REG_DATA, tmp, 2);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_read_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}
	*data = tmp[1];
	
	mutex_unlock(&gsensor_RW_mutex);

	return ret;
}

static int gsensor_write_reg(uint8_t reg, uint8_t data)
{
	struct i2c_client *client = private_microp_client;
	int ret;
	uint8_t tmp[2];

	mutex_lock(&gsensor_RW_mutex);

	tmp[0] = reg;
	tmp[1] = data;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GSENSOR_REG, tmp, 2);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}

	mutex_unlock(&gsensor_RW_mutex);

	return ret;
}

static int gsensor_read_acceleration(short *buf)
{
	struct i2c_client *client = private_microp_client;
	int ret;
	uint8_t tmp[6];
	struct microp_i2c_client_data *cdata;

	mutex_lock(&gsensor_RW_mutex);

	cdata = i2c_get_clientdata(client);

	tmp[0] = 1;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GSENSOR_DATA_REQ,
			      tmp, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}

	msleep(10);

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GSENSOR_DATA,
				 tmp, 6);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c_read_block fail\n",
			__func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}
	buf[0] = (short)(tmp[0] << 8 | tmp[1]);
	buf[0] >>= 6;
	buf[1] = (short)(tmp[2] << 8 | tmp[3]);
	buf[1] >>= 6;
	buf[2] = (short)(tmp[4] << 8 | tmp[5]);
	buf[2] >>= 6;

#ifdef DEBUG_BMA150
	/* Log this to debugfs */
	gsensor_log_status(ktime_get(), buf[0], buf[1], buf[2]);
#endif

	mutex_unlock(&gsensor_RW_mutex);

	return 1;
}

static int gsensor_init_hw(void)
{
	uint8_t reg;
	int ret;

	pr_debug("%s\n", __func__);

	ret = gsensor_read_reg(RANGE_BWIDTH_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg &= 0xe0;
	ret = gsensor_write_reg(RANGE_BWIDTH_REG, reg);
	if (ret < 0 )
		return -EIO;

	ret = gsensor_read_reg(SMB150_CONF2_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg |= (1 << 3);
	ret = gsensor_write_reg(SMB150_CONF2_REG, reg);

	return ret;
}

static int bma150_set_mode(char mode)
{
	uint8_t reg;
	int ret;

	mutex_lock(&gsensor_set_mode_mutex);

	pr_debug("%s mode = %d\n", __func__, mode);
	if (mode == BMA_MODE_NORMAL)
		microp_spi_vote_enable(SPI_GSENSOR, 1);

	ret = gsensor_read_reg(SMB150_CTRL_REG, &reg);
	if (ret < 0 ) {
		mutex_unlock(&gsensor_set_mode_mutex);
		return -EIO;
	}
	reg = (reg & 0xfe) | mode;
	ret = gsensor_write_reg(SMB150_CTRL_REG, reg);

	if (mode == BMA_MODE_SLEEP)
		microp_spi_vote_enable(SPI_GSENSOR, 0);

	mutex_unlock(&gsensor_set_mode_mutex);

	return ret;
}

static int gsensor_read(uint8_t *data)
{
	int ret;
	uint8_t reg = data[0];

	ret = gsensor_read_reg(reg, &data[1]);
	pr_debug("%s reg = %x data = %x\n", __func__, reg, data[1]);
	return ret;
}

static int gsensor_write(uint8_t *data)
{
	int ret;
	uint8_t reg = data[0];

	pr_debug("%s reg = %x data = %x\n", __func__, reg, data[1]);
	ret = gsensor_write_reg(reg, data[1]);
	return ret;
}

static DEFINE_MUTEX(bma150_lock);

static int bma150_open(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	return nonseekable_open(inode, file);
}

static int bma150_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long bma150_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	char rwbuf[8];
	int ret = -1;
	short buf[8], temp;

	switch (cmd) {
	case BMA_IOCTL_READ:
	case BMA_IOCTL_WRITE:
	case BMA_IOCTL_SET_MODE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_from_user(&buf, argp, sizeof(buf)))
			return -EFAULT;
		break;
	default:
		break;
	}

	mutex_lock(&bma150_lock);
	switch (cmd) {
	case BMA_IOCTL_INIT:
		ret = gsensor_init_hw();
		if (ret < 0)
			goto err;
		break;

	case BMA_IOCTL_READ:
		if (rwbuf[0] < 1) {
			ret = -EINVAL;
			goto err;
		}
		ret = gsensor_read(rwbuf);
		if (ret < 0)
			goto err;
		break;
	case BMA_IOCTL_WRITE:
		if (rwbuf[0] < 2) {
			ret = -EINVAL;
			goto err;
		}
		ret = gsensor_write(rwbuf);
		if (ret < 0)
			goto err;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		ret = gsensor_read_acceleration(&buf[0]);
		if (ret < 0)
			goto err;
		break;
	case BMA_IOCTL_SET_MODE:
		bma150_set_mode(rwbuf[0]);
		break;
	case BMA_IOCTL_GET_INT:
		temp = 0;
		break;
	default:
		ret = -ENOTTY;
		goto err;
	}
	mutex_unlock(&bma150_lock);

	switch (cmd) {
	case BMA_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_INT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
err:
	mutex_unlock(&bma150_lock);
	return ret;
}

static struct file_operations bma_fops = {
	.owner = THIS_MODULE,
	.open = bma150_open,
	.release = bma150_release,
	.unlocked_ioctl = bma150_ioctl,
};

static struct miscdevice spi_bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = BMA150_G_SENSOR_NAME,
	.fops = &bma_fops,
};

/*
 * Interrupt
 */
static irqreturn_t microp_i2c_intr_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = to_i2c_client(dev_id);
	cdata = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "intr_irq_handler\n");

	disable_irq_nosync(client->irq);
	schedule_work(&cdata->work.work);
	return IRQ_HANDLED;
}

static void microp_int_dispatch(u32 status)
{
	unsigned int mask;
	int irq;

	while (status) {
		mask = status & -status;
		irq = fls(mask) - 1;
		status &= ~mask;
		generic_handle_irq(FIRST_MICROP_IRQ + irq);
	}
}

static enum hrtimer_restart hr_dispath_irq_func(struct hrtimer *data)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);
	microp_int_dispatch(cdata->intr_status);
	cdata->intr_status = 0;
	return HRTIMER_NORESTART;
}

static void microp_i2c_intr_work_func(struct work_struct *work)
{
	struct microp_i2c_work *up_work;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[3];
	uint16_t intr_status = 0, gpi_status = 0;
	int keycode = 0, ret = 0;
	ktime_t zero_debounce;

	up_work = container_of(work, struct microp_i2c_work, work);
	client = up_work->client;
	cdata = i2c_get_clientdata(client);

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GPI_INT_STATUS, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read interrupt status fail\n",
			 __func__);
	}

	intr_status = data[0]<<8 | data[1];
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_STATUS_CLR,
			      data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: clear interrupt status fail\n",
			 __func__);
	}
	pr_debug("intr_status=0x%02x\n", intr_status);

	if (intr_status & IRQ_SDCARD) {
		microp_read_gpi_status(client, &gpi_status);
		bravo_microp_sdslot_update_status(gpi_status);
	}

	if (intr_status & IRQ_HEADSETIN) {
		cdata->is_hpin_pin_stable = 0;
		wake_lock_timeout(&microp_i2c_wakelock, 3*HZ);
		if (!cdata->headset_is_in)
			schedule_delayed_work(&cdata->hpin_debounce_work,
					msecs_to_jiffies(500));
		else
			schedule_delayed_work(&cdata->hpin_debounce_work,
					msecs_to_jiffies(300));
	}

	if (intr_status & IRQ_REMOTEKEY) {
		if ((get_remote_keycode(&keycode) == 0) &&
			(cdata->is_hpin_pin_stable)) {
			htc_35mm_key_event(keycode, &cdata->is_hpin_pin_stable);
		}
	}

	cdata->intr_status = intr_status;
	zero_debounce = ktime_set(0, 0);  /* No debounce time */
	hrtimer_start(&cdata->gen_irq_timer, zero_debounce, HRTIMER_MODE_REL);

	enable_irq(client->irq);
}

static int microp_function_initialize(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;
	uint8_t data[20];
	uint16_t stat, interrupts = 0;
	int i;
	int ret;
	struct led_classdev *led_cdev;

	cdata = i2c_get_clientdata(client);

	/* Headset */
	for (i = 0; i < 6; i++) {
		data[i] = (uint8_t)(remote_key_adc_table[i] >> 8);
		data[i + 6] = (uint8_t)(remote_key_adc_table[i]);
	}
	ret = i2c_write_block(client,
		MICROP_I2C_WCMD_REMOTEKEY_TABLE, data, 12);
	if (ret)
		goto exit;

	INIT_DELAYED_WORK(
		&cdata->hpin_debounce_work, hpin_debounce_do_work);

	/* SD Card */
	interrupts |= IRQ_SDCARD;

	/* set LED initial state */
	for (i = 0; i < BLUE_LED; i++) {
		led_cdev = &cdata->leds[i].ldev;
		microp_i2c_write_led_mode(client, led_cdev, 0, 0xffff);
	}

#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	/* OJ interrupt */
	interrupts |= IRQ_OJ;
#endif

	/* enable the interrupts */
	ret = microp_interrupt_enable(client, interrupts);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to enable gpi irqs\n",
			__func__);
		goto err_irq_en;
	}

	microp_read_gpi_status(client, &stat);
	bravo_microp_sdslot_update_status(stat);

	return 0;

err_irq_en:
	gpio_free(BRAVO_GPIO_LS_EN_N);
exit:
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void microp_early_suspend(struct early_suspend *h)
{
	struct microp_i2c_client_data *cdata;
	struct i2c_client *client = private_microp_client;
	int ret;

	if (!client) {
		pr_err("%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);

	cdata->microp_is_suspend = 1;

	disable_irq(client->irq);
	ret = cancel_work_sync(&cdata->work.work);
	if (ret != 0) {
		enable_irq(client->irq);
	}
}

void microp_early_resume(struct early_suspend *h)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;

	if (!client) {
		pr_err("%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);

	cdata->microp_is_suspend = 0;
	enable_irq(client->irq);
}
#endif

static int microp_i2c_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
	return 0;
}

static int microp_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static void register_microp_devices(struct platform_device *devices, int num)
{
	int i;
	for (i = 0; i < num; i++)
		platform_device_register((devices + i));
}

static struct {
	const char *name;
	void (*led_set_work)(struct work_struct *);
	struct device_attribute **attrs;
	int attr_cnt;
} microp_leds[] = {
	[GREEN_LED] = {
		.name		= "green",
		.led_set_work	= microp_led_brightness_set_work,
		.attrs		= green_amber_attrs,
		.attr_cnt	= ARRAY_SIZE(green_amber_attrs)
	},
	[AMBER_LED] = {
		.name		= "amber",
		.led_set_work	= microp_led_brightness_set_work,
		.attrs		= green_amber_attrs,
		.attr_cnt	= ARRAY_SIZE(green_amber_attrs)
	},
	[BLUE_LED] = {
		.name		= "blue",
		.led_set_work	= microp_led_brightness_gpo_set_work,
		.attrs		= NULL,
		.attr_cnt	= 0
	},
	[BUTTONS_LED] = {
		.name		= "button-backlight",
		.led_set_work	= microp_led_buttons_brightness_set_work
	},
};

static int microp_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[6];
	int ret;
	int i;
	int j;

	private_microp_client = client;
	cdata = kzalloc(sizeof(struct microp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);

	mutex_init(&cdata->microp_adc_mutex);
	mutex_init(&cdata->microp_i2c_rw_mutex);

	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "failed on get pdata\n");
		goto err_exit;
	}
	pdata->dev_id = (void *)&client->dev;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_VERSION, data, 2);
	if (ret || !(data[0] && data[1])) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp version\n");
		goto err_exit;
	}
	dev_info(&client->dev, "microp version [%02X][%02X]\n",
		  data[0], data[1]);

	ret = gpio_request(BRAVO_GPIO_UP_RESET_N, "microp_i2c_wm");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio reset\n");
		goto err_exit;
	}
	ret = gpio_direction_output(BRAVO_GPIO_UP_RESET_N, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			 "failed on gpio_direction_output reset\n");
		goto err_gpio_reset;
	}

	cdata->version = data[0] << 8 | data[1];
	cdata->microp_is_suspend = 0;
	cdata->spi_devices_vote = 0;
	cdata->spi_devices = SPI_OJ | SPI_GSENSOR;

	cdata->intr_status = 0;
	hrtimer_init(&cdata->gen_irq_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cdata->gen_irq_timer.function = hr_dispath_irq_func;

	wake_lock_init(&microp_i2c_wakelock, WAKE_LOCK_SUSPEND,
			 "microp_i2c_present");

	/* LEDs */
	ret = 0;
	for (i = 0; i < ARRAY_SIZE(microp_leds) && !ret; ++i) {
		struct microp_led_data *ldata = &cdata->leds[i];

		ldata->type = i;
		ldata->ldev.name = microp_leds[i].name;
		ldata->ldev.brightness_set = microp_brightness_set;
		mutex_init(&ldata->led_data_mutex);
		INIT_WORK(&ldata->brightness_work, microp_leds[i].led_set_work);
		spin_lock_init(&ldata->brightness_lock);
		ret = led_classdev_register(&client->dev, &ldata->ldev);
		if (ret) {
			ldata->ldev.name = NULL;
			break;
		}

		for (j = 0; j < microp_leds[i].attr_cnt && !ret; ++j)
			ret = device_create_file(ldata->ldev.dev,
						 microp_leds[i].attrs[j]);
	}
	if (ret) {
		dev_err(&client->dev, "failed to add leds\n");
		goto err_add_leds;
	}

	/* Headset */
	cdata->headset_is_in = 0;
	cdata->is_hpin_pin_stable = 1;
	platform_device_register(&bravo_h35mm);

	ret = device_create_file(&client->dev, &dev_attr_key_adc);

	/* G-sensor */
	ret = misc_register(&spi_bma_device);
	if (ret < 0) {
		pr_err("%s: init bma150 misc_register fail\n",
				__func__);
		goto err_register_bma150;
	}
	
	mutex_init(&gsensor_RW_mutex);
	mutex_init(&gsensor_set_mode_mutex);

	microp_spi_vote_enable(SPI_GSENSOR, 1);

#ifdef DEBUG_BMA150
	debugfs_create_file("gsensor_log", 0444, NULL, NULL, &gsensor_log_fops);
#endif
	/* Setup IRQ handler */
	INIT_WORK(&cdata->work.work, microp_i2c_intr_work_func);
	cdata->work.client = client;

	ret = request_irq(client->irq,
			microp_i2c_intr_irq_handler,
			IRQF_TRIGGER_LOW,
			"microp_interrupt",
			&client->dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_intr;
	}
	ret = set_irq_wake(client->irq, 1);
	if (ret) {
		dev_err(&client->dev, "set_irq_wake failed\n");
		goto err_intr;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend) {
		cdata->early_suspend.level =
				EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		cdata->early_suspend.suspend = microp_early_suspend;
		cdata->early_suspend.resume = microp_early_resume;
		register_early_suspend(&cdata->early_suspend);
	}
#endif

	ret = microp_function_initialize(client);
	if (ret) {
		dev_err(&client->dev, "failed on microp function initialize\n");
		goto err_fun_init;
	}

	register_microp_devices(pdata->microp_devices, pdata->num_devices);

	return 0;

err_fun_init:
err_intr:
	misc_deregister(&spi_bma_device);

err_register_bma150:
	platform_device_unregister(&bravo_h35mm);
	device_remove_file(&client->dev, &dev_attr_key_adc);

err_add_leds:
	for (i = 0; i < ARRAY_SIZE(microp_leds); ++i) {
		if (!cdata->leds[i].ldev.name)
			continue;
		led_classdev_unregister(&cdata->leds[i].ldev);
		for (j = 0; j < microp_leds[i].attr_cnt; ++j)
			device_remove_file(cdata->leds[i].ldev.dev,
					   microp_leds[i].attrs[j]);
	}

	wake_lock_destroy(&microp_i2c_wakelock);
	kfree(cdata);
	i2c_set_clientdata(client, NULL);

err_cdata:
err_gpio_reset:
	gpio_free(BRAVO_GPIO_UP_RESET_N);
err_exit:
	return ret;
}

static int __devexit microp_i2c_remove(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;
	int i;
	int j;

	cdata = i2c_get_clientdata(client);

	for (i = 0; i < ARRAY_SIZE(microp_leds); ++i) {
		struct microp_led_data *ldata = &cdata->leds[i];
		cancel_work_sync(&ldata->brightness_work);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend) {
		unregister_early_suspend(&cdata->early_suspend);
	}
#endif

	for (i = 0; i < ARRAY_SIZE(microp_leds); ++i) {
		if (!cdata->leds[i].ldev.name)
			continue;
		led_classdev_unregister(&cdata->leds[i].ldev);
		for (j = 0; j < microp_leds[i].attr_cnt; ++j)
			device_remove_file(cdata->leds[i].ldev.dev,
					   microp_leds[i].attrs[j]);
	}

	free_irq(client->irq, &client->dev);

	gpio_free(BRAVO_GPIO_UP_RESET_N);

	platform_device_unregister(&bravo_h35mm);

	/* G-sensor */
	misc_deregister(&spi_bma_device);

	kfree(cdata);

	return 0;
}

static const struct i2c_device_id microp_i2c_id[] = {
	{ MICROP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver microp_i2c_driver = {
	.driver = {
		   .name = MICROP_I2C_NAME,
		   },
	.id_table = microp_i2c_id,
	.probe = microp_i2c_probe,
	.suspend = microp_i2c_suspend,
	.resume = microp_i2c_resume,
	.remove = __devexit_p(microp_i2c_remove),
};

static void microp_irq_ack(unsigned int irq)
{
	;
}

static void microp_irq_mask(unsigned int irq)
{
	;
}

static void microp_irq_unmask(unsigned int irq)
{
	;
}

static struct irq_chip microp_irq_chip = {
	.name = "microp",
	.disable = microp_irq_mask,
	.ack = microp_irq_ack,
	.mask = microp_irq_mask,
	.unmask = microp_irq_unmask,
};

static int __init microp_i2c_init(void)
{
	int n, MICROP_IRQ_END = FIRST_MICROP_IRQ + NR_MICROP_IRQS;
	for (n = FIRST_MICROP_IRQ; n < MICROP_IRQ_END; n++) {
		set_irq_chip(n, &microp_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}

	return i2c_add_driver(&microp_i2c_driver);
}

static void __exit microp_i2c_exit(void)
{
	i2c_del_driver(&microp_i2c_driver);
}

module_init(microp_i2c_init);
module_exit(microp_i2c_exit);

MODULE_AUTHOR("Eric Olsen <eolsen@android.com>");
MODULE_DESCRIPTION("MicroP I2C driver");
MODULE_LICENSE("GPL");
