/* drivers/input/touchscreen/himax8250.c
 *
 * Copyright (C) 2011 HTC Corporation.
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

#include <linux/himax8526a.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <mach/msm_hsusb.h>
#include <mach/board.h>
#include <asm/atomic.h>

#define HIMAX_I2C_RETRY_TIMES 10
#define ESD_WORKAROUND
#define FAKE_EVENT

struct himax_ts_data {
	int use_irq;
	struct workqueue_struct *himax_wq;
	struct input_dev *input_dev;
	int fw_ver;
	struct hrtimer timer;
	struct work_struct work;
	struct i2c_client *client;
	uint8_t debug_log_level;
	uint32_t irq;
	int (*power)(int on);
	struct early_suspend early_suspend;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t usb_connected;
	uint8_t *cable_config;
	uint8_t diag_command;
	int16_t *diag_mutual;
	int16_t diag_self[50];
	uint8_t finger_pressed;
	uint8_t first_pressed;
	int pre_finger_data[2];
	uint8_t suspend_mode;
	uint8_t last_slot;
	uint8_t protocol_type;
	struct himax_i2c_platform_data *pdata;
	uint32_t event_htc_enable_type;
	struct himax_config_init_api i2c_api;
#ifdef FAKE_EVENT
	int fake_X_S;
	int fake_Y_S;
	int fake_X_E;
	int fake_Y_E;
#endif
};
static struct himax_ts_data *private_ts;
static uint8_t reset_activate;

#define SWITCH_TO_HTC_EVENT_ONLY	1
#define INJECT_HTC_EVENT		2

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		printk(KERN_INFO "[TP]i2c_read_block retry over %d\n",
			toRetry);
		return -EIO;
	}
	return 0;

}


int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		printk(KERN_ERR "[TP]i2c_write_block retry over %d\n",
			toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t buf[length];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		printk(KERN_ERR "[TP]i2c_write_block retry over %d\n",
		       toRetry);
		return -EIO;
	}
	return 0;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		printk(KERN_INFO "[TP]i2c_read_block retry over %d\n",
		       toRetry);
		return -EIO;
	}
	return 0;
}

static uint8_t himax_command;

static ssize_t himax_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t data[96] = { 0 }, loop_i;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	printk(KERN_INFO "[TP]%x\n", himax_command);
	if (i2c_himax_read(ts_data->client, himax_command, data, 96,
		 HIMAX_I2C_RETRY_TIMES) < 0) {
		printk(KERN_WARNING "%s: read fail\n", __func__);
		return ret;
	}

	ret += sprintf(buf, "command: %x\n", himax_command);
	for (loop_i = 0; loop_i < 96; loop_i++) {
		ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
			ret += sprintf(buf + ret, "\n");
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;

	ts_data = private_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		if (buf[2] == 'x') {
			uint8_t loop_i;
			uint16_t base = 5;
			memcpy(buf_tmp, buf + 3, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
				himax_command = result;
			for (loop_i = 0; loop_i < 100; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w')
						i2c_himax_write(ts_data->client, himax_command,
							&write_da[0], length, HIMAX_I2C_RETRY_TIMES);
					printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
						write_da[0], length);
					for (veriLen = 0; veriLen < length; veriLen++)
						printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));

					printk(KERN_INFO "\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul(buf_tmp, 16, &result))
						write_da[loop_i] = result;
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),
	himax_register_show, himax_register_store);


static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	sprintf(buf, "%s_%#x\n", HIMAX8526A_NAME, ts_data->fw_ver);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);

static ssize_t touch_attn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	sprintf(buf, "attn = %x\n", gpio_get_value(ts_data->irq));
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(attn, 0444, touch_attn_show, NULL);

static ssize_t himax_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	count += sprintf(buf, "%d\n", ts_data->debug_log_level);

	return count;
}

static ssize_t himax_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts_data->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	himax_debug_level_show, himax_debug_level_dump);

static ssize_t himax_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
	ts_data = private_ts;

	mutual_num = ts_data->x_channel * ts_data->y_channel;
	self_num = ts_data->x_channel + ts_data->y_channel;
	width = ts_data->x_channel;
	count += sprintf(buf + count, "Channel: %4d, %4d\n\n", ts_data->x_channel, ts_data->y_channel);

	if (ts_data->diag_command >= 1 && ts_data->diag_command <= 6) {
		if (ts_data->diag_command < 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += sprintf(buf + count, "%4d", ts_data->diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count += sprintf(buf + count, " %3d\n", ts_data->diag_self[width + loop_i/width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count += sprintf(buf + count, "%4d", ts_data->diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					count += sprintf(buf + count, "\n");
			}
		} else if (ts_data->diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				count += sprintf(buf + count, "%4d", ts_data->diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
					count += sprintf(buf + count, "\n");
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += sprintf(buf + count, "%4d", ts_data->diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					count += sprintf(buf + count, "\n");
			}
		}
	}

	return count;
}

static ssize_t himax_diag_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0xFC;
	const uint8_t command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
	uint8_t new_command[2] = {0x91, 0x00};

	ts_data = private_ts;
	printk(KERN_DEBUG "[TP]%s: entered, buf[0]=%c.\n", __func__, buf[0]);
	if (buf[0] == '1' || buf[0] == '3' || buf[0] == '5') {
		new_command[1] = command_ec_128_raw_baseline_flag;
		i2c_himax_master_write(ts_data->client, new_command,
			 sizeof(new_command), HIMAX_I2C_RETRY_TIMES);
		ts_data->diag_command = buf[0] - '0';
	} else if (buf[0] == '2' || buf[0] == '4' || buf[0] == '6') {
		new_command[1] = command_ec_128_raw_flag;
		i2c_himax_master_write(ts_data->client, new_command,
			 sizeof(new_command), HIMAX_I2C_RETRY_TIMES);
		ts_data->diag_command = buf[0] - '0';
	} else {
		new_command[1] = command_ec_24_normal_flag;
		i2c_himax_master_write(ts_data->client, new_command,
			 sizeof(new_command), HIMAX_I2C_RETRY_TIMES);
		ts_data->diag_command = 0;
	}

	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
	himax_diag_show, himax_diag_dump);

static ssize_t himax_set_event_htc(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	unsigned long result = 0;
	ts_data = private_ts;
	if (!strict_strtoul(buf, 10, &result)) {
		ts_data->event_htc_enable_type = result;

		pr_info("[TP]htc event enable = %d\n", ts_data->event_htc_enable_type);
	}
	return count;
}

static ssize_t himax_show_event_htc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	return sprintf(buf, "htc event enable = %d\n", ts_data->event_htc_enable_type);
}
static DEVICE_ATTR(htc_event, (S_IWUSR|S_IRUGO), himax_show_event_htc, himax_set_event_htc);

static ssize_t himax_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	if (reset_activate)
		count += sprintf(buf, "Resetting touch chip in progress.\n");
	else
		count += sprintf(buf, "Reset complete or not trigger yet.\n");

	return count;
}

static ssize_t himax_reset_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	int ret = 0;
	ts_data = private_ts;
	if (buf[0] == '1' && ts_data->pdata->reset) {
		if (ts_data->use_irq)
			disable_irq_nosync(ts_data->client->irq);
		else {
			hrtimer_cancel(&ts_data->timer);
			ret = cancel_work_sync(&ts_data->work);
		}

		printk(KERN_INFO "[TP]%s: Now reset the Touch chip.\n", __func__);

		ts_data->pdata->reset();

		if (ts_data->use_irq)
			enable_irq(ts_data->client->irq);
		else
			hrtimer_start(&ts_data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	return count;
}

static DEVICE_ATTR(reset, (S_IWUSR|S_IRUGO),
	himax_reset_show, himax_reset_set);

#ifdef FAKE_EVENT

static int X_fake_S = 50;
static int Y_fake_S = 600;
static int X_fake_E = 800;
static int Y_fake_E = 600;
static int dx_fake = 2;
static int dy_fake;
static unsigned long report_time = 10000000;

static enum hrtimer_restart himax_ts_timer_fake_event_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts = container_of(timer, struct himax_ts_data, timer);

	static int i;
	static int X_tmp;
	static int Y_tmp;

	if (!i) {
		X_tmp = X_fake_S;
		Y_tmp = Y_fake_S;
		i++;
	}
	if ((dx_fake > 0 ? X_tmp <= X_fake_E : dx_fake ? X_tmp >= X_fake_E : 0) ||
		(dy_fake > 0 ? Y_tmp <= Y_fake_E : dy_fake ? Y_tmp >= Y_fake_E : 0)) {
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 5);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, X_tmp);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, Y_tmp);
			input_mt_sync(ts->input_dev);
		} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 5);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, X_tmp);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, Y_tmp);
		}
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_sync(ts->input_dev);
		X_tmp += dx_fake;
		Y_tmp += dy_fake;
		hrtimer_start(&ts->timer, ktime_set(0, report_time), HRTIMER_MODE_REL);
	} else {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_mt_sync(ts->input_dev);
		} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_sync(ts->input_dev);
		i = 0;
		printk(KERN_INFO "[TP]End of fake event\n");
	}

	return HRTIMER_NORESTART;
}

static ssize_t himax_fake_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	static uint8_t i;
	size_t count = 0;
	int Xres = 0, Yres = 0, Fx = 0, Fy = 0;

	if (!i) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_fake_event_func;
		printk(KERN_INFO "hrtimer_init\n");
		i++;
	}
	count += sprintf(buf + count, "%d,%d,%d,%d,%d,%d,%lu\n",
		ts->fake_X_S, ts->fake_Y_S, ts->fake_X_E,
		ts->fake_Y_E, dx_fake, dy_fake, report_time/1000000);
	if (ts->pdata->screenWidth != 0 && ts->pdata->screenWidth != 0) {
		Xres = ts->pdata->abs_x_max - ts->pdata->abs_x_min;
		Yres = ts->pdata->abs_y_max - ts->pdata->abs_y_min;
		Fx = Xres / ts->pdata->screenWidth;
		Fy = Yres / ts->pdata->screenHeight;
		X_fake_S = ts->fake_X_S * Fx + ts->pdata->abs_x_min;
		Y_fake_S = ts->fake_Y_S * Fy + ts->pdata->abs_y_min;
		X_fake_E = ts->fake_X_E * Fx + ts->pdata->abs_x_min;
		Y_fake_E = ts->fake_Y_E * Fy + ts->pdata->abs_y_min;
	}

	if (dx_fake & dy_fake)
		count += sprintf(buf + count, "dx_fake or dy_fake should one value need to be zero\n");
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return count;
}

static ssize_t himax_fake_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	long value;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	ts_data = private_ts;

	while (1) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				printk(KERN_INFO "buffer size is over 5 char\n");
				return count;
			}
			j = i + 1;

			ret = strict_strtol(buf_tmp, 10, &value);

			switch (k) {
			case 0:
				ts_data->fake_X_S = value;
				break;
			case 1:
				ts_data->fake_Y_S = value;
				break;
			case 2:
				ts_data->fake_X_E = value;
				break;
			case 3:
				ts_data->fake_Y_E = value;
				break;
			case 4:
				dx_fake = value;
				break;
			case 5:
				dy_fake = value;
				break;
			case 6:
				report_time = value*1000000;
			default:
				break;
			}
			k++;
		}
		if (buf[i] == '\n')
			break;
		i++;
	}

	return count;

}

static DEVICE_ATTR(fake_event, (S_IWUSR|S_IRUGO),
	himax_fake_event_show, himax_fake_event_store);

#endif

static struct kobject *android_touch_kobj;

static int himax_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[TP]TOUCH_ERR: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: create_file debug_level failed\n");
		return ret;
	}
	himax_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: create_file register failed\n");
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_htc_event.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_attn.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
#ifdef FAKE_EVENT
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_fake_event.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
#endif

	return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_htc_event.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_attn.attr);
#ifdef FAKE_EVENT
	sysfs_remove_file(android_touch_kobj, &dev_attr_fake_event.attr);
#endif
	kobject_del(android_touch_kobj);
}

inline void himax_ts_work(struct himax_ts_data *ts)
{
	uint8_t buf[128], loop_i, finger_num, finger_pressed, hw_reset_check[2];
	uint8_t finger_on = 0;
#ifdef ESD_WORKAROUND
	uint32_t checksum;
#endif
	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	if (i2c_himax_read(ts->client, 0x86, buf, ts->diag_command ? 128 : 24,
		 HIMAX_I2C_RETRY_TIMES)) {
		printk(KERN_ERR "[TP]%s: can't read data from chip!\n", __func__);
		goto err_workqueue_out;
	} else {
#ifdef ESD_WORKAROUND
		for (loop_i = 0, checksum = 0; loop_i < (ts->diag_command ? 128 : 24); loop_i++)
			checksum += buf[loop_i];
		if (checksum == 0)
			printk(KERN_INFO "[TP] Checksum = 0 detected, reset activate = %d, first_pressed = %d\n", reset_activate, ts->first_pressed);

		if (checksum == 0 && !reset_activate) {
			if (i2c_himax_read(ts->client, 0x84, hw_reset_check, 2, HIMAX_I2C_RETRY_TIMES)) {
				printk(KERN_ERR "[TP]%s: can't read data from chip!\n", __func__);
				goto err_workqueue_out;
			}
			if (hw_reset_check[1] == 0x03 && ts->first_pressed > 0)
				goto workqueue_out;
			else {
				msleep(20);
				printk(KERN_INFO "[TP]%s: ESD reset detected, load sensor config.\n", __func__);
				ts->pdata->loadSensorConfig(ts->client, ts->pdata, &(ts->i2c_api));
				reset_activate = 1;
				goto workqueue_out;
			}
		} else if (checksum == 0 && reset_activate) {
			if (i2c_himax_read(ts->client, 0x84, hw_reset_check, 2, HIMAX_I2C_RETRY_TIMES)) {
				printk(KERN_ERR "[TP]%s: can't read data from chip!\n", __func__);
				goto err_workqueue_out;
			}
			if (hw_reset_check[1] == 0x03 && ts->first_pressed > 0)
				goto workqueue_out;
			else {
				msleep(20);
				printk(KERN_INFO "[TP]%s: back from ESD reset, but reset by ESD again.\n", __func__);
				ts->pdata->loadSensorConfig(ts->client, ts->pdata, &(ts->i2c_api));
				goto workqueue_out;
			}
		} else if (checksum == 0xFF && reset_activate && buf[0] == 0xFF) {
			reset_activate = 0;
			printk(KERN_INFO "[TP]%s: back from ESD reset, ready to serve.\n", __func__);
			goto workqueue_out;
		}
#endif
	}

	if (ts->debug_log_level & 0x1) {
		printk(KERN_INFO "[TP]%s: raw data:\n", __func__);
		for (loop_i = 0; loop_i < 24; loop_i++) {
			printk(KERN_INFO "0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				printk(KERN_INFO "\n");
		}
	}

	if (ts->diag_command >= 1 && ts->diag_command <= 6) {
		int mul_num, self_num;
		int index = 0;
		/* Header: %x, %x, %x, %x\n", buf[24], buf[25], buf[26], buf[27] */
		mul_num = ts->x_channel * ts->y_channel;
		self_num = ts->x_channel + ts->y_channel;

		if (buf[24] == buf[25] && buf[25] == buf[26] && buf[26] == buf[27]
				&& buf[24] > 0) {
			index = (buf[24] - 1) * 50;

			for (loop_i = 0; loop_i < 50; loop_i++) {
				if (index < mul_num) { //mutual
					if ((buf[loop_i * 2 + 28] & 0x80) == 0x80)
						ts->diag_mutual[index + loop_i] = 0 -
							((buf[loop_i * 2 + 28] << 8 | buf[loop_i * 2 + 29]) & 0x4FFF);
					else
						ts->diag_mutual[index + loop_i] =
							buf[loop_i * 2 + 28] << 8 | buf[loop_i * 2 + 29];
				} else {//self
					if (loop_i >= self_num)
						break;

					if ((buf[loop_i * 2 + 28] & 0x80) == 0x80)
						ts->diag_self[loop_i] = 0 -
							((buf[loop_i * 2 + 28] << 8 | buf[loop_i * 2 + 29]) & 0x4FFF);
					else
						ts->diag_self[loop_i] =
							buf[loop_i * 2 + 28] << 8 | buf[loop_i * 2 + 29];
				}
			}
		}
	}

	if (buf[20] == 0xFF && buf[21] == 0xFF) {
		/* finger leave */
		finger_on = 0;
		if (ts->event_htc_enable_type) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
		}

		if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY &&
			ts->protocol_type == PROTOCOL_TYPE_A)
			input_mt_sync(ts->input_dev);

		if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY &&
			ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, ts->last_slot);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}

		if (ts->first_pressed == 1) {
			ts->first_pressed = 2;
			printk(KERN_INFO "[TP]E1@%d, %d\n",
				ts->pre_finger_data[0] , ts->pre_finger_data[1]);
		}

		if (ts->debug_log_level & 0x2)
			printk(KERN_INFO "[TP]All Finger leave\n");
	} else {
		finger_num = buf[20] & 0x0F;
		finger_pressed = buf[21];
		finger_on = 1;
		for (loop_i = 0; loop_i < 4; loop_i++) {
			if (((finger_pressed >> loop_i) & 1) == 1) {
				int base = loop_i * 4;
				int x = buf[base] << 8 | buf[base + 1];
				int y = (buf[base + 2] << 8 | buf[base + 3]);
				int w = buf[16 + loop_i];
				finger_num--;

				if (ts->event_htc_enable_type) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, w << 16 | w);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						((finger_num ==  0) ? BIT(31) : 0) | x << 16 | y);
				}

				if (ts->protocol_type == PROTOCOL_TYPE_B)
					input_mt_slot(ts->input_dev, loop_i);

				if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY) {
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
				}

				if (ts->protocol_type == PROTOCOL_TYPE_A) {
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
					input_mt_sync(ts->input_dev);
				} else {
					ts->last_slot = loop_i;
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				}

				if (!ts->first_pressed) {
					ts->first_pressed = 1;
					printk(KERN_INFO "[TP]S1@%d, %d\n", x, y);
				}
				if (ts->first_pressed == 1) {
					ts->pre_finger_data[0] = x;
					ts->pre_finger_data[1] = y;
				}

				if (ts->debug_log_level & 0x2)
					printk(KERN_INFO "[TP]Finger %d=> X:%d, Y:%d w:%d, z:%d, F:%d\n",
						loop_i + 1, x, y, w, w, loop_i + 1);
			} else {
				if (ts->protocol_type == PROTOCOL_TYPE_B) {
					input_mt_slot(ts->input_dev, loop_i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				}

				if (loop_i == 0 && ts->first_pressed == 1) {
					ts->first_pressed = 2;
					printk(KERN_INFO "[TP]E1@%d, %d\n",
					ts->pre_finger_data[0] , ts->pre_finger_data[1]);
				}
			}
		}
	}
	if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY) {
		input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
		input_sync(ts->input_dev);
	}

workqueue_out:
	return;
err_workqueue_out:
	printk(KERN_INFO "[TP]%s: Now reset the Touch chip.\n", __func__);
	ts->pdata->reset();
	goto workqueue_out;
}

static irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	himax_ts_work((struct himax_ts_data *)ptr);
	return IRQ_HANDLED;
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
	himax_ts_work(ts);
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	return HRTIMER_NORESTART;
}

static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	printk(KERN_INFO "[TP]Touch: cable change to %d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config) {
		if (!ts->suspend_mode) {
			if (connect_status) {
				ts->cable_config[1] = 0x01;
				ts->usb_connected = 0x01;
			} else {
				ts->cable_config[1] = 0x00;
				ts->usb_connected = 0x00;
			}
			i2c_himax_master_write(ts->client, ts->cable_config,
				 sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);

			printk(KERN_INFO "[TP]%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;
			printk(KERN_INFO "[TP]%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

static struct t_usb_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};

static int himax8526a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, err = 0;
	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;
	uint8_t data[5] = { 0 };
	reset_activate = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[TP]%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "[TP]%s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->i2c_api.i2c_himax_master_write = i2c_himax_master_write;
	ts->i2c_api.i2c_himax_read_command = i2c_himax_read_command;
	ts->i2c_api.i2c_himax_write_command = i2c_himax_write_command;

	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (!pdata) {
		printk(KERN_ERR "[TP]%s: platform data is null.\n", __func__);
		goto err_platform_data_null;
	}
	if (pdata->power) {
		ret = pdata->power(1);
		if (ret < 0) {
			printk(KERN_ERR "[TP]power on failed\n");
			goto err_power_failed;
		}
	}
	ts->usb_connected = 0x00;

	if (pdata->loadSensorConfig(client, pdata, &(ts->i2c_api)) < 0) {
		printk(KERN_ERR "[TP]%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
		goto err_detect_failed;
	}
	ts->power = pdata->power;
	ts->pdata = pdata;
	if (pdata->event_htc_enable)
		ts->event_htc_enable_type = SWITCH_TO_HTC_EVENT_ONLY;
	else if (pdata->support_htc_event)
		ts->event_htc_enable_type = INJECT_HTC_EVENT;

	i2c_himax_read(ts->client, 0x31, data, 3, HIMAX_I2C_RETRY_TIMES);
	i2c_himax_read(ts->client, 0x32, &data[3], 1, HIMAX_I2C_RETRY_TIMES);
	printk(KERN_INFO "[TP]0x31=> 0x%2.2X 0x%2.2X 0x%2.2X FW ver:0x%2.2X\n",
		data[0], data[1], data[2], data[3]);

	ts->fw_ver = data[3];
	i2c_himax_read(ts->client, 0xEA, &data[0], 2, HIMAX_I2C_RETRY_TIMES);
	ts->x_channel = data[0];
	ts->y_channel = data[1];

	ts->diag_mutual = kzalloc(ts->x_channel * ts->y_channel * sizeof(uint16_t),
		GFP_KERNEL);
	if (ts->diag_mutual == NULL) {
		printk(KERN_ERR "[TP]%s: allocate diag_mutual failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_diag_mutual_failed;
	}

	ts->cable_config = pdata->cable_config;
	ts->protocol_type = pdata->protocol_type;
	printk(KERN_INFO "[TP]%s: Use Protocol Type %c\n", __func__,
		 ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "[TP]Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "himax-touchscreen";
	ts->input_dev->id.version = ts->fw_ver | ts->pdata->version << 8;

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		ts->input_dev->mtsize = HIMAX8526A_FINGER_SUPPORT_NUM;
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
		0, 3, 0, 0);
	} else {/* PROTOCOL_TYPE_B */
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		input_mt_init_slots(ts->input_dev, HIMAX8526A_FINGER_SUPPORT_NUM);
	}

	printk(KERN_INFO "[TP]input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		pdata->abs_x_min, pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		pdata->abs_x_min, pdata->abs_x_max, 1, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		pdata->abs_y_min, pdata->abs_y_max, 1, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
		pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		pdata->abs_width_min, pdata->abs_width_max, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
		0, ((pdata->abs_pressure_max << 16) | pdata->abs_width_max), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, (BIT(31) | (pdata->abs_x_max << 16) | pdata->abs_y_max), 0, 0);


	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "[TP]%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	private_ts = ts;
	himax_touch_sysfs_init();

#ifdef FAKE_EVENT
	ts->fake_X_S = 10;
	ts->fake_Y_S = 400;
	ts->fake_X_E = 400;
	ts->fake_Y_E = 400;
#endif

	if (ts->cable_config)
		usb_register_notifier(&himax_cable_status_handler);

	if (client->irq) {
		ts->use_irq = 1;
		ret = request_threaded_irq(client->irq, NULL, himax_ts_thread,
				  IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts);
		if (ret == 0) {
			printk(KERN_INFO "[TP]%s: irq enabled at qpio: %d\n", __func__, client->irq);
			ts->irq = pdata->gpio_irq;
		} else {
			ts->use_irq = 0;
			printk(KERN_ERR "[TP]request_irq failed\n");
		}
	} else {
		printk(KERN_INFO "[TP]%s: client->irq is empty, use polling mode.\n", __func__);
	}

	if (!ts->use_irq) {
		ts->himax_wq = create_singlethread_workqueue("himax_touch");
		if (!ts->himax_wq)
			goto err_create_wq_failed;

		INIT_WORK(&ts->work, himax_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		printk(KERN_INFO "[TP]%s: polling mode enabled\n", __func__);
	}
	return 0;

err_create_wq_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_alloc_diag_mutual_failed:
err_detect_failed:
err_platform_data_null:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}

static int himax8526a_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	himax_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);

	if (!ts->use_irq)
		hrtimer_cancel(&ts->timer);

	destroy_workqueue(ts->himax_wq);

	if (ts->protocol_type == PROTOCOL_TYPE_B)
		input_mt_destroy_slots(ts->input_dev);

	input_unregister_device(ts->input_dev);
	kfree(ts->diag_mutual);
	kfree(ts);

	return 0;

}

static int himax8526a_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	uint8_t data = 0x01;
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	uint8_t new_command[2] = {0x91, 0x00};

	i2c_himax_master_write(ts->client, new_command, sizeof(new_command),
		 HIMAX_I2C_RETRY_TIMES);

	printk(KERN_DEBUG "[TP]%s: diag_command= %d\n", __func__, ts->diag_command);

	printk(KERN_INFO "[TP]%s: enter\n", __func__);

	disable_irq(client->irq);

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);
		if (ret)
			enable_irq(client->irq);
	}

	i2c_himax_write_command(ts->client, 0x82, HIMAX_I2C_RETRY_TIMES);
	msleep(120);
	i2c_himax_write_command(ts->client, 0x80, HIMAX_I2C_RETRY_TIMES);
	msleep(120);
	i2c_himax_write(ts->client, 0xD7, &data, 1, HIMAX_I2C_RETRY_TIMES);

	ts->first_pressed = 0;
	ts->suspend_mode = 1;
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(0);

	return 0;
}

static int himax8526a_resume(struct i2c_client *client)
{
	uint8_t data[2] = { 0 };
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
	uint8_t new_command[2] = {0x91, 0x00};

	struct himax_ts_data *ts = i2c_get_clientdata(client);
	printk(KERN_INFO "[TP]%s: enter\n", __func__);
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(1);

	data[0] = 0x00;
	i2c_himax_write(ts->client, 0xD7, &data[0], 1, HIMAX_I2C_RETRY_TIMES);
	usleep(100);

	data[0] = 0x42;
	data[1] = 0x02;
	i2c_himax_master_write(ts->client, data, sizeof(data), HIMAX_I2C_RETRY_TIMES);

	i2c_himax_write_command(ts->client, 0x81, HIMAX_I2C_RETRY_TIMES);
	msleep(50);

	data[0] = 0x02;
	i2c_himax_write(ts->client, 0x35, &data[0], 1, HIMAX_I2C_RETRY_TIMES);

	if (ts->pdata->regCD) {
		usleep(100);
		data[0] = 0x0F;
		data[1] = 0x53;
		i2c_himax_write(ts->client, 0x36, &data[0], 2, HIMAX_I2C_RETRY_TIMES);
		usleep(100);
		i2c_himax_master_write(ts->client, ts->pdata->regCD, 3, HIMAX_I2C_RETRY_TIMES);
		printk(KERN_INFO "[TP]%s: Issue 0x36, 0xDD to prevent potential ESD problem.\n", __func__);
	}

	i2c_himax_write_command(ts->client, 0x83, HIMAX_I2C_RETRY_TIMES);

	printk(KERN_DEBUG "[TP]%s: diag_command= %d\n", __func__, ts->diag_command);

	msleep(10);
	if (ts->diag_command == 1 || ts->diag_command == 3 || ts->diag_command == 5) {
		new_command[1] = command_ec_128_raw_baseline_flag;
		i2c_himax_master_write(ts->client, new_command, sizeof(new_command), HIMAX_I2C_RETRY_TIMES);
	} else if (ts->diag_command == 2 || ts->diag_command == 4 || ts->diag_command == 6) {
		new_command[1] = command_ec_128_raw_flag;
		i2c_himax_master_write(ts->client, new_command, sizeof(new_command), HIMAX_I2C_RETRY_TIMES);
	}
	if (ts->usb_connected)
		ts->cable_config[1] = 0x01;
	else
		ts->cable_config[1] = 0x00;

	i2c_himax_master_write(ts->client, ts->cable_config,
		 sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);

	ts->suspend_mode = 0;

	enable_irq(client->irq);

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax8526a_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax8526a_resume(ts->client);

}
#endif

static const struct i2c_device_id himax8526a_ts_id[] = {
	{HIMAX8526A_NAME, 0 },
	{}
};

static struct i2c_driver himax8526a_driver = {
	.id_table	= himax8526a_ts_id,
	.probe		= himax8526a_probe,
	.remove		= himax8526a_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= himax8526a_suspend,
	.resume		= himax8526a_resume,
#endif
	.driver		= {
		.name = HIMAX8526A_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init himax8526a_init(void)
{
	return i2c_add_driver(&himax8526a_driver);
}

static void __exit himax8526a_exit(void)
{
	i2c_del_driver(&himax8526a_driver);
}

module_init(himax8526a_init);
module_exit(himax8526a_exit);

MODULE_DESCRIPTION("Himax8526a driver");
MODULE_LICENSE("GPL");
