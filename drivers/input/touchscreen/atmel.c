/* drivers/input/touchscreen/atmel.c - ATMEL Touch driver
 *
 * Copyright (C) 2009 HTC Corporation.
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <linux/atmel_qt602240.h>
#include <linux/jiffies.h>
#include <mach/msm_hsusb.h>
#include <linux/slab.h>

#define ATMEL_EN_SYSFS
#define ATMEL_I2C_RETRY_TIMES 10

struct atmel_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *atmel_wq;
	struct work_struct work;
	int (*power) (int on);
	struct early_suspend early_suspend;
	struct info_id_t *id;
	struct object_t *object_table;
	uint8_t finger_count;
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	uint8_t first_pressed;
	uint8_t debug_log_level;
	struct atmel_finger_data finger_data[10];
	uint8_t finger_type;
	uint8_t finger_support;
	uint16_t finger_pressed;
	uint8_t face_suppression;
	uint8_t grip_suppression;
	uint8_t noise_status[2];
	uint16_t *filter_level;
	uint8_t calibration_confirm;
	uint64_t timestamp;
	struct atmel_config_data config_setting[2];
	uint8_t status;
	uint8_t GCAF_sample;
	uint8_t *GCAF_level;
	uint8_t noisethr;
#ifdef ATMEL_EN_SYSFS
	struct device dev;
#endif

};

static struct atmel_ts_data *private_ts;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h);
static void atmel_ts_late_resume(struct early_suspend *h);
#endif

int i2c_atmel_read(struct i2c_client *client, uint16_t address, uint8_t *data, uint8_t length)
{
	int retry;
	uint8_t addr[2];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	addr[0] = address & 0xFF;
	addr[1] = (address >> 8) & 0xFF;

	for (retry = 0; retry < ATMEL_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(10);
	}
	if (retry == ATMEL_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",
			ATMEL_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

int i2c_atmel_write(struct i2c_client *client, uint16_t address, uint8_t *data, uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length + 2];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 2,
			.buf = buf,
		}
	};

	buf[0] = address & 0xFF;
	buf[1] = (address >> 8) & 0xFF;

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 2] = data[loop_i];

	for (retry = 0; retry < ATMEL_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == ATMEL_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_write_block retry over %d\n",
			ATMEL_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

int i2c_atmel_write_byte_data(struct i2c_client *client, uint16_t address, uint8_t value)
{
	i2c_atmel_write(client, address, &value, 1);
	return 0;
}

uint16_t get_object_address(struct atmel_ts_data *ts, uint8_t object_type)
{
	uint8_t loop_i;
	for (loop_i = 0; loop_i < ts->id->num_declared_objects; loop_i++) {
		if (ts->object_table[loop_i].object_type == object_type)
			return ts->object_table[loop_i].i2c_address;
	}
	return 0;
}
uint8_t get_object_size(struct atmel_ts_data *ts, uint8_t object_type)
{
	uint8_t loop_i;
	for (loop_i = 0; loop_i < ts->id->num_declared_objects; loop_i++) {
		if (ts->object_table[loop_i].object_type == object_type)
			return ts->object_table[loop_i].size;
	}
	return 0;
}

uint8_t get_report_ids_size(struct atmel_ts_data *ts, uint8_t object_type)
{
	uint8_t loop_i;
	for (loop_i = 0; loop_i < ts->id->num_declared_objects; loop_i++) {
		if (ts->object_table[loop_i].object_type == object_type)
			return ts->object_table[loop_i].report_ids;
	}
	return 0;
}

#ifdef ATMEL_EN_SYSFS
static ssize_t atmel_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct atmel_ts_data *ts_data;
	struct atmel_i2c_platform_data *pdata;

	ts_data = private_ts;
	pdata = ts_data->client->dev.platform_data;

	ret = gpio_get_value(pdata->gpio_irq);
	printk(KERN_DEBUG "GPIO_TP_INT_N=%d\n", pdata->gpio_irq);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(gpio, 0444, atmel_gpio_show, NULL);
static ssize_t atmel_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct atmel_ts_data *ts_data;
	ts_data = private_ts;
	sprintf(buf, "%s_x%4.4X_x%4.4X\n", "ATMEL",
		ts_data->id->family_id, ts_data->id->version);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, 0444, atmel_vendor_show, NULL);

static uint16_t atmel_reg_addr;

static ssize_t atmel_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t ptr[1];
	struct atmel_ts_data *ts_data;
	ts_data = private_ts;
	if (i2c_atmel_read(ts_data->client, atmel_reg_addr, ptr, 1) < 0) {
		printk(KERN_WARNING "%s: read fail\n", __func__);
		return ret;
	}
	ret += sprintf(buf, "addr: %d, data: %d\n", atmel_reg_addr, ptr[0]);
	return ret;
}

static ssize_t atmel_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct atmel_ts_data *ts_data;
	char buf_tmp[4], buf_zero[200];
	uint8_t write_da;

	ts_data = private_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
		(buf[5] == ':' || buf[5] == '\n')) {
		memcpy(buf_tmp, buf + 2, 3);
		atmel_reg_addr = simple_strtol(buf_tmp, NULL, 10);
		printk(KERN_DEBUG "read addr: 0x%X\n", atmel_reg_addr);
		if (!atmel_reg_addr) {
			printk(KERN_WARNING "%s: string to number fail\n",
								__func__);
			return count;
		}
		printk(KERN_DEBUG "%s: set atmel_reg_addr is: %d\n",
						__func__, atmel_reg_addr);
		if (buf[0] == 'w' && buf[5] == ':' && buf[9] == '\n') {
			memcpy(buf_tmp, buf + 6, 3);
			write_da = simple_strtol(buf_tmp, NULL, 10);
			printk(KERN_DEBUG "write addr: 0x%X, data: 0x%X\n",
						atmel_reg_addr, write_da);
			ret = i2c_atmel_write_byte_data(ts_data->client,
						atmel_reg_addr, write_da);
			if (ret < 0) {
				printk(KERN_ERR "%s: write fail(%d)\n",
								__func__, ret);
			}
		}
	}
	if ((buf[0] == '0') && (buf[1] == ':') && (buf[5] == ':')) {
		memcpy(buf_tmp, buf + 2, 3);
		atmel_reg_addr = simple_strtol(buf_tmp, NULL, 10);
		memcpy(buf_tmp, buf + 6, 3);
		memset(buf_zero, 0x0, sizeof(buf_zero));
		ret = i2c_atmel_write(ts_data->client, atmel_reg_addr,
			buf_zero, simple_strtol(buf_tmp, NULL, 10) - atmel_reg_addr + 1);
		if (buf[9] == 'r') {
			i2c_atmel_write_byte_data(ts_data->client,
				get_object_address(ts_data, GEN_COMMANDPROCESSOR_T6) + 1, 0x55);
			i2c_atmel_write_byte_data(ts_data->client,
				get_object_address(ts_data, GEN_COMMANDPROCESSOR_T6), 0x11);
		}
	}

	return count;
}

static DEVICE_ATTR(register, 0644, atmel_register_show, atmel_register_store);

static ssize_t atmel_regdump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, ret_t = 0;
	struct atmel_ts_data *ts_data;
	uint16_t loop_i;
	uint8_t ptr[1];
	ts_data = private_ts;
	if (ts_data->id->version >= 0x14) {
		for (loop_i = 230; loop_i <= 425; loop_i++) {
			ret_t = i2c_atmel_read(ts_data->client, loop_i, ptr, 1);
			if (ret_t < 0) {
				printk(KERN_WARNING "dump fail, addr: %d\n",
								loop_i);
			}
			count += sprintf(buf + count, "addr[%3d]: %3d, ",
								loop_i , *ptr);
			if (((loop_i - 230) % 4) == 3)
				count += sprintf(buf + count, "\n");
		}
		count += sprintf(buf + count, "\n");
	}
	return count;
}

static ssize_t atmel_regdump_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct atmel_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts_data->debug_log_level = buf[0] - 0x30;

	return count;

}

static DEVICE_ATTR(regdump, 0644, atmel_regdump_show, atmel_regdump_dump);

static ssize_t atmel_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmel_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	count += sprintf(buf, "%d\n", ts_data->debug_log_level);

	return count;
}

static ssize_t atmel_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct atmel_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts_data->debug_log_level = buf[0] - 0x30;

	return count;
}

static DEVICE_ATTR(debug_level, 0644, atmel_debug_level_show, atmel_debug_level_dump);

static struct kobject *android_touch_kobj;

static int atmel_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	atmel_reg_addr = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_regdump.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	return 0;
}

static void atmel_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_regdump.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

#endif
static int check_delta(struct atmel_ts_data*ts)
{
	int8_t data[128];
	uint8_t loop_i;
	int16_t rawdata, count = 0;

	memset(data, 0xFF, sizeof(data));
	i2c_atmel_write_byte_data(ts->client,
		get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 5, 0x10);

	for (loop_i = 0; !(data[0] == 0x10 && data[1] == 0x00) && loop_i < 10; loop_i++) {
		msleep(5);
		i2c_atmel_read(ts->client, get_object_address(ts, DIAGNOSTIC_T37), data, 2);
	}
	if (loop_i == 10)
		printk(KERN_ERR "%s: Diag data not ready\n", __func__);

	i2c_atmel_read(ts->client, get_object_address(ts, DIAGNOSTIC_T37), data, 128);
	if (data[0] == 0x10 && data[1] == 0x00) {
		for (loop_i = 2; loop_i < 127; loop_i += 2) {
			rawdata = data[loop_i+1] << 8 | data[loop_i];
			if (abs(rawdata) > 50)
				count++;
		}
		if (count > 32)
			return 1;
	}
	return 0;
}

static void check_calibration(struct atmel_ts_data*ts)
{
	uint8_t data[82];
	uint8_t loop_i, loop_j, x_limit = 0, check_mask, tch_ch = 0, atch_ch = 0;

	memset(data, 0xFF, sizeof(data));
	i2c_atmel_write_byte_data(ts->client,
		get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 5, 0xF3);

	for (loop_i = 0; !(data[0] == 0xF3 && data[1] == 0x00) && loop_i < 10; loop_i++) {
		msleep(5);
		i2c_atmel_read(ts->client, get_object_address(ts, DIAGNOSTIC_T37), data, 2);
	}

	if (loop_i == 10)
		printk(KERN_ERR "%s: Diag data not ready\n", __func__);

	i2c_atmel_read(ts->client, get_object_address(ts, DIAGNOSTIC_T37), data, 82);
	if (data[0] == 0xF3 && data[1] == 0x00) {
		x_limit = 16 + ts->config_setting[0].config_T28[2];
		x_limit = x_limit << 1;

		for (loop_i = 0; loop_i < x_limit; loop_i += 2) {
			for (loop_j = 0; loop_j < 8; loop_j++) {
				check_mask = 1 << loop_j;
				if (data[2 + loop_i] & check_mask)
					tch_ch++;
				if (data[3 + loop_i] & check_mask)
					tch_ch++;
				if (data[42 + loop_i] & check_mask)
					atch_ch++;
				if (data[43 + loop_i] & check_mask)
					atch_ch++;
			}
		}
	}
	i2c_atmel_write_byte_data(ts->client,
		get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 5, 0x01);

	if (tch_ch && (atch_ch == 0)) {
		if (jiffies > (ts->timestamp + HZ/2) && (ts->calibration_confirm == 1)) {
			ts->calibration_confirm = 2;
			printk(KERN_INFO "%s: calibration confirm\n", __func__);
			i2c_atmel_write_byte_data(ts->client,
			get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 6,
				ts->config_setting[ts->status].config_T8[6]);
			i2c_atmel_write_byte_data(ts->client,
				get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 7,
				ts->config_setting[ts->status].config_T8[7]);
		}
		if (ts->calibration_confirm < 2)
			ts->calibration_confirm = 1;
		ts->timestamp = jiffies;
	} else if ((tch_ch - 25) <= atch_ch && (tch_ch || atch_ch)) {
		ts->calibration_confirm = 0;
		i2c_atmel_write_byte_data(ts->client,
			get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 2, 0x55);
	}
}

static void atmel_ts_work_func(struct work_struct *work)
{
	int ret;
	struct atmel_ts_data *ts = container_of(work, struct atmel_ts_data, work);
	uint8_t data[ts->finger_support * 9];

	uint8_t loop_i, loop_j, report_type, msg_num, msg_byte_num = 8, finger_report;
	msg_num = (ts->finger_count && ts->id->version >= 0x15)
		? ts->finger_count : 1;

	ret = i2c_atmel_read(ts->client, get_object_address(ts,
		GEN_MESSAGEPROCESSOR_T5), data, msg_num * 9 - 2);

	if (ts->debug_log_level & 0x1) {
		for (loop_i = 0; loop_i < msg_num * 9 - 2; loop_i++)
			printk("0x%2.2X ", data[loop_i]);
		printk("\n");
	}

	if (ts->id->version >= 0x15) {
	for (loop_i = 0; loop_i < msg_num; loop_i++) {
		report_type = data[loop_i * 9] - ts->finger_type;
		if (report_type >= 0 && report_type < ts->finger_support) {
			if (ts->calibration_confirm < 2 && ts->id->version >= 0x16)
				check_calibration(ts);
			ts->finger_data[report_type].x = data[loop_i * 9 + 2] << 2 | data[loop_i * 9 + 4] >> 6;
			ts->finger_data[report_type].y = data[loop_i * 9 + 3] << 2 | (data[loop_i * 9 + 4] & 0x0C) >> 2;
			ts->finger_data[report_type].w = data[loop_i * 9 + 5];
			ts->finger_data[report_type].z = data[loop_i * 9 + 6];
			if (data[loop_i * 9 + 1] & 0x20) {
				if ((ts->grip_suppression >> report_type) & 1)
					ts->grip_suppression &= ~(1 << report_type);
				if (((ts->finger_pressed >> report_type) & 1) == 1) {
					ts->finger_count--;
					ts->finger_pressed &= ~(1 << report_type);
					if (!ts->first_pressed) {
						if (!ts->finger_count)
							ts->first_pressed = 1;
						printk(KERN_INFO "E%d@%d,%d\n", report_type + 1,
								ts->finger_data[report_type].x, ts->finger_data[report_type].y);
					}
				}
			} else if ((data[loop_i * 9 + 1] & 0xC0) && (((ts->finger_pressed >> report_type) & 1) == 0)) {
				if (ts->filter_level[0]) {
					if (ts->finger_data[report_type].x < ts->filter_level[0] || ts->finger_data[report_type].x > ts->filter_level[3])
						ts->grip_suppression |= 1 << report_type;
					else if ((ts->finger_data[report_type].x < ts->filter_level[1] || ts->finger_data[report_type].x > ts->filter_level[2])
						&& ((ts->grip_suppression >> report_type) & 1))
						ts->grip_suppression |= 1 << report_type;
					else if (ts->finger_data[report_type].x > ts->filter_level[1] && ts->finger_data[report_type].x < ts->filter_level[2])
						ts->grip_suppression &= ~(1 << report_type);
				}
				if (((ts->grip_suppression >> report_type) & 1) == 0) {
					if (!ts->first_pressed)
						printk(KERN_INFO "S%d@%d,%d\n", report_type + 1,
							ts->finger_data[report_type].x, ts->finger_data[report_type].y);
					ts->finger_count++;
					ts->finger_pressed |= 1 << report_type;
				}
			}
		} else {
			if (data[loop_i * 9] == get_report_ids_size(ts, GEN_COMMANDPROCESSOR_T6)) {
				printk(KERN_INFO "Touch Status: ");
				msg_byte_num = 5;
			} else if (data[loop_i * 9] == get_report_ids_size(ts, PROCI_GRIPFACESUPPRESSION_T20)) {
				if (ts->calibration_confirm < 2 && ts->id->version >= 0x16)
					check_calibration(ts);
				ts->face_suppression = data[loop_i * 9 + 1];
				printk(KERN_INFO "Touch Face suppression %s: ",
					ts->face_suppression ? "Active" : "Inactive");
				msg_byte_num = 2;
			} else if (data[loop_i * 9] == get_report_ids_size(ts, PROCG_NOISESUPPRESSION_T22)) {
				if (data[loop_i * 9 + 1] == 0x10) /* reduce message print */
					msg_byte_num = 0;
				else {
					printk(KERN_INFO "Touch Noise suppression: ");
					msg_byte_num = 4;
					if (ts->status && data[loop_i * 9 + 2] >= ts->GCAF_sample) {
						i2c_atmel_write_byte_data(ts->client,
							get_object_address(ts, GEN_POWERCONFIG_T7), 0x08);
						i2c_atmel_write_byte_data(ts->client,
							get_object_address(ts, GEN_POWERCONFIG_T7) + 1, 0x08);
						for (loop_j = 0; loop_j < 5; loop_j++) {
							if (ts->GCAF_sample < ts->GCAF_level[loop_j]) {
								ts->GCAF_sample = ts->GCAF_level[loop_j];
								break;
							}
						}
						if (loop_j == 5)
							ts->GCAF_sample += 24;
						if (ts->GCAF_sample >= 63) {
							ts->GCAF_sample = 63;
							i2c_atmel_write_byte_data(ts->client,
								get_object_address(ts, PROCG_NOISESUPPRESSION_T22) + 8,
								ts->config_setting[1].config[1]);
							i2c_atmel_write_byte_data(ts->client,
								get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 7, 0x1);
						}
						i2c_atmel_write_byte_data(ts->client,
							get_object_address(ts, SPT_CTECONFIG_T28) + 4, ts->GCAF_sample);
					}
					if (data[loop_i * 9 + 1] & 0x0C && ts->GCAF_sample == 63) {
						ts->noisethr += 30;
						if (ts->noisethr > 255)
							ts->noisethr = 255;
						i2c_atmel_write_byte_data(ts->client,
							get_object_address(ts, PROCG_NOISESUPPRESSION_T22) + 8,
							ts->noisethr);
					}
				}
			}
			if (data[loop_i * 9] != 0xFF) {
				for (loop_j = 0; loop_j < msg_byte_num; loop_j++)
					printk("0x%2.2X ", data[loop_i * 9 + loop_j]);
				if (msg_byte_num)
					printk("\n");
			}
		}
		if (loop_i == msg_num - 1) {
			if (!ts->finger_count || ts->face_suppression) {
				ts->finger_pressed = 0;
				ts->finger_count = 0;
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				if (ts->debug_log_level & 0x2)
					printk(KERN_INFO "Finger leave\n");
			} else {
				for (loop_i = 0, finger_report = 0; loop_i < ts->finger_support; loop_i++) {
					if (((ts->finger_pressed >> loop_i) & 1) == 1) {
						finger_report++;
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							ts->finger_data[loop_i].z);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							ts->finger_data[loop_i].w);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							ts->finger_data[loop_i].x);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							ts->finger_data[loop_i].y);
						input_mt_sync(ts->input_dev);
#else
						input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
							ts->finger_data[loop_i].z << 16 | ts->finger_data[loop_i].w);
						input_report_abs(ts->input_dev, ABS_MT_POSITION,
							(ts->finger_count == finger_report) << 31 |
							ts->finger_data[loop_i].x << 16 | ts->finger_data[loop_i].y);
#endif
						if (ts->debug_log_level & 0x2)
							printk(KERN_INFO "Finger %d=> X:%d, Y:%d w:%d, z:%d, F:%d\n",
								loop_i + 1, ts->finger_data[loop_i].x,
								ts->finger_data[loop_i].y, ts->finger_data[loop_i].w,
								ts->finger_data[loop_i].z, ts->finger_count);
					}
				}
			}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
			input_sync(ts->input_dev);
#endif
		}
	}
	}	else { /*read one message one time */
	report_type = data[0] - ts->finger_type;
	if (report_type >= 0 && report_type < ts->finger_support) {
		/* for issue debug only */
		if ((data[1] & 0x60) == 0x60)
			printk(KERN_INFO"x60 ISSUE happened: %x, %x, %x, %x, %x, %x, %x, %x\n",
				data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		if ((data[1] & 0x20) && (((ts->finger_pressed >> report_type) & 1) == 1)) {
			ts->finger_count--;
			ts->finger_pressed &= ~(1 << report_type);
		} else if ((data[1] & 0xC0) && (((ts->finger_pressed >> report_type) & 1) == 0)) {
			ts->finger_count++;
			ts->finger_pressed |= 1 << report_type;
		}
		ts->finger_data[report_type].x = data[2] << 2 | data[4] >> 6;
		ts->finger_data[report_type].y = data[3] << 2 | (data[4] & 0x0C) >> 2;
		ts->finger_data[report_type].w = data[5];
		ts->finger_data[report_type].z = data[6];
		if (!ts->finger_count) {
			ts->finger_pressed = 0;
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			if (ts->debug_log_level & 0x2)
				printk(KERN_INFO "Finger leave\n");
		} else {
			for (loop_i = 0; loop_i < ts->finger_support; loop_i++) {
				if (((ts->finger_pressed >> loop_i) & 1) == 1) {
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
						ts->finger_data[loop_i].z);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
						ts->finger_data[loop_i].w);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
						ts->finger_data[loop_i].x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
						ts->finger_data[loop_i].y);
					input_mt_sync(ts->input_dev);
					if (ts->debug_log_level & 0x2)
						printk(KERN_INFO "Finger %d=> X:%d, Y:%d w:%d, z:%d, F:%d\n",
							loop_i + 1, ts->finger_data[loop_i].x,
							ts->finger_data[loop_i].y, ts->finger_data[loop_i].w,
							ts->finger_data[loop_i].z, ts->finger_count);
				}
			}
		}
		input_sync(ts->input_dev);
	} else
		printk(KERN_INFO"RAW data: %x, %x, %x, %x, %x, %x, %x, %x\n",
			data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	}
	enable_irq(ts->client->irq);
}

static irqreturn_t atmel_ts_irq_handler(int irq, void *dev_id)
{
	struct atmel_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->atmel_wq, &ts->work);
	return IRQ_HANDLED;
}

static void cable_tp_status_handler_func(int connected)
{
	struct atmel_ts_data *ts;
	ts = private_ts;
	printk(KERN_INFO "Touch: cable change to %d\n", connected);
	if (connected != ts->status) {
		if (connected)
			ts->status = 1;
		else
			ts->status = 0;

		if (ts->config_setting[1].config[0]) {
			if (ts->status) {
				ts->calibration_confirm = 2;
				i2c_atmel_write_byte_data(ts->client,
					get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 6,
					ts->config_setting[ts->status].config_T8[6]);
				i2c_atmel_write_byte_data(ts->client,
					get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 7,
					ts->config_setting[ts->status].config_T8[7]);
			}
			i2c_atmel_write_byte_data(ts->client,
				get_object_address(ts, TOUCH_MULTITOUCHSCREEN_T9) + 7,
				ts->config_setting[ts->status].config[0]);
			if (!ts->status)
				i2c_atmel_write_byte_data(ts->client,
					get_object_address(ts, PROCG_NOISESUPPRESSION_T22) + 8,
					ts->config_setting[ts->status].config[1]);
			i2c_atmel_write_byte_data(ts->client,
				get_object_address(ts, SPT_CTECONFIG_T28) + 3,
				ts->config_setting[ts->status].config[2]);
			i2c_atmel_write_byte_data(ts->client,
				get_object_address(ts, SPT_CTECONFIG_T28) + 4,
				ts->config_setting[ts->status].config[3]);
			ts->GCAF_sample = ts->config_setting[1].config[3];
		} else {
			if (ts->config_setting[1].config_T7 != NULL)
				i2c_atmel_write(ts->client,
					get_object_address(ts, GEN_POWERCONFIG_T7),
					ts->config_setting[ts->status].config_T7,
					get_object_size(ts, GEN_POWERCONFIG_T7));
			if (ts->config_setting[1].config_T8 != NULL)
				i2c_atmel_write(ts->client,
					get_object_address(ts, GEN_ACQUISITIONCONFIG_T8),
					ts->config_setting[1].config_T8,
					get_object_size(ts, GEN_ACQUISITIONCONFIG_T8));
			if (ts->config_setting[ts->status].config_T9 != NULL)
				i2c_atmel_write(ts->client,
					get_object_address(ts, TOUCH_MULTITOUCHSCREEN_T9),
					ts->config_setting[ts->status].config_T9,
					get_object_size(ts, TOUCH_MULTITOUCHSCREEN_T9));
			if (ts->config_setting[ts->status].config_T22 != NULL)
				i2c_atmel_write(ts->client,
					get_object_address(ts, PROCG_NOISESUPPRESSION_T22),
					ts->config_setting[ts->status].config_T22,
					get_object_size(ts, PROCG_NOISESUPPRESSION_T22));
			if (ts->config_setting[ts->status].config_T28 != NULL) {
				i2c_atmel_write(ts->client,
					get_object_address(ts, SPT_CTECONFIG_T28),
					ts->config_setting[ts->status].config_T28,
					get_object_size(ts, SPT_CTECONFIG_T28));
				ts->GCAF_sample = ts->config_setting[ts->status].config_T28[4];
			}
		}
	}
}

static struct t_usb_status_notifier cable_status_handler = {
	.name = "usb_tp_connected",
	.func = cable_tp_status_handler_func,
};

static int atmel_ts_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct atmel_ts_data *ts;
	struct atmel_i2c_platform_data *pdata;
	int ret = 0, i = 0, intr = 0;
	uint8_t loop_i;
	struct i2c_msg msg[2];
	uint8_t data[16];
	uint8_t type_count = 0, CRC_check = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR"%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct atmel_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR"%s: allocate atmel_ts_data failed\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->atmel_wq = create_singlethread_workqueue("atmel_wq");
	if (!ts->atmel_wq) {
		printk(KERN_ERR"%s: create workqueue failed\n", __func__);
		ret = -ENOMEM;
		goto err_cread_wq_failed;
	}

	INIT_WORK(&ts->work, atmel_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (pdata) {
		ts->power = pdata->power;
		intr = pdata->gpio_irq;
	}
	if (ts->power) {
		ret = ts->power(1);
		msleep(2);
		if (ret < 0) {
			printk(KERN_ERR "%s:power on failed\n", __func__);
			goto err_power_failed;
		}
	}

	for (loop_i = 0; loop_i < 10; loop_i++) {
		if (!gpio_get_value(intr))
			break;
		msleep(10);
	}

	if (loop_i == 10)
		printk(KERN_ERR "No Messages\n");

	/* read message*/
	msg[0].addr = ts->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 7;
	msg[0].buf = data;
	ret = i2c_transfer(client->adapter, msg, 1);

	if (ret < 0) {
		printk(KERN_INFO "No Atmel chip inside\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "Touch: 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n",
		data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

	if (data[1] & 0x24) {
		printk(KERN_INFO "atmel_ts_probe(): init err: %x\n", data[1]);
		goto err_detect_failed;
	} else {
		for (loop_i = 0; loop_i < 10; loop_i++) {
			if (gpio_get_value(intr)) {
				printk(KERN_INFO "Touch: No more message\n");
				break;
			}
			ret = i2c_transfer(client->adapter, msg, 1);
			printk(KERN_INFO "Touch: 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n",
				data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
			msleep(10);
		}
	}

	/* Read the info block data. */
	ts->id = kzalloc(sizeof(struct info_id_t), GFP_KERNEL);
	if (ts->id == NULL) {
		printk(KERN_ERR"%s: allocate info_id_t failed\n", __func__);
		goto err_alloc_failed;
	}
	ret = i2c_atmel_read(client, 0x00, data, 7);

	ts->id->family_id = data[0];
	ts->id->variant_id = data[1];
	if (ts->id->family_id == 0x80 && ts->id->variant_id == 0x10)
		ts->id->version = data[2] + 6;
	else
		ts->id->version = data[2];
	ts->id->build = data[3];
	ts->id->matrix_x_size = data[4];
	ts->id->matrix_y_size = data[5];
	ts->id->num_declared_objects = data[6];

	printk(KERN_INFO "info block: 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n",
		ts->id->family_id, ts->id->variant_id,
		ts->id->version, ts->id->build,
		ts->id->matrix_x_size, ts->id->matrix_y_size,
		ts->id->num_declared_objects);

	/*
	 Begin ffolkes multitouch selection
	  Purpose: Some hardware can't support more than 3 multitouch points, so revert back to 3 if we find a 004F touch sensor
	 BEGIN
	*/
	if(ts->id->family_id == 0x4F && pdata->config_T9[14] != 3) {
		printk(KERN_INFO "%d point multitouch disabled due to possible hardware conflict, reverting to 3 point\n", pdata->config_T9[14]);
		pdata->config_T9[14] = 3;
	} else {
		printk(KERN_INFO "%d point multitouch enabled\n", pdata->config_T9[14]);
	}
	/*
	 END
	*/
	
	/* Read object table. */
	ts->object_table = kzalloc(sizeof(struct object_t)*ts->id->num_declared_objects, GFP_KERNEL);
	if (ts->object_table == NULL) {
		printk(KERN_ERR"%s: allocate object_table failed\n", __func__);
		goto err_alloc_failed;
	}
	for (i = 0; i < ts->id->num_declared_objects; i++) {
		ret = i2c_atmel_read(client, i * 6 + 0x07, data, 6);
		ts->object_table[i].object_type = data[0];
		ts->object_table[i].i2c_address = data[1] | data[2] << 8;
		ts->object_table[i].size = data[3] + 1;
		ts->object_table[i].instances = data[4];
		ts->object_table[i].num_report_ids = data[5];
		if (data[5]) {
			ts->object_table[i].report_ids = type_count + 1;
			type_count += data[5];
		}
		if (data[0] == 9)
			ts->finger_type = ts->object_table[i].report_ids;
		printk(KERN_INFO "Type: %2.2X, Start: %4.4X, Size: %2X, Instance: %2X, RD#: %2X, %2X\n",
			ts->object_table[i].object_type , ts->object_table[i].i2c_address,
			ts->object_table[i].size, ts->object_table[i].instances,
			ts->object_table[i].num_report_ids, ts->object_table[i].report_ids);
	}

	if (pdata) {
		while (pdata->version > ts->id->version)
			pdata++;
		if (pdata->source) {
			i2c_atmel_write_byte_data(client,
				get_object_address(ts, SPT_GPIOPWM_T19), 0x7);
			for (loop_i = 0; loop_i < 10; loop_i++) {
				if (!gpio_get_value(intr))
					break;
				msleep(10);
			}
			if (loop_i == 10)
				printk(KERN_ERR "No Messages when check source\n");
			for (loop_i = 0; loop_i < 10; loop_i++) {
				i2c_atmel_read(ts->client, get_object_address(ts,
					GEN_MESSAGEPROCESSOR_T5), data, 2);
				if (data[0] == get_report_ids_size(ts, SPT_GPIOPWM_T19)) {
					while ((data[1] >> 3) != pdata->source)
						pdata++;
					break;
				}
			}
		}

		ts->finger_support = pdata->config_T9[14];
		printk(KERN_INFO"finger_type: %d, max finger: %d\n", ts->finger_type, ts->finger_support);

		/* infoamtion block CRC check */
		if (pdata->object_crc[0]) {
			ret = i2c_atmel_write_byte_data(client,
				get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 2, 0x55);
			for (loop_i = 0; loop_i < 10; loop_i++) {
				if (!gpio_get_value(intr)) {
					ret = i2c_atmel_read(ts->client, get_object_address(ts,
						GEN_MESSAGEPROCESSOR_T5), data, 5);
					if (data[0] == get_report_ids_size(ts, GEN_COMMANDPROCESSOR_T6))
						break;
				}
				msleep(10);
			}
			if (loop_i == 10)
				printk(KERN_INFO "Touch: No checksum read\n");
			else {
				for (loop_i = 0; loop_i < 3; loop_i++) {
					if (pdata->object_crc[loop_i] != data[loop_i + 2]) {
						printk(KERN_ERR"CRC Error: %x, %x\n", pdata->object_crc[loop_i], data[loop_i + 2]);
						break;
					}
				}
				if (loop_i == 3) {
					printk(KERN_INFO "CRC passed: ");
					for (loop_i = 0; loop_i < 3; loop_i++)
						printk("0x%2.2X ", pdata->object_crc[loop_i]);
					printk("\n");
					CRC_check = 1;
				}
			}
		}
		ts->abs_x_min = pdata->abs_x_min;
		ts->abs_x_max = pdata->abs_x_max;
		ts->abs_y_min = pdata->abs_y_min;
		ts->abs_y_max = pdata->abs_y_max;
		ts->abs_pressure_min = pdata->abs_pressure_min;
		ts->abs_pressure_max = pdata->abs_pressure_max;
		ts->abs_width_min = pdata->abs_width_min;
		ts->abs_width_max = pdata->abs_width_max;
		ts->GCAF_level = pdata->GCAF_level;
		printk(KERN_INFO "GCAF_level: %d, %d, %d, %d, %d\n",
			ts->GCAF_level[0], ts->GCAF_level[1], ts->GCAF_level[2],
			ts->GCAF_level[3], ts->GCAF_level[4]);
		ts->filter_level = pdata->filter_level;
		printk(KERN_INFO "filter_level: %d, %d, %d, %d\n",
			ts->filter_level[0], ts->filter_level[1], ts->filter_level[2], ts->filter_level[3]);

		ts->config_setting[0].config_T7
			= ts->config_setting[1].config_T7
			= pdata->config_T7;
		ts->config_setting[0].config_T8
			= ts->config_setting[1].config_T8
			= pdata->config_T8;
		ts->config_setting[0].config_T9 = pdata->config_T9;
		ts->config_setting[0].config_T22 = pdata->config_T22;
		ts->config_setting[0].config_T28 = pdata->config_T28;

		if (pdata->cable_config[0]) {
			ts->config_setting[0].config[0] = pdata->config_T9[7];
			ts->config_setting[0].config[1] = pdata->config_T22[8];
			ts->config_setting[0].config[2] = pdata->config_T28[3];
			ts->config_setting[0].config[3] = pdata->config_T28[4];
			for (loop_i = 0; loop_i < 4; loop_i++)
				ts->config_setting[1].config[loop_i] = pdata->cable_config[loop_i];
			ts->GCAF_sample = ts->config_setting[1].config[3];
			ts->noisethr = pdata->cable_config[1];
		} else {
			if (pdata->cable_config_T7[0])
				ts->config_setting[1].config_T7 = pdata->cable_config_T7;
			if (pdata->cable_config_T8[0])
				ts->config_setting[1].config_T8 = pdata->cable_config_T8;
			if (pdata->cable_config_T9[0]) {
				ts->config_setting[1].config_T9 = pdata->cable_config_T9;
				ts->config_setting[1].config_T22 = pdata->cable_config_T22;
				ts->config_setting[1].config_T28 = pdata->cable_config_T28;
				ts->GCAF_sample = ts->config_setting[ts->status].config_T28[4];
			}
		}
#if 0
///fix me!! TODO		if (usb_get_connect_type())
			ts->status = 1;
#endif
		if (!CRC_check) {
			printk(KERN_INFO "Touch: Config reload\n");

			i2c_atmel_write(ts->client, get_object_address(ts, SPT_CTECONFIG_T28),
				pdata->config_T28, get_object_size(ts, SPT_CTECONFIG_T28));

			ret = i2c_atmel_write_byte_data(client, get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 1, 0x55);
			msleep(10);

			ret = i2c_atmel_write_byte_data(client, get_object_address(ts, GEN_COMMANDPROCESSOR_T6), 0x11);
			msleep(64);

			i2c_atmel_write(ts->client, get_object_address(ts, GEN_COMMANDPROCESSOR_T6),
				pdata->config_T6, get_object_size(ts, GEN_COMMANDPROCESSOR_T6));
			i2c_atmel_write(ts->client, get_object_address(ts, GEN_POWERCONFIG_T7),
				pdata->config_T7, get_object_size(ts, GEN_POWERCONFIG_T7));
			i2c_atmel_write(ts->client, get_object_address(ts, GEN_ACQUISITIONCONFIG_T8),
				pdata->config_T8, get_object_size(ts, GEN_ACQUISITIONCONFIG_T8));
			i2c_atmel_write(ts->client, get_object_address(ts, TOUCH_MULTITOUCHSCREEN_T9),
				pdata->config_T9, get_object_size(ts, TOUCH_MULTITOUCHSCREEN_T9));
			i2c_atmel_write(ts->client, get_object_address(ts, TOUCH_KEYARRAY_T15),
				pdata->config_T15, get_object_size(ts, TOUCH_KEYARRAY_T15));
			i2c_atmel_write(ts->client, get_object_address(ts, SPT_GPIOPWM_T19),
				pdata->config_T19, get_object_size(ts, SPT_GPIOPWM_T19));
			i2c_atmel_write(ts->client, get_object_address(ts, PROCI_GRIPFACESUPPRESSION_T20),
				pdata->config_T20, get_object_size(ts, PROCI_GRIPFACESUPPRESSION_T20));
			i2c_atmel_write(ts->client, get_object_address(ts, PROCG_NOISESUPPRESSION_T22),
				pdata->config_T22, get_object_size(ts, PROCG_NOISESUPPRESSION_T22));
			i2c_atmel_write(ts->client, get_object_address(ts, TOUCH_PROXIMITY_T23),
				pdata->config_T23, get_object_size(ts, TOUCH_PROXIMITY_T23));
			i2c_atmel_write(ts->client, get_object_address(ts, PROCI_ONETOUCHGESTUREPROCESSOR_T24),
				pdata->config_T24, get_object_size(ts, PROCI_ONETOUCHGESTUREPROCESSOR_T24));
			i2c_atmel_write(ts->client, get_object_address(ts, SPT_SELFTEST_T25),
				pdata->config_T25, get_object_size(ts, SPT_SELFTEST_T25));
			i2c_atmel_write(ts->client, get_object_address(ts, PROCI_TWOTOUCHGESTUREPROCESSOR_T27),
				pdata->config_T27, get_object_size(ts, PROCI_TWOTOUCHGESTUREPROCESSOR_T27));
			i2c_atmel_write(ts->client, get_object_address(ts, SPT_CTECONFIG_T28),
				pdata->config_T28, get_object_size(ts, SPT_CTECONFIG_T28));

			ret = i2c_atmel_write_byte_data(client, get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 1, 0x55);

			for (loop_i = 0; loop_i < 10; loop_i++) {
				if (!gpio_get_value(intr))
					break;
				printk(KERN_INFO "Touch: wait for Message(%d)\n", loop_i + 1);
				msleep(10);
			}

			i2c_atmel_read(client, get_object_address(ts, GEN_MESSAGEPROCESSOR_T5), data, 7);
			printk(KERN_INFO "Touch: 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n",
				data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

			ret = i2c_atmel_write_byte_data(client, get_object_address(ts, GEN_COMMANDPROCESSOR_T6), 0x11);
			msleep(64);

			i2c_atmel_read(client, get_object_address(ts, GEN_MESSAGEPROCESSOR_T5), data, 7);
			printk(KERN_INFO "Touch: 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n",
				data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
			/* For Ace */
		}

		if (ts->status) {
			printk(KERN_INFO "Touch: set cable config\n");
			if (ts->config_setting[1].config[0]) {
				i2c_atmel_write_byte_data(ts->client,
					get_object_address(ts, TOUCH_MULTITOUCHSCREEN_T9) + 7,
					ts->config_setting[1].config[0]);
				i2c_atmel_write_byte_data(ts->client,
					get_object_address(ts, SPT_CTECONFIG_T28) + 3,
					ts->config_setting[1].config[2]);
				i2c_atmel_write_byte_data(ts->client,
					get_object_address(ts, SPT_CTECONFIG_T28) + 4,
					ts->config_setting[1].config[3]);
			} else {
				if (ts->config_setting[1].config_T7 != NULL)
					i2c_atmel_write(ts->client,
						get_object_address(ts, GEN_POWERCONFIG_T7),
						ts->config_setting[1].config_T7,
						get_object_size(ts, GEN_POWERCONFIG_T7));
				if (ts->config_setting[1].config_T8 != NULL)
					i2c_atmel_write(ts->client,
						get_object_address(ts, GEN_ACQUISITIONCONFIG_T8),
						ts->config_setting[1].config_T8,
						get_object_size(ts, GEN_ACQUISITIONCONFIG_T8));
				if (ts->config_setting[1].config_T9 != NULL)
					i2c_atmel_write(ts->client,
						get_object_address(ts, TOUCH_MULTITOUCHSCREEN_T9),
						ts->config_setting[1].config_T9,
						get_object_size(ts, TOUCH_MULTITOUCHSCREEN_T9));
				if (ts->config_setting[1].config_T22 != NULL)
					i2c_atmel_write(ts->client,
						get_object_address(ts, PROCG_NOISESUPPRESSION_T22),
						ts->config_setting[1].config_T22,
						get_object_size(ts, PROCG_NOISESUPPRESSION_T22));
				if (ts->config_setting[1].config_T28 != NULL) {
					i2c_atmel_write(ts->client,
						get_object_address(ts, SPT_CTECONFIG_T28),
						ts->config_setting[1].config_T28,
						get_object_size(ts, SPT_CTECONFIG_T28));
				}
			}
		}
	}
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "atmel-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
				ts->abs_x_min, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
				ts->abs_y_min, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				ts->abs_pressure_min, ts->abs_pressure_max,
				0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				ts->abs_width_min, ts->abs_width_max, 0, 0);
#ifndef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
		0, ((ts->abs_pressure_max << 16) | ts->abs_width_max), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, ((1 << 31) | (ts->abs_x_max << 16) | ts->abs_y_max), 0, 0);
#endif


	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"atmel_ts_probe: Unable to register %s input device\n",
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	ret = request_irq(client->irq, atmel_ts_irq_handler, IRQF_TRIGGER_LOW,
			client->name, ts);
	if (ret)
		dev_err(&client->dev, "request_irq failed\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = atmel_ts_early_suspend;
	ts->early_suspend.resume = atmel_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	private_ts = ts;
#ifdef ATMEL_EN_SYSFS
	atmel_touch_sysfs_init();
#endif

	dev_info(&client->dev, "Start touchscreen %s in interrupt mode\n",
			ts->input_dev->name);

//fix me TODO	usb_register_notifier(&cable_status_handler);

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_alloc_failed:
err_detect_failed:
err_power_failed:
	destroy_workqueue(ts->atmel_wq);

err_cread_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return ret;
}

static int atmel_ts_remove(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

#ifdef ATMEL_EN_SYSFS
	atmel_touch_sysfs_deinit();
#endif

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	destroy_workqueue(ts->atmel_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int atmel_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

	printk(KERN_INFO "%s: enter\n", __func__);

	disable_irq(client->irq);

	ret = cancel_work_sync(&ts->work);
	if (ret)
		enable_irq(client->irq);

	ts->finger_pressed = 0;
	ts->finger_count = 0;
	ts->first_pressed = 0;

	i2c_atmel_write_byte_data(client,
		get_object_address(ts, GEN_POWERCONFIG_T7), 0x0);
	i2c_atmel_write_byte_data(client,
		get_object_address(ts, GEN_POWERCONFIG_T7) + 1, 0x0);

	return 0;
}

static int atmel_ts_resume(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

	i2c_atmel_write(ts->client, get_object_address(ts, GEN_POWERCONFIG_T7),
		ts->config_setting[ts->status].config_T7, get_object_size(ts, GEN_POWERCONFIG_T7));
	if (ts->config_setting[1].config[0] && ts->status && !check_delta(ts)) {
		ts->calibration_confirm = 2;
		i2c_atmel_write_byte_data(ts->client,
			get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 6,
			ts->config_setting[ts->status].config_T8[6]);
		i2c_atmel_write_byte_data(ts->client,
			get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 7,
			ts->config_setting[ts->status].config_T8[7]);
	} else {
		ts->calibration_confirm = 0;
		i2c_atmel_write_byte_data(client,
			get_object_address(ts, GEN_COMMANDPROCESSOR_T6) + 2, 0x55);
		i2c_atmel_write_byte_data(client,
			get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 6, 0x0);
		i2c_atmel_write_byte_data(client,
			get_object_address(ts, GEN_ACQUISITIONCONFIG_T8) + 7, 0x0);
	}
	enable_irq(client->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h)
{
	struct atmel_ts_data *ts;
	ts = container_of(h, struct atmel_ts_data, early_suspend);
	atmel_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void atmel_ts_late_resume(struct early_suspend *h)
{
	struct atmel_ts_data *ts;
	ts = container_of(h, struct atmel_ts_data, early_suspend);
	atmel_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id atml_ts_i2c_id[] = {
	{ ATMEL_QT602240_NAME, 0 },
	{ }
};

static struct i2c_driver atmel_ts_driver = {
	.id_table = atml_ts_i2c_id,
	.probe = atmel_ts_probe,
	.remove = atmel_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = atmel_ts_suspend,
	.resume = atmel_ts_resume,
#endif
	.driver = {
			.name = ATMEL_QT602240_NAME,
	},
};

static int __devinit atmel_ts_init(void)
{
	printk(KERN_INFO "atmel_ts_init():\n");
	return i2c_add_driver(&atmel_ts_driver);
}

static void __exit atmel_ts_exit(void)
{
	i2c_del_driver(&atmel_ts_driver);
}

module_init(atmel_ts_init);
module_exit(atmel_ts_exit);

MODULE_DESCRIPTION("ATMEL Touch driver");
MODULE_LICENSE("GPL");

