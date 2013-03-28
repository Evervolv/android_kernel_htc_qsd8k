/* linux/arch/arm/mach-msm/board-primou-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Author: Jay Tu <jay_tu@htc.com>
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb-7x30.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/panel_id.h>
#include <mach/debug_display.h>

#include "../board-primou.h"
#include "../devices.h"
#include "../proc_comm.h"

static struct vreg *V_LCMIO_1V8;
static struct vreg *V_LCMIO_2V8;

static struct clk *axi_clk;

#define PWM_USER_DEF	 	142
#define PWM_USER_MIN		30
#define PWM_USER_DIM		9
#define PWM_USER_MAX		255

#define PWM_NOVATEK_DEF		135
#define PWM_NOVATEK_MIN		9
#define PWM_NOVATEK_MAX		255

#define DEFAULT_BRIGHTNESS      PWM_USER_DEF

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
	int last_shrink_br;
} cabc;

int color_enhance_switch = 1;

enum {
	GATE_ON = 1 << 0,
	CABC_STATE,
};

enum led_brightness brightness_value = DEFAULT_BRIGHTNESS;

static struct workqueue_struct *primou_cabc_wq;
static struct delayed_work primou_cabc_work;

extern unsigned long msm_fb_base;

static int
primou_shrink_pwm(int brightness, int user_def,
		int user_min, int user_max, int panel_def,
		int panel_min, int panel_max)
{
	if (brightness < PWM_USER_DIM)
		return 0;

	if (brightness < user_min)
		return panel_min;

	if (brightness > user_def) {
		brightness = (panel_max - panel_def) *
			(brightness - user_def) /
			(user_max - user_def) +
			panel_def;
	} else {
			brightness = (panel_def - panel_min) *
			(brightness - user_min) /
			(user_def - user_min) +
			panel_min;
	}

	return brightness;
}

static void primou_cabc_open(struct work_struct *w)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	PR_DISP_INFO("Open CABC...");
	client->remote_write(client, 0x2C, 0x5300);
}

static void
primou_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	shrink_br = primou_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_NOVATEK_DEF,
				PWM_NOVATEK_MIN, PWM_NOVATEK_MAX);

	if (!client) {
		PR_DISP_INFO("null mddi client");
		return;
	}

	if (cabc.last_shrink_br == shrink_br) {
		PR_DISP_INFO("[BKL] identical shrink_br");
		return;
	}

	mutex_lock(&cabc.lock);

	client->remote_write(client, shrink_br, 0x5100);

	if (cabc.last_shrink_br == 0 && shrink_br)
		/* 1ms is not enough, more than 5ms is okay */
		queue_delayed_work(primou_cabc_wq, &primou_cabc_work, 100);

	/* Update the last brightness */
	cabc.last_shrink_br = shrink_br;
	brightness_value = val;
	mutex_unlock(&cabc.lock);

	PR_DISP_INFO("[BKL] set brightness to %d\n", shrink_br);
}

static enum led_brightness
primou_get_brightness(struct led_classdev *led_cdev)
{

	return brightness_value;
}

static void
primou_backlight_switch(int on)
{
	enum led_brightness val;

	if (on) {
		PR_DISP_DEBUG("[BKL] turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;
		/*LED core uses get_brightness for default value
		If the physical layer is not ready, we should not count on it*/
		if (val == 0)
			val = brightness_value;
		primou_set_brightness(&cabc.lcd_backlight, val);
	} else {
		clear_bit(GATE_ON, &cabc.status);
		cabc.last_shrink_br = 0;
	}
}
static int
primou_cabc_switch(int on)
{
	struct msm_mddi_client_data *client = cabc.client_data;

	if (on) {
		printk(KERN_DEBUG "turn on CABC\n");
		set_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x03, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	} else {
		printk(KERN_DEBUG "turn off CABC\n");
		clear_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	}
	return 1;
}
static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count);
#define CABC_ATTR(name) __ATTR(name, 0644, auto_backlight_show, auto_backlight_store)
static struct device_attribute auto_attr = CABC_ATTR(cabc);

static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				test_bit(CABC_STATE, &cabc.status));
	return i;
}

static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned long res;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		printk(KERN_ERR "invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	if (primou_cabc_switch(!!res))
		count = -EIO;

err_out:
	return count;
}

static int
primou_ce_switch(int on)
{
	struct msm_mddi_client_data *client = cabc.client_data;

	if (on) {
		printk(KERN_DEBUG "turn on color enhancement\n");
		client->remote_write(client, 0x10, 0xb400);
	} else {
		printk(KERN_DEBUG "turn off color enhancement\n");
		client->remote_write(client, 0x00, 0xb400);
	}
	color_enhance_switch = on;
	return 1;
}

static ssize_t
ce_switch_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
ce_switch_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count);
#define Color_enhance_ATTR(name) __ATTR(name, 0644, ce_switch_show, ce_switch_store)
static struct device_attribute ce_attr = Color_enhance_ATTR(color_enhance);

static ssize_t
ce_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				color_enhance_switch);
	return i;
}

static ssize_t
ce_switch_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned long res;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		printk(KERN_ERR "invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	if (primou_ce_switch(!!res))
		count = -EIO;

err_out:
	return count;
}

static int
primou_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;
	PR_DISP_DEBUG("%s\n", __func__);

	mutex_init(&cabc.lock);
	cabc.last_shrink_br = 0;
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = primou_set_brightness;
	cabc.lcd_backlight.brightness_get = primou_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;

	err = device_create_file(cabc.lcd_backlight.dev, &ce_attr);
	if (err)
		goto err_ce_out;


	err = device_create_file(cabc.lcd_backlight.dev, &auto_attr);
	if (err)
		goto err_auto_out;

	primou_cabc_wq = alloc_ordered_workqueue("cabc_wq", 0);
	INIT_DELAYED_WORK(&(primou_cabc_work), primou_cabc_open);

	return 0;

err_auto_out:
		device_remove_file(&pdev->dev, &auto_attr);

err_ce_out:
		device_remove_file(&pdev->dev, &ce_attr);

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

static struct resource resources_msm_fb[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

#define REG_WAIT (0xffff)
struct nov_regs {
	unsigned reg;
	unsigned val;
};

struct nov_regs pro_lgd_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 20},
	{0xf000, 0x55},
	{0xf001, 0xaa},
	{0xf002, 0x52},
	{0xf003, 0x08},
	{0xf004, 0x01},

	{0xb000, 0x0A},
	{0xb001, 0x0A},
	{0xb002, 0x0A},
	{0xb600, 0x43},
	{0xb601, 0x43},
	{0xb602, 0x43},
	{0xb100, 0x0A},
	{0xb101, 0x0A},
	{0xb102, 0x0A},
	{0xb700, 0x33},
	{0xb701, 0x33},
	{0xb702, 0x33},
	{0xb200, 0x01},
	{0xb201, 0x01},
	{0xb202, 0x01},
	{0xb800, 0x24},
	{0xb801, 0x24},
	{0xb802, 0x24},
	{0xb300, 0x08},
	{0xb301, 0x08},
	{0xb302, 0x08},
	{0xb900, 0x22},
	{0xb901, 0x22},
	{0xb902, 0x22},
	{0xbf00, 0x00},
	{0xba00, 0x22},
	{0xba01, 0x22},
	{0xba02, 0x22},
	{0xc200, 0x02},
	{0xbc00, 0x00},
	{0xbc01, 0xA0},
	{0xbc02, 0x00},
	{0xbd00, 0x00},
	{0xbd01, 0xA0},
	{0xbd02, 0x00},
	{0xd000, 0x0f},
	{0xd001, 0x0f},
	{0xd002, 0x10},
	{0xd003, 0x10},
	{0xd100, 0x00},
	{0xd101, 0x5D},
	{0xd102, 0x00},
	{0xd103, 0x76},
	{0xd104, 0x00},
	{0xd105, 0xa1},
	{0xd106, 0x00},
	{0xd107, 0xc2},
	{0xd108, 0x00},
	{0xd109, 0xdc},
	{0xd10a, 0x00},
	{0xd10b, 0xfb},
	{0xd10c, 0x01},
	{0xd10d, 0x11},
	{0xd10e, 0x01},
	{0xd10f, 0x41},
	{0xd110, 0x01},
	{0xd111, 0x6d},
	{0xd112, 0x01},
	{0xd113, 0x9e},
	{0xd114, 0x01},
	{0xd115, 0xc8},
	{0xd116, 0x02},
	{0xd117, 0x16},
	{0xd118, 0x02},
	{0xd119, 0x54},
	{0xd11a, 0x02},
	{0xd11b, 0x55},
	{0xd11c, 0x02},
	{0xd11d, 0x8e},
	{0xd11e, 0x02},
	{0xd11f, 0xc9},
	{0xd120, 0x02},
	{0xd121, 0xf3},
	{0xd122, 0x03},
	{0xd123, 0x22},
	{0xd124, 0x03},
	{0xd125, 0x45},
	{0xd126, 0x03},
	{0xd127, 0x73},
	{0xd128, 0x03},
	{0xd129, 0x8e},
	{0xd12a, 0x03},
	{0xd12b, 0xaa},
	{0xd12c, 0x03},
	{0xd12d, 0xba},
	{0xd12e, 0x03},
	{0xd12f, 0xcb},
	{0xd130, 0x03},
	{0xd131, 0xde},
	{0xd132, 0x03},
	{0xd133, 0xea},
	{0xd200, 0x00},
	{0xd201, 0x5d},
	{0xd202, 0x00},
	{0xd203, 0x76},
	{0xd204, 0x00},
	{0xd205, 0xa1},
	{0xd206, 0x00},
	{0xd207, 0xc2},
	{0xd208, 0x00},
	{0xd209, 0xdc},
	{0xd20a, 0x00},
	{0xd20b, 0xfb},
	{0xd20c, 0x01},
	{0xd20d, 0x11},
	{0xd20e, 0x01},
	{0xd20f, 0x41},
	{0xd210, 0x01},
	{0xd211, 0x6d},
	{0xd212, 0x01},
	{0xd213, 0x9e},
	{0xd214, 0x01},
	{0xd215, 0xc8},
	{0xd216, 0x02},
	{0xd217, 0x16},
	{0xd218, 0x02},
	{0xd219, 0x54},
	{0xd21a, 0x02},
	{0xd21b, 0x55},
	{0xd21c, 0x02},
	{0xd21d, 0x8e},
	{0xd21e, 0x02},
	{0xd21f, 0xc9},
	{0xd220, 0x02},
	{0xd221, 0xf3},
	{0xd222, 0x03},
	{0xd223, 0x22},
	{0xd224, 0x03},
	{0xd225, 0x45},
	{0xd226, 0x03},
	{0xd227, 0x73},
	{0xd228, 0x03},
	{0xd229, 0x8e},
	{0xd22a, 0x03},
	{0xd22b, 0xaa},
	{0xd22c, 0x03},
	{0xd22d, 0xba},
	{0xd22e, 0x03},
	{0xd22f, 0xcb},
	{0xd230, 0x03},
	{0xd231, 0xde},
	{0xd232, 0x03},
	{0xd233, 0xea},
	{0xd300, 0x00},
	{0xd301, 0x5d},
	{0xd302, 0x00},
	{0xd303, 0x76},
	{0xd304, 0x00},
	{0xd305, 0xa1},
	{0xd306, 0x00},
	{0xd307, 0xc2},
	{0xd308, 0x00},
	{0xd309, 0xdc},
	{0xd30a, 0x00},
	{0xd30b, 0xfb},
	{0xd30c, 0x01},
	{0xd30d, 0x11},
	{0xd30e, 0x01},
	{0xd30f, 0x41},
	{0xd310, 0x01},
	{0xd311, 0x6d},
	{0xd312, 0x01},
	{0xd313, 0x9e},
	{0xd314, 0x01},
	{0xd315, 0xc8},
	{0xd316, 0x02},
	{0xd317, 0x16},
	{0xd318, 0x02},
	{0xd319, 0x54},
	{0xd31a, 0x02},
	{0xd31b, 0x55},
	{0xd31c, 0x02},
	{0xd31d, 0x8e},
	{0xd31e, 0x02},
	{0xd31f, 0xc9},
	{0xd320, 0x02},
	{0xd321, 0xf3},
	{0xd322, 0x03},
	{0xd323, 0x22},
	{0xd324, 0x03},
	{0xd325, 0x45},
	{0xd326, 0x03},
	{0xd327, 0x73},
	{0xd328, 0x03},
	{0xd329, 0x8e},
	{0xd32a, 0x03},
	{0xd32b, 0xaa},
	{0xd32c, 0x03},
	{0xd32d, 0xba},
	{0xd32e, 0x03},
	{0xd32f, 0xcb},
	{0xd330, 0x03},
	{0xd331, 0xde},
	{0xd332, 0x03},
	{0xd333, 0xea},
	{0xd400, 0x00},
	{0xd401, 0x05},
	{0xd402, 0x00},
	{0xd403, 0x22},
	{0xd404, 0x00},
	{0xd405, 0x51},
	{0xd406, 0x00},
	{0xd407, 0x73},
	{0xd408, 0x00},
	{0xd409, 0x8d},
	{0xd40a, 0x00},
	{0xd40b, 0xb2},
	{0xd40c, 0x00},
	{0xd40d, 0xc6},
	{0xd40e, 0x00},
	{0xd40f, 0xf0},
	{0xd410, 0x01},
	{0xd411, 0x15},
	{0xd412, 0x01},
	{0xd413, 0x53},
	{0xd414, 0x01},
	{0xd415, 0x89},
	{0xd416, 0x01},
	{0xd417, 0xcf},
	{0xd418, 0x02},
	{0xd419, 0x09},
	{0xd41a, 0x02},
	{0xd41b, 0x0a},
	{0xd41c, 0x02},
	{0xd41d, 0x48},
	{0xd41e, 0x02},
	{0xd41f, 0x98},
	{0xd420, 0x02},
	{0xd421, 0xd1},
	{0xd422, 0x03},
	{0xd423, 0x18},
	{0xd424, 0x03},
	{0xd425, 0x47},
	{0xd426, 0x03},
	{0xd427, 0x73},
	{0xd428, 0x03},
	{0xd429, 0x8e},
	{0xd42a, 0x03},
	{0xd42b, 0xaa},
	{0xd42c, 0x03},
	{0xd42d, 0xba},
	{0xd42e, 0x03},
	{0xd42f, 0xcb},
	{0xd430, 0x03},
	{0xd431, 0xde},
	{0xd432, 0x03},
	{0xd433, 0xea},
	{0xd500, 0x00},
	{0xd501, 0x05},
	{0xd502, 0x00},
	{0xd503, 0x22},
	{0xd504, 0x00},
	{0xd505, 0x51},
	{0xd506, 0x00},
	{0xd507, 0x73},
	{0xd508, 0x00},
	{0xd509, 0x8d},
	{0xd50a, 0x00},
	{0xd50b, 0xb2},
	{0xd50c, 0x00},
	{0xd50d, 0xc6},
	{0xd50e, 0x00},
	{0xd50f, 0xf0},
	{0xd510, 0x01},
	{0xd511, 0x15},
	{0xd512, 0x01},
	{0xd513, 0x53},
	{0xd514, 0x01},
	{0xd515, 0x89},
	{0xd516, 0x01},
	{0xd517, 0xcf},
	{0xd518, 0x02},
	{0xd519, 0x09},
	{0xd51a, 0x02},
	{0xd51b, 0x0a},
	{0xd51c, 0x02},
	{0xd51d, 0x48},
	{0xd51e, 0x02},
	{0xd51f, 0x98},
	{0xd520, 0x02},
	{0xd521, 0xd1},
	{0xd522, 0x03},
	{0xd523, 0x18},
	{0xd524, 0x03},
	{0xd525, 0x47},
	{0xd526, 0x03},
	{0xd527, 0x73},
	{0xd528, 0x03},
	{0xd529, 0x8e},
	{0xd52a, 0x03},
	{0xd52b, 0xaa},
	{0xd52c, 0x03},
	{0xd52d, 0xba},
	{0xd52e, 0x03},
	{0xd52f, 0xcb},
	{0xd530, 0x03},
	{0xd531, 0xde},
	{0xd532, 0x03},
	{0xd533, 0xea},
	{0xd600, 0x00},
	{0xd601, 0x05},
	{0xd602, 0x00},
	{0xd603, 0x22},
	{0xd604, 0x00},
	{0xd605, 0x51},
	{0xd606, 0x00},
	{0xd607, 0x73},
	{0xd608, 0x00},
	{0xd609, 0x8d},
	{0xd60a, 0x00},
	{0xd60b, 0xb2},
	{0xd60c, 0x00},
	{0xd60d, 0xc6},
	{0xd60e, 0x00},
	{0xd60f, 0xf0},
	{0xd610, 0x01},
	{0xd611, 0x15},
	{0xd612, 0x01},
	{0xd613, 0x53},
	{0xd614, 0x01},
	{0xd615, 0x89},
	{0xd616, 0x01},
	{0xd617, 0xcf},
	{0xd618, 0x02},
	{0xd619, 0x09},
	{0xd61a, 0x02},
	{0xd61b, 0x0a},
	{0xd61c, 0x02},
	{0xd61d, 0x48},
	{0xd61e, 0x02},
	{0xd61f, 0x98},
	{0xd620, 0x02},
	{0xd621, 0xd1},
	{0xd622, 0x03},
	{0xd623, 0x18},
	{0xd624, 0x03},
	{0xd625, 0x47},
	{0xd626, 0x03},
	{0xd627, 0x73},
	{0xd628, 0x03},
	{0xd629, 0x8e},
	{0xd62a, 0x03},
	{0xd62b, 0xaa},
	{0xd62c, 0x03},
	{0xd62d, 0xba},
	{0xd62e, 0x03},
	{0xd62f, 0xcb},
	{0xd630, 0x03},
	{0xd631, 0xde},
	{0xd632, 0x03},
	{0xd633, 0xea},
	//pwm control
	{0xf000, 0x55},
	{0xf001, 0xaa},
	{0xf002, 0x52},
	{0xf003, 0x08},
	{0xf004, 0x00},
	{0xB400, 0x10},
	{0xe000, 0x01},
	{0xe001, 0x03},
	//pwm control
	{0xb100, 0xc8},
	{0xb101, 0x00},
	{0xb600, 0x05},
	{0xb700, 0x71},
	{0xb701, 0x71},
	{0xb800, 0x01},
	{0xb801, 0x05},
	{0xb802, 0x05},
	{0xb803, 0x05},
	{0xb900, 0x00},
	{0xb901, 0x40},
	{0xba00, 0x05},
	{0xbc00, 0x00},
	{0xbc01, 0x00},
	{0xbc02, 0x00},
	{0xbd00, 0x01},
	{0xbd01, 0x8c},
	{0xbd02, 0x14},
	{0xbd03, 0x14},
	{0xbd04, 0x00},
	{0xbe00, 0x01},
	{0xbe01, 0x8c},
	{0xbe02, 0x14},
	{0xbe03, 0x14},
	{0xbe04, 0x00},
	{0xbf00, 0x01},
	{0xbf01, 0x8c},
	{0xbf02, 0x14},
	{0xbf03, 0x14},
	{0xbf04, 0x00},
	{0xc900, 0xc2},
	{0xc901, 0x02},
	{0xc902, 0x50},
	{0xc903, 0x50},
	{0xc904, 0x50},
	//CABC
	{0xD400, 0x05},
	{0xDD00, 0x55},

	{0xE400, 0xFF},
	{0xE401, 0xF7},
	{0xE402, 0xEF},
	{0xE403, 0xE7},
	{0xE404, 0xDF},
	{0xE405, 0xD7},
	{0xE406, 0xCF},
	{0xE407, 0xC7},
	{0xE408, 0xBF},
	{0xE409, 0xB7},
	//CABC END
	//Page Enbale
	{0xFF00, 0xAA},
	{0xFF01, 0x55},
	{0xFF02, 0x25},
	{0xFF03, 0x01},
	//saturation off
	{0xF50C, 0x03},
	// Values for Vivid Color start
	{0xF900, 0x0a},
	{0xF901, 0x00},
	{0xF902, 0x0e},
	{0xF903, 0x1f},
	{0xF904, 0x37},
	{0xF905, 0x55},
	{0xF906, 0x6e},
	{0xF907, 0x6e},
	{0xF908, 0x46},
	{0xF909, 0x28},
	{0xF90A, 0x0e},
	// Vivid Color end
	{0x5300, 0x24},
	{0x3600, 0x00},
	{0x4400, 0x01},
	{0x4401, 0x77},
	//CABC Mode Selection
	{0x5500, 0x03},
	{0x5E00, 0x06},
	{0x3500, 0x00},
	{REG_WAIT, 160},
	{0x2900, 0x00},
};

static struct nov_regs sony_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0xF000, 0x55},
	{0xF001, 0xaa},
	{0xF002, 0x52},
	{0xF003, 0x08},
	{0xF004, 0x00},

	{0xB400, 0x10},
	{0xE000, 0x01},
	{0xE001, 0x03},
	{0xB800, 0x00},
	{0xB801, 0x00},
	{0xB802, 0x00},
	{0xB803, 0x00},
	{0xBC00, 0x00},
	{0xBC01, 0x00},
	{0xBC02, 0x00},
	//CABC
	{0xD400, 0x05},
	{0xDD00, 0x55},

	{0xE400, 0xFF},
	{0xE401, 0xF7},
	{0xE402, 0xEF},
	{0xE403, 0xE7},
	{0xE404, 0xDF},
	{0xE405, 0xD7},
	{0xE406, 0xCF},
	{0xE407, 0xC7},
	{0xE408, 0xBF},
	{0xE409, 0xB7},

	//CABC END
	//Page Enbale
	{0xFF00, 0xAA},
	{0xFF01, 0x55},
	{0xFF02, 0x25},
	{0xFF03, 0x01},
	//saturation off
	{0xF50C, 0x03},

	// Values for Vivid Color start
	{0xF900, 0x0a},
	{0xF901, 0x00},
	{0xF902, 0x0e},
	{0xF903, 0x1f},
	{0xF904, 0x37},
	{0xF905, 0x55},
	{0xF906, 0x6e},
	{0xF907, 0x6e},
	{0xF908, 0x46},
	{0xF909, 0x28},
	{0xF90A, 0x0e},
	// Vivid Color end
	{0x3500, 0x00},
	{0x4400, 0x01},
	{0x4401, 0xB3},
	{0x2900, 0x00},
	{REG_WAIT, 40},
	{0x5500, 0x03},
	{0x5E00, 0x06},
	{0x5300, 0x24},
};

static int
primou_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size = 0;
	unsigned reg, val;
	struct nov_regs *init_seq = NULL;

	PR_DISP_INFO("%s(0x%x)\n", __func__, panel_type);
	client_data->auto_hibernate(client_data, 0);

	switch (panel_type) {
	case PANEL_ID_PRIMO_SONY:
		init_seq = sony_init_seq;
		array_size = ARRAY_SIZE(sony_init_seq);
		break;
	case PANEL_ID_PRIMO_LG:
	default:
		init_seq = pro_lgd_init_seq;
		array_size = ARRAY_SIZE(pro_lgd_init_seq);
		break;
	}

	for (i = 0; i < array_size; i++) {
		reg = cpu_to_le32(init_seq[i].reg);
		val = cpu_to_le32(init_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}
	if (color_enhance_switch == 0)
		client_data->remote_write(client_data, 0x00, 0xb400);

	if (test_bit(CABC_STATE, &cabc.status) == 0)
		primou_cabc_switch(0);
	else
		set_bit(CABC_STATE, &cabc.status);

	client_data->auto_hibernate(client_data, 1);

	if (axi_clk)
		clk_set_rate(axi_clk, 0);

	return 0;
}

static int
primou_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	PR_DISP_DEBUG("%s\n", __func__);
	return 0;
}

static int
primou_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	PR_DISP_DEBUG("%s\n", __func__);

	client_data->auto_hibernate(client_data, 0);
	/* stop running cabc work */
	cancel_delayed_work_sync(&primou_cabc_work);
	if (panel_type == PANEL_ID_PRIMO_SONY) {
		client_data->remote_write(client_data, 0x0, 0x5300);
		primou_backlight_switch(LED_OFF);
		client_data->remote_write(client_data, 0, 0x2800);
		client_data->remote_write(client_data, 0, 0x1000);
	} else {
		client_data->remote_write(client_data, 0x0, 0x5300);
		primou_backlight_switch(LED_OFF);
		client_data->remote_write(client_data, 0, 0x2800);
		hr_msleep(10);
		client_data->remote_write(client_data, 0, 0x1000);
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
primou_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	PR_DISP_DEBUG("%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);
	/* HTC, Add 50 ms delay for stability of driver IC at high temperature */
	hr_msleep(50);
	if (panel_type == PANEL_ID_PRIMO_SONY) {
		client_data->remote_write(client_data, 0x00, 0x3600);
		client_data->remote_write(client_data, 0x24, 0x5300);
	} else {
		client_data->remote_write(client_data, 0x24, 0x5300);
	}
	primou_backlight_switch(LED_FULL);
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = primou_mddi_init,
	.uninit = primou_mddi_uninit,
	.blank = primou_panel_blank,
	.unblank = primou_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 52,
		.height = 88,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
		.vsync_gpio = 30,
	},
};

static void
mddi_power(struct msm_mddi_client_data *client_data, int on)
{
	int rc;
	unsigned config;
	PR_DISP_DEBUG("%s(%s)\n", __func__, on?"on":"off");

	if (on) {
		if (axi_clk)
			clk_set_rate(axi_clk, 192000000);

		config = PCOM_GPIO_CFG(PRIMOU_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(PRIMOU_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

		if (panel_type == PANEL_ID_PRIMO_SONY) {
			config = PCOM_GPIO_CFG(PRIMOU_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
			rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
			vreg_enable(V_LCMIO_1V8);
			hr_msleep(3);
			vreg_enable(V_LCMIO_2V8);
			hr_msleep(5);

			gpio_set_value(PRIMOU_LCD_RSTz, 1);
			hr_msleep(1);
			gpio_set_value(PRIMOU_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(PRIMOU_LCD_RSTz, 1);
			hr_msleep(15);
		} else {
			config = PCOM_GPIO_CFG(PRIMOU_LCD_ID1, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA);
			rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
			vreg_enable(V_LCMIO_1V8);
			hr_msleep(3);
			vreg_enable(V_LCMIO_2V8);
			hr_msleep(5);

			gpio_set_value(PRIMOU_LCD_RSTz, 1);
			hr_msleep(1);
			gpio_set_value(PRIMOU_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(PRIMOU_LCD_RSTz, 1);
			hr_msleep(20);
		}
	} else {
		if (panel_type == PANEL_ID_PRIMO_SONY) {
			hr_msleep(80);
			gpio_set_value(PRIMOU_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(V_LCMIO_1V8);
			vreg_disable(V_LCMIO_2V8);
		} else {
			hr_msleep(20);
			gpio_set_value(PRIMOU_LCD_RSTz, 0);
			vreg_disable(V_LCMIO_2V8);
			vreg_disable(V_LCMIO_1V8);
		}

		config = PCOM_GPIO_CFG(PRIMOU_MDDI_TE, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(PRIMOU_LCD_ID1, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(PRIMOU_LCD_ID0, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}
}

static void
panel_mddi_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	PR_DISP_DEBUG("mddi fixup\n");
	*mfr_name = 0xb9f6;
	*product_code = 0x5560;
}

static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = panel_mddi_fixup,
	.power_client = mddi_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5560),
			.name = "mddi_c_b9f6_5560",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver primou_backlight_driver = {
	.probe = primou_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.overrides = 0,
	.color_format = MSM_MDP_OUT_IF_FMT_RGB888,
#ifdef CONFIG_MDP4_HW_VSYNC
       .xres = 480,
       .yres = 800,
       .back_porch = 20,
       .front_porch = 20,
       .pulse_width = 40,
#endif
};

static struct msm_mdp_platform_data mdp_pdata_sony = {
	.overrides = MSM_MDP_PANEL_FLIP_UD | MSM_MDP_PANEL_FLIP_LR,
	.color_format = MSM_MDP_OUT_IF_FMT_RGB888,
#ifdef CONFIG_MDP4_HW_VSYNC
       .xres = 480,
       .yres = 800,
       .back_porch = 4,
       .front_porch = 2,
       .pulse_width = 4,
#endif
};

int __init primou_init_panel(void)
{
	int rc;

	PR_DISP_INFO("%s: enter.\n", __func__);

	/* lcd panel power */
	V_LCMIO_1V8 = vreg_get(NULL, "wlan2");

	if (IS_ERR(V_LCMIO_1V8)) {
		PR_DISP_ERR("%s: LCMIO_1V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}

	V_LCMIO_2V8 = vreg_get(NULL, "gp13");

	if (IS_ERR(V_LCMIO_2V8)) {
		PR_DISP_ERR("%s: LCMIO_2V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_2V8));
		return -1;
	}

	resources_msm_fb[0].start = msm_fb_base;
	resources_msm_fb[0].end = msm_fb_base + MSM_FB_SIZE - 1;

	if (panel_type == PANEL_ID_PRIMO_SONY)
		msm_device_mdp.dev.platform_data = &mdp_pdata_sony;
	else
		msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	mddi_pdata.clk_rate = 384000000;

	mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;

	axi_clk = clk_get(NULL, "ebi1_mddi_clk");
	if (IS_ERR(axi_clk)) {
		PR_DISP_ERR("%s: failed to get axi clock\n", __func__);
		return PTR_ERR(axi_clk);
	}

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	rc = platform_driver_register(&primou_backlight_driver);
	if (rc)
		return rc;

	set_bit(CABC_STATE, &cabc.status);

	return 0;
}
