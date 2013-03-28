/* linux/arch/arm/mach-msm/board-htcleo-bkl.c
 *
 * Copyright (c) 2010 Cotulla
 * Edited to Common Structure by Markinus
 * Added support for Button backlight manager by Danijel Posilović (dan1j3l)
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/atmega_microp.h>

//#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...) printk(fmt, ## arg)
#else
#define LCMDBG(fmt, arg...) {}
#endif

#define HTCLEO_DEFAULT_BACKLIGHT_BRIGHTNESS 255


static struct led_trigger *htcleo_lcd_backlight;
static int auto_bl_state=0;
static DEFINE_MUTEX(htcleo_backlight_lock);

#ifdef CONFIG_HTCLEO_BTN_BACKLIGHT_MANAGER
static int BUTTON_BACKLIGHT_GPIO = 48;
#endif

static int htcleo_brightness_autobacklight(uint8_t value)
{
	int ret;
	uint8_t data[2];

	LCMDBG("%s:(%d)\n", __func__, value);
	if(value!=0 && value!=1) return -1;

	data[0] = 1;
	data[1] = value;
	ret = microp_i2c_write(MICROP_I2C_WCMD_AUTO_BL_CTL, data, 2);
	if (ret != 0) {
		pr_err("%s: set auto light sensor fail\n", __func__);
		return ret;
	}
	auto_bl_state=value;
	return 0;
}
EXPORT_SYMBOL(htcleo_brightness_autobacklight);

static ssize_t htcleo_auto_bl_get(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d", auto_bl_state);
	return ret;
}

static ssize_t htcleo_auto_bl_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int set_state;
	mutex_lock(&htcleo_backlight_lock);
	sscanf(buf, "%d", &set_state);
	if(set_state!=0 && set_state!=1) return -EINVAL;
	htcleo_brightness_autobacklight(set_state);
	mutex_unlock(&htcleo_backlight_lock);
	return count;
}

static DEVICE_ATTR(auto_bl, 0666,  htcleo_auto_bl_get, htcleo_auto_bl_set);

static int htcleo_brightness_onoff_bkl(int enable)
{
	int ret;
	uint8_t data[1];

#ifdef CONFIG_HTCLEO_BTN_BACKLIGHT_MANAGER
    // Disable button backlight along with screen
    if (!enable)
        gpio_set_value(BUTTON_BACKLIGHT_GPIO, 0);
#endif

	data[0] = enable ? 1 : 0;
	ret = microp_i2c_write(MICROP_I2C_WCMD_BL_EN, data, 1);
	if (ret != 0)
		pr_err("%s: set failed\n", __func__);
	return 0;
}

static int htcleo_brightness_set_bkl(uint8_t value)
{
	int ret;
	uint8_t cmd[2];

	LCMDBG("%s:(%d)\n", __func__, value);

	if (value > 9)
	{
		value = 9;
	}

	// setvalue
	cmd[0] = value << 4;
	ret = microp_i2c_write(MICROP_I2C_WCMD_LCM_BL_MANU_CTL, cmd, 1); // 22
	if (ret < 0)
	{
		pr_err("%s: request adc fail\n", __func__);
		return -EIO;
	}

	return 0;
}

static void htcleo_brightness_set(struct led_classdev *led_cdev, enum led_brightness val)
{
	mutex_lock(&htcleo_backlight_lock);

	// set brigtness level via MicroP
	LCMDBG("htcleo_brightness_set: %d\n", val);
	if (val > 255) val = 255;
	led_cdev->brightness = val;
	if (val < 1)
	{
		htcleo_brightness_onoff_bkl(0);
	}
	else
	{
		htcleo_brightness_onoff_bkl(1);
		htcleo_brightness_set_bkl((val - 1) / 23);
	}
	mutex_unlock(&htcleo_backlight_lock);
}

static enum led_brightness htcleo_brightness_get(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static struct led_classdev htcleo_backlight_led =
{
	.name = "lcd-backlight",
	.brightness = HTCLEO_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = htcleo_brightness_set,
	.brightness_get = htcleo_brightness_get,
};

static int  htcleo_backlight_probe(struct platform_device *pdev)
{
	int rc;
	rc = device_create_file(&pdev->dev, &dev_attr_auto_bl);
	printk(KERN_INFO "%s: HTCLeo Backlight connect with microP: "
			"Probe\n", __func__);

	led_trigger_register_simple("lcd-backlight-gate", &htcleo_lcd_backlight);
	rc = led_classdev_register(&pdev->dev, &htcleo_backlight_led);
	if (rc)
	      LCMDBG("HTCLeo Backlight: failure on register led_classdev\n");
	htcleo_brightness_autobacklight(0);
	return rc;

}

static int htcleo_backlight_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_auto_bl);
	return 0;
}

static struct platform_driver htcleo_backlight_driver = {
	.probe		= htcleo_backlight_probe,
	.remove		= htcleo_backlight_remove,
	.driver		= {
		.name   = "htcleo-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init htcleo_backlight_init(void)
{
	return platform_driver_register(&htcleo_backlight_driver);

}

static void __exit htcleo_backlight_exit(void)
{
	platform_driver_unregister(&htcleo_backlight_driver);
}

module_init(htcleo_backlight_init);
module_exit(htcleo_backlight_exit);

MODULE_DESCRIPTION("BMA150 G-sensor driver");
MODULE_LICENSE("GPL");
