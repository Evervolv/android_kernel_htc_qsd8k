/* arch/arm/mach-msm/board-htcleo-microp.c
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
*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <asm/mach-types.h>
#include <mach/board-htcleo-microp.h>
#include <linux/capella_cm3602.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "board-htcleo.h"

static void htcleo_leds_led_brightness_set_work(struct work_struct *work);

enum led_type {
	GREEN_LED,
	AMBER_LED,
	NUM_LEDS,
};

struct htcleo_leds_led_data {
	int type;
	struct led_classdev ldev;
	struct mutex led_data_mutex;
	struct work_struct brightness_work;
	spinlock_t brightness_lock;
	enum led_brightness brightness;
	uint8_t mode;
	uint8_t blink;
};

static struct htcleo_leds_data {
	struct htcleo_leds_led_data leds[NUM_LEDS];
} the_data;


int htcleo_leds_disable_lights(void)
{
	int ret;
	uint8_t data[4];

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	ret = microp_i2c_write(MICROP_I2C_WCMD_LED_CTRL, data, 1);
	if (ret != 0)
	{
		pr_err("%s: set failed\n", __func__);
	}
	return 0;
}


static int htcleo_leds_write_led_mode(struct led_classdev *led_cdev,
				uint8_t mode)
{
/*	There are 5 different Led Modi;
*	0x0, 0x0: Disabled
*	0x0, 0x1: LED Green
*	0x0, 0x2: LED Amber 
*	0x0, 0x3: LED Green flashing slow ( ca. 6 sek ) 
*	0x0, 0x4: LED Green flashing fast ( ca. 2 sek )
*	0x0, 0x5: LED Amber flashing fast ( ca. 2 sek ) 
*	0x10,0xX: LED Amber and Green flashing alternately
*/
	struct htcleo_leds_led_data *ldata;
	uint8_t data[2] = { 0, 0 };
	int ret;
	static uint8_t oldvalgr=0, oldvalam=0, alternately=0;

	ldata = container_of(led_cdev, struct htcleo_leds_led_data, ldev);

	data[0] = 0x00;
	if (ldata->type == GREEN_LED) {
		switch(mode) {
		  case 0x0:
			if(alternately) {
				data[1]=oldvalam; 
				alternately=0;
			} else
				data[1] = 0x0;  // Disable Light
			break;
		  case 0x1:
			data[1] = 0x1;  // Enable Light
			break;
		  case 0x2:
			if(oldvalam==0x5) { // alternately blinking
				data[0] = 0x10;	
				alternately=1;
			} else
				alternately=0;
			data[1] = 0x3;  // Slow blinking
			break;
		  case 0x3:
			if(oldvalam==0x5) { // alternately blinking
				data[0] = 0x10;	
				alternately=1;
			} else
				alternately=0;
			data[1] = 0x4;  // Fast blinking
			break;
		}
		oldvalgr=data[1];
	} else if (ldata->type == AMBER_LED) {
		switch(mode) {
		  case 0x0:
			if(alternately) {
				data[1]=oldvalgr; 
				alternately=0;
			} else
				data[1] = 0x0;  // Disable Light
			break;
		  case 0x1:
			data[1] = 0x2;  // Enable Light
			break;
		  case 0x2:
		  case 0x3:
			if(oldvalgr==0x3 || oldvalgr==0x4) { // alternately blinking
				data[0] = 0x10;	
				alternately=1;
			} else
				alternately=0;
			data[1] = 0x5;  // Fast blinking
			break;
		}
		oldvalam=data[1];
	}

	ret = microp_i2c_write(MICROP_I2C_WCMD_LED_CTRL, data, 2);
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

static ssize_t htcleo_leds_led_blink_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev;
	struct htcleo_leds_led_data *ldata;
	int ret;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct htcleo_leds_led_data, ldev);

	mutex_lock(&ldata->led_data_mutex);
	ret = sprintf(buf, "%d\n", ldata->blink ? ldata->blink - 1 : 0);
	mutex_unlock(&ldata->led_data_mutex);

	return ret;
}

static ssize_t htcleo_leds_led_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct htcleo_leds_led_data *ldata;
	int val, ret;
	uint8_t mode;

	val = -1;
	sscanf(buf, "%u", &val);

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct htcleo_leds_led_data, ldev);

	mutex_lock(&ldata->led_data_mutex);
	switch (val) {
	case 0: /* stop flashing */
		mode = ldata->mode;
		ldata->blink = 0;
		break;
	case 1:
	case 2:
		mode = val + 1;
		break;

	default:
		mutex_unlock(&ldata->led_data_mutex);
		return -EINVAL;
	}
	mutex_unlock(&ldata->led_data_mutex);

	ret = htcleo_leds_write_led_mode(led_cdev, mode);
	if (ret)
		pr_err("%s set blink failed\n", led_cdev->name);

	return count;
}

static DEVICE_ATTR(blink, 0644, htcleo_leds_led_blink_show,
				htcleo_leds_led_blink_store);

				
static void htcleo_leds_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	unsigned long flags;
	struct i2c_client *client = to_i2c_client(led_cdev->dev->parent);
	struct htcleo_leds_led_data *ldata =
		container_of(led_cdev, struct htcleo_leds_led_data, ldev);

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

static void htcleo_leds_led_brightness_set_work(struct work_struct *work)
{
	unsigned long flags;
	struct htcleo_leds_led_data *ldata =
		container_of(work, struct htcleo_leds_led_data, brightness_work);
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

	ret = htcleo_leds_write_led_mode(led_cdev, mode);
	if (ret) {
		dev_err(&client->dev,
			 "led_brightness_set failed to set mode\n");
	}
}

struct device_attribute *green_amber_attrs[] = {
	&dev_attr_blink,
};

static struct {
	const char *name;
	void (*led_set_work)(struct work_struct *);
	struct device_attribute **attrs;
	int attr_cnt;
} htcleo_leds_leds[] = {
	[GREEN_LED] = {
		.name		= "green",
		.led_set_work   = htcleo_leds_led_brightness_set_work,
		.attrs		= green_amber_attrs,
		.attr_cnt	= ARRAY_SIZE(green_amber_attrs)
	},
	[AMBER_LED] = {
		.name		= "amber",
		.led_set_work   = htcleo_leds_led_brightness_set_work,
		.attrs		= green_amber_attrs,
		.attr_cnt	= ARRAY_SIZE(green_amber_attrs)
	},
};


static int htcleo_leds_probe(struct platform_device *pdev)
{
	int rc, i, j;
	struct htcleo_leds_data *cdata;
		
	rc= 0;

	pr_info("%s\n", __func__);

	cdata = &the_data;
	platform_set_drvdata(pdev, cdata);

	htcleo_leds_disable_lights();
	
	for (i = 0; i < ARRAY_SIZE(htcleo_leds_leds) && !rc; ++i) {
		struct htcleo_leds_led_data *ldata = &cdata->leds[i];

		ldata->type = i;
		ldata->ldev.name = htcleo_leds_leds[i].name;
		ldata->ldev.brightness_set = htcleo_leds_brightness_set;
		mutex_init(&ldata->led_data_mutex);
		INIT_WORK(&ldata->brightness_work, htcleo_leds_leds[i].led_set_work);
		spin_lock_init(&ldata->brightness_lock);
		rc = led_classdev_register(&pdev->dev, &ldata->ldev);
		if (rc) {
			ldata->ldev.name = NULL;
			break;
		}

		for (j = 0; j < htcleo_leds_leds[i].attr_cnt && !rc; ++j)
			rc = device_create_file(ldata->ldev.dev,
						 htcleo_leds_leds[i].attrs[j]);
	}
	if (rc) {
		dev_err(&pdev->dev, "failed to add leds\n");
		goto err_add_leds;
	}
	

	goto done;


err_add_leds:
	for (i = 0; i < ARRAY_SIZE(htcleo_leds_leds); ++i) {
		if (!cdata->leds[i].ldev.name)
			continue;
		led_classdev_unregister(&cdata->leds[i].ldev);
		for (j = 0; j < htcleo_leds_leds[i].attr_cnt; ++j)
			device_remove_file(cdata->leds[i].ldev.dev,
					   htcleo_leds_leds[i].attrs[j]);
	}

done:
	return rc;
}

static struct platform_driver htcleo_leds_driver = {
	.probe = htcleo_leds_probe,
	.driver = {
		.name = "htcleo-leds",
		.owner = THIS_MODULE
	},
};

static int __init htcleo_leds_init(void)
{
	return platform_driver_register(&htcleo_leds_driver);
}

device_initcall(htcleo_leds_init);

MODULE_DESCRIPTION("HTC LEO LED Support");
MODULE_LICENSE("GPL");
