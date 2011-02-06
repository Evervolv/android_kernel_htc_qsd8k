/* arch/arm/mach-msm/board-incrediblec-keypad.c
 *
 * Copyright (C) 2009 Google, Inc
 * Copyright (C) 2009 HTC Corporation.
 *
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/gpio_event.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/keyreset.h>
#include <mach/vreg.h>

#include <asm/mach-types.h>

#include "board-incrediblec.h"


const struct gpio_event_direct_entry incrediblec_keypad_nav_map_x0[] = {
	{
		.gpio = INCREDIBLEC_GPIO_POWER_KEY,
		.code = KEY_POWER
	},
	{
		.gpio = INCREDIBLEC_GPIO_VOLUME_UP,
		.code = KEY_VOLUMEUP
	},
	{
		.gpio = INCREDIBLEC_GPIO_VOLUME_DOWN,
		.code = KEY_VOLUMEDOWN
	},
};

const struct gpio_event_direct_entry incrediblec_keypad_nav_map_x1[] = {
	{
		.gpio = INCREDIBLEC_GPIO_POWER_KEY,
		.code = KEY_POWER
	},
	{
		.gpio = INCREDIBLEC_GPIO_VOLUME_UP,
		.code = KEY_VOLUMEUP
	},
	{
		.gpio = INCREDIBLEC_GPIO_VOLUME_DOWN,
		.code = KEY_VOLUMEDOWN
	},
	{
		.gpio = INCREDIBLEC_GPIO_OJ_ACTION_XB,
		.code = BTN_MOUSE
	},
};

static struct gpio_event_input_info incrediblec_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.info.oj_btn = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.keymap = incrediblec_keypad_nav_map_x1,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap_size = ARRAY_SIZE(incrediblec_keypad_nav_map_x1)
};

static struct gpio_event_info *incrediblec_keypad_info[] = {
	&incrediblec_keypad_nav_info.info,
};

static struct gpio_event_platform_data incrediblec_keypad_data = {
	.name = "incrediblec-keypad",
	.info = incrediblec_keypad_info,
	.info_count = ARRAY_SIZE(incrediblec_keypad_info)
};

static struct platform_device incrediblec_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &incrediblec_keypad_data,
	},
};

static int incrediblec_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data incrediblec_reset_keys_pdata = {
	.keys_up = incrediblec_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

static struct platform_device incrediblec_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &incrediblec_reset_keys_pdata,
};
static int __init incrediblec_init_keypad(void)
{
	int ret;

	if (!machine_is_incrediblec())
		return 0;

	if (system_rev < 2) {
		incrediblec_keypad_nav_info.keymap =
						incrediblec_keypad_nav_map_x0;
		incrediblec_keypad_nav_info.keymap_size =
				ARRAY_SIZE(incrediblec_keypad_nav_map_x0);
	}

	if (platform_device_register(&incrediblec_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	ret = platform_device_register(&incrediblec_keypad_device);
	if (ret != 0)
		return ret;

	return 0;
}

device_initcall(incrediblec_init_keypad);


