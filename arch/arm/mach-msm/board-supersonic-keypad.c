/* arch/arm/mach-msm/board-supersonic-keypad.c
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
#include <mach/vreg.h>
#include <linux/keyreset.h>

#include <asm/mach-types.h>

#include "board-supersonic.h"


static struct gpio_event_direct_entry supersonic_keypad_nav_map[] = {
	{ SUPERSONIC_POWER_KEY,              KEY_POWER      },
	{ SUPERSONIC_VOLUME_UP,              KEY_VOLUMEUP   },
	{ SUPERSONIC_VOLUME_DOWN,            KEY_VOLUMEDOWN },
};

static struct gpio_event_input_info supersonic_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.keymap = supersonic_keypad_nav_map,
	.debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.keymap_size = ARRAY_SIZE(supersonic_keypad_nav_map)
};

static struct gpio_event_info *supersonic_keypad_info[] = {
	&supersonic_keypad_nav_info.info,
};

static struct gpio_event_platform_data supersonic_keypad_data = {
	.name = "supersonic-keypad",
	.info = supersonic_keypad_info,
	.info_count = ARRAY_SIZE(supersonic_keypad_info)
};

static struct platform_device supersonic_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &supersonic_keypad_data,
	},
};

static struct keyreset_platform_data supersonic_reset_keys_pdata = {
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEUP,
		KEY_VOLUMEDOWN,
		0
	},
};

static struct platform_device supersonic_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &supersonic_reset_keys_pdata,
};

static int __init supersonic_init_keypad(void)
{
	int ret;

	if (!machine_is_supersonic())
		return 0;

	if (platform_device_register(&supersonic_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	ret = platform_device_register(&supersonic_keypad_device);
	if (ret != 0)
		return ret;

	return 0;
}

device_initcall(supersonic_init_keypad);


