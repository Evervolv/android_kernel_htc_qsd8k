/* arch/arm/mach-msm/board-bravo-keypad.c
 *
 * Copyright (C) 2009 Google, Inc
 * Copyright (C) 2009 HTC Corporation.
 * Copyright (C) 2010 Giulio Cervera <giulio.cervera@gmail.com>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
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
#include <linux/keyreset.h>
#include <linux/platform_device.h>
#include <mach/vreg.h>

#include <asm/mach-types.h>

#include "board-bravo.h"

static unsigned int bravo_col_gpios[] = {
	BRAVO_GPIO_KP_MKOUT0,
	BRAVO_GPIO_KP_MKOUT1,
	BRAVO_GPIO_KP_MKOUT2,
};

static unsigned int bravo_row_gpios[] = {
	BRAVO_GPIO_KP_MPIN0,
	BRAVO_GPIO_KP_MPIN1,
	BRAVO_GPIO_KP_MPIN2,
};

#define KEYMAP_INDEX(col, row)	((col)*ARRAY_SIZE(bravo_row_gpios) + (row))
#define KEYMAP_SIZE		(ARRAY_SIZE(bravo_col_gpios) * \
				 ARRAY_SIZE(bravo_row_gpios))

/* keypad */
static const unsigned short bravo_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 1)] = BTN_MOUSE, /* OJ Action key */
	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 2)] = KEY_SEARCH,
	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 2)] = KEY_BACK,
};

static struct gpio_event_matrix_info bravo_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = bravo_keymap,
	.output_gpios = bravo_col_gpios,
	.input_gpios = bravo_row_gpios,
	.noutputs = ARRAY_SIZE(bravo_col_gpios),
	.ninputs = ARRAY_SIZE(bravo_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS),
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	.info.oj_btn = true,
#endif
};

static struct gpio_event_direct_entry bravo_keypad_key_map[] = {
	{
		.gpio	= BRAVO_GPIO_POWER_KEY,
		.code	= KEY_POWER,
	},
};

static struct gpio_event_input_info bravo_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = bravo_keypad_key_map,
	.keymap_size = ARRAY_SIZE(bravo_keypad_key_map)
};

static struct gpio_event_info *bravo_input_info[] = {
	&bravo_keypad_matrix_info.info,
	&bravo_keypad_key_info.info,
};

static struct gpio_event_platform_data bravo_input_data = {
	.names = {
#ifdef CONFIG_MACH_BRAVO
		"bravo-keypad",
#else
		"bravoc-keypad",
#endif
		NULL,
	},
	.info = bravo_input_info,
	.info_count = ARRAY_SIZE(bravo_input_info),
};

static struct platform_device bravo_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &bravo_input_data,
	},
};

static int bravo_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0,
};

static struct keyreset_platform_data bravo_reset_keys_pdata = {
	.keys_up	= bravo_reset_keys_up,
	.keys_down	= {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device bravo_reset_keys_device = {
	.name	= KEYRESET_NAME,
	.dev	= {
		.platform_data = &bravo_reset_keys_pdata,
	},
};

static int __init bravo_init_keypad(void)
{
	int ret;

	if (!machine_is_bravo() && !machine_is_bravoc())
		return 0;

	ret = platform_device_register(&bravo_reset_keys_device);
	if (ret != 0)
		return ret;

	ret = platform_device_register(&bravo_input_device);
	if (ret != 0)
		return ret;

	return 0;
}

device_initcall(bravo_init_keypad);
