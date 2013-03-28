/* arch/arm/mach-msm/board-htcleo-keypad.c
 *
 * Author: Markinus
 * Author: Parad0X
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
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include "board-htcleo.h"

#define HTCLEO_DEFAULT_KEYPAD_BRIGHTNESS 0
static DEFINE_MUTEX(htcleo_keypad_brightness_lock);

struct led_data {
	struct mutex led_data_mutex;
	struct work_struct brightness_work;
	spinlock_t brightness_lock;
	enum led_brightness brightness;
	uint8_t oldval;
} keypad_led_data;


static unsigned int htcleo_col_gpios[] = {
	HTCLEO_GPIO_KP_MKOUT0,
	HTCLEO_GPIO_KP_MKOUT1,
	HTCLEO_GPIO_KP_MKOUT2
};

static unsigned int htcleo_row_gpios[] = {
	HTCLEO_GPIO_KP_MPIN0,
	HTCLEO_GPIO_KP_MPIN1,
	HTCLEO_GPIO_KP_MPIN2
};

#define KEYMAP_INDEX(col, row)	((col)*ARRAY_SIZE(htcleo_row_gpios) + (row))
#define KEYMAP_SIZE		(ARRAY_SIZE(htcleo_col_gpios) * \
				 ARRAY_SIZE(htcleo_row_gpios))

/* keypad */
static const unsigned short htcleo_keymap[KEYMAP_SIZE] = {
#if defined(CONFIG_HTCLEO_KEYMAP_DPAD)
	[KEYMAP_INDEX(0, 0)] = KEY_LEFT,	// Volume Up
	[KEYMAP_INDEX(0, 1)] = KEY_RIGHT,	// Volume Down
	[KEYMAP_INDEX(1, 0)] = KEY_DOWN,	// Windows Button
	[KEYMAP_INDEX(1, 1)] = KEY_ENTER,	// Dial Button
	[KEYMAP_INDEX(1, 2)] = KEY_END,  	// Hangup Button
	[KEYMAP_INDEX(2, 0)] = KEY_UP,		// Back Button
	[KEYMAP_INDEX(2, 1)] = KEY_LEFTALT,	// Home Button
#endif
#if defined(CONFIG_HTCLEO_KEYMAP_ANDROID)
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,	// Volume Up
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,	// Volume Down
	[KEYMAP_INDEX(1, 0)] = KEY_MENU,	// Windows Button
	[KEYMAP_INDEX(1, 1)] = KEY_SEND,	// Dial Button
	[KEYMAP_INDEX(1, 2)] = KEY_END,		// Hangup Button
	[KEYMAP_INDEX(2, 0)] = KEY_BACK,	// Back Button
	[KEYMAP_INDEX(2, 1)] = KEY_HOME,	// Home Button
#endif
};

static struct gpio_event_matrix_info htcleo_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = htcleo_keymap,
	.output_gpios = htcleo_col_gpios,
	.input_gpios = htcleo_row_gpios,
	.noutputs = ARRAY_SIZE(htcleo_col_gpios),
	.ninputs = ARRAY_SIZE(htcleo_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS),
};

static struct gpio_event_direct_entry htcleo_keypad_key_map[] = {
	{
		.gpio	= HTCLEO_GPIO_POWER_KEY,
		.code	= KEY_END,		// Power key
	},
};

static struct gpio_event_input_info htcleo_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = 0,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = htcleo_keypad_key_map,
	.keymap_size = ARRAY_SIZE(htcleo_keypad_key_map)
};

static struct gpio_event_info *htcleo_input_info[] = {
	&htcleo_keypad_matrix_info.info,
	&htcleo_keypad_key_info.info,
};

static struct gpio_event_platform_data htcleo_input_data = {
	.names = {
		"htcleo-keypad",
		NULL,
	},
	.info = htcleo_input_info,
	.info_count = ARRAY_SIZE(htcleo_input_info),
};

static struct platform_device htcleo_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &htcleo_input_data,
	},
};

static struct keyreset_platform_data htcleo_reset_keys_pdata = {
	.keys_down = {
		KEY_END,
		KEY_VOLUMEUP,
		KEY_VOLUMEDOWN,
		0
	},
};

static struct platform_device htcleo_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &htcleo_reset_keys_pdata,
};

static void keypad_led_brightness_set_work(struct work_struct *work)
{

	unsigned long flags;

	enum led_brightness brightness;
	uint8_t value;

	spin_lock_irqsave(&keypad_led_data.brightness_lock, flags);
	brightness = keypad_led_data.brightness;
	spin_unlock_irqrestore(&keypad_led_data.brightness_lock, flags);

	value = brightness >= 1 ? 1 : 0;

	/* avoid a flicker that can occur when writing the same value */
	if (keypad_led_data.oldval == value)
		return;
	keypad_led_data.oldval = value;

	gpio_set_value(HTCLEO_GPIO_KP_LED, value);

}

static void keypad_led_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	unsigned long flags;
	mutex_lock(&htcleo_keypad_brightness_lock);

	pr_debug("Setting %s brightness current %d new %d\n",
			led_cdev->name, led_cdev->brightness, brightness);

	if (brightness > 255)
		brightness = 255;
	led_cdev->brightness = brightness;

	spin_lock_irqsave(&keypad_led_data.brightness_lock, flags);
	keypad_led_data.brightness = brightness;
	spin_unlock_irqrestore(&keypad_led_data.brightness_lock, flags);

	schedule_work(&keypad_led_data.brightness_work);
	mutex_unlock(&htcleo_keypad_brightness_lock);
}

static enum led_brightness keypad_led_brightness_get(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static struct led_classdev htcleo_backlight_led = 
{
	.name = "button-backlight",
	.brightness = HTCLEO_DEFAULT_KEYPAD_BRIGHTNESS,
	.brightness_set = keypad_led_brightness_set,
	.brightness_get = keypad_led_brightness_get,
};

static int __init htcleo_init_keypad(void)
{
	int ret;

	if (!machine_is_htcleo())
		return 0;

	ret = platform_device_register(&htcleo_reset_keys_device);
	if (ret != 0) {
		pr_err("%s: register reset key fail\n", __func__);
		goto exit;
	}

	ret = platform_device_register(&htcleo_input_device);
	if (ret != 0)
		goto exit;

	ret = gpio_request(HTCLEO_GPIO_KP_LED, "keypad_led");
	if (ret < 0) {
		pr_err("failed on request gpio keypad backlight on\n");
		goto exit;
	}

	ret = gpio_direction_output(HTCLEO_GPIO_KP_LED, 0);
	if (ret < 0) {
		pr_err("failed on gpio_direction_output keypad backlight on\n");
		goto err_gpio_kpl;
	}

	keypad_led_data.oldval = 0;
	mutex_init(&keypad_led_data.led_data_mutex);
	INIT_WORK(&keypad_led_data.brightness_work, keypad_led_brightness_set_work);
	spin_lock_init(&keypad_led_data.brightness_lock);
	ret = led_classdev_register(&htcleo_input_device.dev, &htcleo_backlight_led);
	if (ret) {
		goto exit;
	}

	return 0;
err_gpio_kpl:
	gpio_free(HTCLEO_GPIO_KP_LED);
exit:
	return ret;

}

device_initcall(htcleo_init_keypad);
