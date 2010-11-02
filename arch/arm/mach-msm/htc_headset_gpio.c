/*
 *
 * /arch/arm/mach-msm/htc_headset_gpio.c
 *
 *  HTC GPIO headset detection driver.
 *
 *  Copyright (C) 2010 HTC, Inc.
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
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>

#define DRIVER_NAME "HS_GPIO"

/* #define DEBUG */

#ifdef DEBUG
#define AJ_DBG(fmt, arg...) \
	printk(KERN_INFO "[Audio Jack] %s " fmt "\r\n", __func__, ## arg)
#else
#define AJ_DBG(fmt, arg...) do {} while (0)
#endif

struct audio_jack_info {
	unsigned int irq_jack;
	int audio_jack_detect;
	int key_enable_gpio;
	int mic_select_gpio;
	int audio_jack_flag;

	struct hrtimer detection_timer;
	ktime_t debounce_time;

	struct work_struct work;

	spinlock_t spin_lock;

	struct wake_lock audiojack_wake_lock;
};

static struct audio_jack_info *pjack_info;

void hs_gpio_key_enable(int enable)
{
	DBG_MSG();

	if (pjack_info->key_enable_gpio)
		gpio_set_value(pjack_info->key_enable_gpio, enable);
}

void hs_gpio_mic_select(int enable)
{
	DBG_MSG();

	if (pjack_info->mic_select_gpio)
		gpio_set_value(pjack_info->mic_select_gpio, enable);
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	hs_notify_hpin_irq();

	AJ_DBG("");

	do {
		value1 = gpio_get_value(pjack_info->audio_jack_detect);
		set_irq_type(pjack_info->irq_jack, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(pjack_info->audio_jack_detect);
	} while (value1 != value2 && retry_limit-- > 0);

	AJ_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	if ((pjack_info->audio_jack_flag == 0) ^ value2) {
		wake_lock_timeout(&pjack_info->audiojack_wake_lock, 4*HZ);

		/* Do the rest of the work in timer context */
		hrtimer_start(&pjack_info->detection_timer,
		pjack_info->debounce_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart detect_35mm_event_timer_func(struct hrtimer *data)
{
	int state;

	AJ_DBG("");
	state = !gpio_get_value(pjack_info->audio_jack_detect);
	if (pjack_info->audio_jack_flag != state) {
		pjack_info->audio_jack_flag = state;
		schedule_work(&pjack_info->work);
	}

	return HRTIMER_NORESTART;
}

static void audiojack_work_func(struct work_struct *work)
{
	int is_insert;
	unsigned long flags = 0;

	spin_lock_irqsave(&pjack_info->spin_lock, flags);
	is_insert = pjack_info->audio_jack_flag;
	spin_unlock_irqrestore(&pjack_info->spin_lock, flags);

	htc_35mm_remote_notify_insert_ext_headset(is_insert);

	if (is_insert)
		pjack_info->debounce_time = ktime_set(0, 200000000);
	else
		pjack_info->debounce_time = ktime_set(0, 500000000);
}

static void hs_gpio_register(void)
{
	struct headset_notifier notifier;

	if (pjack_info->mic_select_gpio) {
		notifier.id = HEADSET_REG_MIC_SELECT;
		notifier.func = hs_gpio_mic_select;
		headset_notifier_register(&notifier);
	}

	if (pjack_info->key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_gpio_key_enable;
		headset_notifier_register(&notifier);
	}
}

static int audiojack_probe(struct platform_device *pdev)
{
	int ret;
	struct htc_headset_gpio_platform_data *pdata = pdev->dev.platform_data;

	SYS_MSG("++++++++++++++++++++");

	pjack_info = kzalloc(sizeof(struct audio_jack_info), GFP_KERNEL);
	if (!pjack_info)
		return -ENOMEM;

	pjack_info->audio_jack_detect = pdata->hpin_gpio;
	pjack_info->key_enable_gpio = pdata->key_enable_gpio;
	pjack_info->mic_select_gpio = pdata->mic_select_gpio;
	pjack_info->audio_jack_flag = 0;

	pjack_info->debounce_time = ktime_set(0, 500000000);
	hrtimer_init(&pjack_info->detection_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pjack_info->detection_timer.function = detect_35mm_event_timer_func;

	INIT_WORK(&pjack_info->work, audiojack_work_func);

	spin_lock_init(&pjack_info->spin_lock);
	wake_lock_init(&pjack_info->audiojack_wake_lock,
		WAKE_LOCK_SUSPEND, "audiojack");

	if (pjack_info->audio_jack_detect) {
		ret = gpio_request(pjack_info->audio_jack_detect,
				   "3.5mm_detect");
		if (ret < 0)
			goto err_request_detect_gpio;

		ret = gpio_direction_input(pjack_info->audio_jack_detect);
		if (ret < 0)
			goto err_set_detect_gpio;

		pjack_info->irq_jack =
			gpio_to_irq(pjack_info->audio_jack_detect);
		if (pjack_info->irq_jack < 0) {
			ret = pjack_info->irq_jack;
			goto err_request_detect_irq;
		}

		ret = request_irq(pjack_info->irq_jack,
				  detect_irq_handler,
				  IRQF_TRIGGER_LOW, "35mm_headset", NULL);
		if (ret < 0)
			goto err_request_detect_irq;

		ret = set_irq_wake(pjack_info->irq_jack, 1);
		if (ret < 0)
			goto err_set_irq_wake;
	}

	hs_gpio_register();

	SYS_MSG("--------------------");

	return 0;

err_set_irq_wake:
	if (pjack_info->audio_jack_detect)
		free_irq(pjack_info->irq_jack, 0);
err_request_detect_irq:
err_set_detect_gpio:
	if (pjack_info->audio_jack_detect)
		gpio_free(pjack_info->audio_jack_detect);
err_request_detect_gpio:
	printk(KERN_ERR "Audiojack: Failed in audiojack_probe\n");

	return ret;
}

static int audiojack_remove(struct platform_device *pdev)
{
	if (pjack_info->audio_jack_detect)
		free_irq(pjack_info->irq_jack, 0);

	if (pjack_info->audio_jack_detect)
		gpio_free(pjack_info->audio_jack_detect);

	return 0;
}

static struct platform_driver audiojack_driver = {
	.probe		= audiojack_probe,
	.remove		= audiojack_remove,
	.driver		= {
		.name		= "HTC_HEADSET_GPIO",
		.owner		= THIS_MODULE,
	},
};

static int __init audiojack_init(void)
{
	return platform_driver_register(&audiojack_driver);
}

static void __exit audiojack_exit(void)
{
	platform_driver_unregister(&audiojack_driver);
}

module_init(audiojack_init);
module_exit(audiojack_exit);

MODULE_DESCRIPTION("HTC GPIO headset detection driver");
MODULE_LICENSE("GPL");
