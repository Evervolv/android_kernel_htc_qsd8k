/* drivers/i2c/chips/tps61310_flashlight.c
 *
 * Copyright (C) 2008-2009 HTC Corporation.
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

#include <mach/msm_iomap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/htc_flashlight.h>

#define FLT_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[FLT]TPS " fmt, ##__VA_ARGS__)
#define FLT_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[FLT]TPS " fmt, ##__VA_ARGS__)
#define FLT_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[FLT][ERR]TPS " fmt, ##__VA_ARGS__)

#define TPS61310_RETRY_COUNT 10

struct tps61310_data {
	struct led_classdev 		fl_lcdev;
	struct early_suspend		fl_early_suspend;
	enum flashlight_mode_flags 	mode_status;
	uint32_t			flash_sw_timeout;
	struct mutex 			tps61310_data_mutex;
	uint32_t			strb0;
	uint32_t			strb1;
	uint8_t 			led_count;
};

static struct i2c_client *this_client;
static struct tps61310_data *this_tps61310;
struct delayed_work tps61310_delayed_work;
static struct workqueue_struct *tps61310_work_queue;
static struct mutex tps61310_mutex;

static int TPS61310_I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < TPS61310_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= TPS61310_RETRY_COUNT) {
		FLT_ERR_LOG("%s retry over %d\n", __func__,
							TPS61310_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int tps61310_i2c_command(uint8_t address, uint8_t data)
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = address;
	buffer[1] = data;
	ret = TPS61310_I2C_TxData(buffer, 2);
	if (ret < 0) {
		FLT_ERR_LOG("%s error\n", __func__);
		return ret;
	}
	return 0;
}

static int flashlight_turn_off(void)
{
	FLT_INFO_LOG("%s\n", __func__);
	gpio_set_value(this_tps61310->strb0, 0);
	gpio_set_value(this_tps61310->strb1, 1);
	tps61310_i2c_command(0x01, 0x00);
	this_tps61310->mode_status = FL_MODE_OFF;
	return 0;
}

int tps61310_flashlight_control(int mode)
{
	int ret = 0;

	mutex_lock(&tps61310_mutex);
	if (this_tps61310->led_count == 1) {
	switch (mode) {
	case FL_MODE_OFF:
		flashlight_turn_off();
	break;
	case FL_MODE_FLASH:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x01, 0x9E);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL1:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
#if defined(CONFIG_MACH_PRIMOU)
		tps61310_i2c_command(0x01, 0x86);
#else
		tps61310_i2c_command(0x01, 0x88);
#endif
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL2:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
#if defined(CONFIG_MACH_PRIMOU)
		tps61310_i2c_command(0x01, 0x88);
#else
		tps61310_i2c_command(0x01, 0x90);
#endif
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL3:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x01, 0x8C);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL4:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x01, 0x90);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL5:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x01, 0x94);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL6:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x01, 0x98);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL7:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x01, 0x9C);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_PRE_FLASH:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x04);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x05);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH_LEVEL_1:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x01);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH_LEVEL_2:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x03);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",
							__func__, mode);
		ret = -EINVAL;
	break;
	}
		} else if (this_tps61310->led_count == 2) {
	switch (mode) {
	case FL_MODE_OFF:
		flashlight_turn_off();
	break;
	case FL_MODE_FLASH:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x02, 0x90);
		tps61310_i2c_command(0x01, 0x90);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
				   msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL1:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
#if defined(CONFIG_MACH_PRIMOU)
		tps61310_i2c_command(0x02, 0x83);
		tps61310_i2c_command(0x01, 0x83);
#else
		tps61310_i2c_command(0x02, 0x84);
		tps61310_i2c_command(0x01, 0x84);
#endif
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL2:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
#if defined(CONFIG_MACH_PRIMOU)
		tps61310_i2c_command(0x02, 0x84);
		tps61310_i2c_command(0x01, 0x84);
#else
		tps61310_i2c_command(0x02, 0x88);
		tps61310_i2c_command(0x01, 0x88);
#endif
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL3:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x02, 0x86);
		tps61310_i2c_command(0x01, 0x86);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL4:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x02, 0x88);
		tps61310_i2c_command(0x01, 0x88);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL5:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x02, 0x8A);
		tps61310_i2c_command(0x01, 0x8A);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL6:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x02, 0x8C);
		tps61310_i2c_command(0x01, 0x8C);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_FLASH_LEVEL7:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x00);
		tps61310_i2c_command(0x02, 0x8E);
		tps61310_i2c_command(0x01, 0x8E);
		gpio_set_value(this_tps61310->strb1, 0);
		gpio_set_value(this_tps61310->strb0, 1);
		queue_delayed_work(tps61310_work_queue, &tps61310_delayed_work,
			msecs_to_jiffies(this_tps61310->flash_sw_timeout));
	break;
	case FL_MODE_PRE_FLASH:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x12);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x1B);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH_LEVEL_1:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x09);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH_LEVEL_2:
		tps61310_i2c_command(0x05, 0x6B);
		tps61310_i2c_command(0x00, 0x12);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH_LED_A:
		tps61310_i2c_command(0x05, 0x69);
		tps61310_i2c_command(0x00, 0x09);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;
	case FL_MODE_TORCH_LED_B:
		tps61310_i2c_command(0x05, 0x6A);
		tps61310_i2c_command(0x00, 0x09);
		gpio_set_value(this_tps61310->strb0, 0);
		gpio_set_value(this_tps61310->strb1, 1);
		tps61310_i2c_command(0x01, 0x40);
	break;

	default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",
							__func__, mode);
		ret = -EINVAL;
	break;
	}
		}
	FLT_INFO_LOG("%s: mode: %d\n", __func__, mode);
	this_tps61310->mode_status = mode;
	mutex_unlock(&tps61310_mutex);

	return ret;
}

static void fl_lcdev_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	enum flashlight_mode_flags mode;
	int ret = -1;

	if (brightness > 0 && brightness <= LED_HALF) {
		if (brightness == (LED_HALF - 2))
			mode = FL_MODE_TORCH_LEVEL_1;
		else if (brightness == (LED_HALF - 1))
			mode = FL_MODE_TORCH_LEVEL_2;
		else if (brightness == 1 && this_tps61310->led_count ==2)
			mode = FL_MODE_TORCH_LED_A;
		else if (brightness == 2 && this_tps61310->led_count ==2)
			mode = FL_MODE_TORCH_LED_B;
		else
			mode = FL_MODE_TORCH;
	} else if (brightness > LED_HALF && brightness <= LED_FULL) {
		if (brightness == (LED_HALF + 1))
			mode = FL_MODE_PRE_FLASH; /* pre-flash mode */
		else
			mode = FL_MODE_FLASH; /* Flashlight mode */
	} else
		/* off and else */
		mode = FL_MODE_OFF;

	ret = tps61310_flashlight_control(mode);
	if (ret) {
		FLT_ERR_LOG("%s: control failure rc:%d\n", __func__, ret);
		return;
	}
}

static void flashlight_early_suspend(struct early_suspend *handler)
{
	FLT_INFO_LOG("%s\n", __func__);
	if (this_tps61310 != NULL && this_tps61310->mode_status)
		flashlight_turn_off();
}

static void flashlight_late_resume(struct early_suspend *handler)
{
}

static void flashlight_turn_off_work(struct work_struct *work)
{
	FLT_INFO_LOG("%s\n", __func__);
	flashlight_turn_off();
}

static int tps61310_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct tps61310_data *tps61310;
	struct TPS61310_flashlight_platform_data *pdata;
	int err = 0;

	FLT_INFO_LOG("%s +\n", __func__);
	pdata = client->dev.platform_data;
	if (!pdata) {
		FLT_ERR_LOG("%s: Assign platform_data error!!\n", __func__);
		return -EINVAL;
	}

	if (pdata->gpio_init)
		pdata->gpio_init();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto check_functionality_failed;
	}

	tps61310 = kzalloc(sizeof(struct tps61310_data), GFP_KERNEL);
	if (!tps61310) {
		FLT_ERR_LOG("%s: kzalloc fail !!!\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, tps61310);
	this_client = client;

	INIT_DELAYED_WORK(&tps61310_delayed_work, flashlight_turn_off_work);
	tps61310_work_queue = create_singlethread_workqueue("tps61310_wq");
	if (!tps61310_work_queue)
		goto err_create_tps61310_work_queue;

	/* Register led class device */
	tps61310->fl_lcdev.name           = FLASHLIGHT_NAME;
	tps61310->fl_lcdev.brightness_set = fl_lcdev_brightness_set;
	tps61310->strb0                   = pdata->tps61310_strb0;
	tps61310->strb1                   = pdata->tps61310_strb1;
	tps61310->flash_sw_timeout	  = pdata->flash_duration_ms;
	tps61310->led_count = (pdata->led_count) ? pdata->led_count : 1;

	if (tps61310->flash_sw_timeout <= 0)
		tps61310->flash_sw_timeout = 600;

	mutex_init(&tps61310_mutex);

	err = led_classdev_register(&client->dev, &tps61310->fl_lcdev);
	if (err < 0) {
		FLT_ERR_LOG("%s: failed on led_classdev_register\n", __func__);
		goto platform_data_null;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	tps61310->fl_early_suspend.suspend = flashlight_early_suspend;
	tps61310->fl_early_suspend.resume  = flashlight_late_resume;
	register_early_suspend(&tps61310->fl_early_suspend);
#endif
	this_tps61310 = tps61310;

	/* voltage drop monitor*/
	tps61310_i2c_command(0x07, 0xF6);

	FLT_INFO_LOG("%s -\n", __func__);
	return 0;


platform_data_null:
	destroy_workqueue(tps61310_work_queue);
	mutex_destroy(&tps61310_mutex);
err_create_tps61310_work_queue:
	kfree(tps61310);
check_functionality_failed:
	return err;
}

static int tps61310_remove(struct i2c_client *client)
{
	struct tps61310_data *tps61310 = i2c_get_clientdata(client);

	led_classdev_unregister(&tps61310->fl_lcdev);
	destroy_workqueue(tps61310_work_queue);
	mutex_destroy(&tps61310_mutex);
	unregister_early_suspend(&tps61310->fl_early_suspend);
	kfree(tps61310);

	FLT_INFO_LOG("%s:\n", __func__);
	return 0;
}

static const struct i2c_device_id tps61310_id[] = {
	{ "TPS61310_FLASHLIGHT", 0 },
	{ }
};
static int tps61310_resume(struct i2c_client *client)
{
	FLT_INFO_LOG("%s:\n", __func__);
	gpio_set_value(this_tps61310->strb1, 1);
	return 0;
}
static int tps61310_suspend(struct i2c_client *client, pm_message_t state)
{
	FLT_INFO_LOG("%s:\n", __func__);
	gpio_set_value(this_tps61310->strb1, 0);
	return 0;
}


static struct i2c_driver tps61310_driver = {
	.probe		= tps61310_probe,
	.remove		= tps61310_remove,
	.suspend	= tps61310_suspend,
	.resume		= tps61310_resume,
	.id_table	= tps61310_id,
	.driver		= {
		.name = "TPS61310_FLASHLIGHT",
	},
};

static int __init tps61310_init(void)
{
	FLT_INFO_LOG("tps61310 Led Flash driver: init\n");
	return i2c_add_driver(&tps61310_driver);
}

static void __exit tps61310_exit(void)
{
	i2c_del_driver(&tps61310_driver);
}

module_init(tps61310_init);
module_exit(tps61310_exit);

MODULE_DESCRIPTION("TPS61310 Led Flash driver");
MODULE_LICENSE("GPL");
