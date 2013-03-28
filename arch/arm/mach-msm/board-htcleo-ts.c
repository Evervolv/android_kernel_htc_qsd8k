/* board-htcleo-ts.c
 *
 * Copyright (C) 2010 Cotulla
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

//#define DEBUG


#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <mach/board-htcleo-ts.h>
#include "board-htcleo.h"
#include "gpio_chip.h"
#include "proc_comm.h"




#define TOUCH_TYPE_UNKNOWN  0
#define TOUCH_TYPE_B8       1
#define TOUCH_TYPE_68       2
#define TOUCH_TYPE_2A       3

#define LEO_TYPE_1          1
#define LEO_TYPE_2          2
#define LEO_TYPE_3          3

#define TYPE_68_DEVID	(0x68 >> 1)
#define TYPE_B8_DEVID	(0xB8 >> 1)
#define TYPE_2A_DEVID	(0x2A >> 1)

#define TS_USE_IRQ		1

#define MAKEWORD(a, b)      ((uint16_t)(((uint8_t)(a)) | ((uint16_t)((uint8_t)(b))) << 8))


struct htcleo_ts_data
{
	struct i2c_client *client;
	struct input_dev *input_dev;
	uint32_t prev_ptcount;
	int ts_type;
	int intr_type;  // 0 - FAILING, 1- RISING
	int pressed1;
	int pressed2;
	struct work_struct work;
#ifndef TS_USE_IRQ	
	struct hrtimer timer;
#endif
	uint16_t version;
	struct early_suspend early_suspend;
};

struct workqueue_struct *htcleo_touch_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void htcleo_ts_early_suspend(struct early_suspend *h);
static void htcleo_ts_late_resume(struct early_suspend *h);
#endif



static int I2C_Read(struct htcleo_ts_data *ts, uint8_t dev, uint8_t addr, uint32_t sz, uint8_t* bf)
{
	struct i2c_msg msg[2];
	int ret = 0;

	msg[0].addr = dev;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = dev;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sz;
	msg[1].buf = bf;

	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0)
	{
		dev_dbg(&ts->client->dev, "TS: I2C_Read(%x %x) failed!\n", dev, addr);
		return 0;
	}
	return 1;
}

static int I2C_ReadNo(struct htcleo_ts_data *ts, uint8_t dev, uint32_t sz, uint8_t* bf)
{
	struct i2c_msg msg[1];
	int ret = 0;

	msg[0].addr = dev;
	msg[0].flags = I2C_M_RD;
	msg[0].len = sz;
	msg[0].buf = bf;

	ret = i2c_transfer(ts->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_dbg(&ts->client->dev, "TS: I2C_ReadNo(%x) failed!\n", dev);
		return 0;
	}
	return 1;
}

static int htcleo_detect_ts_type(struct htcleo_ts_data *ts)
{
	uint8_t bt[4];

	ts->ts_type  = TOUCH_TYPE_UNKNOWN;
	if (I2C_Read(ts, TYPE_B8_DEVID, 0x00, 1, bt))
	{
		dev_dbg(&ts->client->dev, "TS: DETECTED TYPE B8\n");
		ts->ts_type = TOUCH_TYPE_B8;
		return 1;
	}

	if (I2C_Read(ts, TYPE_68_DEVID, 0x00, 1, bt))
	{
		dev_dbg(&ts->client->dev, "TS: DETECTED TYPE 68\n");
		ts->ts_type = TOUCH_TYPE_68;
		return 1;
	}

	if (I2C_ReadNo(ts, TYPE_2A_DEVID, 4, bt) && bt[0] == 0x55)
	{
		dev_dbg(&ts->client->dev, "TS: DETECTED TYPE 2A\n");
		ts->ts_type = TOUCH_TYPE_2A;
		return 1;
	}
	dev_dbg(&ts->client->dev, "TS: NOT DETECTED\n");
	return -1;
}


static int htcleo_reset_ts(struct htcleo_ts_data *ts)
{
	while (gpio_get_value(HTCLEO_GPIO_TS_POWER))
	{
		gpio_set_value(HTCLEO_GPIO_TS_SEL, 1);
		gpio_set_value(HTCLEO_GPIO_TS_POWER, 0);
		gpio_set_value(HTCLEO_GPIO_TS_MULT, 0);
		gpio_set_value(HTCLEO_GPIO_H2W_CLK, 0);
		msleep(10);
	}
	gpio_set_value(HTCLEO_GPIO_TS_MULT, 1);
	gpio_set_value(HTCLEO_GPIO_H2W_CLK, 1);

	while (!gpio_get_value(HTCLEO_GPIO_TS_POWER))
	{
		gpio_set_value(HTCLEO_GPIO_TS_POWER, 1);
	}
	msleep(20);
	while (gpio_get_value(HTCLEO_GPIO_TS_IRQ))
	{
		msleep(10);
	}

	while (gpio_get_value(HTCLEO_GPIO_TS_SEL))
	{
		gpio_set_value(HTCLEO_GPIO_TS_SEL, 0);
	}
	msleep(300);
	dev_dbg(&ts->client->dev, "TS: reset done\n");
	return 1;
}

static int htcleo_init_ts(struct htcleo_ts_data *ts)
{
	uint8_t bt[6];
	struct i2c_msg msg;
	int ret;

	switch (ts->ts_type)
	{
	case TOUCH_TYPE_2A:
		bt[0] = 0xD0;
		bt[1] = 0x00;
		bt[2] = 0x01;

		msg.addr = 0x2A >> 1;
		msg.flags = 0;
		msg.len = 3;
		msg.buf = bt;

		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret < 0)
		{
		    goto error;
		}
	break;
	default:
		dev_dbg(&ts->client->dev, "TS: wrong type %x\n", ts->ts_type);
		goto error;
	}
	dev_dbg(&ts->client->dev, "TS: init done\n");
	return 1;
error:
	return -1;
}


static void htcleo_deinit_ts(void)
{
     gpio_set_value(HTCLEO_GPIO_TS_POWER, 0);
}

static int htcleo_init_intr(struct htcleo_ts_data *ts)
{
	switch (ts->ts_type)
	{
	case TOUCH_TYPE_B8:
		ts->intr_type = 1;
	break;
	case TOUCH_TYPE_68:
	case TOUCH_TYPE_2A:
		ts->intr_type = 0;
	break;
	default:
		dev_dbg(&ts->client->dev, "TS: wrong type %x\n", ts->ts_type);
		ts->intr_type = 0;
		goto error;
	}
	return 1;
    error:
	return -1;
}


static void htcleo_ts_work_func(struct work_struct *work)
{
	uint8_t buf[9];
	uint32_t ptcount;
	uint32_t ptx[2];
	uint32_t pty[2];
	struct htcleo_ts_data *ts = container_of(work, struct htcleo_ts_data, work);    
	int pressed1, pressed2;

	ptcount = 0;
	switch (ts->ts_type)
	{
	case TOUCH_TYPE_2A:
		if (!I2C_ReadNo(ts, TYPE_2A_DEVID, 9, buf))
		{
			dev_dbg(&ts->client->dev, "TS: ReadPos failed\n");
			goto error;
		}
		if (buf[0] != 0x5A)
		{
			dev_dbg(&ts->client->dev, "TS: ReadPos wrmark\n");
			goto error;
		}

		ptcount = (buf[8] >> 1) & 3;
		if (ptcount > 2)
		{
			ptcount = 2;
		}

		if (ptcount >= 1)
		{
			ptx[0] = MAKEWORD(buf[2], (buf[1] & 0xF0) >> 4);
			pty[0] = MAKEWORD(buf[3], (buf[1] & 0x0F) >> 0);
		}
		if (ptcount == 2)
		{
			ptx[1] = MAKEWORD(buf[5], (buf[4] & 0xF0) >> 4);
			pty[1] = MAKEWORD(buf[6], (buf[4] & 0x0F) >> 0);
		}
		break;
	default:
		dev_dbg(&ts->client->dev, "TS: wrong type %x\n", ts->ts_type);
		goto error;
	}


	if (ptcount == 0)
		dev_dbg(&ts->client->dev, "TS: not pressed\n");
	else if (ptcount == 1)
		dev_dbg(&ts->client->dev, "TS: pressed1 (%d, %d)\n", ptx[0], pty[0]);
	else if (ptcount == 2)
		dev_dbg(&ts->client->dev, "TS: pressed2 (%d, %d) (%d, %d)\n", ptx[0], pty[0], ptx[1], pty[1]);
	else
		dev_dbg(&ts->client->dev, "TS: BUGGY!\n");


	if (ptcount == 0)
	{
		pressed1 = 0;
		pressed2 = 0;
	}
	else if (ptcount == 1)
	{
		pressed1 = 1;
		pressed2 = 0;
	}
	else if (ptcount == 2)
	{
		pressed1 = 1;
		pressed2 = 1;
	}
	else
	{
		pressed1 = 0;
		pressed2 = 0;
	}

	if (pressed1)
	{
		dev_dbg(&ts->client->dev, "pressed1\n");
		input_report_abs(ts->input_dev, ABS_X, ptx[0]);
		input_report_abs(ts->input_dev, ABS_Y, pty[0]);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 100);
		input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, 1);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
#ifdef CONFIG_HTCLEO_ENABLE_MULTI_TOUCH
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ptx[0]);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pty[0]);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
#endif
	}
	else if (ts->pressed1)
	{
		dev_dbg(&ts->client->dev, "unpressed1\n");
		input_report_abs(ts->input_dev, ABS_X, ptx[0]);
		input_report_abs(ts->input_dev, ABS_Y, pty[0]);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
#ifdef CONFIG_HTCLEO_ENABLE_MULTI_TOUCH
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ptx[0]);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pty[0]);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
#endif
	}
    #ifdef CONFIG_HTCLEO_ENABLE_MULTI_TOUCH
	input_mt_sync(ts->input_dev);


	if (pressed2)
	{
		dev_dbg(&ts->client->dev, "pressed2\n");
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ptx[1]);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pty[1]);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	}
	else if (ts->pressed2)
	{
		dev_dbg(&ts->client->dev, "unpressed2\n");
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	}
	input_mt_sync(ts->input_dev);
    #endif
	input_sync(ts->input_dev);

	ts->pressed1 = pressed1;
	ts->pressed2 = pressed2;

    error:
	ts->prev_ptcount = ptcount;

	/* prepare for next intr */
	enable_irq(ts->client->irq);
}

#ifndef TS_USE_IRQ	
static enum hrtimer_restart htcleo_ts_timer_func(struct hrtimer *timer)
{
	struct htcleo_ts_data *ts = container_of(timer, struct htcleo_ts_data, timer);
	queue_work(htcleo_touch_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t htcleo_ts_irq_handler(int irq, void *dev_id)
{
	struct htcleo_ts_data *ts = dev_id;
	disable_irq_nosync(ts->client->irq);
	queue_work(htcleo_touch_wq, &ts->work);
	return IRQ_HANDLED;
}
#endif


////////////////////////////////////////////////////////////////////////////

static uint32_t touch_on_gpio_table[] =
{
	PCOM_GPIO_CFG(HTCLEO_GPIO_TS_IRQ, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),
};

static int htcleo_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct htcleo_ts_data *ts;
	int ret = 0;
	int x_start, y_start, x_end, y_end;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct htcleo_ts_data), GFP_KERNEL);
	if (ts == NULL)
	{
		dev_err(&client->dev, "allocate htcleo_ts_data failed\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&ts->work, htcleo_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->prev_ptcount = 0;

	config_gpio_table(touch_on_gpio_table, ARRAY_SIZE(touch_on_gpio_table));

	gpio_request(HTCLEO_GPIO_TS_POWER, "htcleo_ts");
	gpio_request(HTCLEO_GPIO_TS_SEL, "htcleo_ts");
	gpio_request(HTCLEO_GPIO_TS_IRQ, "htcleo_ts");
	gpio_request(HTCLEO_GPIO_TS_MULT, "htcleo_ts");
	gpio_request(HTCLEO_GPIO_H2W_CLK, "htcleo_ts");

	gpio_direction_output(HTCLEO_GPIO_TS_SEL, 1);
	gpio_direction_output(HTCLEO_GPIO_TS_POWER, 0);
	gpio_direction_output(HTCLEO_GPIO_TS_MULT, 0);
	gpio_direction_output(HTCLEO_GPIO_H2W_CLK, 0);
	gpio_direction_input(HTCLEO_GPIO_TS_IRQ);
	msleep(100);

	ret = htcleo_reset_ts(ts);
	if (ret < 0)
	{
		dev_err(&client->dev, "TS: htcleo_reset_ts() failed\n");
		goto err_detect_failed;
	}

	ret = htcleo_detect_ts_type(ts);
	if (ret < 0)
	{
		dev_err(&client->dev, "TS: htcleo_detect_ts_type() failed\n");
		goto err_detect_failed;
	}

    // !!!other types are not supported yet!!!
	if (ts->ts_type == TOUCH_TYPE_68 || ts->ts_type == TOUCH_TYPE_B8)
	{
		dev_err(&ts->client->dev, "TS: NOT SUPPORTED\n");
		BUG();
	}

	ret = htcleo_init_ts(ts);
	if (ret < 0)
	{
		dev_err(&client->dev, "TS: htcleo_init_ts() failed\n");
		goto err_detect_failed;
	}

	ret = htcleo_init_intr(ts);
	if (ret < 0)
	{
		dev_err(&client->dev, "TS: htcleo_init_intr() failed\n");
		goto err_detect_failed;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "htcleo-touchscreen";


    // inital touch calibraion
	switch (ts->ts_type)
	{
	case TOUCH_TYPE_2A:
		x_start = 6;
		y_start = 6;
		x_end = 576;
		y_end = 952;
	break;
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH);
#ifdef CONFIG_HTCLEO_ENABLE_MULTI_TOUCH
	input_set_capability(ts->input_dev, EV_KEY, BTN_2);
#endif

	input_set_abs_params(ts->input_dev, ABS_X,     x_start, x_end, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_Y,     y_start, y_end, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 5, 0, 0);
#ifdef CONFIG_HTCLEO_ENABLE_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, x_start, x_end, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, y_start, y_end, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 1, 0, 0);
#endif
    //    input_set_abs_params(ts->input_dev, ABS_HAT0X, x_start, x_end, 0, 0);
    //    input_set_abs_params(ts->input_dev, ABS_HAT0Y, y_start, y_end, 0, 0);
    //    input_set_abs_params(ts->input_dev, ABS_PRESSURE, pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
    //    input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, pdata->abs_width_min, pdata->abs_width_max, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		dev_err(&client->dev,
		    "htcleo_ts_probe: Unable to register %s input device\n",
		    ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#ifdef TS_USE_IRQ	
	ts->client->irq = gpio_to_irq(HTCLEO_GPIO_TS_IRQ);
	ret = request_irq(ts->client->irq, htcleo_ts_irq_handler,
		      ts->intr_type ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW,
			LEO_TOUCH_DRV_NAME, ts);
#else
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = htcleo_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = htcleo_ts_early_suspend;
	ts->early_suspend.resume = htcleo_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	dev_info(&client->dev, "Start touchscreen %s\n", ts->input_dev->name);

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
    // POWER OFF
	htcleo_deinit_ts();
err_detect_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}



static int htcleo_ts_remove(struct i2c_client *client)
{
	struct htcleo_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);

#ifdef TS_USE_IRQ	
	free_irq(client->irq, ts);
#else
	hrtimer_cancel(&ts->timer);
#endif
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int htcleo_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct htcleo_ts_data *ts = i2c_get_clientdata(client);
	int ret;

#ifdef TS_USE_IRQ	
	disable_irq_nosync(client->irq);
#else
	hrtimer_cancel(&ts->timer);
#endif
	ret = cancel_work_sync(&ts->work);
	if (ret)
	{
		enable_irq(client->irq);
	}

	// POWER OFF
	htcleo_deinit_ts();

	return 0;
}

static int htcleo_ts_resume(struct i2c_client *client)
{
	struct htcleo_ts_data *ts = i2c_get_clientdata(client);

	// POWER ON
	htcleo_reset_ts(ts);
	htcleo_detect_ts_type(ts);
	htcleo_init_ts(ts);
	htcleo_init_intr(ts);

#ifdef TS_USE_IRQ	
	enable_irq(client->irq);
#else
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void htcleo_ts_early_suspend(struct early_suspend *h)
{
	struct htcleo_ts_data *ts;
	ts = container_of(h, struct htcleo_ts_data, early_suspend);
	htcleo_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void htcleo_ts_late_resume(struct early_suspend *h)
{
	struct htcleo_ts_data *ts;
	ts = container_of(h, struct htcleo_ts_data, early_suspend);
	htcleo_ts_resume(ts->client);
}
#endif


static const struct i2c_device_id htcleo_ts_id[] =
{
	{ LEO_TOUCH_DRV_NAME, 0 },
	{ }
};


static struct i2c_driver htcleo_ts_driver =
{
	.driver =
	{
	    .name = LEO_TOUCH_DRV_NAME,
	    .owner = THIS_MODULE,
	},
	.id_table   = htcleo_ts_id,
	.probe = htcleo_ts_probe,
	.remove = htcleo_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = htcleo_ts_suspend,
	.resume = htcleo_ts_resume,
#endif
};


static int __devinit htcleo_ts_init(void)
{
	htcleo_touch_wq = create_singlethread_workqueue("htcleo_touch_wq");
	if (!htcleo_touch_wq)
	{
		return -ENOMEM;
	}
	return i2c_add_driver(&htcleo_ts_driver);
}

static void __exit htcleo_ts_exit(void)
{
	if (htcleo_touch_wq)
	{
		destroy_workqueue(htcleo_touch_wq);
		htcleo_touch_wq = NULL;
	}
	i2c_del_driver(&htcleo_ts_driver);
}

module_init(htcleo_ts_init);
module_exit(htcleo_ts_exit);


MODULE_DESCRIPTION("HTC LEO Touchscreen Support Driver");
MODULE_LICENSE("GPL");
