/* driver/i2c/chip/tap6130.c
 *
 * TI TPA6130 Headset Amp
 *
 * Copyright (C) 2009 HTC Corporation
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

#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <mach/tpa6130.h>
#include <linux/mutex.h>
#include <mach/msm_rpcrouter.h>

#define HEADSET_MTOA_PROG                      0x30100003
#define HEADSET_MTOA_VERS                      0
#define HTC_HEADSET_NULL_PROC                  0
#define HTC_HEADSET_CTL_PROC                   1

static struct i2c_client *this_client;
struct mutex amp_mutex;
static struct tpa6130_platform_data *pdata;

static int i2c_on;
char buffer[2];

static int I2C_TxData(char *txData, int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		pr_err("tpa6130 :I2C transfer error\n");
		return -EIO;
	} else
		return 0;
}

void set_headset_amp(int on)
{
	mutex_lock(&amp_mutex);
	if (on && !i2c_on) {
		buffer[0] = 0x01;
		buffer[1] = 0xC0;
		buffer[2] = 0x3E;
		if (I2C_TxData(buffer, 3) == 0) {
			i2c_on = 1;
			pr_err("tpa6130: turn on headset amp !\n");
		}
	} else if (!on && i2c_on) {
		buffer[0] = 0x01;
		buffer[1] = 0xC1;
		if (I2C_TxData(buffer, 2) == 0) {
			i2c_on = 0;
			pr_err("tpa6130: turn off headset amp !\n");
		}
	}
	mutex_unlock(&amp_mutex);
}

static int handle_headset_call(struct msm_rpc_server *server,
			       struct rpc_request_hdr *req, unsigned len)
{
	struct rpc_headset_amp_ctl_args *args;

	if (!pdata->enable_rpc_server)
		return 0;

	switch (req->procedure) {
	case HTC_HEADSET_NULL_PROC:
		return 0;
	case HTC_HEADSET_CTL_PROC:
		args = (struct rpc_headset_amp_ctl_args *)(req + 1);
		args->on = be32_to_cpu(args->on);
		if (args->on) {
			gpio_set_value(pdata->gpio_hp_sd, 1);
			msleep(10);
			set_headset_amp(args->on);
		} else if (!args->on) {
			set_headset_amp(args->on);
			gpio_set_value(pdata->gpio_hp_sd, 0);
		}
		return 0;
	default:
		pr_err("tpa6130a: the wrong proc for headset server\n");
	}
	return -ENODEV;
}

static struct msm_rpc_server headset_server = {
	.prog = HEADSET_MTOA_PROG,
	.vers = HEADSET_MTOA_VERS,
	.rpc_call = handle_headset_call
};

int tpa6130_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		pr_err("tpa6130: platform data is NULL\n");
		goto fault;
	}

	if (pdata->enable_rpc_server) {
		msm_rpc_create_server(&headset_server);

		ret = gpio_request(pdata->gpio_hp_sd, "tpa6130");
		if (ret < 0) {
			pr_err("tap6130a : gpio request failed\n");
			goto fault;
		}

		ret = gpio_direction_output(pdata->gpio_hp_sd, 1);
		if (ret < 0) {
			pr_err("tap6130a: request reset gpio failed\n");
			goto fault;
		}
		gpio_set_value(pdata->gpio_hp_sd, 0);
	}

	this_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("tpa6130a: i2c check functionality error\n");
		goto fault;
	}

	return 0;
fault:
	return -ENODEV;
}

static int tpa6130_remove(struct i2c_client *client)
{
	return 0;
}
static const struct i2c_device_id tpa6130_id[] = {
	{ TPA6130_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tpa6130_driver = {
	.probe 	        = tpa6130_probe,
	.remove 	= tpa6130_remove,
	.id_table	= tpa6130_id,
	.driver = {
		   .name = TPA6130_I2C_NAME,
		   },
};

static int __init tpa6130_init(void)
{
	pr_err("tpa6130 HP AMP: init\n");
	mutex_init(&amp_mutex);
	return i2c_add_driver(&tpa6130_driver);
}

static void __exit tpa6130_exit(void)
{
	i2c_del_driver(&tpa6130_driver);
}

module_init(tpa6130_init);
module_exit(tpa6130_exit);
