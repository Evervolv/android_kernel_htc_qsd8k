/* driver/i2c/chip/tap2018d1.c
 *
 * TI TPA2018D1 Speaker Amp
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/mutex.h>
#include <linux/tpa2018d1.h>

#include "board-supersonic-tpa2018d1.h"

#define DEBUG (0)

static struct i2c_client *this_client;
static struct tpa2018d1_platform_data *pdata;

struct mutex spk_amp_lock;
static int tpa2018d1_opened;
static int last_spkamp_state;
static char SPK_AMP_CFG[8];
static char DEFAULT_SPK_AMP_ON[] =
			{0x01, 0xc3, 0x20, 0x01, 0x00, 0x08, 0x1a, 0x21};
static char SPK_AMP_0FF[] = {0x01, 0xa2};
static char *config_data;
static int tpa2018d1_num_modes;

static int tpa2018_i2c_write(char *txData, int length)
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
		pr_err("%s: I2C transfer error\n", __func__);
		return -EIO;
	} else
		return 0;
}

static int tpa2018_i2c_read(char *rxData, int length)
{
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("%s: rx[%d] = %2x\n", __func__, i, rxData[i]);
	}
#endif

	return 0;
}

static int tpa2018d1_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&spk_amp_lock);

	if (tpa2018d1_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}

	tpa2018d1_opened = 1;
done:
	mutex_unlock(&spk_amp_lock);
	return rc;
}

static int tpa2018d1_release(struct inode *inode, struct file *file)
{
	mutex_lock(&spk_amp_lock);
	tpa2018d1_opened = 0;
	mutex_unlock(&spk_amp_lock);

	return 0;
}

void tpa2018d1_set_speaker_amp(int on)
{
	mutex_lock(&spk_amp_lock);
	if (on && !last_spkamp_state) {
		gpio_set_value(pdata->gpio_tpa2018_spk_en, 1);
		mdelay(5); /* According to TPA2018D1 Spec */
		if (tpa2018_i2c_write(SPK_AMP_CFG, sizeof(SPK_AMP_CFG)) == 0) {
			last_spkamp_state = 1;
			pr_info("%s: ON, value = %x %x\n", __func__, SPK_AMP_CFG[0], SPK_AMP_CFG[1]);
		}
	} else if (!on && last_spkamp_state) {
		if (tpa2018_i2c_write(SPK_AMP_0FF, sizeof(SPK_AMP_0FF)) == 0) {
			last_spkamp_state = 0;
			mdelay(2);
			gpio_set_value(pdata->gpio_tpa2018_spk_en, 0);
			pr_info("%s: OFF\n", __func__);
		}
	}
	mutex_unlock(&spk_amp_lock);
}

static long
tpa2018d1_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0;
	unsigned char tmp[7];
	int mode = -1;
	int offset = 0;
	unsigned char reg_idx[1] = {0x01};
	struct tpa2018d1_config_data cfg;

	switch (cmd) {
	case TPA2018_SET_CONFIG:
		if (copy_from_user(SPK_AMP_CFG, argp, sizeof(SPK_AMP_CFG)))
		/* TODO: content validation? */
		break;
	case TPA2018_SET_MODE:
		if (copy_from_user(&mode, argp, sizeof(mode)))
			return -EFAULT;
		if (mode >= tpa2018d1_num_modes || mode < 0) {
			pr_err("unsupported tpa2018d1 mode %d\n", mode);
			return -EINVAL;
		}
		memcpy(SPK_AMP_CFG, config_data + mode * TPA2018D1_CMD_LEN,
				TPA2018D1_CMD_LEN);
		break;
	case TPA2018_READ_CONFIG:
		mutex_lock(&spk_amp_lock);
		if (!last_spkamp_state) {
			gpio_set_value(pdata->gpio_tpa2018_spk_en, 1);
			mdelay(5); /* According to TPA2018D1 Spec */
		}

		rc = tpa2018_i2c_write(reg_idx, sizeof(reg_idx));
		if (rc < 0)
			goto err;

		rc = tpa2018_i2c_read(tmp, sizeof(tmp));
		if (rc < 0)
			goto err;

		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			rc = -EFAULT;
err:
		if (!last_spkamp_state)
			gpio_set_value(pdata->gpio_tpa2018_spk_en, 0);
		mutex_unlock(&spk_amp_lock);
		break;
	case TPA2018_SET_PARAM:
		cfg.mode_num = 0;
		cfg.cmd_data = 0;
		if (copy_from_user(&cfg, argp, sizeof(cfg))) {
			pr_err("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}
		tpa2018d1_num_modes = cfg.mode_num;
		if (tpa2018d1_num_modes > TPA2018_NUM_MODES) {
			pr_err("%s: invalid number of modes %d\n", __func__,
					tpa2018d1_num_modes);
			return -EINVAL;
		}
		if (cfg.data_len != tpa2018d1_num_modes*TPA2018D1_CMD_LEN) {
				pr_err("%s: invalid data length %d, expecting %d\n",
						__func__, cfg.data_len,
						tpa2018d1_num_modes * TPA2018D1_CMD_LEN);
				return -EINVAL;
		}
		config_data = kmalloc(cfg.data_len, GFP_KERNEL);
		if (!config_data) {
			pr_err("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(config_data, cfg.cmd_data, cfg.data_len)) {
			pr_err("%s: copy data from user failed.\n", __func__);
			kfree(config_data);
			return -EFAULT;
		}
		/* replace default setting with playback setting */
		if (tpa2018d1_num_modes >= TPA2018_MODE_PLAYBACK) {
			offset = TPA2018_MODE_PLAYBACK * TPA2018D1_CMD_LEN;
			memcpy(SPK_AMP_CFG, config_data + offset,
					TPA2018D1_CMD_LEN);
		}
		break;
	default:
		pr_err("%s: Invalid command\n", __func__);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static struct file_operations tpa2018d1_fops = {
	.owner = THIS_MODULE,
	.open = tpa2018d1_open,
	.release = tpa2018d1_release,
	.unlocked_ioctl = tpa2018d1_ioctl,
};

static struct miscdevice tpa2018d1_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tpa2018d1",
	.fops = &tpa2018d1_fops,
};

int tpa2018d1_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			pr_err("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}

	this_client = client;

	ret = gpio_request(pdata->gpio_tpa2018_spk_en, "tpa2018");
	if (ret < 0) {
		pr_err("%s: gpio request aud_spk_en pin failed\n", __func__);
		goto err_free_gpio_all;
	}

	ret = gpio_direction_output(pdata->gpio_tpa2018_spk_en, 1);
	if (ret < 0) {
		pr_err("%s: request aud_spk_en gpio direction failed\n",
			__func__);
		goto err_free_gpio_all;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_free_gpio_all;
	}

	gpio_set_value(pdata->gpio_tpa2018_spk_en, 0); /* Default Low */

	ret = misc_register(&tpa2018d1_device);
	if (ret) {
		pr_err("%s: tpa2018d1_device register failed\n", __func__);
		goto err_free_gpio_all;
	}
	memcpy(SPK_AMP_CFG, DEFAULT_SPK_AMP_ON, sizeof(DEFAULT_SPK_AMP_ON));
	return 0;

err_free_gpio_all:
	gpio_free(pdata->gpio_tpa2018_spk_en);
err_alloc_data_failed:
	return ret;
}

static int tpa2018d1_remove(struct i2c_client *client)
{
	struct tpa2018d1_platform_data *p2018data = i2c_get_clientdata(client);
	kfree(p2018data);

	return 0;
}

static int tpa2018d1_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int tpa2018d1_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tpa2018d1_id[] = {
	{ TPA2018D1_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tpa2018d1_driver = {
	.probe = tpa2018d1_probe,
	.remove = tpa2018d1_remove,
	.suspend = tpa2018d1_suspend,
	.resume = tpa2018d1_resume,
	.id_table = tpa2018d1_id,
	.driver = {
		.name = TPA2018D1_I2C_NAME,
	},
};

static int __init tpa2018d1_init(void)
{
	pr_info("%s\n", __func__);
	mutex_init(&spk_amp_lock);
	return i2c_add_driver(&tpa2018d1_driver);
}

static void __exit tpa2018d1_exit(void)
{
	i2c_del_driver(&tpa2018d1_driver);
}

module_init(tpa2018d1_init);
module_exit(tpa2018d1_exit);

MODULE_DESCRIPTION("TPA2018D1 Speaker Amp driver");
MODULE_LICENSE("GPL");
