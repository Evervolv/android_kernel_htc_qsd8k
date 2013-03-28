/* board-htcleo-microp.c
 * Copyright (C) 2009 Google.
 * Copyright (C) 2009 HTC Corporation.
 *
 * The Microp on htcleo is an i2c device that supports
 * the following functions
  *  - G-sensor
 *   - Proximity (capella cm3602)
 *   - Interrupts
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
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/bma150.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <asm/setup.h>
#include <mach/htc_pwrsink.h>
#include <mach/board-htcleo-microp.h>

#include "board-htcleo.h"

static uint32_t microp_als_kadc;
static int als_power_control=0;
static DEFINE_MUTEX(capella_cm3602_lock);


extern void p_sensor_irq_handler(void);

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

#define I2C_READ_RETRY_TIMES  10
#define I2C_WRITE_RETRY_TIMES 10

static int i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	struct microp_i2c_client_data *cdata;
	struct i2c_msg msgs[] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &addr,
	},
	{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
	}
	};

	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->microp_i2c_rw_mutex);
	mdelay(1);
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		msleep(5);
	}
	mutex_unlock(&cdata->microp_i2c_rw_mutex);
	dev_dbg(&client->dev, "R [%02X] = %s\n",
			addr, hex2string(data, length));

	if (retry > I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

#define MICROP_I2C_WRITE_BLOCK_SIZE 21
static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	struct microp_i2c_client_data *cdata;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	cdata = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "W [%02X] = %s\n", addr,
	hex2string(data, length));

	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	memcpy((void *)&buf[1], (void *)data, length);

//	mdelay(1);
// Cotulla: extra delay
//	msleep(10);
	mutex_lock(&cdata->microp_i2c_rw_mutex);
	mdelay(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(5);
	}
	if (retry > I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->microp_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->microp_i2c_rw_mutex);
	return 0;
}

int microp_i2c_read(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_read_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_read);

int microp_i2c_write(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_write_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_write);

static int microp_spi_enable(uint8_t on)
{
	struct i2c_client *client;
	int ret;

	client = private_microp_client;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_SPI_EN, &on, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
		return ret;
	}
	msleep(10);
	return ret;
}

int microp_spi_vote_enable(int spi_device, uint8_t enable)
{
	// Only a dummy for the bma_150 driver, enable only the SPI
	int ret;
	ret=0;

	ret = microp_spi_enable(enable);
	return ret;

}

static int microp_read_adc(uint8_t channel, uint16_t *value)
{
	struct i2c_client *client;
	int ret;
	uint8_t cmd[2], data[2];

	client = private_microp_client;
	cmd[0] = 0;
	cmd[1] = 1; //channel;
//	ret = i2c_write_block(client, MICROP_I2C_WCMD_READ_ADC_REQ, cmd, 2);
	ret = i2c_write_block(client, MICROP_I2C_WCMD_READ_ADC_VALUE_REQ, cmd, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: request adc fail\n", __func__);
		return -EIO;
	}

	ret = i2c_read_block(client, MICROP_I2C_RCMD_ADC_VALUE, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
		return -EIO;
	}
	*value = data[0] << 8 | data[1];
	return 0;
}

/**
 * GPI functions
 **/

static int microp_read_gpi_status(struct i2c_client *client, uint16_t *status)
{
	uint8_t data[2];
	int ret;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GPIO_STATUS, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read failed\n", __func__);
		return -EIO;
	}
	*status = (data[0] << 8) | data[1];
	return 0;
}

static int microp_interrupt_get_status(uint16_t *interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	ret = microp_i2c_read(MICROP_I2C_RCMD_GPI_INT_STATUS, data, 2);
	if (ret < 0) {
		pr_err("%s: read interrupt status fail\n",  __func__);
		return ret;
	}

	*interrupt_mask = data[0]<<8 | data[1];
	return 0;
}

static int microp_interrupt_enable( uint16_t interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	data[0] = interrupt_mask >> 8;
	data[1] = interrupt_mask & 0xFF;
	ret = microp_i2c_write(MICROP_I2C_WCMD_GPI_INT_CTL_EN, data, 2);

	if (ret < 0)
		pr_err("%s: enable 0x%x interrupt failed\n", __func__, interrupt_mask);
	return ret;
}

static int microp_interrupt_disable(uint16_t interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	data[0] = interrupt_mask >> 8;
	data[1] = interrupt_mask & 0xFF;
	ret = microp_i2c_write(MICROP_I2C_WCMD_GPI_INT_CTL_DIS, data, 2);

	if (ret < 0)
		pr_err("%s: disable 0x%x interrupt failed\n", __func__, interrupt_mask);
	return ret;
}

/**
 * GPO functions TODO
 **/

int microp_read_gpo_status(uint16_t *status)
{
	uint8_t data[2];
	int ret;
	struct i2c_client *client;

	client = private_microp_client;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GPIO_STATUS, data, 2);
	if (ret < 0) 
	{
		dev_err(&client->dev, "%s: read failed\n", __func__);
		return -EIO;
	}
	*status = (data[0] << 8) | data[1];
	return 0;
}
EXPORT_SYMBOL(microp_read_gpo_status);

int microp_gpo_enable(uint16_t gpo_mask)
{
	uint8_t data[2];
	int ret = -1;
	struct i2c_client *client;

	client = private_microp_client;

	data[0] = gpo_mask >> 8;
	data[1] = gpo_mask & 0xFF;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPO_LED_STATUS_EN, data, 2);

	if (ret < 0)
		dev_err(&client->dev, "%s: enable 0x%x interrupt failed\n", __func__, gpo_mask);
	return ret;
}
EXPORT_SYMBOL(microp_gpo_enable);

int microp_gpo_disable(uint16_t gpo_mask)
{
	uint8_t data[2];
	int ret = -1;
	struct i2c_client *client;

	client = private_microp_client;

	data[0] = gpo_mask >> 8;
	data[1] = gpo_mask & 0xFF;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPO_LED_STATUS_DIS, data, 2);

	if (ret < 0)
		dev_err(&client->dev, "%s: disable 0x%x interrupt failed\n", __func__, gpo_mask);
	return ret;
}
EXPORT_SYMBOL(microp_gpo_disable);


int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	uint16_t interrupts = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);
	if(pwr_device==PS_PWR_ON) { // Switch the Proximity IRQ
		if(enable) {
			microp_gpo_enable(PS_PWR_ON);
			ret = microp_interrupt_get_status(&interrupts);
			if (ret < 0) {
				pr_err("%s: read interrupt status fail\n", __func__);
				return ret;
			}
			interrupts |= IRQ_PROXIMITY;
			ret = microp_interrupt_enable(interrupts);
		}
		else {
			interrupts |= IRQ_PROXIMITY;
			ret = microp_interrupt_disable(interrupts);
			microp_gpo_disable(PS_PWR_ON);
		}
		if (ret < 0) {
			pr_err("%s: failed to enable gpi irqs\n", __func__);
			return ret;
		}
	}
	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		microp_gpo_enable(LS_PWR_ON);
	else if (!on)
		microp_gpo_disable(LS_PWR_ON);
	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

/*
 * Interrupt
 */
static irqreturn_t microp_i2c_intr_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = to_i2c_client(dev_id);
	cdata = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "intr_irq_handler\n");

	disable_irq_nosync(client->irq);
	schedule_work(&cdata->work.work);
	return IRQ_HANDLED;
}

static void microp_i2c_intr_work_func(struct work_struct *work)
{
	struct microp_i2c_work *up_work;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[3];
	uint16_t intr_status = 0;
	int ret = 0;

	up_work = container_of(work, struct microp_i2c_work, work);
	client = up_work->client;
	cdata = i2c_get_clientdata(client);

	ret = microp_interrupt_get_status(&intr_status);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read interrupt status fail\n",
			 __func__);
	}

	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_STATUS_CLR, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: clear interrupt status fail\n",
			 __func__);
	}

	if (intr_status & IRQ_PROXIMITY) {
		p_sensor_irq_handler();
	}

	enable_irq(client->irq);
}


static int microp_function_initialize(struct i2c_client *client)
{    
	struct microp_i2c_client_data *cdata;
	uint16_t stat, interrupts = 0;
	int ret;

	cdata = i2c_get_clientdata(client);

	/* enable the interrupts */
	ret = microp_interrupt_enable(interrupts);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to enable gpi irqs\n",
			__func__);
		goto err_irq_en;
	}

	microp_read_gpi_status(client, &stat);

	return 0;

err_irq_en:
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void microp_early_suspend(struct early_suspend *h)
{
	struct microp_i2c_client_data *cdata;
	struct i2c_client *client = private_microp_client;
	int ret;

	if (!client) {
		pr_err("%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);

	cdata->microp_is_suspend = 1;

	disable_irq(client->irq);
	ret = cancel_work_sync(&cdata->work.work);
	if (ret != 0) {
		enable_irq(client->irq);
	}

}

void microp_early_resume(struct early_suspend *h)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;

	if (!client) {
		pr_err("%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);

	cdata->microp_is_suspend = 0;
	enable_irq(client->irq);
}
#endif

static int microp_i2c_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
	return 0;
}

static int microp_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static void register_microp_devices(struct platform_device *devices, int num)
{
	int i;
	for (i = 0; i < num; i++) {
		platform_device_register(devices + i);
		dev_set_drvdata(&(devices + i)->dev, private_microp_client);
	}
}

static int microp_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	int ret;

	cdata = kzalloc(sizeof(struct microp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocat cdata\n");
		goto err_cdata;
	}
	i2c_set_clientdata(client, cdata);

	private_microp_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "failed on get pdata\n");
		goto err_exit;
	}
	pdata->dev_id = (void *)&client->dev;
	cdata->gpio_reset=pdata->gpio_reset;

	mutex_init(&cdata->microp_i2c_rw_mutex);

	ret = i2c_read_block(client, MICROP_I2C_RCMD_VERSION, data, 2);
	if (ret || !(data[0] && data[1])) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp version\n");
		goto err_exit;
	}
	dev_info(&client->dev, "microp version [%02X][%02X]\n",
		  data[0], data[1]);

	ret = gpio_request(pdata->gpio_reset, "microp_i2c_wm");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio reset\n");
		goto err_exit;
	}
	ret = gpio_direction_output(pdata->gpio_reset, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			 "failed on gpio_direction_output reset\n");
		goto err_gpio_reset;
	}

	cdata->version = data[0] << 8 | data[1];
	cdata->microp_is_suspend = 0;

	wake_lock_init(&microp_i2c_wakelock, WAKE_LOCK_SUSPEND,
			 "microp_i2c_present");
	
	register_microp_devices(pdata->microp_devices, pdata->num_devices);
	
	/* Setup IRQ handler */
	INIT_WORK(&cdata->work.work, microp_i2c_intr_work_func);
	cdata->work.client = client;

	ret = request_irq(client->irq,
			microp_i2c_intr_irq_handler,
			IRQF_TRIGGER_LOW,
			"microp_interrupt",
			&client->dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_intr;
	}
	ret = set_irq_wake(client->irq, 1);
	if (ret) {
		dev_err(&client->dev, "set_irq_wake failed\n");
		goto err_intr;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend) {
		cdata->early_suspend.level =
				EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		cdata->early_suspend.suspend = microp_early_suspend;
		cdata->early_suspend.resume = microp_early_resume;
		register_early_suspend(&cdata->early_suspend);
	}
#endif

	ret = microp_function_initialize(client);
	if (ret) {
		dev_err(&client->dev, "failed on microp function initialize\n");
		goto err_fun_init;
	}
	
	dev_info(&client->dev, "Init Done\n");
	return 0;

err_fun_init:
err_intr:
	wake_lock_destroy(&microp_i2c_wakelock);
	kfree(cdata);
	i2c_set_clientdata(client, NULL);

err_cdata:
err_gpio_reset:
	gpio_free(pdata->gpio_reset);
err_exit:
	dev_info(&client->dev, "Init Error\n");
	return ret;
}

static int __devexit microp_i2c_remove(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend) {
		unregister_early_suspend(&cdata->early_suspend);
	}
#endif

	free_irq(client->irq, &client->dev);

	gpio_free(cdata->gpio_reset);

	kfree(cdata);

	return 0;
}

#define ATAG_ALS	0x5441001b
static int __init parse_tag_microp_als_kadc(const struct tag *tags)
{
	int found = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_ALS) {
			found = 1;
			break;
		}
	}

	if (found)
		microp_als_kadc = t->u.revision.rev;
	pr_debug("%s: microp_als_kadc = 0x%x\n", __func__, microp_als_kadc);
	return 0;
}
__tagtable(ATAG_ALS, parse_tag_microp_als_kadc);

static const struct i2c_device_id microp_i2c_id[] =
{
	{ MICROP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver microp_i2c_driver =
{
	.driver = {
		   .name = MICROP_I2C_NAME,
	},
	.id_table = microp_i2c_id,
	.probe = microp_i2c_probe,
	.suspend = microp_i2c_suspend,
	.resume = microp_i2c_resume,
	.remove = __devexit_p(microp_i2c_remove),
};

static int __init microp_i2c_init(void)
{
	return i2c_add_driver(&microp_i2c_driver);
}

static void __exit microp_i2c_exit(void)
{
	i2c_del_driver(&microp_i2c_driver);
}

module_init(microp_i2c_init);
module_exit(microp_i2c_exit);

MODULE_AUTHOR("Eric Olsen <eolsen@android.com>");
MODULE_DESCRIPTION("MicroP I2C driver");
MODULE_LICENSE("GPL");

static int micropklt_dbg_leds_set(void *dat, u64 val)
{
	struct i2c_client *client;
	char buffer[3] = { 0, 0, 0 };
	int r;

	client = private_microp_client;

	buffer[0] = 0xff & (val >> 8);
	buffer[1] = 0xff & (val >> 16);
	buffer[2] = 0xff & (val >> 24);
	r =i2c_write_block(client, 0xff & val, buffer, 3);
	return r;
}

static int micropklt_dbg_leds_get(void *data, u64 *val) {
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(micropklt_dbg_leds_fops,
		micropklt_dbg_leds_get,
		micropklt_dbg_leds_set, "%llu\n");

		
static int __init micropklt_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("micropklt", NULL);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("raw", 0444, dent, NULL,
			&micropklt_dbg_leds_fops);
	return 0;
}

device_initcall(micropklt_dbg_init);

