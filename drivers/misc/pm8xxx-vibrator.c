/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/vibrator.h>

#include "../staging/android/timed_output.h"

#define VIB_DRV			0x4A

#define VIB_DRV_SEL_MASK	0xf8
#define VIB_DRV_SEL_SHIFT	0x03
#define VIB_DRV_EN_MANUAL_MASK	0xfc

#define VIB_MAX_LEVEL_mV	3100
#define VIB_MIN_LEVEL_mV	1200

#define VIB_DBG_LOG(fmt, ...) \
		({ if (0) printk(KERN_DEBUG "[VIB]" fmt, ##__VA_ARGS__); })
#define VIB_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[VIB]" fmt, ##__VA_ARGS__)
#define VIB_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[VIB][ERR]" fmt, ##__VA_ARGS__)

struct pm8xxx_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	spinlock_t lock;
	struct work_struct work;
	struct device *dev;
	const struct pm8xxx_vibrator_platform_data *pdata;
	int state;
	int level;
	u8  reg_vib_drv;
};

/* REVISIT: just for debugging, will be removed in final working version */
static void __dump_vib_regs(struct pm8xxx_vib *vib, char *msg)
{
	u8 temp;

	VIB_DBG_LOG("%s\n", msg);
	pm8xxx_readb(vib->dev->parent, VIB_DRV, &temp);
	VIB_DBG_LOG("VIB_DRV - %X\n", temp);
}

static int pm8xxx_vib_read_u8(struct pm8xxx_vib *vib,
				 u8 *data, u16 reg)
{
	int rc;

	rc = pm8xxx_readb(vib->dev->parent, reg, data);
	if (rc < 0)
		VIB_ERR_LOG("Error reading pm8xxx: %X - ret %X\n",
				reg, rc);

	return rc;
}

static int pm8xxx_vib_write_u8(struct pm8xxx_vib *vib,
				 u8 data, u16 reg)
{
	int rc;

	rc = pm8xxx_writeb(vib->dev->parent, reg, data);
	if (rc < 0)
		VIB_ERR_LOG("Error writing pm8xxx: %X - ret %X\n",
				reg, rc);
	return rc;
}

static int pm8xxx_vib_set_on(struct pm8xxx_vib *vib)
{
	int rc;
	u8 val1;
	val1 = vib->reg_vib_drv;
	val1 &= ~VIB_DRV_SEL_MASK;
	val1 |= ((vib->level << VIB_DRV_SEL_SHIFT) & VIB_DRV_SEL_MASK);
	VIB_INFO_LOG("%s + val: %x \n", __func__, val1);
	rc = pm8xxx_vib_write_u8(vib, val1, VIB_DRV);
	if (rc < 0){
		VIB_ERR_LOG("%s writing pmic fail, ret:%X\n", __func__, rc);
		return rc;
	}
	__dump_vib_regs(vib, "vib_set_end");
	VIB_INFO_LOG("%s - \n", __func__);
	return rc;
}


static int pm8xxx_vib_set_off(struct pm8xxx_vib *vib)
{
	int rc;
	u8 val2;
	val2 = vib->reg_vib_drv;
	val2 &= ~VIB_DRV_SEL_MASK;
	VIB_INFO_LOG("%s + val: %x \n", __func__, val2);
	rc = pm8xxx_vib_write_u8(vib, val2, VIB_DRV);
	if (rc < 0){
		VIB_ERR_LOG("%s writing pmic fail, ret:%X\n", __func__, rc);
		return rc;
	}
	__dump_vib_regs(vib, "vib_set_end");
	VIB_INFO_LOG("%s - \n", __func__);
	return rc;
}

static void pm8xxx_vib_enable(struct timed_output_dev *dev, int value)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
					 timed_dev);

	VIB_INFO_LOG("%s vibrate period: %d msec\n",__func__,value);
retry:

	if (hrtimer_try_to_cancel(&vib->vib_timer) < 0) {
		cpu_relax();
		goto retry;
	}

	if (value == 0)
		pm8xxx_vib_set_off(vib);
	else {
		value = (value > vib->pdata->max_timeout_ms ?
				 vib->pdata->max_timeout_ms : value);
		pm8xxx_vib_set_on(vib);
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}

static void pm8xxx_vib_update(struct work_struct *work)
{
	struct pm8xxx_vib *vib = container_of(work, struct pm8xxx_vib,
					 work);

	pm8xxx_vib_set_off(vib);
}

static int pm8xxx_vib_get_time(struct timed_output_dev *dev)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart pm8xxx_vib_timer_func(struct hrtimer *timer)
{
	struct pm8xxx_vib *vib = container_of(timer, struct pm8xxx_vib,
							 vib_timer);
	VIB_INFO_LOG("%s \n",__func__);
	pm8xxx_vib_set_off(vib);
	/* schedule_work(&vib->work); */

	return HRTIMER_NORESTART;
}
static ssize_t voltage_level_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct timed_output_dev *time_cdev;
	struct pm8xxx_vib *vib ;
	time_cdev = (struct timed_output_dev *) dev_get_drvdata(dev);
	vib = container_of(time_cdev, struct pm8xxx_vib, timed_dev);
	return sprintf(buf, "voltage input:%dmV\n", vib->level*100);
}

static ssize_t voltage_level_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int voltage_input;
	struct timed_output_dev *time_cdev;
	struct pm8xxx_vib *vib ;
	time_cdev = (struct timed_output_dev *) dev_get_drvdata(dev);
	vib = container_of(time_cdev, struct pm8xxx_vib, timed_dev);

	voltage_input = -1;
	sscanf(buf, "%d ",&voltage_input);
	VIB_INFO_LOG("%s voltage input: %d\n",__func__,voltage_input);
	if (voltage_input < VIB_MIN_LEVEL_mV || voltage_input > VIB_MAX_LEVEL_mV){
		VIB_ERR_LOG("%s invalid voltage level input: %d\n",__func__,voltage_input);
		return -EINVAL;
	}
	vib->level = voltage_input/100;
	return size;
}

static DEVICE_ATTR(voltage_level, S_IRUGO | S_IWUSR, voltage_level_show, voltage_level_store);

#ifdef CONFIG_PM
static int pm8xxx_vib_suspend(struct device *dev)
{
	struct pm8xxx_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	pm8xxx_vib_set_off(vib);

	return 0;
}

static const struct dev_pm_ops pm8xxx_vib_pm_ops = {
	.suspend = pm8xxx_vib_suspend,
};
#endif

static int __devinit pm8xxx_vib_probe(struct platform_device *pdev)

{
	const struct pm8xxx_vibrator_platform_data *pdata =
						pdev->dev.platform_data;
	struct pm8xxx_vib *vib;
	u8 val;
	int rc;

	if (!pdata)
		return -EINVAL;

	if (pdata->level_mV < VIB_MIN_LEVEL_mV ||
			 pdata->level_mV > VIB_MAX_LEVEL_mV)
		return -EINVAL;

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->pdata	= pdata;
	vib->level	= pdata->level_mV / 100;
	vib->dev	= &pdev->dev;

	spin_lock_init(&vib->lock);
	INIT_WORK(&vib->work, pm8xxx_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = pm8xxx_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = pm8xxx_vib_get_time;
	vib->timed_dev.enable = pm8xxx_vib_enable;

	__dump_vib_regs(vib, "boot_vib_default");

	/*
	 * Configure the vibrator, it operates in manual mode
	 * for timed_output framework.
	 */
	rc = pm8xxx_vib_read_u8(vib, &val, VIB_DRV);
	if (rc < 0)
		goto err_read_vib;
	val &= ~VIB_DRV_EN_MANUAL_MASK;
	rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
	if (rc < 0)
		goto err_read_vib;

	vib->reg_vib_drv = val;

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		goto err_read_vib;
	rc = device_create_file(vib->timed_dev.dev, &dev_attr_voltage_level);
	if (rc < 0) {
		VIB_ERR_LOG("%s, create sysfs fail: voltage_level\n", __func__);
	}

	platform_set_drvdata(pdev, vib);

	return 0;

err_read_vib:
	kfree(vib);
	return rc;
}

static int __devexit pm8xxx_vib_remove(struct platform_device *pdev)
{
	struct pm8xxx_vib *vib = platform_get_drvdata(pdev);

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	platform_set_drvdata(pdev, NULL);
	kfree(vib);

	return 0;
}

static struct platform_driver pm8xxx_vib_driver = {
	.probe		= pm8xxx_vib_probe,
	.remove		= __devexit_p(pm8xxx_vib_remove),
	.driver		= {
		.name	= PM8XXX_VIBRATOR_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm8xxx_vib_pm_ops,
#endif
	},
};

static int __init pm8xxx_vib_init(void)
{
	return platform_driver_register(&pm8xxx_vib_driver);
}
module_init(pm8xxx_vib_init);

static void __exit pm8xxx_vib_exit(void)
{
	platform_driver_unregister(&pm8xxx_vib_driver);
}
module_exit(pm8xxx_vib_exit);

MODULE_ALIAS("platform:" PM8XXX_VIBRATOR_DEV_NAME);
MODULE_DESCRIPTION("pm8xxx vibrator driver");
MODULE_LICENSE("GPL v2");
