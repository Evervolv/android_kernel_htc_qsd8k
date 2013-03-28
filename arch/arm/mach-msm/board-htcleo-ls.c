/* board-htcleo-ls.c
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


/*

Light sensor support for HTC LEO
base code copied from microp

because we have not interrupts, we use polling. polling time for now is 1 sec.
user may be able to adjust time in future

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/jiffies.h>
#include <linux/lightsensor.h>
#include <linux/earlysuspend.h>
#include <mach/board-htcleo-microp.h>

#include "board-htcleo.h"
struct early_suspend early_suspend;


//#define CFG_LSENSOR_TYPE 
//#define LSENSOR_ABLK_PLUS_ADC   1
//#define LSENSOR_ABLK_ONLY       2
//#define LSENSOR_ABLK_ONLY       2


#define LSENSOR_POLL_PROMESHUTOK   1000

#define D(x...) pr_debug(x)
// pr_info(x)


static uint16_t lsensor_adc_table[10] = 
{

	0, 5, 20, 70, 150, 240, 330, 425, 515, 590

};


static struct lsensor_data 
{
	struct input_dev *input_dev;
	struct platform_device *dev;
	struct delayed_work work;
	uint32_t old_level;
	int enabled;
	int opened;
} the_data;

int microp_read_lightsensor_value(uint32_t *val);


static DEFINE_MUTEX(api_lock);

// range: adc: 0..1023, value: 0..9
static void map_adc_to_level(uint32_t adc, uint32_t *value)
{
	int i;

	if (adc > 1024)    
	{
		pr_err("%s: adc > 1024\n", __func__);
		*value = 5; // set some good value at error
		return;
	}

	for (i = 9; i >= 0; i--)
	{
		if (adc >= lsensor_adc_table[i])
		{
			D("%s: %d -> %d\n", __func__, adc, i);
			*value = i;
			return;
		}
	}      
	// should not happen
	*value = 5;  // set some good value at error
}

int lightsensor_read_value(uint32_t *val)
{
	int ret;
	uint8_t data[2];

	if (!val)
		return -EIO;

	ret = microp_i2c_read(MICROP_I2C_RCMD_LSENSOR, data, 2);
	if (ret < 0) 
	{
		pr_err("%s: read ls fail\n", __func__);
		return -EIO;
	}

	*val = data[1] | (data[0] << 8);
	D("lsensor adc = %d\n", *val);
	return 0;
}

static void lightsensor_poll_function(struct work_struct *work)
{
	struct lsensor_data* p = &the_data;
	uint32_t adc = 0, level = 0;

	D("%s\n", __func__);
	if (!p->enabled)
	{
		D("  disable\n");
		return;
	}
      
	lightsensor_read_value(&adc);
	
	
	map_adc_to_level(adc, &level);

	//if (level != the_data.old_level)
	{
		input_report_abs(the_data.input_dev, ABS_MISC, (int)level);
		input_sync(the_data.input_dev);
		the_data.old_level = level;
	}
	schedule_delayed_work(&the_data.work, msecs_to_jiffies(LSENSOR_POLL_PROMESHUTOK));
}


static int lightsensor_enable(void)
{
	struct lsensor_data* p = &the_data;
	int rc = -EIO;

	D("%s\n", __func__);

	if (p->enabled)
	{
		pr_err("lsensor: already enabled\n");
		return 0;
	}
	rc = capella_cm3602_power(LS_PWR_ON, 1);
	if (rc < 0)
		return -EIO;
	
	the_data.old_level = -1;

	p->enabled = 1;
	schedule_delayed_work(&the_data.work, msecs_to_jiffies(LSENSOR_POLL_PROMESHUTOK));

	return 0;
}

static int lightsensor_disable(void)
{
	struct lsensor_data* p = &the_data;
	int rc = -EIO;

	D("%s\n", __func__);

	if (!p->enabled)
	{
		pr_err("lsensor: nothing to disable\n");
		return 0;
	}
	rc = capella_cm3602_power(LS_PWR_ON, 0);
	if (rc < 0)
		return -EIO;

	p->enabled = 0;
	cancel_delayed_work_sync(&the_data.work);
	return 0;
}


static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	D("%s\n", __func__);

	mutex_lock(&api_lock);
	if (the_data.opened) 
	{
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	the_data.opened = 1;
	mutex_unlock(&api_lock);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	D("%s\n", __func__);

	mutex_lock(&api_lock);
	the_data.opened = 0;
	mutex_unlock(&api_lock);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc, val;
	struct lsensor_data* p = &the_data;

	mutex_lock(&api_lock);

	D("%s cmd %d\n", __func__, _IOC_NR(cmd));
	if (!the_data.opened)
	{
		return -EIO;
	}

	switch (cmd) 
	{
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) 
		{
			rc = -EFAULT;
			break;
		}
		D("%s ls set to: %d\n", __func__, val);
		rc = val ? lightsensor_enable() : lightsensor_disable();
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = p->enabled;
		D("%s enabled %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	mutex_unlock(&api_lock);
	return rc;
}

static struct file_operations lightsensor_fops = 
{
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

struct miscdevice lightsensor_misc = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lightsensor_suspend(struct early_suspend *h)
{
	lightsensor_disable();
}

static void lightsensor_resume(struct early_suspend *h)
{
	lightsensor_enable();
}
#endif

//////////////////////////////////////////////////////////////////////////

static int lsensor_probe(struct platform_device *pdev)
{       
	int ret = -EIO;    
	struct input_dev *input_dev;

	D("%s: probe\n", __func__);

	platform_set_drvdata(pdev, &the_data);
	the_data.dev = pdev;

	/* Light Sensor */
	/*
	ret = device_create_file(&the_data->dev, &dev_attr_ls_adc);
	ret = device_create_file(&pdev->dev, &dev_attr_ls_auto);
	*/
	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto done;
	}
	the_data.input_dev = input_dev;
	input_set_drvdata(input_dev, &the_data);

	input_dev->name = "lightsensor-level";

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, 0, 9, 0, 0);

	D("%s: registering input device\n", __func__);

	ret = input_register_device(input_dev);
	if (ret < 0) 
	{
		pr_err("%s: can not register input device\n", __func__);
		goto err_free_input_device;
	}

	D("%s: registering misc device\n", __func__);

	ret = misc_register(&lightsensor_misc);
	if (ret < 0)
	{
		pr_err("%s: can not register misc device\n", __func__);
		goto err_unregister_input_device;
	}

	the_data.old_level = -1;
	the_data.enabled=0;
	the_data.opened=0;
	INIT_DELAYED_WORK(&the_data.work, lightsensor_poll_function);
	lightsensor_enable();

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = lightsensor_suspend;
	early_suspend.resume = lightsensor_resume;
	register_early_suspend(&early_suspend);
#endif

	if (!ret)
		goto done;

	misc_deregister(&lightsensor_misc);
err_unregister_input_device:
	input_unregister_device(input_dev);
	goto done;
err_free_input_device:
	input_free_device(input_dev);
done:
	return ret;

	//	device_remove_file(&client->dev, &dev_attr_ls_adc);
	//	device_remove_file(&pdev->dev, &dev_attr_ls_auto);

}

static struct platform_driver lsensor_driver = 
{
    .probe = lsensor_probe,
    .driver = 
    {
        .name = "htcleo-lsensor",
        .owner = THIS_MODULE
    },
};

static int __init lsensor_init(void)
{
    return platform_driver_register(&lsensor_driver);
}

//device_initcall(lsensor_init);
late_initcall(lsensor_init);

MODULE_AUTHOR("Cotulla");
MODULE_DESCRIPTION("LEO LSensor driver");
MODULE_LICENSE("GPL");
// END OF FILE
