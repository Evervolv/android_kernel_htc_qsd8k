/* board-htcleo_btn_backlight-manager.c
*
* Driver for managing buttons backlight
*
* Copyright (C) 2010 Danijel Posilović (dan1j3l) <danijel.posilovic@gmail.com>
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


#include <linux/keyboard.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/timer.h>


static int BUTTON_BACKLIGHT_GPIO = 48;
static int OFF_SEC = 10;
static int auto_off_enabled = 1;

struct timer_list btn_off_timer;

static DEFINE_MUTEX(htcleo_btn_manager_lock);

////////////////////////////////////////////////////

// off_sec sysfs
static ssize_t htcleo_manager_offsec_get(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d", OFF_SEC);
	return ret;
}

static ssize_t htcleo_manager_offsec_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int set_state;

	mutex_lock(&htcleo_btn_manager_lock);

	sscanf(buf, "%d", &set_state);

	if (set_state < 5)
        set_state = 5;

    if (set_state > 60)
        set_state=60;

	OFF_SEC = set_state;

	mutex_unlock(&htcleo_btn_manager_lock);

	return count;
}

static DEVICE_ATTR(off_seconds, 0666,  htcleo_manager_offsec_get, htcleo_manager_offsec_set);


//  auto_off sysfs
static ssize_t htcleo_manager_auto_off_get(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d", auto_off_enabled);
	return ret;
}

static ssize_t htcleo_manager_auto_off_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int set_state;

	mutex_lock(&htcleo_btn_manager_lock);

	sscanf(buf, "%d", &set_state);

	if (set_state < 0)
        set_state = 0;

    if (set_state > 1)
        set_state=1;

	auto_off_enabled = set_state;

	mutex_unlock(&htcleo_btn_manager_lock);

	return count;
}

static DEVICE_ATTR(auto_off, 0666,  htcleo_manager_auto_off_get, htcleo_manager_auto_off_set);


static void btn_delayed_off_function(unsigned long function_parameter){
    gpio_set_value(BUTTON_BACKLIGHT_GPIO, 0);
    del_timer(&btn_off_timer);
}

int buttons_notify(struct notifier_block *nblock, unsigned long code, void *_param) {
    struct keyboard_notifier_param *param = _param;
    int keycode;

    if (code == KBD_KEYCODE) {

        keycode = param->value;

        //printk(KERN_DEBUG "BTN-BCKM: KEYLOGGER %i %s\n", param->value, (param->down ? "down" : "up"));

        // Turn backlight on only if pressed = Dial, home, winkey, back, end button
        if (keycode==231 || keycode==102 || keycode==139 || keycode==158 || keycode==107 ){

            gpio_set_value(BUTTON_BACKLIGHT_GPIO, 1);

            // If auto off enabled then buttons will turn off after declared amount of time, else screen backlight will turn them off
            if (auto_off_enabled){
                del_timer(&btn_off_timer);
                init_timer(&btn_off_timer);
                btn_off_timer.expires = jiffies + OFF_SEC*HZ;
                btn_off_timer.function = btn_delayed_off_function;
                add_timer(&btn_off_timer);
            }
        }

    };

    return 0;
}

static struct notifier_block nb = {
  .notifier_call = buttons_notify
};


static int htcleo_btn_backlight_manager_probe(struct platform_device *pdev)
{
    int rc;

    printk("BTN-BCKM: Registering btn manager...\n");
    register_keyboard_notifier(&nb);
    rc = device_create_file(&pdev->dev, &dev_attr_off_seconds);
    rc = device_create_file(&pdev->dev, &dev_attr_auto_off);

    return 0;
}

static int htcleo_btn_backlight_manager_remove(struct platform_device *pdev)
{
    printk("BTN-BCKM: Deactivating btn manager...\n");
    unregister_keyboard_notifier(&nb);
    device_remove_file(&pdev->dev, &dev_attr_off_seconds);
    device_remove_file(&pdev->dev,&dev_attr_auto_off);
    return 0;
}


static struct platform_driver htcleo_btn_backlight_manager = {
    .probe = htcleo_btn_backlight_manager_probe,
    .remove= htcleo_btn_backlight_manager_remove,
	.driver = {
		.name = "btn_backlight_manager",
		.owner = THIS_MODULE
	},
};

static int __init backlight_manager_init(void)
{
    return platform_driver_register(&htcleo_btn_backlight_manager);
}

static void __exit backlight_manager_exit(void)
{
    platform_driver_unregister(&htcleo_btn_backlight_manager);
}

module_init(backlight_manager_init);
module_exit(backlight_manager_exit);

MODULE_AUTHOR("Danijel Posilović (dan1j3l) <danijel.posilovic@gmail.com>");
MODULE_DESCRIPTION("Button Backlight Manager");
MODULE_LICENSE("GPL");
