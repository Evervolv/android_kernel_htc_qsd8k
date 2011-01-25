/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
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

/* Control bluetooth power for supersonic platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include "proc_comm.h"
#include "board-supersonic.h"

#define HTC_RFKILL_DBG

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";
static int pre_state;

static uint32_t supersonic_bt_init_table[] = {
	/* BT_RTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RTS,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),
	/* BT_CTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_CTS,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_RX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RX,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_TX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_TX,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),

	/* BT_SHUTDOWN_N */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
	/* BT_RESET_N */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_RESET_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),

	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_HOST_WAKE,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_4MA),
	/* BT_CHIP_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
};

static uint32_t supersonic_bt_on_table[] = {
	/* BT_RTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RTS,
				2,
				GPIO_OUTPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_CTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_CTS,
				2,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_RX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RX,
				2,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_TX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_TX,
				2,
				GPIO_OUTPUT,
				GPIO_PULL_UP,
				GPIO_8MA),

	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_HOST_WAKE,
				0,
				GPIO_INPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
	/* BT_CHIP_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
};

static uint32_t supersonic_bt_off_table[] = {
	/* BT_RTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RTS,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),
	/* BT_CTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_CTS,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_RX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RX,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	/* BT_TX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_TX,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),

	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_HOST_WAKE,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_4MA),
	/* BT_CHIP_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void supersonic_config_bt_init(void)
{
	/* set bt initial configuration*/
	config_bt_table(supersonic_bt_init_table,
				ARRAY_SIZE(supersonic_bt_init_table));
	/* BT_RESET_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_RESET_N, 0);

	mdelay(5);

	/* BT_SHUTDOWN_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_SHUTDOWN_N, 1);

	/* BT_RESET_N */

        gpio_direction_output(SUPERSONIC_GPIO_BT_RESET_N, 1);

	mdelay(15);

	/* BT_RESET_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_RESET_N, 0);

	/* BT_SHUTDOWN_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_SHUTDOWN_N, 0);

	/* BT_RTS */
        gpio_direction_output(SUPERSONIC_GPIO_BT_UART1_RTS, 0);

	/* BT_TX */
        gpio_direction_output(SUPERSONIC_GPIO_BT_UART1_TX, 0);

	/* BT_CHIP_WAKE */
        gpio_direction_output(SUPERSONIC_GPIO_BT_CHIP_WAKE, 0);

}

static void supersonic_config_bt_on(void)
{

	#ifdef HTC_RFKILL_DBG
	printk(KERN_INFO "-- RK ON --\n");
	#endif

	/* set bt on configuration*/
	config_bt_table(supersonic_bt_on_table,
				ARRAY_SIZE(supersonic_bt_on_table));
	mdelay(5);
	/* BT_SHUTDOWN_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_SHUTDOWN_N, 1);

	/* BT_RESET_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_RESET_N, 1);

	mdelay(5);

	/* BT_CHIP_WAKE */
        gpio_direction_output(SUPERSONIC_GPIO_BT_CHIP_WAKE, 1);

}

static void supersonic_config_bt_off(void)
{
	#ifdef HTC_RFKILL_DBG
	printk(KERN_INFO "-- RK OFF --\n");
	#endif

	/* BT_RESET_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_RESET_N, 0);

	/* BT_SHUTDOWN_N */
        gpio_direction_output(SUPERSONIC_GPIO_BT_SHUTDOWN_N, 0);


	config_bt_table(supersonic_bt_off_table,
				ARRAY_SIZE(supersonic_bt_off_table));
	mdelay(5);

	/* BT_RTS */
        gpio_direction_output(SUPERSONIC_GPIO_BT_UART1_RTS, 0);

	/* BT_TX */
        gpio_direction_output(SUPERSONIC_GPIO_BT_UART1_TX, 0);

	/* BT_CHIP_WAKE */
        gpio_direction_output(SUPERSONIC_GPIO_BT_CHIP_WAKE, 0);
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (pre_state == blocked) {
		#ifdef HTC_RFKILL_DBG
		printk(KERN_INFO "-- SAME ST --\n");
		#endif
		return 0;
	} else
		pre_state = blocked;

	if (!blocked)
		supersonic_config_bt_on();
	else
		supersonic_config_bt_off();

	return 0;
}

static struct rfkill_ops supersonic_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int supersonic_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true; /* off */

	supersonic_config_bt_init();	/* bt gpio initial config */

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
						 &supersonic_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_reset;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_reset:
	return rc;
}

static int supersonic_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver supersonic_rfkill_driver = {
	.probe = supersonic_rfkill_probe,
	.remove = supersonic_rfkill_remove,
	.driver = {
		.name = "supersonic_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init supersonic_rfkill_init(void)
{
	pre_state = -1;
	if (!machine_is_supersonic())
		return 0;

	return platform_driver_register(&supersonic_rfkill_driver);
}

static void __exit supersonic_rfkill_exit(void)
{
	platform_driver_unregister(&supersonic_rfkill_driver);
}

module_init(supersonic_rfkill_init);
module_exit(supersonic_rfkill_exit);
MODULE_DESCRIPTION("supersonic rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
