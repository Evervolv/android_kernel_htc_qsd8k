/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/smsc911x.h>
#include <linux/mfd/pm8058.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/msm_ssbi.h>
#include <mach/msm_hsusb.h>

#include <mach/vreg.h>
#include "devices.h"
#include "proc_comm.h"
#include "clock-pcom.h"
#include "gpiomux.h"

#define MSM7X30_PM8058_GPIO_BASE	FIRST_BOARD_GPIO
#define MSM7X30_PM8058_GPIO(x)		(MSM7X30_PM8058_GPIO_BASE + (x))
#define MSM7X30_PM8058_IRQ_BASE		FIRST_BOARD_IRQ

#define MSM7X30_GPIO_PMIC_INT_N		27

extern struct sys_timer msm_timer;

static int msm7x30_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};

static void msm7x30_usb_phy_reset(void)
{
	u32 id;
	int ret;

	id = P_USB_PHY_CLK;
	ret = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_ASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}

	msleep(1);

	id = P_USB_PHY_CLK;
	ret = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_DEASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}

static void msm7x30_usb_hw_reset(bool enable)
{
	u32 id;
	int ret;
	u32 func;

	id = P_USB_HS_CLK;
	if (enable)
		func = PCOM_CLKCTL_RPC_RESET_ASSERT;
	else
		func = PCOM_CLKCTL_RPC_RESET_DEASSERT;
	ret = msm_proc_comm(func, &id, NULL);
	if (ret)
		pr_err("%s: Cannot set reset to %d (%d)\n", __func__, enable,
		       ret);
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq	= msm7x30_phy_init_seq,
	.phy_reset	= msm7x30_usb_phy_reset,
	.hw_reset	= msm7x30_usb_hw_reset,
};

static char *usb_functions[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"usb_mass_storage",
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x4e11,
		.num_functions	= ARRAY_SIZE(usb_functions),
		.functions	= usb_functions,
	},
	{
		.product_id	= 0x4e12,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x18d1,
	.product_id	= 0x4e11,
	.version	= 0x0100,
	.product_name		= "Surf7x30",
	.manufacturer_name	= "Qualcomm, Inc.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm, Inc.",
	.product	= "Surf7x30",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) && !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
        &msm_device_uart2,
#endif
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c2,
	&msm_device_hsusb,
	&usb_mass_storage_device,
	&android_usb_device,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct pm8058_pin_config msm7x30_kpd_input_gpio_cfg = {
	.vin_src	= PM8058_GPIO_VIN_SRC_VREG_S3,
	.dir		= PM8058_GPIO_INPUT,
	.pull_up	= PM8058_GPIO_PULL_UP_31P5,
	.strength	= PM8058_GPIO_STRENGTH_OFF,
	.func		= PM8058_GPIO_FUNC_NORMAL,
	.flags		= PM8058_GPIO_INV_IRQ_POL
};

static struct pm8058_pin_config msm7x30_kpd_output_gpio_cfg = {
	.vin_src	= PM8058_GPIO_VIN_SRC_VREG_S3,
	.dir		= PM8058_GPIO_OUTPUT,
	.pull_up	= PM8058_GPIO_PULL_NONE,
	.strength	= PM8058_GPIO_STRENGTH_LOW,
	.func		= PM8058_GPIO_FUNC_1,
	.flags		= (PM8058_GPIO_OPEN_DRAIN |
			   PM8058_GPIO_INV_IRQ_POL),
};

static unsigned int msm7x30_pmic_col_gpios[] = {
	MSM7X30_PM8058_GPIO(0), MSM7X30_PM8058_GPIO(1),
	MSM7X30_PM8058_GPIO(2), MSM7X30_PM8058_GPIO(3),
	MSM7X30_PM8058_GPIO(4), MSM7X30_PM8058_GPIO(5),
	MSM7X30_PM8058_GPIO(6), MSM7X30_PM8058_GPIO(7),
};
static unsigned int msm7x30_pmic_row_gpios[] = {
	MSM7X30_PM8058_GPIO(8), MSM7X30_PM8058_GPIO(9),
	MSM7X30_PM8058_GPIO(10), MSM7X30_PM8058_GPIO(11),
	MSM7X30_PM8058_GPIO(12), MSM7X30_PM8058_GPIO(13),
	MSM7X30_PM8058_GPIO(14), MSM7X30_PM8058_GPIO(15),
	MSM7X30_PM8058_GPIO(16), MSM7X30_PM8058_GPIO(17),
	MSM7X30_PM8058_GPIO(18), MSM7X30_PM8058_GPIO(19),
};

#define KEYMAP_NUM_ROWS		ARRAY_SIZE(msm7x30_pmic_row_gpios)
#define KEYMAP_NUM_COLS		ARRAY_SIZE(msm7x30_pmic_col_gpios)
#define KEYMAP_INDEX(row, col)	(((row) * KEYMAP_NUM_COLS) + (col))
#define KEYMAP_SIZE		(KEYMAP_NUM_ROWS * KEYMAP_NUM_COLS)

static int msm7x30_pmic_keypad_init(struct device *dev)
{
	int i;

	for (i = 0; i < KEYMAP_NUM_COLS; ++i)
		pm8058_gpio_mux(msm7x30_pmic_col_gpios[i],
				&msm7x30_kpd_input_gpio_cfg);
	for (i = 0; i < KEYMAP_NUM_ROWS; ++i)
		pm8058_gpio_mux(msm7x30_pmic_row_gpios[i],
				&msm7x30_kpd_output_gpio_cfg);
	return 0;
}

static const unsigned short msm7x30_pmic_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 6)] = KEY_BACK,
};

static struct pm8058_keypad_platform_data msm7x30_pmic_keypad_pdata = {
	.name			= "msm7x30-keypad",
	.num_drv		= KEYMAP_NUM_ROWS,
	.num_sns		= KEYMAP_NUM_COLS,
	.scan_delay_shift	= 5,
	.drv_hold_clks		= 4,
	.debounce_ms		= 10,
	.keymap			= msm7x30_pmic_keymap,
	.init			= msm7x30_pmic_keypad_init,
};

static struct pm8058_platform_data msm7x30_pm8058_pdata = {
	.irq_base	= MSM7X30_PM8058_IRQ_BASE,
	.gpio_base	= MSM7X30_PM8058_GPIO_BASE,
	.keypad_pdata	= &msm7x30_pmic_keypad_pdata,
};

static struct msm_ssbi_platform_data msm7x30_ssbi_pmic_pdata = {
	.slave		= {
		.name		= "pm8058-core",
		.irq		= MSM_GPIO_TO_INT(MSM7X30_GPIO_PMIC_INT_N),
		.platform_data	= &msm7x30_pm8058_pdata,
	},
	.rspinlock_name	= "D:PMIC_SSBI",
};

static int __init msm7x30_ssbi_pmic_init(void)
{
	int ret;

	pr_info("%s()\n", __func__);
	msm_gpiomux_write(MSM7X30_GPIO_PMIC_INT_N, 0,
			  GPIOMUX_FUNC_GPIO |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);
	ret = gpiochip_reserve(msm7x30_pm8058_pdata.gpio_base,
			       PM8058_NUM_GPIOS);
	WARN(ret, "can't reserve pm8058 gpios. badness will ensue...\n");
	msm_device_ssbi_pmic.dev.platform_data = &msm7x30_ssbi_pmic_pdata;
	return platform_device_register(&msm_device_ssbi_pmic);
}

static struct i2c_board_info surf_i2c_devices[] = {
	/* marimba master is implied at 0x0c */
	{
		I2C_BOARD_INFO("marimba-codec",	0x77),
	},
};

extern void msm_serial_debug_init(unsigned int base, int irq,
				  struct device *clk_device, int signal_irq);

static void __init msm7x30_init(void)
{
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1);
#endif

	msm7x30_ssbi_pmic_init();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(1, surf_i2c_devices,
				ARRAY_SIZE(surf_i2c_devices));

	msm_hsusb_set_vbus_state(1);
	msm_hsusb_set_vbus_state(0);
	msm_hsusb_set_vbus_state(1);
}

static void __init msm7x30_map_io(void)
{
	msm_map_msm7x30_io();
	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
}

static void __init msm7x30_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = (51*1024*1024);
}

MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = msm7x30_fixup,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_FFA, "QCT MSM7X30 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_FLUID, "QCT MSM7X30 FLUID")
#ifdef CONFIG_MSM_DEBUG_UART
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
