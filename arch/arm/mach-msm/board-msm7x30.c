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
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/smsc911x.h>
#include <linux/mfd/pm8058.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/msm_ssbi.h>

#include <mach/vreg.h>
#include "devices.h"
#include "proc_comm.h"
#include "gpiomux.h"

#define MSM7X30_PM8058_GPIO_BASE	FIRST_BOARD_GPIO
#define MSM7X30_PM8058_GPIO(x)		(MSM7X30_PM8058_GPIO_BASE + (x))
#define MSM7X30_PM8058_IRQ_BASE		FIRST_BOARD_IRQ

#define MSM7X30_GPIO_PMIC_INT_N		27

extern struct sys_timer msm_timer;

static int hsusb_phy_init_seq[] = {
	0x30, 0x32,	/* Enable and set Pre-Emphasis Depth to 20% */
	0x02, 0x36,	/* Disable CDR Auto Reset feature */
	-1
};

static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_init_seq		= hsusb_phy_init_seq,
	.mode                   = USB_PERIPHERAL,
	.otg_control		= OTG_PHY_CONTROL,
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
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct pm8058_platform_data msm7x30_pm8058_pdata = {
	.irq_base	= MSM7X30_PM8058_IRQ_BASE,
	.gpio_base	= MSM7X30_PM8058_GPIO_BASE,
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
	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(1, surf_i2c_devices,
				ARRAY_SIZE(surf_i2c_devices));
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
