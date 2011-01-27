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
#include <linux/clk.h>
#include <linux/android_pmem.h>
#include <linux/memblock.h>
#include <linux/wakelock.h>
#include <linux/cyttsp.h>

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
#include <mach/msm_spi.h>

#include <mach/vreg.h>
#include "devices.h"
#include "proc_comm.h"
#include "clock-pcom.h"
#include "gpiomux.h"
#include "board-msm7x30.h"

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
	.usb_connected	= pm8058_notify_charger_connected,
};

static struct wake_lock vbus_wake_lock;

static void msm7x30_vbus_present(bool present)
{
	pr_info("usb_cable_status: %s\n", present ? "inserted" : "removed");
	if (present)
		wake_lock(&vbus_wake_lock);
	msm_hsusb_set_vbus_state(present);
	if (!present)
		wake_unlock(&vbus_wake_lock);
}

static int msm7x30_pcom_charge(u32 max_current, bool is_ac)
{
	u32 status = 0;
	u32 pc_ids[] = {
		PCOM_CHG_USB_IS_PC_CONNECTED,
		PCOM_CHG_USB_IS_CHARGER_CONNECTED,
	};
	int ret = 0;

	pr_info("%s(%u,%d)\n", __func__, max_current, is_ac);

	if (max_current) {
		/* enable charging */
		status = 0;
		msm_proc_comm(pc_ids[!!is_ac], &status, 0);
		if (!status) {
			pr_err("%s: can't set chg type (ac=%d)\n", __func__,
			       is_ac);
			ret = -EINVAL;
			goto err;
		}
		msm_proc_comm(PCOM_CHG_USB_IS_AVAILABLE, &max_current, &status);
		if (!status) {
			pr_err("%s: set_i failed %u\n", __func__, max_current);
			ret = -EINVAL;
			goto err;
		}
	} else {
		msm_proc_comm(PCOM_CHG_USB_IS_AVAILABLE, &max_current, &status);
		if (!status) {
			pr_err("%s: set_i failed %u\n", __func__, max_current);
			ret = -EINVAL;
			goto err;
		}
		msm_proc_comm(PCOM_CHG_USB_IS_DISCONNECTED, &status, 0);
		if (!status) {
			pr_err("%s: can't set disconnect\n", __func__);
			ret = -EINVAL;
			goto err;
		}
	}

err:
	return ret;
}

static struct pm8058_charger_platform_data msm7x30_pmic_charger_pdata = {
	.vbus_present           = msm7x30_vbus_present,
	.charge                 = msm7x30_pcom_charge,
	.supplied_to            = NULL,
	.num_supplicants        = 0,
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

static struct android_pmem_platform_data pmem_pdata = {
	.name		= "pmem",
	.size		= MSM7X30_SURF_PMEM_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.size		= MSM7X30_SURF_PMEM_ADSP_SIZE,
	.no_allocator	= 0,
	.cached		= 0,
};

static struct platform_device android_pmem_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &pmem_adsp_pdata,
	},
};

static struct resource msm_kgsl_resources[] = {
	{
		.name	= "kgsl_phys_memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_kgsl_device = {
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

#define PWR_RAIL_GRP_CLK		8
static int msm7x30_kgsl_power_rail_mode(int follow_clk)
{
	int mode = follow_clk ? 0 : 1;
	int rail_id = PWR_RAIL_GRP_CLK;

	return msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
}

static int msm7x30_kgsl_power(bool on)
{
	int cmd;
	int id;

	if (on) {
		/* turn clock on, turn power on, turn clock off */
		cmd = PCOM_CLKCTL_RPC_RAIL_ENABLE;
		id = P_GRP_3D_CLK;
		msm_proc_comm(cmd, &id, NULL);

		cmd = PCOM_CLKCTL_RPC_ENABLE;
		id = PWR_RAIL_GRP_CLK;
		msm_proc_comm(cmd, &id, NULL);

		cmd = PCOM_CLKCTL_RPC_RAIL_DISABLE;
		id = P_GRP_3D_CLK;
		msm_proc_comm(cmd, &id, NULL);
	} else {
		/* turn clock on, turn power off, turn clock off */
		cmd = PCOM_CLKCTL_RPC_RAIL_ENABLE;
		id = P_GRP_3D_CLK;
		msm_proc_comm(cmd, &id, NULL);

		cmd = PCOM_CLKCTL_RPC_DISABLE;
		id = PWR_RAIL_GRP_CLK;
		msm_proc_comm(cmd, &id, NULL);

		cmd = PCOM_CLKCTL_RPC_RAIL_DISABLE;
		id = P_GRP_3D_CLK;
		msm_proc_comm(cmd, &id, NULL);
	}

	return 0;
}

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) && !defined(CONFIG_MSM_SERIAL_DEBUGGER)
        &msm_device_uart2,
#endif
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_i2c2,
	&msm_device_hsusb,
	&usb_mass_storage_device,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&msm_kgsl_device,
	&msm_device_spi,
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

static const unsigned short msm7x30_fluid_pmic_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_7,
	[KEYMAP_INDEX(0, 1)] = KEY_ENTER,
	[KEYMAP_INDEX(0, 2)] = KEY_UP,
	[KEYMAP_INDEX(0, 4)] = KEY_DOWN,

	[KEYMAP_INDEX(1, 0)] = KEY_POWER,
	[KEYMAP_INDEX(1, 1)] = KEY_SELECT,
	[KEYMAP_INDEX(1, 2)] = KEY_1,
	[KEYMAP_INDEX(1, 3)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(1, 4)] = KEY_VOLUMEDOWN,
};

static const unsigned short msm7x30_surf_pmic_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_7,
	[KEYMAP_INDEX(0, 1)] = KEY_DOWN,
	[KEYMAP_INDEX(0, 2)] = KEY_UP,
	[KEYMAP_INDEX(0, 3)] = KEY_RIGHT,
	[KEYMAP_INDEX(0, 4)] = KEY_ENTER,
	[KEYMAP_INDEX(0, 5)] = KEY_L,
	[KEYMAP_INDEX(0, 6)] = KEY_BACK,
	[KEYMAP_INDEX(0, 7)] = KEY_M,

	[KEYMAP_INDEX(1, 0)] = KEY_LEFT,
	[KEYMAP_INDEX(1, 1)] = KEY_SEND,
	[KEYMAP_INDEX(1, 2)] = KEY_1,
	[KEYMAP_INDEX(1, 3)] = KEY_4,
	[KEYMAP_INDEX(1, 4)] = KEY_CLEAR,
	[KEYMAP_INDEX(1, 5)] = KEY_MSDOS,
	[KEYMAP_INDEX(1, 6)] = KEY_SPACE,
	[KEYMAP_INDEX(1, 7)] = KEY_COMMA,

	[KEYMAP_INDEX(2, 0)] = KEY_6,
	[KEYMAP_INDEX(2, 1)] = KEY_5,
	[KEYMAP_INDEX(2, 2)] = KEY_8,
	[KEYMAP_INDEX(2, 3)] = KEY_3,
	[KEYMAP_INDEX(2, 4)] = KEY_NUMERIC_STAR,
	[KEYMAP_INDEX(2, 5)] = KEY_UP,
	[KEYMAP_INDEX(2, 6)] = KEY_DOWN,
	[KEYMAP_INDEX(2, 7)] = KEY_LEFTSHIFT,

	[KEYMAP_INDEX(3, 0)] = KEY_9,
	[KEYMAP_INDEX(3, 1)] = KEY_NUMERIC_POUND,
	[KEYMAP_INDEX(3, 2)] = KEY_0,
	[KEYMAP_INDEX(3, 3)] = KEY_2,
	[KEYMAP_INDEX(3, 4)] = KEY_SLEEP,
	[KEYMAP_INDEX(3, 5)] = KEY_F1,
	[KEYMAP_INDEX(3, 6)] = KEY_F2,
	[KEYMAP_INDEX(3, 7)] = KEY_F3,

	[KEYMAP_INDEX(4, 0)] = KEY_BACK,
	[KEYMAP_INDEX(4, 1)] = KEY_HOME,
	[KEYMAP_INDEX(4, 2)] = KEY_MENU,
	[KEYMAP_INDEX(4, 3)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(4, 4)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(4, 5)] = KEY_F4,
	[KEYMAP_INDEX(4, 6)] = KEY_F5,
	[KEYMAP_INDEX(4, 7)] = KEY_F6,

	[KEYMAP_INDEX(5, 0)] = KEY_R,
	[KEYMAP_INDEX(5, 1)] = KEY_T,
	[KEYMAP_INDEX(5, 2)] = KEY_Y,
	[KEYMAP_INDEX(5, 3)] = KEY_LEFTALT,
	[KEYMAP_INDEX(5, 4)] = KEY_KPENTER,
	[KEYMAP_INDEX(5, 5)] = KEY_Q,
	[KEYMAP_INDEX(5, 6)] = KEY_W,
	[KEYMAP_INDEX(5, 7)] = KEY_E,

	[KEYMAP_INDEX(6, 0)] = KEY_F,
	[KEYMAP_INDEX(6, 1)] = KEY_G,
	[KEYMAP_INDEX(6, 2)] = KEY_H,
	[KEYMAP_INDEX(6, 3)] = KEY_CAPSLOCK,
	[KEYMAP_INDEX(6, 4)] = KEY_PAGEUP,
	[KEYMAP_INDEX(6, 5)] = KEY_A,
	[KEYMAP_INDEX(6, 6)] = KEY_S,
	[KEYMAP_INDEX(6, 7)] = KEY_D,

	[KEYMAP_INDEX(7, 0)] = KEY_V,
	[KEYMAP_INDEX(7, 1)] = KEY_B,
	[KEYMAP_INDEX(7, 2)] = KEY_N,
	[KEYMAP_INDEX(7, 3)] = KEY_MENU,
	[KEYMAP_INDEX(7, 4)] = KEY_PAGEDOWN,
	[KEYMAP_INDEX(7, 5)] = KEY_Z,
	[KEYMAP_INDEX(7, 6)] = KEY_X,
	[KEYMAP_INDEX(7, 7)] = KEY_C,

	[KEYMAP_INDEX(8, 0)] = KEY_P,
	[KEYMAP_INDEX(8, 1)] = KEY_J,
	[KEYMAP_INDEX(8, 2)] = KEY_K,
	[KEYMAP_INDEX(8, 3)] = KEY_INSERT,
	[KEYMAP_INDEX(8, 4)] = KEY_LINEFEED,
	[KEYMAP_INDEX(8, 5)] = KEY_U,
	[KEYMAP_INDEX(8, 6)] = KEY_I,
	[KEYMAP_INDEX(8, 7)] = KEY_O,

	[KEYMAP_INDEX(9, 0)] = KEY_4,
	[KEYMAP_INDEX(9, 1)] = KEY_5,
	[KEYMAP_INDEX(9, 2)] = KEY_6,
	[KEYMAP_INDEX(9, 3)] = KEY_7,
	[KEYMAP_INDEX(9, 4)] = KEY_8,
	[KEYMAP_INDEX(9, 5)] = KEY_1,
	[KEYMAP_INDEX(9, 6)] = KEY_2,
	[KEYMAP_INDEX(9, 7)] = KEY_3,

	[KEYMAP_INDEX(10, 0)] = KEY_F7,
	[KEYMAP_INDEX(10, 1)] = KEY_F8,
	[KEYMAP_INDEX(10, 2)] = KEY_F9,
	[KEYMAP_INDEX(10, 3)] = KEY_F10,
	[KEYMAP_INDEX(10, 4)] = KEY_FN,
	[KEYMAP_INDEX(10, 5)] = KEY_9,
	[KEYMAP_INDEX(10, 6)] = KEY_0,
	[KEYMAP_INDEX(10, 7)] = KEY_DOT,

	[KEYMAP_INDEX(11, 0)] = KEY_LEFTCTRL,
	[KEYMAP_INDEX(11, 1)] = KEY_F11,
	[KEYMAP_INDEX(11, 2)] = KEY_ENTER,
	[KEYMAP_INDEX(11, 3)] = KEY_SEARCH,
	[KEYMAP_INDEX(11, 4)] = KEY_DELETE,
	[KEYMAP_INDEX(11, 5)] = KEY_RIGHT,
	[KEYMAP_INDEX(11, 6)] = KEY_LEFT,
	[KEYMAP_INDEX(11, 7)] = KEY_RIGHTSHIFT,
};

static struct pm8058_keypad_platform_data msm7x30_pmic_keypad_pdata = {
	.name			= "msm7x30-keypad",
	.num_drv		= KEYMAP_NUM_ROWS,
	.num_sns		= KEYMAP_NUM_COLS,
	.scan_delay_shift	= 5,
	.drv_hold_clks		= 4,
	.debounce_ms		= 10,
	.init			= msm7x30_pmic_keypad_init,
};

static struct pm8058_platform_data msm7x30_pm8058_pdata = {
	.irq_base	= MSM7X30_PM8058_IRQ_BASE,
	.gpio_base	= MSM7X30_PM8058_GPIO_BASE,
	.keypad_pdata	= &msm7x30_pmic_keypad_pdata,
	.charger_pdata	= &msm7x30_pmic_charger_pdata,
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

static struct vreg *vreg_l15; /* gp6 */
static struct vreg *vreg_l8;  /* gp7 */
static struct vreg *vreg_l16; /* gp10 */

static struct vreg *_get_vreg(char *name, u32 mv, bool en)
{
	struct vreg *vr;

	vr = vreg_get(NULL, name);
	if (IS_ERR(vr))
		return NULL;
	if (mv != 0)
		vreg_set_level(vr, mv);
	if (en)
		vreg_enable(vr);
	return vr;
}

static void _put_vreg(struct vreg *vr)
{
	if (vr) {
		vreg_disable(vr);
		vreg_put(vr);
	}
}

static void fluid_cyttsp_init(void)
{
	vreg_l8 = _get_vreg("gp7", 1800, true);
	vreg_l16 = _get_vreg("gp10", 2600, true);
	vreg_l15 = _get_vreg("gp6", 3050, true);

	if (!vreg_l8 || !vreg_l16 || !vreg_l15) {
		pr_err("%s: can't get vregs\n", __func__);
		_put_vreg(vreg_l8);
		_put_vreg(vreg_l15);
		_put_vreg(vreg_l16);
		return;
	}

	/* enable interrupt gpio */
	msm_gpiomux_write(MSM7X30_FLUID_GPIO_TOUCH_INT_N, 0,
			  GPIOMUX_FUNC_GPIO |
			  GPIOMUX_PULL_UP |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_6MA | GPIOMUX_VALID);
}

static int fluid_cyttsp_resume(struct i2c_client *client)
{
	mdelay(10);
	return CY_OK;
}

static struct cyttsp_platform_data fluid_cyttsp_pdata = {
	.panel_maxx = 479,
	.panel_maxy = 799,
	.disp_maxx = 469,
	.disp_maxy = 799,
	.disp_minx = 10,
	.disp_miny = 0,
	.flags = 0,
	.gen = CY_GEN3,	/* or */
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
				CY_GEST_GRP3 | CY_GEST_GRP4 |
				CY_ACT_DIST,
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.resume = fluid_cyttsp_resume,
	.init = NULL,
};

static struct i2c_board_info fluid_i2c_0_board_info[] = {
	{
		I2C_BOARD_INFO("cyttsp-i2c", 0x24),
		.platform_data = &fluid_cyttsp_pdata,
		.irq = MSM_GPIO_TO_INT(MSM7X30_FLUID_GPIO_TOUCH_INT_N),
	}
};

static struct i2c_board_info surf_i2c_devices[] = {
	/* marimba master is implied at 0x0c */
	{
		I2C_BOARD_INFO("marimba-codec",	0x77),
	},
};

static int msm7x30_i2c_0_init(void)
{
	msm_gpiomux_write(70, 0,
			  GPIOMUX_FUNC_1 |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_16MA | GPIOMUX_VALID);
	msm_gpiomux_write(71, 0,
			  GPIOMUX_FUNC_1 |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_16MA | GPIOMUX_VALID);
}

static int msm7x30_spi_init(void)
{
	msm_gpiomux_write(45, 0,
			  GPIOMUX_FUNC_1 |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);
	msm_gpiomux_write(46, 0,
			  GPIOMUX_FUNC_1 |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);
	msm_gpiomux_write(47, 0,
			  GPIOMUX_FUNC_1 |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);
	msm_gpiomux_write(48, 0,
			  GPIOMUX_FUNC_1 |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_INPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);
}

static struct msm_spi_platform_data msm7x30_spi_pdata = {
	.max_clock_speed = 26331429,
};

static ssize_t fluid_virtual_keys_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		       __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":50:842:80:100"
		       ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":170:842:80:100"
		       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":290:842:80:100"
		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":410:842:80:100"
	"\n");
}

static struct kobj_attribute fluid_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp-i2c",
		.mode = S_IRUGO,
	},
	.show = &fluid_virtual_keys_show,
};

static struct attribute *fluid_properties_attrs[] = {
	&fluid_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group fluid_properties_attr_group = {
	.attrs = fluid_properties_attrs,
};

static int fluid_board_props_init(void)
{
	int rc;
	struct kobject *properties_kobj;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (!properties_kobj) {
		rc = -ENOMEM;
		goto err_kobj_create;
	}

	rc = sysfs_create_group(properties_kobj, &fluid_properties_attr_group);
	if (rc)
		goto err_sysfs_create;

	return 0;

err_sysfs_create:
	kobject_put(properties_kobj);
err_kobj_create:
	pr_err("failed to create board_properties\n");
	return rc;
}


extern void msm_serial_debug_init(unsigned int base, int irq,
				  struct device *clk_device, int signal_irq,
				  int wakeup_irq);

static void __init msm7x30_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "board-vbus");
#ifdef CONFIG_DEBUG_LL
	{
		/* HACK: get a fake clock request for uart2 for debug_ll */
		struct clk *uart2_clk;
		uart2_clk = clk_get(&msm_device_uart2.dev, "uart_clk");
		if (IS_ERR(uart2_clk))
			uart2_clk = NULL;
		else
			clk_enable(uart2_clk);
	}
#endif

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART2_PHYS, INT_UART2,
			      &msm_device_uart2.dev, 23, MSM_GPIO_TO_INT(51));
#endif

	if (machine_is_msm7x30_fluid())
		msm7x30_pmic_keypad_pdata.keymap = msm7x30_fluid_pmic_keymap;
	else
		msm7x30_pmic_keypad_pdata.keymap = msm7x30_surf_pmic_keymap;

	msm7x30_ssbi_pmic_init();
	msm7x30_i2c_0_init();
	msm7x30_spi_init();

	/* set the gpu power rail to manual mode so clk en/dis will not
	 * turn off gpu power, and hang it on resume */
	msm7x30_kgsl_power_rail_mode(0);
	msm7x30_kgsl_power(true);

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	msm_device_spi.dev.platform_data = &msm7x30_spi_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));

	if (machine_is_msm7x30_fluid()) {
		fluid_cyttsp_init();
		i2c_register_board_info(0, fluid_i2c_0_board_info,
					ARRAY_SIZE(fluid_i2c_0_board_info));
	}
	i2c_register_board_info(1, surf_i2c_devices,
				ARRAY_SIZE(surf_i2c_devices));

	if (machine_is_msm7x30_fluid())
		fluid_board_props_init();

	msm7x30_board_audio_init();

	msm_hsusb_set_vbus_state(1);
	msm_hsusb_set_vbus_state(0);
	msm_hsusb_set_vbus_state(1);
}

static void __init msm7x30_map_io(void)
{
	msm_map_msm7x30_io();
	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
}

extern void __init msm7x30_allocate_fbmem(void);

static phys_addr_t _reserve_mem(const char *name, unsigned long size,
				unsigned long align)
{
	unsigned long base;

	size = ALIGN(size, align);
	base = memblock_alloc(size, align);
	memblock_free(base, size);
	memblock_remove(base, size);
	pr_info("msm7x30_surf: reserved memory for %s @ 0x%08lx (%lu bytes)\n",
		name, base, size);
	return base;
}

static void __init msm7x30_reserve(void)
{
	struct resource *mem_res;

	msm7x30_allocate_fbmem();

	pmem_pdata.start = _reserve_mem("pmem", pmem_pdata.size, SZ_1M);
	pmem_adsp_pdata.start = _reserve_mem("pmem_adsp", pmem_adsp_pdata.size,
					     SZ_1M);

	mem_res = platform_get_resource_byname(&msm_kgsl_device, IORESOURCE_MEM,
					       "kgsl_phys_memory");
	BUG_ON(!mem_res);
	mem_res->start = _reserve_mem("gpu_mem", MSM7X30_SURF_GPU_MEM_SIZE,
				      SZ_1M);
	mem_res->end = mem_res->start + MSM7X30_SURF_GPU_MEM_SIZE - 1;
}

MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.reserve = msm7x30_reserve,
MACHINE_END

MACHINE_START(MSM7X30_FFA, "QCT MSM7X30 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.reserve = msm7x30_reserve,
MACHINE_END

MACHINE_START(MSM7X30_FLUID, "QCT MSM7X30 FLUID")
#ifdef CONFIG_MSM_DEBUG_UART
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.reserve = msm7x30_reserve,
MACHINE_END
