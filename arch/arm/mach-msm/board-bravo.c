/* arch/arm/mach-msm/board-bravo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
 * Copyright (C) 2010 Giulio Cervera <giulio.cervera@gmail.com>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>

#include <linux/android_pmem.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/capella_cm3602_htc.h>
#include <linux/akm8973.h>
#include <linux/regulator/machine.h>
#include <linux/ds2784_battery.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <../../../drivers/w1/w1.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/msm_serial_hs.h>
#include <mach/bcm_bt_lpm.h>
#include <mach/msm_smd.h>
#include <mach/msm_flashlight.h>
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#include <mach/vreg.h>
#include <mach/board-bravo-microp-common.h>
#include <mach/socinfo.h>

#include "board-bravo.h"
#include "devices.h"
#include "proc_comm.h"
#include "board-bravo-tpa2018d1.h"
#include "board-bravo-smb329.h"

#include <linux/msm_kgsl.h>
#include <linux/regulator/machine.h>
#include "footswitch.h"

#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
#include <linux/curcial_oj.h>
#endif

static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

extern void notify_usb_connected(int);
extern void msm_init_pmic_vibrator(void);
extern void __init bravo_audio_init(void);

extern int microp_headset_has_mic(void);

static int bravo_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};

static void bravo_usb_phy_reset(void)
{
	u32 id;
	int ret;

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}

	msleep(1);

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}

static void bravo_usb_hw_reset(bool enable)
{
	u32 id;
	int ret;
	u32 func;

	id = PCOM_CLKRGM_APPS_RESET_USBH;
	if (enable)
		func = PCOM_CLK_REGIME_SEC_RESET_ASSERT;
	else
		func = PCOM_CLK_REGIME_SEC_RESET_DEASSERT;
	ret = msm_proc_comm(func, &id, NULL);
	if (ret)
		pr_err("%s: Cannot set reset to %d (%d)\n", __func__, enable,
		       ret);
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= bravo_phy_init_seq,
	.phy_reset		= bravo_usb_phy_reset,
	.hw_reset		= bravo_usb_hw_reset,
	.usb_connected		= notify_usb_connected,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
#endif

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c87,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0FFE,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	/*
	XXX: there isn't a equivalent in htc's kernel
	{
		.product_id	= 0x4e14,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	}, */
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		.vendor_id  = USB_ACCESSORY_VENDOR_ID,
		.product_id  = USB_ACCESSORY_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory),
		.functions  = usb_functions_accessory,
	},
	{
		.vendor_id  = USB_ACCESSORY_VENDOR_ID,
		.product_id  = USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory_adb),
		.functions  = usb_functions_accessory_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0c07,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
#endif
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Desire",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x0bb4,
	.vendorDescr	= "HTC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
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

static struct platform_device bravo_rfkill = {
	.name = "bravo_rfkill",
	.id = -1,
};

/* start kgsl */
static struct resource kgsl_3d0_resources[] = {
	{
		.name  = KGSL_3D0_REG_MEMORY,
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_3D0_IRQ,
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwrlevel = {
		{
			.gpu_freq = 0,
			.bus_freq = 128000000,
		},
	},
	.init_level = 0,
	.num_levels = 1,
	.set_grp_async = NULL,
	.idle_timeout = HZ/5,
	.clk_map = KGSL_CLK_GRP | KGSL_CLK_IMEM,
};

struct platform_device msm_kgsl_3d0 = {
	.name = "kgsl-3d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_3d0_resources),
	.resource = kgsl_3d0_resources,
	.dev = {
		.platform_data = &kgsl_3d0_pdata,
	},
};
/* end kgsl */

/* start footswitch regulator */
struct platform_device *msm_footswitch_devices[] = {
	FS_PCOM(FS_GFX3D,  "fs_gfx3d"),
};

unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);
/* end footswitch regulator */

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_venc_pdata = {
	.name		= "pmem_venc",
	.start		= MSM_PMEM_VENC_BASE,
	.size		= MSM_PMEM_VENC_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached		= 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_venc_device = {
	.name		= "android_pmem",
	.id		= 3,
	.dev		= {
		.platform_data = &android_pmem_venc_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int bravo_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(BRAVO_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 0);
		gpio_set_value(BRAVO_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

static struct synaptics_i2c_rmi_platform_data bravo_synaptics_ts_data[] = {
	{
		.version = 0x100,
		.power = bravo_ts_power,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -1 * 0x10000 / 480,
		.inactive_right = -1 * 0x10000 / 480,
		.inactive_top = -5 * 0x10000 / 800,
		.inactive_bottom = -5 * 0x10000 / 800,
		.sensitivity_adjust = 12,
	}
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = BRAVO_LAYOUTS,
	.project_name = BRAVO_PROJECT_NAME,
	.reset = BRAVO_GPIO_COMPASS_RST_N,
	.intr = BRAVO_GPIO_COMPASS_INT_N,
};

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] = {
	{
		.supply = "acpu_vcore",
	},
};

static struct regulator_init_data tps65023_data[5] = {
	{
		.constraints = {
			.name = "dcdc1", /* VREG_MSMC2_1V29 */
			.min_uV = 975000,
#ifdef CONFIG_JESUS_PHONE
			.max_uV = 1350000,
#else
			.max_uV = 1275000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies = tps65023_dcdc1_supplies,
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
	},
	/* dummy values for unused regulators to not crash driver: */
	{
		.constraints = {
			.name = "dcdc2", /* VREG_MSMC1_1V26 */
			.min_uV = 1260000,
			.max_uV = 1260000,
		},
	},
	{
		.constraints = {
			.name = "dcdc3", /* unused */
			.min_uV = 800000,
			.max_uV = 3300000,
		},
	},
	{
		.constraints = {
			.name = "ldo1", /* unused */
			.min_uV = 1000000,
			.max_uV = 3150000,
		},
	},
	{
		.constraints = {
			.name = "ldo2", /* V_USBPHY_3V3 */
			.min_uV = 3300000,
			.max_uV = 3300000,
		},
	},
};

static void ds2482_set_slp_n(unsigned n)
{
	gpio_direction_output(BRAVO_GPIO_DS2482_SLP_N, n);
}

static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name = "light_sensor",
		.category = MICROP_FUNCTION_LSENSOR,
		.levels = { 0x000, 0x001, 0x00F, 0x01E, 0x03C, 0x121, 0x190, 0x2BA, 0x35C, 0x3FF },
		.channel = 6,
		.int_pin = IRQ_LSENSOR,
		.golden_adc = 0xC0,
		.ls_power = capella_cm3602_power,
	},
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_functions[0],
	.irq = MSM_uP_TO_INT(9),
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = BRAVO_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static struct tpa2018d1_platform_data tpa2018_data = {
	.gpio_tpa2018_spk_en = BRAVO_CDMA_GPIO_AUD_SPK_AMP_EN,
};

static struct i2c_board_info base_i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
		.platform_data = bravo_synaptics_ts_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO("bravo-microp", 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO("ds2482", 0x30 >> 1),
		.platform_data = ds2482_set_slp_n,
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
};

static struct i2c_board_info rev_CX_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tpa2018d1", 0x58),
		.platform_data = &tpa2018_data,
	},
	{
		I2C_BOARD_INFO("smb329", 0x6E >> 1),
	},
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
};

void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name 	= "s5k3e2fx",
	.sensor_reset 	= 144,  /* CAM1_RST */
	.sensor_pwd 	= 143,  /* CAM1_PWDN, enabled in a9 */
	/*.vcm_pwd 	= 31,*/ /* CAM1_VCM_EN, enabled in a9 */
	.pdata 		= &msm_camera_device_data,
	.resource 	= msm_camera_resources,
	.num_resources 	= ARRAY_SIZE(msm_camera_resources),
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name	= "msm_camera_s5k3e2fx",
	.dev	= {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

static int __capella_cm3602_power(int on)
{
	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		gpio_direction_output(BRAVO_GPIO_LS_EN_N, 0);
		gpio_direction_output(BRAVO_GPIO_PROXIMITY_EN, 1);
	} else {
		gpio_direction_output(BRAVO_GPIO_LS_EN_N, 1);
	}
	return 0;
};

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	/* TODO eolsen Add Voltage reg control */
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
};

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = BRAVO_GPIO_PROXIMITY_EN,
	.p_out = BRAVO_GPIO_PROXIMITY_INT_N,
	.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_PROXIMITY_INT_N),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

static uint32_t flashlight_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
};

static uint32_t flashlight_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(BRAVO_CDMA_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
};

static int config_bravo_flashlight_gpios(void)
{
	if (is_cdma_version(system_rev)) {
		config_gpio_table(flashlight_gpio_table_rev_CX,
				ARRAY_SIZE(flashlight_gpio_table_rev_CX));
	} else {
		config_gpio_table(flashlight_gpio_table,
				ARRAY_SIZE(flashlight_gpio_table));
	}
	return 0;
}

static struct flashlight_platform_data bravo_flashlight_data = {
	.gpio_init = config_bravo_flashlight_gpios,
	.torch = BRAVO_GPIO_FLASHLIGHT_TORCH,
	.flash = BRAVO_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device bravo_flashlight_device = {
	.name = "flashlight",
	.dev = {
		.platform_data = &bravo_flashlight_data,
	},
};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = BRAVO_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device bravo_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = BRAVO_GPIO_BT_WAKE,
	.gpio_host_wake = BRAVO_GPIO_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off_locked,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

static int ds2784_charge(int on, int fast)
{
	if (is_cdma_version(system_rev)) {
		if (!on)
			smb329_set_charger_ctrl(SMB329_DISABLE_CHG);
		else
			smb329_set_charger_ctrl(fast ? SMB329_ENABLE_FAST_CHG : SMB329_ENABLE_SLOW_CHG);
	}
	else
		gpio_direction_output(BRAVO_GPIO_BATTERY_CHARGER_CURRENT, !!fast);
	gpio_direction_output(BRAVO_GPIO_BATTERY_CHARGER_EN, !on);
	return 0;
}

static int w1_ds2784_add_slave(struct w1_slave *sl)
{
	struct dd {
		struct platform_device pdev;
		struct ds2784_platform_data pdata;
	} *p;

	int rc;

	p = kzalloc(sizeof(struct dd), GFP_KERNEL);
	if (!p) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	rc = gpio_request(BRAVO_GPIO_BATTERY_CHARGER_EN, "charger_en");
	if (rc < 0) {
		pr_err("%s: gpio_request(%d) failed: %d\n", __func__,
			BRAVO_GPIO_BATTERY_CHARGER_EN, rc);
		kfree(p);
		return rc;
	}

	if (!is_cdma_version(system_rev)) {
		rc = gpio_request(BRAVO_GPIO_BATTERY_CHARGER_CURRENT, "charger_current");
		if (rc < 0) {
			pr_err("%s: gpio_request(%d) failed: %d\n", __func__,
					BRAVO_GPIO_BATTERY_CHARGER_CURRENT, rc);
			gpio_free(BRAVO_GPIO_BATTERY_CHARGER_EN);
			kfree(p);
			return rc;
		}
	}

	p->pdev.name = "ds2784-battery";
	p->pdev.id = -1;
	p->pdev.dev.platform_data = &p->pdata;
	p->pdata.charge = ds2784_charge;
	p->pdata.w1_slave = sl;

	platform_device_register(&p->pdev);

	return 0;
}

static struct w1_family_ops w1_ds2784_fops = {
	.add_slave = w1_ds2784_add_slave,
};

static struct w1_family w1_ds2784_family = {
	.fid = W1_FAMILY_DS2784,
	.fops = &w1_ds2784_fops,
};

static int __init ds2784_battery_init(void)
{
	return w1_register_family(&w1_ds2784_family);
}

#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
static void curcial_oj_shutdown(int enable)
{
	uint8_t cmd[3];

	memset(cmd, 0x00, sizeof(uint8_t)*3);
	cmd[2] = 0x20;
	// microp firmware(v04) non-shutdown by default
	microp_i2c_write(0x90, cmd, 3);
	pr_err("%s\n", __func__);
}

#define CURCIAL_OJ_POWER		150
static int curcial_oj_poweron(int on)
{
	uint8_t data[2];

#ifdef CONFIG_MACH_BRAVO
	struct vreg *oj_power = vreg_get(0, "gp2");
	if (IS_ERR(oj_power)) {
		pr_err("%s: Error power domain\n", __func__);
		return 0;
	}

	if (on) {
		vreg_set_level(oj_power, 2750);
		vreg_enable(oj_power);
	} else {
		/* for microp firmware(v04) setting*/
		microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
		if (data[0] < 4) {
			printk("Microp firmware version: %d\n", data[0]);
			return 1;
		}
		vreg_disable(oj_power);
	}
	pr_err("%s: OJ power enable(%d)\n", __func__, on);
#else
	/* for microp firmware(v04) setting*/
	if (on == 0) {
		microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
		if (data[0] < 4) {
			printk("Microp firmware version:%d\n",data[0]);
			return 1;
		}
	}

	gpio_set_value(CURCIAL_OJ_POWER, on);

	if (gpio_get_value(CURCIAL_OJ_POWER) != on) {
		printk(KERN_ERR "%s:OJ:power status fail \n", __func__);
		return 0;
	}
	printk(KERN_ERR "%s:OJ:power status ok \n", __func__);
#endif
	return 1;
}

static void curcial_oj_adjust_xy(uint8_t *data, int16_t *mSumDeltaX, int16_t *mSumDeltaY)
{
	int8_t 	deltaX;
	int8_t 	deltaY;

	if (data[2] == 0x80)
		data[2] = 0x81;
	if (data[1] == 0x80)
		data[1] = 0x81;
	if (1) {
		deltaX = (1)*((int8_t) data[2]); /*X=2*/
		deltaY = (-1)*((int8_t) data[1]); /*Y=1*/
	} else {
		deltaX = (-1)*((int8_t) data[1]);
		deltaY = (1)*((int8_t) data[2]);
	}
	*mSumDeltaX += -((int16_t)deltaX);
	*mSumDeltaY += -((int16_t)deltaY);
}

#define BRAVO_MICROP_VER	0x03
static struct curcial_oj_platform_data bravo_oj_data = {
	.oj_poweron	= curcial_oj_poweron,
	.oj_shutdown	= curcial_oj_shutdown,
	.oj_adjust_xy	= curcial_oj_adjust_xy,
	.microp_version	= BRAVO_MICROP_VER,
	.mdelay_time	= 0,
	.normal_th	= 8,
	.xy_ratio	= 15,
#ifdef CONFIG_MACH_BRAVO
	.interval	= 0,
	.swap		= false,
	.y		= -1,
#else
	.interval	= 10,
	.swap		= true,
	.y		= 1,
#endif
	.x		= 1,
	.share_power	= false,
	.debugflag	= 0,
	.ap_code	= false,
	.sht_tbl	= {0, 1000, 1250, 1500, 1750, 2000, 3000},
	.pxsum_tbl	= {0, 0, 90, 100, 110, 120, 130},
	.degree		= 7,
	.Xsteps = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12,
		14, 16, 18, 20, 22, 24, 26, 27, 28, 29,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.Ysteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.irq		= MSM_uP_TO_INT(12),
};

static struct platform_device bravo_oj = {
	.name = CURCIAL_OJ_NAME,
	.id = -1,
	.dev = {
		.platform_data = &bravo_oj_data,
	}
};
#endif

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
	&bcm_bt_lpm_device,
	&msm_device_uart_dm1,
	&ram_console_device,
	&bravo_rfkill,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb,
	&usb_mass_storage_device,
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_device,
#endif
	&android_usb_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
#ifdef CONFIG_720P_CAMERA
	&android_pmem_venc_device,
#endif
	&msm_kgsl_3d0,
	&msm_device_i2c,
	&msm_camera_sensor_s5k3e2fx,
	&bravo_flashlight_device,
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	&bravo_oj,
#endif
	&capella_cm3602,
};

static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t bt_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_CDMA_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t misc_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_LCD_RST_N, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_LED_3V3_EN, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_DOCK, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t key_int_shutdown_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
};

static void bravo_headset_init(void)
{
	if (is_cdma_version(system_rev))
		return;
	config_gpio_table(key_int_shutdown_gpio_table,
			ARRAY_SIZE(key_int_shutdown_gpio_table));
	gpio_set_value(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
}

#define ATAG_BDADDR 0x43294329  /* bravo bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int __init parse_tag_bdaddr(const struct tag *tag)
{
	unsigned char *b = (unsigned char *)&tag->u;

	if (tag->hdr.size != ATAG_BDADDR_SIZE)
		return -EINVAL;

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
			b[0], b[1], b[2], b[3], b[4], b[5]);

	return 0;
}

__tagtable(ATAG_BDADDR, parse_tag_bdaddr);

static int __init bravo_board_serialno_setup(char *serialno)
{
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
#endif

	android_usb_pdata.serial_number = serialno;
	msm_hsusb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", bravo_board_serialno_setup);

static struct msm_acpu_clock_platform_data bravo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
	.mpll_khz		= 245000
};

static struct msm_acpu_clock_platform_data bravo_cdma_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 235930,
	.wait_for_irq_khz	= 235930,
	.mpll_khz		= 235930
};

#ifdef CONFIG_PERFLOCK
static unsigned bravo_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data bravo_perflock_data = {
	.perf_acpu_table = bravo_perf_acpu_table,
	.table_size = ARRAY_SIZE(bravo_perf_acpu_table),
};
#endif

static void bravo_reset(void)
{
	gpio_set_value(BRAVO_GPIO_PS_HOLD, 0);
};

int bravo_init_mmc(int sysrev, unsigned debug_uart);

static const struct smd_tty_channel_desc smd_cdma_default_channels[] = {
	{ .id = 0, .name = "SMD_DS" },
	{ .id = 19, .name = "SMD_DATA3" },
	{ .id = 27, .name = "SMD_GPSNMEA" }
};

static void __init bravo_init(void)
{
	int ret;

	printk("bravo_init() revision=%d\n", system_rev);

	if (is_cdma_version(system_rev))
		smd_set_channel_list(smd_cdma_default_channels,
				ARRAY_SIZE(smd_cdma_default_channels));

	msm_hw_reset_hook = bravo_reset;

	bravo_board_serialno_setup(board_serialno());

	if (is_cdma_version(system_rev))
		msm_acpu_clock_init(&bravo_cdma_clock_data);
	else
		msm_acpu_clock_init(&bravo_clock_data);

#ifdef CONFIG_PERFLOCK
	perflock_init(&bravo_perflock_data);
#endif

	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1, MSM_GPIO_TO_INT(139));

	config_gpio_table(misc_gpio_table, ARRAY_SIZE(misc_gpio_table));

	if (is_cdma_version(system_rev)) {
		bcm_bt_lpm_pdata.gpio_wake = BRAVO_CDMA_GPIO_BT_WAKE;
		bravo_flashlight_data.torch = BRAVO_CDMA_GPIO_FLASHLIGHT_TORCH;
		config_gpio_table(bt_gpio_table_rev_CX, ARRAY_SIZE(bt_gpio_table_rev_CX));
	} else {
		config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
	}

	gpio_request(BRAVO_GPIO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(BRAVO_GPIO_TP_LS_EN, 0);
	gpio_request(BRAVO_GPIO_TP_EN, "tp_en");
	gpio_direction_output(BRAVO_GPIO_TP_EN, 0);
//	gpio_request(BRAVO_GPIO_PROXIMITY_EN, "proximity_en");
//	gpio_direction_output(BRAVO_GPIO_PROXIMITY_EN, 1);
	gpio_request(BRAVO_GPIO_LS_EN_N, "ls_en");
	gpio_request(BRAVO_GPIO_COMPASS_RST_N, "compass_rst");
	gpio_direction_output(BRAVO_GPIO_COMPASS_RST_N, 1);
	gpio_request(BRAVO_GPIO_COMPASS_INT_N, "compass_int");
	gpio_direction_input(BRAVO_GPIO_COMPASS_INT_N);

	gpio_request(BRAVO_GPIO_DS2482_SLP_N, "ds2482_slp_n");

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));

	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);

	i2c_register_board_info(0, base_i2c_devices,
		ARRAY_SIZE(base_i2c_devices));

	if (is_cdma_version(system_rev)) {
		i2c_register_board_info(0, rev_CX_i2c_devices,
			ARRAY_SIZE(rev_CX_i2c_devices));
	}

	ret = bravo_init_mmc(system_rev, debug_uart);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	bravo_audio_init();
	bravo_headset_init();

	platform_device_register(&bravo_timed_gpios);

	ds2784_battery_init();
}

static void __init bravo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
	mi->bank[1].start = MSM_EBI1_BANK1_BASE;
	mi->bank[1].size = MSM_EBI1_BANK1_SIZE;
}

static void __init bravo_map_io(void)
{
	msm_map_qsd8x50_io();
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",__func__);
}

extern struct sys_timer msm_timer;

#ifdef CONFIG_MACH_BRAVO
MACHINE_START(BRAVO, "bravo")
#else
MACHINE_START(BRAVOC, "bravoc")
#endif
	.boot_params	= 0x20000100,
	.fixup		= bravo_fixup,
	.map_io		= bravo_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= bravo_init,
	.timer		= &msm_timer,
MACHINE_END
