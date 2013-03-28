/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/bma250.h>
#include <linux/cm3629.h>
#include <linux/lightsensor.h>
#include <linux/input.h>
#include <linux/atmel_qt602240.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/himax8526a.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/i2c/isa1200.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pm8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/htc_flashlight.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/system.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera-7x30.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/msm_spi.h>
#include <mach/qdsp5v2_2x/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2_2x/aux_pcm.h>
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/htc_battery.h>
#include <linux/tps65200.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>

#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_G_ANDROID
#include <mach/htc_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"
#include "board-primou.h"
#include <mach/tpa2051d3.h>
#include "board-msm7x30-regulator.h"
#include <mach/board_htc.h>

#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

#ifdef CONFIG_BT
#include <mach/htc_bdaddress.h>
#endif

#define GPIO_2MA       0
#define PMIC_VREG_WLAN_LEVEL	2900

#define ADV7520_I2C_ADDR	0x39

#define FPGA_SDCC_STATUS       0x8E0001A8

#define FPGA_OPTNAV_GPIO_ADDR	0x8E000026
#define OPTNAV_I2C_SLAVE_ADDR	(0xB0 >> 1)

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)

#if 0
#define PMIC_GPIO_WLAN_EXT_POR  22 /* PMIC GPIO 23 is volume up GPIO */
#define BMA150_GPIO_INT 1
#define HAP_LVL_SHFT_MSM_GPIO 24
#endif

int htc_get_usb_accessory_adc_level(uint32_t *buffer);
#include <mach/cable_detect.h>

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

int __init primou_init_panel(void);

static unsigned int engineerid;
unsigned long msm_fb_base;

unsigned int primou_get_engineerid(void)
{
	return engineerid;
}

#define GPIO_INPUT      0
#define GPIO_OUTPUT     1

#define GPIO_NO_PULL    0
#define GPIO_PULL_DOWN  1
#define GPIO_PULL_UP    3

#define PCOM_GPIO_CFG(gpio, func, dir, pull, drvstr) \
		((((gpio) & 0x3FF) << 4)        | \
		((func) & 0xf)                  | \
		(((dir) & 0x1) << 14)           | \
		(((pull) & 0x3) << 15)          | \
		(((drvstr) & 0xF) << 17))

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int primou_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on == 1) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(PRIMOU_TP_RSTz), 1);
	} else if (on == 2) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(PRIMOU_TP_RSTz), 0);
		msleep(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(PRIMOU_TP_RSTz), 1);
		msleep(40);
	}

	return 0;
}

struct atmel_i2c_platform_data primou_ts_atmel_data[] = {
	{
		.version = 0x0011,
		.build = 0xAA,
		.source = 1, /* ALPS */
		.abs_x_min = 0,
		.abs_x_max = 479,
		.abs_y_min = 0,
		.abs_y_max = 948,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.power = primou_ts_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {15, 10, 50},
		.config_T8 = {30, 0, 5, 5, 0, 64, 2, 30, 16, 192},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 45, 2, 1, 0, 5, 2, 0, 5, 20, 17, 15, 0, 0, 223, 1, 5, 0, 50, 50, 148, 40, 143, 35, 18, 15, 58, 59, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 96, 109, 216, 89, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 50, 20, 20, 250, 3, 0, 5},
		.config_T46 = {0, 3, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 66, 8, 0, 0, 0, 0, 18, 30, 0, 10, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 1, 15, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T55 = {0, 0, 0, 0},
		.config_T58 = {114, 35, 3, 0, 50, 0, 0, 0, 0, 0, 0},
		.object_crc = {0x57, 0x79, 0xB9},
		.cable_config = {60, 30, 16, 16, 0, 0},
		.mferr_config = {85, 60, 16, 32, 0, 0, 3, 2, 0, 0, 0, 0, 0},
		.filter_level = {6, 18, 462, 474},
	},
	{
		.version = 0x0011,
		.build = 0xAA,
		.source = 0, /* J-Touch */
		.abs_x_min = 0,
		.abs_x_max = 479,
		.abs_y_min = 0,
		.abs_y_max = 948,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.power = primou_ts_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {15, 10, 50},
		.config_T8 = {30, 0, 5, 5, 0, 64, 2, 30, 16, 192},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 45, 2, 1, 0, 5, 2, 0, 5, 20, 17, 15, 0, 0, 223, 1, 0, 0, 45, 45, 143, 50, 140, 25, 18, 15, 58, 59, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 96, 109, 216, 89, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 50, 20, 20, 250, 3, 0, 5},
		.config_T46 = {0, 3, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 66, 8, 0, 0, 0, 0, 18, 30, 0, 20, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 1, 15, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T55 = {0, 0, 0, 0},
		.config_T58 = {114, 35, 3, 0, 50, 0, 0, 0, 0, 0, 0},
		.object_crc = {0x73, 0x1C, 0x4E},
		.cable_config = {60, 30, 16, 16, 0, 0},
		.mferr_config = {85, 60, 16, 32, 0, 0, 3, 2, 0, 0, 0, 0, 0},
		.filter_level = {6, 18, 462, 474},
	},
	{
		.version = 0x0011,
		.build = 0xF8,
		.source = 1, /* ALPS */
		.abs_x_min = 0,
		.abs_x_max = 479,
		.abs_y_min = 0,
		.abs_y_max = 948,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.power = primou_ts_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {15, 10, 50},
		.config_T8 = {30, 0, 5, 5, 0, 64, 2, 30, 16, 192},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 45, 2, 1, 0, 5, 2, 0, 5, 20, 17, 15, 0, 0, 223, 1, 5, 0, 50, 50, 148, 40, 143, 35, 18, 15, 58, 59, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 96, 109, 216, 89, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 0, 0, 0, 0, 0, 0},
		.config_T46 = {0, 3, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 66, 8, 0, 0, 0, 0, 18, 30, 0, 10, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 1, 15, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.object_crc = {0xDD, 0x54, 0x94},
		.cable_config = {60, 30, 16, 16, 0, 0},
		.noiseLine_config = {85, 60, 16, 32, 0, 0, 5, 1},
		.filter_level = {6, 18, 462, 474},
	},
	{
		.version = 0x0011,
		.build = 0xF8,
		.source = 0, /* J-Touch */
		.abs_x_min = 0,
		.abs_x_max = 479,
		.abs_y_min = 0,
		.abs_y_max = 948,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.power = primou_ts_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {15, 10, 50},
		.config_T8 = {30, 0, 5, 5, 0, 64, 2, 30, 16, 192},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 45, 2, 1, 0, 5, 2, 0, 5, 20, 17, 15, 0, 0, 223, 1, 0, 0, 45, 45, 143, 50, 140, 25, 18, 15, 58, 59, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 96, 109, 216, 89, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 0, 0, 0, 0, 0, 0},
		.config_T46 = {0, 3, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 66, 8, 0, 0, 0, 0, 18, 30, 0, 10, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 1, 15, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.object_crc = {0x8C, 0xE8, 0x03},
		.cable_config = {60, 30, 16, 16, 0, 0},
		.noiseLine_config = {85, 60, 16, 32, 0, 0, 5, 1},
		.filter_level = {6, 18, 462, 474},
	},
	{
		.version = 0x0010,
		.build = 0xAA,
		.source = 1, /* ALPS */
		.abs_x_min = 0,
		.abs_x_max = 479,
		.abs_y_min = 0,
		.abs_y_max = 948,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.power = primou_ts_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {15, 8, 50},
		.config_T8 = {30, 0, 5, 5, 0, 0, 5, 30, 5, 192},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 45, 3, 1, 0, 5, 2, 0, 5, 15, 15, 15, 0, 0, 223, 1, 5, 0, 50, 50, 148, 40, 143, 35, 18, 15, 58, 59, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 96, 109, 216, 89, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 0, 0, 0, 0, 0, 0},
		.config_T46 = {0, 3, 8, 8, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 68, 96, 0, 0, 0, 0, 0, 12, 12, 0, 0, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 0, 0, 0, 0, 0, 0, 35, 3, 5, 2, 0, 5, 10, 10, 239, 238, 35, 35, 143, 75, 138, 50, 18, 17, 4},
		.object_crc = {0xF4, 0xBE, 0xC7},
	},
	{
		.version = 0x0010,
		.build = 0xAA,
		.source = 0, /* J-Touch */
		.abs_x_min = 0,
		.abs_x_max = 479,
		.abs_y_min = 0,
		.abs_y_max = 948,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.power = primou_ts_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {15, 8, 50},
		.config_T8 = {30, 0, 5, 5, 0, 0, 5, 30, 5, 192},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 45, 3, 1, 0, 5, 2, 0, 5, 15, 15, 15, 0, 0, 223, 1, 0, 0, 45, 45, 143, 50, 140, 25, 18, 15, 58, 59, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 96, 109, 216, 89, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 0, 0, 0, 0, 0, 0},
		.config_T46 = {0, 3, 8, 8, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 68, 96, 0, 0, 0, 0, 0, 12, 12, 0, 0, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 0, 0, 0, 0, 0, 0, 35, 3, 5, 2, 0, 5, 10, 10, 239, 238, 35, 35, 143, 75, 138, 50, 18, 17, 4},
		.object_crc = {0xB1, 0x91, 0x12},
	},
};

static struct synaptics_i2c_rmi_platform_data primou_ts_synaptics_data[] = { /* Synatpics sensor */
	{
		.version = 0x3330,
		.packrat_number = 1100754,
		.abs_x_min = 0,
		.abs_x_max = 1090,
		.abs_y_min = 0,
		.abs_y_max = 1750,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.default_config = 3,
		.large_obj_check = 1,
		.customer_register = {0xF9, 0x64, 0x63, 0x32},
		.config = {0x45, 0x30, 0x30, 0x46, 0x00, 0x3F, 0x03, 0x1E,
			0x05, 0xB1, 0x88, 0x0B, 0x19, 0x19, 0x00, 0x00,
			0x4C, 0x04, 0x6C, 0x07, 0x02, 0x14, 0x1E, 0x05,
			0x32, 0x05, 0x11, 0x96, 0x02, 0x01, 0x3C, 0x23,
			0x00, 0x23, 0x00, 0xB2, 0x49, 0x34, 0x48, 0xF8,
			0xA7, 0xF8, 0xA7, 0x00, 0x40, 0x00, 0x00, 0x00,
			0x00, 0x0A, 0x04, 0xC0, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x19, 0x01, 0x00, 0x0A, 0x40, 0x32,
			0xA3, 0x03, 0x23, 0x07, 0x07, 0x6E, 0x0B, 0x13,
			0x00, 0x02, 0xEE, 0x00, 0x80, 0x03, 0x0E, 0x1F,
			0x10, 0x35, 0x00, 0x13, 0x04, 0x00, 0x00, 0x08,
			0xFF, 0x01, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
			0x18, 0x19, 0x1A, 0x1B, 0x11, 0xFF, 0xFF, 0xFF,
			0x12, 0x10, 0x0E, 0x0C, 0x06, 0x04, 0x02, 0x01,
			0x08, 0x09, 0x0A, 0x0B, 0x0D, 0x0F, 0x11, 0x13,
			0x07, 0x05, 0x03, 0xFF, 0xC0, 0xA0, 0xA0, 0xA0,
			0xA0, 0xA0, 0xA8, 0xA8, 0x48, 0x46, 0x45, 0x43,
			0x42, 0x41, 0x3F, 0x3D, 0x00, 0x02, 0x04, 0x06,
			0x08, 0x0A, 0x0C, 0x0F, 0x00, 0x88, 0x13, 0xCD,
			0x64, 0x00, 0xC8, 0x00, 0x80, 0x0A, 0x80, 0xB8,
			0x0B, 0x00, 0xC0, 0x80, 0x00, 0x10, 0x00, 0x10,
			0x00, 0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x02,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x20, 0x20,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x55, 0x58,
			0x5A, 0x5C, 0x5E, 0x60, 0x62, 0x65, 0x00, 0x8C,
			0x00, 0x10, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
	{
		.version = 0x3330,
		.packrat_number = 1092704,
		.abs_x_min = 0,
		.abs_x_max = 1090,
		.abs_y_min = 0,
		.abs_y_max = 1750,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.default_config = 3,
		.large_obj_check = 1,
		.config = { 0x45, 0x30, 0x30, 0x39, 0x00, 0x3F, 0x03, 0x1E,
			0x05, 0xB1, 0x88, 0x0B, 0x19, 0x19, 0x00, 0x00,
			0x4C, 0x04, 0x6C, 0x07, 0x02, 0x14, 0x1E, 0x05,
			0x32, 0x05, 0x11, 0x96, 0x02, 0x01, 0x3C, 0x23,
			0x00, 0x23, 0x00, 0xB2, 0x49, 0x34, 0x48, 0x04,
			0xB8, 0xDA, 0xB6, 0x00, 0x50, 0x00, 0x00, 0x00,
			0x00, 0x0A, 0x04, 0xC0, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x19, 0x01, 0x00, 0x0A, 0x40, 0x32,
			0xA3, 0x03, 0x23, 0x07, 0x07, 0x6E, 0x0B, 0x13,
			0x00, 0x02, 0xEE, 0x00, 0x80, 0x03, 0x0E, 0x1F,
			0x10, 0x35, 0x00, 0x13, 0x04, 0x00, 0x00, 0x08,
			0xFF, 0x01, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
			0x18, 0x19, 0x1A, 0x1B, 0x11, 0xFF, 0xFF, 0xFF,
			0x12, 0x10, 0x0E, 0x0C, 0x06, 0x04, 0x02, 0x01,
			0x08, 0x09, 0x0A, 0x0B, 0x0D, 0x0F, 0x11, 0x13,
			0x07, 0x05, 0x03, 0xFF, 0xC0, 0xA0, 0xA0, 0xA0,
			0xA0, 0xA0, 0xA8, 0xA8, 0x48, 0x46, 0x45, 0x43,
			0x42, 0x41, 0x3F, 0x3D, 0x00, 0x02, 0x04, 0x06,
			0x08, 0x0A, 0x0C, 0x0F, 0x00, 0xA0, 0x0F, 0xFD,
			0x28, 0x00, 0x20, 0x4E, 0xB3, 0xC8, 0xCD, 0xA0,
			0x0F, 0x00, 0xC0, 0x80, 0x00, 0x10, 0x00, 0x10,
			0x00, 0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x02,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x20, 0x20,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x55, 0x58,
			0x5A, 0x5C, 0x5E, 0x60, 0x62, 0x65, 0x00, 0xC8,
			0x00, 0x10, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	},
	{
		.version = 0x3330,
		.abs_x_min = 35,
		.abs_x_max = 1090,
		.abs_y_min = 0,
		.abs_y_max = 1750,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.default_config = 3,
		.large_obj_check = 1,
		.config = {0x30, 0x30, 0x33, 0x30, 0x00, 0x3F, 0x03, 0x1E,
			0x05, 0xB1, 0x09, 0x0B, 0x19, 0x19, 0x00, 0x00,
			0x4C, 0x04, 0x6C, 0x07, 0x02, 0x14, 0x1E, 0x05,
			0x32, 0xA5, 0x13, 0x8A, 0x03, 0x01, 0x3C, 0x17,
			0x00, 0x1A, 0x00, 0xB2, 0x49, 0x34, 0x48, 0x04,
			0xB8, 0xDA, 0xB6, 0x00, 0x50, 0x00, 0x00, 0x00,
			0x00, 0x0A, 0x04, 0x40, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x19, 0x01, 0x00, 0x0A, 0x00, 0x08,
			0xA3, 0x03, 0x23, 0x07, 0x07, 0x6E, 0x0B, 0x13,
			0x00, 0x02, 0xEE, 0x00, 0x80, 0x03, 0x0E, 0x1F,
			0x10, 0x30, 0x00, 0x13, 0x04, 0x00, 0x00, 0x10,
			0xFF, 0x01, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
			0x18, 0x19, 0x1A, 0x1B, 0x11, 0xFF, 0xFF, 0xFF,
			0x12, 0x10, 0x0E, 0x0C, 0x06, 0x04, 0x02, 0x01,
			0x08, 0x09, 0x0A, 0x0B, 0x0D, 0x0F, 0x11, 0x13,
			0x07, 0x05, 0x03, 0xFF, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x36, 0x36, 0x36, 0x36,
			0x36, 0x36, 0x36, 0x36, 0x00, 0x02, 0x04, 0x06,
			0x08, 0x0A, 0x0C, 0x0E, 0x00, 0xA0, 0x0F, 0xFD,
			0x28, 0x00, 0x20, 0x4E, 0xB3, 0xC8, 0xCD, 0xA0,
			0x0F, 0x00, 0xC0, 0x80, 0x00, 0x10, 0x00, 0x10,
			0x00, 0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
			0x30, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
			0x78, 0x53, 0x55, 0x57, 0x59, 0x5B, 0x5D, 0x5F,
			0x00, 0x28, 0x00, 0x10, 0x0A, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00},
	},
	{
		.version = 0x3230,
		.abs_x_min = 35,
		.abs_x_max = 1090,
		.abs_y_min = 0,
		.abs_y_max = 1750,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.default_config = 1,
		.config = {0x50, 0x52, 0x4F, 0x00, 0x84, 0x0F, 0x03, 0x1E,
			0x05, 0x20, 0xB1, 0x00, 0x0B, 0x19, 0x19, 0x00,
			0x00, 0x4C, 0x04, 0x6C, 0x07, 0x1E, 0x05, 0x2D,
			0x27, 0x05, 0xE9, 0x01, 0x01, 0x30, 0x00, 0x30,
			0x00, 0xB2, 0x49, 0x33, 0x48, 0x27, 0x97, 0x1C,
			0x9C, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x0A,
			0x04, 0xC0, 0x00, 0x02, 0x16, 0x01, 0x80, 0x01,
			0x0D, 0x1E, 0x00, 0x2B, 0x00, 0x19, 0x04, 0x1E,
			0x00, 0x10, 0x0A, 0x01, 0x12, 0x13, 0x14, 0x15,
			0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x11, 0xFF,
			0xFF, 0xFF, 0x12, 0x10, 0x0E, 0x0C, 0x06, 0x04,
			0x02, 0x01, 0x08, 0x09, 0x0A, 0x0B, 0x0D, 0x0F,
			0x11, 0x13, 0x07, 0x05, 0x03, 0xFF, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x38, 0x38,
			0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x00, 0x03,
			0x06, 0x09, 0x0C, 0x0F, 0x12, 0x15, 0x00, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0xFF, 0xFF, 0x00, 0xC0, 0x80, 0x00, 0x10,
			0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
			0x02, 0x02, 0x20, 0x30, 0x20, 0x20, 0x20, 0x20,
			0x20, 0x20, 0x51, 0x7E, 0x57, 0x5A, 0x5D, 0x60,
			0x63, 0x66, 0x30, 0x30, 0x00, 0x1E, 0x19, 0x05,
			0x00, 0x00, 0x3D, 0x08},
	},
	{
		.version = 0x0000,
		.abs_x_min = 35,
		.abs_x_max = 965,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
		.default_config = 2,
		.config = {0x30, 0x30, 0x30, 0x31, 0x84, 0x0F, 0x03, 0x1E,
			0x05, 0x00, 0x0B, 0x19, 0x19, 0x00, 0x00, 0xE8,
			0x03, 0x75, 0x07, 0x1E, 0x05, 0x28, 0xF5, 0x28,
			0x1E, 0x05, 0x01, 0x30, 0x00, 0x30, 0x00, 0x00,
			0x48, 0x00, 0x48, 0x0D, 0xD6, 0x56, 0xBE, 0x00,
			0x70, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x04, 0x00,
			0x02, 0xCD, 0x00, 0x80, 0x03, 0x0D, 0x1F, 0x00,
			0x21, 0x00, 0x15, 0x04, 0x1E, 0x00, 0x10, 0x02,
			0x01, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
			0x19, 0x1A, 0x1B, 0x11, 0xFF, 0xFF, 0xFF, 0x03,
			0x05, 0x07, 0x13, 0x11, 0x0F, 0x0D, 0x0B, 0x0A,
			0x09, 0x08, 0x01, 0x02, 0x04, 0x06, 0x0C, 0x0E,
			0x10, 0x12, 0xFF, 0xC0, 0x88, 0xC0, 0x88, 0xC0,
			0x88, 0xC0, 0x88, 0x3A, 0x34, 0x3A, 0x34, 0x3A,
			0x34, 0x3C, 0x34, 0x00, 0x04, 0x08, 0x0C, 0x1E,
			0x14, 0x3C, 0x1E, 0x00, 0x9B, 0x7F, 0x46, 0x20,
			0x4E, 0x9B, 0x7F, 0x28, 0x80, 0xCC, 0xF4, 0x01,
			0x00, 0xC0, 0x80, 0x00, 0x10, 0x00, 0x10, 0x00,
			0x10, 0x00, 0x10, 0x30, 0x30, 0x00, 0x1E, 0x19,
			0x05, 0x00, 0x00, 0x3D, 0x08, 0x00, 0x00, 0x00,
			0xBC, 0x02, 0x80},
	},
};

static void himax_ts_reset(void)
{
	printk(KERN_INFO "%s():\n", __func__);
	gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PRIMOU_TP_RSTz), 0);
	mdelay(10);
	gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PRIMOU_TP_RSTz), 1);
}

static int himax_ts_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata, struct himax_config_init_api *i2c_api);

struct himax_i2c_platform_data_config_type_3 config_type3[] = {
	{
		.version = 0x0D,
		.tw_id = 0,/* Panasonic */

		.c1 = { 0x62, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c2 = { 0x63, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c3 = { 0x64, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c4 = { 0x65, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c5 = { 0x66, 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c6 = { 0x67, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c7 = { 0x68, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c8 = { 0x69, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c9 = { 0x6A, 0x32, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c10 = { 0x6B, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c11 = { 0x6C, 0x12, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c12 = { 0x6D, 0x41, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 },
		.c13 = { 0xC9, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
			  0x0D, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x20, 0x1E,
			  0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c14 = { 0x8A, 0x00, 0x04, 0x00, 0x04, 0x00, 0x08, 0x2A, 0x0F, 0x4F, 0x70,
			  0x28, 0x0F, 0x00, 0x86, 0x01, 0xC5, 0x39, 0xE0, 0x0F, 0x00, 0x0B,
			  0x07, 0xFF, 0xFF, 0x0C, 0x06, 0xFF, 0xFF, 0x0D, 0xFF, 0x08, 0xFF,
			  0x0E, 0x05, 0xFF, 0xFF, 0x0F, 0xFF, 0x09, 0x1A, 0xFF, 0x04, 0x1C,
			  0x19, 0xFF, 0xFF, 0x0A, 0x18, 0xFF, 0x03, 0x1B, 0x17, 0xFF, 0x02,
			  0x1D, 0x16, 0x10, 0xFF, 0xFF, 0x15, 0x11, 0x01, 0xFF, 0x14, 0x00,
			  0x12, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c15 = { 0xC5, 0x0D, 0x1F, 0x00, 0x10, 0x1B, 0x1F, 0x0B },
		.c16 = { 0xC6, 0x11, 0x10, 0x17 },
		.c17 = { 0x7D, 0x00, 0x04, 0x0A, 0x0A, 0x02 },
		.c18 = { 0x7F, 0x08, 0x01, 0x01, 0x01, 0x01, 0x07, 0x08, 0x07, 0x0F, 0x07,
			  0x0F, 0x07, 0x0F, 0x00 },
		.c19 = { 0xD5, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c20 = { 0xE9, 0x00, 0x00 },
		.c21 = { 0xEA, 0x0B, 0x13, 0x00, 0x13 },
		.c22 = { 0xEB, 0x32, 0x32, 0xFA, 0x83 },
		.c23 = { 0xEC, 0x03, 0x0F, 0x0A, 0x2D, 0x2D, 0x00, 0x00, 0x00, 0x00 },
		.c24 = { 0xED, 0x08, 0x06, 0x00, 0x00 },
		.c25 = { 0xEE, 0x00 },
		.c26 = { 0xEF, 0x11, 0x00 },
		.c27 = { 0xF0, 0x40 },
		.c28 = { 0xF1, 0x06, 0x04, 0x06, 0x03 },
		.c29 = { 0xF2, 0x0A, 0x06, 0x14, 0x3C },
		.c30 = { 0xF3, 0x5F },
		.c31 = { 0xF4, 0x7D, 0xB9, 0x2D, 0x3A },
		.c32 = { 0xF6, 0x00, 0x00, 0x1D, 0x76, 0x08 },
		.c33 = { 0xF7, 0x20, 0x78, 0x8F, 0x0F, 0x40 },
		.c34 = { 0x35, 0x02, 0x01 },
		.c35 = { 0x36, 0x0F, 0x53, 0x01 },
		.c36 = { 0x37, 0xFF, 0x08, 0xFF, 0x08 },
		.c37 = { 0x39, 0x03 },
		.c38 = { 0x3A, 0x00 },
		.c39 = { 0x50, 0xAB },
		.c40 = { 0x6E, 0x04 },
		.c41 = { 0x76, 0x01, 0x2D },
		.c42 = { 0x78, 0x03 },
		.c43 = { 0x7A, 0x00, 0x18, 0x0D },
		.c44 = { 0x8B, 0x00, 0x00 },
		.c45 = { 0x8C, 0x30, 0x0C, 0x0C, 0x0C, 0x0C, 0x08, 0x0C, 0x32, 0x24, 0x80,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c46 = { 0x8D, 0xA0, 0x5A, 0x14, 0x0A, 0x32, 0x0A, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c47 = { 0xC2, 0x11, 0x00, 0x00, 0x00 },
		.c48 = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x05, 0x00 },
		.c49 = { 0xD4, 0x01, 0x04, 0x07 },
		.c50 = { 0xDD, 0x05, 0x02 },
		.checksum = { 0xAC, 0x5C, 0x4C },
	},
	{
		.version = 0x0D,
		.tw_id = 1,/* Nissha */

		.c1 = { 0x62, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c2 = { 0x63, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c3 = { 0x64, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c4 = { 0x65, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c5 = { 0x66, 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c6 = { 0x67, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c7 = { 0x68, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c8 = { 0x69, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c9 = { 0x6A, 0x32, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c10 = { 0x6B, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c11 = { 0x6C, 0x12, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c12 = { 0x6D, 0x41, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 },
		.c13 = { 0xC9, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
			 0x0D, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x20, 0x1E,
			 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c14 = { 0x8A, 0x00, 0x04, 0x00, 0x04, 0x00, 0x08, 0x2A, 0x0F, 0x4F, 0x70,
			 0x28, 0x0F, 0x00, 0x86, 0x01, 0xC5, 0x39, 0xE0, 0x0F, 0x00, 0x0B,
			 0x07, 0xFF, 0xFF, 0x0C, 0x06, 0xFF, 0xFF, 0x0D, 0xFF, 0x08, 0xFF,
			 0x0E, 0x05, 0xFF, 0xFF, 0x0F, 0xFF, 0x09, 0x1A, 0xFF, 0x04, 0x1C,
			 0x19, 0xFF, 0xFF, 0x0A, 0x18, 0xFF, 0x03, 0x1B, 0x17, 0xFF, 0x02,
			 0x1D, 0x16, 0x10, 0xFF, 0xFF, 0x15, 0x11, 0x01, 0xFF, 0x14, 0x00,
			 0x12, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c15 = { 0xC5, 0x0A, 0x1F, 0x00, 0x10, 0x1B, 0x1F, 0x0B },
		.c16 = { 0xC6, 0x11, 0x10, 0x16 },
		.c17 = { 0x7D, 0x00, 0x04, 0x0A, 0x0A, 0x02 },
		.c18 = { 0x7F, 0x0A, 0x01, 0x01, 0x01, 0x01, 0x06, 0x04, 0x07, 0x0D, 0x07,
			 0x0D, 0x07, 0x0D, 0x00 },
		.c19 = { 0xD5, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c20 = { 0xE9, 0x00, 0x00 },
		.c21 = { 0xEA, 0x0B, 0x13, 0x00, 0x13 },
		.c22 = { 0xEB, 0x32, 0x32, 0xFA, 0x83 },
		.c23 = { 0xEC, 0x01, 0x0F, 0x0A, 0x2D, 0x2D, 0x00, 0x00, 0x00, 0x00 },
		.c24 = { 0xED, 0x08, 0x06, 0x00, 0x00 },
		.c25 = { 0xEE, 0x00 },
		.c26 = { 0xEF, 0x11, 0x00 },
		.c27 = { 0xF0, 0x40 },
		.c28 = { 0xF1, 0x06, 0x04, 0x06, 0x03 },
		.c29 = { 0xF2, 0x0A, 0x06, 0x14, 0x3C },
		.c30 = { 0xF3, 0x5F },
		.c31 = { 0xF4, 0x7D, 0xB9, 0x2D, 0x3A },
		.c32 = { 0xF6, 0x00, 0x00, 0x1D, 0x76, 0x0A },
		.c33 = { 0xF7, 0x50, 0x64, 0x64, 0x0F, 0x40 },
		.c34 = { 0x35, 0x02, 0x01 },
		.c35 = { 0x36, 0x0F, 0x53, 0x01 },
		.c36 = { 0x37, 0xFF, 0x08, 0xFF, 0x08 },
		.c37 = { 0x39, 0x03 },
		.c38 = { 0x3A, 0x00 },
		.c39 = { 0x50, 0xAB },
		.c40 = { 0x6E, 0x04 },
		.c41 = { 0x76, 0x01, 0x3F },
		.c42 = { 0x78, 0x03 },
		.c43 = { 0x7A, 0x00, 0x18, 0x0D },
		.c44 = { 0x8B, 0x00, 0x00 },
		.c45 = { 0x8C, 0x30, 0x10, 0x10, 0x10, 0x0C, 0x08, 0x0C, 0x32, 0x24, 0x80,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c46 = { 0x8D, 0xA0, 0x5A, 0x14, 0x0A, 0x32, 0x0A, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c47 = { 0xC2, 0x11, 0x00, 0x00, 0x00 },
		.c48 = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x05, 0x00 },
		.c49 = { 0xD4, 0x01, 0x04, 0x07 },
		.c50 = { 0xDD, 0x05, 0x02 },
		.checksum = { 0xAC, 0x5E, 0x4C },
	},
};

#define CC(pa) do { \
	i2c_api->i2c_himax_master_write(client, pa , sizeof(pa), normalRetry); \
} while (0)

struct himax_i2c_platform_data himax_ts_data = {
	.slave_addr = 0x90,
	.abs_x_min = 0,
	.abs_x_max = 1024,
	.abs_y_min = 0,
	.abs_y_max = 950,
	.abs_pressure_min = 0,
	.abs_pressure_max = 200,
	.abs_width_min = 0,
	.abs_width_max = 200,
	.gpio_irq = PRIMOU_GPIO_TP_ATT_N,
	.version = 0x00,
	.tw_id = 0,
	.event_htc_enable = 0,
	.cable_config = { 0x90, 0x00},
	.powerOff3V3 = 0,
	.power = primou_ts_power,
	.protocol_type = PROTOCOL_TYPE_B,
	.screenWidth = 480,
	.screenHeight = 800,
	.loadSensorConfig = himax_ts_loadSensorConfig,
	.reset = himax_ts_reset,
	.type1 = 0,
	.type1_size = 0,
	.type2 = 0,
	.type2_size = 0,
	.type3 = config_type3,
	.type3_size = sizeof(config_type3),
};

static int himax_ts_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata, struct himax_config_init_api *i2c_api)
{
	uint8_t act_len;
	int result;
	char Data = 0;
	char cmd[3] = {0};
	int retryTimes = 0, i = 0;
	const int firstRetry = 3;
	const int normalRetry = 10;
	uint8_t tw_id = 0;
	uint8_t fw_ver = 0;
	struct himax_i2c_platform_data_config_type_1 *type1_selected = 0;
	struct himax_i2c_platform_data_config_type_2 *type2_selected = 0;
	struct himax_i2c_platform_data_config_type_3 *type3_selected = 0;
	uint8_t *type1_checksum = 0;
	uint8_t *type2_checksum = 0;
	uint8_t *type3_checksum = 0;

start:
	if (!pdata || !client || !i2c_api) {
		printk(KERN_ERR "[TP][TOUCH_ERR]%s: Necessary parameters are null!\n", __func__);
		return -1;
	}
	result = i2c_api->i2c_himax_write_command(client, 0x81, firstRetry);
	if (result < 0) {
		printk(KERN_INFO "[TP]No Himax chip inside\n");
		return -EIO;
	} else {
		hr_msleep(150);

		cmd[0] = 0x36;
		cmd[1] = 0x0F;
		cmd[2] = 0x53;
		i2c_api->i2c_himax_master_write(client, cmd , sizeof(cmd), normalRetry);
		cmd[0] = 0xDD;
		cmd[1] = 0x04;
		cmd[2] = 0x02;
		i2c_api->i2c_himax_master_write(client, cmd , sizeof(cmd), normalRetry);
		i2c_api->i2c_himax_write_command(client, 0x83, normalRetry);
		hr_msleep(100);
		i2c_api->i2c_himax_write_command(client, 0x82, normalRetry);
		i2c_api->i2c_himax_write_command(client, 0xF5, normalRetry);
		i2c_api->i2c_himax_read_command(client, 1, &tw_id, &act_len, normalRetry);
		i2c_api->i2c_himax_write_command(client, 0x32, normalRetry);
		i2c_api->i2c_himax_read_command(client, 1, &fw_ver, &act_len, normalRetry);

		if (pdata->type3) {
			for (i = 0; i < pdata->type3_size/sizeof(struct himax_i2c_platform_data_config_type_3); ++i) {
				if (fw_ver >= (pdata->type3)[i].version && tw_id == (pdata->type3)[i].tw_id) {
					type3_selected = &((pdata->type3)[i]);
					pdata->version = (pdata->type3)[i].version;
					pdata->tw_id = (pdata->type3)[i].tw_id;
					pdata->regCD = (pdata->type3)[i].c50;
					if (*((pdata->type3[i]).checksum))
						type3_checksum = (pdata->type3)[i].checksum;
					printk(KERN_INFO "[TP]type3 selected, %X, %X\n", (uint32_t)type3_selected, (uint32_t)type3_checksum);
				}
			}
		}

		if (pdata->type2 && !type3_selected) {
			for (i = 0; i < pdata->type2_size/sizeof(struct himax_i2c_platform_data_config_type_2); ++i) {
				if (fw_ver >= (pdata->type2)[i].version && tw_id == (pdata->type2)[i].tw_id) {
					type2_selected = &((pdata->type2)[i]);
					pdata->version = (pdata->type2[i]).version;
					pdata->tw_id = (pdata->type2[i]).tw_id;
					if (*((pdata->type2[i]).checksum))
						type2_checksum = (pdata->type2[i]).checksum;
					printk(KERN_INFO "[TP]type2 selected, %X, %X\n", (uint32_t)type2_selected, (uint32_t)type2_checksum);
				}
			}
		}

		if (pdata->type1 && !type2_selected && !type3_selected) {
			for (i = 0; i < pdata->type1_size/sizeof(struct himax_i2c_platform_data_config_type_1); ++i) {
				if (fw_ver >= (pdata->type1[i]).version && tw_id == (pdata->type1[i]).tw_id) {
					type1_selected = &((pdata->type1)[i]);
					pdata->version = (pdata->type1[i]).version;
					pdata->tw_id = (pdata->type1[i]).tw_id;
					if (*((pdata->type1[i]).checksum))
						type1_checksum = (pdata->type1[i]).checksum;
					printk(KERN_INFO "[TP]type1 selected, %X, %X\n", (uint32_t)type1_selected, (uint32_t)type1_checksum);
				}
			}
		}
		switch (tw_id & 0x03) {
		case 0x00:/* Panasonic */
			printk(KERN_INFO "[TP]%s: Panasonic touch window detected.\n", __func__);
			break;
		case 0x01:/* nissha */
			printk(KERN_INFO "[TP]%s: Nissha touch window detected.\n", __func__);
			break;
		}
		if (!type1_selected && !type2_selected && !type3_selected) {
			printk(KERN_ERR "[TP][TOUCH_ERR]%s: Couldn't find the matched profile!\n", __func__);
			return -1;
		}

		printk(KERN_INFO "[TP]%s: start initializing Sensor configs\n", __func__);
	}

	do {
		if (retryTimes == 5) {
			pdata->reset();
			hr_msleep(50);
			++retryTimes;
			goto start;
		} else if (retryTimes == 11) {
			printk(KERN_ERR "[TP][TOUCH_ERR]%s: Himax configuration checksum error!\n", __func__);
			return -EIO;
		}

		if (type1_checksum || type2_checksum || type3_checksum) {
			/* Reset Check Sum */
			cmd[0] = 0xAB; cmd[1] = 0x00;
			i2c_api->i2c_himax_master_write(client, cmd , 2, normalRetry);
			/* Start Check Sum */
			cmd[1] = 0x01;
			i2c_api->i2c_himax_master_write(client, cmd , 2, normalRetry);
		}

		if (type3_selected) {
			CC(type3_selected->c1); CC(type3_selected->c2); CC(type3_selected->c3); CC(type3_selected->c4);
			CC(type3_selected->c5); CC(type3_selected->c6); CC(type3_selected->c7); CC(type3_selected->c8);
			CC(type3_selected->c9); CC(type3_selected->c10); CC(type3_selected->c11); CC(type3_selected->c12);
			CC(type3_selected->c13); CC(type3_selected->c14); CC(type3_selected->c15); CC(type3_selected->c16);
			CC(type3_selected->c17); CC(type3_selected->c18); CC(type3_selected->c19); CC(type3_selected->c20);
			CC(type3_selected->c21); CC(type3_selected->c22); CC(type3_selected->c23); CC(type3_selected->c24);
			CC(type3_selected->c25); CC(type3_selected->c26); CC(type3_selected->c27); CC(type3_selected->c28);
			CC(type3_selected->c29); CC(type3_selected->c30); CC(type3_selected->c31); CC(type3_selected->c32);
			CC(type3_selected->c33); CC(type3_selected->c34); CC(type3_selected->c35); CC(type3_selected->c36);
			CC(type3_selected->c37); CC(type3_selected->c38); CC(type3_selected->c39); CC(type3_selected->c40);
			CC(type3_selected->c41); CC(type3_selected->c42); CC(type3_selected->c43); CC(type3_selected->c44);
			CC(type3_selected->c45); CC(type3_selected->c46); CC(type3_selected->c47); CC(type3_selected->c48);
			CC(type3_selected->c49); CC(type3_selected->c50);
		} else if (type2_selected) {
			CC(type2_selected->c1); CC(type2_selected->c2); CC(type2_selected->c3); CC(type2_selected->c4);
			CC(type2_selected->c5); CC(type2_selected->c6); CC(type2_selected->c7); CC(type2_selected->c8);
			CC(type2_selected->c9); CC(type2_selected->c10); CC(type2_selected->c11); CC(type2_selected->c12);
			CC(type2_selected->c13); CC(type2_selected->c14); CC(type2_selected->c15); CC(type2_selected->c16);
			CC(type2_selected->c17); CC(type2_selected->c18); CC(type2_selected->c19); CC(type2_selected->c20);
			CC(type2_selected->c21); CC(type2_selected->c22); CC(type2_selected->c23); CC(type2_selected->c24);
			CC(type2_selected->c25); CC(type2_selected->c26); CC(type2_selected->c27); CC(type2_selected->c28);
			CC(type2_selected->c29); CC(type2_selected->c30); CC(type2_selected->c31); CC(type2_selected->c32);
			CC(type2_selected->c33); CC(type2_selected->c34); CC(type2_selected->c35); CC(type2_selected->c36);
			CC(type2_selected->c37); CC(type2_selected->c38); CC(type2_selected->c39); CC(type2_selected->c40);
			CC(type2_selected->c41); CC(type2_selected->c42); CC(type2_selected->c43); CC(type2_selected->c44);
			CC(type2_selected->c45);
		} else {
		}

		if (type1_checksum || type2_checksum || type3_checksum) {
		/* Stop Check Sum */
			cmd[0] = 0xAB; cmd[1] = 0x10;
			i2c_api->i2c_himax_master_write(client, cmd , 2, normalRetry);

			/* Enter Golden Pattern */
			if (type1_checksum) {
				printk(KERN_INFO "[TP]Check type 1 checksum.\n");
				i2c_api->i2c_himax_master_write(client, type1_selected->checksum, sizeof(type1_selected->checksum), normalRetry);
			} else if (type2_checksum) {
				printk(KERN_INFO "[TP]Check type 2 checksum.\n");
				i2c_api->i2c_himax_master_write(client, type2_selected->checksum, sizeof(type2_selected->checksum), normalRetry);
			} else if (type3_checksum) {
				printk(KERN_INFO "[TP]Check type 3 checksum.\n");
				i2c_api->i2c_himax_master_write(client, type3_selected->checksum, sizeof(type3_selected->checksum), normalRetry);
			}

			/* Read Hardware Check Sum */
			i2c_api->i2c_himax_write_command(client, 0xAB, normalRetry);
			i2c_api->i2c_himax_read_command(client, 1, &Data, &act_len, normalRetry);
		}

		++retryTimes;
	/* Check Software and Hardware Check Sum */
	} while (Data != 0x10 && ((uint32_t)type1_checksum ^ (uint32_t)type2_checksum ^ (uint32_t)type3_checksum));

	/* Turn on reload disable */
	cmd[0] = 0x42; cmd[1] = 0x02;
	i2c_api->i2c_himax_master_write(client, cmd , 2, normalRetry);

	i2c_api->i2c_himax_write_command(client, 0x83, normalRetry);
	hr_msleep(100);

	return result;
}

static ssize_t primou_himax_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	":59:849:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	":240:849:90:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH)	":421:849:90:50"
		"\n");
}

static ssize_t primou_himax_virtual_keys_show_china_mfg(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	":65:849:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	":180:849:83:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH)	":310:849:88:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":445:849:91:50"
		"\n");
}

static ssize_t primou_himax_virtual_keys_show_china(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	":65:849:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	":180:849:83:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH)	":310:849:88:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_WEIBO)	":445:849:91:50"
		"\n");
}

static ssize_t primou_atmel_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)	":65:845:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":180:845:83:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)	":310:845:88:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":445:845:91:50"
		"\n");
}

static ssize_t primou_synaptics_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	":59:849:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	":240:849:90:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH)	":421:849:90:50"
		"\n");
}

static ssize_t primou_synaptics_virtual_keys_show_china_mfg(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	":65:849:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	":180:849:83:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH)	":310:849:88:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":445:849:91:50"
		"\n");
}

static ssize_t primou_synaptics_virtual_keys_show_china(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	":65:849:80:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)	":180:849:83:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH)	":310:849:88:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_WEIBO)	":445:849:91:50"
		"\n");
}

static struct kobj_attribute primou_himax_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.himax-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &primou_himax_virtual_keys_show,
};

static struct kobj_attribute primou_atmel_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &primou_atmel_virtual_keys_show,
};

static struct kobj_attribute primou_synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &primou_synaptics_virtual_keys_show,
};

static struct attribute *primou_properties_attrs[] = {
	&primou_atmel_virtual_keys_attr.attr,
	&primou_synaptics_virtual_keys_attr.attr,
	&primou_himax_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group primou_properties_attr_group = {
	.attrs = primou_properties_attrs,
};


static DEFINE_MUTEX(capella_cm36282_lock);
static int als_power_control;

static int __capella_cm36282_power(int on)
{
	int rc;
	struct vreg *vreg;

	printk(KERN_INFO "%s: system_rev %d\n", __func__, system_rev);

	vreg = vreg_get(0, "gp4");

	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2850);

	printk(KERN_DEBUG "%s: Turn the capella_cm36282 power %s\n",
		__func__, (on) ? "on" : "off");

	if (on) {
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
	}

	return rc;
}

static int capella_cm36282_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm36282_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm36282_power(1);
	else if (!on)
		ret = __capella_cm36282_power(0);

	mutex_unlock(&capella_cm36282_lock);
	return ret;
}

static struct cm3629_platform_data cm36282_pdata = {
	.model = CAPELLA_CM36282,
	.ps_select = CM3629_PS1_ONLY,
	.intr = PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_PS_INT_N),
	.levels = { 3, 5, 7, 75, 132, 2495, 4249, 5012, 5775, 65535},
	.golden_adc = 0xC0C,
	.power = capella_cm36282_power,
	.cm3629_slave_address = 0xC0>>1,
	.ps1_thd_set = 0x03,
	.ps1_thd_no_cal = 0xF1,
	.ps1_thd_with_cal = 0x03,
	.ps_calibration_rule = 1,
	.ps_conf1_val = CM3629_PS_DR_1_320 | CM3629_PS_IT_1_6T |
			CM3629_PS1_PERS_1,
	.ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
			CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
	.ps_conf3_val = CM3629_PS2_PROL_32,
};


static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(ATMEL_MXT224E_NAME, 0x94 >> 1),
		.platform_data = &primou_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(PRIMOU_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x40 >> 1),
		.platform_data = &primou_ts_synaptics_data,
		.irq = MSM_GPIO_TO_INT(PRIMOU_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(HIMAX8526A_NAME, 0x90>>1),
		.platform_data  = &himax_ts_data,
		.irq = MSM_GPIO_TO_INT(PRIMOU_GPIO_TP_ATT_N),
	},
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME, 0xC0 >> 1),
		.platform_data = &cm36282_pdata,
		.irq = MSM_GPIO_TO_INT(
		       PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_PS_INT_N))
	},
};
/*
static struct bma250_platform_data gsensor_platform_data = {
	.intr = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(
			PRIMOU_GPIO_GSENSOR_INT)),
	.chip_layout = 1,
};

static struct i2c_board_info i2c_Sensors_devices[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME, 0x30 >> 1),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_GSENSOR_INT)),
	},
};
*/

static int pm8058_gpios_init(void)
{
	int rc;
/*
	struct pm8xxx_gpio_init_info sdc4_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.output_value   = 0,
		},
	};

	struct pm8xxx_gpio_init_info haptics_enable = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.vin_sel        = 2,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
		},
	};

	struct pm8xxx_gpio_init_info hdmi_5V_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HDMI_5V_EN_V3),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_VPH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.output_value   = 0,
		},
	};
*/
	struct pm8xxx_gpio_init_info gpio15 = {
		PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_FLASH_EN),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 6,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};

	struct pm8xxx_gpio_init_info gpio18 = {
		PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_SPK_SD),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 6,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};

	struct pm8xxx_gpio_init_info gpio19 = {
		PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_AMP_EN),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 6,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};

	struct pm8xxx_gpio_init_info keypad_gpio = {
		PM8058_GPIO_PM_TO_SYS(0),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.out_strength   = PM_GPIO_STRENGTH_NO,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	struct pm8xxx_gpio_init_info sdcc_det = {
		PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_SDMC_CD_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
#endif
	struct pm8xxx_gpio_init_info psensor_gpio = {
	PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_PS_INT_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_NO,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};
	rc = pm8xxx_gpio_config(psensor_gpio.gpio, &psensor_gpio.config);
	if (rc) {
		pr_err("%s PRIMOU_GPIO_PS_INT_N config failed\n", __func__);
		return rc;
	} else
		pr_info("%s [cm3628][PS]PRIMOU_GPIO_PS_INT_N config ok\n", __func__);

	rc = pm8xxx_gpio_config(gpio15.gpio, &gpio15.config);
	if (rc) {
		pr_err("%s PRIMOU_GPIO_FLASH_EN config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config(gpio18.gpio, &gpio18.config);
	if (rc) {
		pr_err("%s PRIMOU_AUD_SPK_SD config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(gpio19.gpio, &gpio19.config);
	if (rc) {
		pr_err("%s PRIMOU_AUD_AMP_EN config failed\n", __func__);
		return rc;
	}
/*
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa() ||
						machine_is_msm7x30_fluid())
		hdmi_5V_en.gpio = PMIC_GPIO_HDMI_5V_EN_V2;
	else
		hdmi_5V_en.gpio = PMIC_GPIO_HDMI_5V_EN_V3;

	hdmi_5V_en.gpio = PM8058_GPIO_PM_TO_SYS(hdmi_5V_en.gpio);

	rc = pm8xxx_gpio_config(hdmi_5V_en.gpio, &hdmi_5V_en.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_HDMI_5V_EN config failed\n", __func__);
		return rc;
	}
*/

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	if (machine_is_msm7x30_fluid())
		sdcc_det.config.inv_int_pol = 1;

	rc = pm8xxx_gpio_config(sdcc_det.gpio, &sdcc_det.config);
	if (rc) {
		pr_err("%s PRIMOU_GPIO_SDMC_CD_N config failed\n", __func__);
		return rc;
	}
#endif

	keypad_gpio.gpio = PRIMOU_VOL_UP;
	pm8xxx_gpio_config(keypad_gpio.gpio, &keypad_gpio.config);
	keypad_gpio.gpio = PRIMOU_VOL_DN;
	pm8xxx_gpio_config(keypad_gpio.gpio, &keypad_gpio.config);
#if 0
	if (machine_is_msm7x30_fluid()) {
		/* Haptics gpio */
		rc = pm8xxx_gpio_config(haptics_enable.gpio,
						&haptics_enable.config);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
							haptics_enable.gpio);
			return rc;
		}
		/* SCD4 gpio */
		rc = pm8xxx_gpio_config(sdc4_en.gpio, &sdc4_en.config);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N config failed\n",
								 __func__);
			return rc;
		}
		rc = gpio_request(sdc4_en.gpio, "sdc4_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(sdc4_en.gpio, 0);
	}
#endif
	return 0;
}
/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &msm7x30_proccomm_regulator_data
	}
};
#endif

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(id - 1),
							&pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8xxx_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static const unsigned int fluid_keymap[] = {
	KEY(0, 0, KEY_7),
	KEY(0, 1, KEY_ENTER),
	KEY(0, 2, KEY_UP),
	/* drop (0,3) as it always shows up in pair with(0,2) */
	KEY(0, 4, KEY_DOWN),

	KEY(1, 0, KEY_CAMERA_SNAPSHOT),
	KEY(1, 1, KEY_SELECT),
	KEY(1, 2, KEY_1),
	KEY(1, 3, KEY_VOLUMEUP),
	KEY(1, 4, KEY_VOLUMEDOWN),
};

static const unsigned int surf_keymap[] = {
	KEY(0, 0, KEY_7),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_UP),
	KEY(0, 3, KEY_RIGHT),
	KEY(0, 4, KEY_ENTER),
	KEY(0, 5, KEY_L),
	KEY(0, 6, KEY_BACK),
	KEY(0, 7, KEY_M),

	KEY(1, 0, KEY_LEFT),
	KEY(1, 1, KEY_SEND),
	KEY(1, 2, KEY_1),
	KEY(1, 3, KEY_4),
	KEY(1, 4, KEY_CLEAR),
	KEY(1, 5, KEY_MSDOS),
	KEY(1, 6, KEY_SPACE),
	KEY(1, 7, KEY_COMMA),

	KEY(2, 0, KEY_6),
	KEY(2, 1, KEY_5),
	KEY(2, 2, KEY_8),
	KEY(2, 3, KEY_3),
	KEY(2, 4, KEY_NUMERIC_STAR),
	KEY(2, 5, KEY_UP),
	KEY(2, 6, KEY_DOWN), /* SYN */
	KEY(2, 7, KEY_LEFTSHIFT),

	KEY(3, 0, KEY_9),
	KEY(3, 1, KEY_NUMERIC_POUND),
	KEY(3, 2, KEY_0),
	KEY(3, 3, KEY_2),
	KEY(3, 4, KEY_SLEEP),
	KEY(3, 5, KEY_F1),
	KEY(3, 6, KEY_F2),
	KEY(3, 7, KEY_F3),

	KEY(4, 0, KEY_BACK),
	KEY(4, 1, KEY_HOME),
	KEY(4, 2, KEY_MENU),
	KEY(4, 3, KEY_VOLUMEUP),
	KEY(4, 4, KEY_VOLUMEDOWN),
	KEY(4, 5, KEY_F4),
	KEY(4, 6, KEY_F5),
	KEY(4, 7, KEY_F6),

	KEY(5, 0, KEY_R),
	KEY(5, 1, KEY_T),
	KEY(5, 2, KEY_Y),
	KEY(5, 3, KEY_LEFTALT),
	KEY(5, 4, KEY_KPENTER),
	KEY(5, 5, KEY_Q),
	KEY(5, 6, KEY_W),
	KEY(5, 7, KEY_E),

	KEY(6, 0, KEY_F),
	KEY(6, 1, KEY_G),
	KEY(6, 2, KEY_H),
	KEY(6, 3, KEY_CAPSLOCK),
	KEY(6, 4, KEY_PAGEUP),
	KEY(6, 5, KEY_A),
	KEY(6, 6, KEY_S),
	KEY(6, 7, KEY_D),

	KEY(7, 0, KEY_V),
	KEY(7, 1, KEY_B),
	KEY(7, 2, KEY_N),
	KEY(7, 3, KEY_MENU), /* REVISIT - SYM */
	KEY(7, 4, KEY_PAGEDOWN),
	KEY(7, 5, KEY_Z),
	KEY(7, 6, KEY_X),
	KEY(7, 7, KEY_C),

	KEY(8, 0, KEY_P),
	KEY(8, 1, KEY_J),
	KEY(8, 2, KEY_K),
	KEY(8, 3, KEY_INSERT),
	KEY(8, 4, KEY_LINEFEED),
	KEY(8, 5, KEY_U),
	KEY(8, 6, KEY_I),
	KEY(8, 7, KEY_O),

	KEY(9, 0, KEY_4),
	KEY(9, 1, KEY_5),
	KEY(9, 2, KEY_6),
	KEY(9, 3, KEY_7),
	KEY(9, 4, KEY_8),
	KEY(9, 5, KEY_1),
	KEY(9, 6, KEY_2),
	KEY(9, 7, KEY_3),

	KEY(10, 0, KEY_F7),
	KEY(10, 1, KEY_F8),
	KEY(10, 2, KEY_F9),
	KEY(10, 3, KEY_F10),
	KEY(10, 4, KEY_FN),
	KEY(10, 5, KEY_9),
	KEY(10, 6, KEY_0),
	KEY(10, 7, KEY_DOT),

	KEY(11, 0, KEY_LEFTCTRL),
	KEY(11, 1, KEY_F11),  /* START */
	KEY(11, 2, KEY_ENTER),
	KEY(11, 3, KEY_SEARCH),
	KEY(11, 4, KEY_DELETE),
	KEY(11, 5, KEY_RIGHT),
	KEY(11, 6, KEY_LEFT),
	KEY(11, 7, KEY_RIGHTSHIFT),
};

static struct matrix_keymap_data surf_keymap_data = {
	.keymap_size    = ARRAY_SIZE(surf_keymap),
	.keymap		= surf_keymap,
};

static struct pm8xxx_keypad_platform_data surf_keypad_data = {
	.input_name		= "surf_keypad",
	.input_phys_device	= "surf_keypad/input0",
	.num_rows		= 12,
	.num_cols		= 8,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 15,
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data		= &surf_keymap_data,
};

static struct matrix_keymap_data fluid_keymap_data = {
	.keymap_size	= ARRAY_SIZE(fluid_keymap),
	.keymap		= fluid_keymap,
};



static struct pm8xxx_keypad_platform_data fluid_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 5,
	.num_cols		= 5,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 15,
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data		= &fluid_keymap_data,
};

static struct pm8xxx_vibrator_platform_data pm8058_vib_pdata = {
       .initial_vibrate_ms  = 0,
       .level_mV = 3000,
       .max_timeout_ms = 15000,
};
static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config         = pm8058_pwm_config,
	.enable         = pm8058_pwm_enable,
};

static struct pmic8058_led pmic8058_ffa_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
};

static struct pmic8058_leds_platform_data pm8058_ffa_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_ffa_leds),
	.leds	= pmic8058_ffa_leds,
};

static struct pmic8058_led pmic8058_surf_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "voice:red",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_0,
	},
	[2] = {
		.name		= "wlan:green",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_2,
	},
};

static struct pmic8058_leds_platform_data pm8058_surf_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_surf_leds),
	.leds	= pmic8058_surf_leds,
};

static struct pmic8058_led pmic8058_fluid_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "flash:led_0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[2] = {
		.name		= "flash:led_1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_fluid_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_fluid_leds),
	.leds	= pmic8058_fluid_leds,
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base		= PMIC8058_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag       = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base		= PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base	= PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.pwm_pdata		= &pm8058_pwm_data,
	.vibrator_pdata		= &pm8058_vib_pdata,
};

#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8058-core",
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data		= &pm8058_7x30_data,
	},
	.rspinlock_name	= "D:PMIC_SSBI",
};
#endif

static struct tps65200_platform_data tps65200_data = {
	.gpio_chg_int = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_CHG_INT)),
};

static struct i2c_board_info i2c_tps_devices[] = {
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_S5K4E5YX
	{
		I2C_BOARD_INFO("s5k4e5yx", 0x20 >> 1),
	},
#endif
};

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
};

static uint32_t camera_on_gpio_table[] = {
};

#if defined(CONFIG_RAWCHIP) || defined(CONFIG_S5k4E5YX)
static int sensor_power_enable(char *power, unsigned volt)
{
	struct vreg *vreg_gp;
	int rc;

	if (power == NULL)
		return EIO;

	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("[CAM] %s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_set_level(vreg_gp, volt);
	if (rc) {
		pr_err("[CAM] %s: vreg wlan set %s level failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}

	rc = vreg_enable(vreg_gp);
	if (rc) {
		pr_err("[CAM] %s: vreg enable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int sensor_power_disable(char *power)
{
	struct vreg *vreg_gp;
	int rc;
	if (power == NULL)
		return -ENODEV;

	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("[CAM] %s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_disable(vreg_gp);
	if (rc) {
		pr_err("[CAM] %s: vreg disable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}
#endif


static int sensor_version = 999;
#ifdef CONFIG_S5K4E5YX
static int primou_sensor_version(void)
{
	if (sensor_version == 999) {
		if (gpio_get_value(PRIMOU_GPIO_CAM_ID) == 0) {
			sensor_version = PRIMOU_S5K4E5YX_EVT2;
		} else {
			sensor_version = PRIMOU_S5K4E5YX_EVT1;
		}
	}
	return sensor_version;
}
static int Primou_s5k4e5yx_vreg_on(void)
{
	int rc = 0;
	pr_info("[CAM] %s camera vreg on\n", __func__);
	/* PM8058 S5K4E5YX Power on Seq *
	 * 1. V_CAM_VCM2V85 VREG_L8	 VREG_GP7 *
	 * 2. V_CAM_D1V5	VREG_L11 VREG_GP2 *
	 * 3. V_CAM_A2V85	VREG_L12 VREG_GP9 *
	 * 4. V_CAMIO_1V8	use LDO enable pin GPIO99 */

	/* V_CAM_VCM2V85 */
	rc = sensor_power_enable("gp7", 2850);
	pr_info("[CAM] sensor_power_enable(\"gp7\", 2850) == %d\n", rc);

	if (primou_sensor_version() == PRIMOU_S5K4E5YX_EVT2) {
		/* EVT2 V_CAM_D1V5 */
		rc = sensor_power_enable("gp2", 1500);
		pr_info("[CAM] sensor_power_enable(\"gp2\", 1500) == %d\n", rc);
	} else {
		/* EVT1 V_CAM_D1V8 */
		rc = sensor_power_enable("gp2", 1800);
		pr_info("[CAM] sensor_power_enable(\"gp2\", 1800) == %d\n", rc);
	}

	/* V_CAM_A2V85 */
	rc = sensor_power_enable("gp9", 2850);
	pr_info("[CAM] sensor_power_enable(\"gp9\", 2850) == %d\n", rc);
	/* msleep(5); */

	/* V_CAMIO_1V8 */
	rc = gpio_request(99, "V_CAMIO_1V8");
	if (rc) {
		pr_err("[CAM] sensor_power_enable\
			(\"gpio %d\", 1.8V) FAILED %d\n",
			99, rc);
	} else {
		gpio_direction_output(99, 1);
		gpio_free(99);
	}
		mdelay(1);

	return rc;
}

static int Primou_s5k4e5yx_vreg_off(void)
{
	int rc;
	pr_info("[CAM] %s camera vreg off\n", __func__);
	/* PM8058 S5K4E5YX Power off Seq *
	 * 1. V_CAM_A2V85	VREG_L12 VREG_GP9 *
	 * 2. V_CAM_D1V5	VREG_L11 VREG_GP2 *
	 * 3. V_CAMIO_1V8	use LDO enable pin GPIO99 *
	 * 4. V_CAM_VCM2V85 VREG_L8	 VREG_GP7 */

	/* V_CAM_A2V85 */
	rc = sensor_power_disable("gp9");
	pr_info("[CAM] sensor_power_disable(\"gp9\") == %d\n", rc);

	mdelay(2);

	/* V_CAM_D1V5 V_CAM_D1V8 */
	rc = sensor_power_disable("gp2");
	pr_info("[CAM] sensor_power_disable(\"gp2\") == %d\n", rc);

	/* V_CAMIO_1V8 */
	rc = gpio_request(99, "V_CAMIO_1V8");
	if (rc) {
		pr_err("[CAM] sensor_power_disable\
			(\"gpio %d\", 1.8V) FAILED %d\n",
			99, rc);
	} else {
		gpio_direction_output(99, 0);
		gpio_free(99);
	}
	msleep(1);

	/* V_CAM_VCM2V85 */
	rc = sensor_power_disable("gp7");
	pr_info("[CAM] sensor_power_disable(\"gp7\") == %d\n", rc);


	return rc;
}
#endif

static int config_camera_on_gpios(void)
{
	pr_info("[CAM] config_camera_on_gpios\n");
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
	pr_info("[CAM] config_camera_off_gpios\n");
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

struct msm_camera_device_platform_data camera_device_data = {
  .camera_gpio_on  = config_camera_on_gpios,
  .camera_gpio_off = config_camera_off_gpios,
  .ioext.mdcphy = MSM_MDC_PHYS,
  .ioext.mdcsz  = MSM_MDC_SIZE,
  .ioext.appphy = MSM_CLK_CTL_PHYS,
  .ioext.appsz  = MSM_CLK_CTL_SIZE,
  .ioext.camifpadphy = 0xAB000000,
  .ioext.camifpadsz  = 0x00000400,
  .ioext.csiphy = 0xA6100000,
  .ioext.csisz  = 0x00000400,
  .ioext.csiirq = INT_CSI,
};

#if defined(CONFIG_ARCH_MSM_FLASHLIGHT) && defined(CONFIG_S5K4E5YX)
static int flashlight_control(int mode)
{
#ifdef CONFIG_FLASHLIGHT_TPS61310
	return tps61310_flashlight_control(mode);
#else
	return 0;
#endif
}
#endif
#if defined(CONFIG_ARCH_MSM_FLASHLIGHT) && defined(CONFIG_S5K4E5YX)
//HTC_START_Simon.Ti_Liu_20120209 linear led
/*
150 mA FL_MODE_FLASH_LEVEL1
200 mA FL_MODE_FLASH_LEVEL2
300 mA FL_MODE_FLASH_LEVEL3
400 mA FL_MODE_FLASH_LEVEL4
500 mA FL_MODE_FLASH_LEVEL5
600 mA FL_MODE_FLASH_LEVEL6
700 mA FL_MODE_FLASH_LEVEL7
*/
static struct camera_led_est msm_camera_sensor_s5k4e5yx_led_table[] = {
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL1,
		.current_ma = 150,
		.lumen_value = 150,
		.min_step = 50,
		.max_step = 70
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL2,
		.current_ma = 200,
		.lumen_value = 220,
		.min_step = 59,
		.max_step = 128
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL3,
		.current_ma = 300,
		.lumen_value = 320,
		.min_step = 54,
		.max_step = 58
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL4,
		.current_ma = 400,
		.lumen_value = 420,
		.min_step = 49,
		.max_step = 53
	},
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL5,
		.current_ma = 500,
		.lumen_value = 520,
		.min_step = 12,
		.max_step = 22
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL6,
		.current_ma = 600,
		.lumen_value = 620,
		.min_step = 41,
		.max_step = 48
	},
	/*
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL7,
		.current_ma = 700,
		.lumen_value = 700,
		.min_step = 21,
		.max_step = 22
	},
	*/
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,//740,//725,   //mk0118
		.min_step = 0,
		.max_step = 40
	},

		{
		.enable = 2,
		.led_state = FL_MODE_FLASH_LEVEL2,
		.current_ma = 200,
		.lumen_value = 220,//245,  //mk0127
		.min_step = 0,
		.max_step = 270
	},
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL3,
		.current_ma = 300,
		.lumen_value = 300,
		.min_step = 0,
		.max_step = 100
	},
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL4,
		.current_ma = 400,
		.lumen_value = 400,
		.min_step = 101,
		.max_step = 200
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL7,
		.current_ma = 700,
		.lumen_value = 700,
		.min_step = 101,
		.max_step = 200
	},
		{
		.enable = 2,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,//740,//725,
		.min_step = 271,
		.max_step = 315
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL5,
		.current_ma = 500,
		.lumen_value = 500,
		.min_step = 25,
		.max_step = 26
	},
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,//740,//725,
		.min_step = 271,
		.max_step = 315
	},
};

static struct camera_led_info msm_camera_sensor_s5k4e5yx_led_info = {
	.enable = 1,
	.low_limit_led_state = FL_MODE_TORCH,
	.max_led_current_ma = 750,
	.num_led_est_table = ARRAY_SIZE(msm_camera_sensor_s5k4e5yx_led_table),
};

static struct camera_flash_info msm_camera_sensor_s5k4e5yx_flash_info = {
	.led_info = &msm_camera_sensor_s5k4e5yx_led_info,
	.led_est_table = msm_camera_sensor_s5k4e5yx_led_table,
};

static struct camera_flash_cfg msm_camera_sensor_s5k4e5yx_flash_cfg = {
	.camera_flash = flashlight_control,
	.num_flash_levels		= FLASHLIGHT_NUM,
	.low_temp_limit			= 10,
	.low_cap_limit			= 15,
	.flash_info             = &msm_camera_sensor_s5k4e5yx_flash_info,
};
//HTC_END
#endif
#ifdef CONFIG_S5K4E5YX
static struct msm_camera_sensor_platform_info sensor_s5k4e5yx_board_info = {
    .mirror_flip = CAMERA_SENSOR_MIRROR_FLIP,
};
static struct msm_camera_sensor_info msm_camera_sensor_s5k4e5yx_data = {
	.sensor_name = "s5k4e5yx",
	.camera_power_on = Primou_s5k4e5yx_vreg_on,
	.camera_power_off = Primou_s5k4e5yx_vreg_off,
	.pdata = &camera_device_data,
	.sensor_pwd = PRIMOU_GPIO_CAM1_PWD,
	.vcm_pwd = PRIMOU_GPIO_CAM1_VCM_PWD,
	.flash_type = MSM_CAMERA_FLASH_LED,
	.sensor_platform_info = &sensor_s5k4e5yx_board_info,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.power_down_disable = false, /* true: disable pwd down function */

	.csi_if = 1,
	.dev_node = 0,
	.gpio_set_value_force = 1,/*use different method of gpio set value*/
	.use_rawchip = 1,
	.sensor_version = primou_sensor_version,
#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
	.flash_cfg = &msm_camera_sensor_s5k4e5yx_flash_cfg,
#endif
};

static struct platform_device msm_camera_sensor_s5k4e5yx = {
  .name = "msm_camera_s5k4e5yx",
  .dev = {
    .platform_data = &msm_camera_sensor_s5k4e5yx_data,
  },
};
#endif

#ifdef CONFIG_RAWCHIP
static int primou_use_ext_1v2(void)
{

		return 1;

}
static int primou_rawchip_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s rawchip vreg on\n", __func__);

	/* V_RAW_1V8 */
	rc = sensor_power_enable("wlan", 1800);
	pr_info("[CAM] rawchip_power_enable(\"wlan\", 1800) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_enable\
			(\"wlan\", 1.8V) FAILED %d\n", rc);
		goto enable_v_raw_1v8_fail;
	}
	mdelay(1);
	if (system_rev >= 2) {
		/* V_RAW_1V2_EN*/
		rc = gpio_request(PRIMOU_GPIO_RAW_1V2_EN, "V_RAW_1V2_EN");
		if (rc) {
			pr_err("[CAM] sensor_power_enable\
				(\"gpio %d\", 1.8V) FAILED %d\n",
				PRIMOU_GPIO_RAW_1V2_EN, rc);
		} else {
			gpio_direction_output(PRIMOU_GPIO_RAW_1V2_EN, 1);
			gpio_free(PRIMOU_GPIO_RAW_1V2_EN);
		}		
	} else {
		/* V_RAWCSI_1V2 */
		rc = sensor_power_enable("gp17", 1200);
		pr_info("[CAM] rawchip_power_enable(\"gp17\", 1200) == %d\n", rc);
		if (rc < 0) {
			pr_err("[CAM] rawchip_power_enable\
				(\"gp17\", 1.2V) FAILED %d\n", rc);
			goto enable_v_rawcsi_1v2_fail;
		}

		/* V_RAW_1V2 */
		rc = sensor_power_enable("gp5", 1200);
		pr_info("[CAM] rawchip_power_enable(\"gp5\", 1200) == %d\n", rc);
		if (rc < 0) {
			pr_err("[CAM] rawchip_power_enable\
				(\"gp5\", 1.2V) FAILED %d\n", rc);
			goto enable_v_raw_1v2_fail;
		}
	}

    return rc;
	if (system_rev >= 2) {
		enable_v_rawcsi_1v2_fail:
			sensor_power_disable("gp17");
		enable_v_raw_1v2_fail:
			sensor_power_disable("wlan");
	}
enable_v_raw_1v8_fail:
	return rc;
}

static int primou_rawchip_vreg_off(void)
{
	int rc = 1;
	pr_info("[CAM] %s rawchip vreg off\n", __func__);

	if (system_rev >= 2) {
		/* V_RAW_1V2_EN */
		rc = gpio_request(PRIMOU_GPIO_RAW_1V2_EN, "V_RAW_1V2_EN");
		if (rc) {
			pr_err("[CAM] sensor_power_disable\
				(\"gpio %d\", 1.8V) FAILED %d\n",
				PRIMOU_GPIO_RAW_1V2_EN, rc);
		} else {
			gpio_direction_output(PRIMOU_GPIO_RAW_1V2_EN, 0);
			gpio_free(PRIMOU_GPIO_RAW_1V2_EN);
		}		
	} else {
		/* V_RAW_1V2 */
		rc = sensor_power_disable("gp5");
		if (rc < 0)
			pr_err("[CAM] rawchip_power_disable\
				(\"gp5\") FAILED %d\n", rc);

		/* V_RAWCSI_1V2 */
		rc = sensor_power_disable("gp17");
		if (rc < 0)
			pr_err("[CAM] rawchip_power_disable\
				(\"gp17\") FAILED %d\n", rc);
		mdelay(1);
	}
	/* V_RAW_1V8 */
	rc = sensor_power_disable("wlan");
	if (rc < 0)
		pr_err("[CAM] rawchip_power_disable\
			(\"wlan\") FAILED %d\n", rc);

	return rc;
}

static uint32_t rawchip_on_gpio_table[] = {
	GPIO_CFG(PRIMOU_GPIO_RAW_RSTN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP Reset */
	GPIO_CFG(PRIMOU_GPIO_RAW_INTR0, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP INT0 */
	GPIO_CFG(PRIMOU_GPIO_RAW_INTR1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP INT1 */
	GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CS, 2 , GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
#if 0
	GPIO_CFG(PRIMOU_GPIO_CAM1_VCM_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMOU_GPIO_CAM1_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
	GPIO_CFG(PRIMOU_GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* MCLK */
	GPIO_CFG(PRIMOU_GPIO_CAM_I2C_SCL, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* I2C SCL*/
	GPIO_CFG(PRIMOU_GPIO_CAM_I2C_SDA, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* I2C SDA*/
	GPIO_CFG(PRIMOU_GPIO_RAW_RSTN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP Reset */
	GPIO_CFG(PRIMOU_GPIO_RAW_INTR0, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP INT0 */
	GPIO_CFG(PRIMOU_GPIO_RAW_INTR1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP INT1 */
};

static uint32_t rawchip_off_gpio_table[] = {
	GPIO_CFG(PRIMOU_GPIO_CAM1_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMOU_GPIO_CAM1_VCM_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMOU_GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* MCLK */
	GPIO_CFG(PRIMOU_GPIO_CAM_I2C_SCL, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA), /* I2C SCL*/
	GPIO_CFG(PRIMOU_GPIO_CAM_I2C_SDA, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* I2C SDA*/
	GPIO_CFG(PRIMOU_GPIO_CAM_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), /* MCLK */
	GPIO_CFG(PRIMOU_GPIO_RAW_RSTN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RAW CHIP Reset */
	GPIO_CFG(PRIMOU_GPIO_RAW_INTR0, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RAW CHIP INT0 */
	GPIO_CFG(PRIMOU_GPIO_RAW_INTR1, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RAW CHIP INT1 */
	GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CS, 0 , GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA),
};

static int config_rawchip_on_gpios(void)
{
	pr_info("[CAM] config_rawchip_on_gpios\n");
	config_gpio_table(rawchip_on_gpio_table,
		ARRAY_SIZE(rawchip_on_gpio_table));
	return 0;
}

static void config_rawchip_off_gpios(void)
{
	pr_info("[CAM] config_rawchip_off_gpios\n");
	config_gpio_table(rawchip_off_gpio_table,
		ARRAY_SIZE(rawchip_off_gpio_table));
}

static struct msm_camera_rawchip_info msm_rawchip_board_info = {
	.rawchip_reset	= PRIMOU_GPIO_RAW_RSTN,
	.rawchip_intr0	= PRIMOU_GPIO_RAW_INTR0,
	.rawchip_intr1	= PRIMOU_GPIO_RAW_INTR1,
	.rawchip_spi_freq = 27, /* MHz, should be the same to spi max_speed_hz */
	.rawchip_mclk_freq = 24, /* MHz, should be the same as cam csi0 mclk_clk_rate */
	.camera_rawchip_power_on = primou_rawchip_vreg_on,
	.camera_rawchip_power_off = primou_rawchip_vreg_off,
	.rawchip_gpio_on = config_rawchip_on_gpios,
	.rawchip_gpio_off = config_rawchip_off_gpios,
	.rawchip_use_ext_1v2 = primou_use_ext_1v2,
};

static struct platform_device msm_rawchip_device = {
	.name	= "rawchip",
	.dev	= {
		.platform_data = &msm_rawchip_board_info,
	},
};
#endif




#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#endif /*CONFIG_MSM_CAMERA*/

#ifdef CONFIG_MSM7KV2_AUDIO

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static struct tpa2051d3_platform_data tpa2051d3_platform_data = {
	.gpio_tpa2051_spk_en = PRIMOU_AUD_SPK_SD,
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	if (machine_is_msm8x60_fluid())
		pm8058_7x30_data.keypad_pdata = &fluid_keypad_data;
	else
		pm8058_7x30_data.keypad_pdata = &surf_keypad_data;

	return 0;
}

#define TIMPANI_RESET_GPIO	1

struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};


/* static struct vreg *vreg_marimba_1; */
static struct vreg *vreg_marimba_2;

static struct msm_gpio timpani_reset_gpio_cfg[] = {
{ GPIO_CFG(TIMPANI_RESET_GPIO, 0, GPIO_CFG_OUTPUT,
	GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "timpani_reset"} };

static int config_timpani_reset(void)
{
	int rc;

	rc = msm_gpios_request_enable(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_gpios_request_enable failed (%d)\n",
				__func__, rc);
	}
	return rc;
}

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	rc = config_timpani_reset();
	if (rc < 0)
		goto out;
#if 0
	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		goto out;
	}
#endif
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
	/*	goto fail_disable_vreg_marimba_1; */
	}

	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 1);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
		msm_gpios_free(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
		vreg_disable(vreg_marimba_2);
	} else
		goto out;

#if 0
fail_disable_vreg_marimba_1:
	vreg_disable(vreg_marimba_1);
#endif
out:
	return rc;
};

static void msm_timpani_shutdown_power(void)
{
	int rc;
#if 0
	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
#endif
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}

	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 0);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
	}

	msm_gpios_free(timpani_reset_gpio_cfg,
				   ARRAY_SIZE(timpani_reset_gpio_cfg));
};

static struct msm_gpio marimba_svlte_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"MARIMBA_SVLTE_CLOCK_ENABLE" },
};

static unsigned int msm_marimba_gpio_config_svlte(int gpio_cfg_marimba)
{
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		if (gpio_cfg_marimba)
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 1);
		else
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 0);
	}

	return 0;
};

static unsigned int msm_marimba_setup_power(void)
{
	int rc;
#if 0
	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
#endif
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = msm_gpios_request_enable(marimba_svlte_config_clock,
				ARRAY_SIZE(marimba_svlte_config_clock));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: msm_gpios_request_enable failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = gpio_direction_output(GPIO_PIN
			(marimba_svlte_config_clock->gpio_cfg), 0);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: gpio_direction_output failed (%d)\n",
					__func__, rc);
			return rc;
		}
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;
#if 0
	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
#endif
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

static int bahama_present(void)
{
	int id;
	switch (id = adie_get_detected_connectivity_type()) {
	case BAHAMA_ID:
		return 1;

	case MARIMBA_ID:
		return 0;

	case TIMPANI_ID:
	default:
	printk(KERN_ERR "%s: unexpected adie connectivity type: %d\n",
			__func__, id);
	return -ENODEV;
	}
}

struct vreg *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc;
	uint32_t irqcfg;
	const char *id = "FMPW";

	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return -ENODEV;
	}
	if (bahama_not_marimba)
		fm_regulator = vreg_get(NULL, "s3");
	else
		fm_regulator = vreg_get(NULL, "s2");

	if (IS_ERR(fm_regulator)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(fm_regulator));
		return -1;
	}
	if (!bahama_not_marimba) {

		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);

		if (rc < 0) {
			printk(KERN_ERR "%s: voltage level vote failed (%d)\n",
				__func__, rc);
			return rc;
		}
	}
	rc = vreg_enable(fm_regulator);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		return rc;
	}

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		printk(KERN_ERR "%s: clock vote failed (%d)\n",
			__func__, rc);
		goto fm_clock_vote_fail;
	}
	/*Request the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa())	{
		rc = marimba_gpio_config(1);
		if (rc < 0)
			printk(KERN_ERR "%s: clock enable for svlte : %d\n",
						__func__, rc);
	}
	irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
		rc = -EIO;
		goto fm_gpio_config_fail;

	}
	return 0;
fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
				  PMAPP_CLOCK_VOTE_OFF);
fm_clock_vote_fail:
	vreg_disable(fm_regulator);
	return rc;

};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA);

	int bahama_not_marimba = bahama_present();
	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
			__func__, bahama_not_marimba);
		return;
	}

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
	}
	if (fm_regulator != NULL) {
		rc = vreg_disable(fm_regulator);

		if (rc) {
			printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
		}
		fm_regulator = NULL;
	}
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		printk(KERN_ERR "%s: clock_vote return val: %d\n",
						__func__, rc);

	/*Disable the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa())	{
		rc = marimba_gpio_config(0);
		if (rc < 0)
			printk(KERN_ERR "%s: clock disable for svlte : %d\n",
						__func__, rc);
	}


	if (!bahama_not_marimba)	{
		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);

		if (rc < 0)
			printk(KERN_ERR "%s: vreg level vote return val: %d\n",
						__func__, rc);
	}
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup =  fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(147),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	.is_fm_soc_i2s_master = false,
	.config_i2s_gpio = NULL,
};


/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

static const char *tsadc_id = "MADC";
static const char *vregs_tsadc_name[] = {
	"gp12",
	"s2",
};
static struct vreg *vregs_tsadc[ARRAY_SIZE(vregs_tsadc_name)];

static const char *vregs_timpani_tsadc_name[] = {
	"s3",
	"gp12",
	"gp16"
};
static struct vreg *vregs_timpani_tsadc[ARRAY_SIZE(vregs_timpani_tsadc_name)];

static int marimba_tsadc_power(int vreg_on)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			if (!vregs_timpani_tsadc[i]) {
				pr_err("%s: vreg_get %s failed(%d)\n",
				__func__, vregs_timpani_tsadc_name[i], rc);
				goto vreg_fail;
			}

			rc = vreg_on ? vreg_enable(vregs_timpani_tsadc[i]) :
				  vreg_disable(vregs_timpani_tsadc[i]);
			if (rc < 0) {
				pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, vregs_timpani_tsadc_name[i],
				       vreg_on ? "enable" : "disable", rc);
				goto vreg_fail;
			}
		}
		/* Vote for D0 and D1 buffer */
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_D1,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			if (!vregs_tsadc[i]) {
				pr_err("%s: vreg_get %s failed (%d)\n",
					__func__, vregs_tsadc_name[i], rc);
				goto vreg_fail;
			}

			rc = vreg_on ? vreg_enable(vregs_tsadc[i]) :
				  vreg_disable(vregs_tsadc[i]);
			if (rc < 0) {
				pr_err("%s: vreg %s %s failed (%d)\n",
					__func__, vregs_tsadc_name[i],
				       vreg_on ? "enable" : "disable", rc);
				goto vreg_fail;
			}
		}
		/* If marimba vote for DO buffer */
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d0 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	msleep(5); /* ensure power is stable */

	return 0;

do_vote_fail:
vreg_fail:
	while (i) {
		if (vreg_on) {
			if (tsadc_adie_type == TIMPANI_ID)
				vreg_disable(vregs_timpani_tsadc[--i]);
			else if (tsadc_adie_type == MARIMBA_ID)
				vreg_disable(vregs_tsadc[--i]);
		} else {
			if (tsadc_adie_type == TIMPANI_ID)
				vreg_enable(vregs_timpani_tsadc[--i]);
			else if (tsadc_adie_type == MARIMBA_ID)
				vreg_enable(vregs_tsadc[--i]);
		}
	}

	return rc;
}

static int marimba_tsadc_vote(int vote_on)
{
	int rc = 0;

	if (adie_get_detected_codec_type() == MARIMBA_ID) {
		int level = vote_on ? 1300 : 0;
		rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, level);
		if (rc < 0)
			pr_err("%s: vreg level %s failed (%d)\n",
			__func__, vote_on ? "on" : "off", rc);
	}

	return rc;
}

static int marimba_tsadc_init(void)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			vregs_timpani_tsadc[i] = vreg_get(NULL,
						vregs_timpani_tsadc_name[i]);
			if (IS_ERR(vregs_timpani_tsadc[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_timpani_tsadc_name[i],
				       PTR_ERR(vregs_timpani_tsadc[i]));
				rc = PTR_ERR(vregs_timpani_tsadc[i]);
				goto vreg_get_fail;
			}
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			vregs_tsadc[i] = vreg_get(NULL, vregs_tsadc_name[i]);
			if (IS_ERR(vregs_tsadc[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_tsadc_name[i],
				       PTR_ERR(vregs_tsadc[i]));
				rc = PTR_ERR(vregs_tsadc[i]);
				goto vreg_get_fail;
			}
		}
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	return 0;

vreg_get_fail:
	while (i) {
		if (tsadc_adie_type == TIMPANI_ID)
			vreg_put(vregs_timpani_tsadc[--i]);
		else if (tsadc_adie_type == MARIMBA_ID)
			vreg_put(vregs_tsadc[--i]);
	}
	return rc;
}

static int marimba_tsadc_exit(void)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			if (vregs_tsadc[i])
				vreg_put(vregs_timpani_tsadc[i]);
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			if (vregs_tsadc[i])
				vreg_put(vregs_tsadc[i]);
		}
		rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, 0);
		if (rc < 0)
			pr_err("%s: vreg level off failed (%d)\n",
						__func__, rc);
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		rc = -ENODEV;
	}

	return rc;
}


static struct msm_ts_platform_data msm_ts_data = {
	.min_x          = 0,
	.max_x          = 4096,
	.min_y          = 0,
	.max_y          = 4096,
	.min_press      = 0,
	.max_press      = 255,
	.inv_x          = 4096,
	.inv_y          = 4096,
	.can_wakeup	= false,
};

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power =  marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.level_vote	     =  marimba_tsadc_vote,
	.tsadc_prechg_en = true,
	.can_wakeup	= false,
	.setup = {
		.pen_irq_en	=	true,
		.tsadc_en	=	true,
	},
	.params2 = {
		.input_clk_khz		=	2400,
		.sample_prd		=	TSADC_CLK_3,
	},
	.params3 = {
		.prechg_time_nsecs	=	6400,
		.stable_time_nsecs	=	6400,
		.tsadc_test_mode	=	0,
	},
	.tssc_data = &msm_ts_data,
};

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
};

static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
	.fm = &marimba_fm_pdata,
	.codec = &mariba_codec_pdata,
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

static void __init primou_init_marimba(void)
{
#if 0
	vreg_marimba_1 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
#endif
	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_2));
		return;
	}
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
};

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
	.tsadc = &marimba_tsadc_pdata,
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

static struct i2c_board_info tpa2051_devices[] = {
	{
		I2C_BOARD_INFO(TPA2051D3_I2C_NAME, 0xE0 >> 1),
		.platform_data = &tpa2051d3_platform_data,
	},
};
#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device htc_drm = {
	.name = "htcdrm",
	.id = 0,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};
#if 0
static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x8A000300,
		.end = 0x8A0003ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(156),
		.end = MSM_GPIO_TO_INT(156),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start		= 0x8D000000,
		.end		= 0x8D000100,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= MSM_GPIO_TO_INT(88),
		.end		= MSM_GPIO_TO_INT(88),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
    { GPIO_CFG(172, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr6" },
    { GPIO_CFG(173, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr5" },
    { GPIO_CFG(174, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr4" },
    { GPIO_CFG(175, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr3" },
    { GPIO_CFG(176, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr2" },
    { GPIO_CFG(177, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr1" },
    { GPIO_CFG(178, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr0" },
    { GPIO_CFG(88, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "smsc911x_irq"  },
};


static void msm7x30_cfg_smsc911x(void)
{
	int rc;

	rc = msm_gpios_request_enable(smsc911x_gpios,
			ARRAY_SIZE(smsc911x_gpios));
	if (rc)
		pr_err("%s: unable to enable gpios\n", __func__);
}
#endif

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
	.product_id	= 0x0ce5,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.fserial_init_string = "tty:modem,tty:autobot,tty:serial,tty:autobot",
	.nluns = 2,
	.usb_id_pin_gpio = PRIMOU_GPIO_USB_ID1_PIN,
	.req_reset_during_switch_func = 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

#endif
#if 0

static struct msm_gpio optnav_config_data[] = {
	{ GPIO_CFG(OPTNAV_CHIP_SELECT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	"optnav_chip_select" },
};

static void __iomem *virtual_optnav;

static int optnav_gpio_setup(void)
{
	int rc = -ENODEV;
	rc = msm_gpios_request_enable(optnav_config_data,
			ARRAY_SIZE(optnav_config_data));

	/* Configure the FPGA for GPIOs */
	virtual_optnav = ioremap(FPGA_OPTNAV_GPIO_ADDR, 0x4);
	if (!virtual_optnav) {
		pr_err("%s:Could not ioremap region\n", __func__);
		return -ENOMEM;
	}
	/*
	 * Configure the FPGA to set GPIO 19 as
	 * normal, active(enabled), output(MSM to SURF)
	 */
	writew(0x311E, virtual_optnav);
	return rc;
}

static void optnav_gpio_release(void)
{
	msm_gpios_disable_free(optnav_config_data,
		ARRAY_SIZE(optnav_config_data));
	iounmap(virtual_optnav);
}

static struct vreg *vreg_gp7;
static struct vreg *vreg_gp4;
static struct vreg *vreg_gp9;
static struct vreg *vreg_usb3_3;

static int optnav_enable(void)
{
	int rc;
	/*
	 * Enable the VREGs L8(gp7), L10(gp4), L12(gp9), L6(usb)
	 * for I2C communication with keyboard.
	 */
	vreg_gp7 = vreg_get(NULL, "gp7");
	rc = vreg_set_level(vreg_gp7, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp7;
	}

	rc = vreg_enable(vreg_gp7);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp7;
	}

	vreg_gp4 = vreg_get(NULL, "gp4");
	rc = vreg_set_level(vreg_gp4, 2600);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp4;
	}

	rc = vreg_enable(vreg_gp4);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp4;
	}

	vreg_gp9 = vreg_get(NULL, "gp9");
	rc = vreg_set_level(vreg_gp9, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp9;
	}

	rc = vreg_enable(vreg_gp9);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp9;
	}

	vreg_usb3_3 = vreg_get(NULL, "usb");
	rc = vreg_set_level(vreg_usb3_3, 3300);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_3_3;
	}

	rc = vreg_enable(vreg_usb3_3);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_3_3;
	}

	/* Enable the chip select GPIO */
	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
	gpio_set_value(OPTNAV_CHIP_SELECT, 0);

	return 0;

fail_vreg_3_3:
	vreg_disable(vreg_gp9);
fail_vreg_gp9:
	vreg_disable(vreg_gp4);
fail_vreg_gp4:
	vreg_disable(vreg_gp7);
fail_vreg_gp7:
	return rc;
}

static void optnav_disable(void)
{
	vreg_disable(vreg_usb3_3);
	vreg_disable(vreg_gp9);
	vreg_disable(vreg_gp4);
	vreg_disable(vreg_gp7);

	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
}

static struct ofn_atlab_platform_data optnav_data = {
	.gpio_setup    = optnav_gpio_setup,
	.gpio_release  = optnav_gpio_release,
	.optnav_on     = optnav_enable,
	.optnav_off    = optnav_disable,
	.rotate_xy     = 0,
	.function1 = {
		.no_motion1_en		= true,
		.touch_sensor_en	= true,
		.ofn_en			= true,
		.clock_select_khz	= 1500,
		.cpi_selection		= 1200,
	},
	.function2 =  {
		.invert_y		= false,
		.invert_x		= true,
		.swap_x_y		= false,
		.hold_a_b_en		= true,
		.motion_filter_en       = true,
	},
};
#endif
#ifdef CONFIG_BOSCH_BMA150
static struct vreg *vreg_gp6;
static int sensors_ldo_enable(void)
{
	int rc;

	/*
	 * Enable the VREGs L8(gp7), L15(gp6)
	 * for I2C communication with sensors.
	 */
	pr_info("sensors_ldo_enable called!!\n");
	vreg_gp7 = vreg_get(NULL, "gp7");
	if (IS_ERR(vreg_gp7)) {
		pr_err("%s: vreg_get gp7 failed\n", __func__);
		rc = PTR_ERR(vreg_gp7);
		goto fail_gp7_get;
	}

	rc = vreg_set_level(vreg_gp7, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level gp7 failed\n", __func__);
		goto fail_gp7_level;
	}

	rc = vreg_enable(vreg_gp7);
	if (rc) {
		pr_err("%s: vreg_enable gp7 failed\n", __func__);
		goto fail_gp7_level;
	}

	vreg_gp6 = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg_gp6)) {
		pr_err("%s: vreg_get gp6 failed\n", __func__);
		rc = PTR_ERR(vreg_gp6);
		goto fail_gp6_get;
	}

	rc = vreg_set_level(vreg_gp6, 3050);
	if (rc) {
		pr_err("%s: vreg_set_level gp6 failed\n", __func__);
		goto fail_gp6_level;
	}

	rc = vreg_enable(vreg_gp6);
	if (rc) {
		pr_err("%s: vreg_enable gp6 failed\n", __func__);
		goto fail_gp6_level;
	}

	return 0;

fail_gp6_level:
	vreg_put(vreg_gp6);
fail_gp6_get:
	vreg_disable(vreg_gp7);
fail_gp7_level:
	vreg_put(vreg_gp7);
fail_gp7_get:
	return rc;
}

static void sensors_ldo_disable(void)
{
	pr_info("sensors_ldo_disable called!!\n");
	vreg_disable(vreg_gp6);
	vreg_put(vreg_gp6);
	vreg_disable(vreg_gp7);
	vreg_put(vreg_gp7);
}
static struct bma150_platform_data bma150_data = {
	.power_on = sensors_ldo_enable,
	.power_off = sensors_ldo_disable,
};

/*
static struct i2c_board_info bma150_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("bma150", 0x38),
		.flags = I2C_CLIENT_WAKE,
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO_INT),
		.platform_data = &bma150_data,
	},
};
*/
#endif

static struct bma250_platform_data gsensor_bma250_platform_data = {
	.intr = PRIMOU_GPIO_GSENSOR_INT,
	.chip_layout = 0,
	.layouts = PRIMOU_LAYOUTS,
};

static struct i2c_board_info i2c_bma250_devices[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME_REMOVE_ECOMPASS, \
				0x30 >> 1),
		.platform_data = &gsensor_bma250_platform_data,
		.irq = MSM_GPIO_TO_INT(PRIMOU_GPIO_GSENSOR_INT),
	},
};

/*
static struct i2c_board_info msm_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("m33c01", OPTNAV_I2C_SLAVE_ADDR),
		.irq		= MSM_GPIO_TO_INT(OPTNAV_IRQ),
		.platform_data = &optnav_data,
	},
};
*/
static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

#ifdef CONFIG_RAWCHIP
static uint32_t msm_spi_on_gpio[] = {
       GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_DO, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
       GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_DI, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA),
       GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CS, 2 /*3*/, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
       GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};
#endif

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_APPS_SLEEP] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
/*
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = {
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.idle_supported = 0,
		.suspend_supported = 0,
		.idle_enabled = 0,
		.suspend_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

static int msm_qsd_spi_dma_config(void)
{
	return -ENOENT;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

#if defined(CONFIG_RAWCHIP)
static struct spi_board_info spi_rawchip_board_info[] __initdata = {
	{
		.modalias	= "spi_rawchip",
		.mode           = SPI_MODE_0,
		.bus_num        = 0,
		.chip_select    = 3,
		.max_speed_hz   = 26331429,
	}
};
#endif

static uint32_t qsd_spi_gpio_on_table[] = {
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_CFG_4MA),
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_DO, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_CFG_4MA),
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_DI, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_CFG_4MA),
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_CFG_4MA)
};

static uint32_t qsd_spi_gpio_off_table[] = {
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_CFG_4MA),
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_DO, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_CFG_4MA),
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_DI, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_CFG_4MA),
	PCOM_GPIO_CFG(PRIMOU_GPIO_MCAM_SPI_CS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_CFG_4MA)
};

static int msm_qsd_spi_gpio_config(void)
{
	config_gpio_table(qsd_spi_gpio_on_table,
		ARRAY_SIZE(qsd_spi_gpio_on_table));
	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	config_gpio_table(qsd_spi_gpio_off_table,
		ARRAY_SIZE(qsd_spi_gpio_off_table));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc;
	static int vbus_is_on;
	struct pm8xxx_gpio_init_info usb_vbus = {
		PM8058_GPIO_PM_TO_SYS(36),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
		return;

	if (on) {
		rc = pm8xxx_gpio_config(usb_vbus.gpio, &usb_vbus.config);
		if (rc) {
			pr_err("%s PMIC GPIO 36 write failed\n", __func__);
			return;
		}
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
		.phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
		.vbus_power = msm_hsusb_vbus_power,
		.power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif

static int phy_init_seq[] = { 0x06, 0x36, 0x0C, 0x31, 0x31, 0x32, 0x1, 0x0D, 0x1, 0x10, -1 };
static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_init_seq		= phy_init_seq,
	.mode			= USB_PERIPHERAL,
	.otg_control		= OTG_PMIC_CONTROL,
	.power_budget		= 750,
	.phy_type = CI_45NM_INTEGRATED_PHY,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

#if defined(CONFIG_FB_MSM_HDMI_ADV7520_PANEL) || defined(CONFIG_BOSCH_BMA150)
/* there is an i2c address conflict between adv7520 and bma150 sensor after
 * power up on fluid. As a solution, the default address of adv7520's packet
 * memory is changed as soon as possible
 */
static int __init fluid_i2c_address_fixup(void)
{
	unsigned char wBuff[16];
	unsigned char rBuff[16];
	struct i2c_msg msgs[3];
	int res;
	int rc = -EINVAL;
	struct vreg *vreg_ldo8;
	struct i2c_adapter *adapter;

	if (machine_is_msm7x30_fluid()) {
		adapter = i2c_get_adapter(0);
		if (!adapter) {
			pr_err("%s: invalid i2c adapter\n", __func__);
			return PTR_ERR(adapter);
		}

		/* turn on LDO8 */
		vreg_ldo8 = vreg_get(NULL, "gp7");
		if (!vreg_ldo8) {
			pr_err("%s: VREG L8 get failed\n", __func__);
			goto adapter_put;
		}

		rc = vreg_set_level(vreg_ldo8, 1800);
		if (rc) {
			pr_err("%s: VREG L8 set failed\n", __func__);
			goto ldo8_put;
		}

		rc = vreg_enable(vreg_ldo8);
		if (rc) {
			pr_err("%s: VREG L8 enable failed\n", __func__);
			goto ldo8_put;
		}

		/* change packet memory address to 0x74 */
		wBuff[0] = 0x45;
		wBuff[1] = 0x74;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 2;

		res = i2c_transfer(adapter, msgs, 1);
		if (res != 1) {
			pr_err("%s: error writing adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* powerdown adv7520 using bit 6 */
		/* i2c read first */
		wBuff[0] = 0x41;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 1;

		msgs[1].addr = ADV7520_I2C_ADDR;
		msgs[1].flags = I2C_M_RD;
		msgs[1].buf = rBuff;
		msgs[1].len = 1;
		res = i2c_transfer(adapter, msgs, 2);
		if (res != 2) {
			pr_err("%s: error reading adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* i2c write back */
		wBuff[0] = 0x41;
		wBuff[1] = rBuff[0] | 0x40;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 2;

		res = i2c_transfer(adapter, msgs, 1);
		if (res != 1) {
			pr_err("%s: error writing adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* for successful fixup, we release the i2c adapter */
		/* but leave ldo8 on so that the adv7520 is not repowered */
		i2c_put_adapter(adapter);
		pr_info("%s: fluid i2c address conflict resolved\n", __func__);
	}
	return 0;

ldo8_disable:
	vreg_disable(vreg_ldo8);
ldo8_put:
	vreg_put(vreg_ldo8);
adapter_put:
	i2c_put_adapter(adapter);
	return rc;
}
fs_initcall_sync(fluid_i2c_address_fixup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_adsp2_pdata = {
	.name = "pmem_adsp2",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_adsp2_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_adsp2_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1
#define QCE_SHA_HMAC_SUPPORT	0
#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};

#endif
static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = SWITCH_CHARGER_TPS65200,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id   = -1,
	.dev  = {
		.platform_data = &htc_battery_pdev_data,
	},
};

#ifdef CONFIG_SUPPORT_DQ_BATTERY
static int __init check_dq_setup(char *str)
{
	if (!strcmp(str, "PASS"))
		tps65200_data.dq_result = 1;
	else
		tps65200_data.dq_result = 0;

	return 1;
}
__setup("androidboot.dq=", check_dq_setup);
#endif

static char *msm_adc_fluid_device_names[] = {
	"LTC_ADC1",
	"LTC_ADC2",
	"LTC_ADC3",
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm BT */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = PRIMOU_GPIO_BT_WAKE,
	.host_wakeup_pin = PRIMOU_GPIO_BT_HOST_WAKE,
};
#endif

#ifdef CONFIG_BT
static struct platform_device primou_rfkill = {
	.name = "primou_rfkill",
	.id = -1,
};
#endif

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};


static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};
#endif /* CONFIG_MSM_SDIO_AL */

static struct resource ram_console_resources[] = {
	{
		.start  = MSM_RAM_CONSOLE_BASE,
		.end    = MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};

/* HEADSET DRIVER BEGIN */

#define HEADSET_DETECT		PRIMOU_GPIO_35MM_HEADSET_DET
#define HEADSET_BUTTON		PRIMOU_AUD_REMO_PRES

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= HEADSET_DETECT,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.driver_flag		= DRIVER_HS_PMIC_RPC_KEY,
	.hpin_gpio		= 0,
	.hpin_irq		= 0,
	.key_gpio		= PM8058_GPIO_PM_TO_SYS(HEADSET_BUTTON),
	.key_irq		= 0,
	.key_enable_gpio	= 0,
	.adc_mic		= 14894,
	.adc_remote		= {0, 3441, 3442, 5855, 5856, 12600},
	.hs_controller		= 0,
	.hs_switch		= 0,
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_pmic,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct headset_adc_config htc_headset_mgr_config[] = {
	{
		.type = HEADSET_MIC,
		.adc_max = 58318,
		.adc_min = 45532,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 45531,
		.adc_min = 33396,
	},
	{
		.type = HEADSET_BEATS_SOLO,
		.adc_max = 33395,
		.adc_min = 23466,
	},
	{
		.type = HEADSET_NO_MIC, /* HEADSET_INDICATOR */
		.adc_max = 23465,
		.adc_min = 9045,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 9044,
		.adc_min = 0,
	},
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.driver_flag		= DRIVER_HS_MGR_FLOAT_DET,
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config),
	.headset_config		= htc_headset_mgr_config,
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

/* HEADSET DRIVER END */

#ifdef CONFIG_FLASHLIGHT_TPS61310
static void config_flashlight_gpios_tps61310(void)
{
	static uint32_t flashlight_gpio_table[] = {
		GPIO_CFG(PRIMOU_GPIO_TORCH_EN, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_CFG_2MA),
	};
	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));

}


static struct TPS61310_flashlight_platform_data primou_flashlight_data = {
	.gpio_init = config_flashlight_gpios_tps61310,
	.tps61310_strb1 = PRIMOU_GPIO_TORCH_EN,
	.tps61310_strb0 = PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_FLASH_EN),
	.flash_duration_ms = 600,
	.led_count = 1,
};

static struct i2c_board_info primou_flashlight[] = {
{
	I2C_BOARD_INFO("TPS61310_FLASHLIGHT", 0x66 >> 1),
	.platform_data	= &primou_flashlight_data,
	},
};
#endif

static struct pm8058_led_config pm_led_config[] = {
        {
                .name = "amber",
                .type = PM8058_LED_DRVX,
                .bank = 4,
                .flags = PM8058_LED_BLINK_EN,
                .out_current = 25,
        },
        {
                .name = "green",
                .type = PM8058_LED_DRVX,
                .bank = 5,
                .flags = PM8058_LED_BLINK_EN,
                .out_current = 35,
        },
        {
                .name = "button-backlight",
                .type = PM8058_LED_DRVX,
                .bank = 6,
                .flags = PM8058_LED_LTU_EN,
                .period_us = USEC_PER_SEC / 1000,
                .start_index = 0,
                .duites_size = 8,
                .duty_time_ms = 32,
                .lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
                .out_current = 18,
        },
};


static struct pm8058_led_platform_data pm8058_leds_data = {
        .led_config = pm_led_config,
        .num_leds = ARRAY_SIZE(pm_led_config),
        .duties = {0, 15, 30, 45, 60, 75, 90, 100,
                   100, 90, 75, 60, 45, 30, 15, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0},
};

static struct platform_device pm8058_leds = {
        .name   = "leds-pm8058",
        .id     = -1,
        .dev    = {
                .platform_data  = &pm8058_leds_data,
        },
};

static struct pm8058_led_config pm_led_config_CH[] = {
        {
                .name = "amber",
                .type = PM8058_LED_DRVX,
                .bank = 4,
                .flags = PM8058_LED_BLINK_EN,
                .out_current = 25,
        },
        {
                .name = "green",
                .type = PM8058_LED_DRVX,
                .bank = 5,
                .flags = PM8058_LED_BLINK_EN,
                .out_current = 35,
        },
        {
                .name = "button-backlight",
                .type = PM8058_LED_DRVX,
                .bank = 6,
                .flags = PM8058_LED_LTU_EN,
                .period_us = USEC_PER_SEC / 1000,
                .start_index = 0,
                .duites_size = 8,
                .duty_time_ms = 32,
                .lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
                .out_current = 24,
        },
};

static struct pm8058_led_platform_data pm8058_leds_data_CH = {
        .led_config = pm_led_config_CH,
        .num_leds = ARRAY_SIZE(pm_led_config_CH),
        .duties = {0, 15, 30, 45, 60, 75, 90, 100,
                   100, 90, 75, 60, 45, 30, 15, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0},
};
static struct platform_device pm8058_leds_CH = {
        .name   = "leds-pm8058",
        .id     = -1,
        .dev    = {
                .platform_data  = &pm8058_leds_data_CH,
        },
};


#define PM8058ADC_16BIT(adc) ((adc * 2200) / 65535) /* vref=2.2v, 16-bits resolution */
int64_t primou_get_usbid_adc(void)
{
	uint32_t adc_value = 0xffffffff;
	htc_get_usb_accessory_adc_level(&adc_value);
	adc_value = PM8058ADC_16BIT(adc_value);
	return adc_value;
}

static uint32_t usb_ID_PIN_input_table[] = {
	GPIO_CFG(PRIMOU_GPIO_USB_ID1_PIN, 0, GPIO_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	GPIO_CFG(PRIMOU_GPIO_USB_ID1_PIN, 0, GPIO_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

void config_primou_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(PRIMOU_GPIO_USB_ID1_PIN, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, PRIMOU_GPIO_USB_ID1_PIN);
	} else {
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, PRIMOU_GPIO_USB_ID1_PIN);
	}
}

static struct cable_detect_platform_data cable_detect_pdata = {
	.detect_type 		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio 	= PRIMOU_GPIO_USB_ID1_PIN,
	.config_usb_id_gpios 	= config_primou_usb_id_gpios,
	.get_adc_cb		= primou_get_usbid_adc,
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

static struct platform_device *devices[] __initdata = {
        &ram_console_device,
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
        &msm_device_uart2,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
        &msm_proccomm_regulator_dev,
#endif
        &asoc_msm_pcm,
        &asoc_msm_dai0,
        &asoc_msm_dai1,
#if defined(CONFIG_SND_MSM_MVS_DAI_SOC)
        &asoc_msm_mvs,
        &asoc_mvs_dai0,
        &asoc_mvs_dai1,
#endif
        &msm_device_smd,
        &msm_device_dmov,
#if 0
        &smc91x_device,
        &smsc911x_device,
        &msm_device_nand,
#endif
        &msm_device_otg,
        &qsd_device_spi,
#ifdef CONFIG_MSM_SSBI
        &msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
        /*&msm_device_ssbi6,*/
        &msm_device_ssbi7,
#endif
        &android_pmem_device,
        &msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
        &msm_rotator_device,
#endif
        &android_pmem_adsp_device,
        &android_pmem_adsp2_device,
        &android_pmem_audio_device,
        &msm_device_i2c,
        &msm_device_i2c_2,
        &hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
        &msm_aictl_device,
        &msm_mi2s_device,
        &msm_lpa_device,
        &msm_aux_pcm_device,
#endif

#ifdef CONFIG_RAWCHIP
       &msm_rawchip_device,
#endif
#ifdef CONFIG_S5K4E5YX
        &msm_camera_sensor_s5k4e5yx,
#endif

        &msm_device_adspdec,
        &qup_device_i2c,
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
        /*&msm_bt_power_device,*/
#endif
        &msm_kgsl_3d0,
        &msm_kgsl_2d0,
        &msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
        &msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
        &msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
        &msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
        /* &msm_device_sdio_al, */
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
                defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
        &qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
                defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
        &qcedev_device,
#endif

        &htc_battery_pdev,
        &msm_adc_device,
        &msm_ebi0_thermal,
        &msm_ebi1_thermal,
#ifdef CONFIG_SERIAL_MSM_HS
        &msm_device_uart_dm1,
#endif
#ifdef CONFIG_BT
        &primou_rfkill,
#endif


        &pm8058_leds,
        &cable_detect_device,
        &htc_headset_mgr,
        &htc_drm,
};

static struct platform_device *devices_CH[] __initdata = {
	&ram_console_device,
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined(CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
#if 0
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#endif
	&msm_device_otg,
	&qsd_device_spi,
#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
	/*&msm_device_ssbi6,*/
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&android_pmem_adsp_device,
	&android_pmem_adsp2_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif

#ifdef CONFIG_RAWCHIP
       &msm_rawchip_device,
#endif

#ifdef CONFIG_S5K4E5YX
	&msm_camera_sensor_s5k4e5yx,
#endif

	&msm_device_adspdec,
	&qup_device_i2c,
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	/*&msm_bt_power_device,*/
#endif
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	/* &msm_device_sdio_al, */
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

	&htc_battery_pdev,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_BT
	&primou_rfkill,
#endif


	&pm8058_leds_CH,
	&cable_detect_device,
	&htc_headset_mgr,
	&htc_drm,
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct vreg *qup_vreg;
#endif
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 384000,
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	/* Remove the gpio_request due to i2c-qup.c is done so. */
	/*if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");*/

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = vreg_get(NULL, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

#ifdef CONFIG_I2C_SSBI
/*
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};*/

static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static void __init primou_init_irq(void)
{
	msm_init_irq();
}
#if 0
static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},
};
#endif
struct vreg *vreg_s3;
struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};
#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct msm_gpio sdc1_lvlshft_cfg_data[] = {
	{GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "sdc1_lvlshft"},
};
#endif
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), "sdc4_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_0"},
};

static struct msm_gpio sdc4_sleep_cfg_data[] = {
	{GPIO_CFG(58, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			"sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = sdc4_sleep_cfg_data,
	},
};

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (dev_id != 4) {
		if (!enable || enabled_once[dev_id - 1])
			return 0;
	}

	if (enable) {
		if (dev_id == 4) {
			printk(KERN_INFO "%s: Enabling SD slot power\n", __func__);
			mdelay(5);
		}
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		if (dev_id == 4) {
			printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
			mdelay(5);
		}
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) && \
	defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)

#define MBP_ON  1
#define MBP_OFF 0

#define MBP_RESET_N \
	GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA)
#if 0 /* 46 is power key GPIO */
#define MBP_INT0 \
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA)
#endif

#define MBP_MODE_CTRL_0 \
	GPIO_CFG(35, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_1 \
	GPIO_CFG(36, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_2 \
	GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_EN \
	GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_DATA \
	GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_CLK \
	GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static struct msm_gpio mbp_cfg_data[] = {
	{GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_reset"},
	{GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_io_voltage"},
};

static int mbp_config_gpios_pre_init(int enable)
{
	int rc = 0;

	if (enable) {
		rc = msm_gpios_request_enable(mbp_cfg_data,
			ARRAY_SIZE(mbp_cfg_data));
		if (rc) {
			printk(KERN_ERR
				"%s: Failed to turnon GPIOs for mbp chip(%d)\n",
				__func__, rc);
		}
	} else
		msm_gpios_disable_free(mbp_cfg_data, ARRAY_SIZE(mbp_cfg_data));
	return rc;
}

static int mbp_setup_rf_vregs(int state)
{
	struct vreg *vreg_rf = NULL;
	struct vreg *vreg_rf_switch	= NULL;
	int rc;

	vreg_rf = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_rf)) {
		pr_err("%s: s2 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf));
		return -EFAULT;
	}
	vreg_rf_switch = vreg_get(NULL, "rf");
	if (IS_ERR(vreg_rf_switch)) {
		pr_err("%s: rf vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf_switch));
		return -EFAULT;
	}

	if (state) {
		rc = vreg_set_level(vreg_rf, 1300);
		if (rc) {
			pr_err("%s: vreg s2 set level failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = vreg_enable(vreg_rf);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(s2) = %d\n",
					__func__, rc);
		}

		rc = vreg_set_level(vreg_rf_switch, 2600);
		if (rc) {
			pr_err("%s: vreg rf switch set level failed (%d)\n",
					__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_rf_switch);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(rf) = %d\n",
					__func__, rc);
		}
	} else {
		(void) vreg_disable(vreg_rf);
		(void) vreg_disable(vreg_rf_switch);
	}
	return 0;
}

static int mbp_setup_vregs(int state)
{/*
	struct vreg *vreg_analog = NULL;
	struct vreg *vreg_io = NULL;
	int rc;

	vreg_analog = vreg_get(NULL, "gp4");
	if (IS_ERR(vreg_analog)) {
		pr_err("%s: gp4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_analog));
		return -EFAULT;
	}
	vreg_io = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_io)) {
		pr_err("%s: s3 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_io));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_analog, 2600);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg enable failed (%d)",
					__func__, rc);
		}
		rc = vreg_set_level(vreg_io, 1800);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg enable failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg disable failed (%d)",
					__func__, rc);
		}
		rc = vreg_disable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg disable failed (%d)",
					__func__, rc);
		}
	}*/
	return rc;
}

static int mbp_set_tcxo_en(int enable)
{
	int rc;
	const char *id = "UBMC";
	struct vreg *vreg_analog = NULL;

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A1,
		enable ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0) {
		printk(KERN_ERR "%s: unable to %svote for a1 clk\n",
			__func__, enable ? "" : "de-");
		return -EIO;
	}
	if (!enable) {
		vreg_analog = vreg_get(NULL, "gp4");
		/*
		if (IS_ERR(vreg_analog)) {
			pr_err("%s: gp4 vreg get failed (%ld)",
					__func__, PTR_ERR(vreg_analog));
			return -EFAULT;
		}

		(void) vreg_disable(vreg_analog);*/
	}
	return rc;
}

static void mbp_set_freeze_io(int state)
{
	if (state)
		gpio_set_value(85, 0);
	else
		gpio_set_value(85, 1);
}

static int mbp_set_core_voltage_en(int enable)
{
	int rc;
	struct vreg *vreg_core1p2 = NULL;

	vreg_core1p2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_core1p2)) {
		pr_err("%s: gp16 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_core1p2));
		return -EFAULT;
	}
	if (enable) {
		rc = vreg_set_level(vreg_core1p2, 1200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		(void) vreg_enable(vreg_core1p2);

		return 80;
	} else {
		gpio_set_value(85, 1);
		return 0;
	}
	return rc;
}

static void mbp_set_reset(int state)
{
	if (state)
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 0);
	else
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 1);
}

static int mbp_config_interface_mode(int state)
{
	if (state) {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_ENABLE);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_0), 0);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_1), 1);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_2), 0);
	} else {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_DISABLE);
	}
	return 0;
}

static int mbp_setup_adc_vregs(int state)
{
	struct vreg *vreg_adc = NULL;
	int rc;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_adc, 2200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_adc);
		if (rc) {
			pr_err("%s: enable vreg adc failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_adc);
		if (rc) {
			pr_err("%s: disable vreg adc failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_power_up(void)
{
	int rc;

	rc = mbp_config_gpios_pre_init(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_gpios_pre_init() done\n", __func__);

	rc = mbp_setup_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: gp4 (2.6) and s3 (1.8) done\n", __func__);

	rc = mbp_set_tcxo_en(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: tcxo clock done\n", __func__);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: set gpio 85 to 1 done\n", __func__);

	udelay(100);
	mbp_set_reset(MBP_ON);

	udelay(300);
	rc = mbp_config_interface_mode(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_interface_mode() done\n", __func__);

	udelay(100 + mbp_set_core_voltage_en(MBP_ON));
	pr_debug("%s: power gp16 1.2V done\n", __func__);

	mbp_set_freeze_io(MBP_ON);
	pr_debug("%s: set gpio 85 to 0 done\n", __func__);

	udelay(100);

	rc = mbp_setup_rf_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s2 1.3V and rf 2.6V done\n", __func__);

	rc = mbp_setup_adc_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s4 2.2V  done\n", __func__);

	udelay(200);

	mbp_set_reset(MBP_OFF);
	pr_debug("%s: close gpio 44 done\n", __func__);

	msleep(20);
exit:
	return rc;
}

static int mbp_power_down(void)
{
	int rc;
	struct vreg *vreg_adc = NULL;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}

	mbp_set_reset(MBP_ON);
	pr_debug("%s: mbp_set_reset(MBP_ON) done\n", __func__);

	udelay(100);

	rc = mbp_setup_adc_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: vreg_disable(vreg_adc) done\n", __func__);

	udelay(5);

	rc = mbp_setup_rf_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_rf_vregs(MBP_OFF) done\n", __func__);

	udelay(5);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: mbp_set_freeze_io(MBP_OFF) done\n", __func__);

	udelay(100);
	rc = mbp_set_core_voltage_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_core_voltage_en(MBP_OFF) done\n", __func__);

	gpio_set_value(85, 1);

	rc = mbp_set_tcxo_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_tcxo_en(MBP_OFF) done\n", __func__);

	rc = mbp_config_gpios_pre_init(MBP_OFF);
	if (rc)
		goto exit;
exit:
	return rc;
}

static void (*mbp_status_notify_cb)(int card_present, void *dev_id);
static void *mbp_status_notify_cb_devid;
static int mbp_power_status;
static int mbp_power_init_done;

static uint32_t mbp_setup_power(struct device *dv,
	unsigned int power_status)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	if (power_status == mbp_power_status)
		goto exit;
	if (power_status) {
		pr_debug("turn on power of mbp slot");
		rc = mbp_power_up();
		mbp_power_status = 1;
	} else {
		pr_debug("turn off power of mbp slot");
		rc = mbp_power_down();
		mbp_power_status = 0;
	}
exit:
	return rc;
};

int mbp_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	mbp_status_notify_cb = callback;
	mbp_status_notify_cb_devid = dev_id;
	return 0;
}

static unsigned int mbp_status(struct device *dev)
{
	return mbp_power_status;
}

static uint32_t msm_sdcc_setup_power_mbp(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;
	uint32_t rc = 0;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_power(dv, vdd);
	if (rc) {
		pr_err("%s: Failed to setup power (%d)\n",
			__func__, rc);
		goto out;
	}
	if (!mbp_power_init_done) {
		mbp_setup_power(dv, 1);
		mbp_setup_power(dv, 0);
		mbp_power_init_done = 1;
	}
	if (vdd >= 0x8000) {
		rc = mbp_setup_power(dv, (0x8000 == vdd) ? 0 : 1);
		if (rc) {
			pr_err("%s: Failed to config mbp chip power (%d)\n",
				__func__, rc);
			goto out;
		}
		if (mbp_status_notify_cb) {
			mbp_status_notify_cb(mbp_power_status,
				mbp_status_notify_cb_devid);
		}
	}
out:
	/* should return 0 only */
	return 0;
}

#endif

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
		!gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PRIMOU_GPIO_SDMC_CD_N));
}
#endif

#ifdef CONFIG_MMC_MSM_SDC4_WP_SUPPORT
static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_msm7x30_surf()))
		return -1;
	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (((readl(wp_addr) >> 4) >> (pdev->id-1)) & 0x01);
	pr_info("%s: WP Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);

	return ret;
}
#endif
#endif

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
#if defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power_mbp,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status	        = mbp_status,
	.register_status_notify = mbp_register_status_notify,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 24576000,
	.nonremovable	= 0,
};
#else
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int primou_sdc2_slot_type = MMC_TYPE_MMC;
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask       = MMC_VDD_165_195 | MMC_VDD_27_28,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.slot_type		= &primou_sdc2_slot_type,
	.nonremovable	= 1,
	.emmc_dma_ch	= 7,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
/* HTC_WIFI_START */
/*
static unsigned int primou_sdc3_slot_type = MMC_TYPE_SDIO_WIFI;
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.slot_type		= &primou_sdc3_slot_type,
	.nonremovable	= 0,
};
*/
/* HTC_WIFI_END */
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int primou_sdc4_slot_type = MMC_TYPE_SD;
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PRIMOU_GPIO_SDMC_CD_N),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif

#ifdef CONFIG_MMC_MSM_SDC4_WP_SUPPORT
	.wpswitch    = msm_sdcc_get_wpswitch,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
	.slot_type     = &primou_sdc4_slot_type,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static void msm_sdc1_lvlshft_enable(void)
{
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */
	rc = vreg_set_level(vreg_mmc, 2850);
	if (rc)
		printk(KERN_ERR "%s: vreg_set_level() = %d \n",	__func__, rc);

	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);
}
#endif

static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

	vreg_mmc = vreg_get(NULL, "gp10");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (machine_is_msm7x30_fluid()) {
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		msm_sdc1_lvlshft_enable();
	}
	sdcc_vreg_data[0].vreg_data = vreg_s3;
	sdcc_vreg_data[0].level = 1800;
	msm_add_sdcc(1, &msm7x30_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm7x30_sdc2_data.swfi_latency =
		msm_pm_data[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	msm_add_sdcc(2, &msm7x30_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
/* HTC_WIFI_START */
	/*
	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
	*/
	primou_init_mmc(system_rev);
/* HTC_WIFI_END*/
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}
#if 0
static void __init msm7x30_init_nand(void)
{
	char *build_id;
	struct flash_platform_data *plat_data;

	build_id = socinfo_get_build_id();
	if (build_id == NULL) {
		pr_err("%s: Build ID not available from socinfo\n", __func__);
		return;
	}

	if (build_id[8] == 'C' &&
			!msm_gpios_request_enable(msm_nand_ebi2_cfg_data,
			ARRAY_SIZE(msm_nand_ebi2_cfg_data))) {
		plat_data = msm_device_nand.dev.platform_data;
		plat_data->interleave = 1;
		printk(KERN_INFO "%s: Interleave mode Build ID found\n",
			__func__);
	}
}
#endif

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

static void __init pmic8058_leds_init(void)
{
	if (machine_is_msm7x30_surf())
		pm8058_7x30_data.leds_pdata = &pm8058_surf_leds_data;
	else if (!machine_is_msm7x30_fluid())
		pm8058_7x30_data.leds_pdata = &pm8058_ffa_leds_data;
	else if (machine_is_msm7x30_fluid())
		pm8058_7x30_data.leds_pdata = &pm8058_fluid_leds_data;
}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

#ifdef CONFIG_PERFLOCK
static unsigned primou_perf_acpu_table[] = {
	245000000,
	768000000,
	1024000000,
};

static struct perflock_platform_data primou_perflock_data = {
	.perf_acpu_table = primou_perf_acpu_table,
	.table_size = ARRAY_SIZE(primou_perf_acpu_table),
};
#endif

#if 0 /* disable isa1200*/
static const char *vregs_isa1200_name[] = {
	"gp7",
	"gp10",
};

static const int vregs_isa1200_val[] = {
	1800,
	2600,
};
static struct vreg *vregs_isa1200[ARRAY_SIZE(vregs_isa1200_name)];

static int isa1200_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
		if (!vregs_isa1200[i]) {
			pr_err("%s: vreg_get %s failed (%d)\n",
				__func__, vregs_isa1200_name[i], rc);
			goto vreg_fail;
		}

		rc = vreg_on ? vreg_enable(vregs_isa1200[i]) :
			  vreg_disable(vregs_isa1200[i]);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed (%d)\n",
				__func__, vregs_isa1200_name[i],
			       vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}

	/* vote for DO buffer */
	rc = pmapp_clock_vote("VIBR", PMAPP_CLOCK_ID_DO,
		vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc)	{
		pr_err("%s: unable to %svote for d0 clk\n",
			__func__, vreg_on ? "" : "de-");
		goto vreg_fail;
	}

	return 0;

vreg_fail:
	while (i)
		vreg_disable(vregs_isa1200[--i]);
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int i, rc;

	if (enable == true) {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
			vregs_isa1200[i] = vreg_get(NULL,
						vregs_isa1200_name[i]);
			if (IS_ERR(vregs_isa1200[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					__func__, vregs_isa1200_name[i],
					PTR_ERR(vregs_isa1200[i]));
				rc = PTR_ERR(vregs_isa1200[i]);
				goto vreg_get_fail;
			}
			rc = vreg_set_level(vregs_isa1200[i],
					vregs_isa1200_val[i]);
			if (rc) {
				pr_err("%s: vreg_set_level() = %d\n",
					__func__, rc);
				goto vreg_get_fail;
			}
		}

		rc = gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO);
			goto vreg_get_fail;
		}

		rc = gpio_request(HAP_LVL_SHFT_MSM_GPIO, "haptics_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO, rc);
			goto vreg_get_fail;
		}

		gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 1);
	} else {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++)
			vreg_put(vregs_isa1200[i]);

		gpio_free(HAP_LVL_SHFT_MSM_GPIO);
	}

	return 0;
vreg_get_fail:
	while (i)
		vreg_put(vregs_isa1200[--i]);
	return rc;
}

static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.power_on = isa1200_power,
	.dev_setup = isa1200_dev_setup,
	.pwm_ch_id = 1, /*channel id*/
	/*gpio to enable haptic*/
	.hap_en_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
};

static struct i2c_board_info msm_isa1200_board_info[] = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};
#endif
/*
static int kp_flip_mpp_config(void)
{
	struct pm8xxx_mpp_config_data kp_flip_mpp = {
		.type = PM8XXX_MPP_TYPE_D_INPUT,
		.level = PM8018_MPP_DIG_LEVEL_S3,
		.control = PM8XXX_MPP_DIN_TO_INT,
	};

	return pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(PM_FLIP_MPP),
						&kp_flip_mpp);
}

static struct flip_switch_pdata flip_switch_data = {
	.name = "kp_flip_switch",
	.flip_gpio = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS) + PM_FLIP_MPP,
	.left_key = KEY_OPEN,
	.right_key = KEY_CLOSE,
	.active_low = 0,
	.wakeup = 1,
	.flip_mpp_config = kp_flip_mpp_config,
};

static struct platform_device flip_switch_device = {
	.name   = "kp_flip_switch",
	.id	= -1,
	.dev    = {
		.platform_data = &flip_switch_data,
	}
};
*/
static void primou_reset(void)
{
	gpio_set_value(PRIMOU_GPIO_PS_HOLD, 0);
}

void primou_add_usb_devices(void)
{
	printk(KERN_INFO "%s rev: %d\n", __func__, system_rev);
	android_usb_pdata.products[0].product_id =
			android_usb_pdata.product_id;


	/* diag bit set */
	if (get_radio_flag() & 0x20000)
		android_usb_pdata.diag_init = 1;

	/* add cdrom support in normal mode */
	if (board_mfg_mode() == 0) {
		android_usb_pdata.nluns = 3;
		android_usb_pdata.cdrom_lun = 0x4;
	}

	config_primou_usb_id_gpios(0);
	msm_device_gadget_peripheral.dev.parent = &msm_device_otg.dev;
	platform_device_register(&msm_device_gadget_peripheral);
	platform_device_register(&android_usb_device);
}

static int __init board_serialno_setup(char *serialno)
{
	android_usb_pdata.serial_number = serialno;
	return 1;
}

#ifdef CONFIG_MDP4_HW_VSYNC
static void primou_te_gpio_config(void)
{
	uint32_t te_gpio_table[] = {
	PCOM_GPIO_CFG(30, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	};
	config_gpio_table(te_gpio_table, ARRAY_SIZE(te_gpio_table));
}
#endif

__setup("androidboot.serialno=", board_serialno_setup);
static void __init primou_init(void)
{
	int rc = 0, i = 0;
	struct kobject *properties_kobj;
	unsigned smem_size;
	uint32_t soc_version = 0;
	struct proc_dir_entry *entry = NULL;
	char *device_mid;

	soc_version = socinfo_get_version();

	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = primou_reset;

	msm_clock_init(&msm7x30_clock_init_data);
	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);

#ifdef CONFIG_PERFLOCK
	perflock_init(&primou_perflock_data);
#endif

#ifdef CONFIG_BT
	bt_export_bd_address();
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.rx_wakeup_irq = gpio_to_irq(PRIMOU_GPIO_BT_HOST_WAKE);
	msm_device_uart_dm1.name = "msm_serial_hs_brcm";
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
	msm_pm_data
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
	msm_device_otg.dev.platform_data = &msm_otg_pdata;

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif
	/*platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);*/
	pmic8058_leds_init();

	buses_init();
	if (board_mfg_mode()==1) {
		pm_led_config[0].out_current = 40;
		htc_headset_mgr_config[0].adc_max = 65535;
	}
        device_mid = get_model_id();
	if (strstr(device_mid, "PK7612")) {
		platform_add_devices(devices_CH, ARRAY_SIZE(devices_CH));
	} else {
		platform_add_devices(devices, ARRAY_SIZE(devices));
	}

	/*usb driver won't be loaded in MFG 58 station and gift mode*/
	if (!(board_mfg_mode() == 6 || board_mfg_mode() == 7))
		primou_add_usb_devices();

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm_qsd_spi_init();


	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(MSM_PM_BOOT_CONFIG_RESET_VECTOR, ioremap(0x0, PAGE_SIZE)));

	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	primou_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	aux_pcm_gpio_init();
	msm_snddev_init();
	primou_audio_init();
#endif
#ifdef CONFIG_RAWCHIP
	spi_register_board_info(spi_rawchip_board_info,
	ARRAY_SIZE(spi_rawchip_board_info));
#endif
#ifdef CONFIG_FLASHLIGHT_TPS61310
	i2c_register_board_info(0, primou_flashlight,
	ARRAY_SIZE(primou_flashlight));

#endif
	/* i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info)); */
	i2c_register_board_info(0, i2c_tps_devices, ARRAY_SIZE(i2c_tps_devices));
#if 0

	if (!machine_is_msm8x55_svlte_ffa() && !machine_is_msm7x30_fluid())
		marimba_pdata.tsadc = &marimba_tsadc_pdata;
#endif

#ifdef CONFIG_BOSCH_BMA150
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, bma150_board_info,
					ARRAY_SIZE(bma150_board_info));
#endif

	i2c_register_board_info(0,
			i2c_bma250_devices, ARRAY_SIZE(i2c_bma250_devices));

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(0, tpa2051_devices,
			ARRAY_SIZE(tpa2051_devices));

	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,
			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));


	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
			ARRAY_SIZE(msm_camera_boardinfo));


	/*bt_power_init();*/
#ifdef CONFIG_I2C_SSBI
	/*msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;*/
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
/*
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, msm_isa1200_board_info,
			ARRAY_SIZE(msm_isa1200_board_info));
	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);
*/
	pm8058_gpios_init();

	entry = create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/emmc FAILED!\n");

	entry = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/dying_processes FAILED!\n");

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
#ifdef CONFIG_RAWCHIP
	config_gpio_table(msm_spi_on_gpio, ARRAY_SIZE(msm_spi_on_gpio));

	config_gpio_table(camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table));
#endif
	if (board_mfg_mode() == 1) {
		for (i = 0; i < ARRAY_SIZE(primou_ts_synaptics_data);  i++)
			primou_ts_synaptics_data[i].mfg_flag = 1;
	}

	i2c_register_board_info(0, i2c_devices,
				ARRAY_SIZE(i2c_devices));
/*
	i2c_register_board_info(0, i2c_Sensors_devices,
				ARRAY_SIZE(i2c_Sensors_devices));
*/
	device_mid = get_model_id();
	if (strstr(device_mid, "PK7612")) {
		if (1 == board_mfg_mode()) {
			primou_synaptics_virtual_keys_attr.show = &primou_synaptics_virtual_keys_show_china_mfg;
			primou_himax_virtual_keys_attr.show = &primou_himax_virtual_keys_show_china_mfg;
		} else {
			primou_synaptics_virtual_keys_attr.show = &primou_synaptics_virtual_keys_show_china;
			primou_himax_virtual_keys_attr.show = &primou_himax_virtual_keys_show_china;
		}
	}

	/*Virtual_key*/
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
				&primou_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	primou_init_keypad();
#ifdef CONFIG_MDP4_HW_VSYNC
	primou_te_gpio_config();
#endif
	primou_init_panel();
	primou_wifi_init();
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_adsp2_size = MSM_PMEM_ADSP2_SIZE;
static int __init pmem_adsp2_size_setup(char *p)
{
	pmem_adsp2_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp2_size", pmem_adsp2_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_device(struct android_pmem_platform_data *pdata, unsigned long start, unsigned long size)
{
	pdata->start = start;
	pdata->size = size;
	if (pdata->start)
		pr_info("%s: pmem %s requests %lu bytes at 0x%p (0x%lx physical).\r\n",
			__func__, pdata->name, size, __va(start), start);
	else
		pr_info("%s: pmem %s requests %lu bytes dynamically.\r\n",
			__func__, pdata->name, size);
}

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	size_pmem_device(&android_pmem_adsp_pdata, 0, pmem_adsp_size);
	size_pmem_device(&android_pmem_adsp2_pdata, 0, pmem_adsp2_size);
	size_pmem_device(&android_pmem_audio_pdata, 0, pmem_audio_size);
	size_pmem_device(&android_pmem_pdata, 0, pmem_sf_size);
	msm7x30_reserve_table[MEMTYPE_EBI1].size += PMEM_KERNEL_EBI1_SIZE;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	if (p->start == 0) {
		pr_info("%s: reserve %lu bytes from memory %d for %s.\r\n", __func__, p->size, p->memory_type, p->name);
		msm7x30_reserve_table[p->memory_type].size += p->size;
	}
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_adsp2_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	reserve_memory_for(&android_pmem_pdata);
#endif
}

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < 0x40000000)
		return MEMTYPE_EBI1;
	if (paddr >= 0x40000000 && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init primou_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init primou_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_base = msm_fb_resources[0].start;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	printk("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

static void __init primou_map_io(void)
{
	msm_shared_ram_phys = 0x00400000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init primou_init_early(void)
{
	primou_allocate_memory_regions();
}

static void __init primou_fixup(struct machine_desc *desc, struct tag *tags,
								char **cmdline, struct meminfo *mi)
{
	engineerid = parse_tag_engineerid(tags);

	mi->nr_banks = 2;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].size = MSM_LINUX_SIZE1;
	mi->bank[1].start = MSM_LINUX_BASE2;
	mi->bank[1].size = MSM_LINUX_SIZE2;
}

MACHINE_START(PRIMOU, "primou")
	.fixup = primou_fixup,
	.map_io = primou_map_io,
	.reserve = primou_reserve,
	.init_irq = primou_init_irq,
	.init_machine = primou_init,
	.timer = &msm_timer,
	.init_early = primou_init_early,
MACHINE_END
