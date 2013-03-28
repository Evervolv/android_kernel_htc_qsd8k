/* linux/arch/arm/mach-msm/board-htcleo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
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
#include <linux/i2c-msm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/regulator/machine.h>
#include <linux/usb/android_composite.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/bma150.h>
#include <linux/akm8973.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <linux/ds2746_battery.h>
#include <linux/msm_kgsl.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>
#include <mach/htc_usb.h>
#include <mach/msm_flashlight.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_hsusb.h>
#ifdef CONFIG_SERIAL_BCM_BT_LPM
#include <mach/bcm_bt_lpm.h>
#endif
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>

#include <mach/board-htcleo-mac.h>
#include <mach/board-htcleo-microp.h>
#include <mach/board-htcleo-ts.h>
#include <mach/socinfo.h>

#include "board-htcleo.h"
#include "devices.h"
#include "proc_comm.h"
#include "dex_comm.h"
#include "footswitch.h"

#define ATAG_MAGLDR_BOOT    0x4C47414D
struct tag_magldr_entry
{
     _Bool fNoNandBoot;
};

extern int __init htcleo_init_mmc(unsigned debug_uart);
extern void __init htcleo_audio_init(void);
extern unsigned char *get_bt_bd_ram(void);
static unsigned int nand_boot = 0;

///////////////////////////////////////////////////////////////////////
// Nand boot Option
///////////////////////////////////////////////////////////////////////
int htcleo_is_nand_boot(void)
{
	return nand_boot;
}

static int __init parse_tag_nand_boot(const struct tag *tag)
{
	struct tag_magldr_entry *mentry = (struct tag_magldr_entry *)(&tag->u);
	nand_boot = !(unsigned int)mentry->fNoNandBoot;
	if(*((unsigned*)&tag->u)==0x004b4c63) nand_boot = 2; // cLK signature
	pr_info("Nand Boot: %d\n", nand_boot);
	return 0;
}
__tagtable(ATAG_MAGLDR_BOOT, parse_tag_nand_boot);



///////////////////////////////////////////////////////////////////////
// Regulator
///////////////////////////////////////////////////////////////////////

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] =
{
    {
        .supply = "acpu_vcore",
    },
};

static struct regulator_init_data tps65023_data[5] =
{
    {
        .constraints = {
            .name = "dcdc1", /* VREG_MSMC2_1V29 */
            .min_uV = HTCLEO_TPS65023_MIN_UV_MV * 1000,
            .max_uV = HTCLEO_TPS65023_MAX_UV_MV * 1000,
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
///////////////////////////////////////////////////////////////////////
// Headset
///////////////////////////////////////////////////////////////////////

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= HTCLEO_GPIO_HDS_DET,
	.mic_detect_gpio	= HTCLEO_GPIO_HDS_MIC,
	.microp_channel		= 1,
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



///////////////////////////////////////////////////////////////////////
// Compass
///////////////////////////////////////////////////////////////////////
static struct akm8973_platform_data compass_platform_data =
{
	.layouts = HTCLEO_LAYOUTS,
	.project_name = HTCLEO_PROJECT_NAME,
	.reset = HTCLEO_GPIO_COMPASS_RST_N,
	.intr = HTCLEO_GPIO_COMPASS_INT_N,
};


///////////////////////////////////////////////////////////////////////
// LED Driver (drivers/leds/leds-microp.c - Atmega microp driver
///////////////////////////////////////////////////////////////////////

static struct microp_led_config led_config[] = {
        {
                .name = "amber",
                .type = LED_RGB,
        },
        {
                .name = "green",
                .type = LED_RGB,
        },
};

static struct microp_led_platform_data microp_leds_data = {
        .num_leds       = ARRAY_SIZE(led_config),
        .led_config     = led_config,
};

///////////////////////////////////////////////////////////////////////
// Microp
///////////////////////////////////////////////////////////////////////
static struct bma150_platform_data htcleo_g_sensor_pdata = {
	.microp_new_cmd = 0,
	.chip_layout = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &htcleo_g_sensor_pdata,
		},
	},
	{
		.name = "htcleo-backlight",
		.id = -1,
	},
	{
		.name = "htcleo-proximity",
		.id = -1,
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},

	},
	{
		.name = "htcleo-lsensor",
		.id = -1,
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = HTCLEO_GPIO_UP_RESET_N,
};

static struct i2c_board_info base_i2c_devices[] =
{
	{
		// Only a dummy
		I2C_BOARD_INFO(LEO_TOUCH_DRV_NAME, 0x22),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_COMPASS_INT_N),
	},
	{
	        I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
};

///////////////////////////////////////////////////////////////////////
// USB
///////////////////////////////////////////////////////////////////////

static uint32_t usb_phy_3v3_table[] =
{
    PCOM_GPIO_CFG(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};

static int htcleo_phy_init_seq[] ={0x0C, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1};

// modified to further reflect bravo kernel code
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= htcleo_phy_init_seq,
	.phy_reset		= msm_hsusb_8x50_phy_reset,
	.accessory_detect = 0, /* detect by ID pin gpio */
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "HD2",
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
	.product_name		= "HD2",
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
static void htcleo_add_usb_devices(void)
{
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 1);
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&android_usb_device);
}

unsigned htcleo_get_vbus_state(void)
{
	if(readl(MSM_SHARED_RAM_BASE+0xef20c))
		return 1;
	else
		return 0;
}


///////////////////////////////////////////////////////////////////////
// Flashlight
///////////////////////////////////////////////////////////////////////

static uint32_t flashlight_gpio_table[] =
{
	PCOM_GPIO_CFG(HTCLEO_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static void config_htcleo_flashlight_gpios(void)
{
	config_gpio_table(flashlight_gpio_table, ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data htcleo_flashlight_data =
{
	.gpio_init  = config_htcleo_flashlight_gpios,
	.torch = HTCLEO_GPIO_FLASHLIGHT_TORCH,
	.flash = HTCLEO_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device htcleo_flashlight_device =
{
	.name = "flashlight",
	.dev =
	{
		.platform_data  = &htcleo_flashlight_data,
	},
};

///////////////////////////////////////////////////////////////////////
// Camera
///////////////////////////////////////////////////////////////////////

static uint32_t camera_off_gpio_table[] =
{
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

static uint32_t camera_on_gpio_table[] =
{
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
	config_gpio_table(camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table, ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] =
{
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

static struct msm_camera_device_platform_data msm_camera_device_data =
{
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static int flashlight_control(int mode)
{
        return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data =
{
	.sensor_name = "s5k3e2fx",
	.sensor_reset = 144,
	/* CAM1_PWDN, enabled in a9 */
	//.sensor_pwd = 143,
	/* CAM1_VCM_EN, enabled in a9 */
	//.vcm_pwd = 31,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg = &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx =
{
	.name     = "msm_camera_s5k3e2fx",
	.dev      = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

///////////////////////////////////////////////////////////////////////
// bluetooth
///////////////////////////////////////////////////////////////////////

/* AOSP style interface */

/*
 * bluetooth mac address will be parsed in msm_nand_probe
 * see drivers/mtd/devices/htcleo_nand.c
 */
char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");
/* end AOSP style interface */

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
#endif
};

#ifdef CONFIG_SERIAL_BCM_BT_LPM
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = HTCLEO_GPIO_BT_CHIP_WAKE,
	.gpio_host_wake = HTCLEO_GPIO_BT_HOST_WAKE,
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
#endif
#endif

static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_CHIP_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static struct platform_device htcleo_rfkill =
{
	.name = "htcleo_rfkill",
	.id = -1,
};

///////////////////////////////////////////////////////////////////////
// SPI
///////////////////////////////////////////////////////////////////////

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start  = INT_SPI_INPUT,
		.end    = INT_SPI_INPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start  = INT_SPI_OUTPUT,
		.end    = INT_SPI_OUTPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start  = INT_SPI_ERROR,
		.end    = INT_SPI_ERROR,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start  = 0xA1200000,
		.end    = 0xA1200000 + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_clk",
		.start  = 17,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_mosi",
		.start  = 18,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_miso",
		.start  = 19,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_cs0",
		.start  = 20,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_pwr",
		.start  = 21,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_cs0",
		.start  = 22,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct spi_platform_data htcleo_spi_pdata = {
	.clk_rate	= 4800000,
};

static struct platform_device qsd_device_spi = {
	.name           = "spi_qsd",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(qsd_spi_resources),
	.resource       = qsd_spi_resources,
	.dev		= {
		.platform_data = &htcleo_spi_pdata
	},
};

///////////////////////////////////////////////////////////////////////
// KGSL (HW3D support)#include <linux/android_pmem.h>
///////////////////////////////////////////////////////////////////////

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
	.pwr_data = {
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
	},
	.clk = {
		.name = {
			.clk = "grp_clk",
		},
	},
	.imem_clk_name = {
		.clk = "imem_clk",
	},
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

///////////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////////

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
	/* .no_allocator	= 0, */
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
	/* .no_allocator	= 0, */
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached		= 1,
};


static struct android_pmem_platform_data android_pmem_venc_pdata = {
	.name		= "pmem_venc",
	.start		= MSM_PMEM_VENC_BASE,
	.size		= MSM_PMEM_VENC_SIZE,
	/* .no_allocator	= 0, */
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
	.id		= 4,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_venc_device = {
	.name		= "android_pmem",
	.id		= 5,
	.dev		= {
		.platform_data = &android_pmem_venc_pdata,
	},
};

///////////////////////////////////////////////////////////////////////
// RAM-Console
///////////////////////////////////////////////////////////////////////

static struct resource ram_console_resources[] = {
	{
		.start	= (resource_size_t) MSM_RAM_CONSOLE_BASE,
		.end	= (resource_size_t) (MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};
///////////////////////////////////////////////////////////////////////
// Power/Battery
///////////////////////////////////////////////////////////////////////

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
	.gpio_mbat_in = -1,
	.gpio_mchg_en_n = HTCLEO_GPIO_BATTERY_CHARGER_ENABLE,
	.gpio_iset = HTCLEO_GPIO_BATTERY_CHARGER_CURRENT,
	.gpio_power = HTCLEO_GPIO_POWER_USB,
	.guage_driver = GUAGE_DS2746,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 0,
	.force_no_rpc = 1,
	.int_data = {
		.chg_int = HTCLEO_GPIO_BATTERY_OVER_CHG,
	},
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

static int get_thermal_id(void)
{
	return THERMAL_600;
}

static struct ds2746_platform_data ds2746_pdev_data = {
	.func_get_thermal_id = get_thermal_id,
};

static struct platform_device ds2746_battery_pdev = {
	.name = "ds2746-battery",
	.id = -1,
	.dev = {
		.platform_data = &ds2746_pdev_data,
	},
};

///////////////////////////////////////////////////////////////////////
// Real Time Clock
///////////////////////////////////////////////////////////////////////

struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};

///////////////////////////////////////////////////////////////////////
// Button backlight manager
///////////////////////////////////////////////////////////////////////
#ifdef CONFIG_HTCLEO_BTN_BACKLIGHT_MANAGER
struct platform_device btn_backlight_manager = {
    .name   = "btn_backlight_manager",
    .id     = -1,
};
#endif

///////////////////////////////////////////////////////////////////////
// Platform Devices
///////////////////////////////////////////////////////////////////////


static struct platform_device *devices[] __initdata =
{
	&ram_console_device,
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	&bcm_bt_lpm_device,
#endif
	&msm_device_uart_dm1,
	&htcleo_rfkill,
	&qsd_device_spi,
	&msm_device_nand,
	&msm_device_smd,
	&msm_device_rtc,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_venc_device,
	&msm_device_i2c,
	&ds2746_battery_pdev,
	&htc_battery_pdev,
	&msm_kgsl_3d0,
	&msm_camera_sensor_s5k3e2fx,
	&htcleo_flashlight_device,
	&htc_headset_mgr,
	&htc_headset_gpio,
#ifdef CONFIG_HTCLEO_BTN_BACKLIGHT_MANAGER
	&btn_backlight_manager,
#endif
};
///////////////////////////////////////////////////////////////////////
// Vibrator
///////////////////////////////////////////////////////////////////////

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = HTCLEO_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device htcleo_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};
///////////////////////////////////////////////////////////////////////
// I2C
///////////////////////////////////////////////////////////////////////

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_8MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

///////////////////////////////////////////////////////////////////////
// Clocks
///////////////////////////////////////////////////////////////////////

static struct msm_acpu_clock_platform_data htcleo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 128000,
	.wait_for_irq_khz	= 128000,
//	.wait_for_irq_khz	= 19200,   // TCXO
};

static unsigned htcleo_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data htcleo_perflock_data = {
	.perf_acpu_table = htcleo_perf_acpu_table,
	.table_size = ARRAY_SIZE(htcleo_perf_acpu_table),
};
///////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////

static void htcleo_reset(void)
{
	// 25 - 16 = 9
	while (1)
	{
	        writel(readl(MSM_GPIOCFG2_BASE + 0x504) | (1 << 9), MSM_GPIOCFG2_BASE + 0x504);// owner
		gpio_set_value(HTCLEO_GPIO_PS_HOLD, 0);
	}
}

static void do_grp_reset(void)
{
   	writel(0x20000, MSM_CLK_CTL_BASE + 0x214);
}

static void do_sdc1_reset(void)
{
	volatile uint32_t* sdc1_clk = MSM_CLK_CTL_BASE + 0x218;

	*sdc1_clk |= (1 << 9);
   	mdelay(1);
	*sdc1_clk &= ~(1 << 9);
}

#ifdef CONFIG_HTCLEO_BLINK_ON_BOOT
static void __init htcleo_blink_camera_led(void){
	volatile unsigned *bank6_in, *bank6_out;
	bank6_in = (unsigned int*)(MSM_GPIO1_BASE + 0x0864);
	bank6_out = (unsigned int*)(MSM_GPIO1_BASE + 0x0814);
	*bank6_out = *bank6_in ^ 0x200000;
	mdelay(50);
	*bank6_out = *bank6_in | 0x200000;
	mdelay(200);
}
#endif // CONFIG_HTCLEO_BLINK_ON_BOOT

///////////////////////////////////////////////////////////////////////
// Init
///////////////////////////////////////////////////////////////////////

static void __init htcleo_init(void)
{
	printk("htcleo_init()\n");
	msm_hw_reset_hook = htcleo_reset;

	do_grp_reset();
	do_sdc1_reset();

	msm_acpu_clock_init(&htcleo_clock_data);

	perflock_init(&htcleo_perflock_data);

	init_dex_comm();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs"; /* for bcm */
	msm_device_uart_dm1.resource[3].end = 6;
#endif

	config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));

	htcleo_audio_init();

	msm_device_i2c_init();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);

	htcleo_init_panel();

#ifdef CONFIG_USB_ANDROID
	htcleo_add_usb_devices();
#endif

	i2c_register_board_info(0, base_i2c_devices, ARRAY_SIZE(base_i2c_devices));
	
	htcleo_init_mmc(0);
	platform_device_register(&htcleo_timed_gpios);
	
#ifdef CONFIG_HTCLEO_BLINK_ON_BOOT
	/* Blink the camera LED shortly to show that we're alive! */
	htcleo_blink_camera_led();
#endif // CONFIG_HTCLEO_BLINK_ON_BOOT

}

///////////////////////////////////////////////////////////////////////
// Bootfunctions
///////////////////////////////////////////////////////////////////////

static void __init htcleo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	/* Blink the camera LED shortly to show that we're alive! */
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_EBI1_BANK0_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_EBI1_BANK0_BASE);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
}

#if defined(CONFIG_VERY_EARLY_CONSOLE)
#if defined(CONFIG_HTC_FB_CONSOLE)
int __init htc_fb_console_init(void);
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
int __init ram_console_early_init(void);
#endif
#endif

static unsigned pmem_sf_size = MSM_PMEM_MDP_SIZE;
static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static void __init htcleo_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		mdp_pmem_pdata.start = __pa(addr);
		mdp_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}
}
static void __init htcleo_map_io(void)
{
	msm_map_common_io();
	htcleo_allocate_memory_regions();
	msm_clock_init();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",__func__);
	
#if defined(CONFIG_VERY_EARLY_CONSOLE)
// Init our consoles _really_ early
#if defined(CONFIG_HTC_FB_CONSOLE)
	htc_fb_console_init();
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
	ram_console_early_init();
#endif
#endif

}

extern struct sys_timer msm_timer;

MACHINE_START(HTCLEO, "htcleo")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= (CONFIG_PHYS_OFFSET + 0x00000100),
	.fixup		= htcleo_fixup,
	.map_io		= htcleo_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= htcleo_init,
	.timer		= &msm_timer,
MACHINE_END
