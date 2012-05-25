/* linux/arch/arm/mach-msm/board-supersonic.c
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

//#include <linux/cy8c_tmg_ts.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>
#include <linux/android_pmem.h>
// #include <linux/synaptics_t1007.h>
#include <linux/input.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602_htc.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/hardware.h>
#include <mach/camera.h>
#include <mach/msm_iomap.h>
#include <mach/htc_battery.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/socinfo.h>
#include <linux/spi/spi.h>

#include "board-supersonic.h"
#include "devices.h"
#include "proc_comm.h"
#include "smd_private.h"
#include <mach/msm_serial_hs.h>
#include <mach/bcm_bt_lpm.h>
#include <mach/tpa6130.h>
#include "board-supersonic-flashlight.h"
#include <linux/atmel_qt602240.h>
#include <mach/vreg.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>


#ifdef CONFIG_MICROP_COMMON
#include <mach/atmega_microp.h>
#endif

#include <mach/msm_hdmi.h>
#include <mach/msm_hsusb.h>

#include "board-supersonic-tpa2018d1.h"

#include <linux/msm_kgsl.h>
#include <linux/regulator/machine.h>
#include "footswitch.h"

#define SMEM_SPINLOCK_I2C	   6

#ifdef CONFIG_ARCH_QSD8X50
extern unsigned char *get_bt_bd_ram(void);
#endif

static unsigned skuid;

static uint opt_usb_h2w_sw;
module_param_named(usb_h2w_sw, opt_usb_h2w_sw, uint, 0);

void msm_init_pmic_vibrator(void);
static void config_supersonic_usb_id_gpios(bool output);
extern void __init supersonic_audio_init(void);
extern void __init supersonic_init_panel(void);
#ifdef CONFIG_MICROP_COMMON
void __init supersonic_microp_init(void);
#endif
static struct htc_battery_platform_data htc_battery_pdev_data = {
	.gpio_mbat_in = SUPERSONIC_GPIO_MBAT_IN,
	.gpio_mchg_en_n = SUPERSONIC_GPIO_MCHG_EN_N,
	.gpio_iset = SUPERSONIC_GPIO_ISET,
	.guage_driver = GUAGE_MODEM,
	.m2a_cable_detect = 1,
	.charger = SWITCH_CHARGER,
	/* After the state of SUC XA, MCHG_EN is chanage to CHG_INT*/
	.int_data.chg_int = MSM_GPIO_TO_INT(SUPERSONIC_GPIO_MCHG_EN_N),
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

#ifdef CONFIG_MICROP_COMMON
static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name	= "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
};

static struct microp_function_config microp_lightsensor = {
	.name = "light_sensor",
	.category = MICROP_FUNCTION_LSENSOR,
	.levels = { 3, 7, 12, 57, 114, 279, 366, 453, 540, 0x3FF },
	.channel = 3,
	.int_pin = 1 << 9,
	.golden_adc = 0x118,
	.ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_lightsensor,
	.irq = MSM_uP_TO_INT(9),
};

static struct microp_led_config led_config[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "wimax",
		.type = LED_WIMAX,
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
};

static struct bma150_platform_data supersonic_g_sensor_pdata = {
	.microp_new_cmd = 1,
};

/* Proximity Sensor (Capella_CM3602)*/
static int __capella_cm3602_power(int on)
{
	int ret;
	struct vreg *vreg = vreg_get(0, "gp1");
	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	ret = vreg_set_level(vreg, 2800);

	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		gpio_direction_output(SUPERSONIC_GPIO_PROXIMITY_EN_N, 1);
		ret = vreg_enable(vreg);
		if (ret < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		vreg_disable(vreg);
		gpio_direction_output(SUPERSONIC_GPIO_PROXIMITY_EN_N, 0);
	}

	return ret;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static unsigned int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
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
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = SUPERSONIC_GPIO_PROXIMITY_EN_N,
	.p_out = MSM_uP_TO_INT(4),
};
/* End Proximity Sensor (Capella_CM3602)*/

static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.remote_int   = 1 << 7,
	.remote_irq   = MSM_uP_TO_INT(7),
	.remote_enable_pin	= 0,
	.adc_channel	= 0x01,
	.adc_remote   = {0, 33, 50, 110, 160, 220},
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &supersonic_g_sensor_pdata,
		},
	},
	{
		.name = "supersonic_proximity",
		.id = -1,
		.dev = {
			.platform_data = &capella_cm3602_pdata,
		},
	},
  {
		.name = "HTC_HEADSET_MICROP",
		.id = -1,
		.dev  = {
			.platform_data	= &htc_headset_microp_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions	 = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = SUPERSONIC_GPIO_UP_RESET_N,
	.microp_ls_on = LS_PWR_ON | PS_PWR_ON,
	.spi_devices = SPI_GSENSOR,
};
#endif

static struct gpio_led supersonic_led_list[] = {
	{
		.name = "button-backlight",
		.gpio = SUPERSONIC_AP_KEY_LED_EN,
		.active_low = 0,
	},
};

static struct gpio_led_platform_data supersonic_leds_data = {
	.num_leds	= ARRAY_SIZE(supersonic_led_list),
	.leds		= supersonic_led_list,
};

static struct platform_device supersonic_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &supersonic_leds_data,
	},
};

static int supersonic_phy_init_seq[] = { 0xC, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1 };


// USB cable out: supersonic_uart_usb_switch(1)
// USB cable in: supersonic_uart_usb_switch(0)
static void supersonic_uart_usb_switch(int uart)
{
		printk(KERN_INFO "%s:uart:%d\n", __func__, uart);
		gpio_set_value(SUPERSONIC_USB_UARTz_SW, uart?1:0); // XA and for USB cable in to reset wimax UART

		if(system_rev && uart) // XB
		{
				if (gpio_get_value(SUPERSONIC_WIMAX_CPU_UARTz_SW))	// Wimax UART
			{
						printk(KERN_INFO "%s:Wimax UART\n", __func__);
						gpio_set_value(SUPERSONIC_USB_UARTz_SW,1);
						gpio_set_value(SUPERSONIC_WIMAX_CPU_UARTz_SW,1);
				}
				else // USB, CPU UART
				{
						printk(KERN_INFO "%s:Non wimax UART\n", __func__);
						gpio_set_value(SUPERSONIC_WIMAX_CPU_UARTz_SW, uart==2?1:0);
				}
		}
}

extern void msm_hsusb_8x50_phy_reset(void);

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq = supersonic_phy_init_seq,
	.phy_reset = msm_hsusb_8x50_phy_reset,
	.usb_id_pin_gpio =	SUPERSONIC_GPIO_USB_ID_PIN,
	.accessory_detect = 1, /* detect by ID pin gpio */
	.usb_uart_switch = supersonic_uart_usb_switch,
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
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
       "accessory",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c8d,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0c03,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0c04,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_ACCESSORY
       {
               .product_id     = USB_ACCESSORY_PRODUCT_ID,
               .num_functions  = ARRAY_SIZE(usb_functions_accessory),
               .functions      = usb_functions_accessory,
       },
       {
               .product_id     = USB_ACCESSORY_ADB_PRODUCT_ID,
               .num_functions  = ARRAY_SIZE(usb_functions_accessory_adb),
               .functions      = usb_functions_accessory_adb,
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
	.product	= "Supersonic",
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
	.vendorID	= 0x18d1,
	.vendorDescr	= "Google, Inc.",
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
	.product_id	= 0x0c8d,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name = "android_usb",
	.id   = -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};


/* 2 : wimax UART, 1 : CPU uart, 0 : usb
CPU_WIMAX_SW -> GPIO160
USB_UART#_SW -> GPIO33

XA : GPIO33 = 0 -> USB
	GPIO33 = 1 -> CPU UART

XB : GPIO33 = 0 -> USB
	GPIO33 = 1 , GPIO160 = 0 -> CPU UART	 // SUPERSONIC_WIMAX_CPU_UARTz_SW (GPIO160)
	GPIO33 = 1 , GPIO160 = 1 -> Wimax UART	 // SUPERSONIC_USB_UARTz_SW (GPIO33)
*/


static int __init supersonic_board_serialno_setup(char *serialno)
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
__setup("androidboot.serialno=", supersonic_board_serialno_setup);


static struct platform_device supersonic_rfkill = {
	.name = "supersonic_rfkill",
	.id = -1,
};

static struct spi_platform_data supersonic_spi_pdata = {
	.clk_rate	= 1200000,
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

#ifdef CONFIG_BUILD_CIQ
static struct android_pmem_platform_data android_pmem_ciq_pdata = {
	.name = "pmem_ciq",
	.start = MSM_PMEM_CIQ_BASE,
	.size = MSM_PMEM_CIQ_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ciq1_pdata = {
	.name = "pmem_ciq1",
	.start = MSM_PMEM_CIQ1_BASE,
	.size = MSM_PMEM_CIQ1_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ciq2_pdata = {
	.name = "pmem_ciq2",
	.start = MSM_PMEM_CIQ2_BASE,
	.size = MSM_PMEM_CIQ2_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ciq3_pdata = {
	.name = "pmem_ciq3",
	.start = MSM_PMEM_CIQ3_BASE,
	.size = MSM_PMEM_CIQ3_SIZE,
/*	.no_allocator	= 0,*/
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
#endif

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
	.id		= 6,
	.dev		= {
		.platform_data = &android_pmem_venc_pdata,
	},
};

#ifdef CONFIG_BUILD_CIQ
static struct platform_device android_pmem_ciq_device = {
	.name = "android_pmem",
	.id = 7,
	.dev = { .platform_data = &android_pmem_ciq_pdata },
};

static struct platform_device android_pmem_ciq1_device = {
	.name = "android_pmem",
	.id = 8,
	.dev = { .platform_data = &android_pmem_ciq1_pdata },
};

static struct platform_device android_pmem_ciq2_device = {
	.name = "android_pmem",
	.id = 9,
	.dev = { .platform_data = &android_pmem_ciq2_pdata },
};

static struct platform_device android_pmem_ciq3_device = {
	.name = "android_pmem",
	.id = 10,
	.dev = { .platform_data = &android_pmem_ciq3_pdata },
};
#endif



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

static int supersonic_atmel_ts_power(int on)
{
	printk(KERN_INFO "supersonic_atmel_ts_power(%d)\n", on);
	if (on) {
		gpio_set_value(SUPERSONIC_GPIO_TP_RST, 0);
		msleep(5);
		gpio_set_value(SUPERSONIC_GPIO_TP_EN, 1);
		msleep(5);
		gpio_set_value(SUPERSONIC_GPIO_TP_RST, 1);
		msleep(40);
	} else {
		gpio_set_value(SUPERSONIC_GPIO_TP_EN, 0);
		msleep(2);
	}
	return 0;
}

struct atmel_i2c_platform_data supersonic_atmel_ts_data[] = {
	{
		.version = 0x016,
		.abs_x_min = 34,
		.abs_x_max = 990,
		.abs_y_min = 15,
		.abs_y_max = 950,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = SUPERSONIC_GPIO_TP_INT_N,
		.power = supersonic_atmel_ts_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 50},
		.config_T8 = {10, 0, 20, 10, 0, 0, 5, 0},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 32, 3, 5, 0, 5, 2, 14, 5, 10, 25, 10, 0, 0, 0, 0, 0, 0, 0, 0, 143, 25, 146, 10, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 14, 0, 1, 8, 12, 16, 30, 40, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8, 60},
		.object_crc = {0x63, 0x27, 0x8E},
		.cable_config = {30, 30, 8, 16},
		.GCAF_level = {20, 24, 28, 40, 63},
		.filter_level = {46, 100, 923, 978},
	},
	{
		.version = 0x015,
		.abs_x_min = 10,
		.abs_x_max = 1012,
		.abs_y_min = 15,
		.abs_y_max = 960,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = SUPERSONIC_GPIO_TP_INT_N,
		.power = supersonic_atmel_ts_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {100, 10, 50},
		.config_T8 = {8, 0, 50, 50, 0, 0, 50, 0},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 32, 3, 5, 0, 5, 2, 14, 5, 10, 25, 10, 0, 0, 0, 0, 0, 0, 0, 0, 143, 25, 146, 10, 20},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {7, 0, 0, 25, 0, -25, 255, 4, 50, 0, 1, 10, 15, 20, 25, 30, 4},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8, 60},
		.object_crc = {0x87, 0xAD, 0xF5},
	},
	{
		.version = 0x014,
		.abs_x_min = 10,
		.abs_x_max = 1012,
		.abs_y_min = 15,
		.abs_y_max = 960,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = SUPERSONIC_GPIO_TP_INT_N,
		.power = supersonic_atmel_ts_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {100, 10, 50},
		.config_T8 = {8, 0, 50, 50, 0, 0, 50, 0},
		.config_T9 = {139, 0, 0, 18, 12, 0, 16, 32, 3, 5, 0, 5, 2, 14, 5, 10, 25, 10, 0, 0, 0, 0, 0, 0, 0, 0, 143, 25, 146, 10, 20},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {7, 0, 0, 25, 0, -25, 255, 4, 50, 0, 1, 10, 15, 20, 25, 30, 4},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 2, 4, 8},
	}
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
			.max_uV = 1300000,
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

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
};

static struct platform_device htc_headset_mgr = {
	.name = "HTC_HEADSET_MGR",
	.id = -1,
	.dev  = {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio	  = SUPERSONIC_GPIO_35MM_HEADSET_DET,
	.key_enable_gpio  = 0,
	.mic_select_gpio  = 0,
};

static struct platform_device htc_headset_gpio = {
	.name = "HTC_HEADSET_GPIO",
	.id = -1,
	.dev  = {
		.platform_data	= &htc_headset_gpio_data,
	},
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = SUPERSONIC_LAYOUTS,
	.project_name = SUPERSONIC_PROJECT_NAME,
	.reset = SUPERSONIC_GPIO_COMPASS_RST_N,
	.intr = SUPERSONIC_GPIO_COMPASS_INT_N,
};

static struct tpa2018d1_platform_data tpa2018_data = {
	.gpio_tpa2018_spk_en = SUPERSONIC_AUD_SPK_EN,
};

/*
 * HDMI platform data
 */

#if 1
#define HDMI_DBG(s...) printk("[hdmi]" s)
#else
#define HDMI_DBG(s...) do {} while (0)
#endif

static int hdmi_power(int on)
{
	HDMI_DBG("%s(%d)\n", __func__, on);

	switch(on) {
	/* Power on/off sequence for normal or D2 sleep mode */
	case 0:
		gpio_set_value(HDMI_RST, 0);
		msleep(2);
		gpio_set_value(V_HDMI_3V3_EN, 0);
		gpio_set_value(V_VGA_5V_SIL9022A_EN, 0);
		msleep(2);
		gpio_set_value(V_HDMI_1V2_EN, 0);
		break;
	case 1:
		gpio_set_value(V_HDMI_1V2_EN, 1);
		msleep(2);
		gpio_set_value(V_VGA_5V_SIL9022A_EN, 1);
		gpio_set_value(V_HDMI_3V3_EN, 1);
		msleep(2);
		gpio_set_value(HDMI_RST, 1);
		msleep(2);
		break;

	/* Power on/off sequence for D3 sleep mode */
	case 2:
		gpio_set_value(V_HDMI_3V3_EN, 0);
		break;
	case 3:
		gpio_set_value(HDMI_RST, 0);
		msleep(2);
		gpio_set_value(V_HDMI_3V3_EN, 1);
		gpio_set_value(V_VGA_5V_SIL9022A_EN, 1);
		msleep(50);
		gpio_set_value(HDMI_RST, 1);
		msleep(10);
		break;
	case 4:
		gpio_set_value(V_VGA_5V_SIL9022A_EN, 0);
		break;
	case 5:
		gpio_set_value(V_VGA_5V_SIL9022A_EN, 1);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static uint32_t hdmi_gpio_on_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R0, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R1, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R2, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R3, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R4, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),

	PCOM_GPIO_CFG(SUPERSONIC_LCD_G0, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G1, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G2, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G3, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G4, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G5, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),

	PCOM_GPIO_CFG(SUPERSONIC_LCD_B0, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B1, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B2, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B3, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B4, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),

	PCOM_GPIO_CFG(SUPERSONIC_LCD_PCLK, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_VSYNC, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_HSYNC, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_DE, 1, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
};

static uint32_t hdmi_gpio_off_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R0, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R1, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R2, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R3, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_R4, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),

	PCOM_GPIO_CFG(SUPERSONIC_LCD_G0, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G1, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G2, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G3, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G4, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_G5, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),

	PCOM_GPIO_CFG(SUPERSONIC_LCD_B0, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B1, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B2, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B3, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_B4, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),

	PCOM_GPIO_CFG(SUPERSONIC_LCD_PCLK, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_VSYNC, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_HSYNC, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
	PCOM_GPIO_CFG(SUPERSONIC_LCD_DE, 0, GPIO_OUTPUT, GPIO_NO_PULL,
			GPIO_2MA),
};


static void suc_hdmi_gpio_on(void)
{
	HDMI_DBG("%s\n", __func__);

	config_gpio_table(hdmi_gpio_on_table, ARRAY_SIZE(hdmi_gpio_on_table));
}

static void suc_hdmi_gpio_off(void)
{
	int i = 0;

	HDMI_DBG("%s\n", __func__);
	config_gpio_table(hdmi_gpio_off_table, ARRAY_SIZE(hdmi_gpio_off_table));

	for (i = SUPERSONIC_LCD_R0; i <= SUPERSONIC_LCD_R4; i++)
		gpio_set_value(i, 0);
	for (i = SUPERSONIC_LCD_G0; i <= SUPERSONIC_LCD_G5; i++)
		gpio_set_value(i, 0);
	for (i = SUPERSONIC_LCD_B0; i <= SUPERSONIC_LCD_DE; i++)
		gpio_set_value(i, 0);
}

static struct hdmi_platform_data hdmi_device_data = {
	.hdmi_res = {
		.start = MSM_HDMI_FB_BASE,
		.end = MSM_HDMI_FB_BASE + MSM_HDMI_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	.power = hdmi_power,
	.hdmi_gpio_on = suc_hdmi_gpio_on,
	.hdmi_gpio_off = suc_hdmi_gpio_off,
};

static struct tpa6130_platform_data headset_amp_platform_data = {
	.enable_rpc_server = 0,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(ATMEL_QT602240_NAME, 0x94 >> 1),
		.platform_data = &supersonic_atmel_ts_data,
		.irq = MSM_GPIO_TO_INT(SUPERSONIC_GPIO_TP_INT_N)
	},
#ifdef CONFIG_MICROP_COMMON
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(SUPERSONIC_GPIO_UP_INT_N)
	},
#endif
	{
		I2C_BOARD_INFO("smb329", 0x6E >> 1),
	},
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
	},
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(SUPERSONIC_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO("s5k3h1gx",	0x20 >> 1),
	},/*samsung for 2nd source main camera*/
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},/*samsung 2nd camera 2nd source*/
	{
		I2C_BOARD_INFO("ov8810", 0x6C >> 1),
	},
	{
		I2C_BOARD_INFO("ov9665", 0x60 >> 1),
	},
	{
		I2C_BOARD_INFO(TPA6130_I2C_NAME, 0xC0 >> 1),
		.platform_data = &headset_amp_platform_data,
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
	{
		I2C_BOARD_INFO("tpa2018d1", 0x58),
		.platform_data = &tpa2018_data,
	},
	{
		I2C_BOARD_INFO("SiL902x-hdmi", 0x76 >> 1),
		.platform_data = &hdmi_device_data,
		.irq = MSM_uP_TO_INT(1),
	},
};

#define ATAG_BDADDR 0x43294329
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

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA SUSPEND*/
	PCOM_GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* VSYNC */
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
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA), /* MCLK */
};

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
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

static void supersonic_maincam_clk_switch(void){
	int rc = 0;
	pr_info("SuperSoinc: clk switch (supersonic)(maincam)\n");
	rc = gpio_request(SUPERSONIC_CLK_SWITCH, "maincam");
	if (rc < 0)
		pr_err("GPIO (%d) request fail\n", SUPERSONIC_CLK_SWITCH);
	else
		gpio_direction_output(SUPERSONIC_CLK_SWITCH, 0);
	gpio_free(SUPERSONIC_CLK_SWITCH);

	return;
}

static void supersonic_seccam_clk_switch(void){
	int rc = 0;
	pr_info("SuperSoinc: Doing clk switch (supersonic)(2ndcam)\n");
	rc = gpio_request(SUPERSONIC_CLK_SWITCH, "seccam");
	if (rc < 0)
		pr_err("GPIO (%d) request fail\n", SUPERSONIC_CLK_SWITCH);
	else
		gpio_direction_output(SUPERSONIC_CLK_SWITCH, 1);
	gpio_free(SUPERSONIC_CLK_SWITCH);

	return;
}

enum msm_camera_source camera_source;
static void supersonic_set_source(enum msm_camera_source source)
{
	camera_source = source;
}

enum msm_camera_source supersonic_get_source(void){
	return camera_source;
}

static int camera_main_probed = 0;
static int supersonic_camera_main_get_probe(void)
{
	return camera_main_probed;
}
static void supersonic_camera_main_set_probe(int probed)
{
	camera_main_probed = probed;
}

static int camera_sec_probed = 0;
static int supersonic_camera_sec_get_probe(void)
{
	return camera_sec_probed;
}
static void supersonic_camera_sec_set_probe(int probed)
{
	camera_sec_probed = probed;
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 10,
	.low_cap_limit		= 15,
};

/*2nd source for 2nd camera*/
static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name = "s5k6aafx",
	.sensor_reset = SUPERSONIC_MAINCAM_RST,
	.sensor_pwd = SUPERSONIC_2NDCAM_PWD,
	.camera_clk_switch = supersonic_seccam_clk_switch,
	.camera_main_get_probe = supersonic_camera_sec_get_probe,
	.camera_main_set_probe = supersonic_camera_sec_get_probe,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.waked_up = 0,
	.need_suspend = 0,
};

static struct platform_device msm_camera_sensor_s5k6aafx = {
	.name	   = "msm_camera_s5k6aafx",
	.dev		= {
		.platform_data = &msm_camera_sensor_s5k6aafx_data,
	},
};

/*samsung for 2nd source main camera*/
static struct msm_camera_sensor_info msm_camera_sensor_s5k3h1_data = {
	.sensor_name	= "s5k3h1gx",
	.sensor_reset	= SUPERSONIC_MAINCAM_RST,
	.sensor_pwd		= SUPERSONIC_MAINCAM_PWD,
	.camera_clk_switch	= supersonic_maincam_clk_switch,
	.camera_set_source = supersonic_set_source,
	.camera_main_get_probe = supersonic_camera_main_get_probe,
	.camera_main_set_probe = supersonic_camera_main_set_probe,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3h1 = {
	.name			= "msm_camera_s5k3h1gx",
	.dev			= {
	.platform_data = &msm_camera_sensor_s5k3h1_data,
	},
};
static struct msm_camera_sensor_info msm_camera_sensor_ov8810_data = {
	.sensor_name	= "ov8810",
	.sensor_reset	= SUPERSONIC_MAINCAM_RST,
	.sensor_pwd		= SUPERSONIC_MAINCAM_PWD,
	.camera_clk_switch	= supersonic_maincam_clk_switch,
	.camera_set_source = supersonic_set_source,
	.camera_main_get_probe = supersonic_camera_main_get_probe,
	.camera_main_set_probe = supersonic_camera_main_set_probe,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.waked_up = 0,
	.need_suspend = 0,
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_ov8810 = {
	.name			= "msm_camera_ov8810",
	.dev			= {
	.platform_data = &msm_camera_sensor_ov8810_data,
	},
};

static struct msm_camera_sensor_info msm_camera_sensor_ov9665_data = {
	.sensor_name	= "ov9665",
	.sensor_reset	= SUPERSONIC_MAINCAM_RST,
	.sensor_pwd = SUPERSONIC_2NDCAM_PWD,
	.camera_clk_switch	= supersonic_seccam_clk_switch,
	.camera_get_source = supersonic_get_source,
	.camera_main_get_probe = supersonic_camera_sec_get_probe,
	.camera_main_get_probe = supersonic_camera_sec_get_probe,
	.pdata		= &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.waked_up = 0,
	.need_suspend = 0,
};

static struct platform_device msm_camera_sensor_ov9665 = {
	.name	   = "msm_camera_ov9665",
	.dev		= {
		.platform_data = &msm_camera_sensor_ov9665_data,
	},
};
static void config_supersonic_flashlight_gpios(void)
{
	static uint32_t flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(SUPERSONIC_GPIO_FLASHLIGHT_TORCH, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(SUPERSONIC_GPIO_FLASHLIGHT_FLASH, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(SUPERSONIC_GPIO_FLASHLIGHT_FLASH_ADJ, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	};
	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data supersonic_flashlight_data = {
	.gpio_init	= config_supersonic_flashlight_gpios,
	.torch = SUPERSONIC_GPIO_FLASHLIGHT_TORCH,
	.flash = SUPERSONIC_GPIO_FLASHLIGHT_FLASH,
	.flash_adj = SUPERSONIC_GPIO_FLASHLIGHT_FLASH_ADJ,
	.flash_duration_ms = 600,
	.led_count = 1,
};

static struct platform_device supersonic_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &supersonic_flashlight_data,
	},
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
		.rx_wakeup_irq = -1,
		.inject_rx_on_wakeup = 0,
		.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
		.gpio_wake = SUPERSONIC_GPIO_BT_CHIP_WAKE,
		.gpio_host_wake = SUPERSONIC_GPIO_BT_HOST_WAKE,
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

static struct platform_device *devices[] __initdata = {
#ifndef CONFIG_MSM_SERIAL_DEBUGGER
	&msm_device_uart1,
#endif
	&bcm_bt_lpm_device,
	&msm_device_uart_dm1,
	&htc_battery_pdev,
	&htc_headset_mgr,
	&htc_headset_gpio,
	&ram_console_device,
	&supersonic_rfkill,
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
//	&android_pmem_camera_device,
#ifdef CONFIG_720P_CAMERA
	&android_pmem_venc_device,
#endif
#ifdef CONFIG_BUILD_CIQ
	&android_pmem_ciq_device,
	&android_pmem_ciq1_device,
	&android_pmem_ciq2_device,
	&android_pmem_ciq3_device,
#endif
	&msm_camera_sensor_s5k3h1,
	&msm_camera_sensor_ov8810,
	&msm_camera_sensor_s5k6aafx,
	&msm_kgsl_3d0,
	&msm_device_i2c,
	&msm_camera_sensor_ov9665,
	&supersonic_flashlight_device,
	&supersonic_leds,
#if defined(CONFIG_SPI_QSD)
	&msm_device_spi,
#endif
};

static uint32_t usb_phy_3v3_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_USB_PHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};

static uint32_t usb_ID_PIN_input_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

static void config_supersonic_usb_id_gpios(bool output)
{
	if (output){
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(SUPERSONIC_GPIO_USB_ID_PIN, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, SUPERSONIC_GPIO_USB_ID_PIN);
	}else{
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, SUPERSONIC_GPIO_USB_ID_PIN);
	}
}

static struct msm_acpu_clock_platform_data supersonic_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
};

int supersonic_init_mmc(int sysrev);

static int OJ_BMA_power(void)
{
	int ret;
	struct vreg *vreg = vreg_get(0, "synt");

	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	ret = vreg_set_level(vreg, 2850);

	ret = vreg_enable(vreg);
	if (ret < 0)
		printk(KERN_ERR "%s: vreg enable failed\n", __func__);

	return 0;
}

unsigned supersonic_get_skuid(void)
{
	return skuid;
}

static ssize_t supersonic_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)	":43:835:86:50"
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":165:835:100:50"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)	":300:835:110:50"
			":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":425:835:90:50"
			"\n");
}

static struct kobj_attribute supersonic_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &supersonic_virtual_keys_show,
};

static struct attribute *supersonic_properties_attrs[] = {
	&supersonic_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group supersonic_properties_attr_group = {
	.attrs = supersonic_properties_attrs,
};

static void supersonic_reset(void)
{
		gpio_set_value(SUPERSONIC_GPIO_PS_HOLD, 0);
}

/* system_rev == higher 16bits of PCBID
XA -> 0000FFFF -> 0x0000
XB -> 0101FFFF -> 0x0101
XC -> 0202FFFF -> 0x0202
*/
static void __init supersonic_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	printk("supersonic_init() revision=%d\n", system_rev);

	msm_hw_reset_hook = supersonic_reset;

	supersonic_board_serialno_setup(board_serialno());

	OJ_BMA_power();

	msm_acpu_clock_init(&supersonic_clock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
				  &msm_device_uart1.dev, 1, MSM_GPIO_TO_INT(SUPERSONIC_GPIO_UART1_RX));
#endif

#ifdef CONFIG_SPI_QSD
	msm_device_spi.dev.platform_data = &supersonic_spi_pdata;
#endif

	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;

	config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
	gpio_request(SUPERSONIC_GPIO_TP_EN, "tp_en");
	gpio_direction_output(SUPERSONIC_GPIO_TP_EN, 0);

	supersonic_audio_init();
	supersonic_init_panel();
#ifdef CONFIG_MICROP_COMMON
	supersonic_microp_init();
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));

	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);

	if (!opt_usb_h2w_sw) {
		msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
		config_supersonic_usb_id_gpios(0);
	}
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	ret = supersonic_init_mmc(system_rev);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&supersonic_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

	msm_init_pmic_vibrator();
}

static void __init supersonic_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	skuid = parse_tag_skuid((const struct tag *)tags);
	printk(KERN_INFO "supersonic_fixup:skuid=0x%x\n", skuid);
	/* First Bank 256MB */
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
//	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;	/*(219*1024*1024);*/

	/* Second Bank 128MB */
	mi->nr_banks++;
	mi->bank[1].start = MSM_EBI1_BANK1_BASE;
//	mi->bank[1].node = PHYS_TO_NID(MSM_EBI1_BANK1_BASE);
	mi->bank[1].size = MSM_EBI1_BANK1_SIZE;
}

static void __init supersonic_map_io(void)
{
	msm_map_qsd8x50_io();
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",__func__);
}

extern struct sys_timer msm_timer;

MACHINE_START(SUPERSONIC, "supersonic")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x20000100,
	.fixup		= supersonic_fixup,
	.map_io		= supersonic_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= supersonic_init,
	.timer		= &msm_timer,
MACHINE_END
