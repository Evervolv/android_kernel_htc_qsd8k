/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/memblock.h>
#include <linux/mfd/pm8058.h>
#include <linux/spi/spi.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>

#include "gpiomux.h"
#include "devices.h"
#include "pmic.h"
#include "board-msm7x30.h"

#define MSM_FB_SIZE		(0x00500000UL)

#define CLK_NS_TO_RATE(ns)	(1000000000UL / (ns))

#define MDDI_CLIENT_CORE_BASE	(0x00108000UL)
#define LCD_CONTROL_BLOCK_BASE	(0x00110000UL)
#define SPI_BLOCK_BASE		(0x00120000UL)
#define PWM_BLOCK_BASE		(0x00140000UL)
#define GPIO_BLOCK_BASE		(0x00150000UL)
#define SYSTEM_BLOCK1_BASE	(0x00160000UL)
#define SYSTEM_BLOCK2_BASE	(0x00170000UL)

#define TTBUSSEL		(MDDI_CLIENT_CORE_BASE | 0x18)
#define DPSET0			(MDDI_CLIENT_CORE_BASE | 0x1C)
#define DPSET1			(MDDI_CLIENT_CORE_BASE | 0x20)
#define DPSUS			(MDDI_CLIENT_CORE_BASE | 0x24)
#define DPRUN			(MDDI_CLIENT_CORE_BASE | 0x28)
#define SYSCKENA		(MDDI_CLIENT_CORE_BASE | 0x2C)

#define BITMAP0			(MDDI_CLIENT_CORE_BASE|0x44)
#define BITMAP1			(MDDI_CLIENT_CORE_BASE|0x48)
#define BITMAP2			(MDDI_CLIENT_CORE_BASE|0x4C)
#define BITMAP3			(MDDI_CLIENT_CORE_BASE|0x50)
#define BITMAP4			(MDDI_CLIENT_CORE_BASE|0x54)

#define SRST			(LCD_CONTROL_BLOCK_BASE|0x00)
#define PORT_ENB		(LCD_CONTROL_BLOCK_BASE|0x04)
#define START			(LCD_CONTROL_BLOCK_BASE|0x08)
#define PORT			(LCD_CONTROL_BLOCK_BASE|0x0C)

#define INTFLG			(LCD_CONTROL_BLOCK_BASE|0x18)
#define INTMSK			(LCD_CONTROL_BLOCK_BASE|0x1C)
#define MPLFBUF			(LCD_CONTROL_BLOCK_BASE|0x20)

#define PXL			(LCD_CONTROL_BLOCK_BASE|0x30)
#define HCYCLE			(LCD_CONTROL_BLOCK_BASE|0x34)
#define HSW			(LCD_CONTROL_BLOCK_BASE|0x38)
#define HDE_START		(LCD_CONTROL_BLOCK_BASE|0x3C)
#define HDE_SIZE		(LCD_CONTROL_BLOCK_BASE|0x40)
#define VCYCLE			(LCD_CONTROL_BLOCK_BASE|0x44)
#define VSW			(LCD_CONTROL_BLOCK_BASE|0x48)
#define VDE_START		(LCD_CONTROL_BLOCK_BASE|0x4C)
#define VDE_SIZE		(LCD_CONTROL_BLOCK_BASE|0x50)
#define WAKEUP			(LCD_CONTROL_BLOCK_BASE|0x54)
#define REGENB			(LCD_CONTROL_BLOCK_BASE|0x5C)
#define VSYNIF			(LCD_CONTROL_BLOCK_BASE|0x60)
#define WRSTB			(LCD_CONTROL_BLOCK_BASE|0x64)
#define RDSTB			(LCD_CONTROL_BLOCK_BASE|0x68)
#define ASY_DATA		(LCD_CONTROL_BLOCK_BASE|0x6C)
#define ASY_DATB		(LCD_CONTROL_BLOCK_BASE|0x70)
#define ASY_DATC		(LCD_CONTROL_BLOCK_BASE|0x74)
#define ASY_DATD		(LCD_CONTROL_BLOCK_BASE|0x78)
#define ASY_DATE		(LCD_CONTROL_BLOCK_BASE|0x7C)
#define ASY_DATF		(LCD_CONTROL_BLOCK_BASE|0x80)
#define ASY_DATG		(LCD_CONTROL_BLOCK_BASE|0x84)
#define ASY_DATH		(LCD_CONTROL_BLOCK_BASE|0x88)
#define ASY_CMDSET		(LCD_CONTROL_BLOCK_BASE|0x8C)
#define MONI			(LCD_CONTROL_BLOCK_BASE|0xB0)
#define VPOS			(LCD_CONTROL_BLOCK_BASE|0xC0)

#define SSICTL			(SPI_BLOCK_BASE|0x00)
#define SSITIME			(SPI_BLOCK_BASE|0x04)
#define SSITX			(SPI_BLOCK_BASE|0x08)
#define SSIINTS			(SPI_BLOCK_BASE|0x14)

#define TIMER0LOAD		(PWM_BLOCK_BASE|0x00)
#define TIMER0CTRL		(PWM_BLOCK_BASE|0x08)
#define PWM0OFF			(PWM_BLOCK_BASE|0x1C)
#define TIMER1LOAD		(PWM_BLOCK_BASE|0x20)
#define TIMER1CTRL		(PWM_BLOCK_BASE|0x28)
#define PWM1OFF			(PWM_BLOCK_BASE|0x3C)
#define TIMER2LOAD		(PWM_BLOCK_BASE|0x40)
#define TIMER2CTRL		(PWM_BLOCK_BASE|0x48)
#define PWM2OFF			(PWM_BLOCK_BASE|0x5C)
#define PWMCR			(PWM_BLOCK_BASE|0x68)

#define GPIODATA		(GPIO_BLOCK_BASE|0x00)
#define GPIODIR			(GPIO_BLOCK_BASE|0x04)
#define GPIOIS			(GPIO_BLOCK_BASE|0x08)
#define GPIOIEV			(GPIO_BLOCK_BASE|0x10)
#define GPIOIC			(GPIO_BLOCK_BASE|0x20)
#define GPIOPC			(GPIO_BLOCK_BASE|0x28)

#define WKREQ			(SYSTEM_BLOCK1_BASE|0x00)
#define CLKENB			(SYSTEM_BLOCK1_BASE|0x04)
#define DRAMPWR			(SYSTEM_BLOCK1_BASE|0x08)
#define INTMASK			(SYSTEM_BLOCK1_BASE|0x0C)
#define CNT_DIS			(SYSTEM_BLOCK1_BASE|0x10)

#define GPIOSEL			(SYSTEM_BLOCK2_BASE|0x00)

struct mddi_table {
	uint32_t reg;
	uint32_t value;
};
static struct mddi_table mddi_toshiba_init_table[] = {
	{ DPSET0,	0x4bec0066 },
	{ DPSET1,	0x00000113 },
	{ DPSUS,	0x00000000 },
	{ DPRUN,	0x00000001 },
	{ 1,		5 },
	{ SYSCKENA,	0x00000001 },
	{ CLKENB,	0x0000A0E9 },
	{ GPIODATA,	0x03FF0000 },
	{ GPIODIR,	0x0000024D },
	{ GPIOSEL,	0x00000173 },
	{ GPIOPC,	0x03C300C0 },
	{ WKREQ,	0x00000000 },

	{ GPIOIS,	0x00000000 },
	{ GPIOIEV,	0x00000001 },
	{ GPIOIC,	0x000003FF },
	{ GPIODATA,	0x00040004 },

	{ GPIODATA,	0x00080008 },
	{ DRAMPWR,	0x00000001 },
	{ CLKENB,	0x0000A0EB },
	{ PWMCR,	0x00000000 },
	{ 1,		1 },
	{ SSICTL,	0x00060399 },
	{ SSITIME,	0x00000100 },
	{ CNT_DIS,	0x00000002 },
	{ SSICTL,	0x0006039b },
	{ SSITX,	0x00000000 },
	{ 1,		7 },
	{ SSITX,	0x00000000 },
	{ 1,		7 },
	{ SSITX,	0x00000000 },
	{ 1,		7 },
	{ SSITX,	0x000800BA },
	{ SSITX,	0x00000111 },
	{ SSITX,	0x00080036 },
	{ SSITX,	0x00000100 },
	{ 1,		1 },
	{ SSITX,	0x0008003A },
	{ SSITX,	0x00000160 },
	{ SSITX,	0x000800B1 },
	{ SSITX,	0x0000015D },
	{ 1,		1 },
	{ SSITX,	0x000800B2 },
	{ SSITX,	0x00000133 },
	{ SSITX,	0x000800B3 },
	{ SSITX,	0x00000122 },
	{ 1,		1 },
	{ SSITX,	0x000800B4 },
	{ SSITX,	0x00000102 },
	{ SSITX,	0x000800B5 },
	{ SSITX,	0x0000011E },
	{ 1,		1 },
	{ SSITX,	0x000800B6 },
	{ SSITX,	0x00000127 },
	{ SSITX,	0x000800B7 },
	{ SSITX,	0x00000103 },
	{ 1,		1 },
	{ SSITX,	0x000800B9 },
	{ SSITX,	0x00000124 },
	{ SSITX,	0x000800BD },
	{ SSITX,	0x000001A1 },
	{ 1,		1 },
	{ SSITX,	0x000800BB },
	{ SSITX,	0x00000100 },
	{ SSITX,	0x000800BF },
	{ SSITX,	0x00000101 },
	{ 1,		1 },
	{ SSITX,	0x000800BE },
	{ SSITX,	0x00000100 },
	{ SSITX,	0x000800C0 },
	{ SSITX,	0x00000111 },
	{ 1,		1 },
	{ SSITX,	0x000800C1 },
	{ SSITX,	0x00000111 },
	{ SSITX,	0x000800C2 },
	{ SSITX,	0x00000111 },
	{ 1,		1 },
	{ SSITX,	0x000800C3 },
	{ SSITX,	0x00080132 },
	{ SSITX,	0x00000132 },
	{ 1,		1 },
	{ SSITX,	0x000800C4 },
	{ SSITX,	0x00080132 },
	{ SSITX,	0x00000132 },
	{ 1,		1 },
	{ SSITX,	0x000800C5 },
	{ SSITX,	0x00080132 },
	{ SSITX,	0x00000132 },
	{ 1,		1 },
	{ SSITX,	0x000800C6 },
	{ SSITX,	0x00080132 },
	{ SSITX,	0x00000132 },
	{ 1,		1 },
	{ SSITX,	0x000800C7 },
	{ SSITX,	0x00080164 },
	{ SSITX,	0x00000145 },
	{ 1,		1 },
	{ SSITX,	0x000800C8 },
	{ SSITX,	0x00000144 },
	{ SSITX,	0x000800C9 },
	{ SSITX,	0x00000152 },
	{ 1,		1 },
	{ SSITX,	0x000800CA },
	{ SSITX,	0x00000100 },
	{ 1,		1 },
	{ SSITX,	0x000800EC },
	{ SSITX,	0x00080101 },
	{ SSITX,	0x000001FC },
	{ 1,		1 },
	{ SSITX,	0x000800CF },
	{ SSITX,	0x00000101 },
	{ 1,		1 },
	{ SSITX,	0x000800D0 },
	{ SSITX,	0x00080110 },
	{ SSITX,	0x00000104 },
	{ 1,		1 },
	{ SSITX,	0x000800D1 },
	{ SSITX,	0x00000101 },
	{ 1,		1 },
	{ SSITX,	0x000800D2 },
	{ SSITX,	0x00080100 },
	{ SSITX,	0x00000128 },
	{ 1,		1 },
	{ SSITX,	0x000800D3 },
	{ SSITX,	0x00080100 },
	{ SSITX,	0x00000128 },
	{ 1,		1 },
	{ SSITX,	0x000800D4 },
	{ SSITX,	0x00080126 },
	{ SSITX,	0x000001A4 },
	{ 1,		1 },
	{ SSITX,	0x000800D5 },
	{ SSITX,	0x00000120 },
	{ 1,		1 },
	{ SSITX,	0x000800EF },
	{ SSITX,	0x00080132 },
	{ SSITX,	0x00000100 },
	{ 1,		1 },
	{ BITMAP0,	0x032001E0 },
	{ BITMAP1,	0x032001E0 },
	{ BITMAP2,	0x014000F0 },
	{ BITMAP3,	0x014000F0 },
	{ BITMAP4,	0x014000F0 },
	{ CLKENB,	0x0000A1EB },
	{ PORT_ENB,	0x00000001 },
	{ PORT,		0x00000004 },
	{ PXL,		0x00000002 },
	{ MPLFBUF,	0x00000000 },
	{ HCYCLE,	0x000000FD },
	{ HSW,		0x00000003 },
	{ HDE_START,	0x00000007 },
	{ HDE_SIZE,	0x000000EF },
	{ VCYCLE,	0x00000325 },
	{ VSW,		0x00000001 },
	{ VDE_START,	0x00000003 },
	{ VDE_SIZE,	0x0000031F },
	{ START,	0x00000001 },
	{ 1,		32 },
	{ SSITX,	0x000800BC },
	{ SSITX,	0x00000180 },
	{ SSITX,	0x0008003B },
	{ SSITX,	0x00000100 },
	{ 1,		1 },
	{ SSITX,	0x000800B0 },
	{ SSITX,	0x00000116 },
	{ 1,		1 },
	{ SSITX,	0x000800B8 },
	{ SSITX,	0x000801FF },
	{ SSITX,	0x000001F5 },
	{ 1,		1 },
	{ SSITX,	0x00000011 },
	{ 1,		5 },
	{ SSITX,	0x00000029 },
};

#define MSM7X30_SURF_DEFAULT_BACKLIGHT_BRIGHTNESS	15

static int msm7x30_backlight_off;
static int msm7x30_backlight_brightness =
			MSM7X30_SURF_DEFAULT_BACKLIGHT_BRIGHTNESS;

static DEFINE_MUTEX(msm7x30_backlight_lock);


static void msm7x30_set_backlight_level(uint8_t level)
{
	if (machine_is_msm7x30_fluid()) {
		gpio_set_value(MSM7X30_PM8058_GPIO(25), !!level);
	} else {
		pmic_set_led_intensity(LED_LCD,
				       MSM7X30_SURF_DEFAULT_BACKLIGHT_BRIGHTNESS *
				       level / LED_FULL);
	}
}

static void msm7x30_process_mddi_table(struct msm_mddi_client_data *client_data,
				       const struct mddi_table *table,
				       size_t count)
{
	int i;
	for (i = 0; i < count; i++) {
		uint32_t reg = table[i].reg;
		uint32_t value = table[i].value;

		if (reg == 0)
			udelay(value);
		else if (reg == 1)
			msleep(value);
		else
			client_data->remote_write(client_data, value, reg);
	}
}

static unsigned wega_reset_gpio = 180;
static unsigned fluid_vee_reset_gpio = 20;

static struct pm8058_pin_config msm7x30_mddi_sleep_clk_cfg_on = {
	.vin_src	= PM8058_GPIO_VIN_SRC_VREG_S3,
	.dir		= PM8058_GPIO_OUTPUT,
	.pull_up	= PM8058_GPIO_PULL_NONE,
	.strength	= PM8058_GPIO_STRENGTH_HIGH,
	.func		= PM8058_GPIO_FUNC_2,
};

static struct pm8058_pin_config msm7x30_mddi_sleep_clk_cfg_off = {
	.vin_src	= PM8058_GPIO_VIN_SRC_VREG_S3,
	.dir		= PM8058_GPIO_OUTPUT,
	.pull_up	= PM8058_GPIO_PULL_NONE,
	.strength	= PM8058_GPIO_STRENGTH_HIGH,
	.func		= PM8058_GPIO_FUNC_NORMAL,
};

static struct pm8058_pin_config msm7x30_fluid_backlight = {
	.vin_src	= PM8058_GPIO_VIN_SRC_VREG_S3,
	.dir		= PM8058_GPIO_OUTPUT,
	.pull_up	= PM8058_GPIO_PULL_NONE,
	.strength	= PM8058_GPIO_STRENGTH_HIGH,
	.func		= PM8058_GPIO_FUNC_NORMAL,
};

static void msm7x30_power_panel(int on)
{
	int rc = 0;
	struct vreg *vreg_ldo12, *vreg_ldo15 = NULL;
	struct vreg *vreg_ldo20, *vreg_ldo16, *vreg_ldo8 = NULL;

	if (on) {
		// XXX enable wega reset gpio

		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		gpio_set_value(180, 0);	/* bring reset line low to hold reset*/
	}

	/* Toshiba WeGA power -- has 3 power source */
	/* 1.5V -- LDO20*/
	vreg_ldo20 = vreg_get(NULL, "gp13");

	if (IS_ERR(vreg_ldo20) || vreg_ldo20 == NULL) {
		rc = PTR_ERR(vreg_ldo20);
		pr_err("%s: gp13 vreg get failed (%d)\n",
		       __func__, rc);
		return;
	}

	/* 1.8V -- LDO12 */
	vreg_ldo12 = vreg_get(NULL, "gp9");

	if (IS_ERR(vreg_ldo12) || vreg_ldo12 == NULL) {
		rc = PTR_ERR(vreg_ldo12);
		pr_err("%s: gp9 vreg get failed (%d)\n",
		       __func__, rc);
		return;
	}

	/* 2.6V -- LDO16 */
	vreg_ldo16 = vreg_get(NULL, "gp10");

	if (IS_ERR(vreg_ldo16) || vreg_ldo16 == NULL) {
		rc = PTR_ERR(vreg_ldo16);
		pr_err("%s: gp10 vreg get failed (%d)\n",
		       __func__, rc);
		return;
	}

	if (machine_is_msm7x30_fluid()) {
		/* 1.8V -- LDO8 */
		vreg_ldo8 = vreg_get(NULL, "gp7");

		if (IS_ERR(vreg_ldo8) || vreg_ldo8 == NULL) {
			rc = PTR_ERR(vreg_ldo8);
			pr_err("%s: gp7 vreg get failed (%d)\n",
				__func__, rc);
			return;
		}
	} else {
		/* lcd panel power */
		/* 3.1V -- LDO15 */
		vreg_ldo15 = vreg_get(NULL, "gp6");

		if (IS_ERR(vreg_ldo15) || vreg_ldo15 == NULL) {
			rc = PTR_ERR(vreg_ldo15);
			pr_err("%s: gp6 vreg get failed (%d)\n",
				__func__, rc);
			return;
		}
	}

	rc = vreg_set_level(vreg_ldo20, 1500);
	if (rc) {
		pr_err("%s: vreg LDO20 set level failed (%d)\n",
		       __func__, rc);
		return;
	}

	rc = vreg_set_level(vreg_ldo12, 1800);
	if (rc) {
		pr_err("%s: vreg LDO12 set level failed (%d)\n",
		       __func__, rc);
		return;
	}

	rc = vreg_set_level(vreg_ldo16, 2600);
	if (rc) {
		pr_err("%s: vreg LDO16 set level failed (%d)\n",
		       __func__, rc);
		return;
	}

	if (machine_is_msm7x30_fluid()) {
		rc = vreg_set_level(vreg_ldo8, 1800);
		if (rc) {
			pr_err("%s: vreg LDO8 set level failed (%d)\n",
				__func__, rc);
			return;
		}
	} else {
		rc = vreg_set_level(vreg_ldo15, 3100);
		if (rc) {
			pr_err("%s: vreg LDO15 set level failed (%d)\n",
				__func__, rc);
			return;
		}
	}

	if (on) {
		rc = vreg_enable(vreg_ldo20);
		if (rc) {
			pr_err("%s: LDO20 vreg enable failed (%d)\n",
			       __func__, rc);
			return;
		}

		rc = vreg_enable(vreg_ldo12);
		if (rc) {
			pr_err("%s: LDO12 vreg enable failed (%d)\n",
			       __func__, rc);
			return;
		}

		rc = vreg_enable(vreg_ldo16);
		if (rc) {
			pr_err("%s: LDO16 vreg enable failed (%d)\n",
			       __func__, rc);
			return;
		}

		if (machine_is_msm7x30_fluid()) {
			rc = vreg_enable(vreg_ldo8);
			if (rc) {
				pr_err("%s: LDO8 vreg enable failed (%d)\n",
					__func__, rc);
				return;
			}
		} else {
			rc = vreg_enable(vreg_ldo15);
			if (rc) {
				pr_err("%s: LDO15 vreg enable failed (%d)\n",
					__func__, rc);
				return;
			}
		}

		mdelay(5);		/* ensure power is stable */

		if (machine_is_msm7x30_fluid()) {
			// XXX enable vee reset gpio

			/* assert vee reset_n */
			gpio_set_value(20, 1);
			gpio_set_value(20, 0);
			mdelay(1);
			gpio_set_value(20, 1);
		}

		gpio_set_value(180, 1);	/* bring reset line high */
		mdelay(10);	/* 10 msec before IO can be accessed */

		rc = pm8058_gpio_mux(MSM7X30_PM8058_GPIO(37),
				     &msm7x30_mddi_sleep_clk_cfg_on);
		if (rc)
			pr_err("%s: pm8058_gpio_mux failure\n", __func__);
	} else {
		vreg_disable(vreg_ldo20);
		vreg_disable(vreg_ldo16);

		gpio_set_value(180, 0);	/* bring reset line low */

		if (machine_is_msm7x30_fluid())
			vreg_disable(vreg_ldo8);
		else
			vreg_disable(vreg_ldo15);

		mdelay(5);	/* ensure power is stable */

		vreg_disable(vreg_ldo12);

		if (machine_is_msm7x30_fluid()) {
			// XXX disable vee_reset_gpio
		}

		rc = pm8058_gpio_mux(MSM7X30_PM8058_GPIO(37),
				     &msm7x30_mddi_sleep_clk_cfg_off);
		if (rc)
			pr_err("%s: pm8058_gpio_mux failure\n", __func__);
	}
}

static void msm7x30_mddi_power_client(struct msm_mddi_client_data *mddi, int on)
{
	msm7x30_power_panel(on);
}

static int msm7x30_mddi_toshiba_client_init(
			struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	client_data->auto_hibernate(client_data, 0);
	msm7x30_process_mddi_table(client_data, mddi_toshiba_init_table,
				 ARRAY_SIZE(mddi_toshiba_init_table));
	client_data->auto_hibernate(client_data, 1);

	return 0;
}

static int msm7x30_mddi_toshiba_client_uninit(
			struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	return 0;
}

static int msm7x30_mddi_panel_unblank(
			struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	mutex_lock(&msm7x30_backlight_lock);
	msm7x30_set_backlight_level(msm7x30_backlight_brightness);
	msm7x30_backlight_off = 0;
	mutex_unlock(&msm7x30_backlight_lock);

	return 0;
}

static int msm7x30_mddi_panel_blank(
			struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	mutex_lock(&msm7x30_backlight_lock);
	msm7x30_set_backlight_level(0);
	msm7x30_backlight_off = 1;
	mutex_unlock(&msm7x30_backlight_lock);

	return 0;
}

static void msm7x30_brightness_set(struct led_classdev *led_cdev,
				     enum led_brightness value)
{
	mutex_lock(&msm7x30_backlight_lock);
	msm7x30_backlight_brightness = value;
	if (!msm7x30_backlight_off)
		msm7x30_set_backlight_level(msm7x30_backlight_brightness);
	mutex_unlock(&msm7x30_backlight_lock);
}

static struct led_classdev msm7x30_backlight_led = {
	.name		= "lcd-backlight",
	.brightness	= MSM7X30_SURF_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set	= msm7x30_brightness_set,
};

static int msm7x30_backlight_probe(struct platform_device *pdev)
{
	led_classdev_register(&pdev->dev, &msm7x30_backlight_led);
	return 0;
}

static int msm7x30_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm7x30_backlight_led);
	return 0;
}

static struct platform_driver msm7x30_backlight_driver = {
	.probe		= msm7x30_backlight_probe,
	.remove		= msm7x30_backlight_remove,
	.driver		= {
		.name		= "msm7x30-backlight",
		.owner		= THIS_MODULE,
	},
};

static struct resource resources_msm_fb[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

void __init msm7x30_allocate_fbmem(void)
{
	unsigned long base;
	unsigned long size;

	size = MSM_FB_SIZE;
	base = memblock_alloc(size, SZ_1M);
	memblock_free(base, size);
	memblock_remove(base, size);
	resources_msm_fb[0].start = base;
	resources_msm_fb[0].end = resources_msm_fb[0].start + size - 1;
	pr_info("%s: allocated %lu bytes at 0x%lx\n", __func__, size, base);
}

#define TOSHIBAWEGA_MFR_NAME	 0xd263
#define TOSHIBAWEGA_PRODUCT_CODE 0x8722

static void toshibawega_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	*mfr_name = TOSHIBAWEGA_MFR_NAME ;
	*product_code = TOSHIBAWEGA_PRODUCT_CODE ;
}

static struct msm_mddi_bridge_platform_data toshiba_client_data = {
	.init = msm7x30_mddi_toshiba_client_init,
	.uninit = msm7x30_mddi_toshiba_client_uninit,
	.blank = msm7x30_mddi_panel_blank,
	.unblank = msm7x30_mddi_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 45,
		.height = 67,
		.output_format = MSM_MDP_OUT_IF_FMT_RGB888,
	},
};

static struct msm_mddi_platform_data mddi_pdata = {
	.clk_rate = 445500000,
	.fixup = toshibawega_fixup,
	.power_client = msm7x30_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0xd263 << 16 | 0x8722),
			.name = "mddi_c_simple",
			.id = 0,
			.client_data = &toshiba_client_data,
			.clk_rate = 0,
		},
	},
};

/********************** FLUID LCDC PANEL CODE */
struct {
	u8 addr;
	u8 data;
} fluid_sharp_init_tbl[] = {
	{  15, 0x01 },
	{   5, 0x01 },
	{   7, 0x10 },
	{   9, 0x1E },
	{  10, 0x04 },
	{  17, 0xFF },
	{  21, 0x8A },
	{  22, 0x00 },
	{  23, 0x82 },
	{  24, 0x24 },
	{  25, 0x22 },
	{  26, 0x6D },
	{  27, 0xEB },
	{  28, 0xB9 },
	{  29, 0x3A },
	{  49, 0x1A },
	{  50, 0x16 },
	{  51, 0x05 },
	{  55, 0x7F },
	{  56, 0x15 },
	{  57, 0x7B },
	{  60, 0x05 },
	{  61, 0x0C },
	{  62, 0x80 },
	{  63, 0x00 },
	{  92, 0x90 },
	{  97, 0x01 },
	{  98, 0xFF },
	{ 113, 0x11 },
	{ 114, 0x02 },
	{ 115, 0x08 },
	{ 123, 0xAB },
	{ 124, 0x04 },
	{   6, 0x02 },
	{ 133, 0x00 },
	{ 134, 0xFE },
	{ 135, 0x22 },
	{ 136, 0x0B },
	{ 137, 0xFF },
	{ 138, 0x0F },
	{ 139, 0x00 },
	{ 140, 0xFE },
	{ 141, 0x22 },
	{ 142, 0x0B },
	{ 143, 0xFF },
	{ 144, 0x0F },
	{ 145, 0x00 },
	{ 146, 0xFE },
	{ 147, 0x22 },
	{ 148, 0x0B },
	{ 149, 0xFF },
	{ 150, 0x0F },
	{ 202, 0x30 },
	{  30, 0x01 },
	{   4, 0x01 },
	{  31, 0x41 },
};

static struct spi_device *lcdc_spi_client;

static int fluid_spi_write(u8 reg, u8 data)
{
	u8 tx_buf[2];
	int rc;
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = tx_buf,
	};

	if (!lcdc_spi_client) {
		pr_err("%s: lcdc_spi_client is NULL\n", __func__);
		return -EINVAL;
	}

	spi_setup(lcdc_spi_client);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = reg;
	tx_buf[1] = data;
	t.rx_buf = NULL;
	t.len = 2;
	rc = spi_sync(lcdc_spi_client, &m);
	return rc;
}

int fluid_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	/* TODO: Turn backlight off? */
	return 0;
}

int fluid_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	/* TODO: Turn backlight on? */
	return 0;
}

int fluid_panel_init(struct msm_lcdc_panel_ops *ops)
{
	int i;

	msm7x30_power_panel(true);

	for (i = 0; i < ARRAY_SIZE(fluid_sharp_init_tbl); i++)
		fluid_spi_write(fluid_sharp_init_tbl[i].addr,
				fluid_sharp_init_tbl[i].data);
	mdelay(10);
	fluid_spi_write(31, 0xC1);
	mdelay(10);
	fluid_spi_write(31, 0xD9);
	fluid_spi_write(31, 0xDF);

	return 0;
}

static struct msm_lcdc_timing fluid_lcdc_timing = {
	.clk_rate		= 24576000,
	.hsync_pulse_width	= 10,
	.hsync_back_porch	= 20,
	.hsync_front_porch	= 10,
	.hsync_skew		= 0,
	.vsync_pulse_width	= 2,
	.vsync_back_porch	= 2,
	.vsync_front_porch	= 2,
	.vsync_act_low		= 1,
	.hsync_act_low		= 1,
	.den_act_low		= 0,
};

static struct msm_fb_data fluid_lcdc_fb_data = {
	.xres		= 480,
	.yres		= 800,
	.width		= 57,
	.height		= 94,
	.output_format	= MSM_MDP_OUT_IF_FMT_RGB666,
};

static struct msm_lcdc_panel_ops fluid_lcdc_panel_ops = {
	.init		= fluid_panel_init,
	.blank		= fluid_panel_blank,
	.unblank	= fluid_panel_unblank,
};

static struct msm_lcdc_platform_data fluid_lcdc_platform_data = {
	.panel_ops	= &fluid_lcdc_panel_ops,
	.timing		= &fluid_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &fluid_lcdc_fb_data,
	.fb_resource	= resources_msm_fb,
};

static struct platform_device fluid_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &fluid_lcdc_platform_data,
	},
};

static int __devinit fluid_lcdc_sharp_spi_probe(struct spi_device *spi)
{
	int rc;

	lcdc_spi_client = spi;
	lcdc_spi_client->bits_per_word = 32;

	rc = platform_device_register(&fluid_lcdc_device);
	if (rc)
		return rc;
	return 0;
}

static int __devexit fluid_lcdc_sharp_spi_remove(struct spi_device *spi)
{
	lcdc_spi_client = NULL;
	return 0;
}

static struct spi_driver fluid_lcdc_sharp_spi_driver = {
	.driver = {
		.name  = "fluid_lcdc_sharp",
		.owner = THIS_MODULE,
	},
	.probe		= fluid_lcdc_sharp_spi_probe,
	.remove		= __devexit_p(fluid_lcdc_sharp_spi_remove),
};

static struct spi_board_info fluid_lcdc_sharp_spi_board_info[] __initdata = {
	{
		.modalias	= "fluid_lcdc_sharp",
		.mode		= SPI_MODE_1,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 26331429,
	}
};

static struct platform_device msm7x30_backlight = {
	.name = "msm7x30-backlight",
};

int __init msm7x30_init_panel(void)
{
	int rc;

	msm_gpiomux_write(180, 0,
			  GPIOMUX_FUNC_GPIO |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_OUTPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);
	msm_gpiomux_write(20, 0,
			  GPIOMUX_FUNC_GPIO |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_OUTPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);

	if (machine_is_msm7x30_fluid()) {
		int i;
		int mux_val = GPIOMUX_FUNC_1 | GPIOMUX_PULL_NONE |
			GPIOMUX_DIR_OUTPUT |
			GPIOMUX_DRV_2MA | GPIOMUX_VALID;

		msm_gpiomux_write(22, 0, mux_val);
		msm_gpiomux_write(25, 0, mux_val);

		for (i = 90; i <= 109; i++)
			msm_gpiomux_write(i, 0, mux_val);
	}

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	if (machine_is_msm7x30_fluid()) {
		rc = spi_register_driver(&fluid_lcdc_sharp_spi_driver);
		if (rc)
			return rc;
		spi_register_board_info(fluid_lcdc_sharp_spi_board_info,
			ARRAY_SIZE(fluid_lcdc_sharp_spi_board_info));
		pm8058_gpio_mux(MSM7X30_PM8058_GPIO(25),
				&msm7x30_fluid_backlight);
		gpio_request(MSM7X30_PM8058_GPIO(25), "lcd_backlight");
	} else {
		msm_device_mddi0.dev.platform_data = &mddi_pdata;
		rc = platform_device_register(&msm_device_mddi0);
		if (rc)
			return rc;
	}

	platform_device_register(&msm7x30_backlight);
	return platform_driver_register(&msm7x30_backlight_driver);
}

device_initcall(msm7x30_init_panel);
