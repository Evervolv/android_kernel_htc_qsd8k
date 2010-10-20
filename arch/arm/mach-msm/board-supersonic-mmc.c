/* linux/arch/arm/mach-msm/board-supersonic-mmc.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>

#include "board-supersonic.h"
#include "devices.h"
#include "proc_comm.h"

#define DEBUG_SDSLOT_VDD 1

static bool opt_disable_sdcard;
static int __init supersonic_disablesdcard_setup(char *str)
{
	opt_disable_sdcard = (bool)simple_strtol(str, NULL, 0);
	return 1;
}

__setup("board_supersonic.disable_sdcard=", supersonic_disablesdcard_setup);

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static struct vreg	*sdslot_vreg;
static uint32_t		sdslot_vdd = 0xffffffff;
static uint32_t		sdslot_vreg_enabled;

static struct {
	int mask;
	int level;
} mmc_vdd_table[] = {
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};

static uint32_t supersonic_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;
	int ret;

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
#if DEBUG_SDSLOT_VDD
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
#endif
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(sdslot_vreg);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		ret = vreg_enable(sdslot_vreg);
		if (ret)
			pr_err("%s: Error enabling vreg (%d)\n", __func__, ret);
		udelay(500);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask != (1 << vdd))
			continue;
		ret = vreg_set_level(sdslot_vreg, mmc_vdd_table[i].level);
		if (ret)
			pr_err("%s: Error setting level (%d)\n", __func__, ret);
#if DEBUG_SDSLOT_VDD
		printk(KERN_INFO "%s: Setting level to %u (%s)\n",
					__func__, mmc_vdd_table[i].level,
				ret?"Failed":"Success");
#endif
		return 0;
	}

	pr_err("%s: Invalid VDD (%d) specified\n", __func__, vdd);
	return 0;
}

static unsigned int supersonic_sdslot_status(struct device *dev)
{
	return (system_rev > 0)?1:!gpio_get_value(SUPERSONIC_GPIO_SDMC_CD_N);
}

#define SUPERSONIC_MMC_VDD	(MMC_VDD_28_29 | MMC_VDD_29_30)

static unsigned int supersonic_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data supersonic_sdslot_data = {
	.ocr_mask	= SUPERSONIC_MMC_VDD,
	.status		= supersonic_sdslot_status,
	.translate_vdd	= supersonic_sdslot_switchvdd,
//	.slot_type	= &supersonic_sdslot_type,
};

int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
		 unsigned int stat_irq, unsigned long stat_irq_flags);



/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(51, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(56, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* WLAN IRQ */
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data supersonic_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
supersonic_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int supersonic_wifi_cd;	/* WiFi virtual 'card detect' status */

static unsigned int supersonic_wifi_status(struct device *dev)
{
	return supersonic_wifi_cd;
}

static struct mmc_platform_data supersonic_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= supersonic_wifi_status,
	.register_status_notify	= supersonic_wifi_status_register,
	.embedded_sdio		= &supersonic_wifi_emb_data,
};

int supersonic_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	supersonic_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(supersonic_wifi_set_carddetect);

int supersonic_wifi_power(int on)
{
	int rc = 0;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
		mdelay(50);
		if (rc)
			return rc;
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
	}

	mdelay(100);
	gpio_set_value(SUPERSONIC_GPIO_WIFI_SHUTDOWN_N, on); /* WIFI_SHUTDOWN */
	mdelay(100);
	return 0;
}
EXPORT_SYMBOL(supersonic_wifi_power);

int supersonic_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}


/* ---------------- WiMAX GPIO Settings --------------- */
static uint32_t wimax_power_pin_gpio_table[] = {
	PCOM_GPIO_CFG(48, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(106, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(154, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(155, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
    PCOM_GPIO_CFG(156, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA)
};

static uint32_t wimax_on_gpio_table[] = {
	PCOM_GPIO_CFG(88, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	 /*WiMax_Host_2*/
	PCOM_GPIO_CFG(159, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
};

static uint32_t wimax_off_gpio_table[] = {
	PCOM_GPIO_CFG(88, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	/*WiMax_Host_2*/
	PCOM_GPIO_CFG(159, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
};


static void (*wimax_status_cb)(int card_present, void *dev_id);
static void *wimax_status_cb_devid;
static int supersonic_wimax_cd = 0;
static int supersonic_wimax_sdio_status = 0;

static int supersonic_wimax_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	if (wimax_status_cb)
		return -EAGAIN;
	printk("%s\n", __func__);
	wimax_status_cb = callback;
	wimax_status_cb_devid = dev_id;
	return 0;
}

static unsigned int supersonic_wimax_status(struct device *dev)
{
	printk("%s\n", __func__);
	return supersonic_wimax_cd;
}

void supersonic_wimax_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	supersonic_wimax_cd = val;
	if (wimax_status_cb) {
		wimax_status_cb(val, wimax_status_cb_devid);
	} else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
}
EXPORT_SYMBOL(supersonic_wimax_set_carddetect);

static struct mmc_platform_data supersonic_wimax_data = {
	.ocr_mask		= MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.status			= supersonic_wimax_status,
	.register_status_notify	= supersonic_wimax_status_register,
	.embedded_sdio		= NULL,
};

struct _vreg
{
	const char *name;
	unsigned id;
};


/* 2 : wimax UART, 1 : CPU uart, 0 : usb
CPU_WIMAX_SW -> GPIO160  (SUPERSONIC_WIMAX_CPU_UARTz_SW)
USB_UART#_SW -> GPIO33  (SUPERSONIC_USB_UARTz_SW)

XA : GPIO33 = 0 -> USB
    GPIO33 = 1 -> CPU UART

XB : GPIO33 = 0 -> USB
    GPIO33 = 1 , GPIO160 = 0 -> CPU UART
    GPIO33 = 1 , GPIO160 = 1 -> Wimax UART
*/
int supersonic_wimax_uart_switch(int uart)
{
	printk("%s uart:%d\n", __func__, uart);
	
	gpio_set_value(SUPERSONIC_USB_UARTz_SW, uart?1:0);
	if(system_rev && uart)
		gpio_set_value(SUPERSONIC_WIMAX_CPU_UARTz_SW, uart==2?1:0);
	return uart?1:0; 
}
EXPORT_SYMBOL(supersonic_wimax_uart_switch);

int supersonic_wimax_power(int on)
{
	printk("%s\n", __func__);

	if (on) {
		/*Power ON sequence*/
		gpio_set_value(154, 1);
		gpio_set_value(48, 1);
                mdelay(5);
		gpio_set_value(106, 0);
                gpio_set_value(156, 1);
                gpio_set_value(155, 1);
 		mdelay(5);
		gpio_set_value(106, 1);
 		mdelay(1150);

		config_gpio_table(wimax_on_gpio_table,
				  ARRAY_SIZE(wimax_on_gpio_table));
	} else {
		/*Power OFF sequence*/
		config_gpio_table(wimax_off_gpio_table,
				  ARRAY_SIZE(wimax_off_gpio_table));
		gpio_set_value(88, 0);	/*WiMax_SDIO_CLK_1 OL*/
		gpio_set_value(159, 0);	/*WiMax_Host_2 OL*/

		gpio_set_value(106, 1);
		mdelay(5);
		gpio_set_value(156, 0);
		gpio_set_value(155, 0);
                gpio_set_value(106, 0);
		mdelay(5);
		gpio_set_value(154, 0);
		gpio_set_value(48, 0);
		mdelay(5);
	}
	return 0;
}
EXPORT_SYMBOL(supersonic_wimax_power);

int supersonic_wimax_set_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	supersonic_wimax_sdio_status = on;
	return 0;
}
EXPORT_SYMBOL(supersonic_wimax_set_status);

int supersonic_wimax_get_status()
{
	//printk(KERN_INFO "%s status:%d\n", __func__, supersonic_wimax_sdio_status);
	return supersonic_wimax_sdio_status;
}
EXPORT_SYMBOL(supersonic_wimax_get_status);

int __init supersonic_init_mmc(unsigned int sys_rev)
{
	uint32_t id;

	wifi_status_cb = NULL;

	printk(KERN_INFO "%s()+\n", __func__);

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(SUPERSONIC_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	msm_add_sdcc(1, &supersonic_wifi_data, 0, 0);

        /* Initial WiMAX */
	printk("config wimax power gpio table\n");
	config_gpio_table(wimax_power_pin_gpio_table,
			  ARRAY_SIZE(wimax_power_pin_gpio_table));

	msm_add_sdcc(3, &supersonic_wimax_data,0,0);

	if (opt_disable_sdcard) {
		pr_info("%s: sdcard disabled on cmdline\n", __func__);
		goto done;
	}

	sdslot_vreg_enabled = 0;

	sdslot_vreg = vreg_get(0, "gp6");
	if (IS_ERR(sdslot_vreg))
		return PTR_ERR(sdslot_vreg);

	if (system_rev == 0) { /* XA board */
		set_irq_wake(MSM_GPIO_TO_INT(SUPERSONIC_GPIO_SDMC_CD_N), 1);

		msm_add_sdcc(2, &supersonic_sdslot_data,
			MSM_GPIO_TO_INT(SUPERSONIC_GPIO_SDMC_CD_N),
			IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);
	} else
		msm_add_sdcc(2, &supersonic_sdslot_data, 0, 0);

done:
	printk(KERN_INFO "%s()-\n", __func__);
	return 0;
}
