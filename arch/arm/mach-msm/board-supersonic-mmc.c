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
#include <asm/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/mmc.h>

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

static struct msm_mmc_platform_data supersonic_sdslot_data = {
	.ocr_mask	= SUPERSONIC_MMC_VDD,
	.status		= supersonic_sdslot_status,
	.translate_vdd	= supersonic_sdslot_switchvdd,
	.slot_type	= &supersonic_sdslot_type,
};

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
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
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

static struct msm_mmc_platform_data supersonic_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.built_in		= 1,
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

static int supersonic_wifi_power_state;

int supersonic_wifi_power(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
		mdelay(50);
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
	}

	mdelay(100);
	gpio_set_value(SUPERSONIC_GPIO_WIFI_SHUTDOWN_N, on); /* WIFI_SHUTDOWN */
	mdelay(200);

	supersonic_wifi_power_state = on;
	return 0;
}

static int supersonic_wifi_reset_state;

int supersonic_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	supersonic_wifi_reset_state = on;
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
static int mmc_wimax_cd = 0;
static int mmc_wimax_sdio_status = 0;
static int mmc_wimax_netlog_status = 0;
static int mmc_wimax_sdio_interrupt_log_status = 0;
static int mmc_wimax_netlog_withraw_status = 0;
static int mmc_wimax_cliam_host_status = 0;
static int mmc_wimax_busclk_pwrsave = 1; // Default is dynamic CLK OFF
static int mmc_wimax_CMD53_timeout_trigger_counter = 0;
static int mmc_wimax_hostwakeup_gpio = 40; // GPIO40
static int mmc_wimax_thp_log_status = 0;
static int mmc_wimax_sdio_hw_reset = 0; // Rollback to default disabled HW RESET
static int mmc_wimax_packet_filter = 0; 

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
	return mmc_wimax_cd;
}

void mmc_wimax_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	mmc_wimax_cd = val;
	if (wimax_status_cb) {
		wimax_status_cb(val, wimax_status_cb_devid);
	} else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
}
EXPORT_SYMBOL(mmc_wimax_set_carddetect);

static unsigned int supersonic_wimax_type = MMC_TYPE_SDIO_WIMAX;

static struct msm_mmc_platform_data supersonic_wimax_data = {
	.ocr_mask		= MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.built_in		= 1,
	.status			= supersonic_wimax_status,
	.register_status_notify	= supersonic_wimax_status_register,
	.embedded_sdio		= NULL,
	.slot_type		= &supersonic_wimax_type,
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
int wimax_uart_switch = 0;
int mmc_wimax_uart_switch(int uart)
{
	printk("%s uart:%d\n", __func__, uart);
	wimax_uart_switch = uart;

	gpio_set_value(SUPERSONIC_USB_UARTz_SW, uart?1:0);
	if(system_rev && uart)
		gpio_set_value(SUPERSONIC_WIMAX_CPU_UARTz_SW, uart==2?1:0);
	return uart?1:0; 
}
EXPORT_SYMBOL(mmc_wimax_uart_switch);

int mmc_wimax_get_uart_switch(void)
{
	printk("%s uart:%d\n", __func__, wimax_uart_switch);
	return wimax_uart_switch?1:0; 	
}
EXPORT_SYMBOL(mmc_wimax_get_uart_switch);

static int supersonic_wimax_power_state;


int mmc_wimax_power(int on)
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
supersonic_wimax_power_state = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_power);

int mmc_wimax_set_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_status);

int mmc_wimax_get_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_sdio_status;
}
EXPORT_SYMBOL(mmc_wimax_get_status);

int mmc_wimax_set_cliam_host_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_cliam_host_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_cliam_host_status);

int mmc_wimax_get_cliam_host_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_cliam_host_status;
}
EXPORT_SYMBOL(mmc_wimax_get_cliam_host_status);

int mmc_wimax_set_netlog_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_netlog_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_netlog_status);

int mmc_wimax_get_netlog_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_netlog_status;
}
EXPORT_SYMBOL(mmc_wimax_get_netlog_status);

int mmc_wimax_set_netlog_withraw_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_netlog_withraw_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_netlog_withraw_status);

int mmc_wimax_get_netlog_withraw_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_netlog_withraw_status);
	return mmc_wimax_netlog_withraw_status;
}
EXPORT_SYMBOL(mmc_wimax_get_netlog_withraw_status);

int mmc_wimax_set_sdio_interrupt_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_interrupt_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdio_interrupt_log);

int mmc_wimax_get_sdio_interrupt_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_interrupt_log_status);
	return mmc_wimax_sdio_interrupt_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_sdio_interrupt_log);

int mmc_wimax_set_packet_filter(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_packet_filter = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_packet_filter);

int mmc_wimax_get_packet_filter(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_packet_filter);
	return mmc_wimax_packet_filter;
}
EXPORT_SYMBOL(mmc_wimax_get_packet_filter);

int mmc_wimax_set_thp_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_thp_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_thp_log);

int mmc_wimax_get_thp_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_thp_log_status);
	return mmc_wimax_thp_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_thp_log);

int mmc_wimax_set_busclk_pwrsave(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_busclk_pwrsave = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_busclk_pwrsave);

int mmc_wimax_get_busclk_pwrsave(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_busclk_pwrsave);
	return mmc_wimax_busclk_pwrsave;
}
EXPORT_SYMBOL(mmc_wimax_get_busclk_pwrsave);

int mmc_wimax_set_sdio_hw_reset(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_hw_reset = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdio_hw_reset);

int mmc_wimax_get_sdio_hw_reset(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_hw_reset);
	return mmc_wimax_sdio_hw_reset;
}
EXPORT_SYMBOL(mmc_wimax_get_sdio_hw_reset);

int mmc_wimax_set_CMD53_timeout_trigger_counter(int counter)
{
	printk(KERN_INFO "%s counter:%d\n", __func__, counter);
	mmc_wimax_CMD53_timeout_trigger_counter = counter;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_CMD53_timeout_trigger_counter);

int mmc_wimax_get_CMD53_timeout_trigger_counter(void)
{
	//printk(KERN_INFO "%s counter:%d\n", __func__, mmc_wimax_CMD53_timeout_trigger_counter);
	return mmc_wimax_CMD53_timeout_trigger_counter;
}
EXPORT_SYMBOL(mmc_wimax_get_CMD53_timeout_trigger_counter);

int mmc_wimax_get_hostwakeup_gpio(void)
{
	return mmc_wimax_hostwakeup_gpio;
}
EXPORT_SYMBOL(mmc_wimax_get_hostwakeup_gpio);

static int mmc_wimax_is_gpio_irq_enabled = 0;

int mmc_wimax_set_gpio_irq_enabled(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_is_gpio_irq_enabled = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_gpio_irq_enabled);

int mmc_wimax_get_gpio_irq_enabled(void)
{
	return mmc_wimax_is_gpio_irq_enabled;
}
EXPORT_SYMBOL(mmc_wimax_get_gpio_irq_enabled);

void mmc_wimax_enable_host_wakeup(int on)
{
	if (mmc_wimax_sdio_status)
	{	
		if (on) {
			if (!mmc_wimax_is_gpio_irq_enabled) {
				printk("set GPIO%d as waketup source\n", mmc_wimax_get_hostwakeup_gpio());
				enable_irq(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));
				enable_irq_wake(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));
				mmc_wimax_is_gpio_irq_enabled = 1;
			}
		}
		else {
			if (mmc_wimax_is_gpio_irq_enabled) {
				printk("disable GPIO%d wakeup source\n", mmc_wimax_get_hostwakeup_gpio());
				disable_irq_wake(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));				
				disable_irq_nosync(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));
				mmc_wimax_is_gpio_irq_enabled = 0;
			}
		}
	}
	else {
		printk("%s mmc_wimax_sdio_status is OFF\n", __func__);
	}
}
EXPORT_SYMBOL(mmc_wimax_enable_host_wakeup);

int __init supersonic_init_mmc(unsigned int sys_rev)
{
	uint32_t id;

	printk(KERN_INFO "%s()+\n", __func__);

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(SUPERSONIC_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	gpio_set_value(SUPERSONIC_GPIO_WIFI_SHUTDOWN_N, 0);

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
