/* linux/arch/arm/mach-msm/board-htcleo-mmc.c
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
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>
#include <mach/gpio.h>
#include <mach/board-htcleo-mmc.h>

#include "board-htcleo.h"
#include "devices.h"
#include "proc_comm.h"
#include "pmic.h"

#undef HTCLEO_DEBUG_MMC

static int __init htcleo_disablesdcard_setup(char *str)
{
	opt_disable_sdcard = (bool)simple_strtol(str, NULL, 0);
	return 1;
}

__setup("board_htcleo.disable_sdcard=", htcleo_disablesdcard_setup);

static uint32_t  sdcard_on_gpio_table[] =
{
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
};

static uint32_t  sdcard_off_gpio_table[] =
{
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static struct vreg	*sdslot_vreg;
static struct vreg	*wlan_vreg_3;
static uint32_t		sdslot_vdd = 0xffffffff;
static uint32_t		sdslot_vreg_enabled;

static struct {
	int mask;
	int level;
} mmc_vdd_table[] = {
	{ MMC_VDD_165_195,	1800 },
	{ MMC_VDD_20_21,	2050 },
	{ MMC_VDD_21_22,	2150 },
	{ MMC_VDD_22_23,	2250 },
	{ MMC_VDD_23_24,	2350 },
	{ MMC_VDD_24_25,	2450 },
	{ MMC_VDD_25_26,	2550 },
	{ MMC_VDD_26_27,	2650 },
	{ MMC_VDD_27_28,	2750 },
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2950 },
};

static uint32_t htcleo_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	// We have to find the right vreg Interface on Leo
	int i;
	int ret = 0;

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	printk("@@@ SD power %d @@@\n", vdd);

	if (vdd == 0) 
	{
		config_gpio_table(sdcard_off_gpio_table,ARRAY_SIZE(sdcard_off_gpio_table));             
		vreg_disable(sdslot_vreg);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) 
	{
		ret = vreg_enable(sdslot_vreg);
		if (ret)
			pr_err("%s: Error enabling vreg (%d)\n", __func__, ret);
		config_gpio_table(sdcard_on_gpio_table, ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask != (1 << vdd))
			continue;
		ret = vreg_set_level(sdslot_vreg, mmc_vdd_table[i].level);
		if (ret)
			pr_err("%s: Error setting level (%d)\n", __func__, ret);
		return 0;
	}

	pr_err("%s: Invalid VDD (%d) specified\n", __func__, vdd);
	return 0;
}

static unsigned int htcleo_sdslot_status(struct device *dev)
{
	return !gpio_get_value(HTCLEO_GPIO_SD_STATUS);
}

#define HTCLEO_MMC_VDD	(MMC_VDD_165_195 | MMC_VDD_20_21 | \
				 MMC_VDD_21_22  | MMC_VDD_22_23 | \
				 MMC_VDD_23_24 | MMC_VDD_24_25 | \
				 MMC_VDD_25_26 | MMC_VDD_26_27 | \
				 MMC_VDD_27_28 | MMC_VDD_28_29 | \
				 MMC_VDD_29_30)

static struct mmc_platform_data htcleo_sdslot_data =
{
	.ocr_mask		= HTCLEO_MMC_VDD,
	.status			= htcleo_sdslot_status,
	.register_status_notify	= NULL,
	.translate_vdd		= htcleo_sdslot_switchvdd,
};

static uint32_t wifi_on_gpio_table[] =
{
	PCOM_GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] =
{
	PCOM_GPIO_CFG(51, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(56, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data htcleo_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
#if defined(CONFIG_BCMDHD)
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
#endif
};

static int htcleo_wifi_cd = 0; /* WIFI virtual 'card detect' status */

static int htcleo_wifi_status_register(
			void (*callback)(int card_present, void *dev_id),
			void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int htcleo_wifi_status(struct device *dev)
{
	return htcleo_wifi_cd;
}

static struct mmc_platform_data htcleo_wifi_data = {
	/* 
	 * lowered wifi vdd to 2650 for now, might test lower vdds later
	 * incredible uses 2050 and seems to work without issues
	 * by marc1706
	 */
	.ocr_mask		= MMC_VDD_26_27,
	.built_in		= 1,
	.status			= htcleo_wifi_status,
	.register_status_notify	= htcleo_wifi_status_register,
	.embedded_sdio		= &htcleo_wifi_emb_data,
};

int htcleo_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	htcleo_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

int htcleo_wifi_power(int on)
{
	printk("%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,ARRAY_SIZE(wifi_on_gpio_table));
		mdelay(50);
		vreg_enable(wlan_vreg_3);
	} else {
		config_gpio_table(wifi_off_gpio_table, ARRAY_SIZE(wifi_off_gpio_table));
		mdelay(100);
		vreg_disable(wlan_vreg_3);
	}
	
	mdelay(100);
	gpio_set_value(HTCLEO_GPIO_WIFI_SHUTDOWN_N, on); /* WIFI_SHUTDOWN */
	mdelay(100);

	htcleo_wifi_power_state = on;
	return 0;
}

int htcleo_wifi_reset(int on)
{
	printk("%s: do nothing\n", __func__);
	htcleo_wifi_reset_state = on;
	return 0;
}

int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
		 unsigned int stat_irq, unsigned long stat_irq_flags);

int __init htcleo_init_mmc(unsigned debug_uart)
{
	uint32_t id;

	printk("%s()+\n", __func__);

	/* initial WIFI_SHUTDOWN# */	
	id = PCOM_GPIO_CFG(HTCLEO_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, NULL);
	gpio_set_value(HTCLEO_GPIO_WIFI_SHUTDOWN_N, 0);

	msm_add_sdcc(1, &htcleo_wifi_data, 0, 0);

	if (debug_uart) {
		pr_info("%s: sdcard disabled due to debug uart\n", __func__);
		goto done;
	}
	if (opt_disable_sdcard) {
		pr_info("%s: sdcard disabled on cmdline\n", __func__);
		goto done;
	}

	sdslot_vreg_enabled = 0;

	sdslot_vreg = vreg_get(NULL, "gp6");
	if (IS_ERR(sdslot_vreg))
		return PTR_ERR(sdslot_vreg);
//	wlan_vreg_1 = PM_VREG_WLAN_ID;
//	wlan_vreg_2 = PM_VREG_MSME1_ID;

	wlan_vreg_3 = vreg_get(NULL, "rftx");
	if (IS_ERR(wlan_vreg_3))
		return PTR_ERR(wlan_vreg_3);

	set_irq_wake(MSM_GPIO_TO_INT(HTCLEO_GPIO_SD_STATUS), 1);
	msm_add_sdcc(2, &htcleo_sdslot_data,
		      MSM_GPIO_TO_INT(HTCLEO_GPIO_SD_STATUS),
		      IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);

done:
	printk("%s()-\n", __func__);
	return 0;
}

#if defined(HTCLEO_DEBUG_MMC) && defined(CONFIG_DEBUG_FS)

static int htcleommc_dbg_wifi_reset_set(void *data, u64 val)
{
	htcleo_wifi_reset((int) val);
	return 0;
}

static int htcleommc_dbg_wifi_reset_get(void *data, u64 *val)
{
	*val = htcleo_wifi_reset_state;
	return 0;
}

static int htcleommc_dbg_wifi_cd_set(void *data, u64 val)
{
	htcleo_wifi_set_carddetect((int) val);
	return 0;
}

static int htcleommc_dbg_wifi_cd_get(void *data, u64 *val)
{
	*val = htcleo_wifi_cd;
	return 0;
}

static int htcleommc_dbg_wifi_pwr_set(void *data, u64 val)
{
	htcleo_wifi_power((int) val);
	return 0;
}

static int htcleommc_dbg_wifi_pwr_get(void *data, u64 *val)
{
	*val = htcleo_wifi_power_state;
	return 0;
}

static int htcleommc_dbg_sd_pwr_set(void *data, u64 val)
{
	htcleo_sdslot_switchvdd(NULL, (unsigned int) val);
	return 0;
}

static int htcleommc_dbg_sd_pwr_get(void *data, u64 *val)
{
	*val = sdslot_vdd;
	return 0;
}

static int htcleommc_dbg_sd_cd_set(void *data, u64 val)
{
	return -ENOSYS;
}

static int htcleommc_dbg_sd_cd_get(void *data, u64 *val)
{
	*val = htcleo_sdslot_data.status(NULL);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(htcleommc_dbg_wifi_reset_fops,
			htcleommc_dbg_wifi_reset_get,
			htcleommc_dbg_wifi_reset_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(htcleommc_dbg_wifi_cd_fops,
			htcleommc_dbg_wifi_cd_get,
			htcleommc_dbg_wifi_cd_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(htcleommc_dbg_wifi_pwr_fops,
			htcleommc_dbg_wifi_pwr_get,
			htcleommc_dbg_wifi_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(htcleommc_dbg_sd_pwr_fops,
			htcleommc_dbg_sd_pwr_get,
			htcleommc_dbg_sd_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(htcleommc_dbg_sd_cd_fops,
			htcleommc_dbg_sd_cd_get,
			htcleommc_dbg_sd_cd_set, "%llu\n");

static int __init htcleommc_dbg_init(void)
{
	struct dentry *dent;

	if (!machine_is_htcleo() && !machine_is_htcleo())
		return 0;

	dent = debugfs_create_dir("htcleo_mmc_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("wifi_reset", 0644, dent, NULL,
			    &htcleommc_dbg_wifi_reset_fops);
	debugfs_create_file("wifi_cd", 0644, dent, NULL,
			    &htcleommc_dbg_wifi_cd_fops);
	debugfs_create_file("wifi_pwr", 0644, dent, NULL,
			    &htcleommc_dbg_wifi_pwr_fops);
	debugfs_create_file("sd_pwr", 0644, dent, NULL,
			    &htcleommc_dbg_sd_pwr_fops);
	debugfs_create_file("sd_cd", 0644, dent, NULL,
			    &htcleommc_dbg_sd_cd_fops);
	return 0;
}

device_initcall(htcleommc_dbg_init);
#endif
