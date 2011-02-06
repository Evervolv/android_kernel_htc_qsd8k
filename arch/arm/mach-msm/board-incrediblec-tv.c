/*
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/msm_mdp.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>

#include "board-incrediblec.h"
#include "devices.h"
#include "proc_comm.h"

static struct resource msm_tvenc_resources[] = {
	{
		.name   = "msm_tv",
		.start  = 0xaa400000,
		.end    = 0xaa400000 + 0x1000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start	= MSM_TV_FB_BASE,
		.end	= MSM_TV_FB_BASE + MSM_TV_FB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	}
};

static struct msm_fb_data incrediblec_tvenc_fb_data = {
	.xres           = 480,
	.yres           = 720,
	/* Typical geometry of 17" CRT */
	.width          = 338,
	.height         = 270,
	.output_format  = MDP_YCRYCB_H2V1,
};

static int incrediblec_tv_video_relay(int on_off)
{
	pr_info("[tv]: %s(%d)\n", __func__, on_off);
	on_off = !!on_off;
	gpio_set_value(INCREDIBLEC_VIDEO_SHDN_N, on_off);
	if (system_rev < 2)
		gpio_set_value(INCREDIBLEC_AV_SWITCH, on_off);

	return 0;
}

static struct msm_tvenc_platform_data incrediblec_tvenc_platform_data = {
	.fb_id          = 1,
	.fb_data        = &incrediblec_tvenc_fb_data,
	.fb_resource    = &msm_tvenc_resources[1],
	.video_relay	= &incrediblec_tv_video_relay,
};

static struct platform_device msm_tvenc_device = {
	.name   	= "msm_tv",
	.id     	= 0,
	.num_resources  = ARRAY_SIZE(msm_tvenc_resources),
	.resource       = msm_tvenc_resources,
	.dev    = {
		.platform_data = &incrediblec_tvenc_platform_data,
	},
};

int __init incrediblec_init_tv(void)
{
	int ret, engid;
	uint32_t config;

	if (!machine_is_incrediblec())
		return 0;

	engid = incrediblec_get_engineerid();
	if (0 == engid || 0xF == engid) {
		msm_tvenc_resources[1].start = MSM_TV_FB_XA_BASE;
		msm_tvenc_resources[1].end = msm_tvenc_resources[1].start +
			MSM_TV_FB_SIZE - 1;
	} else if (engid >= 3) {
		msm_tvenc_resources[1].start =
			MSM_TV_FB_BASE + MSM_MEM_128MB_OFFSET;
		msm_tvenc_resources[1].end = msm_tvenc_resources[1].start +
			MSM_TV_FB_SIZE - 1;
	}

	config = PCOM_GPIO_CFG(INCREDIBLEC_VIDEO_SHDN_N, 0, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	gpio_set_value(INCREDIBLEC_VIDEO_SHDN_N, 1);
	config = PCOM_GPIO_CFG(INCREDIBLEC_TV_LOAD_DET, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

	if (system_rev < 2) {
		config = PCOM_GPIO_CFG(INCREDIBLEC_AV_SWITCH, 0, GPIO_OUTPUT,
				GPIO_NO_PULL, GPIO_16MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}

	if ((ret = platform_device_register(&msm_tvenc_device)) != 0)
		return ret;

	return 0;
}

device_initcall(incrediblec_init_tv);
