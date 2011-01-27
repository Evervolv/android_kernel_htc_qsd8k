/* linux/arch/arm/mach-msm/board-msm7x30-audio.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
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

#include <linux/gpio.h>
#include <linux/kernel.h>

#include <mach/vreg.h>

#include "gpiomux.h"
#include "board-msm7x30.h"

static void msm7x30_speaker_amp_init(void)
{
	msm_gpiomux_write(82, 0,
			  GPIOMUX_FUNC_GPIO |
			  GPIOMUX_PULL_NONE |
			  GPIOMUX_DIR_OUTPUT |
			  GPIOMUX_DRV_2MA | GPIOMUX_VALID);

	gpio_request(82, "poweramp");
	gpio_direction_output(82, 1);
}

static void msm7x30_marimba_init(void)
{
	struct vreg *vr;

	vr = vreg_get(NULL, "s3");
	vreg_set_level(vr, 1800);
	vreg_enable(vr);

	vr = vreg_get(NULL, "gp16");
	vreg_set_level(vr, 1200);
	vreg_enable(vr);

	vr = vreg_get(NULL, "usb2");
	vreg_set_level(vr, 1800);
	vreg_enable(vr);
}

void msm7x30_board_audio_init(void)
{
	msm7x30_marimba_init();
	msm7x30_speaker_amp_init();
}
