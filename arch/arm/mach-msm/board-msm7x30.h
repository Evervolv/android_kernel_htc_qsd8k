/* linux/arch/arm/mach-msm/board-msm7x30.h
 *
 * Copyright (C) 2011 Google, Inc.
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_MSM7X30_H
#define __ARCH_ARM_MACH_MSM_BOARD_MSM7X30_H

#include <mach/gpio.h>
#include <mach/irqs.h>

#define MSM7X30_PM8058_GPIO_BASE	FIRST_BOARD_GPIO
#define MSM7X30_PM8058_GPIO(x)		(MSM7X30_PM8058_GPIO_BASE + (x))
#define MSM7X30_PM8058_IRQ_BASE		FIRST_BOARD_IRQ

#define MSM7X30_GPIO_PMIC_INT_N		27
#define MSM7X30_FLUID_GPIO_TOUCH_INT_N	150

#define MSM7X30_SURF_PMEM_SIZE		0x02000000
#define MSM7X30_SURF_PMEM_ADSP_SIZE	0x01800000

#define MSM7X30_SURF_GPU_MEM_SIZE	0x00500000

void msm7x30_board_audio_init(void);

#endif
