/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_IRQS_H
#define __ASM_ARCH_MSM_IRQS_H

#define MSM_IRQ_BIT(irq)     (1 << ((irq) & 31))

#define NR_BOARD_IRQS 64
#define NR_MICROP_IRQS 16

#define FIRST_SIRC_IRQ (NR_MSM_IRQS)
#define FIRST_GPIO_IRQ (NR_MSM_IRQS + NR_SIRC_IRQS)
#define FIRST_BOARD_IRQ (NR_MSM_IRQS + NR_SIRC_IRQS + NR_GPIO_IRQS)
#define FIRST_MICROP_IRQ (FIRST_BOARD_IRQ + NR_BOARD_IRQS)

#if defined(CONFIG_ARCH_MSM7X30)
#include "irqs-7x30.h"
#elif defined(CONFIG_ARCH_QSD8X50)
#include "irqs-8x50.h"
#include "sirc.h"
#elif defined(CONFIG_ARCH_MSM8X60)
#include "irqs-8x60.h"
#elif defined(CONFIG_ARCH_MSM_ARM11)
#include "irqs-7x00.h"
#else
#error "Unknown architecture specification"
#endif

#if defined(CONFIG_MACH_BRAVO) || defined(CONFIG_MACH_BRAVOC) || defined(CONFIG_MACH_INCREDIBLEC) || defined(CONFIG_MACH_SUPERSONIC)
#define NR_IRQS (NR_MSM_IRQS + NR_SIRC_IRQS + NR_GPIO_IRQS + NR_BOARD_IRQS \
                + NR_MICROP_IRQS)
#define MSM_INT_TO_GPIO(n) ((n) - NR_MSM_IRQS)
#define MSM_uP_TO_INT(n) (FIRST_MICROP_IRQ + (n))
#define MSM_INT_TO_GPIO(n) ((n) - NR_MSM_IRQS)
#define MSM_uP_TO_INT(n) (FIRST_MICROP_IRQ + (n))
#else
#define NR_IRQS (NR_MSM_IRQS + NR_SIRC_IRQS + NR_GPIO_IRQS + NR_BOARD_IRQS)
#endif

#define MSM_GPIO_TO_INT(n) (FIRST_GPIO_IRQ + (n))
#define MSM_INT_TO_REG(base, irq) (base + irq / 32)
#endif
