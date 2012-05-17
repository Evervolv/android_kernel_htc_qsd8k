/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* The MSM Hardware supports multiple flavors of physical memory.
 * This file captures hardware specific information of these types.
*/

#ifndef __ASM_ARCH_MSM_MEMTYPES_H
#define __ASM_ARCH_MSM_MEMTYPES_H

#include <mach/memory.h>
#include <linux/init.h>

int __init meminfo_init(unsigned int, unsigned int);
/* Redundant check to prevent this from being included outside of 7x30 */
#if defined(CONFIG_ARCH_MSM7X30)
unsigned int get_num_populated_chipselects(void);
#endif

unsigned int get_num_memory_banks(void);
unsigned int get_memory_bank_size(unsigned int);
unsigned int get_memory_bank_start(unsigned int);
int soc_change_memory_power(u64, u64, int);

enum {
	MEMTYPE_NONE = -1,
	MEMTYPE_SMI_KERNEL = 0,
	MEMTYPE_SMI,
	MEMTYPE_EBI0,
	MEMTYPE_EBI1,
	MEMTYPE_MAX,
};

void msm_reserve(void);

#define MEMTYPE_FLAGS_FIXED	0x1
#define MEMTYPE_FLAGS_1M_ALIGN	0x2

struct memtype_reserve {
	unsigned long start;
	unsigned long size;
	unsigned long limit;
	int flags;
};

struct reserve_info {
	struct memtype_reserve *memtype_reserve_table;
	void (*calculate_reserve_sizes)(void);
	int (*paddr_to_memtype)(unsigned int);
	unsigned long low_unstable_address;
	unsigned long max_unstable_size;
	unsigned long bank_size;
};

extern struct reserve_info *reserve_info;
#endif
