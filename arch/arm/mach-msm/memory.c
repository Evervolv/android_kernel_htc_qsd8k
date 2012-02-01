/* arch/arm/mach-msm/memory.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/mm.h>
#include <linux/mm_types.h>
#include <asm/pgtable.h>

int arch_io_remap_pfn_range(struct vm_area_struct *vma, unsigned long addr,
			    unsigned long pfn, unsigned long size, pgprot_t prot)
{
	unsigned long pfn_addr = pfn << PAGE_SHIFT;
	if ((pfn_addr >= 0x88000000) && (pfn_addr < 0xD0000000)) {
		prot = pgprot_device(prot);
		printk("remapping device %lx\n", prot);
	}
	return remap_pfn_range(vma, addr, pfn, size, prot);
}

#define CACHE_LINE_SIZE 32

/* These cache related routines make the assumption that the associated
 * physical memory is contiguous. They will operate on all (L1
 * and L2 if present) caches.
 */
void clean_and_invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	unsigned long vaddr;

	for (vaddr = vstart; vaddr < vstart + length; vaddr += CACHE_LINE_SIZE)
		asm ("mcr p15, 0, %0, c7, c14, 1" : : "r" (vaddr));
#ifdef CONFIG_OUTER_CACHE
	outer_flush_range(pstart, pstart + length);
#endif
	asm ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
	asm ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));
}

void clean_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	unsigned long vaddr;

	for (vaddr = vstart; vaddr < vstart + length; vaddr += CACHE_LINE_SIZE)
		asm ("mcr p15, 0, %0, c7, c10, 1" : : "r" (vaddr));
#ifdef CONFIG_OUTER_CACHE
	outer_clean_range(pstart, pstart + length);
#endif
	asm ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
	asm ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));
}

void invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	unsigned long vaddr;

	for (vaddr = vstart; vaddr < vstart + length; vaddr += CACHE_LINE_SIZE)
		asm ("mcr p15, 0, %0, c7, c6, 1" : : "r" (vaddr));
#ifdef CONFIG_OUTER_CACHE
	outer_inv_range(pstart, pstart + length);
#endif
	asm ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
	asm ("mcr p15, 0, %0, c7, c5, 0" : : "r" (0));
}
