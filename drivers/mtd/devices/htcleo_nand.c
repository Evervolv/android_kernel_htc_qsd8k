/* drivers/mtd/devices/htcleo-nand.c
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

// WARNING: This driver is designed for HTC LEO (HD2) only.
// Changed numbers to match Leo flash layout
// Yaffs2 reading and writing with MTD_OOB_AUTO works
// -Cotulla

// Added MTD_OOB_PLACE access to spare, removed most magic constants
// YAFFS2 should not pass improper oob len anymore (fix in yaffs_mtdif2.c)
// Added missing capabilities for userland mtd interaction (multi-page, data/oob only program)
// -Rajko

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>
#include <linux/slab.h>

#include <asm/dma.h>
#include <asm/mach/flash.h>

#include <mach/dma.h>
#include <mach/board-htcleo-mac.h>

unsigned crci_mask;

#if defined(CONFIG_ARCH_MSM7X30)
#define MSM_NAND_BASE 0xA0200000
#else
#define MSM_NAND_BASE 0xA0A00000
#endif

#include "msm_nand.h"

#define MSM_NAND_DMA_BUFFER_SIZE SZ_4K
#define MSM_NAND_DMA_BUFFER_SLOTS \
	(MSM_NAND_DMA_BUFFER_SIZE / (sizeof(((atomic_t *)0)->counter) * 8))

#define MSM_NAND_CFG0_RAW 0xA80420C0
#define MSM_NAND_CFG1_RAW 0x5045D

#define SUPPORT_WRONG_ECC_CONFIG 0
#define IGNORE_ARM9_CONFIG       0
#define VERBOSE                  0

//#define ENABLE_FLASH_RW_DUMP
//#define ENABLE_ENTRY_TRACE

static struct nand_hw_info *nand_info;
struct nand_hw_info {
	uint32_t flash_id;
	uint8_t maker_id;
	uint8_t maker_name[10];
	uint8_t width;
	uint32_t size;
	uint32_t block_count;
	uint32_t page_count;
	uint32_t page_size;
};

struct msm_nand_chip {
	struct device *dev;
	wait_queue_head_t wait_queue;
	atomic_t dma_buffer_busy;
	unsigned dma_channel;
	uint8_t *dma_buffer;
	dma_addr_t dma_addr;
	unsigned CFG0, CFG1;
#if SUPPORT_WRONG_ECC_CONFIG
	uint32_t ecc_buf_cfg;
	uint32_t saved_ecc_buf_cfg;
#endif
	struct nand_hw_info dev_info;
};

#define CFG1_WIDE_FLASH (1U << 1)

/* TODO: move datamover code out */

#define SRC_CRCI_NAND_CMD  CMD_SRC_CRCI(DMOV_NAND_CRCI_CMD)
#define DST_CRCI_NAND_CMD  CMD_DST_CRCI(DMOV_NAND_CRCI_CMD)
#define SRC_CRCI_NAND_DATA CMD_SRC_CRCI(DMOV_NAND_CRCI_DATA)
#define DST_CRCI_NAND_DATA CMD_DST_CRCI(DMOV_NAND_CRCI_DATA)

#define msm_virt_to_dma(chip, vaddr) \
	((void)(*(vaddr)), (chip)->dma_addr + \
	 ((uint8_t *)(vaddr) - (chip)->dma_buffer))

/**
* msm_nand_oob_64 - oob info for 2KB page
*/
// Rajko - proper table resembling the layout used now, used in some internal calculations
// outside of this file, only the oobavail and oobfree parts are used
static struct nand_ecclayout msm_nand_oob_64 = 
{
	.eccbytes = 40,
	.eccpos = 
	{
		0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  // 10, 11, 12, 13, // 14, 15, // ecc, oob, bbf (inaccessible)
		16, 17, 18, 19, 20, 21, 22, 23, 24, 25, // 26, 27, 28, 29, // 30, 31, // ecc, oob, bbf (inaccessible)
		32, 33, 34, 35, 36, 37, 38, 39, 40, 41, // 42, 43, 44, 45, // 46, 47, // ecc, oob, bbf (inaccessible)
		48, 49, 50, 51, 52, 53, 54, 55, 56, 57, // 58, 59, 60, 61, // 62, 63, // ecc, oob, bbf (inaccessible)
	},
	.oobavail = 16,
	.oobfree = 
	{
		{10, 4},
		{26, 4},
		{42, 4},
		{58, 4},
	}
};

#define NUM_PROTECTED_BLOCKS (0x212)

struct flash_identification {
	uint32_t flash_id;
	uint32_t mask;
	uint32_t density;
	uint32_t widebus;
	uint32_t pagesize;
	uint32_t blksize;
	uint32_t oobsize;
};

static struct flash_identification supported_flash[] =
{
	/* Flash ID   ID Mask Density(MB)  Wid Pgsz   Blksz   oobsz    Manuf */
	{0x00000000, 0xFFFFFFFF,         0, 0,    0,         0,  0, }, /*ONFI*/
	{0x1500aaec, 0xFF00FFFF, (256<<20), 0, 2048, (2048<<6), 64, }, /*Samsung 2Gbit*/
	{0x5500baec, 0xFF00FFFF, (256<<20), 1, 2048, (2048<<6), 64, }, /*Samsung 2Gbit*/
	{0x6600bcec, 0xFF00FFFF, (512<<20), 1, 4096, (4096<<6), 128,}, /*Samsung 4Gbit 4K Page*/
	{0x0000aaec, 0x0000FFFF, (256<<20), 1, 2048, (2048<<6), 64, }, /*Samsung 2Gbit*/
	{0x0000acec, 0x0000FFFF, (512<<20), 1, 2048, (2048<<6), 64, }, /*Samsung 4Gbit*/
	{0x0000bcec, 0x0000FFFF, (512<<20), 1, 2048, (2048<<6), 64, }, /*Samsung 4Gbit*/
	{0x6601b3ec, 0xFFFFFFFF, (1024<<20),1, 4096, (4096<<6), 128,}, /*Samsung 8Gbit 4Kpage*/
	{0x0000b3ec, 0x0000FFFF, (1024<<20),1, 2048, (2048<<6), 64, }, /*Samsung 8Gbit*/

	{0x1500aa98, 0xFFFFFFFF, (256<<20), 0, 2048, (2048<<6), 64, }, /*Toshiba 2Gbit*/
	{0x5500ba98, 0xFFFFFFFF, (256<<20), 1, 2048, (2048<<6), 64, }, /*Toshiba 2Gbit*/

	{0xd580b12c, 0xFFFFFFFF, (128<<20), 1, 2048, (2048<<6), 64, }, /*Micron 1Gbit*/
	{0x0000ba2c, 0x0000FFFF, (256<<20), 1, 2048, (2048<<6), 64, }, /*Micron 2Gbit*/
	{0x0000bc2c, 0x0000FFFF, (512<<20), 1, 2048, (2048<<6), 64, }, /*Micron 4Gbit*/
	{0x0000b32c, 0x0000FFFF, (1024<<20),1, 2048, (2048<<6), 64, }, /*Micron 8Gbit*/

	{0x0000baad, 0x0000FFFF, (256<<20), 1, 2048, (2048<<6), 64, }, /*Hynix 2Gbit*/
	{0x0000bcad, 0x0000FFFF, (512<<20), 1, 2048, (2048<<6), 64, }, /*Hynix 4Gbit*/
	{0x0000b3ad, 0x0000FFFF, (1024<<20),1, 2048, (2048<<6), 64, }, /*Hynix 8Gbit*/
	/* Note: Width flag is 0 for 8 bit Flash and 1 for 16 bit flash      */
	/* Note: The First row will be filled at runtime during ONFI probe   */
};

static void *msm_nand_get_dma_buffer(struct msm_nand_chip *chip, size_t size)
{
	unsigned int bitmask, free_bitmask, old_bitmask;
	unsigned int need_mask, current_need_mask;
	int free_index;

	need_mask = (1UL << DIV_ROUND_UP(size, MSM_NAND_DMA_BUFFER_SLOTS)) - 1;
	bitmask = atomic_read(&chip->dma_buffer_busy);
	free_bitmask = ~bitmask;
	do {
		free_index = __ffs(free_bitmask);
		current_need_mask = need_mask << free_index;

		if (size + free_index * MSM_NAND_DMA_BUFFER_SLOTS >=
						MSM_NAND_DMA_BUFFER_SIZE)
			return NULL;

		if ((bitmask & current_need_mask) == 0) {
			old_bitmask =
				atomic_cmpxchg(&chip->dma_buffer_busy,
				bitmask,
				bitmask | current_need_mask);
			if (old_bitmask == bitmask)
				return chip->dma_buffer +
				free_index * MSM_NAND_DMA_BUFFER_SLOTS;
			free_bitmask = 0; /* force return */
		}
		/* current free range was too small, clear all free bits */
		/* below the top busy bit within current_need_mask */
		free_bitmask &=
			~(~0U >> (32 - fls(bitmask & current_need_mask)));
	} while (free_bitmask);

	return NULL;
}

static void msm_nand_release_dma_buffer(struct msm_nand_chip *chip,
					void *buffer, size_t size)
{
	int index;
	unsigned int used_mask;

	used_mask = (1UL << DIV_ROUND_UP(size, MSM_NAND_DMA_BUFFER_SLOTS)) - 1;
	index = ((uint8_t *)buffer - chip->dma_buffer) /
		MSM_NAND_DMA_BUFFER_SLOTS;
	atomic_sub(used_mask << index, &chip->dma_buffer_busy);

	wake_up(&chip->wait_queue);
}

uint32_t flash_read_id(struct msm_nand_chip *chip)
{
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		unsigned data[5];
	} *dma_buffer;
	uint32_t rv;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->data[0] = 0 | 4;
	dma_buffer->data[1] = MSM_NAND_CMD_FETCH_ID;
	dma_buffer->data[2] = 1;
	dma_buffer->data[3] = 0xeeeeeeee;
	dma_buffer->data[4] = 0xeeeeeeee;
	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data) - 1);

	dma_buffer->cmd[0].cmd = 0 | CMD_OCB;
	dma_buffer->cmd[0].src = msm_virt_to_dma(chip, &dma_buffer->data[0]);
	dma_buffer->cmd[0].dst = MSM_NAND_FLASH_CHIP_SELECT;
	dma_buffer->cmd[0].len = 4;

	dma_buffer->cmd[1].cmd = DST_CRCI_NAND_CMD;
	dma_buffer->cmd[1].src = msm_virt_to_dma(chip, &dma_buffer->data[1]);
	dma_buffer->cmd[1].dst = MSM_NAND_FLASH_CMD;
	dma_buffer->cmd[1].len = 4;

	dma_buffer->cmd[2].cmd = 0;
	dma_buffer->cmd[2].src = msm_virt_to_dma(chip, &dma_buffer->data[2]);
	dma_buffer->cmd[2].dst = MSM_NAND_EXEC_CMD;
	dma_buffer->cmd[2].len = 4;

	dma_buffer->cmd[3].cmd = SRC_CRCI_NAND_DATA;
	dma_buffer->cmd[3].src = MSM_NAND_FLASH_STATUS;
	dma_buffer->cmd[3].dst = msm_virt_to_dma(chip, &dma_buffer->data[3]);
	dma_buffer->cmd[3].len = 4;

	dma_buffer->cmd[4].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[4].src = MSM_NAND_READ_ID;
	dma_buffer->cmd[4].dst = msm_virt_to_dma(chip, &dma_buffer->data[4]);
	dma_buffer->cmd[4].len = 4;
	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	pr_info("status: %x\n", dma_buffer->data[3]);
	pr_info("nandid: %x maker %02x device %02x\n",
		dma_buffer->data[4], dma_buffer->data[4] & 0xff,
		(dma_buffer->data[4] >> 8) & 0xff);
	rv = dma_buffer->data[4];
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
	return rv;
}

int flash_read_config(struct msm_nand_chip *chip)
{
	struct {
		dmov_s cmd[2];
		unsigned cmdptr;
		unsigned cfg0;
		unsigned cfg1;
	} *dma_buffer;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));
	dma_buffer->cfg0 = 0;
	dma_buffer->cfg1 = 0;

	dma_buffer->cmd[0].cmd = CMD_OCB;
	dma_buffer->cmd[0].src = MSM_NAND_DEV0_CFG0;
	dma_buffer->cmd[0].dst = msm_virt_to_dma(chip, &dma_buffer->cfg0);
	dma_buffer->cmd[0].len = 4;

	dma_buffer->cmd[1].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[1].src = MSM_NAND_DEV0_CFG1;
	dma_buffer->cmd[1].dst = msm_virt_to_dma(chip, &dma_buffer->cfg1);
	dma_buffer->cmd[1].len = 4;
	BUILD_BUG_ON(1 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	chip->CFG0 = dma_buffer->cfg0;
	chip->CFG1 = dma_buffer->cfg1;
	pr_info("msm_nand: read CFG0 = %x, CFG1 = %x\n", chip->CFG0, chip->CFG1);

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if ((chip->CFG0 == 0) || (chip->CFG1 == 0))
		return -1;

	return 0;
}

unsigned flash_rd_reg(struct msm_nand_chip *chip, unsigned addr)
{
	struct {
		dmov_s cmd;
		unsigned cmdptr;
		unsigned data;
	} *dma_buffer;
	unsigned rv;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->cmd.cmd = CMD_LC;
	dma_buffer->cmd.src = addr;
	dma_buffer->cmd.dst = msm_virt_to_dma(chip, &dma_buffer->data);
	dma_buffer->cmd.len = 4;

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, &dma_buffer->cmd) >> 3) | CMD_PTR_LP;
	dma_buffer->data = 0xeeeeeeee;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();
	rv = dma_buffer->data;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	return rv;
}

void flash_wr_reg(struct msm_nand_chip *chip, unsigned addr, unsigned val)
{
	struct {
		dmov_s cmd;
		unsigned cmdptr;
		unsigned data;
	} *dma_buffer;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->cmd.cmd = CMD_LC;
	dma_buffer->cmd.src = msm_virt_to_dma(chip, &dma_buffer->data);
	dma_buffer->cmd.dst = addr;
	dma_buffer->cmd.len = 4;

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, &dma_buffer->cmd) >> 3) | CMD_PTR_LP;
	dma_buffer->data = val;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
}

static dma_addr_t
msm_nand_dma_map(struct device *dev, void *addr, size_t size,
		 enum dma_data_direction dir)
{
	struct page *page;
	unsigned long offset = (unsigned long)addr & ~PAGE_MASK;
	if (virt_addr_valid(addr))
		page = virt_to_page(addr);
	else {
		if (WARN_ON(size + offset > PAGE_SIZE))
			return ~0;
		page = vmalloc_to_page(addr);
	}
	return dma_map_page(dev, page, offset, size, dir);
}

static int msm_nand_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	struct msm_nand_chip *chip = mtd->priv;

	struct 
	{
		dmov_s cmd[8 * 5 + 3];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
#if SUPPORT_WRONG_ECC_CONFIG
			uint32_t ecccfg;
			uint32_t ecccfg_restore;
#endif
			struct {
				uint32_t flash_status;
				uint32_t buffer_status;
			} result[8];
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	unsigned n;
	unsigned page = 0;
	uint32_t cwdatasize;
	uint32_t cwoobsize;
	int err, pageerr, rawerr;
	dma_addr_t data_dma_addr = 0;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t data_dma_addr_curr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	uint32_t oob_col = 0;
	unsigned page_count;
	unsigned pages_read = 0;
	unsigned start_sector = 0;
	uint32_t ecc_errors;
	uint32_t total_ecc_errors = 0;
	unsigned cwperpage;
	unsigned ooboffs;
	unsigned page_oob_count=0;
	unsigned page_oob_done;
	uint32_t oob_count=0;
	uint32_t oob_done_size=0;
	uint32_t data_count=0; //number of bytes mapped into dma
	int readdata=0;
	int readoob=0;
	uint32_t readcmd;

#ifdef ENABLE_ENTRY_TRACE
	char* oobType = "MTD_OOB_RAW";
	printk("+&&&nand_readoob&&&\n");
#endif

	if (mtd->writesize == 2048)
		page = from >> 11;

	if (mtd->writesize == 4096)
		page = from >> 12;

	cwperpage = (mtd->writesize >> 9);
	cwdatasize = mtd->writesize/cwperpage;

	if (ops->mode == MTD_OOB_AUTO)
	{
		cwoobsize = mtd->oobavail/cwperpage;
		ooboffs=cwdatasize + (mtd->ecclayout->eccbytes/cwperpage); //after data and ecc
	}
	else
	{
		cwoobsize = (mtd->oobsize/cwperpage) - 2; //dont read 2 inaccessible bbf bytes
		ooboffs=cwdatasize; //after data
	}

	if (from & (mtd->writesize - 1)) 
	{
		pr_err("%s: unsupported from, 0x%llx\n", __func__, from);
		return -EINVAL;
	}
	if (ops->datbuf != NULL && (ops->len % mtd->writesize) != 0) 
	{
		/* when ops->datbuf is NULL, ops->len may refer to ooblen */
		pr_err("%s: unsupported ops->len, %d\n", __func__, ops->len);
		return -EINVAL;
	}
	if (ops->datbuf && ops->len > 0)
		readdata=1;

	if (ops->oobbuf)
	{
		if (ops->ooblen > 0)
		{
			oob_count = ops->ooblen;
			readoob=1;
		}
		else if (!readdata && ops->len > 0)
		{
			oob_count = ops->len;
			readoob=1;
		}
	}

	if (!readdata && !readoob)
	{
		pr_err("%s: nothing to do\n", __func__);
		return -EINVAL;
	}
	if (readoob && ops->ooboffs != 0) 
	{
		pr_err("%s: unsupported ops->ooboffs, %d\n", __func__, ops->ooboffs);
		return -EINVAL;
	}

#ifdef ENABLE_ENTRY_TRACE
	if (ops->mode == MTD_OOB_AUTO)
		oobType = "MTD_OOB_AUTO";
	else if (ops->mode == MTD_OOB_PLACE)
		oobType = "MTD_OOB_PLACE";

	printk("msm_nand_read_oob [%08X %08X] %08X %08X %d %s\n", 
		(uint32_t)(from), (uint32_t)ops->len, (uint32_t)ops->datbuf, 
		(uint32_t)ops->oobbuf, oob_count, readoob?oobType:"OOB_NONE");
#endif

	if (readoob)
		readcmd=MSM_NAND_CMD_PAGE_READ_ALL;
	else
		readcmd=MSM_NAND_CMD_PAGE_READ;

	if (readoob && !readdata)
	{
		uint32_t maxpageoob;
		if (ops->mode == MTD_OOB_AUTO)
			maxpageoob=mtd->oobavail;
		else
			maxpageoob=mtd->oobsize;

		page_count = oob_count/maxpageoob;
		if (oob_count%maxpageoob != 0)
		{
			//assume caller wants to read only one page's oob
			page_oob_count=oob_count;
			page_count=1;	
		}
		else
		{
			page_oob_count = maxpageoob;
		}
	} 
	else
	{
		page_count = ops->len / mtd->writesize;
		if (oob_count%page_count != 0)
		{
			pr_err("%s: OOB num pages != DATA num pages oob_count=%d\n", __func__, oob_count);
			return -EINVAL;			
		}
		page_oob_count = oob_count/page_count;
	}

#if VERBOSE
	printk("msm_nand_read_oob ops->len=%d oob_count=%d page_count=%d page_oob_count=%d\n", ops->len, oob_count, page_count, page_oob_count);
#endif

	if (readdata) 
	{
		/* memset(ops->datbuf, 0x55, ops->len); */
		data_dma_addr_curr = data_dma_addr = msm_nand_dma_map(chip->dev, ops->datbuf, ops->len, DMA_FROM_DEVICE);
		data_count = ops->len; //initially map whole buf to dma
		if (dma_mapping_error(chip->dev, data_dma_addr)) 
		{
			pr_err("msm_nand_read_oob: failed to get dma addr for %p\n", ops->datbuf);
			return -EIO;
		}
	}
	if (readoob) 
	{
		memset(ops->oobbuf, 0xff, oob_count);
		oob_dma_addr_curr = oob_dma_addr = msm_nand_dma_map(chip->dev, ops->oobbuf, oob_count, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(chip->dev, oob_dma_addr)) 
		{
			pr_err("msm_nand_read_oob: failed to get dma addr for %p\n", ops->oobbuf); 
			err = -EIO;
			goto err_dma_map_oobbuf_failed;
		}
	}

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	if (readoob)
	{
		oob_col = start_sector * (512+16);
		if (chip->CFG1 & CFG1_WIDE_FLASH)
			oob_col >>= 1;
	}

#if VERBOSE
	printk("msm_nand_read_oob start_sector=%d oob_col=%d ooboffs=%d\n", start_sector, oob_col, ooboffs);
#endif

	err = 0;
	while (page_count-- > 0) 
	{
		page_oob_done=0;	

		cmd = dma_buffer->cmd;

		/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		dma_buffer->data.cmd = readcmd;
		dma_buffer->data.addr0 = (page << 16) | oob_col;
		/* qc example is (page >> 16) && 0xff !? */
		dma_buffer->data.addr1 = (page >> 16) & 0xff;
		/* flash0 + undoc bit */
		dma_buffer->data.chipsel = 0 | 4;

		dma_buffer->data.cfg0 = (chip->CFG0 & ~(7U << 6)) | (((cwperpage-1) - start_sector) << 6);
		dma_buffer->data.cfg1 = chip->CFG1;

		/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;

		BUILD_BUG_ON(8 != ARRAY_SIZE(dma_buffer->data.result));

		for (n = start_sector; n < cwperpage; n++) 
		{
			/* flash + buffer status return words */
			dma_buffer->data.result[n].flash_status = 0xeeeeeeee;
			dma_buffer->data.result[n].buffer_status = 0xeeeeeeee;

			/* block on cmd ready, then
			* write CMD / ADDR0 / ADDR1 / CHIPSEL
			* regs in a burst
			*/
			cmd->cmd = DST_CRCI_NAND_CMD;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
			cmd->dst = MSM_NAND_FLASH_CMD;
			if (n == start_sector)
				cmd->len = 16;
			else
				cmd->len = 4;
			cmd++;

			if (n == start_sector) 
			{
				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0);
				cmd->dst = MSM_NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
#if SUPPORT_WRONG_ECC_CONFIG
				if (chip->saved_ecc_buf_cfg != chip->ecc_buf_cfg) 
				{
					dma_buffer->data.ecccfg = chip->ecc_buf_cfg;
					cmd->cmd = 0;
					cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.ecccfg);
					cmd->dst = MSM_NAND_EBI2_ECC_BUF_CFG;
					cmd->len = 4;
					cmd++;
				}
#endif
			}

			/* kick the execute register */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = MSM_NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

			/* block on data ready, then
			* read the status register
			*/
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = MSM_NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.result[n]);
			/* MSM_NAND_FLASH_STATUS + MSM_NAND_BUFFER_STATUS */
			cmd->len = 8;
			cmd++;

			/* read data block
			* (only valid if status says success)
			*/
			if (readdata) 
			{
				cmd->cmd = 0;
				cmd->src = MSM_NAND_FLASH_BUFFER;
				cmd->dst = data_dma_addr_curr;
				data_dma_addr_curr += cwdatasize;
				cmd->len = cwdatasize;
				cmd++;
			}
			if (readoob && page_oob_done < page_oob_count) 
			{
				cmd->cmd = 0;
				cmd->src = MSM_NAND_FLASH_BUFFER + ooboffs;	
				cmd->dst = oob_dma_addr_curr;

				if (cwoobsize < (page_oob_count-page_oob_done) )
					cmd->len = cwoobsize;
				else
					cmd->len = page_oob_count-page_oob_done;

#if VERBOSE
				printk("msm_nand_read_oob oob_dma_addr_curr=%08x cwoobsize=%d page_oob_done=%d page_oob_count=%d\n", 
					(uint32_t)oob_dma_addr_curr,cwoobsize,page_oob_done,page_oob_count);
#endif
				oob_dma_addr_curr += cmd->len;
				page_oob_done += cmd->len;

				if (ops->mode != MTD_OOB_AUTO) //seek over 2 skipped bytes in output
				{
					int maxinc=page_oob_count-page_oob_done;
					if (maxinc>2)
						maxinc=2;

					oob_dma_addr_curr += maxinc;
					page_oob_done += maxinc;
				}

				if (cmd->len > 0)
					cmd++;				
			}
		}
#if SUPPORT_WRONG_ECC_CONFIG
		if (chip->saved_ecc_buf_cfg != chip->ecc_buf_cfg) 
		{
			dma_buffer->data.ecccfg_restore = chip->saved_ecc_buf_cfg;
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.ecccfg_restore);
			cmd->dst = MSM_NAND_EBI2_ECC_BUF_CFG;
			cmd->len = 4;
			cmd++;
		}
#endif

		BUILD_BUG_ON(8 * 5 + 3 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;

		dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

		dsb();
		msm_dmov_exec_cmd(
			chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(
			msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
		dsb();

		/* if any of the writes failed (0x10), or there
		* was a protection violation (0x100), we lose
		*/
		pageerr = rawerr = 0;

		for (n = start_sector; n < cwperpage; n++) 
		{
			if (dma_buffer->data.result[n].flash_status & 0x110) 
			{
#if VERBOSE
				printk("msm_nand_read_oob we lose at n=%d\n", n);
#endif
				rawerr = -EIO;
				break;
			}
		}

		if (rawerr) 
		{
			if (readdata) 
			{
				uint8_t *datbuf = ops->datbuf + pages_read * mtd->writesize;
				uint32_t data_done = data_dma_addr_curr-data_dma_addr;
#if VERBOSE
				printk("msm_nand_read_oob dma_unmap_page %08x\n", (uint32_t)data_dma_addr_curr);
#endif
				dma_unmap_page(chip->dev, data_dma_addr, data_count, DMA_FROM_DEVICE);
				data_count -= data_done; // we wont be mapping the already transferred bytes to avoid corruption
				//data now ready to be read by cpu
				
				for (n = 0; n < mtd->writesize; n++) 
				{
					if ((n % 512) == 0xF3 && datbuf[n] == 0x76)
					{
						datbuf[n] = 0xff;
					}
					if (datbuf[n] != 0xff) 
					{
						pageerr = rawerr;
#if VERBOSE
						printk("msm_nand_read_oob data pageerr=rawerr at n=%d\n", n);
#endif
						break;
					}
				}

				//remap dma if there's still data to be read
				if (data_count > 0)
				{
					int offset = ops->len-data_count;
#if VERBOSE
					printk("msm_nand_read_oob dma_map_page offset=%d length=%d\n", offset, data_count);
#endif
					data_dma_addr = msm_nand_dma_map(chip->dev, ops->datbuf+offset, data_count, DMA_FROM_DEVICE);
					data_dma_addr_curr = data_dma_addr;

					if (dma_mapping_error(chip->dev, data_dma_addr)) {
						pr_err("msm_nand_read_oob: failed to re-get dma addr "
			       				"for %p+%d initial_size=%d remaining_size=%d\n", ops->datbuf, offset, ops->len, data_count);
						return -EIO;
					}
				}
				else
				{
					data_dma_addr_curr = data_dma_addr = 0;
				}
			}
			if (readoob && oob_done_size < oob_count) 
			{
				//no sync needed for BIDIR dma buffers
				uint8_t *pageoobbuf = ops->oobbuf + oob_done_size;

				for (n = 0; n < page_oob_done; n++)
				{
					if (pageoobbuf[n] != 0xff) 
					{
						pageerr = rawerr;
#if VERBOSE
						printk("msm_nand_read_oob oob pageerr=rawerr at n=%d\n", n);
#endif
						break;
					}
				}
			}
		}
		if (pageerr) 
		{
			for (n = start_sector; n < cwperpage; n++) 
			{
				if (dma_buffer->data.result[n].buffer_status & 0x8) 
				{
#if VERBOSE
					printk("msm_nand_read_oob oob pageerr ecc uncorrectable at n=%d\n", n);
#endif
					/* not thread safe */
					mtd->ecc_stats.failed++;
					pageerr = -EBADMSG;
					break;
				}
			}
		}
		if (!rawerr) 
		{ 
			/* check for correctable errors */
			for (n = start_sector; n < cwperpage; n++) 
			{
				ecc_errors = dma_buffer->data.result[n].buffer_status & 0x7;
				if (ecc_errors) 
				{
#if VERBOSE
					printk("msm_nand_read_oob oob pageerr ecc correctable n=%d ecc_errors=%d\n", n, ecc_errors);
#endif
					total_ecc_errors += ecc_errors;
					/* not thread safe */
					mtd->ecc_stats.corrected += ecc_errors;
					if (ecc_errors > 1)
						pageerr = -EUCLEAN;
				}
			}
		}
		if (pageerr && (pageerr != -EUCLEAN || err == 0))
			err = pageerr;

#if VERBOSE
		if (rawerr && !pageerr) 
		{
			pr_err("msm_nand_read_oob %llx %x %x empty page\n", (loff_t)page * mtd->writesize, ops->len, oob_count);
		} 
		else 
		{
			pr_info("msm_nand_read_oob error status: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n",
				dma_buffer->data.result[0].flash_status,
				dma_buffer->data.result[0].buffer_status,
				dma_buffer->data.result[1].flash_status,
				dma_buffer->data.result[1].buffer_status,
				dma_buffer->data.result[2].flash_status,
				dma_buffer->data.result[2].buffer_status,
				dma_buffer->data.result[3].flash_status,
				dma_buffer->data.result[3].buffer_status,
				dma_buffer->data.result[4].buffer_status,
				dma_buffer->data.result[4].buffer_status,
				dma_buffer->data.result[5].buffer_status,
				dma_buffer->data.result[5].buffer_status,
				dma_buffer->data.result[6].buffer_status,
				dma_buffer->data.result[6].buffer_status,
				dma_buffer->data.result[7].buffer_status,
				dma_buffer->data.result[7].buffer_status);
		}
#endif
		if (err && err != -EUCLEAN && err != -EBADMSG)
			break;

		if (page_oob_done < page_oob_count) //we didnt have enough oob bytes to fill their per page oob size (yaffs hack)
		{
#if VERBOSE
			printk("msm_nand_read_oob: couldn't provide %d out of %d oob bytes, oob_done_size=%d\n", 
				page_oob_count-page_oob_done, page_oob_count, oob_done_size);
#endif
			//just seek the outbuf for the bytes we couldnt fill
			oob_dma_addr_curr+= page_oob_count-page_oob_done;
			page_oob_done=page_oob_count; //all done now		
		}

		oob_done_size += page_oob_done;

		pages_read++;
		page++;
	}
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (readoob)
		dma_unmap_page(chip->dev, oob_dma_addr, oob_count, DMA_BIDIRECTIONAL);
err_dma_map_oobbuf_failed:
	if (readdata && data_dma_addr != 0)
		dma_unmap_page(chip->dev, data_dma_addr, ops->len, DMA_FROM_DEVICE);

#ifdef ENABLE_FLASH_RW_DUMP
	// dump spare
	if (readoob && oob_done_size)
	{
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, ops->oobbuf, oob_done_size);				
		printk("\n");
	}
	// dump data
	if (readdata && ops->len)
	{
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, ops->datbuf, ops->len);
	}
#endif
	if (readdata)
		ops->retlen = mtd->writesize * pages_read;
	else
		ops->retlen = 0;

	if (readoob)
		ops->oobretlen = oob_done_size;
	else
		ops->oobretlen = 0;

	if (err)
		pr_err("msm_nand_read_oob %llx %x %x failed %d, corrected %d\n",
			from, ops->datbuf ? ops->len : 0, oob_count, err, total_ecc_errors);
#ifdef ENABLE_ENTRY_TRACE
	else
		printk("-&&&nand_readoob&&&\n");
#endif
	return err;
}


static int
msm_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
			  size_t *retlen, u_char *buf)
{
	int ret;
	struct mtd_oob_ops ops;

#ifdef ENABLE_ENTRY_TRACE
	printk("msm_nand_read %llx %d\n", from, len);
#endif

	ops.mode = MTD_OOB_PLACE;
	ops.len = len;
	ops.retlen = 0;
	ops.ooblen = 0;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ret =  msm_nand_read_oob(mtd, from, &ops);
	*retlen = ops.retlen;
	return ret;
}

static int
msm_nand_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	struct msm_nand_chip *chip = mtd->priv;
	struct {
		dmov_s cmd[8 * 6 + 3];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
#if SUPPORT_WRONG_ECC_CONFIG
			uint32_t ecccfg;
			uint32_t ecccfg_restore;
#endif
			uint32_t flash_status[8];
			uint32_t zeroes;
		} data;
	} *dma_buffer;

	dmov_s *cmd;
	unsigned n;
	unsigned page = 0;
	uint32_t cwdatasize;
	uint32_t cwoobsize;
	int err=0;
	dma_addr_t data_dma_addr = 0;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t data_dma_addr_curr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	unsigned page_count;
	unsigned pages_written = 0;
	unsigned cwperpage;
	unsigned ooboffs=0;
	unsigned page_oob_count=0;
	unsigned page_oob_done;
	uint32_t oob_count=0;
	uint32_t oob_done_size=0;
	int writedata=0;
	int writeoob=0;
	uint32_t writecmd;
	static uint8_t dummyud[512];
	static uint8_t dummyecc[10];
	static int dummiesdone=0;
	dma_addr_t dummyecc_dma_addr = 0;

#ifdef ENABLE_ENTRY_TRACE
	char* oobType = "MTD_OOB_RAW";
	printk("+&&&nand_writeoob&&&\n");
#endif

	if (mtd->writesize == 2048)
		page = to >> 11;

	if (mtd->writesize == 4096)
		page = to >> 12;

	cwperpage = (mtd->writesize >> 9);
	cwdatasize = mtd->writesize/cwperpage;
	cwoobsize = mtd->oobavail/cwperpage; //can only write the user bytes per cw

	if (to & (mtd->writesize - 1)) 
	{
		pr_err("%s: unsupported to, 0x%llx\n", __func__, to);
		return -EINVAL;
	}
	if (to < (NUM_PROTECTED_BLOCKS*mtd->erasesize))
	{
		pr_err("%s: cannot write to page in protected area, to=0x%llx\n",
			__func__, to);
		return -EINVAL;
	}
	if (ops->datbuf != NULL && (ops->len % mtd->writesize) != 0) 
	{
		/* when ops->datbuf is NULL, ops->len may refer to ooblen */
		pr_err("%s: unsupported ops->len, %d\n", __func__, ops->len);
		return -EINVAL;
	}

	if (ops->datbuf && ops->len > 0)
		writedata=1;

	if (ops->oobbuf)
	{
		if (ops->ooblen > 0)
		{
			oob_count = ops->ooblen;
			writeoob=1;
		}
		else if (!writedata && ops->len > 0)
		{
			oob_count = ops->len;
			writeoob=1;
		}
	}

	if (!writedata && !writeoob)
	{
		pr_err("%s: nothing to do\n", __func__);
		return -EINVAL;
	}
	if (writeoob && ops->ooboffs != 0)
	{
		pr_err("%s: unsupported ops->ooboffs, %d\n", __func__, ops->ooboffs);
		return -EINVAL;
	}

#ifdef ENABLE_ENTRY_TRACE
	if (ops->mode == MTD_OOB_PLACE)
		oobType = "MTD_OOB_PLACE";
	else if (ops->mode == MTD_OOB_AUTO)
		oobType = "MTD_OOB_AUTO";

	printk("msm_nand_write_oob [%08X %08X] %08X %08X %d %s\n", 
		(uint32_t)(to), writedata?(uint32_t)ops->len:0, (uint32_t)ops->datbuf, 
		(uint32_t)ops->oobbuf, oob_count, writeoob?oobType:"OOB_NONE");
#endif

	if (writeoob)
		writecmd=MSM_NAND_CMD_PRG_PAGE_ALL;
	else
		writecmd=MSM_NAND_CMD_PRG_PAGE;

	if (!writedata) // skip over ecc bytes in flash buf if ecc disabled
		ooboffs = mtd->ecclayout->eccbytes/cwperpage;

	if (writeoob && !writedata)
	{
		uint32_t maxpageoob;
		if (ops->mode == MTD_OOB_AUTO)
			maxpageoob=mtd->oobavail;
		else
			maxpageoob=mtd->oobsize;

		page_count = oob_count/maxpageoob;
		if (oob_count%maxpageoob != 0)
		{
			//assume caller wants to write only one page's oob
			page_oob_count=oob_count;
			page_count=1;	
		}
		else
		{
			page_oob_count = maxpageoob;
		}

		if (!dummiesdone)
		{
			memset(dummyud,0xFF,sizeof(dummyud));
			memset(dummyecc,0xFF,sizeof(dummyecc));
			dummiesdone=1;
		}
	} 
	else
	{
		page_count = ops->len / mtd->writesize;
		if (oob_count%page_count != 0)
		{
			pr_err("%s: OOB num pages != DATA num pages oob_count=%d\n", __func__, oob_count);
			return -EINVAL;			
		}
		page_oob_count = oob_count/page_count;
	}

#if VERBOSE
	printk("msm_nand_write_oob ops->len=%d oob_count=%d page_count=%d page_oob_count=%d\n", ops->len, oob_count, page_count, page_oob_count);
#endif

	if (writedata) 
	{
		data_dma_addr_curr = data_dma_addr = msm_nand_dma_map(chip->dev, ops->datbuf, ops->len, DMA_TO_DEVICE);
		if (dma_mapping_error(chip->dev, data_dma_addr)) 
		{
			pr_err("msm_nand_write_oob: failed to get dma addr for %p\n", ops->datbuf);
			return -EIO;
		}
	}
	if (writeoob) 
	{
		oob_dma_addr_curr = oob_dma_addr = msm_nand_dma_map(chip->dev, ops->oobbuf, oob_count, DMA_TO_DEVICE);
		if (dma_mapping_error(chip->dev, oob_dma_addr)) 
		{
			pr_err("msm_nand_write_oob: failed to get dma addr for %p\n", ops->oobbuf);
			err = -EIO;
			goto err_dma_map_oobbuf_failed;
		}

		if (!writedata)
		{
			data_dma_addr_curr = data_dma_addr = msm_nand_dma_map(chip->dev, dummyud, sizeof(dummyud), DMA_TO_DEVICE);
			if (dma_mapping_error(chip->dev, data_dma_addr)) 
			{
				pr_err("msm_nand_write_oob: failed to get dma addr for %p\n", dummyud);
				err = -EIO;
				goto err_dummyud_dma_failed;
			}
			dummyecc_dma_addr = msm_nand_dma_map(chip->dev, dummyecc, sizeof(dummyecc), DMA_TO_DEVICE);
			if (dma_mapping_error(chip->dev, dummyecc_dma_addr)) 
			{
				pr_err("msm_nand_write_oob: failed to get dma addr for %p\n", dummyecc);
				err = -EIO;
				goto err_dummyecc_dma_failed;
			}
		}
	}

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	while (page_count-- > 0) 
	{
		page_oob_done=0;

		cmd = dma_buffer->cmd;

		/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		dma_buffer->data.cmd = writecmd;
		dma_buffer->data.addr0 = page << 16;
		dma_buffer->data.addr1 = (page >> 16) & 0xff;
		dma_buffer->data.chipsel = 0 | 4; /* flash0 + undoc bit */
		dma_buffer->data.zeroes = 0;

		dma_buffer->data.cfg0 = chip->CFG0;
		dma_buffer->data.cfg1 = chip->CFG1;

		if (!writedata)
			dma_buffer->data.cfg1 |= (1 <<  0); //disable ecc calc and write

		/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;

		BUILD_BUG_ON(8 != ARRAY_SIZE(dma_buffer->data.flash_status));

		for (n = 0; n < cwperpage ; n++) 
		{
			/* status return words */
			dma_buffer->data.flash_status[n] = 0xeeeeeeee;
			/* block on cmd ready, then
			* write CMD / ADDR0 / ADDR1 / CHIPSEL regs in a burst
			*/
			cmd->cmd = DST_CRCI_NAND_CMD;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
			cmd->dst = MSM_NAND_FLASH_CMD;
			if (n == 0)
				cmd->len = 16;
			else
				cmd->len = 4;
			cmd++;

			if (n == 0) 
			{
				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0);
				cmd->dst = MSM_NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
#if SUPPORT_WRONG_ECC_CONFIG
				if (chip->saved_ecc_buf_cfg != chip->ecc_buf_cfg) 
				{
					dma_buffer->data.ecccfg = chip->ecc_buf_cfg;
					cmd->cmd = 0;
					cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.ecccfg);
					cmd->dst = MSM_NAND_EBI2_ECC_BUF_CFG;
					cmd->len = 4;
					cmd++;
				}
#endif
			}

			/* write data block */
			if (writedata || (!pages_written && !n))
			{
				cmd->cmd = 0;
				cmd->src = data_dma_addr_curr;
				if (writedata)
					data_dma_addr_curr += cwdatasize;
				cmd->dst = MSM_NAND_FLASH_BUFFER;
				cmd->len = cwdatasize;
				cmd++;
			}

			if (writeoob && page_oob_done < page_oob_count) 
			{
				if (ops->mode != MTD_OOB_AUTO) // skip over ecc bytes in input buf
				{
					int cweccsize=mtd->ecclayout->eccbytes/cwperpage;
					int maxinc=page_oob_count-page_oob_done;
					if (maxinc>cweccsize)
						maxinc=cweccsize;

					oob_dma_addr_curr += maxinc;
					page_oob_done += maxinc;
				}

				if (!writedata) //if writing spare only, have to fill ecc with 0xFF
				{
					cmd->cmd = 0;
					cmd->dst = MSM_NAND_FLASH_BUFFER + cwdatasize;
					cmd->src = dummyecc_dma_addr;
					cmd->len = sizeof(dummyecc);
					cmd++;
				}

				cmd->cmd = 0;
				cmd->dst = MSM_NAND_FLASH_BUFFER + cwdatasize + ooboffs;
				cmd->src = oob_dma_addr_curr;

				if (cwoobsize < (page_oob_count-page_oob_done) )
					cmd->len = cwoobsize;
				else
					cmd->len = page_oob_count-page_oob_done;

#if VERBOSE
				printk("msm_nand_write_oob oob_dma_addr_curr=%08x cwoobsize=%d page_oob_done=%d page_oob_count=%d\n", 
					(uint32_t)oob_dma_addr_curr,cwoobsize,page_oob_done,page_oob_count);
#endif
				oob_dma_addr_curr += cmd->len;
				page_oob_done += cmd->len;

				if (ops->mode != MTD_OOB_AUTO) // skip over bbf bytes in input buf
				{
					int maxinc=page_oob_count-page_oob_done;
					if (maxinc>2)
						maxinc=2;

					oob_dma_addr_curr += maxinc;
					page_oob_done += maxinc;
				}

				if (cmd->len > 0)
					cmd++;				
			}

			/* kick the execute register */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = MSM_NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

			/* block on data ready, then
			* read the status register
			*/
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = MSM_NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.flash_status[n]);
			cmd->len = 4;
			cmd++;

			/* clear the status register in case the OP_ERR is set
			* due to the write, to work around a h/w bug */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.zeroes);
			cmd->dst = MSM_NAND_FLASH_STATUS;
			cmd->len = 4;
			cmd++;
		}
#if SUPPORT_WRONG_ECC_CONFIG
		if (chip->saved_ecc_buf_cfg != chip->ecc_buf_cfg) 
		{
			dma_buffer->data.ecccfg_restore = chip->saved_ecc_buf_cfg;
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.ecccfg_restore);
			cmd->dst = MSM_NAND_EBI2_ECC_BUF_CFG;
			cmd->len = 4;
			cmd++;
		}
#endif
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;
		BUILD_BUG_ON(8 * 6 + 3 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

		dsb();
		msm_dmov_exec_cmd(chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
		dsb();

		/* if any of the writes failed (0x10), or there was a
		* protection violation (0x100), or the program success
		* bit (0x80) is unset, we lose
		*/
		err = 0;
		for (n = 0; n < cwperpage; n++)
		{
			if (dma_buffer->data.flash_status[n] & 0x110) 
			{
				if (dma_buffer->data.flash_status[n] & 0x10)
					pr_err("msm_nand: critical write error, 0x%x(%d)\n", page, n);

				err = -EIO;
				break;
			}
			if (!(dma_buffer->data.flash_status[n] & 0x80))
			{
				err = -EIO;
				break;
			}
		}

#if VERBOSE
		pr_info("write page %d: status: %x %x %x %x %x %x %x %x\n", page,
			dma_buffer->data.flash_status[0],
			dma_buffer->data.flash_status[1],
			dma_buffer->data.flash_status[2],
			dma_buffer->data.flash_status[3],
			dma_buffer->data.flash_status[4],
			dma_buffer->data.flash_status[5],
			dma_buffer->data.flash_status[6],
			dma_buffer->data.flash_status[7]);
#endif
		if (err)
			break;

		if (page_oob_done < page_oob_count) //they wanted to write more than we could store (yaffs hack)
		{
#if VERBOSE
			printk("msm_nand_write_oob: no space to write %d out of %d oob bytes, oob_done_size=%d\n", 
				page_oob_count-page_oob_done, page_oob_count, oob_done_size);
#endif
			//just seek the outbuf for the bytes we couldnt write
			oob_dma_addr_curr+= page_oob_count-page_oob_done;
			page_oob_done=page_oob_count; //all done now		
		}

		oob_done_size += page_oob_done;

		pages_written++;
		page++;
	}

	if (writedata)
		ops->retlen = mtd->writesize * pages_written;
	else
		ops->retlen = 0;

	if (writeoob)
		ops->oobretlen = oob_done_size;
	else
		ops->oobretlen = 0;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (!writedata && writeoob)
		dma_unmap_page(chip->dev, dummyecc_dma_addr, sizeof(dummyecc), DMA_TO_DEVICE);
err_dummyecc_dma_failed:
	if (!writedata && writeoob)
		dma_unmap_page(chip->dev, data_dma_addr, sizeof(dummyud), DMA_TO_DEVICE);
err_dummyud_dma_failed:
	if (writeoob)
		dma_unmap_page(chip->dev, oob_dma_addr, oob_count, DMA_TO_DEVICE);
err_dma_map_oobbuf_failed:
	if (writedata)
		dma_unmap_page(chip->dev, data_dma_addr, ops->len, DMA_TO_DEVICE);

#ifdef ENABLE_FLASH_RW_DUMP
	// dump spare
	if (ops->oobbuf && oob_done_size)
	{
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, ops->oobbuf, oob_done_size);				
		printk("\n");
	}

	// dump data
	if (ops->datbuf && ops->len)
	{
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, ops->datbuf, ops->len);
	}
#endif

	if (err)
		pr_err("msm_nand_write_oob %llx %x %x failed %d\n", to, ops->len, oob_count, err);
#ifdef ENABLE_ENTRY_TRACE
	else
		printk("-&&&nand_writeoob&&&\n");
#endif
	return err;
}

static int msm_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
						  size_t *retlen, const u_char *buf)
{
	int ret;
	struct mtd_oob_ops ops;

#ifdef ENABLE_ENTRY_TRACE
	printk("nand_write\n");
#endif
	ops.mode = MTD_OOB_PLACE;
	ops.len = len;
	ops.retlen = 0;
	ops.ooblen = 0;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ret =  msm_nand_write_oob(mtd, to, &ops);
	*retlen = ops.retlen;
	return ret;
}

static int
msm_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int err;
	struct msm_nand_chip *chip = mtd->priv;
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		unsigned data[9];
	} *dma_buffer;
	unsigned page = 0;

	if (mtd->writesize == 2048)
		page = instr->addr >> 11;

	if (mtd->writesize == 4096)
		page = instr->addr >> 12;

#ifdef ENABLE_ENTRY_TRACE
	printk("msm_nand_erase(%d)\n", page);
#endif

	if (instr->addr & (mtd->erasesize - 1)) {
		pr_err("%s: unsupported erase address, 0x%llx\n",
			__func__, instr->addr);
		return -EINVAL;
	}

	if (instr->addr < (NUM_PROTECTED_BLOCKS*mtd->erasesize))
	{
		pr_err("%s: cannot erase block in protected area, addr=0x%llx\n",
			__func__, instr->addr);
		return -EINVAL;
	}
	if (instr->len != mtd->erasesize) {
		pr_err("%s: unsupported erase len, %lld\n",
			__func__, instr->len);
		return -EINVAL;
	}

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->data[0] = MSM_NAND_CMD_BLOCK_ERASE;
	dma_buffer->data[1] = page;
	dma_buffer->data[2] = 0;
	dma_buffer->data[3] = 0 | 4;
	dma_buffer->data[4] = 1;
	dma_buffer->data[5] = 0xeeeeeeee;
	dma_buffer->data[6] = chip->CFG0 & (~(7 << 6));  /* CW_PER_PAGE = 0 */
	dma_buffer->data[7] = chip->CFG1;
	dma_buffer->data[8] = 0;
	BUILD_BUG_ON(8 != ARRAY_SIZE(dma_buffer->data) - 1);

	dma_buffer->cmd[0].cmd = DST_CRCI_NAND_CMD | CMD_OCB;
	dma_buffer->cmd[0].src = msm_virt_to_dma(chip, &dma_buffer->data[0]);
	dma_buffer->cmd[0].dst = MSM_NAND_FLASH_CMD;
	dma_buffer->cmd[0].len = 16;

	dma_buffer->cmd[1].cmd = 0;
	dma_buffer->cmd[1].src = msm_virt_to_dma(chip, &dma_buffer->data[6]);
	dma_buffer->cmd[1].dst = MSM_NAND_DEV0_CFG0;
	dma_buffer->cmd[1].len = 8;

	dma_buffer->cmd[2].cmd = 0;
	dma_buffer->cmd[2].src = msm_virt_to_dma(chip, &dma_buffer->data[4]);
	dma_buffer->cmd[2].dst = MSM_NAND_EXEC_CMD;
	dma_buffer->cmd[2].len = 4;

	dma_buffer->cmd[3].cmd = SRC_CRCI_NAND_DATA;
	dma_buffer->cmd[3].src = MSM_NAND_FLASH_STATUS;
	dma_buffer->cmd[3].dst = msm_virt_to_dma(chip, &dma_buffer->data[5]);
	dma_buffer->cmd[3].len = 4;

	/* clear the status register in case the OP_ERR is set
	 * due to the write, to work around a h/w bug */
	dma_buffer->cmd[4].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[4].src = msm_virt_to_dma(chip, &dma_buffer->data[8]);
	dma_buffer->cmd[4].dst = MSM_NAND_FLASH_STATUS;
	dma_buffer->cmd[4].len = 4;

	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->cmd) - 1);
	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	/* we fail if there was an operation error, a mpu error, or the
	 * erase success bit was not set.
	 */

	if (dma_buffer->data[5] & 0x110 || !(dma_buffer->data[5] & 0x80)) {
		if (dma_buffer->data[5] & 0x10)
			pr_warning("msm_nand: critical erase error, 0x%llx\n",
				   instr->addr);
		err = -EIO;
	} else
		err = 0;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (err) {
		pr_err("%s: erase failed, 0x%llx\n", __func__, instr->addr);
		instr->fail_addr = instr->addr;
		instr->state = MTD_ERASE_FAILED;
	} else {
		instr->state = MTD_ERASE_DONE;
		instr->fail_addr = 0xffffffff;
		mtd_erase_callback(instr);
	}

	return err;
}

static int
msm_nand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct msm_nand_chip *chip = mtd->priv;
	int ret;
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
			uint32_t ecccfg;
			struct {
				uint32_t flash_status;
				uint32_t buffer_status;
			} result;
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	uint8_t *buf;
	unsigned page = 0;
	unsigned cwperpage;

	if (mtd->writesize == 2048)
		page = ofs >> 11;

	if (mtd->writesize == 4096)
		page = ofs >> 12;

	cwperpage = (mtd->writesize >> 9);

	/* Check for invalid offset */
	if (ofs > mtd->size)
		return -EINVAL;
	if (ofs & (mtd->erasesize - 1)) {
		pr_err("%s: unsupported block address, 0x%x\n",
			 __func__, (uint32_t)ofs);
		return -EINVAL;
	}

	if (ofs < (NUM_PROTECTED_BLOCKS*mtd->erasesize))
	{
		//protected blocks are always good
		return 0;
	} 

	wait_event(chip->wait_queue,
		(dma_buffer = msm_nand_get_dma_buffer(chip ,
					 sizeof(*dma_buffer) + 4)));
	buf = (uint8_t *)dma_buffer + sizeof(*dma_buffer);

	/* Read 4 bytes starting from the bad block marker location
	 * in the last code word of the page
	 */

	cmd = dma_buffer->cmd;

	dma_buffer->data.cmd = MSM_NAND_CMD_PAGE_READ;
	dma_buffer->data.cfg0 = MSM_NAND_CFG0_RAW & ~(7U << 6);
	dma_buffer->data.cfg1 = MSM_NAND_CFG1_RAW | (chip->CFG1 & CFG1_WIDE_FLASH);

	if (chip->CFG1 & CFG1_WIDE_FLASH)
		dma_buffer->data.addr0 = (page << 16) |
			((528*(cwperpage-1)) >> 1);
	else
		dma_buffer->data.addr0 = (page << 16) |
			(528*(cwperpage-1));

	dma_buffer->data.addr1 = (page >> 16) & 0xff;
	dma_buffer->data.chipsel = 0 | 4;

	dma_buffer->data.exec = 1;

	dma_buffer->data.result.flash_status = 0xeeeeeeee;
	dma_buffer->data.result.buffer_status = 0xeeeeeeee;

	cmd->cmd = DST_CRCI_NAND_CMD;
	cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
	cmd->dst = MSM_NAND_FLASH_CMD;
	cmd->len = 16;
	cmd++;

	cmd->cmd = 0;
	cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0);
	cmd->dst = MSM_NAND_DEV0_CFG0;
	cmd->len = 8;
	cmd++;

	cmd->cmd = 0;
	cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
	cmd->dst = MSM_NAND_EXEC_CMD;
	cmd->len = 4;
	cmd++;

	cmd->cmd = SRC_CRCI_NAND_DATA;
	cmd->src = MSM_NAND_FLASH_STATUS;
	cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.result);
	cmd->len = 8;
	cmd++;

	cmd->cmd = 0;
	cmd->src = MSM_NAND_FLASH_BUFFER + (mtd->writesize - (528*(cwperpage-1)));
	cmd->dst = msm_virt_to_dma(chip, buf);
	cmd->len = 4;
	cmd++;

	BUILD_BUG_ON(5 != ARRAY_SIZE(dma_buffer->cmd));
	BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
	dma_buffer->cmd[0].cmd |= CMD_OCB;
	cmd[-1].cmd |= CMD_OCU | CMD_LC;

	dma_buffer->cmdptr = (msm_virt_to_dma(chip,
				dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(chip->dma_channel, crci_mask, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	ret = 0;
	if (dma_buffer->data.result.flash_status & 0x110)
		ret = -EIO;

	if (!ret) {
		/* Check for bad block marker byte */
		if (chip->CFG1 & CFG1_WIDE_FLASH) {
			if (buf[0] != 0xFF || buf[1] != 0xFF)
				ret = 1;
		} else {
			if (buf[0] != 0xFF)
				ret = 1;
		}
	}

	if (ret==1)
		printk("###BAD BLOCK %d %08X###\n", (uint32_t)(ofs) / mtd->erasesize, (uint32_t)ofs);
	else if (ret != 0)
		printk("###ERROR CHECKING BLOCK %d %08X###\n", (uint32_t)(ofs) / mtd->erasesize, (uint32_t)ofs);

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer) + 4);
	return ret;
}

static int
msm_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	/* struct msm_nand_chip *this = mtd->priv; */
	int ret;

	ret = msm_nand_block_isbad(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}

	return -EIO;
}

/**
 * msm_nand_suspend - [MTD Interface] Suspend the msm_nand flash
 * @param mtd		MTD device structure
 */
static int msm_nand_suspend(struct mtd_info *mtd)
{
	return 0;
}

/**
 * msm_nand_resume - [MTD Interface] Resume the msm_nand flash
 * @param mtd		MTD device structure
 */
static void msm_nand_resume(struct mtd_info *mtd)
{
}

/**
 * Export 3 attributes for HTC SSD HW INFO tool
 * >info    :basic HW spec of this NAND chip
 * >vendor  :vendor information
 * >pagesize:page size, either 2048 or 4096
 */
static int param_get_vendor_name(char *buffer, struct kernel_param *kp)
{
	return sprintf(buffer, "%s", nand_info->maker_name);
}
module_param_call(vendor, NULL, param_get_vendor_name, NULL, S_IRUGO);

static int param_get_nand_info(char *buffer, struct kernel_param *kp)
{
	int result = 0;
	result += sprintf(buffer, "<<  NAND INFO  >>\n");
	result += sprintf(buffer + result, "flash id\t =%X\n",
				nand_info->flash_id);
	result += sprintf(buffer + result, "vendor\t\t =%s\n",
				nand_info->maker_name);
	result += sprintf(buffer + result, "width\t\t =%d bits\n",
				nand_info->width);
	result += sprintf(buffer + result, "size\t\t =%d MB\n",
				nand_info->size>>20);
	result += sprintf(buffer + result, "block count\t =%d\n",
				nand_info->block_count);
	result += sprintf(buffer + result, "page count\t =%d",
				nand_info->page_count);
	return result;
}
module_param_call(info, NULL, param_get_nand_info, NULL, S_IRUGO);

static int param_get_page_size(char *buffer, struct kernel_param *kp)
{
	return sprintf(buffer, "%d", nand_info->page_size);
}
module_param_call(pagesize, NULL, param_get_page_size, NULL, S_IRUGO);

/**
 * scanmac - Scan for HTC Leo mac addresses
 * 
 * @author: Franck78, Rick_1995, marc1706
 */
void scanmac(struct mtd_info *mtd)
{
	unsigned char *iobuf;
	int ret;
	loff_t addr;
	struct mtd_oob_ops ops;

	iobuf = kmalloc(2048/*mtd->erasesize*/, GFP_KERNEL);
	if (!iobuf) {
		printk("%s: error: cannot allocate memory\n",__func__);
		return;
	}

	ops.mode = MTD_OOB_PLACE;
	ops.len = 2048;
	ops.datbuf = iobuf;
	ops.ooblen = 0;
	ops.oobbuf = NULL;
	ops.retlen = 0;

	addr = ((loff_t) 505*0x20000);
	ret = msm_nand_read_oob(mtd, addr, &ops);
	if (ret == -EUCLEAN)
		ret = 0;
	if (ret || ops.retlen != 2048 ) {
		printk("%s: error: read(%d) failed at %#llx\n",__func__,
		       ops.retlen, addr);
		goto out;
	}
	sprintf(nvs_mac_addr, "macaddr=%02x:%02x:%02x:%02x:%02x:%02x\n",
		iobuf[40],iobuf[41],iobuf[42],iobuf[43],iobuf[44],iobuf[45]);
	pr_info("Device WiFi MAC Address: %s\n", nvs_mac_addr);

	addr = ((loff_t) 505*0x20000 + 6*0x800);
	ret = msm_nand_read_oob(mtd, addr, &ops);
	if (ret == -EUCLEAN)
		ret = 0;
	if (ret || ops.retlen != 2048 ) {
		printk("%s: error: read(%d) failed at %#llx\n",__func__,
		       ops.retlen, addr);
		goto out;
	}
	sprintf(bdaddr, "%02x:%02x:%02x:%02x:%02x:%02x",iobuf[5],iobuf[4],
		iobuf[3],iobuf[2],iobuf[1],iobuf[0]);
	pr_info("Device Bluetooth MAC Address: %s\n", bdaddr);
	ret = 0;

out:
	kfree(iobuf);
	if (ret) printk("Find MAC Error %d occurred\n", ret);
	return;
}

/**
* msm_nand_scan - [msm_nand Interface] Scan for the msm_nand device
* @param mtd	   MTD device structure
* @param maxchips  Number of chips to scan for
*
* This fills out all the not initialized function pointers
* with the defaults.
* The flash ID is read and the mtd/chip structures are
* filled with the appropriate values.
*/
int msm_nand_scan(struct mtd_info *mtd, int maxchips)
{
	unsigned n;
	struct msm_nand_chip *chip = mtd->priv;
	uint32_t flash_id = 0, i = 1, mtd_writesize;
	uint8_t dev_found = 0;
	uint8_t wide_bus;
	uint8_t index;

	/* Read the Flash ID from the Nand Flash Device */
	flash_id = flash_read_id(chip);
	for (index = 1; index < ARRAY_SIZE(supported_flash); index++) {
		if ((flash_id & supported_flash[index].mask) ==
		    (supported_flash[index].flash_id &
		     (supported_flash[index].mask))) {
			dev_found = 1;
			break;
		}
	}

	if (dev_found) {
		wide_bus       = supported_flash[index].widebus;
		mtd->size      = supported_flash[index].density  * i;
		mtd->writesize = supported_flash[index].pagesize * i;
		mtd->oobsize   = supported_flash[index].oobsize  * i;
		mtd->erasesize = supported_flash[index].blksize  * i;
		mtd_writesize = mtd->writesize;

		pr_info("Found a supported NAND device\n");
		pr_info("NAND Id  : 0x%X\n", supported_flash[index].
			flash_id);
		pr_info("Buswidth : %d Bits \n", (wide_bus) ? 16 : 8);
		pr_info("Density  : %lld MByte\n", (mtd->size>>20));
		pr_info("Pagesize : %d Bytes\n", mtd->writesize);
		pr_info("Erasesize: %d Bytes\n", mtd->erasesize);
		pr_info("Oobsize  : %d Bytes\n", mtd->oobsize);
	} else {
		pr_err("Unsupported Nand,Id: 0x%x \n", flash_id);
		return -ENODEV;
	}

	if (mtd->writesize != 2048)
	{
		pr_err("ERROR: unsupported flash parameters!\n");
		return -ENODEV;
	}

	if (flash_read_config(chip)) 
	{
		pr_err("ERROR: could not save CFG0 & CFG1 state\n");
		return -ENODEV;
	}

	pr_info("CFG0 Old: 0x%08x \n", chip->CFG0);
	pr_info("CFG1 Old: 0x%08x \n", chip->CFG1);

	chip->CFG0 = (((mtd->writesize >> 9)-1) << 6) /* 4/8 cw/per page for 2/4k */
		|  (512 <<  9)  /* 512 user data bytes */
		|   (10 << 19)  /* 10 parity bytes */
		|    (4 << 23)  /* spare size */
		|    (5 << 27)  /* 5 address cycles */
		|    (1 << 30)  /* Read status before data */
		|    (1 << 31); /* Send read cmd */
	chip->CFG1 = chip->CFG1
#if IGNORE_ARM9_CONFIG
		/* use ARM11 own setting on CFG1 */
		|    (7 <<  2)  /* 8 recovery cycles */
		|    (0 <<  5)  /* Allow CS deassertion */
		|    (2 << 17)  /* 6 cycle tWB/tRB */
#endif
		|  ((mtd->writesize - (528 * ((mtd->writesize >> 9) - 1)) + 1) <<  6)  /* Bad block marker location */
		| (wide_bus << 1); /* Wide flash bit */
	chip->CFG1 = chip->CFG1
		&   ~(1 <<  0)  /* Enable ecc */
		&   ~(1 << 16); /* Bad block in user data area */

	pr_info("CFG0 Init  : 0x%08x \n", chip->CFG0);
	pr_info("CFG1 Init  : 0x%08x \n", chip->CFG1);
	pr_info("CFG0: cw/page=%d ud_sz=%d ecc_sz=%d spare_sz=%d "
		"num_addr_cycles=%d\n", (chip->CFG0 >> 6) & 7,
		(chip->CFG0 >> 9) & 0x3ff, (chip->CFG0 >> 19) & 15,
		(chip->CFG0 >> 23) & 15, (chip->CFG0 >> 27) & 7);

	n = flash_rd_reg(chip, MSM_NAND_DEV_CMD1);
	pr_info("DEV_CMD1: %x\n", n);

	n = flash_rd_reg(chip, MSM_NAND_EBI2_ECC_BUF_CFG);
	pr_info(KERN_INFO "NAND_EBI2_ECC_BUF_CFG: %x\n", n);

#if SUPPORT_WRONG_ECC_CONFIG
	chip->ecc_buf_cfg = 0x1FF; 
	chip->saved_ecc_buf_cfg = n;
#endif

	if (mtd->oobsize == 64) 
	{
		mtd->oobavail = msm_nand_oob_64.oobavail;
		mtd->ecclayout = &msm_nand_oob_64;
	} 
	else 
	{
		pr_err("Unsupported Nand, oobsize: 0x%x \n", mtd->oobsize);
		return -ENODEV;
	}

	/* Fill in remaining MTD driver data */
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	/* mtd->ecctype = MTD_ECC_SW; */
	mtd->erase = msm_nand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = msm_nand_read;
	mtd->write = msm_nand_write;
	mtd->read_oob = msm_nand_read_oob;
	mtd->write_oob = msm_nand_write_oob;
	/* mtd->sync = msm_nand_sync; */
	mtd->lock = NULL;
	/* mtd->unlock = msm_nand_unlock; */
	mtd->suspend = msm_nand_suspend;
	mtd->resume = msm_nand_resume;
	mtd->block_isbad = msm_nand_block_isbad;
	mtd->block_markbad = msm_nand_block_markbad;
	mtd->owner = THIS_MODULE;

	/* Information provides to HTC SSD HW Info tool */
	nand_info = &chip->dev_info;
	nand_info->flash_id = flash_id;
	nand_info->maker_id = (flash_id & 0xff);
	switch (nand_info->maker_id) 
	{
	case 0xec:
		strcpy(nand_info->maker_name, "Samsung");
		break;
	case 0xad:
		strcpy(nand_info->maker_name, "Hynix");
		break;
	case 0x2c:
		strcpy(nand_info->maker_name, "Micron");
		break;
	default:
		strcpy(nand_info->maker_name, "Unknown");
		break;
	}
	nand_info->width = (wide_bus? 16: 8);
	nand_info->size = mtd->size;
	nand_info->page_size = mtd->writesize;
	nand_info->page_count = mtd->erasesize/mtd->writesize;
	nand_info->block_count = mtd->size;
	do_div(nand_info->block_count, nand_info->page_size * nand_info->page_count);

	/* Unlock whole block */
	/* msm_nand_unlock_all(mtd); */

	/* return this->scan_bbt(mtd); */
	scanmac(mtd);

#if VERBOSE
	for (i=0;i<nand_info->block_count;i++)
	{
		int blockretval = msm_nand_block_isbad(mtd,i*mtd->erasesize);
		if(blockretval == 1)
		{
			pr_info("msm_nand block %d is bad\n",i);
		}
		else if (blockretval == -EINVAL)
		{
			pr_info("msm_nand block %d -EINVAL\n",i);
		}
		else if (blockretval == -EIO)
		{
			pr_info("msm_nand block %d -EIO\n",i);
		}
	}
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(msm_nand_scan);

/**
 * msm_nand_release - [msm_nand Interface] Free resources held by the msm_nand device
 * @param mtd		MTD device structure
 */
void msm_nand_release(struct mtd_info *mtd)
{
	/* struct msm_nand_chip *this = mtd->priv; */

#ifdef CONFIG_MTD_PARTITIONS
	/* Deregister partitions */
	del_mtd_partitions(mtd);
#endif
	/* Deregister the device */
	del_mtd_device(mtd);
}
EXPORT_SYMBOL_GPL(msm_nand_release);

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL,  };
#endif

struct msm_nand_info {
	struct mtd_info		mtd;
	struct mtd_partition	*parts;
	struct msm_nand_chip	msm_nand;
};

static int __devinit msm_nand_probe(struct platform_device *pdev)
{
	struct msm_nand_info *info;
	struct flash_platform_data *pdata = pdev->dev.platform_data;
	int err, i;

	if (pdev->num_resources != 1) {
		pr_err("invalid num_resources");
		return -ENODEV;
	}
	if (pdev->resource[0].flags != IORESOURCE_DMA) {
		pr_err("invalid resource type");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct msm_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->msm_nand.dev = &pdev->dev;

	init_waitqueue_head(&info->msm_nand.wait_queue);

	info->msm_nand.dma_channel = pdev->resource[0].start;
	/* this currently fails if dev is passed in */
	info->msm_nand.dma_buffer =
		dma_alloc_coherent(/*dev*/ NULL, MSM_NAND_DMA_BUFFER_SIZE,
				   &info->msm_nand.dma_addr, GFP_KERNEL);
	if (info->msm_nand.dma_buffer == NULL) {
		err = -ENOMEM;
		goto out_free_info;
	}

	pr_info("msm_nand: allocated dma buffer at %p, dma_addr %x\n",
		info->msm_nand.dma_buffer, info->msm_nand.dma_addr);

	info->mtd.name = dev_name(&pdev->dev);
	info->mtd.priv = &info->msm_nand;
	info->mtd.owner = THIS_MODULE;

	if (msm_nand_scan(&info->mtd, 1)) {
		err = -ENXIO;
		goto out_free_dma_buffer;
	}

	printk("#MTD# parts in atag = %d\n", pdata->nr_parts);

#ifdef CONFIG_MTD_PARTITIONS
	/* Re-calculate the partition offset and size with correct page size */
	for (i = 0; i < pdata->nr_parts; i++) {
		pdata->parts[i].offset = pdata->parts[i].offset
			* info->mtd.erasesize;
		pdata->parts[i].size = pdata->parts[i].size
			* info->mtd.erasesize;
	}

	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (err <= 0 && pdata && pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		err = add_mtd_device(&info->mtd);

	dev_set_drvdata(&pdev->dev, info);

	return 0;

out_free_dma_buffer:
	dma_free_coherent(/*dev*/ NULL, SZ_4K, info->msm_nand.dma_buffer,
			  info->msm_nand.dma_addr);
out_free_info:
	kfree(info);

	return err;
}

static int __devexit msm_nand_remove(struct platform_device *pdev)
{
	struct msm_nand_info *info = dev_get_drvdata(&pdev->dev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (info) {
#ifdef CONFIG_MTD_PARTITIONS
		if (info->parts)
			del_mtd_partitions(&info->mtd);
		else
#endif
			del_mtd_device(&info->mtd);

		msm_nand_release(&info->mtd);
		dma_free_coherent(/*dev*/ NULL, SZ_4K,
				  info->msm_nand.dma_buffer,
				  info->msm_nand.dma_addr);
		kfree(info);
	}

	return 0;
}

#define DRIVER_NAME "msm_nand"

static struct platform_driver msm_nand_driver = {
	.probe		= msm_nand_probe,
	.remove		= __devexit_p(msm_nand_remove),
	.driver = {
		.name		= DRIVER_NAME,
	}
};

MODULE_ALIAS(DRIVER_NAME);

static int __init msm_nand_init(void)
{
	return platform_driver_register(&msm_nand_driver);
}

static void __exit msm_nand_exit(void)
{
	platform_driver_unregister(&msm_nand_driver);
}

module_init(msm_nand_init);
module_exit(msm_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("htcleo nand driver code");


