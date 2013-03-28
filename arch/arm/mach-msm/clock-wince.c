/* arch/arm/mach-msm/clock.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include <mach/msm_iomap.h>
#include <asm/io.h>

#include "clock.h"
#include "proc_comm.h"

//#define ENABLE_CLOCK_INFO   1

extern struct clk msm_clocks[];

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static LIST_HEAD(clocks);

enum {
	DEBUG_UNKNOWN_ID	= 1<<0,
	DEBUG_UNKNOWN_FREQ	= 1<<1,
	DEBUG_MDNS		= 1<<2,
	DEBUG_UNKNOWN_CMD	= 1<<3,
};
static int debug_mask=DEBUG_MDNS;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#if 1
#define D(x...) printk(KERN_DEBUG "clock-wince: " x)
#else
#define D(x...) do {} while (0)
#endif

struct mdns_clock_params
{
	unsigned long freq;
	uint32_t calc_freq;
	uint32_t md;
	uint32_t ns;
	uint32_t pll_freq;
	uint32_t clk_id;
};

struct msm_clock_params
{
	unsigned clk_id;
	uint32_t glbl;	  // Whitch config reg GLBL_CLK_ENA or GLBL_CLK_ENA_2
	unsigned idx;
	unsigned offset;  // Offset points to .ns register
	unsigned ns_only; // value to fill in ns register, rather than using mdns_clock_params look-up table
	char	*name;
};

static int max_clk_rate[NR_CLKS], min_clk_rate[NR_CLKS];

#define GLBL_CLK_ENA           ((uint32_t)MSM_CLK_CTL_BASE)
#define GLBL_CLK_ENA_2         ((uint32_t)MSM_CLK_CTL_BASE + 0x220)

#if defined(CONFIG_ARCH_QSD8X50)
#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 32 * (n))
#else
#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#endif
#define TCX0			19200000 // Hz
#define PLL_FREQ(l, m, n)	(TCX0 * (l) + TCX0 * (m) / (n))

static unsigned int pll_get_rate(int n)
{
	unsigned int mode, L, M, N, freq;

 if (n == -1) return TCX0;
#if defined(CONFIG_ARCH_QSD8X50)
 if (n > 1)
#else
 if (n > 3)
#endif
  return 0;
 else
 {
	mode = readl(PLLn_BASE(n) + 0x0);
	L = readl(PLLn_BASE(n) + 0x4);
	M = readl(PLLn_BASE(n) + 0x8);
	N = readl(PLLn_BASE(n) + 0xc);
	freq = PLL_FREQ(L, M, N);
	printk(KERN_INFO "PLL%d: MODE=%08x L=%08x M=%08x N=%08x freq=%u Hz (%u MHz)\n",
		n, mode, L, M, N, freq, freq / 1000000); \
 }

 return freq;
}

static unsigned int idx2pll(uint32_t idx)
{
 int ret;

 switch(idx)
 {
  case 0: /* TCX0 */
   ret=-1;
  break;
  case 1: /* PLL1 */
   ret=1;
  break;
  case 4: /* PLL0 */
   ret=0;
  break;
  default:
   ret=4; /* invalid */
 }

 return ret;
}
static struct msm_clock_params msm_clock_parameters[] = {
	// Full ena/md/ns clock
	{ .clk_id = SDC1_CLK, .glbl = GLBL_CLK_ENA, .idx =  7, .offset = 0xa4, .name="SDC1_CLK",},
	{ .clk_id = SDC2_CLK, .glbl = GLBL_CLK_ENA, .idx =  8, .offset = 0xac, .name="SDC2_CLK",},
#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = SDC3_CLK, .glbl = GLBL_CLK_ENA, .idx = 27, .offset = 0x3d8, .name="SDC3_CLK",},
	{ .clk_id = SDC4_CLK, .glbl = GLBL_CLK_ENA, .idx = 28, .offset = 0x3e0, .name="SDC4_CLK",},
#else
	{ .clk_id = SDC3_CLK, .glbl = GLBL_CLK_ENA, .idx = 27, .offset = 0xb4, .name="SDC3_CLK",},
	{ .clk_id = SDC4_CLK, .glbl = GLBL_CLK_ENA, .idx = 28, .offset = 0xbc, .name="SDC4_CLK",},
#endif
#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = UART1DM_CLK, .glbl = GLBL_CLK_ENA, .idx = 17, .offset = 0x124, .name="UART1DM_CLK",},
	{ .clk_id = UART2DM_CLK, .glbl = GLBL_CLK_ENA, .idx = 26, .offset = 0x12c, .name="UART2DM_CLK",},
	{ .clk_id = USB_HS_CLK, .glbl = GLBL_CLK_ENA_2, .idx = 7, .offset = 0x3e8, .ns_only = 0xb41, .name="USB_HS_CLK",},
#else
	{ .clk_id = UART1DM_CLK, .glbl = GLBL_CLK_ENA, .idx = 17, .offset = 0xd4, .name="UART1DM_CLK",},
	{ .clk_id = UART2DM_CLK, .glbl = GLBL_CLK_ENA, .idx = 26, .offset = 0xdc, .name="UART2DM_CLK",},
	{ .clk_id = USB_HS_CLK, .glbl = GLBL_CLK_ENA, .idx = 25, .offset = 0x2c0, .ns_only = 0xb00, .name="USB_HS_CLK",},
#endif
	// these both enable the GRP and IMEM clocks.
	{ .clk_id = GRP_CLK, .glbl = GLBL_CLK_ENA, .idx = 3, .offset = 0x84, .ns_only = 0xa80, .name="GRP_CLK", }, 
	{ .clk_id = IMEM_CLK, .glbl = GLBL_CLK_ENA, .idx = 3, .offset = 0x84, .ns_only = 0xa80, .name="IMEM_CLK", },


	// MD/NS only; offset = Ns reg
#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = VFE_CLK, .glbl = GLBL_CLK_ENA, .idx = 2, .offset = 0x40, .name="VFE_CLK", },
#else
	{ .clk_id = VFE_CLK, .offset = 0x44, .name="VFE_CLK", },
#endif
	
	// Enable bit only; bit = 1U << idx
	{ .clk_id = MDP_CLK, .glbl = GLBL_CLK_ENA, .idx = 9, .name="MDP_CLK",},
	
	// NS-reg only; offset = Ns reg, ns_only = Ns value
#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = GP_CLK, .glbl = GLBL_CLK_ENA, .offset = 0x58, .ns_only = 0x800, .name="GP_CLK" },
#else
	{ .clk_id = GP_CLK, .offset = 0x5c, .ns_only = 0xa06, .name="GP_CLK" },
#endif

#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = PMDH_CLK, .glbl = GLBL_CLK_ENA_2, .idx = 4, .offset = 0x8c, .ns_only = 0x00c, .name="PMDH_CLK"},
#else
	{ .clk_id = PMDH_CLK, .offset = 0x8c, .ns_only = 0xa0c, .name="PMDH_CLK"},
#endif

#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = I2C_CLK, .offset = 0x64, .ns_only = 0xa00, .name="I2C_CLK"},
#else
	{ .clk_id = I2C_CLK, .offset = 0x68, .ns_only = 0xa00, .name="I2C_CLK"},
#endif

#if defined(CONFIG_ARCH_QSD8X50)
	{ .clk_id = SPI_CLK, .glbl = GLBL_CLK_ENA_2, .idx = 13, .offset = 0x14c, .ns_only = 0xa08, .name="SPI_CLK"},
#endif

#if defined(CONFIG_ARCH_QSD8X50)
//	{ .clk_id = UART1_CLK, .offset = 0xc0, .ns_only = 0xa00, .name="UART1_CLK"},
#else
//	{ .clk_id = UART1_CLK, .offset = 0xe0, .ns_only = 0xa00, .name="UART1_CLK"},
#endif
};

// This formula is used to generate md and ns reg values
#define MSM_CLOCK_REG(frequency,M,N,D,PRE,a5,SRC,MNE,pll_frequency) { \
	.freq = (frequency), \
	.md = ((0xffff & (M)) << 16) | (0xffff & ~((D) << 1)), \
	.ns = ((0xffff & ~((N) - (M))) << 16) \
	    | ((0xff & (0xa | (MNE))) << 8) \
	    | ((0x7 & (a5)) << 5) \
	    | ((0x3 & (PRE)) << 3) \
	    | (0x7 & (SRC)), \
	.pll_freq = (pll_frequency), \
	.calc_freq = 1000*((pll_frequency/1000)*M/((PRE+1)*N)), \
}

struct mdns_clock_params msm_clock_freq_parameters[] = {

	MSM_CLOCK_REG(  144000,   3, 0x64, 0x32, 3, 3, 0, 1, 19200000), /* SD, 144kHz */
	MSM_CLOCK_REG(  400000,   1, 0x30, 0x15, 0, 3, 0, 1, 19200000), /* SD, 400kHz */
#if 0 /* wince uses this clock setting for UART2DM */
	MSM_CLOCK_REG( 1843200,     3, 0x64, 0x32, 3, 2, 4, 1, 245760000), /*  115200*16=1843200 */
//	MSM_CLOCK_REG(            , 2, 0xc8, 0x64, 3, 2, 1, 1, 768888888), /* 1.92MHz for 120000 bps */
#else
	MSM_CLOCK_REG( 7372800,   3, 0x64, 0x32, 0, 2, 4, 1, 245760000), /*  460800*16, will be divided by 4 for 115200 */
#endif
	MSM_CLOCK_REG(12000000,   1, 0x20, 0x10, 1, 3, 1, 1, 768000000), /* SD, 12MHz */
	MSM_CLOCK_REG(14745600,   3, 0x32, 0x19, 0, 2, 4, 1, 245760000), /* BT, 921600 (*16)*/
	MSM_CLOCK_REG(19200000,   1, 0x0a, 0x05, 3, 3, 1, 1, 768000000), /* SD, 19.2MHz */
	MSM_CLOCK_REG(24000000,   1, 0x10, 0x08, 1, 3, 1, 1, 768000000), /* SD, 24MHz */
	MSM_CLOCK_REG(24576000,   1, 0x0a, 0x05, 0, 2, 4, 1, 245760000), /* SD, 24,576000MHz */
	MSM_CLOCK_REG(25000000,  14, 0xd7, 0x6b, 1, 3, 1, 1, 768000000), /* SD, 25MHz */
	MSM_CLOCK_REG(32000000,   1, 0x0c, 0x06, 1, 3, 1, 1, 768000000), /* SD, 32MHz */
	MSM_CLOCK_REG(48000000,   1, 0x08, 0x04, 1, 3, 1, 1, 768000000), /* SD, 48MHz */
	MSM_CLOCK_REG(50000000,  25, 0xc0, 0x60, 1, 3, 1, 1, 768000000), /* SD, 50MHz */
	MSM_CLOCK_REG(58982400,   6, 0x19, 0x0c, 0, 2, 4, 1, 245760000), /* BT, 3686400 (*16) */
	MSM_CLOCK_REG(64000000,0x19, 0x60, 0x30, 0, 2, 4, 1, 245760000), /* BT, 4000000 (*16) */
};

static void set_grp_clk( int on )
{
	int i = 0;
	int status = 0;
	int control;

	if ( on != 0 )
	{
		//axi_reset
		writel(readl(MSM_CLK_CTL_BASE+0x208) |0x20,          MSM_CLK_CTL_BASE+0x208); //AXI_RESET
		//row_reset
		writel(readl(MSM_CLK_CTL_BASE+0x214) |0x20000,       MSM_CLK_CTL_BASE+0x214); //ROW_RESET
		//vdd_grp gfs_ctl
		writel(                              0x11f,          MSM_CLK_CTL_BASE+0x284); //VDD_GRP_GFS_CTL
		// very rough delay
		mdelay(20);
		//grp NS
		writel(readl(MSM_CLK_CTL_BASE+0x84)  |0x800,         MSM_CLK_CTL_BASE+0x84); //GRP_NS_REG
		writel(readl(MSM_CLK_CTL_BASE+0x84)  |0x80,          MSM_CLK_CTL_BASE+0x84); //GRP_NS_REG
		writel(readl(MSM_CLK_CTL_BASE+0x84)  |0x200,         MSM_CLK_CTL_BASE+0x84); //GRP_NS_REG
		//grp idx
		writel(readl(MSM_CLK_CTL_BASE)       |0x8,           MSM_CLK_CTL_BASE);
		//grp clk ramp
		writel(readl(MSM_CLK_CTL_BASE+0x290) &(~(0x4)),      MSM_CLK_CTL_BASE+0x290); //MSM_RAIL_CLAMP_IO
		//Suppress bit 0 of grp MD (?!?)
		writel(readl(MSM_CLK_CTL_BASE+0x80)  &(~(0x1)),      MSM_CLK_CTL_BASE+0x80);  //PRPH_WEB_NS_REG
		//axi_reset
		writel(readl(MSM_CLK_CTL_BASE+0x208) &(~(0x20)),     MSM_CLK_CTL_BASE+0x208); //AXI_RESET
		//row_reset
#if defined(CONFIG_ARCH_QSD8X50)
		writel(readl(MSM_CLK_CTL_BASE+0x218) &(~(0x20000)),  MSM_CLK_CTL_BASE+0x218); //ROW_RESET
#else
		writel(readl(MSM_CLK_CTL_BASE+0x214) &(~(0x20000)),  MSM_CLK_CTL_BASE+0x214); //ROW_RESET
#endif
	}
	else
	{
		//grp NS
		writel(readl(MSM_CLK_CTL_BASE+0x84)  |0x800,         MSM_CLK_CTL_BASE+0x84); //GRP_NS_REG
		writel(readl(MSM_CLK_CTL_BASE+0x84)  |0x80,          MSM_CLK_CTL_BASE+0x84); //GRP_NS_REG
		writel(readl(MSM_CLK_CTL_BASE+0x84)  |0x200,         MSM_CLK_CTL_BASE+0x84); //GRP_NS_REG
		//grp idx
		writel(readl(MSM_CLK_CTL_BASE)       |0x8,           MSM_CLK_CTL_BASE);
		//grp MD
		writel(readl(MSM_CLK_CTL_BASE+0x80)  |0x1,      	 MSM_CLK_CTL_BASE+0x80);  //PRPH_WEB_NS_REG

		while ( status == 0 && i < 100) {
			i++;
			status = readl(MSM_CLK_CTL_BASE+0x84) & 0x1;			
		}
		
		//axi_reset
		writel(readl(MSM_CLK_CTL_BASE+0x208) |0x20,     	MSM_CLK_CTL_BASE+0x208); //AXI_RESET
		//row_reset
#if defined(CONFIG_ARCH_QSD8X50)
		writel(readl(MSM_CLK_CTL_BASE+0x218) |0x20000,  	MSM_CLK_CTL_BASE+0x218); //ROW_RESET
#else
		writel(readl(MSM_CLK_CTL_BASE+0x214) |0x20000,  	MSM_CLK_CTL_BASE+0x214); //ROW_RESET
#endif
		//grp NS
		writel(readl(MSM_CLK_CTL_BASE+0x84)  &(~(0x800)),   MSM_CLK_CTL_BASE+0x84);  //GRP_NS_REG
		writel(readl(MSM_CLK_CTL_BASE+0x84)  &(~(0x80)),    MSM_CLK_CTL_BASE+0x84);  //GRP_NS_REG
		writel(readl(MSM_CLK_CTL_BASE+0x84)  &(~(0x200)),   MSM_CLK_CTL_BASE+0x84);  //GRP_NS_REG
		//grp clk ramp
		writel(readl(MSM_CLK_CTL_BASE+0x290) |0x4,      	MSM_CLK_CTL_BASE+0x290); //MSM_RAIL_CLAMP_IO
		writel(                              0x11f,         MSM_CLK_CTL_BASE+0x284); //VDD_GRP_GFS_CTL

		control = readl(MSM_CLK_CTL_BASE+0x288); //VDD_VDC_GFS_CTL
		if ( control & 0x100 )
			writel(readl(MSM_CLK_CTL_BASE) &(~(0x8)),      	MSM_CLK_CTL_BASE);
	}
}

static inline struct msm_clock_params msm_clk_get_params(uint32_t id)
{
	int i;
	struct msm_clock_params empty = { };
	for (i = 0; i < ARRAY_SIZE(msm_clock_parameters); i++) {
		if (id == msm_clock_parameters[i].clk_id) {
			return msm_clock_parameters[i];
		}
	}
	return empty;
}

static inline uint32_t msm_clk_enable_bit(uint32_t id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	if (!params.idx) return 0;
	return 1U << params.idx;
}

static inline uint32_t msm_clk_get_glbl(uint32_t id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	if (!params.glbl) return 0;
	return params.glbl;
}

static inline unsigned msm_clk_reg_offset(uint32_t id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	return params.offset;
}

static int set_mdns_host_clock(uint32_t id, unsigned long freq)
{
	int n;
	unsigned offset;
	int retval;
	bool found;
	struct msm_clock_params params;
	uint32_t nsreg;
	found = 0;
	retval = -EINVAL;
	
	params = msm_clk_get_params(id);
	offset = params.offset;

	if(debug_mask&DEBUG_MDNS)
		D("set mdns: %u, %lu; bitidx=%u, offset=%x, ns=%x\n", id, freq, 
	  params.idx, params.offset, params.ns_only);

	if (!params.offset)
	{
		printk(KERN_WARNING "%s: FIXME! Don't know how to set clock %u - no known Md/Ns reg\n", __func__, id);
		return -ENOTSUPP;
	}

	// Turn off clock-enable bit if supported
	if (params.idx > 0 && params.glbl > 0)
		writel(readl(params.glbl) & ~(1U << params.idx), params.glbl);

	if (params.ns_only > 0)
	{
		nsreg = readl(MSM_CLK_CTL_BASE + offset) & 0xfffff000;
		writel( nsreg | params.ns_only, MSM_CLK_CTL_BASE + offset);

		found = 1;
		retval = 0;

	} else {
		for (n = ARRAY_SIZE(msm_clock_freq_parameters)-1; n >= 0; n--) {
			if (freq >= msm_clock_freq_parameters[n].freq) {
				// This clock requires MD and NS regs to set frequency:
				writel(msm_clock_freq_parameters[n].md, MSM_CLK_CTL_BASE + offset - 4);
				writel(msm_clock_freq_parameters[n].ns, MSM_CLK_CTL_BASE + offset);
//				msleep(5);
				if(debug_mask&DEBUG_MDNS)
					D("%s: %u, freq=%lu calc_freq=%u pll%d=%u expected pll =%u\n", __func__, id, 
				  msm_clock_freq_parameters[n].freq,
				  msm_clock_freq_parameters[n].calc_freq,
				  msm_clock_freq_parameters[n].ns&7,
				  pll_get_rate(idx2pll(msm_clock_freq_parameters[n].ns&7)),
				  msm_clock_freq_parameters[n].pll_freq );
				retval = 0;
				found = 1;
				break;
			}
		}
	}

	// Turn clock-enable bit back on, if supported
	if (params.idx > 0 && params.glbl > 0)
		writel(readl(params.glbl) | (1U << params.idx), params.glbl);

	if (!found && debug_mask&DEBUG_UNKNOWN_FREQ) {
		printk(KERN_WARNING "clock-wince: FIXME! set_sdcc_host_clock could not "
		       "find suitable parameter for freq %lu\n", freq);
	}

//     return retval;
       return 0;
}

static unsigned long get_mdns_host_clock(uint32_t id)
{
	int n;
	unsigned offset;
	uint32_t mdreg;
	uint32_t nsreg;
	unsigned long freq = 0;

	offset = msm_clk_reg_offset(id);
	if (offset == 0)
		return -EINVAL;

	mdreg = readl(MSM_CLK_CTL_BASE + offset - 4);
	nsreg = readl(MSM_CLK_CTL_BASE + offset);

	for (n = 0; n < ARRAY_SIZE(msm_clock_freq_parameters); n++) {
		if (msm_clock_freq_parameters[n].md == mdreg &&
			msm_clock_freq_parameters[n].ns == nsreg) {
			freq = msm_clock_freq_parameters[n].freq;
			break;
		}
	}

	return freq;
}

// Cotullaz "new" clock functions
static int new_clk_set_rate(uint32_t id, unsigned long rate)
{
    unsigned clk = -1;
    unsigned speed = 0;
    switch (id)
    {
    case ICODEC_RX_CLK:
        if (rate > 11289600)     speed = 9;
        else if (rate > 8192000) speed = 8;
        else if (rate > 6144000) speed = 7;
        else if (rate > 5644800) speed = 6;
        else if (rate > 4096000) speed = 5;
        else if (rate > 3072000) speed = 4;
        else if (rate > 2822400) speed = 3;
        else if (rate > 2048000) speed = 2;
        else speed = 1;
        clk = 50;
        break;
    case ICODEC_TX_CLK:
        if (rate > 11289600) speed = 9;
        else if (rate > 8192000) speed = 8;
        else if (rate > 6144000) speed = 7;
        else if (rate > 5644800) speed = 6;
        else if (rate > 4096000) speed = 5;
        else if (rate > 3072000) speed = 4;
        else if (rate > 2822400) speed = 3;
        else if (rate > 2048000) speed = 2;
        else speed = 1;
        clk = 52;
        break;
    case ECODEC_CLK:
        if (rate > 2048000) speed = 3;
        else if (rate > 128000) speed = 2;
        else speed = 1;
        clk = 42;
        break;   
    case SDAC_MCLK:
        if (rate > 1411200) speed = 9;
        else if (rate > 1024000) speed = 8;
        else if (rate > 768000) speed = 7;
        else if (rate > 705600) speed = 6;
        else if (rate > 512000) speed = 5;
        else if (rate > 384000) speed = 4;
        else if (rate > 352800) speed = 3;
        else if (rate > 256000) speed = 2;
        else speed = 1;
        clk = 64;
        break;

    case UART1DM_CLK:
        if (rate > 61440000) speed = 15;
        else if (rate > 58982400) speed = 14;
        else if (rate > 56000000) speed = 13;
        else if (rate > 51200000) speed = 12;
        else if (rate > 48000000) speed = 11;
        else if (rate > 40000000) speed = 10;
        else if (rate > 32000000) speed = 9;
        else if (rate > 24000000) speed = 8;
        else if (rate > 16000000) speed = 7;
        else if (rate > 15360000) speed = 6;
        else if (rate > 14745600) speed = 5;
        else if (rate >  7680000) speed = 4;
        else if (rate >  7372800) speed = 3;
        else if (rate >  3840000) speed = 2;
        else speed = 1;
        clk = 78;
        break;
    case UART2DM_CLK:
        if (rate > 61440000) speed = 15;
        else if (rate > 58982400) speed = 14;
        else if (rate > 56000000) speed = 13;
        else if (rate > 51200000) speed = 12;
        else if (rate > 48000000) speed = 11;
        else if (rate > 40000000) speed = 10;
        else if (rate > 32000000) speed = 9;
        else if (rate > 24000000) speed = 8;
        else if (rate > 16000000) speed = 7;
        else if (rate > 15360000) speed = 6;
        else if (rate > 14745600) speed = 5;
        else if (rate >  7680000) speed = 4;
        else if (rate >  7372800) speed = 3;
        else if (rate >  3840000) speed = 2;
        else speed = 1;
        clk = 80;
        break;


    case VFE_MDC_CLK:
        if (rate == 96000000) speed = 37;
        else if (rate == 48000000) speed = 32;
        else if (rate == 24000000) speed = 22;
        else if (rate == 12000000) speed = 14;
        else if (rate ==  6000000) speed = 6;
        else if (rate ==  3000000) speed = 1;
        else 
        {
            printk("wrong MDC clock %d\n", rate);
            return 0;
        }
        clk = 40;
        break;

    case VFE_CLK:
        if (rate == 36000000) speed = 1;
        else if (rate == 48000000) speed = 2;
        else if (rate == 64000000) speed = 3;
        else if (rate == 78000000) speed = 4;
        else if (rate == 96000000) speed = 5;
        else 
        {
            printk("wrong clock %d\n", rate);
            return 0;
        }
        clk = 41;
        break;

	case SPI_CLK:
		if (rate > 15360000) speed = 5;
		else if (rate > 9600000) speed = 4;
		else if (rate > 4800000) speed = 3;
		else if (rate >  960000) speed = 2;
		else speed = 1;
		clk = 95;
		break;

// Cotulla: I am too lazy...
#define MHZ(x)  ((x) * 1000 * 1000)
#define KHZ(x)  ((x) * 1000)

    case SDC1_CLK:
		if (rate > MHZ(50)) speed = 14;
        else if (rate > KHZ(49152)) speed = 13;
        else if (rate > MHZ(45)) speed = 12;
        else if (rate > MHZ(40)) speed = 11;
        else if (rate > MHZ(35)) speed = 10;
        else if (rate > MHZ(30)) speed = 9;
        else if (rate > MHZ(25)) speed = 8;
        else if (rate > MHZ(20)) speed = 7;
        else if (rate > MHZ(15)) speed = 6;
        else if (rate > MHZ(10)) speed = 5;
        else if (rate > MHZ(5))  speed = 4;
        else if (rate > KHZ(400))speed = 3;
        else if (rate > KHZ(144))speed = 2;
        else speed = 1;
        clk = 66;
        break;
    case SDC2_CLK:
		if (rate > MHZ(50)) speed = 14;
        else if (rate > KHZ(49152)) speed = 13;
        else if (rate > MHZ(45)) speed = 12;
        else if (rate > MHZ(40)) speed = 11;
        else if (rate > MHZ(35)) speed = 10;
        else if (rate > MHZ(30)) speed = 9;
        else if (rate > MHZ(25)) speed = 8;
        else if (rate > MHZ(20)) speed = 7;
        else if (rate > MHZ(15)) speed = 6;
        else if (rate > MHZ(10)) speed = 5;
        else if (rate > MHZ(5))  speed = 4;
        else if (rate > KHZ(400))speed = 3;
        else if (rate > KHZ(144))speed = 2;
        else speed = 1;
        clk = 67;
        break;

#undef MHZ
#undef KHZ

        // both none
    case SDC1_PCLK:
    case SDC2_PCLK:
        return 0;
        break;

    default:
        return -1;  
    }

#ifdef ENABLE_CLOCK_INFO
	printk("clk_rate %d : %d\n", clk, speed);
#endif
    msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_SPEED, &clk, &speed);
    return 0;
}

static int new_clk_enable(uint32_t id)
{
    unsigned clk = -1;
    switch (id)
    {
    case ICODEC_RX_CLK:
        clk = 50;
        break;
    case ICODEC_TX_CLK:
        clk = 52;
        break;
    case ECODEC_CLK:
        clk = 42;
        break;   
    case SDAC_MCLK:
        clk = 64;
        break;
    case IMEM_CLK:
        clk = 55;
        break;
    case GRP_CLK:
        clk = 56;
        break;
    case ADM_CLK:
        clk = 19;
        break;

    case UART1DM_CLK:
        clk = 78;
        break;
    case UART2DM_CLK:
        clk = 80;
        break;

    case VFE_AXI_CLK:
        clk = 24;
        break;
    case VFE_MDC_CLK:
        clk = 40;
        break;
    case VFE_CLK:
        clk = 41;
        break;
    case MDC_CLK:
        clk = 53; // ??
        break;

	case SPI_CLK:
		clk = 95;
		break;

    case MDP_CLK:
        clk = 9;
        break;

    case SDC1_CLK:
        clk = 66;
        break;
    case SDC2_CLK:
        clk = 67;
        break;
    case SDC1_PCLK:
        clk = 17;
        break;
    case SDC2_PCLK:
        clk = 16;
        break;

    default:
        return -1;  
    }
        
#ifdef ENABLE_CLOCK_INFO
	printk("clk_on %d\n", clk);
#endif
    msm_proc_comm(PCOM_CLK_REGIME_SEC_ENABLE, &clk, 0);
    return 0;
}

static int new_clk_disable(uint32_t id)
{
    unsigned clk = -1;
    switch (id)
    {
    case ICODEC_RX_CLK:
        clk = 50;
        break;
    case ICODEC_TX_CLK:
        clk = 52;
        break;
    case ECODEC_CLK:
        clk = 42;
        break;   
    case SDAC_MCLK:
        clk = 64;
        break;
    case IMEM_CLK:
        clk = 55;
        break;
    case GRP_CLK:
        clk = 56;
        break;
    case ADM_CLK:
        clk = 19;
        break;

    case UART1DM_CLK:
        clk = 78;
        break;
    case UART2DM_CLK:
        clk = 80;
        break;

    case VFE_AXI_CLK:
        clk = 24;
        break;
    case VFE_MDC_CLK:
        clk = 40;
        break;
    case VFE_CLK:
        clk = 41;
        break;
    case MDC_CLK:
        clk = 53; // WTF??
        break;

    case SPI_CLK:
        clk = 95;
        break;

    case MDP_CLK:
        clk = 9;
        break;

    case SDC1_CLK:
        clk = 66;
        break;
    case SDC2_CLK:
        clk = 67;
        break;
    case SDC1_PCLK:
        clk = 17;
        break;
    case SDC2_PCLK:
        clk = 16;
        break;

    default:
        return -1;  
    }

#ifdef ENABLE_CLOCK_INFO
	printk("clk_off %d\n", clk);
#endif
    msm_proc_comm(PCOM_CLK_REGIME_SEC_DISABLE, &clk, 0);
    return 0;
}

static long new_clk_get_rate(uint32_t id)
{
    unsigned clk = -1;
    unsigned rate;
    switch (id)
    {
    case ICODEC_RX_CLK:
        clk = 50;
        break;
    case ICODEC_TX_CLK:
        clk = 52;
        break;
    case ECODEC_CLK:
        clk = 42;
        break;
    case SDAC_MCLK:
        clk = 64;
        break;
    case IMEM_CLK:
        clk = 55;
        break;
    case GRP_CLK:
        clk = 56;
        break;
    case ADM_CLK:
        clk = 19;
        break;

    case UART1DM_CLK:
        clk = 78;
        break;
    case UART2DM_CLK:
        clk = 80;
        break;

    case VFE_AXI_CLK:
        clk = 24;
        break;
    case VFE_MDC_CLK:
        clk = 40;
        break;
    case VFE_CLK:
        clk = 41;
        break;
    case MDC_CLK:
        clk = 53; // ??
        break;

    case SPI_CLK:
        clk = 95;
        break;

    case MDP_CLK:
        clk = 9;
        break;

    case SDC1_CLK:
        clk = 66;
        break;
    case SDC2_CLK:
        clk = 67;
        break;
    case SDC1_PCLK:
        clk = 17;
        break;
    case SDC2_PCLK:
        clk = 16;
        break;

    default:
        return 0;
    }

    msm_proc_comm(PCOM_CLK_REGIME_SEC_MSM_GET_CLK_FREQ_KHZ, &clk, &rate);
    return clk*1000;
}

static int new_clk_set_flags(uint32_t id, unsigned long flags)
{    
    if (id == VFE_CLK)
    {        
        // INTERNAL 0x00000100 << 1        
        if (flags == (0x00000100 << 1))
        {        
            uint32_t f = 0; 
            msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_VFE_SRC, &f, 0);    
#ifdef ENABLE_CLOCK_INFO
            printk("internal VFE source\n");
#endif
            return 0;
        }
        // EXTERNAL 0x00000100
        else if (flags == 0x00000100)
        {
            uint32_t f = 1; 
            msm_proc_comm(PCOM_CLK_REGIME_SEC_SEL_VFE_SRC, &f, 0);    
#ifdef ENABLE_CLOCK_INFO
            printk("external VFE source\n");
#endif
            return 0;
        }        
    }
    return -1;
}


//////////////////////////////////////////////////////////

static int pc_clk_enable(uint32_t id)
{
	struct msm_clock_params params;
	int r;

	r = new_clk_enable(id);
	if (r != -1) return r;

	params = msm_clk_get_params(id);

	//XXX: too spammy, extreme debugging only: D(KERN_DEBUG "%s: %d\n", __func__, id);
	
	if ( id == IMEM_CLK || id == GRP_CLK )
	{
		set_grp_clk( 1 );
		writel(readl(params.glbl) | (1U << params.idx), params.glbl);
		return 0;
	}

	if (params.idx > 0 && params.glbl > 0)
	{
		writel(readl(params.glbl) | (1U << params.idx), params.glbl);
		return 0;
	} else if (params.ns_only > 0 && params.offset)
	{
		writel((readl(MSM_CLK_CTL_BASE + params.offset) &0xfffff000) | params.ns_only, MSM_CLK_CTL_BASE + params.offset);
		return 0;
	}
	if(debug_mask&DEBUG_UNKNOWN_ID)
		printk(KERN_WARNING "%s: FIXME! enabling a clock that doesn't have an ena bit "
		       "or ns-only offset: %u\n", __func__, id);

	return 0;
}

static void pc_clk_disable(uint32_t id)
{
	struct msm_clock_params params;
	int r;
	params = msm_clk_get_params(id);

    r = new_clk_disable(id);
    if (r != -1) return;

	//XXX: D(KERN_DEBUG "%s: %d\n", __func__, id);
	
	if ( id == IMEM_CLK || id == GRP_CLK )
	{
		set_grp_clk( 1 );
		writel(readl(params.glbl) & ~(1U << params.idx), params.glbl);
		return;
	}

	if (params.idx > 0 && params.glbl > 0)
	{
		writel(readl(params.glbl) & ~(1U << params.idx), params.glbl);
	} else if (params.ns_only > 0 && params.offset)
	{
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0xfffff000, MSM_CLK_CTL_BASE + params.offset);
	} else {
		if(debug_mask&DEBUG_UNKNOWN_ID)
			printk(KERN_WARNING "%s: FIXME! disabling a clock that doesn't have an "
			       "ena bit: %u\n", __func__, id);
	}
}

static int pc_clk_set_rate(uint32_t id, unsigned long rate)
{
	int retval, r;

	retval = 0;

    r = new_clk_set_rate(id, rate);
	if (r != -1) return r;

	if(DEBUG_MDNS)
		D("%s: id=%u rate=%lu\n", __func__, id, rate);

	retval = set_mdns_host_clock(id, rate);

	return retval;
}

static int pc_clk_set_min_rate(uint32_t id, unsigned long rate)
{
	if (id < NR_CLKS)
	 min_clk_rate[id]=rate;
	else if(debug_mask&DEBUG_UNKNOWN_ID)
	 printk(KERN_WARNING " FIXME! clk_set_min_rate not implemented; %u:%lu NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static int pc_clk_set_max_rate(uint32_t id, unsigned long rate)
{
	if (id < NR_CLKS)
	 max_clk_rate[id]=rate;
	else if(debug_mask&DEBUG_UNKNOWN_ID)
	 printk(KERN_WARNING " FIXME! clk_set_min_rate not implemented; %u:%lu NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static unsigned long pc_clk_get_rate(uint32_t id)
{
	unsigned long rate = 0;

	rate = new_clk_get_rate(id);

	if(rate == 0) {
		switch (id) {
		/* known MD/NS clocks, MSM_CLK dump and arm/mach-msm/clock-7x30.c */
		case SDC1_CLK:
		case SDC2_CLK:
		case SDC3_CLK:
		case SDC4_CLK:
		case UART1DM_CLK:
		case UART2DM_CLK:
		case USB_HS_CLK:
		case SDAC_CLK:
		case TV_DAC_CLK:
		case TV_ENC_CLK:
		case USB_OTG_CLK:
			rate = get_mdns_host_clock(id);
			break;

		case SDC1_PCLK:
		case SDC2_PCLK:
		case SDC3_PCLK:
		case SDC4_PCLK:
			rate = 64000000; /* g1 value */
			break;

		default:
			//TODO: support all clocks
			if(debug_mask&DEBUG_UNKNOWN_ID)
				printk("%s: unknown clock: id=%u\n", __func__, id);
			rate = 0;
		}
	}

	return rate;
}

static int pc_clk_set_flags(uint32_t id, unsigned long flags)
{
	int r;

    r = new_clk_set_flags(id, flags);
	if (r != -1) return r;

	if(debug_mask&DEBUG_UNKNOWN_CMD)
		printk(KERN_WARNING "%s not implemented for clock: id=%u, flags=%lu\n", __func__, id, flags);
	return 0;
}

static int pc_clk_is_enabled(uint32_t id)
{
	int is_enabled = 0;
	unsigned bit;
	uint32_t glbl;
	glbl = msm_clk_get_glbl(id);
	bit = msm_clk_enable_bit(id);
	if (bit > 0 && glbl>0)
	{
		is_enabled = (readl(glbl) & bit) != 0;
	}
	//XXX: is this necessary?
	if (id==SDC1_PCLK || id==SDC2_PCLK || id==SDC3_PCLK || id==SDC4_PCLK)
		is_enabled = 1;
	return is_enabled;
}

long pc_clk_round_rate(unsigned id, unsigned rate)
{

	/* Not really supported; pc_clk_set_rate() does rounding on it's own. */
	return rate;
}

static int pc_pll_request(unsigned id, unsigned on)
{
	if(debug_mask&DEBUG_UNKNOWN_CMD)
		printk(KERN_WARNING "%s not implemented for PLL=%u\n", __func__, id);

	return 0;
}

/*
 * Standard clock functions defined in include/linux/clk.h
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == dev)
			goto found_it;

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == NULL)
			goto found_it;

	clk = ERR_PTR(-ENOENT);
found_it:
	mutex_unlock(&clocks_mutex);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	if (clk->id == ACPU_CLK)
	{
		return -ENOTSUPP;
	}
	spin_lock_irqsave(&clocks_lock, flags);
	clk->count++;
	if (clk->count == 1)
		pc_clk_enable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	BUG_ON(clk->count == 0);
	clk->count--;
	if (clk->count == 0)
		pc_clk_disable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

int pc_clk_reset(unsigned id, enum clk_reset_action action)
{
         int rc;
 
        if (action == CLK_RESET_ASSERT)
                 rc = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_ASSERT, &id, NULL);
        else
                 rc = msm_proc_comm(PCOM_CLKCTL_RPC_RESET_DEASSERT, &id, NULL);
 
         if (rc < 0)
                 return rc;
         else
                return (int)id < 0 ? -EINVAL : 0;
}

int clk_reset(struct clk *clk, enum clk_reset_action action)
{
	if (!clk->ops->reset)
		clk->ops->reset = &pc_clk_reset;
	return clk->ops->reset(clk->remote_id, action);
}
EXPORT_SYMBOL(clk_reset);

unsigned long clk_get_rate(struct clk *clk)
{
	return pc_clk_get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	if (clk->flags & CLKFLAG_USE_MAX_TO_SET) {
		ret = pc_clk_set_max_rate(clk->id, rate);
		if (ret)
			return ret;
	}
	if (clk->flags & CLKFLAG_USE_MIN_TO_SET) {
		ret = pc_clk_set_min_rate(clk->id, rate);
		if (ret)
			return ret;
	}

	if (clk->flags & CLKFLAG_USE_MAX_TO_SET ||
		clk->flags & CLKFLAG_USE_MIN_TO_SET)
		return ret;

	return pc_clk_set_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->round_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	return pc_clk_set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);


void __init msm_clock_init(void)
{
	struct clk *clk;

	spin_lock_init(&clocks_lock);
	mutex_lock(&clocks_mutex);
	for (clk = msm_clocks; clk && clk->name; clk++) {
		list_add_tail(&clk->list, &clocks);
	}
	mutex_unlock(&clocks_mutex);
}

void clk_enter_sleep(int from_idle)
{
}

void clk_exit_sleep(void)
{
}

int clks_print_running(void)
{
	struct clk *clk;
	int clk_on_count = 0;
	char buf[100];
	char *pbuf = buf;
	int size = sizeof(buf);
	int wr;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);

	list_for_each_entry(clk, &clocks, list) {
		if (clk->count) {
			clk_on_count++;
			wr = snprintf(pbuf, size, " %s", clk->name);
			if (wr >= size)
				break;
			pbuf += wr;
			size -= wr;
		}
	}
	if (clk_on_count)
		pr_info("clocks on:%s\n", buf);

	spin_unlock_irqrestore(&clocks_lock, flags);
	return !clk_on_count;
}
EXPORT_SYMBOL(clks_print_running);

int clks_allow_tcxo_locked(void)
{
	struct clk *clk;
	struct hlist_node *pos;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	list_for_each_entry(clk, &clocks, list) {
		if (clk->count)
			return 0;
	}

	spin_unlock_irqrestore(&clocks_lock, flags);
	return 1;
}
EXPORT_SYMBOL(clks_allow_tcxo_locked);

int clks_allow_tcxo_locked_debug(void)
{
	struct clk *clk;
	int clk_on_count = 0;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);

	list_for_each_entry(clk, &clocks, list) {
		if (clk->count) {
			pr_info("%s: '%s' not off.\n", __func__, clk->name);
			clk_on_count++;
		}
	}
	pr_info("%s: %d clks are on.\n", __func__, clk_on_count);

	spin_unlock_irqrestore(&clocks_lock, flags);
	return !clk_on_count;
}
EXPORT_SYMBOL(clks_allow_tcxo_locked_debug);


/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	unsigned count = 0;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clocks_lock, flags);
			if (!clk->count && clk->id != MDP_CLK) {
				count++;
				pc_clk_disable(clk->id);
			}
			spin_unlock_irqrestore(&clocks_lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);

	// reset imem config, I guess all devices need this so somewhere here would be good.
	// it needs to be moved to somewhere else.
	//writel( 0, MSM_IMEM_BASE ); // IMEM addresses have to ve checked and enabled
	//pr_info("reset imem_config\n");
	return 0;
}
late_initcall(clock_late_init);

struct clk_ops clk_ops_pcom = {
	.enable = pc_clk_enable,
	.disable = pc_clk_disable,
	.auto_off = pc_clk_disable,
//	.reset = pc_clk_reset,
	.set_rate = pc_clk_set_rate,
	.set_min_rate = pc_clk_set_min_rate,
	.set_max_rate = pc_clk_set_max_rate,
	.set_flags = pc_clk_set_flags,
	.get_rate = pc_clk_get_rate,
	.is_enabled = pc_clk_is_enabled,
	.round_rate = pc_clk_round_rate,
};
