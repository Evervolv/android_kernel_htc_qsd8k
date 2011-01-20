/*
 * Copyright (C) 2010 Google, Inc.
 * Author: Dima Zavin <dima@android.com>
 *
 * Based on code from Code Aurora Forum.
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

#include <linux/clk.h>
#include <linux/io.h>

#include "mdp_hw.h"

static void mdp_dma_to_mddi(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
	struct mdp_info *mdp = priv;
	uint32_t dma2_cfg;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */
	uint32_t fmt, pattern;

	/* XXX: HACK! hardcode to do mddi on primary */
	mdp_writel(mdp, 0x2, MDP_DISP_INTF_SEL);

	/* configure source, base layer */
	mdp_writel(mdp, ((height << 16) | (width)), MDP_PIPE_RGB_SRC_SIZE(0));
	mdp_writel(mdp, 0, MDP_PIPE_RGB_SRC_XY(0));
	mdp_writel(mdp, addr, MDP_PIPE_RGB_SRC_ADDR(0));
	mdp_writel(mdp, stride, MDP_PIPE_RGB_SRC_Y_STRIDE(0));

	switch (mdp->dma_format) {
	case DMA_IBUF_FORMAT_XRGB8888:
		fmt = PPP_CFG_MDP_XRGB_8888(SRC);
		pattern = PPP_PACK_PATTERN_MDP_BGRA_8888;
		break;
	case DMA_IBUF_FORMAT_RGB565:
		fmt = PPP_CFG_MDP_RGB_565(SRC);
		pattern = PPP_PACK_PATTERN_MDP_RGB_565;
		break;
	default:
		BUG();
		break;
	}

	mdp_writel(mdp, fmt, MDP_PIPE_RGB_SRC_FORMAT(0));
	mdp_writel(mdp, pattern, MDP_PIPE_RGB_SRC_UNPACK_PATTERN(0));

	/* configure destination */
	/* setup size, address, and stride in the overlay engine */
	mdp_writel(mdp, (height << 16) | (width), MDP_OVERLAYPROC_OUT_SIZE(0));
	mdp_writel(mdp, addr, MDP_OVERLAYPROC_FB_ADDR(0));
	mdp_writel(mdp, stride, MDP_OVERLAYPROC_FB_Y_STRIDE(0));

	/* output i/f config is in dma_p */
	dma2_cfg = DMA_PACK_ALIGN_LSB;

	dma2_cfg |= mdp->dma_format;
	dma2_cfg |= mdp->dma_pack_pattern;
	dma2_cfg |= DMA_DITHER_EN;

	/* 666 18BPP */
	dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_DMA_P_OUT_XY);
	mdp_writel(mdp, ld_param, MDP_MDDI_PARAM_WR_SEL);
	mdp_writel(mdp, (MDDI_VDO_PACKET_DESC << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_MDDI_PARAM);

	mdp_writel(mdp, 0x1, MDP_MDDI_DATA_XFR);
	mdp_writel(mdp, dma2_cfg, MDP_DMA_P_CONFIG);

	/* start the overlay fetch */
	mdp_writel(mdp, 0, MDP_OVERLAYPROC_START(0));
}

int mdp_hw_init(struct mdp_info *mdp)
{
	int ret;

	ret = mdp_out_if_register(&mdp->mdp_dev, MSM_MDDI_PMDH_INTERFACE, mdp,
				  MDP_DMA_P_DONE, mdp_dma_to_mddi);
	if (ret)
		return ret;

	mdp_writel(mdp, 0, MDP_INTR_ENABLE);
	mdp_writel(mdp, 0, MDP_DMA_P_HIST_INTR_ENABLE);
	mdp_writel(mdp, 0, MDP_LCDC_EN);
	mdp_writel(mdp, 0xffffffff, MDP_CGC_EN);

	/* XXX: why set this? QCT says it should be > mdp_pclk,
	 * but they never set the clkrate of pclk */
	clk_set_rate(mdp->clk, 122880000); /* 122.88 Mhz */
	pr_info("%s: mdp_clk=%lu\n", __func__, clk_get_rate(mdp->clk));

	/* this should work for any mdp_clk freq. 
	 * TODO: use different value for mdp_clk freqs >= 90Mhz */
	/* 8 bytes-burst x 8 req */
	mdp_writel(mdp, 0x27, MDP_DMA_P_FETCH_CFG);
	/* 16 bytes-burst x 4 req */
	/* TODO: do same for vg pipes */
	mdp_writel(mdp, 0xc3, MDP_PIPE_RGB_FETCH_CFG(0));
	mdp_writel(mdp, 0xc3, MDP_PIPE_RGB_FETCH_CFG(1));

	mdp_writel(mdp, 0x3, MDP_EBI2_PORTMAP_MODE);

	/* 3 pending requests */
	mdp_writel(mdp, 0x02222, MDP_MAX_RD_PENDING_CMD_CONFIG);

	/* RGB1 -> Layer 0 base */
	mdp_writel(mdp, 1 << 8, MDP_LAYERMIXER_IN_CFG);

	mdp_writel(mdp, 1, MDP_OVERLAYPROC_CFG(0));
	mdp_writel(mdp, 0, MDP_OVERLAYPROC_CFG(1));

	mdp_writel(mdp, 0, MDP_OVERLAYPROC_OPMODE(0));

	return 0;
}

