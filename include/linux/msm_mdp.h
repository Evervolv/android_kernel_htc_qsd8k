/* include/linux/msm_mdp.h
 *
 * Copyright (C) 2007 Google Incorporated
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
#ifndef _MSM_MDP_H_
#define _MSM_MDP_H_

#include <linux/types.h>

#define MSMFB_IOCTL_MAGIC 'm'
#define MSMFB_GRP_DISP          _IOW(MSMFB_IOCTL_MAGIC, 1, unsigned int)
#define MSMFB_BLIT              _IOW(MSMFB_IOCTL_MAGIC, 2, unsigned int)

/* drewis: adding for android 4.0 */
#define MSMFB_OVERLAY_SET       _IOWR(MSMFB_IOCTL_MAGIC, 135, \
						struct mdp_overlay)
#define MSMFB_OVERLAY_UNSET     _IOW(MSMFB_IOCTL_MAGIC, 136, unsigned int)
#define MSMFB_OVERLAY_PLAY      _IOW(MSMFB_IOCTL_MAGIC, 137, \
						struct msmfb_overlay_data)
#define MSMFB_OVERLAY_GET      _IOR(MSMFB_IOCTL_MAGIC, 140, \
						struct mdp_overlay)
#define MSMFB_OVERLAY_3D       _IOWR(MSMFB_IOCTL_MAGIC, 147, \
						struct msmfb_overlay_3d)
/* drewis: end */

#if 0 /* drewis: superseded below */
enum {
	MDP_RGB_565,		/* RGB 565 planar */
	MDP_XRGB_8888,		/* RGB 888 padded */
	MDP_Y_CBCR_H2V2,	/* Y and CbCr, pseudo planar w/ Cb is in MSB */
	MDP_ARGB_8888,		/* ARGB 888 */
	MDP_RGB_888,		/* RGB 888 planar */
	MDP_Y_CRCB_H2V2,	/* Y and CrCb, pseudo planar w/ Cr is in MSB */
	MDP_YCRYCB_H2V1,	/* YCrYCb interleave */
	MDP_Y_CRCB_H2V1,	/* Y and CrCb, pseduo planar w/ Cr is in MSB */
	MDP_Y_CBCR_H2V1,	/* Y and CrCb, pseduo planar w/ Cr is in MSB */
	MDP_RGBA_8888,		/* ARGB 888 */
	MDP_BGRA_8888,		/* ABGR 888 */
	MDP_RGBX_8888,		/* RGBX 888 */
	MDP_IMGTYPE_LIMIT	/* Non valid image type after this enum */
};
#endif

/* drewis: added for android 4.0 */
#define FB_TYPE_3D_PANEL 0x10101010
#define MDP_IMGTYPE2_START 0x10000
enum {
	MDP_RGB_565,      /* RGB 565 planer */
	MDP_XRGB_8888,    /* RGB 888 padded */
	MDP_Y_CBCR_H2V2,  /* Y and CbCr, pseudo planer w/ Cb is in MSB */
	MDP_ARGB_8888,    /* ARGB 888 */
	MDP_RGB_888,      /* RGB 888 planer */
	MDP_Y_CRCB_H2V2,  /* Y and CrCb, pseudo planer w/ Cr is in MSB */
	MDP_YCRYCB_H2V1,  /* YCrYCb interleave */
	MDP_Y_CRCB_H2V1,  /* Y and CrCb, pseduo planer w/ Cr is in MSB */
	MDP_Y_CBCR_H2V1,   /* Y and CrCb, pseduo planer w/ Cr is in MSB */
	MDP_RGBA_8888,    /* ARGB 888 */
	MDP_BGRA_8888,	  /* ABGR 888 */
	MDP_RGBX_8888,	  /* RGBX 888 */
	MDP_Y_CRCB_H2V2_TILE,  /* Y and CrCb, pseudo planer tile */
	MDP_Y_CBCR_H2V2_TILE,  /* Y and CbCr, pseudo planer tile */
	MDP_Y_CR_CB_H2V2,  /* Y, Cr and Cb, planar */
	MDP_Y_CR_CB_GH2V2,  /* Y, Cr and Cb, planar aligned to Android YV12 */
	MDP_Y_CB_CR_H2V2,  /* Y, Cb and Cr, planar */
	MDP_Y_CRCB_H1V1,  /* Y and CrCb, pseduo planer w/ Cr is in MSB */
	MDP_Y_CBCR_H1V1,  /* Y and CbCr, pseduo planer w/ Cb is in MSB */
	MDP_IMGTYPE_LIMIT,
	MDP_BGR_565 = MDP_IMGTYPE2_START,      /* BGR 565 planer */
	MDP_FB_FORMAT,    /* framebuffer format */
	MDP_IMGTYPE_LIMIT2 /* Non valid image type after this enum */
};
/* drewis: end */

enum {
	PMEM_IMG,
	FB_IMG,
};

/* flag values */
#define MDP_ROT_NOP	0
#define MDP_FLIP_LR	0x1
#define MDP_FLIP_UD	0x2
#define MDP_ROT_90	0x4
#define MDP_ROT_180	(MDP_FLIP_UD|MDP_FLIP_LR)
#define MDP_ROT_270	(MDP_ROT_90|MDP_FLIP_UD|MDP_FLIP_LR)
#define MDP_ROT_MASK	0x7
#define MDP_DITHER	0x8
#define MDP_BLUR	0x10
#define MDP_BLEND_FG_PREMULT 0x20000

#define MDP_TRANSP_NOP	0xffffffff
#define MDP_ALPHA_NOP	0xff

/* drewis: added for android 4.0 */
#define MDP_DEINTERLACE 0x80000000
#define MDP_BLIT_NON_CACHED		0x01000000
#define MDP_OV_PIPE_SHARE		0x00800000
#define MDP_OV_PLAY_NOWAIT		0x00200000
#define MDP_SOURCE_ROTATED_90		0x00100000
/* drewis: end */

struct mdp_rect {
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
};

struct mdp_img {
	uint32_t width;
	uint32_t height;
	uint32_t format;
	uint32_t offset;
	int memory_id;		/* the file descriptor */
};

struct mdp_blit_req {
	struct mdp_img src;
	struct mdp_img dst;
	struct mdp_rect src_rect;
	struct mdp_rect dst_rect;
	uint32_t alpha;
	uint32_t transp_mask;
	uint32_t flags;
};

struct mdp_blit_req_list {
	uint32_t count;
	struct mdp_blit_req req[];
};

/* drewis: added for android 4.0 */
struct msmfb_data {
	uint32_t offset;
	int memory_id;
	int id;
	uint32_t flags;
	uint32_t priv;
};

#define MSMFB_NEW_REQUEST -1

struct msmfb_overlay_data {
	uint32_t id;
	struct msmfb_data data;
	uint32_t version_key;
	struct msmfb_data plane1_data;
	struct msmfb_data plane2_data;
};

struct msmfb_img {
	uint32_t width;
	uint32_t height;
	uint32_t format;
};

struct dpp_ctrl {
	/*
	 *'sharp_strength' has inputs = -128 <-> 127
	 *  Increasingly positive values correlate with increasingly sharper
	 *  picture. Increasingly negative values correlate with increasingly
	 *  smoothed picture.
	 */
	int8_t sharp_strength;
};

struct mdp_overlay {
	struct msmfb_img src;
	struct mdp_rect src_rect;
	struct mdp_rect dst_rect;
	uint32_t z_order;	/* stage number */
	uint32_t is_fg;		/* control alpha & transp */
	uint32_t alpha;
	uint32_t transp_mask;
	uint32_t flags;
	uint32_t id;
	uint32_t user_data[8];
	struct dpp_ctrl dpp;
};

struct msmfb_overlay_3d {
	uint32_t is_3d;
	uint32_t width;
	uint32_t height;
};
/* drewis: end */

#endif /* _MSM_MDP_H_ */
