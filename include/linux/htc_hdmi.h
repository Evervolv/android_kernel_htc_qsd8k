/* include/linux/htc_hdmi.h
 *
 * Copyright (c) 2010 HTC
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

#ifndef _HTC_HDMI_H_
#define _HTC_HDMI_H_

enum {
        hdmi_mode = 0, /* 0: mirror, 1: presentation */
        hdmi_enabled,
        fb_enabled,
};

#define HDMI_IOCTL_MAGIC	'h'
#define HDMI_SET_MODE		_IOW(HDMI_IOCTL_MAGIC, 1, unsigned)
#define HDMI_GET_MODE		_IOR(HDMI_IOCTL_MAGIC, 2, unsigned)
#define HDMI_DISABLE		_IOW(HDMI_IOCTL_MAGIC, 3, unsigned)
#define HDMI_ENABLE		_IOW(HDMI_IOCTL_MAGIC, 4, unsigned)
#define HDMI_GET_STATE		_IOR(HDMI_IOCTL_MAGIC, 5, unsigned)
#define HDMI_BLIT		_IOW(HDMI_IOCTL_MAGIC, 6, unsigned)
#define HDMI_CABLE_STAT		_IOR(HDMI_IOCTL_MAGIC, 7, unsigned)
#define HDMI_ESTABLISH_TIMING	_IOR(HDMI_IOCTL_MAGIC, 8, unsigned)
#define HDMI_GET_EDID		_IOR(HDMI_IOCTL_MAGIC, 9, unsigned)
#define HDMI_GET_DISPLAY_INFO	_IOR(HDMI_IOCTL_MAGIC, 10, unsigned)

#define HDMI_GET_MIRRORING      _IOR(HDMI_IOCTL_MAGIC, 30, unsigned)
#define HDMI_SET_MIRRORING      _IOW(HDMI_IOCTL_MAGIC, 31, unsigned)
#define HDMI_GET_STATISTICS     _IOR(HDMI_IOCTL_MAGIC, 32, unsigned)
#define HDMI_CLEAR_STATISTICS   _IOW(HDMI_IOCTL_MAGIC, 33, unsigned)
#define HDMI_GET_VSYNC_MODE     _IOR(HDMI_IOCTL_MAGIC, 34, unsigned)
#define HDMI_SET_VSYNC_MODE     _IOW(HDMI_IOCTL_MAGIC, 35, unsigned)

#define ASPECT(w, h)            (w << 8 | h)
struct video_mode {
        unsigned short  width, height, refresh_rate, aspect;
        bool            interlaced, supported;
        char            *descrption;
};

enum {
        PROGRESSIVE,
        INTERLACE,
};

struct display_info {
    unsigned int    visible_width;      /* in mm */
    unsigned int    visible_height; 
    unsigned int    resolution_width;   /* in pixel */
    unsigned int    resolution_height;
};

/* Gathered statistics for mirroring */
struct mirror_statistics {
    unsigned int    frames;                 /* Number of panel frames requested */
    unsigned int    underflows;             /* Number of times we underflowed the LCDC */
    s64         statisticsTime;             /* Mirror time, in ns */
};

/* Panel state while mirroring */
enum {
    VSYNC_ALL = 0, 
    VSYNC_PANEL_ONLY, 
    VSYNC_HDMI_ONLY, 
    VSYNC_NONE
};

#endif

