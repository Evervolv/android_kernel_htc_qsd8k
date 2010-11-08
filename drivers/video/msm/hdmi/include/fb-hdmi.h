#ifndef _FB_HDMI_H_
#define _FB_HDMI_H_

#include <linux/device.h>
#include <linux/fb.h>

enum hd_res {
	hd_720p = 0,	/* 1280 * 720 */
	svga,		/* 800 * 600 */
	pal,		/* 720 * 576 */
	edtv,		/* 720 * 480 */
	vga,		/* 640 * 480 */
};

struct msm_lcdc_timing;
struct hdmi_device {
	struct device dev;
	int (*check_res)(struct hdmi_device *, struct fb_var_screeninfo *);
	struct msm_lcdc_timing *(*set_res)(struct hdmi_device *,
					struct fb_var_screeninfo *);
	int (*get_cable_state)(struct hdmi_device *, int *);
	int (*get_establish_timing)(struct hdmi_device *, u8 *);
};

struct class_interface;
int register_hdmi_client(struct class_interface *class_intf);
#endif
