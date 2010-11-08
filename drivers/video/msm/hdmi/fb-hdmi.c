/*
 * Copyright (C) 2009 HTC
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
 * Referenced from drivers/video/msm/msm_fb.c, Google Incorporated.
 *
 */
#define DEBUG
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/msm_mdp.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/msm_fb.h>
#include <mach/board.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/htc_hdmi.h>
#ifdef CONFIG_HTC_HEADSET_MGR
#include <mach/htc_headset_mgr.h>
#endif

#include "include/fb-hdmi.h"
#include "include/sil902x.h"

#if 1
#define HDMI_DBG(s...) printk("[hdmi/fb]" s)
#else
#define HDMI_DBG(s...) do {} while (0)
#endif

struct update_info_t {
	int left;
	int top;
	int eright; /* exclusive */
	int ebottom; /* exclusive */
	unsigned yoffset;
};

struct hdmifb_info {
	struct fb_info *fb;
	struct msm_panel_data *panel;
	struct notifier_block fb_hdmi_event;
	struct msmfb_callback dma_callback;
	struct msmfb_callback vsync_callback;
	struct update_info_t update_info;
	struct early_suspend earlier_suspend;
	struct early_suspend early_suspend;
	spinlock_t update_lock;
	int xres;
	int yres;
	unsigned long state;
	atomic_t use_count;
};

static struct mdp_device *mdp;
static struct hdmi_device *hdmi;

static unsigned PP[16];

void hdmi_pre_change(struct hdmi_info *hdmi);
void hdmi_post_change(struct hdmi_info *info, struct fb_var_screeninfo *var);

static int hdmifb_open(struct fb_info *info, int user)
{
	return 0;
}

static int hdmifb_release(struct fb_info *info, int user)
{
	return 0;
}

static int hdmifb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 size;

	if (mdp->check_output_format(mdp, var->bits_per_pixel))
		return -EINVAL;
	if (hdmi->check_res(hdmi, var))
		return -EINVAL;

	size = var->xres_virtual * var->yres_virtual *
		(var->bits_per_pixel >> 3);
	if (size > info->fix.smem_len)
		return -EINVAL;
	return 0;
}

static int hdmifb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct hdmifb_info *hdmi_fb = info->par;
	struct msm_panel_data *panel = hdmi_fb->panel;
	struct msm_lcdc_timing *timing;
        struct hdmi_info *hinfo = container_of(hdmi, struct hdmi_info,
                                        hdmi_dev);

	HDMI_DBG("%s\n", __func__);
	/* we only support RGB ordering for now */
	if (var->bits_per_pixel == 32 || var->bits_per_pixel == 24) {
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
	} else if (var->bits_per_pixel == 16) {
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
	} else
		return -1;

	HDMI_DBG("set res (%d, %d)\n", var->xres, var->yres);
	timing = hdmi->set_res(hdmi, var);
	panel->adjust_timing(panel, timing, var->xres, var->yres);
	hdmi_post_change(hinfo, var);

	mdp->set_output_format(mdp, var->bits_per_pixel);

	hdmi_fb->xres = var->xres;
	hdmi_fb->yres = var->yres;
	fix->line_length = var->xres * var->bits_per_pixel / 8;
	return 0;
}

/* core update function */
static void
hdmifb_pan_update(struct fb_info *info, uint32_t left, uint32_t top,
		uint32_t eright, uint32_t ebottom, uint32_t yoffset)
{
	struct hdmifb_info *hdmi_fb = info->par;
	struct msm_panel_data *panel = hdmi_fb->panel;
	unsigned long irq_flags;

	/* printk(KERN_DEBUG "%s\n", __func__); */
	if ((test_bit(fb_enabled, &hdmi_fb->state) == 0) ||
	    (test_bit(hdmi_enabled, &hdmi_fb->state) == 0))
		return;

	spin_lock_irqsave(&hdmi_fb->update_lock, irq_flags);
	hdmi_fb->update_info.left = left;
	hdmi_fb->update_info.top = top;
	hdmi_fb->update_info.eright = eright;
	hdmi_fb->update_info.ebottom = ebottom;
	hdmi_fb->update_info.yoffset = yoffset;
	spin_unlock_irqrestore(&hdmi_fb->update_lock, irq_flags);
	panel->request_vsync(panel, &hdmi_fb->vsync_callback);
}

/* fb ops, fb_pan_display */
static int
hdmifb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* full update */
	hdmifb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
			var->yoffset);
	return 0;
}

static void hdmifb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	hdmifb_pan_update(p, rect->dx, rect->dy, rect->dx + rect->width,
			rect->dy + rect->height, 0);
}

static void hdmifb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	hdmifb_pan_update(p, area->dx, area->dy, area->dx + area->width,
			area->dy + area->height, 0);
}

static void hdmifb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	hdmifb_pan_update(p, image->dx, image->dy, image->dx + image->width,
			image->dy + image->height, 0);
}

static int hdmifb_change_mode(struct fb_info *info, unsigned int mode)
{
	struct hdmifb_info *hdmi_fb = info->par;

	/* printk(KERN_DEBUG "%s mode = %d\n", __func__, mode); */

	if (mode)
		set_bit(hdmi_mode, &hdmi_fb->state);
	else
		clear_bit(hdmi_mode, &hdmi_fb->state);
	return 0;
}

struct hdmifb_info *_hdmi_fb;
int hdmifb_get_mode(void)
{
        return test_bit(hdmi_mode, &_hdmi_fb->state);
}

bool hdmifb_suspending = false;

static int hdmifb_pause(struct fb_info *fb, unsigned int mode)
{
	int ret = 0;
	struct hdmifb_info *hdmi_fb = fb->par;
	struct msm_panel_data *panel = hdmi_fb->panel;
	struct hdmi_info *info = container_of(hdmi, struct hdmi_info,
			hdmi_dev);

	pr_info("%s: %d %s\n", __func__, atomic_read(&hdmi_fb->use_count),
		mode == 1 ? "pause" : "resume");

	if (mode == 1) {
		hdmifb_suspending = false;
		HDMI_DBG("%s: hdmifb_suspending = false\n", __func__);
		/* pause */
		if (atomic_read(&hdmi_fb->use_count) == 0)
			goto done;
		if (atomic_dec_return(&hdmi_fb->use_count) == 0) {
			hdmi_pre_change(info);
			ret = panel->blank(panel);
			clear_bit(hdmi_enabled, &hdmi_fb->state);
#ifdef CONFIG_HTC_HEADSET_MGR
			switch_send_event(BIT_HDMI_AUDIO, 0);
#endif
		}
	} else if (mode == 0) {
		/* resume */
		if (atomic_inc_return(&hdmi_fb->use_count) == 1) {
			hdmi_pre_change(info);
			ret = panel->unblank(panel);
/*
			// set timing again to prevent TV been out of range
			var = &fb->var;
			timing = hdmi->set_res(hdmi, var);
			panel->adjust_timing(panel, timing, var->xres, var->yres);
			hdmi_post_change(info, var);
*/
			set_bit(hdmi_enabled, &hdmi_fb->state);
#ifdef CONFIG_HTC_HEADSET_MGR
			switch_send_event(BIT_HDMI_AUDIO, 1);
#endif
		}
	} else
		ret = -EINVAL;
done:
	return ret;
}

static int hdmifb_blit(struct fb_info *info, void __user *p)
{
	struct mdp_blit_req req;
	struct mdp_blit_req_list req_list;
	int i;
	int ret;

	if (copy_from_user(&req_list, p, sizeof(req_list)))
		return -EFAULT;

	for (i = 0; i < req_list.count; i++) {
		struct mdp_blit_req_list *list =
			(struct mdp_blit_req_list *)p;
		if (copy_from_user(&req, &list->req[i], sizeof(req)))
			return -EFAULT;
		req.flags |= MDP_DITHER;
		ret = mdp->blit(mdp, info, &req);
		if (ret)
			return ret;
	}
	return 0;
}

enum ioctl_cmd_index {
       CMD_SET_MODE,
       CMD_GET_MODE,
       CMD_DISABLE,
       CMD_ENABLE,
       CMD_GET_STATE,
       CMD_BLIT,
       CMD_CABLE_STAT,
       CMD_ESTABLISH_TIMING,
};

static char *cmd_str[] = {
       "HDMI_SET_MODE",
       "HDMI_GET_MODE",
       "HDMI_DISABLE",
       "HDMI_ENABLE",
       "HDMI_GET_STATE",
       "HDMI_BLIT",
       "HDMI_CABLE_STAT",
       "HDMI_ESTABLISH_TIMING",
};

static int hdmifb_ioctl(struct fb_info *p, unsigned int cmd, unsigned long arg)
{
	struct hdmifb_info *hdmi_fb = p->par;
	void __user *argp = (void __user *)arg;
	unsigned int val;
	int ret = -EINVAL;
        struct hdmi_info *hinfo = container_of(hdmi, struct hdmi_info,
                                        hdmi_dev);

/*
	if (cmd != HDMI_BLIT)
		HDMI_DBG("%s, cmd=%d=%s\n", __func__, cmd - HDMI_SET_MODE,
				cmd_str[cmd-HDMI_SET_MODE]);
*/

	switch (cmd) {
	case HDMI_SET_MODE:
		get_user(val, (unsigned __user *) arg);
		//pr_info("[hdmi] SET_MODE: %d\n", val);
		ret = hdmifb_change_mode(p, val);
		break;
	case HDMI_GET_MODE:
/*
		pr_info("[hdmi] GET_MODE: %d\n",
			test_bit(hdmi_mode, &hdmi_fb->state));
*/
		ret = put_user(test_bit(hdmi_mode, &hdmi_fb->state),
			(unsigned __user *) arg);
		break;
	case HDMI_DISABLE:
		get_user(val, (unsigned __user *) arg);
		ret = hdmifb_pause(p, 1);
		break;
	case HDMI_ENABLE:
		get_user(val, (unsigned __user *) arg);
		ret = hdmifb_pause(p, 0);
		break;
	case HDMI_GET_STATE:
		ret = put_user(test_bit(hdmi_enabled, &hdmi_fb->state),
			(unsigned __user *) arg);
		break;
	case HDMI_BLIT:
		if (test_bit(hdmi_enabled, &hdmi_fb->state))
			ret = hdmifb_blit(p, argp);
		else
			ret = -EPERM;
		break;
	case HDMI_CABLE_STAT: {
		int connect;
		ret = hdmi->get_cable_state(hdmi, &connect);
		ret = put_user(connect, (unsigned __user *) arg);
		break;
	}
	case HDMI_ESTABLISH_TIMING: {
		u8 tmp[3];
		hdmi->get_establish_timing(hdmi, tmp);
		ret = copy_to_user((unsigned __user *) arg, tmp, 3);
		if (ret)
			ret = -EFAULT;
		break;
	}
	case HDMI_GET_EDID:
		ret = copy_to_user((unsigned __user *) arg,
			hinfo->edid_buf, 512);
		break;
	case HDMI_GET_DISPLAY_INFO: {
		struct display_info dinfo;
		u8 *ptr = hinfo->edid_buf;
		dinfo.visible_width =
			(((u32)ptr[68] & 0xf0) << 4) | ptr[66];
		dinfo.visible_height =
			(((u32)ptr[68] & 0x0f) << 8) | ptr[67];
		dinfo.resolution_width =
			(((u32)ptr[58] & 0xf0) << 4) | ptr[56];
		dinfo.resolution_height =
			(((u32)ptr[61] & 0xf0) << 4) | ptr[59];
		ret = copy_to_user((unsigned __user *) arg,
			&dinfo, sizeof(dinfo));
		break;
	}
	default:
		printk(KERN_ERR "hdmi: unknown cmd, cmd = %d\n", cmd);
	}
	return ret;
}

static struct fb_ops hdmi_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = hdmifb_open,
	.fb_release = hdmifb_release,
	.fb_check_var = hdmifb_check_var,
	.fb_set_par = hdmifb_set_par,
	.fb_pan_display = hdmifb_pan_display,
	.fb_fillrect = hdmifb_fillrect,
	.fb_copyarea = hdmifb_copyarea,
	.fb_imageblit = hdmifb_imageblit,
	.fb_ioctl = hdmifb_ioctl,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hdmifb_suspend(struct early_suspend *h)
{
	struct hdmifb_info *hdmi_fb = container_of(h, struct hdmifb_info,
						early_suspend);
	struct msm_panel_data *panel = hdmi_fb->panel;

	HDMI_DBG("%s, use_count=%d\n", __func__,
		atomic_read(&hdmi_fb->use_count));
	hdmifb_suspending = true;
	HDMI_DBG("%s: hdmifb_suspending = true\n", __func__);
	if (atomic_read(&hdmi_fb->use_count) &&
		false == test_bit(hdmi_enabled, &hdmi_fb->state)
		) {
		if (panel->blank)
			panel->blank(panel);
	}

	if (panel->suspend)
		panel->suspend(panel);

	clear_bit(hdmi_enabled, &hdmi_fb->state);
	clear_bit(fb_enabled, &hdmi_fb->state);
}

static void hdmifb_resume(struct early_suspend *h)
{
	struct hdmifb_info *hdmi_fb = container_of(h, struct hdmifb_info,
						early_suspend);
	struct msm_panel_data *panel = hdmi_fb->panel;

	HDMI_DBG("%s\n", __func__);
	if (panel->resume)
		panel->resume(panel);

	atomic_set(&hdmi_fb->use_count, 0);
	set_bit(fb_enabled, &hdmi_fb->state);
}
#endif

#define BITS_PER_PIXEL 16

static void setup_fb_info(struct hdmifb_info *hdmi_fb)
{
	struct fb_info *fb_info = hdmi_fb->fb;
	int r;

	/* finish setting up the fb_info struct */
	strncpy(fb_info->fix.id, "hdmi_fb", 16);
	fb_info->fix.ypanstep = 1;

	fb_info->fbops = &hdmi_fb_ops;
	fb_info->flags = FBINFO_DEFAULT;

	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR;
	fb_info->fix.line_length = hdmi_fb->xres * 2;

	fb_info->var.xres = hdmi_fb->xres;
	fb_info->var.yres = hdmi_fb->yres;
	fb_info->var.width = hdmi_fb->panel->fb_data->width;
	fb_info->var.height = hdmi_fb->panel->fb_data->height;
	fb_info->var.xres_virtual = hdmi_fb->xres;
	fb_info->var.yres_virtual = hdmi_fb->yres * 2;
	fb_info->var.bits_per_pixel = BITS_PER_PIXEL;
	fb_info->var.accel_flags = 0;
	fb_info->var.yoffset = 0;

	fb_info->var.red.offset = 11;
	fb_info->var.red.length = 5;
	fb_info->var.red.msb_right = 0;
	fb_info->var.green.offset = 5;
	fb_info->var.green.length = 6;
	fb_info->var.green.msb_right = 0;
	fb_info->var.blue.offset = 0;
	fb_info->var.blue.length = 5;
	fb_info->var.blue.msb_right = 0;

	r = fb_alloc_cmap(&fb_info->cmap, 16, 0);
	fb_info->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;
}

static int
setup_fbmem(struct hdmifb_info *hdmi_fb, struct platform_device *pdev)
{
	struct fb_info *fb = hdmi_fb->fb;
	struct resource *res;
	unsigned long size = hdmi_fb->xres * hdmi_fb->yres *
			     (BITS_PER_PIXEL >> 3) * 2;
	unsigned char *fbram;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	/* check the resource is large enough to fit the fb */
	if (resource_size(res) < size) {
		printk(KERN_ERR "allocated resource(%d) is too small(%lu)"
				"for fb\n", resource_size(res), size);
		return -ENOMEM;
	}

	fb->fix.smem_start = res->start;
	fb->fix.smem_len = resource_size(res);

	fbram = ioremap(res->start, resource_size(res));
	if (fbram == 0) {
		printk(KERN_ERR "hdmi_fb: cannot allocate fbram!\n");
		return -ENOMEM;
	}

	fb->screen_base = fbram;
	memset(fbram, 0, resource_size(res));

	printk(KERN_DEBUG "HDMI FB: 0x%x 0x%x\n", res->start, res->end);
	return 0;
}

/* Called from dma interrupt handler, must not sleep */
static void hdmi_handle_dma(struct msmfb_callback *callback)
{
	/* printk(KERN_DEBUG "%s\n", __func__); */
}

/* Called from vsync interrupt handler, must not sleep */
static void hdmi_handle_vsync(struct msmfb_callback *callback)
{
	uint32_t x, y, w, h;
	unsigned yoffset;
	unsigned addr;
	unsigned long irq_flags;
	struct fb_info *mirror_fb = registered_fb[0], *fb_hdmi;
	struct hdmifb_info *hdmi = container_of(callback, struct hdmifb_info,
						vsync_callback);
	struct msm_panel_data *panel = hdmi->panel;

	spin_lock_irqsave(&hdmi->update_lock, irq_flags);
	x = hdmi->update_info.left;
	y = hdmi->update_info.top;
	w = hdmi->update_info.eright - x;
	h = hdmi->update_info.ebottom - y;
	yoffset = hdmi->update_info.yoffset;
	hdmi->update_info.left = hdmi->xres + 1;
	hdmi->update_info.top = hdmi->yres + 1;
	hdmi->update_info.eright = 0;
	hdmi->update_info.ebottom = 0;
	if (unlikely(w > hdmi->xres || h > hdmi->yres ||
		w == 0 || h == 0)) {
		printk(KERN_INFO "invalid update: %d %d %d "
				"%d\n", x, y, w, h);
		goto error;
	}
	spin_unlock_irqrestore(&hdmi->update_lock, irq_flags);

	addr = ((hdmi->xres * (yoffset + y) + x) * 2);
	if (test_bit(hdmi_mode, &hdmi->state) == 0) {
		mdp->dma(mdp, addr + mirror_fb->fix.smem_start,
			hdmi->xres * 2, w, h, x, y, &hdmi->dma_callback,
			panel->interface_type);
	} else {
		fb_hdmi = hdmi->fb;
		mdp->dma(mdp, addr + fb_hdmi->fix.smem_start,
			hdmi->xres * 2, w, h, x, y, &hdmi->dma_callback,
			panel->interface_type);
	}
	return;
error:
	spin_unlock_irqrestore(&hdmi->update_lock, irq_flags);
}

static int hdmifb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct hdmifb_info *hdmi_fb;
	struct msm_panel_data *panel = pdev->dev.platform_data;
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	if (!panel) {
		pr_err("hdmi_fb_probe: no platform data\n");
		return -EINVAL;
	}

	if (!panel->fb_data) {
		pr_err("hdmi_fb_probe: no fb_data\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct hdmifb_info), &pdev->dev);
	if (!info)
		return -ENOMEM;

	hdmi_fb = info->par;
	_hdmi_fb = hdmi_fb;
	hdmi_fb->fb = info;
	hdmi_fb->panel = panel;
	set_bit(hdmi_mode, &hdmi_fb->state);
	hdmi_fb->dma_callback.func = hdmi_handle_dma;
	hdmi_fb->vsync_callback.func = hdmi_handle_vsync;
	hdmi_fb->xres = panel->fb_data->xres;
	hdmi_fb->yres = panel->fb_data->yres;
	spin_lock_init(&hdmi_fb->update_lock);

	ret = setup_fbmem(hdmi_fb, pdev);
	if (ret)
		goto error_setup_fbmem;

	setup_fb_info(hdmi_fb);

	ret = register_framebuffer(info);
	if (ret)
		goto error_register_fb;

	printk(KERN_INFO "hdmi_fb %d * %d initialed\n",
			hdmi_fb->xres, hdmi_fb->yres);

#ifdef CONFIG_HAS_EARLYSUSPEND
	hdmi_fb->early_suspend.suspend = hdmifb_suspend;
	hdmi_fb->early_suspend.resume = hdmifb_resume;
	hdmi_fb->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&hdmi_fb->early_suspend);
#endif
	/* blank panel explicitly because we turn on clk on initial */
	if (panel->blank)
		panel->blank(panel);
	set_bit(fb_enabled, &hdmi_fb->state);
	return 0;

error_register_fb:
error_setup_fbmem:
	framebuffer_release(hdmi_fb->fb);
	printk(KERN_ERR "msm probe fail with %d\n", ret);
	return ret;
}

static struct platform_driver hdmi_frame_buffer = {
	.probe = hdmifb_probe,
	.driver = {.name = "msm_hdmi"},
};

static int hdmifb_add_mdp_device(struct device *dev,
				struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (mdp)
		return 0;
	mdp = container_of(dev, struct mdp_device, dev);
	return platform_driver_register(&hdmi_frame_buffer);
}

static void hdmifb_remove_mdp_device(struct device *dev,
				struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (dev != &mdp->dev)
		return;
	platform_driver_unregister(&hdmi_frame_buffer);
	mdp = NULL;
}

static struct class_interface hdmi_fb_interface = {
	.add_dev = &hdmifb_add_mdp_device,
	.remove_dev = &hdmifb_remove_mdp_device,
};

static int hdmifb_add_hdmi_device(struct device *dev,
				struct class_interface *class_intf)
{
	dev_dbg(dev, "%s\n", __func__);

	if (hdmi)
		return 0;
	hdmi = container_of(dev, struct hdmi_device, dev);
	return 0;
}

static struct class_interface hdmi_interface = {
	.add_dev = hdmifb_add_hdmi_device,
};

static int __init hdmifb_init(void)
{
	int rc;

	rc = register_mdp_client(&hdmi_fb_interface);
	if (rc)
		return rc;

	rc = register_hdmi_client(&hdmi_interface);
	if (rc)
		return rc;
	return 0;
}

module_init(hdmifb_init);
