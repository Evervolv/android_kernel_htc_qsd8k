/*
 * Copyright (C) 2009 HTC Corporation.
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <mach/msm_hdmi.h>
#include <mach/msm_fb.h>
#include "include/fb-hdmi.h"
#include "include/sil902x.h"

#ifdef CONFIG_HTC_HEADSET_MGR
#include <mach/htc_headset_mgr.h>
#endif

#if 1
#define HDMI_DBG(s...) printk("[hdmi/tx]" s)
#else
#define HDMI_DBG(s...) do {} while (0)
#endif

#define HDMI_NAME "SiL902x-hdmi"
//#define HDMI_DEBUGFS

static struct class *hdmi_class;

enum {
	ESTABLISHED_TIMING_OFFSET = 35,
	LONG_DESCR_LEN = 18,
	NUM_DETAILED_DESC = 4,
	NUM_STANDARD_TIMING = 8,
};

#if 1
int hdmi_read(struct i2c_client *client, u8 cmd)
#else
#define hdmi_read(client, cmd) _hdmi_read(client, cmd, __func__)
int _hdmi_read(struct i2c_client *client, u8 cmd, const char *caller)
#endif
{
	int ret = -EIO, retry = 10;

	while (retry--) {
		ret = i2c_smbus_read_byte_data(client, cmd);
		if (ret >= 0)
			break;
		msleep(1);
	}
/*
	if (retry!=9)
		HDMI_DBG("%s, retry=%d, caller=%s\n", __func__, 10-retry,
			caller);
*/

	return ret;
}

int tpi_readb(struct hdmi_info *hdmi, u8 reg)
{
	int i, ret = -EIO, retrial = 10, timeout = 1;
	struct i2c_client *client = hdmi->client;

	for (i = 1 ; i < retrial ; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret >= 0)
			break;
		msleep(timeout++);
	}
	return ret;
}

int tpi_readb_oneshoot(struct hdmi_info *hdmi, u8 reg)
{
	return i2c_smbus_read_byte_data(hdmi->client, reg);
}

#if 1
int hdmi_write_byte(struct i2c_client *client, u8 reg, u8 val)
#else
#define hdmi_write_byte(client, reg, val) \
	_hdmi_write_byte(client, reg, val, __func__)
int _hdmi_write_byte(struct i2c_client *client, u8 reg, u8 val, const char *caller)
#endif
{
	int ret = -EIO, retry = 10;

	while (retry--) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret == 0)
			break;
		msleep(1);
	}
/*
	if (retry!=9) HDMI_DBG("%s, retry=%d, caller=%s\n", __func__,
		10 - retry, caller);
*/

	return ret;
}

int tpi_writeb(struct hdmi_info *hdmi, u8 reg, u8 val)
{
	int i, ret = -EIO, retrial = 10, timeout = 1;
	struct i2c_client *client = hdmi->client;

	for (i = 1 ; i < retrial ; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret == 0)
			break;
		msleep(timeout);
	}
	return ret;
}

int tpi_writeb_oneshot(struct hdmi_info *hdmi, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(hdmi->client, reg, val);
}

int hdmi_enable_int(struct i2c_client *client)
{
	u8 data;

	HDMI_DBG("%s\n", __func__);
	data = hdmi_read(client, HDMI_INT_EN);
	return hdmi_write_byte(client, HDMI_INT_EN, data | 0x01);
}

int hdmi_disable_int(struct i2c_client *client)
{
	u8 data;

	HDMI_DBG("%s\n", __func__);
	data = hdmi_read(client, HDMI_INT_EN);
	return hdmi_write_byte(client, HDMI_INT_EN, data & 0xfe);
}

/*
 * Tx is brought to low-power state, off audio codec.
 * i2c alive. Still be able to response to INT.
 */
int hdmi_standby(struct hdmi_info *hdmi)
{
	u8 data;
	int ret;
	struct i2c_client *client = hdmi->client;

	HDMI_DBG("%s\n", __func__);
#if 0
	/* D2 sleep mode */
	data = hdmi_read(client, HDMI_POWER);
	return hdmi_write_byte(client, HDMI_POWER, (data & 0xfc) | 0x02);
#else
	if (SLEEP == hdmi->sleeping)
		return 0;
	/* D3 sleep mode */
	hdmi->sleeping = SLEEP;
	hdmi->cable_connected = false;
	data = hdmi_write_byte(client, 0x3c, hdmi_read(client, 0x3c) | 1);
	data = hdmi_write_byte(client, 0x3c, hdmi_read(client, 0x3c) & ~2);
	HDMI_DBG("%s: INT_EN=%02x\n", __func__, hdmi_read(client, 0x3c));
	data = hdmi_read(client, HDMI_POWER);
	data |= 4;
	ret = hdmi_write_byte(client, HDMI_POWER, data );
	if (ret)
		dev_err(&client->dev,
			"error on entering D3 sleep mode: into cold mode\n");
#if 0
	ret = hdmi_write_byte(client, HDMI_POWER, 7);
#else
	tpi_writeb_oneshot(hdmi, HDMI_POWER, 7);
#endif
/*
	if (ret)
		dev_err(&client->dev,
			"error on entering D3 sleep mode: set D3 mode\n");
*/
#endif
	return ret;
}

int hdmi_wakeup(struct hdmi_info *hdmi)
{
	int err = -EIO;
	int ret;
	u8 data;
	struct i2c_client *client = hdmi->client;

	HDMI_DBG("%s\n", __func__);
#if 0
	data = hdmi_read(client, HDMI_POWER);
	err = hdmi_write_byte(client, HDMI_POWER, data & 0xfc);
	if (err)
		goto exit;
#else
	/* Exiting D3 sleep mode */
	ret = hdmi_write_byte(client, 0xc7, 0);
	if (ret)
		dev_err(&client->dev,
			"error on exiting D3 sleep mode: 0xc7=0\n");

	data = hdmi_read(client, HDMI_POWER);
	data = ( data & 0xfc ) ;
	ret = hdmi_write_byte(client, HDMI_POWER, data );
	if (ret)
		dev_err(&client->dev,
			"error on exiting D3 sleep mode: 0x1e=0\n");
        /* Enable insternal TMDS source termination */
        hdmi_write_byte(client, 0xbc, 0x01);
        hdmi_write_byte(client, 0xbd, 0x82);
        data = hdmi_read(client, 0xbe);
        hdmi_write_byte(client, 0xbe, data | 0x01);

	hdmi->sleeping = AWAKE;
#endif

/*
	data = hdmi_read(client, HDMI_SYS_CTL);
	dev_info(&client->dev, "%s, HDMI_SYS_CTL=0x%x\n", __func__, data);
	err = hdmi_write_byte(client, HDMI_SYS_CTL, 0x01);
	if (err)
		goto exit;
*/
	return 0;
exit:
	dev_err(&client->dev, "%s: fail, err = %d\n", __func__, err);
	return err;
}

static int
hdmi_check_res(struct hdmi_device *hdmi_device, struct fb_var_screeninfo *var)
{
	if (((var->xres == 1280) && (var->yres == 720)) ||
	    ((var->xres == 800) && (var->yres == 600)) ||
	    ((var->xres == 720) && (var->yres == 576)) ||
	    ((var->xres == 720) && (var->yres == 480)) ||
	    ((var->xres == 640) && (var->yres == 480))) {
		dev_info(&hdmi_device->dev, "resolution check successfully\n");
		/* check pixel clock also */
		return 0;
	}

	return -EINVAL;
}

static struct msm_lcdc_timing hdmi_lcdc_timing[] = {
	[hd_720p] = {
		.clk_rate               = 74250000,
		.hsync_pulse_width      = 40,
		.hsync_back_porch       = 220,
		.hsync_front_porch      = 110,
		.hsync_skew             = 0,
		.vsync_pulse_width      = 5,
		.vsync_back_porch       = 20,
		.vsync_front_porch      = 5,
		.vsync_act_low          = 0,
		.hsync_act_low          = 0,
		.den_act_low            = 0,
	},
	[svga] = {
		.clk_rate               = 40000000,
		.hsync_pulse_width      = 128,
		.hsync_back_porch       = 88,
		.hsync_front_porch      = 40,
		.hsync_skew             = 0,
		.vsync_pulse_width      = 4,
		.vsync_back_porch       = 23,
		.vsync_front_porch      = 1,
		.vsync_act_low          = 0,
		.hsync_act_low          = 0,
		.den_act_low            = 0,
	},
	[pal] = {
		.clk_rate               = 27027000,
		.hsync_pulse_width      = 64,
		.hsync_back_porch       = 68,
		.hsync_front_porch      = 12,
		.hsync_skew             = 0,
		.vsync_pulse_width      = 5,
		.vsync_back_porch       = 39,
		.vsync_front_porch      = 5,
		.vsync_act_low          = 1,
		.hsync_act_low          = 1,
		.den_act_low            = 0,
	},
	[edtv] = {
		.clk_rate               = 27027000,
		.hsync_pulse_width      = 62,
		.hsync_back_porch       = 60,
		.hsync_front_porch      = 16,
		.hsync_skew             = 0,
		.vsync_pulse_width      = 6,
		.vsync_back_porch       = 30,
		.vsync_front_porch      = 9,
#if 1
		.vsync_act_low          = 1,
		.hsync_act_low          = 1,
#else
		.vsync_act_low          = 0,
		.hsync_act_low          = 0,
#endif

		.den_act_low            = 0,
	},
	[vga] = {
		.clk_rate               = 25175000,
		.hsync_pulse_width      = 96,
		.hsync_back_porch       = 48,
		.hsync_front_porch      = 16,
		.hsync_skew             = 0,
		.vsync_pulse_width      = 2,
		//.vsync_pulse_width      = 3,
		.vsync_back_porch       = 33,
		.vsync_front_porch      = 10,
		.vsync_act_low          = 1,
		.hsync_act_low          = 1,
		.den_act_low            = 0,
	},
};

static struct msm_lcdc_timing *
hdmi_set_res(struct hdmi_device *hdmi_device, struct fb_var_screeninfo *var)
{
	struct hdmi_info *info = container_of(hdmi_device, struct hdmi_info,
					hdmi_dev);

	printk(KERN_DEBUG "%s, info->res=%d=(%d x %d)\n",
		__func__, info->res, var->xres, var->yres);
	if ((var->xres == 1280) && (var->yres == 720))
		info->res = hd_720p;
	else if ((var->xres == 800) && (var->yres == 600))
		info->res = svga;
	else if ((var->xres == 720) && (var->yres == 576))
		info->res = pal;
	else if ((var->xres == 720) && (var->yres == 480))
		info->res = edtv;
	else if ((var->xres == 640) && (var->yres == 480))
		info->res = vga;
	else
		return ERR_PTR(-EINVAL);
/*
	if (info->user_playing)
		avc_send_avi_info_frames(info);
*/
	return &hdmi_lcdc_timing[info->res];
}

static int hdmi_get_cable_state(struct hdmi_device *hdmi_device, int *connect)
{
#if 0
	struct hdmi_info *info = container_of(hdmi_device, struct hdmi_info,
					hdmi_dev);
	struct i2c_client *client = info->client;
	u8 status;

	*connect = 0;
	status = hdmi_read(client, HDMI_INT_STAT);
	if (status & HOT_PLUG_STATE)
		*connect = 1;
#else
	struct hdmi_info *hdmi =
		container_of(hdmi_device, struct hdmi_info, hdmi_dev);
	*connect = hdmi->cable_connected;
#endif
//	HDMI_DBG("%s, state=%s\n", __func__, *connect ? "on" : "off" );
	return 0;
}

static int
hdmi_get_established_timing(struct hdmi_device *hdmi_device, u8 *byte)
{
	struct hdmi_info *info = container_of(hdmi_device, struct hdmi_info,
					hdmi_dev);

	HDMI_DBG("%s\n", __func__);
	memcpy(byte, &info->edid_buf[ESTABLISHED_TIMING_OFFSET], 3);
	return 0;
}

#if 0
// FIXME: remove the parameter: data
static u8 hdmi_request_ddc(struct i2c_client *client, int request, u8 data)
{
	int retry = 10;

	HDMI_DBG("%s, request=%d\n", __func__, request);
	if (request) {
		data = hdmi_read(client, HDMI_SYS_CTL);
		hdmi_write_byte(client, HDMI_SYS_CTL, (data | 0x04));
		msleep(1);
		hdmi_write_byte(client, HDMI_SYS_CTL, (data | 0x06));
		msleep(1);
	} else {
		hdmi_write_byte(client, HDMI_SYS_CTL, (data & 0xf9));
		hdmi_write_byte(client, HDMI_SYS_CTL, (data & 0xf9));
		/* make sure bit [2:1] = 00 */
		data = hdmi_read(client, HDMI_SYS_CTL);
		while ((data & 0x03) & retry--)
			msleep(1);
	}
	return data;
}

#else
// FIXME: remove the static varible. if caller need to presev the reg,
//        it should use hdmi_read() first.
static u8 hdmi_request_ddc(struct i2c_client *client, int request)
{
	int retry = 10;
	static u8 val = 0;
	u8 tmp;

        HDMI_DBG("%s, request=%d\n", __func__, request);

	if (request) {
		val = hdmi_read(client, HDMI_SYS_CTL);
		hdmi_write_byte(client, HDMI_SYS_CTL, (val | 0x04));
		msleep(1);
		hdmi_write_byte(client, HDMI_SYS_CTL, (val | 0x06));
		msleep(1);

	} else {
		do {
			hdmi_write_byte(client, HDMI_SYS_CTL, (val & 0xf9));
			tmp = hdmi_read(client, HDMI_SYS_CTL);
			msleep(1);
			/* make sure bit [2:1] = 00 */
		}       while ((tmp & 0x06) & retry--) ;
	}

	return 0;
}
#endif

static uint8_t timing_id[][3] = {
	{ 0x81, 0xc0, 1 << 6 },	/* 1280x720	*/
	{ 0x3b, 0x80, 1 << 5 },	/* 720x576	*/
	{ 0x3b, 0x00, 1 << 4 },	/* 720x480	*/
};

//----------------------------------------------------------------------
static irqreturn_t hdmi_irq_handler(int irq, void *data)
{
	struct hdmi_info *hdmi = (struct hdmi_info *) data;
        HDMI_DBG("%s\n", __func__);

	disable_irq_nosync(hdmi->client->irq);
	hdmi->isr_enabled = false;
	hdmi->first = true;
	if (!hdmi->cable_connected) {
		hdmi->timer.expires = jiffies + INTERVAL_HDCP_POLLING;
		add_timer(&hdmi->timer);
	}

        return IRQ_HANDLED;
}
/* ---------------------------------------------------------------- */
extern bool hdmifb_suspending;
static int hdmi_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	struct hdmi_info *info = container_of(ops, struct hdmi_info,
					hdmi_lcdc_ops);

	HDMI_DBG("%s\n", __func__);

	info->user_playing = false;
	info->video_streaming= false;
#if 0
	/* if called from suspending */
	if (hdmifb_suspending) {
		/* to avoid timer been revoked after standby */
		HDMI_DBG("suspending=true, disable timer\n");
		cancel_work_sync(&info->polling_work);
		del_timer(&info->timer);

		HDMI_DBG("%s\n", __func__);
		mutex_lock(&info->lock);
		hdmi_standby(info);
		info->power(2);
		mutex_unlock(&info->lock);
	}
#endif
	return 0;
}

static int hdmi_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	struct hdmi_info *info = container_of(ops, struct hdmi_info,
					hdmi_lcdc_ops);
	struct i2c_client *client = info->client;

	HDMI_DBG("%s\n", __func__);
	clk_set_rate(info->ebi1_clk, 120000000);
	if (info->suspending == true) {
		HDMI_DBG("%s :actived before panel_init\n", __func__);
		msleep(500);
	}

	info->user_playing = true;

	return 0;
}

static int hdmi_panel_init(struct msm_lcdc_panel_ops *ops)
{
	u8 conn;
	struct hdmi_info *hd = container_of(ops, struct hdmi_info,
						hdmi_lcdc_ops);
	struct i2c_client *client = hd->client;

	HDMI_DBG("%s\n", __func__);

	if (hd->hdmi_gpio_on)
		hd->hdmi_gpio_on();
	/* Turn-on 5V to ensure hot-plug detection */
	hd->power(5);

#if 0
	/* For D2 sleep mode */
	hd->power(1);

	ret = hdmi_write_byte(client, HDMI_EN_REG, 0x00);
	if (ret < 0)
		goto fail;

	hdmi_disable_int(client);

	data = hdmi_read(client, HDMI_POWER);
	if (data & 0xfc) {
		dev_info(&client->dev, "power state = %d\n", data & 0xfc);
	} else {
		dev_info(&client->dev, "bring HDMI back\n");
		hdmi_enable_int(client);
		HDMI_DBG("hotplug state=%d\n", hd->cable_connected);
	}
#else
	if (hd->polling) {
		mutex_lock(&hd->lock);
		hd->power(3);
		hdmi_wakeup(hd);
		hd->first = true;
		mod_timer(&hd->timer, jiffies + INTERVAL_HDCP_POLLING);
		conn = hdmi_read(client, HDMI_INT_STAT) & HOT_PLUG_STATE;
		tpi_init(hd);
#ifdef CONFIG_HTC_HEADSET_MGR
		switch_send_event(BIT_HDMI_AUDIO, conn);
#endif
		mutex_unlock(&hd->lock);
	}
	hd->suspending = false;
#endif
	return 0;
/*
fail:
	return ret;
*/
}

static int hdmi_panel_uninit(struct msm_lcdc_panel_ops *ops)
{
	struct hdmi_info *info = container_of(ops, struct hdmi_info,
						hdmi_lcdc_ops);
	HDMI_DBG("%s\n", __func__);

	if (info->hdmi_gpio_off)
		info->hdmi_gpio_off();
#if 0
	/* For D2 sleep mode */
	info->power(0);
#endif

        if (hdmifb_suspending) {
                /* to avoid timer been revoked after standby */
                HDMI_DBG("suspending=true, disable timer\n");
                cancel_work_sync(&info->polling_work);
                del_timer(&info->timer);
		flush_scheduled_work();

                HDMI_DBG("%s\n", __func__);
                mutex_lock(&info->lock);
                hdmi_standby(info);
                info->power(4);
                mutex_unlock(&info->lock);
        }
	info->suspending = true;

	return 0;
}

int avc_set_video_parm(struct hdmi_info *hdmi);
int avc_set_blank_screen(struct hdmi_info *hdmi);
void hdmi_pre_change(struct hdmi_info *hdmi) {
	if (hdmi->sleeping == SLEEP)
		return;
	mutex_lock(&hdmi->polling_lock);
	HDMI_DBG("%s\n", __func__);
	avc_set_blank_screen(hdmi);
	hdcp_off(hdmi);
	mutex_unlock(&hdmi->polling_lock);
}

void hdmi_post_change(struct hdmi_info *info, struct fb_var_screeninfo *var)
{
	u8 data[4];
	int i, ret, retry = 10;
	struct msm_lcdc_timing *timing;
	unsigned h_total, v_total, h_curr, v_curr;

	if (info->sleeping == SLEEP)
		return;

	mutex_lock(&info->polling_lock);
	HDMI_DBG("%s\n", __func__);
	timing = &hdmi_lcdc_timing[info->res];

	h_total = var->xres + timing->hsync_pulse_width +
		timing->hsync_back_porch + timing->hsync_front_porch;
	v_total = var->yres + timing->vsync_pulse_width +
		timing->vsync_back_porch + timing->vsync_front_porch;
	/* Waiting for video stream until steady */
	for (i = 0; i < retry ; i++) {
		/* TODO: error handling. */
		/* Read current horizontal/vertical info of video */
		data[0] = hdmi_read(info->client, 0x6a);
		data[1] = hdmi_read(info->client, 0x6b);
		data[2] = hdmi_read(info->client, 0x6c);
		data[3] = hdmi_read(info->client, 0x6d);
		h_curr = ((int)data[1]) << 8 | data[0];
		v_curr = ((int)data[3]) << 8 | data[2];
		if (h_curr == h_total && v_curr == v_total)
			break;
		msleep(17);
	}

	avc_set_video_parm(info);
	avc_send_avi_info_frames(info);
	info->video_streaming = true;

	mutex_unlock(&info->polling_lock);
}

static struct msm_fb_data hdmi_lcdc_fb_data = {
#if 1
	.xres           = 1280,
	.yres           = 720,
#else
	.xres           = 720,
	.yres           = 480,
#endif
	.width          = 94,
	.height         = 57,
	.output_format  = 0,
};

static struct msm_lcdc_platform_data hdmi_lcdc_platform_data = {
	.timing = &hdmi_lcdc_timing[hd_720p],
	.fb_id = 0,
	.fb_data = &hdmi_lcdc_fb_data,
};

static struct platform_device hdmi_lcdc_device = {
	.name   = "msm_mdp_hdmi",
	.id     = -1,
};

int register_hdmi_client(struct class_interface *interface)
{
	if (!hdmi_class) {
		pr_err("mdp: no hdmi_class when register hdmi client\n");
		return -ENODEV;
	}
	interface->class = hdmi_class;
	return class_interface_register(interface);
}

#if defined(OLD_DEBUGFS)
static spinlock_t hdmi_dbgfs_lock;
ssize_t hdmi_dbgfs_open(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
        return 0;
}

static ssize_t hdmi_edid_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        static char line[80], buffer[80*8*4];
	static char hextab[] = "0123456789abcdef";
        int i, j, n = 0, v, len, offset, line_size;
        unsigned long irq_flags;
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

	len = ((int)hdmi->edid_buf[0x7e]+1) * 128;
        spin_lock_irqsave(&hdmi_dbgfs_lock, irq_flags);
        memset(line, ' ', 79);
        line[79] = '\0';
        offset = strlen("0000 | ");
	line_size = offset + 3 * 16 + 1;

        for (i = 0; i < len / 16 ; i++) {
                scnprintf(line, offset + 1, "%04x | ", (i << 4));
                for (j = 0; j < 16 ; j++) {
                        v = hdmi->edid_buf[i * 16 + j];
                        line[offset + j * 3] = hextab[v / 16];
                        line[offset + j * 3 + 1] = hextab[v % 16];
                }
		line[line_size - 1] = '\n';
		strncpy(buffer + i * line_size, line, line_size);
		n += line_size;
        }
        spin_unlock_irqrestore(&hdmi_dbgfs_lock, irq_flags);
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

#if 0
static ssize_t hdmi_dbgfs_write(struct file *filp, const char __user *buf,
                size_t count, loff_t *ppos)
{
        unsigned long v;
        unsigned long irq_flags;
        char buff[80];
        struct tv_reg_data *trd = (struct tv_reg_data *)filp->private_data;

        if (count >= sizeof(buff))
                return -EINVAL;
        if (copy_from_user(&buff, buf, 80))
                return -EFAULT;
        buff[count] = 0;

        spin_lock_irqsave(&hdmi_dbgfs_lock, irq_flags);
        strict_strtoul(buff, 16, &v);
        buff[strlen(buff)]=0;
        writel(v, tvenc_base+trd->offset);
        spin_unlock_irqrestore(&hdmi_dbgfs_lock, irq_flags);

        return count;
}
#endif

static ssize_t hdmi_cable_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
	int n;
        char buffer[80];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        n = scnprintf(buffer, 80, "%d\n", hdmi->cable_connected);
	n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t hdmi_sleeping_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
	int n;
        char buffer[80];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

	n = scnprintf(buffer, 80, "%d\n", hdmi->sleeping);
	n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

int hdmifb_get_mode(void);
static ssize_t hdmi_fb_mode_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
	int n;
        char buffer[80];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        n = scnprintf(buffer, 80, "%d\n", hdmifb_get_mode());
	n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t hdmi_isr_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n;
        char buffer[80];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        n = scnprintf(buffer, 80, "%d\n", hdmi->isr_enabled);
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations hdmi_fops[] = {
	{
		.open  = hdmi_dbgfs_open,
		.read  = hdmi_edid_read,
	},
	{ 	/* cable*/
		.open  = hdmi_dbgfs_open,
		.read  = hdmi_cable_read,
	},
	{	/* sleeping */
		.open  = hdmi_dbgfs_open,
		.read  = hdmi_sleeping_read,
	},
        {       /* fb_mode */
                .open  = hdmi_dbgfs_open,
                .read  = hdmi_fb_mode_read,
        },
        {       /* isr_enabled */
                .open  = hdmi_dbgfs_open,
                .read  = hdmi_isr_read,
        },

};

static int hdmi_debugfs_init(struct hdmi_info *hdmi)
{
        struct dentry *dent_hdmi;
        int ret;

        spin_lock_init(&hdmi_dbgfs_lock);
        dent_hdmi = debugfs_create_dir("hdmi", 0);
        if (IS_ERR(dent_hdmi))
                return PTR_ERR(dent_hdmi);
	debugfs_create_file("edid", 0644, dent_hdmi, hdmi, &hdmi_fops[0]);
	debugfs_create_file("cable", 0444, dent_hdmi, hdmi, &hdmi_fops[1]);
	debugfs_create_file("sleeping", 0444, dent_hdmi, hdmi, &hdmi_fops[2]);
	debugfs_create_file("fb_mode", 0444, dent_hdmi, hdmi, &hdmi_fops[3]);
	debugfs_create_file("isr", 0444, dent_hdmi, hdmi, &hdmi_fops[4]);

        return 0;
}
#endif

static int __init hdmi_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hdmi_info *hd;
	struct hdmi_platform_data *pdata;
	int ret = -EIO;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "No supported I2C func\n");
		ret = -ENOTSUPP;
		goto exit;
	}

	hd = kzalloc(sizeof(*hd), GFP_KERNEL);
	if (hd == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	hd->client = client;
	i2c_set_clientdata(client, hd);
	mutex_init(&hd->lock);
	mutex_init(&hd->lock2);
	mutex_init(&hd->polling_lock);

	hd->ebi1_clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(hd->ebi1_clk)) {
		dev_err(&client->dev, "get ebi1 clk fail\n");
		goto fail_get_ebi1;
	}

	pdata = client->dev.platform_data;
	if (unlikely(!pdata) || unlikely(!pdata->power)) {
		dev_err(&client->dev, "No platform data\n");
		ret = -ENXIO;
		goto fail_power;
	} else {
		if (pdata->hdmi_gpio_on)
			pdata->hdmi_gpio_on();
		hd->power = pdata->power;
		ret = hd->power(1);
		if (ret) {
			dev_err(&client->dev, "hdmi power on failed\n");
			ret = -EIO;
			goto fail_power;
		}
	}

	ret = hdmi_write_byte(client, HDMI_EN_REG, 0x00);
	if (ret < 0) {
		ret = -EIO;
		goto fail_hdmi_init;
	}

	ret = hdmi_read(client, HDMI_IDENTIFY);
	if (ret < 0) {
		ret = -EIO;
		goto fail_hdmi_init;
	} else if (ret != 0xb0) {
		dev_err(&client->dev, "can not recognize, 0x%x\n", ret);
		ret = -ENXIO;
		goto fail_hdmi_init;
	}

	hdmi_disable_int(client);

	hd->user_playing = false;
	tpi_prepare(hd);
	ret = request_irq(client->irq, hdmi_irq_handler, IRQF_TRIGGER_LOW,
			client->name, hd);
	if (ret) {
		/* HDMI did not care if interrupt fail */
		dev_err(&client->dev, "request irq fail, err = %d\n", ret);
	} else {
		ret = hdmi_enable_int(client);
		if (ret) {
			free_irq(client->irq, hd);
			ret = -ENOTSUPP;
		}
	}

	dev_info(&client->dev, "hdmi is on line with irq %s\n",
		ret ? "Disabled" : "Enabled");

	/* set up "panel" */
	hd->hdmi_lcdc_ops.init = hdmi_panel_init;
	hd->hdmi_lcdc_ops.uninit = hdmi_panel_uninit;
	hd->hdmi_lcdc_ops.blank = hdmi_panel_blank;
	hd->hdmi_lcdc_ops.unblank = hdmi_panel_unblank;
	hd->hdmi_gpio_on = pdata->hdmi_gpio_on;
	hd->hdmi_gpio_off = pdata->hdmi_gpio_off;

	hdmi_lcdc_platform_data.panel_ops = &hd->hdmi_lcdc_ops;
	hdmi_lcdc_platform_data.fb_resource = &pdata->hdmi_res;
	hdmi_lcdc_device.dev.platform_data = &hdmi_lcdc_platform_data;
	ret = platform_device_register(&hdmi_lcdc_device);
	if (ret)
		goto fail_hdmi_init;

	hd->hdmi_dev.check_res = hdmi_check_res;
	hd->hdmi_dev.set_res = hdmi_set_res;
	hd->hdmi_dev.get_cable_state = hdmi_get_cable_state;
	hd->hdmi_dev.get_establish_timing = hdmi_get_established_timing;

	hd->hdmi_dev.dev.parent = &client->dev;
	hd->hdmi_dev.dev.class = hdmi_class;
	//snprintf(hd->hdmi_dev.dev.bus_id, BUS_ID_SIZE, "hdmi%d", 0);
	dev_set_name(&hd->hdmi_dev.dev, "hdmi%d", 0);
	ret = device_register(&hd->hdmi_dev.dev);
	if (ret)
		dev_err(&client->dev, "device register fail\n");

#if defined(HDMI_DEBUGFS)
	hdmi_debugfs_init(hd);
#endif
	/* check any pending interrupt */
	hdmi_irq_handler(client->irq, hd);
	return 0;

fail_hdmi_init:
fail_get_ebi1:
	clk_put(hd->ebi1_clk);
fail_power:
	kfree(hd);
exit:
	dev_err(&client->dev, "%s fail, err = %d\n", __func__, ret);
	return ret;
}

/* -------------------------------------------------------------------- */

static const struct i2c_device_id hdmi_id[] = {
	{HDMI_NAME, 0},
	{ }
};

static struct i2c_driver hdmi_driver = {
	.probe		= hdmi_probe,
	/*.remove	= hdmi_remove,*/
	.id_table	= hdmi_id,
	.driver		= {
		.name = HDMI_NAME,
	},
};

static int __init hdmi_init(void)
{
	hdmi_class = class_create(THIS_MODULE, "msm_hdmi");
	if (IS_ERR(hdmi_class)) {
		printk(KERN_ERR "Error creating hdmi class\n");
		return PTR_ERR(hdmi_class);
	}
	return i2c_add_driver(&hdmi_driver);
}

static void __exit hdmi_exit(void)
{
	i2c_del_driver(&hdmi_driver);
}

module_init(hdmi_init);
module_exit(hdmi_exit);

MODULE_DESCRIPTION("Sil902x hdmi driver");
MODULE_LICENSE("GPL");
