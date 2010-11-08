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
 */

/* TODO:
 * 1. Mutex while active
 */
#include <linux/delay.h>
#include <mach/msm_fb.h>
#include "../include/fb-hdmi.h"
#include "../include/sil902x.h"
#include "../include/tpi.h"

#if 1
#define HDMI_DBG(s...) printk("[hdmi/avc]" s)
#else
#define HDMI_DBG(s...) do {} while (0)
#endif

static u8 video_param[][8] = {
	[hd_720p] = {0x01, 0x1d, 0x70, 0x17, 0x72, 0x06, 0xee, 0x02},
	[svga] = {0xa0, 0x0f, 0x70, 0x17, 0x20, 0x04, 0x74, 0x02},
	[pal] = {0x8c, 0x0a, 0x88, 0x13, 0x60, 0x03, 0x71, 0x02}, /* 576p50 */
	[edtv] = {0x8c, 0x0a, 0x70, 0x17, 0x5a, 0x03, 0x0d, 0x02},/* 480p60 */
	[vga] = {0x8c, 0x0a, 0x70, 0x17, 0x20, 0x03, 0x0d, 0x02},
};

#if 0
static u8 avi_info_frame[][14] = {
	[hd_720p] = {0x32, 0x0d, 0xa8, 0x84, 0x04, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00},
	[svga] = {0xd1, 0x0e, 0x08, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00},
	[pal] = {0x64, 0x0d, 0x68, 0x84, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00}, /* 576p50 */
	[edtv] = {0x73, 0x0d, 0x68, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00},
	[vga] = {0x5E, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00},
};
#else
static u8 avi_info_frame[][14] = {
	[hd_720p] = {0x43, 0x00, 0x28, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
		     0x00, 0x00, 0x00, 0x00, 0x00},
	[svga] = {0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00},
	[pal] = {0x46, 0x00, 0x18, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00}, /* 576p50 */
	[edtv] = {0x55, 0x00, 0x18, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00},
	[vga] = {0x5E, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00},
};
#endif

static u8 audio_info_frame[] =
	{ 0xc2, 0x84, 0x01, 0x0a, 0x71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#if 0
int hdmi_active9022(struct i2c_client *client)
{
	int i, ret = -EIO;
	struct hdmi_info *info = i2c_get_clientdata(client);
	u8 *video_parm = &video_param[info->res][0];

	HDMI_DBG("%s+\n", __func__);
        mutex_lock(&info->polling_lock);
	mutex_lock(&info->lock);

	//hdcp_off(info);
	DisableTMDS(info);
	msleep(128);

        /* choose video mode */
	ret = i2c_smbus_write_i2c_block_data(client, 0, 8, video_parm);

	/* wakeup Sil9022 to D0 state */
	ret = hdmi_write_byte(client, HDMI_POWER, 0);

	if (edid_check_sink_type(info)) {
		/* HDMI Output */
		ReadModifyWriteTPI(info, TPI_SYSTEM_CONTROL,
			OUTPUT_MODE_MASK, OUTPUT_MODE_HDMI);
		/* audio configuration */
		ret = hdmi_write_byte(client, 0x26, 0x91);
		ret = hdmi_write_byte(client, 0x25, 0x03);
		ret = hdmi_write_byte(client, 0x27, 0x59);
		ret = hdmi_write_byte(client, 0x28, 0x00);
		ret = hdmi_write_byte(client, 0x1f, 0x80);
		ret = hdmi_write_byte(client, 0x20, 0x90);
		ret = hdmi_write_byte(client, 0x21, 0x00);
		//ret = hdmi_write_byte(client, 0x24, 0x00);//0x00 for 44.1k
		ret = hdmi_write_byte(client, 0x24, 0x02);//0x02 for 48k
		ret = hdmi_write_byte(client, 0x25, 0x00);
		ret = hdmi_write_byte(client, 0xbc, 0x02);
		ret = hdmi_write_byte(client, 0xbd, 0x24);
		ret = hdmi_write_byte(client, 0xbe, 0x92);
		ret = hdmi_write_byte(client, 0xbc, 0x02);
		ret = hdmi_write_byte(client, 0xbd, 0x2f);
		ret = hdmi_write_byte(client, 0xbe, 0);
		ret = hdmi_write_byte(client, 0x26, 0x81);
	} else {
		ReadModifyWriteTPI(info, TPI_SYSTEM_CONTROL,
			OUTPUT_MODE_MASK, OUTPUT_MODE_DVI);
		SetAudioMute(info, AUDIO_MUTE_MUTED);
	}
	ret = hdmi_write_byte(client, HDMI_PIXEL_DATA, 0x60);

	ret = i2c_smbus_write_i2c_block_data(client, HDMI_AUDIO_INFO_FRAME,
			15, audio_info_frame);
	hdmi_write_byte(client, 0x09, 0);
	hdmi_write_byte(client, 0x0a, 0);
	for (i = 0; i < 14 ;i++)
		hdmi_write_byte(client, 0xc + i, avi_info_frame[info->res][i]);
	
	EnableTMDS(info);
	HDMI_DBG("%s-\n", __func__);
	mutex_unlock(&info->lock);
	mutex_unlock(&info->polling_lock);

	return ret;
}
#endif

int avc_set_video_parm(struct hdmi_info *hdmi)
{
	int ret;
	u8 *video_parm = video_param[hdmi->res];

	ret = i2c_smbus_write_i2c_block_data(hdmi->client, 0, 8, video_parm);

	return ret;
}

int avc_set_blank_screen(struct hdmi_info *hdmi)
{
	HDMI_DBG("%s+\n", __func__);
	hdmi_write_byte(hdmi->client, 0x09, 0x03);
	hdmi_write_byte(hdmi->client, 0x19, 0x00);
	hdmi_write_byte(hdmi->client, 0x26,
		hdmi_read(hdmi->client, 0x26) | 0x10);
}

int hdmi_active9022_dup(struct i2c_client *client)
{
	int i, ret = -EIO;
	struct hdmi_info *info = i2c_get_clientdata(client);
	u8 *video_parm = &video_param[info->res][0];

	HDMI_DBG("%s+\n", __func__);

	//hdcp_off(info);
	DisableTMDS(info);
	msleep(128);

	/* choose video mode */
	ret = i2c_smbus_write_i2c_block_data(client, 0, 8, video_parm);

	/* wakeup Sil9022 to D0 state */
	ret = hdmi_write_byte(client, HDMI_POWER, 0);

	if (edid_check_sink_type(info)) {
		/* HDMI Output */
		ReadModifyWriteTPI(info, TPI_SYSTEM_CONTROL,
			OUTPUT_MODE_MASK, OUTPUT_MODE_HDMI);
		/* audio configuration */
		ret = hdmi_write_byte(client, 0x26, 0x91);
		ret = hdmi_write_byte(client, 0x25, 0x03);
		ret = hdmi_write_byte(client, 0x27, 0x59);
		ret = hdmi_write_byte(client, 0x28, 0x00);
		ret = hdmi_write_byte(client, 0x1f, 0x80);
		ret = hdmi_write_byte(client, 0x20, 0x90);
		ret = hdmi_write_byte(client, 0x21, 0x00);
		//ret = hdmi_write_byte(client, 0x24, 0x00);//0x00 for 44.1k
		ret = hdmi_write_byte(client, 0x24, 0x02);//0x02 for 48k
		ret = hdmi_write_byte(client, 0x25, 0x00);
		ret = hdmi_write_byte(client, 0xbc, 0x02);
		ret = hdmi_write_byte(client, 0xbd, 0x24);
		ret = hdmi_write_byte(client, 0xbe, 0x92);
		ret = hdmi_write_byte(client, 0xbc, 0x02);
		ret = hdmi_write_byte(client, 0xbd, 0x2f);
		ret = hdmi_write_byte(client, 0xbe, 0);
		ret = hdmi_write_byte(client, 0x26, 0x81);
	} else {
		ReadModifyWriteTPI(info, TPI_SYSTEM_CONTROL,
			OUTPUT_MODE_MASK, OUTPUT_MODE_DVI);
		SetAudioMute(info, AUDIO_MUTE_MUTED);
	}
	ret = hdmi_write_byte(client, HDMI_PIXEL_DATA, 0x60);

	ret = i2c_smbus_write_i2c_block_data(client, HDMI_AUDIO_INFO_FRAME,
			15, audio_info_frame);
	hdmi_write_byte(client, 0x09, 0x03);
	hdmi_write_byte(client, 0x0a, 0);
	for (i = 0; i < 14 ;i++)
		hdmi_write_byte(client, 0xc + i, avi_info_frame[info->res][i]);

	EnableTMDS(info);

	HDMI_DBG("%s-\n", __func__);
	return ret;
}

bool avc_send_avi_info_frames(struct hdmi_info *hdmi)
{
        int i;

	HDMI_DBG("%s res=%d\n", __func__, hdmi->res);
	for (i = 0; i < 14 ;i++)
		hdmi_write_byte(hdmi->client, 0xc + i,
				avi_info_frame[hdmi->res][i]);

        return true;
}

#if 0
/* FIXME: intergrate with active9022 */
bool InitVideo(struct hdmi_info *hdmi, u8 Mode, u8 TclkSel, bool Init)
{
	int Pattern, ret;
	u8 *video_parm = &video_param[hdmi->res][0];

	/* Use TPI 0x08[7:6] for 9022A/24A video clock multiplier */
	Pattern = (1 << 6) & 0xc0;
	ReadSetWriteTPI(hdmi, TPI_PIX_REPETITION, Pattern);

	ret = i2c_smbus_write_block_data(hdmi->client, 0, 8, video_parm);

	/* input format */
	hdmi_write_byte(hdmi->client, 0x09, 0);
	hdmi_write_byte(hdmi->client, 0x0a, 0);
	if(Init) {
		hdmi_write_byte(hdmi->client, 0x08, 0x60);
		hdmi_write_byte(hdmi->client, 0x09, 0);
		hdmi_write_byte(hdmi->client, 0x0a, 0);
		hdmi_write_byte(hdmi->client, 0x60, 0x4);
	}

	// termination ???
	//ret = (read_back_door_register(0x1, 0x82) & 0x3f ) | 0x25;
	//write_back_door_register();
#if 0
	for (i = 0; i < 14 ;i++) {
		hdmi_write_byte(hdmi->client, 0xc + i, avi_info_frame[info->res][i]);
	}
#endif

	return true;
}
#endif

bool avc_init_video(struct hdmi_info *hdmi, u8 mode, u8 TclkSel, bool Init)
{
        int ret;
        u8 *video_parm = &video_param[hdmi->res][0];

	HDMI_DBG("%s\n", __func__);
        /* Use TPI 0x08[7:6] for 9022A/24A video clock multiplier */
	hdmi_write_byte(hdmi->client, HDMI_PIXEL_DATA, 0x60);

        ret = i2c_smbus_write_i2c_block_data(hdmi->client, 0, 8, video_parm);

        /* input format */
        hdmi_write_byte(hdmi->client, TPI_INPUT_FORMAT_REG, 0x00);
        hdmi_write_byte(hdmi->client, TPI_OUTPUT_FORMAT_REG, 0x00);
#if 0
        if (Init) {
                hdmi_write_byte(hdmi->client, 0x08, 0x60);
                hdmi_write_byte(hdmi->client, 0x09, 0);
        	hdmi_write_byte(hdmi->client, 0x0a, 0);
		/* Default to External Sync mode + disable VSync adjustment */
                hdmi_write_byte(hdmi->client, 0x60, 0x4);
		/* Disable DE generator by default */
                hdmi_write_byte(hdmi->client, 0x63, 0x0);
        }

#endif
        ret = (tpi_read_backdoor_register(hdmi, INTERNAL_PAGE_1, TMDS_CONT_REG)
		& 0x3f ) | 0x25;
        tpi_write_backdoor_register(hdmi, INTERNAL_PAGE_1, TMDS_CONT_REG, ret);

        return true;
}

void avc_set_basic_audio(struct hdmi_info *hdmi)
{
	int ret;
	struct i2c_client *client = hdmi->client;
	HDMI_DBG("%s\n", __func__);

        ret = hdmi_write_byte(client, 0x26, 0x91);
        ret = hdmi_write_byte(client, 0x25, 0x03);
        ret = hdmi_write_byte(client, 0x27, 0x59);
        ret = hdmi_write_byte(client, 0x28, 0x00);
        ret = hdmi_write_byte(client, 0x1f, 0x80);
        ret = hdmi_write_byte(client, 0x20, 0x90);
        ret = hdmi_write_byte(client, 0x21, 0x00);
        //ret = hdmi_write_byte(client, 0x24, 0x00);//0x00 for 44.1k
        ret = hdmi_write_byte(client, 0x24, 0x02);//0x02 for 48k
        ret = hdmi_write_byte(client, 0x25, 0x00);
        ret = hdmi_write_byte(client, 0xbc, 0x02);
        ret = hdmi_write_byte(client, 0xbd, 0x24);
        ret = hdmi_write_byte(client, 0xbe, 0x92);
        ret = hdmi_write_byte(client, 0xbc, 0x02);
        ret = hdmi_write_byte(client, 0xbd, 0x2f);
        ret = hdmi_write_byte(client, 0xbe, 0);
        ret = hdmi_write_byte(client, 0x26, 0x81);

	ret = i2c_smbus_write_i2c_block_data(client, HDMI_AUDIO_INFO_FRAME,
			15, audio_info_frame);
}

/* simplifier version of ChangeVideoMode() */
u8 avc_change_video_mode(struct hdmi_info *hdmi, int *resolution)
{
	HDMI_DBG("%s\n", __func__);

	hdcp_off(hdmi);
	DisableTMDS(hdmi);
	/* allow control InfoFrames to pass through to the sink device. */
	mdelay(128);

	// FIXME: video mode
	avc_init_video(hdmi, vid_mode, 0, 0);

	hdmi_write_byte(hdmi->client, TPI_PIX_REPETITION, 0x60);
	hdmi_write_byte(hdmi->client, TPI_INPUT_FORMAT, 0);

	if (edid_check_sink_type(hdmi))
		ReadSetWriteTPI(hdmi, 0x1a, 0x01);
	else
		ReadSetWriteTPI(hdmi, 0x1a, 0x00);

	/* FIXME: 720p/480p ?*/
	hdmi_write_byte(hdmi->client, TPI_OUTPUT_FORMAT, 0);

	/* set 0x60[7] = 0 for External Sync */
	ReadClearWriteTPI(hdmi, TPI_SYNC_GEN_CTRL, 0x80);
	/* clear 0x63[6] = 0 to disable internal DE */
	ReadClearWriteTPI(hdmi, 0x63, 1 << 6);

	/* InfoFrames - only if output mode is HDMI */
	if (edid_check_sink_type(hdmi))
		avc_send_avi_info_frames(hdmi);

	/* SETTING UP AVI InfoFrames CLEARS 0x63 and 0x60[5] */
	hdmi_write_byte(hdmi->client, TPI_SYNC_GEN_CTRL, 1 << 2);
        /* SETTING UP AVI InfoFrames CLEARS 0x63 */
	hdmi_write_byte(hdmi->client, 0x63, 0);

	/* YC Input Mode Select */
	hdmi_write_byte(hdmi->client, 0x0b, 0);	// 0x0b
	EnableTMDS(hdmi);

	return VIDEO_MODE_SET_OK;
}
