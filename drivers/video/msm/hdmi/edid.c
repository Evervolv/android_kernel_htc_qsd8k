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

 * Common function for accessing/debugging EDID data.
 * Reference:
	http://en.wikipedia.org/wiki/Extended_display_identification_data
*/

#include <linux/device.h>
#include <linux/htc_hdmi.h>
#include <linux/debugfs.h>

#include "include/fb-hdmi.h"
#include "include/sil902x.h"
                           
#if 1
#define EDID_DBG(s...) printk("[hdmi/edid]" s)
#else
#define EDID_DBG(s...) do {} while (0)
#endif

static struct video_mode established_timing_db[] = {
	{800, 600, 60, ASPECT(4, 3), PROGRESSIVE, false, "800x600 @ 60 Hz"},
	{800, 600, 56, ASPECT(4, 3), PROGRESSIVE, false, "800x600 @ 56 Hz"},
	{640, 480, 75, ASPECT(4, 3), PROGRESSIVE, false, "800x600 @ 75 Hz"},
	{640, 480, 72, ASPECT(4, 3), PROGRESSIVE, false, "640x480 @ 72 Hz"},
	{640, 480, 67, ASPECT(4, 3), PROGRESSIVE, false, "640x480 @ 67 Hz"},
	{640, 480, 60, ASPECT(4, 3), PROGRESSIVE, false, "640x480 @ 60 Hz"},
	{720, 400, 88, ASPECT(4, 3), PROGRESSIVE, false, "720x400 @ 88 Hz"},
	{720, 400, 70, ASPECT(4, 3), PROGRESSIVE, false, "720x400 @ 70 Hz"},

	{1280, 1024, 75, ASPECT(4, 3), PROGRESSIVE, false, "1280x1024@75 Hz"},
	{1024, 768, 75, ASPECT(4, 3), PROGRESSIVE, false, "1024x768@75 Hz"},
	{1024, 768, 70, ASPECT(4, 3), PROGRESSIVE, false, "1024x768@70 Hz"},
	{1024, 768, 60, ASPECT(4, 3), PROGRESSIVE, false, "1024x768@60 Hz"},
	{1024, 768, 87, ASPECT(4, 3), INTERLACE, false,
		"1024x768@87 Hz (Interlaced)"},
	{832, 624, 75, ASPECT(4, 3), PROGRESSIVE, false, "832x624@75 Hz"},
	{800, 600, 75, ASPECT(4, 3), PROGRESSIVE, false, "800x600@75 Hz"},
	{800, 600, 72, ASPECT(4, 3), PROGRESSIVE, false, "800x600@72 Hz"},

	{1152, 870, 75, ASPECT(4, 3), PROGRESSIVE, false, "1152x870 @ 75 Hz"},
};

static struct video_mode standard_timing_db[8];

static struct video_mode additional_timing_db[] = {
	{640, 480, 60, ASPECT(4, 3), PROGRESSIVE, false,
		" 1 DMT0659   4:3                640x480p @ 59.94/60Hz"},
	{720, 480, 60, ASPECT(4, 3), PROGRESSIVE, false,
		" 2 480p      4:3                720x480p @ 59.94/60Hz"},
	{720, 480, 60, ASPECT(16, 9), PROGRESSIVE, false,
		" 3 480pH    16:9                720x480p @ 59.94/60Hz"},
	{1280, 720, 60, ASPECT(16, 9), PROGRESSIVE, false,
		" 4 720p     16:9               1280x720p @ 59.94/60Hz"},
	{1920, 1080, 60, ASPECT(4, 3), INTERLACE, false,
		" 5 1080i    16:9              1920x1080i @ 59.94/60Hz"},
	{720, 480, 60, ASPECT(4, 3), INTERLACE, false,
		" 6 480i      4:3          720(1440)x480i @ 59.94/60Hz"},
	{720, 480, 60, ASPECT(16, 9), INTERLACE, false,
		" 7 480iH    16:9          720(1440)x480i @ 59.94/60Hz"},
	{720, 240, 60, ASPECT(4, 3), PROGRESSIVE, false,
		" 8 240p      4:3          720(1440)x240p @ 59.94/60Hz"},
	{720, 480, 60, ASPECT(16, 9), PROGRESSIVE, false,
		" 9 240pH    16:9          720(1440)x240p @ 59.94/60Hz"},
	{2880, 480, 60, ASPECT(4, 3), INTERLACE, false,
		"10 480i4x    4:3             (2880)x480i @ 59.94/60Hz"},
	{2880, 480, 60, ASPECT(16, 9), INTERLACE, false,
		"11 480i4xH  16:9             (2880)x480i @ 59.94/60Hz"},
	{2880, 240, 60, ASPECT(4, 3), PROGRESSIVE, false,
		"12 240p4x    4:3             (2880)x240p @ 59.94/60Hz"},
	{2880, 240, 60, ASPECT(16, 9), PROGRESSIVE, false,
		"13 240p4xH  16:9             (2880)x240p @ 59.94/60Hz"},
	{1440, 480, 60, ASPECT(4, 3), PROGRESSIVE, false,
		"14 480p2x    4:3               1440x480p @ 59.94/60Hz"},
	{1440, 480, 60, ASPECT(16, 9), PROGRESSIVE, false,
		"15 480p2xH  16:9               1440x480p @ 59.94/60Hz"},
	{1920, 1080, 60, ASPECT(16, 9), PROGRESSIVE, false,
		"16 1080p    16:9              1920x1080p @ 59.94/60Hz"},
	{720, 576, 50, ASPECT(4, 3), PROGRESSIVE, false,
		"17 576p      4:3                720x576p @ 50Hz"},
	{720, 576, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"18 576pH    16:9                720x576p @ 50Hz"},
	{1280, 720, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"19 720p50   16:9               1280x720p @ 50Hz"},
	{1920, 1080, 50, ASPECT(16, 9), INTERLACE, false,
		"20 1080i25  16:9              1920x1080i @ 50Hz*"},
	{1440, 576, 50, ASPECT(4, 3), INTERLACE, false,
		"21 576i      4:3          720(1440)x576i @ 50Hz"},
	{1440, 576, 50, ASPECT(4, 3), PROGRESSIVE, false,
		"22 576iH    16:9          720(1440)x576i @ 50Hz"},
	{720, 288, 50, ASPECT(4, 3), PROGRESSIVE, false,
		"23 288p      4:3          720(1440)x288p @ 50Hz"},
	{720, 288, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"24 288pH    16:9          720(1440)x288p @ 50Hz"},
	{2880, 576, 50, ASPECT(4, 3), INTERLACE, false,
		"25 576i4x    4:3             (2880)x576i @ 50Hz"},
	{2880, 576, 50, ASPECT(16, 9), INTERLACE, false,
		"26 576i4xH  16:9             (2880)x576i @ 50Hz"},
	{2880, 288, 50, ASPECT(4, 3), PROGRESSIVE, false,
		"27 288p4x    4:3             (2880)x288p @ 50Hz"},
	{2880, 288, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"28 288p4xH  16:9             (2880)x288p @ 50Hz"},
	{1440, 576, 50, ASPECT(4, 3), PROGRESSIVE, false,
		"29 576p2x    4:3               1440x576p @ 50Hz"},
	{1440, 576, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"30 576p2xH  16:9               1440x576p @ 50Hz"},
	{1920, 1080, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"31 1080p50  16:9              1920x1080p @ 50Hz"},
	{1920, 1080, 24, ASPECT(16, 9), PROGRESSIVE, false,
		"32 1080p24  16:9              1920x1080p @ 23.98/24Hz"},
	{1920, 1080, 25, ASPECT(16, 9), PROGRESSIVE, false,
		"33 1080p25  16:9              1920x1080p @ 25Hz"},
	{1920, 1080, 30, ASPECT(16, 9), PROGRESSIVE, false,
		"34 1080p30  16:9              1920x1080p @ 29.97/30Hz"},
	{2880, 480, 60, ASPECT(4, 3), PROGRESSIVE, false,
		"35 480p4x    4:3             (2880)x480p @ 59.94/60Hz"},
	{2880, 480, 60, ASPECT(16, 9), PROGRESSIVE, false,
		"36 480p4xH  16:9             (2880)x480p @ 59.94/60Hz"},
	{2880, 576, 50, ASPECT(4, 3), PROGRESSIVE, false,
		"37 576p4x    4:3             (2880)x576p @ 50Hz"},
	{2880, 576, 50, ASPECT(16, 9), PROGRESSIVE, false,
		"38 576p4xH  16:9             (2880)x576p @ 50Hz"},
	{1920, 1080, 50, ASPECT(16, 9), INTERLACE, false,
		"39 108Oi25  16:9  1920x1080i(1250 Total) @ 50Hz*"},
	{1920, 1080, 100, ASPECT(16, 9), INTERLACE, false,
		"40 1080i50  16:9              1920x1080i @ 100Hz"},
	{1280, 720, 100, ASPECT(16, 9), PROGRESSIVE, false,
		"41 720p100  16:9               1280x720p @ 100Hz"},
	{720, 576, 100, ASPECT(4, 3), PROGRESSIVE, false,
		"42 576p100   4:3                720x576p @ 100Hz"},
	{720, 576, 100, ASPECT(16, 9), PROGRESSIVE, false,
		"43 576p100H 16:9                720x576p @ 100Hz"},
	{720, 576, 100, ASPECT(4, 3), INTERLACE, false,
		"44 576i50    4:3          720(1440)x576i @ 100Hz"},
	{720, 576, 100, ASPECT(16, 9), INTERLACE, false,
		"45 576i50H  16:9          720(1440)x576i @ 100Hz"},
	{1920, 1080, 120, ASPECT(16, 9), INTERLACE, false,
		"46 1080i60  16:9              1920x1080i @ 119.88/120Hz"},
	{1280, 720, 120, ASPECT(16, 9), PROGRESSIVE, false,
		"47 720p120  16:9               1280x720p @ 119.88/120Hz"},
	{720, 480, 120, ASPECT(4, 3), PROGRESSIVE, false,
		"48 480p119   4:3                720x480p @ 119.88/120Hz"},
	{720, 480, 120, ASPECT(16, 9), PROGRESSIVE, false,
		"49 480p119H 16:9                720x480p @ 119.88/120Hz"},
	{720, 480, 120, ASPECT(4, 3), INTERLACE, false,
		"50 480i59    4:3          720(1440)x480i @ 119.88/120Hz"},
	{720, 480, 120, ASPECT(16, 9), INTERLACE, false,
		"51 480i59H  16:9          720(1440)x480i @ 119.88/120Hz"},
	{720, 576, 200, ASPECT(4, 3), PROGRESSIVE, false,
		"52 576p200   4:3                720x576p @ 200Hz"},
	{720, 576, 200, ASPECT(16, 9), PROGRESSIVE, false,
		"53 576p200H 16:9                720x576p @ 200Hz"},
	{720, 576, 200, ASPECT(4, 3), INTERLACE, false,
		"54 576i100   4:3          720(1440)x576i @ 200Hz"},
	{720, 576, 200, ASPECT(16, 9), INTERLACE, false,
		"55 576i100H 16:9          720(1440)x576i @ 200Hz"},
	{720, 480, 240, ASPECT(4, 3), PROGRESSIVE, false,
		"56 480p239   4:3                720x480p @ 239.76/240Hz"},
	{720, 480, 240, ASPECT(16, 9), PROGRESSIVE, false,
		"57 480p239H 16:9                720x480p @ 239.76/240Hz"},
	{720, 480, 240, ASPECT(4, 3), INTERLACE, false,
		"58 480i119   4:3          720(1440)x480i @ 239.76/240Hz"},
	{720, 480, 240, ASPECT(16, 9), INTERLACE, false,
		"59 480i119H 16:9          720(1440)x480i @ 239.76/240Hz"},
	{1280, 720, 24, ASPECT(16, 9), PROGRESSIVE, false,
		"60 720p24   16:9               1280x720p @ 23.98/24Hz"},
	{1280, 720, 25, ASPECT(16, 9), PROGRESSIVE, false,
		"61 720p25   16:9               1280x720p @ 25Hz"},
	{1280, 720, 30, ASPECT(16, 9), PROGRESSIVE, false,
		"62 720p30   16:9               1280x720p @ 29.97/30Hz"},
	{1920, 1080, 120, ASPECT(16, 9), PROGRESSIVE, false,
		"63 1080p120 16:9               1920x1080 @ 119.88/120Hz"},
};

/* device supported modes in CEA */
enum {
	CEA_MODE_640X480P_60HZ_4_3 = 0,
	CEA_MODE_720X480P_60HZ_4_3 = 1,
	CEA_MODE_720X480P_60HZ_16_9 = 2,
	CEA_MODE_1280X720P_60HZ_16_9 = 3,
	CEA_MODE_720X576P_50HZ_4_3 = 16,
	CEA_MODE_720X576P_50HZ_16_9 = 17,
};

/* device supported modes in established timing */
enum {
	ESTABLISHED_MODE_800X600_60HZ = 0,
	ESTABLISHED_MODE_640X480_60HZ = 5,
};

int init_edid_info(struct edid_info_struct *edid_info)
{
	edid_info->is_valid = false;
	mutex_init(&edid_info->access_lock);	

	return 0;
}

/* Byte 35-37 of block-0 */
static char *established_timing_str[] = {
       "800x600 @ 60 Hz",
       "800x600 @ 56 Hz",
       "640x480 @ 75 Hz",
       "640x480 @ 72 Hz",
       "640x480 @ 67 Hz",
       "640x480 @ 60 Hz",
       "720x400 @ 88 Hz",
       "720x400 @ 70 Hz",

       "1280x1024@75 Hz",
       "1024x768@75 Hz",
       "1024x768@70 Hz",
       "1024x768@60 Hz",
       "1024x768@87 Hz (Interlaced)",
       "832x624@75 Hz",
       "800x600@75 Hz",
       "800x600@72 Hz",

       "",
       "",
       "",
       "",
       "",
       "",
       "",
       "1152x870 @ 75 Hz",
};

/* E-EDID Video data block:  */
static char *vdb_modes_str[] = {
       " 1 DMT0659   4:3                640x480p @ 59.94/60Hz",
       " 2 480p      4:3                720x480p @ 59.94/60Hz",
       " 3 480pH    16:9                720x480p @ 59.94/60Hz",
       " 4 720p     16:9               1280x720p @ 59.94/60Hz",
       " 5 1080i    16:9              1920x1080i @ 59.94/60Hz",
       " 6 480i      4:3          720(1440)x480i @ 59.94/60Hz",
       " 7 480iH    16:9          720(1440)x480i @ 59.94/60Hz",
       " 8 240p      4:3          720(1440)x240p @ 59.94/60Hz",
       " 9 240pH    16:9          720(1440)x240p @ 59.94/60Hz",
       "10 480i4x    4:3             (2880)x480i @ 59.94/60Hz",
       "11 480i4xH  16:9             (2880)x480i @ 59.94/60Hz",
       "12 240p4x    4:3             (2880)x240p @ 59.94/60Hz",
       "13 240p4xH  16:9             (2880)x240p @ 59.94/60Hz",
       "14 480p2x    4:3               1440x480p @ 59.94/60Hz",
       "15 480p2xH  16:9               1440x480p @ 59.94/60Hz",
       "16 1080p    16:9              1920x1080p @ 59.94/60Hz",
       "17 576p      4:3                720x576p @ 50Hz",
       "18 576pH    16:9                720x576p @ 50Hz",
       "19 720p50   16:9               1280x720p @ 50Hz",
       "20 1080i25  16:9              1920x1080i @ 50Hz*",
       "21 576i      4:3          720(1440)x576i @ 50Hz",
       "22 576iH    16:9          720(1440)x576i @ 50Hz",
       "23 288p      4:3          720(1440)x288p @ 50Hz",
       "24 288pH    16:9          720(1440)x288p @ 50Hz",
       "25 576i4x    4:3             (2880)x576i @ 50Hz",
       "26 576i4xH  16:9             (2880)x576i @ 50Hz",
       "27 288p4x    4:3             (2880)x288p @ 50Hz",
       "28 288p4xH  16:9             (2880)x288p @ 50Hz",
       "29 576p2x    4:3               1440x576p @ 50Hz",
       "30 576p2xH  16:9               1440x576p @ 50Hz",
       "31 1080p50  16:9              1920x1080p @ 50Hz",
       "32 1080p24  16:9              1920x1080p @ 23.98/24Hz",
       "33 1080p25  16:9              1920x1080p @ 25Hz",
       "34 1080p30  16:9              1920x1080p @ 29.97/30Hz",
       "35 480p4x    4:3             (2880)x480p @ 59.94/60Hz",
       "36 480p4xH  16:9             (2880)x480p @ 59.94/60Hz",
       "37 576p4x    4:3             (2880)x576p @ 50Hz",
       "38 576p4xH  16:9             (2880)x576p @ 50Hz",
       "39 108Oi25  16:9  1920x1080i(1250 Total) @ 50Hz*",
       "40 1080i50  16:9              1920x1080i @ 100Hz",
       "41 720p100  16:9               1280x720p @ 100Hz",
       "42 576p100   4:3                720x576p @ 100Hz",
       "43 576p100H 16:9                720x576p @ 100Hz",
       "44 576i50    4:3          720(1440)x576i @ 100Hz",
       "45 576i50H  16:9          720(1440)x576i @ 100Hz",
       "46 1080i60  16:9              1920x1080i @ 119.88/120Hz",
       "47 720p120  16:9               1280x720p @ 119.88/120Hz",
       "48 480p119   4:3                720x480p @ 119.88/120Hz",
       "49 480p119H 16:9                720x480p @ 119.88/120Hz",
       "50 480i59    4:3          720(1440)x480i @ 119.88/120Hz",
       "51 480i59H  16:9          720(1440)x480i @ 119.88/120Hz",
       "52 576p200   4:3                720x576p @ 200Hz",
       "53 576p200H 16:9                720x576p @ 200Hz",
       "54 576i100   4:3          720(1440)x576i @ 200Hz",
       "55 576i100H 16:9          720(1440)x576i @ 200Hz",
       "56 480p239   4:3                720x480p @ 239.76/240Hz",
       "57 480p239H 16:9                720x480p @ 239.76/240Hz",
       "58 480i119   4:3          720(1440)x480i @ 239.76/240Hz",
       "59 480i119H 16:9          720(1440)x480i @ 239.76/240Hz",
       "60 720p24   16:9               1280x720p @ 23.98/24Hz",
       "61 720p25   16:9               1280x720p @ 25Hz",
       "62 720p30   16:9               1280x720p @ 29.97/30Hz",
       "63 1080p120 16:9               1920x1080 @ 119.88/120Hz",
};

int edid_dump_video_modes(u8 *edid_buf)
{
       int i, v1, v2, width, height, ret, aspect;
       char *str_aspect[] = { "16:10", "4:3", "5:4", "16:9" };

       switch (edid_buf[0]) {
       case 0:
               pr_info("block type 0: supported mode:\n");
               v1 = edid_buf[35] | edid_buf[36] << 8 | edid_buf[37] << 16;
               /* Established timing */
               pr_info("established timing: {%02x, %02x, %02x}\n",
                       edid_buf[35], edid_buf[36], edid_buf[37]);
               for (i = 0 ; i < 18; i++ ) {
                       v1 >>= 1;
                       if (v1 & 1) pr_info("%s\n", established_timing_str[i]);
               };

               pr_info("Standard timing identification:\n");
               /* Standard timing identification */
               for (i = 0; i < 8; i++) {
                       v1 = edid_buf[38+i*2];
                       v2 = edid_buf[38+i*2+1];
                       width = v1 * 8 + 248;
                       aspect = v2 >> 6;
                       switch (aspect) {
                       case 0: height = width * 10 / 16; break;
                       case 1: height = width * 3 / 4; break;
                       case 2: height = width * 4 / 5; break;
                       case 3: height = width * 9 / 16; break;
                       }
                       pr_info("%dx%d, %s, %d Hz\n", width, height,
                               str_aspect[aspect], (v2 & ~(3 << 6)) + 60);
               }
               ret = 0;
               break;
       case 2:
               pr_info("block type 2: supported mode:\n");
               pr_info("edid_buf[4]=%x\n", edid_buf[4]);
               for( i = 0; i < (edid_buf[4] & 0x1f); i++) {
                       pr_info("%s\n", vdb_modes_str[edid_buf[5+i] & 0x7f]);
               }
               ret = 0;
               break;

       default:
               ret = -EINVAL;
               break;
       }
       return ret;
}

bool edid_do_checksum(u8 *data)
{
	int i;
	u8 sum = 0;

	for (i = 0; i < EDID_BLOCK_SIZE; i++)
		sum += data[i];
	EDID_DBG("%s: result=%s\n", __func__, sum ? "fail" : "pass");
	return sum ? false : true;
}

static bool edid_check_header(u8 *data)
{
	int ret, i = 0;
	/* EDID 8 bytes header */
	static u8 header[] = {0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0};

	for (i = 0; i < ARRAY_SIZE(header); i++)
		if (data[i] != header[i])
			break;
	ret = (i == ARRAY_SIZE(header)) ? true : false;
	EDID_DBG("%s: result=%s\n", __func__, ret ? "pass" : "fail");

	return ret;
}

bool edid_check_hdmi_sink(struct hdmi_info *hdmi, int block)
{
	int ret = false, index = 4;
	u8 *data = &hdmi->edid_buf[block * 128];
	/* block offset where long descriptors start */
	int long_desc_offset = data[LONG_DESCR_PTR_IDX];
	int tag_code, data_block_len;

	while (index < long_desc_offset) {
		tag_code = (data[index] >> 5) & 0x7;
		data_block_len = data[index++] & 0x1f;
		if (tag_code == VENDOR_SPEC_D_BLOCK &&
		    data[index] == 0x03 &&
		    data[index + 1] == 0x0c &&
		    data[index + 2] == 0x00) {
			ret = true;
			break;
		} else
			index += data_block_len;
	}
	hdmi->edid_info.hdmi_sink = ret;
	EDID_DBG("%s: ret=%s\n", __func__, ret ? "yes" : "no");
	return ret;
}

struct edid_black_list_info {
	u8 mfr_model[4];
	u8 prefer_modes[3];
};

struct edid_black_list_info edid_black_list[] = {
	{ {0x4c, 0x2d, 0xa5, 0x02}, {0, 0, 0x40} },	// 720p only
	{ {0x4c, 0x2d, 0x0d, 0x05}, {0, 0, 0x40} },	// 720p only

	//{ {0x5a, 0x63, 0x20, 0x2b}, {0, 0, 0x40} },	// Viewsonic test
};

/* By comparing the Manufacture(0x8, 0x9) and Model field(0xa, 0xb) of EDID,
 * to check if the attached TV is a known less-compatibile one.
 */
int edid_fixup_compatibility_list(struct hdmi_info *hdmi)
{
	int i, ret = -1;

	/* FIXME: magic numbers...*/
	for (i = 0; i < ARRAY_SIZE(edid_black_list); i++) {
		if (!memcmp(hdmi->edid_buf + 8, edid_black_list[i].mfr_model, 4)){
#if 0
			EDID_DBG("%s: found in blacklist %d\n", __func__, i);
			EDID_DBG("%s: old timing = {%02x, %02x, %02x}\n",
				__func__,
				hdmi->edid_buf[35], hdmi->edid_buf[36],
				hdmi->edid_buf[37]);
			memcpy(hdmi->edid_buf + 35,
				edid_black_list[i].prefer_modes, 3);
			EDID_DBG("%s: new timing = {%02x, %02x, %02x}\n",
				__func__,
				hdmi->edid_buf[35], hdmi->edid_buf[36],
				hdmi->edid_buf[37]);
#else
			EDID_DBG("%s: found in compatibility %d\n", __func__, i);
			memcpy(hdmi->edid_buf + 35,
				edid_black_list[i].prefer_modes, 3);
#endif
			ret = i;
			break;
		}
	}

	return ret;
}

u8 edid_simple_parsing(struct hdmi_info *hdmi)
{
	u8 *edid_buf = hdmi->edid_buf;
	int i, index, ret = -EINVAL;
	struct edid_info_struct *edid_info = &hdmi->edid_info;
	unsigned v1, width, height, aspect;
	unsigned extensions;

	EDID_DBG("%s\n", __func__);
	// FIXME: integrate with edid_check()
	if (!edid_do_checksum(edid_buf)) {
		pr_err("%s: checksum error\n", __func__);
		//return EDID_CHECKSUM_ERROR;
	}
	if (!edid_check_header(edid_buf)) {
		pr_err("%s: incorrect header\n", __func__);
		return INCORRECT_EDID_HEADER;
	}
        edid_info->under_scan = ((edid_buf[MISC_SUPPORT_IDX]) >> 7) & 0x1;
        edid_info->basic_audio = ((edid_buf[MISC_SUPPORT_IDX]) >> 6) & 0x1;
        edid_info->ycbcr_4_4_4 = ((edid_buf[MISC_SUPPORT_IDX]) >> 5) & 0x1;
        edid_info->ycbcr_4_2_2 = ((edid_buf[MISC_SUPPORT_IDX]) >> 4) & 0x1;

	// FIXME: 0x7e
	extensions = edid_buf[0x7e];
	EDID_DBG("%s: extensions=%d\n", __func__, extensions);
	if (!extensions) {
		hdmi->edid_info.hdmi_sink = false;
		return NO_861_EXTENSIONS;
	}
	//return;

	/* reset all supported */
	for (i = 0 ; i < ARRAY_SIZE(additional_timing_db); i++)
		additional_timing_db[i].supported = false;
	for (i = 0 ; i < ARRAY_SIZE(established_timing_db); i++)
		established_timing_db[i].supported = false;

	/* Block 0: established timing */
	pr_info("established timing: {%02x, %02x, %02x}\n",
			edid_buf[35], edid_buf[36], edid_buf[37]);

	v1 = edid_buf[35] | edid_buf[36] << 8 | (!!edid_buf[37]) << 16;

	for (i = 0 ; i < 17; i++ )	// 17 bits defined in established timing
		established_timing_db[i].supported = ((v1 >>= 1) & 1) ;

#if 0
	/* standard timing identification */
	for (i = 0; i < 8; i++) {
		width = edid_buf[38 + i * 2] * 8 + 248;
		v1 = edid_buf[38 + i * 2 + 1];
		switch (v1 >> 6) {
		case 0: height = width * 10 / 16; aspect = ASPECT(16, 10); break;
		case 1: height = width * 3 / 4; aspect = ASPECT(4, 3); break;
		case 2: height = width * 4 / 5; aspect = ASPECT(5, 4); break;
		case 3: height = width * 9 / 16; aspect = ASPECT(16, 9); break;
		}
		standard_timing_db[i].width = width;
		standard_timing_db[i].height = height;
		standard_timing_db[i].aspect = aspect;
		standard_timing_db[i].refresh_rate = (v1 & ~(3 << 6)) + 60;
		standard_timing_db[i].supported = true;
	}
#endif

	if (extensions == 1) {
		EDID_DBG("Block-1: additional timing\n");
		/* Block-1: additional timing */
		for( i = 0; i < (edid_buf[128 + 4] & 0x1f); i++) {
			index = edid_buf[128 + 5 + i] & 0x7f;
			additional_timing_db[index-1].supported = true;
			EDID_DBG("%s\n", additional_timing_db[index-1].descrption);
			}
		edid_check_hdmi_sink(hdmi, 1);
	} else {
		EDID_DBG("Block-2: additional timing\n");
		for( i = 0; i < (edid_buf[384 + 4] & 0x1f); i++) {
			index = edid_buf[384 + 5 + i] & 0x7f;
			additional_timing_db[index-1].supported = true;
			EDID_DBG("%s\n", additional_timing_db[index-1].descrption);
			}
		edid_check_hdmi_sink(hdmi, 3);
	}

	edid_buf[35] = 0;
	edid_buf[36] = 0;
	edid_buf[37] = 0;

	/* edid_buf[37] bit4: 480p, bit5: 576p, bit6: 720p */
	if (additional_timing_db[CEA_MODE_720X480P_60HZ_4_3].supported ||
	    additional_timing_db[CEA_MODE_720X480P_60HZ_16_9].supported) {
		EDID_DBG("decide to support 480P\n");
		edid_buf[37] |= (1<<4);
	}

	if (additional_timing_db[CEA_MODE_720X576P_50HZ_4_3].supported ||
	    additional_timing_db[CEA_MODE_720X576P_50HZ_16_9].supported) {
		EDID_DBG("decide to support 576P\n");
		edid_buf[37] |= (1<<5);
	}

	if (additional_timing_db[CEA_MODE_1280X720P_60HZ_16_9].supported) {
		EDID_DBG("decide to support 720P\n");
		edid_buf[37] |= (1<<6);
	}

	if (established_timing_db[ESTABLISHED_MODE_800X600_60HZ].supported) {
		EDID_DBG("decide to support 800x600\n");
		edid_buf[36] |= (1<<6);
	}

	if (established_timing_db[ESTABLISHED_MODE_640X480_60HZ].supported) {
		EDID_DBG("decide to support 640x480\n");
		edid_buf[35] |= (1<<5);
	}
	edid_fixup_compatibility_list(hdmi);

	return ret;
}

// FIXME: modify the checking routines into inline function.
bool edid_is_video_mode_supported(struct video_mode *vmode)
{
	int i;
	struct video_mode *vmode_db;

	vmode_db = established_timing_db;
	for (i = 0, vmode_db = established_timing_db;
			i < ARRAY_SIZE(established_timing_db); i++)
		if ( (vmode->width == vmode_db[i].width) && 
		     (vmode->height == vmode_db[i].height) &&
		     (vmode->refresh_rate == vmode_db[i].refresh_rate ) &&
		     (vmode_db[i].interlaced == PROGRESSIVE) &&
		     (vmode_db[i].supported == true ))
			return true;
	for (i = 0, vmode_db = standard_timing_db;
			i < ARRAY_SIZE(standard_timing_db); i++)
		if ( (vmode->width == vmode_db[i].width) &&
		     (vmode->height == vmode_db[i].height) &&
		     (vmode->refresh_rate == vmode_db[i].refresh_rate ) &&
		     (vmode_db[i].interlaced == PROGRESSIVE) &&
		     (vmode_db[i].supported == true ))
			return true;
        for (i = 0, vmode_db = additional_timing_db;
                        i < ARRAY_SIZE(additional_timing_db); i++)
                if ( (vmode->width == vmode_db[i].width) &&
                     (vmode->height == vmode_db[i].height) &&
                     (vmode->refresh_rate == vmode_db[i].refresh_rate ) &&
                     (vmode_db[i].interlaced == PROGRESSIVE) &&
		     (vmode_db[i].supported == true ))
                        return true;
	return false;
}

bool edid_check_sink_type(struct hdmi_info *hdmi)
{
        EDID_DBG("%s: ret=%d\n", __func__, hdmi->edid_info.hdmi_sink);
        return hdmi->edid_info.hdmi_sink;
}

int edid_dump_hex(u8 *src, int src_size, char *output, int output_size)
{
        char line[80];
        static char hextab[] = "0123456789abcdef";
        int i, j, n = 0, v, len, offset, line_size;

        len = src_size;
        memset(line, ' ', 79);
        line[79] = '\0';
        offset = strlen("0000 | ");
        line_size = offset + 3 * 16 + 1;
        for (i = 0; i < len / 16 ; i++) {
                scnprintf(line, offset + 1, "%04x | ", (i << 4));
                for (j = 0; j < 16 ; j++) {
                        v = src[i * 16 + j];
                        line[offset + j * 3] = hextab[v / 16];
                        line[offset + j * 3 + 1] = hextab[v % 16];
                }
                line[line_size - 1] = '\n';
                strncpy(output+ n, line, line_size);
		if ((n + line_size) > output_size)
			break;
		else
                	n += line_size;
        }

        return n;
}

/*============================================================================*/
static ssize_t edid_dbg_open(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
        return 0;
}

static char hex_buff[2048];
static ssize_t edid_buffered_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
	int n;
	int extensions;
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

	extensions = hdmi->edid_buf[0x7e] + 1;
	edid_dump_video_modes(hdmi->edid_buf);// fixme: crashed
	if (extensions == 1)
		edid_dump_video_modes(hdmi->edid_buf + 128);
	else
		edid_dump_video_modes(hdmi->edid_buf + 384);
	//edid_simple_parsing(hdmi); // FIXME: crashed...
	n = edid_dump_hex(hdmi->edid_buf, (hdmi->edid_buf[0x7e] + 1) * 128,
		hex_buff, 2048);
        return simple_read_from_buffer(buf, count, ppos, hex_buff, n);
}

static struct file_operations edid_debugfs_fops[] = {
        {
                .open  = edid_dbg_open,
                .read  = edid_buffered_read,
                .write = NULL,
        }
};

// TODO: error handling
int edid_debugfs_init(struct hdmi_info *hdmi)
{
        //int ret;
	struct dentry *edid_dent;

        edid_dent = debugfs_create_dir("edid", hdmi->debug_dir);
        if (IS_ERR(edid_dent))
                return PTR_ERR(edid_dent);

        debugfs_create_file("hex_dump", 0444, edid_dent, hdmi,
                &edid_debugfs_fops[0]);

        return 0;
}

