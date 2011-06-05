#ifndef _EDID_H_
#define _EDID_H_

#define MAX_VIDEO_MODES			32
struct edid_info_struct {
        bool            is_valid;
        struct          mutex access_lock;
        bool under_scan;
        bool basic_audio;
        bool ycbcr_4_4_4;
        bool ycbcr_4_2_2;
        bool hdmi_sink;
};

#define EDID_BLOCK_SIZE			128
#define EDID_HDR_NO_OF_FF   		0x06

#define EDID_BLOCK_0_OFFSET		0x00
#define EDID_BLOCK_1_OFFSET		0x80
#define EDID_BLOCK_SIZE			128
#define EDID_HDR_NO_OF_FF		0x06
#define NUM_OF_EXTEN_ADDR		0x7E

#define EDID_TAG_ADDR       		0x00
#define EDID_REV_ADDR       		0x01
#define EDID_TAG_IDX        		0x02
#define LONG_DESCR_PTR_IDX  		0x02
#define MISC_SUPPORT_IDX    		0x03

#define ESTABLISHED_TIMING_INDEX        35
#define NUM_OF_STANDARD_TIMINGS		8
#define STANDARD_TIMING_OFFSET		38
#define LONG_DESCR_LENi			18
#define NUM_OF_DETAILED_DESCRIPTORS	4

#define DETAILED_TIMING_OFFSET		0x36

/* Offsets within a Long Descriptors Block */
#define PIX_CLK_OFFSET			0
#define H_ACTIVE_OFFSET			2
#define H_BLANKING_OFFSET		3
#define V_ACTIVE_OFFSET			5
#define V_BLANKING_OFFSET		6
#define H_SYNC_OFFSET			8
#define H_SYNC_PW_OFFSET		9
#define V_SYNC_OFFSET			10
#define V_SYNC_PW_OFFSET		10
#define H_IMAGE_SIZE_OFFSET		12
#define V_IMAGE_SIZE_OFFSET		13
#define H_BORDER_OFFSET			15
#define V_BORDER_OFFSET			16
#define FLAGS_OFFSET			17

#define AR16_10				0
#define AR4_3				1
#define AR5_4				2
#define AR16_9				3

#define EDID_EXTENSION_TAG		0x02
#define EDID_REV_THREE			0x03
#define EDID_DATA_START			0x04

#define EDID_BLOCK_0			0x00
#define EDID_BLOCK_2_3			0x01

#define AUDIO_DESCR_SIZE		3

/* Data Block Tag Codes */
#define AUDIO_D_BLOCK			0x01
#define VIDEO_D_BLOCK			0x02
#define VENDOR_SPEC_D_BLOCK		0x03
#define SPKR_ALLOC_D_BLOCK		0x04
#define USE_EXTENDED_TAG		0x07

/* Extended Data Block Tag Codes */
#define COLORIMETRY_D_BLOCK		0x05

#define VIDEO_CAPABILITY_D_BLOCK	0x00
#define HDMI_SIGNATURE_LEN		0x03

#define CEC_PHYS_ADDR_LEN		0x02
#endif
