/*
 * HTC mode Server Header
 *
 * Copyright (C) 2011 HTC Corporation
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
#ifndef _HTC_MODE_SERVER_H_
#define _HTC_MODE_SERVER_H_

#include <linux/completion.h>

#define HSML_PROTOCOL_VERSION		0x0006

#define HTC_MODE_CONTROL_REQ		0x12


#define CLIENT_INFO_SERVER_ROTATE_USED		(1 << 1)

#define PIXEL_FORMAT_RGB565					(1 << 0)

#define CTRL_CONF_TOUCH_EVENT_SUPPORTED		(1 << 0)
#define CTRL_CONF_NUM_SIMULTANEOUS_TOUCH	(4 << 1)


#define HSML_SERVER_NONCE_SIZE 		20
#define HSML_CLIENT_SIG_SIZE		168
#define HSML_SERVER_SIG_SIZE		148

enum {
	CLIENT_INFO_MESGID = 0,
	SERVER_INFO_MESGID,
	TOUCH_EVENT_MESGID,
	AUTH_SERVER_NONCE_MESGID,
	AUTH_CLIENT_NONCE_MESGID,
	AUTH_RESPONSE_MESGID,
	OBU_INFO_MESGID
};

enum {
	HSML_FB_HEADER_ID = 0,
	HSML_TOUCH_EVENT_ID,
	HSML_KEY_EVENT_ID
};

enum {
	FB_HEADER_MSGID = 0,

};

enum {
	HSML_REQ_GET_SERVER_VERSION = 0x40,
	HSML_REQ_SEND_CLIENT_INFO,
	HSML_REQ_GET_SERVER_INFO,
	HSML_REQ_GET_FB,
	HSML_REQ_STOP,
	HSML_REQ_SEND_EXTENDED_CLIENT_INFO,
	HSML_REQ_GET_SERVER_NONCE,
	HSML_REQ_SET_CLIENT_AUTH,
	HSML_REQ_GET_SERVER_AUTH,

	HSML_REQ_COUNT
};


struct msm_client_info {
	u8 mesg_id;
	u16 width;
	u16 height;
	u32 display_conf;
	u32 pixel_format;
	u32 ctrl_conf;
} __attribute__ ((__packed__));

struct msm_server_info {
	u8 mesg_id;
	u16 width;
	u16 height;
	u32 pixel_format;
	u32 ctrl_conf;
} __attribute__ ((__packed__));


struct htcmode_protocol {
	u16 version;
	u16 vendor;
	u8 request;
	struct msm_client_info client_info;
	struct msm_server_info server_info;
	char nonce[HSML_SERVER_NONCE_SIZE];
	u8 client_sig[HSML_CLIENT_SIG_SIZE];
	u8 server_sig[HSML_SERVER_SIG_SIZE];
	u8 notify_authenticator;
	u8 auth_in_progress;
	u8 auth_result;
};

struct hsml_header {
	u8 msg_id;
	u16 x;
	u16 y;
	u16 w;
	u16 h;
	u8 signature[8];
	u8 reserved[494];
	u8 checksum;
} __attribute__ ((__packed__));

struct touch_content {
	u16 x;
	u16 y;
	u8 event_id;
	u8 pressure;
} __attribute__ ((__packed__));

struct touch_event {
	u8 mesg_id;
    u8 num_touch;
} __attribute__ ((__packed__));

struct key_event {
	u8 mesg_id;
    u8 down;
    u32 code;
} __attribute__ ((__packed__));

extern int msmfb_get_var(struct msm_fb_info *tmp);
extern void msmfb_set_var(unsigned char *addr, int area);
extern int msmfb_get_fb_area(void);

#endif /* _HTC_MODE_SERVER_H_ */
