/* arch/arm/mach-msm/qdsp5v2/voice.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>

#include "../dal.h"

#include "adsp_audio.h"

#define VOICE_DAL_DEVICE	0x02000075
#define VOICE_DAL_PORT		"SMD_DAL00"

/* Commands sent to Modem */
#define CMD_VOICE_INIT                  0x1
#define CMD_ACQUIRE_DONE                0x2
#define CMD_RELEASE_DONE                0x3
#define CMD_DEVICE_INFO                 0x4
#define CMD_DEVICE_CHANGE               0x6

/* EVENTS received from MODEM */
#define EVENT_ACQUIRE_START             0x51
#define EVENT_RELEASE_START             0x52
#define EVENT_CHANGE_START              0x54
#define EVENT_NETWORK_RECONFIG          0x53

#define NETWORK_CDMA		0
#define NETWORK_GSM		1
#define NETWORK_WCDMA		2
#define NETWORK_WCDMA_WB	3

#define VOICE_DALRPC_CMD DAL_OP_FIRST_DEVICE_API

struct voice_header {
	uint32_t id;
	uint32_t data_len;
};

struct voice_init {
	struct voice_header hdr;
	void *cb_handle;
};

struct voice_device {
	struct voice_header hdr;
	uint32_t rx_device;
	uint32_t tx_device;
	uint32_t rx_volume;
	uint32_t rx_mute;
	uint32_t tx_mute;
	uint32_t rx_sample;
	uint32_t tx_sample;
};

struct voice_network {
	struct voice_header hdr;
	uint32_t network_info;
};

struct voice_event {
	/* common DAL event header */
	uint32_t evt_handle;
	uint32_t evt_cookie;
	uint32_t evt_length;

	/* voice event header */
	uint32_t id;
	uint32_t length;

	union {
		uint32_t network;
	} u;
};
	
struct msm_voice {
	struct dal_client *client;

	uint32_t next;
};

static struct msm_voice the_voice;

static int voice_cmd_acquire_done(struct msm_voice *voice)
{
	struct voice_header cmd;
	int rc;

	cmd.id = CMD_ACQUIRE_DONE;
	cmd.data_len = 0;

	return dal_call_f5(voice->client, VOICE_DALRPC_CMD,
			   &cmd, sizeof(cmd));
}

static void voice_work_func(struct work_struct *work)
{
	struct msm_voice *voice = &the_voice;
	struct voice_device cmd;
	int rc;

	pr_info("voice: doing work...\n");

	switch (voice->next) {
	case EVENT_ACQUIRE_START:
		audio_route_path("handset");

		cmd.hdr.id = CMD_DEVICE_INFO;
		cmd.hdr.data_len = sizeof(cmd) - sizeof(cmd.hdr);
		cmd.rx_device = 1;
		cmd.tx_device = 2;
		cmd.rx_volume = -500; /* millibels */
		cmd.tx_mute = 0;
		cmd.rx_mute = 0;
		cmd.rx_sample = 48000 / 1000;
		cmd.tx_sample = 48000 / 1000;

		rc = dal_call_f5(voice->client,
				 VOICE_DALRPC_CMD,
				 &cmd, sizeof(cmd));
		if (rc < 0) {
			pr_err("voice: device info failed\n");
		}

		rc = voice_cmd_acquire_done(voice);
		break;

	case EVENT_RELEASE_START:
		audio_route_path("speaker");
		break;
	}
}

static DECLARE_WORK(voice_work, voice_work_func);

static void voice_dal_callback(void *data, int len, void *cookie)
{
	struct msm_voice *voice = cookie;
	struct voice_event *evt = data;

	voice->next = evt->id;

	switch (evt->id) {
	case EVENT_ACQUIRE_START:
		pr_info("voice: ACQUIRE_START (net %d)\n",
			evt->u.network);
		break;
	case EVENT_RELEASE_START:
		pr_info("voice: RELEASE_START\n");

		break;
	case EVENT_CHANGE_START:
		pr_info("voice: CHANGE_START\n");
		break;
	case EVENT_NETWORK_RECONFIG:
		pr_info("voice: NETWORK_RECONFIG\n");
		break;
	};

	schedule_work(&voice_work);
}

static int voice_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t voice_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	return count;
}

static const struct file_operations voice_fops = {
	.owner		= THIS_MODULE,
	.open		= voice_open,
	.write		= voice_write,
};

struct miscdevice voice_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "voice",
	.fops	= &voice_fops,
};

int msm_voice_init(void)
{
	struct msm_voice *voice = &the_voice;
	struct voice_init cmd;
	int rc;

	pr_info("voice: init()\n");

	voice->client = dal_attach(VOICE_DAL_DEVICE,
				   VOICE_DAL_PORT,
				   voice_dal_callback,
				   voice);

	if (!voice->client) {
		pr_err("voice: cannot attach to service\n");
		return -ENODEV;
	}

	cmd.hdr.id = CMD_VOICE_INIT;
	cmd.hdr.data_len = sizeof(cmd) - sizeof(cmd.hdr);
	cmd.cb_handle = NULL;
	rc = dal_call_f5(voice->client, VOICE_DALRPC_CMD, &cmd, sizeof(cmd));

	if (rc < 0) {
		pr_err("voice: init failed\n");
		return -ENODEV;
	}

	return misc_register(&voice_misc);
}
