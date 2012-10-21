/* arch/arm/mach-msm/include/mach/msm_qdsp6_audio.h
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef _MACH_MSM_QDSP6_Q6AUDIO_
#define _MACH_MSM_QDSP6_Q6AUDIO_

#define AUDIO_FLAG_READ		0
#define AUDIO_FLAG_WRITE	1

struct audio_buffer {
	dma_addr_t phys;
	void *data;
	uint32_t size;
	uint32_t used;	/* 1 = CPU is waiting for DSP to consume this buf */
};

struct audio_client {
	struct audio_buffer buf[2];
	int cpu_buf;	/* next buffer the CPU will touch */
	int dsp_buf;	/* next buffer the DSP will touch */
	int running;
	int session;
	
	int open_done;
	int open_status;
	wait_queue_head_t wait;
	struct dal_client *client;

	int cb_status;
	uint32_t flags;
};

#define Q6_HW_HANDSET	0
#define Q6_HW_HEADSET	1
#define Q6_HW_SPEAKER	2
#define Q6_HW_TTY	3
#define Q6_HW_BT_SCO	4
#define Q6_HW_BT_A2DP	5

#define Q6_HW_COUNT	6

#define DEVICE_ID_HANDSET_MIC      0
#define DEVICE_ID_SPKR_PHONE_MIC   1
#define DEVICE_ID_HEADSET_MIC      2
#define DEVICE_ID_TTY_HEADSET_MIC  3
#define DEVICE_ID_BT_SCO_MIC       4
#define DEVICE_ID_MIC_COUNT        5
#define MAX_MIC_LEVEL              1000

struct q6_hw_info {
	int min_gain;
	int max_gain;
};

/* Obtain a 16bit signed, interleaved audio channel of the specified
 * rate (Hz) and channels (1 or 2), with two buffers of bufsz bytes.
 */
struct audio_client *q6audio_open_pcm(uint32_t bufsz, uint32_t rate,
				      uint32_t channels, uint32_t flags,
				      uint32_t acdb_id);

struct audio_client *q6voice_open(uint32_t flags, uint32_t acdb_id);

struct audio_client *q6audio_open_mp3(uint32_t bufsz, uint32_t rate,
				      uint32_t channels, uint32_t acdb_id);

struct audio_client *q6fm_open(void);

struct audio_client *q6audio_open_aac(uint32_t bufsz, uint32_t rate,
				      uint32_t flags, void *data, uint32_t acdb_id);

struct audio_client *q6audio_open_qcelp(uint32_t bufsz, uint32_t rate,
				      void *data, uint32_t acdb_id);

int q6audio_close(struct audio_client *ac);
int q6voice_close(struct audio_client *ac);
int q6audio_mp3_close(struct audio_client *ac);
int q6fm_close(struct audio_client *ac);
int q6audio_aac_close(struct audio_client *ac);
int q6audio_qcelp_close(struct audio_client *ac);

int q6audio_read(struct audio_client *ac, struct audio_buffer *ab);
int q6audio_write(struct audio_client *ac, struct audio_buffer *ab);
int q6audio_async(struct audio_client *ac);

int q6audio_do_routing(uint32_t route, uint32_t acdb_id);
int q6audio_set_rx_mute(int mute);
int q6audio_set_tx_mute(int mute);
int q6audio_reinit_acdb(char* filename);
int q6audio_update_acdb(uint32_t id_src, uint32_t id_dst);
int q6audio_set_rx_volume(int level);
int q6audio_set_tx_volume(int mute);
int q6audio_set_stream_volume(struct audio_client *ac, int vol);
int q6audio_set_tx_dev_volume(int device_id, int level);
int q6audio_get_tx_dev_volume(int device_id);

struct q6audio_analog_ops {
	void (*init)(void);
	void (*speaker_enable)(int en);
	void (*headset_enable)(int en);
	void (*receiver_enable)(int en);
	void (*bt_sco_enable)(int en);
	void (*int_mic_enable)(int en);
	void (*ext_mic_enable)(int en);
	void (*i2s_enable)(int en);
	int (*get_rx_vol)(uint8_t hw, int level);
};

void q6audio_register_analog_ops(struct q6audio_analog_ops *ops);
void q6audio_set_acdb_file(char* filename);

#endif
