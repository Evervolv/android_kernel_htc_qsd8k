/* arch/arm/mach-msm/qdsp6_10/q6audio.c
 *
 * Copyright (C) 2010 Cotulla
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

#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>

#include "dal.h"
#include "dal_audio.h"
#include "dal_audio_format.h"
#include "dal_acdb.h"
#include "dal_adie.h"
#include <mach/msm_qdsp6_audio_1550.h>
#include <linux/msm_audio_1550.h>
#include <mach/htc_acoustic_qsd.h>
#include <mach/msm_audio_qcp.h>

#include <linux/gpio.h>

#include "q6audio_devices.h"
#include "../dex_comm.h"

#if 0
#define TRACE(x...) pr_info("Q6: "x)
#else
#define TRACE(x...) do{}while(0)
#endif

#if 0
#define AUDIO_INFO(x...) pr_info("Audio: "x)
#else
#define AUDIO_INFO(x...) do{}while(0)
#endif


#define CB_EVENT_COOKIE		0xC00CE13E 

struct mic_level{
	int device_id;
	int level;
};

// same gain range is used for every mic
static int min_mic_gain = -5000;
static int max_mic_gain = 1500;
/*
* level range 0 - 1000
* level -1 -> level is not defined by user so default value(gain) is set through acbd firmware
*/
static struct mic_level user_mic_levels[DEVICE_ID_MIC_COUNT] =
{
	[0] = 
		{
			.device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_MIC,
			.level = -1
		},
	[1] = 
		{
			.device_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC,
			.level = -1
		},
	[2] = 
		{
			.device_id = ADSP_AUDIO_DEVICE_ID_HEADSET_MIC,
			.level = -1
		},
	[3] = 
		{
			.device_id = ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC,
			.level = -1
		},
	[4] = 
		{
			.device_id = ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC,
			.level = -1
		}
};

#if 1

// LEO
static struct q6_hw_info q6_audio_hw[Q6_HW_COUNT] =
{
    [Q6_HW_HANDSET] =
	{
        .min_gain = -2000,
        .max_gain = 600,
    },
    [Q6_HW_HEADSET] =
	{
        .min_gain = -2000,
        .max_gain = 600,
    },
    [Q6_HW_SPEAKER] =
	{
        .min_gain = -1500,
        .max_gain = 500,
    },
    [Q6_HW_TTY] =
	{
        .min_gain = -2000,
        .max_gain = 600,
    },
    [Q6_HW_BT_SCO] =
	{
        .min_gain = -1100,
        .max_gain = 400,
    },
    [Q6_HW_BT_A2DP] =
	{
        .min_gain = -1100,
        .max_gain = 400,
    },
};

#else

// Old one (from Desire)
static struct q6_hw_info q6_audio_hw[Q6_HW_COUNT] =
{
    [Q6_HW_HANDSET] = {
        .min_gain = -2000,
        .max_gain = 0,
    },
    [Q6_HW_HEADSET] = {
        .min_gain = -2000,
        .max_gain = 0,
    },
    [Q6_HW_SPEAKER] = {
        .min_gain = -1500,
        .max_gain = 0,
    },
    [Q6_HW_TTY] = {
        .min_gain = -2000,
        .max_gain = 0,
    },
    [Q6_HW_BT_SCO] = {
        .min_gain = -2000,
        .max_gain = 0,
    },
    [Q6_HW_BT_A2DP] = {
        .min_gain = -2000,
        .max_gain = 0,
    },
};
#endif

extern int global_now_phone_call;

static struct audio_client * audio_test(void);
static void callback(void *data, int len, void *cookie);
static int audio_init(struct audio_client *ac);
static int audio_info(struct audio_client *ac);
static int q6audio_init_rx_volumes(void);

static struct wake_lock wakelock;
static struct wake_lock idlelock;
static int idlecount;
static DEFINE_MUTEX(idlecount_lock);

void audio_prevent_sleep(void)
{
    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&idlecount_lock);
    if (++idlecount == 1) {
        wake_lock(&wakelock);
        wake_lock(&idlelock);
    }
    mutex_unlock(&idlecount_lock);
}

void audio_allow_sleep(void)
{
    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&idlecount_lock);
    if (--idlecount == 0) {
        wake_unlock(&idlelock);
        wake_unlock(&wakelock);
    }
    mutex_unlock(&idlecount_lock);
}

static struct clk *icodec_rx_clk;
static struct clk *icodec_tx_clk;
static struct clk *ecodec_clk;
static struct clk *sdac_clk;

static struct q6audio_analog_ops default_analog_ops;
static struct q6audio_analog_ops *analog_ops = &default_analog_ops;
static uint32_t tx_clk_freq = 8000;
static int tx_mute_status = 0;
static int rx_vol_level = 100;
static char acdb_file[64] = "default.acdb";
static uint32_t tx_acdb = 0;
static uint32_t rx_acdb = 0;
static int acdb_use_rpc = 0;
static int acdb_use_map = 0;

/////////////////////////////////////////////////////////////////////////////////
// helper functions for device parameters
/////////////////////////////////////////////////////////////////////////////////

void q6audio_register_analog_ops(struct q6audio_analog_ops *ops)
{
    AUDIO_INFO("%s\n", __func__);
    pr_info("register analog ops = %p\n", ops);
    analog_ops = ops;
}

void q6audio_set_acdb_file(char* filename)
{
    AUDIO_INFO("%s\n", __func__);
    if (filename) {
        pr_info("audio: set acdb file as %s\n", filename);
        strncpy(acdb_file, filename, sizeof(acdb_file)-1);
    }
}

static struct q6_device_info *q6_lookup_device(uint32_t device_id)
{
    struct q6_device_info *di = q6_audio_devices;
    AUDIO_INFO("%s\n", __func__);
    for (;;) {
        if (di->id == device_id)
            return di;
        if (di->id == 0) {
            pr_err("q6_lookup_device: bogus id 0x%08x\n",
                   device_id);
            return di;
        }
        di++;
    }
}

static uint32_t q6_device_to_codec(uint32_t device_id)
{
    struct q6_device_info *di = q6_lookup_device(device_id);
    AUDIO_INFO("%s\n", __func__);
    return di->codec;
}

static uint32_t q6_device_to_dir(uint32_t device_id)
{
    struct q6_device_info *di = q6_lookup_device(device_id);
    AUDIO_INFO("%s\n", __func__);
    return di->dir;
}

static uint32_t q6_device_to_cad_id(uint32_t device_id)
{
    struct q6_device_info *di = q6_lookup_device(device_id);
    AUDIO_INFO("%s\n", __func__);
    return di->cad_id;
}

static uint32_t q6_device_to_path(uint32_t device_id)
{
    struct q6_device_info *di = q6_lookup_device(device_id);
    AUDIO_INFO("%s\n", __func__);
    return di->path;
}

static uint32_t q6_device_to_rate(uint32_t device_id)
{
    struct q6_device_info *di = q6_lookup_device(device_id);
    AUDIO_INFO("%s\n", __func__);
    return di->rate;
}

int q6_device_volume(uint32_t device_id, int level)
{
    struct q6_device_info *di = q6_lookup_device(device_id);
    AUDIO_INFO("%s\n", __func__);
    if (analog_ops->get_rx_vol)
        return analog_ops->get_rx_vol(di->hw, level);
    else {
        struct q6_hw_info *hw;
        hw = &q6_audio_hw[di->hw];
        return hw->min_gain + ((hw->max_gain - hw->min_gain) * level) / 100;
    }
}

/////////////////////////////////////////////////////////////////////////////////
// ADIE functions
/////////////////////////////////////////////////////////////////////////////////

static inline int adie_open(struct dal_client *client) 
{
    return dal_call_f0(client, DAL_OP_OPEN, 0);
}

static inline int adie_close(struct dal_client *client) 
{
    return dal_call_f0(client, DAL_OP_CLOSE, 0);
}

static inline int adie_set_path(struct dal_client *client,
                uint32_t id, uint32_t path_type)
{
    AUDIO_INFO("%s\n", __func__);
    return dal_call_f1(client, ADIE_OP_SET_PATH, id, path_type);
}

static inline int adie_set_path_freq_plan(struct dal_client *client,
                                         uint32_t path_type, uint32_t plan)
{
    AUDIO_INFO("%s\n", __func__);
    return dal_call_f1(client, ADIE_OP_SET_PATH_FREQUENCY_PLAN,
               path_type, plan);
}

static inline int adie_proceed_to_stage(struct dal_client *client,
                    uint32_t path_type, uint32_t stage)
{
    AUDIO_INFO("%s\n", __func__);
    return dal_call_f1(client, ADIE_OP_PROCEED_TO_STAGE,
               path_type, stage);
}

static inline int adie_mute_path(struct dal_client *client,
                 uint32_t path_type, uint32_t mute_state)
{
    AUDIO_INFO("%s\n", __func__);
    return dal_call_f1(client, ADIE_OP_MUTE_PATH, path_type, mute_state);
}

static int adie_refcount;

static struct dal_client *adie;
static struct dal_client *adsp;
static struct dal_client *acdb;

static int adie_enable(void)
{
    AUDIO_INFO("%s\n", __func__);
    adie_refcount++;
    if (adie_refcount == 1)
        adie_open(adie);
    return 0;
}

static int adie_disable(void)
{
    AUDIO_INFO("%s\n", __func__);
    adie_refcount--;
    if (adie_refcount == 0)
        adie_close(adie);
    return 0;
}

/* 4k DMA scratch page used for exchanging acdb device config tables
 * and stream format descriptions with the DSP.
 */
static void *audio_data;
static dma_addr_t audio_phys;
// this memory used to pass open params into DSP
static void *params_data;
static dma_addr_t params_phys;


#define SESSION_MIN 0
#define SESSION_MAX 64

static DEFINE_MUTEX(session_lock);
static DEFINE_MUTEX(audio_lock);
static DEFINE_MUTEX(open_mem_lock);


static struct audio_client *session[SESSION_MAX];

static int session_alloc(struct audio_client *ac)
{
    int n;

    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&session_lock);
    for (n = SESSION_MIN; n < SESSION_MAX; n++) {
        if (!session[n]) {
            session[n] = ac;
            mutex_unlock(&session_lock);
            return n;
        }
    }
    mutex_unlock(&session_lock);
    return -ENOMEM;
}

static void session_free(int n, struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&session_lock);
    if (session[n] == ac)
        session[n] = 0;
    mutex_unlock(&session_lock);
}

void audio_client_dump(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    dal_trace_dump(ac->client);
}


static void audio_client_free(struct audio_client *ac)
{
    AUDIO_INFO("%s: session %d\n", __func__, ac->session);

    session_free(ac->session, ac);

    if (ac->buf[0].data)
        dma_free_coherent(NULL, ac->buf[0].size, ac->buf[0].data, ac->buf[0].phys);
    if (ac->buf[1].data)
        dma_free_coherent(NULL, ac->buf[1].size, ac->buf[1].data, ac->buf[1].phys);

    if (ac->client)
        dal_detach(ac->client);
    kfree(ac);
}

static struct audio_client *audio_client_alloc(unsigned bufsz)
{
    struct audio_client *ac;
    struct dal_client *dsp;
    int n;

    AUDIO_INFO("%s\n", __func__);
    ac = kzalloc(sizeof(*ac), GFP_KERNEL);
    if (!ac)
    {
        AUDIO_INFO("%s: alloc error\n", __func__);
        return 0;
    }

    n = session_alloc(ac);
    if (n < 0)
    {
	    AUDIO_INFO("%s: session alloc error\n", __func__);
        goto fail_session;
    }
    ac->session = n;
    AUDIO_INFO("%s: session %d\n", __func__, ac->session);

    if (bufsz > 0) 
    {
        ac->buf[0].data = dma_alloc_coherent(NULL, bufsz, &ac->buf[0].phys, GFP_KERNEL);
        if (!ac->buf[0].data)
        {
            goto fail;
        }

        ac->buf[1].data = dma_alloc_coherent(NULL, bufsz, &ac->buf[1].phys, GFP_KERNEL);
        if (!ac->buf[1].data)
        {
            goto fail;
        }

        ac->buf[0].size = bufsz;
        ac->buf[1].size = bufsz;
    }

    init_waitqueue_head(&ac->wait);


//    dsp = dal_attach(AUDIO_DAL_DEVICE, AUDIO_DAL_PORT, callback, 0);
    dsp = dal_attach_ex(AUDIO_DAL_DEVICE, "AudioQdsp", AUDIO_DAL_PORT, callback, 0);
    if (!dsp) 
    {
        pr_err("audio_client_alloc: cannot attach to adsp\n");
        goto fail;
    }
    ac->client = dsp;

    audio_init(ac);
    audio_info(ac);
    audio_client_dump(ac);

    return ac;

fail:
    if (ac->buf[0].data)
        dma_free_coherent(NULL, ac->buf[0].size, ac->buf[0].data, ac->buf[0].phys);
    if (ac->buf[1].data)
        dma_free_coherent(NULL, ac->buf[1].size, ac->buf[1].data, ac->buf[1].phys);    
    session_free(n, ac);
fail_session:
    audio_client_free(ac);
    return 0;
}


static int audio_ioctl(struct audio_client *ac, uint32_t cmd, void *ptr, uint32_t len)
{
    int r;
    AUDIO_INFO("%s\n", __func__);
    r = dal_call_f6(ac->client, AUDIO_OP_IOCTL, cmd, ptr, len);
    return r;
}

static int audio_command(struct audio_client *ac, uint32_t cmd)
{
    struct adsp_command_hdr rpc;
    AUDIO_INFO("%s %x\n", __func__, cmd);
    memset(&rpc, 0, sizeof(rpc));
    return audio_ioctl(ac, cmd, NULL, 0);
}

static int audio_open_control(struct audio_client *ac)
{
    int r;
    struct adsp_open_command rpc;
 
    AUDIO_INFO("%s\n", __func__);
    memset(&rpc, 0, sizeof(rpc));

    rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_DEV;

    // reset flag before call, callback() will set it later
    ac->open_done = 0;
    r = dal_call_f5(ac->client, AUDIO_OP_OPEN, &rpc, sizeof(rpc));
    if (r != 0)    
        return r;

// wait for async event
    if (!wait_event_timeout(ac->wait, (ac->open_done == 1), 3 * HZ))
    {
        pr_err("wait for open async event failed!\n");
        return -1;
    }
    return ac->open_status;
}

static int audio_out_open(struct audio_client *ac, uint32_t bufsz,
              uint32_t rate, uint32_t channels)
{
    int r;
    struct adsp_open_command rpc;
    union adsp_audio_format *ptr;

    AUDIO_INFO("%s\n", __func__);
    printk("audio_out_open: %x %d %d\n", bufsz, rate, channels);
    mutex_lock(&open_mem_lock);

    memset(&rpc, 0, sizeof(rpc));
    ptr = (union adsp_audio_format*)params_data;
    memset(ptr, 0, sizeof(union adsp_audio_format));

    rpc.numdev = 1;
//    rpc.dev[0] = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO;
    rpc.dev[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;
    rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
    rpc.buf_max_size = bufsz;

    rpc.format = ADSP_AUDIO_FORMAT_PCM;
    rpc.pblock = params_phys;
    rpc.blocklen = sizeof(struct adsp_audio_standard_format);
//    rpc.pblock = 0;
//    rpc.blocklen = 0;

    rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_WRITE;

    ptr->standard.channels = channels;
    ptr->standard.bits_per_sample = 16;
    ptr->standard.sampling_rate = rate;
    ptr->standard.is_signed = 1;
    ptr->standard.is_interleaved = 1;

    AUDIO_INFO("open out %p\n", ac);
    ac->open_done = 0;
    r = dal_call_f5(ac->client, AUDIO_OP_OPEN, &rpc, sizeof(rpc));
    
// wait for async event
    if (!wait_event_timeout(ac->wait, (ac->open_done == 1), 3 * HZ))
    {
        mutex_unlock(&open_mem_lock);
        pr_err("wait for open async event failed!\n");
        return -1;
    }
    mutex_unlock(&open_mem_lock);
//    return r;
    return ac->open_status;
}
static void update_all_audio_tx_volume(struct audio_client *ac);

static int audio_in_open(struct audio_client *ac, uint32_t bufsz,
#ifdef CONFIG_QSD_AUDIO_CALLREC
			uint32_t flags,
#endif
             uint32_t rate, uint32_t channels)
{
    int r;
    struct adsp_open_command rpc;
    union adsp_audio_format *ptr;

    AUDIO_INFO("%s\n", __func__);
    printk("audio_in_open: %x %d %d\n", bufsz, rate, channels);
    mutex_lock(&open_mem_lock);
    memset(&rpc, 0, sizeof(rpc));
    ptr = (union adsp_audio_format*)params_data;
    memset(ptr, 0, sizeof(union adsp_audio_format));

    rpc.numdev = 1;
    rpc.dev[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;
#ifdef CONFIG_QSD_AUDIO_CALLREC
	if (flags == AUDIO_FLAG_READ)
		rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_RECORD;
	else
		rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_MIXED_RECORD;
#else
    rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_RECORD;
#endif
    rpc.buf_max_size = bufsz;

    rpc.format = ADSP_AUDIO_FORMAT_PCM;
    rpc.pblock = params_phys;
    rpc.blocklen = sizeof(struct adsp_audio_standard_format);
    rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_READ;

    ptr->standard.channels = channels;
    ptr->standard.bits_per_sample = 16;
    ptr->standard.sampling_rate = rate;
    ptr->standard.is_signed = 1;
    ptr->standard.is_interleaved = 1;

    AUDIO_INFO("%p: open in\n", ac);
    ac->open_done = 0;
    r = dal_call_f5(ac->client, AUDIO_OP_OPEN, &rpc, sizeof(rpc));
// wait for async event
    if (!wait_event_timeout(ac->wait, (ac->open_done == 1), 3 * HZ))
    {
        mutex_unlock(&open_mem_lock);
        pr_err("wait for open async event failed (IN)!\n");
        return -1;
    }
    mutex_unlock(&open_mem_lock);
//    return r;
    return ac->open_status;
}

static int audio_mp3_open(struct audio_client *ac, uint32_t bufsz,
              uint32_t rate, uint32_t channels)
{
    int r;
    struct adsp_open_command rpc;
    union adsp_audio_format *ptr;

    AUDIO_INFO("%s\n", __func__);
    printk("audio_mp3_open: %x %d %d\n", bufsz, rate, channels);
    mutex_lock(&open_mem_lock);
    memset(&rpc, 0, sizeof(rpc));
    ptr = (union adsp_audio_format*)params_data;
    memset(ptr, 0, sizeof(union adsp_audio_format));

    rpc.numdev = 1;
    rpc.dev[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;
    rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
    rpc.buf_max_size = bufsz;

    rpc.format = ADSP_AUDIO_FORMAT_MP3;
    rpc.pblock = params_phys;
    rpc.blocklen = sizeof(struct adsp_audio_standard_format);
    rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_WRITE;

    ptr->standard.channels = channels;
    ptr->standard.bits_per_sample = 16;
    ptr->standard.sampling_rate = rate;
    ptr->standard.is_signed = 1;
    ptr->standard.is_interleaved = 1;

    r = dal_call_f5(ac->client, AUDIO_OP_OPEN, &rpc, sizeof(rpc));
// wait for async event
    if (!wait_event_timeout(ac->wait, (ac->open_done == 1), 3 * HZ))
    {
    mutex_unlock(&open_mem_lock);
        pr_err("wait for open async event failed (MP3)!\n");
        return -1;
    }
    mutex_unlock(&open_mem_lock);
//    return r;
    return ac->open_status;
}

static int audio_aac_open(struct audio_client *ac, uint32_t bufsz, void *data)
{
    int r;
    struct aac_format *af = data;
    struct adsp_open_command rpc;
    uint32_t *aac_type;
    int idx = 0;  // sizeof(uint32_t);
    struct adsp_audio_binary_format *fmt = (struct adsp_audio_binary_format *)params_data;
    union adsp_audio_format *ptr;


    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&open_mem_lock);
    memset(&rpc, 0, sizeof(rpc));
    ptr = (union adsp_audio_format*)params_data;
    memset(ptr, 0, sizeof(union adsp_audio_format));

    rpc.numdev = 1;
    rpc.dev[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;

    rpc.format = ADSP_AUDIO_FORMAT_MPEG4_AAC;
    rpc.pblock = params_phys;

    aac_type = (uint32_t *)(fmt->data);
    switch (af->block_formats) 
    {
    case 0xffff:
        if (ac->flags & AUDIO_FLAG_WRITE)
            *aac_type = ADSP_AUDIO_AAC_ADTS;
        else
            *aac_type = ADSP_AUDIO_AAC_MPEG4_ADTS;
        break;
    case 0:
        if (ac->flags & AUDIO_FLAG_WRITE)
            *aac_type = ADSP_AUDIO_AAC_ADIF;
        else
            *aac_type = ADSP_AUDIO_AAC_RAW;
        break;
    case 1:
        *aac_type = ADSP_AUDIO_AAC_RAW;
        break;
    case 2:
        *aac_type = ADSP_AUDIO_AAC_LOAS;
        break;
    case 3:
        *aac_type = ADSP_AUDIO_AAC_FRAMED_RAW;
        break;
    case 4:
        *aac_type = ADSP_AUDIO_AAC_RAW;
        break;
    default:
        pr_err("unsupported AAC type %d\n", af->block_formats);
        mutex_unlock(&open_mem_lock);
        return -EINVAL;
    }

    AUDIO_INFO("aac_open: type %x, obj %d, idx %d\n",
          *aac_type, af->audio_object_type, idx);
    fmt->data[idx++] = (u8)(((af->audio_object_type & 0x1F) << 3) |
                ((af->sample_rate >> 1) & 0x7));
    fmt->data[idx] = (u8)(((af->sample_rate & 0x1) << 7) |
                ((af->channel_config & 0x7) << 3));

    switch (af->audio_object_type) {
    case AAC_OBJECT_ER_LC:
    case AAC_OBJECT_ER_LTP:
    case AAC_OBJECT_ER_LD:
        /* extension flag */
        fmt->data[idx++] |= 0x1;
        fmt->data[idx] = (u8)(
            ((af->aac_section_data_resilience_flag & 0x1) << 7) |
            ((af->aac_scalefactor_data_resilience_flag & 0x1) << 6) |
            ((af->aac_spectral_data_resilience_flag & 0x1) << 5) |
            ((af->ep_config & 0x3) << 2));
        break;

    case AAC_OBJECT_ER_SCALABLE:
        fmt->data[idx++] |= 0x1;
        /* extension flag */
        fmt->data[idx++] = (u8)(
            ((af->aac_section_data_resilience_flag & 0x1) << 4) |
            ((af->aac_scalefactor_data_resilience_flag & 0x1) << 3) |
            ((af->aac_spectral_data_resilience_flag & 0x1) << 2) |
            ((af->ep_config >> 1) & 0x1));
        fmt->data[idx] = (u8)((af->ep_config & 0x1) << 7);
        break;

    case AAC_OBJECT_BSAC:
        fmt->data[++idx] = (u8)((af->ep_config & 0x3) << 6);
        break;

    default:
        pr_err("dbg unknown object type \n");
        break;
    }
//    fmt->num_bytes = idx + 1;
    rpc.blocklen = idx + 1;

    TRACE("aac_open: format %x%x %x%x%x%x %x%x, \n",
          fmt->data[0], fmt->data[1], fmt->data[2], fmt->data[3],
          fmt->data[4], fmt->data[5], fmt->data[6], fmt->data[7]);

    rpc.config.aac.bit_rate = af->bit_rate;
    if (ac->flags & AUDIO_FLAG_WRITE) 
    {
        rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_WRITE;
        rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
    } 
    else 
    {
        rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_READ;
        rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_RECORD;
    }

    if ((af->sbr_on_flag == 0) && (af->sbr_ps_on_flag == 0)) 
    {
        rpc.config.aac.encoder_mode = ADSP_AUDIO_ENC_AAC_LC_ONLY_MODE;
    } 
    else if ((af->sbr_on_flag == 1) && (af->sbr_ps_on_flag == 0)) 
    {
        rpc.config.aac.encoder_mode = ADSP_AUDIO_ENC_AAC_PLUS_MODE;
    } 
    else if ((af->sbr_on_flag == 1) && (af->sbr_ps_on_flag == 1)) 
    {
        rpc.config.aac.encoder_mode = ADSP_AUDIO_ENC_ENHANCED_AAC_PLUS_MODE;
    } 
    else 
    {
        pr_err("unsupported SBR flag\n");
        mutex_unlock(&open_mem_lock);
        return -EINVAL;
    }
    rpc.buf_max_size = bufsz; /* XXX ??? */

    TRACE("aac_open: opcode %x, stream_context 0x%x, "
          "mode %d, bytes %d, bbuffer size %d\n",
          rpc.opcode, rpc.stream_context,
          rpc.config.aac.encoder_mode, rpc.blocklen, bufsz);

    r = dal_call_f5(ac->client, AUDIO_OP_OPEN, &rpc, sizeof(rpc));
    mutex_unlock(&open_mem_lock);
    return r;
}

static int audio_qcelp_open(struct audio_client *ac, uint32_t bufsz,
             void *data)
{
    int r;
    struct msm_audio_qcelp_config *qf = data;
    struct adsp_open_command rpc;
    struct adsp_audio_standard_format *fmt;
//    union adsp_audio_format *ptr;


    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&open_mem_lock);
    memset(&rpc, 0, sizeof(rpc));
    fmt = (struct adsp_audio_standard_format *)params_data;
    memset(fmt, 0, sizeof(struct adsp_audio_standard_format));

    rpc.numdev = 1;
    rpc.dev[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;
    rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_RECORD;

    rpc.format = ADSP_AUDIO_FORMAT_V13K_FS;
    rpc.pblock = params_phys;
    rpc.blocklen = sizeof(struct adsp_audio_standard_format);
    rpc.opcode = ADSP_AUDIO_OPCODE_OPEN_READ;

    fmt->sampling_rate = 8000;
    fmt->channels = 1;
    fmt->bits_per_sample = 16;
    fmt->is_signed = 1;
    fmt->is_interleaved = 0;

    rpc.config.qcelp13k.min_rate = (uint16_t) qf->min_bit_rate;
    rpc.config.qcelp13k.max_rate = (uint16_t) qf->max_bit_rate;
    rpc.buf_max_size = bufsz; /* XXX ??? */

    r = dal_call_f5(ac->client, AUDIO_OP_OPEN, &rpc, sizeof(rpc));

    mutex_unlock(&open_mem_lock);
    return r;
}

static int audio_close(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    AUDIO_INFO("%p: close\n", ac);
    audio_command(ac, ADSP_AUDIO_IOCTL_CMD_STREAM_STOP);
//    audio_command(ac, ADSP_AUDIO_IOCTL_CMD_CLOSE);

    dal_call_f0(ac->client, AUDIO_OP_CLOSE, ac->session);
    return 0;
}

static int audio_set_table(struct audio_client *ac,int32_t device_id, int size)
{
    struct adsp_set_dev_cfg_table_command rpc;

    AUDIO_INFO("%s: %x %d\n", __func__, device_id, size);

//    print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, audio_data, size);

    memset(&rpc, 0, sizeof(rpc));
    if (q6_device_to_dir(device_id) == Q6_TX)
        rpc.hdr.data = tx_clk_freq;
    rpc.device_id = device_id;
    rpc.phys_addr = audio_phys;
    rpc.phys_size = size;
    rpc.phys_used = size;

    TRACE("control: set table %x\n", device_id);
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_SET_DEVICE_CONFIG_TABLE, &rpc, sizeof(rpc));    
}

int q6audio_read(struct audio_client *ac, struct audio_buffer *ab)
{
    struct adsp_buffer_command rpc;
    int r;

    AUDIO_INFO("%s: %X %X %X\n", __func__, ab->phys , ab->size, ab->used);
    memset(&rpc, 0, sizeof(rpc));
    rpc.hdr.context = ac->session;
    rpc.hdr.data = (uint32_t)ab->data;
    rpc.buffer.addr = ab->phys;
    rpc.buffer.max_size = ab->size;
    rpc.buffer.actual_size = ab->used;
    rpc.buffer.flags = ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR | ADSP_AUDIO_BUFFER_FLAG_START_SET;

    TRACE("%p: read\n", ac);
//    r = dal_call(ac->client, AUDIO_OP_DATA, 5, &rpc, sizeof(rpc), &res, sizeof(res));

    r = dal_call_f5(ac->client, AUDIO_OP_READ, &rpc, sizeof(rpc));
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_read);

int q6audio_write(struct audio_client *ac, struct audio_buffer *ab)
{
    struct adsp_buffer_command rpc;
    int r;

    AUDIO_INFO("%s: %X %X %X\n", __func__, ab->phys , ab->size, ab->used);
    memset(&rpc, 0, sizeof(rpc));
    rpc.hdr.context = ac->session;
    rpc.hdr.data = (uint32_t)ab->data;
    rpc.buffer.addr = ab->phys;
    rpc.buffer.max_size = ab->size;
    rpc.buffer.actual_size = ab->used;
    rpc.buffer.start = (~0x7fffffffffffffffLL);
    rpc.buffer.flags = ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR | ADSP_AUDIO_BUFFER_FLAG_START_SET;

    TRACE("%p: write\n", ac);
//    r = dal_call(ac->client, AUDIO_OP_DATA, 5, &rpc, sizeof(rpc), &res, sizeof(res));

    r = dal_call_f5(ac->client, AUDIO_OP_WRITE, &rpc, sizeof(rpc));
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_write);

static int audio_rx_volume(struct audio_client *ac, uint32_t dev_id, int32_t volume)
{
    struct adsp_set_dev_volume_command rpc;

    AUDIO_INFO("%s: ac: %p dev_id 0x%08x, volume = %d\n", __func__, ac, dev_id , volume);

    memset(&rpc, 0, sizeof(rpc));
    rpc.device_id = dev_id;
    rpc.path = ADSP_PATH_RX;
    rpc.volume = volume;
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL, &rpc, sizeof(rpc));
}

static int audio_tx_volume(struct audio_client *ac, uint32_t dev_id, int32_t volume)
{
    struct adsp_set_dev_volume_command rpc;

    AUDIO_INFO("%s: %p dev_id 0x%08x, volume = %d\n", __func__, ac, dev_id , volume);

    memset(&rpc, 0, sizeof(rpc));
    rpc.device_id = dev_id;
    rpc.path = ADSP_PATH_TX;
    rpc.volume = volume;
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL, &rpc, sizeof(rpc));
}


static int audio_rx_mute(struct audio_client *ac, uint32_t dev_id, int mute)
{
    struct adsp_set_dev_mute_command rpc;

    AUDIO_INFO("%s: dev_id 0x%08x, mute %d\n", __func__, dev_id , mute);

    memset(&rpc, 0, sizeof(rpc));
    rpc.device_id = dev_id;
    rpc.path = ADSP_PATH_RX;
    rpc.mute = !!mute;
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE, &rpc, sizeof(rpc));
}


static int audio_tx_mute(struct audio_client *ac, uint32_t dev_id, int mute)
{
    struct adsp_set_dev_mute_command rpc;

    AUDIO_INFO("%s: dev_id 0x%08x, mute %d\n", __func__, dev_id , mute);

    memset(&rpc, 0, sizeof(rpc));
    rpc.device_id = dev_id;
    rpc.path = ADSP_PATH_TX;
    rpc.mute = !!mute;
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE, &rpc, sizeof(rpc));
}

static int audio_stream_volume(struct audio_client *ac, int volume)
{
    struct adsp_set_volume_command rpc;
    int rc;

    AUDIO_INFO("%s: volume %d\n", __func__, volume);
//
// CotullaHACK: this function called from libaudio-qsd8k / AudioHardware.cpp
// status = ioctl(mFd, AUDIO_SET_VOLUME, &stream_volume);
// stream_volume have fixed -300 value, replace it to zero
//
//    printk("$$$ audio_stream_volume(%d)\n", volume);
    volume = 0;

    memset(&rpc, 0, sizeof(rpc));
    rpc.volume = volume;
    rc = audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_SET_STREAM_VOL, &rpc, sizeof(rpc));
    return rc;
}

static int audio_stream_mute(struct audio_client *ac, int mute)
{
    struct adsp_set_mute_command rpc;
    int rc;

    AUDIO_INFO("%s: mute %d\n", __func__, mute);
    memset(&rpc, 0, sizeof(rpc));
    rpc.mute = mute;
    rc = audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE, &rpc, sizeof(rpc));
    return rc;
}

static int audio_stream_eq(struct audio_client *ac, struct cad_audio_eq_cfg *eq_cfg)
{
    struct adsp_audio_set_equalizer_command rpc;
    int rc;
    uint32_t i = 0;

    AUDIO_INFO("%s\n", __func__);
    memset(&rpc, 0, sizeof(rpc));
    rpc.enable = eq_cfg->enable;
    rpc.num_bands = eq_cfg->num_bands;
    for (i = 0; i < rpc.num_bands; i++)
    {
        rpc.eq_bands[i].band_idx = eq_cfg->eq_bands[i].band_idx;
        rpc.eq_bands[i].filter_type = eq_cfg->eq_bands[i].filter_type;
        rpc.eq_bands[i].center_freq_hz = eq_cfg->eq_bands[i].center_freq_hz;
        rpc.eq_bands[i].filter_gain = eq_cfg->eq_bands[i].filter_gain;
        rpc.eq_bands[i].q_factor = eq_cfg->eq_bands[i].q_factor;
#if 1
        pr_info(">>>>>> band_idx       = %d\n", rpc.eq_bands[i].band_idx);
        pr_info(">>>>>> filter_type    = %d\n", rpc.eq_bands[i].filter_type);
        pr_info(">>>>>> center_freq_hz = %d\n", rpc.eq_bands[i].center_freq_hz);
        pr_info(">>>>>> filter_gain    = %d\n", rpc.eq_bands[i].filter_gain);
        pr_info(">>>>>> q_factor       = %d\n\n", rpc.eq_bands[i].q_factor);
#endif
    }
    rc = audio_ioctl(ac, ADSP_AUDIO_IOCTL_SET_SESSION_EQ_CONFIG, &rpc, sizeof(rpc));
    return rc;
}

static void callback(void *data, int len, void *cookie)
{
//    struct adsp_event_hdr *e = data;
    struct adsp_audio_dal_event *e = data;
    struct audio_client *ac;
    struct adsp_audio_event* ae;

//    printk("dal event\n");
    AUDIO_INFO("%s audio callback: CB: %X LOC: %X SIZE=%X, event_id = %X\n", __func__, e->cb_evt, e->loc_evt, e->size, e->ae.event_id);
    TRACE("audio callback: CB: %X LOC: %X SIZE=%X\n",  e->cb_evt, e->loc_evt, e->size);

    if (e->cb_evt != CB_EVENT_COOKIE)
    {
        pr_err("audio callback: bad cb_event %X\n", e->cb_evt);
        return;
    }

    ae = &e->ae;
    if (ae->session >= SESSION_MAX)
    {
        pr_err("audio callback: bogus session %d\n", ae->session);
        return;
    }
    ac = session[ae->session];
    if (!ac)
    {
        pr_err("audio callback: unknown session %d\n", ae->session);
        return;
    }

    if (ae->event_id == ADSP_AUDIO_EVT_STATUS_OPEN)
    {
        pr_info("open done!\n");
        ac->open_status = ae->status;
        ac->open_done = 1;
        wake_up(&ac->wait);
        return;
    }

    if (ae->event_id == ADSP_AUDIO_IOCTL_CMD_STREAM_EOS) 
	{
        TRACE("%p: CB stream eos\n", ac);
        if (ae->status)
        {
            pr_err("playback status %d\n", ae->status);
        }
        if (ac->cb_status == -EBUSY) 
		{
            ac->cb_status = ae->status;
            wake_up(&ac->wait);
        }
        return;
    }

    if (ae->event_id == ADSP_AUDIO_EVT_STATUS_BUF_DONE) 
    {
        TRACE("%p: CB done (%d)\n", ac, ae->status);
//        TRACE("%p: actual_size %d, buffer_size %d\n",
//              ac, abe->buffer.actual_size, ac->buf[ac->dsp_buf].size);

        if (ae->status)
        {
            pr_err("buffer status %d\n", ae->status);
        }
        ac->buf[ac->dsp_buf].used = 0;
        ac->dsp_buf ^= 1;
        wake_up(&ac->wait);
        return;
    }

    TRACE("%p: CB %08x status %d\n", ac, ae->event_id, ae->status);
    if (ae->status)
    {
        pr_warning("audio_cb: s=%d e=%08x status=%d\n", ae->session, ae->event_id, ae->status);
    }

    if (ac->cb_status == -EBUSY) 
    {
        ac->cb_status = ae->status;
        wake_up(&ac->wait);
    }
}


struct rpc_config_info
{
    u32     cb_evt;
    u32     loc_evt;
    u32     session_id;
    u32     processor_id;
};

struct rpc_info
{
   uint32_t size;
   uint32_t version;
   char name[32];

};




static int audio_init(struct audio_client *ac)
{

    int r;
    struct rpc_config_info info;

    AUDIO_INFO("%s\n", __func__);

    info.session_id = ac->session;
    info.processor_id = 1;  // AARM
    info.cb_evt = CB_EVENT_COOKIE;
    info.loc_evt = (u32)(((u32)ac->session + 1) << 16);

    r = dal_call_f5(ac->client, AUDIO_OP_INIT, &info, sizeof(info));    
    return r;
}

static int audio_info(struct audio_client *ac)
{
    int r;
    struct rpc_info info;

    AUDIO_INFO("%s\n", __func__);
 
    r = dal_call_f9(ac->client, DAL_OP_INFO,  &info, sizeof(info));    
    return r;
}

static struct audio_client *ac_control;
static int zero_already_inited;

struct audio_config_data {
    uint32_t device_id;
    uint32_t sample_rate;
    uint32_t offset;
    uint32_t length;
};

struct audio_config_database {
    uint8_t magic[8];
    uint32_t entry_count;
    uint32_t unused;
    struct audio_config_data entry[0];
};

void *acdb_data;
const struct firmware *acdb_fw;
extern struct miscdevice q6_control_device;

static int acdb_init(char *filename)
{
    const struct audio_config_database *db;
    const struct firmware *fw;
    int n;

    //    return -ENODEV;
    AUDIO_INFO("%s\n", __func__);
#ifdef CONFIG_MACH_HTCLEO
    pr_info("acdb: trying htcleo.acdb\n");
    if(request_firmware(&fw, "htcleo.acdb", q6_control_device.this_device) < 0) {
        pr_info("acdb: load 'htcleo.acdb' failed, trying 'default.acdb'\n");
        acdb_use_map = 0;
        if (request_firmware(&fw, filename, q6_control_device.this_device) < 0) {
            pr_err("acdb: load 'default.acdb' failed...\n");
            return -ENODEV;
        }
    } else {
        pr_info("acdb: 'htcleo.acdb' found, using translation\n");
        acdb_use_map = 1;
    }
#else
    pr_info("acdb: load '%s'\n", filename);
    acd_use_map = 0;
    if (request_firmware(&fw, filename, q6_control_device.this_device) < 0) {
        pr_err("acdb: load 'default.acdb' failed...\n");
        return -ENODEV;
    }
#endif
    db = (void*) fw->data;

    if (fw->size < sizeof(struct audio_config_database)) {
        pr_err("acdb: undersized database\n");
        goto fail;
    }
    if (strcmp(db->magic, "ACDB1.0")) {
        pr_err("acdb: invalid magic\n");
        goto fail;
    }
    if (db->entry_count > 1024) {
        pr_err("acdb: too many entries\n");
        goto fail;
    }
    if (fw->size < (sizeof(struct audio_config_database) +
            db->entry_count * sizeof(struct audio_config_data))) {
        pr_err("acdb: undersized TOC\n");
        goto fail;
    }
    for (n = 0; n < db->entry_count; n++) {
        if (db->entry[n].length > 4096) {
            pr_err("acdb: entry %d too large (%d)\n",
                   n, db->entry[n].length);
            goto fail;
        }
        if ((db->entry[n].offset + db->entry[n].length) > fw->size) {
            pr_err("acdb: entry %d outside of data\n", n);
            goto fail;
        }
    }
    if (acdb_data)
        release_firmware(acdb_fw);
    acdb_data = (void*) fw->data;
    acdb_fw = fw;
    return 0;
fail:
    release_firmware(fw);
    return -ENODEV;
}

static int q6audio_init(void)
{
    struct audio_client *ac = 0;
    int res;

    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&audio_lock);
    if (ac_control) 
    {
        res = 0;
        AUDIO_INFO("%s no init all done\n", __func__);
        goto done;
    }

    pr_info("q6audio_init\n");

    icodec_rx_clk = clk_get(0, "icodec_rx_clk");
    icodec_tx_clk = clk_get(0, "icodec_tx_clk");
    ecodec_clk = clk_get(0, "ecodec_clk");
    sdac_clk = clk_get(0, "sdac_clk");
    audio_data = dma_alloc_coherent(NULL, 4096, &audio_phys, GFP_KERNEL);
    params_data = dma_alloc_coherent(NULL, 4096, &params_phys, GFP_KERNEL);
    AUDIO_INFO("%s allocated", __func__);
//    printk("allocated: %p %x\n", params_data, params_phys);

//    pr_info("audio: init: INIT\n");

    if (!zero_already_inited)
    {
        zero_already_inited = 1;
        pr_info("audio: init: first run, init dummy control\n");
        audio_client_alloc(0);
    }

    ac = audio_client_alloc(0);
    if (!ac) 
    {
        pr_err("audio_init: cannot allocate client\n");
        res = -ENOMEM;
        goto done;
    }

//    pr_info("audio: init: OPEN control\n");
    if (audio_open_control(ac)) 
    {
        pr_err("audio_init: cannot open control channel\n");
        res = -ENODEV;
        goto done;
    }

//    pr_info("audio: init: attach ACDB\n");
    acdb = dal_attach(ACDB_DAL_DEVICE, ACDB_DAL_PORT, 0, 0);
    if (!acdb) 
    {
        pr_err("audio_init: cannot attach to acdb channel\n");
        res = -ENODEV;
        goto done;
    }

//    pr_info("audio: init: attach ADIE\n");
    adie = dal_attach(ADIE_DAL_DEVICE, ADIE_DAL_PORT, 0, 0);
    if (!adie) {
        pr_err("audio_init: cannot attach to adie\n");
        res = -ENODEV;
        goto done;
    }

    if (analog_ops->init)
        analog_ops->init();

    if (!acdb_data && !acdb_use_rpc) 
    {
        if (acdb_init(acdb_file)) 
        {
            pr_info("use RPC to query ACDB.\n");
            acdb_use_rpc = 1;
        }
    }

    res = 0;
    ac_control = ac;

    wake_lock_init(&idlelock, WAKE_LOCK_IDLE, "audio_pcm_idle");
    wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "audio_pcm_suspend");

    q6audio_init_rx_volumes();

// TEST HERE
//    pr_info("START TEST\n");
//    audio_tx_mute(ac_control, ADSP_AUDIO_DEVICE_ID_DEFAULT, 1);
//    pr_info("END TEST\n");
done:
    if ((res < 0) && ac)
    {
        audio_client_free(ac);
        printk("   init failed!\n");
    }
    mutex_unlock(&audio_lock);
    return res;
}


static int map_cad_dev_to_virtual(int cd)
{
    AUDIO_INFO("%s\n", __func__);
    if (global_now_phone_call)
    {
        return cd;
    }

    switch (cd)
    {
    case 1:  return 507;
    case 2:  return 208;
    case 3:  return 307;
    case 5:  return 407;
    case 6:  return 507;
    case 7:  return 607;
    case 17: return 408;
    default: return cd;
    }
    return 0;
}

static int acdb_get_config_table(uint32_t device_id, uint32_t sample_rate)
{
    AUDIO_INFO("%s\n", __func__);

    if (q6audio_init())
        return 0;

    if (acdb_use_rpc) 
    {
        struct acdb_cmd_device_table rpc;
        struct acdb_result res;
        int r;
        uint32_t new_device_id;

        memset(audio_data, 0, 4096);
        memset(&rpc, 0, sizeof(rpc));

        new_device_id = map_cad_dev_to_virtual(device_id);
        printk(" ACDB MAP: %d -> %d\n", device_id, new_device_id);

        rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
        rpc.command_id = ACDB_GET_DEVICE_TABLE;
        rpc.device_id = new_device_id; //0x25F; //device_id;
        rpc.sample_rate_id = sample_rate;
        rpc.total_bytes = 4096;
        rpc.unmapped_buf = audio_phys;
        rpc.res_size = sizeof(res) - (2 * sizeof(uint32_t));

//        printk("ACDB dal call\n");
        r = dal_call(acdb, ACDB_OP_IOCTL, 8, &rpc, sizeof(rpc), &res, sizeof(res));
//	r = dal_call_f8(acdb, ACDB_OP_IOCTL, &rpc, sizeof(rpc), &res, sizeof(res));

	pr_info("acdb ret: %d %X %X %X %d\n", res. dal_status, res.size, res.unmapped_buf, res.used_bytes, res.result);
        if ((r == sizeof(res)) && (res.result == 0)) 
	{
            pr_info("acdb: %d bytes for device %d, rate %d.\n", res.used_bytes, device_id, sample_rate);
            return res.used_bytes;
        }
        return 0;
    } 
    else 
    {
        struct audio_config_database *db;
        int n;

        printk("table\n");
        // htcleo use custom table with wince values, it must be mapped
        if(acdb_use_map)
            device_id = map_cad_dev_to_virtual(device_id);
        db = acdb_data;
        for (n = 0; n < db->entry_count; n++) 
        {
            if (db->entry[n].device_id != device_id)
                continue;
            if (db->entry[n].sample_rate != sample_rate)
                continue;
            break;
        }

        if (n == db->entry_count) 
	{
            pr_err("acdb: no entry for device %d, rate %d.\n",
                   device_id, sample_rate);
            return 0;
        }

        pr_info("acdb: %d bytes for device %d, rate %d.\n",
            db->entry[n].length, device_id, sample_rate);

        memcpy(audio_data, acdb_data + db->entry[n].offset, db->entry[n].length);
        return db->entry[n].length;
    }
}

//static uint32_t audio_rx_path_id = ADIE_PATH_HANDSET_RX;
//static uint32_t audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR;
static uint32_t audio_rx_path_id = ADIE_PATH_SPEAKER_RX;
static uint32_t audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO;
static uint32_t audio_rx_device_group = -1;
static uint32_t audio_tx_path_id = ADIE_PATH_HANDSET_TX;
static uint32_t audio_tx_device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_MIC;
static uint32_t audio_tx_device_group = -1;

static int qdsp6_devchg_notify(struct audio_client *ac, uint32_t dev_type, uint32_t dev_id)
{
    struct adsp_device_switch_command rpc;

    AUDIO_INFO("%s\n", __func__);

    if (dev_type != ADSP_AUDIO_RX_DEVICE && dev_type != ADSP_AUDIO_TX_DEVICE)
        return -EINVAL;

    memset(&rpc, 0, sizeof(rpc));
    if (dev_type == ADSP_AUDIO_RX_DEVICE) 
    {
        rpc.old_device = audio_rx_device_id;
        rpc.new_device = dev_id;
    } 
    else 
    {
        rpc.old_device = audio_tx_device_id;
        rpc.new_device = dev_id;
    }
    rpc.device_class = 0;
    rpc.device_type = dev_type;
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE, &rpc, sizeof(rpc));
}

static int qdsp6_standby(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    return audio_command(ac, ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY);
}

static int qdsp6_start(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    return audio_command(ac, ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT);
}

static void audio_rx_analog_enable(int en)
{
    AUDIO_INFO("%s\n", __func__);
    switch (audio_rx_device_id) {
    case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO:
    case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO:
    case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR:
        if (analog_ops->headset_enable)
            analog_ops->headset_enable(en);
        break;
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET:
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET:
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET:
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET:
        if (analog_ops->headset_enable)
            analog_ops->headset_enable(en);
        if (analog_ops->speaker_enable)
            analog_ops->speaker_enable(en);
        break;
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO:
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO:
        if (analog_ops->speaker_enable)
            analog_ops->speaker_enable(en);
        break;
    case ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR:
        if (analog_ops->bt_sco_enable)
            analog_ops->bt_sco_enable(en);
        break;
    case ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR:
        if (analog_ops->receiver_enable)
            analog_ops->receiver_enable(en);
        break;
/*
 TODO: we need this? 
    case ADSP_AUDIO_DEVICE_ID_I2S_SPKR:
        if (analog_ops->i2s_enable)
            analog_ops->i2s_enable(en);
        break;
*/
    }

}

static void audio_tx_analog_enable(int en)
{
    AUDIO_INFO("%s\n", __func__);
    switch (audio_tx_device_id) {
    case ADSP_AUDIO_DEVICE_ID_HANDSET_MIC:
    case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC:
        if (analog_ops->int_mic_enable)
            analog_ops->int_mic_enable(en);
        break;
    case ADSP_AUDIO_DEVICE_ID_HEADSET_MIC:
    case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC:
        if (analog_ops->ext_mic_enable)
            analog_ops->ext_mic_enable(en);
        break;
    case ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC:
        if (analog_ops->bt_sco_enable)
            analog_ops->bt_sco_enable(en);
        break;
    }
}

static int audio_update_acdb(uint32_t adev, uint32_t acdb_id)
{
    uint32_t sample_rate;
    int sz = -1;

    printk("%s (%d, %d)\n", __func__, adev, acdb_id);
    AUDIO_INFO("%s (%x, %d)\n", __func__, adev, acdb_id);
//    dex_comm(PCOM_UPDATE_ACDB, 0, 0);
    sample_rate = q6_device_to_rate(adev);

    if (q6_device_to_dir(adev) == Q6_RX)
        rx_acdb = acdb_id;
    else
        tx_acdb = acdb_id;

    if (acdb_id != 0)
    {
        sz = acdb_get_config_table(acdb_id, sample_rate);
    }
    printk("res1 = %d\n", sz);
    if (sz <= 0) 
    {
        acdb_id = q6_device_to_cad_id(adev);
        printk("  new acdb = %d\n", acdb_id);
        AUDIO_INFO("  new acdb = %d\n", acdb_id);
        sz = acdb_get_config_table(acdb_id, sample_rate);
    }
    printk("res2 = %d\n", sz);
    if (sz > 0)
    {
        printk("call audio_set_table\n");
        AUDIO_INFO("call audio_set_table\n");
        audio_set_table(ac_control, adev, sz);
    }
    return 0;
}

#ifdef CONFIG_QSD_AUDIO_CALLREC
static void adie_rx_path_enable(uint32_t acdb_id)
{
	AUDIO_INFO("%s\n", __func__);
	adie_enable();
	adie_set_path(adie, audio_rx_path_id, ADIE_PATH_RX);
	adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);

	adie_proceed_to_stage(adie, ADIE_PATH_RX,
		ADIE_STAGE_DIGITAL_READY);
	adie_proceed_to_stage(adie, ADIE_PATH_RX,
		ADIE_STAGE_DIGITAL_ANALOG_READY);
}

static void q6_rx_path_enable(int reconf, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
	audio_update_acdb(audio_rx_device_id, acdb_id);
	if (!reconf)
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, audio_rx_device_id);
	qdsp6_standby(ac_control);
	qdsp6_start(ac_control);
}
#endif

static void _audio_rx_path_enable(int reconf, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    adie_enable();
    adie_set_path(adie, audio_rx_path_id, ADIE_PATH_RX);
    adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);

    adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_READY);
    adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_ANALOG_READY);

    audio_update_acdb(audio_rx_device_id, acdb_id);
    if (!reconf)
        qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, audio_rx_device_id);
    qdsp6_standby(ac_control);
    qdsp6_start(ac_control);

    audio_update_acdb(audio_rx_device_id, acdb_id);

    audio_rx_analog_enable(1);
}
//function to calculate mic gain; for now we use the same gain range for all mic
static int get_audio_tx_volume(uint32_t device_id, int level)
{
    AUDIO_INFO("%s\n", __func__);
    return min_mic_gain + ((max_mic_gain - min_mic_gain) * level) / MAX_MIC_LEVEL;
}

static DEFINE_MUTEX(audio_path_lock);
static int audio_rx_path_refcount;
static int audio_tx_path_refcount;

static void _update_audio_tx_volume(struct audio_client *ac, uint32_t dev_id)
{
    int i, level = -1, volume, ret;
    AUDIO_INFO("%s device_id %x\n", __func__, dev_id);
    for (i = 0; i < DEVICE_ID_MIC_COUNT; i++ ) {
        if (user_mic_levels[i].device_id == dev_id)
        {
            level = user_mic_levels[i].level;
            break;
        }
    }
    if (level==-1)
    {
        AUDIO_INFO("%s returning level = -1\n", __func__);
        return;
    }
    volume = get_audio_tx_volume(dev_id, level);
    ret = audio_tx_volume(ac, dev_id, volume);
    AUDIO_INFO("%s device_id = %x, level= %d, volume= %d, ret = %d", __func__, dev_id, level, volume, ret);
    return;
}

static void update_all_audio_tx_volume(struct audio_client *ac)
{
    int i, volume, ret;
    AUDIO_INFO("%s\n", __func__);
    for (i = 0; i < DEVICE_ID_MIC_COUNT; i++ ) {
        if (user_mic_levels[i].level==-1)
        {
            continue;
        }
        mutex_lock(&audio_path_lock);
        volume = get_audio_tx_volume(user_mic_levels[i].device_id, user_mic_levels[i].level);
        ret = audio_tx_volume(ac, user_mic_levels[i].device_id, volume);
        mutex_unlock(&audio_path_lock);
        AUDIO_INFO("%s device_id = %x, level= %d, volume= %d, ret = %d", __func__, user_mic_levels[i].device_id, user_mic_levels[i].level, volume, ret);
    }
    return;
}

static void _audio_tx_path_enable(int reconf, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    audio_tx_analog_enable(1);

    adie_enable();
    adie_set_path(adie, audio_tx_path_id, ADIE_PATH_TX);

    if (tx_clk_freq > 8000)
        adie_set_path_freq_plan(adie, ADIE_PATH_TX, 48000);
    else
        adie_set_path_freq_plan(adie, ADIE_PATH_TX, 8000);

    adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_READY);
    adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_ANALOG_READY);

    audio_update_acdb(audio_tx_device_id, acdb_id);

    if (!reconf)
        qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, audio_tx_device_id);
    qdsp6_standby(ac_control);
    qdsp6_start(ac_control);
    audio_tx_mute(ac_control, audio_tx_device_id, tx_mute_status);
    _update_audio_tx_volume(ac_control, audio_tx_device_id);
}

static void _audio_rx_path_disable(void)
{
    AUDIO_INFO("%s\n", __func__);
    audio_rx_analog_enable(0);

    adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_ANALOG_OFF);
    adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_OFF);
    adie_disable();
}

static void _audio_tx_path_disable(void)
{
    AUDIO_INFO("%s\n", __func__);
    audio_tx_analog_enable(0);

    adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_ANALOG_OFF);
    adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_OFF);
    adie_disable();
}

static int icodec_rx_clk_refcount;
static int icodec_tx_clk_refcount;
static int ecodec_clk_refcount;
static int sdac_clk_refcount;

static void _audio_rx_clk_enable(void)
{
    uint32_t device_group = q6_device_to_codec(audio_rx_device_id);
    AUDIO_INFO("%s: device 0x%08x, group %d\n",
           __func__, audio_rx_device_id, device_group);
    switch(device_group) {
    case Q6_ICODEC_RX:
        icodec_rx_clk_refcount++;
        AUDIO_INFO("%s: icodec_rx_clk_refcount = %d\n",
               __func__, icodec_rx_clk_refcount);
        if (icodec_rx_clk_refcount == 1) {
            clk_set_rate(icodec_rx_clk, 12288000);
            clk_enable(icodec_rx_clk);
            AUDIO_INFO("%s: icodec_rx_clk enabled\n", __func__);
        }
        break;
    case Q6_ECODEC_RX:
        ecodec_clk_refcount++;
        AUDIO_INFO("%s: ecodec_clk_refcount = %d\n",
               __func__, ecodec_clk_refcount);
        if (ecodec_clk_refcount == 1) {
            clk_set_rate(ecodec_clk, 2048000);
            clk_enable(ecodec_clk);
            AUDIO_INFO("%s: ecodec_clk enabled\n", __func__);
        }
        break;
    case Q6_SDAC_RX:
        sdac_clk_refcount++;
        AUDIO_INFO("%s: sdac_clk_refcount = %d\n",
               __func__, sdac_clk_refcount);
        if (sdac_clk_refcount == 1) {
            clk_set_rate(sdac_clk, 12288000);
            clk_enable(sdac_clk);
            AUDIO_INFO("%s: sdac_clk enabled\n", __func__);
        }
        break;
    default:
        return;
    }
    audio_rx_device_group = device_group;
}

static void _audio_tx_clk_enable(void)
{
    uint32_t device_group = q6_device_to_codec(audio_tx_device_id);
    AUDIO_INFO("%s: device 0x%08x, group %d\n",
           __func__, audio_tx_device_id, device_group);
    switch (device_group) {
    case Q6_ICODEC_TX:
        icodec_tx_clk_refcount++;
        AUDIO_INFO("%s: icodec_tx_clk_refcount = %d\n",
               __func__, icodec_tx_clk_refcount);
        if (icodec_tx_clk_refcount == 1) {
            clk_set_rate(icodec_tx_clk, tx_clk_freq * 256);
            clk_enable(icodec_tx_clk);
            AUDIO_INFO("%s: icodec_tx_clk enabled\n", __func__);
        }
        break;
    case Q6_ECODEC_TX:
        ecodec_clk_refcount++;
        AUDIO_INFO("%s: ecodec_clk_refcount = %d\n",
               __func__, ecodec_clk_refcount);
        if (ecodec_clk_refcount == 1) {
            clk_set_rate(ecodec_clk, 2048000);
            clk_enable(ecodec_clk);
            AUDIO_INFO("%s: ecodec_clk enabled\n", __func__);
        }
        break;
    case Q6_SDAC_TX:
        /* TODO: In QCT BSP, clk rate was set to 20480000 */
        sdac_clk_refcount++;
        AUDIO_INFO("%s: sdac_clk_refcount = %d\n",
               __func__, sdac_clk_refcount);
        if (sdac_clk_refcount == 1) {
            clk_set_rate(sdac_clk, 12288000);
            clk_enable(sdac_clk);
            AUDIO_INFO("%s: sdac_clk enabled\n", __func__);
        }
        break;
    default:
        return;
    }
    audio_tx_device_group = device_group;
}

static void _audio_rx_clk_disable(void)
{
    AUDIO_INFO("%s: group %d\n", __func__, audio_rx_device_group);
    switch (audio_rx_device_group) {
    case Q6_ICODEC_RX:
        icodec_rx_clk_refcount--;
        AUDIO_INFO("%s: icodec_rx_clk_refcount = %d\n",
               __func__, icodec_rx_clk_refcount);
        if (icodec_rx_clk_refcount == 0) {
            clk_disable(icodec_rx_clk);
            audio_rx_device_group = -1;
            AUDIO_INFO("%s: icodec_rx_clk disabled\n", __func__);
        }
        break;
    case Q6_ECODEC_RX:
        ecodec_clk_refcount--;
        AUDIO_INFO("%s: ecodec_clk_refcount = %d\n",
               __func__, ecodec_clk_refcount);
        if (ecodec_clk_refcount == 0) {
            clk_disable(ecodec_clk);
            audio_rx_device_group = -1;
            AUDIO_INFO("%s: ecodec_clk disabled\n", __func__);
        }
        break;
    case Q6_SDAC_RX:
        sdac_clk_refcount--;
        AUDIO_INFO("%s: sdac_clk_refcount = %d\n",
               __func__, sdac_clk_refcount);
        if (sdac_clk_refcount == 0) {
            clk_disable(sdac_clk);
            audio_rx_device_group = -1;
            AUDIO_INFO("%s: sdac_clk disabled\n", __func__);
        }
        break;
    default:
        pr_err("audiolib: invalid rx device group %d\n",
            audio_rx_device_group);
        break;
    }
}

static void _audio_tx_clk_disable(void)
{
    AUDIO_INFO("%s: group %d\n", __func__, audio_tx_device_group);
    switch (audio_tx_device_group) {
    case Q6_ICODEC_TX:
        icodec_tx_clk_refcount--;
        AUDIO_INFO("%s: icodec_tx_clk_refcount = %d\n",
               __func__, icodec_tx_clk_refcount);
        if (icodec_tx_clk_refcount == 0) {
            clk_disable(icodec_tx_clk);
            audio_tx_device_group = -1;
            AUDIO_INFO("%s: icodec_tx_clk disabled\n", __func__);
        }
        break;
    case Q6_ECODEC_TX:
        ecodec_clk_refcount--;
        AUDIO_INFO("%s: ecodec_clk_refcount = %d\n",
               __func__, ecodec_clk_refcount);
        if (ecodec_clk_refcount == 0) {
            clk_disable(ecodec_clk);
            audio_tx_device_group = -1;
            AUDIO_INFO("%s: ecodec_clk disabled\n", __func__);
        }
        break;
    case Q6_SDAC_TX:
        sdac_clk_refcount--;
        AUDIO_INFO("%s: sdac_clk_refcount = %d\n",
               __func__, sdac_clk_refcount);
        if (sdac_clk_refcount == 0) {
            clk_disable(sdac_clk);
            audio_tx_device_group = -1;
            AUDIO_INFO("%s: sdac_clk disabled\n", __func__);
        }
        break;
    default:
        pr_err("audiolib: invalid tx device group %d\n",
            audio_tx_device_group);
        break;
    }
}

static void _audio_rx_clk_reinit(uint32_t rx_device)
{
    uint32_t device_group = q6_device_to_codec(rx_device);

    AUDIO_INFO("%s\n", __func__);

    if (device_group != audio_rx_device_group)
        _audio_rx_clk_disable();

    audio_rx_device_id = rx_device;
    audio_rx_path_id = q6_device_to_path(rx_device);

    if (device_group != audio_rx_device_group)
        _audio_rx_clk_enable();

}

static void _audio_tx_clk_reinit(uint32_t tx_device)
{
    uint32_t device_group = q6_device_to_codec(tx_device);

    AUDIO_INFO("%s\n", __func__);
    if (device_group != audio_tx_device_group)
        _audio_tx_clk_disable();

    audio_tx_device_id = tx_device;
    audio_tx_path_id = q6_device_to_path(tx_device);

    if (device_group != audio_tx_device_group)
        _audio_tx_clk_enable();
}


static int audio_rx_path_enable(int en, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&audio_path_lock);
    if (en) {
        audio_rx_path_refcount++;
        if (audio_rx_path_refcount == 1) {
            _audio_rx_clk_enable();
            _audio_rx_path_enable(0, acdb_id);
        }
    } else {
        audio_rx_path_refcount--;
        if (audio_rx_path_refcount == 0) {
            _audio_rx_path_disable();
            _audio_rx_clk_disable();
        }
    }
    mutex_unlock(&audio_path_lock);
    return 0;
}

static int audio_tx_path_enable(int en, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&audio_path_lock);
    if (en) {
        audio_tx_path_refcount++;
        if (audio_tx_path_refcount == 1) {
            _audio_tx_clk_enable();
            _audio_tx_path_enable(0, acdb_id);
        }
    } else {
        audio_tx_path_refcount--;
        if (audio_tx_path_refcount == 0) {
            _audio_tx_path_disable();
            _audio_tx_clk_disable();
        }
    }
    mutex_unlock(&audio_path_lock);
    return 0;
}

int q6audio_reinit_acdb(char* filename) 
{
    int res;

    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    mutex_lock(&audio_path_lock);
    if (strlen(filename) < 0) {
        res = -EINVAL;
        goto done;
    }
    res = acdb_init(filename);
    if (!res)
        strcpy(acdb_file, filename);
done:
    mutex_unlock(&audio_path_lock);
    return res;

}

int q6audio_update_acdb(uint32_t id_src, uint32_t id_dst)
{
    int res;

    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    mutex_lock(&audio_path_lock);
    res = audio_update_acdb(id_dst, id_src);
    if (res)
        goto done;

    if (q6_device_to_dir(id_dst) == Q6_RX)
        qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, id_dst);
    else
        qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, id_dst);
    qdsp6_standby(ac_control);
    qdsp6_start(ac_control);
done:
    mutex_unlock(&audio_path_lock);
    return res;
}


int q6audio_get_tx_dev_volume(int device_id)
{
    AUDIO_INFO("%s\n", __func__);
    if (device_id > DEVICE_ID_MIC_COUNT || device_id < 0)
    {
        pr_err("unsupported device id %d\n", device_id);
        return -EINVAL;
    }
    return user_mic_levels[device_id].level;
}


int q6audio_set_tx_dev_volume(int device_id, int level)
{
    AUDIO_INFO("%s\n", __func__);

    if (level < -1)
        level = -1;
    if (level > MAX_MIC_LEVEL)
        level = MAX_MIC_LEVEL;
    if (device_id > DEVICE_ID_MIC_COUNT || device_id < 0)
    {
        AUDIO_INFO("%s unsupported device id %d\n", __func__, device_id);
        pr_err("unsupported device id %d\n", device_id);
        return -EINVAL;
    }
    user_mic_levels[device_id].level = level;
    if (ac_control) 
    {
        mutex_lock(&audio_path_lock);	
        _update_audio_tx_volume(ac_control, user_mic_levels[device_id].device_id);	
        mutex_unlock(&audio_path_lock);

    }

    return 0;
}

int q6audio_set_tx_volume(int level)
{
    int vol;

	AUDIO_INFO("%s\n", __func__);

    mutex_lock(&audio_path_lock);
    vol = q6_device_volume(audio_tx_device_id, level);
	audio_tx_volume(ac_control, audio_tx_device_id, vol);
	//_update_audio_tx_volume(ac_control, audio_tx_device_id);
    mutex_unlock(&audio_path_lock);

    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_set_tx_volume);

int q6audio_set_tx_mute(int mute)
{
    uint32_t adev;

    AUDIO_INFO("%s mute= %d\n", __func__, mute);
    if (q6audio_init())
        return 0;

    mutex_lock(&audio_path_lock);

    if (mute == tx_mute_status) {
        mutex_unlock(&audio_path_lock);
        AUDIO_INFO("%s no change %d\n", __func__, mute);
        return 0;
    }

    adev = audio_tx_device_id;
    audio_tx_mute(ac_control, adev, mute);
    tx_mute_status = mute;
    if (!mute)
        _update_audio_tx_volume(ac_control, audio_tx_device_id);
    mutex_unlock(&audio_path_lock);
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_set_tx_mute);

int q6audio_set_stream_volume(struct audio_client *ac, int vol)
{
    AUDIO_INFO("%s %d\n", __func__, vol);
    if (vol > 1200 || vol < -4000) {
        pr_err("unsupported volume level %d\n", vol);
        return -EINVAL;
    }
    mutex_lock(&audio_path_lock);
    audio_stream_mute(ac, 0);
    audio_stream_volume(ac, vol);
    mutex_unlock(&audio_path_lock);
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_set_stream_volume);

int q6audio_set_stream_eq(struct audio_client *ac, struct cad_audio_eq_cfg *eq_cfg)
{
    AUDIO_INFO("%s\n", __func__);
    mutex_lock(&audio_path_lock);
    audio_stream_eq(ac, eq_cfg);
    mutex_unlock(&audio_path_lock);
    return 0;
}


int q6audio_set_rx_dev_volume(int level)
{
    int vol;

    AUDIO_INFO("%s\n", __func__);

    mutex_lock(&audio_path_lock);

    vol = q6_device_volume(audio_rx_device_id, level);
    printk("$$ DEV=%08X: vol is %d\n", audio_rx_device_id, vol);
    audio_rx_volume(ac_control, audio_rx_device_id, vol);
    //this is needed beacause some audio library don't do
    // mic mute -> mic unmute when the call is made, 
    // instead library is constantly seting audio volume
    _update_audio_tx_volume(ac_control, audio_tx_device_id);
    mutex_unlock(&audio_path_lock);
    return 0;
}

int q6audio_set_rx_volume(int level)
{
#if 0
    uint32_t adev;
    int vol;

    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    if (level < 0 || level > 100)
        return -EINVAL;

    mutex_lock(&audio_path_lock);
    adev = ADSP_AUDIO_DEVICE_ID_VOICE;
    vol = q6_device_volume(audio_rx_device_id, level);
    audio_rx_mute(ac_control, adev, 0);
    printk("@@@@ rx volume: adev=%d, rx_dev_id=%d, level=%d @@@@\n", adev, audio_rx_device_id, vol);
    audio_rx_volume(ac_control, adev, vol);
    rx_vol_level = level;
    mutex_unlock(&audio_path_lock);
#else
    AUDIO_INFO("%s\n", __func__);
    q6audio_set_rx_dev_volume(level);
#endif
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_set_rx_volume);

static int q6audio_init_rx_volumes(void)
{
    int vol;
    struct q6_device_info *di = q6_audio_devices;

    AUDIO_INFO("%s\n", __func__);

    mutex_lock(&audio_path_lock);

    AUDIO_INFO("$$$ q6audio_init_rx_volumes\n");
    while (1)
    {
        if (di->id == 0) break;

        vol = q6_device_volume(di->id, 100);
        audio_rx_volume(ac_control, di->id, vol);
        AUDIO_INFO("$$ DEV=%08X: vol is %d\n", di->id, vol);

        di++;
    }

    mutex_unlock(&audio_path_lock);
    return 0;
}


int q6audio_set_rx_mute(int mute)
{
    uint32_t adev;

    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    if (mute < 0 || mute > 1)
        return -EINVAL;

    mutex_lock(&audio_path_lock);
    AUDIO_INFO("%s: set mute status %d\n", __func__, mute);
    adev = ADSP_AUDIO_DEVICE_ID_VOICE;
    audio_rx_mute(ac_control, adev, mute);
    mutex_unlock(&audio_path_lock);
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_set_rx_mute);

static void do_rx_routing(uint32_t device_id, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    if (device_id == audio_rx_device_id) {
        if (acdb_id != rx_acdb) {
            audio_update_acdb(device_id, acdb_id);
            qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, device_id);
            qdsp6_standby(ac_control);
            qdsp6_start(ac_control);
        }
        return;
    }

    if (audio_rx_path_refcount > 0) {
        qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, device_id);
        _audio_rx_path_disable();
        _audio_rx_clk_reinit(device_id);
        _audio_rx_path_enable(1, acdb_id);
    } else {
        audio_rx_device_id = device_id;
        audio_rx_path_id = q6_device_to_path(device_id);
    }
}

static void do_tx_routing(uint32_t device_id, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    if (device_id == audio_tx_device_id) {
        if (acdb_id != tx_acdb) {
            audio_update_acdb(device_id, acdb_id);
            qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, device_id);
            qdsp6_standby(ac_control);
            qdsp6_start(ac_control);
        }
        AUDIO_INFO("%s, update_tx_volume", __func__);
        _update_audio_tx_volume(ac_control, device_id);
        return;
    }

    if (audio_tx_path_refcount > 0) {
        qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, device_id);
        _audio_tx_path_disable();
        _audio_tx_clk_reinit(device_id);
        _audio_tx_path_enable(1, acdb_id);
    } else {
        audio_tx_device_id = device_id;
        audio_tx_path_id = q6_device_to_path(device_id);
        AUDIO_INFO("%s, update_tx_volume2", __func__);
        _update_audio_tx_volume(ac_control, audio_tx_device_id);
    }
}

int q6audio_do_routing(uint32_t device_id, uint32_t acdb_id)
{
    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    mutex_lock(&audio_path_lock);

    switch(q6_device_to_dir(device_id)) {
    case Q6_RX:
        do_rx_routing(device_id, acdb_id);
        break;
    case Q6_TX:
        do_tx_routing(device_id, acdb_id);
        break;
    }

    mutex_unlock(&audio_path_lock);
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_do_routing);

int q6audio_set_route(const char *name)
{
    uint32_t route;
    AUDIO_INFO("%s\n", __func__);
    if (!strcmp(name, "speaker")) {
        route = ADIE_PATH_SPEAKER_STEREO_RX;
    } else if (!strcmp(name, "headphones")) {
        route = ADIE_PATH_HEADSET_STEREO_RX;
    } else if (!strcmp(name, "handset")) {
        route = ADIE_PATH_HANDSET_RX;
    } else {
        return -EINVAL;
    }

    mutex_lock(&audio_path_lock);
    if (route == audio_rx_path_id)
        goto done;

    audio_rx_path_id = route;

    if (audio_rx_path_refcount > 0) 
    {
        _audio_rx_path_disable();
        _audio_rx_path_enable(1, 0);
    }
    if (audio_tx_path_refcount > 0)
    {
        _audio_tx_path_disable();
        _audio_tx_path_enable(1, 0);
    }
done:
    mutex_unlock(&audio_path_lock);
    return 0;
}

struct audio_client *q6audio_open_pcm(uint32_t bufsz, uint32_t rate,
                      uint32_t channels, uint32_t flags, uint32_t acdb_id)
{
#if 1
    int rc, retry = 5;
    struct audio_client *ac;

    AUDIO_INFO("%s write= %d \n", __func__, flags& AUDIO_FLAG_WRITE);
    if (q6audio_init())
        return 0;

//    printk("afer init();\n");
    ac = audio_client_alloc(bufsz);
    if (!ac)
    {
        printk("audio_client_alloc failed\n");
        return 0;
    }
//    printk("after alloc);\n");
    ac->flags = flags;

    mutex_lock(&audio_path_lock);

    if (ac->flags & AUDIO_FLAG_WRITE) 
    {
        audio_rx_path_refcount++;
        if (audio_rx_path_refcount == 1) 
        {
            _audio_rx_clk_enable();
#ifdef CONFIG_QSD_AUDIO_CALLREC
			q6_rx_path_enable(0, acdb_id);
			adie_rx_path_enable(acdb_id);
#else
//            audio_update_acdb(audio_rx_device_id, acdb_id);
            qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, audio_rx_device_id);
            qdsp6_standby(ac_control);
            qdsp6_start(ac_control);
            audio_update_acdb(audio_rx_device_id, acdb_id);
#endif
        }
    } 
    else
    {
        /* TODO: consider concurrency with voice call */
#ifdef CONFIG_QSD_AUDIO_CALLREC
		if (audio_tx_path_refcount > 0) {
			tx_clk_freq = 8000;
		} else {
			tx_clk_freq = rate;
		}
#else
		tx_clk_freq = rate;
#endif
        audio_tx_path_refcount++;
        if (audio_tx_path_refcount == 1)
        {
#ifdef CONFIG_QSD_AUDIO_CALLREC
			tx_clk_freq = rate;
#endif
            _audio_tx_clk_enable();
            _audio_tx_path_enable(0, acdb_id);
        }
    }
//    printk("about to open\n");

    for (retry = 5;;retry--)
    {
        if (ac->flags & AUDIO_FLAG_WRITE)
            rc = audio_out_open(ac, bufsz, rate, channels);
        else
#ifdef CONFIG_QSD_AUDIO_CALLREC
			rc = audio_in_open(ac, bufsz, flags, rate, channels);
#else
			rc = audio_in_open(ac, bufsz, rate, channels);
#endif
        if (rc == 0)
            break;
        if (retry == 0)
        {
    //       BUG();
            break;
        }
        pr_err("q6audio: open pcm error %d, retrying\n", rc);
        msleep(1);
    }

    if (retry == 0)
    {
        if (ac->flags & AUDIO_FLAG_WRITE)
            audio_rx_path_enable(0, 0);
        else
            audio_tx_path_enable(0, 0);
        audio_client_free(ac);
        pr_err("q6audio: open pcm error\n");
        return NULL;
    }

//    printk("after open\n");

    if (ac->flags & AUDIO_FLAG_WRITE) 
    {
        if (audio_rx_path_refcount == 1) 
        {
#ifndef CONFIG_QSD_AUDIO_CALLREC
            adie_enable();
            adie_set_path(adie, audio_rx_path_id, ADIE_PATH_RX);
            adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);

            adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_READY);
            adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_ANALOG_READY);

#endif
            audio_rx_analog_enable(1);
        }
    }
//    printk("about to start session\n");

    mutex_unlock(&audio_path_lock);

    for (retry = 5;;retry--) 
    {
        rc = audio_command(ac, ADSP_AUDIO_IOCTL_CMD_SESSION_START);
        if (rc == 0)
            break;
        if (retry == 0)
        {
//            BUG();
            break;
        }
        pr_err("q6audio: stream start error %d, retrying\n", rc);
    }

    if (retry == 0)
    {
        audio_close(ac);
        if (ac->flags & AUDIO_FLAG_WRITE)
            audio_rx_path_enable(0, 0);
        else
            audio_tx_path_enable(0, 0);
        audio_client_free(ac);
        pr_err("q6audio: open pcm error2\n");
        return NULL;
    }

    if (!(ac->flags & AUDIO_FLAG_WRITE))
    {
        ac->buf[0].used = 1;
        ac->buf[1].used = 1;
        q6audio_read(ac, &ac->buf[0]);
        q6audio_read(ac, &ac->buf[1]);
    }

    audio_prevent_sleep();
    if (!(ac->flags & AUDIO_FLAG_WRITE))
    {
        update_all_audio_tx_volume(ac_control);		
    }
	
    return ac;
#else
    return audio_test();
#endif
}
EXPORT_SYMBOL_GPL(q6audio_open_pcm);

int q6audio_close(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    audio_close(ac);
    if (ac->flags & AUDIO_FLAG_WRITE)
        audio_rx_path_enable(0, 0);
    else
        audio_tx_path_enable(0, 0);

    audio_client_free(ac);
    audio_allow_sleep();
    return 0;
}
EXPORT_SYMBOL_GPL(q6audio_close);

struct audio_client *q6voice_open(uint32_t flags, uint32_t acdb_id)
{
    struct audio_client *ac;

    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    ac = audio_client_alloc(0);
    if (!ac)
        return 0;

    ac->flags = flags;
    if (ac->flags & AUDIO_FLAG_WRITE) {
        audio_rx_path_enable(1, acdb_id);
        audio_rx_mute(ac_control, ADSP_AUDIO_DEVICE_ID_VOICE, 0);
    } else {
        tx_clk_freq = 8000;
        audio_tx_path_enable(1, acdb_id);
        AUDIO_INFO("%s start session\n", __func__);
        // this is a good location for updating the mic gain but 
        // for some reason this doesn't work (too early to set mic gain)
        //update_all_audio_tx_volume(ac_control);		
    }

    return ac;
}

int q6voice_close(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    if (ac->flags & AUDIO_FLAG_WRITE) {
        audio_rx_mute(ac_control, ADSP_AUDIO_DEVICE_ID_VOICE, 1);
        audio_rx_path_enable(0, 0);
    } else
        audio_tx_path_enable(0, 0);

    audio_client_free(ac);
    return 0;
}

struct audio_client *q6audio_open_mp3(uint32_t bufsz, uint32_t rate,
                      uint32_t channels, uint32_t acdb_id)
{
    struct audio_client *ac;

    AUDIO_INFO("%s\n", __func__);
    printk("q6audio_open_mp3()\n");

    if (q6audio_init())
        return 0;

    ac = audio_client_alloc(bufsz);
    if (!ac)
        return 0;

    ac->flags = AUDIO_FLAG_WRITE;
    audio_rx_path_enable(1, acdb_id);

    audio_mp3_open(ac, bufsz, rate, channels);
    audio_command(ac, ADSP_AUDIO_IOCTL_CMD_SESSION_START);

    return ac;
}

int q6audio_mp3_close(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    audio_close(ac);
    audio_rx_path_enable(0, 0);
    audio_client_free(ac);
    return 0;
}

int q6audio_async(struct audio_client *ac)
{
    struct adsp_command_hdr rpc;
    AUDIO_INFO("%s\n", __func__);
    memset(&rpc, 0, sizeof(rpc));
//    rpc.response_type = ADSP_AUDIO_RESPONSE_ASYNC;
    return audio_ioctl(ac, ADSP_AUDIO_IOCTL_CMD_STREAM_EOS, &rpc, sizeof(rpc));
}



struct audio_client *q6audio_open_aac(uint32_t bufsz, uint32_t rate,
                      uint32_t flags, void *data, uint32_t acdb_id)
{
    struct audio_client *ac;

    AUDIO_INFO("%s\n", __func__);
    TRACE("q6audio_open_aac flags=%d rate=%d\n", flags, rate);

    if (q6audio_init())
        return 0;

    ac = audio_client_alloc(bufsz);
    if (!ac)
        return 0;

    ac->flags = flags;
    if (ac->flags & AUDIO_FLAG_WRITE)
        audio_rx_path_enable(1, acdb_id);
    else {
        /* TODO: consider concourrency with voice call */
        tx_clk_freq = rate;
        audio_tx_path_enable(1, acdb_id);
    }

    audio_aac_open(ac, bufsz, data);
    audio_command(ac, ADSP_AUDIO_IOCTL_CMD_SESSION_START);

    if (!(ac->flags & AUDIO_FLAG_WRITE)) {
        ac->buf[0].used = 1;
        ac->buf[1].used = 1;
        q6audio_read(ac, &ac->buf[0]);
        q6audio_read(ac, &ac->buf[1]);
    }
    audio_prevent_sleep();
    return ac;
}

int q6audio_aac_close(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    audio_close(ac);
    if (ac->flags & AUDIO_FLAG_WRITE)
        audio_rx_path_enable(0, 0);
    else
        audio_tx_path_enable(0, 0);

    audio_client_free(ac);
    audio_allow_sleep();
    return 0;
}

struct audio_client *q6fm_open(void)
{
    struct audio_client *ac;

    AUDIO_INFO("%s\n", __func__);
    printk("q6fm_open()\n");

    if (q6audio_init())
        return 0;

    if (audio_rx_device_id != ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO &&
        audio_rx_device_id != ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO)
        return 0;

    ac = audio_client_alloc(0);
    if (!ac)
        return 0;

    ac->flags = AUDIO_FLAG_WRITE;
    audio_rx_path_enable(1, 0);
    enable_aux_loopback(1);

    return ac;
}

int q6fm_close(struct audio_client *ac)
{
     AUDIO_INFO("%s\n", __func__);
    printk("q6fm_close\n");

    audio_rx_path_enable(0, 0);
    enable_aux_loopback(0);
    audio_client_free(ac);
    return 0;
}

struct audio_client *q6audio_open_qcelp(uint32_t bufsz, uint32_t rate,
                      void *data, uint32_t acdb_id)
{
    struct audio_client *ac;

    AUDIO_INFO("%s\n", __func__);
    if (q6audio_init())
        return 0;

    ac = audio_client_alloc(bufsz);
    if (!ac)
    {
        return 0;
    }

    ac->flags = AUDIO_FLAG_READ;
    tx_clk_freq = rate;
    audio_tx_path_enable(1, acdb_id);

    audio_qcelp_open(ac, bufsz, data);
    audio_command(ac, ADSP_AUDIO_IOCTL_CMD_SESSION_START);

    ac->buf[0].used = 1;
    ac->buf[1].used = 1;
    q6audio_read(ac, &ac->buf[0]);
    q6audio_read(ac, &ac->buf[1]);

    audio_prevent_sleep();
    update_all_audio_tx_volume(ac_control);	
    return ac;
}

int q6audio_qcelp_close(struct audio_client *ac)
{
    AUDIO_INFO("%s\n", __func__);
    audio_close(ac);
    audio_tx_path_enable(0, 0);
    audio_client_free(ac);
    audio_allow_sleep();
    return 0;
}

///////////////////////////////////////////////////////////////////////////

int acdb_get_table(int dev_id, int sample_rate)
{
        struct acdb_cmd_device_table rpc;
        struct acdb_result res;
        int r;

        AUDIO_INFO("%s\n", __func__);
        memset(audio_data, 0, 4096);
        memset(&rpc, 0, sizeof(rpc));

        rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
        rpc.command_id = ACDB_GET_DEVICE_TABLE;
        rpc.device_id = dev_id;
        rpc.sample_rate_id = sample_rate;
        rpc.total_bytes = 4096;
        rpc.unmapped_buf = audio_phys;
        rpc.res_size = sizeof(res) - (2 * sizeof(uint32_t));

        r = dal_call(acdb, ACDB_OP_IOCTL, 8, &rpc, sizeof(rpc), &res, sizeof(res));

        if ((r == sizeof(res)) && (res.dal_status == 0)) 
        {
            pr_info("acdb: %d bytes for device %d, rate %d.\n",
                res.used_bytes, dev_id, sample_rate);
            return res.used_bytes;
        }
        return 0;
}

static struct audio_client * audio_test(void)
{
    struct audio_client *ac = 0;
    struct audio_client *ac2 = 0;
    int size;
    struct rpc_info info;

    AUDIO_INFO("%s\n", __func__);
    pr_info("audio: init: codecs\n");
    icodec_rx_clk = clk_get(0, "icodec_rx_clk");
    icodec_tx_clk = clk_get(0, "icodec_tx_clk");
    ecodec_clk = clk_get(0, "ecodec_clk");
    sdac_clk = clk_get(0, "sdac_clk");
    audio_data = dma_alloc_coherent(NULL, 4096, &audio_phys, GFP_KERNEL);
    params_data = dma_alloc_coherent(NULL, 4096, &params_phys, GFP_KERNEL);
    printk("allocated: %p %08X\n", params_data, params_phys); 

            clk_set_rate(icodec_rx_clk, 12288000);
            clk_enable(icodec_rx_clk);
            clk_set_rate(ecodec_clk, 2048000);
            clk_enable(ecodec_clk);
            clk_set_rate(sdac_clk, 12288000);
            clk_enable(sdac_clk);

    // 1.  attach ADIE
    adie = dal_attach_ex(ADIE_DAL_DEVICE, "NULL", ADIE_DAL_PORT , 0, 0);
    if (!adie) 
    {
        pr_err("audio_init: cannot attach to adie\n");
        return 0;
    }

    if (analog_ops->init)
        analog_ops->init();

    audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO; 
    audio_rx_analog_enable(1);

    dal_call_f9(adie, DAL_OP_INFO,  &info, sizeof(info));    


    acdb = dal_attach(ACDB_DAL_DEVICE, ACDB_DAL_PORT, 0, 0);
    if (!acdb) 
    {
        pr_err("audio_init: cannot attach to acdb channel\n");
        return 0;
    }

    dal_call_f9(acdb, DAL_OP_INFO,  &info, sizeof(info));    

    audio_client_alloc(0);

   // audio_client_alloc(0);        

    ac = audio_client_alloc(0);    

    audio_open_control(ac);    
//mdelay(1000);

    audio_rx_mute(ac, ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR, 0);
    audio_rx_volume(ac, ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR, 0);
    
    acdb_get_table(7, 48000);

    audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR;
    qdsp6_devchg_notify(ac, ADSP_AUDIO_RX_DEVICE, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO);

    qdsp6_standby(ac);
    qdsp6_start(ac);

    size = acdb_get_table(0x25F, 48000);    
    audio_set_table(ac, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO, size);

    audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO;
    qdsp6_devchg_notify(ac, ADSP_AUDIO_RX_DEVICE, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO);

    qdsp6_standby(ac);    
    qdsp6_start(ac);

    audio_rx_mute(ac, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO, 0);
    audio_rx_volume(ac, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO, 500);


    size = acdb_get_table(507, 8000);    
    audio_set_table(ac, ADSP_AUDIO_DEVICE_ID_HANDSET_MIC, size);

    audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_MIC;
    qdsp6_devchg_notify(ac, ADSP_AUDIO_TX_DEVICE, ADSP_AUDIO_DEVICE_ID_HANDSET_MIC);

    qdsp6_standby(ac);    
    qdsp6_start(ac);

    size = acdb_get_table(0x25F, 48000);    
    audio_set_table(ac, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO, size);

    ac2 = audio_client_alloc(4096);    

    audio_out_open(ac2, 4096, 44100, 2);

    adie_open(adie);

    adie_set_path(adie, ADIE_PATH_SPEAKER_RX, ADIE_PATH_RX);
    adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);
    adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_READY);
    adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_ANALOG_READY);

    audio_command(ac2, ADSP_AUDIO_IOCTL_CMD_SESSION_START);

    audio_rx_mute(ac, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO, 0);
    audio_rx_volume(ac, ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO, 500);

    return ac2;
}


// END OF FILE
