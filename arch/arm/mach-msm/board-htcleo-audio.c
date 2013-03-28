/* arch/arm/mach-msm/board-htcleo-audio.c
 *
 * Copyright (C) 2010 Cotulla
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

#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/msm_qdsp6_audio_1550.h>
#include <mach/htc_acoustic_qsd.h>
#include <asm/gpio.h>
#include <mach/gpio.h>
#include <mach/htc_headset_mgr.h>
#include <mach/board-htcleo-audio.h>

#include "board-htcleo.h"
#include "devices.h"
#include "dex_comm.h"
#include "proc_comm.h"
#include "pmic.h"

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define PROCFS_MAX_SIZE     1024
#define PROCFS_NAME         "mic_level"

#if 1
#define D(fmt, args...) printk(KERN_INFO "Audio: "fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

static struct proc_dir_entry* mic_gain_file;

static struct mutex mic_lock;
static struct mutex bt_sco_lock;

#if 1

// LEO
static struct q6_hw_info q6_audio_hw[Q6_HW_COUNT] =
{
    [Q6_HW_HANDSET] =
	{
        .min_gain = -2000,
        .max_gain = 500,
    },
    [Q6_HW_HEADSET] =
	{
        .min_gain = -2000,
        .max_gain = 500,
    },
    [Q6_HW_SPEAKER] =
	{
        .min_gain = -1500,
        .max_gain = 700,
    },
    [Q6_HW_TTY] =
	{
        .min_gain = 0,
        .max_gain = 0,
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

// old desire one
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

//-----------proc file interface--------------------------------
/**
 * This function is called then the /proc file is read
 *
 */
int 
mic_level_read(char *buffer,
          char **buffer_location,
          off_t offset, int buffer_length, int *eof, void *data)
{
    int ret;
    char temp_buff[1024];
    printk(KERN_INFO "mic_level_read (/proc/%s) called\n", PROCFS_NAME);

    if (offset > 0) {
        /* we have finished to read, return 0 */
        ret  = 0;
    } else {
        /* fill the buffer, return the buffer size */
        sprintf(buffer, "Usage: echo 'device_id level' > /proc/mic_level\n");
        strcat(buffer, "Level range: 0-1000; -1 use level from acdb\n");
        strcat(buffer, "Current levels (device_id):\n");
        sprintf(temp_buff, "HANDSET (%d): %d\n", DEVICE_ID_HANDSET_MIC, q6audio_get_tx_dev_volume(DEVICE_ID_HANDSET_MIC));
        strcat(buffer, temp_buff);
        sprintf(temp_buff, "SPKR_PHONE (%d): %d\n", DEVICE_ID_SPKR_PHONE_MIC, q6audio_get_tx_dev_volume(DEVICE_ID_SPKR_PHONE_MIC));
        strcat(buffer, temp_buff);
        sprintf(temp_buff, "HEADSET (%d): %d\n", DEVICE_ID_HEADSET_MIC, q6audio_get_tx_dev_volume(DEVICE_ID_HEADSET_MIC));
        strcat(buffer, temp_buff);
        sprintf(temp_buff, "TTY_HEADSET (%d): %d\n", DEVICE_ID_TTY_HEADSET_MIC, q6audio_get_tx_dev_volume(DEVICE_ID_TTY_HEADSET_MIC));
        strcat(buffer, temp_buff);
        sprintf(temp_buff, "BT_SCO (%d): %d\n", DEVICE_ID_BT_SCO_MIC, q6audio_get_tx_dev_volume(DEVICE_ID_BT_SCO_MIC));
        strcat(buffer, temp_buff);
        ret = strlen(buffer);
    }
    return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int mic_level_write(struct file *file, const char *buffer, unsigned long count,
		   void *data)
{
    char temp_buff[512];
    int device_id=0, level=0, ret, procfs_buffer_size;
    /* get buffer size */
    procfs_buffer_size = count;
    if (procfs_buffer_size > 512 ) {
        procfs_buffer_size = 512;
    }

    /* write data to the buffer */
    if ( copy_from_user(temp_buff, buffer, procfs_buffer_size) ) {
        return -EFAULT;
    }
    sscanf(temp_buff, "%d %d", &device_id, &level);
    ret  = q6audio_set_tx_dev_volume(device_id, level);
    if (ret<0)
        return -EFAULT;
    procfs_buffer_size=strlen(temp_buff);
    return procfs_buffer_size;
}
//--------------------------------------------------------------

void htcleo_headset_enable(int en)
{
    D("%s %d\n", __func__, en);
    /* enable audio amp */
    if (en) mdelay(60);
    gpio_set_value(HTCLEO_AUD_JACKHP_EN, !!en);
}

void htcleo_speaker_enable(int en)
{
    struct spkr_config_mode scm;
    memset(&scm, 0, sizeof(scm));

    D("%s %d\n", __func__, en);
    if (en) 
    {
        scm.is_right_chan_en = 0;
        scm.is_left_chan_en = 1;
        scm.is_stereo_en = 0;
        scm.is_hpf_en = 0; //1;  // CotullaTODO: check it
        pmic_spkr_en_mute(LEFT_SPKR, 0);
        pmic_spkr_en_mute(RIGHT_SPKR, 0);
        pmic_set_spkr_configuration(&scm);
        pmic_spkr_set_mux_hpf_corner_freq(SPKR_FREQ_0_76KHZ);
        pmic_spkr_en(LEFT_SPKR, 1);
        pmic_spkr_en(RIGHT_SPKR, 0);

        pmic_spkr_en_hpf(ON_CMD); // +LEO

        /* unmute */
        pmic_spkr_en_mute(LEFT_SPKR, 1);
        mdelay(40);
    } 
    else 
    {
        pmic_spkr_en_mute(LEFT_SPKR, 0);

        pmic_spkr_en_hpf(OFF_CMD); // +LEO
        pmic_spkr_en(LEFT_SPKR, 0);
        pmic_spkr_en(RIGHT_SPKR, 0);

        pmic_set_spkr_configuration(&scm);
    }
}

void htcleo_receiver_enable(int en)
{
//    if (is_cdma_version(system_rev) &&
//        ((system_rev == 0xC1) || (system_rev == 0xC2))) {
        struct spkr_config_mode scm;
        memset(&scm, 0, sizeof(scm));

        D("%s %d\n", __func__, en);
        if (en) 
        {
            scm.is_right_chan_en = 1;
            scm.is_left_chan_en = 0;
            scm.is_stereo_en = 0;
            scm.is_hpf_en = 1;
            pmic_spkr_en_mute(RIGHT_SPKR, 0);
            pmic_set_spkr_configuration(&scm);
            pmic_spkr_en(RIGHT_SPKR, 1);

            /* unmute */
            pmic_spkr_en_mute(RIGHT_SPKR, 1);
        } 
        else 
        {
            pmic_spkr_en_mute(RIGHT_SPKR, 0);

            pmic_spkr_en(RIGHT_SPKR, 0);

            pmic_set_spkr_configuration(&scm);
   
     }
 //   }
}
           
static uint32_t bt_sco_enable[] = 
{
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_OUT, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_IN, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_SYNC, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_CLK, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA)
};

static uint32_t bt_sco_disable[] = 
{
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_OUT, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_IN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_SYNC, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
    PCOM_GPIO_CFG(HTCLEO_BT_PCM_CLK, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA)
};

void htcleo_bt_sco_enable(int en)
{
    static int bt_sco_refcount;
    D("%s %d\n", __func__, en);

    mutex_lock(&bt_sco_lock);
    if (en) 
    {
        if (++bt_sco_refcount == 1)
        {
            config_gpio_table(bt_sco_enable, ARRAY_SIZE(bt_sco_enable));
        }        
    } 
    else 
    {
        if (--bt_sco_refcount == 0) 
        {
            config_gpio_table(bt_sco_disable, ARRAY_SIZE(bt_sco_disable));
            gpio_set_value(HTCLEO_BT_PCM_OUT, 0);
        }
    }
    mutex_unlock(&bt_sco_lock);
}

void htcleo_mic_enable(int en)
{
    static int old_state = 0, new_state = 0;

    D("%s %d\n", __func__, en);

    mutex_lock(&mic_lock);
    if (!!en)
        new_state++;
    else
        new_state--;

    if (new_state == 1 && old_state == 0) 
    {
        gpio_set_value(HTCLEO_AUD_2V5_EN, 1);
        mdelay(60);
    } 
    else if (new_state == 0 && old_state == 1)
    {
        gpio_set_value(HTCLEO_AUD_2V5_EN, 0);
    }
    else
    {
        D("%s: do nothing %d %d\n", __func__, old_state, new_state);
    }

    old_state = new_state;
    mutex_unlock(&mic_lock);
}


void htcleo_analog_init(void)
{
    D("%s\n", __func__);
    /* stereo pmic init */
    pmic_spkr_set_gain(LEFT_SPKR, SPKR_GAIN_PLUS12DB);
    pmic_spkr_set_gain(RIGHT_SPKR, SPKR_GAIN_PLUS12DB);
    pmic_spkr_en_right_chan(OFF_CMD);
    pmic_spkr_en_left_chan(OFF_CMD);
    pmic_spkr_add_right_left_chan(OFF_CMD);
    pmic_spkr_en_stereo(OFF_CMD);
    pmic_spkr_select_usb_with_hpf_20hz(OFF_CMD);
    pmic_spkr_bypass_mux(OFF_CMD);
    pmic_spkr_en_hpf(OFF_CMD);
    pmic_spkr_en_sink_curr_from_ref_volt_cir(OFF_CMD);
    pmic_spkr_set_mux_hpf_corner_freq(SPKR_FREQ_0_76KHZ);
    pmic_mic_set_volt(MIC_VOLT_1_80V);

    gpio_request(HTCLEO_AUD_JACKHP_EN, "aud_jackhp_en");
    gpio_request(HTCLEO_BT_PCM_OUT, "bt_pcm_out");

    gpio_direction_output(HTCLEO_AUD_JACKHP_EN, 0);

    mutex_lock(&bt_sco_lock);
    config_gpio_table(bt_sco_disable, ARRAY_SIZE(bt_sco_disable));
    gpio_direction_output(HTCLEO_BT_PCM_OUT, 0);
    mutex_unlock(&bt_sco_lock);

}

int htcleo_get_rx_vol(uint8_t hw, int level)
{
    int vol;
    struct q6_hw_info *info;

    if (level > 100)
        level = 100;
    else if (level < 0)
        level = 0;

    // TODO: is it correct?
    info = &q6_audio_hw[hw];
    vol = info->min_gain + ((info->max_gain - info->min_gain) * level) / 100;

    D("%s %d\n", __func__, vol);
    return vol;
}

static struct qsd_acoustic_ops acoustic = 
{
    .enable_mic_bias = htcleo_mic_enable,
};

static struct q6audio_analog_ops ops = 
{
    .init = htcleo_analog_init,
    .speaker_enable = htcleo_speaker_enable,
    .headset_enable = htcleo_headset_enable,
    .receiver_enable = htcleo_receiver_enable,
    .bt_sco_enable = htcleo_bt_sco_enable,
    .int_mic_enable = htcleo_mic_enable,
    .ext_mic_enable = htcleo_mic_enable,
    .get_rx_vol = htcleo_get_rx_vol,
};

static void hs_mic_register(void)
{
	struct headset_notifier notifier;
	notifier.id = HEADSET_REG_MIC_BIAS;
	notifier.func = htcleo_mic_enable;
	headset_notifier_register(&notifier);
}

void __init htcleo_audio_init(void)
{
    mutex_init(&mic_lock);
    mutex_init(&bt_sco_lock);
    q6audio_register_analog_ops(&ops);
    acoustic_register_ops(&acoustic);
    hs_mic_register();

    mic_gain_file = create_proc_entry(PROCFS_NAME, 0644, NULL);

    if (mic_gain_file == NULL) {
        remove_proc_entry(PROCFS_NAME, NULL);
        printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
            PROCFS_NAME);
        return;
    }

    mic_gain_file->read_proc  = mic_level_read;
    mic_gain_file->write_proc = mic_level_write;
    //mic_gain_file->mode       = S_IFREG | S_IRUGO;
    mic_gain_file->uid        = 0;
    mic_gain_file->gid        = 0;

    printk(KERN_INFO "/proc/%s created\n", PROCFS_NAME);
//        q6audio_set_acdb_file("default_PMIC.acdb");
}

// END OF FILE
