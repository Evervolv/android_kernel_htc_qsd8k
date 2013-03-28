/* linux/arch/arm/mach-msm/board-primou-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <mach/tpa2051d3.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <mach/dal.h>
#include "board-primou.h"
#include <mach/qdsp5v2_2x/snddev_icodec.h>
#include <mach/qdsp5v2_2x/snddev_ecodec.h>
#include <mach/qdsp5v2_2x/audio_def.h>
#include <mach/qdsp5v2_2x/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <mach/htc_acdb_7x30.h>

static struct mutex bt_sco_lock;
static struct workqueue_struct *audio_wq;
static void audio_work_func(struct work_struct *work);
static DECLARE_WORK(audio_work, audio_work_func);
static atomic_t beats_enabled = ATOMIC_INIT(0);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)
#define PMGPIO(x) (x-1)
#define PRIMOU_ACDB_SMEM_SIZE        (0xE000)
#define PRIMOU_ACDB_RADIO_BUFFER_SIZE (1024 * 3072)

static struct vreg *vreg_audio_n1v8;

static struct q5v2_hw_info q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_gain[VOC_NB_INDEX] = 600,
		.min_gain[VOC_NB_INDEX] = -1400,
		.max_gain[VOC_WB_INDEX] = 600,
		.min_gain[VOC_WB_INDEX] = -1400,
	},
	[Q5V2_HW_HEADSET] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = -2000,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = -2000,
	},
	[Q5V2_HW_SPEAKER] = {
		.max_gain[VOC_NB_INDEX] = 600,
		.min_gain[VOC_NB_INDEX] = -900,
		.max_gain[VOC_WB_INDEX] = 600,
		.min_gain[VOC_WB_INDEX] = -900,
	},
	[Q5V2_HW_BT_SCO] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = -1500,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = -1500,
	},
	[Q5V2_HW_TTY] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_HS_SPKR] = {
		.max_gain[VOC_NB_INDEX] = -500,
		.min_gain[VOC_NB_INDEX] = -2000,
		.max_gain[VOC_WB_INDEX] = -500,
		.min_gain[VOC_WB_INDEX] = -2000,
	},
	[Q5V2_HW_USB_HS] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_HAC] = {
		.max_gain[VOC_NB_INDEX] = 100,
		.min_gain[VOC_NB_INDEX] = -1900,
		.max_gain[VOC_WB_INDEX] = 100,
		.min_gain[VOC_WB_INDEX] = -1900,
	},
};

static unsigned aux_pcm_gpio_off[] = {
	GPIO_CFG(138, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_CLK  */
};


static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

void primou_hs_n1v8_enable(int en)
{
	int rc = 0;

	pr_aud_info("%s: %d\n", __func__, en);

	if (!vreg_audio_n1v8) {

		vreg_audio_n1v8 = vreg_get(NULL, "ncp");

		if (IS_ERR(vreg_audio_n1v8)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_audio_n1v8));
			rc = PTR_ERR(vreg_audio_n1v8);
			goto  vreg_aud_hp_1v8_faill;
		}
	}

	if (en) {
		rc = vreg_enable(vreg_audio_n1v8);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
			goto vreg_aud_hp_1v8_faill;
		}
	} else {
		rc = vreg_disable(vreg_audio_n1v8);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
			goto vreg_aud_hp_1v8_faill;
		}
	}
	return;
vreg_aud_hp_1v8_faill:
	printk(KERN_ERR "%s: failed (%ld)\n",
		__func__, PTR_ERR(vreg_audio_n1v8));
}

void primou_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_SPK_SD), 1);
		mdelay(100);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_SPK_SD), 0);
	}
}

void primou_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
}

void primou_snddev_pre_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		primou_hs_n1v8_enable(1);
	} else {
		primou_hs_n1v8_enable(0);
	}
}

void primou_snddev_hs_spk_pamp_on(int en)
{
	primou_snddev_poweramp_on(en);
	primou_snddev_hsed_pamp_on(en);
}

void primou_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1)
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));

			gpio_set_value(PRIMOU_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(PRIMOU_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(PRIMOU_GPIO_BT_PCM_CLK, 0);

		}
	}
	mutex_unlock(&bt_sco_lock);
}

void primou_snddev_imic_pamp_on(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en) {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		mdelay(60);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
	}
}

void primou_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
#if 0
	if (en)
		gpio_request(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_CODEC_EN), "aud_2v85_en");
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_CODEC_EN), 1);
		gpio_set_value(PRIMOU_AUD_CODEC_EN, 1);
	else
		gpio_set_value(PRIMOU_AUD_CODEC_EN, 0);
#endif
}

int primou_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info *info;
	int vol, maxv, minv;

	info = &q5v2_audio_hw[hw];
	maxv = info->max_gain[network];
	minv = info->min_gain[network];
	vol = minv + ((maxv - minv) * level) / 100;
	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void primou_mic_bias_enable(int en, int shift)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en) {
		gpio_set_value(PRIMOU_AUD_CODEC_EN, 1);
	} else {
		gpio_set_value(PRIMOU_AUD_CODEC_EN, 0);
	}
}

uint32_t primou_get_smem_size(void)
{
	return PRIMOU_ACDB_SMEM_SIZE;
}

uint32_t primou_get_acdb_radio_buffer_size(void)
{
	return PRIMOU_ACDB_RADIO_BUFFER_SIZE;
}

int primou_support_aic3254(void)
{
	return 0;
}

int primou_support_adie(void)
{
	return 1;
}

int primou_support_back_mic(void)
{
	return 0;
}


int primou_support_beats(void)
{
	return 1;
}

static void audio_work_func(struct work_struct *work)
{
	int en = atomic_read(&beats_enabled);
	int gain;

	pr_aud_info("%s: %d\n", __func__, en);

	if (en) {
		for (gain = 0x10; gain >= 0x4; gain -= 0x4) {
			adie_codec_set_device_analog_volume(NULL, 2, gain);
			if (gain > 0x4)
				mdelay(50);
		}
	} else {
		for (gain = 0x08; gain <= 0x14; gain += 0x4) {
			adie_codec_set_device_analog_volume(NULL, 2, gain);
			if (gain < 0x14)
				mdelay(30);
		}
	}
}

void primou_enable_beats(int en)
{
#if 0
	pr_aud_info("%s: %d\n", __func__, en);
	if (!audio_wq) {
		pr_aud_info("%s: non-wq case\n", __func__);
		if (en)
			adie_codec_set_device_analog_volume(NULL, 2, 0x04);
		else
			adie_codec_set_device_analog_volume(NULL, 2, 0x14);
	} else {
		pr_aud_info("%s: wq case\n", __func__);
		atomic_set(&beats_enabled, en);
		queue_work(audio_wq, &audio_work);
	}
#endif
}

static struct q5v2audio_icodec_ops iops = {
	.support_aic3254 = primou_support_aic3254,
	.support_adie = primou_support_adie,
};

static struct acdb_ops acdb = {
	.get_acdb_radio_buffer_size = primou_get_acdb_radio_buffer_size,
};

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= primou_snddev_poweramp_on,
	.headset_enable	= primou_snddev_hsed_pamp_on,
	.bt_sco_enable = primou_snddev_bt_sco_pamp_on,
	.headset_speaker_enable = primou_snddev_hs_spk_pamp_on,
	.int_mic_enable = primou_snddev_imic_pamp_on,
	.ext_mic_enable = primou_snddev_emic_pamp_on,
	.fm_headset_enable = primou_snddev_hsed_pamp_on,
	.fm_speaker_enable = primou_snddev_poweramp_on,
	.qtr_headset_enable = primou_snddev_pre_hsed_pamp_on
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = primou_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = primou_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = primou_mic_bias_enable,
	.support_aic3254 = primou_support_aic3254,
	.support_back_mic = primou_support_back_mic,
	.support_beats = primou_support_beats,
	.enable_beats = primou_enable_beats
};

void __init primou_audio_init(void)
{

	mutex_init(&bt_sco_lock);

#ifdef CONFIG_MSM7KV2_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_icodec_ops(&iops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
	acdb_register_ops(&acdb);
#endif

	gpio_request(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_SPK_SD), "AUD_SPK_EN");
	gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_SPK_SD), 1);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_SPK_SD), 0);
	gpio_request(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_CODEC_EN), "aud_2v85_en");
	gpio_direction_output(PM8058_GPIO_PM_TO_SYS(PRIMOU_AUD_CODEC_EN), 1);
	gpio_set_value(PRIMOU_AUD_CODEC_EN, 0);

	primou_hs_n1v8_enable(0);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(PRIMOU_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(PRIMOU_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(PRIMOU_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);

	audio_wq = create_workqueue("AUDIO_EFFECT_VOLUME");
	if (audio_wq == NULL) {
		pr_aud_info("%s: cannot create workqueue\n", __func__);
	}

}
