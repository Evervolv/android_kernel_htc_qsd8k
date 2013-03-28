/* arch/arm/mach-msm/qdsp6/pcm_in.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
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

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio_1550.h>

#define BUFSZ (256)

#ifdef CONFIG_QSD_AUDIO_CALLREC

#define AUDIO_FLAG_INCALL_MIXED       2

struct pcm {
	struct audio_client *ac;
	uint32_t sample_rate;
	uint32_t channel_count;
	uint32_t buffer_size;
	uint32_t rec_mode;
};
#else
static DEFINE_MUTEX(pcm_in_lock);
static uint32_t sample_rate = 8000;
static uint32_t channel_count = 1;
static uint32_t buffer_size = BUFSZ;
static int pcm_in_opened = 0;
#endif

void audio_client_dump(struct audio_client *ac);

static long q6_in_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef CONFIG_QSD_AUDIO_CALLREC
	struct pcm *pcm = file->private_data;
#endif
	int rc = 0;

	switch (cmd) {
	case AUDIO_SET_VOLUME:
		break;
	case AUDIO_GET_STATS: {
		struct msm_audio_stats stats;
		memset(&stats, 0, sizeof(stats));
		if (copy_to_user((void*) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}
	case AUDIO_START: {
		uint32_t acdb_id;
		rc = 0;

		if (arg == 0) {
			acdb_id = 0;
		} else if (copy_from_user(&acdb_id, (void*) arg, sizeof(acdb_id))) {
			rc = -EFAULT;
			break;
		}

#ifndef CONFIG_QSD_AUDIO_CALLREC
		mutex_lock(&pcm_in_lock);
		if (file->private_data) {
#else
		if (pcm->ac) {
#endif
			rc = -EBUSY;
		} else {
#ifdef CONFIG_QSD_AUDIO_CALLREC
			pcm->ac = q6audio_open_pcm(pcm->buffer_size,
					pcm->sample_rate, pcm->channel_count,
					pcm->rec_mode, acdb_id);
			if (!pcm->ac)
				rc = -ENOMEM;
#else
			file->private_data = q6audio_open_pcm(
				BUFSZ, sample_rate, channel_count, AUDIO_FLAG_READ, acdb_id);
			if (!file->private_data)
				rc = -ENOMEM;
#endif
		}
#ifndef CONFIG_QSD_AUDIO_CALLREC
		mutex_unlock(&pcm_in_lock);
#endif
		break;
	}
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (copy_from_user(&config, (void*) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
#ifdef CONFIG_QSD_AUDIO_CALLREC
		pcm->sample_rate = config.sample_rate;
		pcm->channel_count = config.channel_count;
		pcm->buffer_size = config.buffer_size;
#else
		sample_rate = config.sample_rate;
		channel_count = config.channel_count;
		buffer_size = config.buffer_size;
#endif
		break;
	}
#ifdef CONFIG_QSD_AUDIO_CALLREC
	case AUDIO_SET_INCALL: {
		struct msm_voicerec_mode voicerec_mode;
		if (copy_from_user(&voicerec_mode, (void *)arg,
			sizeof(struct msm_voicerec_mode)))
			return -EFAULT;
		if (voicerec_mode.rec_mode != AUDIO_FLAG_READ &&
			voicerec_mode.rec_mode != AUDIO_FLAG_INCALL_MIXED) {
			pcm->rec_mode = AUDIO_FLAG_READ;
			pr_err("invalid rec_mode\n");
			rc = -EINVAL;
		} else
			pcm->rec_mode = voicerec_mode.rec_mode;
		break;
	}
#endif
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_count = 2;
#ifdef CONFIG_QSD_AUDIO_CALLREC
		config.buffer_size = pcm->buffer_size;
		config.sample_rate = pcm->sample_rate;
		config.channel_count = pcm->channel_count;
#else
		config.buffer_size = buffer_size;
		config.sample_rate = sample_rate;
		config.channel_count = channel_count;
#endif
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		if (copy_to_user((void*) arg, &config, sizeof(config))) {
			rc = -EFAULT;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int q6_in_open(struct inode *inode, struct file *file)
{
#ifdef CONFIG_QSD_AUDIO_CALLREC
	struct pcm *pcm;

	pr_info("pcm_in: open\n");
	pcm = kzalloc(sizeof(struct pcm), GFP_KERNEL);

	if (!pcm)
		return -ENOMEM;

	pcm->channel_count = 1;
	pcm->sample_rate = 8000;
	pcm->buffer_size = BUFSZ;
	pcm->rec_mode = AUDIO_FLAG_READ;
	file->private_data = pcm;
	return 0;
#else
	int rc;

	pr_info("pcm_in: open\n");
	mutex_lock(&pcm_in_lock);
	if (pcm_in_opened) {
		pr_err("pcm_in: busy\n");
		rc = -EBUSY;
	} else {
		pcm_in_opened = 1;
		rc = 0;
	}
	mutex_unlock(&pcm_in_lock);
	return rc;
#endif
}

static ssize_t q6_in_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos)
{
#ifdef CONFIG_QSD_AUDIO_CALLREC
	struct pcm *pcm = file->private_data;
#endif
	struct audio_client *ac;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer;
	int res;

#ifndef CONFIG_QSD_AUDIO_CALLREC
	mutex_lock(&pcm_in_lock);
	ac = file->private_data;
#else
	ac = pcm->ac;
#endif
	if (!ac) {
		res = -ENODEV;
		goto fail;
	}
	while (count > 0) {
		ab = ac->buf + ac->cpu_buf;

		if (ab->used)
			if (!wait_event_timeout(ac->wait, (ab->used == 0), 5*HZ)) {
				audio_client_dump(ac);
				pr_err("pcm_read: timeout. dsp dead?\n");
				//BUG();
				res = -EFAULT;
				goto fail;
			}

		xfer = count;
		if (xfer > ab->size)
			xfer = ab->size;

		if (copy_to_user(buf, ab->data, xfer)) {
			res = -EFAULT;
			goto fail;
		}

		buf += xfer;
		count -= xfer;

		ab->used = 1;
		q6audio_read(ac, ab);
		ac->cpu_buf ^= 1;
	}
fail:
	res = buf - start;
#ifndef CONFIG_QSD_AUDIO_CALLREC
	mutex_unlock(&pcm_in_lock);
#endif
	return res;
}

static int q6_in_release(struct inode *inode, struct file *file)
{
	int rc = 0;
#ifdef CONFIG_QSD_AUDIO_CALLREC
	struct pcm *pcm = file->private_data;
	if (pcm->ac)
		rc = q6audio_close(pcm->ac);
	kfree(pcm);
#else
	mutex_lock(&pcm_in_lock);
	if (file->private_data)
		rc = q6audio_close(file->private_data);
	pcm_in_opened = 0;
	mutex_unlock(&pcm_in_lock);
#endif
	pr_info("pcm_in: release\n");
	return rc;
}

static struct file_operations q6_in_fops = {
	.owner		= THIS_MODULE,
	.open		= q6_in_open,
	.read		= q6_in_read,
	.release	= q6_in_release,
	.unlocked_ioctl	= q6_in_ioctl,
};

struct miscdevice q6_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_in",
	.fops	= &q6_in_fops,
};

static int __init q6_in_init(void) {
	return misc_register(&q6_in_misc);
}

device_initcall(q6_in_init);
