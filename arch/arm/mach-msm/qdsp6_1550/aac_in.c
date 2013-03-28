/* arch/arm/mach-msm/qdsp6/aac_in.c
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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/msm_audio_1550.h>
#include <mach/msm_qdsp6_audio_1550.h>

#define BUFSZ (4096)
#define DMASZ (BUFSZ * 2)

#if 0
#define TRACE(x...) pr_info("Q6: "x)
#else
#define TRACE(x...) do{}while(0)
#endif

static DEFINE_MUTEX(aac_in_lock);
static int aac_in_opened = 0;
static struct aac_format *af;

void audio_client_dump(struct audio_client *ac);

static long aac_in_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
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
		} else if (copy_from_user(&acdb_id,
				(void*) arg, sizeof(acdb_id))) {
			rc = -EFAULT;
			break;
		}

		mutex_lock(&aac_in_lock);
		if (file->private_data) {
			rc = -EBUSY;
		} else {
			file->private_data = q6audio_open_aac(
				BUFSZ, 48000, AUDIO_FLAG_READ, af, acdb_id);
			if (!file->private_data)
				rc = -ENOMEM;
		}
		mutex_unlock(&aac_in_lock);
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
		if (config.sample_rate != 48000)
			pr_info("only 48KHz AAC encode supported\n");
		af->channel_config = config.channel_count;
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = BUFSZ;
		config.buffer_count = 2;
		config.sample_rate = 48000;
		config.channel_count = af->channel_config;
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

static int aac_in_open(struct inode *inode, struct file *file)
{
	int rc;

	pr_info("aac_in: open\n");
	mutex_lock(&aac_in_lock);
	if (aac_in_opened) {
		pr_err("aac_in: busy\n");
		rc = -EBUSY;
	} else {
		af = kzalloc(sizeof(*af), GFP_KERNEL);
		memset(af, 0, sizeof(struct aac_format));
		af->sample_rate = 3; /* 48000 */
		af->channel_config = 1;
		af->block_formats = AUDIO_AAC_FORMAT_ADTS;
		af->audio_object_type = 2; /* CAD to ADSP format */
		af->bit_rate = 192000;

		aac_in_opened = 1;
		rc = 0;
	}
	mutex_unlock(&aac_in_lock);
	return rc;
}

static ssize_t aac_in_read(struct file *file, char __user *buf,
			   size_t count, loff_t *pos)
{
	struct audio_client *ac;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer, res = 0;

	mutex_lock(&aac_in_lock);
	ac = file->private_data;
	if (!ac) {
		res = -ENODEV;
		goto fail;
	}
	while (count > 0) {
		ab = ac->buf + ac->cpu_buf;

		if (ab->used)
			if (!wait_event_timeout(ac->wait, (ab->used == 0), 5*HZ)) {
				audio_client_dump(ac);
				pr_err("aac_read: timeout. dsp dead?\n");
				BUG();
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
	mutex_unlock(&aac_in_lock);
	return res;
}

static int aac_in_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	pr_info("aac_in: release\n");
	mutex_lock(&aac_in_lock);
	if (file->private_data)
		rc = q6audio_close(file->private_data);
	kfree(af);
	aac_in_opened = 0;
	mutex_unlock(&aac_in_lock);
	return rc;
}

static struct file_operations aac_in_fops = {
	.owner	= THIS_MODULE,
	.open	= aac_in_open,
	.read	= aac_in_read,
	.release  = aac_in_release,
	.unlocked_ioctl	= aac_in_ioctl,
};

struct miscdevice aac_in_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msm_aac_in",
	.fops = &aac_in_fops,
};

static int __init aac_in_init(void) {
	return misc_register(&aac_in_misc);
}

device_initcall(aac_in_init);
