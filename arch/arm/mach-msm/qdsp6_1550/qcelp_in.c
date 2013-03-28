/* arch/arm/mach-msm/qdsp6/qcelp_in.c
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
#include <linux/msm_audio.h>
#include <mach/msm_qdsp6_audio_1550.h>
#include <mach/msm_audio_qcp.h>

#define BUFSZ (734)
#define DMASZ (BUFSZ * 2)

#if 0
#define TRACE(x...) pr_info("Q6: "x)
#else
#define TRACE(x...) do{}while(0)
#endif

static DEFINE_MUTEX(qcelp_in_lock);
static int qcelp_in_opened = 0;
static struct msm_audio_qcelp_config *qf;

void audio_client_dump(struct audio_client *ac);

static long qcelp_in_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
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
		} else if (copy_from_user(&acdb_id, (void*) arg, sizeof(acdb_id))) {
			rc = -EFAULT;
			break;
		}

		mutex_lock(&qcelp_in_lock);
		if (file->private_data) {
			rc = -EBUSY;
		} else {
			file->private_data = q6audio_open_qcelp(
				BUFSZ, 8000, qf, acdb_id);
			if (!file->private_data)
				rc = -ENOMEM;
		}
		mutex_unlock(&qcelp_in_lock);
		break;
	}
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_GET_CONFIG:
		if (copy_to_user((void *)arg, qf,
				sizeof(struct msm_audio_qcelp_config)))
			return -EFAULT;
		break;
	case AUDIO_SET_CONFIG:
		if (copy_from_user(qf, (void *)arg,
				sizeof(struct msm_audio_qcelp_config)))
			return -EFAULT;
		if (qf->min_bit_rate > 4 || qf->min_bit_rate < 1) {
			pr_err("invalid min bitrate\n");
			return -EINVAL;
		}
		if (qf->max_bit_rate > 4 || qf->max_bit_rate < 1) {
			pr_err("invalid max bitrate\n");
			return -EINVAL;
		}
		if (qf->cdma_rate > CDMA_RATE_ERASURE ||
			qf->cdma_rate < CDMA_RATE_BLANK) {
			pr_err("invalid qcelp cdma rate\n");
			return -EINVAL;
		}
		break;
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int qcelp_in_open(struct inode *inode, struct file *file)
{
	int rc;

	pr_info("qcelp_in: open\n");
	 mutex_lock(&qcelp_in_lock);
	 if (qcelp_in_opened) {
		pr_err("qcelp_in: busy\n");
		rc = -EBUSY;
	 } else {
		qf = kzalloc(sizeof(*qf), GFP_KERNEL);
		memset(qf, 0, sizeof(struct msm_audio_qcelp_config));
		qf->channels = 1;
		qf->cdma_rate = 0x04; /* CDMA_RATE_FULL */
		qf->min_bit_rate = 1;
		qf->max_bit_rate = 4;
		qcelp_in_opened = 1;
		rc = 0;
	 }
	 mutex_unlock(&qcelp_in_lock);
	 return rc;
}

static ssize_t qcelp_in_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	 struct audio_client *ac;
	 struct audio_buffer *ab;
	 const char __user *start = buf;
	 int xfer, res = 0;

	 mutex_lock(&qcelp_in_lock);
	 ac = file->private_data;
	 if (!ac) {
		res = -ENODEV;
		pr_err("qcelp_in_read ac NULL\n");
		goto fail;
	 }
	 while (count > 0) {
		ab = ac->buf + ac->cpu_buf;
		TRACE("qcelp_in_read wait count=%d ab=%d ac->buf=%d cpu_buf=%d ac->buf[1]=%d\n",
		      count, ab, ac->buf, ac->cpu_buf, &(ac->buf[1]));
		if (ab->used)
			wait_event(ac->wait, (ab->used == 0));
		TRACE(" qcelp_in_read event arrive ab->size=%d\n", ab->size);
		xfer = count;
		if (xfer > ab->size)
			xfer = ab->size;

		if (copy_to_user(buf, ab->data, xfer)) {
			res = -EFAULT;
			pr_err("Tomdbg copy to user failed \n");
			goto fail;
		}
		TRACE("qcelp_in read buf = %d,xfer = %d,cnt = %d\n", buf, xfer, count);

		buf += xfer;
		count -= xfer;

		ab->used = 1;
		q6audio_read(ac, ab);
		ac->cpu_buf ^= 1;
	 }
fail:
	 res = buf - start;
	 mutex_unlock(&qcelp_in_lock);
	 return res;
}

static int qcelp_in_release(struct inode *inode, struct file *file)
{
	 int rc = 0;
	 pr_info("qcelp_in: release\n");
	 mutex_lock(&qcelp_in_lock);
	 if (file->private_data)
		rc = q6audio_close(file->private_data);
	 kfree(qf);
	 qcelp_in_opened = 0;
	 mutex_unlock(&qcelp_in_lock);
	 return rc;
}

static struct file_operations qcelp_in_fops = {
	.owner	  = THIS_MODULE,
	.open	  = qcelp_in_open,
	.read	  = qcelp_in_read,
	.release  = qcelp_in_release,
	.unlocked_ioctl	= qcelp_in_ioctl,
};

struct miscdevice qcelp_in_misc = {
	 .minor = MISC_DYNAMIC_MINOR,
	 .name = "msm_qcelp_in",
	 .fops = &qcelp_in_fops,
};

static int __init qcelp_in_init(void) {
	 return misc_register(&qcelp_in_misc);
}

device_initcall(qcelp_in_init);
