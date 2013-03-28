/* arch/arm/mach-msm/board-htcleo-acoustic.c
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
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#include <mach/msm_smd.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <mach/htc_acoustic_qsd.h>
#include <mach/msm_qdsp6_audio_1550.h>

#include "smd_private.h"
#include "dex_comm.h"

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_UPDATE_ADIE \
	_IOW(ACOUSTIC_IOCTL_MAGIC, 24, unsigned int)


#define HTC_ACOUSTIC_TABLE_SIZE        (0x20000)

#define D(fmt, args...) printk(KERN_INFO "htc-acoustic: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "htc-acoustic: "fmt, ##args)

static uint32_t htc_acoustic_vir_addr;
static struct mutex api_lock;
static struct qsd_acoustic_ops *the_ops;

void acoustic_register_ops(struct qsd_acoustic_ops *ops)
{
	the_ops = ops;
}

int turn_mic_bias_on(int on)
{
	D("%s called %d\n", __func__, on);
	if (the_ops->enable_mic_bias)
		the_ops->enable_mic_bias(on);
	return 0;
}
EXPORT_SYMBOL(turn_mic_bias_on);

int force_headset_speaker_on(int enable)
{
	printk("force_headset_speaker_on((%d)\n", enable);

	if (enable)
		dex_audio(0x53);
	else
		dex_audio(0x54);

	return 0;
}
EXPORT_SYMBOL(force_headset_speaker_on);

int enable_aux_loopback(uint32_t enable)
{
	printk("enable_aux_loopback(%d)\n", enable);

	if (enable)
		dex_audio(0x51);
	else
		dex_audio(0x52);

	return 0;
}
EXPORT_SYMBOL(enable_aux_loopback);

int set_aux_gain(int level)
{
   /* struct aux_gain_req {
        struct rpc_request_hdr hdr;
        int level;
    } aux_req;

    D("%s called %d\n", __func__, level);

    if (is_rpc_connect() == -1)
        return -1;

    aux_req.level = cpu_to_be32(level);
    return  msm_rpc_call(endpoint,
        ONCRPC_SET_AUX_PGA_GAIN_PROC,
        &aux_req, sizeof(aux_req), 5 * HZ);*/
    return 0;
}
EXPORT_SYMBOL(set_aux_gain);


static int acoustic_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pgoff;
	int rc = -EINVAL;
	size_t size;

	D("mmap\n");

	mutex_lock(&api_lock);

	size = vma->vm_end - vma->vm_start;

	if (vma->vm_pgoff != 0) {
		E("mmap failed: page offset %lx\n", vma->vm_pgoff);
		goto done;
	}

	if (!htc_acoustic_vir_addr) {
		E("mmap failed: smem region not allocated\n");
		rc = -EIO;
		goto done;
	}

	pgoff = MSM_SHARED_RAM_PHYS +
		(htc_acoustic_vir_addr - (uint32_t)MSM_SHARED_RAM_BASE);
	pgoff = ((pgoff + 4095) & ~4095);
	htc_acoustic_vir_addr = ((htc_acoustic_vir_addr + 4095) & ~4095);

	if (pgoff <= 0) {
		E("pgoff wrong. %ld\n", pgoff);
		goto done;
	}

	if (size <= HTC_ACOUSTIC_TABLE_SIZE) {
		pgoff = pgoff >> PAGE_SHIFT;
	} else {
		E("size > HTC_ACOUSTIC_TABLE_SIZE  %d\n", size);
		goto done;
	}

	vma->vm_flags |= VM_IO | VM_RESERVED;
	rc = io_remap_pfn_range(vma, vma->vm_start, pgoff,
				size, vma->vm_page_prot);

	if (rc < 0)
		E("mmap failed: remap error %d\n", rc);

done:	mutex_unlock(&api_lock);
	return rc;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	int rc = -EIO;

	D("open\n");

	mutex_lock(&api_lock);

	if (!htc_acoustic_vir_addr)
	{
		htc_acoustic_vir_addr = (uint32_t)MSM_SHARED_RAM_BASE + 0xF8000;
	}

	rc = 0;
	mutex_unlock(&api_lock);
	return rc;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long acoustic_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int rc = 0;

	D("ioctl\n");

	mutex_lock(&api_lock);

	switch (cmd) {
	case ACOUSTIC_UPDATE_ADIE: 
	{
		unsigned data = 0xE5;

		D("ioctl: ACOUSTIC_UPDATE_ADIE called %d.\n", current->pid);

// CotullaTODO: finish this code. if we need android tables really...

		D("ioctl: done.\n");
		break;
	}
	default:
		E("ioctl: invalid command\n");
		rc = -EINVAL;
	}

	mutex_unlock(&api_lock);
	return rc;
}

struct rpc_set_uplink_mute_args {
	int mute;
};

static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.mmap = acoustic_mmap,
	.unlocked_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
	mutex_init(&api_lock);
	return misc_register(&acoustic_misc);
}

static void __exit acoustic_exit(void)
{
	misc_deregister(&acoustic_misc);
}

module_init(acoustic_init);
module_exit(acoustic_exit);

