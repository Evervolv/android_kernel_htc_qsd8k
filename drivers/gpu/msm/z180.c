/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/uaccess.h>

#include "kgsl.h"
#include "kgsl_cffdump.h"

#include "z180.h"
#include "z180_reg.h"

#define DRIVER_VERSION_MAJOR   3
#define DRIVER_VERSION_MINOR   1

#define Z180_DEVICE(device) \
		KGSL_CONTAINER_OF(device, struct z180_device, dev)

#define GSL_VGC_INT_MASK \
	 (REG_VGC_IRQSTATUS__MH_MASK | \
	  REG_VGC_IRQSTATUS__G2D_MASK | \
	  REG_VGC_IRQSTATUS__FIFO_MASK)

#define VGV3_NEXTCMD_JUMP        0x01

#define VGV3_NEXTCMD_NEXTCMD_FSHIFT 12
#define VGV3_NEXTCMD_NEXTCMD_FMASK 0x7

#define VGV3_CONTROL_MARKADD_FSHIFT 0
#define VGV3_CONTROL_MARKADD_FMASK 0xfff

#define Z180_PACKET_SIZE 15
#define Z180_MARKER_SIZE 10
#define Z180_CALL_CMD     0x1000
#define Z180_MARKER_CMD   0x8000
#define Z180_STREAM_END_CMD 0x9000
#define Z180_STREAM_PACKET 0x7C000176
#define Z180_STREAM_PACKET_CALL 0x7C000275
#define Z180_PACKET_COUNT 8
#define Z180_RB_SIZE (Z180_PACKET_SIZE*Z180_PACKET_COUNT \
			  *sizeof(uint32_t))

#define NUMTEXUNITS             4
#define TEXUNITREGCOUNT         25
#define VG_REGCOUNT             0x39

#define PACKETSIZE_BEGIN        3
#define PACKETSIZE_G2DCOLOR     2
#define PACKETSIZE_TEXUNIT      (TEXUNITREGCOUNT * 2)
#define PACKETSIZE_REG          (VG_REGCOUNT * 2)
#define PACKETSIZE_STATE        (PACKETSIZE_TEXUNIT * NUMTEXUNITS + \
				 PACKETSIZE_REG + PACKETSIZE_BEGIN + \
				 PACKETSIZE_G2DCOLOR)
#define PACKETSIZE_STATESTREAM  (ALIGN((PACKETSIZE_STATE * \
				 sizeof(unsigned int)), 32) / \
				 sizeof(unsigned int))

#define Z180_INVALID_CONTEXT UINT_MAX

/* z180 MH arbiter config*/
#define Z180_CFG_MHARB \
	(0x10 \
		| (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
		| (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

#define Z180_TIMESTAMP_EPSILON 20000
#define Z180_IDLE_COUNT_MAX 1000000

#define Z180_CMDWINDOW_TARGET_MASK		0x000000FF
#define Z180_CMDWINDOW_ADDR_MASK		0x00FFFF00
#define Z180_CMDWINDOW_TARGET_SHIFT		0
#define Z180_CMDWINDOW_ADDR_SHIFT		8

static int z180_start(struct kgsl_device *device, unsigned int init_ram);
static int z180_stop(struct kgsl_device *device);
static int z180_wait(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs);
static void z180_regread(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value);
static void z180_regwrite(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int value);
static int z180_cmdwindow_write(struct kgsl_device *device,
				enum kgsl_cmdwindow_type target,
				unsigned int addr,
				unsigned int data);
static void z180_regread_isr(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value);
static void z180_regwrite_isr(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int value);
static void __devinit z180_getfunctable(struct kgsl_functable *ftbl);

#define Z180_MMU_CONFIG					     \
	(0x01							     \
	| (MMU_CONFIG << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT) \
	| (MMU_CONFIG << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT) \
	| (MMU_CONFIG << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT))

static struct z180_device device_2d0 = {
	.dev = {
		.name = DEVICE_2D0_NAME,
		.id = KGSL_DEVICE_2D0,
		.ver_major = DRIVER_VERSION_MAJOR,
		.ver_minor = DRIVER_VERSION_MINOR,
		.mmu = {
			.config = Z180_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			.reg = {
				.config = ADDR_MH_MMU_CONFIG,
				.mpu_base = ADDR_MH_MMU_MPU_BASE,
				.mpu_end = ADDR_MH_MMU_MPU_END,
				.va_range = ADDR_MH_MMU_VA_RANGE,
				.pt_page = ADDR_MH_MMU_PT_BASE,
				.page_fault = ADDR_MH_MMU_PAGE_FAULT,
				.tran_error = ADDR_MH_MMU_TRAN_ERROR,
				.invalidate = ADDR_MH_MMU_INVALIDATE,
				.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
				.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
				.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR,
				.axi_error = ADDR_MH_AXI_ERROR,
			},
		},
		.pwrctrl = {
			.regulator_name = "fs_gfx2d0",
			.irq_name = KGSL_2D0_IRQ,
		},
		.mutex = __MUTEX_INITIALIZER(device_2d0.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
		.iomemname = KGSL_2D0_REG_MEMORY,
	},
};

static struct z180_device device_2d1 = {
	.dev = {
		.name = DEVICE_2D1_NAME,
		.id = KGSL_DEVICE_2D1,
		.ver_major = DRIVER_VERSION_MAJOR,
		.ver_minor = DRIVER_VERSION_MINOR,
		.mmu = {
			.config = Z180_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			.reg = {
				.config = ADDR_MH_MMU_CONFIG,
				.mpu_base = ADDR_MH_MMU_MPU_BASE,
				.mpu_end = ADDR_MH_MMU_MPU_END,
				.va_range = ADDR_MH_MMU_VA_RANGE,
				.pt_page = ADDR_MH_MMU_PT_BASE,
				.page_fault = ADDR_MH_MMU_PAGE_FAULT,
				.tran_error = ADDR_MH_MMU_TRAN_ERROR,
				.invalidate = ADDR_MH_MMU_INVALIDATE,
				.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
				.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
				.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR,
				.axi_error = ADDR_MH_AXI_ERROR,
			},
		},
		.pwrctrl = {
			.regulator_name = "fs_gfx2d1",
			.irq_name = KGSL_2D1_IRQ,
		},
		.mutex = __MUTEX_INITIALIZER(device_2d1.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
		.iomemname = KGSL_2D1_REG_MEMORY,
	},
};

static irqreturn_t z180_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;
	unsigned int status;
	struct kgsl_device *device = (struct kgsl_device *) data;
	struct z180_device *z180_dev = Z180_DEVICE(device);

	z180_regread_isr(device, ADDR_VGC_IRQSTATUS >> 2, &status);

	if (status & GSL_VGC_INT_MASK) {
		z180_regwrite_isr(device,
			ADDR_VGC_IRQSTATUS >> 2, status & GSL_VGC_INT_MASK);

		result = IRQ_HANDLED;

		if (status & REG_VGC_IRQSTATUS__FIFO_MASK)
			KGSL_DRV_ERR(device, "z180 fifo interrupt\n");
		if (status & REG_VGC_IRQSTATUS__MH_MASK)
			kgsl_mh_intrcallback(device);
		if (status & REG_VGC_IRQSTATUS__G2D_MASK) {
			int count;

			z180_regread_isr(device,
					 ADDR_VGC_IRQ_ACTIVE_CNT >> 2,
					 &count);

			count >>= 8;
			count &= 255;
			z180_dev->timestamp += count;

			wake_up_interruptible(&device->wait_queue);

			atomic_notifier_call_chain(
				&(device->ts_notifier_list),
				device->id, NULL);
		}
	}

	if ((device->pwrctrl.nap_allowed == true) &&
		(device->requested_state == KGSL_STATE_NONE)) {
		device->requested_state = KGSL_STATE_NAP;
		queue_work(device->work_queue, &device->idle_check_ws);
	}
	mod_timer(&device->idle_timer,
			jiffies + device->pwrctrl.interval_timeout);

	return result;
}

static int z180_cleanup_pt(struct kgsl_device *device,
			       struct kgsl_pagetable *pagetable)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);

	kgsl_mmu_unmap(pagetable, &device->mmu.dummyspace);

	kgsl_mmu_unmap(pagetable, &device->memstore);

	kgsl_mmu_unmap(pagetable, &z180_dev->ringbuffer.cmdbufdesc);

	return 0;
}

static int z180_setup_pt(struct kgsl_device *device,
			     struct kgsl_pagetable *pagetable)
{
	int result = 0;
	struct z180_device *z180_dev = Z180_DEVICE(device);

	result = kgsl_mmu_map_global(pagetable, &device->mmu.dummyspace,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);

	if (result)
		goto error;

	result = kgsl_mmu_map_global(pagetable, &device->memstore,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);
	if (result)
		goto error_unmap_dummy;

	result = kgsl_mmu_map_global(pagetable,
				     &z180_dev->ringbuffer.cmdbufdesc,
				     GSL_PT_PAGE_RV);
	if (result)
		goto error_unmap_memstore;
	return result;

error_unmap_dummy:
	kgsl_mmu_unmap(pagetable, &device->mmu.dummyspace);

error_unmap_memstore:
	kgsl_mmu_unmap(pagetable, &device->memstore);

error:
	return result;
}

static inline unsigned int rb_offset(unsigned int index)
{
	return index*sizeof(unsigned int)*(Z180_PACKET_SIZE);
}

static void addmarker(struct z180_ringbuffer *rb, unsigned int index)
{
	char *ptr = (char *)(rb->cmdbufdesc.hostptr);
	unsigned int *p = (unsigned int *)(ptr + rb_offset(index));

	*p++ = Z180_STREAM_PACKET;
	*p++ = (Z180_MARKER_CMD | 5);
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = Z180_STREAM_PACKET;
	*p++ = 5;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
}

static void addcmd(struct z180_ringbuffer *rb, unsigned int index,
			unsigned int cmd, unsigned int nextcnt)
{
	char * ptr = (char *)(rb->cmdbufdesc.hostptr);
	unsigned int *p = (unsigned int *)(ptr + (rb_offset(index)
			   + (Z180_MARKER_SIZE * sizeof(unsigned int))));

	*p++ = Z180_STREAM_PACKET_CALL;
	*p++ = cmd;
	*p++ = Z180_CALL_CMD | nextcnt;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
}

static int z180_cmdstream_start(struct kgsl_device *device)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);
	int result;
	unsigned int cmd = VGV3_NEXTCMD_JUMP << VGV3_NEXTCMD_NEXTCMD_FSHIFT;

	z180_dev->timestamp = 0;
	z180_dev->current_timestamp = 0;

	addmarker(&z180_dev->ringbuffer, 0);

	result = z180_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_MODE, 4);
	if (result != 0)
		return result;

	result = z180_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_NEXTADDR,
			z180_dev->ringbuffer.cmdbufdesc.gpuaddr);
	if (result != 0)
		return result;

	result = z180_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_NEXTCMD, cmd | 5);
	if (result != 0)
		return result;

	result = z180_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_WRITEADDR, device->memstore.gpuaddr);

	if (result != 0)
		return result;

	cmd = (int)(((1) & VGV3_CONTROL_MARKADD_FMASK)
			<< VGV3_CONTROL_MARKADD_FSHIFT);

	result = z180_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_CONTROL, cmd);

	if (result != 0)
		return result;

	result = z180_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_CONTROL, 0);
	if (result != 0)
		return result;

	return result;
}

static int room_in_rb(struct z180_device *device)
{
	int ts_diff;

	ts_diff = device->current_timestamp - device->timestamp;

	return ts_diff < Z180_PACKET_COUNT;
}

static int z180_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = 0;
	struct z180_device *z180_dev = Z180_DEVICE(device);

	if (z180_dev->current_timestamp > z180_dev->timestamp)
		status = z180_wait(device, z180_dev->current_timestamp,
					timeout);

	if (status)
		KGSL_DRV_ERR(device, "z180_waittimestamp() timed out\n");

	return status;
}

static int z180_setstate(struct kgsl_device *device, uint32_t flags)
{
#ifdef CONFIG_MSM_KGSL_MMU
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		z180_idle(device, KGSL_TIMEOUT_DEFAULT);
		z180_regwrite(device, ADDR_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		z180_regwrite(device, ADDR_MH_MMU_VA_RANGE,
				     (device->mmu.hwpagetable->
				      va_base | (device->mmu.hwpagetable->
						 va_range >> 16)));
		z180_regwrite(device, ADDR_MH_MMU_INVALIDATE,
				     mh_mmu_invalidate);
	}

	if (flags & KGSL_MMUFLAGS_TLBFLUSH)
		z180_regwrite(device, ADDR_MH_MMU_INVALIDATE,
			     mh_mmu_invalidate);
#endif
	return 0;
}

int
z180_cmdstream_issueibcmds(struct kgsl_device_private *dev_priv,
			struct kgsl_context *context,
			struct kgsl_ibdesc *ibdesc,
			unsigned int numibs,
			uint32_t *timestamp,
			unsigned int ctrl)
{
	unsigned int result = 0;
	unsigned int ofs        = PACKETSIZE_STATESTREAM * sizeof(unsigned int);
	unsigned int cnt        = 5;
	unsigned int nextaddr   = 0;
	unsigned int index	= 0;
	unsigned int nextindex;
	unsigned int nextcnt    = Z180_STREAM_END_CMD | 5;
	struct kgsl_memdesc tmp = {0};
	unsigned int cmd;
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_pagetable *pagetable = dev_priv->process_priv->pagetable;
	struct z180_device *z180_dev = Z180_DEVICE(device);
	unsigned int sizedwords;

	if (device->state & KGSL_STATE_HUNG) {
		return -EINVAL;
		goto error;
	}
	if (numibs != 1) {
		KGSL_DRV_ERR(device, "Invalid number of ibs: %d\n", numibs);
		result = -EINVAL;
		goto error;
	}
	cmd = ibdesc[0].gpuaddr;
	sizedwords = ibdesc[0].sizedwords;

	tmp.hostptr = (void *)*timestamp;

	KGSL_CMD_INFO(device, "ctxt %d ibaddr 0x%08x sizedwords %d\n",
		context->id, cmd, sizedwords);
	/* context switch */
	if ((context->id != (int)z180_dev->ringbuffer.prevctx) ||
	    (ctrl & KGSL_CONTEXT_CTX_SWITCH)) {
		KGSL_CMD_INFO(device, "context switch %d -> %d\n",
			context->id, z180_dev->ringbuffer.prevctx);
		kgsl_mmu_setstate(device, pagetable);
		cnt = PACKETSIZE_STATESTREAM;
		ofs = 0;
	}
	z180_setstate(device, kgsl_pt_get_flags(device->mmu.hwpagetable,
						    device->id));

	result = wait_event_interruptible_timeout(device->wait_queue,
				  room_in_rb(z180_dev),
				  msecs_to_jiffies(KGSL_TIMEOUT_DEFAULT));
	if (result < 0) {
		KGSL_CMD_ERR(device, "wait_event_interruptible_timeout "
			"failed: %d\n", result);
		goto error;
	}
	result = 0;

	index = z180_dev->current_timestamp % Z180_PACKET_COUNT;
	z180_dev->current_timestamp++;
	nextindex = z180_dev->current_timestamp % Z180_PACKET_COUNT;
	*timestamp = z180_dev->current_timestamp;

	z180_dev->ringbuffer.prevctx = context->id;

	addcmd(&z180_dev->ringbuffer, index, cmd + ofs, cnt);

	/* Make sure the next ringbuffer entry has a marker */
	addmarker(&z180_dev->ringbuffer, nextindex);

	nextaddr = z180_dev->ringbuffer.cmdbufdesc.gpuaddr
		+ rb_offset(nextindex);

	tmp.hostptr = (void *)(tmp.hostptr +
			(sizedwords * sizeof(unsigned int)));
	tmp.size = 12;

	kgsl_sharedmem_writel(&tmp, 4, nextaddr);
	kgsl_sharedmem_writel(&tmp, 8, nextcnt);

	/* sync memory before activating the hardware for the new command*/
	mb();

	cmd = (int)(((2) & VGV3_CONTROL_MARKADD_FMASK)
		<< VGV3_CONTROL_MARKADD_FSHIFT);

	z180_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, cmd);
	z180_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, 0);
error:
	return result;
}

static int z180_ringbuffer_init(struct kgsl_device *device)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);
	memset(&z180_dev->ringbuffer, 0, sizeof(struct z180_ringbuffer));
	z180_dev->ringbuffer.prevctx = Z180_INVALID_CONTEXT;
	return kgsl_sharedmem_alloc_coherent(
			&z180_dev->ringbuffer.cmdbufdesc,
			Z180_RB_SIZE);
}

static void z180_ringbuffer_close(struct kgsl_device *device)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);
	kgsl_sharedmem_free(&z180_dev->ringbuffer.cmdbufdesc);
	memset(&z180_dev->ringbuffer, 0, sizeof(struct z180_ringbuffer));
}

static int __devinit z180_probe(struct platform_device *pdev)
{
	int status = -EINVAL;
	struct kgsl_device *device = NULL;
	struct z180_device *z180_dev;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;
	device->parentdev = &pdev->dev;

	z180_getfunctable(&device->ftbl);

	z180_dev = Z180_DEVICE(device);
	spin_lock_init(&z180_dev->cmdwin_lock);

	status = z180_ringbuffer_init(device);
	if (status != 0)
		goto error;

	status = kgsl_device_platform_probe(device, z180_isr);
	if (status)
		goto error_close_ringbuffer;

	return status;

error_close_ringbuffer:
	z180_ringbuffer_close(device);
error:
	device->parentdev = NULL;
	return status;
}

static int __devexit z180_remove(struct platform_device *pdev)
{
	struct kgsl_device *device = NULL;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;

	kgsl_device_platform_remove(device);

	z180_ringbuffer_close(device);

	return 0;
}

static int z180_start(struct kgsl_device *device, unsigned int init_ram)
{
	int status = 0;

	device->state = KGSL_STATE_INIT;
	device->requested_state = KGSL_STATE_NONE;
	KGSL_PWR_WARN(device, "state -> INIT, device %d\n", device->id);

	kgsl_pwrctrl_enable(device);

	/* Set up MH arbiter.  MH offsets are considered to be dword
	 * based, therefore no down shift. */
	z180_regwrite(device, ADDR_MH_ARBITER_CONFIG, Z180_CFG_MHARB);

	z180_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030F27);
	z180_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG2, 0x004B274F);

	z180_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0x3);

	status = kgsl_mmu_start(device);
	if (status)
		goto error_clk_off;

	status = z180_cmdstream_start(device);
	if (status)
		goto error_mmu_stop;

	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);
	return 0;
error_clk_off:
	z180_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0);
	kgsl_pwrctrl_disable(device);
error_mmu_stop:
	kgsl_mmu_stop(device);
	return status;
}

static int z180_stop(struct kgsl_device *device)
{
	z180_idle(device, KGSL_TIMEOUT_DEFAULT);

	del_timer(&device->idle_timer);

	kgsl_mmu_stop(device);

	/* Disable the clocks before the power rail. */
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);

	kgsl_pwrctrl_disable(device);

	return 0;
}

static int z180_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
	{
		struct kgsl_devinfo devinfo;

		if (sizebytes != sizeof(devinfo)) {
			status = -EINVAL;
			break;
		}

		memset(&devinfo, 0, sizeof(devinfo));
		devinfo.device_id = device->id+1;
		devinfo.chip_id = 0;
		devinfo.mmu_enabled = kgsl_mmu_enabled();

		if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
				0) {
			status = -EFAULT;
			break;
		}
		status = 0;
	}
	break;
	case KGSL_PROP_MMU_ENABLE:
		{
#ifdef CONFIG_MSM_KGSL_MMU
			int mmuProp = 1;
#else
			int mmuProp = 0;
#endif
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &mmuProp, sizeof(mmuProp))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;

	default:
		KGSL_DRV_ERR(device, "invalid property: %d\n", type);
		status = -EINVAL;
	}
	return status;
}

static unsigned int z180_isidle(struct kgsl_device *device)
{
	int status = false;
	struct z180_device *z180_dev = Z180_DEVICE(device);

	int timestamp = z180_dev->timestamp;

	if (timestamp == z180_dev->current_timestamp)
		status = true;

	return status;
}

static int z180_resume_context(struct kgsl_device *device)
{
	/* Context is in the pre-amble, automatically restored. */

	return 0;
}

static int z180_suspend_context(struct kgsl_device *device)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);

	z180_dev->ringbuffer.prevctx = Z180_INVALID_CONTEXT;

	return 0;
}

/* Not all Z180 registers are directly accessible.
 * The _z180_(read|write)_simple functions below handle the ones that are.
 */
static void _z180_regread_simple(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;

	BUG_ON(offsetwords * sizeof(uint32_t) >= device->regspace.sizebytes);

	reg = (unsigned int *)(device->regspace.mmio_virt_base
			+ (offsetwords << 2));

	/*ensure this read finishes before the next one.
	 * i.e. act like normal readl() */
	*value = __raw_readl(reg);
	rmb();

}

static void _z180_regwrite_simple(struct kgsl_device *device,
				 unsigned int offsetwords,
				 unsigned int value)
{
	unsigned int *reg;

	BUG_ON(offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes);

	reg = (unsigned int *)(device->regspace.mmio_virt_base
			+ (offsetwords << 2));
	kgsl_cffdump_regwrite(device->id, offsetwords << 2, value);
	/*ensure previous writes post before this one,
	 * i.e. act like normal writel() */
	wmb();
	__raw_writel(value, reg);
}


/* The MH registers must be accessed through via a 2 step write, (read|write)
 * process. These registers may be accessed from interrupt context during
 * the handling of MH or MMU error interrupts. Therefore a spin lock is used
 * to ensure that the 2 step sequence is not interrupted.
 */
static void _z180_regread_mmu(struct kgsl_device *device,
			     unsigned int offsetwords,
			     unsigned int *value)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);
	unsigned long flags;

	spin_lock_irqsave(&z180_dev->cmdwin_lock, flags);
	_z180_regwrite_simple(device, (ADDR_VGC_MH_READ_ADDR >> 2),
				offsetwords);
	_z180_regread_simple(device, (ADDR_VGC_MH_DATA_ADDR >> 2), value);
	spin_unlock_irqrestore(&z180_dev->cmdwin_lock, flags);
}


static void _z180_regwrite_mmu(struct kgsl_device *device,
			      unsigned int offsetwords,
			      unsigned int value)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);
	unsigned int cmdwinaddr;
	unsigned long flags;

	cmdwinaddr = ((KGSL_CMDWINDOW_MMU << Z180_CMDWINDOW_TARGET_SHIFT) &
			Z180_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((offsetwords << Z180_CMDWINDOW_ADDR_SHIFT) &
			Z180_CMDWINDOW_ADDR_MASK);

	spin_lock_irqsave(&z180_dev->cmdwin_lock, flags);
	_z180_regwrite_simple(device, ADDR_VGC_MMUCOMMANDSTREAM >> 2,
			     cmdwinaddr);
	_z180_regwrite_simple(device, ADDR_VGC_MMUCOMMANDSTREAM >> 2, value);
	spin_unlock_irqrestore(&z180_dev->cmdwin_lock, flags);
}

/* the rest of the code doesn't want to think about if it is writing mmu
 * registers or normal registers so handle it here
 */
static void _z180_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_AXI_HALT_CONTROL) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		_z180_regread_mmu(device, offsetwords, value);
	} else {
		_z180_regread_simple(device, offsetwords, value);
	}
}

static void _z180_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_CLNT_INTF_CTRL_CONFIG2) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		_z180_regwrite_mmu(device, offsetwords, value);

	} else {
		_z180_regwrite_simple(device, offsetwords, value);
	}
}


static void z180_regread(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	kgsl_pre_hwaccess(device);
	_z180_regread(device, offsetwords, value);
}

static void z180_regread_isr(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	_z180_regread(device, offsetwords, value);
}

static void z180_regwrite(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int value)
{
	kgsl_pre_hwaccess(device);
	_z180_regwrite(device, offsetwords, value);
}

static void z180_regwrite_isr(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int value)
{
	_z180_regwrite(device, offsetwords, value);
}

static int z180_cmdwindow_write(struct kgsl_device *device,
		enum kgsl_cmdwindow_type target, unsigned int addr,
		unsigned int data)
{
	unsigned int cmdwinaddr;
	unsigned int cmdstream;

	if (target < KGSL_CMDWINDOW_MIN ||
		target > KGSL_CMDWINDOW_MAX) {
		KGSL_DRV_ERR(device, "invalid target\n");
		return -EINVAL;
	}

	if (target == KGSL_CMDWINDOW_MMU)
		cmdstream = ADDR_VGC_MMUCOMMANDSTREAM;
	else
		cmdstream = ADDR_VGC_COMMANDSTREAM;

	cmdwinaddr = ((target << Z180_CMDWINDOW_TARGET_SHIFT) &
			Z180_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((addr << Z180_CMDWINDOW_ADDR_SHIFT) &
			Z180_CMDWINDOW_ADDR_MASK);

	z180_regwrite(device, cmdstream >> 2, cmdwinaddr);
	z180_regwrite(device, cmdstream >> 2, data);

	return 0;
}

static unsigned int z180_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);
	/* get current EOP timestamp */
	return z180_dev->timestamp;
}

static int z180_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	mutex_unlock(&device->mutex);
	status = z180_wait(device, timestamp, msecs);
	mutex_lock(&device->mutex);

	return status;
}

static int z180_wait(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	long timeout = 0;

	timeout = wait_io_event_interruptible_timeout(
			device->wait_queue,
			kgsl_check_timestamp(device, timestamp),
			msecs_to_jiffies(msecs));

	if (timeout > 0)
		status = 0;
	else if (timeout == 0) {
		status = -ETIMEDOUT;
		device->state = KGSL_STATE_HUNG;
		KGSL_PWR_WARN(device, "state -> HUNG, device %d\n", device->id);
	} else
		status = timeout;

	return status;
}

static long
z180_ioctl_cmdwindow_write(struct kgsl_device_private *dev_priv,
					   void *data)
{
	struct kgsl_cmdwindow_write *param = data;

	return z180_cmdwindow_write(dev_priv->device,
					param->target,
					param->addr,
					param->data);
}

static int
z180_drawctxt_destroy(struct kgsl_device *device,
			  struct kgsl_context *context)
{
	struct z180_device *z180_dev = Z180_DEVICE(device);

	z180_idle(device, KGSL_TIMEOUT_DEFAULT);

	if (z180_dev->ringbuffer.prevctx == context->id) {
		z180_dev->ringbuffer.prevctx = Z180_INVALID_CONTEXT;
		device->mmu.hwpagetable = device->mmu.defaultpagetable;
		kgsl_setstate(device, KGSL_MMUFLAGS_PTUPDATE);
	}

	return 0;
}

static long z180_ioctl(struct kgsl_device_private *dev_priv,
			   unsigned int cmd, void *data)
{
	int result = 0;

	switch (cmd) {
	case IOCTL_KGSL_CMDWINDOW_WRITE:
		result = z180_ioctl_cmdwindow_write(dev_priv, data);
		break;
	default:
		KGSL_DRV_INFO(dev_priv->device,
			"invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	return result;

}

static void z180_power_stats(struct kgsl_device *device,
			    struct kgsl_power_stats *stats)
{
	stats->total_time = 0;
	stats->busy_time = 0;
}

static void __devinit z180_getfunctable(struct kgsl_functable *ftbl)
{
	if (ftbl == NULL)
		return;
	ftbl->device_regread = z180_regread;
	ftbl->device_regwrite = z180_regwrite;
	ftbl->device_regread_isr = z180_regread_isr;
	ftbl->device_regwrite_isr = z180_regwrite_isr;
	ftbl->device_setstate = z180_setstate;
	ftbl->device_idle = z180_idle;
	ftbl->device_isidle = z180_isidle;
	ftbl->device_suspend_context = z180_suspend_context;
	ftbl->device_resume_context = z180_resume_context;
	ftbl->device_start = z180_start;
	ftbl->device_stop = z180_stop;
	ftbl->device_getproperty = z180_getproperty;
	ftbl->device_waittimestamp = z180_waittimestamp;
	ftbl->device_readtimestamp = z180_readtimestamp;
	ftbl->device_issueibcmds = z180_cmdstream_issueibcmds;
	ftbl->device_drawctxt_create = NULL;
	ftbl->device_drawctxt_destroy = z180_drawctxt_destroy;
	ftbl->device_ioctl = z180_ioctl;
	ftbl->device_setup_pt = z180_setup_pt;
	ftbl->device_cleanup_pt = z180_cleanup_pt;
	ftbl->device_power_stats = z180_power_stats,
}

static struct platform_device_id z180_id_table[] = {
	{ DEVICE_2D0_NAME, (kernel_ulong_t)&device_2d0.dev, },
	{ DEVICE_2D1_NAME, (kernel_ulong_t)&device_2d1.dev, },
	{ },
};
MODULE_DEVICE_TABLE(platform, z180_id_table);

static struct platform_driver z180_platform_driver = {
	.probe = z180_probe,
	.remove = __devexit_p(z180_remove),
	.suspend = kgsl_suspend_driver,
	.resume = kgsl_resume_driver,
	.id_table = z180_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_2D_NAME,
		.pm = &kgsl_pm_ops,
	}
};

static int __init kgsl_2d_init(void)
{
	return platform_driver_register(&z180_platform_driver);
}

static void __exit kgsl_2d_exit(void)
{
	platform_driver_unregister(&z180_platform_driver);
}

module_init(kgsl_2d_init);
module_exit(kgsl_2d_exit);

MODULE_DESCRIPTION("2D Graphics driver");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kgsl_2d");
