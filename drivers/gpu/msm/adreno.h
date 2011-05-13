/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __ADRENO_H
#define __ADRENO_H

#include "adreno_drawctxt.h"
#include "adreno_ringbuffer.h"

#define DEVICE_3D_NAME "kgsl-3d"
#define DEVICE_3D0_NAME "kgsl-3d0"

#define ADRENO_DEVICE(device) \
		KGSL_CONTAINER_OF(device, struct adreno_device, dev)

/* Flags to control command packet settings */
#define KGSL_CMD_FLAGS_PMODE		0x00000001
#define KGSL_CMD_FLAGS_NO_TS_CMP	0x00000002
#define KGSL_CMD_FLAGS_NOT_KERNEL_CMD	0x00000004

/* Command identifiers */
#define KGSL_CONTEXT_TO_MEM_IDENTIFIER	0xDEADBEEF
#define KGSL_CMD_IDENTIFIER		0xFEEDFACE

struct adreno_device {
	struct kgsl_device dev;    /* Must be first field in this struct */
	unsigned int chip_id;
	struct kgsl_memregion gmemspace;
	struct adreno_context *drawctxt_active;
	wait_queue_head_t ib1_wq;
	unsigned int *pfp_fw;
	size_t pfp_fw_size;
	unsigned int *pm4_fw;
	size_t pm4_fw_size;
	struct adreno_ringbuffer ringbuffer;
	unsigned int mharb;
};

int adreno_idle(struct kgsl_device *device, unsigned int timeout);
void adreno_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value);
void adreno_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value);
void adreno_regread_isr(struct kgsl_device *device,
			     unsigned int offsetwords,
			     unsigned int *value);
void adreno_regwrite_isr(struct kgsl_device *device,
			      unsigned int offsetwords,
			      unsigned int value);

uint8_t *kgsl_sharedmem_convertaddr(struct kgsl_device *device,
	unsigned int pt_base, unsigned int gpuaddr, unsigned int *size);

enum adreno_gpurev {
	ADRENO_REV_UNKNOWN = 0,
	ADRENO_REV_A200 = 200,
	ADRENO_REV_A205 = 205,
	ADRENO_REV_A220 = 220,
	ADRENO_REV_A225 = 225,
};

enum adreno_gpurev adreno_get_rev(struct adreno_device *adreno_dev);

static inline int adreno_is_a200(struct adreno_device *adreno_dev)
{
	return (adreno_get_rev(adreno_dev) == ADRENO_REV_A200);
}

static inline int adreno_is_a205(struct adreno_device *adreno_dev)
{
	return (adreno_get_rev(adreno_dev) == ADRENO_REV_A200);
}

static inline int adreno_is_a20x(struct adreno_device *adreno_dev)
{
	enum adreno_gpurev rev = adreno_get_rev(adreno_dev);
	return (rev  == ADRENO_REV_A200 || rev == ADRENO_REV_A205);
}

static inline int adreno_is_a220(struct adreno_device *adreno_dev)
{
	return (adreno_get_rev(adreno_dev) == ADRENO_REV_A220);
}

static inline int adreno_is_a225(struct adreno_device *adreno_dev)
{
	return (adreno_get_rev(adreno_dev) == ADRENO_REV_A225);
}

static inline int adreno_is_a22x(struct adreno_device *adreno_dev)
{
	enum adreno_gpurev rev = adreno_get_rev(adreno_dev);
	return (rev  == ADRENO_REV_A220 || rev == ADRENO_REV_A225);
}

#endif /*__ADRENO_H */
