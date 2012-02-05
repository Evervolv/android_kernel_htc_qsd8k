/* Copyright (c) 2010-2011 Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MSM_FOOTSWITCH__
#define __MSM_FOOTSWITCH__

#include <linux/regulator/machine.h>

/* Device IDs */
#define FS_GFX2D0	0
#define FS_GFX2D1	1
#define FS_GFX3D	2
#define FS_IJPEG	3
#define FS_MDP		4
#define FS_MFC		5
#define FS_ROT		6
#define FS_VED		7
#define FS_VFE		8
#define FS_VPE		9
#define MAX_FS		10

#define FS_GENERIC(_drv_name, _id, _name) (&(struct platform_device){ \
	.name	= (_drv_name), \
	.id	= (_id), \
	.dev	= { \
		.platform_data = &(struct regulator_init_data){ \
			.constraints = { \
				.valid_modes_mask = REGULATOR_MODE_NORMAL, \
				.valid_ops_mask   = REGULATOR_CHANGE_STATUS, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = \
				&(struct regulator_consumer_supply) \
				REGULATOR_SUPPLY((_name), NULL), \
		} \
	}, \
})
#define FS_PCOM(_id, _name) FS_GENERIC("footswitch-pcom", (_id), (_name))
#define FS_8X60(_id, _name) FS_GENERIC("footswitch-8x60", (_id), (_name))

#endif
