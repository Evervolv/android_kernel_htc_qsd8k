/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef __MSM_AUDIO_QCP_H
#define __MSM_AUDIO_QCP_H

#include <linux/msm_audio.h>

#define CDMA_RATE_BLANK		0x00
#define CDMA_RATE_EIGHTH	0x01
#define CDMA_RATE_QUARTER	0x02
#define CDMA_RATE_HALF		0x03
#define CDMA_RATE_FULL		0x04
#define CDMA_RATE_ERASURE	0x05

struct msm_audio_qcelp_config {
	uint32_t channels;
	uint32_t cdma_rate;
	uint32_t min_bit_rate;
	uint32_t max_bit_rate;
};

struct msm_audio_evrc_config {
	uint32_t channels;
	uint32_t cdma_rate;
	uint32_t min_bit_rate;
	uint32_t max_bit_rate;
	uint8_t bit_rate_reduction;
	uint8_t hi_pass_filter;
	uint8_t	noise_suppressor;
	uint8_t	post_filter;
};

#endif /* __MSM_AUDIO_QCP_H */
