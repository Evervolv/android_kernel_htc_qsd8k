/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#ifndef __KGSL_PWRCTRL_H
#define __KGSL_PWRCTRL_H

/*****************************************************************************
** power flags
*****************************************************************************/
#define KGSL_PWRFLAGS_POWER_ON 0
#define KGSL_PWRFLAGS_CLK_ON   1
#define KGSL_PWRFLAGS_AXI_ON   2
#define KGSL_PWRFLAGS_IRQ_ON   3

#define KGSL_PWRFLAGS_ON   1
#define KGSL_PWRFLAGS_OFF  0

#define KGSL_DEFAULT_PWRLEVEL 1
#define KGSL_MAX_CLKS 5

struct platform_device;

struct kgsl_pwrctrl {
	int interrupt_num;
	int have_irq;
	struct clk *ebi1_clk;
	struct clk *grp_clks[KGSL_MAX_CLKS];
	unsigned long power_flags;
	struct kgsl_pwrlevel pwrlevels[KGSL_MAX_PWRLEVELS];
	unsigned int active_pwrlevel;
	int thermal_pwrlevel;
	unsigned int num_pwrlevels;
	unsigned int interval_timeout;
	struct regulator *gpu_reg;
	uint32_t pcl;
	unsigned int nap_allowed;
	struct adreno_context *suspended_ctxt;
	const char *regulator_name;
	const char *irq_name;
	const char *src_clk_name;
	s64 time;
	unsigned int no_switch_cnt;
	unsigned int idle_pass;
};

void kgsl_pwrctrl_clk(struct kgsl_device *device, int state);
void kgsl_pwrctrl_axi(struct kgsl_device *device, int state);
void kgsl_pwrctrl_pwrrail(struct kgsl_device *device, int state);
void kgsl_pwrctrl_irq(struct kgsl_device *device, int state);
int kgsl_pwrctrl_init(struct kgsl_device *device);
void kgsl_pwrctrl_close(struct kgsl_device *device);
void kgsl_timer(unsigned long data);
void kgsl_idle_check(struct work_struct *work);
void kgsl_pre_hwaccess(struct kgsl_device *device);
void kgsl_check_suspended(struct kgsl_device *device);
int kgsl_pwrctrl_sleep(struct kgsl_device *device);
void kgsl_pwrctrl_wake(struct kgsl_device *device);
void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
	unsigned int level);
int kgsl_pwrctrl_init_sysfs(struct kgsl_device *device);
void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device);
void kgsl_pwrctrl_enable(struct kgsl_device *device);
void kgsl_pwrctrl_disable(struct kgsl_device *device);
static inline unsigned long kgsl_get_clkrate(struct clk *clk)
{
	return (clk != NULL) ? clk_get_rate(clk) : 0;
}

#endif /* __KGSL_PWRCTRL_H */
