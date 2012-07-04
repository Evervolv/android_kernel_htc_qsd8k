/* arch/arm/mach-msm/cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, Code Aurora Forum. All rights reserved.
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include "acpuclock.h"

static int override_cpu;

static int set_cpu_freq(struct cpufreq_policy *policy, unsigned int new_freq)
{
	int ret = 0;
	struct cpufreq_freqs freqs;

	freqs.old = policy->cur;
	if (override_cpu) {
		if (policy->cur == policy->max)
			return 0;
		else
			freqs.new = policy->max;
	} else
		freqs.new = new_freq;
	freqs.cpu = policy->cpu;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	ret = acpuclk_set_rate(new_freq * 1000, 0);
	if (!ret)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int index;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

	if (policy->cur == table[index].frequency)
		return 0;

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
		policy->cpu, target_freq, relation,
		policy->min, policy->max, table[index].frequency);
#endif

	return set_cpu_freq(policy, table[index].frequency);
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static int msm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(policy->cpu);

	BUG_ON(cpufreq_frequency_table_cpuinfo(policy, table));
	policy->cur = acpuclk_get_rate();
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	policy->min = CONFIG_MSM_CPU_FREQ_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_MAX;
#endif
	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;
	return 0;
}

static ssize_t store_mfreq(struct sysdev_class *class,
			struct sysdev_class_attribute *attr,
			const char *buf, size_t count)
{
	u64 val;

	if (strict_strtoull(buf, 0, &val) < 0) {
		pr_err("Invalid parameter to mfreq\n");
		return 0;
	}
	if (val)
		override_cpu = 1;
	else
		override_cpu = 0;
	return count;
}

static SYSDEV_CLASS_ATTR(mfreq, 0200, NULL, store_mfreq);

static struct freq_attr *msm_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.name		= "msm",
	.attr		= msm_cpufreq_attr,
};

static int __init msm_cpufreq_register(void)
{
	int err = sysfs_create_file(&cpu_sysdev_class.kset.kobj,
			&attr_mfreq.attr);
	if (err)
		pr_err("Failed to create sysfs mfreq\n");

	return cpufreq_register_driver(&msm_cpufreq_driver);
}

late_initcall(msm_cpufreq_register);
