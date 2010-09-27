/*
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kernel_stat.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include "avs.h"

#define AVSDSCR_INPUT 0x01004860 /* magic # from circuit designer */
#define TSCSR_INPUT   0x00000001 /* enable temperature sense */

#define TEMPRS 16                /* total number of temperature regions */
#define GET_TEMPR() (avs_get_tscsr() >> 28) /* scale TSCSR[CTEMP] to regions */

struct mutex avs_lock;

static struct avs_state_s
{
	u32 freq_cnt;		/* Frequencies supported list */
	short *avs_v;		/* Dyanmically allocated storage for
				 * 2D table of voltages over temp &
				 * freq.  Used as a set of 1D tables.
				 * Each table is for a single temp.
				 * For usage see avs_get_voltage
				 */
	int (*set_vdd) (int);	/* Function Ptr for setting voltage */
	int changing;		/* Clock frequency is changing */
	u32 freq_idx;		/* Current frequency index */
	int vdd;		/* Current ACPU voltage */
} avs_state;

struct clkctl_acpu_speed {
	unsigned acpu_khz;
	int	 min_vdd;
	int	 max_vdd;
};


struct clkctl_acpu_speed acpu_vdd_tbl[] = {
	{  19200, 950, 950 },
	{ 128000, 950, 950 },
	{ 245000, 950, 975 },
	{ 256000, 950, 975 },
	{ 384000, 950, 1000 },
	{ 422400, 950, 1000 },
	{ 460800, 975, 1025 },
	{ 499200, 1000, 1050 },
	{ 537600, 1000, 1050 },
	{ 576000, 1025, 1075 },
	{ 614400, 1050, 1100 },
	{ 652800, 1075, 1125 },
	{ 691200, 1100, 1150 },
	{ 729600, 1125, 1175 },
	{ 768000, 1150, 1200 },
	{ 806400, 1175, 1225 },
	{ 844800, 1200, 1250 },
	{ 883200, 1200, 1250 },
	{ 921600, 1225, 1275 },
	{ 960000, 1225, 1275 },
	{ 998400, 1225, 1275 },
	{ 0 },
};

/*
 *  Update the AVS voltage vs frequency table, for current temperature
 *  Adjust based on the AVS delay circuit hardware status
 */
static void avs_update_voltage_table(short *vdd_table)
{
	u32 avscsr;
	int cpu;
	int vu;
	int l2;
	int i;
	u32 cur_freq_idx;
	short cur_voltage;

	cur_freq_idx = avs_state.freq_idx;
	cur_voltage = avs_state.vdd;

	avscsr = avs_test_delays();
	AVSDEBUG("avscsr=%x, avsdscr=%x\n", avscsr, avs_get_avsdscr());

	/*
	 * Read the results for the various unit's AVS delay circuits
	 * 2=> up, 1=>down, 0=>no-change
	 */
	cpu = ((avscsr >> 23) & 2) + ((avscsr >> 16) & 1);
	vu  = ((avscsr >> 28) & 2) + ((avscsr >> 21) & 1);
	l2  = ((avscsr >> 29) & 2) + ((avscsr >> 22) & 1);

	if ((cpu == 3) || (vu == 3) || (l2 == 3)) {
		printk(KERN_ERR "AVS: Dly Synth O/P error\n");
	} else if ((cpu == 2) || (l2 == 2) || (vu == 2)) {
		/*
		 * even if one oscillator asks for up, increase the voltage,
		 * as its an indication we are running outside the
		 * critical acceptable range of v-f combination.
		 */
		AVSDEBUG("cpu=%d l2=%d vu=%d\n", cpu, l2, vu);
		AVSDEBUG("Voltage up at %d\n", cur_freq_idx);

		if (cur_voltage >= VOLTAGE_MAX || cur_voltage >= acpu_vdd_tbl[cur_freq_idx].max_vdd)
			printk(KERN_ERR
				"AVS: Voltage can not get high enough!\n");

		/* Raise the voltage for all frequencies */
		for (i = 0; i < avs_state.freq_cnt; i++) {
			vdd_table[i] = cur_voltage + VOLTAGE_STEP;
			if (vdd_table[i] > VOLTAGE_MAX)
				vdd_table[i] = VOLTAGE_MAX;
			else if (vdd_table[i] > acpu_vdd_tbl[i].max_vdd)
				vdd_table[i] = acpu_vdd_tbl[i].max_vdd;
		}
	} else if ((cpu == 1) && (l2 == 1) && (vu == 1)) {
		if ((cur_voltage - VOLTAGE_STEP >= VOLTAGE_MIN) &&
		    (cur_voltage - VOLTAGE_STEP >= acpu_vdd_tbl[cur_freq_idx].min_vdd) &&
		    (cur_voltage <= vdd_table[cur_freq_idx])) {
			vdd_table[cur_freq_idx] = cur_voltage - VOLTAGE_STEP;
			AVSDEBUG("Voltage down for %d and lower levels\n",
				cur_freq_idx);

			/* clamp to this voltage for all lower levels */
			for (i = 0; i < cur_freq_idx; i++) {
				if (vdd_table[i] > vdd_table[cur_freq_idx])
					vdd_table[i] = vdd_table[cur_freq_idx];
			}
		}
	}
}

/*
 * Return the voltage for the target performance freq_idx and optionally
 * use AVS hardware to check the present voltage freq_idx
 */
static short avs_get_target_voltage(int freq_idx, bool update_table)
{
	unsigned cur_tempr = GET_TEMPR();
	unsigned temp_index = cur_tempr*avs_state.freq_cnt;

	/* Table of voltages vs frequencies for this temp */
	short *vdd_table = avs_state.avs_v + temp_index;

	if (update_table)
		avs_update_voltage_table(vdd_table);

	if (vdd_table[freq_idx] > acpu_vdd_tbl[freq_idx].max_vdd) {
		pr_info("%dmV too high for %d.\n", vdd_table[freq_idx], acpu_vdd_tbl[freq_idx].acpu_khz);
		vdd_table[freq_idx] = acpu_vdd_tbl[freq_idx].max_vdd;
	}
	if (vdd_table[freq_idx] < acpu_vdd_tbl[freq_idx].min_vdd) {
		pr_info("%dmV too low for %d.\n", vdd_table[freq_idx], acpu_vdd_tbl[freq_idx].acpu_khz);
		vdd_table[freq_idx] = acpu_vdd_tbl[freq_idx].min_vdd;
	}

	return vdd_table[freq_idx];
}


/*
 * Set the voltage for the freq_idx and optionally
 * use AVS hardware to update the voltage
 */
static int avs_set_target_voltage(int freq_idx, bool update_table)
{
	int rc = 0, new_voltage;

	if (freq_idx < 0 || freq_idx >= avs_state.freq_cnt) {
		AVSDEBUG("Out of range :%d\n", freq_idx);
		return -EINVAL;
	}

	new_voltage = avs_get_target_voltage(freq_idx, update_table);
	if (avs_state.vdd != new_voltage) {
		/*AVSDEBUG*/pr_info("AVS setting V to %d mV @%d MHz\n",
			new_voltage, acpu_vdd_tbl[freq_idx].acpu_khz / 1000);
		rc = avs_state.set_vdd(new_voltage);
		if (rc)
			return rc;
		avs_state.vdd = new_voltage;
	}
	return rc;
}

/*
 * Notify avs of clk frquency transition begin & end
 */
int avs_adjust_freq(u32 freq_idx, int begin)
{
	int rc = 0;

	if (!avs_state.set_vdd) {
		/* AVS not initialized */
		return 0;
	}

	if (freq_idx < 0 || freq_idx >= avs_state.freq_cnt) {
		AVSDEBUG("Out of range :%d\n", freq_idx);
		return -EINVAL;
	}

	mutex_lock(&avs_lock);
	if ((begin && (freq_idx > avs_state.freq_idx)) ||
	    (!begin && (freq_idx < avs_state.freq_idx))) {
		/* Update voltage before increasing frequency &
		 * after decreasing frequency
		 */
		rc = avs_set_target_voltage(freq_idx, 0);
		if (rc)
			goto aaf_out;

		avs_state.freq_idx = freq_idx;
	}
	avs_state.changing = begin;
aaf_out:
	mutex_unlock(&avs_lock);

	return rc;
}


static struct delayed_work avs_work;
static struct workqueue_struct  *kavs_wq;
#define AVS_DELAY ((CONFIG_HZ * 50 + 999) / 1000)

static void do_avs_timer(struct work_struct *work)
{
	int cur_freq_idx;

	mutex_lock(&avs_lock);
	if (!avs_state.changing) {
		/* Only adjust the voltage if clk is stable */
		cur_freq_idx = avs_state.freq_idx;
		avs_set_target_voltage(cur_freq_idx, 1);
	}
	mutex_unlock(&avs_lock);
	queue_delayed_work_on(0, kavs_wq, &avs_work, AVS_DELAY);
}


static void __init avs_timer_init(void)
{
	INIT_DELAYED_WORK_DEFERRABLE(&avs_work, do_avs_timer);
	queue_delayed_work_on(0, kavs_wq, &avs_work, AVS_DELAY);
}

static void __exit avs_timer_exit(void)
{
	cancel_delayed_work(&avs_work);
}

static int __init avs_work_init(void)
{
	kavs_wq = create_workqueue("avs");
	if (!kavs_wq) {
		printk(KERN_ERR "AVS initialization failed\n");
		return -EFAULT;
	}
	avs_timer_init();

	return 1;
}

static void __exit avs_work_exit(void)
{
	avs_timer_exit();
	destroy_workqueue(kavs_wq);
}

int __init avs_init(int (*set_vdd)(int), u32 freq_cnt, u32 freq_idx)
{
	int i;

	mutex_init(&avs_lock);

	if (freq_cnt == 0)
		return -EINVAL;

	avs_state.freq_cnt = freq_cnt;

	if (freq_idx >= avs_state.freq_cnt)
		return -EINVAL;

	avs_state.avs_v = kmalloc(TEMPRS * avs_state.freq_cnt *
		sizeof(avs_state.avs_v[0]), GFP_KERNEL);

	if (avs_state.avs_v == 0)
		return -ENOMEM;

	for (i = 0; i < TEMPRS*avs_state.freq_cnt; i++)
		avs_state.avs_v[i] = VOLTAGE_MAX;

	avs_reset_delays(AVSDSCR_INPUT);
	avs_set_tscsr(TSCSR_INPUT);

	avs_state.set_vdd = set_vdd;
	avs_state.changing = 0;
	avs_state.freq_idx = -1;
	avs_state.vdd = -1;
	avs_adjust_freq(freq_idx, 0);

	avs_work_init();

	return 0;
}

void __exit avs_exit()
{
	avs_work_exit();

	kfree(avs_state.avs_v);
}


