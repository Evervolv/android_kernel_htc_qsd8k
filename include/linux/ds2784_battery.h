/* include/linux/ds2784_battery.h
 *
 * Copyright (C) 2009 HTC Corporation
 * Copyright (C) 2009 Google, Inc.
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
#ifndef _DS2784_BATTERY_H_
#define _DS2784_BATTERY_H_
#include <linux/notifier.h>
#include <mach/htc_battery.h>

enum ds2784_notify_evt_t{
	DS2784_CHARGING_CONTROL = 0,
	DS2784_LEVEL_UPDATE,
	DS2784_BATTERY_FAULT,
	DS2784_OVER_TEMP,
	DS2784_NUM_EVENTS,
};

#ifdef CONFIG_BATTERY_DS2784
extern int ds2784_register_notifier(struct notifier_block *nb);
extern int ds2784_unregister_notifier(struct notifier_block *nb);
extern int ds2784_get_battery_info(struct battery_info_reply *batt_info);
extern ssize_t htc_battery_show_attr(struct device_attribute *attr,
			char *buf);
#else
static int ds2784_register_notifier(struct notifier_block *nb) { return 0; }
static int ds2784_unregister_notifier(struct notifier_block *nb) { return 0; }
static int ds2784_get_battery_info(struct battery_info_reply *batt_info)
{
	batt_info->level = 10;
	return 0;
}
extern ssize_t htc_battery_show_attr(struct device_attribute *attr,
			char *buf){ return 0; }

struct ds2784_platform_data {
	int (*charge)(int on, int fast);
	void *w1_slave;
};

#endif

#endif //endif _DS2784_BATTERY_H_