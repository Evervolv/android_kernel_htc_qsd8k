/* arch/arm/mach-msm/include/mach/msm_hdmi.h
 * Copyright (C) 2009 HTC Corporation.
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

#ifndef _MSM_HDMI_H_
#define _MSM_HDMI_H_

struct hdmi_platform_data {
	struct resource hdmi_res;
	/* power on hdmi chip */
	int (*power)(int on); /* mandatory */
	void (*hdmi_gpio_on)(void); /* optional */
	void (*hdmi_gpio_off)(void); /* optional */
};
#endif
