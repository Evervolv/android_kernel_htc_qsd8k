/* arch/arm/mach-msm/include/mach/htc_fast_clk.h
 *
 * Copyright (C) 2011 HTC, Inc.
 * Author: assd bt <htc_ssdbt@htc.com>
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

#define ID_WIFI	0
#define ID_BT	1
#define CLK_OFF	0
#define CLK_ON	1

struct htc_fast_clk_platform_data {
	int mode;
	/* fast clock control pin */
	int fast_clk_pin;
	char *id;
};

int htc_wifi_bt_fast_clk_ctl(int on, int id);
void htc_fast_clk_config(int fast_clk_pin);
