/* board-htcleo-mmc.h
 *
 * Copyright (C) 2011 marc1706
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

#ifndef HTCLEO_MMC_H
#define HTCLEO_MMC_H

static bool opt_disable_sdcard;
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int htcleo_wifi_power_state;
static int htcleo_wifi_reset_state;

int htcleo_wifi_set_carddetect(int val);
int htcleo_wifi_power(int on);
int htcleo_wifi_reset(int on);
int __init htcleo_init_mmc(unsigned debug_uart);

#endif // HTCLEO_MMC_H

