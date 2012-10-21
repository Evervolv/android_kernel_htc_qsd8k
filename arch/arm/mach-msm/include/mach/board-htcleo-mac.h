/* arch/arm/mach-msm/include/mach/board-htcleo-mac.h
 *
 * Copyright (C) 2012 Marc Alexander.
 * Author: Marc Alexander<admin@m-a-styles.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_MAC_H
#define __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_MAC_H

#define NVS_MACADDR_SIZE	0x1AU

extern char nvs_mac_addr[NVS_MACADDR_SIZE];

#define BDADDR_STR_SIZE 18

extern char bdaddr[BDADDR_STR_SIZE];

#endif
