/*
 * Author: Markinus
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

#ifndef _ARCH_ARM_MACH_MSM_AMSS_PARA_H
#define _ARCH_ARM_MACH_MSM_AMSS_PARA_H

extern unsigned int __amss_version;

struct amss_value {
	int id;
	uint32_t numval;
	char* strval;
};

// The enum with AMSS IDs, it have to have the same order and size as the default para struct in the c file!
enum amss_ids {
	AUDMGR_PROG_VERS=0,
	AUDMGR_VERS,
	AUDMGR_CB_PROG_VERS,
	AUDMGR_CB_VERS,
	TIME_REMOTE_MTOA_VERS,
	RPC_TIME_REMOTE_MTOA_NULL,
	RPC_TIME_TOD_SET_APPS_BASES,
	PM_LIBVERS,
	RPC_SND_VERS,
	SND_SET_DEVICE_PROC,
	SND_SET_VOLUME_PROC,
	RPC_ADSP_RTOS_ATOM_PROG_VERS,
	RPC_ADSP_RTOS_ATOM_VERS,
	RPC_ADSP_RTOS_MTOA_VERS,
	RPC_ADSP_RTOS_ATOM_NULL_PROC,
	RPC_ADSP_RTOS_MTOA_NULL_PROC,
	RPC_ADSP_RTOS_APP_TO_MODEM_PROC,
	RPC_ADSP_RTOS_MODEM_TO_APP_PROC,
	RPC_DOG_KEEPALIVE_NULL,  
	RPC_DOG_KEEPALIVE_BEACON,
	DOG_KEEPALIVE_VERS,
	HTC_PROCEDURE_SET_VIB_ON_OFF,
	APP_TIMEREMOTE_PDEV_NAME,
};

extern uint32_t amss_get_num_value(int);
  
extern void amss_get_str_value(int, char*, size_t);

 
#endif	//LOCK

