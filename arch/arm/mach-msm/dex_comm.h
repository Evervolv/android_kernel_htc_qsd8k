/* arch/arm/mach-msm/proc_comm.h
 *
 * Copyright (c) 2007 QUALCOMM Incorporated
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

#ifndef _ARCH_ARM_MACH_MSM_MSM_DEX_COMM_CE_H_
#define _ARCH_ARM_MACH_MSM_MSM_DEX_COMM_CE_H_

// DEX_* research taken from http://wiki.xda-developers.com/index.php?pagename=RaphaelDEX
// Research by cr2

#define DEX_HAS_DATA    0x100
#define DEX_STATUS_FAIL 0x200

enum {
	DEX_PMIC_WLAN_ON = 0x2,
	DEX_PMIC_WLAN_OFF = 0x3,
	DEX_PMIC_VIBRA_ON = 0x4,
	DEX_PMIC_VIBRA_OFF = 0x5,
	DEX_PMIC_IR_ON = 0x6,
	DEX_PMIC_IR_OFF = 0x7,
	DEX_PMIC_CAM_ON = 0x8,
	DEX_PMIC_CAM_OFF = 0x9,
	DEX_PMIC_VGACAM_ON = 0xa,
	DEX_PMIC_VGACAM_OFF = 0xb,
	DEX_PMIC_SD_ON = 0xc,
	DEX_PMIC_SD_OFF = 0xd,
	DEX_PMIC_LCD_ON = 0xe,
	DEX_PMIC_LCD_OFF = 0xf,
	DEX_PMIC_MDDI_ON = 0x10,
	DEX_PMIC_MDDI_OFF = 0x11,
	DEX_PMIC_BT_ON = 0x12,
	DEX_PMIC_BT_OFF = 0x13,
	DEX_POWER_OFF = 0x14,
	DEX_PMIC_REG_ON = 0x15,
	DEX_PMIC_REG_OFF = 0x16,
	DEX_VIBRA_ON = 0x17,
	DEX_VIBRA_OFF = 0x18,
	DEX_SET_AUDIO_PATH = 0x19,
	DEX_PMIC_REG_VOLTAGE = 0x1a,
	DEX_SETUSB_DPLUS = 0x1b,
	DEX_AUDIO_CALL = 0x1c,

	//???
	DEX_SET_L2_LOCK_BUS_CLK = 0x1d,
	DEX_ARM9_LOW_SPEED = 0x1d,
	DEX_REGISTER_VOCODER_PCM = 0x1e,
	DEX_UNREGISTER_VOCODER_PCM = 0x1f,
	DEX_SET_CLOCK_ON = 0x20,
	DEX_SET_CLOCK_OFF = 0x21,
	DEX_RESET_ARM9 = 0x22,

	DEX_PMIC_TVOUT_AUTO_ON = 0x25,
	DEX_PMIC_TVOUT_AUTO_FF = 0x26,

	DEX_LCD_STATUS = 0x30,
	DEX_CONFIG_MPP_PINS = 0x31,
	DEX_SET_CHARGER_STATUS = 0x32,
	DEX_TASK_REGISTER = 0x33,
	DEX_TASK_UNREGISTER = 0x34,

	DEX_UPDATE_ACDB = 0x80,
	DEX_READ_RTC = 0x81,
	DEX_WRITE_RTC = 0x82,
	DEX_SET_ALARM_RTC = 0x84,

	DEX_GET_BATTERY_DATA = 0x8a,
	DEX_GET_BATTERY_ID = 0x8b,

	DEX_NOTIFY_ARM9_REBOOT = 0x8e,

	DEX_GET_TX_POWER = 0x90,
	DEX_GET_NETWORK_BAND = 0x91,
	DEX_GET_GSM_TX_BAND = 0x92,

	DEX_GET_SLEEP_CLOCK = 0xa2,

	DEX_FOTA_READ = 0xa4,
	DEX_FOTA_WRITE = 0xa5,
};


// ??
#define GPIO_ENABLE     0
#define GPIO_DISABLE    1

// .dir (1b)
#define GPIO_INPUT      0
#define GPIO_OUTPUT     1

// .pull (2b)
#define GPIO_NO_PULL    0
#define GPIO_PULL_DOWN  1
#define GPIO_KEEPER     2
#define GPIO_PULL_UP    3

// .drvstr (4b)
#define GPIO_2MA        0
#define GPIO_4MA        1
#define GPIO_6MA        2
#define GPIO_8MA        3
#define GPIO_10MA       4
#define GPIO_12MA       5
#define GPIO_14MA       6
#define GPIO_16MA       7

#define DEX_GPIO_CFG(a, b, c, d, e, f) (struct msm_gpio_config){ \
		.gpio = (a), \
		.dir = (c), \
		.out_op = (f), \
		.pull = (d), \
		.func = (b), \
		.drvstr = (e) \
	}


int dex_comm(unsigned cmd, unsigned *data1, unsigned *data2);
int dex_audio(int param);

int init_dex_comm(void);

#endif


