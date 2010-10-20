/* arch/arm/mach-msm/board-supersonic.h
 *
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
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SUPERSONIC_H
#define __ARCH_ARM_MACH_MSM_BOARD_SUPERSONIC_H

#include <mach/board.h>

#define MSM_SMI_BASE		0x02B00000
#define MSM_SMI_SIZE		0x01500000

#define MSM_HDMI_FB_BASE	0x02B00000
#define MSM_HDMI_FB_SIZE	0x00400000

#define MSM_PMEM_VENC_BASE	0x02F00000
#define MSM_PMEM_VENC_SIZE	0x00800000

#define MSM_GPU_MEM_BASE	0x03700000
#define MSM_GPU_MEM_SIZE	0x00500000

#define MSM_RAM_CONSOLE_BASE	0x03C00000
#define MSM_RAM_CONSOLE_SIZE	0x00040000

#ifdef CONFIG_BUILD_CIQ
#define MSM_PMEM_CIQ_BASE		MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE
#define MSM_PMEM_CIQ_SIZE		SZ_64K
#define MSM_PMEM_CIQ1_BASE	MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ1_SIZE	MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ2_BASE	MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ2_SIZE	MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ3_BASE	MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ3_SIZE	MSM_PMEM_CIQ_SIZE
#endif

#define MSM_FB_BASE		0x03D00000
#define MSM_FB_SIZE		0x00300000

#define MSM_EBI1_BANK0_BASE	0x20000000
//#define MSM_EBI1_BANK0_SIZE	0x0E000000	/* radio < 3210 */
#define MSM_EBI1_BANK0_SIZE	0x0E800000 /*for radio >=3210 */

/* 4Gb/512MB DRAM */
#define MSM_EBI1_BANK1_BASE	0x30000000
#define MSM_EBI1_BANK1_SIZE	0x0C000000

#define MSM_PMEM_MDP_BASE	0x3C000000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_PMEM_ADSP_BASE	0x3E000000
#define MSM_PMEM_ADSP_SIZE	0x02000000

#define SUPERSONIC_GPIO_UP_INT_N		35
#define SUPERSONIC_GPIO_UP_RESET_N		108

#define SUPERSONIC_GPIO_TP_RST		34
#define SUPERSONIC_GPIO_TP_INT_N		38
#define SUPERSONIC_GPIO_TP_EN			100	/* V_TP3V3_EN */

//#define SUPERSONIC_GPIO_POWER_KEY		94
#define SUPERSONIC_GPIO_SDMC_CD_N		28

/* BT */
#define SUPERSONIC_GPIO_BT_UART1_RTS	(43)
#define SUPERSONIC_GPIO_BT_UART1_CTS	(44)
#define SUPERSONIC_GPIO_BT_UART1_RX		(45)
#define SUPERSONIC_GPIO_BT_UART1_TX		(46)
#define SUPERSONIC_GPIO_BT_RESET_N		(27)
#define SUPERSONIC_GPIO_BT_SHUTDOWN_N	(146)
#define SUPERSONIC_GPIO_BT_HOST_WAKE	(86)
#define SUPERSONIC_GPIO_BT_CHIP_WAKE	(87)

#define SUPERSONIC_GPIO_COMPASS_RST_N	107
#define SUPERSONIC_GPIO_COMPASS_INT_N	36
#define SUPERSONIC_PROJECT_NAME        "supersonic"
#define SUPERSONIC_LAYOUTS             { \
		{ {  0,  1, 0}, { -1,  0, 0}, {0, 0, 1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, 1} }, \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0, 1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1, 0} }  \
					}

/* Proximity */
#define SUPERSONIC_GPIO_PROXIMITY_EN_N	109

/* Battery */
#define SUPERSONIC_GPIO_MBAT_IN		39
#define SUPERSONIC_GPIO_MCHG_EN_N	22
#define SUPERSONIC_GPIO_ISET		16

/*Audio */
#define SUPERSONIC_AUD_JACKHP_EN       157
#define SUPERSONIC_AUD_2V5_EN          26
#define SUPERSONIC_AUD_SPK_EN            129

/* Bluetooth PCM */
#define SUPERSONIC_BT_PCM_OUT          68
#define SUPERSONIC_BT_PCM_IN           69
#define SUPERSONIC_BT_PCM_SYNC         70
#define SUPERSONIC_BT_PCM_CLK          71

//#define SUPERSONIC_MENU_KEY   40
#define SUPERSONIC_VOLUME_UP     41
#define SUPERSONIC_VOLUME_DOWN   42
#define SUPERSONIC_POWER_KEY     94

/* flash light */
#define SUPERSONIC_GPIO_FLASHLIGHT_FLASH (84)
#define SUPERSONIC_GPIO_FLASHLIGHT_TORCH (85)
#define SUPERSONIC_GPIO_FLASHLIGHT_FLASH_ADJ (31)

/* AP Key Led turn on*/
#define SUPERSONIC_AP_KEY_LED_EN                     (32)

/* UART/USB switch : high -> UART, low -> HSUSB */
#define SUPERSONIC_USB_UARTz_SW			33
#define SUPERSONIC_WIMAX_CPU_UARTz_SW	160

/* USB PHY 3V3 enable*/
#define SUPERSONIC_USB_PHY_3V3_ENABLE   (104)
#define SUPERSONIC_GPIO_USB_CABLE_IN_PIN          (82)
#define SUPERSONIC_GPIO_USB_ID_PIN          (37)
/* 35mm headset */
#define SUPERSONIC_GPIO_35MM_HEADSET_DET (153)
#if 0 /* TODO */
//#define SUPERSONIC_GPIO_H2W_POWER (27)
//#define SUPERSONIC_GPIO_CABLE_IN1 (38)
#define SUPERSONIC_GPIO_CABLE_IN (37)
//#define SUPERSONIC_GPIO_H2W_DATA (139)
//#define SUPERSONIC_GPIO_H2W_CLK (140)
#endif

/* UART1*/
#define SUPERSONIC_GPIO_UART1_RX (139)
#define SUPERSONIC_GPIO_UART1_TX (140)

/* Wifi */
#define SUPERSONIC_GPIO_WIFI_SHUTDOWN_N	147
#define SUPERSONIC_GPIO_WIFI_IRQ		152
/*camera*/
#define SUPERSONIC_MAINCAM_PWD	105
#define SUPERSONIC_MAINCAM_RST	99
#define SUPERSONIC_2NDCAM_PWD	120
#define SUPERSONIC_CLK_SWITCH	102

#define SUPERSONIC_LCD_RST	(113)
unsigned supersonic_get_skuid(void);

/* HDMI */
#define HDMI_RST               (111)
#define V_HDMI_1V2_EN          (119)
#define V_VGA_5V_SIL9022A_EN   (127)
#define V_HDMI_3V3_EN          (128)
#define SUPERSONIC_I2S_CLK     (142)
#define SUPERSONIC_I2S_WS      (143)
#define SUPERSONIC_I2S_DOUT    (145)

/* LCD RGB */
#define SUPERSONIC_LCD_R0      (114)
#define SUPERSONIC_LCD_R1      (115)
#define SUPERSONIC_LCD_R2      (116)
#define SUPERSONIC_LCD_R3      (117)
#define SUPERSONIC_LCD_R4      (118)

#define SUPERSONIC_LCD_G0      (121)
#define SUPERSONIC_LCD_G1      (122)
#define SUPERSONIC_LCD_G2      (123)
#define SUPERSONIC_LCD_G3      (124)
#define SUPERSONIC_LCD_G4      (125)
#define SUPERSONIC_LCD_G5      (126)

#define SUPERSONIC_LCD_B0      (130)
#define SUPERSONIC_LCD_B1      (131)
#define SUPERSONIC_LCD_B2      (132)
#define SUPERSONIC_LCD_B3      (133)
#define SUPERSONIC_LCD_B4      (134)

#define SUPERSONIC_LCD_PCLK    (135)
#define SUPERSONIC_LCD_VSYNC   (136)
#define SUPERSONIC_LCD_HSYNC   (137)
#define SUPERSONIC_LCD_DE      (138)

#define SUPERSONIC_GPIO_PS_HOLD		(25)

#endif /* __ARCH_ARM_MACH_MSM_BOARD_SUPERSONIC_H */
