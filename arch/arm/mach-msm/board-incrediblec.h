/* arch/arm/mach-msm/board-incrediblec.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_INCREDIBLEC_H
#define __ARCH_ARM_MACH_MSM_BOARD_INCREDIBLEC_H

#include <mach/board.h>


#define MSM_SMI_BASE		0x02B00000
#define MSM_SMI_SIZE		0x01500000

#ifdef CONFIG_720P_CAMERA
#define MSM_PMEM_VENC_BASE	0x02B00000
#define MSM_PMEM_VENC_SIZE	0x00800000
/* rest 4MB SMI */
#else
#define MSM_PMEM_CAMERA_BASE	0x02B00000
#define MSM_PMEM_CAMERA_SIZE	0x00C00000
#endif

#define MSM_GPU_MEM_BASE	0x03700000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x03A00000
#define MSM_RAM_CONSOLE_SIZE	0x00040000

#define MSM_FB_BASE		0x03B00000
#define MSM_FB_SIZE		0x00300000

#define MSM_EBI1_BANK0_BASE	0x20000000
#define MSM_EBI1_BANK0_SIZE	0x0E800000

#define MSM_EBI1_BANK1_BASE     0x30000000
#define MSM_EBI1_BANK1_SIZE     0x03E00000

#define MSM_PMEM_MDP_BASE	0x33E00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_PMEM_ADSP_BASE	0x35E00000
#define MSM_PMEM_ADSP_SIZE	0x02000000

#define MSM_TV_FB_BASE          0x37E00000
#define MSM_TV_FB_SIZE          0x00200000

#define MSM_MEM_128MB_OFFSET	0x08000000

/* 4G2G MCP */
#define MSM_PMEM_ADSP_XA_BASE   0x29000000

#define MSM_TV_FB_XA_BASE       0x2B900000

#define MSM_PMEM_MDP_XA_BASE    0x2BB00000

#define MSM_GPU_MEM_XA_BASE     0x03700000

#define MSM_LINUX_XA_SIZE       0x09000000
/* 4G2G END */

#define INCREDIBLEC_GPIO_UP_INT_N		35
#define INCREDIBLEC_GPIO_UP_RESET_N	108

#define INCREDIBLEC_GPIO_TP_RST		34
#define INCREDIBLEC_GPIO_TP_INT_N		145
/*#define INCREDIBLEC_GPIO_TP_LS_EN		93*/
#define INCREDIBLEC_GPIO_TP_EN			98

#define INCREDIBLEC_GPIO_SDMC_CD_N		28

/* BT */
#define INCREDIBLEC_GPIO_BT_UART1_RTS (43)
#define INCREDIBLEC_GPIO_BT_UART1_CTS (44)
#define INCREDIBLEC_GPIO_BT_UART1_RX (45)
#define INCREDIBLEC_GPIO_BT_UART1_TX (46)
#define INCREDIBLEC_GPIO_BT_RESET_N (146)
#define INCREDIBLEC_GPIO_BT_HOST_WAKE (86)
#define INCREDIBLEC_GPIO_BT_CHIP_WAKE (87)
#define INCREDIBLEC_GPIO_BT_SHUTDOWN_N (128)

#define INCREDIBLEC_GPIO_COMPASS_RST_N	107
#define INCREDIBLEC_GPIO_COMPASS_INT_N	36
#define INCREDIBLEC_PROJECT_NAME        "incrediblec"
#define INCREDIBLEC_LAYOUTS             { \
		{ {  0,  1, 0}, { -1,  0, 0}, {0, 0, 1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, 1} }, \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0, 1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1, 0} }  \
					}

/* Proximity */
#define INCREDIBLEC_GPIO_PROXIMITY_EN_N	120

/* Battery */
#define INCREDIBLEC_GPIO_MBAT_IN		39
#define INCREDIBLEC_GPIO_MCHG_EN_N	22
#define INCREDIBLEC_GPIO_ISET		16

/*Audio */
#define INCREDIBLEC_AUD_JACKHP_EN       157
#define INCREDIBLEC_AUD_2V5_EN          26

/* Bluetooth PCM */
#define INCREDIBLEC_BT_PCM_OUT          68
#define INCREDIBLEC_BT_PCM_IN           69
#define INCREDIBLEC_BT_PCM_SYNC         70
#define INCREDIBLEC_BT_PCM_CLK          71

#define INCREDIBLEC_GPIO_MENU_KEY	40
#define INCREDIBLEC_GPIO_VOLUME_UP	41
#define INCREDIBLEC_GPIO_VOLUME_DOWN	42
#define INCREDIBLEC_GPIO_POWER_KEY	94
#define INCREDIBLEC_GPIO_OJ_ACTION_XB	33

/* flash light */
#define INCREDIBLEC_GPIO_FLASHLIGHT_FLASH	(84)
#define INCREDIBLEC_GPIO_FLASHLIGHT_TORCH	(85)
#define INCREDIBLEC_GPIO_FLASHLIGHT_FLASH_ADJ	(31)

/* 35mm headset */
#define INCREDIBLEC_GPIO_35MM_HEADSET_DET (153)
#define INCREDIBLEC_GPIO_CABLE_IN1 (38)
#define INCREDIBLEC_GPIO_CABLE_IN2 (37)
#define INCREDIBLEC_GPIO_H2W_DATA (139)
#define INCREDIBLEC_GPIO_H2W_CLK (140)
#define INCREDIBLEC_GPIO_UART3_RX (139)
#define INCREDIBLEC_GPIO_UART3_TX (140)

/* Wifi */
#define INCREDIBLEC_GPIO_WIFI_SHUTDOWN_N	127
#define INCREDIBLEC_GPIO_WIFI_IRQ		152

/* SPI */
#define INCREDIBLEC_SPI_CLK                     (17)
#define INCREDIBLEC_SPI_DO                      (18)
#define INCREDIBLEC_SPI_CS                      (20)

#define INCREDIBLEC_LCD_RST_ID1                 (29)
#define INCREDIBLEC_LCD_ID0                     (32)

/* TV-out */
#define INCREDIBLEC_TV_LOAD_DET			(82)
#define INCREDIBLEC_VIDEO_SHDN_N                (109)
#define INCREDIBLEC_AV_SWITCH			(119)

/* LCD */
#define INCREDIBLEC_LCD_R0                      (113)
#define INCREDIBLEC_LCD_R1                      (114)
#define INCREDIBLEC_LCD_R2                      (115)
#define INCREDIBLEC_LCD_R3                      (116)
#define INCREDIBLEC_LCD_R4                      (117)
#define INCREDIBLEC_LCD_R5                      (118)
#define INCREDIBLEC_LCD_G0                      (121)
#define INCREDIBLEC_LCD_G1                      (122)
#define INCREDIBLEC_LCD_G2                      (123)
#define INCREDIBLEC_LCD_G3                      (124)
#define INCREDIBLEC_LCD_G4                      (125)
#define INCREDIBLEC_LCD_G5                      (126)
#define INCREDIBLEC_LCD_B0                      (129)
#define INCREDIBLEC_LCD_B1                      (130)
#define INCREDIBLEC_LCD_B2                      (131)
#define INCREDIBLEC_LCD_B3                      (132)
#define INCREDIBLEC_LCD_B4                      (133)
#define INCREDIBLEC_LCD_B5                      (134)
#define INCREDIBLEC_LCD_PCLK                    (135)
#define INCREDIBLEC_LCD_VSYNC                   (136)
#define INCREDIBLEC_LCD_HSYNC                   (137)
#define INCREDIBLEC_LCD_DE                      (138)

/* USB PHY 3V3 enable*/
#define INCREDIBLEC_USB_PHY_3V3_ENABLE                      (104)
#define INCREDIBLEC_GPIO_USB_CABLE_IN_PIN          (144)
#define INCREDIBLEC_GPIO_USB_ID_PIN          (112)

/* AP Key Led turn on*/
#define INCREDIBLEC_AP_KEY_LED_EN                     (143)

/*Camera*/
#define INCREDIBLEC_CAM_PWD	(100)
#define INCREDIBLEC_CAM_RST	(99)

#define INCREDIBLEC_GPIO_PS_HOLD	(25)

unsigned int incrediblec_get_engineerid(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_INCREDIBLEC_H */
