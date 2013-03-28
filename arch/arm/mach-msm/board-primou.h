/* linux/arch/arm/mach-msm/board-primou.h
 * Copyright (C) 2007-2009 HTC Corporation.
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_PRIMOU_H
#define __ARCH_ARM_MACH_MSM_BOARD_PRIMOU_H

#include <mach/board.h>

#define PRIMOU_GPIO_UART2_RX 	51
#define PRIMOU_GPIO_UART2_TX 	52

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x13F00000
#define MSM_LINUX_SIZE1		0x0C100000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP2_SIZE     0x002C0000

#define MSM_PMEM_AUDIO_SIZE	0x00200000

#define MSM_PMEM_ADSP_SIZE	0x02600000

#define PMEM_KERNEL_EBI1_SIZE   0x00700000

#define MSM_PMEM_SF_SIZE	0x01000000

#define MSM_FB_SIZE		0x00500000

#define PRIMOU_GPIO_WIFI_IRQ             147
#define PRIMOU_GPIO_WIFI_SHUTDOWN_N       39

#define PRIMOU_GPIO_KEYPAD_POWER_KEY		46
#define PRIMOU_GPIO_TORCH_EN			98

#define PRIMOU_GPIO_CHG_INT	180

#define PRIMOU_LAYOUTS			{ \
		{ { 0,  1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ { -1, 0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 0,  0,  1}, {1, 0,  0} }  \
					}
#define PRIMOU_MDDI_TE			(30)
#define PRIMOU_LCD_RSTz		(126)
#define PRIMOU_LCD_ID1			(128)
#define PRIMOU_LCD_ID0			(129)
/*
#define PRIMOU_LCD_PCLK               (90)
#define PRIMOU_LCD_DE                 (91)
#define PRIMOU_LCD_VSYNC              (92)
#define PRIMOU_LCD_HSYNC              (93)
#define PRIMOU_LCD_G2                 (94)
#define PRIMOU_LCD_G3                 (95)
#define PRIMOU_LCD_G4                 (96)
#define PRIMOU_LCD_G5                 (97)
#define PRIMOU_LCD_G6                 (98)
#define PRIMOU_LCD_G7                 (99)
#define PRIMOU_LCD_B3                 (100)
#define PRIMOU_LCD_B4                 (101)
#define PRIMOU_LCD_B5                 (102)
#define PRIMOU_LCD_B6                 (103)
#define PRIMOU_LCD_B7                 (104)
#define PRIMOU_LCD_R3                 (105)
#define PRIMOU_LCD_R4                 (106)
#define PRIMOU_LCD_R5                 (107)
#define PRIMOU_LCD_R6                 (108)
#define PRIMOU_LCD_R7                 (109)
*/
/* BT */
#define PRIMOU_GPIO_BT_UART1_RTS      (134)
#define PRIMOU_GPIO_BT_UART1_CTS      (135)
#define PRIMOU_GPIO_BT_UART1_RX       (136)
#define PRIMOU_GPIO_BT_UART1_TX       (137)
#define PRIMOU_GPIO_BT_WAKE           (31)
#define PRIMOU_GPIO_BT_SHUTDOWN_N     (38)
#define PRIMOU_GPIO_BT_RESET_N        (41)
#define PRIMOU_GPIO_BT_HOST_WAKE      (44)


#define PRIMOU_GPIO_BT_PCM_OUT        (138)
#define PRIMOU_GPIO_BT_PCM_IN         (139)
#define PRIMOU_GPIO_BT_PCM_SYNC       (140)
#define PRIMOU_GPIO_BT_PCM_CLK        (141)


/* USB */
#define PRIMOU_GPIO_USB_ID_PIN			(49)
#define PRIMOU_GPIO_USB_ID1_PIN			(145)

/*
#define PRIMOU_SPI_CS2                (87)
#define PRIMOU_SPI_CLK                (45)
*/
#define PRIMOU_GPIO_PS_HOLD	(29)

#define PRIMOU_AUD_CODEC_EN          (36)
/*#define PRIMOU_AUD_MICPATH_SEL		(127)*/

/* 35mm headset */
#define PRIMOU_GPIO_35MM_HEADSET_DET	(26)

/* EMMC */
#define PRIMOU_GPIO_EMMC_RST			  (88)

/* Touch */
#define PRIMOU_GPIO_TP_ATT_N			(20)

/* Camera */
#define PRIMOU_S5K4E5YX_EVT2					(2)
#define PRIMOU_S5K4E5YX_EVT1					(1)
#define PRIMOU_GPIO_CAM_I2C_SCL					(16)
#define PRIMOU_GPIO_CAM_I2C_SDA					(17)
#define PRIMOU_GPIO_CAM1_PWD						(35)
#define PRIMOU_GPIO_CAM1_VCM_PWD					(34)
#define PRIMOU_GPIO_CAM_MCLK						(15)
#define PRIMOU_GPIO_CAM_ID						(87)

#ifdef CONFIG_RAWCHIP
/* Rawchip */
#define PRIMOU_GPIO_RAW_INTR0					(142)
#define PRIMOU_GPIO_RAW_INTR1					(18)
#define PRIMOU_GPIO_RAW_RSTN						(19)
#define PRIMOU_GPIO_RAW_1V2_EN					(104)
/* Rawchip SPI */
#define PRIMOU_GPIO_MCAM_SPI_DO					(47)
#define PRIMOU_GPIO_MCAM_SPI_DI					(48)
#define PRIMOU_GPIO_MCAM_SPI_CLK					(45)
#define PRIMOU_GPIO_MCAM_SPI_CS					(89)
#endif
#define PMIC_GPIO_INT		27
#define PRIMOU_GPIO_PS_HOLD	(29) /*ACPU GPIO Number 29*/

/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define PRIMOU_SIM_DOOR				PMGPIO(3)
#define PRIMOU_GPIO_GSENSOR_INT2		PMGPIO(6)
#define PRIMOU_GPIO_GSENSOR_INT		PMGPIO(7)
#define PRIMOU_AUD_REMO_PRES			PMGPIO(8)
#define PRIMOU_AUD_SPK_SD				PMGPIO(12)
#define PRIMOU_GPIO_FLASH_EN			PMGPIO(15)
#define PRIMOU_VOL_UP					PMGPIO(16)
#define PRIMOU_VOL_DN					PMGPIO(17)
/*#define PRIMOU_AUD_HANDSET_ENO		PMGPIO(19)//Primou doesn't have this pin, sync mecha audio.*/
/*#define PRIMOU_GPIO_PS_EN				PMGPIO(20)*/
#define PRIMOU_TP_RSTz					PMGPIO(21)
#define PRIMOU_GPIO_PS_INT_N			PMGPIO(22)
/*#define PRIMOU_GPIO_UP_INT_N			PMGPIO(23)*/
/* Primo#U move amber/green to LED_DRV0/LED_DRV1
#define PRIMOU_GREEN_LED				PMGPIO(24)
#define PRIMOU_AMBER_LED				PMGPIO(25)
*/
#define PRIMOU_AUD_AMP_EN				PMGPIO(26)
#define PRIMOU_H_SIM_RST				PMGPIO(27)
#define PRIMOU_SIM_RST					PMGPIO(28)
#define PRIMOU_H_SIM_CLK				PMGPIO(29)
#define PRIMOU_SIM_CLK					PMGPIO(30)
#define PRIMOU_CHG_STAT				PMGPIO(33)
#define PRIMOU_GPIO_SDMC_CD_N				PMGPIO(34)
/*#define PRIMOU_GPIO_LS_EN				PMGPIO(35)*/
/*#define PRIMOU_GPIO_uP_RST 			PMGPIO(36)*/
#define PRIMOU_GPIO_COMPASS_INT_N	PMGPIO(37)
#define PRIMOU_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)

/*display*/
extern struct platform_device msm_device_mdp;
extern struct platform_device msm_device_mddi0;
extern int panel_type;

int primou_init_mmc(unsigned int sys_rev);
void __init primou_audio_init(void);
int primou_init_keypad(void);
int __init primou_wifi_init(void);

/*primou and primouw will share one panel code */
int __init primou_init_panel(void);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_PRIMOU_H */
