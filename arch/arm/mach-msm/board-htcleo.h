/* arch/arm/mach-msm/board-htcleo.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_H
#define __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_H

#include <mach/board.h>

#define MSM_EBI1_BANK0_BASE     0x11800000
//#define MSM_EBI1_BANK0_SIZE     0x1E800000 /* 488MB */
#define MSM_EBI1_BANK0_SIZE     0x1E7C0000 /* 488MB - 0x00040000 RAM CONSOLE*/

/* Don't change that */
#define MSM_SMI_BASE		0x00000000
#define MSM_SMI_SIZE		0x04000000

/* Begin SMI region */
/* First part of SMI is used for OEMSBL & AMSS */
#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + 0x02500000)
#define MSM_PMEM_SMI_SIZE	0x01B00000

#define MSM_PMEM_VENC_BASE      0x02B00000
#define MSM_PMEM_VENC_SIZE      0x00800000

#define MSM_GPU_PHYS_BASE       0x03300000
#define MSM_GPU_PHYS_SIZE       0x00500000

//#define MSM_RAM_CONSOLE_BASE	0x03A00000
//#define MSM_RAM_CONSOLE_SIZE	0x00040000

#define MSM_FB_BASE		0x03B00000
#define MSM_FB_SIZE		0x00300000

#define MSM_PMEM_MDP_BASE	0x3B700000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_PMEM_ADSP_BASE	0x3D700000
#define MSM_PMEM_ADSP_SIZE	0x02900000
/* Begin EBI region */
#define PMEM_KERNEL_EBI1_SIZE	0x00028000

//#define MSM_PMEM_SF_SIZE        0x02000000

/* MSM_RAM_CONSOLE uses the last 0x00040000 of EBI memory, defined in msm_iomap.h
#define MSM_RAM_CONSOLE_SIZE    0x00040000
#define MSM_RAM_CONSOLE_BASE    (MSM_EBI1_BANK0_BASE + MSM_EBI1_BANK0_SIZE - MSM_RAM_CONSOLE_SIZE) //0x2FFC0000
*/

#define MSM_PANEL_PHYS        0x00080000
#define MSM_PANEL_SIZE        0x00080000

/* End EBI region */

#define HTCLEO_GPIO_PS_HOLD			25

#define HTCLEO_GPIO_UP_INT_N			90
#define HTCLEO_GPIO_UP_RESET_N			91
#define HTCLEO_GPIO_LS_EN_N			119

#define HTCLEO_GPIO_TP_INT_N			92
#define HTCLEO_GPIO_TP_LS_EN			93
#define HTCLEO_GPIO_TP_EN			160

#define HTCLEO_GPIO_POWER_KEY			94
#define HTCLEO_GPIO_SD_STATUS			153

#define HTCLEO_GPIO_WIFI_SHUTDOWN_N		129
#define HTCLEO_GPIO_WIFI_IRQ			152

#define HTCLEO_GPIO_VIBRATOR_ON         	100

/* Flashlight */
#define HTCLEO_GPIO_FLASHLIGHT_TORCH		159
#define HTCLEO_GPIO_FLASHLIGHT_FLASH    	143



#define HTCLEO_AUD_JACKHP_EN  			157
#define HTCLEO_AUD_2V5_EN     			158
#define HTCLEO_BT_PCM_OUT     			68
#define HTCLEO_BT_PCM_IN      			69
#define HTCLEO_BT_PCM_SYNC    			70
#define HTCLEO_BT_PCM_CLK    			71

/* Headset */
#define HTCLEO_GPIO_HDS_MIC      		35
#define HTCLEO_GPIO_HDS_DET      		145

/* Keypad */
#define HTCLEO_GPIO_KP_MKOUT0    		33
#define HTCLEO_GPIO_KP_MKOUT1    		32
#define HTCLEO_GPIO_KP_MKOUT2    		31
#define HTCLEO_GPIO_KP_MPIN0     		42
#define HTCLEO_GPIO_KP_MPIN1     		41
#define HTCLEO_GPIO_KP_MPIN2     		40
#define HTCLEO_GPIO_KP_LED	 		48

/* Bluetooth */
#define HTCLEO_GPIO_BT_UART1_RTS    		43
#define HTCLEO_GPIO_BT_UART1_CTS    		44
#define HTCLEO_GPIO_BT_UART1_RX     		45
#define HTCLEO_GPIO_BT_UART1_TX     		46
#define HTCLEO_GPIO_BT_RESET_N      		146
#define HTCLEO_GPIO_BT_SHUTDOWN_N   		128
#define HTCLEO_GPIO_BT_HOST_WAKE    		37
#define HTCLEO_GPIO_BT_CHIP_WAKE    		57

/* Battery */
#define HTCLEO_GPIO_BATTERY_CHARGER_ENABLE	22
#define HTCLEO_GPIO_BATTERY_CHARGER_CURRENT	16
#define HTCLEO_GPIO_BATTERY_OVER_CHG		147
#define HTCLEO_GPIO_POWER_USB     		109
#define HTCLEO_GPIO_USBPHY_3V3_ENABLE		104

/* Touchscreen */
#define HTCLEO_GPIO_TS_POWER			160
#define HTCLEO_GPIO_TS_IRQ      		92
#define HTCLEO_GPIO_TS_SEL      		108
#define HTCLEO_GPIO_TS_MULT			82
#define HTCLEO_GPIO_H2W_CLK			27

#define HTCLEO_GPIO_LED_3V3_EN			85


/* Compass */
#define HTCLEO_GPIO_COMPASS_INT_N		39
#define HTCLEO_GPIO_COMPASS_RST_N		107
#define HTCLEO_PROJECT_NAME          		"htcleo"
#define HTCLEO_LAYOUTS { 			   \
	{ {  0,  1, 0}, { -1,  0,  0}, { 0, 0,  1} }, \
	{ {  0, -1, 0}, {  1,  0,  0}, { 0, 0, -1} }, \
	{ { -1,  0, 0}, {  0, -1,  0}, { 0, 0,  1} }, \
	{ { -1,  0, 0}, {  0,  0, -1}, { 0, 1,  0} }  \
}

/* Display */
#define HTCLEO_GPIO_LCM_POWER			88
#define HTCLEO_GPIO_LCM_RESET			29
#define HTCLEO_LCD_R1				(114)
#define HTCLEO_LCD_R2				(115)
#define HTCLEO_LCD_R3				(116)
#define HTCLEO_LCD_R4				(117)
#define HTCLEO_LCD_R5				(118)
#define HTCLEO_LCD_G0				(121)
#define HTCLEO_LCD_G1				(122)
#define HTCLEO_LCD_G2				(123)
#define HTCLEO_LCD_G3				(124)
#define HTCLEO_LCD_G4				(125)
#define HTCLEO_LCD_G5				(126)
#define HTCLEO_LCD_B1				(130)
#define HTCLEO_LCD_B2				(131)
#define HTCLEO_LCD_B3				(132)
#define HTCLEO_LCD_B4				(133)
#define HTCLEO_LCD_B5				(134)
#define HTCLEO_LCD_PCLK	       			(135)
#define HTCLEO_LCD_VSYNC	      		(136)
#define HTCLEO_LCD_HSYNC			(137)
#define HTCLEO_LCD_DE				(138)
	
/* Voltage driver */
#define HTCLEO_TPS65023_MIN_UV_MV		(800)
#define HTCLEO_TPS65023_MAX_UV_MV		(1350)
#define HTCLEO_TPS65023_UV_STEP_MV		(25)

/* LEDS */
#define LED_RGB	(1 << 0)
struct microp_led_config {
        const char *name; 
        uint32_t type;    
        uint8_t init_value;
        uint8_t fade_time; 
        uint16_t led_pin;  
        uint8_t mask_w[3]; 
};
  
struct microp_led_platform_data {
        struct microp_led_config *led_config;
        int num_leds;
};




int htcleo_pm_set_vreg(int enable, unsigned id);
int __init htcleo_init_panel(void);
int htcleo_is_nand_boot(void);
unsigned htcleo_get_vbus_state(void);
void config_camera_on_gpios(void);
void config_camera_off_gpios(void);
int is_valid_mac_address(char *mac);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_H */
