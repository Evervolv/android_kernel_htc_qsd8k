/* arch/arm/mach-msm/board-bravo.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
 * Copyright (C) 2010 Kali- <kalimero@ngi.it>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
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

#ifndef _LINUX_BOARD_BRAVO_MICROP_COMMON_H
#define _LINUX_BOARD_BRAVO_MICROP_COMMON_H

#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#define MICROP_I2C_NAME "bravo-microp"

#define MICROP_LSENSOR_ADC_CHAN				6
#define MICROP_REMOTE_KEY_ADC_CHAN			7

#define MICROP_I2C_WCMD_MISC				0x20
#define MICROP_I2C_WCMD_SPI_EN				0x21
#define MICROP_I2C_WCMD_AUTO_BL_CTL			0x23
#define MICROP_I2C_RCMD_SPI_BL_STATUS			0x24
#define MICROP_I2C_WCMD_BUTTONS_LED_CTRL		0x25
#define MICROP_I2C_RCMD_VERSION				0x30
#define MICROP_I2C_WCMD_ADC_TABLE			0x42
#define MICROP_I2C_WCMD_LED_MODE			0x53
#define MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME		0x54
#define MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME		0x55
#define MICROP_I2C_RCMD_BLUE_LED_REMAIN_TIME		0x57
#define MICROP_I2C_WCMD_READ_ADC_VALUE_REQ		0x60
#define MICROP_I2C_RCMD_ADC_VALUE			0x62
#define MICROP_I2C_WCMD_REMOTEKEY_TABLE			0x63
#define MICROP_I2C_WCMD_LCM_REGISTER			0x70
#define MICROP_I2C_WCMD_GSENSOR_REG			0x73
#define MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ		0x74
#define MICROP_I2C_RCMD_GSENSOR_REG_DATA		0x75
#define MICROP_I2C_WCMD_GSENSOR_DATA_REQ		0x76
#define MICROP_I2C_RCMD_GSENSOR_X_DATA			0x77
#define MICROP_I2C_RCMD_GSENSOR_Y_DATA			0x78
#define MICROP_I2C_RCMD_GSENSOR_Z_DATA			0x79
#define MICROP_I2C_RCMD_GSENSOR_DATA			0x7A
#define MICROP_I2C_WCMD_OJ_REG				0x7B
#define MICROP_I2C_WCMD_OJ_REG_DATA_REQ			0x7C
#define MICROP_I2C_RCMD_OJ_REG_DATA			0x7D
#define MICROP_I2C_WCMD_OJ_POS_DATA_REQ			0x7E
#define MICROP_I2C_RCMD_OJ_POS_DATA			0x7F
#define MICROP_I2C_WCMD_GPI_INT_CTL_EN			0x80
#define MICROP_I2C_WCMD_GPI_INT_CTL_DIS			0x81
#define MICROP_I2C_RCMD_GPI_INT_STATUS			0x82
#define MICROP_I2C_RCMD_GPI_STATUS			0x83
#define MICROP_I2C_WCMD_GPI_INT_STATUS_CLR		0x84
#define MICROP_I2C_RCMD_GPI_INT_SETTING			0x85
#define MICROP_I2C_RCMD_REMOTE_KEYCODE			0x87
#define MICROP_I2C_WCMD_REMOTE_KEY_DEBN_TIME		0x88
#define MICROP_I2C_WCMD_REMOTE_PLUG_DEBN_TIME		0x89
#define MICROP_I2C_WCMD_SIMCARD_DEBN_TIME		0x8A
#define MICROP_I2C_WCMD_GPO_LED_STATUS_EN		0x90
#define MICROP_I2C_WCMD_GPO_LED_STATUS_DIS		0x91
#define MICROP_I2C_WCMD_OJ_INT_STATUS			0xA8

#define IRQ_OJ						(1<<12)
#define IRQ_GSENSOR					(1<<10)
#define IRQ_LSENSOR					(1<<9)
#define IRQ_REMOTEKEY					(1<<7)
#define IRQ_HEADSETIN					(1<<2)
#define IRQ_SDCARD					(1<<0)

#define SPI_GSENSOR					(1 << 0)
#define SPI_LCM						(1 << 1)
#define SPI_OJ						(1 << 2)

#define MICROP_FUNCTION_LSENSOR				1
#define MICROP_FUNCTION_REMOTEKEY			2
#define MICROP_FUNCTION_LCD_BL				3
#define MICROP_FUNCTION_RMK_VALUE			4
#define MICROP_FUNCTION_INTR				11
#define MICROP_FUNCTION_GSENSOR				12
#define MICROP_FUNCTION_LED				13
#define MICROP_FUNCTION_HPIN				14
#define MICROP_FUNCTION_RESET_INT			15
#define MICROP_FUNCTION_SIM_CARD			16
#define MICROP_FUNCTION_SDCARD				17
#define MICROP_FUNCTION_OJ				18
#define MICROP_FUNCTION_P				19

#define LS_PWR_ON					(1 << 0)
#define ALS_CALIBRATED					0x6DA5
#define ATAG_ALS					0x5441001b

/* I2C functions for drivers */
int microp_i2c_read(uint8_t addr, uint8_t *data, int length);
int microp_i2c_write(uint8_t addr, uint8_t *data, int length);
int microp_read_adc(uint8_t channel, uint16_t *value);
int microp_spi_vote_enable(int spi_device, uint8_t enable);
int microp_write_interrupt(struct i2c_client *client,
			uint16_t interrupt, uint8_t enable);
struct i2c_client *get_microp_client(void);

struct microp_function_config {
	const char      *name;
	uint8_t         category;
	uint8_t         init_value;
	uint8_t         channel;
	uint8_t         fade_time;
	uint32_t        sub_categ;
	uint16_t        levels[10];
	uint16_t        dutys[10];
	uint16_t        int_pin;
	uint16_t        golden_adc;
	uint8_t         mask_r[3];
	uint8_t         mask_w[3];
	uint32_t        ls_gpio_on;
	int (*ls_power)(int, uint8_t);
};

struct microp_i2c_platform_data {
	struct microp_function_config   *microp_function;
	struct platform_device *microp_devices;
	int                     num_devices;
	int                     num_functions;
	uint32_t                gpio_reset;
	uint32_t                microp_ls_on;
	void                    *dev_id;
	uint8_t                 microp_mic_status;
	uint8_t                 function_node[20];
	uint32_t                cmd_diff;
	uint32_t                spi_devices;
	uint32_t                spi_devices_init;
};

struct lightsensor_platform_data{
	struct i2c_client *client;
	struct microp_function_config   *config;
	int irq;
	int old_intr_cmd;
};

struct microp_ops {
        int (*init_microp_func)(struct i2c_client *);
        int (*als_pwr_enable)(int pwr_device, uint8_t en);
        int (*als_intr_enable)(struct i2c_client *,
                        uint32_t als_func, uint8_t en);
        void (*als_level_change)(struct i2c_client *, uint8_t *data);
        void (*headset_enable)(int en);
        void (*spi_enable)(int en);
};

#endif /* _LINUX_BOARD_BRAVO_MICROP_COMMON_H */
