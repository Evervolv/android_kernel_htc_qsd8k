/*
 * include/linux/synaptics_i2c_rmi.h - platform data structure for f75375s sensor
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef _LINUX_SYNAPTICS_I2C_RMI_H
#define _LINUX_SYNAPTICS_I2C_RMI_H

#define SYNAPTICS_I2C_RMI_NAME "synaptics-rmi-ts"
#define SYNAPTICS_T1007_NAME "synaptics-t1007"
#define SYNAPTICS_T1021_NAME "synaptics-t1021"
#define SYNAPTICS_3K_NAME "synaptics-3k"
#define SYNAPTICS_3K_INCELL_NAME "synaptics-3k-incell"
#define SYNAPTICS_3200_NAME "synaptics-3200"


#define SYN_CONFIG_SIZE 32 * 16
#define SYN_MAX_PAGE 3
#define SYN_BL_PAGE 1
#define SYN_F01DATA_BASEADDR 0x0013
#define SYN_PROCESS_ERR -1

#define SYN_AND_REPORT_TYPE_A		0
#define	SYN_AND_REPORT_TYPE_B		1
#define SYN_AND_REPORT_TYPE_HTC		2

#define TAP_DX_OUTER		0
#define TAP_DY_OUTER		1
#define TAP_TIMEOUT		2
#define TAP_DX_INTER		3
#define TAP_DY_INTER		4

#define CUS_REG_SIZE		4
#define CUS_REG_BASE		0
#define CUS_BALLISTICS_CTRL	1
#define CUS_LAND_CTRL		2
#define CUS_LIFT_CTRL		3

#define SENSOR_ID_CHECKING_EN	1 << 16

enum {
	SYNAPTICS_FLIP_X = 1UL << 0,
	SYNAPTICS_FLIP_Y = 1UL << 1,
	SYNAPTICS_SWAP_XY = 1UL << 2,
	SYNAPTICS_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

enum {
	FINGER_1_REPORT = 1 << 0,
	FINGER_2_REPORT = 1 << 1,
};

struct synaptics_virtual_key {
	int keycode;
	int range_min;
	int range_max;
};

struct synaptics_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	struct synaptics_virtual_key *virtual_key;
	uint8_t virtual_key_num;
	uint8_t sensitivity;
	uint8_t finger_support;
	uint32_t gap_area;
	uint32_t key_area;
	uint32_t flags;
	unsigned long irqflags;
	uint32_t inactive_left; /* 0x10000 = screen width */
	uint32_t inactive_right; /* 0x10000 = screen width */
	uint32_t inactive_top; /* 0x10000 = screen height */
	uint32_t inactive_bottom; /* 0x10000 = screen height */
	uint32_t snap_left_on; /* 0x10000 = screen width */
	uint32_t snap_left_off; /* 0x10000 = screen width */
	uint32_t snap_right_on; /* 0x10000 = screen width */
	uint32_t snap_right_off; /* 0x10000 = screen width */
	uint32_t snap_top_on; /* 0x10000 = screen height */
	uint32_t snap_top_off; /* 0x10000 = screen height */
	uint32_t snap_bottom_on; /* 0x10000 = screen height */
	uint32_t snap_bottom_off; /* 0x10000 = screen height */
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int fuzz_p;
	int fuzz_w;
	uint32_t display_width;
	uint32_t display_height;
	int8_t sensitivity_adjust;
	uint32_t dup_threshold;
	uint32_t margin_inactive_pixel[4];
	uint16_t filter_level[4];
	uint8_t reduce_report_level[5];
	uint8_t noise_information;
	uint8_t jumpfq_enable;
	uint8_t cable_support;
	uint8_t config[SYN_CONFIG_SIZE];
	int gpio_irq;
	int gpio_reset;
	uint8_t default_config;
	uint8_t report_type;
	uint8_t large_obj_check;
	uint16_t tw_pin_mask;
	uint32_t sensor_id;
	uint32_t packrat_number;
	uint8_t support_htc_event;
	uint8_t mfg_flag;
	uint8_t customer_register[CUS_REG_SIZE];
};

struct page_description {
	uint8_t addr;
	uint8_t value;
};

struct syn_finger_data {
	int x;
	int y;
	int w;
	int z;
};

struct function_t {
	uint8_t function_type;
	uint8_t interrupt_source;
	uint16_t data_base;
	uint16_t control_base;
	uint16_t command_base;
	uint16_t query_base;
};
enum {
	QUERY_BASE,
	COMMAND_BASE,
	CONTROL_BASE,
	DATA_BASE,
	INTR_SOURCE,
	FUNCTION
};

#endif /* _LINUX_SYNAPTICS_I2C_RMI_H */
