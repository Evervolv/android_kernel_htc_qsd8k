/* arch/arm/mach-msm/include/mach/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <mach/mmc.h>
#include <asm/setup.h>

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long mpll_khz;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
};

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
	uint32_t camifpadphy;
	uint32_t camifpadsz;
	uint32_t csiphy;
	uint32_t csisz;
	uint32_t csiirq;
};

struct msm_camera_device_platform_data {
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	struct msm_camera_io_ext ioext;
};
enum msm_camera_csi_data_format {
	CSI_8BIT,
	CSI_10BIT,
	CSI_12BIT,
};
struct msm_camera_csi_params {
	enum msm_camera_csi_data_format data_format;
	uint8_t lane_cnt;
	uint8_t lane_assign;
	uint8_t settle_cnt;
	uint8_t dpcm_scheme;
};

struct msm_camera_legacy_device_platform_data {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
	struct msm_camsensor_device_platform_data *sensor_info;
};

#define MSM_CAMERA_FLASH_NONE 0
#define MSM_CAMERA_FLASH_LED  1
#define MSM_CAMERA_FLASH_SRC_PMIC (0x00000001<<0)
#define MSM_CAMERA_FLASH_SRC_PWM  (0x00000001<<1)

struct msm_camera_sensor_flash_pmic {
	uint32_t low_current;
	uint32_t high_current;
};

struct msm_camera_sensor_flash_pwm {
	uint32_t freq;
	uint32_t max_load;
	uint32_t low_load;
	uint32_t high_load;
	uint32_t channel;
};

struct msm_camera_sensor_flash_src {
	int flash_sr_type;

	union {
		struct msm_camera_sensor_flash_pmic pmic_src;
		struct msm_camera_sensor_flash_pwm pwm_src;
	} _fsrc;
};

struct msm_camera_sensor_flash_data {
	int flash_type;
	struct msm_camera_sensor_flash_src *flash_src;
};

struct camera_flash_cfg {
	int num_flash_levels;
	int (*camera_flash)(int level);
	uint16_t low_temp_limit;
	uint16_t low_cap_limit;
	uint8_t postpone_led_mode;
};

enum msm_camera_source{
	MAIN_SOURCE,
	SECOND_SOURCE,
};

struct msm_camera_sensor_info {
	const char *sensor_name;
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void(*camera_clk_switch)(void);
	/*power*/
	char *camera_analog_pwd;
	char *camera_io_pwd;
	char *camera_vcm_pwd;
	char *camera_digital_pwd;
	int analog_pwd1_gpio;
	int (*camera_power_on)(void);
	int (*camera_power_off)(void);
	void(*camera_set_source)(enum msm_camera_source);
	enum msm_camera_source(*camera_get_source)(void);
	int (*camera_main_get_probe)(void);
	void (*camera_main_set_probe)(int);
	int mclk;
	int flash_type; /* for back support */
	uint8_t led_high_enabled;
	int need_suspend;
	struct msm_camera_device_platform_data *pdata;
	struct resource *resource;
	uint8_t num_resources;
	uint32_t waked_up;
	wait_queue_head_t event_wait;
	uint32_t kpi_sensor_start;
	uint32_t kpi_sensor_end;
	struct camera_flash_cfg* flash_cfg;
	int csi_if;
	struct msm_camera_csi_params csi_params;
	int sensor_lc_disable; /* for sensor lens correction support */
	uint8_t (*preview_skip_frame)(void);
};
struct clk;

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

extern struct sys_timer msm_timer;

/* common init routines for use by arch/arm/mach-msm/board-*.c */
void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);
int __init msm_add_sdcc(unsigned int controller,
			struct msm_mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K)
void msm_hsusb_set_vbus_state(int online);
/* START: add USB connected notify function */
struct t_usb_status_notifier{
	struct list_head notifier_link;
	const char *name;
	void (*func)(int online);
};
	int usb_register_notifier(struct t_usb_status_notifier *);
	static LIST_HEAD(g_lh_usb_notifier_list);
/* END: add USB connected notify function */
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

char *board_serialno(void);
int __init parse_tag_skuid(const struct tag *tags);
int __init parse_tag_engineerid(const struct tag *tags);
int __init parse_tag_memsize(const struct tag *tags);
int board_mfg_mode(void);
void __init msm_snddev_init(void);
void msm_snddev_poweramp_on(void);
void msm_snddev_poweramp_off(void);
void msm_snddev_hsed_pamp_on(void);
void msm_snddev_hsed_pamp_off(void);
void msm_snddev_tx_route_config(void);
void msm_snddev_tx_route_deconfig(void);

extern int emmc_partition_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data);

#endif
