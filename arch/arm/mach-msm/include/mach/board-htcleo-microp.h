/*
 * Definitions for BMA150 G-sensor chip.
 */
#ifndef HTCLEO_MICROP_H
#define HTCLEO_MICROP_H

#include <linux/ioctl.h>

static struct wake_lock microp_i2c_wakelock;

static struct i2c_client *private_microp_client;

struct microp_i2c_platform_data {
	struct platform_device *microp_devices;
	int			num_devices;
	uint32_t		gpio_reset;
	void 			*dev_id;
	uint8_t			microp_mic_status;
	uint8_t			function_node[20];
	uint32_t		cmd_diff;
};

struct microp_int_pin {
	uint16_t int_gsensor;
	uint16_t int_lsensor;
	uint16_t int_reset;
	uint16_t int_simcard;
	uint16_t int_hpin;
	uint16_t int_remotekey;
};

struct microp_i2c_work {
	struct work_struct work;
	struct i2c_client *client;
	int (*intr_debounce)(uint8_t *pin_status);
	void (*intr_function)(uint8_t *pin_status);
};

struct microp_i2c_client_data {
	struct mutex microp_i2c_rw_mutex;
	uint8_t gpio_reset;
	uint16_t version;
	struct microp_i2c_work work;
	struct delayed_work hpin_debounce_work;
	struct early_suspend early_suspend;
	uint8_t enable_early_suspend;
	uint8_t enable_reset_button;
	int microp_is_suspend;
	uint32_t microp_als_kadc;
	struct hrtimer gen_irq_timer;
	uint16_t intr_status;
};

#define MICROP_I2C_NAME "htcleo-microp"

#define MICROP_LSENSOR_ADC_CHAN				6
#define MICROP_REMOTE_KEY_ADC_CHAN			7

#define MICROP_I2C_WCMD_MISC			0x20
#define MICROP_I2C_WCMD_SPI_EN			0x21
#define MICROP_I2C_WCMD_LCM_BL_MANU_CTL		0x22
#define MICROP_I2C_WCMD_AUTO_BL_CTL		0x23
#define MICROP_I2C_RCMD_SPI_BL_STATUS		0x24
#define MICROP_I2C_WCMD_LED_PWM			0x25
#define MICROP_I2C_WCMD_BL_EN			0x26
#define MICROP_I2C_RCMD_VERSION			0x30
#define MICROP_I2C_RCMD_LSENSOR			0x33
#define MICROP_I2C_WCMD_ADC_TABLE		0x42
#define MICROP_I2C_WCMD_LED_CTRL		0x51
#define MICROP_I2C_WCMD_LED_MODE		0x53
#define MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME	0x54
#define MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME	0x55
#define MICROP_I2C_RCMD_LED_REMAIN_TIME		0x56
#define MICROP_I2C_RCMD_BLUE_LED_REMAIN_TIME	0x57
#define MICROP_I2C_RCMD_LED_STATUS		0x58
#define MICROP_I2C_WCMD_JOGBALL_LED_MODE	0x5A
#define MICROP_I2C_WCMD_JOGBALL_LED_PWM_SET	0x5C
#define MICROP_I2C_WCMD_READ_ADC_VALUE_REQ	0x60
#define MICROP_I2C_RCMD_ADC_VALUE		0x62
#define MICROP_I2C_WCMD_REMOTEKEY_TABLE		0x63
#define MICROP_I2C_WCMD_ADC_REQ			0x64
#define MICROP_I2C_WCMD_LCM_BURST		0x6A
#define MICROP_I2C_WCMD_LCM_BURST_EN		0x6B
#define MICROP_I2C_WCMD_LCM_REGISTER		0x70
#define MICROP_I2C_WCMD_GSENSOR_REG		0x73
#define MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ	0x74
#define MICROP_I2C_RCMD_GSENSOR_REG_DATA	0x75
#define MICROP_I2C_WCMD_GSENSOR_DATA_REQ	0x76
#define MICROP_I2C_RCMD_GSENSOR_X_DATA		0x77
#define MICROP_I2C_RCMD_GSENSOR_Y_DATA		0x78
#define MICROP_I2C_RCMD_GSENSOR_Z_DATA		0x79
#define MICROP_I2C_RCMD_GSENSOR_DATA		0x7A
#define MICROP_I2C_WCMD_OJ_REG			0x7B
#define MICROP_I2C_WCMD_OJ_REG_DATA_REQ		0x7C
#define MICROP_I2C_RCMD_OJ_REG_DATA		0x7D
#define MICROP_I2C_WCMD_OJ_POS_DATA_REQ		0x7E
#define MICROP_I2C_RCMD_OJ_POS_DATA		0x7F
#define MICROP_I2C_WCMD_GPI_INT_CTL_EN		0x80
#define MICROP_I2C_WCMD_GPI_INT_CTL_DIS		0x81
#define MICROP_I2C_RCMD_GPI_INT_STATUS		0x82
#define MICROP_I2C_RCMD_GPIO_STATUS		0x83
#define MICROP_I2C_WCMD_GPI_INT_STATUS_CLR	0x84
#define MICROP_I2C_RCMD_GPI_INT_SETTING		0x85
#define MICROP_I2C_RCMD_REMOTE_KEYCODE		0x87
#define MICROP_I2C_WCMD_REMOTE_KEY_DEBN_TIME	0x88
#define MICROP_I2C_WCMD_REMOTE_PLUG_DEBN_TIME	0x89
#define MICROP_I2C_WCMD_SIMCARD_DEBN_TIME	0x8A
#define MICROP_I2C_WCMD_GPO_LED_STATUS_EN	0x90
#define MICROP_I2C_WCMD_GPO_LED_STATUS_DIS	0x91
#define MICROP_I2C_RCMD_GPO_LED_STATUS		0x92
#define MICROP_I2C_WCMD_OJ_INT_STATUS		0xA8
#define MICROP_I2C_RCMD_MOBEAM_STATUS		0xB1
#define MICROP_I2C_WCMD_MOBEAM_DL		0xB2
#define MICROP_I2C_WCMD_MOBEAM_SEND		0xB3


#define IRQ_GSENSOR				(1<<10)
#define IRQ_LSENSOR  				(1<<9)
#define IRQ_REMOTEKEY				(1<<7)
#define IRQ_HEADSETIN				(1<<2)
#define IRQ_PROXIMITY   			(1<<1)
#define IRQ_SDCARD	    			(1<<0)

#define READ_GPI_STATE_HPIN			(1<<2)
#define READ_GPI_STATE_SDCARD			(1<<0)

#define PS_PWR_ON				(1 << 0)
#define LS_PWR_ON				(1 << 1)

#define SPI_GSENSOR				(1 << 0)

#define CAPELLA_CM3602_IOCTL_MAGIC 'c'
#define CAPELLA_CM3602_IOCTL_GET_ENABLED \
		_IOR(CAPELLA_CM3602_IOCTL_MAGIC, 1, int *)
#define CAPELLA_CM3602_IOCTL_ENABLE \
		_IOW(CAPELLA_CM3602_IOCTL_MAGIC, 2, int *)

int microp_i2c_read(uint8_t addr, uint8_t *data, int length);
int microp_i2c_write(uint8_t addr, uint8_t *data, int length);
int microp_spi_vote_enable(int spi_device, uint8_t enable);
int capella_cm3602_power(int pwr_device, uint8_t enable);
int microp_read_gpo_status(uint16_t *status);
int microp_gpo_enable(uint16_t gpo_mask);
int microp_gpo_disable(uint16_t gpo_mask);

#ifdef CONFIG_HAS_EARLYSUSPEND
void microp_early_suspend(struct early_suspend *h);
void microp_early_resume(struct early_suspend *h);
#endif // CONFIG_HAS_EARLYSUSPEND


#endif
