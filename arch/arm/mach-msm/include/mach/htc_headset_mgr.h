/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_mgr.h
 *
 *  HTC headset manager driver.
 *
 *  Copyright (C) 2010 HTC, Inc.
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

#ifndef HTC_HEADSET_MGR_H
#define HTC_HEADSET_MGR_H

#include <mach/msm_rpcrouter.h>

#include <linux/switch.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#define SYS_MSG(fmt, arg...) \
	printk(KERN_INFO "[" DRIVER_NAME "] (%s) " fmt "\n", __func__, ## arg)
#if 0
#define DBG_MSG(fmt, arg...) \
	printk(KERN_INFO "##### [" DRIVER_NAME "] (%s) " fmt "\n", \
	       __func__, ## arg)
#else
#define DBG_MSG(fmt, arg...) {}
#endif

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_##_name = \
	__ATTR(flag, _mode, _show, _store)

#define DRIVER_HS_MGR_RPC_SERVER	(1 << 0)

#define HS_DEF_MIC_ADC_10_BIT		200
#define HS_DEF_MIC_ADC_16_BIT		14894 /* (0.5 / 2.2) * (2 ^ 16) */

#define HS_DELAY_ZERO			0
#define HS_DELAY_MIC_BIAS		200
#define HS_DELAY_MIC_DETECT		500
#define HS_DELAY_INSERT			500
#define HS_DELAY_REMOVE			200
#define HS_DELAY_BUTTON			500

#define HS_JIFFIES_ZERO			msecs_to_jiffies(HS_DELAY_ZERO)
#define HS_JIFFIES_MIC_BIAS		msecs_to_jiffies(HS_DELAY_MIC_BIAS)
#define HS_JIFFIES_MIC_DETECT		msecs_to_jiffies(HS_DELAY_MIC_DETECT)
#define HS_JIFFIES_INSERT		msecs_to_jiffies(HS_DELAY_INSERT)
#define HS_JIFFIES_REMOVE		msecs_to_jiffies(HS_DELAY_REMOVE)
#define HS_JIFFIES_BUTTON		msecs_to_jiffies(HS_DELAY_BUTTON)

/* Definitions for Headset RPC Server */
#define HS_RPC_SERVER_PROG		0x30100004
#define HS_RPC_SERVER_VERS		0x00000000
#define HS_RPC_SERVER_PROC_NULL		0
#define HS_RPC_SERVER_PROC_KEY		1

/* Definitions for Headset RPC Client */
#define HS_RPC_CLIENT_PROG		0x30100005
#define HS_RPC_CLIENT_VERS		0x00000000
#define HS_RPC_CLIENT_PROC_NULL		0
#define HS_RPC_CLIENT_PROC_ADC		1

#define HS_MGR_KEYCODE_END	KEY_END			/* 107 */
#define HS_MGR_KEYCODE_MUTE	KEY_MUTE		/* 113 */
#define HS_MGR_KEYCODE_VOLDOWN	KEY_VOLUMEDOWN		/* 114 */
#define HS_MGR_KEYCODE_VOLUP	KEY_VOLUMEUP		/* 115 */
#define HS_MGR_KEYCODE_FORWARD	KEY_NEXTSONG		/* 163 */
#define HS_MGR_KEYCODE_PLAY	KEY_PLAYPAUSE		/* 164 */
#define HS_MGR_KEYCODE_BACKWARD	KEY_PREVIOUSSONG	/* 165 */
#define HS_MGR_KEYCODE_MEDIA	KEY_MEDIA		/* 226 */
#define HS_MGR_KEYCODE_SEND	KEY_SEND		/* 231 */

#define HEADSET_NO_MIC		0
#define HEADSET_MIC		1
#define HEADSET_METRICO		2

#define HTC_35MM_UNPLUG 0
#define HTC_35MM_NO_MIC 1
#define HTC_35MM_MIC 2

enum {
	HEADSET_REG_REMOTE_ADC,
	HEADSET_REG_RPC_KEY,
	HEADSET_REG_MIC_STATUS,
	HEADSET_REG_MIC_BIAS,
	HEADSET_REG_MIC_SELECT,
	HEADSET_REG_KEY_INT_ENABLE,
	HEADSET_REG_KEY_ENABLE,
};

enum {
	HS_MGR_KEY_INVALID	= -1,
	HS_MGR_KEY_NONE		= 0,
	HS_MGR_KEY_PLAY		= 1,
	HS_MGR_KEY_BACKWARD	= 2,
	HS_MGR_KEY_FORWARD	= 3,
};

struct hs_rpc_server_args_key {
	uint32_t adc;
};

struct hs_rpc_client_req_adc {
	struct rpc_request_hdr hdr;
};

struct hs_rpc_client_rep_adc {
	struct rpc_reply_hdr hdr;
	uint32_t adc;
};

struct headset_notifier {
	int id;
	void *func;
};

struct hs_notifier_func {
	int (*remote_adc)(int *);
	void (*rpc_key)(int);
	int (*mic_status)(void);
	int (*mic_bias_enable)(int);
	void (*mic_select)(int);
	int (*key_int_enable)(int);
	void (*key_enable)(int);
};

struct htc_headset_mgr_platform_data {
	unsigned int driver_flag;

	int cable_in1;
	int cable_in2;
	int h2w_clk;
	int h2w_data;
	int debug_uart;
	int headset_mic_35mm;

	void (*h2w_power)(int);
	void (*config)(int);
	void (*set_dat)(int);
	void (*set_clk)(int);
	void (*set_dat_dir)(int);
	void (*set_clk_dir)(int);
	int (*get_dat)(void);
	int (*get_clk)(void);
};

#define BIT_HEADSET		(1 << 0)
#define BIT_HEADSET_NO_MIC	(1 << 1)
#define BIT_TTY_FULL		(1 << 2)
#define BIT_FM_HEADSET		(1 << 3)
#define BIT_FM_SPEAKER		(1 << 4)
#define BIT_TTY_VCO		(1 << 5)
#define BIT_TTY_HCO		(1 << 6)
#define BIT_35MM_HEADSET	(1 << 7)
#define BIT_TV_OUT		(1 << 8)
#define BIT_USB_CRADLE		(1 << 9)
#define BIT_TV_OUT_AUDIO	(1 << 10)
#define BIT_HDMI_CABLE		(1 << 11)
#define BIT_HDMI_AUDIO		(1 << 12)
#define BIT_USB_HEADSET		(1 << 13)

enum {
	STATUS_DISCONNECTED		= 0,
	STATUS_CONNECTED_ENABLED	= 1,
	STATUS_CONNECTED_DISABLED	= 2,
};

enum {
	H2W_GPIO	= 0,
	H2W_UART1	= 1,
	H2W_UART3	= 2,
	H2W_BT		= 3
};

enum {
	NO_DEVICE		= 0,
	HTC_HEADSET		= 1,
	NORMAL_HEARPHONE	= 2,
	H2W_DEVICE		= 3,
	USB_CRADLE		= 4,
	UART_DEBUG		= 5,
	H2W_TVOUT		= 6,
	USB_HEADSET		= 7,
};

#define RESEND_DELAY		(3)	/* ms */
#define MAX_ACK_RESEND_TIMES	(6)	/* follow spec */
#define MAX_HOST_RESEND_TIMES	(3)	/* follow spec */
#define MAX_HYGEIA_RESEND_TIMES	(5)

#define H2W_ASCR_DEVICE_INI	(0x01)
#define H2W_ASCR_ACT_EN		(0x02)
#define H2W_ASCR_PHONE_IN	(0x04)
#define H2W_ASCR_RESET		(0x08)
#define H2W_ASCR_AUDIO_IN	(0x10)

#define H2W_LED_OFF		(0x0)
#define H2W_LED_BKL		(0x1)
#define H2W_LED_MTL		(0x2)

#define H2W_PhoneIn		(0x01)
#define H2W_MuteLed		(0x02)

typedef enum {
	/* === system group 0x0000~0x00FF === */
	/* (R) Accessory type register */
	H2W_SYSTEM		= 0x0000,
	/* (R) Maximum group address */
	H2W_MAX_GP_ADD		= 0x0001,
	/* (R/W) Accessory system control register0 */
	H2W_ASCR0		= 0x0002,

	/* === key group 0x0100~0x01FF === */
	/* (R) Key group maximum sub address */
	H2W_KEY_MAXADD		= 0x0100,
	/* (R) ASCII key press down flag */
	H2W_ASCII_DOWN		= 0x0101,
	/* (R) ASCII key release up flag */
	H2W_ASCII_UP		= 0x0102,
	/* (R) Function key status flag */
	H2W_FNKEY_UPDOWN	= 0x0103,
	/* (R/W) Key device status */
	H2W_KD_STATUS		= 0x0104,

	/* === led group 0x0200~0x02FF === */
	/* (R) LED group maximum sub address */
	H2W_LED_MAXADD		= 0x0200,
	/* (R/W) LED control register0 */
	H2W_LEDCT0		= 0x0201,

	/* === crdl group 0x0300~0x03FF === */
	/* (R) Cardle group maximum sub address */
	H2W_CRDL_MAXADD		= 0x0300,
	/* (R/W) Cardle group function control register0 */
	H2W_CRDLCT0		= 0x0301,

	/* === car kit group 0x0400~0x04FF === */
	H2W_CARKIT_MAXADD	= 0x0400,

	/* === usb host group 0x0500~0x05FF === */
	H2W_USBHOST_MAXADD	= 0x0500,

	/* === medical group 0x0600~0x06FF === */
	H2W_MED_MAXADD		= 0x0600,
	H2W_MED_CONTROL		= 0x0601,
	H2W_MED_IN_DATA		= 0x0602,
} H2W_ADDR;

typedef struct H2W_INFO {
	/* system group */
	unsigned char CLK_SP;
	int SLEEP_PR;
	unsigned char HW_REV;
	int AUDIO_DEVICE;
	unsigned char ACC_CLASS;
	unsigned char MAX_GP_ADD;

	/* key group */
	int KEY_MAXADD;
	int ASCII_DOWN;
	int ASCII_UP;
	int FNKEY_UPDOWN;
	int KD_STATUS;

	/* led group */
	int LED_MAXADD;
	int LEDCT0;

	/* medical group */
	int MED_MAXADD;
	unsigned char AP_ID;
	unsigned char AP_EN;
	unsigned char DATA_EN;
} H2W_INFO;

typedef enum {
	H2W_500KHz	= 1,
	H2W_250KHz	= 2,
	H2W_166KHz	= 3,
	H2W_125KHz	= 4,
	H2W_100KHz	= 5,
	H2W_83KHz	= 6,
	H2W_71KHz	= 7,
	H2W_62KHz	= 8,
	H2W_55KHz	= 9,
	H2W_50KHz	= 10,
} H2W_SPEED;

struct h2w_info {
	unsigned int driver_flag;

	unsigned long hpin_jiffies;

	struct class *htc_accessory_class;
	struct device *tty_dev;
	struct device *fm_dev;
	struct device *mic_dev;
	struct device *mute_dev;
	struct device *phonein_dev;
	struct mutex mutex_lock;
	struct mutex mutex_rc_lock;

	struct switch_dev sdev;
	struct input_dev *input;
	unsigned long insert_jiffies;

	int ignore_btn;
	atomic_t btn_state;

	int tty_enable_flag;
	int fm_flag;
	int mic_switch_flag;
	int rc_flag;

	unsigned int irq;
	unsigned int irq_btn;
	unsigned int irq_btn_35mm;

	int cable_in1;
	int cable_in2;
	int h2w_clk;
	int h2w_data;
	int debug_uart;
	int headset_mic_35mm;

	/* The variables were used by 35mm headset*/
	int key_level_flag;
	int ext_35mm_status;
	int h2w_35mm_status;
	int is_ext_insert;
	int mic_bias_state;
	int metrico_status; /* For HW Metrico lab test */

	/* The variables are used by USB headset */
	int usb_dev_type;
	int usb_dev_status;

	void (*insert_11pin_35mm)(int *);
	void (*remove_11pin_35mm)(void);

	void (*configure) (int);
	int (*get_path) (void);
	void (*h2w_power)(int);
	void (*set_dat)(int);
	void (*set_clk)(int);
	void (*set_dat_dir)(int);
	void (*set_clk_dir)(int);
	int (*get_dat)(void);
	int (*get_clk)(void);

	H2W_INFO h2w_info;
	H2W_SPEED speed;

	struct wake_lock headset_wake_lock;
};

int headset_notifier_register(struct headset_notifier *notifier);

void insert_headset(int);
void remove_headset(void);

void headset_button_event(int is_press, int type);
void button_pressed(int type);
void button_released(int type);

void button_h2w_do_work(struct work_struct *w);
void detect_h2w_do_work(struct work_struct *w);

void headset_ext_detect(int type);

extern int switch_send_event(unsigned int bit, int on);

/* notify the 3.5mm driver of events */
int htc_35mm_remote_notify_ext_headset_irq(int insert);
int htc_35mm_remote_notify_insert_ext_headset(int insert);
int htc_35mm_remote_notify_microp_ready(void);
int htc_35mm_remote_notify_button_status(int key_level);
int htc_35mm_remote_notify_irq_enable(int enable);

void hs_notify_hpin_irq(void);
int hs_hpin_stable(void);

#endif
