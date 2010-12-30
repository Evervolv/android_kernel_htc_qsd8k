/* linux/arch/arm/mach-msm/board-supersonic-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Author: Jay Tu <jay_tu@htc.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
/* #include <mach/pmic.h> */

#include "board-supersonic.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif
extern int panel_type;
enum {
	PANEL_SHARP,
	PANEL_AUO,
};

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
};
static struct vreg *vreg_lcd_2v8;
static struct vreg *vreg_lcd_1v8;

#define REG_WAIT (0xffff)
struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_seq[] = {
	{0xc000, 0x86},
	{0xc001, 0x00},
	{0xc002, 0x86},
	{0xc003, 0x00},
	{0xc100, 0x40},
	{0xc200, 0x02},
	{0xc202, 0x32},
	{0xe000, 0x0e},
	{0xe001, 0x34},
	{0xe002, 0x3F},
	{0xe003, 0x49},
	{0xe004, 0x1D},
	{0xe005, 0x2C},
	{0xe006, 0x5F},
	{0xe007, 0x3A},
	{0xe008, 0x20},
	{0xe009, 0x28},
	{0xe00a, 0x80},
	{0xe00b, 0x13},
	{0xe00c, 0x32},
	{0xe00d, 0x56},
	{0xe00e, 0x79},
	{0xe00f, 0xB8},
	{0xe010, 0x55},
	{0xe011, 0x57},
	{0xe100, 0x0e},
	{0xe101, 0x34},
	{0xe102, 0x3F},
	{0xe103, 0x49},
	{0xe104, 0x1D},
	{0xe105, 0x2C},
	{0xe106, 0x5F},
	{0xe107, 0x3A},
	{0xe108, 0x20},
	{0xe109, 0x28},
	{0xe10a, 0x80},
	{0xe10b, 0x13},
	{0xe10c, 0x32},
	{0xe10d, 0x56},
	{0xe10e, 0x79},
	{0xe10f, 0xB8},
	{0xe110, 0x55},
	{0xe111, 0x57},

	{0xe200, 0x0E},
	{0xe201, 0x34},
	{0xe202, 0x3F},
	{0xe203, 0x49},
	{0xe204, 0x1D},
	{0xe205, 0x2C},
	{0xe206, 0x5F},
	{0xe207, 0x3A},
	{0xe208, 0x20},
	{0xe209, 0x28},
	{0xe20A, 0x80},
	{0xe20B, 0x13},
	{0xe20C, 0x32},
	{0xe20D, 0x56},
	{0xe20E, 0x79},
	{0xe20F, 0xB8},
	{0xe210, 0x55},
	{0xe211, 0x57},

	{0xe300, 0x0E},
	{0xe301, 0x34},
	{0xe302, 0x3F},
	{0xe303, 0x49},
	{0xe304, 0x1D},
	{0xe305, 0x2C},
	{0xe306, 0x5F},
	{0xe307, 0x3A},
	{0xe308, 0x20},
	{0xe309, 0x28},
	{0xe30A, 0x80},
	{0xe30B, 0x13},
	{0xe30C, 0x32},
	{0xe30D, 0x56},
	{0xe30E, 0x79},
	{0xe30F, 0xB8},
	{0xe310, 0x55},
	{0xe311, 0x57},
	{0xe400, 0x0E},
	{0xe401, 0x34},
	{0xe402, 0x3F},
	{0xe403, 0x49},
	{0xe404, 0x1D},
	{0xe405, 0x2C},
	{0xe406, 0x5F},
	{0xe407, 0x3A},
	{0xe408, 0x20},
	{0xe409, 0x28},
	{0xe40A, 0x80},
	{0xe40B, 0x13},
	{0xe40C, 0x32},
	{0xe40D, 0x56},
	{0xe40E, 0x79},
	{0xe40F, 0xB8},
	{0xe410, 0x55},
	{0xe411, 0x57},
	{0xe500, 0x0E},
	{0xe501, 0x34},
	{0xe502, 0x3F},
	{0xe503, 0x49},
	{0xe504, 0x1D},
	{0xe505, 0x2C},
	{0xe506, 0x5F},
	{0xe507, 0x3A},
	{0xe508, 0x20},
	{0xe509, 0x28},
	{0xe50A, 0x80},
	{0xe50B, 0x13},
	{0xe50C, 0x32},
	{0xe50D, 0x56},
	{0xe50E, 0x79},
	{0xe50F, 0xB8},
	{0xe510, 0x55},
	{0xe511, 0x57},

	{0x3a00, 0x05},

	/* cabc */
	{0x4e00, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x03},
	{0x5100, 0xff},
	{0x5301, 0x10},
	{0x6A18, 0xff},
	{0x6A17, 0x01},
	{0xF402, 0x14},

	{0x3500, 0x00},
	{0x1100, 0x0},
	{REG_WAIT, 120},
};

struct s1d_regs {
	unsigned reg;
	unsigned val;
} s1d13775_init_seq[] = {
	{0x001C, 0x1500},
	{0x0020, 0x3043},
	{0x0024, 0x401A},
	{0x0028, 0x031A},
	{0x002C, 0x0001},
	{REG_WAIT, 0x0004}, /* increase delay 1ms -> 4ms */
	{0x0084, 0x0215},
	{0x0088, 0x0038},
	{0x008C, 0x2113},
	{0x002C, 0x0002},
	{REG_WAIT, 0x0004}, /* increase delay 1ms -> 4ms */
	{0x002C, 0x0003},
	{0x0100, 0x3702},
	{0x0104, 0x0180},
	{0x0140, 0x003F},
	{0x0144, 0x00EF},
	{0x0148, 0x0016},
	{0x014C, 0x0005},
	{0x0150, 0x0006},
	{0x0154, 0x032B},
	{0x0158, 0x031F},
	{0x015C, 0x0009},
	{0x0160, 0x0002},
	{0x0164, 0x0003},
	{0x0168, 0x00A2},
	{0x0180, 0x0057},
	{0x0184, 0x00DB},
	{0x0188, 0x00E3},
	{0x018C, 0x0000},
	{0x0190, 0x0000},
	{0x0280, 0x0000},
	{0x0284, 0x0002},
	{0x0288, 0x0000},
	{0x028C, 0x0001},
	{0x0294, 0x0000},
	{0x0400, 0x8000},
	{0x0404, 0x10C8},
	{0x0480, 0x0001},
	{0x0500, 0x0000},
	{0x0504, 0x0011},
	{0x0508, 0x0000},
	{0x0510, 0x0000},
	{0x0518, 0x002E},
	{0x051C, 0x00c7},
	{0x0520, 0x01DF},
	{0x0524, 0x031f},
	{0x0528, 0x0000},
	{0x052C, 0x0000},
	{0x0530, 0x0000},
	{0x0534, 0x0000},

	{0x0604, 0x0108},
	{0x060C, 0x0000},
	{0x0610, 0x00ff},

	{0x0648, 0x0020},
	{0x0800, 0x0000},
	{0x0804, 0x000A},
	{0x0808, 0x0400},
	{0x080C, 0x0400},
	{0x0814, 0x0000},
	{0x081C, 0x0000},
	{0x0824, 0x002E},
	{0x0828, 0x00C7},
	{0x082C, 0x01DF},
	{0x0830, 0x031F},
	{0x0834, 0x0000},
	{0x0838, 0x0000},
	{0x083C, 0x0000},
	{0x0840, 0x0000},
	{0x0844, 0x01DF},
	{0x0848, 0x031F},
	{0x0870, 0x0064},
	{0x0874, 0x0064},
	{0x0878, 0x00C7},
	{0x087C, 0x00C7},
	{0x1410, 0x0004},
	{0x1414, 0x00FF},
	{0x1420, 0x0000},
	{0x1424, 0x0000},
	{0x1428, 0x01DF},
	{0x142C, 0x031F},
	{0x1430, 0xDC00},
	{0x1434, 0x0005},
	{0x1440, 0x0000},
	{0x1444, 0x0000},
	{0x1448, 0x01DF},
	{0x144C, 0x031F},
	{0x1450, 0x0000},
	{0x1454, 0x0000},
	{0x1458, 0x01DF},
	{0x145C, 0x031F},
	{0x1460, 0x0000},
	{0x1464, 0x0000},
	{0x1468, 0x01DF},
	{0x146C, 0x031F},
	{0x1470, 0x0000},
	{0x1474, 0x0000},
	{0x1478, 0x01DF},
	{0x147C, 0x031F},
	{0x14A4, 0x0110},
	{0x14A8, 0xAFC8},
	{0x14AC, 0x0FF0},
	{0x14B0, 0x0202},
	{0x14B4, 0x0080},
	{0x14A0, 0x0002},
	{0x1508, 0x0000},
	{0x150C, 0x0000},
	{0x1510, 0x0000},
	{0x1514, 0x0000},
	{0x1520, 0x0000},
	{0x1524, 0x0000},
	{0x1528, 0x0000},
	{0x152C, 0x0000},
	{0x1530, 0x0000},
	{0x1534, 0x0000},
	{0x1538, 0x0000},
	{0x153C, 0x0000},
	{0x1540, 0x0000},
	{0x1544, 0x0000},
	{0x1548, 0x0000},
	{0x154C, 0x0000},
	{0x1550, 0x0000},
	{0x1554, 0x0000},
	{0x1558, 0x0000},
	{0x1600, 0x0000},
	{0x1604, 0x0020},
	{0x1608, 0x0040},
	{0x160C, 0x0060},
	{0x1610, 0x0080},
	{0x1614, 0x00A0},
	{0x1618, 0x00C0},
	{0x161C, 0x00E0},
	{0x1620, 0x0100},
	{0x1624, 0x0000},
	{0x1628, 0x0020},
	{0x162C, 0x0040},
	{0x1630, 0x0060},
	{0x1634, 0x0080},
	{0x1638, 0x00A0},
	{0x163C, 0x00C0},
	{0x1640, 0x00E0},
	{0x1644, 0x0100},
	{0x1648, 0x0000},
	{0x164C, 0x0020},
	{0x1650, 0x0040},
	{0x1654, 0x0060},
	{0x1658, 0x0080},
	{0x165C, 0x00A0},
	{0x1660, 0x00C0},
	{0x1664, 0x00E0},
	{0x1668, 0x0100},
	{0x1680, 0x0000},
	{0x1684, 0x0000},
	{0x1688, 0x0000},
	{0x168C, 0x0000},
	{0x1694, 0x0000},
	{0x16A0, 0x0000},
	{0x16A4, 0x0000},
	{0x16A8, 0x0000},
	{0x16AC, 0x0000},
	{0x16B4, 0x0000},
	{0x16C0, 0x0000},
	{0x16C4, 0x0000},
	{0x16C8, 0x0000},
	{0x16CC, 0x0000},
	{0x16D4, 0x0000},
	{0x16E0, 0x0000},
	{0x16E4, 0x0000},
	{0x16E8, 0x0000},
	{0x16EC, 0x0000},
	{0x16F4, 0x0000},
	{0x1700, 0x0000},
	{0x1704, 0x0000},
	{0x1708, 0x0000},
	{0x170C, 0x0000},
	{0x1714, 0x0000},
	{0x1720, 0x0000},
	{0x1724, 0x0000},
	{0x1728, 0x0000},
	{0x172C, 0x0000},
	{0x1734, 0x0000},
	{0x1740, 0x0000},
	{0x1744, 0x0000},
	{0x1748, 0x0000},
	{0x174C, 0x0000},
	{0x1754, 0x0000},
	{0x1760, 0x0000},
	{0x1764, 0x0000},
	{0x1768, 0x0000},
	{0x176C, 0x0000},
	{0x1774, 0x0000},
	{0x0300, 0x7000},
	{0x0304, 0x0000},
	{0x0308, 0x0000},
	{0x030C, 0x0000},
	{0x0310, 0x0000},
	{0x0314, 0x0000},
	{0x0318, 0xF7FF},
	{0x031C, 0xFFFF},
	{0x0320, 0x000F},
	{0x0324, 0x0000},
	{0x0328, 0x0000},
	{0x032C, 0x0000},
};

struct s1d_regs pwm_seq[] = {
	{0x001C, 0x0010},
	{0x14A0, 0x0001},
	{0x14A4, 0x0110},
	{0x14B0, 0x3030},
	{0x14A8, 0x09C4},
	{0x14AC, 0x0FF0},
};
extern int qspi_send_9bit(unsigned char id, unsigned data);
extern int qspi_send_16bit(unsigned char id, unsigned data);

static void suc_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if (val < 30)
		shrink_br = 5;
	else if ((val >= 30) && (val <= 143))
		shrink_br = 104 * (val - 30) / 113 + 5;
	else
		shrink_br = 145 * (val - 144) / 111 + 110;
	mutex_lock(&cabc.lock);
	if (panel_type == PANEL_SHARP) {
		int i, reg, val;
		for (i = 0; i < ARRAY_SIZE(pwm_seq); i++) {
			reg = pwm_seq[i].reg;
			val = pwm_seq[i].val;
			if (reg == REG_WAIT)
				msleep(val);
			else
				client->remote_write(client, cpu_to_le32(val), reg);
		}
		client->remote_write(client, shrink_br, 0x14B4);
	} else {
		qspi_send_16bit(0x1, 0x55);
		qspi_send_16bit(0x0, 0x00);
		qspi_send_16bit(0x2, 0x00);

		qspi_send_16bit(0x1, 0x51);
		qspi_send_16bit(0x0, 0x00);
		qspi_send_16bit(0x2, shrink_br);
	}
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
suc_get_brightness(struct led_classdev *led_cdev)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	if (panel_type == PANEL_SHARP)
		return client->remote_read(client, 0x14B4);
	else
		return client->remote_read(client, 0x5100);
}

#define DEFAULT_BRIGHTNESS 100
static void suc_backlight_switch(int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
		 * If the physical layer is not ready, we should
		 * not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		suc_set_brightness(&cabc.lcd_backlight, val);
	} else {
		clear_bit(GATE_ON, &cabc.status);
		suc_set_brightness(&cabc.lcd_backlight, 0);
	}
}

static int suc_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = suc_set_brightness;
	cabc.lcd_backlight.brightness_get = suc_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static int
supersonic_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, ret;
	unsigned reg, val;

	if (panel_type == PANEL_SHARP) {
		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(s1d13775_init_seq); i++) {
			reg = s1d13775_init_seq[i].reg;
			val = s1d13775_init_seq[i].val;
			if (reg == REG_WAIT)
				msleep(val);
			else
				client_data->remote_write(client_data, cpu_to_le32(val), reg);
		}
		client_data->auto_hibernate(client_data, 1);

		struct spi_cmd {
			unsigned char reg;
			unsigned char val;
			unsigned int delay;
		} sharp_spi[] = {
			{0x0, 0x11, 100},

			{0x0, 0xB9, 0},
			{0x1, 0xFF, 0},
			{0x1, 0x83, 0},
			{0x1, 0x63, 0},

			{0x0, 0x3A, 0},
			{0x1, 0x50, 0},
		};

		/* FIXME */

		for (i = 0; i < ARRAY_SIZE(sharp_spi); i++) {
			ret = qspi_send_9bit(sharp_spi[i].reg, sharp_spi[i].val);
			if (ret < 0)
				printk("%s: spi_write fail!\n", __func__);
			else if (sharp_spi[i].delay)
				msleep(sharp_spi[i].delay);
		}
	}
	else {
		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq); i++) {
			reg = cpu_to_le32(nov_init_seq[i].reg);
			val = cpu_to_le32(nov_init_seq[i].val);
			if (reg == REG_WAIT)
				msleep(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		client_data->auto_hibernate(client_data, 1);
	}
	return 0;
}

static int
supersonic_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	if (panel_type == PANEL_SHARP) {
		int i, ret;
		struct spi_cmd {
		unsigned char reg;
		unsigned char val;
		unsigned int delay;
		} sharp_spi[] = {
			{0x0, 0x28, 0},
			{0x0, 0x10, 100},
		};

		/* FIXME */

	        for (i = 0; i < ARRAY_SIZE(sharp_spi); i++) {
			ret = qspi_send_9bit(sharp_spi[i].reg, sharp_spi[i].val);
			if (ret < 0)
				printk("%s: spi_write fail!\n", __func__);
			else if (sharp_spi[i].delay)
				msleep(sharp_spi[i].delay);
		}
	}
	else
		client_data->remote_write(client_data, 0, 0x2800);

	return 0;
}

/* FIXME: remove after XA03 */
static int backlight_control(int on)
{
	struct i2c_adapter *adap = i2c_get_adapter(0);
	struct i2c_msg msg;
	u8 buf[] = {0x90, 0x00, 0x00, 0x08};
	int ret = -EIO, max_retry = 3;

	msg.addr = 0xcc >> 1;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	if (on == 0)
		buf[0] = 0x91;

	while (max_retry--) {
		ret = i2c_transfer(adap, &msg, 1);
		if (ret != 1)
			msleep(1);
		else {
			ret = 0;
			break;
		}
		ret = -EIO;
	}

	if (ret)
		printk(KERN_ERR "backlight control fail\n");
	return 0;
}

static int
supersonic_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	suc_backlight_switch(LED_OFF);
	backlight_control(0);
	return 0;
}

static int
supersonic_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	if (panel_type == PANEL_AUO) {
		suc_backlight_switch(LED_FULL);
		client_data->remote_write(client_data, 0x01, 0xB101);
		client_data->remote_write(client_data, 0x82, 0xB102);
		client_data->remote_write(client_data, 0x5A, 0xB107);
		client_data->remote_write(client_data, 0x00, 0x4400);
		client_data->remote_write(client_data, 0xC8, 0x4401);
		client_data->remote_write(client_data, 0x00, 0x2900);
		msleep(100);
		client_data->remote_write(client_data, 0x24, 0x5300);
	} else {
		suc_backlight_switch(LED_FULL);
		client_data->remote_write(client_data, 0x3043, 0x0020);
		client_data->remote_write(client_data, 0x10C8, 0x0404);
		client_data->remote_write(client_data, 0x4000, 0x0600);
		msleep(10);
		qspi_send_9bit(0x0, 0x29);
		client_data->remote_write(client_data, 0x7000, 0x0324);
		client_data->remote_write(client_data, 0x4000, 0x0600);
	}

	backlight_control(1);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = supersonic_mddi_init,
	.uninit = supersonic_mddi_uninit,
	.blank = supersonic_panel_blank,
	.unblank = supersonic_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
	},
};

static struct msm_mddi_bridge_platform_data epson_client_data = {
	.init = supersonic_mddi_init,
	.uninit = supersonic_mddi_uninit,
	.blank = supersonic_panel_blank,
	.unblank = supersonic_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
	},
};


#define	SPI_CLK	17
#define	SPI_DO	18
#define	SPI_DI	19
#define	SPI_CS	20

#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA)
static uint32_t spi_on_gpio_table[] = {
	LCM_GPIO_CFG(SPI_CLK, 1),
	LCM_GPIO_CFG(SPI_CS, 1),
	LCM_GPIO_CFG(SPI_DO, 1),
	PCOM_GPIO_CFG(SPI_DI, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
};

static uint32_t spi_off_gpio_table[] = {
	LCM_GPIO_CFG(SPI_CLK, 0),
	LCM_GPIO_CFG(SPI_CS, 0),
	LCM_GPIO_CFG(SPI_DO, 0),
	PCOM_GPIO_CFG(SPI_DI, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
};

static int spi_gpio_switch(int on)
{
        config_gpio_table(
                !!on ? spi_on_gpio_table : spi_off_gpio_table,
                ARRAY_SIZE(spi_on_gpio_table));

        return 0;
}

static void
mddi_novatec_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");

	if (on) {
		on_off = 1;
		/* 2V8 */
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcd_2v8);

		/* 1V8 */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcd_1v8);
		msleep(15);

		gpio_set_value(SUPERSONIC_LCD_RST, 1);
		msleep(1);
		gpio_set_value(SUPERSONIC_LCD_RST, 0);
		msleep(5);
		gpio_set_value(SUPERSONIC_LCD_RST, 1);
		msleep(50);
		spi_gpio_switch(1);
	} else {
		on_off = 0;
		gpio_set_value(SUPERSONIC_LCD_RST, 0);
		msleep(120);

		/* 1V8 */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcd_1v8);

		/* 2V8 */
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcd_2v8);
		spi_gpio_switch(0);
	}
}

static void
mddi_epson_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");

	if (on) {
		on_off = 1;
		/* 2V8 */
		gpio_set_value(149, 1);
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcd_2v8);
		msleep(5);
		/* 1V8 */
		gpio_set_value(16, 1);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcd_1v8);
		msleep(10);

		gpio_set_value(151, 1);
		msleep(2);

		gpio_set_value(SUPERSONIC_LCD_RST, 1);
		msleep(1);
		gpio_set_value(SUPERSONIC_LCD_RST, 0);
		msleep(5);
		gpio_set_value(SUPERSONIC_LCD_RST, 1);
		msleep(50);
		spi_gpio_switch(1);
	} else {
		on_off = 0;
		gpio_set_value(SUPERSONIC_LCD_RST, 0);
		msleep(2);
		gpio_set_value(151, 0);
		msleep(120);

		/* 1V8 */
		gpio_set_value(16, 0);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcd_1v8);
		msleep(5);
		/* 2V8 */
		gpio_set_value(149, 0);
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcd_2v8);
		spi_gpio_switch(0);
	}
}

static struct msm_mddi_platform_data mddi_pdata = {
	.clk_rate = 384000000,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5582),
			.name = "mddi_c_b9f6_5582",
			.id = 1,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			.product_id = (0x4ca3 << 16 | 0x0000),
			.name = "mddi_c_4ca3_0000",
			.id = 0,
			.client_data = &epson_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver suc_backlight_driver = {
	.probe = suc_backlight_probe,
	.driver = {
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.dma_channel = MDP_DMA_S,
};

int __init supersonic_init_panel(void)
{
	int rc;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcd_1v8 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcd_1v8))
		return PTR_ERR(vreg_lcd_1v8);

	vreg_lcd_2v8 = vreg_get(0, "synt");
	if (IS_ERR(vreg_lcd_2v8))
		return PTR_ERR(vreg_lcd_2v8);

	if (panel_type == PANEL_SHARP)
		mdp_pdata.overrides |= MSM_MDP_PANEL_IGNORE_PIXEL_DATA;
	else
		mdp_pdata.overrides &= ~MSM_MDP_PANEL_IGNORE_PIXEL_DATA;

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	if (panel_type)
		mddi_pdata.power_client = mddi_novatec_power;
	else
		mddi_pdata.power_client = mddi_epson_power;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	if (panel_type)
		suc_backlight_driver.driver.name = "nov_cabc";
	else
		suc_backlight_driver.driver.name = "eps_cabc";
	rc = platform_driver_register(&suc_backlight_driver);
	if (rc)
		return rc;

	return 0;
}
