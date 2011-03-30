/* drivers/power/ds2784_battery.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Copyright (C) 2009 Google, Inc.
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
 * additional code by XDA members RogerPodacter and theloginwithnoname, 2010
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/android_alarm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/ds2784_battery.h>

#include "../w1/w1.h"
#include "w1_ds2784.h"

extern int is_ac_power_supplied(void);

struct battery_status {
	int timestamp;

	int voltage_uV;		/* units of uV */
	int current_uA;		/* units of uA */
	int current_avg_uA;
	int charge_uAh;

	u16 temp_C;		/* units of 0.1 C */

	u8 percentage;		/* battery percentage */
	u8 charge_source;
	u8 status_reg;
	u8 battery_full;	/* battery full (don't charge) */

	u8 cooldown;		/* was overtemp */
	u8 charge_mode;
} __attribute__((packed));


#define SOURCE_NONE	0
#define SOURCE_USB	1
#define SOURCE_AC	2

#define CHARGE_OFF	0
#define CHARGE_SLOW	1
#define CHARGE_FAST	2
#define CHARGE_BATT_DISABLE	3 /* disable charging at battery */

#define TEMP_CRITICAL	600 /* no charging at all */
#define TEMP_HOT	500 /* no fast charge, no charge > 4.1v */
#define TEMP_WARM	450 /* no fast charge above this */

#define TEMP_HOT_MAX_MV	4100 /* stop charging here when hot */
#define TEMP_HOT_MIN_MV	3800 /* resume charging here when hot */
#define CE_DISABLE_MIN_MV 4100

#define BATTERY_LOG_MAX 1024
#define BATTERY_LOG_MASK (BATTERY_LOG_MAX - 1)

/* When we're awake or running on wall power, sample the battery
 * gauge every FAST_POLL seconds.  If we're asleep and on battery
 * power, sample every SLOW_POLL seconds
 */
#define FAST_POLL	(1 * 60)
#define SLOW_POLL	(10 * 60)

static DEFINE_MUTEX(battery_log_lock);
static struct battery_status battery_log[BATTERY_LOG_MAX];
static unsigned battery_log_head;
static unsigned battery_log_tail;

void battery_log_status(struct battery_status *s)
{
	unsigned n;
	mutex_lock(&battery_log_lock);
	n = battery_log_head;
	memcpy(battery_log + n, s, sizeof(struct battery_status));
	n = (n + 1) & BATTERY_LOG_MASK;
	if (n == battery_log_tail)
		battery_log_tail = (battery_log_tail + 1) & BATTERY_LOG_MASK;
	battery_log_head = n;
	mutex_unlock(&battery_log_lock);
}

static const char *battery_source[3] = { "none", " usb", "  ac" };
static const char *battery_mode[4] = { " off", "slow", "fast", "full" };

static int battery_log_print(struct seq_file *sf, void *private)
{
	unsigned n;
	mutex_lock(&battery_log_lock);
	seq_printf(sf, "timestamp    mV     mA avg mA      uAh   dC   %%   src  mode   reg full\n");
	for (n = battery_log_tail; n != battery_log_head; n = (n + 1) & BATTERY_LOG_MASK) {
		struct battery_status *s = battery_log + n;
		seq_printf(sf, "%9d %5d %6d %6d %8d %4d %3d  %s  %s  0x%02x %d\n",
			   s->timestamp, s->voltage_uV / 1000,
			   s->current_uA / 1000, s->current_avg_uA / 1000,
			   s->charge_uAh, s->temp_C,
			   s->percentage,
			   battery_source[s->charge_source],
			   battery_mode[s->charge_mode],
			   s->status_reg, s->battery_full);
	}
	mutex_unlock(&battery_log_lock);
	return 0;
}


struct ds2784_device_info {
	struct device *dev;

	/* DS2784 data, valid after calling ds2784_battery_read_status() */
	char raw[DS2784_DATA_SIZE];	/* raw DS2784 data */

	struct battery_status status;

	struct power_supply bat;
	struct workqueue_struct *monitor_wqueue;
	struct work_struct monitor_work;
	struct alarm alarm;
	struct wake_lock work_wake_lock;

	int (*charge)(int on, int fast);
	struct w1_slave *w1_slave;

	u8 dummy; /* dummy battery flag */
	u8 last_charge_mode; /* previous charger state */
	u8 slow_poll;

	ktime_t last_poll;
	ktime_t last_charge_seen;
};

#define psy_to_dev_info(x) container_of((x), struct ds2784_device_info, bat)

static struct wake_lock vbus_wake_lock;

#define BATT_RSNSP			(67)	/*Passion battery source 1*/

static enum power_supply_property battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int battery_initial;

static int battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static void battery_ext_power_changed(struct power_supply *psy);

#define to_ds2784_device_info(x) container_of((x), struct ds2784_device_info, \
					      bat);

static void ds2784_parse_data(u8 *raw, struct battery_status *s)
{
	short n;

	/* Get status reg */
	s->status_reg = raw[DS2784_REG_STS];

	/* Get Level */
	s->percentage = raw[DS2784_REG_RARC];

	/* Get Voltage: Unit=4.886mV, range is 0V to 4.99V */
	n = (((raw[DS2784_REG_VOLT_MSB] << 8) |
	      (raw[DS2784_REG_VOLT_LSB])) >> 5);

	s->voltage_uV = n * 4886;

	/* Get Current: Unit= 1.5625uV x Rsnsp(67)=104.68 */
	n = ((raw[DS2784_REG_CURR_MSB]) << 8) |
		raw[DS2784_REG_CURR_LSB];
	s->current_uA = ((n * 15625) / 10000) * 67;

	n = ((raw[DS2784_REG_AVG_CURR_MSB]) << 8) |
		raw[DS2784_REG_AVG_CURR_LSB];
	s->current_avg_uA = ((n * 15625) / 10000) * 67;

	/* Get Temperature:
	 * 11 bit signed result in Unit=0.125 degree C.
	 * Convert to integer tenths of degree C.
	 */
	n = ((raw[DS2784_REG_TEMP_MSB] << 8) |
		(raw[DS2784_REG_TEMP_LSB])) >> 5;

	s->temp_C = (n * 10) / 8;

	/* RAAC is in units of 1.6mAh */
	s->charge_uAh = ((raw[DS2784_REG_RAAC_MSB] << 8) |
			  raw[DS2784_REG_RAAC_LSB]) * 1600;
}

static int w1_ds2784_io(struct w1_slave *sl, char *buf, int addr, size_t count, int io)
{
	if (!sl)
		return 0;

	mutex_lock(&sl->master->mutex);

	if (addr > DS2784_DATA_SIZE || addr < 0) {
		count = 0;
		goto out;
	}
	if (addr + count > DS2784_DATA_SIZE)
		count = DS2784_DATA_SIZE - addr;

	if (!w1_reset_select_slave(sl)) {
		if (!io) {
			w1_write_8(sl->master, W1_DS2784_READ_DATA);
			w1_write_8(sl->master, addr);
			count = w1_read_block(sl->master, buf, count);
		} else {
			w1_write_8(sl->master, W1_DS2784_WRITE_DATA);
			w1_write_8(sl->master, addr);
			w1_write_block(sl->master, buf, count);
			/* XXX w1_write_block returns void, not n_written */
		}
	}

out:
	mutex_unlock(&sl->master->mutex);

	return count;
}

static int w1_ds2784_read(struct w1_slave *sl, char *buf, int addr, size_t count)
{
	return w1_ds2784_io(sl, buf, addr, count, 0);
}

static int w1_ds2784_write(struct w1_slave *sl, char *buf, int addr, size_t count)
{
	return w1_ds2784_io(sl, buf, addr, count, 1);
}

static int ds2784_set_cc(struct ds2784_device_info *di, bool enable)
{
	int ret;

	if (enable)
		di->raw[DS2784_REG_PORT] |= 0x02;
	else
		di->raw[DS2784_REG_PORT] &= ~0x02;
	ret = w1_ds2784_write(di->w1_slave, di->raw + DS2784_REG_PORT,
			      DS2784_REG_PORT, 1);
	if (ret != 1) {
		dev_warn(di->dev, "call to w1_ds2784_write failed (0x%p)\n",
			 di->w1_slave);
		return 1;
	}
	return 0;
}

static int ds2784_battery_read_status(struct ds2784_device_info *di)
{
	int ret, start, count;

	/* The first time we read the entire contents of SRAM/EEPROM,
	 * but after that we just read the interesting bits that change. */
	if (di->raw[DS2784_REG_RSNSP] == 0x00) {
		start = 0;
		count = DS2784_DATA_SIZE;
	} else {
		start = DS2784_REG_PORT;
		count = DS2784_REG_CURR_LSB - start + 1;
	}

	ret = w1_ds2784_read(di->w1_slave, di->raw + start, start, count);
	if (ret != count) {
		dev_warn(di->dev, "call to w1_ds2784_read failed (0x%p)\n",
			di->w1_slave);
		return 1;
	}

	if (battery_initial == 0) {
		if (!memcmp(di->raw + 0x20, "DUMMY!", 6)) {
			unsigned char acr[2];

			di->dummy = 1;
			pr_info("batt: dummy battery detected\n");

			/* reset ACC register to ~500mAh, since it may have zeroed out */
			acr[0] = 0x05;
			acr[1] = 0x06;
			w1_ds2784_write(di->w1_slave, acr, DS2784_REG_ACCUMULATE_CURR_MSB, 2);
		}
		battery_initial = 1;
	}

	ds2784_parse_data(di->raw, &di->status);

	pr_info("batt: %3d%%, %d mV, %d mA (%d avg), %d.%d C, %d mAh\n",
		di->status.percentage,
		di->status.voltage_uV / 1000, di->status.current_uA / 1000,
		di->status.current_avg_uA / 1000,
		di->status.temp_C / 10, di->status.temp_C % 10,
		di->status.charge_uAh / 1000);

	return 0;
}

static int battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct ds2784_device_info *di = psy_to_dev_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (di->status.charge_source) {
		case CHARGE_OFF:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case CHARGE_FAST:
		case CHARGE_SLOW:
			if (di->status.battery_full)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if (di->status.charge_mode == CHARGE_OFF ||
				 di->status.charge_mode == CHARGE_BATT_DISABLE)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (di->status.temp_C >= TEMP_HOT)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* XXX todo */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		if (di->dummy)
			val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		else
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->dummy)
			val->intval = 75;
		else
			val->intval = di->status.percentage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->status.voltage_uV;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->status.temp_C;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->status.current_uA;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->status.current_avg_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = di->status.charge_uAh;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t set_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int reg;
	int val;
	int check;

	sscanf(buf, "%x %x", &reg, &val);

	if (reg < 0 || reg > 255)
    	    return -EINVAL;
	if (val < 0 || val > 255)
    	    return -EINVAL;

	di->raw[reg] = val;
	check = w1_ds2784_write(di->w1_slave, di->raw + reg, reg, 1);

	if (check != 1) {
	    dev_warn(di->dev, "w1_ds2784_write register failed (ox%p)\n", di->w1_slave);
	}

	pr_info("batt: register 0x%02x changed to 0x%02x by user\n", reg, val);

	return count;
}

static DEVICE_ATTR(setreg, 0666, NULL, set_reg);

static ssize_t dump_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);

	int addr;
	int ret = 0;
	int val;
	int printbytes = 0;

	for (addr = 0; addr <= 0xb1; addr++, printbytes++) {
		if (addr == 0 || addr == 0x30 || addr == 0x80) {
			if (addr == 0x30) {
                		addr = 0x60;
			}
			if (addr == 0x80) {
                    	addr = 0xb0;
			}

			if (PAGE_SIZE-ret > 2) {
				ret+= snprintf(&buf[ret], PAGE_SIZE-ret, "\n%02x:", addr);
			}

			w1_ds2784_read(di->w1_slave, di->raw + addr, addr, 1);
			val = di->raw[addr];
			printbytes = 0;
		}

		if (printbytes >= 16) {
			if (PAGE_SIZE-ret > 2) {
				ret+= snprintf(&buf[ret], PAGE_SIZE-ret, "\n%02x:", addr);
			}
			printbytes = 0;
		}

		if (PAGE_SIZE-ret > 2) {
			val = di->raw[addr];
				ret+= snprintf(&buf[ret], PAGE_SIZE-ret, " %02x", val);
		}
		else
		{
			break;
		}
	}
	if (PAGE_SIZE-ret > 2) {
             ret+= snprintf(&buf[ret], PAGE_SIZE-ret, "\n");
	}

	return ret;
}

static DEVICE_ATTR(dumpreg, 0444, dump_regs, NULL);

static ssize_t show_status_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int ret;
	int statusreg;
	int check;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_STS, DS2784_REG_STS, 1);

	if (check != 1) {
	    dev_warn(di->dev, "w1_ds2784_read Status Register failed (ox%p)\n", di->w1_slave);
	}

	statusreg = di->raw[DS2784_REG_STS];

	ret = sprintf(buf, "0x%02x\n", statusreg);
	return ret;
}

static ssize_t store_status_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int val;
	int check;

	sscanf(buf, "%x", &val);

	if (val < 0 || val > 255)
    	    return -EINVAL;

	di->raw[DS2784_REG_STS] = val;
	check = w1_ds2784_write(di->w1_slave, di->raw + DS2784_REG_STS, DS2784_REG_STS, 1);

	if (check != 1) {
	    dev_warn(di->dev, "w1_ds2784_write Status Register failed (ox%p)\n", di->w1_slave);
	}

	pr_info("batt: Status Register set to: 0x%02x \n", val);

	return count;
}

static DEVICE_ATTR(statusreg, 0666, show_status_reg, store_status_reg);

static ssize_t show_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int ret;
	int getvoltage;
	int check;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_VOLT_MSB, DS2784_REG_VOLT_MSB, 2);

	getvoltage = (((di->raw[DS2784_REG_VOLT_MSB]<<8) | (di->raw[DS2784_REG_VOLT_LSB]))>>5)*4886;

	if (check != 2) {
	    dev_warn(di->dev, "w1_ds2784_read Voltage failed (ox%p)\n", di->w1_slave);
	}

	ret = sprintf(buf, "%d\n", getvoltage);
	return ret;
}

static DEVICE_ATTR(getvoltage, 0644, show_voltage, NULL);  // deprecated
static DEVICE_ATTR(voltageNow, 0444, show_voltage, NULL);

static ssize_t show_current(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	short n;
	int ret;
	int getcurrent;
	int check;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_CURR_MSB, DS2784_REG_CURR_MSB, 2);

	if (check != 2) {
	    dev_warn(di->dev, "w1_ds2784_read Current failed (ox%p)\n", di->w1_slave);
	}

	n = ((di->raw[DS2784_REG_CURR_MSB]) << 8) | di->raw[DS2784_REG_CURR_LSB];
	getcurrent = ((n * 15625) / 10000) * 67;

	ret = sprintf(buf, "%d\n", getcurrent);
	return ret;
}

static DEVICE_ATTR(getcurrent, 0644, show_current, NULL);  // deprecated
static DEVICE_ATTR(currentNow, 0444, show_current, NULL);

static ssize_t show_avgcurrent(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	short n;
	int ret;
	int getavgcurrent;
	int check;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_AVG_CURR_MSB, DS2784_REG_AVG_CURR_MSB, 2);

	if (check != 2) {
	    dev_warn(di->dev, "w1_ds2784_read Avg Current failed (ox%p)\n", di->w1_slave);
	}

	n = ((di->raw[DS2784_REG_AVG_CURR_MSB]) << 8) | di->raw[DS2784_REG_AVG_CURR_LSB];
	getavgcurrent = ((n * 15625) / 10000) * 67;

	ret = sprintf(buf, "%d\n", getavgcurrent);
	return ret;
}

// compatibility til app update
static DEVICE_ATTR(getavgcurrent, 0644, show_avgcurrent, NULL);  // deprecated
static DEVICE_ATTR(currentAverage, 0444, show_avgcurrent, NULL);

static ssize_t show_age(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int ret;
	int check;
	int age;
	int ageraw;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_AGE_SCALAR, DS2784_REG_AGE_SCALAR, 1);

	if (check != 1) {
	dev_warn(di->dev, "w1_ds2784_read age_scalar failed (ox%p)\n", di->w1_slave);
	}

	ageraw = di->raw[DS2784_REG_AGE_SCALAR];
	age = (ageraw * 100) / 128;

	pr_info("%d\n", age);

	ret = sprintf(buf, "%d\n", age);

	return ret;
}

static ssize_t set_age(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int age;
	int check;

	sscanf(buf, "%d", &age);

	di->raw[DS2784_REG_AGE_SCALAR] = ((age * 128) / 100);

	check = w1_ds2784_write(di->w1_slave, di->raw + DS2784_REG_AGE_SCALAR, DS2784_REG_AGE_SCALAR, 1);

	if (check != 1) {
	dev_warn(di->dev, "w1_ds2784_write age_scalar failed (ox%p)\n", di->w1_slave);
	}

	pr_info("batt: age_scalar set to: %d percent\n", age);

	return count;
}

static DEVICE_ATTR(age, 0666, show_age, set_age);

static ssize_t show_AEvolt(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int ret;
	int check;
	int rawaevolt;
 	int aevolt;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_ACTIVE_EMPTY_VOLT, DS2784_REG_ACTIVE_EMPTY_VOLT, 1);

	if (check != 1) {
	    dev_warn(di->dev, "w1_ds2784_read Active Empty Voltage failed (ox%p)\n", di->w1_slave);
	}
	rawaevolt = di->raw[DS2784_REG_ACTIVE_EMPTY_VOLT];
	aevolt = (rawaevolt * 1952) / 100;

	// pr_info("batt: Active Empty Voltage is: %d volts\n", aevolt);

	ret = sprintf(buf, "%d\n", aevolt);

	return ret;
}

static ssize_t set_AEvolt (struct device *dev, struct device_attribute *attr, const char  *buf, size_t count)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int val;
	int check;
	int temp;

	sscanf(buf, "%d", &val);

	di->raw[DS2784_REG_ACTIVE_EMPTY_VOLT] = ( ( val * 100 ) / 1952 ) ;

	check = w1_ds2784_write(di->w1_slave, di->raw + DS2784_REG_ACTIVE_EMPTY_VOLT, DS2784_REG_ACTIVE_EMPTY_VOLT, 1);

	if (check != 1) {
	    dev_warn(di->dev, "w1_ds2784_write Active Empty Voltage failed (ox%p)\n", di->w1_slave);
	}

	temp = ( ( val * 100 ) / 1952 ) ;

	pr_info("batt: Active Empty Voltage set to: %d volts\n", temp);

	return count;
}

static DEVICE_ATTR(voltAE, 0644, show_AEvolt, set_AEvolt);  // deprecated
static DEVICE_ATTR(voltageActiveEmpty, 0666, show_AEvolt, set_AEvolt);

static ssize_t show_full40(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int ret;
	int check;
	int full40mAh, full40raw;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_FULL_40_MSB, DS2784_REG_FULL_40_MSB, 2);

	if (check != 2) {
	    dev_warn(di->dev, "w1_ds2784_read Full40 mAh failed (ox%p)\n", di->w1_slave);
	}

	full40raw = ((di->raw[DS2784_REG_FULL_40_MSB]) << 8) | di->raw[DS2784_REG_FULL_40_LSB];

	full40mAh = ((full40raw * 625) / 100) / 15;

	// no need to put this in log, only when writing
	// pr_info("batt: Full40 mAh capacity is: %d mAh\n", full40mAh);

	ret = sprintf(buf, "%dmAh\n", full40mAh);

	return ret;
}

// backwards compatibility removed
// static DEVICE_ATTR(getFull40, 0644, show_full40, NULL);

// again, backwards compatibility until app updated
static DEVICE_ATTR(getfull40, 0644, show_full40, NULL);  // deprecated

// use "proper" permissions, appropriate to read/write nature of file
// correct the filename - permissions indicate whether value changeable from now on
static DEVICE_ATTR(full40, 0444, show_full40, NULL);

static ssize_t show_mAh(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);
	int ret;
	int check;
	int mAh;

	check = w1_ds2784_read(di->w1_slave, di->raw + DS2784_REG_RAAC_MSB, DS2784_REG_RAAC_MSB, 2);

	if (check != 2) {
	dev_warn(di->dev, "w1_ds2784_read mAh failed (ox%p)\n", di->w1_slave);
	}

	mAh = ((di->raw[DS2784_REG_RAAC_MSB] << 8) | di->raw[DS2784_REG_RAAC_LSB]) * 1600;

	ret = sprintf(buf, "%d\n", mAh);

	return ret;
}

static DEVICE_ATTR(getmAh, 0644, show_mAh, NULL);  // deprecated
static DEVICE_ATTR(mAh, 0444, show_mAh, NULL);

static void ds2784_battery_update_status(struct ds2784_device_info *di)
{
	u8 last_level;
	last_level = di->status.percentage;

	ds2784_battery_read_status(di);

	if ((last_level != di->status.percentage) || (di->status.temp_C > 450))
		power_supply_changed(&di->bat);
}

static DEFINE_MUTEX(charge_state_lock);

static bool check_timeout(ktime_t now, ktime_t last, int seconds)
{
	ktime_t timeout = ktime_add(last, ktime_set(seconds, 0));
	return ktime_sub(timeout, now).tv64 < 0;
}

static int battery_adjust_charge_state(struct ds2784_device_info *di)
{
	unsigned source;
	int rc = 0;
	int temp, volt;
	u8 charge_mode;
	bool charge_timeout = false;

	mutex_lock(&charge_state_lock);

	temp = di->status.temp_C;
	volt = di->status.voltage_uV / 1000;

	source = di->status.charge_source;

	/* initially our charge mode matches our source:
	 * NONE:OFF, USB:SLOW, AC:FAST
	 */
	charge_mode = source;

	/* shut off charger when full:
	 * - CHGTF flag is set
	 */

	if (di->status.status_reg & 0x80) {
		di->status.battery_full = 1;
		charge_mode = CHARGE_BATT_DISABLE;
	}
	else
		di->status.battery_full = 0;


	if (temp >= TEMP_HOT) {
		if (temp >= TEMP_CRITICAL)
			charge_mode = CHARGE_BATT_DISABLE;

		/* once we charge to max voltage when hot, disable
		 * charging until the temp drops or the voltage drops
		 */
		if (volt >= TEMP_HOT_MAX_MV)
			di->status.cooldown = 1;
	}

	/* when the battery is warm, only charge in slow charge mode */
	if ((temp >= TEMP_WARM) && (charge_mode == CHARGE_FAST))
		charge_mode = CHARGE_SLOW;

	if (di->status.cooldown) {
		if ((temp < TEMP_WARM) || (volt <= TEMP_HOT_MIN_MV))
			di->status.cooldown = 0;
		else
			charge_mode = CHARGE_BATT_DISABLE;
	}

	if (di->status.current_uA > 1024)
		di->last_charge_seen = di->last_poll;
	else if (di->last_charge_mode != CHARGE_OFF &&
		 check_timeout(di->last_poll, di->last_charge_seen, 60 * 60)) {
		if (di->last_charge_mode == CHARGE_BATT_DISABLE) {
			/* The charger is only powering the phone. Toggle the
			 * enable line periodically to prevent auto shutdown.
			 */
			di->last_charge_seen = di->last_poll;
			pr_info("batt: charging POKE CHARGER\n");
			di->charge(0, 0);
			udelay(10);
			di->charge(1, source == CHARGE_FAST);
		} else {
			/* The charger has probably stopped charging. Turn it
			 * off until the next sample period.
			 */
			charge_timeout = true;
			charge_mode = CHARGE_OFF;
		}
	}

	if (source == CHARGE_OFF)
		charge_mode = CHARGE_OFF;

	/* Don't use CHARGE_BATT_DISABLE unless the voltage is high since the
	 * voltage drop over the discharge-path diode can cause a shutdown.
	 */
	if (charge_mode == CHARGE_BATT_DISABLE && volt < CE_DISABLE_MIN_MV)
		charge_mode = CHARGE_OFF;

	if (di->last_charge_mode == charge_mode)
		goto done;

	di->last_charge_mode = charge_mode;
	di->status.charge_mode = charge_mode;

	switch (charge_mode) {
	case CHARGE_OFF:
		di->charge(0, 0);
		ds2784_set_cc(di, true);
		if (temp >= TEMP_CRITICAL)
			pr_info("batt: charging OFF [OVERTEMP]\n");
		else if (di->status.cooldown)
			pr_info("batt: charging OFF [COOLDOWN]\n");
		else if (di->status.battery_full)
			pr_info("batt: charging OFF [FULL]\n");
		else if (charge_timeout)
			pr_info("batt: charging OFF [TIMEOUT]\n");
		else
			pr_info("batt: charging OFF\n");
		break;
	case CHARGE_BATT_DISABLE:
		di->last_charge_seen = di->last_poll;
		ds2784_set_cc(di, false);
		di->charge(1, source == CHARGE_FAST);
		if (temp >= TEMP_CRITICAL)
			pr_info("batt: charging BATTOFF [OVERTEMP]\n");
		else if (di->status.cooldown)
			pr_info("batt: charging BATTOFF [COOLDOWN]\n");
		else if (di->status.battery_full)
			pr_info("batt: charging BATTOFF [FULL]\n");
		else
			pr_info("batt: charging BATTOFF [UNKNOWN]\n");
		break;
	case CHARGE_SLOW:
		di->last_charge_seen = di->last_poll;
		ds2784_set_cc(di, true);
		di->charge(1, 0);
		pr_info("batt: charging SLOW\n");
		break;
	case CHARGE_FAST:
		di->last_charge_seen = di->last_poll;
		ds2784_set_cc(di, true);
		di->charge(1, 1);
		pr_info("batt: charging FAST\n");
		break;
	}
	rc = 1;
done:
	mutex_unlock(&charge_state_lock);
	return rc;
}

static void ds2784_program_alarm(struct ds2784_device_info *di, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(di->last_poll, low_interval);

	alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static void ds2784_battery_work(struct work_struct *work)
{
	struct ds2784_device_info *di =
		container_of(work, struct ds2784_device_info, monitor_work);
	struct timespec ts;
	unsigned long flags;

	ds2784_battery_update_status(di);

	di->last_poll = alarm_get_elapsed_realtime();

	if (battery_adjust_charge_state(di))
		power_supply_changed(&di->bat);

	ts = ktime_to_timespec(di->last_poll);
	di->status.timestamp = ts.tv_sec;
	battery_log_status(&di->status);

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);
	wake_unlock(&di->work_wake_lock);
	ds2784_program_alarm(di, FAST_POLL);
	local_irq_restore(flags);
}

static void ds2784_battery_alarm(struct alarm *alarm)
{
	struct ds2784_device_info *di =
		container_of(alarm, struct ds2784_device_info, alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
}

static void battery_ext_power_changed(struct power_supply *psy)
{
	struct ds2784_device_info *di;
	int got_power;

	di = psy_to_dev_info(psy);
	got_power = power_supply_am_i_supplied(psy);

	if (got_power) {
		if (is_ac_power_supplied())
			di->status.charge_source = SOURCE_AC;
		else
			di->status.charge_source = SOURCE_USB;
		wake_lock(&vbus_wake_lock);
	} else {
		di->status.charge_source = SOURCE_NONE;
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
	}
	battery_adjust_charge_state(di);
	power_supply_changed(psy);
}

static int ds2784_battery_probe(struct platform_device *pdev)
{
	int rc, ret;  /*Added "ret" as a check for creating the sysfs files*/
	struct ds2784_device_info *di;
	struct ds2784_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	platform_set_drvdata(pdev, di);

	pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->charge || !pdata->w1_slave) {
		pr_err("%s: pdata missing or invalid\n", __func__);
		rc = -EINVAL;
		goto fail_register;
	}

	di->charge = pdata->charge;
	di->w1_slave = pdata->w1_slave;

	di->dev = &pdev->dev;

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = battery_properties;
	di->bat.num_properties = ARRAY_SIZE(battery_properties);
	di->bat.external_power_changed = battery_ext_power_changed;
	di->bat.get_property = battery_get_property;
	di->last_charge_mode = 0xff;

	rc = power_supply_register(&pdev->dev, &di->bat);
	if (rc)
		goto fail_register;

	ret = device_create_file(&pdev->dev, &dev_attr_setreg);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for setreg\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_dumpreg);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for dumpreg\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_statusreg);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for statusreg\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_getvoltage);  // deprecated

	ret = device_create_file(&pdev->dev, &dev_attr_voltageNow);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for voltage\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_getcurrent);  // deprecated

	ret = device_create_file(&pdev->dev, &dev_attr_currentNow);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for current\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_getavgcurrent);  // deprecated

	ret = device_create_file(&pdev->dev, &dev_attr_currentAverage);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for avg current\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_age);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for age\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_voltAE);  // deprecated

	ret = device_create_file(&pdev->dev, &dev_attr_voltageActiveEmpty);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for voltAE\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_getfull40);  // deprecated
	
	ret = device_create_file(&pdev->dev, &dev_attr_full40);
	if (ret < 0)
		pr_err("%s: Failed to create sysfs entry for getfull40\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_getmAh);  // deprecated

	ret = device_create_file(&pdev->dev, &dev_attr_mAh);
	if(ret < 0)
	    pr_err("%s: Failed to create sysfs entry for mAh\n", __func__);

	INIT_WORK(&di->monitor_work, ds2784_battery_work);
	di->monitor_wqueue = create_freezable_workqueue(dev_name(&pdev->dev));

	/* init to something sane */
	di->last_poll = alarm_get_elapsed_realtime();

	if (!di->monitor_wqueue) {
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND,
			"ds2784-battery");
	alarm_init(&di->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			ds2784_battery_alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
	return 0;

fail_workqueue:
	power_supply_unregister(&di->bat);
fail_register:
	kfree(di);
	return rc;
}

static int ds2784_suspend(struct device *dev)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);

	/* If we are on battery, reduce our update rate until
	 * we next resume.
	 */
	if (di->status.charge_source == SOURCE_NONE) {
		ds2784_program_alarm(di, SLOW_POLL);
		di->slow_poll = 1;
	}
	return 0;
}

static int ds2784_resume(struct device *dev)
{
	struct ds2784_device_info *di = dev_get_drvdata(dev);

	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.
	 */
	if (di->slow_poll) {
		ds2784_program_alarm(di, FAST_POLL);
		di->slow_poll = 0;
	}
	return 0;
}

static struct dev_pm_ops ds2784_pm_ops = {
	.suspend	= ds2784_suspend,
	.resume		= ds2784_resume,
};

static struct platform_driver ds2784_battery_driver = {
	.driver = {
		.name = "ds2784-battery",
		.pm = &ds2784_pm_ops,
	},
	.probe = ds2784_battery_probe,
};

static int battery_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, battery_log_print, NULL);
}

static struct file_operations battery_log_fops = {
	.open = battery_log_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init ds2784_battery_init(void)
{
	debugfs_create_file("battery_log", 0444, NULL, NULL, &battery_log_fops);
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	return platform_driver_register(&ds2784_battery_driver);
}

module_init(ds2784_battery_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Justin Lin <Justin_lin@htc.com>");
MODULE_DESCRIPTION("ds2784 battery driver");
