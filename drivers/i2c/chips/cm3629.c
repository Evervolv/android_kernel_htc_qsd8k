/* drivers/i2c/chips/cm3629.c - cm3629 optical sensors driver
 *
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/cm3629.h>
#include <linux/pl_sensor.h>
#include <linux/capella_cm3602.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <mach/board.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define POLLING_PROXIMITY 1
#define NO_IGNORE_BOOT_MODE 1

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#ifdef POLLING_PROXIMITY
#define POLLING_DELAY		200
#define TH_ADD			3
#endif
static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

#ifdef POLLING_PROXIMITY
static void polling_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
#endif

static uint8_t sensor_chipId[3] = {0};
static void report_near_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_near_work, report_near_do_work);

struct cm3629_info {
	struct class *cm3629_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

	struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int model;

	int intr_pin;
	int phantom_gpio;
	int als_enable;

	int ps_enable;
	int ps_irq_flag;
	int led;

	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;

	int ls_calibrate;

	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;

	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int lightsensor_opened;
	uint16_t cm3629_slave_address;
	uint8_t ps_select;
	uint8_t ps1_thd_set;
	uint8_t ps1_thh_diff;
	uint8_t ps2_thd_set;
	uint8_t original_ps_thd_set;
	int current_level;
	uint16_t current_adc;
	/* Command code 0x02, intelligent cancel level */
	uint8_t inte_ps1_canc;
	uint8_t inte_ps2_canc;
	uint8_t ps_conf1_val;
	uint8_t ps_conf2_val;
	uint8_t ps_conf3_val;
	uint8_t ps_calibration_rule; /* For Saga */
	int ps_pocket_mode;

	unsigned long j_start;
	unsigned long j_end;
	int mfg_mode;

	uint8_t *mapping_table;
	uint8_t mapping_size;
	uint8_t ps_base_index;

	uint8_t ps1_thd_no_cal;
	uint8_t ps1_thd_with_cal;
	uint8_t ps2_thd_no_cal;
	uint8_t ps2_thd_with_cal;
	uint8_t enable_polling_ignore;
	uint8_t ls_cmd;
	uint8_t ps1_adc_offset;
	uint8_t ps2_adc_offset;
	uint8_t ps_debounce;
	uint16_t ps_delay_time;
};

static uint8_t ps1_canc_set;
static uint8_t ps2_canc_set;
static uint8_t ps1_offset_adc;
static uint8_t ps2_offset_adc;
static struct cm3629_info *lp_info;
int enable_cm3629_log;
int f_cm3629_level = -1;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct cm3629_info *lpi);
static int lightsensor_disable(struct cm3629_info *lpi);
static void psensor_initial_cmd(struct cm3629_info *lpi);
#if (0)
static int I2C_RxData(uint16_t slaveAddr, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm3629_info *lpi = lp_info;

	struct i2c_msg msgs[] = {
		/*{
		 .addr = lp_info->i2c_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },*/
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/

		D("[PS][cm3629 warning] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][cm3629 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}
#endif

static int I2C_RxData_2(char *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm3629_info *lpi = lp_info;

	struct i2c_msg msgs[] = {
		{
		 .addr = lp_info->i2c_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = lp_info->i2c_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/

		D("[PS][cm3629 warning] %s, i2c err, ISR gpio %d  = %d \n",
				__func__, lpi->intr_pin, val);
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][cm3629 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}
static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm3629_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		D("[PS][cm3629 warning] %s, i2c err, slaveAddr 0x%x, register 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], txData[1], lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][cm3629 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm3629_I2C_Read2(uint16_t slaveAddr,
	uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err(
			"[PS_ERR][cm3629 error]%s: length %d> 2: \n",
			__func__, length);
		return ret;
	}
	buffer[0] = cmd;
	ret = I2C_RxData_2(buffer, length);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][cm3629 error]%s: I2C_RxData fail, slave addr: 0x%x\n",
			__func__, slaveAddr);
		return ret;
	}

	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
		/*printk(KERN_INFO "[cm3629] %s: I2C_RxData[0x%x] = 0x%x\n",
		__func__, slaveAddr, buffer[i]);*/
	}
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[cm3629] %s: I2C_RxData[0x%x] = 0x%x\n",
		__func__, slaveAddr, buffer);
#endif
	return ret;
}

static int _cm3629_I2C_Write2(uint16_t SlaveAddress,
				uint8_t cmd, uint8_t *data, int length)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[cm3629] %s: _cm3629_I2C_Write_Byte[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	if (length > 3) {
		pr_err(
			"[PS_ERR][cm3629 error]%s: length %d> 2: \n",
			__func__, length);
		return ret;
	}

	buffer[0] = cmd;
	buffer[1] = *data;
	buffer[2] = *(data+1);
	ret = I2C_TxData(SlaveAddress, buffer, length);
	if (ret < 0) {
		pr_err("[PS_ERR][cm3629 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}
#if 0
static int _cm3629_I2C_Read_Byte(uint16_t slaveAddr, uint8_t *pdata)
{
	uint8_t buffer = 0;
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;
	buffer = *pdata;
	ret = I2C_RxData(slaveAddr, &buffer, 1);
	if (ret < 0) {
		pr_err(
			"[CM3629_ error]%s: I2C_RxData fail, slave addr: 0x%x\n",
			__func__, slaveAddr);
		return ret;
	}

	*pdata = buffer;
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM3629_] %s: I2C_RxData[0x%x] = 0x%x\n",
		__func__, slaveAddr, buffer);
#endif
	return ret;
}
static int _cm3629_I2C_Write_Byte(uint16_t SlaveAddress,
				uint8_t cmd, uint8_t data)
{
	char buffer[2];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[cm3629] %s: _cm3629_I2C_Write_Byte[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = data;
	ret = I2C_TxData(SlaveAddress, buffer, 2);
	if (ret < 0) {
		pr_err("[PS_ERR][cm3629 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}
#endif
static int get_ls_adc_value(uint32_t *als_step, bool resume)
{

	struct cm3629_info *lpi = lp_info;
	uint8_t	lsb, msb;
	int ret = 0;
	char cmd[3];
	char ls_cmd;

	if (als_step == NULL)
		return -EFAULT;

	if (resume) {
		if (sensor_chipId[0] != 0x29)
			ls_cmd = (CM3629_ALS_IT_80ms | CM3629_ALS_PERS_1);/* disable CM3629_ALS_INT_EN */
		else
			ls_cmd = (CM3629_ALS_IT_50ms | CM3629_ALS_PERS_1);/* disable CM3629_ALS_INT_EN */
		D("[LS][cm3629] %s:resume %d\n",
		__func__, resume);
	} else
		ls_cmd = (lpi->ls_cmd);/* disable CM3629_ALS_INT_EN */

	cmd[0] = ls_cmd;
	cmd[1] = 0;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_config_cmd, cmd, 3);

	if (ret < 0) {
		pr_err(
			"[LS][cm3629 error]%s: _cm3629_I2C_Write_Byte fail\n",
			__func__);
		return -EIO;
	}

	/* Read ALS data */

	ret = _cm3629_I2C_Read2(lpi->cm3629_slave_address, ALS_data, cmd, 2);
	if (ret < 0) {
		pr_err(
			"[LS][cm3629 error]%s: _cm3629_I2C_Read_Byte  fail\n",
			__func__);
		return -EIO;
	}
	lsb = cmd[0];
	msb = cmd[1];

	*als_step = (uint32_t)msb;
	*als_step <<= 8;
	*als_step |= (uint32_t)lsb;

	D("[LS][cm3629] %s: raw adc = 0x%X, ls_calibrate = %d\n",
		__func__, *als_step, lpi->ls_calibrate);


	if (!lpi->ls_calibrate) {
		*als_step = (*als_step) * lpi->als_gadc / lpi->als_kadc;
		if (*als_step > 0xFFFF)
			*als_step = 0xFFFF;
	}


	return ret;
}

static int get_ps_adc_value(uint8_t *ps1_adc, uint8_t *ps2_adc)
{
	int ret = 0;
	struct cm3629_info *lpi = lp_info;
	char cmd[3];

	if (ps1_adc == NULL || ps2_adc == NULL)
		return -EFAULT;

	ret = _cm3629_I2C_Read2(lpi->cm3629_slave_address, PS_data, cmd, 2);
	if (ret < 0) {
		pr_err("[PS_ERR][cm3629 error] %s: _cm3629_I2C_Read_Byte "
		       "MSB fail\n", __func__);
		return -EIO;
	}

	*ps1_adc = cmd[0];
	*ps2_adc = cmd[1];
/*
	pr_info("[PS][cm3629] %s: PS1_ADC = 0x%02X, PS2_ADC = 0x%02X\n",
		__func__, *ps1_adc, *ps2_adc);
*/
	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm3629_info *lpi = lp_info;
	char cmd[3] = {0};

	uint8_t	high_msb;
	uint8_t	high_lsb;
	uint8_t	low_msb;
	uint8_t	low_lsb;
	D("[cm3629] %s: low_thd = 0x%X, high_thd = 0x%x \n",
		__func__, low_thd, high_thd);
	high_msb = (uint8_t) (high_thd >> 8);
	high_lsb = (uint8_t) (high_thd & 0x00ff);
	low_msb	 = (uint8_t) (low_thd >> 8);
	low_lsb	 = (uint8_t) (low_thd & 0x00ff);

	cmd[0] = high_lsb;
	cmd[1] = high_msb;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_high_thd, cmd, 3);

	cmd[0] = low_lsb;
	cmd[1] = low_msb;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_low_thd, cmd, 3);
	return ret;
}

static void report_near_do_work(struct work_struct *w)
{
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629]  %s: delay 500ms, report proximity NEAR\n", __func__);
	if (lpi->phantom_gpio > 0)
		gpio_set_value(lpi->phantom_gpio, 0);
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);
	input_sync(lpi->ps_input_dev);
	blocking_notifier_call_chain(&psensor_notifier_list, 2, NULL);
}

static void report_psensor_input_event(struct cm3629_info *lpi, int interrupt_flag)
{
	uint8_t ps_thd_set = 0;
	uint8_t ps_adc = 0;
	uint8_t ps1_adc = 0;
	uint8_t ps2_adc = 0;
	int val, ret = 0;

	if (interrupt_flag > 1 && lpi->ps_enable == 0) {
		D("[PS][cm3629] P-sensor disable but intrrupt occur, "
		  "record_init_fail %d.\n", record_init_fail);
		return;
	}

	if (lpi->ps_debounce == 1 && lpi->mfg_mode != NO_IGNORE_BOOT_MODE)
		cancel_delayed_work(&report_near_work);

	lpi->j_end = jiffies;
	/* D("%s: j_end = %lu", __func__, lpi->j_end); */

	ret = get_ps_adc_value(&ps1_adc, &ps2_adc);

	if (lpi->ps_select == CM3629_PS2_ONLY) {
		ps_thd_set = lpi->ps2_thd_set + 1;
		ps_adc = ps2_adc;
	} else {
		if (lpi->ps1_thh_diff == 0)
			ps_thd_set = lpi->ps1_thd_set + 1;
		else
			ps_thd_set = lpi->ps1_thd_set + lpi->ps1_thh_diff;
		ps_adc = ps1_adc;
	}
	if (interrupt_flag == 0) {/*interrupt_fla = 0 meam polling mode*/
		if (ret == 0) {
			val = (ps_adc >= ps_thd_set) ? 0 : 1;
		} else {/*i2c err, report far to workaround*/
			val = 1;
			ps_adc = 0;
			D("[PS][cm3629] proximity i2c err, report %s, "
			  "ps_adc=%d, record_init_fail %d\n",
			  val ? "FAR" : "NEAR", ps_adc, record_init_fail);
		}
	} else {/*interrupt_fla = 2 meam close isr,  interrupt_fla = 1 mean far isr*/
		val = (interrupt_flag == 2) ? 0 : 1;
	}

	if (lpi->ps_debounce == 1 && lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
		if (val == 0) {
			D("[PS][cm3629] delay proximity %s, ps_adc=%d, High thd= %d, interrupt_flag %d\n",
			  val ? "FAR" : "NEAR", ps_adc, ps_thd_set, interrupt_flag);
			queue_delayed_work(lpi->lp_wq, &report_near_work,
					msecs_to_jiffies(lpi->ps_delay_time));
			return;
		} else {
			/* dummy report */
			input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
			input_sync(lpi->ps_input_dev);
		}
	}
	D("[PS][cm3629] proximity %s, ps_adc=%d, , High thd= %d, interrupt_flag %d\n",
	  val ? "FAR" : "NEAR", ps_adc, ps_thd_set, interrupt_flag);
	if ((lpi->enable_polling_ignore == 1) && (val == 0) &&
		(lpi->mfg_mode != NO_IGNORE_BOOT_MODE) &&
	    (time_before(lpi->j_end, (lpi->j_start + NEAR_DELAY_TIME)))) {
		D("[PS][cm3629] Ignore NEAR event\n");
		lpi->ps_pocket_mode = 1;
	} else {
		/* 0 is close, 1 is far */
		input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
		if (lpi->phantom_gpio > 0)
			gpio_set_value(lpi->phantom_gpio, val);
		input_sync(lpi->ps_input_dev);
		blocking_notifier_call_chain(&psensor_notifier_list, val+2, NULL);
	}
}

static void enable_als_interrupt(void)/*enable als interrupt*/
{
	char cmd[3];
	struct cm3629_info *lpi = lp_info;
	int ret = 0;

	cmd[0] = (lpi->ls_cmd | CM3629_ALS_INT_EN);
	cmd[1] = 0;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_config_cmd, cmd, 3);
	if (ret != 0) {
		lpi->als_enable = 0;
		D("[LS][cm3629] L-sensor i2c err, enable interrupt error\n");
	} else
		lpi->als_enable = 1;
}

static void report_lsensor_input_event(struct cm3629_info *lpi, bool resume)
{/*when resume need report a data, so the paramerter need to quick reponse*/
	uint32_t adc_value = 0;
	int level = 0, i, ret = 0;

	mutex_lock(&als_get_adc_mutex);

	ret = get_ls_adc_value(&adc_value, resume);
	if (resume) {
		if (sensor_chipId[0] != 0x29)
			adc_value = adc_value*4;/*because the cm3629_ALS_IT for 320ms - >80ms*/
		else
			adc_value = adc_value*8;/*because the cm3629_ALS_IT for 400ms - >50ms*/
	}
	for (i = 0; i < 10; i++) {
		if (adc_value <= (*(lpi->adc_table + i))) {
			level = i;
			if (*(lpi->adc_table + i))
				break;
		}
		if (i == 9) {/*avoid  i = 10, because 'cali_table' of size is 10 */
			level = i;
			break;
		}
	}
	ret = set_lsensor_range(((i == 0) || (adc_value == 0)) ? 0 :
			*(lpi->cali_table + (i - 1)) + 1,
		*(lpi->cali_table + i));

	if (ret < 0)
		printk(KERN_ERR "[LS][cm3629 error] %s fail\n", __func__);

	if ((i == 0) || (adc_value == 0))
		D("[LS][cm3629] %s: ADC=0x%03X, Level=%d, l_thd equal 0, h_thd = 0x%x \n",
			__func__, adc_value, level, *(lpi->cali_table + i));
	else
		D("[LS][cm3629] %s: ADC=0x%03X, Level=%d, l_thd = 0x%x, h_thd = 0x%x \n",
			__func__, adc_value, level, *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));

	lpi->current_level = level;
	lpi->current_adc = adc_value;
	/*D("[cm3629] %s: *(lpi->cali_table + (i - 1)) + 1 = 0x%X, *(lpi->cali_table + i) = 0x%x \n", __func__, *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));*/
	if (f_cm3629_level >= 0) {
		D("[LS][cm3629] L-sensor force level enable level=%d f_cm3629_level=%d\n", level, f_cm3629_level);
		level = f_cm3629_level;
	}
	input_report_abs(lpi->ls_input_dev, ABS_MISC, level);
	input_sync(lpi->ls_input_dev);
	enable_als_interrupt();
	mutex_unlock(&als_get_adc_mutex);

}

static void enable_ps_interrupt(char *ps_conf)
{
	struct cm3629_info *lpi = lp_info;
	int ret;
	char cmd[2] = {0};

	lpi->ps_enable = 1;

	cmd[0] = lpi->ps1_thd_set;
	if (lpi->ps1_thh_diff == 0)
		cmd[1] = lpi->ps1_thd_set + 1;
	else
		cmd[1] = lpi->ps1_thd_set + lpi->ps1_thh_diff;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		PS_1_thd, cmd, 3);

	cmd[0] = lpi->ps2_thd_set;
	cmd[1] = lpi->ps2_thd_set + 1;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		PS_2_thd, cmd, 3);

	cmd[0] = ps_conf[2];
	cmd[1] = CM3629_PS_255_STEPS;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
				 PS_config_ms, cmd, 3);

	cmd[0] = ps_conf[0];/*power on at last step*/
	cmd[1] = ps_conf[1];
	D("[PS][cm3629] %s, write cmd[0] = 0x%x, cmd[1] = 0x%x\n",
		__func__, cmd[0], cmd[1]);
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address, PS_config, cmd, 3);

	if (ret != 0) {
		lpi->ps_enable = 0;
		D("[PS][cm3629] P-sensor i2c err, enable interrupt error\n");
	}
	_cm3629_I2C_Read2(lpi->cm3629_slave_address, PS_config, cmd, 2);
	D("[PS][cm3629] %s, read value => cmd[0] = 0x%x, cmd[1] = 0x%x\n", __func__, cmd[0], cmd[1]);
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm3629_info *lpi = lp_info;
	uint8_t cmd[3];
	uint8_t add = 0;

	/* Check ALS or PS */
	wake_lock_timeout(&(lpi->ps_wake_lock), 3*HZ);
	_cm3629_I2C_Read2(lpi->cm3629_slave_address, INT_FLAG, cmd, 2);
	add = cmd[1];
	/* D("[cm3629] %s:, INTERRUPT = 0x%x \n", __func__, add); */
	if ((add & CM3629_PS1_IF_AWAY) || (add & CM3629_PS1_IF_CLOSE) ||
	    (add & CM3629_PS2_IF_AWAY) || (add & CM3629_PS2_IF_CLOSE)) {
		if ((add & CM3629_PS1_IF_AWAY) || (add & CM3629_PS2_IF_AWAY))
			report_psensor_input_event(lpi, 1);/*1 meam far*/
		else
			report_psensor_input_event(lpi, 2);/*2 meam close*/
	} else if (((add & CM3629_ALS_IF_L) == CM3629_ALS_IF_L) ||
		     ((add & CM3629_ALS_IF_H) == CM3629_ALS_IF_H))
		report_lsensor_input_event(lpi, 0);
	 else
		pr_err("[PS][cm3629 error]%s error: unkown interrupt: 0x%x!\n",
		__func__, add);

	enable_irq(lpi->irq);
}

#ifdef POLLING_PROXIMITY
static uint8_t mid_value(uint8_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++)
		for (j = (i + 1); j < size; j++)
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint8_t *ps_adc1, uint8_t *ps_adc2)
{
	int ret = 0;
	int i = 0;
	uint8_t mid_adc1 = 0;
	uint8_t mid_adc2 = 0;
	uint8_t adc1[3] = {0, 0, 0};
	uint8_t adc2[3] = {0, 0, 0};

	for (i = 0; i < 3; i++) {
		ret = get_ps_adc_value(&adc1[i], &adc2[i]);
		if (ret < 0) {
			pr_err("[PS_ERR][cm3629 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}
	}

/*
	D(" Before sort: adc1[] = [%d, %d, %d], adc2[] = [%d, %d, %d]\n",
	  adc1[0], adc1[1], adc1[2], adc2[0], adc2[1], adc2[2]);
*/
	mid_adc1 = mid_value(adc1, 3);
	mid_adc2 = mid_value(adc2, 3);
/*
	D("After sort: adc1[] = [%d, %d, %d], adc2[] = [%d, %d, %d]\n",
	  adc1[0], adc1[1], adc1[2], adc2[0], adc2[1], adc2[2]);
*/

	*ps_adc1 = mid_adc1;
	*ps_adc2 = mid_adc2;

	return 0;
}

static void polling_do_work(struct work_struct *w)
{
	struct cm3629_info *lpi = lp_info;
	uint8_t ps_adc1 = 0;
	uint8_t ps_adc2 = 0;
	int i = 0;
	int ret = 0;
	char cmd[3];

	/*D("lpi->ps_enable = %d\n", lpi->ps_enable);*/
	if (lpi->ps_enable == 0)
		return;

	ret = get_stable_ps_adc_value(&ps_adc1, &ps_adc2);
/*
	D("[cm3629] Polling: ps_adc1 = 0x%02X, ps_adc2 = 0x%02X, "
	  "ps_next_base_value = 0x%02X, ps1_thd_set = 0x%02X\n",
	  ps_adc1, ps_adc2, lpi->mapping_table[lpi->ps_base_index],
	  lpi->ps1_thd_set);
*/

	if ((ps_adc1 == 0) || (ret < 0)) {
		queue_delayed_work(lpi->lp_wq, &polling_work,
			msecs_to_jiffies(POLLING_DELAY));
		return;
	}

	for (i = lpi->ps_base_index; i >= 1; i--) {
		if (ps_adc1 > lpi->mapping_table[i])
			break;
		else if ((ps_adc1 > lpi->mapping_table[(i-1)]) &&
		    (ps_adc1 <= lpi->mapping_table[i])) {
			lpi->ps_base_index = (i-1);

			if (i == (lpi->mapping_size - 1))
				lpi->ps1_thd_set = 0xFF;
			else
				lpi->ps1_thd_set = (lpi->mapping_table[i] +
						   TH_ADD);

			/* settng command code(0x01) = 0x03*/
			cmd[0] = lpi->ps1_thd_set;
			if (lpi->ps1_thh_diff == 0)
				cmd[1] = lpi->ps1_thd_set + 1;
			else
				cmd[1] = lpi->ps1_thd_set + lpi->ps1_thh_diff;
			_cm3629_I2C_Write2(lpi->cm3629_slave_address,
				PS_1_thd, cmd, 3);
			D("[PS][cm3629] SET THD: lpi->ps1_thd_set = %d\n",
				lpi->ps1_thd_set);
			break;
		}
	}

	queue_delayed_work(lpi->lp_wq, &polling_work,
		msecs_to_jiffies(POLLING_DELAY));
}
#endif

static irqreturn_t cm3629_irq_handler(int irq, void *data)
{
	struct cm3629_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	if (enable_cm3629_log)
		D("[PS][cm3629] %s\n", __func__);

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm3629_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, enable);

	return 0;
}

static void ls_initial_cmd(struct cm3629_info *lpi)
{
	char cmd[3] = {0};

	cmd[0] = (lpi->ls_cmd | CM3629_ALS_SD);
	cmd[1] = 0;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_config_cmd, cmd, 3);

	_cm3629_I2C_Read2(lpi->cm3629_slave_address, ALS_config_cmd, cmd, 2);
	D("[LS][cm3629] %s, cmd[0] = 0x%x, cmd[1] = 0x%x\n", __func__, cmd[0], cmd[1]);
}

static void psensor_intelligent_cancel_cmd(struct cm3629_info *lpi)
{
	char cmd[2] = {0};

	cmd[0] = lpi->inte_ps1_canc;
	cmd[1] = lpi->inte_ps2_canc;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address, PS_CANC, cmd, 3);
}

static void psensor_initial_cmd(struct cm3629_info *lpi)
{
	char cmd[2] = {0};

	cmd[0] = lpi->ps_conf1_val;
	cmd[1] = lpi->ps_conf2_val;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address, PS_config, cmd, 3);

	cmd[0] = lpi->ps_conf3_val;
	cmd[1] = CM3629_PS_255_STEPS;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address, PS_config_ms, cmd, 3);

	cmd[0] = lpi->ps1_thd_set;
	if (lpi->ps1_thh_diff == 0)
		cmd[1] = lpi->ps1_thd_set + 1;
	else
		cmd[1] = lpi->ps1_thd_set + lpi->ps1_thh_diff;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		PS_1_thd, cmd, 3);

	cmd[0] = lpi->ps2_thd_set;
	cmd[1] = lpi->ps2_thd_set + 1;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		PS_2_thd, cmd, 3);

	psensor_intelligent_cancel_cmd(lpi);

	D("[PS][cm3629] %s, finish\n", __func__);
}

static int psensor_enable(struct cm3629_info *lpi)
{
	int ret;
	char ps_conf[3];
	char cmd[2];
#ifdef POLLING_PROXIMITY
	uint8_t ps_adc1 = 0;
	uint8_t ps_adc2 = 0;
#endif

	D("[PS][cm3629] %s\n", __func__);
	if (lpi->ps_enable) {
		D("[PS][cm3629] %s: already enabled\n", __func__);
		return 0;
	}
	if (lpi->phantom_gpio > 0)
		gpio_set_value(lpi->phantom_gpio, 1);
	blocking_notifier_call_chain(&psensor_notifier_list, 1, NULL);
	lpi->j_start = jiffies;
	/*D("%s: j_start = %lu", __func__, lpi->j_start);*/

	/* dummy report */
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
	input_sync(lpi->ps_input_dev);

	psensor_initial_cmd(lpi);

	if (lpi->enable_polling_ignore == 1 &&
		lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
		/* default report FAR */
		input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 1);
		input_sync(lpi->ps_input_dev);
		blocking_notifier_call_chain(&psensor_notifier_list, 1+2, NULL);
	} else
		report_psensor_input_event(lpi, 0);

	cmd[0] = lpi->ps_conf1_val | CM3629_PS1_SD | CM3629_PS2_SD;
	cmd[1] = lpi->ps_conf2_val;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
				PS_config, cmd, 3);

	psensor_intelligent_cancel_cmd(lpi);

	ps_conf[0] = lpi->ps_conf1_val;
	ps_conf[1] = lpi->ps_conf2_val;
	ps_conf[2] = lpi->ps_conf3_val;

	if (lpi->ps_select == CM3629_PS2_ONLY)
		ps_conf[1] = (lpi->ps_conf2_val | CM3629_PS2_INT_BOTH);
	else if (lpi->ps_select == CM3629_PS1_ONLY)
		ps_conf[1] = (lpi->ps_conf2_val | CM3629_PS1_INT_BOTH);

	enable_ps_interrupt(ps_conf);

	ret = irq_set_irq_wake(lpi->irq, 1);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: fail to enable irq %d as wake interrupt\n",
			__func__, lpi->irq);
		return ret;
	}

#ifdef POLLING_PROXIMITY
	if (lpi->enable_polling_ignore == 1) {
		if (lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
			ret = get_stable_ps_adc_value(&ps_adc1, &ps_adc2);
			D("[PS][cm3629] INITIAL ps_adc1 = 0x%02X", ps_adc1);
			if ((ret == 0) && (lpi->mapping_table != NULL) &&
			    ((ps_adc1 >= lpi->ps1_thd_set - 1)))
				queue_delayed_work(lpi->lp_wq, &polling_work,
					msecs_to_jiffies(POLLING_DELAY));
		}
	}
#endif
	return ret;
}

static int psensor_disable(struct cm3629_info *lpi)
{
	int ret = -EIO;
	char cmd[2];

	lpi->ps_pocket_mode = 0;

	D("[PS][cm3629] %s\n", __func__);
	if (!lpi->ps_enable) {
		D("[PS][cm3629] %s: already disabled\n", __func__);
		return 0;
	}

	ret = irq_set_irq_wake(lpi->irq, 0);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: fail to disable irq %d as wake interrupt\n",
			__func__, lpi->irq);
		return ret;
	}

	cmd[0] = lpi->ps_conf1_val | CM3629_PS1_SD | CM3629_PS2_SD;
	cmd[1] = lpi->ps_conf2_val;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
				PS_config, cmd, 3);
	if (ret < 0) {
		pr_err("[PS][cm3629 error]%s: disable psensor fail\n", __func__);
		return ret;
	}

	cmd[0] = lpi->ps_conf3_val;
	cmd[1] = CM3629_PS_255_STEPS;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
				PS_config_ms, cmd, 3);

	blocking_notifier_call_chain(&psensor_notifier_list, 0, NULL);
	lpi->ps_enable = 0;

#ifdef POLLING_PROXIMITY
	if (lpi->enable_polling_ignore == 1 && lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
		cancel_delayed_work(&polling_work);
		lpi->ps_base_index = (lpi->mapping_size - 1);

		lpi->ps1_thd_set = lpi->original_ps_thd_set;
		/* settng command code(0x01) = 0x03*/
		cmd[0] = lpi->ps1_thd_set;
		if (lpi->ps1_thh_diff == 0)
			cmd[1] = lpi->ps1_thd_set + 1;
		else
			cmd[1] = lpi->ps1_thd_set + lpi->ps1_thh_diff;
		_cm3629_I2C_Write2(lpi->cm3629_slave_address,
			PS_1_thd, cmd, 3);
	}
#endif
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629] %s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629] %s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val)
			return psensor_enable(lpi);
		else
			return psensor_disable(lpi);
		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[PS][cm3629 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

static struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};

static void lightsensor_set_kvalue(struct cm3629_info *lpi)
{
	if (!lpi) {
		pr_err("[LS][cm3629 error]%s: ls_info is empty\n", __func__);
		return;
	}

	D("[LS][cm3629] %s: ALS calibrated als_kadc=0x%x\n",
			__func__, als_kadc);

	if (als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		D("[LS][cm3629] %s: no ALS calibrated\n", __func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0) ?
				lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}
	D("[LS][cm3629] %s: als_kadc=0x%x, als_gadc=0x%x\n",
		__func__, lpi->als_kadc, lpi->als_gadc);
}
static void psensor_set_THH(struct cm3629_info *lpi, uint8_t ps1_thd_set)
{
	char cmd[2] = {0};

	if (ps1_thd_set < (lpi->ps1_thh_diff + 1)) {
		D("[PS][cm3629 Error]%s:calibration value(THH) = %d < %d, it is too small\n",
		 __func__, ps1_thd_set, (lpi->ps1_thh_diff + 1));
		lpi->ps1_thd_set = lpi->ps1_thd_no_cal;
		lpi->ps2_thd_set = lpi->ps2_thd_no_cal;
	} else {
		lpi->ps1_thd_set = ps1_thd_set - lpi->ps1_thh_diff;
	}
	cmd[0] = lpi->ps1_thd_set;
	if (lpi->ps1_thh_diff == 0)
		cmd[1] = lpi->ps1_thd_set + 1;
	else
		cmd[1] = lpi->ps1_thd_set + lpi->ps1_thh_diff;

	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
	PS_1_thd, cmd, 3);
	D("[PS][cm3629]%s:P-sensor calibration value(THH) = %d\n",
		__func__, ps1_thd_set);
}
static void psensor_set_kvalue(struct cm3629_info *lpi)
{
	uint8_t ps_conf1_val;

	D("[PS][cm3629] %s: PS calibrated ps_kparam1 = 0x%04X, "
	  "ps_kparam2 = 0x%04X\n", __func__, ps_kparam1, ps_kparam2);
	ps_conf1_val = lpi->ps_conf1_val;
	/* Only use ps_kparam2 for CM36282/CM36292 */
	if (ps_kparam1 >> 16 == PS_CALIBRATED) {
		if (lpi->ps_calibration_rule != 5) {
			lpi->inte_ps1_canc = (uint8_t) (ps_kparam2 & 0xFF);
			lpi->inte_ps2_canc = (uint8_t) ((ps_kparam2 >> 8) & 0xFF);
		} else {
			psensor_set_THH(lpi, (uint8_t) (ps_kparam2 & 0xFF));
		}
		if (lpi->ps_calibration_rule == 3) {/*for vigor*/

			if ((lpi->ps_conf1_val & CM3629_PS_IT_1_6T) == CM3629_PS_IT_1_6T) {
				D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1_6T\n",
				  __func__, lpi->ps_conf1_val);
				lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1_6T));
			} else if ((lpi->ps_conf1_val & CM3629_PS_IT_1_3T) == CM3629_PS_IT_1_3T) {
				D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1_3T\n",
				  __func__, lpi->ps_conf1_val);
				lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1_3T));
			} else if ((lpi->ps_conf1_val & CM3629_PS_IT_1T) == CM3629_PS_IT_1T) {
				D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1T\n",
				  __func__, lpi->ps_conf1_val);
				lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1T));
			} else
				D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  2T or unknown\n",
				  __func__, lpi->ps_conf1_val);

			D("[PS][cm3629] %s: clear  lpi->ps_conf1_val  0x%x\n",
				  __func__, lpi->ps_conf1_val);

			if (((ps_kparam2 >> 16) & 0xFF) == 0)
				lpi->ps_conf1_val = (lpi->ps_conf1_val | CM3629_PS_IT_1T);
			else if (((ps_kparam2 >> 16) & 0xFF) == 0x1)
				lpi->ps_conf1_val = (lpi->ps_conf1_val | CM3629_PS_IT_1_3T);
			else if (((ps_kparam2 >> 16) & 0xFF) == 0x2)
				lpi->ps_conf1_val = (lpi->ps_conf1_val | CM3629_PS_IT_1_6T);
			else {
				lpi->ps_conf1_val = ps_conf1_val;
				D("[PS]%s: ((ps_kparam2 >> 16) & 0xFF)  = 0x%X is strange\n",
				__func__, ((ps_kparam2 >> 16) & 0xFF));
			}
			D("[PS][cm3629] %s:   lpi->ps_conf1_val  0x%x\n",
			  __func__, lpi->ps_conf1_val);
		}
		if (lpi->ps_calibration_rule != 5) {
			D("[PS][cm3629] %s: PS calibrated inte_ps1_canc = 0x%02X, "
			  "inte_ps2_canc = 0x%02X, ((ps_kparam2 >> 16) & 0xFF) = 0x%X\n", __func__,
			  lpi->inte_ps1_canc, lpi->inte_ps2_canc, ((ps_kparam2 >> 16) & 0xFF));
		}
	} else {
		if (lpi->ps_calibration_rule >= 1) {
			lpi->ps1_thd_set = lpi->ps1_thd_no_cal;
			lpi->ps2_thd_set = lpi->ps2_thd_no_cal;
			D("[PS][cm3629] %s: PS1_THD=%d, PS2_THD=%d, "
			  "no calibration\n", __func__,
			  lpi->ps1_thd_set, lpi->ps2_thd_set);
		}
		D("[PS][cm3629] %s: Proximity NOT calibrated\n", __func__);
	}

}

static int lightsensor_update_table(struct cm3629_info *lpi)
{
	uint16_t data[10];
	int i;
	for (i = 0; i < 10; i++) {
		if (*(lpi->adc_table + i) < 0xFFFF) {
			data[i] = *(lpi->adc_table + i)
					* lpi->als_kadc / lpi->als_gadc;
		} else {
			data[i] = *(lpi->adc_table + i);
		}
		D("[LS][cm3629] %s: Calibrated adc_table: data[%d], %x\n",
			__func__, i, data[i]);
	}
	memcpy(lpi->cali_table, data, 20);
	return 0;
}

static int lightsensor_enable(struct cm3629_info *lpi)
{
	int ret = 0;
	char cmd[3] = {0};

	mutex_lock(&als_enable_mutex);
	D("[LS][cm3629] %s\n", __func__);

	if (sensor_chipId[0] != 0x29)
		cmd[0] = (CM3629_ALS_IT_80ms | CM3629_ALS_PERS_1);/* disable CM3629_ALS_INT_EN */
	else
		cmd[0] = (CM3629_ALS_IT_50ms | CM3629_ALS_PERS_1);

	cmd[1] = 0;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_config_cmd, cmd, 3);
	if (ret < 0)
		pr_err(
		"[LS][cm3629 error]%s: set auto light sensor fail\n",
		__func__);
	else {
		if (lpi->mfg_mode != NO_IGNORE_BOOT_MODE)
			msleep(160);
		else
			msleep(85);

		input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
		input_sync(lpi->ls_input_dev);
		report_lsensor_input_event(lpi, 1);
	}

	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm3629_info *lpi)
{
	int ret = 0;
	char cmd[3] = {0};
	mutex_lock(&als_disable_mutex);

	D("[LS][cm3629] %s\n", __func__);

	cmd[0] = lpi->ls_cmd | CM3629_ALS_SD;
	cmd[1] = 0;
	ret = _cm3629_I2C_Write2(lpi->cm3629_slave_address,
		ALS_config_cmd, cmd, 3);

	if (ret < 0)
		pr_err("[LS][cm3629 error]%s: disable auto light sensor fail\n",
			__func__);
	else
		lpi->als_enable = 0;

	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm3629_info *lpi = lp_info;
	int rc = 0;

	D("[LS][cm3629] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][cm3629 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm3629_info *lpi = lp_info;

	D("[LS][cm3629] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm3629_info *lpi = lp_info;

	/*D("[cm3629] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][cm3629] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][cm3629] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LS][cm3629 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};


static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint8_t ps_adc1 = 0;
	uint8_t ps_adc2 = 0;
	int ret;
	struct cm3629_info *lpi = lp_info;
	int int_gpio;

	int_gpio = gpio_get_value(lpi->intr_pin);
	get_ps_adc_value(&ps_adc1, &ps_adc2);

	if (lpi->ps_calibration_rule == 1) {
		D("[PS][cm3629] %s: PS1_ADC=0x%02X, PS2_ADC=0x%02X, "
		  "PS1_Offset=0x%02X, PS2_Offset=0x%02X\n", __func__,
		  ps_adc1, ps_adc2, lpi->ps1_adc_offset, lpi->ps2_adc_offset);
		ps_adc1 = (ps_adc1 >= lpi->ps1_adc_offset) ?
			  ps_adc1 - lpi->ps1_adc_offset : 0;
		ps_adc2 = (ps_adc2 >= lpi->ps2_adc_offset) ?
			  ps_adc2 - lpi->ps2_adc_offset : 0;
	}

	ret = sprintf(buf, "ADC[0x%02X], ENABLE = %d, intr_pin = %d, "
		      "ps_pocket_mode = %d, model = %s, ADC2[0x%02X]\n",
		      ps_adc1, lpi->ps_enable, int_gpio, lpi->ps_pocket_mode,
		      (lpi->model == CAPELLA_CM36282) ? "CM36282" : "CM36292",
		      ps_adc2);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct cm3629_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
		&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	if (lpi->ps_calibration_rule == 3 &&
		(ps_en == 10 || ps_en == 13 || ps_en == 16)) {/*for vigor*/
		if ((lpi->ps_conf1_val & CM3629_PS_IT_1_6T) == CM3629_PS_IT_1_6T) {
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1_6T\n",
			  __func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1_6T));
		} else if ((lpi->ps_conf1_val & CM3629_PS_IT_1_3T) == CM3629_PS_IT_1_3T) {
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1_3T\n",
			  __func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1_3T));
		} else if ((lpi->ps_conf1_val & CM3629_PS_IT_1T) == CM3629_PS_IT_1T) {
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1T\n",
			  __func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1T));
		} else
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  2T or unknown\n",
			  __func__, lpi->ps_conf1_val);

		D("[PS][cm3629] %s: clear  lpi->ps_conf1_val  0x%x\n",
			  __func__, lpi->ps_conf1_val);

		if (ps_en == 10)
			lpi->ps_conf1_val = lpi->ps_conf1_val | CM3629_PS_IT_1T;
		else if (ps_en == 13)
			lpi->ps_conf1_val = lpi->ps_conf1_val | CM3629_PS_IT_1_3T;
		else if (ps_en == 16)
			lpi->ps_conf1_val = lpi->ps_conf1_val | CM3629_PS_IT_1_6T;

		D("[PS][cm3629] %s: change lpi->ps_conf1_val  0x%x\n",
			  __func__, lpi->ps_conf1_val);
	}
	D("[PS][cm3629] %s: ps_en=%d\n",
			__func__, ps_en);

	if (ps_en)
		psensor_enable(lpi);
	else
		psensor_disable(lpi);

	return count;
}

static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

static ssize_t ps_kadc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3629_info *lpi = lp_info;

	if (ps_kparam1 >> 16 == PS_CALIBRATED) {
		if (lpi->ps_calibration_rule != 5)
			ret = sprintf(buf, "P-sensor calibrated,"
			      "INTE_PS1_CANC = (0x%02X), "
			      "INTE_PS2_CANC = (0x%02X)\n",
			      lpi->inte_ps1_canc, lpi->inte_ps2_canc);
		else
			ret = sprintf(buf, "P-sensor calibrated(only for PRO_C),"
			      "INTE_PS1_THH= (0x%02X), "
			      "INTE_PS1_THL = (0x%02X)\n",
			      lpi->ps1_thd_set + lpi->ps1_thh_diff, lpi->ps1_thd_set);

	} else
		ret = sprintf(buf, "P-sensor NOT calibrated,"
			      "INTE_PS1_CANC = (0x%02X), "
			      "INTE_PS2_CANC = (0x%02X)\n",
			      lpi->inte_ps1_canc, lpi->inte_ps2_canc);

	return ret;
}

static ssize_t ps_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int param1, param2;
	char ps_conf[3];
	struct cm3629_info *lpi = lp_info;
	uint8_t ps_conf1_val;

	sscanf(buf, "0x%x 0x%x", &param1, &param2);
	D("[PS][cm3629]%s: store value = 0x%X, 0x%X\n", __func__, param1, param2);
	ps_conf1_val = lpi->ps_conf1_val;
	if (lpi->ps_calibration_rule == 3) {/*for vigor*/

		if ((lpi->ps_conf1_val & CM3629_PS_IT_1_6T) == CM3629_PS_IT_1_6T) {
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1_6T\n",
			  __func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1_6T));
		} else if ((lpi->ps_conf1_val & CM3629_PS_IT_1_3T) == CM3629_PS_IT_1_3T) {
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1_3T\n",
			  __func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1_3T));
		} else if ((lpi->ps_conf1_val & CM3629_PS_IT_1T) == CM3629_PS_IT_1T) {
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  CM3629_PS_IT_1T\n",
			  __func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (lpi->ps_conf1_val & (~CM3629_PS_IT_1T));
		} else
			D("[PS][cm3629] %s: lpi->ps_conf1_val  0x%x,  2T or unknown\n",
			  __func__, lpi->ps_conf1_val);

		D("[PS][cm3629] %s: clear  lpi->ps_conf1_val  0x%x\n",
			  __func__, lpi->ps_conf1_val);

		if (((param2 >> 16) & 0xFF) == 0)
			lpi->ps_conf1_val = (lpi->ps_conf1_val | CM3629_PS_IT_1T);
		else if (((param2 >> 16) & 0xFF) == 0x1)
			lpi->ps_conf1_val = (lpi->ps_conf1_val | CM3629_PS_IT_1_3T);
		else if (((param2 >> 16) & 0xFF) == 0x2)
			lpi->ps_conf1_val = (lpi->ps_conf1_val | CM3629_PS_IT_1_6T);
		else {
			lpi->ps_conf1_val = ps_conf1_val;
			D("[PS]%s: ((param2 >> 16) & 0xFF)  = 0x%X is diffrent\n",
			__func__, ((param2 >> 16) & 0xFF));
		}
		D("[PS]%s: ((param2 >> 16) & 0xFF)  = 0x%X\n",
		 __func__, ((param2 >> 16) & 0xFF));
		D("[PS][cm3629] %s:   lpi->ps_conf1_val  0x%x\n",
			  __func__, lpi->ps_conf1_val);
	}
	if (lpi->ps_calibration_rule >= 1 && lpi->ps_calibration_rule != 5) {
		lpi->ps1_thd_set = lpi->ps1_thd_with_cal;
		lpi->ps2_thd_set = lpi->ps2_thd_with_cal;
		D("[PS][cm3629] %s: PS1_THD=%d, PS2_THD=%d, "
		  "after calibration\n", __func__,
		  lpi->ps1_thd_set, lpi->ps2_thd_set);
	} else if (lpi->ps_calibration_rule == 5) {
		psensor_set_THH(lpi, (param2 & 0xFF));
	}
	if (lpi->ps_enable) {
		ps_conf[0] = lpi->ps_conf1_val;
		ps_conf[1] = lpi->ps_conf2_val;
		ps_conf[2] = lpi->ps_conf3_val;

		if (lpi->ps_select == CM3629_PS2_ONLY)
			ps_conf[1] = (lpi->ps_conf2_val | CM3629_PS2_INT_BOTH);
		else if (lpi->ps_select == CM3629_PS1_ONLY)
			ps_conf[1] = (lpi->ps_conf2_val | CM3629_PS1_INT_BOTH);

		enable_ps_interrupt(ps_conf);
	}
	if (lpi->ps_calibration_rule != 5) {
		ps1_canc_set = lpi->inte_ps1_canc = (param2 & 0xFF);
		ps2_canc_set = lpi->inte_ps2_canc = ((param2 >> 8) & 0xFF);
		psensor_intelligent_cancel_cmd(lpi);
		D("[PS][cm3629]%s: inte_ps1_canc = 0x%02X, inte_ps2_canc = 0x%02X, lpi->ps_conf1_val  = 0x%02X\n",
			__func__, lpi->inte_ps1_canc, lpi->inte_ps2_canc, lpi->ps_conf1_val);
	}
	return count;
}

static DEVICE_ATTR(ps_kadc, 0664, ps_kadc_show, ps_kadc_store);


static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3629_info *lpi = lp_info;

	ret = sprintf(buf, "PS1_CANC = 0x%02X, PS2_CANC = 0x%02X\n",
		      lpi->inte_ps1_canc, lpi->inte_ps2_canc);

	return ret;
}
static ssize_t ps_canc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps1_canc = 0;
	int ps2_canc = 0;
	struct cm3629_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &ps1_canc, &ps2_canc);

	lpi->inte_ps1_canc = (uint8_t) ps1_canc;
	lpi->inte_ps2_canc = (uint8_t) ps2_canc;
	psensor_intelligent_cancel_cmd(lpi);

	D("[PS] %s: PS1_CANC = 0x%02X, PS2_CANC = 0x%02X\n",
	  __func__, lpi->inte_ps1_canc, lpi->inte_ps2_canc);

	return count;
}
static DEVICE_ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);

static ssize_t ps_i2c_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0, i;
	struct cm3629_info *lpi = lp_info;
	uint8_t chip_id[3] = {0};
	uint8_t data[26] = {0};

	for (i = 0; i <= CH_ID; i++) {
		_cm3629_I2C_Read2(lpi->cm3629_slave_address, ALS_config_cmd + i,
				  chip_id, 2);
		data[i*2] = chip_id[0];
		data[(i*2)+1] = chip_id[1];
	}
	ret = sprintf(buf,
		"0x0L=0x%02X, 0x0H=0x%02X, 0x1L=0x%02X, 0x1H=0x%02X, "
		"0x2L=0x%02X, 0x2H=0x%02X, 0x3L=0x%02X, 0x3H=0x%02X,\n"
		"0x4L=0x%02X, 0x4H=0x%02X, 0x5L=0x%02X, 0x5H=0x%02X, "
		"0x6L=0x%02X, 0x6H=0x%02X, 0x7L=0x%02X, 0x7H=0x%02X,\n"
		"0x8L=0x%02X, 0x8H=0x%02X, 0x9L=0x%02X, 0x9H=0x%02X, "
		"0xaL=0x%02X, 0xaH=0x%02X, 0xbL=0x%02X, 0xbH=0x%02X,\n"
		"0xcL=0x%02X, 0xcH=0x%02X.",
		data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7],
		data[8], data[9], data[10], data[11],
		data[12], data[13], data[14], data[15],
		data[16], data[17], data[18], data[19],
		data[20], data[21], data[22], data[23],
		data[24], data[25]);

	return ret;
}
static ssize_t ps_i2c_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm3629_info *lpi = lp_info;
	char *token[10];
	int i, ret = 0;
	uint8_t reg = 0, value[3] = {0}, read_value[3] = {0};

	printk(KERN_INFO "[CM3629_] %s\n", buf);
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");
	 ret = strict_strtoul(token[0], 16, (unsigned long *)&(reg));
	 ret = strict_strtoul(token[1], 16, (unsigned long *)&(value[0]));
	 ret = strict_strtoul(token[2], 16, (unsigned long *)&(value[1]));
	_cm3629_I2C_Write2(lpi->cm3629_slave_address,
		reg, value, 3);

	D("[CM3629] Set REG=0x%x, value[0]=0x%x, value[1]=0x%x\n",
	  reg, value[0], value[1]);
	_cm3629_I2C_Read2(lpi->cm3629_slave_address, reg, read_value, 2);

	D("[CM3629] Get REG=0x%x, value[0]=0x%x, value[1]=0x%x\n",
	  reg, read_value[0], read_value[1]);

	switch (reg) {
	case PS_1_thd:
		lpi->ps1_thd_set = value[0];
	case PS_2_thd:
		lpi->ps2_thd_set = value[0];
	default:
		D("[CM3629] NO parameter update for register 0x%02X\n", reg);
	}

	return ret;
}
static DEVICE_ATTR(ps_i2c, 0664, ps_i2c_show, ps_i2c_store);

static ssize_t ps_hw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3629_info *lpi = lp_info;

	ret = sprintf(buf, "PS: ps_conf1_val=0x%x, ps1_thd_set=0x%x, "
		"inte_ps1_canc=0x%02X, inte_ps2_canc=0x%02X, "
		"ps_conf2_val=0x%x, ps1_adc_offset=0x%02X, "
		"ps2_adc_offset=0x%02X, LS: ls_cmd=0x%x\n",
		lpi->ps_conf1_val, lpi->ps1_thd_set, lpi->inte_ps1_canc,
		lpi->inte_ps2_canc, lpi->ps_conf2_val,
		lpi->ps1_adc_offset, lpi->ps2_adc_offset, lpi->ls_cmd);

	return ret;
}
static ssize_t ps_hw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	struct cm3629_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS]%s: store value = 0x%x\n", __func__, code);
	if (code == 1) {
		lpi->inte_ps1_canc = 0;
		lpi->inte_ps2_canc = 0;
		lpi->ps1_adc_offset = 0;
		lpi->ps2_adc_offset = 0;
		psensor_intelligent_cancel_cmd(lpi);
		D("[PS]%s: Reset ps1_canc=%d, ps2_canc=%d, adc_offset1=%d, "
		  "adc_offset2=%d\n", __func__, lpi->inte_ps1_canc,
		  lpi->inte_ps2_canc, lpi->ps1_adc_offset, lpi->ps2_adc_offset);
	} else {
		lpi->inte_ps1_canc = ps1_canc_set;
		lpi->inte_ps2_canc = ps2_canc_set;
		lpi->ps1_adc_offset = ps1_offset_adc;
		lpi->ps2_adc_offset = ps2_offset_adc;
		psensor_intelligent_cancel_cmd(lpi);
		D("[PS]%s: Recover ps1_canc=%d, ps2_canc=%d, adc_offset1=%d, "
		  "adc_offset2=%d\n", __func__, lpi->inte_ps1_canc,
		  lpi->inte_ps2_canc, lpi->ps1_adc_offset, lpi->ps2_adc_offset);
	}

	return count;
}
static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm3629_info *lpi = lp_info;

	/*because 3628 is interrupt mode*/
	report_lsensor_input_event(lpi, 0);

	D("[LS][cm3629] %s: ADC = 0x%04X, Level = %d \n",
		__func__, lpi->current_adc, lpi->current_level);
	ret = sprintf(buf, "ADC[0x%04X] => level %d\n",
		lpi->current_adc, lpi->current_level);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	/*Todo: becase cm3629 registers can't be read, need to think ho w*/
	int ret = 0;
	struct cm3629_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm3629_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		ret = lightsensor_enable(lpi);
	} else {
		lpi->ls_calibrate = 0;
		ret = lightsensor_disable(lpi);
	}

	D("[LS][cm3629] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, lpi->ls_calibrate, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][cm3629 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0664,
	ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3629_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x, gadc = 0x%x, kadc while this boot"
			" = 0x%x\n",
			lpi->als_kadc, lpi->als_gadc, als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm3629_info *lpi = lp_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);
	if (kadc_temp <= 0 || lpi->golden_adc <= 0) {
		printk(KERN_ERR "[LS][cm3629 error] %s: kadc_temp=0x%x, als_gadc=0x%x\n",
			__func__, kadc_temp, lpi->golden_adc);
		return -EINVAL;
	}
	mutex_lock(&als_get_adc_mutex);
	lpi->als_kadc = kadc_temp;
	lpi->als_gadc = lpi->golden_adc;
	printk(KERN_INFO "[LS]%s: als_kadc=0x%x, als_gadc=0x%x\n",
			__func__, lpi->als_kadc, lpi->als_gadc);

	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "[LS][cm3629 error] %s: update ls table fail\n", __func__);
	mutex_unlock(&als_get_adc_mutex);
	return count;
}

static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);

static ssize_t ls_adc_table_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
			"[cm3629]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
			i, *(lp_info->adc_table + i),
			*(lp_info->adc_table + i),
			i, *(lp_info->cali_table + i),
			*(lp_info->cali_table + i));
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{

	struct cm3629_info *lpi = lp_info;
	char *token[10];
	unsigned long tempdata[10];
	int i, ret;

	printk(KERN_INFO "[LS][cm3629]%s\n", buf);
	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		ret = strict_strtoul(token[i], 16, &(tempdata[i]));
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			printk(KERN_ERR
			"[LS][cm3629 error] adc_table[%d] =  0x%lx Err\n",
			i, tempdata[i]);
			return count;
		}
	}
	mutex_lock(&als_get_adc_mutex);
	for (i = 0; i < 10; i++) {
		lpi->adc_table[i] = tempdata[i];
		printk(KERN_INFO
		"[LS][cm3629]Set lpi->adc_table[%d] =  0x%x\n",
		i, *(lp_info->adc_table + i));
	}
	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "[LS][cm3629 error] %s: update ls table fail\n",
		__func__);
	mutex_unlock(&als_get_adc_mutex);
	D("[LS][cm3629] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_adc_table, 0664,
	ls_adc_table_show, ls_adc_table_store);


static ssize_t ls_fLevel_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fLevel = %d\n", f_cm3629_level);
}
static ssize_t ls_fLevel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm3629_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "%d", &value);
	(value >= 0)?(value = min(value, 10)):(value = max(value, -1));
	f_cm3629_level = value;
	input_report_abs(lpi->ls_input_dev, ABS_MISC, f_cm3629_level);
	input_sync(lpi->ls_input_dev);
	printk(KERN_INFO "[LS]set fLevel = %d\n", f_cm3629_level);

	msleep(1000);
	f_cm3629_level = -1;
	return count;
}
static DEVICE_ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store);

static int lightsensor_setup(struct cm3629_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][cm3629 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][cm3629 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][cm3629 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct cm3629_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[PS][cm3629 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

static int cm3629_read_chip_id(struct cm3629_info *lpi)
{
	uint8_t chip_id[3] = {0};
	int ret = 0;

	als_power(0);
	msleep(5);
	als_power(1);
	msleep(5);

	ret = _cm3629_I2C_Read2(lpi->cm3629_slave_address, CH_ID, chip_id, 2);
	if (ret >= 0) {
		if ((chip_id[0] != 0x29) && (chip_id[0] != 0x92) && (chip_id[0] != 0x82) && (chip_id[0] != 0x83)) {
			ret = -1;
			D("[PS][cm3629] %s, chip_id  Err value = 0x%x, 0x%x, ret %d\n",
				__func__, chip_id[0], chip_id[1], ret);
		} else
			D("[PS][cm3629] %s, chip_id value = 0x%x, 0x%x, ret %d\n",
				__func__, chip_id[0], chip_id[1], ret);
	} else
		D("[PS][cm3629] %s, read chip_id i2c err ret %d\n",
				__func__, ret);
	sensor_chipId[0] = chip_id[0];

	return ret;
}
static int cm3629_setup(struct cm3629_info *lpi)
{
	int ret = 0;
	char cmd[3] = {0};

	ret = gpio_request(lpi->intr_pin, "gpio_cm3629_intr");
	if (ret < 0) {
		pr_err("[PS][cm3629 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	ls_initial_cmd(lpi);
	psensor_initial_cmd(lpi);

	/* Disable P-sensor by default */
	cmd[0] = lpi->ps_conf1_val | CM3629_PS1_SD | CM3629_PS2_SD;
	cmd[1] = lpi->ps_conf2_val;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address, PS_config, cmd, 3);

	cmd[0] = lpi->ps_conf3_val;
	cmd[1] = CM3629_PS_255_STEPS;
	_cm3629_I2C_Write2(lpi->cm3629_slave_address, PS_config_ms, cmd, 3);

	ret = request_any_context_irq(lpi->irq,
			cm3629_irq_handler,
			IRQF_TRIGGER_LOW,
			"cm3629",
			lpi);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

static void cm3629_early_suspend(struct early_suspend *h)
{
	struct cm3629_info *lpi = lp_info;

	D("[LS][cm3629] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
	else
		D("[LS][cm3629] %s: ioctl disable_lighsensor\n", __func__);

}

static void cm3629_late_resume(struct early_suspend *h)
{
	struct cm3629_info *lpi = lp_info;

	D("[LS][cm3629] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
	else
		D("[LS][cm3629] %s: ioctl enable_lighsensor\n", __func__);
}

static int cm3629_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3629_info *lpi;
	struct cm3629_platform_data *pdata;
	int irq = 0;

	D("[PS][cm3629] %s\n", __func__);


	lpi = kzalloc(sizeof(struct cm3629_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/*D("[cm3629] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[PS][cm3629 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}

	irq = gpio_to_irq(pdata->intr);/*for kernel 3.0*/

	lpi->irq = irq;
	pr_info("[PS][CM3629]%s: client->irq %d, irq %d!!\n",
			__func__, client->irq, irq);
	lpi->mfg_mode = board_mfg_mode();

	i2c_set_clientdata(client, lpi);
	lpi->model = pdata->model;
	lpi->intr_pin = pdata->intr;
	lpi->phantom_gpio = pdata->phantom_gpio;
	lpi->adc_table = pdata->levels;
	lpi->golden_adc = pdata->golden_adc;
	lpi->power = pdata->power;
	lpi->cm3629_slave_address = pdata->cm3629_slave_address;
	lpi->ps_select = pdata->ps_select;
	lpi->ps1_thd_set = pdata->ps1_thd_set;
	lpi->ps1_thh_diff = pdata->ps1_thh_diff;
	lpi->ps2_thd_set = pdata->ps2_thd_set;
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf2_val = pdata->ps_conf2_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	lpi->ps_calibration_rule = pdata->ps_calibration_rule;
	lpi->j_start = 0;
	lpi->j_end = 0;
	lpi->mapping_table = pdata->mapping_table;
	lpi->mapping_size = pdata->mapping_size;
	lpi->ps_base_index = (pdata->mapping_size - 1);
	lpi->enable_polling_ignore = pdata->enable_polling_ignore;
	lpi->ps1_thd_no_cal = pdata->ps1_thd_no_cal;
	lpi->ps1_thd_with_cal = pdata->ps1_thd_with_cal;
	lpi->ps2_thd_no_cal = pdata->ps2_thd_no_cal;
	lpi->ps2_thd_with_cal = pdata->ps2_thd_with_cal;
	lpi->ls_cmd  = pdata->ls_cmd;
	lpi->ps1_adc_offset = pdata->ps1_adc_offset;
	lpi->ps2_adc_offset = pdata->ps2_adc_offset;
	lpi->ps_debounce = pdata->ps_debounce;
	lpi->ps_delay_time = pdata->ps_delay_time;

	lp_info = lpi;

	ret = cm3629_read_chip_id(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][cm3629 error]%s: cm3629_read_chip_id error!\n", __func__);
		goto err_cm3629_read_chip_id;
	}

	if (pdata->ls_cmd == 0) {
		if (sensor_chipId[0] != 0x29)
			lpi->ls_cmd  = CM3629_ALS_IT_320ms | CM3629_ALS_PERS_1;
		else
			lpi->ls_cmd  = CM3629_ALS_IT_400ms | CM3629_ALS_PERS_1;

		pr_info("[PS][cm3629]%s: lp_info->ls_cmd = 0x%x!\n",
			__func__, lp_info->ls_cmd);
	}
	D("[PS][cm3629] %s: ls_cmd 0x%x, ps1_adc_offset=0x%02X, "
	  "ps2_adc_offset=0x%02X, ps_debounce=0x%x, ps1_thh_diff %d\n",
	  __func__, lpi->ls_cmd, lpi->ps1_adc_offset,
	  lpi->ps2_adc_offset, lpi->ps_debounce, lpi->ps1_thh_diff);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][cm3629 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][cm3629 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}

	lightsensor_set_kvalue(lpi);
	ret = lightsensor_update_table(lpi);
	if (ret < 0) {
		pr_err("[LS][cm3629 error]%s: update ls table fail\n",
			__func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm3629_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][cm3629 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	psensor_set_kvalue(lpi);

#ifdef POLLING_PROXIMITY
	if (lpi->enable_polling_ignore == 1)
		lpi->original_ps_thd_set = lpi->ps1_thd_set;
#endif
	ret = cm3629_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][cm3629 error]%s: cm3629_setup error!\n", __func__);
		goto err_cm3629_setup;
	}
	ps1_canc_set = lpi->inte_ps1_canc;
	ps2_canc_set = lpi->inte_ps2_canc;
	ps1_offset_adc = lpi->ps1_adc_offset;
	ps2_offset_adc = lpi->ps2_adc_offset;
	lpi->cm3629_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm3629_class)) {
		ret = PTR_ERR(lpi->cm3629_class);
		lpi->cm3629_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm3629_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_kadc);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc_table);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_flevel);
	if (ret)
		goto err_create_ls_device_file;

	lpi->ps_dev = device_create(lpi->cm3629_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ls_device_file;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_kadc);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_canc);
	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);
	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_i2c);
	if (ret)
		goto err_create_ps_device;

	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm3629_early_suspend;
	lpi->early_suspend.resume = cm3629_late_resume;
	register_early_suspend(&lpi->early_suspend);

	D("[PS][cm3629] %s: Probe success!\n", __func__);

	return ret;

err_create_ps_device:
	device_unregister(lpi->ps_dev);
err_create_ls_device_file:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm3629_class);
err_create_class:
err_cm3629_setup:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ps_wake_lock));
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
	misc_deregister(&psensor_misc);
err_psensor_setup:
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
err_cm3629_read_chip_id:
err_platform_data_null:
	kfree(lpi);
	return ret;
}

static const struct i2c_device_id cm3629_i2c_id[] = {
	{CM3629_I2C_NAME, 0},
	{}
};

static struct i2c_driver cm3629_driver = {
	.id_table = cm3629_i2c_id,
	.probe = cm3629_probe,
	.driver = {
		.name = CM3629_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init cm3629_init(void)
{
	return i2c_add_driver(&cm3629_driver);
}

static void __exit cm3629_exit(void)
{
	i2c_del_driver(&cm3629_driver);
}

module_init(cm3629_init);
module_exit(cm3629_exit);

MODULE_DESCRIPTION("cm3629 Driver");
MODULE_LICENSE("GPL");
