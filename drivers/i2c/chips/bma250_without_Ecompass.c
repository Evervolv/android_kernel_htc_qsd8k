/* drivers/i2c/chips/bma250.c - bma250 G-sensor driver
 *
 * Copyright (C) 2008-2009 HTC Corporation.
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

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/bma250.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include<linux/earlysuspend.h>
#include <linux/akm8975.h>

/*#define EARLY_SUSPEND_BMA 1*/

#define D(x...) printk(KERN_DEBUG "[GSNR][BMA250 NO_COMP] " x)
#define I(x...) printk(KERN_INFO "[GSNR][BMA250 NO_COMP] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA250 NO_COMP ERROR] " x)
#define DIF(x...) \
	if (debug_flag) \
		printk(KERN_DEBUG "[GSNR][BMA250 NO_COMP DEBUG] " x)

#define DEFAULT_RANGE	BMA_RANGE_2G
#define DEFAULT_BW	BMA_BW_31_25HZ

#define RETRY_TIMES	10

static struct i2c_client *this_client;

struct bma250_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend;
};

int ignore_first_event;

static struct bma250_platform_data *pdata;
static atomic_t PhoneOn_flag = ATOMIC_INIT(0);
#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(_name, _mode, _show, _store)

#define	ASENSE_TARGET			0x02D0		/*720*/
/* Added for printing GSensor log once in a while for debugging*/
#define TIME_LOG		50	/*200 ms * 50 ~= 10 sec*/
static unsigned int numCount;
bool sensors_on;
static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t t_flag;
static atomic_t mv_flag;
static short suspend_flag;

struct akm8975_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct delayed_work input_work;
	int	poll_interval;
	struct early_suspend early_suspend;
};
short user_offset[3];

struct mutex gsensor_lock;
struct akm8975_data *akm8975_misc_data;

static int debug_flag;
static char update_user_calibrate_data;

static int BMA_I2C_RxData(char *rxData, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		},
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (retry = 0; retry <= RETRY_TIMES; retry++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;
		else
			mdelay(10);
	}

	if (retry > RETRY_TIMES) {
		E("%s: retry over %d\n", __func__, RETRY_TIMES);
		return -EIO;
	} else
		return 0;
}

static int BMA_I2C_TxData(char *txData, int length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (retry = 0; retry <= RETRY_TIMES; retry++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;
		else
			mdelay(10);
	}

	if (retry > RETRY_TIMES) {
		E("%s: retry over %d\n", __func__, RETRY_TIMES);
		return -EIO;
	} else
		return 0;
}

static int BMA_Init(void)
{
	char buffer[4] = "";
	int ret;
	unsigned char range = 0, bw = 0;

	memset(buffer, 0, 4);

	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(buffer, 2);
	if (ret < 0)
		return -1;
	D("%s: bma250_RANGE_SEL_REG++: range = 0x%02x, bw = 0x%02x\n",
		__func__, buffer[0], buffer[1]);
	range = (buffer[0] & 0xF0) | DEFAULT_RANGE;
	bw = (buffer[1] & 0xE0) | DEFAULT_BW;

	/* Multiple write msgs */
	/*buffer[3] = bw;
	buffer[2] = bma250_BW_SEL_REG;
	buffer[1] = range;
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 4);
	if (ret < 0)
		return -1;*/

	buffer[1] = bw;
	buffer[0] = bma250_BW_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	if (ret < 0) {
		E("%s: Write bma250_BW_SEL_REG fail\n", __func__);
		return -1;
	}

	buffer[1] = range;
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	if (ret < 0) {
		E("%s: Write bma250_BW_SEL_REG fail\n", __func__);
		return -1;
	}

	/* Debug use */
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(buffer, 2);
	if (ret < 0)
		return -1;

	D("%s: bma250_RANGE_SEL_REG--: range = 0x%02x, bw = 0x%02x\n",
		__func__, buffer[0], buffer[1]);

	return 0;

}

static int BMA_TransRBuff(short *rbuf)
{
	char buffer[6];
	int ret;

	memset(buffer, 0, 6);

	buffer[0] = bma250_X_AXIS_LSB_REG;
	ret = BMA_I2C_RxData(buffer, 6);
	if (ret < 0)
		return ret;

	/*D("%s: buffer(0, 1, 2, 3, 4, 5) = (0x%02x, 0x%02x, "
		"0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
		__func__, buffer[0], buffer[1], buffer[2],
		buffer[3], buffer[4], buffer[5]);*/

	rbuf[0] = (short)(buffer[1] << 8 | buffer[0]);
	rbuf[0] >>= 6;
	rbuf[1] = (short)(buffer[3] << 8 | buffer[2]);
	rbuf[1] >>= 6;
	rbuf[2] = (short)(buffer[5] << 8 | buffer[4]);
	rbuf[2] >>= 6;

	DIF("%s: (x, y, z) = (%d, %d, %d)\n",
		__func__, rbuf[0], rbuf[1], rbuf[2]);

	return 1;
}

/* set  operation mode: 0 = normal, 1 = suspend */
static int BMA_set_mode(unsigned char mode)
{
	char buffer[2] = "";
	int ret = 0;
	unsigned char data1 = 0;

	printk(KERN_INFO "[GSNR] Gsensor %s\n", mode ? "disable" : "enable");

	memset(buffer, 0, 2);

	D("%s: mode = 0x%02x\n", __func__, mode);
	if (mode < 2) {
		buffer[0] = bma250_MODE_CTRL_REG;
		ret = BMA_I2C_RxData(buffer, 1);
		if (ret < 0)
			return -1;
		/*D("%s: MODE_CTRL_REG++ = 0x%02x\n", __func__, buffer[0]);*/

		switch (mode) {
		case bma250_MODE_NORMAL:
			ignore_first_event = 1;
			data1 = buffer[0] & 0x7F;
			break;
		case bma250_MODE_SUSPEND:
			data1 = buffer[0] | 0x80;
			break;
		default:
			break;
		}

		/*D("%s: data1 = 0x%02x\n", __func__, data1);*/
		buffer[0] = bma250_MODE_CTRL_REG;
		buffer[1] = data1;
		ret = BMA_I2C_TxData(buffer, 2);
	} else
		ret = E_OUT_OF_RANGE;

	/* Debug use */
	/*buffer[0] = bma250_MODE_CTRL_REG;
	ret = BMA_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;
	D("%s: MODE_CTRL_REG-- = 0x%02x\n", __func__, buffer[0]);*/

	return ret;
}

static int BMA_GET_INT(void)
{
	int ret;
	ret = gpio_get_value(pdata->intr);
	return ret;
}

static int bma_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int bma_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long bma_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{

	void __user *argp = (void __user *)arg;

	char rwbuf[8] = "";
	int ret = -1;
	short buf[8], temp;
	int kbuf = 0;

	DIF("%s: cmd = 0x%x\n", __func__, cmd);

	switch (cmd) {
	case BMA_IOCTL_READ:
	case BMA_IOCTL_WRITE:
	case BMA_IOCTL_SET_MODE:
	case BMA_IOCTL_SET_CALI_MODE:
	case BMA_IOCTL_SET_UPDATE_USER_CALI_DATA:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_from_user(&buf, argp, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		if (copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case BMA_IOCTL_INIT:
		ret = BMA_Init();
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		/*ret = BMA_I2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;*/
		break;
	case BMA_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		/*ret = BMA_I2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;*/
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		pdata->gs_kvalue = kbuf;
		I("%s: Write calibration value: 0x%X\n",
			__func__, pdata->gs_kvalue);
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		ret = BMA_TransRBuff(&buf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if ((pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
			rwbuf[0] = 0;
			rwbuf[1] = 0;
			rwbuf[2] = 0;
		} else {
			rwbuf[0] = (pdata->gs_kvalue >> 16) & 0xFF;
			rwbuf[1] = (pdata->gs_kvalue >>  8) & 0xFF;
			rwbuf[2] =  pdata->gs_kvalue        & 0xFF;
		}
		DIF("%s: CALI(x, y, z) = (%d, %d, %d)\n",
			__func__, rwbuf[0], rwbuf[1], rwbuf[2]);
		break;
	case BMA_IOCTL_SET_MODE:
		BMA_set_mode(rwbuf[0]);
		break;
	case BMA_IOCTL_GET_INT:
		temp = BMA_GET_INT();
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (pdata)
			temp = pdata->chip_layout;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (pdata)
			temp = pdata->calibration_mode;
		break;
	case BMA_IOCTL_SET_CALI_MODE:
		if (pdata)
			pdata->calibration_mode = rwbuf[0];
		break;
	case BMA_IOCTL_GET_UPDATE_USER_CALI_DATA:
		temp = update_user_calibrate_data;
		break;
	case BMA_IOCTL_SET_UPDATE_USER_CALI_DATA:
		update_user_calibrate_data = rwbuf[0];
		break;

	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case BMA_IOCTL_READ:
		/*if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;*/
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_INT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_UPDATE_USER_CALI_DATA:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static void gsensor_poll_work_func(struct work_struct *work)
{
	short buffer[8];
	short gval[3], offset_buf[3], g_acc[3];
	short m_Alayout[3][3];
	int i, j, k;
	struct akm8975_data *akm = container_of(work, struct akm8975_data,
						input_work.work);

	mutex_lock(&gsensor_lock);

	if (atomic_read(&a_flag)) {
		if (BMA_TransRBuff(&buffer[0]) < 0) {
			E("gsensor_poll_work_func: gsensor_read error\n");
			mutex_unlock(&gsensor_lock);
			return;
		}

		if ((pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
			offset_buf[0] = 0;
			offset_buf[1] = 0;
			offset_buf[2] = 0;
		} else {
			offset_buf[0] = (pdata->gs_kvalue >> 16) & 0xFF;
			offset_buf[1] = (pdata->gs_kvalue >>  8) & 0xFF;
			offset_buf[2] =  pdata->gs_kvalue        & 0xFF;

			for (i = 0; i < 3; i++) {
				if (offset_buf[i] > 127)
					offset_buf[i] = offset_buf[i] - 256;
			}
		}
		for (i = 0; i < 3; i++) {
			/*256 = accprms->AS[i] from /data/misc/AccPrmsF.ini"*/
			gval[i] = ((buffer[i] + offset_buf[i] - user_offset[i])
					*ASENSE_TARGET) / 256;
			/*I("gval[%d] = %d, buffer[%d] = %d\n", i, gval[i], i,
				buffer[i]);
			I("offset_buf[%d] = %d, user_offset[%d] = %d\n",
				i, offset_buf[i],  i, user_offset[i]);*/
		}
		for (j = 0; j < 3; j++) {
			for (k = 0; k < 3; k++)
				m_Alayout[j][k] = pdata->layouts[3][j][k];
		}

		/* Acceleration data transformation*/
		g_acc[0] = (gval[0])*m_Alayout[0][0] +
			   (gval[1])*m_Alayout[0][1] +
			   (gval[2])*m_Alayout[0][2];
		g_acc[1] = (gval[0])*m_Alayout[1][0] +
			   (gval[1])*m_Alayout[1][1] +
			   (gval[2])*m_Alayout[1][2];
		g_acc[2] = (gval[0])*m_Alayout[2][0] +
			   (gval[1])*m_Alayout[2][1] +
			   (gval[2])*m_Alayout[2][2];

		if (ignore_first_event >= 3) {
			input_report_abs(akm->input_dev, ABS_X, g_acc[0]);
			input_report_abs(akm->input_dev, ABS_Y, g_acc[1]);
			input_report_abs(akm->input_dev, ABS_Z, g_acc[2]);
			input_sync(akm->input_dev);
		} else {
			ignore_first_event++;
			D("Ignore X = %d\n", g_acc[0]);
			D("Ignore Y = %d\n", g_acc[1]);
			D("Ignore Z = %d\n", g_acc[2]);
		}

		numCount++;
		/* Added for printing GSensor log once in a while for debugging
		 */
		if ((numCount % TIME_LOG) == 0)
			D("GSensor [%d,%d,%d]\n", gval[0], gval[1], gval[2]);
	}

	/* E("gsensor_poll_work_func:working %d, %d\n", a_flag, t_flag); */
	mutex_unlock(&gsensor_lock);
	schedule_delayed_work(&akm->input_work,
			msecs_to_jiffies(akm->poll_interval));
}

int gsensor_off(void)
{
	int 	ret = 0;

	mutex_lock(&gsensor_lock);
	D("gsensor_off++\n");

	numCount = 0;

	BMA_set_mode(bma250_MODE_SUSPEND);

	D("gsensor_off--\n");
	mutex_unlock(&gsensor_lock);

	return ret;
}

int sensor_open(void)
{
	int ret;
	char buffer, mode = -1;
	int i = 0;

	D("sensor_open++\n");

	if (!(atomic_read(&a_flag) || atomic_read(&t_flag))) {
		E("sensor_open:unable to open due to the flag:%d, %d\n",
			atomic_read(&a_flag), atomic_read(&t_flag));
		return 0;
	}

	if (atomic_read(&a_flag)) {
		for (i = 0; i < RETRY_TIMES; i++) {
			ret = BMA_Init();
			if (ret) {
				E("BMA_Init error\n");
				return ret;
			}
			BMA_set_mode(bma250_MODE_NORMAL);

			usleep(2000);

			buffer = bma250_MODE_CTRL_REG;
			ret = BMA_I2C_RxData(&buffer, 1);
			if (ret < 0) {
				E("%s: Read bma250_MODE_CTRL_REG fail!\n",
					__func__);
				return -1;
			}
			mode = ((buffer & 0x80) >> 7);
			if (mode == 1) {
				D("%s: Set mode retry = %d\n", __func__, i);
				if (i >= (RETRY_TIMES - 1)) {
					E("%s: Set mode Fail!!\n", __func__);
					return -1;
				}
			} else
				break;
		}
	}

	if (akm8975_misc_data->poll_interval <= 0) {
		E("[Gsensor] the delay interval was set to:%d, use default"
			" value\n",
			akm8975_misc_data->poll_interval);
		akm8975_misc_data->poll_interval = 200;
	}

	#if 1  /* HTC_CSP_START */
	/*
	input_handle_event() check current and last g-sensor values.
	If these two values are the same, current value would not be reported.
	Set 0xffff as defalt g-sersor value, therefore the first gsensor
	value will always be reported.
	*/
	akm8975_misc_data->input_dev->absinfo[ABS_X].value = 0xffff;
	akm8975_misc_data->input_dev->absinfo[ABS_Y].value = 0xffff;
	akm8975_misc_data->input_dev->absinfo[ABS_Z].value = 0xffff;
	#endif /* HTC_CSP_END */

	sensors_on = true;

	#if 1  /* HTC_CSP_START */
	schedule_delayed_work(&akm8975_misc_data->input_work, 0);
	#else
	schedule_delayed_work(&akm8975_misc_data->input_work,
		msecs_to_jiffies(akm8975_misc_data->poll_interval));
	#endif

	return 0;
}

int sensor_close(void)
{
	int ret = 0;

	D("sensor_close++\n");

	if (!(atomic_read(&a_flag))) {
		ret = gsensor_off();
		if (ret < 0)
			E("gsensor_off error\n");
	}

	cancel_delayed_work(&akm8975_misc_data->input_work);
	sensors_on = false;

	return ret;
}

/* GSensor AKM control interface */
void sensor_poll_interval_set(uint32_t poll_interval)
{

/*User space app would do this after early supsend, so comment out the
follwoing statements */
/*
	if (suspend_flag) {
		E("sensor_poll_interval_set:unable to set due to early"
			" suspend");
		return;
	}
*/
	D("sensor_poll_interval_set: %d ms\n", poll_interval);

	if (poll_interval <= 0) {
		E("sensor_poll_interval_reset: 200 ms\n");
		poll_interval = 200;
	}

	akm8975_misc_data->poll_interval = poll_interval;

	if (sensors_on == false && suspend_flag == 0)
		sensor_open();
}

int sensor_poll_interval_get(void)
{
	E("sensor_poll_interval_get:%d msec\n",
		akm8975_misc_data->poll_interval);
	return akm8975_misc_data->poll_interval;
}

int akm_aot_open(struct inode *inode, struct file *file)
{
	I("gsensor_aot_open\n");
	sensor_open();

	return 0;
}

int akm_aot_release(struct inode *inode, struct file *file)
{
	I("gsensor_aot_release\n");
	sensor_close();

	return 0;
}

long akm_aot_ioctl(struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag = 0;


	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_TFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_YPR:
		if (copy_from_user(&user_offset, argp, sizeof(user_offset)))
			return -EFAULT;
		D("akm ECS_IOCTL_SET_YPR: %d, %d, %d\n", user_offset[0],
			user_offset[1], user_offset[2]);
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		atomic_set(&a_flag, flag);
		D("ESC_IOCTL_APP_SET_AFLAG: %d\n", atomic_read(&a_flag));
		if (flag == 0) {
			D("set Aflag to close gsensor\n");
			sensor_close();
		}
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		D("ESC_IOCTL_APP_GET_AFLAG: %d\n", atomic_read(&a_flag));
		break;
	case ECS_IOCTL_APP_SET_TFLAG:
		atomic_set(&t_flag, flag);
		D("ESC_IOCTL_APP_SET_TFLAG: %d\n", atomic_read(&t_flag));
		if (flag == 0) {
			D("set Tflag to close Tsensor\n");
			sensor_close();
		}
		break;
	case ECS_IOCTL_APP_GET_TFLAG:
		flag = atomic_read(&t_flag);
		D("ESC_IOCTL_APP_GET_TFLAG: %d\n", atomic_read(&t_flag));
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		D("ESC_IOCTL_APP_SET_DELAY: %d", flag);
		sensor_poll_interval_set(flag);
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = sensor_poll_interval_get();
		break;
	case ECS_IOCTL_SET_YPR:
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_YPR:
		break;
	default:
		break;
	}

	return 0;
}

const struct file_operations akm_aot_fops = {
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	.unlocked_ioctl = akm_aot_ioctl,
};

struct miscdevice akm_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8975_aot",
	.fops = &akm_aot_fops,
};

#ifdef EARLY_SUSPEND_BMA

static void bma250_early_suspend(struct early_suspend *handler)
{
	if (!atomic_read(&PhoneOn_flag))
		BMA_set_mode(bma250_MODE_SUSPEND);
	else
		D("bma250_early_suspend: PhoneOn_flag is set\n");
}

static void bma250_late_resume(struct early_suspend *handler)
{
	BMA_set_mode(bma250_MODE_NORMAL);
}

#else /* EARLY_SUSPEND_BMA */

static int bma250_suspend(struct i2c_client *client, pm_message_t mesg)
{
	BMA_set_mode(bma250_MODE_SUSPEND);

	return 0;
}

static int bma250_resume(struct i2c_client *client)
{
	BMA_set_mode(bma250_MODE_NORMAL);
	return 0;
}
#endif /* EARLY_SUSPEND_BMA */

static ssize_t bma250_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", atomic_read(&PhoneOn_flag));
	return s - buf;
}

static ssize_t bma250_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		atomic_set(&PhoneOn_flag, 1);
		D("bma250_store: PhoneOn_flag=%d\n",
			atomic_read(&PhoneOn_flag));
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		atomic_set(&PhoneOn_flag, 0);
		D("bma250_store: PhoneOn_flag=%d\n",
			atomic_read(&PhoneOn_flag));
		return count;
	}
	E("bma250_store: invalid argument\n");
	return -EINVAL;

}

static DEVICE_ACCESSORY_ATTR(PhoneOnOffFlag, 0664, \
	bma250_show, bma250_store);

static ssize_t debug_flag_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char buffer, range = -1, bandwidth = -1, mode = -1;
	int ret;

	buffer = bma250_BW_SEL_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	bandwidth = (buffer & 0x1F);

	buffer = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	range = (buffer & 0xF);

	buffer = bma250_MODE_CTRL_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	mode = ((buffer & 0x80) >> 7);

	s += sprintf(s, "debug_flag = %d, range = 0x%x, bandwidth = 0x%x, "
		"mode = 0x%x\n", debug_flag, range, bandwidth, mode);

	return s - buf;
}
static ssize_t debug_flag_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
/*	char buffer[3] = "", range = -1, bandwidth = -1;
	int ret;*/

	debug_flag = -1;
	sscanf(buf, "%d", &debug_flag);

	D("%s: debug_flag = %d\n", __func__, debug_flag);
/*
	buffer[1] = DEFAULT_BW;
	buffer[0] = bma250_BW_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	if (ret < 0) {
		E("%s: Write bma250_BW_SEL_REG fail\n", __func__);
		return -1;
	}
	D("%s: Write bma250_BW_SEL_REG success!\n", __func__);

	buffer[1] = DEFAULT_RANGE;
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	if (ret < 0) {
		E("%s: Write bma250_BW_SEL_REG fail\n", __func__);
		return -1;
	}
	D("%s: Write bma250_RANGE_SEL_REG success!\n", __func__);

	buffer[0] = bma250_BW_SEL_REG;
	ret = BMA_I2C_RxData(&buffer[0], 1);
	if (ret < 0)
		return -1;
	bandwidth = (buffer[0] & 0x1F);

	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(&buffer[0], 1);
	if (ret < 0)
		return -1;
	range = (buffer[0] & 0xF);

	D("%s: range = 0x%x, bandwidth = 0x%x\n", __func__, range, bandwidth);
*/
	return count;

}

static DEVICE_ACCESSORY_ATTR(debug_en, 0664, \
	debug_flag_show, debug_flag_store);

static int bma250_registerAttr(void)
{
	int ret;
	struct class *htc_accelerometer_class;
	struct device *accelerometer_dev;

	htc_accelerometer_class = class_create(THIS_MODULE,
					"htc_accelerometer");
	if (IS_ERR(htc_accelerometer_class)) {
		ret = PTR_ERR(htc_accelerometer_class);
		htc_accelerometer_class = NULL;
		goto err_create_class;
	}

	accelerometer_dev = device_create(htc_accelerometer_class,
				NULL, 0, "%s", "accelerometer");
	if (unlikely(IS_ERR(accelerometer_dev))) {
		ret = PTR_ERR(accelerometer_dev);
		accelerometer_dev = NULL;
		goto err_create_accelerometer_device;
	}

	/* register the attributes */
	ret = device_create_file(accelerometer_dev, &dev_attr_PhoneOnOffFlag);
	if (ret)
		goto err_create_accelerometer_device_file;

	/* register the debug_en attributes */
	ret = device_create_file(accelerometer_dev, &dev_attr_debug_en);
	if (ret)
		goto err_create_accelerometer_debug_en_device_file;

	return 0;

err_create_accelerometer_debug_en_device_file:
	device_remove_file(accelerometer_dev, &dev_attr_PhoneOnOffFlag);
err_create_accelerometer_device_file:
	device_unregister(accelerometer_dev);
err_create_accelerometer_device:
	class_destroy(htc_accelerometer_class);
err_create_class:

	return ret;
}

static const struct file_operations bma_fops = {
	.owner = THIS_MODULE,
	.open = bma_open,
	.release = bma_release,
	.unlocked_ioctl = bma_ioctl,
};

static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &bma_fops,
};

static int bma250_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct bma250_data *bma;
	char buffer[2];
	int err = 0;
	struct akm8975_data *akm;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	bma = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!bma) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, bma);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		E("bma250_init_client: platform data is NULL\n");
		goto exit_platform_data_null;
	}

	pdata->gs_kvalue = gs_kvalue;
	I("BMA250 G-sensor I2C driver: gs_kvalue = 0x%X\n",
		pdata->gs_kvalue);

	this_client = client;
	ignore_first_event = 0;

	buffer[0] = bma250_CHIP_ID_REG;
	err = BMA_I2C_RxData(buffer, 1);
	if (err < 0)
		goto exit_wrong_ID;
	D("%s: CHIP ID = 0x%02x\n", __func__, buffer[0]);
	if ((buffer[0] != 0x3) && (buffer[0] != 0xF9)) {
		E("Wrong chip ID of BMA250 or BMA250E!!\n");
		goto exit_wrong_ID;
	}

	err = BMA_Init();
	if (err < 0) {
		E("bma250_probe: bma_init failed\n");
		goto exit_init_failed;
	}

	err = misc_register(&bma_device);
	if (err) {
		E("bma250_probe: device register failed\n");
		goto exit_misc_device_register_failed;
	}

#ifdef EARLY_SUSPEND_BMA
	bma->early_suspend.suspend = bma250_early_suspend;
	bma->early_suspend.resume = bma250_late_resume;
	register_early_suspend(&bma->early_suspend);
#endif

	err = bma250_registerAttr();
	if (err) {
		E("%s: set spi_bma150_registerAttr fail!\n", __func__);
		goto err_registerAttr;
	}

	akm = kzalloc(sizeof(struct akm8975_data), GFP_KERNEL);
	if (!akm) {
		err = -ENOMEM;
		E(
		       "%s : Failed"
		       " kzalloc data\n", __func__);
		goto exit_alloc_data_failed_akm;
	}
	akm8975_misc_data = akm;
	akm->input_dev = input_allocate_device();

	if (!akm->input_dev) {
		err = -ENOMEM;
		E(
		       "%s : Failed"
		       " to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, akm->input_dev->evbit);
	/* x-axis acceleration */
	input_set_abs_params(akm->input_dev, ABS_X, -1872, 1872, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(akm->input_dev, ABS_Y, -1872, 1872, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(akm->input_dev, ABS_Z, -1872, 1872, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(akm->input_dev, ABS_WHEEL, -32768, 3, 0, 0);

	akm->input_dev->name = "compass";

	err = input_register_device(akm->input_dev);

	if (err) {
		E(
		       "%s : Unable to register"
		       " input device: %s\n", __func__,
		       akm->input_dev->name);
		goto exit_input_register_device_failed;
	}
	err = misc_register(&akm_aot_device);
	if (err) {
		E(
		       "%s : akm_aot_device register "
		       "failed\n",  __func__);
		goto exit_misc_device_register_failed_akm;
	}
	mutex_init(&gsensor_lock);

	INIT_DELAYED_WORK(&akm->input_work, gsensor_poll_work_func);

	atomic_set(&a_flag, 1);
	I("%s: BMA250_NO_COMP: I2C retry 10 times version. OK\n", __func__);

	debug_flag = 0;
	suspend_flag = 0;
	sensors_on = false;
	numCount = 0;

	return 0;

exit_misc_device_register_failed_akm:
exit_input_register_device_failed:
	input_free_device(akm->input_dev);
exit_input_dev_alloc_failed:
	kfree(akm);
exit_alloc_data_failed_akm:
err_registerAttr:
exit_misc_device_register_failed:
exit_init_failed:
exit_wrong_ID:
exit_platform_data_null:
	kfree(bma);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int bma250_remove(struct i2c_client *client)
{
	struct bma250_data *bma = i2c_get_clientdata(client);
	kfree(bma);
	return 0;
}

static const struct i2c_device_id bma250_id[] = {
	{ BMA250_I2C_NAME_REMOVE_ECOMPASS, 0 },
	{ }
};

static struct i2c_driver bma250_driver = {
	.probe = bma250_probe,
	.remove = bma250_remove,
	.id_table	= bma250_id,

#ifndef EARLY_SUSPEND_BMA
	.suspend = bma250_suspend,
	.resume = bma250_resume,
#endif
	.driver = {
		   .name = BMA250_I2C_NAME_REMOVE_ECOMPASS,
		   },
};

static int __init bma250_init(void)
{
	I("BMA250 G-sensor driver: init\n");
	return i2c_add_driver(&bma250_driver);
}

static void __exit bma250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}

module_init(bma250_init);
module_exit(bma250_exit);

MODULE_DESCRIPTION("BMA250 G-sensor driver");
MODULE_LICENSE("GPL");

