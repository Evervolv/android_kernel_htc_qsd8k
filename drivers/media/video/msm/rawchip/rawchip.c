/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "rawchip.h"

#define MSM_RAWCHIP_NAME "rawchip"

static struct rawchip_id_info_t yushan_id_info = {
	.rawchip_id_reg_addr = 0x5c04,
	.rawchip_id = 0x02030200,
};

static struct rawchip_info_t rawchip_info = {
	.rawchip_id_info = &yushan_id_info,
};

static struct rawchip_ctrl *rawchipCtrl;

static struct class *rawchip_class;
static dev_t rawchip_devno;

int rawchip_intr0, rawchip_intr1;
atomic_t interrupt, interrupt2;
struct yushan_int_t yushan_int;
struct yushan_int_t {
	spinlock_t yushan_spin_lock;
	wait_queue_head_t yushan_wait;
};


static irqreturn_t yushan_irq_handler(int irq, void *dev_id){

	unsigned long flags;

	//smp_mb();
	spin_lock_irqsave(&yushan_int.yushan_spin_lock,flags);
	//CDBG("[CAM] %s detect INT0, interrupt:%d \n",__func__, interrupt);
	//if (atomic_read(&start_counting))
	//atomic_set(&yushan_int.frame_count, 1);
	//smp_mb();
	atomic_set(&interrupt, 1);
	CDBG("[CAM] %s after detect INT0, interrupt:%d \n",__func__, atomic_read(&interrupt));
	//interrupt = 1;
	//Yushan_ISR();
	//CDBG("[CAM] %s atomic_set\n",__func__);
	//Yushan_ClearInterruptEvent(1);
	wake_up(&yushan_int.yushan_wait);
	spin_unlock_irqrestore(&yushan_int.yushan_spin_lock,flags);

	return IRQ_HANDLED;
}

static irqreturn_t yushan_irq_handler2(int irq, void *dev_id){

	unsigned long flags;

	spin_lock_irqsave(&yushan_int.yushan_spin_lock,flags);
	atomic_set(&interrupt2, 1);
	CDBG("[CAM] %s after detect INT1, interrupt:%d \n", __func__, atomic_read(&interrupt2));
	wake_up(&yushan_int.yushan_wait);
	spin_unlock_irqrestore(&yushan_int.yushan_spin_lock,flags);

	return IRQ_HANDLED;
}






static int rawchip_get_interrupt(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	int rc = 0;
	struct rawchip_stats_event_ctrl se;
	int timeout;
	uint8_t interrupt_type;

	CDBG("%s\n", __func__);

	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	timeout = (int)se.timeout_ms;

	CDBG("[CAM] %s: timeout %d\n", __func__, timeout);

	interrupt_type = Yushan_parse_interrupt();
	se.type = 10;
	se.length = sizeof(interrupt_type);
	if (copy_to_user((void *)(se.data),
			&interrupt_type,
			se.length)) {
			pr_err("[CAM] %s, ERR_COPY_TO_USER 1\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		pr_err("[CAM] %s, ERR_COPY_TO_USER 2\n", __func__);
		rc = -EFAULT;
		goto end;
	}
end:
	return rc;
}

static int rawchip_get_af_status(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	int rc = 0;
	struct rawchip_stats_event_ctrl se;
	int timeout;
	uint32_t pAfStatsGreen[20];

	CDBG("%s\n", __func__);

	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	timeout = (int)se.timeout_ms;

	CDBG("[CAM] %s: timeout %d\n", __func__, timeout);

	rc = Yushan_get_AFSU(pAfStatsGreen);
	if (rc < 0) {
		pr_err("[CAM] %s, Yushan_get_AFSU failed\n", __func__);
		rc = -EFAULT;
		goto end;
	}
	se.type = 5;
	se.length = sizeof(pAfStatsGreen);

	if (copy_to_user((void *)(se.data),
			pAfStatsGreen,
			se.length)) {
			pr_err("[CAM] %s, ERR_COPY_TO_USER 1\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		pr_err("[CAM] %s, ERR_COPY_TO_USER 2\n", __func__);
		rc = -EFAULT;
		goto end;
	}

end:
	return rc;
}

static int rawchip_update_aec_awb_params(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	struct rawchip_stats_event_ctrl se;
	rawchip_update_aec_awb_params_t *update_aec_awb_params;

	CDBG("%s\n", __func__);
	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	update_aec_awb_params = kmalloc(se.length, GFP_ATOMIC);
	if (!update_aec_awb_params) {
		pr_err("[CAM] %s %d: kmalloc failed\n", __func__,
			__LINE__);
		return -ENOMEM;
	}
	if (copy_from_user(update_aec_awb_params,
		(void __user *)(se.data),
		se.length)) {
		pr_err("[CAM] %s %d: copy_from_user failed\n", __func__,
			__LINE__);
		kfree(update_aec_awb_params);
		return -EFAULT;
	}

	CDBG("%s gain=%d line=%d\n", __func__,
		update_aec_awb_params->aec_params.gain, update_aec_awb_params->aec_params.line);
	CDBG("%s rg_ratio=%d bg_ratio=%d\n", __func__,
		update_aec_awb_params->awb_params.rg_ratio, update_aec_awb_params->awb_params.bg_ratio);

	Yushan_Update_AEC_AWB_Params(update_aec_awb_params);

	kfree(update_aec_awb_params);
	return 0;
}

static int rawchip_update_af_params(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	struct rawchip_stats_event_ctrl se;
	rawchip_update_af_params_t *update_af_params;

	CDBG("%s\n", __func__);
	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	update_af_params = kmalloc(se.length, GFP_ATOMIC);
	if (!update_af_params) {
		pr_err("[CAM] %s %d: kmalloc failed\n", __func__,
			__LINE__);
		return -ENOMEM;
	}
	if (copy_from_user(update_af_params,
		(void __user *)(se.data),
		se.length)) {
		pr_err("[CAM] %s %d: copy_from_user failed\n", __func__,
			__LINE__);
		kfree(update_af_params);
		return -EFAULT;
	}

	CDBG("active_number=%d\n", update_af_params->af_params.active_number);
	CDBG("sYushanAfRoi[0] %d %d %d %d\n",
		update_af_params->af_params.sYushanAfRoi[0].bXStart,
		update_af_params->af_params.sYushanAfRoi[0].bXEnd,
		update_af_params->af_params.sYushanAfRoi[0].bYStart,
		update_af_params->af_params.sYushanAfRoi[0].bYEnd);

	Yushan_Update_AF_Params(update_af_params);

	kfree(update_af_params);
	return 0;
}

static int rawchip_update_3a_params(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	struct rawchip_stats_event_ctrl se;
	rawchip_newframe_ack_enable_t  *enable_newframe_ack;

	CDBG("%s\n", __func__);
	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	enable_newframe_ack = kmalloc(se.length, GFP_ATOMIC);
	if (!enable_newframe_ack) {
		pr_err("[CAM] %s %d: kmalloc failed\n", __func__,
			__LINE__);
		return -ENOMEM;
	}
	if (copy_from_user(enable_newframe_ack,
		(void __user *)(se.data),
		se.length)) {
		pr_err("[CAM] %s %d: copy_from_user failed\n", __func__,
			__LINE__);
		kfree(enable_newframe_ack);
		return -EFAULT;
	}

	CDBG("enable_newframe_ack=%d\n", *enable_newframe_ack);
	Yushan_Update_3A_Params(*enable_newframe_ack);
	CDBG("rawchip_update_3a_params done\n");

	kfree(enable_newframe_ack);
	return 0;
}

int rawchip_power_up(const struct msm_camera_rawchip_info *pdata)
{
	int rc = 0;
	CDBG("[CAM]%s\n", __func__);
	pdata->rawchip_gpio_on();

	if (pdata->camera_rawchip_power_on == NULL) {
		pr_err("[CAM]rawchip power on platform_data didn't register\n");
		return -EIO;
	}
	rc = pdata->camera_rawchip_power_on();
	if (rc < 0) {
		pr_err("[CAM] rawchip power on failed\n");
		goto enable_power_on_failed;
	}

	rc = msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
	if (rc < 0) {
		pr_err("[CAM] enable MCLK failed\n");
		goto enable_mclk_failed;
	}
	mdelay(1); /*Mu Lee for sequence with raw chip 20120116*/

	rc = gpio_request(pdata->rawchip_reset, "rawchip");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed\n", pdata->rawchip_reset);
		goto enable_reset_failed;
	}
	gpio_direction_output(pdata->rawchip_reset, 1);
	gpio_free(pdata->rawchip_reset);
	mdelay(1); /*Mu Lee for sequence with raw chip 20120116*/

	yushan_spi_write(0x0008, 0x7f);
	mdelay(1);

	return rc;

enable_reset_failed:
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
enable_mclk_failed:
	if (pdata->camera_rawchip_power_off == NULL)
		pr_err("rawchip power off platform_data didn't register\n");
	else
		pdata->camera_rawchip_power_off();
enable_power_on_failed:
	return rc;
}

int rawchip_power_down(const struct msm_camera_rawchip_info *pdata)
{
	int rc = 0;
	CDBG("%s\n", __func__);

	rc = gpio_request(pdata->rawchip_reset, "rawchip");
	if (rc < 0)
		pr_err("GPIO(%d) request failed\n", pdata->rawchip_reset);
	gpio_direction_output(pdata->rawchip_reset, 0);
	gpio_free(pdata->rawchip_reset);

	mdelay(1);

	rc = msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	if (rc < 0)
		pr_err("[CAM] disable MCLK failed\n");

	if (pdata->camera_rawchip_power_off == NULL) {
		pr_err("rawchip power off platform_data didn't register\n");
		return -EIO;
	}

	rc = pdata->camera_rawchip_power_off();
	if (rc < 0)
		pr_err("[CAM] rawchip power off failed\n");
	pdata->rawchip_gpio_off();
	return rc;
}


int rawchip_match_id(void)
{
	int rc = 0;
	uint32_t chipid = 0;
	CDBG("%s\n", __func__);

	SPI_Read(rawchip_info.rawchip_id_info->rawchip_id_reg_addr, 4, (uint8_t*)(&chipid));
	if (rc < 0) {
		pr_err("%s: read id failed\n", __func__);
		return rc;
	}

	pr_info("[CAM]rawchip id: 0x%x requested id: 0x%x\n", chipid, rawchip_info.rawchip_id_info->rawchip_id);
	if (chipid != rawchip_info.rawchip_id_info->rawchip_id) {
		pr_info("[CAM]rawchip_match_id chip id does not match\n");
		return -ENODEV;
	}

       return rc;
}

void rawchip_release(void)
{
	struct msm_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	pr_info("[CAM] %s\n", __func__);

	rawchip_power_down(pdata);

	CDBG("[CAM] rawchip free irq");
	free_irq(MSM_GPIO_TO_INT(pdata->rawchip_intr0), 0);
	free_irq(MSM_GPIO_TO_INT(pdata->rawchip_intr1), 0);
}

int rawchip_open_init(void)
{
	int rc = 0;
	struct msm_camera_rawchip_info *pdata = rawchipCtrl->pdata;
	int read_id_retry = 0;

	pr_info("%s\n", __func__);

open_read_id_retry:
	rc = rawchip_power_up(pdata);
	if (rc < 0)
		return rc;

	rc = rawchip_match_id();
	if (rc < 0) {
		if (read_id_retry < 3) {
			pr_info("retry read rawchip ID: %d\n", read_id_retry);
			read_id_retry++;
			rawchip_power_down(pdata);
			goto open_read_id_retry;
		}
		goto open_init_failed;
	}

	init_waitqueue_head(&yushan_int.yushan_wait);
	spin_lock_init(&yushan_int.yushan_spin_lock);
	atomic_set(&interrupt, 0);
	atomic_set(&interrupt2, 0);

	/*create irq*/
	rc = request_irq(MSM_GPIO_TO_INT(pdata->rawchip_intr0), yushan_irq_handler,
		IRQF_TRIGGER_RISING, "yushan_irq", 0);
	if (rc < 0) {
		pr_err("request irq intr0 failed\n");
		goto open_init_failed;
	}

	rc = request_irq(MSM_GPIO_TO_INT(pdata->rawchip_intr1), yushan_irq_handler2,
		IRQF_TRIGGER_RISING, "yushan_irq2", 0);
	if (rc < 0) {
		pr_err("request irq intr1 failed\n");
		free_irq(MSM_GPIO_TO_INT(pdata->rawchip_intr0), 0);
		goto open_init_failed;
	}

	rawchipCtrl->rawchip_init = 0;
	rawchip_intr0 = pdata->rawchip_intr0;
	rawchip_intr1 = pdata->rawchip_intr1;
	return rc;

open_init_failed:
	pr_err("%s: rawchip_open_init failed\n", __func__);
	rawchip_power_down(pdata);

	return rc;
}

int rawchip_probe_init(void)
{
	int rc = 0;
	struct msm_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	pr_info("%s\n", __func__);

	if (rawchipCtrl == NULL) {
		pr_err("already failed in __msm_rawchip_probe\n");
		return -EINVAL;
	}

	rc = rawchip_spi_init();
	if (rc < 0) {
		pr_err("%s: failed to register spi driver\n", __func__);
		return rc;
	}

	rc = rawchip_power_up(pdata);
	if (rc < 0)
		return rc;

	rc = rawchip_match_id();
	if (rc < 0)
		goto probe_init_fail;

	pr_info("%s: probe_success\n", __func__);
	return rc;

probe_init_fail:
	pr_err("%s: rawchip_probe_init failed\n", __func__);
	rawchip_power_down(pdata);
	return rc;
}

void rawchip_probe_deinit(void)
{
	struct msm_camera_rawchip_info *pdata = rawchipCtrl->pdata;
	CDBG("%s\n", __func__);

	rawchip_power_down(pdata);
}

static int rawchip_fops_open(struct inode *inode, struct file *filp)
{
	struct rawchip_ctrl *raw_dev = container_of(inode->i_cdev,
		struct rawchip_ctrl, cdev);

	filp->private_data = raw_dev;

	return 0;
}

static unsigned int rawchip_fops_poll(struct file *filp,
	struct poll_table_struct *pll_table)
{
	int rc = 0;
	unsigned long flags;

	poll_wait(filp, &yushan_int.yushan_wait, pll_table);

	spin_lock_irqsave(&yushan_int.yushan_spin_lock, flags);
	if (atomic_read(&interrupt) || atomic_read(&interrupt2)) {
		atomic_set(&interrupt, 0);
		atomic_set(&interrupt2, 0);
		rc = POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&yushan_int.yushan_spin_lock, flags);

	return rc;
}

static long rawchip_fops_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int rc = 0;
	struct rawchip_ctrl *raw_dev = filp->private_data;
	void __user *argp = (void __user *)arg;

	CDBG("%s:%d cmd = %d\n", __func__, __LINE__, _IOC_NR(cmd));

	switch (cmd) {
	case RAWCHIP_IOCTL_GET_INT:
		mutex_lock(&raw_dev->raw_ioctl_get_lock);
		CDBG("RAWCHIP_IOCTL_GET_INT\n");
		rawchip_get_interrupt(raw_dev, argp);
		mutex_unlock(&raw_dev->raw_ioctl_get_lock);
		break;
	case RAWCHIP_IOCTL_GET_AF_STATUS:
		mutex_lock(&raw_dev->raw_ioctl_get_lock);
		CDBG("RAWCHIP_IOCTL_GET_AF_STATUS\n");
		rawchip_get_af_status(raw_dev, argp);
		mutex_unlock(&raw_dev->raw_ioctl_get_lock);
		break;
	case RAWCHIP_IOCTL_UPDATE_AEC_AWB:
		mutex_lock(&raw_dev->raw_ioctl_update_lock);
		CDBG("RAWCHIP_IOCTL_UPDATE_AEC\n");
		rawchip_update_aec_awb_params(raw_dev, argp);
		mutex_unlock(&raw_dev->raw_ioctl_update_lock);
		break;
	case RAWCHIP_IOCTL_UPDATE_AF:
		mutex_lock(&raw_dev->raw_ioctl_update_lock);
		CDBG("RAWCHIP_IOCTL_UPDATE_AF\n");
		rawchip_update_af_params(raw_dev, argp);
		mutex_unlock(&raw_dev->raw_ioctl_update_lock);
		break;
	case RAWCHIP_IOCTL_UPDATE_3A:
		mutex_lock(&raw_dev->raw_ioctl_update_lock);
		CDBG("RAWCHIP_IOCTL_UPDATE_3A\n");
		rawchip_update_3a_params(raw_dev, argp);
		mutex_unlock(&raw_dev->raw_ioctl_update_lock);
		break;
	}

	return rc;
}

static  const struct  file_operations rawchip_fops = {
	.owner	  = THIS_MODULE,
	.open	   = rawchip_fops_open,
	.unlocked_ioctl = rawchip_fops_ioctl,
	.poll  = rawchip_fops_poll,
};

static int setup_rawchip_cdev(void)
{
	int rc = 0;
	struct device *dev;

	pr_info("%s\n", __func__);

	rc = alloc_chrdev_region(&rawchip_devno, 0, 1, MSM_RAWCHIP_NAME);
	if (rc < 0) {
		pr_err("%s: failed to allocate chrdev\n", __func__);
		goto alloc_chrdev_region_failed;
	}

	if (!rawchip_class) {
		rawchip_class = class_create(THIS_MODULE, MSM_RAWCHIP_NAME);
		if (IS_ERR(rawchip_class)) {
			rc = PTR_ERR(rawchip_class);
			pr_err("%s: create device class failed\n",
				__func__);
			goto class_create_failed;
		}
	}

	dev = device_create(rawchip_class, NULL,
		MKDEV(MAJOR(rawchip_devno), MINOR(rawchip_devno)), NULL,
		"%s%d", MSM_RAWCHIP_NAME, 0);
	if (IS_ERR(dev)) {
		pr_err("%s: error creating device\n", __func__);
		rc = -ENODEV;
		goto device_create_failed;
	}

	cdev_init(&rawchipCtrl->cdev, &rawchip_fops);
	rawchipCtrl->cdev.owner = THIS_MODULE;
	rawchipCtrl->cdev.ops   =
		(const struct file_operations *) &rawchip_fops;
	rc = cdev_add(&rawchipCtrl->cdev, rawchip_devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev\n", __func__);
		rc = -ENODEV;
		goto cdev_add_failed;
	}

	return rc;

cdev_add_failed:
	device_destroy(rawchip_class, rawchip_devno);
device_create_failed:
	class_destroy(rawchip_class);
class_create_failed:
	unregister_chrdev_region(rawchip_devno, 1);
alloc_chrdev_region_failed:
	return rc;
}

static void rawchip_tear_down_cdev(void)
{
	cdev_del(&rawchipCtrl->cdev);
	device_destroy(rawchip_class, rawchip_devno);
	class_destroy(rawchip_class);
	unregister_chrdev_region(rawchip_devno, 1);
}

static int rawchip_driver_probe(struct platform_device *pdev)
{
	int rc = 0;
	pr_info("%s\n", __func__);

	rawchipCtrl = kzalloc(sizeof(struct rawchip_ctrl), GFP_ATOMIC);
	if (!rawchipCtrl) {
		pr_err("%s: could not allocate mem for rawchip_dev\n", __func__);
		return -ENOMEM;
	}

	rc = setup_rawchip_cdev();
	if (rc < 0) {
		kfree(rawchipCtrl);
		return rc;
	}

	mutex_init(&rawchipCtrl->raw_ioctl_get_lock);
	mutex_init(&rawchipCtrl->raw_ioctl_update_lock);
	rawchipCtrl->pdata = pdev->dev.platform_data;

	return rc;
}

static int rawchip_driver_remove(struct platform_device *pdev)
{
	rawchip_tear_down_cdev();

	mutex_destroy(&rawchipCtrl->raw_ioctl_get_lock);
	mutex_destroy(&rawchipCtrl->raw_ioctl_update_lock);

	kfree(rawchipCtrl);

	return 0;
}

static struct  platform_driver rawchip_driver = {
	.probe  = rawchip_driver_probe,
	.remove = rawchip_driver_remove,
	.driver = {
		.name = "rawchip",
		.owner = THIS_MODULE,
	},
};

static int __init rawchip_driver_init(void)
{
	int rc;
	rc = platform_driver_register(&rawchip_driver);
	return rc;
}

static void __exit rawchip_driver_exit(void)
{
	platform_driver_unregister(&rawchip_driver);
}

MODULE_DESCRIPTION("rawchip driver");
MODULE_VERSION("rawchip 0.1");

module_init(rawchip_driver_init);
module_exit(rawchip_driver_exit);


