#include <linux/delay.h>
#include <linux/debugfs.h>
#include <mach/msm_fb.h>
#include <mach/msm_hdmi.h>

#include "../include/fb-hdmi.h"
#include "../include/sil902x.h"
#include "../include/tpi.h"

#if 1
#define HDCP_DBG(fmt, arg...) printk( "[hdmi/hdcp]%s: " fmt, __func__, ##arg)
#else
#define HDCP_DBG(fmt...) do {} while (0)
#endif

#define hdcp_err(fmt, arg...) pr_err( "[hdmi/hdcp]%s: " fmt, __func__, ##arg)

//#define HDCP_DEBUG              /* Remove this definition when releasing!!! */
#if defined(HDCP_DEBUG)
#define HDCP_OVERWRITE_CONTROL          0x1
#define HDCP_SUPPORT                    0x2
#define HDCP_AVALABLE                   0x4
static int hdcp_control = 0x7;
module_param_named(hdcp_control, hdcp_control, int,
                   S_IRUGO | S_IWUSR | S_IWGRP);
#endif

bool HDCP_TxSupports;
bool HDCP_Started;
u8 HDCP_LinkProtectionLevel;

//////////////////////////////////////////////////////////////////////////////
// FUNCTION     :   IsHDCP_Supported()
// PURPOSE      :   Check Tx revision number to find if this Tx supports HDCP
//                  by reading the HDCP revision number from TPI register 0x30.
// RETURNS      :   true if Tx supports HDCP. false if not.
//////////////////////////////////////////////////////////////////////////////
bool hdcp_check_support(struct hdmi_info *hdmi)
{
	u8 HDCP_Rev;
	bool HDCP_Supported;

	HDCP_Supported = true;
	/* Check Device ID */
	HDCP_Rev = hdmi_read(hdmi->client, TPI_HDCP_REVISION_DATA_REG);
	if (HDCP_Rev !=
		(HDCP_MAJOR_REVISION_VALUE | HDCP_MINOR_REVISION_VALUE))
		HDCP_Supported = false;

	HDCP_DBG("ret=%d\n", HDCP_Supported);
	return HDCP_Supported;
}

//////////////////////////////////////////////////////////////////////////////
// FUNCTION      :  AreAKSV_OK()
// PURPOSE       :  Check if AKSVs contain 20 '0' and 20 '1'
// INPUT PARAMS  :  None
// OUTPUT PARAMS :  None
// GLOBALS USED  :  TBD
// RETURNS       :  true if 20 zeros and 20 ones found in AKSV. false OTHERWISE
//////////////////////////////////////////////////////////////////////////////
static bool hdcp_check_aksv(struct hdmi_info *hdmi)
{
	int ret;
	u8 B_Data[AKSV_SIZE];
	u8 i, j, NumOfOnes = 0;

	memset(B_Data, 0, AKSV_SIZE);
#if 0
	ReadBlockTPI(hdmi, TPI_AKSV_1_REG, AKSV_SIZE, B_Data);
#else
	for (i = 0; i < 5; i++) {
		B_Data[i] = hdmi_read(hdmi->client, TPI_AKSV_1_REG+i);
	}
#endif
	HDCP_DBG(" askv={%02x, %02x, %02x, %02x, %02x}\n",
		B_Data[0], B_Data[1], B_Data[2], B_Data[3], B_Data[4]);
	for (i=0; i < AKSV_SIZE; i++)
		for (j=0; j < BYTE_SIZE; j++) {
			if (B_Data[i] & 0x01)
				NumOfOnes++;
			B_Data[i] >>= 1;
		}
	if (NumOfOnes != NUM_OF_ONES_IN_KSV)
		ret = false;
	else ret = true;

	HDCP_DBG(":ret=%s\n", ret ? "true" : "false");

	return true;
}

//////////////////////////////////////////////////////////////////////////////
// FUNCTION     :   HDCP_On()
// PURPOSE      :   Switch hdcp on.
// INPUT PARAMS :   None
// OUTPUT PARAMS:   None
// GLOBALS USED :   HDCP_Started set to true
// RETURNS      :   None
//////////////////////////////////////////////////////////////////////////////
//void hdcp_on(struct hdmi_info *hdmi)
void hdcp_on(struct hdmi_info *hdmi, const char *caller)
{
	HDCP_DBG(", caller=%s\n", caller);
	hdmi_write_byte(hdmi->client, TPI_HDCP_CONTROL_DATA_REG, PROTECTION_LEVEL_MAX);
	HDCP_Started = true;
}

//////////////////////////////////////////////////////////////////////////////
// FUNCTION     :   HDCP_Off()
// PURPOSE      :   Switch hdcp off.
// GLOBALS USED :   HDCP_Started set to false
// RETURNS      :   None
//////////////////////////////////////////////////////////////////////////////
void hdcp_off(struct hdmi_info *hdmi)
{
	HDCP_DBG("\n");

	SetInputColorSpace(hdmi, INPUT_COLOR_SPACE_BLACK_MODE);
	SetAudioMute(hdmi, AUDIO_MUTE_MUTED);
	hdmi_write_byte(hdmi->client, TPI_HDCP_CONTROL_DATA_REG, PROTECTION_LEVEL_MIN);
	HDCP_Started = false;
	HDCP_LinkProtectionLevel = EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE;
}

void hdcp_init(struct hdmi_info *hdmi)
{
	HDCP_DBG("\n");

	HDCP_TxSupports = false;
	HDCP_Started = false;
	HDCP_LinkProtectionLevel = EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE;

	/* TX-related... need only be done once. */
	if (!hdcp_check_support(hdmi)) {
		hdcp_err("TX does not support HDCP\n");
		return;
	}
	if (!hdcp_check_aksv(hdmi)) {
		hdcp_err("Illegal AKSV\n");
		return;
	}
	HDCP_TxSupports = true;
}

void hdcp_restart(struct hdmi_info *hdmi)
{
	HDCP_DBG("\n");
	DisableTMDS(hdmi);
	hdcp_off(hdmi);
	EnableTMDS(hdmi);
}

void hdcp_check_status(struct hdmi_info *hdmi, u8 InterruptStatusImage)
{
	u8 QueryData, LinkStatus, RegImage, NewLinkProtectionLevel;

	if (HDCP_TxSupports == false)
		return;

	if ((HDCP_LinkProtectionLevel ==
		(EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE)) &&
		(HDCP_Started == false)) {
			QueryData = hdmi_read(hdmi->client, TPI_HDCP_QUERY_DATA_REG);
		/* Is HDCP avaialable */
		if (QueryData & PROTECTION_TYPE_MASK) {
			hdcp_on(hdmi, __func__);
		}
	}
	/* Check if Link Status has changed: */
	if (InterruptStatusImage & SECURITY_CHANGE_EVENT) {
		HDCP_DBG("SECURITY_CHANGE_EVENT\n");
		LinkStatus = hdmi_read(hdmi->client, TPI_HDCP_QUERY_DATA_REG);
		LinkStatus &= LINK_STATUS_MASK;
		tpi_clear_interrupt(hdmi, SECURITY_CHANGE_EVENT);
		switch (LinkStatus) {
		case LINK_STATUS_NORMAL:
			HDCP_DBG("Link = Normal\n");
			break;
		case LINK_STATUS_LINK_LOST:
			HDCP_DBG("Link = Lost\n");
			hdcp_restart(hdmi);
			break;
		case LINK_STATUS_RENEGOTIATION_REQ:
			HDCP_DBG("Link = Renegotiation Required\n");
			hdcp_off(hdmi);
			hdcp_on(hdmi, __func__);
			break;
		case LINK_STATUS_LINK_SUSPENDED:
			HDCP_DBG("Link = Suspended\n");
			hdcp_on(hdmi, __func__);
			break;
		}
	}
	/* Check if HDCP state has changed: */
	if (InterruptStatusImage & HDCP_CHANGE_EVENT) {
		HDCP_DBG("HDCP_CHANGE_EVENT\n");
		RegImage = hdmi_read(hdmi->client, TPI_HDCP_QUERY_DATA_REG);
		NewLinkProtectionLevel = RegImage &
			(EXTENDED_LINK_PROTECTION_MASK | LOCAL_LINK_PROTECTION_MASK);
		if (NewLinkProtectionLevel != HDCP_LinkProtectionLevel) {
			HDCP_LinkProtectionLevel = NewLinkProtectionLevel;
			switch (HDCP_LinkProtectionLevel) {
			case (EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE):
				HDCP_DBG("Protection = None\n");
				hdcp_restart(hdmi);
				break;
			case LOCAL_LINK_PROTECTION_SECURE:
				SetAudioMute(hdmi, AUDIO_MUTE_NORMAL);
				//SetInputColorSpace (hdmi, INPUT_COLOR_SPACE_YCBCR422);
				SetInputColorSpace (hdmi, INPUT_COLOR_SPACE_RGB);
				HDCP_DBG("Protection = Local, Video Unmuted\n");
				break;
			case (EXTENDED_LINK_PROTECTION_SECURE | LOCAL_LINK_PROTECTION_SECURE):
				HDCP_DBG("Protection = Extended\n");
				break;
			default:
				HDCP_DBG("Protection = Extended but not Local?\n");
				hdcp_restart(hdmi);
				break;
			}
		}
		tpi_clear_interrupt(hdmi, HDCP_CHANGE_EVENT);
	}
}

/*----------------------------------------------------------------------------*/
#if defined(HDMI_DEBUGFS)
static ssize_t hdcp_supported_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n=0;
        char buffer[80];
        //struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

	HDMI_DBG("%s\n", __func__);
        //n = scnprintf(buffer, 80, "%d\n", is_hdcp_supported(hdmi));
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t hdcp_available_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n=0;
        char buffer[80];
        //struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

	HDMI_DBG("%s\n", __func__);
        //n = scnprintf(buffer, 80, "%d\n", is_hdcp_available(hdmi));
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t hdcp_on_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
	hdcp_on(hdmi, __func__);
	return 0;
}

static ssize_t hdcp_off_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
        hdcp_off(hdmi);
        return 0;
}

static ssize_t hdcp_restart_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
        hdcp_restart(hdmi);
        return 0;
}

static ssize_t hdcp_handle_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        //struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
        //handle_hdcp(hdmi);
        return 0;
}

static ssize_t hdcp_poll_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        //struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
        //hdcp_poll(hdmi);
        return 0;
}

static ssize_t hdcp_aksv_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
        hdcp_check_aksv(hdmi);
        return 0;
}

static ssize_t hdcp_bksv_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        //struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
        //hdcp_check_bksv(hdmi);
        return 0;
}

static struct file_operations hdcp_debugfs_fops[] = {
        { .open  = hdmi_dbgfs_open, .read  = hdcp_supported_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_available_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_on_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_off_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_restart_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_poll_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_aksv_read, },
        { .open  = hdmi_dbgfs_open, .read  = hdcp_bksv_read, },
};

// mutex ?
int hdcp_debugfs_init(struct hdmi_info *hdmi)
{
        struct dentry *hdcp_dent;

        hdcp_dent = debugfs_create_dir("hdcp", hdmi->debug_dir);
        if (IS_ERR(hdcp_dent))
                return PTR_ERR(hdcp_dent);

        //FIXME: error handling
        debugfs_create_file("supported", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[0]);
        debugfs_create_file("available", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[1]);
        debugfs_create_file("on", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[2]);
        debugfs_create_file("off", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[3]);
        debugfs_create_file("restart", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[4]);
        debugfs_create_file("poll", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[5]);
        debugfs_create_file("aksv", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[6]);
        debugfs_create_file("bksv", 0444, hdcp_dent, hdmi,
                &hdcp_debugfs_fops[7]);

        return 0;
}
#endif

