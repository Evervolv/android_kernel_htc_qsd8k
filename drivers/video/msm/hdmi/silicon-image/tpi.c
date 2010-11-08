#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <mach/msm_fb.h>
#include <mach/msm_hdmi.h>
#include <linux/interrupt.h>

// FIXME: remove this if unnecessary in the future
#ifdef CONFIG_HTC_HEADSET_MGR
#include <mach/htc_headset_mgr.h>
#endif

#if 1
#define HDMI_DBG(s...) printk("[hdmi/tpi]" s)
#else
#define HDMI_DBG(s...) do {} while (0)
#endif

#include "../include/fb-hdmi.h"
#include "../include/sil902x.h"
#include "../include/tpi.h"

#define NEW_INTEGRATE
#define DBG_POLLING     0x1
static int debug_mask;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DLOG(mask, fmt, args...) \
do { \
        if (debug_mask & mask) \
                printk(KERN_INFO "[hdmi/sil]: "fmt, ##args); \
} while (0)

#define X1	0x01
#define AFTER_INIT              1

void HotPlugService (struct hdmi_info *hdmi);
// FIXME: should be decide by detection
static bool dsRxPoweredUp;
static bool edidDataValid;
static bool tmdsPoweredUp;
u8 pvid_mode, vid_mode = 16;
u8 systemInitialized;

void tpi_clear_interrupt(struct hdmi_info *hdmi, u8 pattern)
{
	/* write "1" to clear interrupt bit, and 0 won't effect origin value. */
	hdmi_write_byte(hdmi->client, TPI_INTERRUPT_STATUS_REG, pattern);
}

//////////////////////////////////////////////////////////////////////////////
// FUNCTION      :  EnableInterrupts()
// PURPOSE       :  Enable the interrupts specified in the input parameter
// INPUT PARAMS  :  A bit pattern with "1" for each interrupt that needs to be
//                  set in the Interrupt Enable Register (TPI offset 0x3C)
// OUTPUT PARAMS :  void
// GLOBALS USED  :  None
// RETURNS       :  TRUE
//////////////////////////////////////////////////////////////////////////////
bool tpi_enable_interrupts(struct hdmi_info *hdmi, u8 Interrupt_Pattern)
{
	HDMI_DBG("%s, reg=%02x, pat=%02x\n", __func__, TPI_INTERRUPT_EN, Interrupt_Pattern);
	ReadSetWriteTPI(hdmi, TPI_INTERRUPT_EN, Interrupt_Pattern);
	return true;
}

static void tpi_disable_interrupts(struct hdmi_info *hdmi, u8 pattern)
{
/*
	HDMI_DBG("%s, reg=%02x, pat=%02x\n", __func__,
			TPI_INTERRUPT_EN, pattern);
*/
	ReadClearWriteTPI(hdmi, TPI_INTERRUPT_EN, pattern);
}

static void tpi_clear_pending_event(struct hdmi_info *hdmi)
{
	int retry = 100;

	if (hdmi->sleeping == SLEEP) return;
	while (retry--) {
		hdmi_write_byte(hdmi->client, 0x3c, 1);
		hdmi_write_byte(hdmi->client, 0x3d, 1);
		if (hdmi_read(hdmi->client, 0x3d) & 0x01)
			msleep(1);
		else
			break;
	}
	if (retry < 19) HDMI_DBG("%s: retry=%d\n", __func__, 19 - retry);
}

//////////////////////////////////////////////////////////////////////////////
// FUNCTION     :   ReadBackDoorRegister()
// PURPOSE      :   Read a 922x register value from a backdoor register
//                  Write:
//                      1. 0xBC => Internal page num
//                      2. 0xBD => Backdoor register offset
//                  Read:
//                      3. 0xBE => Returns the backdoor register value
// INPUT PARAMS :   Internal page number, backdoor register offset, pointer to
//                  buffer to store read value
// OUTPUT PARAMS:   Buffer that stores the read value
// RETURNS      :   TRUE 
// NOTE         :   This workaround is needed for the 9220/2 only.
//////////////////////////////////////////////////////////////////////////////
int tpi_read_backdoor_register(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset)
{
	// FIXME: error handling
	struct i2c_client *client = hdmi->client;

	/* Internal page */
	hdmi_write_byte(client, TPI_INTERNAL_PAGE_REG, PageNum);
	/* Indexed register */
	hdmi_write_byte(client, TPI_REGISTER_OFFSET_REG, RegOffset);
	/* Read value into buffer */
	return hdmi_read(client, TPI_REGISTER_VALUE_REG);
}

void tpi_write_backdoor_register(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset, u8 RegValue) {
	/* Internal page */
	hdmi_write_byte(hdmi->client, TPI_INTERNAL_PAGE_REG, PageNum);
	/* Indexed register */
	hdmi_write_byte(hdmi->client, TPI_REGISTER_OFFSET_REG, RegOffset);
	/* Read value into buffer */
	hdmi_write_byte(hdmi->client, TPI_REGISTER_VALUE_REG, RegValue);
}

#define TPI_INTERNAL_PAGE_REG		0xBC
#define TPI_INDEXED_OFFSET_REG		0xBD
#define TPI_INDEXED_VALUE_REG		0xBE
#define INDEXED_PAGE_0			0x01
#define INDEXED_PAGE_1			0x02
#define INDEXED_PAGE_2			0x03

void ReadModifyWriteIndexedRegister(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset, u8 Mask, u8 Value)
{
	u8 Tmp;

	hdmi_write_byte(hdmi->client, TPI_INTERNAL_PAGE_REG, PageNum);
	hdmi_write_byte(hdmi->client, TPI_INDEXED_OFFSET_REG, RegOffset);
	Tmp = hdmi_read(hdmi->client, TPI_INDEXED_VALUE_REG);

	Tmp &= ~Mask;
	Tmp |= (Value & Mask);

	hdmi_write_byte(hdmi->client, TPI_INDEXED_VALUE_REG, Tmp);
}

void ReadSetWriteTPI(struct hdmi_info *hdmi, u8 Offset, u8 Pattern)
{
	u8 Tmp;
	struct i2c_client *client = hdmi->client;

	Tmp = hdmi_read(client, Offset);
	Tmp |= Pattern;
	hdmi_write_byte(client, Offset, Tmp);
}

int tpi_set_bit(struct hdmi_info *hdmi, u8 reg, u8 pattern)
{
	return hdmi_write_byte(hdmi->client, reg,
		hdmi_read(hdmi->client, reg) | pattern);
}
////
void ReadModifyWriteTPI(struct hdmi_info *hdmi, u8 Offset, u8 Mask, u8 Value)
{
	u8 Tmp;
	struct i2c_client *client = hdmi->client;

	Tmp = hdmi_read(client, Offset);
	Tmp &= ~Mask;
	Tmp |= (Value & Mask);
	hdmi_write_byte(client, Offset, Tmp);
}
////
void ReadClearWriteTPI(struct hdmi_info *hdmi, u8 Offset, u8 Pattern)
{
	u8 Tmp;

	Tmp = hdmi_read(hdmi->client, Offset);
	Tmp &= ~Pattern;
	hdmi_write_byte(hdmi->client, Offset, Tmp);
}
void tpi_clear_bit(struct hdmi_info *hdmi, u8 reg, u8 pattern)
{
	hdmi_write_byte(hdmi->client, reg,
		hdmi_read(hdmi->client, reg) & pattern);
}
////

/* Caller: ChangeVideoMode(), HDCP_Poll(), HotPlugServiceLoop(), RestartHDCP()
 */

void EnableTMDS(struct hdmi_info *hdmi)
{
	u8 val;
#if 1
	/* 0x1A[4] = 0 */
	ReadClearWriteTPI(hdmi, TPI_SYSTEM_CONTROL, BIT_TMDS_OUTPUT);

	if (edid_check_sink_type(hdmi))
		hdmi_write_byte(hdmi->client, 0x26,
			hdmi_read(hdmi->client, 0x26) & ~0x10);

#else
	struct i2c_client *client = hdmi->i2c_client;

	val = hdmi_read(client, TPI_SYSTEM_CONTROL);
	hdmi_write_byte(client, TPI_SYSTEM_CONTROL, val & ~BIT_TMDS_OUTPUT);
	HDMI_DBG("%s, reg 0x1a: %02x->%02x\n", __func__,
		val, val & ~BIT_TMDS_OUTPUT); 
#endif
	
}

/* Caller: ChangeVideoMode(), HDCP_Poll(), TPI_Poll(), RestartHDCP(),
 * 	OnHdmiCableDisconnected()
 */

void DisableTMDS(struct hdmi_info *hdmi)
{
	/* 0x1A[4] = 1 */
	//ReadClearWriteTPI(hdmi, TPI_SYSTEM_CONTROL, BIT_TMDS_OUTPUT);
	ReadSetWriteTPI(hdmi, TPI_SYSTEM_CONTROL, BIT_TMDS_OUTPUT);
}
static void OnDownstreamRxPoweredDown(struct hdmi_info *hdmi)
{
	HDMI_DBG("%s\n", __func__);
	dsRxPoweredUp = false;
	hdcp_off(hdmi);
}

static void OnDownstreamRxPoweredUp(struct hdmi_info *hdmi)
{
        HDMI_DBG("%s\n", __func__);
	dsRxPoweredUp = true;
	HotPlugService(hdmi);
#ifdef CONFIG_HTC_HEADSET_MGR
	/* send cable in event */
	switch_send_event(BIT_HDMI_CABLE, 1);
	HDMI_DBG("Cable inserted.\n");
#endif
	pvid_mode = vid_mode;
        hdmi_active9022_dup(hdmi->client);
}

bool GetDDC_Access(struct hdmi_info *hdmi, u8* SysCtrlRegVal)
{
	u8 sysCtrl, TPI_ControlImage, DDCReqTimeout = T_DDC_ACCESS;

	HDMI_DBG("%s\n", __func__);
	/* Read and store original value. Will be passed into ReleaseDDC() */
	sysCtrl = hdmi_read(hdmi->client, TPI_SYSTEM_CONTROL);
	*SysCtrlRegVal = sysCtrl;

	sysCtrl |= BIT_DDC_BUS_REQ;
	hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, sysCtrl);

	/* Loop till 0x1A[1] reads "1" */
	while (DDCReqTimeout--) {
		TPI_ControlImage = hdmi_read(hdmi->client, TPI_SYSTEM_CONTROL);

		/* When 0x1A[1] reads "1" */
		if (TPI_ControlImage & BIT_DDC_BUS_GRANT) {
			sysCtrl |= BIT_DDC_BUS_GRANT;
			/* lock host DDC bus access (0x1A[2:1] = 11) */
			hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, sysCtrl);
			return true;
		}
		/* 0x1A[2] = "1" - Requst the DDC bus */
		hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, sysCtrl);
		mdelay(200);
	}

	/* Failure... restore original value. */
	hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, sysCtrl);
	return false;
}

bool ReleaseDDC(struct hdmi_info *hdmi, u8 SysCtrlRegVal)
{
	u8 DDCReqTimeout = T_DDC_ACCESS, TPI_ControlImage;

	HDMI_DBG("%s\n", __func__);
	/* Just to be sure bits [2:1] are 0 before it is written */
	SysCtrlRegVal &= ~(0x6);
	/* Loop till 0x1A[1] reads "0" */
	while (DDCReqTimeout--) {
		/* Cannot use ReadClearWriteTPI() here. A read of
		 * TPI_SYSTEM_CONTROL is invalid while DDC is granted.
		 * Doing so will return 0xFF, and cause an invalid value to be
		 * written back. 
		 */
		/* 0x1A[2:1] = "0" - release the DDC bus */
		//ReadClearWriteTPI(TPI_SYSTEM_CONTROL,BITS_2_1);

		hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, SysCtrlRegVal);
		TPI_ControlImage = hdmi_read(hdmi->client, TPI_SYSTEM_CONTROL);
		/* When 0x1A[2:1] read "0" */
		if (!(TPI_ControlImage & 0x6))
			return true;
	}

	/* Failed to release DDC bus control */
	return false;
}

int tpi_read_edid(struct hdmi_info *hdmi)
{
	u8 SysCtrlReg;
	int ret, edid_blocks = 0;
	struct i2c_msg msg;
	u8 i2c_buff[2];
	u8 pbuf[] = {1, 0, 1, 128} ;

	struct i2c_msg paging_msg[] = {
		{
			.addr = 0x30, .flags = 0, .len = 1, .buf = &pbuf[0],
		},
		{
			.addr = 0x50, .flags = 0, .len = 1, .buf = &pbuf[1],
		},
		{	//Block-2
			.addr = 0x50, .flags = I2C_M_RD, .len = 128, .buf = &hdmi->edid_buf[256],
		},
                {
                        .addr = 0x30, .flags = 0, .len = 1, .buf = &pbuf[2],
                },
                {
                        .addr = 0x50, .flags = 0, .len = 1, .buf = &pbuf[3],
                },
                {       //Block-3
                        .addr = 0x50, .flags = I2C_M_RD, .len = 128, .buf = &hdmi->edid_buf[384],
                },
	};

	HDMI_DBG("%s\n", __func__);
#if 0
	DisableTMDS(hdmi);
#else
	u8 val;
	val = hdmi_read(hdmi->client, TPI_SYSTEM_CONTROL);
	//hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, val|BIT_4|BIT_6);
	hdmi_write_byte(hdmi->client, TPI_SYSTEM_CONTROL, val|BIT_4);
#endif

	if (!GetDDC_Access(hdmi, &SysCtrlReg)) {
		pr_err("%s: DDC bus request failed\n", __func__);
		return DDC_BUS_REQ_FAILURE;
	}

	// Block-0
	memset(hdmi->edid_buf, 0, 512);

	msg.addr = 0x50;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = hdmi->edid_buf;
	ret = i2c_transfer(hdmi->client->adapter, &msg, 1);
	if (ret < 0)
		dev_err(&hdmi->client->dev, "%s: i2c transfer error\n", __func__);

	msg.addr = 0x50;
	msg.flags = I2C_M_RD;
	msg.len = 128;
	msg.buf = hdmi->edid_buf;
	ret = i2c_transfer(hdmi->client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&hdmi->client->dev, "%s: i2c transfer error\n", __func__);
                goto end_read_edid;
	} else {
		if (hdmi->edid_buf[0x7e] <= 3)
			edid_blocks = hdmi->edid_buf[0x7e] ;

                dev_info(&hdmi->client->dev, "EDID blocks = %d\n", edid_blocks);

		if (edid_blocks == 0 ) {
                        goto end_read_edid;
                }
		// Block-1
		msg.addr = 0x50;
		msg.flags = 0;
		msg.len = 1;
		i2c_buff[0] = 128;
		msg.buf = i2c_buff;
		ret = i2c_transfer(hdmi->client->adapter, &msg, 1);

		msg.addr = 0x50;
		msg.flags = I2C_M_RD;
		msg.len = 128;
		msg.buf = &hdmi->edid_buf[128];
		ret = i2c_transfer(hdmi->client->adapter, &msg, 1);
	}

	if (edid_blocks > 1) {
		// block 2/3
		i2c_transfer(hdmi->client->adapter, paging_msg, 3);
		i2c_transfer(hdmi->client->adapter, &paging_msg[3], 3);
	}

end_read_edid:
	if (!ReleaseDDC(hdmi, SysCtrlReg)) {
		pr_err("%s: DDC bus release failed\n", __func__);
		return DDC_BUS_REQ_FAILURE;
	}

	edid_simple_parsing(hdmi);

	return 0;
}

//////////////////////////////////////////////////////////////////////////////
// FUNCTION     :   HotPlugService()
// PURPOSE      :   Implement Hot Plug Service Loop activities
// INPUT PARAMS :   None
// OUTPUT PARAMS:   void
// GLOBALS USED :   LinkProtectionLevel
// RETURNS      :   An error code that indicates success or cause of failure
//////////////////////////////////////////////////////////////////////////////

extern bool HDCP_TxSupports;
static bool tmdsPoweredUp;
void HotPlugService (struct hdmi_info *hdmi)
{
        HDMI_DBG("%s\n", __func__);

	mutex_lock(&hdmi->lock);
	tpi_disable_interrupts(hdmi, 0xFF);

/*
	// use 1st mode supported by sink
	//vid_mode = EDID_Data.VideoDescriptor[0];
	vid_mode = 0;
*/
	avc_init_video(hdmi, vid_mode, X1, AFTER_INIT);

	hdmi_write_byte(hdmi->client, HDMI_POWER, 0);
	if (edid_check_sink_type(hdmi))
		avc_send_avi_info_frames(hdmi);

	/* This check needs to be changed to if HDCP is required by the content 
	   once support has been added by RX-side library. */
	if (HDCP_TxSupports == true) {
		HDMI_DBG("TMDS -> Enabled\n");
                /* turn on black mode will lost around 3 secs frames thus remove it */
		//SetInputColorSpace(hdmi, INPUT_COLOR_SPACE_BLACK_MODE);
#if 1
		ReadModifyWriteTPI(hdmi, TPI_SYSTEM_CONTROL,
			LINK_INTEGRITY_MODE_MASK | TMDS_OUTPUT_CONTROL_MASK,
			LINK_INTEGRITY_DYNAMIC | TMDS_OUTPUT_CONTROL_ACTIVE);
#else
		ReadModifyWriteTPI(hdmi, TPI_SYSTEM_CONTROL,
			LINK_INTEGRITY_MODE_MASK | TMDS_OUTPUT_CONTROL_MASK,
			LINK_INTEGRITY_DYNAMIC);
#endif
		tmdsPoweredUp = true;
	} else {
		EnableTMDS(hdmi);
	}

	if (edid_check_sink_type(hdmi))
		avc_set_basic_audio(hdmi);
	else
		SetAudioMute(hdmi, AUDIO_MUTE_MUTED);

	tpi_enable_interrupts(hdmi, HOT_PLUG_EVENT | RX_SENSE_EVENT |
		AUDIO_ERROR_EVENT | SECURITY_CHANGE_EVENT |
		V_READY_EVENT | HDCP_CHANGE_EVENT);

	//complete(&hdmi->hotplug_completion);
	mutex_unlock(&hdmi->lock);
}

static bool tpi_start(struct hdmi_info *hdmi)
{
	u8 devID = 0x00;
	u16 wID = 0x0000;

	hdmi_write_byte(hdmi->client, TPI_ENABLE, 0x00);            // Write "0" to 72:C7 to start HW TPI mode
	mdelay(100);

	devID = tpi_read_backdoor_register(hdmi, 0x00, 0x03);
	wID = devID;
	wID <<= 8;
	devID = tpi_read_backdoor_register(hdmi, 0x00, 0x02);
	wID |= devID;
	devID = hdmi_read(hdmi->client, TPI_DEVICE_ID);
	HDMI_DBG("%s, ID=%04X\n", __func__, (u32)wID);

	if (devID == SiI_DEVICE_ID) {
		return true;
	}

	pr_err("%s: Unsupported TX\n", __func__);
	return false;
}

bool tpi_init(struct hdmi_info *hdmi)
{
	tmdsPoweredUp = false;
	hdmi->cable_connected = false;
	dsRxPoweredUp = false;
	edidDataValid = false;

	/* Enable HW TPI mode, check device ID */
	if (tpi_start(hdmi)) {	
		hdcp_init(hdmi);
		return true;
	}
	return 0;
}

void SetAudioMute(struct hdmi_info *hdmi, u8 audioMute)
{
	ReadModifyWriteTPI(hdmi, TPI_AUDIO_INTERFACE_REG, AUDIO_MUTE_MASK, audioMute);
}

void SetInputColorSpace(struct hdmi_info *hdmi, u8 inputColorSpace)
{
	ReadModifyWriteTPI(hdmi, TPI_INPUT_FORMAT_REG, INPUT_COLOR_SPACE_MASK, inputColorSpace);
	/* Must be written for previous write to take effect. Just write read value unmodified. */
	ReadModifyWriteTPI(hdmi, TPI_END_RIGHT_BAR_MSB, 0x00, 0x00);
}

static char edid_hex_buff[2048];
int lcdc_enable_video(void);
int lcdc_disable_video(void);
void tpi_cable_conn(struct hdmi_info *hdmi)
{
	HDMI_DBG("%s\n", __func__);

	hdmi->cable_connected = true;
	tpi_write_backdoor_register(hdmi, INTERNAL_PAGE_0, 0xCE, 0x00); // Clear BStatus
	tpi_write_backdoor_register(hdmi, INTERNAL_PAGE_0, 0xCF, 0x00);

//-----------------------------------------------
	hdmi_write_byte(hdmi->client, 0x09, 0x03);
	hdmi_write_byte(hdmi->client, 0x19, 0x00); // go to blank mode, avoid screen noise

/*
	HDMI_DBG("solomon: H/V total=%02x, %02x, %02x, %02x\n",
		hdmi_read(hdmi->client, 0x6a),
		hdmi_read(hdmi->client, 0x6b),
		hdmi_read(hdmi->client, 0x6c),
		hdmi_read(hdmi->client, 0x6d)
		);
*/

	lcdc_enable_video();
	msleep(160);
/*
	//clk_set_rate(hdmi->ebi1_clk, 120000000);
	HDMI_DBG("solomon: H/V total=%02x, %02x, %02x, %02x\n",
		hdmi_read(hdmi->client, 0x6a),
		hdmi_read(hdmi->client, 0x6b),
		hdmi_read(hdmi->client, 0x6c),
		hdmi_read(hdmi->client, 0x6d)
		);
*/
	EnableTMDS(hdmi);	
	
//-----------------------------------------------

	tpi_read_edid(hdmi);
	memset(edid_hex_buff, 0, 2048);
	edid_dump_hex(hdmi->edid_buf, 256, edid_hex_buff, 2048);
	printk("EDID data:\n%s\n=====", edid_hex_buff);
	/* select output mode (HDMI/DVI) according to sink capabilty */
	if (edid_check_sink_type(hdmi))
		ReadModifyWriteTPI(hdmi, TPI_SYSTEM_CONTROL, OUTPUT_MODE_MASK, OUTPUT_MODE_HDMI);
	else
		ReadModifyWriteTPI(hdmi, TPI_SYSTEM_CONTROL, OUTPUT_MODE_MASK, OUTPUT_MODE_DVI);

	hdmi->first = false;
#if 0
#ifdef CONFIG_HTC_HEADSET_MGR
	/* send cable in event */
	switch_send_event(BIT_HDMI_CABLE, 1);
	HDMI_DBG("Cable inserted.\n");
#endif
#endif
}

void tpi_cable_disconn(struct hdmi_info *hdmi, bool into_d3)
{
	HDMI_DBG("%s, into_d3=%d\n", __func__, into_d3);

	hdmi->cable_connected = false;
	dsRxPoweredUp = false;
	edidDataValid = false;
	hdcp_off(hdmi);
	DisableTMDS(hdmi);
#if 1
        /* wait for debounce */
        msleep(20);
	tpi_clear_pending_event(hdmi);
#else
	reg = hdmi_read(hdmi->client, 0x3d);
	if (!(reg & 0x0c))
		tpi_clear_pending_event(hdmi);
#endif
	if (into_d3) {
		mutex_lock(&hdmi->lock);
		HDMI_DBG("%s, playing=%d\n", __func__, hdmi->user_playing);
		if (false == hdmi->user_playing)
			lcdc_disable_video();
		clk_set_rate(hdmi->ebi1_clk, 0);
		hdmi_standby(hdmi);
		hdmi->power(2);
		memset(hdmi->edid_buf, 0, 512);
		mutex_unlock(&hdmi->lock);
	}
#ifdef CONFIG_HTC_HEADSET_MGR
	HDMI_DBG("Cable unplugged.\n");
	switch_send_event(BIT_HDMI_CABLE, 0);
#endif
}

static char *str_debug_interrupt[] = {
	"HOT_PLUG_EVENT\t\t\t",
	"RECEIVER_SENSE_EVENT\t\t",
	"HOT_PLUG_PIN_STATE\t\t",
	"RX_SENSE_MASK\t\t\t",
	"AUDIO_ERROR_EVENT\t\t",
	"HDCP_SECURITY_CHANGE_EVENT\t",
	"HDCP_VPRIME_VALUE_READY_EVENT\t",
	"HDCP_AUTH_STATUS_CHANGE_EVENT\t",
};

void tpi_debug_interrupt(struct hdmi_info *hdmi, u8 old_status, u8 new_status)
{
	int i, diff, on_off;
	HDMI_DBG("%s: status changed, %02x to %02x\n", __func__,
		old_status, new_status);
	for (i = 7; i >= 0; i--) {
		diff = (old_status ^ new_status) & (1 << i);
		if (!diff)
			continue;
		on_off = new_status & (1 << i);
		HDMI_DBG("%d-%s->%s\n", i, str_debug_interrupt[i],
			on_off ? "on" : "off");
	}
}
//////////////////////////////////////////////////////////////////////////////
// FUNCTION     :   TPI_Poll ()
// PURPOSE      :   Poll Interrupt Status register for new interrupts
// INPUT PARAMS :   None
// OUTPUT PARAMS:   None
// GLOBALS USED :   LinkProtectionLevel
// RETURNS      :   None
//////////////////////////////////////////////////////////////////////////////
static u8 last_status = 0;
static void tpi_poll(struct hdmi_info *hdmi)
{
	u8 status, orig_status;
	int retry = 20;

	mutex_lock(&hdmi->polling_lock);
	orig_status = status = hdmi_read(hdmi->client, TPI_INTERRUPT_STATUS_REG);
	if (last_status != status) {
		tpi_debug_interrupt(hdmi, last_status, status);
	}
	last_status = status;
        DLOG(DBG_POLLING, "%s, INT_STAT=%02x\n", __func__, status);
#if 0
	if (status & HOT_PLUG_EVENT) {
#else
	if (hdmi->first || status & HOT_PLUG_EVENT) {
		if (hdmi->first) hdmi->first = false;
#endif
		// Enable HPD interrupt bit
		ReadSetWriteTPI(hdmi, TPI_INTERRUPT_ENABLE_REG, HOT_PLUG_EVENT);
		// Repeat this loop while cable is bouncing:
		do {
			//DLOG(DBG_POLLING, "TPI: Interrupt status image - 2= %02x\n", status);
			hdmi_write_byte(hdmi->client, TPI_INTERRUPT_STATUS_REG, HOT_PLUG_EVENT);
			// Delay for metastability protection and to help filter out connection bouncing
			mdelay(T_HPD_DELAY);
			// Read Interrupt status register
			status = hdmi_read(hdmi->client, TPI_INTERRUPT_STATUS_REG);
			//DLOG(DBG_POLLING, "TPI: Interrupt status image - 3= %02x\n", status);
			if (!retry--) { 
				HDMI_DBG("%s: retry failed\n", __func__);
				break;
			}

		} while (status & HOT_PLUG_EVENT);// loop as long as HP interrupts recur
		DLOG(DBG_POLLING, "int status: %02x, after debouncing: %02x\n",
			orig_status, status);

		//DLOG(DBG_POLLING, "TPI->hdmiCableConnected = %d\n", hdmi->cable_connected);
		if (((status & HOT_PLUG_STATE) >> 2) != hdmi->cable_connected) {
			DLOG(DBG_POLLING, "cable status changed: from %d to %d\n",
				hdmi->cable_connected, !!(status & HOT_PLUG_STATE));
			//DLOG(DBG_POLLING, "TPI-> CONDITION\n");
			if (hdmi->cable_connected == true)
				tpi_cable_disconn(hdmi, status & 0x8 ? false : true);
			else {
				tpi_cable_conn(hdmi);
				ReadModifyWriteIndexedRegister(hdmi, INDEXED_PAGE_0, 0x0A, 0x08, 0x08);
			}
			if (hdmi->cable_connected == false) {
				mutex_unlock(&hdmi->polling_lock);
				return;
			}
		} else if ( false == hdmi->cable_connected)
			/* only occur while booting without cable attached. */
			tpi_cable_disconn(hdmi, true);
	}

	// Check rx power
	if (((status & RX_SENSE_STATE) >> 3) != dsRxPoweredUp)
	{
		if (hdmi->cable_connected == true) {
			if (dsRxPoweredUp == true)
				OnDownstreamRxPoweredDown(hdmi);
			else
				OnDownstreamRxPoweredUp(hdmi);
		}
		tpi_clear_interrupt(hdmi, RX_SENSE_EVENT);
	}

	// Check if Audio Error event has occurred:
	if (status & AUDIO_ERROR_EVENT)
		//  The hardware handles the event without need for host intervention (PR, p. 31)
		tpi_clear_interrupt(hdmi, AUDIO_ERROR_EVENT);

	if (hdmi->video_streaming) {
		if ((hdmi->cable_connected == true) && (dsRxPoweredUp == true))
			hdcp_check_status(hdmi, status);
	}
	mutex_unlock(&hdmi->polling_lock);
}

static void tpi_work_func(struct work_struct *work)
{
	u8 reg = 0;
	struct hdmi_info *hdmi =
		container_of(work, struct hdmi_info, polling_work);

	if (hdmi->sleeping == SLEEP) {
		mutex_lock(&hdmi->lock);
		hdmi->power(3);
		hdmi_wakeup(hdmi);
		tpi_init(hdmi);
		hdcp_off(hdmi);
		mutex_unlock(&hdmi->lock);
	}

	tpi_poll(hdmi);
#if 1
	mutex_lock(&hdmi->lock);
	if (hdmi->sleeping == AWAKE)
		reg = hdmi_read(hdmi->client, 0x3d) & 0x0c;
	if (hdmi->cable_connected || reg) {
		hdmi->polling = true;
		mod_timer(&hdmi->timer, jiffies + INTERVAL_HDCP_POLLING);
	} else {
		enable_irq(hdmi->client->irq);
		hdmi->isr_enabled = true;
		hdmi->polling = false;
	}
	mutex_unlock(&hdmi->lock);
#else
        if (hdmi->sleeping == AWAKE) {
		reg = hdmi_read(hdmi->client, 0x3d);
		if (reg & 0x0c) {
                	hdmi->polling = true;
                	mod_timer(&hdmi->timer, jiffies + INTERVAL_HDCP_POLLING);
		} else {
			tpi_clear_pending_event(hdmi);
		}	
	}
		
        if (hdmi->cable_connected ) {
                hdmi->polling = true;
                mod_timer(&hdmi->timer, jiffies + INTERVAL_HDCP_POLLING);
        } else {
                enable_irq(hdmi->client->irq);
                hdmi->isr_enabled = true;
                hdmi->polling = false;
        }
#endif
/*
	HDMI_DBG("after polling: reg=%02x, conn=%d, isr=%d, polling=%d\n",
		reg, hdmi->cable_connected, hdmi->isr_enabled, hdmi->polling);
*/
}

static void tpi_timer_func(unsigned long arg)
{
	struct hdmi_info *hdmi = (struct hdmi_info *) arg;

        schedule_work(&hdmi->polling_work);
}

int tpi_prepare(struct hdmi_info *hdmi)
{
        HDMI_DBG("%s\n", __func__);
        init_timer(&hdmi->timer);
        hdmi->timer.data = (unsigned long)hdmi;
        hdmi->timer.function = tpi_timer_func;
	hdmi->cable_connected = false;

	init_completion(&hdmi->hotplug_completion);
        INIT_WORK(&hdmi->polling_work, tpi_work_func);

        return 0;
}

/*============================================================================*/
#if defined(HDMI_DEBUGFS)
static ssize_t tpi_dbg_open(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
        return 0;
}

static ssize_t tpi_ddc_request_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        //struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;
	return 0;
}

static ssize_t tpi_ddc_request_write(struct file *filp, const char __user *buf,
                size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t tpi_isr_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n = 0;
        char buffer[4];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        HDMI_DBG("%s\n", __func__);
        n = scnprintf(buffer, 4, "%d\n", hdmi->isr_enabled);
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t tpi_polling_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n = 0;
        char buffer[4];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        HDMI_DBG("%s\n", __func__);
        n = scnprintf(buffer, 4, "%d\n", hdmi->polling);
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t tpi_int_status_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n = 0;
        char buffer[8];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        HDMI_DBG("%s\n", __func__);
        n = scnprintf(buffer, 8, "%02x\n", hdmi_read(hdmi->client, 0x3d));
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t tpi_int_enable_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n = 0;
        char buffer[8];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        HDMI_DBG("%s\n", __func__);
        n = scnprintf(buffer, 8, "%02x\n", hdmi_read(hdmi->client, 0x3c));
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static ssize_t tpi_avc_read(struct file *filp, char __user *buf,
                size_t count, loff_t *ppos)
{
        int n = 0;
        char buffer[8];
        struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

        HDMI_DBG("%s\n", __func__);
/*
        n = scnprintf(buffer, 8, "%02x\n", hdmi_read(hdmi->client, 0x3c));
        n++;
        buffer[n] = 0;
        return simple_read_from_buffer(buf, count, ppos, buffer, n);
*/
	hdmi_active9022(hdmi->client);
	return 0;
}

static struct file_operations tpi_debugfs_fops[] = {
        {
                .open  = tpi_dbg_open,
                .read  = tpi_ddc_request_read,
                .write = tpi_ddc_request_write,
        },
	{
                .open  = tpi_dbg_open,
                .read  = tpi_isr_read,
	},
	{
                .open  = tpi_dbg_open,
                .read  = tpi_polling_read,
	},
        {
                .open  = tpi_dbg_open,
                .read  = tpi_int_status_read,
        },
        {
                .open  = tpi_dbg_open,
                .read  = tpi_int_enable_read,
        },
        {
                .open  = tpi_dbg_open,
                .read  = tpi_avc_read,
        },
};

int tpi_debugfs_init(struct hdmi_info *hdmi)
{
        struct dentry *tpi_dent;

        tpi_dent = debugfs_create_dir("tpi", hdmi->debug_dir);
        if (IS_ERR(tpi_dent))
                return PTR_ERR(tpi_dent);

	//FIXME: error handling
        debugfs_create_file("ddc_request", 0644, tpi_dent, hdmi,
                &tpi_debugfs_fops[0]);
        debugfs_create_file("isr_enabled", 0444, tpi_dent, hdmi,
                &tpi_debugfs_fops[1]);
        debugfs_create_file("polling", 0444, tpi_dent, hdmi,
                &tpi_debugfs_fops[2]);
        debugfs_create_file("int_stat", 0444, tpi_dent, hdmi,
                &tpi_debugfs_fops[3]);
        debugfs_create_file("int_ena", 0444, tpi_dent, hdmi,
                &tpi_debugfs_fops[4]);
        debugfs_create_file("avc", 0444, tpi_dent, hdmi,
                &tpi_debugfs_fops[5]);

        return 0;
}
#endif
