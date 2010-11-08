#ifndef __SIL902X_H_
#define __SIL902X_H_

#include <linux/htc_hdmi.h>
#include <mach/msm_fb.h>
#include "edid.h"

struct hdmi_info {
        struct hdmi_device hdmi_dev;
        struct i2c_client *client;
        struct msm_lcdc_panel_ops hdmi_lcdc_ops;
        struct work_struct work;
        struct delayed_work hdmi_delay_work;
        struct mutex lock;
        struct mutex lock2;
        struct clk *ebi1_clk;
        int (*power)(int on);
        void (*hdmi_gpio_on)(void);
        void (*hdmi_gpio_off)(void);
        enum hd_res res;
	// FIXME: move to edid_info_struct
        u8 edid_buf[128 * 4];
        enum {
                SLEEP,
                AWAKE,
        } sleeping;
        bool                    polling;
        bool                    cable_connected;
        bool                    isr_enabled;
        bool                    first;
	struct completion	hotplug_completion;
        struct timer_list       timer;
        struct work_struct      polling_work;
	struct dentry           *debug_dir;
	struct edid_info_struct	edid_info;
        struct mutex polling_lock;
	bool			suspending;
        bool                    user_playing;
        bool                    video_streaming;
};

enum {
	HDMI_PIXEL_DATA		= 0x08,
	HDMI_AVI_INFO_FRAME	= 0x0c,
	HDMI_AUDIO_INFO_FRAME	= 0xbf,
	HDMI_SYS_CTL 		= 0x1a,
	HDMI_POWER 		= 0x1e,
	HDMI_IDENTIFY 		= 0x1b,
	HDMI_INT_EN 		= 0x3c,
	HDMI_INT_STAT 		= 0x3d,
	HDMI_EN_REG		= 0xc7,
};

/* flag bitmap for register HDMI_INT_STAT */
enum {
	HOT_PLUG_PENDING 	= (1U << 0),
	RX_PENDING 		= (1U << 1),
	HOT_PLUG_STATE 		= (1U << 2),
	RX_STATE 		= (1U << 3),
	AUDIO_ERR 		= (1U << 4),
	SECURITY_STATE 		= (1U << 5),
	HDCP_VALUE 		= (1U << 6),
	HDCP_AUTH 		= (1U << 7),
};

enum ErrorMessages {
	INIT_SYSTEM_SUCCESSFUL,                     // 0

	BLACK_BOX_OPEN_FAILURE,
	BLACK_BOX_OPENED_SUCCESSFULLY,
	HW_RESET_FAILURE,
	TPI_ENABLE_FAILURE,
	INTERRUPT_EN_FAILURE,
	INTERRUPT_POLLING_FAILED,

	NO_SINK_CONNECTED,
	DDC_BUS_REQ_FAILURE,
	HDCP_FAILURE,
	HDCP_OK,                                    // 10
	RX_AUTHENTICATED,
	SINK_DOES_NOT_SUPPORT_HDCP,
	TX_DOES_NOT_SUPPORT_HDCP,

	ILLEGAL_AKSV,
	SET_PROTECTION_FAILURE,
	REVOKED_KEYS_FOUND,
	REPEATER_AUTHENTICATED,
	INT_STATUS_READ_FAILURE,

	PROTECTION_OFF_FAILED,
	PROTECTION_ON_FAILED,                       // 20
	INTERRUPT_POLLING_OK,

	EDID_PARSING_FAILURE,
	VIDEO_SETUP_FAILURE,
	TPI_READ_FAILURE,
	TPI_WRITE_FAILURE,

	INIT_VIDEO_FAILURE,
	DE_CANNOT_BE_SET_WITH_EMBEDDED_SYNC,
	SET_EMBEDDED_SYC_FAILURE,
	V_MODE_NOT_SUPPORTED,

	AUD_MODE_NOT_SUPPORTED,                     // 30
	I2S_NOT_SET,

	EDID_READ_FAILURE,
	EDID_CHECKSUM_ERROR,
	INCORRECT_EDID_HEADER,
	EDID_EXT_TAG_ERROR,

	EDID_REV_ADDR_ERROR,
	EDID_V_DESCR_OVERFLOW,
	INCORRECT_EDID_FILE,
	UNKNOWN_EDID_TAG_CODE,
	NO_DETAILED_DESCRIPTORS_IN_THIS_EDID_BLOCK, // 40
	CONFIG_DATA_VALID,
	CONFIG_DATA_INVALID,

	GPIO_ACCESS_FAILED,
	GPIO_CONFIG_ERROR,

	HP_EVENT_GOING_TO_SERVICE_LOOP,
	EDID_PARSED_OK,
	VIDEO_MODE_SET_OK,
	AUDIO_MODE_SET_OK,

	I2S_MAPPING_SUCCESSFUL,
	I2S_INPUT_CONFIG_SUCCESSFUL,                // 50
	I2S_HEADER_SET_SUCCESSFUL,
	INTERRUPT_POLLING_SUCCESSFUL,

	HPD_LOOP_EXITED_SUCCESSFULY,
	HPD_LOOP_FAILED,
	SINK_CONNECTED,
	HP_EVENT_RETURNING_FROM_SERVICE_LOOP,
	AVI_INFOFRAMES_SETTING_FAILED,

	TMDS_ENABLING_FAILED,
	DE_SET_OK,
	DE_SET_FAILED,
	NO_861_EXTENSIONS,

	GO_OK,
	IMAGE_PKTS_UPDATED_OK,
	MONITORING_BLOCKED,
	LINK_NORMAL,
	LINK_LOST,
	RENEGOTIATION_REQUIRED,

	LINK_SUSPENDED,
	EDID_SHORT_DESCRIPTORS_PARSED_OK,
	EDID_LONG_DESCRIPTORS_PARSED_OK,
	DDC_BUS_RELEASE_FAILURE,
	FAILED_GETTING_BKSV,

	PLL_SETUP_FAILUE,
	ERR_RX_QUEUE_FULL,
	ERR_TX_QUEUE_FULL,
	GBD_SET_SUCCESSFULLY,
	BACKDOOR_SETTING_FAILED,
	ERR_TX_QUEUE_EMPTY
};

#define BIT_0                   0x01
#define BIT_1                   0x02
#define BIT_2                   0x04
#define BIT_3                   0x08
#define BIT_4                   0x10
#define BIT_5                   0x20
#define BIT_6                   0x40
#define BIT_7                   0x80

#define INTERVAL_HDCP_POLLING           (HZ / 25)

#define REQUEST_RELEASE_DDC_BEFORE_HDCP
#define T_HDCP_ACTIVATION		500
#define T_HDCP_DEACTIVATION		200

#define T_HPD_DELAY		10
#define TPI_INTERRUPT_EN	0x3c
#define ALL			0xff
#define DUMMY                   0xFD

#define SiI_DEVICE_ID           0xB0

#define T_DDC_ACCESS    50
// TPI Control Masks
// =================
#define BIT_OUTPUT_MODE     0x01
#define BIT_DDC_BUS_GRANT   0x02
#define BIT_DDC_BUS_REQ     0x04
#define BIT_TMDS_OUTPUT     0x10

#define TPI_INTERNAL_PAGE_REG		0xBC
#define TPI_REGISTER_OFFSET_REG		0xBD
#define TPI_REGISTER_VALUE_REG		0xBE

/* HDCP Control Masks */
#define BIT_PROTECT_LEVEL		0x01
#define BIT_PROTECT_TYPE		0x02
#define BIT_REPEATER			0x08
#define BIT_LOCAL_PROTECT		0x40
#define BIT_EXT_PROTECT			0x80

#define BITS_LINK_LOST			0x10
#define BITS_RENEGOTIATION		0x20

#define BIT_TMDS_OUTPUT			0x10

#define BIT_AUDIO_MUTE      0x10

#define TPI_HDCP_REVISION_DATA_REG			(0x30)
#define HDCP_MAJOR_REVISION_MASK			(BIT_7 | BIT_6 | BIT_5 | BIT_4)
#define HDCP_MAJOR_REVISION_VALUE			(0x10)

#define HDCP_MINOR_REVISION_MASK			(BIT_3 | BIT_2 | BIT_1 | BIT_0)
#define HDCP_MINOR_REVISION_VALUE			(0x02)

#define HDCP_REVISION       0x12
#define SET_PROT_ATTEMPTS   0x05

#define AKSV_SIZE			5
#define BYTE_SIZE			8
#define NUM_OF_ONES_IN_KSV    		20

// Interrupt Masks
//================
#define HOT_PLUG_EVENT          0x01
#define RX_SENSE_EVENT          0x02
#define TPI_HOT_PLUG_STATE          0x04
#define RX_SENSE_STATE          0x08

#define AUDIO_ERROR_EVENT       0x10
#define SECURITY_CHANGE_EVENT   0x20
#define V_READY_EVENT           0x40
#define HDCP_CHANGE_EVENT       0x80

#define NON_MASKABLE_INT		0xFF

/* Protection Levels */
#define NO_PROTECTION			0x00
#define LOCAL_PROTECTION		0x01
#define EXTENDED_PROTECTION		0x03

#define LINK_NORMAL			0
#define MAX_V_DESCRIPTORS		20
#define MAX_A_DESCRIPTORS		10
#define MAX_SPEAKER_CONFIGURATIONS	 4

#define HDMI_DEBUGFS_ROOT		"hdmi"

/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ///
/*\
| | HDCP Implementation
| |
| | HDCP link security logic is implemented in certain transmitters; unique
| |   keys are embedded in each chip as part of the solution. The security
| |   scheme is fully automatic and handled completely by the hardware.
\*/
/// HDCP Query Data Register ============================================== ///

#define TPI_HDCP_QUERY_DATA_REG                         (0x29)

#define EXTENDED_LINK_PROTECTION_MASK           (BIT_7)
#define EXTENDED_LINK_PROTECTION_NONE           (0x00)
#define EXTENDED_LINK_PROTECTION_SECURE         (0x80)

#define LOCAL_LINK_PROTECTION_MASK                      (BIT_6)
#define LOCAL_LINK_PROTECTION_NONE                      (0x00)
#define LOCAL_LINK_PROTECTION_SECURE            (0x40)

#define LINK_STATUS_MASK                                        (BIT_5 | BIT_4)
#define LINK_STATUS_NORMAL                                      (0x00)
#define LINK_STATUS_LINK_LOST                           (0x10)
#define LINK_STATUS_RENEGOTIATION_REQ           (0x20)
#define LINK_STATUS_LINK_SUSPENDED                      (0x30)

#define HDCP_REPEATER_MASK                                      (BIT_3)
#define HDCP_REPEATER_NO                                        (0x00)
#define HDCP_REPEATER_YES                                       (0x08)

#define CONNECTOR_TYPE_MASK                                     (BIT_2 | BIT_0)
#define CONNECTOR_TYPE_DVI                                      (0x00)
#define CONNECTOR_TYPE_RSVD                                     (0x01)
#define CONNECTOR_TYPE_HDMI                                     (0x04)
#define CONNECTOR_TYPE_FUTURE                           (0x05)

#define PROTECTION_TYPE_MASK                            (BIT_1)
#define PROTECTION_TYPE_NONE                            (0x00)
#define PROTECTION_TYPE_HDCP                            (0x02)

/// HDCP Control Data Register ============================================ ///

#define TPI_HDCP_CONTROL_DATA_REG                       (0x2A)

#define PROTECTION_LEVEL_MASK                           (BIT_0)
#define PROTECTION_LEVEL_MIN                            (0x00)
#define PROTECTION_LEVEL_MAX                            (0x01)

/*---------------------------------------------------------------------------*/

#if 0
/* Caller: ChangeVideoMode(), HDCP_Poll(), HotPlugServiceLoop(), RestartHDCP()
 */
#define EnableTMDS(hdmi)	ReadClearWriteTPI(hdmi, TPI_SYSTEM_CONTROL, BIT_TMDS_OUTPUT)  // 0x1A[4] = 0

/* Caller: ChangeVideoMode(), HDCP_Poll(), TPI_Poll(), RestartHDCP(),
 * 	   OnHdmiCableDisconnected()
 */
#define DisableTMDS(hdmi)	ReadSetWriteTPI(hdmi, TPI_SYSTEM_CONTROL, BIT_TMDS_OUTPUT)    // 0x1A[4] = 1
#else

void EnableTMDS(struct hdmi_info *hdmi);
void DisableTMDS(struct hdmi_info *hdmi);

#endif

// FIXME: fix the global variables
extern u8 pvid_mode, vid_mode;
extern u8 LinkProtectionLevel;
extern u8 systemInitialized;
/*---------------------------------------------------------------------------*/
int hdmi_read(struct i2c_client *client, u8 cmd);
int hdmi_write_byte(struct i2c_client *client, u8 reg, u8 val);
int hdmi_enable_int(struct i2c_client *client);
int hdmi_disable_int(struct i2c_client *client);
int hdmi_read_edid(struct hdmi_info *info, struct i2c_client *client);
int hdmi_standby(struct hdmi_info *hdmi);
int hdmi_wakeup(struct hdmi_info *hdmi);
int read_backdoor_register(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset);
void ReadSetWriteTPI(struct hdmi_info *hdmi, u8 Offset, u8 Pattern);
void ReadModifyWriteTPI(struct hdmi_info *hdmi, u8 Offset, u8 Mask, u8 Value);
void tpi_clear_interrupt(struct hdmi_info *hdmi, u8 pattern);
bool tpi_init(struct hdmi_info *hdmi);
void ReadClearWriteTPI(struct hdmi_info *hdmi, u8 Offset, u8 Pattern);
s32 ReadBlockTPI(struct hdmi_info *hdmi, u8 TPI_Offset, u16 NBytes, u8 *pData);
bool IsRepeater(struct hdmi_info *hdmi);
bool GetDDC_Access(struct hdmi_info *hdmi, u8* SysCtrlRegVal);
bool ReleaseDDC(struct hdmi_info *hdmi, u8 SysCtrlRegVal);
int tpi_read_backdoor_register(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset)
;
void tpi_write_backdoor_register(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset, u8 RegValue);

int HotPlugServiceLoop(struct hdmi_info *hdmi);

int hdmi_active9022(struct i2c_client *client);
int hdmi_active9022_dup(struct i2c_client *client);
bool avc_send_avi_info_frames(struct hdmi_info *hdmi);
bool avc_init_video(struct hdmi_info *hdmi, u8 mode, u8 TclkSel, bool Init);
//void hdcp_on(struct hdmi_info *hdmi);
void hdcp_off(struct hdmi_info *hdmi);
void hdcp_check_status(struct hdmi_info *hdmi, u8 InterruptStatusImage);
void hdcp_init(struct hdmi_info *hdmi);
int hdcp_debugfs_init(struct hdmi_info *hdmi);

extern u8	EDID_TempData[];
u8 edid_simple_parsing(struct hdmi_info *hdmi);

int edid_dump_hex(u8 *src, int src_size, char *output, int output_size);
bool edid_is_video_mode_supported(struct video_mode *vmode);
int edid_debugfs_init(struct hdmi_info *hdmi);
bool edid_check_sink_type(struct hdmi_info *hdmi);
int HotPlugServiceLoop(struct hdmi_info *hdmi);
int tpi_prepare(struct hdmi_info *hdmi);

//bool InitVideo(struct hdmi_info *hdmi, u8 Mode, u8 TclkSel, bool Init);
void avc_set_basic_audio(struct hdmi_info *hdmi);

int hdmi_debugfs_init(struct hdmi_info *hdmi);
int tpi_debugfs_init(struct hdmi_info *hdmi);
ssize_t hdmi_dbgfs_open(struct inode *inode, struct file *file);

void SetAudioMute(struct hdmi_info *hdmi, u8 audioMute);
void SetInputColorSpace(struct hdmi_info *hdmi, u8 inputColorSpace);

void WriteBackDoorRegister(struct hdmi_info *hdmi, u8 PageNum, u8 RegOffset, u8 RegValue);

#define	TPI_INPUT_FORMAT		0x09
#define	TPI_OUTPUT_FORMAT		0x0A

#define	TPI_SYSTEM_CONTROL		0x1A

#define	TPI_DEVICE_POWER_STATE_CTRL_REG		(0x1E)

#define CTRL_PIN_CONTROL_MASK				(BIT_4)
#define CTRL_PIN_TRISTATE					(0x00)
#define CTRL_PIN_DRIVEN_TX_BRIDGE			(0x10)

#define TX_POWER_STATE_D0					(0x00)
#define TX_POWER_STATE_D1					(0x01)
#define TX_POWER_STATE_D2					(0x02)
#define TX_POWER_STATE_D3					(0x03)

#define	TPI_AUDIO_INTERFACE		0x26

#define	TPI_HDCP_QUERY_DATA		0x29
#define	TPI_HDCP_CTRL			0x2A

#endif
