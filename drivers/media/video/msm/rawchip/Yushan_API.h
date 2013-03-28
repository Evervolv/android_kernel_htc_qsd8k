#ifndef _YUSHAN_API_H_
#define _YUSHAN_API_H_

#ifdef __cplusplus
extern "C"{
#endif   /*__cplusplus*/

/*******************************************************************************
################################################################################
#                             C STMicroelectronics
#    Reproduction and Communication of this document is strictly prohibited
#      unless specifically authorized in writing by STMicroelectronics.
#------------------------------------------------------------------------------
#                             Imaging Division
################################################################################
File Name:	Yushan_API.h
Author:		Rajdeep Patel
Description:API header file.
********************************************************************************/

#include <linux/types.h>
#include <mach/board.h>
#include <linux/platform_device.h>

#include "yushan_registermap.h"
#include "DxODOP_regMap.h"
#include "DxODPP_regMap.h"
#include "DxOPDP_regMap.h"


#include "rawchip_spi.h"


/************************************************************************
						Debugging
************************************************************************/
#define YUSHAN_DEBUG 1
#if YUSHAN_DEBUG
	#ifdef WIN32
		#include <stdio.h>
		extern	FILE	* fLogFile;
		extern	FILE	*HtmlFileLog;
		/* ST debugging log */
		#define __func__  __FUNCTION__
		#define DEBUGLOG(...)  fprintf(HtmlFileLog, __VA_ARGS__)
	#else
		#define DEBUGLOG  pr_info
	#endif
#else
	#define DEBUGLOG(...)	/*  Nothing */
#endif

#define YUSHAN_VERBOSE 0
#if YUSHAN_VERBOSE
	#ifdef WIN32
		#define VERBOSELOG(...)   fprintf(HtmlFileLog, __VA_ARGS__)
	#else
		#define VERBOSELOG  pr_info
	#endif
#else
	#define VERBOSELOG(...)	/*  Nothing */
#endif

#ifdef WIN32
	#if YUSHAN_DEBUG == 0
		#include <stdio.h>
		extern	FILE	*fLogFile;
		extern	FILE	*HtmlFileLog;
		/* ST debugging log */
		#define __func__  __FUNCTION__
	#endif
	#define ERRORLOG(...)   fprintf(HtmlFileLog, __VA_ARGS__)
#else
	#define ERRORLOG  pr_info
#endif


typedef unsigned char bool_t;
//typedef unsigned char uint8_t;
//typedef signed char int8_t;
//typedef unsigned short uint16_t;
//typedef unsigned int uint32_t;
//typedef float float_t;

// Defined for only proto
#define ASIC
#ifdef ASIC
#define PROTO_SPECIFIC						0
#else
#define PROTO_SPECIFIC						1
#endif
#define ST_SPECIFIC							1

#define API_MAJOR_VERSION					12
#define API_MINOR_VERSION					0

#define TRUE								1
#define SUCCESS								1
#define FALSE								0
#define FAILURE								0
// Definition for referring the Lane Id
#define MIPI_CLK_LANE					  1
#define MIPI_DATA_LANE1					2
#define MIPI_DATA_LANE2					3
#define MIPI_DATA_LANE3					4
#define MIPI_DATA_LANE4					5
// Definition for referring the Status of the MIPI lanes
#define MIPI_STOP_STATE					1
#define MIPI_ULP_STATE					2
#define MIPI_ACTIVE_STATE				3



#define RAW8								8
#define	RAW10								10
#define	RAW10_8								108		// Some arbit value just to mark 10 to 8

// Private #defines used in functions internally

#define IDP_GEN_PIX_WIDTH					10
#define DXO_CLK_LIMIT						0x12C0000	// 19660800 // 300*65536 // 16p16 Fixed point notation
#define SYS_CLK_LIMIT						0x0C80000	// 13107200 // 200*65536 // 16p16 Fixed point notation

#define PLL_CLK_INDEX				    	0
#define IDIV_INDEX				    		1
#define ODIV_INDEX		          			2
#define LOOP_INDEX	     					3

#define DXO_MIN_LINEBLANKING				150
#define DXO_MIN_LINELENGTH					1000
#define DXO_ACTIVE_FRAMELENGTH				500000
#define DXO_FULL_FRAMELENGTH				900000


// Default DXO_XXX_IDs (Read from DXO_XXX.h files)
#define DXO_PDP_HW_MICROCODE_ID				((DxOPDP_dfltVal_ucode_id_15_8<<8)|			\
												(DxOPDP_dfltVal_hw_id_7_0<<16)|(DxOPDP_dfltVal_hw_id_15_8<<24))
#define DXO_DOP_HW_MICROCODE_ID				((DxODOP_dfltVal_ucode_id_15_8<<8)|			\
												(DxODOP_dfltVal_hw_id_7_0<<16)|(DxODOP_dfltVal_hw_id_15_8<<24))
#define DXO_DPP_HW_MICROCODE_ID				((DxODPP_dfltVal_ucode_id_15_8<<8)|			\
												(DxODPP_dfltVal_hw_id_7_0<<16)|(DxODPP_dfltVal_hw_id_15_8<<24))

#define DXO_PDP_CALIBRATIONCODE_ID			((DxOPDP_dfltVal_calib_id_0_7_0)|(DxOPDP_dfltVal_calib_id_1_7_0<<8)|	\
												(DxOPDP_dfltVal_calib_id_2_7_0<<16)|(DxOPDP_dfltVal_calib_id_3_7_0<<24))
#define DXO_DOP_CALIBRATIONCODE_ID			((DxODOP_dfltVal_calib_id_0_7_0)|(DxODOP_dfltVal_calib_id_1_7_0<<8)|	\
												(DxODOP_dfltVal_calib_id_2_7_0<<16)|(DxODOP_dfltVal_calib_id_3_7_0<<24))
#define DXO_DPP_CALIBRATIONCODE_ID			((DxODPP_dfltVal_calib_id_0_7_0)|(DxODPP_dfltVal_calib_id_1_7_0<<8)|	\
												(DxODPP_dfltVal_calib_id_2_7_0<<16)|(DxODPP_dfltVal_calib_id_3_7_0<<24))


// *************************** Private #defines end here

// Id of different interrupts in the system. 

#define ADD_INTR_TO_LIST							0x01
#define	DEL_INTR_FROM_LIST							0x00

#define TOTAL_INTERRUPT_SETS						14
#define TOTAL_INTERRUPT_COUNT						83
#define INTERRUPT_SET_SIZE							0x18
#define FALSE_ALARM									0xFF

#define YUSHAN_INTR_BASE_ADDR						0x0C00
#define	YUSHAN_OFFSET_INTR_STATUS					0x00	// Added just for the sake of uniformity
#define	YUSHAN_OFFSET_INTR_ENABLE_STATUS			0x04
#define YUSHAN_OFFSET_INTR_STATUS_CLEAR				0x08
#define YUSHAN_OFFSET_INTR_DISABLE					0x10
#define YUSHAN_OFFSET_INTR_ENABLE					0x14

#define DXO_DOP_BASE_ADDR							0x8000
#define DXO_DPP_BASE_ADDR							0x10000
#define DXO_PDP_BASE_ADDR							0x6000

#define DxODOP_bootAddress                          0x6800
#define DxODPP_bootAddress                          0xd000
#define DxOPDP_bootAddress                          0x1a00

// IP_RESET_IDs

#define DFT_RESET									1
#define T1_DMA										2
#define CSI2_RX										3
#define IT_POINT									4
#define IDP_GEN										5
#define MIPI_RX_DTCHK								6
#define SMIA_PATTERN_GEN							7
#define SMIA_DCPX									8
#define P2W_FIFO_WR									9
#define P2W_FIFO_RD									10
#define CSI2_TX_WRAPPER								11
#define CSI2_TX										12
#define LINE_FILTER_BYPASS							13
#define DT_FILTER_BYPASS							14
#define LINE_FILTER_DXO								15
#define DT_FILTER_DXO								16
#define EOF_RESIZE_PRE_DXO							17
#define LBE_PRE_DXO									18
#define EOF_RESIZE_POST_DXO							19
#define LECCI_RESET									20
#define LBE_POST_DXO								21

#define TOTAL_MODULES								21



// Interrupts
#define EVENT_CSI2RX_ECC_ERR						1
#define EVENT_CSI2RX_CHKSUM_ERR						2
#define EVENT_CSI2RX_SHORTPACKET					3
#define EVENT_CSI2RX_SYNCPULSE_MISSED				4

#define EVENT_PDP_BOOT							    5
#define EVENT_PDP_EOF_EXECCMD				  		6
#define EVENT_DXOPDP_EOP						  	7
#define EVENT_DXOPDP_NEWFRAMECMD_ACK				8
#define EVENT_DXOPDP_NEWFRAMEPROC_ACK				9
#define EVENT_DXOPDP_NEWFRAME_ERR					10
	
#define EVENT_DPP_BOOT							  	11
#define EVENT_DPP_EOF_EXECCMD						12
#define EVENT_DXODPP_EOP							13
#define EVENT_DXODPP_NEWFRAMECMD_ACK				14
#define EVENT_DXODPP_NEWFRAMEPROC_ACK				15
#define EVENT_DXODPP_NEWFRAME_ERR					16

#define EVENT_DOP7_BOOT							  	17
#define EVENT_DOP7_EOF_EXECCMD						18
#define EVENT_DXODOP7_EOP						  	19
#define EVENT_DXODOP7_NEWFRAMECMD_ACK				20
#define EVENT_DXODOP7_NEWFRAMEPROC_ACK				21
#define EVENT_DXODOP7_NEWFRAME_ERR					22

#define EVENT_CSI2TX_SP_ERR							23
#define EVENT_CSI2TX_LP_ERR							24
#define EVENT_CSI2TX_DATAINDEX_ERR					25
#define EVENT_CSI2TX_FRAMEFINISH					26

#define	EVENT_RX_PHY_ERR_SOT_SOFT_DL1				27	
#define	EVENT_RX_PHY_ERR_SOT_HARD_DL1				28
#define	EVENT_RX_PHY_ERR_EOT_DL1					29
#define	EVENT_RX_PHY_ERR_ESC_DL1					30
#define	EVENT_RX_PHY_ERR_CTRL_DL1					31
#define	EVENT_RX_PHY_ULPM_ENTER_DL1					32
#define	EVENT_RX_PHY_ULPM_EXIT_DL1					33

#define	EVENT_RX_PHY_ULPM_ENTER_CLK					34

#define	EVENT_RX_PHY_ERR_SOT_SOFT_DL2				35
#define	EVENT_RX_PHY_ERR_SOT_HARD_DL2				36
#define	EVENT_RX_PHY_ERR_EOT_DL2					37
#define	EVENT_RX_PHY_ERR_ESC_DL2					38
#define	EVENT_RX_PHY_ERR_CTRL_DL2					39
#define	EVENT_RX_PHY_ULPM_ENTER_DL2					40
#define	EVENT_RX_PHY_ULPM_EXIT_DL2					41

#define	EVENT_RX_PHY_ULPM_EXIT_CLK					42
#define	EVENT_RX_PHY_ERR_SOT_SOFT_DL3				43
#define	EVENT_RX_PHY_ERR_SOT_HARD_DL3				44
#define	EVENT_RX_PHY_ERR_EOT_DL3					45 
#define	EVENT_RX_PHY_ERR_ESC_DL3					46 
#define	EVENT_RX_PHY_ERR_CTRL_DL3					47 
#define	EVENT_RX_PHY_ULPM_ENTER_DL3					48 
#define	EVENT_RX_PHY_ULPM_EXIT_DL3					49 

#define	EVENT_RESERVED								50

#define	EVENT_RX_PHY_ERR_SOT_SOFT_DL4				51
#define	EVENT_RX_PHY_ERR_SOT_HARD_DL4				52 
#define	EVENT_RX_PHY_ERR_EOT_DL4					53 
#define	EVENT_RX_PHY_ERR_ESC_DL4					54 
#define	EVENT_RX_PHY_ERR_CTRL_DL4					55 
#define	EVENT_RX_PHY_ULPM_ENTER_DL4					56 
#define	EVENT_RX_PHY_ULPM_EXIT_DL4					57

#define EVENT_TXPHY_CTRL_ERR_D1						58
#define EVENT_TXPHY_CTRL_ERR_D2						59
#define EVENT_TXPHY_CTRL_ERR_D3						60
#define EVENT_TXPHY_CTRL_ERR_D4						61

#define EVENT_UNMATCHED_IMAGE_SIZE_ERROR			62
#define	EVENT_UNMANAGED_DATA_TYPE					63		
#define	PRE_DXO_WRAPPER_PROTOCOL_ERR				64	
#define	PRE_DXO_WRAPPER_FIFO_OVERFLOW				65	
#define EVENT_BAD_FRAME_DETECTION					66	
#define EVENT_TX_DATA_FIFO_OVERFLOW					67	
#define EVENT_TX_INDEX_FIFO_OVERFLOW				68	

#define EVENT_RX_CHAR_COLOR_BAR_0_ERR				69
#define EVENT_RX_CHAR_COLOR_BAR_1_ERR				70
#define EVENT_RX_CHAR_COLOR_BAR_2_ERR				71
#define EVENT_RX_CHAR_COLOR_BAR_3_ERR				72
#define EVENT_RX_CHAR_COLOR_BAR_4_ERR				73
#define EVENT_RX_CHAR_COLOR_BAR_5_ERR				74
#define EVENT_RX_CHAR_COLOR_BAR_6_ERR				75
#define EVENT_RX_CHAR_COLOR_BAR_7_ERR				76

#define	EVENT_POST_DXO_WRAPPER_PROTOCOL_ERR			77	
#define	EVENT_POST_DXO_WRAPPER_FIFO_OVERFLOW		78	

#define EVENT_LINESIZE_REPROGRAM_DONE				79	
#define EVENT_PLL_STABLE							80
#define EVENT_LDO_STABLE							81

#define EVENT_LINE_POSITION_INTR					82		// ITPOINT

#define EVENT_TX_DATA_UNDERFLOW						83		// P2W_DATA_FIFO_UNDERFLOW
#define EVENT_TX_INDEX_UNDERFLOW					84		// P2W_LC_FIFO_UNDERFLOW



#define TIME_5MS				1
#define TIME_10MS				2
#define TIME_20MS				3
#define TIME_50MS				4
#define TIME_200MS				5


#define YUSHAN_FRAME_FORMAT_NORMAL_MODE		0	
#define YUSHAN_FRAME_FORMAT_VF_MODE			1
#define YUSHAN_FRAME_FORMAT_STILL_MODE		2

#define DXO_BOOT_CMD			1


#define RESET_MODULE						1
#define	DERESET_MODULE						0

struct yushan_reg_conf {
	uint16_t addr;
	uint8_t  data;
};

struct yushan_reg_t {
	struct yushan_reg_conf *pdpcode;
	uint16_t pdpcode_size;

	struct yushan_reg_conf *pdpclib;
	uint16_t pdpclib_size;

	struct yushan_reg_conf *dppcode;
	uint16_t dppcode_size;

	struct yushan_reg_conf *dppclib;
	uint16_t dppclib_size;

	struct yushan_reg_conf *dopcode;
	uint16_t dopcode_size;

	struct yushan_reg_conf *dopclib;
	uint16_t dopclib_size;

};

extern struct yushan_reg_t yushan_regs;



// Enum to check DXO constraints
typedef enum
{
	DXO_NO_ERR=100,
	DXO_LINEBLANK_ERR,
	DXO_FULLLINE_ERR, 
	DXO_ACTIVE_FRAMELENGTH_ERR, 
	DXO_FULLFRAMELENGTH_ERR, 
	DXO_FRAMEBLANKING_ERR,
	//DXO_CONSTRAINTS_ERR = 200
	DXO_LOLIMIT_EXCEED_HILIMIT  = 200
}Yushan_DXO_Errors_e;


// Frame format
typedef struct
{    
	uint16_t 	uwWordcount; 
	// The count in bytes of each packet which is valid for 
	// Yushan to recieve
	uint8_t 	bDatatype;
	// The data type of packet corresponding to above WordCount
	uint8_t 	bActiveDatatype;
	// This flag will specify whether the corresponding packet is 
	// Active image data which needs to be passed to DxO for processing
	uint8_t 	bSelectStillVfMode;
	// This flag specifies whether the Datatype and wordcount 
	// identifies it as an Viewfinder mode or Still Mode
	// Possible Value : YUSHAN_VF_MODE
	//			  YUSHAN_STILL_MODE
	//			  YUSHAN_NONE_MODE
	// In the array of this frame format , the values for VF and 
	// STILL should not be duplicated
} Yushan_Frame_Format_t;




// Structure containing mandatory registers, which need to be programmed in the 
// Initialization phase.
typedef	struct
{
	// Number of lanes at Receiver and Transmitter is same
	// Possible Values are 1,2,3
	uint8_t 	bNumberOfLanes;
	// The is the pixel Format coming from the sensor
	// Possible Values : 	Raw10 : 0x0a0a
	//	Raw8  : 0x0808
	//	10to8 : 0x0a08
	uint16_t 	uwPixelFormat;
	// Per Lane  in Mbps. So the MIPI clock would be uwBitRate/2
	uint16_t 	uwBitRate; 
	// External clock.. 16.16 Fixed point notation
	uint32_t 	fpExternalClock; 
	uint32_t	fpSpiClock;
	// Active pixels in a line( Give the worst case here for stills).
	uint16_t	uwActivePixels;
	// Line blanking For Vf
	uint16_t	uwLineBlankVf;
	// Line blanking For Still
	uint16_t	uwLineBlankStill;
	// Number of Active Lines
	uint16_t	uwLines;
	// Number of Lines for FrameBlanking
	uint16_t	uwFrameBlank;
	// Structure containing allowed combination of data type and 
	// word count. The initial release requires it to be available 
	// at start for all viewfinder and still format
	// Need to discuss with HTC if they would like it to be update
	// during run time
	Yushan_Frame_Format_t sFrameFormat[15];
/* Yushan API 10.0 Start */
	// Number of Dxo setting commands per frame
	uint8_t		bDxoSettingCmdPerFrame;
/* Yushan API 10.0 End */
	/* Flag to decide on using External LDO */
	bool_t		bUseExternalLDO;
	
	
}Yushan_Init_Struct_t;


// Structure to load New context data.
typedef struct
{
	// Active pixels in a line( Give the worst case here for stills).
	uint16_t	uwActivePixels;			// HSize
	// Line blanking
	uint16_t	uwLineBlank;
	// Active frame length (VSize): For DXO Image Char
	uint16_t	uwActiveFrameLength;	// VSize;
	// STILL/VF or NORMAL
	uint8_t		bSelectStillVfMode;
	// Similar as the programming in Yushan_Init_Struct_t
	uint16_t	uwPixelFormat;

}Yushan_New_Context_Config_t;



// Page containing pointers to DXO image (UCode). This will be useful for 
// uploading the DXO image.
typedef	struct
{
	uint8_t 	*pDxoPdpRamImage[2];
	// Location where DxO Pdp Ram Image is available
	uint16_t	uwDxoPdpStartAddr;
	// IP First address of Micro code
	uint16_t	uwDxoPdpBootAddr;
	// IP Boot address, where first add of Microcode has to be written
	uint16_t 	uwDxoPdpRamImageSize[2];
	// Size of the Pdp Ram Image
	uint16_t	uwBaseAddrPdpMicroCode[2];
	// Starting Address of Micro code [ Offset withing Pdp Ram]
	uint8_t 	*pDxoDppRamImage[2];
	// Location where DxO Dpp Ram Image is available
	uint16_t	uwDxoDppStartAddr;
	// IP First address of Micro code
	uint16_t	uwDxoDppBootAddr;
	// IP Boot address, where first add of Microcode has to be written
	uint16_t 	uwDxoDppRamImageSize[2];
	// Size of the Dpp Ram Image 
	uint16_t	uwBaseAddrDppMicroCode[2];
	// Starting Address of Micro code [ Offset withing Dpp Ram]
	uint8_t 	*pDxoDopRamImage[2];
	// Location where DxO Dop Ram Image is available
	uint16_t	uwDxoDopStartAddr;
	// IP First address of Micro code
	uint16_t	uwDxoDopBootAddr;
	// IP Boot address, where first add of Microcode has to be written
	uint16_t 	uwDxoDopRamImageSize[2];
	// Size of the Dop Ram Image 
	uint16_t	uwBaseAddrDopMicroCode[2];
	// Starting Address of Micro code [ Offset withing Dop Ram]
} Yushan_Init_Dxo_Struct_t;



// Report Errors in Yushan programming.
typedef struct
{
	// Report the Dxo constraint error.
	uint8_t		bDxoConstraints;
	// Report the minimu value required against bDxoConstraints.
	uint32_t	udwDxoConstraintsMinValue;
	// The variable will carry the minimum blank required, if TxLineBlank<MinLIneBlanking
	// else contains 0.
	//uint16_t	uwTxLineBlankingRequired;

}Yushan_SystemStatus_t;




// The following information are available DxO Integration doc
// Image Char information
typedef	struct
{	
	uint8_t 	bImageOrientation;
	uint16_t 	uwXAddrStart;
	uint16_t 	uwYAddrStart;
	uint16_t 	uwXAddrEnd;
	uint16_t 	uwYAddrEnd;
	uint16_t 	uwXEvenInc;  
	uint16_t 	uwXOddInc;
	uint16_t 	uwYEvenInc;  
	uint16_t 	uwYOddInc;
	uint8_t 	bBinning;    
}Yushan_ImageChar_t;


// Gains
typedef	struct
{
	uint16_t 	uwAnalogGainCodeGR;//Coding to be specified
	uint16_t 	uwAnalogGainCodeR;
	uint16_t 	uwAnalogGainCodeB;
	uint16_t 	uwPreDigGainGR;
	uint16_t 	uwPreDigGainR;
	uint16_t 	uwPreDigGainB;
	uint16_t 	uwExposureTime;    
	uint8_t	bRedGreenRatio;// coded on 2.6 (from 0x10 from 0xff). 
	uint8_t	bBlueGreenRatio; // coded on 2.6 (from 0x10 from 0xff).
}Yushan_GainsExpTime_t;



// DXO_DPP Tuning
typedef	struct
{
	//uint16_t 	uwSaturationValue;				// Removed as per Dxo recommendations
	uint8_t 	bTemporalSmoothing;   
	uint16_t 	uwFlashPreflashRating;
	uint8_t 	bFocalInfo;
}Yushan_DXO_DPP_Tuning_t;



// DXO_DOP Tuning
typedef	struct
{
	//uint16_t 	uwForceClosestDistance;			// Removed as per Dxo recommendations
	//uint16_t 	uwForceFarthestDistance;
	uint8_t 	bEstimationMode;
	uint8_t 	bSharpness;
	uint8_t 	bDenoisingLowGain;
	uint8_t 	bDenoisingMedGain;
	uint8_t 	bDenoisingHiGain;
	uint8_t 	bNoiseVsDetailsLowGain;
	uint8_t 	bNoiseVsDetailsMedGain;
	uint8_t 	bNoiseVsDetailsHiGain;
	uint8_t 	bTemporalSmoothing;    
}Yushan_DXO_DOP_Tuning_t;



/* DXO_PDP Tuning */
typedef	struct
{
	uint8_t 	bDeadPixelCorrectionLowGain;
	uint8_t 	bDeadPixelCorrectionMedGain;
	uint8_t 	bDeadPixelCorrectionHiGain;
}Yushan_DXO_PDP_Tuning_t;




/* DxODOP_ROI_active_number: */
/* Specified how many ROIs are valid and needs programming */
typedef struct
{
	uint8_t		bDxoDopRoiActiveNumber;
}Yushan_DXO_ROI_Active_Number_t;



/* AF ROI */
typedef	struct
{
	uint8_t 	bXStart;
	uint8_t 	bYStart;
	uint8_t 	bXEnd;
	uint8_t 	bYEnd;
}Yushan_AF_ROI_t;

/* AF stats */
typedef struct
{
	uint32_t 	udwAfStatsGreen;
	uint32_t 	udwAfStatsRed;
	uint32_t 	udwAfStatsBlue;
	uint32_t 	udwAfStatsConfidence;
}Yushan_AF_Stats_t;

/* Version Information */
typedef struct
{
	uint32_t 	udwDopVersion;
	uint32_t 	udwDppVersion;
	uint32_t 	udwPdpVersion;
	uint32_t 	udwDopCalibrationVersion;
	uint32_t 	udwDppCalibrationVersion;
	uint32_t 	udwPdpCalibrationVersion;
	uint8_t 	bApiMajorVersion;
	uint8_t 	bApiMinorVersion;
}Yushan_Version_Info_t;


#define RAWCHIP_INT_TYPE_ERROR (0x01<<0)
#define RAWCHIP_INT_TYPE_NEW_FRAME (0x01<<1)
#define RAWCHIP_INT_TYPE_PDP_EOF_EXECCMD (0x01<<2)
#define RAWCHIP_INT_TYPE_DPP_EOF_EXECCMD (0x01<<3)
#define RAWCHIP_INT_TYPE_DOP_EOF_EXECCMD (0x01<<4)
#define RAWCHIP_INT_TYPE_ERROR_FATAL (0x01<<5)

struct rawchip_sensor_init_data {
	uint8_t spi_clk;
	uint8_t ext_clk;
	uint8_t lane_cnt;
	uint16_t bitrate;
	uint16_t width;
	uint16_t height;
	uint16_t blk_pixels;
	uint16_t blk_lines;
};

typedef enum {
  RAWCHIP_NEWFRAME_ACK_NOCHANGE,
  RAWCHIP_NEWFRAME_ACK_ENABLE,
  RAWCHIP_NEWFRAME_ACK_DISABLE,
} rawchip_newframe_ack_enable_t;

typedef struct {
  uint16_t gain;
  uint16_t line;
} rawchip_aec_params_t;

typedef struct {
  uint8_t rg_ratio; /* Q6 format */
  uint8_t bg_ratio; /* Q6 format */
} rawchip_awb_params_t;

typedef struct {
  int update;
  rawchip_aec_params_t aec_params;
  rawchip_awb_params_t awb_params;
} rawchip_update_aec_awb_params_t;

typedef struct {
  uint8_t active_number;
  Yushan_AF_ROI_t sYushanAfRoi[5];
} rawchip_af_params_t;

typedef struct {
  int update;
  rawchip_af_params_t af_params;
} rawchip_update_af_params_t;


/****************************************************************************************
							EXTERNAL FUNCTIONS
****************************************************************************************/
#define Yushan_DXO_Sync_Reset_Dereset(bFlagResetOrDereset) 	SPI_Write(YUSHAN_RESET_CTRL+3, 1, &bFlagResetOrDereset)

/* Initialization API function */
bool_t	Yushan_Init_LDO(bool_t	bUseExternalLDO);
bool_t	Yushan_Init_Clocks(Yushan_Init_Struct_t *sInitStruct, Yushan_SystemStatus_t *sSystemStatus, uint32_t *udwIntrMask);
bool_t	Yushan_Init(Yushan_Init_Struct_t * sInitStruct);
bool_t	Yushan_Init_Dxo(Yushan_Init_Dxo_Struct_t * sDxoStruct, bool_t fBypassDxoUpload);

/* Update API Function */
bool_t	Yushan_Update_ImageChar(Yushan_ImageChar_t * sImageChar);
bool_t	Yushan_Update_SensorParameters(Yushan_GainsExpTime_t * sGainsExpInfo);
bool_t Yushan_Update_DxoPdp_TuningParameters(Yushan_DXO_PDP_Tuning_t * sDxoPdpTuning);
bool_t	Yushan_Update_DxoDpp_TuningParameters(Yushan_DXO_DPP_Tuning_t * sDxoDppTuning);
bool_t	Yushan_Update_DxoDop_TuningParameters(Yushan_DXO_DOP_Tuning_t * sDxoDopTuning);
bool_t	Yushan_Update_Commit(uint8_t  bPdpMode, uint8_t  bDppMode, uint8_t  bDopMode);

/* Interrupt functions */
bool_t	Yushan_Intr_Enable(uint8_t *pIntrMask);
bool_t Yushan_WaitForInterruptEvent(uint8_t,uint32_t);
bool_t Yushan_WaitForInterruptEvent2 (uint8_t bInterruptId ,uint32_t udwTimeOut);
void	Yushan_ISR(void);
void	Yushan_ISR2(void);
void	Yushan_Intr_Status_Read (uint8_t *bListOfInterrupts);
void	Yushan_Read_IntrEvent(uint8_t bIntrSetID, uint32_t *udwListOfInterrupts);
void	Yushan_AddnRemoveIDInList(uint8_t bInterruptID, uint32_t *udwListOfInterrupts, bool_t fAddORClear);
bool_t	Yushan_CheckForInterruptIDInList(uint8_t bInterruptID);//, uint32_t *udwListOfInterrupts);
bool_t	Yushan_Intr_Status_Clear(uint8_t *bListOfInterrupts);

uint8_t Yushan_parse_interrupt(void);



/* DXO Version Information */
bool_t Yushan_Get_Version_Information(Yushan_Version_Info_t * sYushanVersionInfo );



/* Optional Functions */
bool_t Yushan_Context_Config_Update(Yushan_New_Context_Config_t	*sYushanNewContextConfig);

bool_t Yushan_Swap_Rx_Pins (bool_t fClkLane, bool_t fDataLane1, bool_t fDataLane2, bool_t fDataLane3, bool_t fDataLane4);
bool_t Yushan_Invert_Rx_Pins (bool_t fClkLane, bool_t fDataLane1, bool_t fDataLane2, bool_t fDataLane3, bool_t fDataLane4);
bool_t Yushan_Enter_Standby_Mode (void);
bool_t Yushan_Exit_Standby_Mode(Yushan_Init_Struct_t * sInitStruct);
bool_t Yushan_Assert_Reset(uint32_t bModuleMask, uint8_t bResetORDeReset);
bool_t Yushan_AF_ROI_Update(Yushan_AF_ROI_t  *sYushanAfRoi, uint8_t bNumOfActiveRoi);
bool_t	Yushan_PatternGenerator(Yushan_Init_Struct_t * sInitStruct, uint8_t	bPatternReq, bool_t	bDxoBypassForTestPattern);
void	Yushan_DCPX_CPX_Enable(void);


/* Internal functions */
bool_t		Yushan_CheckDxoConstraints(uint32_t udwParameters, uint32_t udwMinLimit, uint32_t fpDxo_Clk, uint32_t fpPixel_Clk, uint16_t uwFullLine, uint32_t * pMinValue);
uint32_t	Yushan_Compute_Pll_Divs(uint32_t fpExternal_Clk, uint32_t fpTarget_PllClk);
uint32_t	Yushan_ConvertTo16p16FP(uint16_t);


/* Status Functions */
bool_t	Yushan_ContextUpdate_Wrapper(Yushan_New_Context_Config_t	*sYushanNewContextConfig);
bool_t Yushan_Dxo_Dop_Af_Run(Yushan_AF_ROI_t	*sYushanAfRoi, uint32_t *pAfStatsGreen, uint8_t	bRoiActiveNumber);
int Yushan_get_AFSU(uint32_t *pAfStatsGreen);
#if 0
bool_t Yushan_Read_AF_Statistics(uint32_t  *sYushanAFStats);
#endif
void Reset_Yushan(void);

void select_mode(uint8_t mode);

int Yushan_sensor_open_init(void);
void Yushan_dump_register(void);
void Yushan_dump_all_register(void);

int Yushan_Update_AEC_AWB_Params(rawchip_update_aec_awb_params_t *update_aec_awb_params);
int Yushan_Update_AF_Params(rawchip_update_af_params_t *update_af_params);
int Yushan_Update_3A_Params(rawchip_newframe_ack_enable_t enable_newframe_ack);
void	Yushan_Status_Snapshot(void);


#ifdef __cplusplus
}
#endif   /*__cplusplus*/


#endif /* _YUSHAN_API_H_ */
