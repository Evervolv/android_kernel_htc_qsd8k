/*******************************************************************************
################################################################################
#                             C STMicroelectronics
#    Reproduction and Communication of this document is strictly prohibited
#      unless specifically authorized in writing by STMicroelectronics.
#------------------------------------------------------------------------------
#                             Imaging Division
################################################################################
File Name:	Yushan_API.c
Author:		Rajdeep Patel
Description: Contains API functions to be used by host to Initialize Yushan clocks,
			 regsiters and DXO parameters. Also BOOT the DXO IPs.
********************************************************************************/

#include "Yushan_API.h"
#include <mach/gpio.h>
#ifdef YUSHAN_API_DEBUG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define DxO

/* Used by Interrupts */
uint32_t	udwProtoInterruptList[3];


struct yushan_int_t {
	spinlock_t yushan_spin_lock;
	wait_queue_head_t yushan_wait;
};

static uint8_t interrupt_err_count = 0;

bool_t	gPllLocked;

extern int rawchip_intr0, rawchip_intr1;
extern atomic_t interrupt, interrupt2;
extern struct yushan_int_t yushan_int;

Yushan_New_Context_Config_t 	sYushanFullContextConfig;
Yushan_New_Context_Config_t 	sYushanVideoFastContextConfig;
Yushan_New_Context_Config_t 	sYushanQTRContextConfig;

/*******************************************************************
API Function:	Yushan_Init_LDO. Performs the LDO initialization
Input:			void
Return:			SUCCESS(1),FALIURE(0)
********************************************************************/
bool_t	Yushan_Init_LDO(bool_t	bUseExternalLDO)
{

	bool_t		fStatus = SUCCESS;
	uint8_t		bSpiReadData = 0, bLDOTrimValue = 0, bSpiData = 0;
	uint16_t	uwCount = 500;	/* Some high number */


	/* NVM Ready check */
	SPI_Read(YUSHAN_IOR_NVM_STATUS, 1, (uint8_t *)(&bSpiReadData));
	while (bSpiReadData != 1) {
		/* NVM is not ready to be read. Check again the READY_STATUS */
		SPI_Read(YUSHAN_IOR_NVM_STATUS, 1, (uint8_t *)(&bSpiReadData));
		/* Decrement the counter. */
		uwCount--;
		if (!uwCount)
			break;	/* Maximum wait ends here */
	}

	/* Enter, if NVM is ready for read */
	if (bSpiReadData == 1) {
		/* Now read the LDO trim IDEAL value */
		SPI_Read(YUSHAN_IOR_NVM_DATA_WORD_3, 1, (uint8_t *)(&bLDOTrimValue));
		if (bLDOTrimValue>>3 == 1)				/* Check for Validity of IDEAL Trim value */
			bLDOTrimValue &= 0xF7;
		else {
			/* No Valid Ideal trim value, read the Default value */
			bLDOTrimValue = 0;
			SPI_Read(YUSHAN_IOR_NVM_DATA_WORD_2 + 3, 1, (uint8_t *)(&bLDOTrimValue));
			if (bLDOTrimValue>>3 == 1)			/* Check for Validity of Default Trim value */
				bLDOTrimValue &= 0xF7;
			else							/* No Ideal, nor valid default */
				bLDOTrimValue = 0 ;			/* DEFAULT_LDO_TRIM_VALUE;*/
		}

		/* Write the LDO_Trim value to the LDO */
		SPI_Write(YUSHAN_PRIVATE_TEST_LDO_NVM_CTRL, 1, (uint8_t *)(&bLDOTrimValue));

		/* Mode transition of LDO: Sequence has to be followed as it is done below: */
		bSpiReadData = 0;
		SPI_Read(YUSHAN_PRIVATE_TEST_LDO_CTRL,  1, (uint8_t *)(&bSpiReadData));
		bSpiData = bSpiReadData;

		CDBG("[CAM] Yushan bUseExternalLDO = %d\n", bUseExternalLDO);
		if(!bUseExternalLDO) {
			/* Commented for runing LDO in Test Mode */
		bSpiData = (bSpiData & 0xF7);			/* tst_BGPDLV to 0 */
		SPI_Write(YUSHAN_PRIVATE_TEST_LDO_CTRL,  1, (uint8_t *)(&bSpiData));
		bSpiData = (bSpiData & 0xFE);			/* tst_DLVDPDLV to 0 */
		SPI_Write(YUSHAN_PRIVATE_TEST_LDO_CTRL,  1, (uint8_t *)(&bSpiData));
			bSpiData = (bSpiData&0xFD);			/* tst_REGPDLV to 0 */
			SPI_Write(YUSHAN_PRIVATE_TEST_LDO_CTRL,  1, (uint8_t *)(&bSpiData));
		} else {
			/* For running LDO in external/test mode */
			bSpiData |= 0x04;
			SPI_Write(YUSHAN_PRIVATE_TEST_LDO_CTRL,  1, (uint8_t *)(&bSpiData));
		}

		/* Wait for LDO_STABLE interrupt */
		/* pr_err("Waiting for <B>EVENT_LDO_STABLE </B>Interrupt"); */

		fStatus &= Yushan_WaitForInterruptEvent(EVENT_LDO_STABLE, TIME_20MS);

		if (!fStatus) {
			pr_err("[CAM] LDO Interrupt Failed\n");
			return FAILURE;
		} else {
			/* pr_err("<B>LDO Interrupt </B> received");*/
		}

	} else {
		pr_err("[CAM] NVM is not Ready for Read operation\n");
		return FAILURE;
	}

	return SUCCESS;

}

/*******************************************************************
API Function:	Init_Clk_Yushan, perform the clock initialization of
				Yushan and starts the Pll.
	Input:			InitStruct
Return:			SUCCESS(1),FALIURE(0)
********************************************************************/
bool_t Yushan_Init_Clocks(Yushan_Init_Struct_t *sInitStruct, Yushan_SystemStatus_t *sSystemStatus, uint32_t	*udwIntrMask)
{

	uint32_t		fpTargetPll_Clk, fpSys_Clk, fpPixel_Clk, fpDxo_Clk, fpIdp_Clk, fpDxoClockLow;		// 16.16 Fixed Point notation
	uint32_t		fpSysClk_Div, fpIdpClk_div, fpDxoClk_Div, fpTempVar;				// 16.16 Fixed Point notation

	uint32_t		udwSpiData = 0;
	uint8_t			bRx_PixelWidth, bTx_PixelWidth;
	int8_t			bCount;
	uint8_t			fStatus = SUCCESS, fDXOStatus = DXO_NO_ERR;
	uint8_t			bSpiData;
	uint32_t 		udwFullLine, udwFullFrame, udwFrameBlank;

	/* IntrBitMask: Total 12 Bytes (83 Intr). Currently DXO intrs and Pll Lock intr has been enabled */
	//uint32_t		udwIntrMask[] = {0x00110410, 0x00000000, 0x00018000};
	/* uint32_t		udwIntrMask[] = {0x7DE1861B, 0xA600007C, 0x000DBFFD}; */
	uint16_t		udwIntrSpiData=0;
	/* Set of Dividers */
	/* float_t		fpDividers[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 8, 10};*/
	uint32_t		fpDividers[] = {0x10000, 0x18000, 0x20000, 0x28000, 0x30000, 0x38000, 0x40000, 0x48000, 0x50000, 0x58000, 0x60000,	\
										0x68000, 0x70000, 0x78000, 0x80000, 0xA0000};

	/* Switch to Host Clock */
	CDBG("[CAM] Switching to Host clock while evaluating different clock dividers\n");
	bSpiData = 0x00;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));/* Tx, Rx, Dxo, Idp clks enabled. */
	bSpiData = 0x02;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));/* Tx, Rx, Dxo, Idp clks enabled. */
	bSpiData = 0x19;
	fStatus &= SPI_Write(YUSHAN_PLL_CTRL_MAIN, 1, (uint8_t *)(&bSpiData));	/* Pll in power down state keeping earlier config intact */
	/* Rx and Tx Pixel width */
	bRx_PixelWidth = bTx_PixelWidth = (sInitStruct->uwPixelFormat) & (0x000F);

	/* fpTargetPll_Clk is same as BitRate per lane. */
	fpTargetPll_Clk = Yushan_ConvertTo16p16FP((uint16_t)sInitStruct->uwBitRate);
	/* Compute Pll components */
	fpTargetPll_Clk = Yushan_Compute_Pll_Divs(sInitStruct->fpExternalClock, fpTargetPll_Clk);
	if (!fpTargetPll_Clk) {
		pr_err("[CAM] Wrong PLL Clk calculated. Returning");
		return FAILURE;
	}

	/* IDP_CLK: */
	/* fpidp_clk = (sInitStruct->uwBitRate * bNumberOfLanes)/bTx_PixelWidth;*/
	/* or fpidp_clk = fpTargetPll_Clk/fpIdpClk_div;*/

	/* fpIdpClk_div = (bTx_PixelWidth/sInitStruct->bNumberOfLanes);*/
	/* fpIdpClk_div = Yushan_ConvertTo16p16FP((uint16_t)(bTx_PixelWidth/sInitStruct->bNumberOfLanes));*/
	/* Above the IdpClk_div will be (2*0x10000) in case PixWidth is 10 and NoLanes is 4, since (PixWidth/NoLanes) = 2, */
	/* not 2.5. Multiplying PixWidth by 0x10000 makes the result in fixed point without loosing on precesion post */
	/* decimal point. */
	fpIdpClk_div = (Yushan_ConvertTo16p16FP(bTx_PixelWidth)/sInitStruct->bNumberOfLanes);

	/******** DXO_Clk: Compute the lowest DxO clock matching constraints *********/

	/* Below may results in loss of precesion as division takes place before changing to fixed point. */
	/* fpPixel_Clk = Yushan_ConvertTo16p16FP((uint16_t)((sInitStruct->uwBitRate * sInitStruct->bNumberOfLanes) / bRx_PixelWidth));*/
	/* Result in fixed point notation and not loosen on precesion post decimal point. // Chk multplication not exceeding 32 bit max value. */
	fpPixel_Clk = ((Yushan_ConvertTo16p16FP(sInitStruct->uwBitRate) * sInitStruct->bNumberOfLanes) / bRx_PixelWidth);
	fpIdp_Clk = Yushan_ConvertTo16p16FP((uint16_t)(fpTargetPll_Clk/fpIdpClk_div));

	/* FullLine and FullFrame */
	udwFullLine  = sInitStruct->uwActivePixels + sInitStruct->uwLineBlankStill;
	udwFullFrame = sInitStruct->uwLines + sInitStruct->uwFrameBlank;

	/* Compute the lower limit of DxO clock:
	*  (SpiClock/8)* 30: There can be times, when SPIClk< 8 (division may yield 0. So instead of
	*  using float value) first multiply it by 30 and then divide the result by 8. Fine as here
	*  SpiClk is in fixed point notation. */
	fpTempVar = Yushan_ConvertTo16p16FP((uint16_t)((sInitStruct->fpSpiClock * 30) / 0x80000));
	fpDxoClockLow = (fpTempVar >= fpIdp_Clk) ? fpTempVar : fpIdp_Clk;

	if (fpDxoClockLow > DXO_CLK_LIMIT) {
		pr_err("[CAM] Lower limit of DxO clock is greater than higher limit:%d\n", fpDxoClockLow);
		sSystemStatus->bDxoConstraints = DXO_LOLIMIT_EXCEED_HILIMIT;
		return FAILURE;
	}

	/* Max index must be the index corresponding to PixelWidth. */
	bCount = ((bTx_PixelWidth == 8) ? 14 : 15);

	sSystemStatus->udwDxoConstraintsMinValue = 0;
	/* pick the highest divider for which all the Dxo constraints satisfies. */
	while (bCount >= 0) {
		fpDxoClk_Div = fpDividers[bCount];
		fpDxo_Clk = Yushan_ConvertTo16p16FP((uint16_t)(fpTargetPll_Clk / fpDxoClk_Div));

		if ((fpDxo_Clk >= fpDxoClockLow) && (fpDxo_Clk <= DXO_CLK_LIMIT)) {
			/* Checking DXO constraints with LineBlanking, FullLine, ActiveFrameLength, FullFrame and FrameBalnking */
			fDXOStatus = ((fStatus = Yushan_CheckDxoConstraints(sInitStruct->uwLineBlankStill, DXO_MIN_LINEBLANKING, fpDxo_Clk, fpPixel_Clk, 1, &sSystemStatus->udwDxoConstraintsMinValue)) == 0)? DXO_LINEBLANK_ERR : DXO_NO_ERR;
			if(fDXOStatus!=DXO_NO_ERR) { bCount--; continue;}
			fDXOStatus = ((fStatus = Yushan_CheckDxoConstraints(udwFullLine, DXO_MIN_LINELENGTH, fpDxo_Clk, fpPixel_Clk, 1, &sSystemStatus->udwDxoConstraintsMinValue)) == 0)? DXO_FULLLINE_ERR: DXO_NO_ERR ;
			if(fDXOStatus!=DXO_NO_ERR) { bCount--; continue;}
			/* From next three checks, udwFullLine avoided to make computation easier. */
			/* Checking constraints in terms of LineCount not in terms of Clks. */
			fDXOStatus = ((fStatus = Yushan_CheckDxoConstraints(sInitStruct->uwLines, ((DXO_ACTIVE_FRAMELENGTH/udwFullLine) + 1), fpDxo_Clk, fpPixel_Clk, 1, &sSystemStatus->udwDxoConstraintsMinValue)) == 0)? DXO_ACTIVE_FRAMELENGTH_ERR: DXO_NO_ERR;
			if(fDXOStatus!=DXO_NO_ERR) { bCount--; continue;}
			fDXOStatus = ((fStatus = Yushan_CheckDxoConstraints(udwFullFrame, ((DXO_FULL_FRAMELENGTH/udwFullLine) + 1), fpDxo_Clk, fpPixel_Clk, 1, &sSystemStatus->udwDxoConstraintsMinValue)) == 0)? DXO_FULLFRAMELENGTH_ERR: DXO_NO_ERR;
			if(fDXOStatus!=DXO_NO_ERR) { bCount--; continue;}

			/* udwFrameBlank = ((8 * udwFullLine) * (fpDxo_Clk/fpPixel_Clk));
			*  The below sequence is imp, as in absence of float, DxoClk/PixClk may yield 0, instead of some meaningful values.
			*  so multiply DxoClk by 8 first and then divide it by PixClk. Avoiding FullLine to make checks easier. */
			udwFrameBlank = ((8*fpDxo_Clk)/fpPixel_Clk);						
/* Yushan API 10.0 Start */
			if (sInitStruct->bDxoSettingCmdPerFrame == 1)
				udwFrameBlank += (135000/udwFullLine)+1;   /*(35000 + 100000);		For 1 SettingsPerFrame. */
			if (sInitStruct->bDxoSettingCmdPerFrame > 1)
				udwFrameBlank += (220000/udwFullLine)+1;   /*(35000 + 185000);		For more than 1 SettingsPerFrame.*/
/* Yushan API 10.0 End */

			fDXOStatus = ((fStatus = Yushan_CheckDxoConstraints(sInitStruct->uwFrameBlank, udwFrameBlank, fpDxo_Clk, fpPixel_Clk, 1, &sSystemStatus->udwDxoConstraintsMinValue)) == 0)? DXO_FRAMEBLANKING_ERR: DXO_NO_ERR;
			if(fDXOStatus!=DXO_NO_ERR) { bCount--; continue;}

			/* For the current divider, no error occured, so loop can be break here. */
			break;

		}

		/* This is required when Dxo_Clk does not lie between low and high limit.
		*  and also Dxo constraints have not yet checked. */
		bCount--;
	}

	/* Report DXO Status. */
	sSystemStatus->bDxoConstraints = fDXOStatus;

	/* If any of the DXO_CLK_DIV does not satisfy constraints: */
	if (bCount < 0) {
		pr_err("[CAM] DXO Dividers exhausted and returning FAILURE\n");
		return FAILURE;
	}

	/* Compute the highest system clock. Finding the lowest divider for which
	*  system clock is just below the system clk limit. */
	for (bCount = 0; bCount < 10; bCount++) {
		fpSysClk_Div = fpDividers[bCount];
		fpSys_Clk = Yushan_ConvertTo16p16FP((uint16_t)(fpTargetPll_Clk / fpSysClk_Div));
		if (fpSys_Clk <= SYS_CLK_LIMIT)
			break;/* Got the desired SysClk_Div. */
	}


	fStatus = SUCCESS;

	/* Enabling Clks */
	bSpiData = 0x7F;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 1, (uint8_t *)(&bSpiData));/* Tx, Rx, Dxo, Idp clks enabled. */

	/* DXO out of Reset */
	/* Togling the resets */
	bSpiData = 0x00;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL, 1, (uint8_t *)(&bSpiData));
	bSpiData = 0x04;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL + 1, 1, (uint8_t *)(&bSpiData));
	bSpiData = 0x00;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL + 2, 1, (uint8_t *)(&bSpiData));
	bSpiData = 0x37;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL, 1, (uint8_t *)(&bSpiData));
	bSpiData = 0x0f;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL + 1, 1, (uint8_t *)(&bSpiData));
	bSpiData = 0x07;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL + 2, 1, (uint8_t *)(&bSpiData));
	bSpiData = 1;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL + 3, 1, (uint8_t *)(&bSpiData));
	bSpiData = 0;
	fStatus &= SPI_Write(YUSHAN_RESET_CTRL + 3, 1, (uint8_t *)(&bSpiData));

	/* Enable/Disable Interrupts after resetting the IPs */
	Yushan_Intr_Enable((uint8_t *)udwIntrMask);
	/* Route DOP7 INT to INTR1 */
	  udwIntrSpiData = 0x0008;
	  SPI_Write(YUSHAN_IOR_NVM_SEND_ITR_PAD1 ,2,(uint8_t *)&udwIntrSpiData);	

	/* LDO Initialization */
#if 1 /* ndef PROTO_SPECIFIC */
	fStatus &= Yushan_Init_LDO(sInitStruct->bUseExternalLDO);
	if (!fStatus) {
		pr_err("[CAM] LDO setup FAILED\n");
		return FAILURE;
	} else {
		/* pr_err("<B>LDO setup done </B>");*/
	}
#endif
	/* Writing DXO, SYS and IDP Dividers to their respective regsiters */
	/* Converting fixed point to non fixed point */
	bSpiData = (uint8_t)(fpDxoClk_Div>>16);
	bSpiData = bSpiData << 1;
/* Yushan API 10.0 Start */
	if ((fpDxoClk_Div&0x8000)==0x8000)				/* Decimal point (0x8000fp = 0.5floating) */
		bSpiData |= 0x01;
/* Yushan API 10.0 End */
	fStatus &= SPI_Write(YUSHAN_CLK_DIV_FACTOR,   1, (uint8_t *)(&bSpiData));
	bSpiData = (uint8_t)(fpIdpClk_div>>16);
	bSpiData = bSpiData << 1;
/* Yushan API 10.0 Start */
	if ((fpIdpClk_div&0x8000)==0x8000)				/* Decimal point (0x8000fp = 0.5floating) */
		bSpiData |= 0x01;
/* Yushan API 10.0 End */
	fStatus &= SPI_Write(YUSHAN_CLK_DIV_FACTOR+1, 1, (uint8_t *) (&bSpiData));
	bSpiData = (uint8_t)(fpSysClk_Div>>16);
	bSpiData = bSpiData << 1;
/* Yushan API 10.0 Start */
	if ((fpSysClk_Div&0x8000)==0x8000)				/* Decimal point (0x8000fp = 0.5floating) */
		bSpiData |= 0x01;
/* Yushan API 10.0 End */
	fStatus &= SPI_Write(YUSHAN_CLK_DIV_FACTOR + 2, 1, (uint8_t *) (&bSpiData));

	#if 0 /*def PROTO_SPECIFIC*/
      bSpiData = 1;
	    fStatus &= SPI_Write(YUSHAN_PLL_CTRL_MAIN + 1, 1, (uint8_t *) (&bSpiData));
      bSpiData = 25;
	    fStatus &= SPI_Write(YUSHAN_PLL_LOOP_OUT_DF + 1, 1, (uint8_t *) (&bSpiData));
      bSpiData = 3;
	    fStatus &= SPI_Write(YUSHAN_PLL_LOOP_OUT_DF, 1, (uint8_t *) (&bSpiData));
		  bSpiData = (sInitStruct->uwPixelFormat & 0x0f);
	    fStatus &= SPI_Write(0, 1, (uint8_t *) (&bSpiData));
      bSpiData = 10;
	    fStatus &= SPI_Write(2, 1, (uint8_t *) (&bSpiData));
	#endif

	/* Enable Pll */
	bSpiData = 0x18;
	fStatus &= SPI_Write(YUSHAN_PLL_CTRL_MAIN, 1, (uint8_t *)(&bSpiData));	/* Pll in functional state keeping earlier config intact */

	if(!fStatus) return FAILURE;

	/* After programming all the clocks and enabling the PLL and its parameters,
	*  wait till Pll gets lock/stable. */
	fStatus &= Yushan_WaitForInterruptEvent(EVENT_PLL_STABLE, TIME_20MS);

	if (!fStatus) {
		pr_err("EVENT_PLL_STABLE fail");
		return FAILURE;
	}

#if 1
	/* For glitch free switching, first disable the clocks */
	/* Saving the context/data of the Clk_Ctrl */
	udwSpiData = 0;
	SPI_Read(YUSHAN_CLK_CTRL,   4, (uint8_t *)(&udwSpiData));

	/* Disable the clocks */
	bSpiData = 0x00;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 2, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 3, 1, (uint8_t *)(&bSpiData));

	/* No_bypass Pll already done above. */
#if 0
	bSpiData = 0;
	SPI_Read(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));
	bSpiData &= 0xFD;/* bypass Pll set to 0 */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));
#endif

	/* Switch Sys clk to Pll clk from Host clk */
	bSpiData = 0x01;/* Clk_Sys_Sel set to 1 */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));

	/* Enable the clocks */
	udwSpiData &= 0xFDFF;/* Keeping Bypass Pll Off (0) */
	udwSpiData |= 0x0100;/* Keeping Sys_Sel On (1) */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 4, (uint8_t *)(&udwSpiData));
#else
	/* Switch System clk to Pll clk */
	bSpiData = 0x01;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));
#endif

	return fStatus;
}

/*****************************************************************
API Internal Function:	Convert to 16 point 16 fixed point notation
Input:					Unsigned 16 bit integer.
******************************************************************/
uint32_t Yushan_ConvertTo16p16FP(uint16_t uwNonFPValue)
{
	return (((uint32_t)(uwNonFPValue)) << 16);
}

/************************************************************
API Function: Upload DXO micro code and perform the boot operation.
Input:		  DXO_InitStruct
*************************************************************/
bool_t	Yushan_Init_Dxo(Yushan_Init_Dxo_Struct_t *sDxoStruct, bool_t fBypassDxoUpload)
{

	uint8_t bSpiData;
	uint32_t	udwHWnMicroCodeID = 0; /*, udwCalibrationDataID = 0, udwErrorCode = 0;*/
	uint32_t udwDxoBaseAddress = 0, udwSpiBaseIndex = 0;
	bool_t	fStatus = 1;
	/* uint8_t readback= 0;*/

	/******************************* DXO DOP RAM ***************************************************/
	CDBG("[CAM] Yushan_Init_Dxo E");

	udwSpiBaseIndex = 0x08000;
	fStatus = SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	udwDxoBaseAddress = DXO_DOP_BASE_ADDR;

	if (!fBypassDxoUpload) {
		CDBG("[CAM] load DxO DOP firmware\n");
		fStatus &= SPI_Write_4thByte((uint16_t)(sDxoStruct->uwBaseAddrDopMicroCode[0]+udwDxoBaseAddress), sDxoStruct->uwDxoDopRamImageSize[0], sDxoStruct->pDxoDopRamImage[0]);
		fStatus &= SPI_Write_4thByte((uint16_t)(sDxoStruct->uwBaseAddrDopMicroCode[1]+udwDxoBaseAddress), sDxoStruct->uwDxoDopRamImageSize[1], sDxoStruct->pDxoDopRamImage[1]);
	}
	/* Boot Sequence */
	/* As per the scenerio files, first program the start address of the code */
#if 1
	fStatus &= SPI_Write((uint16_t)(DxODOP_bootAddress + udwDxoBaseAddress), 2,  (uint8_t*)(&sDxoStruct->uwBaseAddrDopMicroCode[0]));
#else
	bSpiData = 0x00;
	fStatus &=  SPI_Write(DxODOP_bootAddress + udwDxoBaseAddress, 1,  (uint8_t *)(&bSpiData));
	bSpiData = 0x03;
	fStatus &=  SPI_Write(DxODOP_bootAddress + udwDxoBaseAddress + 1, 1,  (uint8_t *)(&bSpiData));
#endif

#if 0
	pr_info("/*---------------------------------------*\\");
	SPI_Read(DxODOP_bootAddress + udwDxoBaseAddress    , 1, &readback);
	pr_info("DXO DOP 0x%x = %x", DxODOP_bootAddress    , readback);
	SPI_Read(DxODOP_bootAddress + udwDxoBaseAddress + 1, 1, &readback);
	pr_info("DXO DOP 0x%x = %x", DxODOP_bootAddress + 1, readback);
	pr_info("\\*---------------------------------------*/");
#endif

	/* Give Boot Command and Wait for the EOB interrupt. */
	bSpiData = DXO_BOOT_CMD;
	fStatus &= SPI_Write((uint16_t)(DxODOP_boot + udwDxoBaseAddress), 1,  (uint8_t*)(&bSpiData));

	/******************************* DXO DPP RAM *****************************************************/
/* Yushan API 10.0 Start */
	udwSpiBaseIndex = 0x10000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	/* NewAddr = (ActualAddress - udwSpiBaseIndex) + 0x8000:  0x8000 is the SPIWindow_1 base address.
	*  ActualAddress = DXO_XXX_BASE_ADDR + Offset;
	*  when NewAddr, is in window (0x8000-> 0xFFFF), then SPI slave will check the SPI base address
	*  to know the actual physical address to access. */
	udwDxoBaseAddress = (0x8000 + DXO_DPP_BASE_ADDR) - udwSpiBaseIndex;/* OR ## DXO_DPP_BASE_ADDR-0x8000;*/
	/* Loading Microcode, setting address for boot and Boot command. Then Verifying the DPP IP */
	if (!fBypassDxoUpload) {
		CDBG("[CAM] load DxO DPP firmware\n");
		fStatus &= SPI_Write_4thByte((uint16_t)(sDxoStruct->uwBaseAddrDppMicroCode[0]+ udwDxoBaseAddress), sDxoStruct->uwDxoDppRamImageSize[0], sDxoStruct->pDxoDppRamImage[0]);
		/* Setting base address to the DPP calibration data base address. */
		udwSpiBaseIndex = DXO_DPP_BASE_ADDR + sDxoStruct->uwBaseAddrDppMicroCode[1]; /* 0x018000; */
		SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));
		udwDxoBaseAddress = ((DXO_DPP_BASE_ADDR + sDxoStruct->uwBaseAddrDppMicroCode[1]) - udwSpiBaseIndex) + 0x8000; /*(0x8000 + DXO_DPP_BASE_ADDR) - udwSpiBaseIndex;*/

		fStatus &= SPI_Write_4thByte((uint16_t)(udwDxoBaseAddress), sDxoStruct->uwDxoDppRamImageSize[1], sDxoStruct->pDxoDppRamImage[1]);
	}
	/* Since " SET_ADDR_DPP_BOOT + DXO_DPP_BASE_ADDR " exceed 0x018000, changing the access base address */
	udwSpiBaseIndex = 0x18000;
	udwDxoBaseAddress = 0x8000; /*(0x8000 + DXO_DPP_BASE_ADDR) - udwSpiBaseIndex;*/ /* OR DXO_DPP_BASE_ADDR-0x10000; */
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	/* Boot Sequence */
	/* As per the scenerio files, first program the start address of the code */
#if 1
	fStatus &= SPI_Write((uint16_t)((DXO_DPP_BASE_ADDR + DxODPP_bootAddress) - udwSpiBaseIndex + udwDxoBaseAddress), 2,  (uint8_t*)(&sDxoStruct->uwBaseAddrDppMicroCode[0]));
#else
	bSpiData=0x00;
	fStatus &=  SPI_Write(DxODPP_bootAddress+udwDxoBaseAddress, 1,  (uint8_t*)(&bSpiData));	
	bSpiData=0x03;
	fStatus &=  SPI_Write(DxODPP_bootAddress+udwDxoBaseAddress + 1, 1,  (uint8_t*)(&bSpiData));		
#endif

#if 0
	pr_info("/*---------------------------------------*\\");
	SPI_Read(DxODPP_bootAddress + udwDxoBaseAddress    , 1, &readback);
	pr_info("DXO DPP 0x%x = %x", DxODPP_bootAddress	  , readback);
	SPI_Read(DxODPP_bootAddress + udwDxoBaseAddress + 1, 1, &readback);
	pr_info("DXO DPP 0x%x = %x", DxODPP_bootAddress + 1, readback);
	pr_info("\\*---------------------------------------*/");
#endif
	/* Give Boot Command and Wait for the EOB interrupt. */
	bSpiData = DXO_BOOT_CMD;
	fStatus &= SPI_Write(((uint16_t)(DXO_DPP_BASE_ADDR + DxODPP_boot) - udwSpiBaseIndex + udwDxoBaseAddress), 1,  (uint8_t*)(&bSpiData));
/* Yushan API 10.0 End */

	/* Restoring the SpiBaseAddress to 0x08000. */
	udwSpiBaseIndex = 0x8000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	/*********************************************** DXO PDP RAM ************************************/
	/* Loading Microcode, setting address for boot and Boot command. Then Verifying the PDP IP */
	if (!fBypassDxoUpload) {
		CDBG("[CAM] load DxO PDP firmware\n");
		fStatus &= SPI_Write_4thByte(sDxoStruct->uwBaseAddrPdpMicroCode[0] + DXO_PDP_BASE_ADDR , sDxoStruct->uwDxoPdpRamImageSize[0], sDxoStruct->pDxoPdpRamImage[0]);
		fStatus &= SPI_Write_4thByte(sDxoStruct->uwBaseAddrPdpMicroCode[1] + DXO_PDP_BASE_ADDR, sDxoStruct->uwDxoPdpRamImageSize[1], sDxoStruct->pDxoPdpRamImage[1]);
	}
	/* Boot Sequence */
	/* As per the scenerio files, first program the start address of the code */
#if 1
	fStatus &= SPI_Write(DxOPDP_bootAddress + DXO_PDP_BASE_ADDR, 2,  (uint8_t *)(&sDxoStruct->uwBaseAddrPdpMicroCode[0]));
#else
	bSpiData = 0x34;
	fStatus &=  SPI_Write(DxOPDP_bootAddress + DXO_PDP_BASE_ADDR, 1,  (uint8_t *)(&bSpiData));
	bSpiData = 0x02;
	fStatus &=  SPI_Write(DxOPDP_bootAddress + DXO_PDP_BASE_ADDR + 1, 1,  (uint8_t *)(&bSpiData));
#endif

#if 0
	pr_info("/*---------------------------------------*\\");
	SPI_Read(DxOPDP_bootAddress + DXO_PDP_BASE_ADDR    , 1, &readback);
	pr_info("DXO PDP 0x%x = %x", DxOPDP_bootAddress	  , readback);
	SPI_Read(DxOPDP_bootAddress + DXO_PDP_BASE_ADDR + 1, 1, &readback);
	pr_info("DXO PDP 0x%x = %x", DxOPDP_bootAddress + 1, readback);
	pr_info("\\*---------------------------------------*/");
#endif

	/* Give Boot Command and Wait for the EOB interrupt. */
	bSpiData = DXO_BOOT_CMD;
	fStatus &= SPI_Write(DxOPDP_boot + DXO_PDP_BASE_ADDR, 1,  (uint8_t *)(&bSpiData));

	/* Wait for the End of Boot interrupts. This indicates the completion of Boot command */
	fStatus &= Yushan_WaitForInterruptEvent2 (EVENT_DOP7_BOOT, TIME_10MS);
	if (!fStatus) {
		pr_err("Wait for the End of Boot interrupts EVENT_DOP7_BOOT Failed\n");
		return fStatus;
	}

	/* Verifying DXO_XXX HW, Micro code and Calibration data IDs. */

	/* Verifying DOP Ids */
	SPI_Read((DxODOP_ucode_id_7_0 + DXO_DOP_BASE_ADDR) , 4, (uint8_t *)(&udwHWnMicroCodeID));
#if 0
	SPI_Read((DxODOP_calib_id_0_7_0 + DXO_DOP_BASE_ADDR), 4, (uint8_t *)(&udwCalibrationDataID));
	SPI_Read((DxODOP_error_code_7_0 + DXO_DOP_BASE_ADDR), 1, (uint8_t *)(&udwErrorCode));
	CDBG("DXO DOP udwHWnMicroCodeID:%x;udwCalibrationDataID:%x;udwErrorCode:%x\n",
		udwHWnMicroCodeID, udwCalibrationDataID, udwErrorCode);
#endif

	udwHWnMicroCodeID &= 0xFFFFFF00;	/* Not checking the ucode minor version */
	if (udwHWnMicroCodeID != DXO_DOP_HW_MICROCODE_ID) {
		pr_info("[CAM] DXO DOP udwHWnMicroCodeID:%x", udwHWnMicroCodeID);
		/* return FAILURE;*/
	}

	/* Wait for the End of Boot interrupts. This indicates the completion of Boot command */
	fStatus &= Yushan_WaitForInterruptEvent (EVENT_DPP_BOOT, TIME_10MS);
	if (!fStatus) {
		pr_err("Wait for the End of Boot interrupts EVENT_DPP_BOOT Failed\n");
		return fStatus;
	}

	/* Verifying DPP Ids */
	udwSpiBaseIndex = 0x010000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	SPI_Read(DxODPP_ucode_id_7_0 + DXO_DPP_BASE_ADDR-0x8000, 4, (uint8_t *)(&udwHWnMicroCodeID));
#if 0
	SPI_Read(DxODPP_calib_id_0_7_0+DXO_DPP_BASE_ADDR-0x8000, 4, (uint8_t *)(&udwCalibrationDataID));
	SPI_Read(DxOPDP_error_code_7_0+DXO_DPP_BASE_ADDR-0x8000, 1, (uint8_t *)(&udwErrorCode));
	CDBG("DXO DPP udwHWnMicroCodeID:%x;udwCalibrationDataID:%x;udwErrorCode:%x\n",
		udwHWnMicroCodeID, udwCalibrationDataID, udwErrorCode);
#endif

	udwHWnMicroCodeID &= 0xFFFFFF00;	/*  Not checking the ucode minor version */
	if (udwHWnMicroCodeID != DXO_DPP_HW_MICROCODE_ID) {
		pr_err("[CAM] DXO DPP udwHWnMicroCodeID:%x", udwHWnMicroCodeID);
		/*return FAILURE;*/
	}
	/* Restoring the SpiBaseAddress to 0x08000. */
	udwSpiBaseIndex = 0x08000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	/* Wait for the End of Boot interrupts. This indicates the completion of Boot command */
	fStatus &= Yushan_WaitForInterruptEvent (EVENT_PDP_BOOT, TIME_10MS);
	if (!fStatus) {
		pr_err("Wait for the End of Boot interrupts EVENT_PDP_BOOT Failed\n");
		return fStatus;
	}

	/* Verifying PDP Ids */
	SPI_Read((DxOPDP_ucode_id_7_0 + DXO_PDP_BASE_ADDR) , 4, (uint8_t *)(&udwHWnMicroCodeID));
#if 0
	SPI_Read((DxOPDP_calib_id_0_7_0 + DXO_PDP_BASE_ADDR), 4, (uint8_t *)(&udwCalibrationDataID));
	SPI_Read((DxOPDP_error_code_7_0 + DXO_PDP_BASE_ADDR), 1, (uint8_t *)(&udwErrorCode));
	CDBG("DXO PDP udwHWnMicroCodeID:%x;udwCalibrationDataID:%x;udwErrorCode:%x\n",
		udwHWnMicroCodeID, udwCalibrationDataID, udwErrorCode);
#endif

	udwHWnMicroCodeID &= 0xFFFFFF00;/* Not checking the ucode minor version */
	if (udwHWnMicroCodeID != DXO_PDP_HW_MICROCODE_ID) {
		pr_err("[CAM] DXO PDP udwHWnMicroCodeID:%x", udwHWnMicroCodeID);
		/* return FAILURE;*/
	}

	CDBG("[CAM] Yushan_Init_Dxo X");

	return SUCCESS;
}




/************************************************************************************
API Function:	Init Yushan, perform the basic initialization
				of Yushan.
Input:			InitStruct
Return:
*************************************************************************************/
bool_t Yushan_Init(Yushan_Init_Struct_t *sInitStruct)
{
	uint32_t spi_base_address, udwSpiData;
	uint16_t uwHsizeStill = 0;
	uint16_t uwHsizeVf = 0;
	uint16_t uwLecciStill;
	uint16_t uwLecciVf;
	uint8_t bCount, bStillIndex = 0, bVfIndex = 0;
#ifdef DxO
	uint8_t bBypassDxo = 0;
#else
	uint8_t bBypassDxo = 1;
#endif
	uint8_t bSofEofLength = 32;
	uint8_t bDxoClkDiv , bPixClkDiv;
	uint8_t		bDXODecimalFactor = 0, bIdpDecimalFactor = 0; /* Yushan API 10.0*/

	/* Writing the base address  in order to access full 64KB */
	spi_base_address = 0x8000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (unsigned char *) &spi_base_address);

	udwSpiData = 0x1;
	SPI_Write(YUSHAN_IOR_NVM_CTRL, 1, (unsigned char *) &udwSpiData);/* NVM Control */

	/* MIPI  Initialization */
	udwSpiData = 4000/sInitStruct->uwBitRate;
	SPI_Write(YUSHAN_MIPI_TX_UIX4, 1, (unsigned char *) &udwSpiData);
	SPI_Write(YUSHAN_MIPI_RX_UIX4, 1, (unsigned char *) &udwSpiData);

	if (sInitStruct->bNumberOfLanes == 1)
		udwSpiData = 0x11;/* Enabling Clock Lane and Data Lane1 */
	else if (sInitStruct->bNumberOfLanes == 2)
		udwSpiData = 0x31;/* Enabling Clock Lane and Data Lane1 */
	else if (sInitStruct->bNumberOfLanes == 4)
		udwSpiData = 0xf1;/* Enabling Clock Lane and Data Lane1 */

	/* Exempting tx_request_hs_cl in MIPI_TX_EN */
	SPI_Write(YUSHAN_MIPI_TX_ENABLE, 1, (unsigned char *) &udwSpiData);
	SPI_Write(YUSHAN_MIPI_RX_ENABLE, 1, (unsigned char *) &udwSpiData);
	/* Adding delay (ideal: 3Microsec) between TX_Enable and setting of tx_request_hs_cl. */
	/* Re writing MIPI UIX4 */
	udwSpiData = 4000/sInitStruct->uwBitRate;
	SPI_Write(YUSHAN_MIPI_TX_UIX4, 1, (unsigned char *) &udwSpiData);
	SPI_Write(YUSHAN_MIPI_RX_UIX4, 1, (unsigned char *) &udwSpiData);

	/* Reading from RX_ENABLE */
	SPI_Read(YUSHAN_MIPI_RX_ENABLE, 1, (unsigned char *) &udwSpiData);
	udwSpiData |= 0x02;/* Enable Clk HS */
	SPI_Write(YUSHAN_MIPI_TX_ENABLE, 1, (unsigned char *) &udwSpiData);

	/* CSI2Rx Initialization */

	udwSpiData = sInitStruct->bNumberOfLanes;
	SPI_Write(YUSHAN_CSI2_RX_NB_DATA_LANES, 1, (unsigned char *) &udwSpiData);

	udwSpiData = sInitStruct->uwPixelFormat & 0x0f;
	SPI_Write(YUSHAN_CSI2_RX_IMG_UNPACKING_FORMAT, 1, (unsigned char *) &udwSpiData);

	/* YUSHAN_CSI2_RX_BYTE2PIXEL_READ_TH */
	udwSpiData = 4; /* 32; - tmp workaround for DOP stop  */
	SPI_Write(YUSHAN_CSI2_RX_BYTE2PIXEL_READ_TH, 1, (unsigned char *) &udwSpiData);

	/* SMIA Formatter Initialization */

	udwSpiData = sInitStruct->uwPixelFormat & 0x0f;
	SPI_Write(YUSHAN_SMIA_FM_PIX_WIDTH, 1, (unsigned char *) &udwSpiData);

	udwSpiData = 0;
	SPI_Write(YUSHAN_SMIA_FM_GROUPED_PARAMETER_HOLD, 1, (unsigned char *) &udwSpiData);
	udwSpiData = 0x19;
	SPI_Write(YUSHAN_SMIA_FM_CTRL, 1, (unsigned char *) &udwSpiData);
	/* Disabling the SMIA_FM_EOF_INT_EN */
	udwSpiData = 0;
	SPI_Write(YUSHAN_SMIA_FM_EOF_INT_EN, 1, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x3;*/
	/* SPI_Write(YUSHAN_SMIA_FM_EOF_INT_CTRL, 1, (unsigned char *) &udwSpiData);*/

	/* CSITX Wrapper  */
	udwSpiData = ((((sInitStruct->uwPixelFormat & 0x0f) * bSofEofLength) / 32) << 8) | 0x01;
	SPI_Write(YUSHAN_CSI2_TX_WRAPPER_THRESH, 2, (unsigned char *) &udwSpiData);

	/* EOF Resize PreDxo */

	udwSpiData = 1;
	SPI_Write(YUSHAN_EOF_RESIZE_PRE_DXO_AUTOMATIC_CONTROL, 1, (unsigned char *) &udwSpiData);
	SPI_Write(YUSHAN_EOF_RESIZE_PRE_DXO_ENABLE, 1, (unsigned char *) &udwSpiData);

	/* EOF Resize PostDxo */
	udwSpiData = 0;/* Manual Mode */
	SPI_Write(YUSHAN_EOF_RESIZE_POST_DXO_AUTOMATIC_CONTROL, 1, (unsigned char *) &udwSpiData);
	udwSpiData = bSofEofLength;
	SPI_Write(YUSHAN_EOF_RESIZE_POST_DXO_H_SIZE, 1, (unsigned char *) &udwSpiData);
	udwSpiData = 1;
	SPI_Write(YUSHAN_EOF_RESIZE_POST_DXO_ENABLE, 1, (unsigned char *) &udwSpiData);

	SPI_Read(YUSHAN_CLK_DIV_FACTOR, 1, (unsigned char *) &bDxoClkDiv);
	SPI_Read(YUSHAN_CLK_DIV_FACTOR + 1, 1, (unsigned char *) &bPixClkDiv);
	/* uwHsizeStill= (sImageChar->uwXAddrEnd - sImageChar->uwXAddrStart+ 1)/sImageChar->uwXOddInc;*/
	/* uwHsizeVf= (sImageChar->uwXAddrEnd - sImageChar->uwXAddrStart+ 1)/sImageChar->uwXOddInc;*/

	/* Idp Gen */

	for (bCount = 0 ; bCount < 14 ; bCount++) {

		udwSpiData = sInitStruct->sFrameFormat[bCount].uwWordcount |
			(sInitStruct->sFrameFormat[bCount].bDatatype << 16) |
			(sInitStruct->sFrameFormat[bCount].bActiveDatatype << 24);

		SPI_Write((uint16_t)(YUSHAN_IDP_GEN_WC_DI_0+4*bCount),4,(unsigned char*)&udwSpiData);

		udwSpiData = sInitStruct->sFrameFormat[bCount].uwWordcount ;
		SPI_Write((uint16_t)(YUSHAN_CSI2_TX_PACKET_SIZE_0+0xc*bCount),4,(unsigned char*)&udwSpiData);

		udwSpiData = sInitStruct->sFrameFormat[bCount].bDatatype ;
		SPI_Write((uint16_t)(YUSHAN_CSI2_TX_DI_INDEX_CTRL_0+0xc*bCount),1,(unsigned char*)&udwSpiData);

		if (sInitStruct->sFrameFormat[bCount].bSelectStillVfMode == YUSHAN_FRAME_FORMAT_VF_MODE) {
			uwHsizeVf = (sInitStruct->sFrameFormat[bCount].uwWordcount * 8) / (sInitStruct->uwPixelFormat & 0x0f);
			bVfIndex = bCount;
		}

		if (sInitStruct->sFrameFormat[bCount].bSelectStillVfMode == YUSHAN_FRAME_FORMAT_STILL_MODE) {
			uwHsizeStill = (sInitStruct->sFrameFormat[bCount].uwWordcount * 8) / (sInitStruct->uwPixelFormat & 0x0f);
			bStillIndex = bCount;
		}
	}

	udwSpiData = bVfIndex | (bStillIndex << 4) | (bSofEofLength << 8) | 0x3ff0000;/********* Check ****************/
	SPI_Write(YUSHAN_IDP_GEN_CONTROL, 4, (unsigned char *) &udwSpiData);
	udwSpiData = 0xc810;
	SPI_Write(YUSHAN_IDP_GEN_ERROR_LINES_EOF_GAP, 2, (unsigned char *) &udwSpiData);

	/* CSI2TX */

	udwSpiData = sInitStruct->bNumberOfLanes - 1;
	SPI_Write(YUSHAN_CSI2_TX_NUMBER_OF_LANES, 1, (unsigned char *) &udwSpiData);

	udwSpiData = 1;
	SPI_Write(YUSHAN_CSI2_TX_PACKET_CONTROL, 1, (unsigned char *) &udwSpiData);
	udwSpiData = 1;
	SPI_Write(YUSHAN_CSI2_TX_ENABLE, 1, (unsigned char *) &udwSpiData);

	if (bBypassDxo) {
		/* DT Filter */
		/***** DT Filter 0 ******/
		udwSpiData = 0x1;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH0, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0xd;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH1, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x02;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH2, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x03;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH3, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x01;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_ENABLE, 1, (unsigned char *) &udwSpiData);

		/***** DT Filter 1 ******/
		udwSpiData = 0x5;
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH0, 1, (unsigned char *) &udwSpiData);
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH2, 1, (unsigned char *) &udwSpiData);
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH3, 1, (unsigned char *) &udwSpiData);
		/* udwSpiData = 0x02;*/
		udwSpiData = 0x05;
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH1, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x01;
		SPI_Write(YUSHAN_DTFILTER_DXO_ENABLE, 1, (unsigned char *) &udwSpiData);
	} else {
		/* DT Filter */
		/***** DT Filter 0 ******/
		udwSpiData = 0x1;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH0, 1, (unsigned char *) &udwSpiData);
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH1, 1, (unsigned char *) &udwSpiData);
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH2, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x3;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH3, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x01;
		SPI_Write(YUSHAN_DTFILTER_BYPASS_ENABLE, 1, (unsigned char *) &udwSpiData);

		/***** DT Filter 1 ******/
		udwSpiData = 0xd;
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH0, 1, (unsigned char *) &udwSpiData);
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH2, 1, (unsigned char *) &udwSpiData);
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH3, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x02;
		SPI_Write(YUSHAN_DTFILTER_DXO_MATCH1, 1, (unsigned char *) &udwSpiData);
		udwSpiData = 0x01;
		SPI_Write(YUSHAN_DTFILTER_DXO_ENABLE, 1, (unsigned char *) &udwSpiData);
	}

	/* T1 DMA programming */

	/* For PDP */
	udwSpiData = (uwHsizeStill & 0xff) | ((uwHsizeVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x6205;*/
	udwSpiData = DXO_PDP_BASE_ADDR + DxOPDP_visible_line_size_7_0;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwHsizeStill>>8) & 0xff) | (((uwHsizeVf>>8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 8, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x6206;*/
	udwSpiData = DXO_PDP_BASE_ADDR + DxOPDP_visible_line_size_15_8;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 8 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = 0x0101;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 16, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x7a0c;*/
	udwSpiData = DXO_PDP_BASE_ADDR + DxOPDP_newFrameCmd;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 16 + 2, 3, (unsigned char *) &udwSpiData);

	/* For DPP */
	udwSpiData = (uwHsizeStill & 0xff) | ((uwHsizeVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 24, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x10205;*/
	udwSpiData = DXO_DPP_BASE_ADDR + DxODPP_visible_line_size_7_0;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 24 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwHsizeStill>>8) & 0xff) | (((uwHsizeVf>>8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 32, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x10206;*/
	udwSpiData = DXO_DPP_BASE_ADDR + DxODPP_visible_line_size_15_8;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 32 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = 0x0101;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 40, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x1d00c;*/
	udwSpiData = DXO_DPP_BASE_ADDR + DxODPP_newFrameCmd;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 40 + 2, 3, (unsigned char *) &udwSpiData);

	/* For DOP */
	udwSpiData = (uwHsizeStill & 0xff) | ((uwHsizeVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 48 , 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x8205;*/
	udwSpiData = DXO_DOP_BASE_ADDR + DxODOP_visible_line_size_7_0;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 48 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwHsizeStill>>8) & 0xff) | (((uwHsizeVf>>8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 56, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0x8206;*/
	udwSpiData = DXO_DOP_BASE_ADDR + DxODOP_visible_line_size_15_8;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 56 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = 0x0101;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 64, 2, (unsigned char *) &udwSpiData);
	/* udwSpiData = 0xe80c;*/
	udwSpiData = DXO_DOP_BASE_ADDR + DxODOP_newFrameCmd;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 64 + 2, 3, (unsigned char *) &udwSpiData);


	/* For LBE PreDxo */
	udwSpiData = (uwHsizeStill & 0xff) | ((uwHsizeVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 72, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LBE_PRE_DXO_H_SIZE;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 72 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwHsizeStill>>8) & 0xff) | (((uwHsizeVf>>8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 80, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LBE_PRE_DXO_H_SIZE + 1;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 80 + 2, 3, (unsigned char *) &udwSpiData);

	/* For LBE PostDxo */
	udwSpiData = (uwHsizeStill & 0xff) | ((uwHsizeVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 88, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LBE_POST_DXO_H_SIZE;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 88 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwHsizeStill>>8) & 0xff) | (((uwHsizeVf>>8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 96, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LBE_POST_DXO_H_SIZE + 1;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 96 + 2, 3, (unsigned char *) &udwSpiData);

	/* For Lecci */
	udwSpiData = (uwHsizeStill & 0xff) | ((uwHsizeVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 104, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LECCI_LINE_SIZE;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 104 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwHsizeStill>>8) & 0xff) | (((uwHsizeVf>>8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 112, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LECCI_LINE_SIZE + 1;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 112 + 2, 3, (unsigned char *) &udwSpiData);

	/***** Lecci prog ******/
	/* Min_interline = Freq_DXO/Freq_Pix*interline. */

	if (bDxoClkDiv != 0 && bPixClkDiv != 0) {
		uwLecciVf    = sInitStruct->uwLineBlankVf*bPixClkDiv / bDxoClkDiv;
		uwLecciStill = sInitStruct->uwLineBlankStill*bPixClkDiv / bDxoClkDiv;
	} else {
		uwLecciVf    = 300;
		uwLecciStill = 300;
	}

	/* For Lecci */
	udwSpiData = (uwLecciStill & 0xff) | ((uwLecciVf & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 120, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LECCI_MIN_INTERLINE;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 120 + 2, 3, (unsigned char *) &udwSpiData);
	udwSpiData = ((uwLecciStill >> 8) & 0xff) | (((uwLecciVf >> 8) & 0xff) << 8);
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 128, 2, (unsigned char *) &udwSpiData);
	udwSpiData = YUSHAN_LECCI_MIN_INTERLINE + 1;
	SPI_Write(YUSHAN_T1_DMA_MEM_LOWER_ELT + 128 + 2, 3, (unsigned char *) &udwSpiData);


	udwSpiData = 17;/* 9 enteried in T1 DMA memory */
	SPI_Write(YUSHAN_T1_DMA_REG_REFILL_ELT_NB, 1, (unsigned char *) &udwSpiData);
	udwSpiData = 1;
	SPI_Write(YUSHAN_T1_DMA_REG_ENABLE, 1, (unsigned char *) &udwSpiData);

	/* LBE PreDxo */
	/*************************CHECK*********************/

	udwSpiData = 4; /*****************Check*****************/
	SPI_Write(YUSHAN_LBE_PRE_DXO_READ_START, 1, (unsigned char *) &udwSpiData);
	udwSpiData = 1; /*****************Check*****************/
	SPI_Write(YUSHAN_LBE_PRE_DXO_ENABLE, 1, (unsigned char *) &udwSpiData);

	/* LBE Post Dxo */
	/*************************CHECK*********************/
	udwSpiData=80;  /*****************Check*****************/
	SPI_Write(YUSHAN_LBE_POST_DXO_READ_START, 1, (unsigned char *) &udwSpiData);
	udwSpiData = 1; /*****************Check*****************/
	SPI_Write(YUSHAN_LBE_POST_DXO_ENABLE, 1, (unsigned char *) &udwSpiData);

	/***** Lecci prog ******/
	/* Ref_target = Freq_Pix/ Freq_DXO *20 */
	/* Window_size = if (Freq_DXO/Freq_Pix)  12 else 20 */
	bDXODecimalFactor = bIdpDecimalFactor = 0;

	if(((bDxoClkDiv&0x01) == 1))		/* Checking for Decimal point */
		bDXODecimalFactor = 1;		/* It is (2*0.5). In case of WindowSize/Ref_Traget.. (2*ClkDiv) */
	if(((bPixClkDiv&0x01) == 1))		/* Checking for Decimal point */
		bIdpDecimalFactor = 1;

	bDxoClkDiv = bDxoClkDiv>>1;		/* Integral part of divider */
	bPixClkDiv = bPixClkDiv>>1;		/* Integral part of divider */

	if(bDxoClkDiv==bPixClkDiv)
		udwSpiData = (((2*bDxoClkDiv + bDXODecimalFactor)<<8) | 0x1);
	else
		udwSpiData = (((2*bDxoClkDiv + bDXODecimalFactor)<<8)|((bPixClkDiv*2 + bIdpDecimalFactor)-1));
/* Yushan API 10.0 End */

	SPI_Write(YUSHAN_LECCI_OUT_BURST_CTRL,2,(unsigned char*)&udwSpiData);
	udwSpiData=0x01;
	SPI_Write(YUSHAN_LECCI_ENABLE,1,(unsigned char*)&udwSpiData);

	CDBG("[CAM] Yushan_Init return success\n");
	return SUCCESS;

}


/*************************************************************
API Functtion: Update DXO image characteristics parameters.
Same for all the the three IPs
Input: ImageStruct
**************************************************************/
bool_t Yushan_Update_ImageChar(Yushan_ImageChar_t *sImageChar)
{
	uint8_t  *pData;
	bool_t fStatus;
	uint32_t udwSpiData;
	uint8_t  bData[20];

	/* Assumption that the sturcture is packed and there is no padding. */
	pData = (uint8_t *)bData;

	bData[0] = sImageChar->bImageOrientation;
	bData[1] = sImageChar->uwXAddrStart & 0xff;
	bData[2] = sImageChar->uwXAddrStart >> 8;
	bData[3] = sImageChar->uwYAddrStart & 0xff;
	bData[4] = sImageChar->uwYAddrStart >> 8;
	bData[5] = sImageChar->uwXAddrEnd & 0xff;
	bData[6] = sImageChar->uwXAddrEnd >> 8;
	bData[7] = sImageChar->uwYAddrEnd & 0xff;
	bData[8] = sImageChar->uwYAddrEnd >> 8;
	bData[9] = sImageChar->uwXEvenInc & 0xff;
	bData[10] = sImageChar->uwXEvenInc >> 8;
	bData[11] = sImageChar->uwXOddInc & 0xff;
	bData[12] = sImageChar->uwXOddInc >> 8;
	bData[13] = sImageChar->uwYEvenInc & 0xff;
	bData[14] = sImageChar->uwYEvenInc >> 8;
	bData[15] = sImageChar->uwYOddInc & 0xff;
	bData[16] = sImageChar->uwYOddInc >> 8;
	bData[17] = sImageChar->bBinning ;

	/* Programming to the DXO IPs */
	fStatus = SPI_Write(DXO_PDP_BASE_ADDR + DxOPDP_image_orientation_7_0, 18, pData);
	bData[17] = 0;
	fStatus = SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_image_orientation_7_0, 18, pData);
	udwSpiData = 0x10000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);
	fStatus = SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_image_orientation_7_0-0x8000, 18, pData);

	/* Restoring the SpiBaseAddress to 0x08000. */
	udwSpiData = 0x8000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);

	return fStatus;

}

/*************************************************************
API Functtion: Update Sensor paramters. Same for all the the
three IPs.
Input: GainExpInfo
**************************************************************/
bool_t Yushan_Update_SensorParameters(Yushan_GainsExpTime_t *sGainsExpInfo)
{
	uint8_t *pData;
	bool_t fStatus;
	uint32_t udwSpiData;
	uint8_t bData[20];

	pData = (uint8_t *)bData;

	bData[0] = sGainsExpInfo->uwAnalogGainCodeGR & 0xff;
	bData[1] = sGainsExpInfo->uwAnalogGainCodeGR >> 8;
	bData[2] = sGainsExpInfo->uwAnalogGainCodeR & 0xff;
	bData[3] = sGainsExpInfo->uwAnalogGainCodeR >> 8;
	bData[4] = sGainsExpInfo->uwAnalogGainCodeB & 0xff;
	bData[5] = sGainsExpInfo->uwAnalogGainCodeB >> 8;
	bData[6] = sGainsExpInfo->uwPreDigGainGR & 0xff;
	bData[7] = sGainsExpInfo->uwPreDigGainGR >> 8;
	bData[8] = sGainsExpInfo->uwPreDigGainR & 0xff;
	bData[9] = sGainsExpInfo->uwPreDigGainR >> 8;
	bData[10] = sGainsExpInfo->uwPreDigGainB & 0xff;
	bData[11] = sGainsExpInfo->uwPreDigGainB >> 8;
	bData[12] = sGainsExpInfo->uwExposureTime & 0xff;
	bData[13] = sGainsExpInfo->uwExposureTime >> 8;

	/* Programming to the DXO IPs */
	fStatus = SPI_Write(DXO_PDP_BASE_ADDR + DxOPDP_analogue_gain_code_greenr_7_0, 14, pData);
	fStatus = SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_analogue_gain_code_greenr_7_0, 12, pData);
	pData = (uint8_t *) &(sGainsExpInfo->bRedGreenRatio);
	fStatus = SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_red_green_ratio_7_0, 2, pData);
	udwSpiData = 0x10000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);
	pData = (uint8_t *)(sGainsExpInfo);
	fStatus = SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_analogue_gain_code_greenr_7_0-0x8000, 2, pData);
	pData = (uint8_t *) &(sGainsExpInfo->uwPreDigGainGR);
	fStatus = SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_pre_digital_gain_greenr_7_0-0x8000, 2, pData);
	pData = (uint8_t *) &(sGainsExpInfo->uwExposureTime);
	fStatus = SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_exposure_time_7_0-0x8000, 2, pData);

	/* Restoring the SpiBaseAddress to 0x08000. */
	udwSpiData = 0x8000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);

	return fStatus;

}

/***************************************************************************
API Function: Update Pdp Tuning paramter for Dead pixel correction.
***************************************************************************/
bool_t Yushan_Update_DxoPdp_TuningParameters(Yushan_DXO_PDP_Tuning_t *sDxoPdpTuning)
{
	uint8_t  *pData, fStatus;
	uint8_t bData[3];

	pData = (uint8_t *)bData;
	bData[0] = sDxoPdpTuning->bDeadPixelCorrectionLowGain;
	bData[1] = sDxoPdpTuning->bDeadPixelCorrectionMedGain;
	bData[2] = sDxoPdpTuning->bDeadPixelCorrectionHiGain;

	/* Only three bytes need to be written. Fourth byte write will overwrite the */
	/* DxOPDP_frame_number_7_0 register */
	fStatus = SPI_Write(DXO_PDP_BASE_ADDR + DxOPDP_dead_pixels_correction_lowGain_7_0, 3, pData);

	return fStatus;
}

/***************************************************************************
API Function: Update Dpp Tuning parameters
***************************************************************************/
bool_t Yushan_Update_DxoDpp_TuningParameters(Yushan_DXO_DPP_Tuning_t *sDxoDppTuning)
{

	uint8_t  *pData;
	bool_t	fStatus;
	uint32_t udwSpiData;
	uint8_t bData[10];

	pData = (uint8_t *)bData;

	bData[0] = sDxoDppTuning->bTemporalSmoothing;
	bData[1] = sDxoDppTuning->uwFlashPreflashRating & 0xff ;
	bData[2] = sDxoDppTuning->uwFlashPreflashRating >> 8;
	bData[3] = sDxoDppTuning->bFocalInfo;

	udwSpiData = 0x10000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);
	fStatus = SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_temporal_smoothing_7_0-0x8000, 4,  pData);

	/* Restoring the SpiBaseAddress to 0x08000. */
	udwSpiData = 0x8000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);

	return fStatus;
}


/***************************************************************************
API Function: Update Dop Tuning parameters
Input: DOP Tuning parameters
***************************************************************************/
bool_t Yushan_Update_DxoDop_TuningParameters(Yushan_DXO_DOP_Tuning_t *sDxoDopTuning)
{
	uint8_t *pData;
	bool_t fStatus;
	uint8_t bData[10];

	pData = (uint8_t *) bData;

#if 0
	bData[0] = sDxoDopTuning->uwForceClosestDistance & 0xff ;
	bData[1] = sDxoDopTuning->uwForceClosestDistance >> 8;
	bData[2] = sDxoDopTuning->uwForceFarthestDistance & 0xff ;
	bData[3] = sDxoDopTuning->uwForceFarthestDistance >> 8;
#endif
	bData[0] = sDxoDopTuning->bEstimationMode ;
	/* First five consecutive registers written. */
	fStatus = SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_estimation_mode_7_0, 1, pData);

	bData[0] = sDxoDopTuning->bSharpness;
	bData[1] = sDxoDopTuning->bDenoisingLowGain;
	bData[2] = sDxoDopTuning->bDenoisingMedGain;
	bData[3] = sDxoDopTuning->bDenoisingHiGain;
	bData[4] = sDxoDopTuning->bNoiseVsDetailsLowGain ;
	bData[5] = sDxoDopTuning->bNoiseVsDetailsMedGain ;
	bData[6] = sDxoDopTuning->bNoiseVsDetailsHiGain ;
	bData[7] = sDxoDopTuning->bTemporalSmoothing ;

	fStatus &= SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_sharpness_7_0, 8, pData);

	return fStatus;
}

/***************************************************************************
API Function: Update_Commit. This function will apply the changes programmed
in the DXO parameters. This function will be called at the start of the frame
so that the changes will be well reflected in the coming frame.
***************************************************************************/
bool_t Yushan_Update_Commit(uint8_t  bPdpMode, uint8_t  bDppMode, uint8_t  bDopMode)
{
	uint8_t bData, *pData;
	bool_t fStatus = SUCCESS;
	uint32_t udwSpiData ;
	uint8_t bSpiData;

	/* Each Block Enable Flag*/
	uint8_t PDP_enable = 1;
	uint8_t black_level_enable = 1;
	uint8_t dead_pixel_enable = 1;
	uint8_t phase_repair_enable = 1;
	uint8_t DOP_enable = 1;
	uint8_t denoise_enable = 1;
	uint8_t DPP_enable = 1;


	bData = 1;
	pData = &bData;

	/* bPdpMode = 0; bDppMode = 0; bDopMode = 0; */
	
	/* pr_info("[CAM] %s:%d, %d, %d\n",__func__, bPdpMode, bDppMode, bDopMode); */

	/* Setting DXO mode */
#if 1
	if (!PDP_enable)
		bSpiData = 0x0;
	else {
		bSpiData = 0x1;

		if (!black_level_enable)
			bSpiData = bSpiData | (1<<4);
		else
			bSpiData = bSpiData & (~(1<<4));

		if (!dead_pixel_enable)
			bSpiData = bSpiData | (1<<3);
		else
			bSpiData = bSpiData & (~(1<<3));
		if (!phase_repair_enable)
			bSpiData = bSpiData | (1<<5);
		else
			bSpiData = bSpiData & (~(1<<5));
	}
	CDBG("PDP_mode = %d\n", bSpiData);
	SPI_Write(DXO_PDP_BASE_ADDR+DxOPDP_mode_7_0, 1, (uint8_t *)(&bSpiData));
#else
	SPI_Write(DXO_PDP_BASE_ADDR + DxOPDP_mode_7_0, 1, &bPdpMode);
#endif
	/* Giving executing command, so that the programmed mode will be executed. */
	SPI_Write(DXO_PDP_BASE_ADDR + DxOPDP_execCmd, 1, pData);

	/* Setting DXO mode */
#if 1
	if (!DOP_enable)
		bSpiData = 0x00;
	else {
		if (!denoise_enable)
			bSpiData = 0x11;
		else
			bSpiData = 0x1;
	}
	CDBG("DOP_mode = %d\n", bSpiData);
	SPI_Write(DXO_DOP_BASE_ADDR+DxODOP_mode_7_0, 1, (uint8_t *)(&bSpiData));
#else
	SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_mode_7_0, 1, &bDopMode);
#endif
	/* Giving executing command, so that the programmed mode will be executed. */
	SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_execCmd, 1, pData);

	udwSpiData = 0x10000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);
	/* Setting DXO mode */
#if 1
	if (!DPP_enable)
		bSpiData = 0x00;
	else
		bSpiData = 0x03;
	CDBG("DPP_mode = %d\n", bSpiData);
	SPI_Write(DXO_DPP_BASE_ADDR+DxODPP_mode_7_0-0x8000, 1, (uint8_t *)&bSpiData);
#else
	SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_mode_7_0-0x8000, 1, &bDppMode);
#endif
	udwSpiData = 0x18000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *) &udwSpiData);
	/* Giving executing command, so that the programmed mode will be executed. */
	SPI_Write(DXO_DPP_BASE_ADDR + DxODPP_execCmd-0x10000, 1, pData);


	/* Restoring the SpiBaseAddress to 0x08000. */
	udwSpiData = 0x8000 ;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS,4,(uint8_t *)&udwSpiData);

#if 0
	/* Wait for the EndOfExec interrupts. This indicates the completion of the */
	/* execution command */
	fStatus = Yushan_WaitForInterruptEvent(EVENT_PDP_EOF_EXECCMD, TIME_10MS);
	if (!fStatus)
	{
		pr_err("[CAM] EVENT_PDP_EOF_EXECCMD fail\n");
		return FAILURE;
	}

	fStatus = Yushan_WaitForInterruptEvent2(EVENT_DOP7_EOF_EXECCMD, TIME_10MS);
	if (!fStatus)
	{
		pr_err("[CAM] EVENT_DOP7_EOF_EXECCMD fail\n");
		return FAILURE;
	}

	fStatus = Yushan_WaitForInterruptEvent(EVENT_DPP_EOF_EXECCMD, TIME_10MS);
	if (!fStatus)
	{
		pr_err("[CAM] EVENT_DPP_EOF_EXECCMD fail\n");
		return FAILURE;
	}
#endif

	return fStatus;
}

/********************************************************************
API Function:	Enable Interrupts or Disable Interrupts.
Input:			Pointer to 88bit Interrupt_BitMask, which carry 1 or
				0 for the interrupts to be enabled or disabled resp.
Return:			Success or Failure
*********************************************************************/
bool_t Yushan_Intr_Enable(uint8_t *pIntrMask)
{
	/* Two counters, bIntrCount to trace through Interrupt bits 1 through 83 and another counter */
	/* bIntr32Count to trace through 32 to 0 for checking interrupts in 32 bit double word. */
	uint8_t		bTotalEventInASet, *pSpiData;
	uint8_t		bIntrCount = 1, bIntr32Count = 1, bIntrCountInSet = 0, bIntrSetID = 1;
	uint16_t	uwIntrSetOffset;
	uint32_t	udwLocalIntrMask, udwInterruptSetting, *pLocalIntrMask, udwSpiData = 0;

	/* First element 0 does not represents any set. */
	uint8_t		bFirstIndexForSet[] = {0, 1, 5, 11, 17, 23, 27, 58, 62, 69, 77, 79, 82, 83};

	/* First four bytes of input IntrMask */
	pLocalIntrMask = (uint32_t *)(pIntrMask);
	udwLocalIntrMask = *pLocalIntrMask;

	/* For the first set, TotalElement = (5-1);*/
	bTotalEventInASet = (bFirstIndexForSet[2] - bFirstIndexForSet[1]);

	/* Trace through IntrMask on bit basis. Plus one to tackle writes of the last set. */
	while (bIntrCount <= (TOTAL_INTERRUPT_COUNT + 1)) {
		/* If counter/index reaches the number of interrupts in the set, then write the packet to the
		*  interrupt registers. The time when these two registers are equal, double word packet has been
		*  already created considering all the bits of that set and index has now been pointing to one
		*  location more than the number of interrupts/events. */
		if (bIntrCountInSet == bTotalEventInASet) {
			/* InterruptSet offset from the Interrupt base address. Minus one due to the fact that */
			/* InterSetID starts with 1 not 0. */
			uwIntrSetOffset = ((bIntrSetID - 1) * INTERRUPT_SET_SIZE);
			/* Write udwSpiData to the Interrupt Enable regsiter. Since clks are not dirven by Pll, no auto */
			/*  incremental writes. */
			pSpiData = (uint8_t *)(&udwSpiData);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE), 1, pSpiData);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE+1), 1, pSpiData+1);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE+2), 1, pSpiData+2);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE+3), 1, pSpiData+3);

			/* pr_info("Yushan_Intr_Enable: udwSpiData:  0x%x, uwIntrSetOffset: 0x%x\n", udwSpiData, uwIntrSetOffset);*/

			/* Complement of udwSpiData. */
			udwSpiData = (uint32_t)(~udwSpiData);
			/* write complemented udwSpiData to the Interrupt Disable regsiters. */
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_DISABLE), 1, pSpiData);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_DISABLE+1), 1, pSpiData+1);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_DISABLE+2), 1, pSpiData+2);
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_DISABLE+3), 1, pSpiData+3);

			/* Reset the counter tracking the interrupts in the current set. Also reset the data. */
			bIntrCountInSet = 0;
			udwSpiData = 0;

			/* As Intr32Count and IntrCount has been incremented and not used while doing SPI writes. */
			bIntr32Count--;
			bIntrCount--;
			/* Increase the SetID too. */
			bIntrSetID++;

			/* Set the ElementInASet. */
			if (bIntrSetID < 13)
				bTotalEventInASet = (bFirstIndexForSet[bIntrSetID + 1] - bFirstIndexForSet[bIntrSetID]);
			else if (bIntrSetID == 13)
				bTotalEventInASet = 2;/* Only two events in the last IntrSet */

		} else {
			/* udwInterruptSetting gives the per bit setting, 1 or 0. */
			udwInterruptSetting = ((udwLocalIntrMask>>(bIntr32Count-1)) & 0x01);
			/* Create a double word */
			udwSpiData |= (udwInterruptSetting<<(bIntrCountInSet));

			/* track to next interrupt bit. */
			bIntrCountInSet++;
		}

		/* If double word has been exhausted, look for the next four bytes. */
		if (!(bIntr32Count % 32)) {
			/* Reset the 32Count counter to 1, so as to shift udwLocalIntrMask by not more than 32 bits(max). */
			bIntr32Count = 1;

			/* After every four bytes, take next double word from the *pIntrMask. */
			pLocalIntrMask++;
			udwLocalIntrMask	= *pLocalIntrMask;
		} else
			bIntr32Count++;

		/* Increase the top counter. */
		bIntrCount++;

	}

	return SUCCESS;
}

/******************************************************************
API Internal Function:	DXO Constraints check part of Clk_Init
Input:					DXO inputs, Clks and Min limits
Return:					SUCCESS OR FAILURE, if DXO constraints
						met or violated, respectively.
*******************************************************************/
bool_t Yushan_CheckDxoConstraints(uint32_t udwParameters, uint32_t udwMinLimit, uint32_t fpDxo_Clk, uint32_t fpPixel_Clk, uint16_t uwFullLine, uint32_t *pMinValue)
{
	/* fpDxo_Clk and fpPixel_Clk are in fixed point notations. */

	/* uint32_t	udwLocalParameters, udwLocalMinLimit, udwLocalFullLine;*/
	uint16_t	uwLocalDxoClk, uwLocalPixelClk;
	uint32_t	udwTemp1, udwTemp2;

	/* Converting the params back to non fixed point format */
	uwLocalDxoClk = fpDxo_Clk>>16;
	uwLocalPixelClk = fpPixel_Clk>>16;

	/* In the absence of foating point arithmatic. Multiply numerator by some big value, lets 0xFFFF:
	*  so that divisions (below in comment) do not yield 0 for both the divisions.
	*  (parameter/pixelclk) < (MinLimit/DxoClk): */
	udwTemp1 = (udwParameters * 0xFFFF)/uwLocalPixelClk;
	udwTemp2 = (udwMinLimit * 0xFFFF)/uwLocalDxoClk;

	/* if ((parameter/pixelclk) < (MinLimit/DxoClk)) is same as [(((parameter*0xFFFF)/pixelclk) < ((MinLimit *0xFFFF)/DxoClk))] */
	if (udwTemp1 < udwTemp2) {
		/* Dxo Constraints not met:
		*  return ceil(((MinLimit/DxoClk)*PixelClk)/FullLine);*/

		/* Dividing again by 0xFFFF to get the actual value. */
		udwTemp1 = ((udwTemp2*uwLocalPixelClk)/0xFFFF);/* /uwFullLine);FFFFF); /uwFullLine);FullLine is 1 here. */
		*pMinValue = (udwTemp1 + 1);

		return FAILURE;
	}

	/* No error or exception */
	return SUCCESS;
}

/*******************************************************************************
API Internal Function:	Computing Pll dividers and multipliers
Input:					Ext_Clk (from Host), and Pll clk desired.
Constraints:			6<=Ext_Clk<= 350 and 7.93 <=pll_clk<= 1000
Return:					Pll CLk calculated or FAILURE
********************************************************************************/
uint32_t Yushan_Compute_Pll_Divs(uint32_t fpExternal_Clk, uint32_t fpTarget_PllClk)
{
	/* fploop: Pll multiplier. */
	/* fpidiv: Input div factor. */
	/* bODiv: Output div factor. */
	uint32_t		fpIDiv, fpClkPll;/* Fixed point */
	uint32_t		fpBelow[4], fpAbove[4], *pDivPll;/* Fixed point notation */

	uint16_t		uwFreqVco, uwIdFreq;
	uint8_t			bODiv, bLoop, bIDivCount = 0;
	uint8_t			bSpiData;
	bool_t			fStatus = SUCCESS;


	/* Arrays for saving Pll paramters above or below desired Pll_Clk. */
	fpBelow[PLL_CLK_INDEX] = 0;
	fpAbove[PLL_CLK_INDEX] = 0x1000000;

	/* below = {"clock": 0} */
	/* above = {"clock": 1000000} */
/* Yushan API 10.0 Start */
	for (bIDivCount = 1; bIDivCount < 8; bIDivCount++) {
		/*bIDivCount = 1;*/
/* Yushan API 10.0 End */
		/* Corresponding IDiv in Fixed point format */
		fpIDiv = Yushan_ConvertTo16p16FP((uint16_t)bIDivCount);
		/* fpidfreq: Input freq to Pll post input divider.
		*  As division yields to be non Fixed point notation. So no need to convert the result in
		*  fixed point format for the sake of computation complexity. */
		uwIdFreq = fpExternal_Clk/fpIDiv;
		if ((uwIdFreq >= 6) && (uwIdFreq <= 50)) {
			for (bLoop = 10; bLoop < 168; bLoop++) {
				/* VCO_Freq at the output of PLL. */
				/* FreqVCO = IDFreq * 2 * Loop */
				uwFreqVco = uwIdFreq * 2 * bLoop;/* Not in Fixed point for the sake of computation complexity */
				if ((uwFreqVco >= 1000) && (uwFreqVco <= 2000)) {
					for (bODiv = 1; bODiv < 64; bODiv++) {/* Not in fixed point for the sake of computation complexity */

						/* fpClkPll = fpFreqVco / (2 * fpODiv);*/
						fpClkPll = Yushan_ConvertTo16p16FP((uint16_t)(uwFreqVco / (2 * bODiv)));/* Can be replaced by */
						/* fpClkPll = ((uwFreqVco*0x10000)/(2 * bODiv));*/

						if ((fpClkPll <= fpTarget_PllClk) && (fpClkPll >= fpBelow[PLL_CLK_INDEX]))
						{ fpBelow[PLL_CLK_INDEX] = fpClkPll, fpBelow[IDIV_INDEX] = fpIDiv, fpBelow[LOOP_INDEX] = Yushan_ConvertTo16p16FP((uint16_t)bLoop), fpBelow[ODIV_INDEX] = Yushan_ConvertTo16p16FP((uint16_t)bODiv); }
						if ((fpClkPll >= fpTarget_PllClk) && (fpClkPll <= fpAbove[PLL_CLK_INDEX]))
						{ fpAbove[PLL_CLK_INDEX] = fpClkPll, fpAbove[IDIV_INDEX] = fpIDiv, fpAbove[LOOP_INDEX] = Yushan_ConvertTo16p16FP((uint16_t)bLoop), fpAbove[ODIV_INDEX] = Yushan_ConvertTo16p16FP((uint16_t)bODiv); }
					}
				}
			}
		}
	}

	if ((fpTarget_PllClk - fpBelow[PLL_CLK_INDEX]) < (fpAbove[PLL_CLK_INDEX] - fpTarget_PllClk))
		pDivPll = &fpBelow[PLL_CLK_INDEX];
	else
		pDivPll = &fpAbove[PLL_CLK_INDEX];

	/* program pll dividers to H/W. */
	/* Shifting towards right will yeilds non fixed point values */
	bSpiData = (uint8_t)((*(pDivPll + 1))>>16);
	fStatus	&= SPI_Write(YUSHAN_PLL_CTRL_MAIN + 1, 1, (uint8_t *)  (&bSpiData));
	bSpiData = (uint8_t)((*(pDivPll + 2))>>16);
	fStatus	&= SPI_Write(YUSHAN_PLL_LOOP_OUT_DF, 1, (uint8_t *) (&bSpiData));
	bSpiData = (uint8_t)((*(pDivPll + 3))>>16);
	fStatus	&= SPI_Write(YUSHAN_PLL_LOOP_OUT_DF + 1, 1, (uint8_t *) (&bSpiData));

	if (!fStatus)
		return FAILURE;/* If any writes fails, then return FAILURE. */
	else	/* returning pll_clk */
		return *pDivPll;
}

/****************************************************************************
API Function:	Read DXO IP versions
Input:			pointer to sYushanVersionInfo struct
Return:			Update sYushanVersionInfo
*****************************************************************************/
bool_t Yushan_Get_Version_Information(Yushan_Version_Info_t *sYushanVersionInfo)
{
#if 0
	uint32_t udwSpiBaseIndex;

	SPI_Read((DxODOP_ucode_id_7_0 + DXO_DOP_BASE_ADDR) , 4, (uint8_t *)(&sYushanVersionInfo->udwDopVersion));
	SPI_Read((DxODOP_calib_id_0_7_0 + DXO_DOP_BASE_ADDR), 4, (uint8_t *)(&sYushanVersionInfo->udwDopCalibrationVersion));

	udwSpiBaseIndex = 0x010000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	SPI_Read(DxODPP_ucode_id_7_0 + DXO_DPP_BASE_ADDR-0x8000, 4, (uint8_t *)(&sYushanVersionInfo->udwDppVersion));
	SPI_Read(DxODPP_calib_id_0_7_0 + DXO_DPP_BASE_ADDR-0x8000, 4, (uint8_t *)(&sYushanVersionInfo->udwDppCalibrationVersion));

	udwSpiBaseIndex = 0x08000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	SPI_Read((DxOPDP_ucode_id_7_0 + DXO_PDP_BASE_ADDR) , 4, (uint8_t *)(&sYushanVersionInfo->udwPdpVersion));
	SPI_Read((DxOPDP_calib_id_0_7_0 + DXO_PDP_BASE_ADDR), 4, (uint8_t *)(&sYushanVersionInfo->udwPdpCalibrationVersion));

#endif
	sYushanVersionInfo->bApiMajorVersion = API_MAJOR_VERSION;
	sYushanVersionInfo->bApiMinorVersion = API_MINOR_VERSION;

	return SUCCESS;
}

/*********************************************************************
				Interrupt Functions Starts Here
**********************************************************************/
/*******************************************************************
WaitForInterruptEvent:
Waiting for Interrupt and adding the same in the Interrupt list.
*******************************************************************/
bool_t Yushan_WaitForInterruptEvent (uint8_t bInterruptId ,uint32_t udwTimeOut)
{

	int					 counterLimit;
	/* int counter = 0;*/
	bool_t				fStatus = 0; /*, INTStatus=0;*/
	/* int rc = 0;*/

	switch ( udwTimeOut )
	{
	case TIME_5MS :
		counterLimit = 100 ;
		break;
	case TIME_10MS :
		counterLimit = 200 ;
		break;
	case TIME_20MS :
		counterLimit = 400 ;
		break;
	case TIME_50MS :
		counterLimit = 1000 ;
		break;
	case TIME_200MS :
		counterLimit = 4000 ;
		break;
	default :
		counterLimit = 50 ;
		break;
}

	CDBG("[CAM] %s begin interrupt wait\n",__func__);
	//init_waitqueue_head(&yushan_int.yushan_wait);
	//rc = wait_event_interruptible_timeout(yushan_int.yushan_wait,
	wait_event_timeout(yushan_int.yushan_wait,
	atomic_read(&interrupt),
		counterLimit/200);/* counterLimit/200 */
	CDBG("[CAM] %s end interrupt: %d; interrupt id:%d wait\n", __func__, atomic_read(&interrupt), bInterruptId);
	if(atomic_read(&interrupt))
	{
		/* INTStatus = 1; */
		atomic_set(&interrupt, 0);
	}
	Yushan_ISR();
	fStatus = Yushan_CheckForInterruptIDInList(bInterruptId);		// Work on ProtoIntrList
	CDBG("[CAM] %s Yushan_CheckForInterruptIDInList:%d \n",__func__, fStatus);
	/* Interrupt has been served, so remove the same from the InterruptList. */
	Yushan_AddnRemoveIDInList(bInterruptId, udwProtoInterruptList, DEL_INTR_FROM_LIST); // Worked on ProtoIntrList
	/* fStatus = CheckForInterruptIDInList(bInterruptId);*/		/* Work on ProtoIntrList */
	CDBG("[CAM] %s Del Yushan_CheckForInterruptIDInList:%d \n",__func__, fStatus);
	if ((fStatus)/*||(INTStatus)*/)
		return SUCCESS;
	else
		return FAILURE;
}

bool_t Yushan_WaitForInterruptEvent2 (uint8_t bInterruptId ,uint32_t udwTimeOut)
{

	int					 counterLimit;
	/* int counter = 0;*/
	bool_t				fStatus = 0; /*, INTStatus=0;*/
	/* int rc = 0;*/

	switch ( udwTimeOut )
	{
	case TIME_5MS :
		counterLimit = 100 ;
		break;
	case TIME_10MS :
		counterLimit = 200 ;
		break;
	case TIME_20MS :
		counterLimit = 400 ;
		break;
	case TIME_50MS :
		counterLimit = 1000 ;
		break;
	case TIME_200MS :
		counterLimit = 4000 ;
		break;
	default :
		counterLimit = 50 ;
		break;
	}

	CDBG("[CAM] %s begin interrupt wait\n", __func__);
	//init_waitqueue_head(&yushan_int.yushan_wait);
	//rc = wait_event_interruptible_timeout(yushan_int.yushan_wait,
	wait_event_timeout(yushan_int.yushan_wait,
	atomic_read(&interrupt2),
	counterLimit/200);/* counterLimit/200 */
	CDBG("[CAM] %s end interrupt: %d; interrupt id:%d wait\n", __func__, atomic_read(&interrupt2), bInterruptId);
	if(atomic_read(&interrupt2))
	{
		/* INTStatus = 1; */
		atomic_set(&interrupt2, 0);
	}
	Yushan_ISR();
	fStatus = Yushan_CheckForInterruptIDInList(bInterruptId);		// Work on ProtoIntrList
	CDBG("[CAM] %s Yushan_CheckForInterruptIDInList:%d \n",__func__, fStatus);
	/*  Interrupt has been served, so remove the same from the InterruptList. */
	Yushan_AddnRemoveIDInList(bInterruptId, udwProtoInterruptList, DEL_INTR_FROM_LIST); // Worked on ProtoIntrList
	/* fStatus = CheckForInterruptIDInList(bInterruptId);	*/	/* Work on ProtoIntrList */
	CDBG("[CAM] %s Del Yushan_CheckForInterruptIDInList:%d \n",__func__, fStatus);
	if ((fStatus)/*||(INTStatus)*/)
		return SUCCESS;
	else
		return FAILURE;
}


/*extern void sensor_streaming_on(void);*/
/*extern void sensor_streaming_off(void);*/
void Reset_Yushan(void)
{
	uint8_t	bSpiData;
	Yushan_Init_Dxo_Struct_t	sDxoStruct;
	sDxoStruct.pDxoPdpRamImage[0] = (uint8_t *)yushan_regs.pdpcode;
	sDxoStruct.pDxoDppRamImage[0] = (uint8_t *)yushan_regs.dppcode;
	sDxoStruct.pDxoDopRamImage[0] = (uint8_t *)yushan_regs.dopcode;
	sDxoStruct.pDxoPdpRamImage[1] = (uint8_t *)yushan_regs.pdpclib;
	sDxoStruct.pDxoDppRamImage[1] = (uint8_t *)yushan_regs.dppclib;
	sDxoStruct.pDxoDopRamImage[1] = (uint8_t *)yushan_regs.dopclib;

	sDxoStruct.uwDxoPdpRamImageSize[0] = yushan_regs.pdpcode_size;
	sDxoStruct.uwDxoDppRamImageSize[0] = yushan_regs.dppcode_size;
	sDxoStruct.uwDxoDopRamImageSize[0] = yushan_regs.dopcode_size;
	sDxoStruct.uwDxoPdpRamImageSize[1] = yushan_regs.pdpclib_size;
	sDxoStruct.uwDxoDppRamImageSize[1] = yushan_regs.dppclib_size;
	sDxoStruct.uwDxoDopRamImageSize[1] = yushan_regs.dopclib_size;

	sDxoStruct.uwBaseAddrPdpMicroCode[0] = yushan_regs.pdpcode->addr;
	sDxoStruct.uwBaseAddrDppMicroCode[0] = yushan_regs.dppcode->addr;
	sDxoStruct.uwBaseAddrDopMicroCode[0] = yushan_regs.dopcode->addr;
	sDxoStruct.uwBaseAddrPdpMicroCode[1] = yushan_regs.pdpclib->addr;
	sDxoStruct.uwBaseAddrDppMicroCode[1] = yushan_regs.dppclib->addr;
	sDxoStruct.uwBaseAddrDopMicroCode[1] = yushan_regs.dopclib->addr;
	pr_err("[CAM] %s\n", __func__);
	/*sensor_streaming_off();*/
	Yushan_Assert_Reset(0x001F0F10, RESET_MODULE);
	bSpiData = 1;
	Yushan_DXO_Sync_Reset_Dereset(bSpiData);
	Yushan_Assert_Reset(0x001F0F10, DERESET_MODULE);
	bSpiData = 0;
	Yushan_DXO_Sync_Reset_Dereset(bSpiData);
	Yushan_Init_Dxo(&sDxoStruct, 1);
	msleep(10);
	/*Yushan_sensor_open_init();*/
	/*Yushan_ContextUpdate_Wrapper(&sYushanFullContextConfig);*/
	/*sensor_streaming_on();*/
}

uint8_t Yushan_parse_interrupt(void)
{

	uint8_t		bCurrentInterruptID = 0;
	uint8_t		bAssertOrDeassert = 0, bInterruptWord = 0;
	/* The List consists of the Current asserted events. Have to be assignmed memory by malloc.
	*  and set every element to 0. */
	uint32_t	*udwListOfInterrupts;
	uint8_t	bSpiData;
	uint32_t udwSpiBaseIndex;
	uint8_t interrupt_type = 0;


	udwListOfInterrupts	= kmalloc(96, GFP_KERNEL);

	/* Call Yushan_Intr_Status_Read  for interrupts and add the same to the udwListOfInterrupts. */
	/* bInterruptID = Yushan_ReadIntr_Status();*/
	Yushan_Intr_Status_Read((uint8_t *)udwListOfInterrupts);

	/* Clear InterruptStatus */
	Yushan_Intr_Status_Clear((uint8_t *)udwListOfInterrupts);

	/* Adding the Current Interrupt asserted to the Proto List */
	while (bCurrentInterruptID < (TOTAL_INTERRUPT_COUNT + 1)) {
		bAssertOrDeassert = ((udwListOfInterrupts[bInterruptWord])>>(bCurrentInterruptID%32))&0x01;

		if (bAssertOrDeassert && interrupt_err_count < 10) {
			CDBG("[CAM] %s:bCurrentInterruptID:%d\n", __func__, bCurrentInterruptID+1);
			switch (bCurrentInterruptID+1) {
			case EVENT_PDP_EOF_EXECCMD :
				CDBG("[CAM] %s:[AF_INT]EVENT_PDP_EOF_EXECCMD\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_PDP_EOF_EXECCMD;
				break;

			case EVENT_DPP_EOF_EXECCMD :
				CDBG("[CAM] %s:[AF_INT]EVENT_DPP_EOF_EXECCMD\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_DPP_EOF_EXECCMD;
				break;

			case EVENT_DOP7_EOF_EXECCMD :
				CDBG("[CAM] %s:[AF_INT]EVENT_DOP7_EOF_EXECCMD\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_DOP_EOF_EXECCMD;
				break;

			case EVENT_DXODOP7_NEWFRAMEPROC_ACK :
				CDBG("[CAM] %s:[AF_INT]EVENT_DXODOP7_NEWFRAMEPROC_ACK\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_NEW_FRAME;
				break;

			case EVENT_CSI2RX_ECC_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_CSI2RX_ECC_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_CSI2RX_CHKSUM_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_CSI2RX_CHKSUM_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_CSI2RX_SYNCPULSE_MISSED :
				pr_err("[CAM] %s:[ERR]EVENT_CSI2RX_SYNCPULSE_MISSED\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_DXOPDP_NEWFRAME_ERR :
				SPI_Read(DXO_PDP_BASE_ADDR + DxOPDP_error_code_7_0, 1, &bSpiData);
				pr_err("[CAM] %s:[ERR]EVENT_DXOPDP_NEWFRAME_ERR, error code =%d\n", __func__, bSpiData);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_DXODPP_NEWFRAME_ERR :
				udwSpiBaseIndex = 0x010000;
				SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

				SPI_Read(DXO_DPP_BASE_ADDR + DxODPP_error_code_7_0-0x8000, 1, &bSpiData);
				pr_err("[CAM] %s:[ERR]EVENT_DXODPP_NEWFRAME_ERR, error code =%d\n", __func__, bSpiData);

				udwSpiBaseIndex = 0x08000;
				SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_DXODOP7_NEWFRAME_ERR :
				SPI_Read(DXO_DOP_BASE_ADDR + DxODOP_error_code_7_0, 1, &bSpiData);
				pr_err("[CAM] %s:[ERR]EVENT_DXODOP7_NEWFRAME_ERR, error code =%d\n", __func__, bSpiData);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_CSI2TX_SP_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_CSI2TX_SP_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR_FATAL;
				break;

			case EVENT_CSI2TX_LP_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_CSI2TX_LP_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_CSI2TX_DATAINDEX_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_CSI2TX_DATAINDEX_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_SOFT_DL1 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL1\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_HARD_DL1 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL1\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_EOT_DL1 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL1\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_ESC_DL1 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL1\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_CTRL_DL1 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL1\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_SOFT_DL2 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL2\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_HARD_DL2 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL2\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_EOT_DL2 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL2\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_ESC_DL2 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL2\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_CTRL_DL2 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL2\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_SOFT_DL3 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL3\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_HARD_DL3 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL3\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_EOT_DL3 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL3\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_ESC_DL3 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL3\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_CTRL_DL3:
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL3\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_SOFT_DL4 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL4\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_SOT_HARD_DL4 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL4\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_EOT_DL4:
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL4\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_ESC_DL4:
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL4\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_PHY_ERR_CTRL_DL4 :
				pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL4\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TXPHY_CTRL_ERR_D1 :
				pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D1\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TXPHY_CTRL_ERR_D2 :
				pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D2\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TXPHY_CTRL_ERR_D3 :
				pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D3\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TXPHY_CTRL_ERR_D4 :
				pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D4\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_UNMATCHED_IMAGE_SIZE_ERROR :
				pr_err("[CAM] %s:[ERR]EVENT_UNMATCHED_IMAGE_SIZE_ERROR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case PRE_DXO_WRAPPER_PROTOCOL_ERR :
				pr_err("[CAM] %s:[ERR]PRE_DXO_WRAPPER_PROTOCOL_ERR\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case PRE_DXO_WRAPPER_FIFO_OVERFLOW :
				pr_err("[CAM] %s:[ERR]PRE_DXO_WRAPPER_FIFO_OVERFLOW\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_BAD_FRAME_DETECTION :
				pr_err("[CAM] %s:[ERR]EVENT_BAD_FRAME_DETECTION\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TX_DATA_FIFO_OVERFLOW :
				pr_err("[CAM] %s:[ERR]EVENT_TX_DATA_FIFO_OVERFLOW\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TX_INDEX_FIFO_OVERFLOW :
				pr_err("[CAM] %s:[ERR]EVENT_TX_INDEX_FIFO_OVERFLOW\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_0_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_0_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_1_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_1_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_2_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_2_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_3_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_3_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_4_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_4_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_5_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_5_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_6_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_6_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_RX_CHAR_COLOR_BAR_7_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_7_ERR\n", __func__);
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_POST_DXO_WRAPPER_PROTOCOL_ERR :
				pr_err("[CAM] %s:[ERR]EVENT_POST_DXO_WRAPPER_PROTOCOL_ERR\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_POST_DXO_WRAPPER_FIFO_OVERFLOW :
				pr_err("[CAM] %s:[ERR]EVENT_POST_DXO_WRAPPER_FIFO_OVERFLOW\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TX_DATA_UNDERFLOW :
				pr_err("[CAM] %s:[ERR]EVENT_TX_DATA_UNDERFLOW\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			case EVENT_TX_INDEX_UNDERFLOW :
				pr_err("[CAM] %s:[ERR]EVENT_TX_INDEX_UNDERFLOW\n", __func__);
				/* Reset_Yushan();*/
				interrupt_type |= RAWCHIP_INT_TYPE_ERROR;
				break;

			}
		}
		bCurrentInterruptID++;

		if (bCurrentInterruptID%32 == 0)
			bInterruptWord++;
	}

	kfree(udwListOfInterrupts);

	if (gpio_get_value(rawchip_intr0) == 1) {
		atomic_set(&interrupt, 1);
		wake_up(&yushan_int.yushan_wait);
	}
	if (gpio_get_value(rawchip_intr1) == 1) {
		atomic_set(&interrupt2, 1);
		wake_up(&yushan_int.yushan_wait);
	}

	if (interrupt_type & RAWCHIP_INT_TYPE_ERROR || interrupt_type & RAWCHIP_INT_TYPE_ERROR_FATAL)	Yushan_Status_Snapshot();

	if ((interrupt_type & RAWCHIP_INT_TYPE_ERROR) && interrupt_err_count <= 10) {
		interrupt_err_count++;
	}

	return interrupt_type;
}

/******************************************************************************
API Internal Function:	Interrupt Service Routine: Look for Interrupts asserted.
						Add the ID to the list only once. Usually ISR provoked
						by any associated Intr, but this function has to be called
						whenever interrupts are expected.
Input:					None
Return:					InterruptID or FAILURE
*******************************************************************************/
void Yushan_ISR()
{

	uint8_t		bCurrentInterruptID = 0;
	uint8_t		bAssertOrDeassert = 0, bInterruptWord = 0;
	/* The List consists of the Current asserted events. Have to be assignmed memory by malloc.
	*  and set every element to 0. */
	uint32_t	*udwListOfInterrupts;
	uint8_t	bSpiData;
	uint32_t udwSpiBaseIndex;

	udwListOfInterrupts	= (uint32_t *) kmalloc(96, GFP_KERNEL);

	/* Call Yushan_Intr_Status_Read  for interrupts and add the same to the udwListOfInterrupts. */
	/* bInterruptID = Yushan_ReadIntr_Status();*/
	Yushan_Intr_Status_Read((uint8_t *)udwListOfInterrupts);

	/* Clear InterruptStatus */
	Yushan_Intr_Status_Clear((uint8_t *) udwListOfInterrupts);

	/* Adding the Current Interrupt asserted to the Proto List */
	while (bCurrentInterruptID < (TOTAL_INTERRUPT_COUNT + 1)) {
		bAssertOrDeassert = ((udwListOfInterrupts[bInterruptWord])>>(bCurrentInterruptID%32))&0x01;

		if (bAssertOrDeassert) {
			Yushan_AddnRemoveIDInList((uint8_t)(bCurrentInterruptID+1), udwProtoInterruptList, ADD_INTR_TO_LIST);

			CDBG("[CAM] %s:bCurrentInterruptID:%d\n",__func__, bCurrentInterruptID+1);
			switch (bCurrentInterruptID + 1) {
				case EVENT_PDP_EOF_EXECCMD :
					CDBG("[CAM] %s:[AF_INT]EVENT_PDP_EOF_EXECCMD\n", __func__);
					break;

				case EVENT_DPP_EOF_EXECCMD :
					CDBG("[CAM] %s:[AF_INT]EVENT_DPP_EOF_EXECCMD\n", __func__);
					break;

				case EVENT_DOP7_EOF_EXECCMD :
					CDBG("[CAM] %s:[AF_INT]EVENT_DOP7_EOF_EXECCMD\n", __func__);
					break;

				case EVENT_DXODOP7_NEWFRAMEPROC_ACK :
					CDBG("[CAM] %s:[AF_INT]EVENT_DXODOP7_NEWFRAMEPROC_ACK\n", __func__);
					break;
				case EVENT_CSI2RX_ECC_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_CSI2RX_ECC_ERR\n", __func__);
					break;
				case EVENT_CSI2RX_CHKSUM_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_CSI2RX_CHKSUM_ERR\n", __func__);
					break;
				case EVENT_CSI2RX_SYNCPULSE_MISSED :
					pr_err("[CAM] %s:[ERR]EVENT_CSI2RX_SYNCPULSE_MISSED\n", __func__);
					break;
				case EVENT_DXOPDP_NEWFRAME_ERR :
				{
					SPI_Read(DXO_PDP_BASE_ADDR + DxOPDP_error_code_7_0, 1, &bSpiData);
					pr_err("[CAM] %s:[ERR]EVENT_DXOPDP_NEWFRAME_ERR, error code =%d\n", __func__, bSpiData);
					/* Reset_Yushan(); */
					break;
				}
				case EVENT_DXODPP_NEWFRAME_ERR :
				{
					udwSpiBaseIndex = 0x010000;
					SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

					SPI_Read(DXO_DPP_BASE_ADDR + DxODPP_error_code_7_0-0x8000, 1, &bSpiData);
					pr_err("[CAM] %s:[ERR]EVENT_DXODPP_NEWFRAME_ERR, error code =%d\n", __func__, bSpiData);

					udwSpiBaseIndex = 0x08000;
					SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));
					/* Reset_Yushan();*/
					break;
				}
				case EVENT_DXODOP7_NEWFRAME_ERR :
				{
					SPI_Read(DXO_DOP_BASE_ADDR + DxODOP_error_code_7_0, 1, &bSpiData);
					pr_err("[CAM] %s:[ERR]EVENT_DXODOP7_NEWFRAME_ERR, error code =%d\n", __func__, bSpiData);
					/* Reset_Yushan(); */
					break;
				}
				case EVENT_CSI2TX_SP_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_CSI2TX_SP_ERR\n", __func__);
					break;
				case EVENT_CSI2TX_LP_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_CSI2TX_LP_ERR\n", __func__);
					break;
				case EVENT_CSI2TX_DATAINDEX_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_CSI2TX_DATAINDEX_ERR\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_SOFT_DL1 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL1\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_HARD_DL1 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL1\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_EOT_DL1 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL1\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_ESC_DL1 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL1\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_CTRL_DL1 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL1\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_SOFT_DL2 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL2\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_HARD_DL2 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL2\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_EOT_DL2 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL2\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_ESC_DL2 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL2\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_CTRL_DL2 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL2\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_SOFT_DL3 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL3\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_HARD_DL3 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL3\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_EOT_DL3 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL3\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_ESC_DL3 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL3\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_CTRL_DL3:
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL3\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_SOFT_DL4 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_SOFT_DL4\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_SOT_HARD_DL4 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_SOT_HARD_DL4\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_EOT_DL4:
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_EOT_DL4\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_ESC_DL4:
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_ESC_DL4\n", __func__);
					break;
				case EVENT_RX_PHY_ERR_CTRL_DL4 :
					pr_err("[CAM] %s:[ERR]EVENT_RX_PHY_ERR_CTRL_DL4\n", __func__);
					break;
				case EVENT_TXPHY_CTRL_ERR_D1 :
					pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D1\n", __func__);
					break;
				case EVENT_TXPHY_CTRL_ERR_D2 :
					pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D2\n", __func__);
					break;
				case EVENT_TXPHY_CTRL_ERR_D3 :
					pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D3\n", __func__);
					break;
				case EVENT_TXPHY_CTRL_ERR_D4 :
					pr_err("[CAM] %s:[ERR]EVENT_TXPHY_CTRL_ERR_D4\n", __func__);
					break;
				case EVENT_UNMATCHED_IMAGE_SIZE_ERROR :
					pr_err("[CAM] %s:[ERR]EVENT_UNMATCHED_IMAGE_SIZE_ERROR\n", __func__);
					break;
				case PRE_DXO_WRAPPER_PROTOCOL_ERR :
					pr_err("[CAM] %s:[ERR]PRE_DXO_WRAPPER_PROTOCOL_ERR\n", __func__);
					/* Reset_Yushan(); */
					break;
				case PRE_DXO_WRAPPER_FIFO_OVERFLOW :
					pr_err("[CAM] %s:[ERR]PRE_DXO_WRAPPER_FIFO_OVERFLOW\n", __func__);
					/* Reset_Yushan(); */
					break;
				case EVENT_BAD_FRAME_DETECTION :
					pr_err("[CAM] %s:[ERR]EVENT_BAD_FRAME_DETECTION\n", __func__);
					break;
				case EVENT_TX_DATA_FIFO_OVERFLOW :
					pr_err("[CAM] %s:[ERR]EVENT_TX_DATA_FIFO_OVERFLOW\n", __func__);
					/* Reset_Yushan(); */
					break;
				case EVENT_TX_INDEX_FIFO_OVERFLOW :
					pr_err("[CAM] %s:[ERR]EVENT_TX_INDEX_FIFO_OVERFLOW\n", __func__);
					/* Reset_Yushan(); */
					break;
				case EVENT_RX_CHAR_COLOR_BAR_0_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_0_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_1_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_1_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_2_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_2_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_3_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_3_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_4_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_4_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_5_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_5_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_6_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_6_ERR\n", __func__);
					break;
				case EVENT_RX_CHAR_COLOR_BAR_7_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_RX_CHAR_COLOR_BAR_7_ERR\n", __func__);
					break;
				case EVENT_POST_DXO_WRAPPER_PROTOCOL_ERR :
					pr_err("[CAM] %s:[ERR]EVENT_POST_DXO_WRAPPER_PROTOCOL_ERR\n",__func__);
					/* Reset_Yushan(); */
					break;
				case EVENT_POST_DXO_WRAPPER_FIFO_OVERFLOW :
					pr_err("[CAM] %s:[ERR]EVENT_POST_DXO_WRAPPER_FIFO_OVERFLOW\n", __func__);
					/* Reset_Yushan(); */
					break;
				case EVENT_TX_DATA_UNDERFLOW :
					pr_err("[CAM] %s:[ERR]EVENT_TX_DATA_UNDERFLOW\n", __func__);
					/* Reset_Yushan(); */
					break;
				case EVENT_TX_INDEX_UNDERFLOW :
					pr_err("[CAM] %s:[ERR]EVENT_TX_INDEX_UNDERFLOW\n", __func__);
					/* Reset_Yushan(); */
					break;
			}
		}
		bCurrentInterruptID++;

		if (bCurrentInterruptID%32 == 0)
			bInterruptWord++;
	}

	kfree(udwListOfInterrupts);

	if (gpio_get_value(rawchip_intr0) == 1) {
		atomic_set(&interrupt, 1);
		wake_up(&yushan_int.yushan_wait);
	}
	if (gpio_get_value(rawchip_intr1) == 1) {
		atomic_set(&interrupt2, 1);
		wake_up(&yushan_int.yushan_wait);
	}
}

void Yushan_ISR2() /* for DOP7 */
{

	uint8_t		bCurrentInterruptID = 16;
	uint8_t		bAssertOrDeassert = 0, bInterruptWord = 0;
	/* The List consists of the Current asserted events. Have to be assignmed memory by malloc.
	*  and set every element to 0. */
	uint32_t	*udwListOfInterrupts;
	uint8_t	bSpiData;

	udwListOfInterrupts	= (uint32_t *) kmalloc(96, GFP_KERNEL);

	/* Call Yushan_Intr_Status_Read  for interrupts and add the same to the udwListOfInterrupts. */
	/* bInterruptID = Yushan_ReadIntr_Status();*/
	Yushan_Intr_Status_Read((uint8_t *)udwListOfInterrupts);

#if 1

	/* Adding the Current Interrupt asserted to the Proto List */
	while(bCurrentInterruptID<21)  
	{
		bAssertOrDeassert = ((udwListOfInterrupts[bInterruptWord])>>(bCurrentInterruptID%32))&0x01;

		if(bAssertOrDeassert)
		{				
			Yushan_AddnRemoveIDInList((uint8_t)(bCurrentInterruptID+1), udwProtoInterruptList, ADD_INTR_TO_LIST);
#if 1
			pr_info("[CAM] %s:bCurrentInterruptID:%d\n", __func__, bCurrentInterruptID+1);
			switch(bCurrentInterruptID+1)
			{
			case EVENT_DXODOP7_NEWFRAMEPROC_ACK :
				pr_info("[CAM] %s:[AF_INT]EVENT_DXODOP7_NEWFRAMEPROC_ACK\n", __func__);
				break;
			case EVENT_DXODOP7_NEWFRAME_ERR :
				{
					SPI_Read(DXO_DOP_BASE_ADDR + DxODOP_error_code_7_0, 1, &bSpiData);
						pr_err("[CAM] %s:[ERR]EVENT_DXODOP7_NEWFRAME_ERR, error code =%d\n",__func__, bSpiData);
					/* Reset_Yushan(); */
					break;
				}
			}
#endif
		}
		bCurrentInterruptID++;

		if (bCurrentInterruptID%32 == 0)
			bInterruptWord++;
	}
	/* Clear InterruptStatus */
	Yushan_Intr_Status_Clear((uint8_t *) udwListOfInterrupts);
#endif

	kfree(udwListOfInterrupts);
}


/*******************************************************************
CheckForInterruptIDInList:
Check InterruptID in the Interrupt list.
*******************************************************************/
bool_t Yushan_CheckForInterruptIDInList(uint8_t bInterruptID) /*, uint32_t *udwListOfInterrupts) */
{

	bool_t		fStatus = 0;
	uint8_t		bIntrDWordInList, bIndexInCurrentDWord;
	uint32_t	*pListOfInterrupt;


	pListOfInterrupt =  &udwProtoInterruptList[0];


	if ((1 <= bInterruptID) && (bInterruptID <= 32))
		bIntrDWordInList = 0;
	else if ((33 <= bInterruptID) && (bInterruptID <= 64))
		bIntrDWordInList = 1;
	else
		bIntrDWordInList = 2;

	/* Shift pointer by (bIntrDWordInList * 32) elements. */
	pListOfInterrupt = pListOfInterrupt + bIntrDWordInList;

	/* Carry those 32 bit, among which Interrupt in interest exists.
	*  Mnus one as the list is consistence with the bit mask, used, for enabling
	*  the interrupts */
	bIndexInCurrentDWord = (bInterruptID - (bIntrDWordInList * 32)) - 1;


	/* Check whether the desired Interrupt has been aserted or not. */
	fStatus = (bool_t)(((*pListOfInterrupt)>>bIndexInCurrentDWord) & 0x00000001);

	return fStatus;

}

/******************************************************************
API Internal Function:	Yushan_ReadIntr_Status
Input:					InterruptList
Return:					ID of the Interrupt asserted
*******************************************************************/
void Yushan_Intr_Status_Read(uint8_t *bListOfInterrupts)
{
	uint16_t	uwSpiData = 0;
	uint8_t		bIsIntrSetAsserted, bIntrSetID = 0;/* bIntrEventID = FALSE_ALARM;*/
	bool_t		fStatus = SUCCESS;
	uint8_t		bSpiData = 0;

	/* Clearing the interrupt list */
	memset((void *)bListOfInterrupts, 0, 96);

	/* Read NVM_INTR_STATUS: The register masking all the system interrupts
	*  and reading the same will give the idea of the actual set where interrupt
	*  has been asserted. No auto increment reads in case Pll not locked. */
	if (gPllLocked) {
		fStatus &= SPI_Read(YUSHAN_IOR_NVM_INTR_STATUS, 2, (uint8_t *)(&uwSpiData));
		/* pr_info("[CAM] INT Status: 0x%x\n", uwSpiData);*/
	} else {
		fStatus &= SPI_Read(YUSHAN_IOR_NVM_INTR_STATUS, 1, (uint8_t *)(&bSpiData));
		uwSpiData = (uint16_t) bSpiData;
		fStatus &= SPI_Read(YUSHAN_IOR_NVM_INTR_STATUS + 1, 1, (uint8_t *)(&bSpiData));
		uwSpiData |= (((uint16_t) bSpiData) << 8);
	}

	/*  Check now the sets where interrupt bits have been set or asserted.
	*  Note: More than one interrupt can be asserted at one point of time and may
	*  belong to different interrupt sets. */
	while (bIntrSetID < TOTAL_INTERRUPT_SETS) {
		/* Checking which (first) interrupt set has trigerred the interrupt. */
		bIsIntrSetAsserted = ((uwSpiData>>bIntrSetID) & 0x01);

		if (bIsIntrSetAsserted == 0x01) {
			/* Checking all the events which have triggered the interrupts and add those
			*  interrupts in the ListOfInterrupt. */
			Yushan_Read_IntrEvent(bIntrSetID, (uint32_t *)bListOfInterrupts);

		}
		/* Move to next SetID */
		bIntrSetID++;
	}

}

/******************************************************************
API Internal Function:	Function for reading exact interrupt event.
Input:					Asserted IntrSetID.
Return:					InterruptEventID as define in Yushan_API.h
*******************************************************************/
void Yushan_Read_IntrEvent(uint8_t bIntrSetID, uint32_t *udwListOfInterrupts)
{

	uint8_t		bInterruptID, bTotalEventInASet, bIntrCountInSet = 0, bTempIntrStatus = 0;
	uint8_t		bFirstIndexForSet[] = {1, 5, 11, 17, 23, 27, 58, 62, 69, 77, 79, 82, 83};
	uint16_t	uwIntrSetOffset;
	uint32_t	udwIntrStatus, udwIntrEnableStatus, udwCombinedStatus;
	/* bool_t		fStatus = SUCCESS;, fIntrInListAlready = FAILURE;*/

	if (bIntrSetID != 13)
		bTotalEventInASet = (bFirstIndexForSet[bIntrSetID + 1] - bFirstIndexForSet[bIntrSetID]);
	else
		bTotalEventInASet = 2;/* Only two events in the last IntrSet */

	/* Move offset to points to the current set. */
	uwIntrSetOffset = (bIntrSetID * INTERRUPT_SET_SIZE);

	/* First read Intr_Status and Enable status of the asserted set.
	*  NOTE: NVM_INTR_STATUS register will set its bits only when events (1 or more), of a particular set, have been enabled
	*  and interrupts to those events, of the same set have been asserted. So it is must for us to check both of the Interrupt
	*  Enable Status and Interrupt Status regsiters. Auto Increment only after Pll locked. */

	if (gPllLocked) {	/* AutoIncrement if Pll locked */
		/* IntrStatus */
		SPI_Read((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset), 4, (uint8_t *)(&udwIntrStatus));
		/* IntrEnableStatus */
		SPI_Read((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE_STATUS), 4, (uint8_t *)(&udwIntrEnableStatus));
	} else {
		SPI_Read((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset),   1, (uint8_t *)(&bTempIntrStatus));
		udwIntrStatus = (uint32_t)(bTempIntrStatus);
		SPI_Read((uint16_t)((YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset)+1), 1, (uint8_t *)(&bTempIntrStatus));
		udwIntrStatus |= ((uint32_t)(bTempIntrStatus)) << 8;
		SPI_Read((uint16_t)((YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset)+2), 1, (uint8_t *)(&bTempIntrStatus));
		udwIntrStatus |= ((uint32_t)(bTempIntrStatus)) << 16;
		SPI_Read((uint16_t)((YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset)+3), 1, (uint8_t *)(&bTempIntrStatus));
		udwIntrStatus |= ((uint32_t)(bTempIntrStatus)) << 24;

		/* IntrEnableStatus */
		SPI_Read((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE_STATUS),    1, (uint8_t *)(&bTempIntrStatus));
		udwIntrEnableStatus = (uint32_t)(bTempIntrStatus);
		SPI_Read((uint16_t)((YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE_STATUS) +1), 1, (uint8_t *)(&bTempIntrStatus));
		udwIntrEnableStatus |= ((uint32_t)(bTempIntrStatus)) << 8;
		SPI_Read((uint16_t)((YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE_STATUS) +2), 1, (uint8_t *)(&bTempIntrStatus));
		udwIntrEnableStatus |= ((uint32_t)(bTempIntrStatus)) << 16;
		SPI_Read((uint16_t)((YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_ENABLE_STATUS) +3), 1, (uint8_t *)(&bTempIntrStatus));
		udwIntrEnableStatus |= ((uint32_t)(bTempIntrStatus)) << 24;
	}

	CDBG("[CAM] udwIntrStatus:  0x%x, udwIntrEnableStatus: 0x%x\n", udwIntrStatus, udwIntrEnableStatus);

	/* Scan in the set for the asserted event. */
	while (bIntrCountInSet < bTotalEventInASet) {
		/* Check if the Interrupt has been enabled and asserted. Then return the first InterruptEventID (being defined in
		*  Yushan_API.h) for which both the signals are high.
		*  Note: There can be more than one event asserted in a set, but return only the first one and clear its respective
		*  status before returning. */
		udwCombinedStatus = (udwIntrStatus & udwIntrEnableStatus);
		/* if ( ((udwIntrStatus>>bIntrCountInSet) & 0x01) && ((udwIntrEnableStatus>>bIntrCountInSet) & 0x01) ) */
		if ((udwCombinedStatus>>bIntrCountInSet) & 0x01) {
			/* Entered since both the IntrStatus and EnableStatus are true for the event, at bit, indexed at bIntrCountInSet.
			*  Add IntrID to the ListOfInterrupts. */
			bInterruptID = (bFirstIndexForSet[bIntrSetID] + bIntrCountInSet);

			Yushan_AddnRemoveIDInList(bInterruptID, udwListOfInterrupts, ADD_INTR_TO_LIST);
		}

		bIntrCountInSet++;
	}
}

/**********************************************************************
Generic function:	To add or remove ( set or clear  bit corresponding
					to) interrupt mentioned in input.
Input:				Interrupt ID to be added or removed from the list
					and a Flag to decide to add or remove.
Return:
***********************************************************************/
void Yushan_AddnRemoveIDInList(uint8_t bInterruptID, uint32_t *udwListOfInterrupts, bool_t fAddORClear)
{

	uint8_t		bIntrDWordInList, bIndexInCurrentDWord;
	uint32_t	*pListOfInterrupt, udwTempIntrList, udwMask = 0x00000001;

	/* Point to 96bit (global) IntrList of which only first 83bits are valid for 83 interrupts. */
	pListOfInterrupt = udwListOfInterrupts;

	/* Scan the list for the received ID and add the same to the list, if requested.
	*  The list will be a 96 bit (3 element array of double word) of which
	*  83bits will be valid. Bit will carry 1, if corresponding interrupt has been aserted else will
	*  be zero. */
	if ((1 <= bInterruptID) && (bInterruptID <= 32))
		bIntrDWordInList = 0;
	else if ((33 <= bInterruptID) && (bInterruptID <= 64))
		bIntrDWordInList = 1;
	else
		bIntrDWordInList = 2;

	/* Shift pointer by (bIntrDWordInList * 32) elements. */
	pListOfInterrupt = pListOfInterrupt + bIntrDWordInList;

	/* Minus one makes the list in line with the Bit mask required, input to Intr_Enable function */
	bIndexInCurrentDWord = (bInterruptID - (bIntrDWordInList * 32)) - 1;

	/* Intermediate List */
	/* udwTempIntrList = ( ((*pListOfInterrupt)>>(bIndexInCurrentDWord)) | udwMask);// udwMask = 0x00000001 */

	/* Modifying the list. */
	if (fAddORClear == ADD_INTR_TO_LIST) {
		/* For adding intr in list */
		udwTempIntrList = (((*pListOfInterrupt)>>(bIndexInCurrentDWord)) | udwMask);/* udwMask = 0x00000001 */
		*pListOfInterrupt |= (udwTempIntrList << (bIndexInCurrentDWord));
	} else if (fAddORClear == DEL_INTR_FROM_LIST) {
		/* For Clearing intr from the list */
		udwTempIntrList = (((*pListOfInterrupt)>>(bIndexInCurrentDWord)) & udwMask);/* udwMask = 0x00000001 */
		*pListOfInterrupt &= ~(udwTempIntrList << (bIndexInCurrentDWord));
	}

}

/********************************************************************
API Function:	Clear Status of Interrupts asserted or occured
Input:			Pointer to 83bit udwListOfInterrupts, which carry 1 or
				0 for the interrupts to be cleared.
Return:			Success or Failure
*********************************************************************/
bool_t Yushan_Intr_Status_Clear(uint8_t *bListOfInterrupts)
{
	/* Two counters, bIntrCount to trace through Interrupt bits 1 through 83 and another counter
	*  bIntr32Count to trace through 32 to 0 for checking interrupts in 32 bit double word. */
	uint8_t		bTotalEventInASet;/*, *pSpiData;*/
	uint8_t		bIntrCount = 1, bIntr32Count = 1, bIntrCountInSet = 0, bIntrSetID = 1;
	uint16_t	uwIntrSetOffset;
	uint32_t	udwLocalIntrMask, udwInterruptSetting, *pLocalIntrMask, udwSpiData = 0;
	uint8_t bSpiData;

	/* First element 0 does not represents any set. */
	uint8_t		bFirstIndexForSet[] = {0, 1, 5, 11, 17, 23, 27, 58, 62, 69, 77, 79, 82, 83};

	/* pr_info("[CAM] bListOfInterrupts: 0x%x\n", *bListOfInterrupts);*/
	pLocalIntrMask = (uint32_t *) bListOfInterrupts;
	udwLocalIntrMask	= *pLocalIntrMask;

	/* For the first set, TotalElement = (5-1);*/
	bTotalEventInASet = (bFirstIndexForSet[2] - bFirstIndexForSet[1]);

	/* Trace through IntrMask on bit basis. Plus one to tackle writes of the last set. */
	while (bIntrCount <= (TOTAL_INTERRUPT_COUNT + 1)) {

		/* If counter/index reaches the number of interrupts in the set, then write the packet to the
		*  interrupt registers. The time when these two registers are equal, double word packet has been
		*  already created considering all the bits of that set and index has now been pointing to one
		*  location more than the number of interrupts/events. */
		if (bIntrCountInSet == bTotalEventInASet) {
			/* InterruptSet offset from the Interrupt base address. Minus one due to the fact that
			*  InterSetID starts with 1 not 0. */
			uwIntrSetOffset = ((bIntrSetID - 1) * INTERRUPT_SET_SIZE);
			/* Write udwSpiData to the Interrupt Enable regsiter. Since clks are not dirven by Pll, no auto
			*  incremental writes.
			*  Auto increments writes possible after Pll locked */
			/* pr_info("[CAM] ClearINTStatus: 0x%x\n", udwSpiData);*/
			/* udwSpiData = 0xFFFFFFFF;*/
			SPI_Write((uint16_t)(YUSHAN_INTR_BASE_ADDR + uwIntrSetOffset + YUSHAN_OFFSET_INTR_STATUS_CLEAR), 4, (uint8_t *)(&udwSpiData));

			/* Reset the counter tracking the interrupts in the current set. Also reset the data. */
			bIntrCountInSet = 0;
			udwSpiData = 0;

			/* As Intr32Count and IntrCount has been incremented and not used while doing SPI writes. */
			bIntr32Count--;
			bIntrCount--;
			/* Increase the SetID too. */
			bIntrSetID++;

			/* Set the ElementInASet. */
			if (bIntrSetID < 13)
				bTotalEventInASet = (bFirstIndexForSet[bIntrSetID + 1] - bFirstIndexForSet[bIntrSetID]);
			else if (bIntrSetID == 13)
				bTotalEventInASet = 2;/* Only two events in the last IntrSet */

		} else {
			/* udwInterruptSetting gives the per bit setting, 1 or 0. */
			udwInterruptSetting = ((udwLocalIntrMask>>(bIntr32Count-1)) & 0x01);
			/* Create a double word */
			udwSpiData |= (udwInterruptSetting<<(bIntrCountInSet));

			/* track to next interrupt bit. */
			bIntrCountInSet++;
		}

		/* If double word has been exhausted, look for the next four bytes. */
		if (!(bIntr32Count % 32)) {
			/* Reset the 32Count counter to 1, so as to shift udwLocalIntrMask by not more than 32 bits(max). */
			bIntr32Count = 1;

			/* After every 32bits, take next double word from the *pLocalIntrMask. */
			pLocalIntrMask++;
			udwLocalIntrMask	= *pLocalIntrMask;
		} else
			bIntr32Count++;

		/* Increase the top counter. */
		bIntrCount++;

	}
	/* disable extra INT */
	bSpiData = 0;
	SPI_Write(YUSHAN_SMIA_FM_EOF_INT_EN, 1,  &bSpiData);

	return SUCCESS;
}


/*********************************************************************
				Interrupt Functions Ends Here
**********************************************************************/



/*********************************************************************
				Optional Functions Starts Here
**********************************************************************/

/*************************************************************
API Functtion: Update the CPX and DCPX IPs of Yushan in case of
RAW 10 to 8 pixel format
Input: void
**************************************************************/
void	Yushan_DCPX_CPX_Enable(void)
{

	uint8_t	bSpiData = 0;

	/* DCPX */
	bSpiData = 0x01;
	SPI_Write(YUSHAN_SMIA_DCPX_ENABLE, 1, &bSpiData);/* DCPX - DCPX_ENABLE */
	bSpiData = 0x08;
	SPI_Write(YUSHAN_SMIA_DCPX_MODE_REQ, 1, &bSpiData);/* DCPX - DCPX_MODE_REQ */
	bSpiData = 0x0A;
	SPI_Write(YUSHAN_SMIA_DCPX_MODE_REQ + 1, 1, &bSpiData);/* DCPX - DCPX_MODE_REQ */

	/* CPX */
	bSpiData = 0x01;
	SPI_Write(YUSHAN_SMIA_CPX_CTRL_REQ, 1, &bSpiData);/* CPX - CPX_CTRL_REQ */
	/* bSpiData = 0x01 */
	SPI_Write(YUSHAN_SMIA_CPX_MODE_REQ, 1, &bSpiData);/* CPX - CPX_MODE_REQ */

}

/*********************************************************************
API Function:	This function will update the context configuration.
Input:			Function will ask for inputs, i.e., New resolution, format
				and mode, i.e., preview or capture.

Return:			Success or Failure
**********************************************************************/
bool_t	Yushan_Context_Config_Update(Yushan_New_Context_Config_t	*sYushanNewContextConfig)
{
	/* bool_t		bStatus = sUCCESS;*/
	uint8_t		bVfStillIndex, bVFIndex, bStillIndex, bVFMask = 0;
	uint8_t		bDataType, bCurrentDataType = 0, bActiveDatatype = 1, bRawFormat = 0, bWCAlreadyPresent = 0;
	uint8_t		bPixClkDiv = 0, bDxoClkDiv = 0, bCount = 0;
	uint16_t	uwNewHSize = 0, uwLecci = 0;
	uint32_t	udwSpiData = 0;


	/* RawFormat */
	/* bRawFormat = (sYushanNewContextConfig->uwPixelFormat& 0x0F);*/
	/* bDataType */
	/* bDataType = (bRawFormat == 0x08) ? 0x2a:0x2b;*/

	if ((sYushanNewContextConfig->uwPixelFormat & 0x0F) == 0x0A) {
		bRawFormat = RAW10;
		bDataType  = 0x2b;
	} else if((sYushanNewContextConfig->uwPixelFormat&0x0F)==0x08) {
		bRawFormat = RAW8;
		if (((sYushanNewContextConfig->uwPixelFormat >> 8) & 0x0F) == 0x08)
			bDataType = 0x2a;
		else /* 10 to 8 case */
			bDataType = 0x30;/* bRawFormat at Yushan should be RAW8, not RAW10_8 */
	}

	/* Read the Index information of the VF or STILL resolution. */
	SPI_Read(YUSHAN_IDP_GEN_CONTROL, 1, (uint8_t *)(&bVfStillIndex));
	bVFIndex = bVfStillIndex & 0x0F;
	bStillIndex = (bVfStillIndex & 0xF0)>>4;

	/* Take new cofiguration */
	uwNewHSize = sYushanNewContextConfig->uwActivePixels;

	/* Find out if the required Wordcount is still present in IDP_Gen_WCIndex matrix. Also looking for its
	*  corresponding mode. In case WC not present, add new WC after the last valid entry */
	for (bCount = 0; bCount < 14; bCount++) {
		SPI_Read((uint16_t)(YUSHAN_IDP_GEN_WC_DI_0+4*bCount),4,(unsigned char*)&udwSpiData);

		if (udwSpiData == 0) /* Not valid Entry. Add new WC here */
			break;

		bCurrentDataType = (udwSpiData>>16);							/* Retaining Datatype */
		udwSpiData = ((udwSpiData & 0xFFFF)*8)/bRawFormat;				/* Resolution/H_Size against WC */

		/* Check if the desired resolution is already present and with the same data type */
		if ((uwNewHSize == udwSpiData) && (bCurrentDataType == bDataType)) {
			bWCAlreadyPresent = 1;
			if ((bCount == bVFIndex) || (bCount == bStillIndex))
				return SUCCESS;			/* No need for reprogramming required */
			else
				break;					/* Only T1_DMA programming required */
		}
	}

	/* Preparing Idp_Gen_Control packet */
	SPI_Read(YUSHAN_IDP_GEN_CONTROL, 4, (uint8_t *)(&udwSpiData));
	if (sYushanNewContextConfig->bSelectStillVfMode == YUSHAN_FRAME_FORMAT_VF_MODE) {
		udwSpiData &= 0xFFFFFFF0;
		udwSpiData |= bCount;
		/* For T1_DMA Programming */
		bVFMask = 1;
	} else if (sYushanNewContextConfig->bSelectStillVfMode == YUSHAN_FRAME_FORMAT_STILL_MODE) {
		udwSpiData &= 0xFFFFFF0F;
		udwSpiData |= bCount << 4;
		/* For T1_DMA Programming */
		bVFMask = 0;
	}

	/* Program the Idp_Gen_Ctrl */
	SPI_Write((uint16_t)(YUSHAN_IDP_GEN_CONTROL), 4, (uint8_t *)(&udwSpiData));

	/* Reprogram the Idp_Gen_Ctrl with new VF/STILL index. */
	if (bWCAlreadyPresent && (bCount != 14)) {
		/* WC already present with Normal mode programmed. */
		SPI_Write((uint16_t)((YUSHAN_IDP_GEN_WC_DI_0+(4*bCount))+3),	1,	(unsigned char*)&bActiveDatatype);
	} else if (bCount != 14) {
		/* New entry to be added at bCount */
		/* Wordcount */
		udwSpiData = (sYushanNewContextConfig->uwActivePixels*bRawFormat)/8;
		SPI_Write((uint16_t)(YUSHAN_CSI2_TX_PACKET_SIZE_0+0xc*bCount),	4,	(unsigned char*)&udwSpiData);

		SPI_Write((uint16_t)(YUSHAN_CSI2_TX_DI_INDEX_CTRL_0+0xc*bCount),	1,	(unsigned char*)&bDataType);

		udwSpiData = udwSpiData | (bDataType << 16) | (bActiveDatatype << 24);
		SPI_Write((uint16_t)(YUSHAN_IDP_GEN_WC_DI_0+4*bCount),	4,	(unsigned char*)&udwSpiData);
	} else {
		/* No entry vacant. Reconfiguration unsuccessful */
		return FAILURE;
	}

	/* T1_DMA Programming. */
	/* For PDP */
	udwSpiData = (uwNewHSize & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT + bVFMask),1,(unsigned char*)&udwSpiData);
	udwSpiData = ((uwNewHSize>>8) & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+8+ bVFMask),1,(unsigned char*)&udwSpiData);

	/* For DPP */
	udwSpiData = (uwNewHSize & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+24+ bVFMask),1,(unsigned char*)&udwSpiData);
	udwSpiData = ((uwNewHSize>>8) & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+32+ bVFMask),1,(unsigned char*)&udwSpiData);

	/* For DOP */
	udwSpiData = (uwNewHSize & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+48+ bVFMask),1,(unsigned char*)&udwSpiData);
	udwSpiData = ((uwNewHSize>>8) & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+56+ bVFMask),1,(unsigned char*)&udwSpiData);

	/* For LBE PreDxo */
	udwSpiData = (uwNewHSize & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+72+ bVFMask),1,(unsigned char*)&udwSpiData);
	udwSpiData = ((uwNewHSize>>8) & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+80+ bVFMask),1,(unsigned char*)&udwSpiData);

	/* For LBE PostDxo */
	udwSpiData = (uwNewHSize & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+88+ bVFMask),1,(unsigned char*)&udwSpiData);
	udwSpiData = ((uwNewHSize>>8) & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+96+ bVFMask),1,(unsigned char*)&udwSpiData);

	/* For Lecci */
	udwSpiData = (uwNewHSize & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+104+ bVFMask),1,(unsigned char*)&udwSpiData);
	udwSpiData = ((uwNewHSize>>8) & 0xFF);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+112+ bVFMask),1,(unsigned char*)&udwSpiData);

	/***** Lecci prog ******/
	/* Min_interline = (Freq_Pix*interline)/Freq_DXO;. */
	SPI_Read(YUSHAN_CLK_DIV_FACTOR, 1, (unsigned char *) &bDxoClkDiv);
	SPI_Read(YUSHAN_CLK_DIV_FACTOR + 1, 1, (unsigned char *) &bPixClkDiv);

	if ((bDxoClkDiv != 0) && (bPixClkDiv != 0))
		uwLecci = (sYushanNewContextConfig->uwLineBlank * bPixClkDiv) / bDxoClkDiv;
	else
		uwLecci = 300;

	/* For Lecci */
	udwSpiData = (uwLecci & 0xff);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+120+bVFMask),1,(unsigned char*)&udwSpiData);

	udwSpiData = ((uwLecci>>8) & 0xff);
	SPI_Write((uint16_t)(YUSHAN_T1_DMA_MEM_LOWER_ELT+128+bVFMask),1,(unsigned char*)&udwSpiData);

	return SUCCESS;

}

/*********************************************************************
API Function:	This function will update the DXO_DOP AF ROIs (Five ROIs).
Input:			Structure for ROI and NumOfActiveROI
Return:			Success or Failure
**********************************************************************/
bool_t Yushan_AF_ROI_Update(Yushan_AF_ROI_t  *sYushanAfRoi, uint8_t bNumOfActiveRoi)
{

	uint8_t		bSpiData[4];
	uint8_t		bStatus = SUCCESS, bCount = 0;

	/* bNumOfActiveRoi = 1;*/
	if (!bNumOfActiveRoi) /* NO ROI ACTIVATED */
		return SUCCESS;
	else {
		/* Program active number of ROI */
		bStatus &= SPI_Write(DXO_DOP_BASE_ADDR + DxODOP_ROI_active_number_7_0, 1, (uint8_t *)(&bNumOfActiveRoi));
	}

	/* pr_info("[CAM] AF_ROI_Update:active number:%d, xstart:%d %d %d %d \n", bNumOfActiveRoi, sYushanAfRoi->bXStart, sYushanAfRoi->bYStart, sYushanAfRoi->bXEnd, sYushanAfRoi->bYEnd);*/

	/* Program the active number of ROIs */
	while (bCount < bNumOfActiveRoi) {
		bSpiData[0] = (sYushanAfRoi->bXStart);
		bSpiData[1] = (sYushanAfRoi->bYStart);
		bSpiData[2] = (sYushanAfRoi->bXEnd);
		bSpiData[3] = (sYushanAfRoi->bYEnd);

		/* SPI transaction */
		/* bCount *8 will take care of the offset from the base adress. '8': Number of bytes in one ROI programming. */
		bStatus &= SPI_Write((uint16_t)(DXO_DOP_BASE_ADDR+DxODOP_ROI_0_x_start_7_0 + bCount*4), 4, (uint8_t*)(&bSpiData[0]));
		/* Count increment */
		bCount++;

		/* Points to the next ROI struct */
		sYushanAfRoi++;
	}

	return bStatus;
}

/******************************************************************
API Function:	The function will put the Yushan in standby mode. It
				will stop the MIPI phy blocks, stop the PLL and LDO
				to reduce the power consumption.
Input:
Return:			Success or Failure
*******************************************************************/
bool_t Yushan_Enter_Standby_Mode(void)
{
	uint8_t		bSpiData;
	uint32_t	udwSpiData;
	bool_t		fStatus = SUCCESS;

	/* Disable the MIPI RX and TX. */
	bSpiData = 0;
	fStatus &= SPI_Write(YUSHAN_MIPI_RX_ENABLE, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_MIPI_TX_ENABLE, 1, (uint8_t *)(&bSpiData));


	/* Switch the Clocks to Host clk and then disable the Pll. */

	/* For glitch free switching, first disable the clocks */
	/* Saving the context/data of the Clk_Ctrl */
	SPI_Read(YUSHAN_CLK_CTRL,   1, (uint8_t *)(&bSpiData));		/* Saving the First */
	SPI_Read(YUSHAN_CLK_CTRL + 2, 2, (uint8_t *)(&udwSpiData));	/* Third and fourth byte. */
	udwSpiData = (bSpiData | (udwSpiData << 16));

	/* Disable the clocks */
	bSpiData = 0x00;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 2, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 3, 1, (uint8_t *)(&bSpiData));

	/* Switch Sys clk to Host clk from Pll clk */
	SPI_Read(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));
	bSpiData &= 0xFE;	/* Clk_Sys_Sel set to 0 */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));

	/* Bypass Pll */
	/* SPI_Read(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));*/	/* No need of read, as the data written is saved in bSpiData */
	bSpiData |= ((bSpiData>>1) | 0x01) << 1;						/* bypass Pll set to 1 */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));

	/* Enable the clock */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 1, (uint8_t *)(&udwSpiData));		/* First byte of the context */
	udwSpiData = (udwSpiData >> 16);										/* Third and fourth byte */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 2, 1, (uint8_t *)(&udwSpiData));
	udwSpiData = (udwSpiData >> 8);
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 3, 1, (uint8_t *)(&udwSpiData));

	/* Now disable the Pll clk */
	SPI_Read(YUSHAN_PLL_CTRL_MAIN, 1, (uint8_t *)(&bSpiData));
	bSpiData |= 0x01;			/* Power down  = 1 */
	fStatus &= SPI_Write(YUSHAN_PLL_CTRL_MAIN, 1, (uint8_t *)(&bSpiData));

	/* Disable the LDO */

	return fStatus;

}

/*********************************************************************
API Function:	The function will bring the Yushan out of standby mode.
				It will enable the MIPI phy blocks,	PLL and LDO.
Input:			sInitStruct for NumberOfLanes information
Return:			Success or Failure
**********************************************************************/
bool_t Yushan_Exit_Standby_Mode(Yushan_Init_Struct_t *sInitStruct)
{
	uint8_t		bSpiData;
	uint32_t	udwSpiData;
	bool_t		fStatus = SUCCESS;


	/* Switch the Clocks to Pll clk */

	/* Now Enable the Pll clk */
	SPI_Read(YUSHAN_PLL_CTRL_MAIN, 1, (uint8_t *)(&bSpiData));
	bSpiData &= 0xFE;	/* Power down  = 0 */
	SPI_Write(YUSHAN_PLL_CTRL_MAIN, 1, (uint8_t *)(&bSpiData));

	fStatus &= Yushan_WaitForInterruptEvent(EVENT_PLL_STABLE, TIME_20MS);

	if (!fStatus)
		return fStatus;


	/* For glitch free switching, first disable the clocks */
	/* Saving the context/data of the Clk_Ctrl */
	SPI_Read(YUSHAN_CLK_CTRL,   1, (uint8_t *)(&bSpiData));		/* Saving the First */
	udwSpiData = (uint32_t)(bSpiData);
	SPI_Read(YUSHAN_CLK_CTRL + 2, 1, (uint8_t *)(&bSpiData));		/* Third byte. */
	udwSpiData |= (((uint32_t)(bSpiData)) << 8);
	SPI_Read(YUSHAN_CLK_CTRL + 3, 1, (uint8_t *)(&bSpiData));		/* Fourth byte. */
	udwSpiData |= (((uint32_t)(bSpiData)) << 16);


	/* Disable the clocks */
	bSpiData = 0x00;
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 2, 1, (uint8_t *)(&bSpiData));
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 3, 1, (uint8_t *)(&bSpiData));

	/* No bypass Pll */
	SPI_Read(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));
	bSpiData &= (bSpiData & 0xFD);						/* bypass Pll set to 0 */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));

	/* Switch Sys clk to Pll clk from Host clk */
	/* SPI_Read(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));*/
	bSpiData |= 0x01;											/* Clk_Sys_Sel set to 1 */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 1, 1, (uint8_t *)(&bSpiData));

	/* Enable the clocks */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL, 1, (uint8_t *)(&udwSpiData));		/* First byte of the context */
	udwSpiData = (udwSpiData >> 16);										/* Third and fourth byte */
	fStatus &= SPI_Write(YUSHAN_CLK_CTRL + 2, 2, (uint8_t *)(&udwSpiData));


	/* Enable the MIPI RX and TX. */
	if (sInitStruct->bNumberOfLanes == 1)
		udwSpiData = 0x11;/* Enabling Clock Lane and Data Lane1 */
	else
		if (sInitStruct->bNumberOfLanes == 2)
			udwSpiData = 0x31;/* Enabling Clock Lane and Data Lane2 */
		else
			if (sInitStruct->bNumberOfLanes == 4)
				udwSpiData = 0xf1;/* Enabling Clock Lane and Data Lane3 */

	fStatus &= SPI_Write(YUSHAN_MIPI_RX_ENABLE, 1, (uint8_t *)(&udwSpiData));
	udwSpiData |= 0x02;/* Enable Clk HS */
	fStatus &= SPI_Write(YUSHAN_MIPI_TX_ENABLE, 1, (uint8_t *)(&udwSpiData));

	return fStatus;

}

/******************************************************************
API Function:	This function will swap the P & N pins at CSI
				Receiver for the lanes which are passed as TRUE.
Input:			Clk_Lane, DataLane1, 2, 3 and 4.
Return:			Success or Failure
*******************************************************************/
bool_t Yushan_Swap_Rx_Pins(bool_t fClkLane, bool_t fDataLane1, bool_t fDataLane2, bool_t fDataLane3, bool_t fDataLane4)
{
	bool_t		fStatus;/* = SUCCESS;*/
	uint8_t		bSpiData = 0;

	bSpiData = (fClkLane | (fDataLane1 << 4) | (fDataLane2 << 5) | (fDataLane3 << 6) | (fDataLane4 << 7));

	pr_info("[CAM] Spi data for Swapping Rx pins is %4.4x\n", bSpiData);

	fStatus = SPI_Write(YUSHAN_MIPI_RX_SWAP_PINS, 1, &bSpiData);

	return fStatus;
}

/******************************************************************
API Function:	This function will invert the HS signals of the CSI
				Receiver for the lanes which are passed	as TRUE. This
				is an optional function which might	be required for
				specific platforms only
Input:			Clk_Lane, DataLane1, 2, 3 and 4.
Return:			Success or Failure
*******************************************************************/
bool_t Yushan_Invert_Rx_Pins(bool_t fClkLane, bool_t fDataLane1, bool_t fDataLane2, bool_t fDataLane3, bool_t fDataLane4)
{

	bool_t		fStatus;/* = SUCCESS;*/
	uint8_t		bSpiData = 0;

	bSpiData = (fClkLane | (fDataLane1 << 4) | (fDataLane2 << 5) | (fDataLane3 << 6) | (fDataLane4 << 7));

	pr_info("[CAM] Spi data for Inverting Rx pins is %4.4x\n", bSpiData);

	fStatus = SPI_Write(YUSHAN_MIPI_RX_INVERT_HS, 1, &bSpiData);


	return fStatus;
}

/******************************************************************
API Function:	This function will reset the modules as per the input.
Input:			bModuleMask, 32bits Bitmask of Modules which are required
				to be reset
Return:			Success or Failure
*******************************************************************/
bool_t Yushan_Assert_Reset(uint32_t udwModuleMask, uint8_t bResetORDeReset)
{
	uint8_t bCurrentModuleToReset, bCurrentModule = 0;
	uint8_t	bSpiData;
	bool_t	bSetORReset;

	if (!(udwModuleMask & 0xFFFFFFFF)) {
		pr_info("[CAM] Give some ModuleID for reset\n");
		return FAILURE;	/*********chk what to return *********/
	}

	while (bCurrentModule < TOTAL_MODULES) {
		bSetORReset = ((udwModuleMask>>bCurrentModule) & 0x01);

		/* Check if the current module has to be reset or not. */
		if (!bSetORReset) {
			bCurrentModule++;
			pr_info("[CAM] Current Module is %d\n", bCurrentModule);
			continue;
		} else {
			bCurrentModuleToReset = bCurrentModule + 1;
			bCurrentModule++;
		}

		/* Reset or Dereset the IPs */
		if (bResetORDeReset)
			/* Reset Data */
			bSpiData = 0x11;
		else
			/* Dereset data */
			bSpiData = 0x01;

		/* Debug */
		pr_info("[CAM] Current module is %d and Module to reset/Dereset is %d\n", bCurrentModule, bCurrentModuleToReset);

		/* Selectively reset the IPs */
		switch (bCurrentModuleToReset) {
#if 0
		case DFT_RESET:
			break;
#endif
		case T1_DMA:
			SPI_Write(YUSHAN_T1_DMA_REG_ENABLE, 1, &bSpiData);
			break;
		case CSI2_RX:
			/* Reset or Dereset the CSI2_Rx: Byte, Pixel Clk */
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x70;
			else
				/* Dereset data */
				bSpiData = 0x00;

				SPI_Write(YUSHAN_CSI2_RX_ENABLE, 1, &bSpiData);

				/* Reinstating the values */
				if (bResetORDeReset)
					/* Reset Data */
					bSpiData = 0x11;
				else
					/* Dereset data */
					bSpiData = 0x01;
				break;
		case IT_POINT:
			SPI_Write(YUSHAN_ITPOINT_ENABLE, 1, &bSpiData);
			break;
		case IDP_GEN:
			/* Read for Auto_Run bit. */
			SPI_Read(YUSHAN_IDP_GEN_AUTO_RUN, 1, &bSpiData);
			if (bResetORDeReset)
				bSpiData |= 0x10;
			else
				bSpiData &= 0x01;
				SPI_Write(YUSHAN_IDP_GEN_AUTO_RUN, 1, &bSpiData);
				break;
		case MIPI_RX_DTCHK:
			SPI_Write(YUSHAN_MIPI_RX_DTCHK_ENABLE, 1, &bSpiData);
			break;
		case SMIA_PATTERN_GEN:
			SPI_Write(YUSHAN_PATTERN_GEN_ENABLE, 1, &bSpiData);
			break;
		case SMIA_DCPX:
			SPI_Write(YUSHAN_SMIA_DCPX_ENABLE, 1, &bSpiData);
			break;
		case P2W_FIFO_WR:
			/* Reset or Dereset the P2W_FIFO_WR */
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x03;
			else
				/* Dereset data */
				bSpiData = 0x00;
			SPI_Write(YUSHAN_P2W_FIFO_WR_CTRL, 1, &bSpiData);
			/* Reinstating the values */
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x11;
			else
				/* Dereset data */
				bSpiData = 0x01;
			break;
		case P2W_FIFO_RD:
			/* Reset or Dereset the P2W_FIFO_RD */
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x03;
			else
				/* Dereset data */
				bSpiData = 0x00;
			SPI_Write(YUSHAN_P2W_FIFO_RD_CTRL, 1, &bSpiData);
			/* Reinstating the values */
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x11;
			else
				/* Dereset data */
				bSpiData = 0x01;
				break;
		case CSI2_TX_WRAPPER:
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x01;
			else
				/* Dereset data */
				bSpiData = 0x00;
			SPI_Write(YUSHAN_CSI2_TX_WRAPPER_CTRL, 1, &bSpiData);
			/* Reinstating the values */
			if (bResetORDeReset)
				/* Reset Data */
				bSpiData = 0x11;
			else
				/* Dereset data */
				bSpiData = 0x01;
			break;

		case CSI2_TX:
			SPI_Write(YUSHAN_CSI2_TX_ENABLE, 1, &bSpiData);
			break;
		case LINE_FILTER_BYPASS:
			SPI_Write(YUSHAN_LINE_FILTER_BYPASS_ENABLE, 1, &bSpiData);
			break;
		case DT_FILTER_BYPASS:
			SPI_Write(YUSHAN_DTFILTER_BYPASS_ENABLE, 1, &bSpiData);
			break;
		case LINE_FILTER_DXO:
			SPI_Write(YUSHAN_LINE_FILTER_DXO_ENABLE, 1, &bSpiData);
			break;
		case DT_FILTER_DXO:
			SPI_Write(YUSHAN_DTFILTER_DXO_ENABLE, 1, &bSpiData);
			break;
		case EOF_RESIZE_PRE_DXO:
			SPI_Write(YUSHAN_EOF_RESIZE_PRE_DXO_ENABLE, 1, &bSpiData);
			break;
		case LBE_PRE_DXO:
			SPI_Write(YUSHAN_LBE_PRE_DXO_ENABLE, 1, &bSpiData);
			break;
		case EOF_RESIZE_POST_DXO:
			SPI_Write(YUSHAN_EOF_RESIZE_POST_DXO_ENABLE, 1, &bSpiData);
			break;
		case LECCI_RESET:
			SPI_Write(YUSHAN_LECCI_ENABLE, 1,  &bSpiData);
			break;
		case LBE_POST_DXO:
			SPI_Write(YUSHAN_LBE_POST_DXO_ENABLE, 1,  &bSpiData);
			break;

		}

	}
	return SUCCESS;
}


/*******************************************************************************
						STATUS FUNCTIONS
*******************************************************************************/

/******************************************************************
API Function:	This function will read the AF statistics based on the
				active ROIs
Input:			Pointer to the frst 32bit element of the ROI[0].
Return:			Success or Failure
*******************************************************************/
bool_t Yushan_Read_AF_Statistics(uint32_t  *sYushanAFStats)
{

	uint8_t		bStatus = SUCCESS, bNumOfActiveRoi = 0,  bCount = 0;
	uint32_t	udwSpiData[4];
	uint32_t	*pLocalAFStats;

	/* To avoid disturbing the input parameter. */
	pLocalAFStats = sYushanAFStats;


	/* Read active number of ROI */
	bStatus &= SPI_Read(DXO_DOP_BASE_ADDR + DxODOP_ROI_active_number_7_0, 1, (uint8_t *)(&bNumOfActiveRoi));
	/* pr_err("[CAM]%s, bNumOfActiveRoi:%d\n", __func__, bNumOfActiveRoi);*/

	if (!bNumOfActiveRoi) /* NO ROI ACTIVATED */
		return SUCCESS;

#if 0
	bStatus &= SPI_Read(DXO_DOP_BASE_ADDR + DxODOP_ROI_0_stats_G_7_0 , 16*5, (uint8_t *)(sYushanAFStats));
	/* pr_info("[CAM]%s, G:%d, R:%d, B:%d, confidence:%d \n", __func__,  sYushanAFStats[0], sYushanAFStats[1], sYushanAFStats[2], sYushanAFStats[3]);*/
#else
	bNumOfActiveRoi = 1;
	/* Program the active number of ROIs */
	while (bCount < bNumOfActiveRoi) {
		/* Reading the stats from the DXO registers. */
		bStatus &= SPI_Read((uint16_t)(DXO_DOP_BASE_ADDR + DxODOP_ROI_0_stats_G_7_0 + bCount*16), 16, (uint8_t *)(&udwSpiData[0]));
		/* Copying the stats to the AF Stats struct */
		*pLocalAFStats	= udwSpiData[0];
		*(pLocalAFStats + 1) = udwSpiData[1];
		*(pLocalAFStats + 2) = udwSpiData[2];
		*(pLocalAFStats + 3) = udwSpiData[3];
		/* pr_info("[CAM]%s,ROI_%d, G:%d, R:%d, B:%d, confidence:%d \n", __func__, bCount, udwSpiData[0], udwSpiData[1], udwSpiData[2], udwSpiData[3]);*/

		/* Counter increment */
		bCount++;

		/* Points to the next ROI struct (Each ROI has four 32bits elements). */
		pLocalAFStats = sYushanAFStats + bCount * 4;
	}
#endif

	return bStatus;
}

/*******************************************************************
Yushan Pattern Gen Test
*******************************************************************/
bool_t	Yushan_PatternGenerator(Yushan_Init_Struct_t *sInitStruct, uint8_t	bPatternReq, bool_t	bDxoBypassForTestPattern)
{
	uint8_t		bSpiData, bDxoClkDiv, bPixClkDiv;
	uint32_t	udwSpiData = 0, uwHSize, uwVSize;

	pr_info("[CAM] Pattern Gen Programming Starts Here\n");

	uwHSize = sInitStruct->uwActivePixels;
	uwVSize = sInitStruct->uwLines;

	/* CSI2_TX */
	sInitStruct->sFrameFormat[1].bDatatype = 0x12;
	sInitStruct->sFrameFormat[1].uwWordcount = (uwHSize)*(sInitStruct->uwPixelFormat & 0x0F)/8;

	udwSpiData = sInitStruct->sFrameFormat[1].uwWordcount;
	SPI_Write(YUSHAN_CSI2_TX_PACKET_SIZE_0 + 0xc*1, 4, (unsigned char *) &udwSpiData);

	udwSpiData = sInitStruct->sFrameFormat[1].bDatatype ;
	SPI_Write(YUSHAN_CSI2_TX_DI_INDEX_CTRL_0 + 0xc*1, 1, (unsigned char *) &udwSpiData);

	/* LECCI Bypass */
	/* udwSpiData = 0x01;*/
	SPI_Write(YUSHAN_LECCI_BYPASS_CTRL, 1, (unsigned char*)&bDxoBypassForTestPattern); 
	SPI_Read(YUSHAN_CLK_DIV_FACTOR,1,(unsigned char*)&bDxoClkDiv);
	SPI_Read(YUSHAN_CLK_DIV_FACTOR+1,1,(unsigned char*)&bPixClkDiv);
	//Min_interline = Freq_DXO/Freq_Pix*interline.
	if ( bDxoClkDiv !=0 && bPixClkDiv !=0 )
	{
	  udwSpiData    = sInitStruct->uwLineBlankStill*bPixClkDiv/ bDxoClkDiv;
	}
	else
	{
	  udwSpiData    = 300;
	}
	SPI_Write(YUSHAN_LECCI_MIN_INTERLINE, 2, (unsigned char*)&udwSpiData);
	/* H Size */
	udwSpiData = uwHSize;
	SPI_Write(YUSHAN_LECCI_LINE_SIZE, 2, (unsigned char *) &udwSpiData);
	SPI_Write(YUSHAN_LBE_PRE_DXO_H_SIZE, 2, (unsigned char *) &udwSpiData);
	SPI_Write(YUSHAN_LBE_POST_DXO_H_SIZE, 2, (unsigned char *) &udwSpiData);

	/* Pattern Selection */
	SPI_Write(YUSHAN_PATTERN_GEN_PATTERN_TYPE_REQ, 1, &bPatternReq);
	/* Program resolution for color bars:	IDP_Gen */
	udwSpiData = (uwHSize|((uwHSize+sInitStruct->uwLineBlankStill)<<16)); /* Yushan API 10.0 */
	SPI_Write(YUSHAN_IDP_GEN_LINE_LENGTH, 4, (uint8_t *)(&udwSpiData));
	udwSpiData = (uwVSize | (0xFFF0 << 16));
	SPI_Write(YUSHAN_IDP_GEN_FRAME_LENGTH, 4, (uint8_t *)(&udwSpiData));

/* Yushan API 10.0 Start */
	/* For Color Bar mode: Disable DCPX in case of RAW10_8 */
	if(sInitStruct->uwPixelFormat == 0x0a08) {
		/* getch(); */
		bSpiData = 0x00;
		SPI_Write(YUSHAN_SMIA_DCPX_ENABLE, 1, &bSpiData); /* DCPX - DCPX_DISABLE */
	}
/* Yushan API 10.0 End */

	/* prog required tpat_data for SOLID pattern */

	/* Enable Pattern Gen */
	bSpiData = 1;
	SPI_Write(YUSHAN_PATTERN_GEN_ENABLE, 1, &bSpiData);

	/* Auto Run mode of Idp_Gen */
	bSpiData = 1;
	SPI_Write(YUSHAN_IDP_GEN_AUTO_RUN, 1, &bSpiData);

	pr_info("[CAM] Pattern Gen Programming Ends Here\n");

	return SUCCESS;
		}

/*******************************************************************
API Function:	Yushan_DXO_Lecci_Bypass
*******************************************************************/
void	Yushan_DXO_Lecci_Bypass(void)
{
	uint8_t	bSpiData;

	/* LECCI Bypass */
	bSpiData=0x01;
	SPI_Write(YUSHAN_LECCI_BYPASS_CTRL, 1, (unsigned char*)&bSpiData);

}

/*******************************************************************
API Function:	Yushan_DXO_DTFilter_Bypass
*******************************************************************/
void	Yushan_DXO_DTFilter_Bypass(void)
{
	uint32_t	udwSpiData=0;

	/* DT Filter */
	/***** DT Filter 0 ******/
	udwSpiData=0x1;
	SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH0,1,(unsigned char*)&udwSpiData);
	udwSpiData=0xd;
	SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH1,1,(unsigned char*)&udwSpiData);
	udwSpiData=0x02;
	SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH2,1,(unsigned char*)&udwSpiData);
	udwSpiData=0x03;
	SPI_Write(YUSHAN_DTFILTER_BYPASS_MATCH3,1,(unsigned char*)&udwSpiData);
	udwSpiData=0x01;
	SPI_Write(YUSHAN_DTFILTER_BYPASS_ENABLE,1,(unsigned char*)&udwSpiData);

	/***** DT Filter 1 ******/

	udwSpiData=0x5;
	SPI_Write(YUSHAN_DTFILTER_DXO_MATCH0,1,(unsigned char*)&udwSpiData);
	SPI_Write(YUSHAN_DTFILTER_DXO_MATCH2,1,(unsigned char*)&udwSpiData);
	SPI_Write(YUSHAN_DTFILTER_DXO_MATCH3,1,(unsigned char*)&udwSpiData);
	/*  udwSpiData=0x02; */
	udwSpiData=0x05;
	SPI_Write(YUSHAN_DTFILTER_DXO_MATCH1,1,(unsigned char*)&udwSpiData);
	udwSpiData=0x01;
	SPI_Write(YUSHAN_DTFILTER_DXO_ENABLE,1,(unsigned char*)&udwSpiData);


	/* return SUCCESS; */
}

void ASIC_Test(void)
{
	pr_info("[CAM] ASIC_Test E\n");
	mdelay(10);
	/* rawchip_spi_write_2B1B(0x0008, 0x6f);*//* CLKMGR - CLK_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x000c, 0x00);/* CLKMGR - RESET_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x000d, 0x00);/* CLKMGR - RESET_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x000c, 0x3f);/* CLKMGR - RESET_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x000d, 0x07);/* CLKMGR - RESET_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x000f, 0x00);/* Unreferenced register - Warning */
	mdelay(10);

	/* LDO enable sequence */
	rawchip_spi_write_2B1B(0x1405, 0x03);
	mdelay(10);
	rawchip_spi_write_2B1B(0x1405, 0x02);
	mdelay(10);
	rawchip_spi_write_2B1B(0x1405, 0x00);

	/* HD */
	/*rawchip_spi_write_2B1B(0x0015, 0x14);*//* Unreferenced register - Warning */
	rawchip_spi_write_2B1B(0x0015, 0x19);/* CLKMGR - PLL_LOOP_OUT_DF */
	rawchip_spi_write_2B1B(0x0014, 0x03);/* CLKMGR - PLL_LOOP_OUT_DF */

	mdelay(10);
	rawchip_spi_write_2B1B(0x0000, 0x0a);/* CLKMGR - CLK_DIV_FACTOR */
	mdelay(10);
	rawchip_spi_write_2B1B(0x0001, 0x0a);/* Unreferenced register - Warning */
	mdelay(10);
	rawchip_spi_write_2B1B(0x0002, 0x14);/* Unreferenced register - Warning */
	mdelay(10);
	rawchip_spi_write_2B1B(0x0010, 0x18);/* CLKMGR - PLL_CTRL_MAIN */
	mdelay(10);
	rawchip_spi_write_2B1B(0x0009, 0x01);/* Unreferenced register - Warning */
	mdelay(10);
	rawchip_spi_write_2B1B(0x1000, 0x01);/* NVM - IOR_NVM_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2000, 0xff);/* MIPIRX - DPHY_SC_4SF_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2004, 0x06);/* MIPIRX - DPHY_SC_4SF_UIX4 */
	mdelay(10);
	/* rawchip_spi_write_2B1B(0x5000, 0xff);*//* MIPITX - DPHY_MC_4MF_ENABLE */
	mdelay(10);
	/* rawchip_spi_write_2B1B(0x5004, 0x06);*//* MIPITX - DPHY_MC_4MF_UIX4 */
	rawchip_spi_write_2B1B(0x5004, 0x14);/* MIPITX - DPHY_MC_4MF_UIX4 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2408, 0x04);/* CSI2RX - CSI2_RX_NB_DATA_LANES */
	mdelay(10);
	rawchip_spi_write_2B1B(0x240c, 0x0a);/* CSI2RX - CSI2_RX_IMG_UNPACKING_FORMAT */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2420, 0x01);/* CSI2RX - CSI2_RX_BYTE2PIXEL_READ_TH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2428, 0x2b);/* CSI2RX - CSI2_RX_DATA_TYPE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4400, 0x01);/* SMIAF - SMIAF_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4404, 0x0a);/* SMIAF - SMIAF_PIX_WIDTH */
	mdelay(10);

	/* HD */
	rawchip_spi_write_2B1B(0x4a05, 0x04);/* PW2_FIFO THRESHOLD */

	/* rawchip_spi_write_2B1B(0x2c09, 0x08);*//* Unreferenced register - Warning */
	rawchip_spi_write_2B1B(0x2c09, 0x10);/* Unreferenced register - Warning */

	mdelay(10);
	rawchip_spi_write_2B1B(0x2c0c, 0x30);/* IDP_GEN - IDP_GEN_LINE_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c0d, 0x0a);/* IDP_GEN - IDP_GEN_LINE_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c0e, 0xa0);/* IDP_GEN - IDP_GEN_LINE_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c0f, 0x0f);/* IDP_GEN - IDP_GEN_LINE_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c10, 0xa8);/* IDP_GEN - IDP_GEN_FRAME_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c11, 0x07);/* IDP_GEN - IDP_GEN_FRAME_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c12, 0xf0);/* IDP_GEN - IDP_GEN_FRAME_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c13, 0xff);/* IDP_GEN - IDP_GEN_FRAME_LENGTH */
	mdelay(10);
	rawchip_spi_write_2B1B(0x3400, 0x01);/* PAT_GEN - PATTERN_GEN_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x3401, 0x00);/* PAT_GEN - PATTERN_GEN_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x3402, 0x00);/* PAT_GEN - PATTERN_GEN_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x3403, 0x00);/* PAT_GEN - PATTERN_GEN_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x3408, 0x02);/* PAT_GEN - PATTERN_GEN_PATTERN_TYPE_REQ */
	mdelay(10);
	rawchip_spi_write_2B1B(0x3409, 0x00);/* PAT_GEN - PATTERN_GEN_PATTERN_TYPE_REQ */
	mdelay(10);
	rawchip_spi_write_2B1B(0x340a, 0x00);/* PAT_GEN - PATTERN_GEN_PATTERN_TYPE_REQ */
	mdelay(10);
	rawchip_spi_write_2B1B(0x340b, 0x00);/* PAT_GEN - PATTERN_GEN_PATTERN_TYPE_REQ */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5880, 0x01);/* EOFREPRE - EOFRESIZE_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5888, 0x01);/* EOFREPRE - EOFRESIZE_AUTOMATIC_CONTROL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4400, 0x11);/* SMIAF - SMIAF_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4408, 0x01);/* SMIAF - SMIAF_GROUPED_PARAMETER_HOLD */
	mdelay(10);
	rawchip_spi_write_2B1B(0x440c, 0x03);/* SMIAF - SMIAF_EOF_INT_EN */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c00, 0x01);/* CSI2TX - CSI2_TX_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c08, 0x01);/* CSI2TX - CSI2_TX_NUMBER_OF_LANES */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c10, 0x01);/* CSI2TX - CSI2_TX_PACKET_CONTROL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c4c, 0x14);/* CSI2TX - CSI2_TX_PACKET_SIZE_0 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c4d, 0x00);/* CSI2TX - CSI2_TX_PACKET_SIZE_0 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c50, 0x2b);/* CSI2TX - CSI2_TX_DI_INDEX_CTRL_0 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c51, 0x00);/* CSI2TX - CSI2_TX_DI_INDEX_CTRL_0 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c5c, 0x2b);/* CSI2TX - CSI2_TX_DI_INDEX_CTRL_1 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c5d, 0x00);/* CSI2TX - CSI2_TX_DI_INDEX_CTRL_1 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c58, 0xbc);/* CSI2TX - CSI2_TX_PACKET_SIZE_1 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x4c59, 0x0c);/* CSI2TX - CSI2_TX_PACKET_SIZE_1 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5828, 0x01);/* DTFILTER0 - DTFILTER_MATCH0 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x582c, 0x02);/* DTFILTER0 - DTFILTER_MATCH1 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5830, 0x0d);/* DTFILTER0 - DTFILTER_MATCH2 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5834, 0x03);/* DTFILTER0 - DTFILTER_MATCH3 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5820, 0x01);/* DTFILTER0 - DTFILTER_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5868, 0xff);/* DTFILTER1 - DTFILTER_MATCH0 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x586c, 0xff);/* DTFILTER1 - DTFILTER_MATCH1 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5870, 0xff);/* DTFILTER1 - DTFILTER_MATCH2 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5874, 0xff);/* DTFILTER1 - DTFILTER_MATCH3 */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5860, 0x01);/* DTFILTER1 - DTFILTER_ENABLE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c08, 0x94);/* LECCI - LECCI_MIN_INTERLINE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c09, 0x02);/* LECCI - LECCI_MIN_INTERLINE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c0c, 0xfc);/* LECCI - LECCI_OUT_BURST_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c10, 0x90);/* LECCI - LECCI_LINE_SIZE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c11, 0x01);/* LECCI - LECCI_LINE_SIZE */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c14, 0x01);/* LECCI - LECCI_BYPASS_CTRL */
	mdelay(10);
	rawchip_spi_write_2B1B(0x5c00, 0x01);/* LECCI - LECCI_ENABLE */
	mdelay(10);

	/* HD */
	rawchip_spi_write_2B1B(0x5000, 0x33);/* LECCI - LECCI_ENABLE */
	mdelay(100);

	rawchip_spi_write_2B1B(0x2c00, 0x01);/* IDP_GEN - IDP_GEN_AUTO_RUN */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c01, 0x00);/* IDP_GEN - IDP_GEN_AUTO_RUN */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c02, 0x00);/* IDP_GEN - IDP_GEN_AUTO_RUN */
	mdelay(10);
	rawchip_spi_write_2B1B(0x2c03, 0x00);/* IDP_GEN - IDP_GEN_AUTO_RUN */
	msleep(2000);

	pr_info("[CAM] ASIC_Test X\n");
}

/* #define	COLOR_BAR */
int Yushan_sensor_open_init(void)
{
	/* color bar test */
#ifdef COLOR_BAR
	 int32_t rc = 0;
	ASIC_Test();
	/* frame_counter();*/
#else
	/*uint16_t		uwHSizeSecondary= 1640,uwVSizeSecondary= 1232;*/
	/*bool_t			bDxoBypassForTestPattern = TRUE, bTestPatternMode = 1;*/
#ifdef DxO
	bool_t	bBypassDxoUpload = 0;
	Yushan_Version_Info_t sYushanVersionInfo;
	uint8_t bPdpMode = 0, bDppMode = 0, bDopMode = 0;
#endif
	uint8_t			bPixelFormat = RAW10;/*, bPatternReq= 2;*/
	/*uint8_t 		bSpiFreq = 0x80; *//*SPI_CLK_8MHZ;*/
	/* uint32_t		udwSpiFreq = 0; */
	Yushan_Init_Struct_t	sInitStruct;
	Yushan_Init_Dxo_Struct_t	sDxoStruct;
	Yushan_ImageChar_t sImageChar;
	Yushan_GainsExpTime_t sGainsExpTime;
	Yushan_DXO_DPP_Tuning_t sDxoDppTuning;
	Yushan_DXO_PDP_Tuning_t sDxoPdpTuning;
	Yushan_DXO_DOP_Tuning_t sDxoDopTuning;
	Yushan_SystemStatus_t			sSystemStatus;
	uint32_t		udwIntrMask[] = {0x7DE38E3B, 0xA600007C, 0x000DBFFD};	// DXO_DOP_NEW_FR_PR enabled
#if 0
	Yushan_AF_ROI_t					sYushanAfRoi[5];
	Yushan_DXO_ROI_Active_Number_t	sYushanDxoRoiActiveNumber;
	Yushan_AF_Stats_t				sYushanAFStats[5];
	Yushan_New_Context_Config_t		sYushanNewContextConfig;
#endif
	uint8_t bSpiData;
	uint8_t bStatus;
	/* uint32_t spiData;*/

	 uint16_t uwHSize = sYushanFullContextConfig.uwActivePixels;/* 2608 */
	 uint16_t uwVSize = sYushanFullContextConfig.uwActiveFrameLength;/* 1960 */

	 printk("[CAM]Yushan_sensor_open_init, W=%d, H=%d E\n", uwHSize, uwVSize);

	 /* full size context configuration */
	sYushanFullContextConfig.bSelectStillVfMode	=	YUSHAN_FRAME_FORMAT_STILL_MODE;
	sYushanFullContextConfig.uwPixelFormat 	=	0x0A0A;

	  /* video fast size context configuration */
	 sYushanVideoFastContextConfig.bSelectStillVfMode	 =	 YUSHAN_FRAME_FORMAT_VF_MODE;
	 sYushanVideoFastContextConfig.uwPixelFormat	 =	 0x0A0A;

	  /* QTR size context configuration */
	 sYushanQTRContextConfig.bSelectStillVfMode =	 YUSHAN_FRAME_FORMAT_VF_MODE;
	 sYushanQTRContextConfig.uwPixelFormat  =	 0x0A0A;

 #endif

	CDBG("[CAM] Yushan API Version : %d.%d \n", API_MAJOR_VERSION, API_MINOR_VERSION);

#ifndef COLOR_BAR
	/* Default Values */
	sInitStruct.bNumberOfLanes		=	2;
	sInitStruct.fpExternalClock		=	24;
	sInitStruct.uwBitRate			=	612;/*612;//912;//970;///eric color */
	sInitStruct.uwPixelFormat = bPixelFormat;

	if ((sInitStruct.uwPixelFormat & 0x0F) == 0x0A)
		bPixelFormat = RAW10;
	else if ((sInitStruct.uwPixelFormat & 0x0F) == 0x08) {
		if (((sInitStruct.uwPixelFormat>>8) & 0x0F) == 0x08)
			bPixelFormat = RAW8;
		else /* 10 to 8 case */
			bPixelFormat = RAW10_8;
	}
#endif

#ifndef COLOR_BAR
	sDxoStruct.pDxoPdpRamImage[0] = (uint8_t *)yushan_regs.pdpcode;
	sDxoStruct.pDxoDppRamImage[0] = (uint8_t *)yushan_regs.dppcode;
	sDxoStruct.pDxoDopRamImage[0] = (uint8_t *)yushan_regs.dopcode;
	sDxoStruct.pDxoPdpRamImage[1] = (uint8_t *)yushan_regs.pdpclib;
	sDxoStruct.pDxoDppRamImage[1] = (uint8_t *)yushan_regs.dppclib;
	sDxoStruct.pDxoDopRamImage[1] = (uint8_t *)yushan_regs.dopclib;

	sDxoStruct.uwDxoPdpRamImageSize[0] = yushan_regs.pdpcode_size;
	sDxoStruct.uwDxoDppRamImageSize[0] = yushan_regs.dppcode_size;
	sDxoStruct.uwDxoDopRamImageSize[0] = yushan_regs.dopcode_size;
	sDxoStruct.uwDxoPdpRamImageSize[1] = yushan_regs.pdpclib_size;
	sDxoStruct.uwDxoDppRamImageSize[1] = yushan_regs.dppclib_size;
	sDxoStruct.uwDxoDopRamImageSize[1] = yushan_regs.dopclib_size;

	sDxoStruct.uwBaseAddrPdpMicroCode[0] = yushan_regs.pdpcode->addr;
	sDxoStruct.uwBaseAddrDppMicroCode[0] = yushan_regs.dppcode->addr;
	sDxoStruct.uwBaseAddrDopMicroCode[0] = yushan_regs.dopcode->addr;
	sDxoStruct.uwBaseAddrPdpMicroCode[1] = yushan_regs.pdpclib->addr;
	sDxoStruct.uwBaseAddrDppMicroCode[1] = yushan_regs.dppclib->addr;
	sDxoStruct.uwBaseAddrDopMicroCode[1] = yushan_regs.dopclib->addr;

#if 0
	pr_info("/*---------------------------------------*\\");
	pr_info("array base ADDRs %x %x %x %x %x %x",
		sDxoStruct.uwBaseAddrPdpMicroCode[0], sDxoStruct.uwBaseAddrDppMicroCode[0], sDxoStruct.uwBaseAddrDopMicroCode[0],
		sDxoStruct.uwBaseAddrPdpMicroCode[1], sDxoStruct.uwBaseAddrDppMicroCode[1], sDxoStruct.uwBaseAddrDopMicroCode[1]);
	pr_info("array 1st values %x %x %x %x %x %x",
		*sDxoStruct.pDxoPdpRamImage[0], *sDxoStruct.pDxoDppRamImage[0], *sDxoStruct.pDxoDopRamImage[0],
		*sDxoStruct.pDxoPdpRamImage[1], *sDxoStruct.pDxoDppRamImage[1], *sDxoStruct.pDxoDopRamImage[1]);
	pr_info("array sizes %x %x %x %x %x %x",
		sDxoStruct.uwDxoPdpRamImageSize[0], sDxoStruct.uwDxoDppRamImageSize[0], sDxoStruct.uwDxoDopRamImageSize[0],
		sDxoStruct.uwDxoPdpRamImageSize[1], sDxoStruct.uwDxoDppRamImageSize[1], sDxoStruct.uwDxoDopRamImageSize[1]);
	pr_info("Boot Addr %x %x %x",
		sDxoStruct.uwDxoPdpBootAddr, sDxoStruct.uwDxoDppBootAddr, sDxoStruct.uwDxoDopBootAddr);
	pr_info("\\*---------------------------------------*/");
#endif

	/* Configuring the SPI Freq: Dimax */
	/* Yushan_SPIConfigure (bSpiFreq);*/
#if 0
	/* Converting for Yushan */
	switch (bSpiFreq) {
	case 0x80:	/* 8Mhz */
		udwSpiFreq = 8 << 16;
		break;
	case 0x00:	/* 4Mhz */
		udwSpiFreq = 4 << 16;
		break;
	case 0x81:	/* 2Mhz */
		udwSpiFreq = 2 << 16;
		break;
	case 0x01:	/* 1Mhz */
		udwSpiFreq = 1 << 16;
		break;
	case 0x82:	/* 500Khz = 1Mhz/2 */
		udwSpiFreq = 1 << 8;
		break;
	case 0x02:	/* 250Khz = 1Mhz/4 */
		udwSpiFreq = 1 << 4;
		break;
	case 0x03:	/* 125Khz = 1Mhz/8 */
		udwSpiFreq = 1 << 2;
		break;
	  }
#endif

	sInitStruct.fpSpiClock			=	27*(1 << 16);/* 0x80000;*/ /* udwSpiFreq;*/
	sInitStruct.fpExternalClock		=	sInitStruct.fpExternalClock << 16;/* 0x180000 for 24Mbps */

	sInitStruct.uwActivePixels = uwHSize;
	sInitStruct.uwLineBlankStill = sYushanFullContextConfig.uwActivePixels - uwHSize + 192;
	sInitStruct.uwLineBlankVf = 192;/* sYushanFullContextConfig.uwActivePixels - sYushanQTRContextConfig.uwActivePixels*//*uwHSize 192 320*/;
	sInitStruct.uwLines = uwVSize;
	sInitStruct.uwFrameBlank = 112;//36;/* 42;1960-uwVSize + 42*//*16*//*136*/;
	sInitStruct.bUseExternalLDO = 1;

	sImageChar.bImageOrientation = 3;/* flip */
	sImageChar.uwXAddrStart = 0;
	sImageChar.uwYAddrStart = 0;
	sImageChar.uwXAddrEnd = uwHSize - 1;
	sImageChar.uwYAddrEnd = uwVSize - 1;
	sImageChar.uwXEvenInc = 1;
	sImageChar.uwXOddInc = 1;
	sImageChar.uwYEvenInc = 1;
	sImageChar.uwYOddInc = 1;
	sImageChar.bBinning = 0x11;

	memset(sInitStruct.sFrameFormat, 0, sizeof(Yushan_Frame_Format_t)*15);
	if ((bPixelFormat == RAW8) || (bPixelFormat == RAW10_8)) {
		CDBG("[CAM] bPixelFormat==RAW8");
		sInitStruct.sFrameFormat[0].uwWordcount = (uwHSize);	/* For RAW10 this value should be uwHSize*10/8 */
		sInitStruct.sFrameFormat[0].bDatatype = 0x2a;		  /* For RAW10 this value should be 0x2b */
		sInitStruct.sFrameFormat[1].uwWordcount = (uwHSize);
		sInitStruct.sFrameFormat[1].bDatatype = 0x2b;
		sInitStruct.sFrameFormat[1].bActiveDatatype = 1;
		sInitStruct.sFrameFormat[1].bSelectStillVfMode = YUSHAN_FRAME_FORMAT_VF_MODE;
		/* For the wordcount 0 and type of Raw8/10 in active data. */
		sInitStruct.sFrameFormat[4].bDatatype = 0x2a;
	} else { /* if (bPixelFormat ==RAW10) */
		CDBG("[CAM] bPixelFormat==RAW10");
		sInitStruct.sFrameFormat[0].uwWordcount = (uwHSize*10)/8;	 /* For RAW10 this value should be uwHSize*10/8 */
		sInitStruct.sFrameFormat[0].bDatatype = 0x2b;		  /* For RAW10 this value should be 0x2b */
		sInitStruct.sFrameFormat[1].uwWordcount = (sYushanQTRContextConfig.uwActivePixels*10)/8;
		sInitStruct.sFrameFormat[1].bDatatype = 0x2b;
		sInitStruct.sFrameFormat[1].bActiveDatatype = 1;
		sInitStruct.sFrameFormat[1].bSelectStillVfMode = YUSHAN_FRAME_FORMAT_VF_MODE;
		/* For the wordcount 0 and type of Raw8/10 in active data. */
		sInitStruct.sFrameFormat[4].bDatatype = 0x2b;
	}
	/* Overwritting Data Type for 10 to 8 Pixel format */
	if (bPixelFormat == RAW10_8) {
		sInitStruct.sFrameFormat[0].bDatatype = 0x30;
		sInitStruct.sFrameFormat[1].bDatatype = 0x30;
		sInitStruct.sFrameFormat[4].bDatatype = 0x30;
	}
	sInitStruct.sFrameFormat[0].bActiveDatatype = 1;
	sInitStruct.sFrameFormat[0].bSelectStillVfMode = YUSHAN_FRAME_FORMAT_STILL_MODE;

	/* add frame formats for embedded lines for both modes*/
	sInitStruct.sFrameFormat[2].bDatatype = 0x12;
	sInitStruct.sFrameFormat[2].uwWordcount = sInitStruct.sFrameFormat[0].uwWordcount;
	sInitStruct.sFrameFormat[2].bActiveDatatype = 0x12; /* Status lines datatype */
	sInitStruct.sFrameFormat[2].bSelectStillVfMode = YUSHAN_FRAME_FORMAT_NORMAL_MODE;

	sInitStruct.sFrameFormat[3].bDatatype = 0x12;
	sInitStruct.sFrameFormat[3].uwWordcount = sInitStruct.sFrameFormat[1].uwWordcount;
	sInitStruct.sFrameFormat[3].bActiveDatatype = 0x12; /* Status lines datatype */
	sInitStruct.sFrameFormat[3].bSelectStillVfMode = YUSHAN_FRAME_FORMAT_NORMAL_MODE;

	/* Frame format with WC=0 */
	sInitStruct.sFrameFormat[4].uwWordcount = 0; /* One wordcount to be 0 (CSI2_RX will flush last line in InterFrame) */
	sInitStruct.sFrameFormat[4].bActiveDatatype = 1; /* for fix of Bug#51 */
	sInitStruct.sFrameFormat[4].bSelectStillVfMode = YUSHAN_FRAME_FORMAT_NORMAL_MODE;

	sGainsExpTime.uwAnalogGainCodeGR = 0x20;/* 0x0 10x=>140;1x=>20 */
	sGainsExpTime.uwAnalogGainCodeR = 0x20;
	sGainsExpTime.uwAnalogGainCodeB = 0x20;
	sGainsExpTime.uwPreDigGainGR = 0x100;
	sGainsExpTime.uwPreDigGainR = 0x100;
	sGainsExpTime.uwPreDigGainB = 0x100;
	sGainsExpTime.uwExposureTime = 0x200;
	sGainsExpTime.bRedGreenRatio = 0x40;
	sGainsExpTime.bBlueGreenRatio = 0x40;

	  sDxoDppTuning.bTemporalSmoothing=0x63;
	sDxoDppTuning.uwFlashPreflashRating = 0;
	sDxoDppTuning.bFocalInfo = 0;

	sDxoPdpTuning.bDeadPixelCorrectionLowGain = 0x80;
	sDxoPdpTuning.bDeadPixelCorrectionMedGain = 0x80;
	sDxoPdpTuning.bDeadPixelCorrectionHiGain = 0x80;

#if 0
	sDxoDopTuning.uwForceClosestDistance = 0;	/* Removed follwing DXO recommendation */
	sDxoDopTuning.uwForceFarthestDistance = 0;
#endif
	sDxoDopTuning.bEstimationMode = 1;
	sDxoDopTuning.bSharpness = 0x70;//0x70; //0x60;
	sDxoDopTuning.bDenoisingLowGain = 0x75;//0x80;//0x60;/* 0xFF for de-noise verify, original:0x1 */
	sDxoDopTuning.bDenoisingMedGain = 0x70;//0x80;
	sDxoDopTuning.bDenoisingHiGain = 0x60;//0x90;
	sDxoDopTuning.bNoiseVsDetailsLowGain = 0x90;//0x85//0xA0;/* Noise v.s. Texture by Graidient. Higher value cause sharp image */
	sDxoDopTuning.bNoiseVsDetailsMedGain = 0x80;
	sDxoDopTuning.bNoiseVsDetailsHiGain = 0x80;
	sDxoDopTuning.bTemporalSmoothing = 0x26;/* Transistion btw three mode of sharpess within diff frames */

	gPllLocked = 0;
	CDBG("[CAM] Yushan_common_init Yushan_Init_Clocks\n");
	bStatus = Yushan_Init_Clocks(&sInitStruct, &sSystemStatus, udwIntrMask) ;
	if (bStatus != 1) {
		pr_err("[CAM] Clock Init FAILED\n");
		pr_err("[CAM] Yushan_common_init Yushan_Init_Clocks=%d\n", bStatus);
		pr_err("[CAM] Min Value Required %d\n", sSystemStatus.udwDxoConstraintsMinValue);
		pr_err("[CAM] Error Code : %d\n", sSystemStatus.bDxoConstraints);
		/* return -1;*/
	} else
		CDBG("[CAM] Clock Init Done \n");
	/* Pll Locked. Done to allow auto incremental SPI transactions. */
	gPllLocked = 1;

	CDBG("[CAM] Yushan_common_init Yushan_Init\n");
	bStatus = Yushan_Init(&sInitStruct) ;
	CDBG("[CAM] Yushan_common_init Yushan_Init=%d\n", bStatus);

	/* Initialize DCPX and CPX of Yushan */
	if (bPixelFormat == RAW10_8)
		Yushan_DCPX_CPX_Enable();

	if (bStatus == 0) {
		pr_err("[CAM] Yushan Init FAILED\n");
		return -1;
	}
	/* The initialization is done */

#ifdef DxO
	CDBG("[CAM] Yushan_common_init Yushan_Init_Dxo\n");
	/* bBypassDxoUpload = 1; */
	bStatus = Yushan_Init_Dxo(&sDxoStruct, bBypassDxoUpload);
	CDBG("[CAM] Yushan_common_init Yushan_Init_Dxo=%d\n", bStatus);
	if (bStatus == 1) {
		CDBG("[CAM] DXO Upload and Init Done\n");
	} else {
		pr_err("[CAM] DXO Upload and Init FAILED\n");
		/* return -1;*/
	}
	CDBG("[CAM] Yushan_common_init Yushan_Get_Version_Information\n");

	bStatus = Yushan_Get_Version_Information(&sYushanVersionInfo) ;
#if 0
	pr_info("Yushan_common_init Yushan_Get_Version_Information=%d\n", bStatus);

	pr_info("API Version : %d.%d \n", sYushanVersionInfo.bApiMajorVersion, sYushanVersionInfo.bApiMinorVersion);
	pr_info("DxO Pdp Version : %x \n", sYushanVersionInfo.udwPdpVersion);
	pr_info("DxO Dpp Version : %x \n", sYushanVersionInfo.udwDppVersion);
	pr_info("DxO Dop Version : %x \n", sYushanVersionInfo.udwDopVersion);
	pr_info("DxO Pdp Calibration Version : %x \n", sYushanVersionInfo.udwPdpCalibrationVersion);
	pr_info("DxO Dpp Calibration Version : %x \n", sYushanVersionInfo.udwDppCalibrationVersion);
	pr_info("DxO Dop Calibration Version : %x \n", sYushanVersionInfo.udwDopCalibrationVersion);
#endif
/* #endif */

#if 0
	/* For Test Pattern */
	if (bTestPatternMode == 1) {
		/* Pattern Gen */
		Yushan_PatternGenerator(&sInitStruct, bPatternReq, bDxoBypassForTestPattern);
		/* frame_counter(); */
		return 0;
	}
#else

#endif

	/* Updating DXO */
	Yushan_Update_ImageChar(&sImageChar);
	Yushan_Update_SensorParameters(&sGainsExpTime);
	Yushan_Update_DxoDpp_TuningParameters(&sDxoDppTuning);
	Yushan_Update_DxoDop_TuningParameters(&sDxoDopTuning);
	Yushan_Update_DxoPdp_TuningParameters(&sDxoPdpTuning);
	bPdpMode = 1; bDppMode = 3; bDopMode = 1;
	bStatus = Yushan_Update_Commit(bPdpMode, bDppMode, bDopMode);
	CDBG("[CAM] Yushan_common_init Yushan_Update_Commit=%d\n", bStatus);

	/* Wait for the EndOfExec interrupts. This indicates the completion of the */
	/* execution command */
	bStatus &= Yushan_WaitForInterruptEvent(EVENT_PDP_EOF_EXECCMD, TIME_10MS);
	if (!bStatus)
	{
		pr_err("[CAM] EVENT_PDP_EOF_EXECCMD fail\n");
		return -1;
	}

	bStatus &= Yushan_WaitForInterruptEvent2(EVENT_DOP7_EOF_EXECCMD, TIME_10MS);
	if (!bStatus)
	{
		pr_err("[CAM] EVENT_DOP7_EOF_EXECCMD fail\n");
		return -1;
	}

	bStatus &= Yushan_WaitForInterruptEvent(EVENT_DPP_EOF_EXECCMD, TIME_10MS);
	if (!bStatus)
	{
		pr_err("[CAM] EVENT_DPP_EOF_EXECCMD fail\n");
		return -1;
	}

	if (bStatus == 1)
		CDBG("[CAM] DXO Commit Done\n");
	else {
		pr_err("[CAM] DXO Commit FAILED\n");
		/* return -1;*/
	}

#endif

	/* disable extra INT */
	bSpiData = 0;
	SPI_Write(YUSHAN_SMIA_FM_EOF_INT_EN, 1,  &bSpiData);

	/* select_mode (0x18);*/
	CDBG("[CAM] Yushan_Set_AF_Register\n");
	bSpiData = 0x10;
	SPI_Write(DXO_DOP_BASE_ADDR+0x02a4/*DxODOP_af_strategy_7_0*/, 1, &bSpiData);

	return (bStatus == SUCCESS) ? 0 : -1;
#endif
	return 0;
	}

/*********************************************************************
Function to programm AF_ROI and Check the stats too
**********************************************************************/
bool_t Yushan_Dxo_Dop_Af_Run(Yushan_AF_ROI_t	*sYushanAfRoi, uint32_t *pAfStatsGreen, uint8_t	bRoiActiveNumber)
{

	uint8_t		bStatus = SUCCESS;
	/* uint16_t	uwSpiData = 0;*/
	uint8_t		bPdpMode = 0, bDppMode = 0, bDopMode = 0;
#if 1
	uint32_t		enableIntrMask[] = {0x00338E30, 0x00000000, 0x00018000};
	uint32_t		disableIntrMask[] = {0x00238E30, 0x00000000, 0x00018000};	
#endif

	/* Recommended by DXO */
	bPdpMode = 1; bDppMode = 3; bDopMode = 1;

	/* pr_info("[CAM] Active Number of ROI is more than 0.\nUpdating the ROIs post streaming\n\n"); */

	if (bRoiActiveNumber)
	{
		Yushan_Intr_Enable((uint8_t*)enableIntrMask);
		/* Program the AF_ROI */
		bStatus = Yushan_AF_ROI_Update(&sYushanAfRoi[0], bRoiActiveNumber);
		bStatus &= Yushan_Update_Commit(bPdpMode, bDppMode, bDopMode);

		/* Reading the AF Statistics */
		bStatus &= Yushan_WaitForInterruptEvent2(EVENT_DXODOP7_NEWFRAMEPROC_ACK, TIME_20MS);
	}
	else
		Yushan_Intr_Enable((uint8_t*)disableIntrMask);

#if 0
	if (bStatus) {
		/* pr_info("[CAM] Successufully DXO Commited and received EVENT_DXODOP7_NEWFRAMEPROC_ACK \n");*/
		#if 0
		yushan_go_to_position(0, 0);
		for(i=1; i<=42; i++)
		{
			s5k3h2yx_move_focus(1, i);
			bStatus = Yushan_Read_AF_Statistics(pAfStatsGreen);
		}
		#endif
		bStatus = Yushan_Read_AF_Statistics(pAfStatsGreen);
	}

	if (!bStatus) {
		pr_err("ROI AF Statistics read failed\n");
		return FAILURE;
	} else
		pr_err("Read ROI AF Statistics successfully\n");
#endif

	return SUCCESS;

}

int Yushan_get_AFSU(uint32_t *pAfStatsGreen)
{

	uint8_t		bStatus = SUCCESS;

	bStatus = Yushan_Read_AF_Statistics(pAfStatsGreen);
	if (bStatus == FAILURE) {
		pr_err("[CAM] Get AFSU statistic data fail\n");
		return -1;
	}
	CDBG("[CAM] GET_AFSU:G:%d, R:%d, B:%d, confi:%d\n",
		pAfStatsGreen[0], pAfStatsGreen[1], pAfStatsGreen[2], pAfStatsGreen[3]);
	return 0;
}

/*********************************************************************
Function: Context Update Wrapper
**********************************************************************/
#define SENSOR_FULL_SIZE_WIDTH 2608
#define SENSOR_FULL_SIZE_HEIGHT 1960

#define SENSOR_QTR_SIZE_WIDTH 1304
#define SENSOR_QTR_SIZE_HEIGHT 980

#define SENSOR_VIDEO_SIZE_WIDTH_FAST_4X  0x28c
#define SENSOR_VIDEO_SIZE_HEIGHT_FAST_4X   0x187 

Yushan_ImageChar_t	sImageChar_context;
bool_t	Yushan_ContextUpdate_Wrapper(Yushan_New_Context_Config_t	*sYushanNewContextConfig)
{

	bool_t	bStatus = SUCCESS;
	uint8_t	bPdpMode = 0, bDppMode = 0, bDopMode = 0;

		CDBG("[CAM] Reconfiguration starts:%d,%d,%d\n",
			sYushanNewContextConfig->uwActiveFrameLength,
			sYushanNewContextConfig->uwActivePixels,
			sYushanNewContextConfig->uwLineBlank);
		bStatus = Yushan_Context_Config_Update(sYushanNewContextConfig);
		/* sYushanAEContextConfig = (Yushan_New_Context_Config_t*)&sYushanNewContextConfig; */

	/* Now update the DXO */
	sImageChar_context.bImageOrientation = 3;
	sImageChar_context.uwXAddrStart = 0;
	sImageChar_context.uwYAddrStart = 0;

	sImageChar_context.uwXEvenInc = 1;
	sImageChar_context.uwXOddInc = 1;
	sImageChar_context.uwYEvenInc = 1;
	sImageChar_context.uwYOddInc=1;
	sImageChar_context.bBinning = 0x11;

//	sImageChar_context.uwXAddrEnd= sYushanNewContextConfig->uwActivePixels - 1;
//	sImageChar_context.uwYAddrEnd= sYushanNewContextConfig->uwActiveFrameLength -1;
	sImageChar_context.uwXAddrEnd= SENSOR_FULL_SIZE_WIDTH - 1;
	sImageChar_context.uwYAddrEnd= SENSOR_FULL_SIZE_HEIGHT -1;

//	if(sYushanNewContextConfig->uwActivePixels == 1304) ///eric dirty code for 5M
	
	//QTR size  -------------------------------------------------------------------
	if ((sYushanNewContextConfig->uwActivePixels ==
		SENSOR_QTR_SIZE_WIDTH) &&
		(sYushanNewContextConfig->uwActiveFrameLength ==
		SENSOR_QTR_SIZE_HEIGHT))
	{
// Same as FULL size.
		sImageChar_context.uwYOddInc = 3;
		sImageChar_context.uwXOddInc = 3;
		sImageChar_context.bBinning = 0x22;
		}
	//FAST_VIDEO_4X_SIZE size  -------------------------------------------------------------------
	else if ((sYushanNewContextConfig->uwActivePixels ==
		SENSOR_VIDEO_SIZE_WIDTH_FAST_4X) &&
		(sYushanNewContextConfig->uwActiveFrameLength ==
		SENSOR_VIDEO_SIZE_HEIGHT_FAST_4X))
	{
//WA, FIX ME
#if 1
		sImageChar_context.uwXAddrEnd= 0xa2f;
		sImageChar_context.uwYAddrEnd= 0x6e1;
		sImageChar_context.uwYAddrStart = 0xc6;
		sImageChar_context.uwYOddInc = 7;
		sImageChar_context.uwXOddInc = 7;
		sImageChar_context.bBinning = 0x22;
#endif
	}

	pr_info("[CAM] Yushan_ContextUpdate_Wrapper, X_end =%d, Y_end =%d\n", sImageChar_context.uwXAddrEnd, sImageChar_context.uwYAddrEnd);
	/* default enable DxO */
	bPdpMode = 1; bDppMode = 3; bDopMode = 1;
	Yushan_Update_ImageChar(&sImageChar_context);
	bStatus &= Yushan_Update_Commit(bPdpMode, bDppMode, bDopMode);

	/* Wait for the EndOfExec interrupts. This indicates the completion of the */
	/* execution command */
	bStatus &= Yushan_WaitForInterruptEvent(EVENT_PDP_EOF_EXECCMD, TIME_10MS);
	if (!bStatus)
	{
		pr_err("[CAM] EVENT_PDP_EOF_EXECCMD fail\n");
		return -1;
	}

	bStatus &= Yushan_WaitForInterruptEvent2(EVENT_DOP7_EOF_EXECCMD, TIME_10MS);
	if (!bStatus)
	{
		pr_err("[CAM] EVENT_DOP7_EOF_EXECCMD fail\n");
		return -1;
	}

	bStatus &= Yushan_WaitForInterruptEvent(EVENT_DPP_EOF_EXECCMD, TIME_10MS);
	if (!bStatus)
	{
		pr_err("[CAM] EVENT_DPP_EOF_EXECCMD fail\n");
		return -1;
	}

	if (bStatus)
		CDBG("[CAM] DXO Commit, Post Context Reconfigration, Done\n");
	else
		pr_err("[CAM] DXO Commit, Post Context Reconfigration, FAILED\n");

	return bStatus;

}

#if 0
void Yushan_Write_Exp_Time_Gain(uint16_t yushan_line, uint16_t yushan_gain)
{
#if 0
	Yushan_GainsExpTime_t sGainsExpTime;
	uint32_t udwSpiBaseIndex;
	uint32_t spidata;
	float ratio = 0.019;
	pr_info("[CAM] Yushan_Write_Exp_Time_Gain, yushan_gain:%d, yushan_line:%d", yushan_gain, yushan_line);
#endif

	sGainsExpTime.uwAnalogGainCodeGR= yushan_gain;
	sGainsExpTime.uwAnalogGainCodeR = yushan_gain;
	sGainsExpTime.uwAnalogGainCodeB = yushan_gain;
	sGainsExpTime.uwPreDigGainGR = 0x100;
	sGainsExpTime.uwPreDigGainR = 0x100;
	sGainsExpTime.uwPreDigGainB = 0x100;
	sGainsExpTime.uwExposureTime = (uint16_t)((19*yushan_line/1000));

	if (sGainsExpTime.bRedGreenRatio == 0) sGainsExpTime.bRedGreenRatio=0x40;
	if (sGainsExpTime.bBlueGreenRatio == 0) sGainsExpTime.bBlueGreenRatio=0x40;

	pr_err("[CAM] uwExposureTime: %d\n", sGainsExpTime.uwExposureTime);
	Yushan_Update_SensorParameters(&sGainsExpTime);
#if 0
	pr_info("DxO Regiser Dump Start******* \n");
	SPI_Read((0x821E), 4, (uint8_t *)(&spidata));
	pr_info("DxO DOP 0x821E : %x \n", spidata);
	SPI_Read((0x8222), 4, (uint8_t *)(&spidata));
	pr_info("DxO DOP 0x8222 : %x \n", spidata);
	SPI_Read((0x8226), 4, (uint8_t *)(&spidata));
	pr_info("DxO DOP 0x8226 : %x \n", spidata);
	SPI_Read((0x822A), 4, (uint8_t *)(&spidata));
	pr_info("DxO DOP 0x822A : %x \n", spidata);
	SPI_Read((0x822E), 2, (uint8_t *)(&spidata));
	pr_info("DxO DOP 0x822E : %x \n", spidata);
	SPI_Read((0x8204), 4, (uint8_t *)(&spidata));
	pr_info("DxO DOP Cali_ID : %x \n", spidata);

	udwSpiBaseIndex = 0x010000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));
	SPI_Read((0x821E), 4, (uint8_t *)(&spidata));
	pr_info("DxO DPP 0x821E : %x \n", spidata);
	SPI_Read((0x8222), 4, (uint8_t *)(&spidata));
	pr_info("DxO DPP 0x8222 : %x \n", spidata);
	SPI_Read((0x8226), 4, (uint8_t *)(&spidata));
	pr_info("DxO DPP 0x8226 : %x \n", spidata);
	SPI_Read((0x822A), 2, (uint8_t *)(&spidata));
	pr_info("DxO DPP 0x822A : %x \n", spidata);
	SPI_Read((0x8204), 4, (uint8_t *)(&spidata));
	pr_info("DxO DPP Cali_ID : %x \n", spidata);

	udwSpiBaseIndex = 0x08000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));
	SPI_Read((0x621E), 4, (uint8_t *)(&spidata));
	pr_info("DxO PDP 0x621E : %x \n", spidata);
	SPI_Read((0x6222), 4, (uint8_t *)(&spidata));
	pr_info("DxO PDP 0x6222 : %x \n", spidata);
	SPI_Read((0x6226), 4, (uint8_t *)(&spidata));
	pr_info("DxO PDP 0x6226 : %x \n", spidata);
	SPI_Read((0x622A), 2, (uint8_t *)(&spidata));
	pr_info("DxO PDP 0x622A : %x \n", spidata);
	SPI_Read((0x6204), 4, (uint8_t *)(&spidata));
	pr_info("DxO PDP Cali_ID : %x \n", spidata);
#endif

}
#endif

int Yushan_Update_AEC_AWB_Params(rawchip_update_aec_awb_params_t *update_aec_awb_params)
{
			uint8_t		bStatus = SUCCESS;
	Yushan_GainsExpTime_t sGainsExpTime;

	sGainsExpTime.uwAnalogGainCodeGR = update_aec_awb_params->aec_params.gain;
	sGainsExpTime.uwAnalogGainCodeR = update_aec_awb_params->aec_params.gain;
	sGainsExpTime.uwAnalogGainCodeB = update_aec_awb_params->aec_params.gain;
	sGainsExpTime.uwPreDigGainGR = 0x100;
	sGainsExpTime.uwPreDigGainR = 0x100;
	sGainsExpTime.uwPreDigGainB = 0x100;
	sGainsExpTime.uwExposureTime = (uint16_t)((19*update_aec_awb_params->aec_params.line/1000));
	sGainsExpTime.bRedGreenRatio = update_aec_awb_params->awb_params.rg_ratio;
	sGainsExpTime.bBlueGreenRatio = update_aec_awb_params->awb_params.bg_ratio;
#if 0
	if (sGainsExpTime.bRedGreenRatio == 0)
		sGainsExpTime.bRedGreenRatio = 0x40;
	if (sGainsExpTime.bBlueGreenRatio == 0)
		sGainsExpTime.bBlueGreenRatio = 0x40;
#endif

	CDBG("[CAM] uwExposureTime: %d\n", sGainsExpTime.uwExposureTime);
	bStatus = Yushan_Update_SensorParameters(&sGainsExpTime);

	return (bStatus == SUCCESS) ? 0 : -1;
	}

int Yushan_Update_AF_Params(rawchip_update_af_params_t *update_af_params)
{

	uint8_t bStatus = SUCCESS;
	bStatus = Yushan_AF_ROI_Update(&update_af_params->af_params.sYushanAfRoi[0],
		update_af_params->af_params.active_number);
	return (bStatus == SUCCESS) ? 0 : -1;
}

int Yushan_Update_3A_Params(rawchip_newframe_ack_enable_t enable_newframe_ack)
{
	uint8_t bStatus = SUCCESS;
	uint8_t bPdpMode = 1, bDppMode = 3, bDopMode = 1;
	uint32_t		enableIntrMask[] = {0x7DF38E3B, 0xA600007C, 0x000DBFFD};
	uint32_t		disableIntrMask[] = {0x7DE38E3B, 0xA600007C, 0x000DBFFD};
	if (enable_newframe_ack == RAWCHIP_NEWFRAME_ACK_ENABLE)
		Yushan_Intr_Enable((uint8_t *)enableIntrMask);
	else if (enable_newframe_ack == RAWCHIP_NEWFRAME_ACK_DISABLE)
		Yushan_Intr_Enable((uint8_t *)disableIntrMask);
	bStatus = Yushan_Update_Commit(bPdpMode, bDppMode, bDopMode);
	return (bStatus == SUCCESS) ? 0 : -1;
}

void Yushan_dump_register(void)
{
	uint16_t read_data = 0;

	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_STATUS=%x\n", read_data);
}

void Yushan_dump_all_register(void)
{
	uint16_t read_data = 0;
	uint8_t i;
	for (i = 0; i < 50; i++) {
		/* Yushan's in counting */
		rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_FRAME_NUMBER, &read_data);
		pr_info("[CAM] Yushan's in counting=%d\n", read_data);

		/* Yushan's out counting */
		rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_FRAME_NO_0, &read_data);
		pr_info("[CAM] Yushan's out counting=%d\n", read_data);

		mdelay(30);
	}

	rawchip_spi_read_2B2B(YUSHAN_CLK_DIV_FACTOR, &read_data);
	pr_info("[CAM]YUSHAN_CLK_DIV_FACTOR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CLK_DIV_FACTOR_2, &read_data);
	pr_info("[CAM]YUSHAN_CLK_DIV_FACTOR_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CLK_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_CLK_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_RESET_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_RESET_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PLL_CTRL_MAIN, &read_data);
	pr_info("[CAM]YUSHAN_PLL_CTRL_MAIN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PLL_LOOP_OUT_DF, &read_data);
	pr_info("[CAM]YUSHAN_PLL_LOOP_OUT_DF=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PLL_SSCG_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_PLL_SSCG_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_HOST_IF_SPI_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_HOST_IF_SPI_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_HOST_IF_SPI_DEVADDR, &read_data);
	pr_info("[CAM]YUSHAN_HOST_IF_SPI_DEVADDR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, &read_data);
	pr_info("[CAM]YUSHAN_HOST_IF_SPI_BASE_ADDRESS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2RX_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2RX_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_PDP_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_PDP_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_PDP_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_PDP_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_PDP_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_PDP_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_PDP_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_PDP_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_PDP_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_PDP_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_PDP_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_PDP_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DPP_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DPP_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DPP_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DPP_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DPP_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DPP_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DPP_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DPP_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DPP_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DPP_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DPP_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DPP_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DOP7_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DOP7_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DOP7_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DOP7_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DOP7_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DOP7_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DOP7_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DOP7_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DOP7_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DOP7_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_DOP7_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_DOP7_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_CSI2TX_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_CSI2TX_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_PHY_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_PHY_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_PHY_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_PHY_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_PHY_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_PHY_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_PHY_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_PHY_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_PHY_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_PHY_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_PHY_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_PHY_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_TX_PHY_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_TX_PHY_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_TX_PHY_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_TX_PHY_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_TX_PHY_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_TX_PHY_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_TX_PHY_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_TX_PHY_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_TX_PHY_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_TX_PHY_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_TX_PHY_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_TX_PHY_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_IDP_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_IDP_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_CHAR_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_CHAR_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_CHAR_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_CHAR_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_CHAR_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_CHAR_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_CHAR_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_CHAR_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_CHAR_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_CHAR_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_RX_CHAR_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_RX_CHAR_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_LBE_POST_DXO_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_LBE_POST_DXO_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_LBE_POST_DXO_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_LBE_POST_DXO_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_LBE_POST_DXO_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_LBE_POST_DXO_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_LBE_POST_DXO_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_LBE_POST_DXO_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_LBE_POST_DXO_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_LBE_POST_DXO_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_LBE_POST_DXO_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_LBE_POST_DXO_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_SYS_DOMAIN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_SYS_DOMAIN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_SYS_DOMAIN_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_SYS_DOMAIN_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_SYS_DOMAIN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_SYS_DOMAIN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_SYS_DOMAIN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_SYS_DOMAIN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_SYS_DOMAIN_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_SYS_DOMAIN_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_SYS_DOMAIN_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_SYS_DOMAIN_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_ITPOINT_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_ITPOINT_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_ITPOINT_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_ITPOINT_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_ITPOINT_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_ITPOINT_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_ITPOINT_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_ITPOINT_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_ITPOINT_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_ITPOINT_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_ITPOINT_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_ITPOINT_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_EN_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_EN_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_EN_STATUS_BCLR, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_EN_STATUS_BCLR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITM_P2W_UFLOW_EN_STATUS_BSET, &read_data);
	pr_info("[CAM]YUSHAN_ITM_P2W_UFLOW_EN_STATUS_BSET=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_DATA_WORD_0, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_DATA_WORD_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_DATA_WORD_1, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_DATA_WORD_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_DATA_WORD_2, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_DATA_WORD_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_DATA_WORD_3, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_DATA_WORD_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_HYST, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_HYST=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_PDN, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_PDN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_PUN, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_PUN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_LOWEMI, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_LOWEMI=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_PAD_IN, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_PAD_IN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_RATIO_PAD, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_RATIO_PAD=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_SEND_ITR_PAD1, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_SEND_ITR_PAD1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_INTR_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_INTR_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IOR_NVM_LDO_STS_REG, &read_data);
	pr_info("[CAM]YUSHAN_IOR_NVM_LDO_STS_REG=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_REG_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_REG_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_REG_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_REG_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_REG_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_REG_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_REG_REFILL_ELT_NB, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_REG_REFILL_ELT_NB=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_REG_REFILL_ERROR, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_REG_REFILL_ERROR=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_REG_DFV_CONTROL, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_REG_DFV_CONTROL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_MEM_PAGE, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_MEM_PAGE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_MEM_LOWER_ELT, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_MEM_LOWER_ELT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_T1_DMA_MEM_UPPER_ELT, &read_data);
	pr_info("[CAM]YUSHAN_T1_DMA_MEM_UPPER_ELT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_UIX4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_UIX4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_SWAP_PINS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_SWAP_PINS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_INVERT_HS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_INVERT_HS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_STOP_STATE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_STOP_STATE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_ULP_STATE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_ULP_STATE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_CLK_ACTIVE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_CLK_ACTIVE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_FORCE_RX_MODE_DL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_FORCE_RX_MODE_DL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_TEST_RESERVED, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_TEST_RESERVED=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_ESC_DL_STS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_ESC_DL_STS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_EOT_BYPASS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_EOT_BYPASS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_HSRX_SHIFT_CL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_HSRX_SHIFT_CL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_HS_RX_SHIFT_DL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_HS_RX_SHIFT_DL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_VIL_CL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_VIL_CL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_VIL_DL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_VIL_DL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_OVERSAMPLE_BYPASS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_OVERSAMPLE_BYPASS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_OVERSAMPLE_FLAG1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_OVERSAMPLE_FLAG1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_SKEW_OFFSET_1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_SKEW_OFFSET_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_SKEW_OFFSET_2, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_SKEW_OFFSET_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_SKEW_OFFSET_3, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_SKEW_OFFSET_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_SKEW_OFFSET_4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_SKEW_OFFSET_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_OFFSET_CL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_OFFSET_CL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_CALIBRATE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_CALIBRATE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_SPECS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_SPECS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_COMP, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_COMP=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_MIPI_IN_SHORT, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_MIPI_IN_SHORT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_LANE_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_LANE_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_VER_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_VER_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_NB_DATA_LANES, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_NB_DATA_LANES=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_IMG_UNPACKING_FORMAT, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_IMG_UNPACKING_FORMAT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_WAIT_AFTER_PACKET_END, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_WAIT_AFTER_PACKET_END=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_MULTIPLE_OF_5_HSYNC_EXTENSION_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_MULTIPLE_OF_5_HSYNC_EXTENSION_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_MULTIPLE_OF_5_HSYNC_EXTENSION_PADDING_DATA, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_MULTIPLE_OF_5_HSYNC_EXTENSION_PADDING_DATA=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_CHARACTERIZATION_MODE, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_CHARACTERIZATION_MODE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_BYTE2PIXEL_READ_TH, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_BYTE2PIXEL_READ_TH=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_VIRTUAL_CHANNEL, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_VIRTUAL_CHANNEL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_DATA_TYPE, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_DATA_TYPE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_FRAME_NUMBER, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_FRAME_NUMBER=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_LINE_NUMBER, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_LINE_NUMBER=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_DATA_FIELD, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_DATA_FIELD=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_WORD_COUNT, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_WORD_COUNT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_ECC_ERROR_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_ECC_ERROR_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_RX_DFV, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_RX_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_PIX_POS, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_PIX_POS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_LINE_POS, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_LINE_POS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_PIX_CNT, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_PIX_CNT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_LINE_CNT, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_LINE_CNT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_FRAME_CNT, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_FRAME_CNT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_ITPOINT_DFV, &read_data);
	pr_info("[CAM]YUSHAN_ITPOINT_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_AUTO_RUN, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_AUTO_RUN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_CONTROL, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_CONTROL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_LINE_LENGTH, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_LINE_LENGTH=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_FRAME_LENGTH, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_FRAME_LENGTH=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_ERROR_LINES_EOF_GAP, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_ERROR_LINES_EOF_GAP=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_0, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_1, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_2, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_3, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_4, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_5, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_6, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_7, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_8, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_8=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_9, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_9=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_10, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_10=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_11, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_11=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_12, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_12=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_13, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_13=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_WC_DI_14, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_WC_DI_14=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_IDP_GEN_DFV, &read_data);
	pr_info("[CAM]YUSHAN_IDP_GEN_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_VERSION_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_VERSION_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLORBAR_WIDTH_BY4_M1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLORBAR_WIDTH_BY4_M1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_0, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_2, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_3, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_5, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_6, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_7, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_VAL_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_0, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_2, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_3, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_5, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_6, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_7, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_IGNORE_ERR_CNT_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_0, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_2, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_3, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_5, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_6, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_7, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERRVAL_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_0, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_2, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_3, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_5, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_6, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_7, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_COLOR_BAR_ERR_POS_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_RX_DTCHK_DFV, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_RX_DTCHK_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_PATTERN_TYPE_REQ, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_PATTERN_TYPE_REQ=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_TPAT_DATA_RG, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_TPAT_DATA_RG=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_TPAT_DATA_BG, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_TPAT_DATA_BG=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_TPAT_HCUR_WP, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_TPAT_HCUR_WP=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_TPAT_VCUR_WP, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_TPAT_VCUR_WP=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_PATTERN_GEN_PATTERN_TYPE_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_PATTERN_GEN_PATTERN_TYPE_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_DCPX_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_DCPX_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_DCPX_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_DCPX_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_DCPX_ENABLE_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_DCPX_ENABLE_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_DCPX_MODE_REQ, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_DCPX_MODE_REQ=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_DCPX_MODE_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_DCPX_MODE_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_CPX_CTRL_REQ, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_CPX_CTRL_REQ=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_CPX_MODE_REQ, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_CPX_MODE_REQ=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_CPX_CTRL_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_CPX_CTRL_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_CPX_MODE_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_CPX_MODE_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_FM_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_FM_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_FM_PIX_WIDTH, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_FM_PIX_WIDTH=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_FM_GROUPED_PARAMETER_HOLD, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_FM_GROUPED_PARAMETER_HOLD=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_FM_EOF_INT_EN, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_FM_EOF_INT_EN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_SMIA_FM_EOF_INT_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_SMIA_FM_EOF_INT_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_P2W_FIFO_WR_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_P2W_FIFO_WR_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_P2W_FIFO_WR_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_P2W_FIFO_WR_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_P2W_FIFO_RD_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_P2W_FIFO_RD_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_P2W_FIFO_RD_STATUS, &read_data);
	pr_info("[CAM]YUSHAN_P2W_FIFO_RD_STATUS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_WRAPPER_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_WRAPPER_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_WRAPPER_THRESH, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_WRAPPER_THRESH=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_WRAPPER_CHAR_EN, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_WRAPPER_CHAR_EN=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_VERSION_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_VERSION_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_NUMBER_OF_LANES, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_NUMBER_OF_LANES=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LANE_MAPPING, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LANE_MAPPING=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_CONTROL, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_CONTROL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_INTERPACKET_DELAY, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_INTERPACKET_DELAY=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_STATUS_LINE_SIZE, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_STATUS_LINE_SIZE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_STATUS_LINE_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_STATUS_LINE_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_VC_CTRL_0, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_VC_CTRL_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_VC_CTRL_1, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_VC_CTRL_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_VC_CTRL_2, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_VC_CTRL_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_VC_CTRL_3, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_VC_CTRL_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_FRAME_NO_0, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_FRAME_NO_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_FRAME_NO_1, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_FRAME_NO_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_FRAME_NO_2, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_FRAME_NO_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_FRAME_NO_3, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_FRAME_NO_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_BYTE_COUNT, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_BYTE_COUNT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_CURRENT_DATA_IDENTIFIER, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_CURRENT_DATA_IDENTIFIER=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DFV, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_0, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_0, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_0, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_1, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_1, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_1, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_2, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_2, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_2, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_3, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_3, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_3, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_4, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_4, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_4, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_5, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_5, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_5, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_5=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_6, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_6, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_6, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_6=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_7, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_7, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_7, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_7=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_8, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_8=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_8, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_8=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_8, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_8=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_9, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_9=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_9, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_9=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_9, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_9=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_10, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_10=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_10, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_10=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_10, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_10=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_11, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_11=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_11, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_11=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_11, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_11=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_12, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_12=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_12, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_12=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_12, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_12=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_13, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_13=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_13, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_13=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_13, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_13=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_14, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_14=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_14, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_14=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_14, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_14=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_PACKET_SIZE_15, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_PACKET_SIZE_15=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_DI_INDEX_CTRL_15, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_DI_INDEX_CTRL_15=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_CSI2_TX_LINE_NO_15, &read_data);
	pr_info("[CAM]YUSHAN_CSI2_TX_LINE_NO_15=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_UIX4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_UIX4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_SWAP_PINS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_SWAP_PINS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_INVERT_HS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_INVERT_HS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_STOP_STATE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_STOP_STATE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_FORCE_TX_MODE_DL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_FORCE_TX_MODE_DL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_ULP_STATE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_ULP_STATE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_ULP_EXIT, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_ULP_EXIT=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_ESC_DL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_ESC_DL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_HSTX_SLEW, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_HSTX_SLEW=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_SKEW, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_SKEW=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_GPIO_CL, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_GPIO_CL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_GPIO_DL1, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_GPIO_DL1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_GPIO_DL2, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_GPIO_DL2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_GPIO_DL3, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_GPIO_DL3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_GPIO_DL4, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_GPIO_DL4=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_SPECS, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_SPECS=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_SLEW_RATE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_SLEW_RATE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_TEST_RESERVED, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_TEST_RESERVED=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_TCLK_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_TCLK_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_MIPI_TX_TCLK_POST_DELAY, &read_data);
	pr_info("[CAM]YUSHAN_MIPI_TX_TCLK_POST_DELAY=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_BYPASS_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_BYPASS_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_BYPASS_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_BYPASS_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_BYPASS_LSTART_LEVEL, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_BYPASS_LSTART_LEVEL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_BYPASS_LSTOP_LEVEL, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_BYPASS_LSTOP_LEVEL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_BYPASS_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_BYPASS_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_BYPASS_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_BYPASS_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_BYPASS_MATCH0, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_BYPASS_MATCH0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_BYPASS_MATCH1, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_BYPASS_MATCH1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_BYPASS_MATCH2, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_BYPASS_MATCH2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_BYPASS_MATCH3, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_BYPASS_MATCH3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_DXO_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_DXO_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_DXO_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_DXO_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_DXO_LSTART_LEVEL, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_DXO_LSTART_LEVEL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LINE_FILTER_DXO_LSTOP_LEVEL, &read_data);
	pr_info("[CAM]YUSHAN_LINE_FILTER_DXO_LSTOP_LEVEL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_DXO_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_DXO_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_DXO_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_DXO_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_DXO_MATCH0, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_DXO_MATCH0=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_DXO_MATCH1, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_DXO_MATCH1=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_DXO_MATCH2, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_DXO_MATCH2=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_DTFILTER_DXO_MATCH3, &read_data);
	pr_info("[CAM]YUSHAN_DTFILTER_DXO_MATCH3=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_PRE_DXO_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_PRE_DXO_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_PRE_DXO_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_PRE_DXO_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_PRE_DXO_AUTOMATIC_CONTROL, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_PRE_DXO_AUTOMATIC_CONTROL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_PRE_DXO_H_SIZE, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_PRE_DXO_H_SIZE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_PRE_DXO_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_LBE_PRE_DXO_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_PRE_DXO_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_LBE_PRE_DXO_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_PRE_DXO_DFV, &read_data);
	pr_info("[CAM]YUSHAN_LBE_PRE_DXO_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_PRE_DXO_H_SIZE, &read_data);
	pr_info("[CAM]YUSHAN_LBE_PRE_DXO_H_SIZE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_PRE_DXO_READ_START, &read_data);
	pr_info("[CAM]YUSHAN_LBE_PRE_DXO_READ_START=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_POST_DXO_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_POST_DXO_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_POST_DXO_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_POST_DXO_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_POST_DXO_AUTOMATIC_CONTROL, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_POST_DXO_AUTOMATIC_CONTROL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_EOF_RESIZE_POST_DXO_H_SIZE, &read_data);
	pr_info("[CAM]YUSHAN_EOF_RESIZE_POST_DXO_H_SIZE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LECCI_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_LECCI_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LECCI_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_LECCI_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LECCI_MIN_INTERLINE, &read_data);
	pr_info("[CAM]YUSHAN_LECCI_MIN_INTERLINE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LECCI_OUT_BURST_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_LECCI_OUT_BURST_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LECCI_LINE_SIZE, &read_data);
	pr_info("[CAM]YUSHAN_LECCI_LINE_SIZE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LECCI_BYPASS_CTRL, &read_data);
	pr_info("[CAM]YUSHAN_LECCI_BYPASS_CTRL=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_POST_DXO_ENABLE, &read_data);
	pr_info("[CAM]YUSHAN_LBE_POST_DXO_ENABLE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_POST_DXO_VERSION, &read_data);
	pr_info("[CAM]YUSHAN_LBE_POST_DXO_VERSION=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_POST_DXO_DFV, &read_data);
	pr_info("[CAM]YUSHAN_LBE_POST_DXO_DFV=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_POST_DXO_H_SIZE, &read_data);
	pr_info("[CAM]YUSHAN_LBE_POST_DXO_H_SIZE=%x\n", read_data);
	rawchip_spi_read_2B2B(YUSHAN_LBE_POST_DXO_READ_START, &read_data);
	pr_info("[CAM]YUSHAN_LBE_POST_DXO_READ_START=%x\n", read_data);

}

/******************************************************************
API Function:	This function will read current status and configuration
				for all Yushan Blocks and dump it in log
				DEBUG MODE at least must de enabled
Input:
Return:
*******************************************************************/
#define YUSHAN_REGISTER_CHECK(reg_addr) udwSpiData = 0; SPI_Read(reg_addr, 4, (uint8_t *)(&udwSpiData)); DEBUGLOG("[CAM] %s: %s: 0x%08x\n", __func__, #reg_addr, udwSpiData);
#define YUSHAN_DOP_REGISTER_CHECK(reg_addr) udwSpiData = 0; SPI_Read(reg_addr + DXO_DOP_BASE_ADDR, 1, (uint8_t *)(&udwSpiData)); DEBUGLOG("[CAM] %s: %s: 0x%02x\n", __func__, #reg_addr, udwSpiData);
#define YUSHAN_PDP_REGISTER_CHECK(reg_addr) udwSpiData = 0; SPI_Read(reg_addr + DXO_PDP_BASE_ADDR, 1, (uint8_t *)(&udwSpiData)); DEBUGLOG("[CAM] %s: %s: 0x%02x\n", __func__, #reg_addr, udwSpiData);
#define YUSHAN_DPP_REGISTER_CHECK(reg_addr) udwSpiData = 0; SPI_Read(reg_addr + DXO_DPP_BASE_ADDR-0x8000, 1, (uint8_t *)(&udwSpiData)); DEBUGLOG("[CAM] %s: %s: 0x%02x\n", __func__, #reg_addr, udwSpiData);
void Yushan_Status_Snapshot(void)
{
	uint32_t	udwSpiData;
	uint32_t	udwSpiBaseIndex;

	VERBOSELOG("[CAM] %s: Start\n", __func__);

	DEBUGLOG("[CAM] %s: **** CLK CONFIG  CHECK ****\n", __func__);
	YUSHAN_REGISTER_CHECK(YUSHAN_CLK_DIV_FACTOR);
	YUSHAN_REGISTER_CHECK(YUSHAN_CLK_DIV_FACTOR_2);
	YUSHAN_REGISTER_CHECK(YUSHAN_CLK_CTRL);
	YUSHAN_REGISTER_CHECK(YUSHAN_PLL_CTRL_MAIN);

	DEBUGLOG("[CAM] %s: **** CSI2 RX INTERFACE  CHECK ****\n", __func__);
	YUSHAN_REGISTER_CHECK(YUSHAN_CSI2_RX_FRAME_NUMBER);
	YUSHAN_REGISTER_CHECK(YUSHAN_CSI2_RX_DATA_TYPE);
	YUSHAN_REGISTER_CHECK(YUSHAN_CSI2_RX_WORD_COUNT);
	YUSHAN_REGISTER_CHECK(YUSHAN_ITM_CSI2RX_STATUS);

	DEBUGLOG("[CAM] %s: **** PRE-DXO CHECK ****\n", __func__);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_0);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_1);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_2);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_3);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_4);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_5);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_6);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_7);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_8);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_9);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_10);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_11);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_12);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_13);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_WC_DI_14);
	YUSHAN_REGISTER_CHECK(YUSHAN_IDP_GEN_CONTROL);
	YUSHAN_REGISTER_CHECK(YUSHAN_ITM_IDP_STATUS);
	YUSHAN_REGISTER_CHECK(YUSHAN_T1_DMA_REG_STATUS);

	DEBUGLOG("[CAM] %s: **** DXO PDP CHECK ****\n", __func__);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_ucode_id_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_ucode_id_15_8);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_hw_id_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_hw_id_15_8);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_calib_id_0_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_calib_id_1_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_calib_id_2_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_calib_id_3_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_error_code_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_frame_number_7_0);
	YUSHAN_PDP_REGISTER_CHECK(DxOPDP_frame_number_15_8);

	DEBUGLOG("[CAM] %s: **** DXO DPP CHECK ****\n", __func__);
	udwSpiBaseIndex = 0x010000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_ucode_id_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_ucode_id_15_8);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_hw_id_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_hw_id_15_8);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_calib_id_0_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_calib_id_1_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_calib_id_2_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_calib_id_3_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_error_code_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_frame_number_7_0);
	YUSHAN_DPP_REGISTER_CHECK(DxODPP_frame_number_15_8);
	udwSpiBaseIndex = 0x08000;
	SPI_Write(YUSHAN_HOST_IF_SPI_BASE_ADDRESS, 4, (uint8_t *)(&udwSpiBaseIndex));

	DEBUGLOG("[CAM] %s: **** DXO DOP CHECK ****\n", __func__);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_ucode_id_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_ucode_id_15_8);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_hw_id_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_hw_id_15_8);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_calib_id_0_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_calib_id_1_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_calib_id_2_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_calib_id_3_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_error_code_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_frame_number_7_0);
	YUSHAN_DOP_REGISTER_CHECK(DxODOP_frame_number_15_8);

	VERBOSELOG("[CAM] %s: End\n", __func__);
}

