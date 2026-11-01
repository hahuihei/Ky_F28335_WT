//----------------------------------------------------------------------------------
//	FILE:			{ProjectName}-Settings.h
//
//	Description:    This file contains the definitions for this project, and is 
//					linked to both {ProjectName}-Main.c and {ProjectName}-DPL-ISR.asm 
//					(where X is the project name).  
//
//	Type: 			Device Independent
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments ? 2010
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 9 April 2010 - MB
//----------------------------------------------------------------------------------

#ifndef _KY_SETTINGS_H
#define _KY_SETTINGS_H

//**********************************************************************************
//  NOTE: WHEN CHANGING THIS FILE PLEASE REBUILD ALL
//**********************************************************************************

//==================================================================================
// System Settings
//----------------------------------------------------------------------------------
#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI          3.14159265358979
#define EXPF        0                   //1-扩展帧ID 0-标准帧ID
#define GRID_FREQ   50
#define KvInv       _IQ15(375.0)
#define VdcRef      (0.75)

#define CPU_FREQ 150000000
//DCDC
#define FLYBACK_FREQ    40000.0                         //DCDC PWM频率40kHz
#define FLYBACK_PWM_PERIOD (CPU_FREQ/(FLYBACK_FREQ*2))  //DCDC PWM周期值CPU_FREQ/(FLYBACK_FREQ*2)=1875
#define BOOST_PWM_PERIOD 1875                           //Boost PWM 40kHz
//0.13us死区(20tick@150MHz)。SiC+UCC21520关断约60ns，20tick约2倍余量，不要再小于15
#define FLYBACK_DEADBAND_RED 20
#define FLYBACK_DEADBAND_FED 20

#define Vbus_SENSE_MAX      375.0
#define IGrid_SENSE_MAX     51.6        //全桥电流采样方式-AC峰值采样电流

// Define the system frequency (MHz)
#define SYSTEM_FREQUENCY 150

// ****************************************************************************
// ---- SPI peripherals - functional usage ----
// ============================================
#define  SYS_SPI  SpiaRegs

// ****************************************************************************
#define  TWO_PI            (2*PI)
#define  SAMPLING_FREQ     10000  /*  in Hz  */
#define  DELAY_LENGTH      16

//===========================================================================
/* The below macro sets the ADC sampling instant within every EPWM PRD.
 * To fine tune it, declare it as int32 variable and try experimenting
 * with different values in debug session
 */
#define   ADC_SAMP_INST           750    /* adc sampling instant set up  */
//===========================================================================

#define   TIMECNT_1uS             SYSTEM_FREQUENCY     /* Sys freq in MHz */
#define   TIMECNT_6p25uS         (TIMECNT_1uS * 6.25)
#define   TIMECNT_100uS          (TIMECNT_1uS * 100)

#define   SINE_CARRIER           (250*2.5)  /* 2.5 for Delfino,  1 for Piccolo  */

// ****************************************************************************
#define   RUN_LED()  {GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;}

#define     mPWM3_ON()  {EALLOW; \
                        EPwm3Regs.TZCLR.bit.OST = 1; \
                        EDIS;}

#define     mPWM3_OFF() {EALLOW; \
                        EPwm3Regs.TZFRC.bit.OST = 1; \
                        EDIS;}

#define     mPWM4_ON()  {EALLOW; \
                        EPwm4Regs.TZCLR.bit.OST = 1; \
                        EDIS;}

#define     mPWM4_OFF() {EALLOW; \
                        EPwm4Regs.TZFRC.bit.OST = 1; \
                        EDIS;}

#endif //_PROJSETTINGS_H
