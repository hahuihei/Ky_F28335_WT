//----------------------------------------------------------------------------------
//	FILE:			PWM_2ch_UpDwnCnt_Cnf.c
//
//	Description:	180 degree offset A and B channel PWM output configuration function
//					Symmetric PWM with symmetric Dead-Band
//					Configures the PWM channel in UP-DOWN Count mode. 
//
//	Version: 		1.0
//
//  Target:  		TMS320F2802x, 
//					TMS320F2803x, 
//
// The function call is:
//
// 		PWM_2ch_UpDwnCntDB_CNF(int16 n, int16 period, int16 mode, int16 phase)
//
// Function arguments defined as:
//-------------------------------
// n = 		Target ePWM module, 1,2,...16.  e.g. if n=2, then target is ePWM2
// period = PWM period in Sysclks
// mode =	Master/Slave mode, e.g. mode=1 for master, mode=0 for slave
// phase =	phase offset from upstream master in Sysclks,
//			applicable only if mode=0, i.e. slave
//--------------------------------------------------------------------------------
#include "PeripheralHeaderIncludes.h"
#include "DSP2833x_EPWM_defines.h"

extern volatile struct EPWM_REGS *ePWM[];

void PWM_2ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 mode, int16 phase) {
	// Time Base SubModule Registers	
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_SHADOW;				// set Shadow load
	(*ePWM[n]).TBPRD = period / 2;						// half-period due to up-down count
	(*ePWM[n]).TBPHS.half.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV = TB_DIV1;

	if(mode == 1) { // config as a Master
		(*ePWM[n]).TBCTL.bit.PHSEN = TB_DISABLE;
		(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; 	// sync "down-stream" 
	}
	if(mode == 0) { // config as a Slave (Note: Phase+2 value used to compensate for logic delay)
		(*ePWM[n]).TBCTL.bit.PHSEN = TB_ENABLE;
		(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

		if ((0 <= phase) && (phase <= 2)) {
			(*ePWM[n]).TBPHS.half.TBPHS = (2 - phase);
			(*ePWM[n]).TBCTL.bit.PHSDIR = TB_UP;		// set to count up after sync
		}
		else if ((2 < phase) && (phase <= (period / 2) + 2)) {
			(*ePWM[n]).TBPHS.half.TBPHS = (phase - 2);
			(*ePWM[n]).TBCTL.bit.PHSDIR = TB_DOWN;		// set to count down after sync
		}
		else if (((period / 2) + 2 < phase) && (phase <= period)) {
			(*ePWM[n]).TBPHS.half.TBPHS = (period - phase + 2);
			(*ePWM[n]).TBCTL.bit.PHSDIR = TB_UP;		// set to count up after sync
		}
	}
	
	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.half.CMPA = 0;			// set duty 50% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_SET;
	(*ePWM[n]).AQCTLA.bit.CAD = AQ_CLEAR;

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPB = 0;						// set duty 50% initially
	(*ePWM[n]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADBMODE = CC_CTR_PRD;
	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLB.bit.CBU = AQ_CLEAR;
	(*ePWM[n]).AQCTLB.bit.CBD = AQ_SET;
}

void PWM_2ch_UpDwnCnt_UpdateDB(int16 n, int16 dbRED, int16 dbFED) {
	(*ePWM[n]).DBRED=dbRED;
	(*ePWM[n]).DBFED=dbFED;
}

//DCDC全桥使用的PWM配置
void PWM_1ch_UpDwnCnt_LLC(int16 n, Uint16 period,int deadband_rising,int deadband_falling)
{
    (*ePWM[n]).TBPRD = period/2;                     // Set timer period
    (*ePWM[n]).TBPHS.half.TBPHS = 0x0000;            // Phase is 0
    (*ePWM[n]).TBCTR = 0x0000;                       // Clear counter

    // Setup compare
    (*ePWM[n]).CMPA.half.CMPA = 5;
    (*ePWM[n]).CMPB = period/2-5;

    // Setup TBCLK
    (*ePWM[n]).TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    (*ePWM[n]).TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    (*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    (*ePWM[n]).TBCTL.bit.CLKDIV = TB_DIV1;          // Slow so we can observe on the scope

    //---------------------------------------------------------------
    //以下AB通道是输出独立的PWM设置
    // Setup shadowing CMPCTL计数器比较控制寄存器
    (*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[n]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    (*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    (*ePWM[n]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    //(*ePWM[n]).AQCTLA.bit.CAU = AQ_SET;           // Set PWM3A on CAU
    //(*ePWM[n]).AQCTLA.bit.CAD = AQ_CLEAR;         // Clear PWM3A on CAD

    //(*ePWM[n]).AQCTLB.bit.CAU = AQ_CLEAR;         // Clear PWM3B on CAU
    //(*ePWM[n]).AQCTLB.bit.CAD = AQ_SET;           // Set PWM3B on CAD

    (*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;           // Set PWM1A on CAU
    (*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;             // Clear PWM1A on CAD

    (*ePWM[n]).AQCTLB.bit.CBU = AQ_SET;             // Clear PWM3B on CAU
    (*ePWM[n]).AQCTLB.bit.CBD = AQ_CLEAR;           // Set PWM1B on CAD

    // Active high complementary PWMs - Setup the deadband
    //(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //(*ePWM[n]).DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //(*ePWM[n]).DBCTL.bit.IN_MODE = DBA_ALL;
    //---------------------------------------------------------------

    (*ePWM[n]).DBRED = deadband_rising;
    (*ePWM[n]).DBFED = deadband_falling;

   // Interrupt where we will change the deadband
    (*ePWM[n]).ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
    (*ePWM[n]).ETSEL.bit.INTEN = 1;                  // Enable INT
    (*ePWM[n]).ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 3rd event

    EDIS;
}
