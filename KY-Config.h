/* ==================================================================================
File name:        CONFIG.H

Originator:	Digital Control Systems Group
			Texas Instruments
Description:Header file containing definitions for CONFIG.C

Target: TMS320F2833x family
=====================================================================================
History:
-------------------------------------------------------------------------------------
 04-15-2012	Version 1.0:
------------------------------------------------------------------------------------*/
#ifndef CONFIG_H_
#define CONFIG_H_

#include "DCLF32.h"

#define TFT     1
#define Grid    0   //1-Grid(PLL锁市电) 0-Inv(内部SGEN 50Hz，开环方波输出用0)

#define SPWM_ON {EALLOW; \
                        EPwm1Regs.TZCLR.bit.OST = 1;\
                        EPwm2Regs.TZCLR.bit.OST = 1;\
                        EDIS;}
#define SPWM_OFF {EALLOW; \
                        EPwm1Regs.TZFRC.bit.OST = 1;\
                        EPwm2Regs.TZFRC.bit.OST = 1;\
                        EDIS;}

// functions defined in "config.c"
void eADC_SOC_CNF2(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);
void ADC_INIT(volatile struct ADC_REGS *v);
void ePWM_1ch_CNF(volatile struct EPWM_REGS *v, int16 period, int16 mode, int16 phase);
void InitGpios(void);
void InitScicGpio();
void spi_init(volatile struct SPI_REGS *v);

void SmallRelayStart(void);
void RelayStart(void);
void RelayEfficient(void);
void RelayStop(void);
void FanStart(void);
void FanStop(void);

float DCL_runPI_L4(DCL_PI *p, float rk, float yk);

#define MATH_EMAVG2_F_MACRO(In, Out, Mul) Out = ((In - Out)*Mul + Out);

#endif /*CONFIG_H_*/
