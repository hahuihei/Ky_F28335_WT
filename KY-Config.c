//----------------------------------------------------------------------------------
//  FILE:           Resolver-Main.C
//
//  Description:    Contains all peripheral configurations
//
//  Version:        1.0
//
//  Target:         TMS320F2833x(Delfino)
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments ? 2004-2012
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date      | Description / Status
//----------------------------------------------------------------------------------
//
#include "PeripheralHeaderIncludes.h"
#include "DSP2833x_EPWM_defines.h"
#include "KY-Config.h"
//--------------------------------------------------------------------------
//  Description:    Single (A output) channel PWM configuration function
//                  Configures the PWM channel in UP Count mode.
//
// period = PWM period in Sysclks
// mode =   Master/Slave mode, e.g. mode=1 for master, mode=0 for slave
// phase =  phase offset from upstream master in Sysclks,
//          applicable only if mode=0, i.e. slave
//--------------------------------------------------------------------------
void ePWM_1ch_CNF(volatile struct EPWM_REGS *v, int16 period, int16 mode, int16 phase)
{
    ///*
    v->TBPRD=period;
    v->TBPHS.half.TBPHS=0;
    v->TBCTR = 0x0000;
    v->TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;
    v->TBCTL.bit.PHSDIR = TB_DOWN;
    v->TBCTL.bit.PHSEN=TB_DISABLE;
    v->TBCTL.bit.PRDLD=TB_SHADOW;           //开启TBPRD的shadow寄存器 //TBCLK=SYSCLK/(HSPCLKDIV*CLKDIV)
    v->TBCTL.bit.HSPCLKDIV=TB_DIV1;         //对应HSPCLKDIV=1
    v->TBCTL.bit.CLKDIV=TB_DIV1;            //对应CLKDIV=1
    v->TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;
    v->CMPCTL.bit.SHDWAMODE=CC_SHADOW;
    v->CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;
    v->AQCTLA.bit.CAD=AQ_SET;               //到达比较值时电平为高
    v->AQCTLA.bit.CAU=AQ_CLEAR;             //到达比较值时电平为低
    v->AQCTLA.bit.ZRO = AQ_NO_ACTION;
    v->AQCTLA.bit.PRD = AQ_NO_ACTION;

    v->CMPA.half.CMPA=0.5 * period;
    v->DBCTL.bit.IN_MODE=0;                 //双边沿延时
    v->DBCTL.bit.OUT_MODE=3;                //使能双边沿延时
    v->DBRED=145;
    v->DBFED=145;
    v->DBCTL.bit.POLSEL=2;                  //A不翻转，B翻转
    //*/
}

/*------------------------------------------------------------------------------
 ADC Initialization Macro Definition 
------------------------------------------------------------------------------*/

extern void DSP28x_usDelay(signed long Count);

#define  CPU_CLOCK_SPEED      6.6667L       // 10.000L for a 100MHz CPU clock speed
#define  ADC_usDELAY          50000L
#define  DELAY_US(A)          DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_CLOCK_SPEED) - 9.0L) / 5.0L)
#define  ADC_cal_ptr          (void (*) (void)) 0x380080

#pragma CODE_SECTION(ADC_INIT,"ramfuncs");
void ADC_INIT(volatile struct ADC_REGS *v)
{
    (*ADC_cal_ptr)();                       // call ADC_cal() from OTP - ADCCLK must have been already enabled

    DELAY_US(10000L);                       // Delay before converting ADC channels
    v->ADCTRL3.all = 0x00E0;                // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);                  // Delay before converting ADC channels

    // ADC clock frequency control
    v->ADCTRL3.bit.ADCCLKPS = 1;
    v->ADCTRL1.bit.CPS      = 1;

    // signal acquisition time control
    v->ADCTRL1.bit.ACQ_PS = 6;

    // sequencer initialisation
    v->ADCTRL2.bit.INT_ENA_SEQ1 = 0;
    v->ADCTRL2.bit.RST_SEQ1     = 1;
    v->ADCTRL2.bit.RST_SEQ2     = 1;

    //ADC configuration
    v->ADCTRL3.bit.SMODE_SEL    = 1;        // simultaneous sampling 同步采样
    v->ADCTRL1.bit.SEQ_CASC     = 1;        // Cascaded Mode
    v->ADCMAXCONV.bit.MAX_CONV1 = 0x7;

    v->ADCCHSELSEQ1.bit.CONV00=0x0;         // A0 & B0 --> AdcRegs.ADCRESULT0 & AdcRegs.ADCRESULT1
    v->ADCCHSELSEQ1.bit.CONV01=0x1;         // A1 & B1 --> AdcRegs.ADCRESULT2 & AdcRegs.ADCRESULT3
    v->ADCCHSELSEQ1.bit.CONV02=0x2;         // A2 & B2 --> AdcRegs.ADCRESULT4 & AdcRegs.ADCRESULT5
    v->ADCCHSELSEQ1.bit.CONV03=0x3;         // A3 & B3 --> AdcRegs.ADCRESULT6 & AdcRegs.ADCRESULT7

    v->ADCCHSELSEQ2.bit.CONV04=0x4;         // A4 & B4 --> AdcRegs.ADCRESULT8 & AdcRegs.ADCRESULT9
    v->ADCCHSELSEQ2.bit.CONV05=0x5;         // A5 & B5 --> AdcRegs.ADCRESULT10 & AdcRegs.ADCRESULT11
    v->ADCCHSELSEQ2.bit.CONV06=0x6;         // A6 & B6 --> AdcRegs.ADCRESULT12 & AdcRegs.ADCRESULT13
    v->ADCCHSELSEQ2.bit.CONV07=0x7;         // A7 & B7 --> AdcRegs.ADCRESULT14 & AdcRegs.ADCRESULT15

    // enable ADC interrupt
    v->ADCTRL2.bit.INT_ENA_SEQ1 = 1;        // enable SEQ1 INT
    v->ADCST.bit.INT_SEQ1_CLR   = 1;        // clear pending SEQ1_INT
    EDIS;

    return;
}

void InitGpios(void)
{
    EALLOW;

    //LED1
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;        //GPIO口,LED1
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;         //方向为输出
    GpioDataRegs.GPBSET.bit.GPIO51 = 1;         //输出高电平

    //LED2
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;        //GPIO口,LED2
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;         //方向为输出
    GpioDataRegs.GPBSET.bit.GPIO50 = 1;         //输出高电平

    //LED3
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;        //GPIO口,LED3
    GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;         //方向为输出
    GpioDataRegs.GPBSET.bit.GPIO49 = 1;         //输出高电平

    //风扇输出测试口，使用的PWM模式，在此不用IO模式
    //GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;      //风扇输出测试口
    //GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;       //上拉电阻使能
    //GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;       //方向为输出
    //GpioDataRegs.GPASET.bit.GPIO14 = 1;       //输出高电平

    //驱动芯片Nsi6602B DISABLE信号
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;        //驱动芯片Nsi6602B DISABLE信号
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;         //方向为输出
    GpioDataRegs.GPBSET.bit.GPIO48 = 1;         //输出高电平，直接使能Nsi6602B

    //主继电器启动信号
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;        //主继电器启动信号
    GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO36 = 1;       //输出低电平

    //主继电器维持信号
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;        //主继电器维持信号
    GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO37 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO37 = 1;       //输出低电平

    //辅助继电器母线充电启动信号
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;        //辅助继电器启动信号
    GpioCtrlRegs.GPBPUD.bit.GPIO38 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO38 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO38 = 1;       //输出低电平

    //A相电流保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;        //A相电流保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;         //方向为输入

    //B相电流保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;        //B相电流保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;         //方向为输入

    //C相电流保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;        //C相电流保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0;         //方向为输入

    //A相电压保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;        //A相电压保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;         //方向为输入

    //B相电压保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;        //B相电压保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0;         //方向为输入

    //C相电压保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;        //C相电压保护信号，正常运行输入为高电平，保护时为低电平
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 0;         //方向为输入

    //A相电压保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;        //A相电压保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;       //输出低电平

    //B相电压保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;        //B相电压保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;       //输出低电平

    //C相电压保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;        //C相电压保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;       //输出低电平

    //A相电流保护解锁信号，高电平解锁
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;        //A相电流保护解锁信号，高电平解锁
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;         //方向为输出
    GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;       //输出低电平

    //B相电流保护解锁信号，高电平解锁
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;        //B相电流保护解锁信号，高电平解锁
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;         //方向为输出
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;       //输出低电平

    //C相电流保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;        //C相电流保护解锁信号，高电平解锁
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;         //方向为输出
    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;       //输出低电平

    //Test
    GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO70 = 0;         //上拉电阻使能
    GpioCtrlRegs.GPCDIR.bit.GPIO70 = 1;         //方向为输出
    //GpioDataRegs.GPCCLEAR.bit.GPIO70 = 1;     //输出低电平
    GpioDataRegs.GPCSET.bit.GPIO70 = 1;         //输出高电平

    EDIS;
}

void InitScicGpio()
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled disabled by the user.
    // This will enable the pullups for the specified pins.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;  // Enable pull-up for GPIO62 (SCIRXDC)
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;  // Enable pull-up for GPIO63 (SCITXDC)

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.
    //
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

    //
    // Configure SCI-C pins using GPIO regs
    // This specifies which of the possible GPIO pins will be SCI functional
    // pins.
    //
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 to SCIRXDC
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 to SCITXDC

    EDIS;
}

//启动缓冲继电器，开始对母线充电
void SmallRelayStart(void)
{
    EALLOW;
    GpioDataRegs.GPBSET.bit.GPIO38 = 1;
    EDIS;
}
//主继电器启动
void RelayStart(void)
{
    EALLOW;
    GpioDataRegs.GPASET.bit.GPIO20 = 1;         //启动信号
    EDIS;
}
//节能的同时关闭辅助继电器
void RelayEfficient(void)
{
    EALLOW;
    GpioDataRegs.GPBCLEAR.bit.GPIO36 = 1;       //主继电器启动信号
    GpioDataRegs.GPBCLEAR.bit.GPIO38 = 1;       //辅助继电器关闭
    EDIS;
}
//关闭所有继电器
void RelayStop(void)
{
    EALLOW;
    GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;
    EDIS;
}

void FanStart(void)
{
    //
    EALLOW;
    //EPwm8Regs.TZCLR.bit.OST = 1;
    EDIS;
}

void FanStop(void)
{
    EALLOW;
    //EPwm8Regs.TZFRC.bit.OST = 1;
    EDIS;
}

//*****************************************************************************
// To interface with Motor controller to provide position feedback
//------------------------------------------------------------------
void spi_init(volatile struct SPI_REGS *v)
{
    // Initialize SPI FIFO registers
    v->SPIFFTX.bit.SPIRST     = 1;          // enable fifo - 0xE040;
    v->SPIFFTX.bit.SPIFFENA   = 1;          // enable fifo enhancements
    v->SPIFFTX.bit.TXFIFO     = 1;          // re-enable tx fifo
    v->SPIFFTX.bit.TXFFIENA   = 0;          // disable tx fifo int based on txffil match
    v->SPIFFTX.bit.TXFFINTCLR = 1;          // clr TXFFINT flag in SPIFFTX
    
    //v->SPIFFRX.bit.RXFFOVFCLR  = 1;       // clr RXFFOVF flag in SPIFFRX - 0x2044;
    v->SPIFFRX.bit.RXFIFORESET = 1;         // re-enable rx fifo
    v->SPIFFRX.bit.RXFFINTCLR  = 1;         // clr RXFFINT flag in SPIFFRX
    v->SPIFFRX.bit.RXFFIENA    = 0;         // disable rx fifo int based on rxffil match
    //v->SPIFFRX.bit.RXFFIL      = 2;       // two rx data
    
    v->SPIFFCT.all=0x0;                     // no time space between consecutive words in a packet
    
    // Initialize SPI
    v->SPICCR.bit.SPISWRESET  = 0;          // reset SPI - 0x000F
    v->SPICCR.bit.SPILBK      = 0;          // no loop back
    v->SPICCR.bit.CLKPOLARITY = 0;          // clk polarity is rising edge
    v->SPICCR.bit.SPICHAR     = 0xf;        // data length = 16b

    v->SPICTL.bit.OVERRUNINTENA = 0;        // disable rx overrun flag bit interrupts (SPISTS.7)
    v->SPICTL.bit.CLK_PHASE     = 0;        // Enable normal phase - 0x0002
    v->SPICTL.bit.MASTER_SLAVE  = 1;        // master mode
    v->SPICTL.bit.TALK          = 1;        // enable talk
    v->SPICTL.bit.SPIINTENA     = 0;        // SPI int disable
    
    v->SPIBRR = 9;                          // Baud rate

    v->SPICCR.bit.SPISWRESET    = 1;        // Relinquish SPI from Reset
    
    v->SPIPRI.bit.FREE          = 1;        // Set so breakpoints don't disturb xmission
    //v->SPIPRI.bit.STEINV        = 0;      // SPISTE pin in normal mode (no inversion)
    //v->SPIPRI.bit.TRIWIRE       = 0;      // 4 wire SPI
    
    return;
}

float DCL_runPI_L4(DCL_PI *p, float rk, float yk)
{
    float v1, v2, v4, v5, v9;

    //asm(" MSETFLG   RNDF32=1");1为就近舍入向偶舍入，0为截断舍入，
    //所以如果DCL_runPI_L4用在CLA中必须加入__asm(" MSETFLG RNDF32=1")，
    //如果不用在CLA中而是用在FPU中就去掉这个代码。
    //asm(" MSETFLG   RNDF32=1");
    v1 = rk - yk;
    v2 = p->Kp * v1;
    v4 = (v1 * p->Ki * p->i6) + p->i10;
    p->i10 = v4;
    v5 = v2 + v4;
    v9 = (v5 > p->Umax) ? p->Umax : v5;
    v9 = (v9 < p->Umin) ? p->Umin : v9;
    p->i6 = (v5 == v9) ? 1.0f : 0.0f;

    return(v9);
}

//*****************************************************************************
