//----------------------------------------------------------------------------------
//  FILE:           KY-Main.C
//
//  Description:    F28335 无线充电驱动程序
//
//  Version:        1.2
//
//  Target:         TMS320F28335
//
//----------------------------------------------------------------------------------
//  版权所有kypine VX13550290166 20260415
//----------------------------------------------------------------------------------
// Although this project is made up of several files, the most important ones are:
//   "KY-Main.C"    - this file
//      - Application Initialization, Peripheral config,
//      - Application management
//      - Slower background code loops and Task scheduling
//   "KY-DevInit_F28335.C
//      - Device Initialization, e.g. Clock, PLL, WD, GPIO mapping
//      - Peripheral clock enables
//      - DevInit file will differ per each F28335 device series, e.g. F280x, F2833x,
//   "KY-Settings.h"
//      - Global defines (settings) project selections are found here
//      - This file is referenced by both C and ASM files.

#include "PeripheralHeaderIncludes.h"
#include "DSP2833x_EPwm_defines.h"
#include "DSP2833x_Device.h"
#include "IQmathLib.h"
#include <math.h>
#include <string.h>
#include "KY-Settings.h"
#include "KY-Config.h"
#include "KY_pbdata.h"
#include "KY_SCI_Uart.h"
#include "KY_CAN_Config.h"
#include "sgen.h"
#include "SineAnalyzer_diff.h"
#include "SPLL_1ph.h"
#include "pid_grando.h"
#include "PWM_1phInv_unipolar.h"
#include "CNTL_3P3Z_F.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);  //state A0
void B0(void);  //state B0
void C0(void);  //state C0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2

// B branch states
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

// C branch states
void C1(void);  //state C1
void C2(void);  //state C2

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch
//----------------------------------------------------------------------------------
void DeviceInit(void);
#ifdef FLASH
    void InitFlash();
    void MemCopy();
#endif

//-------------------------------- CONFIG -------------------------------------------

// --- project functions ---
extern void DSP28x_usDelay(unsigned long Count);

void EPWM1_ISR(void);
void ADC_ISR(void);

void PWM_2ch_UpDwnCnt_CNF(int16 n, int16 period, int16 mode, int16 phase);
void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 mode, int16 phase,int deadband_rising,int deadband_falling);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// -------------------------------- FRAMEWORK --------------------------------------

int16   VTimer0[4],                 // Virtual Timers slaved off CPU Timer 0
        VTimer1[4],                 // Virtual Timers slaved off CPU Timer 1
        VTimer2[4];                 // Virtual Timers slaved off CPU Timer 2

// Used for running BackGround in flash, and ISR in RAM
//extern Uint16 * RamfuncsLoadStart, * RamfuncsLoadEnd, * RamfuncsRunStart;

// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] =
                  { &EPwm1Regs,         //intentional: (ePWM[0] not used)
                    &EPwm1Regs,
                    &EPwm2Regs,
                    &EPwm3Regs,
                    &EPwm4Regs,
                    &EPwm5Regs,
                    &EPwm6Regs
                  };

//--------------------------------- SINE GEN LIB ------------------------------------
SGENHP_2 sgen = SGENHP_2_DEFAULTS;

PID_GRANDO_CONTROLLER   pidGRANDO_Iinv = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER   pidGRANDO_Vinv = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};

// ------------- Sine Analyzer Block to measure RMS, frequency and ZCD ---------
SineAnalyzer_diff sine_mainsV = SineAnalyzer_diff_DEFAULTS;

// ------------- Software PLL for Grid Tie Applications ------------------------------
SPLL_1ph spll1;

int16 PVInverterState;
int16 CloseVloopInv;
int16 CloseIloopInv;
int16 ClearInvTrip;
int16 ResetPLL;

int16 SlewClrInvTrip;

//float inv_ref_vol_inst,inv_meas_vol_inst;
_iq24 inv_ref_vol_inst,inv_meas_vol_inst;
float inv_meas_vol;
float inv_ref_cur_inst, inv_meas_cur_inst,inv_emavg_cur;
float inv_meas_cur_lleg1_inst;
float inv_meas_cur_lleg2_inst;
float InvModIndex;
float inv_Iset,inv_Iset_ref;
float InvSine;
float duty=0.0;
float Grid_Freq=GRID_FREQ;
float Vac_in;
float Offset_Volt;
float ScaleFactor;

Uint16 ZCDDetect=0;
int16 sine_prev=0;

float  V165,Vac_FB,Vbus_FB;
float  Ileg1_fb;
float  Ileg2_fb;
float  vbus_meas_inst;

int16 CloseIloopInv, ClearInvTrip,CloseFlybackLoop, ClearFlyBackTrip, RlyConnect;
float duty_flyback_pu, duty_flyback_pu2, duty_inv_pu;
int16  duty_flyback, duty_flyback2, duty_inv;
int16  phase_flyback;
int16  PERIOD;

//------------------------ Display Values ------------------------------------------
// Monitor ("Get")                      // Display as:
float  Gui_Vpv;
float  Gui_Ipv;
float  Gui_Vbus;
float  Gui_Iavg;
long   Gui_Vrms;

// CNTL 3p3z 用于逆变控制环，运行在RAM
#pragma DATA_SECTION(cntl3p3z_InvI_vars,"data")
#pragma DATA_SECTION(cntl3p3z_InvI_coeff,"dataRAM")
CNTL_3P3Z_F_VARS cntl3p3z_InvI_vars;
CNTL_3P3Z_F_COEFFS cntl3p3z_InvI_coeff;

DCL_PI Udc_PI;
float Vdcref;                   //电压闭环用的
float Vdcexp;                   //母线前馈电压限定值

//*****************************************************************************
//----------------------------------------------------------------------------------------------------------
//SCI
//#define COMM_ID       0xA2                    //串口命令识别地址
#define COMM_ID         0xEE                    //串口命令识别地址
#define ID_I            0                       //串口命令第0位
#define CMD_I           1                       //串口命令第1位
#define ADDR_I          2                       //串口命令第2位
#define DATA_I          3                       //串口命令第3位
#define DATB_I          4                       //串口命令第4位
Uint16 send_timer=0;                            //LCD数据发送时间
Uint16 timerNum=10;                             //发送间隔变量
Uart_Msg SCI_Msg={0, {0},0,0, 0};
unsigned char HandleCommAG(void);
void sendTFT(float32 U,Uint16 SID,Uint16 ID);
void sci_tft(void);

char cPV_State[40];
float SET_VAL=2.0;                              // 默认设置值
//----------------------------------------------------------------------------------------------------------

//测试用的
Uint16 ky1=0;
Uint16 ky2=0;
Uint16 ky3=0;

//----------------------------------------------------------------------------------------------------------
//ADC
char ADCflag = 0;

//TEMP 温度
Uint16 Rvt1=0;
float Rv1=32.116;
Uint16 Rvt2=0;
float Rv2=32.116;

//CANA
CAN_MSG sendMsg;
Uint16 sendData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
Uint16 CAN_SendCount = 0;

Uint16  Gui_InvStart;
Uint16  Gui_InvStop;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// MAIN CODE - starts here
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main(void)
{
    DeviceInit();   // Device Life support & GPIO
    //InitGpios();  // 在DeviceInit中有GPIO初始化
//-------------------------------- FRAMEWORK --------------------------------------

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler with -d FLASH

#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
    InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)

//====================================================================================
// Initializing variables
//====================================================================================


//====================================================================================
// Timing sync for background loops
//====================================================================================
// Timer period definitions found in PeripheralHeaderIncludes.h
    CpuTimer0Regs.PRD.all =  mSec5;     // A tasks
    CpuTimer1Regs.PRD.all =  mSec10;    // B tasks
    CpuTimer2Regs.PRD.all =  mSec1;     // C tasks

// Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;
    C_Task_Ptr = &C1;

    VTimer0[0] = 0;
    VTimer1[0] = 0;
    VTimer2[0] = 0;

//====================================================================================
// Initialize PWM DAC
//====================================================================================

//==============================================================
// Initialize EPWMs needed for resolver
//==============================================================
    //ePWM_1ch_CNF(&EPwm1Regs, 3750, 1, 0);               // 3750 20k
    // configure the PWM1 for the inverter driver
    // 20Khz = (150Mhz/20Khz)/2 = 3750
    // From the datasheet of the mosfet used the Ton and Toff is 264ns
    // therefore 150 Mhz => ~40 Ticks
    PWM_1phInv_unipolar_CNF(1,3750,45,45);

    SPWM_OFF;
    // 互补的 PWM3 100kHz
    PWM_1ch_UpDwnCnt_CNF(3, BOOST_PWM_PERIOD*2, 1, (CPU_FREQ / (FLYBACK_FREQ * 2)) - 2,FLYBACK_DEADBAND_RED,FLYBACK_DEADBAND_FED);
    // 互补的 PWM4 100kHz
    PWM_1ch_UpDwnCnt_CNF(4, BOOST_PWM_PERIOD*2, 0, (CPU_FREQ / (FLYBACK_FREQ * 2)) - 2,FLYBACK_DEADBAND_RED,FLYBACK_DEADBAND_FED);

    // 输出方波 100kHz：150MHz/(2*750) = 100kHz
    EPwm3Regs.TBPRD = 750;
    EPwm4Regs.TBPRD = 750;

    // 强制两路严格同相位：停TBCLK -> 计数器同时清0 -> 同时放开（不依赖EPWM3->EPWM4同步链路）
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm4Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm3Regs.TBCTR = 0;
    EPwm4Regs.TBCTR = 0;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
    PERIOD = BOOST_PWM_PERIOD;
    mPWM3_OFF();
    mPWM4_OFF();

    //驱动电源50kHz
    PWM_2ch_UpDwnCnt_CNF(5, 1500, 1, 0);
//==============================================================
// ADC initialization
//==============================================================
    // Configure the Start of Conversion for the ADC.
    // configure ch3 to trigger ADC INT 2 and wait for clr flg to re-trigger of the ADC INT
    ADC_INIT(&AdcRegs);

    // Set up ADC SOC trigger
    EALLOW;
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x1;           // enable SOC from EPWMA trigger
    EDIS;

//====================================================================================
// Initializing SPI
//====================================================================================
    spi_init(&SYS_SPI);

    // CAN模块初始化
    InitECanA();

    scic_init();
    strcpy(cPV_State,"就绪");

    //Inverter

    // Signal Generator module initialisation
    sgen.offset=0;
    sgen.gain=0x7fff;           // gain=1 in Q15
    sgen.freq=0x14F8CF92;       // freq = (Required Freq/Max Freq)*2^31
                                // = (50/305.17)*2^31 = 0x14f8cf92
    sgen.step_max=0x3E7FB26;    // Max Freq= (step_max * sampling freq)/2^32
                                // =(0x3E7FB26*20k)/2^32 = 305.17
    sgen.phase=0x80000000;      // Phase= (required Phase)/180 in Q31 format
                                //      =  (+90/180) in Q31 = 8000h

     //sine analyzer initialization
    sine_mainsV.Vin=0;
    sine_mainsV.SampleFreq=_IQ15(20000.0);
    sine_mainsV.Threshold=0.0;

    //CNTL3P3Z Inverter Current Loop  CNTL3P3Z 逆变器电流环
    CNTL_3P3Z_F_VARS_init(&cntl3p3z_InvI_vars);
    CNTL_3P3Z_F_COEFFS_init(&cntl3p3z_InvI_coeff);
    cntl3p3z_InvI_coeff.Coeff_A1 = (1.584);
    cntl3p3z_InvI_coeff.Coeff_A2 = (-0.6978);
    cntl3p3z_InvI_coeff.Coeff_A3 = (0.1137);
    cntl3p3z_InvI_coeff.Coeff_B0 = (0.2866);
    cntl3p3z_InvI_coeff.Coeff_B1 = (-0.3173);
    cntl3p3z_InvI_coeff.Coeff_B2 = (0.3338);
    cntl3p3z_InvI_coeff.Coeff_B3 = (-0.2616);
    cntl3p3z_InvI_coeff.Max = (0.95);
    cntl3p3z_InvI_coeff.Min = (-0.95);

    pidGRANDO_Iinv.param.Kp=0.8;
    pidGRANDO_Iinv.param.Ki=(0.15);
    pidGRANDO_Iinv.param.Kd=(0.0);
    pidGRANDO_Iinv.param.Kr=(1.0);
    pidGRANDO_Iinv.param.Umax=(1.0);
    pidGRANDO_Iinv.param.Umin=(-1.0);

    //并网母线电压前馈环
    pidGRANDO_Vinv.param.Kp=(3.0);
    pidGRANDO_Vinv.param.Ki=(0.005);
    pidGRANDO_Vinv.param.Kd=(0.0);
    pidGRANDO_Vinv.param.Kr=(1.0);
    pidGRANDO_Vinv.param.Umax=(0.95);
    pidGRANDO_Vinv.param.Umin=(0.000);

    //PFC电压环
    Udc_PI.Kp = 0.1;                //0.05
    Udc_PI.Ki = 0.001;              //0.0005 0.001
    Udc_PI.Imax = 22.5;
    Udc_PI.Imin = -22.5;
    Udc_PI.Umax = 22.5;
    Udc_PI.Umin = -22.5;
    //Udc_PI.Imax = 22.5;
    //Udc_PI.Imin = 0.0;
    //Udc_PI.Umax = 22.5;
    //Udc_PI.Umin = 0.0;
    Udc_PI.i10 = 0;
    Udc_PI.i11 = 0;

    Vdcref=0;                       //电压闭环用的
    Vdcexp=300.0;                   //母线前馈电压限定值

    SPLL_1ph_init(GRID_FREQ,(0.00005),&spll1);

    CloseVloopInv = 0;
    CloseIloopInv = 0;
    ClearInvTrip=0;
    InvModIndex = 0.5;
    ScaleFactor = (1.0);
    //inv_Iset = (0.1);             //Grid
    inv_Iset = (0.02);              //PFC
    inv_Iset_ref=0;
    InvSine=0;
    Gui_Vrms=0;
    SlewClrInvTrip=1;
    inv_emavg_cur = 0.0;

    Offset_Volt=(0.5); // the input sinusoid is offset with 1.65 V

    PVInverterState=0;
    Gui_InvStart=0;
    Gui_InvStop=0;

    ClearFlyBackTrip=0;
    duty_flyback_pu=(0.5);
    duty_flyback_pu2=(0.0);
    duty_flyback=0;
    duty_flyback2=0;
    // keep the PLL in reset by default
    ResetPLL=1;

//====================================================================================
// INTERRUPTS & ISR INITIALIZATION (best to run this section after other initialization)
//====================================================================================
    //Also Set the appropriate # define's in the Resolver-Settings.h
    //to enable interrupt management in the ISR
    EALLOW;// This is needed to write to EALLOW protected registers

    // CBC6 is still CPU EMU trip, no doc tells if this is true so just guessing
    EPwm1Regs.TZSEL.bit.CBC6=0x1;
    EPwm2Regs.TZSEL.bit.CBC6=0x1;

    // Enable EPWM1 interrupt (20KHz ISR)
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // INT on zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;              // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;         // Generate INT on every event
    EPwm1Regs.ETCLR.bit.INT    = 1;                     // Enable more interrupts

    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;              // Enable TBCLK

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;                  // Enable PIE group 3 interrupt 1 for EPWM1_INT
    PieVectTable.EPWM1_INT = &EPWM1_ISR;                // EPWM1_ISR ISR;
    //PieVectTable.ECAN1INTA = &eCAN_ISRA;              // eCAN1-A中断
    PieVectTable.ECAN0INTA  = &eCAN_ISRA;               // eCAN0-A中断
    IER |= M_INT3;                                      // Enable CPU INT3 for EPWM1 INT

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;                  // Enable PIE group 1 interrupt 6 for ADC_INT
    PieVectTable.ADCINT = &ADC_ISR;
    //PieCtrlRegs.PIEIER9.bit.INTx6 = 1;                // eCAN1-A中断使能 (组9.6)
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;                  // eCAN0-A中断使能 (组9.5)
    IER |= M_INT1;                                      // Enable CPU INT1 for ADC INT
    IER |= M_INT9;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    EDIS;

//=================================================================================
//  BACKGROUND (BG) LOOP
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
    for(;;)  //infinite loop
    {
        /*
        //下面方法查询法可以用
        // 读取中断标志
        if(ECanaRegs.CANRMP.bit.RMP1 == 1) {
            // 接收消息到邮箱1
            g_RxMsg2.msgID = 0x222;        // 接收ID
            g_RxMsg2.msgID_IDE = 0;        // 扩展帧
            g_RxMsg2.msgDLC = 8;           // 数据长度8字节
            CAN_ReceiveMessageA(1, (CAN_MSG*)&g_RxMsg2);
            g_RxFlag2 = 1;

            // 清除接收消息挂起标志
            ECanaRegs.CANRMP.bit.RMP1 = 1;
        }
        */

        // State machine entry & exit point
        //===========================================================
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================

        //计算平均值
        MATH_EMAVG2_F_MACRO(fabs(inv_meas_cur_inst), inv_emavg_cur, 0.0003);
    }
} //END MAIN CODE

//************************************************************************************************
//************************************************************************************************


//=================================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------

//--------------------------------------------------------
void A0(void)
//--------------------------------------------------------
{
    // loop rate synchronizer for A-tasks
    if(CpuTimer0Regs.TCR.bit.TIF == 1)
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;  // clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();                // jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------

        VTimer0[0]++;                   // virtual timer 0, instance 0 (spare)
    }

    Alpha_State_Ptr = &B0;              // Comment out to allow only A tasks
}

//--------------------------------------------------------
void B0(void)
//--------------------------------------------------------
{
    // loop rate synchronizer for B-tasks
    if(CpuTimer1Regs.TCR.bit.TIF == 1)
    {
        CpuTimer1Regs.TCR.bit.TIF = 1;  // clear flag

        //-----------------------------------------------------------
        (*B_Task_Ptr)();                // jump to a B Task (B1,B2,B3,...)
        //-----------------------------------------------------------
        VTimer1[0]++;                   // virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &C0;              // Allow C state tasks
}

//--------------------------------------------------------
void C0(void)
//--------------------------------------------------------
{
    // loop rate synchronizer for C-tasks
    if(CpuTimer2Regs.TCR.bit.TIF == 1)
    {
        CpuTimer2Regs.TCR.bit.TIF = 1;  // clear flag

        //-----------------------------------------------------------
        (*C_Task_Ptr)();                // jump to a C Task (C1,C2,C3,...)
        //-----------------------------------------------------------
        VTimer2[0]++;                   //virtual timer 2, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;              // Back to State A0
}


//=================================================================================
//  A - TASKS 5ms
//=================================================================================
//--------------------------------------------------------
void A1(void)
//--------------------------------------------------------
{
    ky1++;
    if(ky1>1000)ky1=0;

    //5ms*2(A2)*30=300ms
    CAN_SendCount++;
    if(CAN_SendCount==30)
    {
        // 构建发送消息
        sendMsg.msgID = 0x111;              // 发送ID
        sendMsg.msgID_IDE = 0;              // 标准帧

        //sendMsg.msgID = 0x95555555;       // 发送ID
        //sendMsg.msgID_IDE = 1;            // 扩展帧

        sendMsg.msgDLC = 8;                 // 数据长度8字节

        // 填充数据
        sendMsg.msgData[0] = sendData[0];
        sendMsg.msgData[1] = sendData[1];
        sendMsg.msgData[2] = sendData[2];
        sendMsg.msgData[3] = sendData[3];
        sendMsg.msgData[4] = sendData[4];
        sendMsg.msgData[5] = sendData[5];
        sendMsg.msgData[6] = sendData[6];
        sendMsg.msgData[7] = sendData[7];

        // 发送消息
        CAN_SendMessageA(0, &sendMsg);
    }
    if(CAN_SendCount==60)
    {
        CAN_SendCount=0;
        // 构建发送消息
        //sendMsg.msgID = 0x333;            // 发送ID
        //sendMsg.msgID_IDE = 0;            // 标准帧

        sendMsg.msgID = 0x129C00F0;         // 发送扩展帧ID 0x129C00F0
        sendMsg.msgID_IDE = 1;              // 扩展帧

        sendMsg.msgDLC = 8;                 // 数据长度8字节

        // 填充数据
        sendMsg.msgData[0] = sendData[0];
        sendMsg.msgData[1] = sendData[1];
        sendMsg.msgData[2] = sendData[2];
        sendMsg.msgData[3] = sendData[3];
        sendMsg.msgData[4] = sendData[4];
        sendMsg.msgData[5] = sendData[5];
        sendMsg.msgData[6] = sendData[6];
        sendMsg.msgData[7] = sendData[7];

        // 发送消息
        CAN_SendMessageA(0, &sendMsg);
    }

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    A_Task_Ptr = &A2;
    //-------------------
}

//-----------------------------------------------------------------
void A2(void) 
//-----------------------------------------------------------------
{
#if(TFT==1)
    //串口通讯处理
    if(SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag == 1)
    {
       SCI_Msg.timerOut++;
    }

    //等待串口调试助手发送 操作命令
    //SCI_Msg.timerOut在定时器timer0 中
    if(SCI_Msg.timerOut >= RTU_TIMEROUT)
    {
        SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag = 0;
        SCI_Msg.timerOut = 0;
        SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 1;

        //串口命令
        HandleCommAG();

        SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 0;
    }
#endif

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A1;
    //-------------------
}

//=================================================================================
//  B - TASKS 10ms
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) 
//----------------------------------------
{
    //TFT显示
    sci_tft();

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) 
//----------------------------------------
{
    RUN_LED();                          // Toggle LED3 on cc

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void)  
//----------------------------------------
{
    Gui_Vbus=vbus_meas_inst*Vbus_SENSE_MAX;
    Gui_Iavg=inv_emavg_cur*IGrid_SENSE_MAX;

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}

//=================================================================================
//  C - TASKS 1ms
//=================================================================================

//--------------------------------- USER ------------------------------------------

//------------------------------------------------------
void C1(void)
//------------------------------------------------------
{
    ky2++;
    if(ky2>1000)ky2=0;

    if (ClearFlyBackTrip==1)
    {
        ClearFlyBackTrip=0;
        mPWM3_ON();
        mPWM4_ON();
    }

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C2
    C_Task_Ptr = &C2;
    //-----------------
}

//----------------------------------------
void C2(void)
//----------------------------------------
{
    //  C - TASKS 1ms
    // Inverter State ==1 , Check grid voltage, i.e. Vrms Real > _IQ15(12)
    // Inverter State ==2 , Check if panel voltage is available, i.e. Vpv > 5V
    //                      if true enable MPPT
    // Inverter State ==3 , Check if Gui_Vbus>33V
    //                      enable closed voltage loop regulation and current loop regulation
    // Inverter State == 4, Check if inv_Iset > 0.5 pu, Clear Inverter Trip
    // Inverter State == 5, wait for stop command, if stop, trip all PWM's, shut down MPPT and return to state 0, reset all values

    switch(PVInverterState)
    {
        case 0: // wait for the command to start the inverter
                if(Gui_InvStart==1)
                {
                    PVInverterState=1;
                    Gui_InvStart=0;

                    //1000*2ms=2s
                    SlewClrInvTrip=1000;
                }
                break;
        case 1: // once the inverter command is issued check if the grid is present
                if(Gui_Vrms>_IQ15(80.0))
                {
                    RelayStart();
                    SlewClrInvTrip--;
                    if(SlewClrInvTrip==0)
                    {
                        PVInverterState=2;
                    }
                    // take the PLL out of reset
                }
                break;
        case 2: // AC is present, check if PV is present
                if(Gui_Vbus>(120.0))  //PFC
                //if(Gui_Vbus>(220.0))    //Grid
                {
                    PVInverterState=3;
                    //Enable MPPT
                    //MPPT_ENABLE=1;
                }
                break;
        case 3: // Check if DC Bus is greater than 33V , as currently the inverter is off this woudl happen quickly
                PVInverterState=4;
                CloseVloopInv=1;
                CloseIloopInv=1;
                break;
        case 4: // Check if there is enough current command then only connect the grid
                // this is for safety, as the board does not have reverse current sense,
                //if(inv_Iset>(-0.5))   //PFC
                //if(inv_Iset>(0.05))     //Grid
                //{
                    PVInverterState=5;
                    ClearInvTrip=1;
                //}
                break;
        case 5: // Wait for shut down sequence
                if(Gui_InvStop==1)
                {
                    Gui_InvStop=0;
                    // Run the reset sequence
                    // Trip the PWM for the inverter
                    // software force the trip of inverter to disable the inverter completely
                    SPWM_OFF;
                    RelayStop();

                    //MPPT_ENABLE=0;
                    //mppt_incc1.mppt_first=1;
                    //mppt_pno1.mppt_first=1;

                    //VpvRef=(0.9);

                    // Wait for the command to be restarted
                    PVInverterState=0;

                    CloseVloopInv=0;
                    CloseIloopInv=0;
                }
                break;
        case 6: // wait for the power sequence
                break;
        default:
                break;
    }

    if(Gui_Vrms<_IQ15(50.0) && PVInverterState <3)
        ResetPLL=1;
    else
        ResetPLL=0;

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C1
    C_Task_Ptr = &C1;
    //-----------------
}

//************************************************************************************************
#pragma CODE_SECTION(ADC_ISR,"ramfuncs");
interrupt void ADC_ISR ( void )
{
    //同步采样
    V165 = (AdcRegs.ADCRESULT5>>4)*0.000244140625;  //B2
    Vac_FB = AdcRegs.ADCRESULT0>>4;                 //A0
    Vbus_FB = AdcRegs.ADCRESULT8>>4;                //A4
    Gui_Vrms = _IQ15mpy (KvInv, sine_mainsV.Vrms);
    vbus_meas_inst=Vbus_FB*0.000244140625;

#if(Grid==1)
    // PLL Start 1/4096=0.000244140625
    spll1.AC_input=((float)Vac_FB*0.000244140625-0.5)*2.0;
    //SPLL_1ph_run(&spll1);
    SPLL_1ph_MACRO(spll1);
    InvSine     =(spll1.sin[0]);
#else
    sgen.calc(&sgen);
    InvSine = (float)sgen.out1*0.000030517578125;
#endif

    //  Voltage loop
    pidGRANDO_Vinv.term.Fbk = (0.66667);            // 250.0V/375.0V 110VAC
    pidGRANDO_Vinv.term.Ref = vbus_meas_inst;

    if (CloseVloopInv==1)
    {
        //并网母线前馈环
        //PID_GR_MACRO(pidGRANDO_Vinv);
        //inv_Iset=pidGRANDO_Vinv.term.Out;

        //PFC电压环
        if(Vdcref > Vdcexp)
            Vdcref = Vdcref - 0.02;
        else
            Vdcref = Vdcref + 0.02;
        //Vbus_FB*0.000244140625*375;
        //inv_Iset = -1.0*DCL_runPI_L4(&Udc_PI,Vdcref,Vbus_FB*0.09155);
        //inv_Iset = DCL_runPI_L4(&Udc_PI,Vdcref,Vbus_FB*0.09155)/51.6A;
        inv_Iset_ref = -0.0193798*DCL_runPI_L4(&Udc_PI,Vdcref,Vbus_FB*0.09155);
        inv_Iset = 0.0193798*DCL_runPI_L4(&Udc_PI,Vdcref,Vbus_FB*0.09155);
        inv_Iset= (inv_Iset>0.1)?0.1:inv_Iset;
        inv_Iset= (inv_Iset<0.02)?0.02:inv_Iset;
    }

    if (ResetPLL==1)
    {
        SPLL_1ph_init(GRID_FREQ,(0.00005),&spll1);
        ResetPLL=0;
    }

    //if (sine_mainsV.ZCD==1 && ClearInvTrip==1)
    if (ClearInvTrip==1)
    {
        SPWM_ON;
        ClearInvTrip=0;
    }

    //inv_ref_cur_inst = inv_Iset*InvSine;      //Grid
    inv_ref_cur_inst = -1.0*inv_Iset*InvSine;   //PFC

    Ileg1_fb = AdcRegs.ADCRESULT14>>4;          //A7
    Ileg2_fb = AdcRegs.ADCRESULT10>>4;          //A5

    inv_meas_cur_lleg1_inst=Ileg1_fb*0.000244140625 -0.5;           //0.000244140625=1/4096
    inv_meas_cur_lleg2_inst=Ileg2_fb*0.000244140625 -0.5;
    inv_meas_cur_inst = (inv_meas_cur_lleg1_inst - inv_meas_cur_lleg2_inst)*2.0;

    inv_meas_vol =(Vac_FB*0.000244140625-0.5)*2.0;
    inv_meas_vol_inst = ((long)((long) Vac_FB<<12)-_IQ24(0.5))<<1;  // 交流电压电流采样都要左移1放大1倍

    pidGRANDO_Iinv.term.Fbk=inv_meas_cur_inst;
    pidGRANDO_Iinv.term.Ref=inv_ref_cur_inst;
    //cntl3p3z_InvI_vars.Fdbk=inv_meas_cur_inst;
    //cntl3p3z_InvI_vars.Ref=inv_ref_cur_inst;

    if(CloseIloopInv==1)
    {
        PID_GR_MACRO(pidGRANDO_Iinv);
        //CNTL_3P3Z_F_FUNC(&cntl3p3z_InvI_coeff,&cntl3p3z_InvI_vars);
    }

    // Apply inverter o/p correction
    if (CloseIloopInv ==0)
    {
        //方波运行：只取正弦的符号，占空比拉满(近100%/0)，输出50Hz方波
        duty = (InvSine >= 0.0) ? 0.99 : -0.99;
        PWMDRV_1phInv_unipolar(1,3750.0,(duty));
    }
    else
    {
        PWMDRV_1phInv_unipolar(1,3750.0,pidGRANDO_Iinv.term.Out);       //并网可以
        //PWMDRV_1phInv_unipolar(1,3750.0,cntl3p3z_InvI_vars.Out);      //并网可以
        //PWMDRV_1phInv_unipolar(1,3750.0,(cntl3p3z_InvI_vars.Out+inv_meas_vol)/vbus_meas_inst);//并网效果差
    }

    // 100kHz 方波输出：EPWM3/EPWM4 两腿同相位、50%占空
    // 驱动板上两腿是互补反接的：EPWM4A->Q21(左腿下)、EPWM4B->Q9(左腿上)；
    // EPWM3A->Q10(右腿上)、EPWM3B->Q22(右腿下)。两路A同相位，V_SA-V_SB 才是 ±BUS 方波。
    EPwm3Regs.CMPA.half.CMPA = 375;                 // 50%占空
    EPwm4Regs.CMPA.half.CMPA = 375;
    EPwm3Regs.AQCSFRC.bit.CSFA = 0;                 // 关闭AQ连续强制，恢复CMPA比较
    EPwm4Regs.AQCSFRC.bit.CSFA = 0;

// ------------------------------------------------------------------------------
//    Connect inputs to the sine analyzer block , compute RMS, Freq, ZCD
// ------------------------------------------------------------------------------
    sine_mainsV.Vin =inv_meas_vol_inst>>9;
    SineAnalyzer_diff_MACRO (sine_mainsV);

    //EPwm2Regs.CMPA.half.CMPA = 375;
    //EPwm3Regs.CMPA.half.CMPA = 375;
    //EPwm4Regs.CMPA.half.CMPA = 375;
    //EPwm5Regs.CMPA.half.CMPA = 375;
    //EPwm6Regs.CMPA.half.CMPA = 375;

    // reset ADC SEQ1, clear ADC INT flag and ack PIE INT for ADC
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;         // clear pending ADC SEQ1 INT flag
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;           // Reset ADC SEQ1
    PieCtrlRegs.PIEACK.all  = PIEACK_GROUP1;    // ADC ISR ACK
    return;
}
//************************************************************************************************

//=================================================================================
// --- ISR at 10KHz rate ---
//=================================================================================
#pragma CODE_SECTION(EPWM1_ISR,"ramfuncs");
interrupt void EPWM1_ISR(void)
{
    //ky3++;
    //if(ky3>1000)ky3=0;

    if((Vbus_FB*0.09155)>400.0)
    {
        //
        Gui_InvStop=1;

        SPWM_OFF;
        RelayStop();

        PVInverterState=0;

        CloseVloopInv=0;
        CloseIloopInv=0;
    }

    EPwm5Regs.CMPA.half.CMPA = 625;             //1500/2=750 625=750-125小电源板输入12VDC输出是18VDC
    EPwm5Regs.CMPB=125;

    EPwm1Regs.ETCLR.bit.INT = 1;                // clear EPwm1 INT flag
    PieCtrlRegs.PIEACK.all  = PIEACK_GROUP3;    // EPWM1 ISR ACK
    return;
}

//---------------------------------------------------------------------
//操作串口命令协议，SCI串口命令参数BaudRate=19200
//#define COMM_ID       0xA2        //串口命令识别地址
//#define ID_I          0           //串口命令第0位
//#define CMD_I         1           //串口命令第1位
//#define ADDR_I        2           //串口命令第2位
//#define DATA_I        3           //串口命令第3位
//#define DATB_I        4           //串口命令第4位
//---------------------------------------------------------------------
unsigned char HandleCommAG()
{
    char s1[]="Enter the command is not correct!\r\n";
    char s2[]="The command format is:EE 00 00\r\n";
    int len;
    unsigned char addr,getdata,getdatb,getdatc,getdatx;
    //s1字符串输出到串口调试助手上，如果以16进制查看都是ascii码
    if(SCI_Msg.rxData[ID_I] != COMM_ID)                 //ID_I=0   COMM_ID=0xEE
    {
        SCI_Msg.rxWriteIndex = 0;
        SCI_Msg.rxReadIndex = 0;
        //如果命令不正确就直接返回也可在此写命令不正确要执行的代码
        len=strlen(s1);                                 //strlen #include <string.h>
        send_msgc(s1,len);
        return 1;
    }
    //解析操作
    switch(SCI_Msg.rxData[CMD_I])                       //CMD_I=1表示串口接收到的数据rxData中的数组指针位置
    {
        //获得帮助，命令示例：
        //EE  功能    地址  数据
        //EE  00   00
        case 0:
            addr =  SCI_Msg.rxData[ADDR_I];             //ADDR_I=2 串口命令第2位
            //getdata = SCI_Msg.rxData[DATA_I];         //DATA_I=3 得到调试模式值，必须是0x00-0xFF
            if(addr==0)
            {
                len=strlen(s2);
                send_msgc(s2,len);
            }
            if(addr==1)
            {
                //DisTypeData=1;
            }
            if(addr==2)
            {
                //DisTypeData=0;
            }
            if(addr==3)
            {
                //放电模式
                strcpy(cPV_State,"并网");
            }
            if(addr==4)
            {
                //PFC 充电模式
                strcpy(cPV_State,"PFC");
            }
            if(addr==5)
            {
                //增加占空比
                //inv_Iset=inv_Iset+0.01;
                //手动移相+
                duty_flyback_pu2=duty_flyback_pu2+(0.05);
            }
            if(addr==6)
            {
                //减小占空比
                //inv_Iset=inv_Iset-0.01;
                //手动移相-
                duty_flyback_pu2=duty_flyback_pu2-(0.05);
            }
            break;
        //EE  01   00
        case 1:
            addr =  SCI_Msg.rxData[ADDR_I];                     //ADDR_I=2 串口命令第2位
            if(addr==0)
            {
                //DCDC 开启
                duty_flyback_pu=(0.5);
                duty_flyback_pu2=(0.0);
                RlyConnect=1;
                ClearFlyBackTrip=1;
            }
            if(addr==1)
            {
                //DCDC 闭环
            }
            if(addr==2)
            {
                //减小频率，增加电压
            }
            if(addr==3)
            {
                //增加频率，减小电压
            }
            if(addr==4)
            {
                //停止DCDC
                CloseFlybackLoop=0;
                mPWM3_OFF();
                mPWM4_OFF();
                duty_flyback_pu=(0.5);
                duty_flyback_pu2=(0.0);
            }
            if(addr==5)
            {
                //启动辅助继电器
            }
            if(addr==6)
            {
                //开启同步整流充电
            }
            break;
        //EE  02   00
        case 2:
            addr =  SCI_Msg.rxData[ADDR_I];             //ADDR_I=2 串口命令第2位
            if(addr==0)
            {
                //DCAC 开启
                ClearInvTrip=1;
            }
            if(addr==1)
            {
                //RlyConnect=1;
                RelayStart();
            }
            if(addr==2)
            {
                //InvModIndex=InvModIndex+_IQ24(0.1);
            }
            if(addr==3)
            {
                //InvModIndex=InvModIndex-_IQ24(0.1);
            }
            if(addr==4)
            {
                //mSPWM_OFF();
            }
            if(addr==5)
            {
                //JDQ1_OFF();
            }
            break;
         //EE  03   00
        case 3:
            addr =  SCI_Msg.rxData[ADDR_I];             //ADDR_I=2 串口命令第2位
            if(addr==0)
            {
                //MPPT_ENABLE=1;
            }
            if(addr==1)
            {
                //DCAC 闭环
                CloseVloopInv=1;
                CloseIloopInv=1;
            }
            if(addr==2)
            {
                Gui_InvStart=1;
                strcpy(cPV_State,"开始");
            }
            if(addr==3)
            {
                //停止MPPT
            }
            if(addr==4)
            {
                Gui_InvStop=1;
                strcpy(cPV_State,"关机");
            }
            if(addr==5)
            {
                //TEST 离网测试PWM驱动信号
                RelayStop();
                ClearInvTrip = 1;
                strcpy(cPV_State,"离网");
            }
            break;
            //获得TFT 页面ID=1 的操作
            //       0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
            //设置电压 EE B1 11 00 02 00 08 11 35 30 00 FF FC FF FF 50V
            //设置电流 EE B1 11 00 01 00 05 11 32 30 00 FF FC FF FF 20A
            //设置充电电压 pv_ref_bv_inst=_IQ24(0.113);              //29.4VDC充电
            //设置充电电压 pv_ref_bv_inst=_IQ24(0.160);              //45.5VDC充电
            //设置充电电压 pv_ref_bv_inst=_IQ24(0.180);              //49.5VDC充电
        case 0xB1:
            addr =  SCI_Msg.rxData[6];
            //设置充电电压15-55V，达到控制充电电压
            if(addr==4)
            {
                //设置充电电压15-55V，达到控制充电电压
                if(addr==0x04)
                {
                    if(SCI_Msg.rxData[11]==0)
                    {
                        //合并[31 32为一个10进制数
                        getdata = SCI_Msg.rxData[8]-0x30;
                        getdatb = SCI_Msg.rxData[9]-0x30;
                        getdatc = SCI_Msg.rxData[10]-0x30;
                        SET_VAL=getdata*100+getdatb*10+getdatc;
                    }
                }
            }

            //设置充电电流1.0-10.9A，一位小数
            if(addr==0x0D)
            {
                if(SCI_Msg.rxData[11]==0)
                {
                    //由于rxData是16进制数-0x30变成10进制数
                    getdata = SCI_Msg.rxData[8]-0x30;

                    getdatx = SCI_Msg.rxData[10]-0x30;
                    SET_VAL=getdata+getdatx/10.0;
                }
                else if(SCI_Msg.rxData[12]==0)
                {
                    //合并[31 32为一个10进制数
                    getdata = SCI_Msg.rxData[8]-0x30;
                    getdatb = SCI_Msg.rxData[9]-0x30;

                    getdatx = SCI_Msg.rxData[11]-0x30;
                    SET_VAL=getdata*10+getdatb+getdatx/10.0;
                }
            }

        default:break;
    }

    SCI_Msg.rxWriteIndex = 0;
    SCI_Msg.rxReadIndex = 0;

    return 0;
}

void sendTFT(float32 U,Uint16 SID,Uint16 ID)
{
    //广州大彩设备出厂前均默认 RS232 电平，若用户需要 TTL 通讯方式，只需将 J5 焊盘短路
    //短路后就不能用RS232进行串口测试，但可以用VisualTFT下载工程
    //在工程配置中可以设置与F28035的传输波特率
    //TTL 电平模式下兼容 3.3V 和 5V IO 输入输出
    //SCI发送数据，波特率19200
    //ASCII(., 0, 1, 2, 3, 4, 5, 6, 7,  8, 9)
    //=HEX(2E,30,31,32, 33,34,35,36,37,38,39)
    //EE B1 10 Screen_id Control_id     Strings         FF FC FF FF
    //EE B1 10   00 01     00 01     32 33 2E 30 35     FF FC FF FF =23.05
    // 0  1  2    3  4     5   6     7   8  9  10  11   12 13 14 15
    char str[10];                                       //char str[10];定义不能放到main()前的全局定义上，否则串口命令不能正常运行，原因不明
    //ftoa()使用时系统不能正常运行，现象是：如果1和2路拉载开启，串口发送A2 03 00命令时第1路拉载的PWM方波会停止
    //如果不使用ftoa()系统能正常运行，也不会出现上面现象，应该找一个更好的函数
    float_TO_ascii(U,str);

    send_Tdata[4] = SID;                    //页面控件ID

    send_Tdata[6] = ID;                     //控件ID

    send_Tdata[7] = str[0];                 //send_Tdata[7-11]为ASCII数值
    send_Tdata[8] = str[1];
    send_Tdata[9] = str[2];
    send_Tdata[10]= str[3];
    send_Tdata[11]= str[4];

    send_msg(send_Tdata);                   //发送数据
}

void sci_tft(void)
{
#if(TFT==1)
    // TFT Display
    if(send_timer==timerNum)
    {
        sendTFT(0.0,0x01,0x01);
        //sendTFT(RMS.Va_rms,0x01,0x01);
    }
    if(send_timer==(timerNum*2))
    {
        sendTFT(0.0,0x01,0x02);
        //sendTFT(RMS.Ia_rms,0x01,0x02);
    }
    if(send_timer==(timerNum*3))
    {
        sendTFT(Gui_Vbus,0x01,0x03);
        //sendTFT(Vdc.Vpn,0x01,0x03);
    }
    ///*
    //备用
    if(send_timer==(timerNum*4))
    {
        //sendTFT(THIPWM.modulation_index,0x01,0x04);
        sendTFT(0.0,0x01,0x04);
    }
    //*/
    if(send_timer==(timerNum*5))
    {
        sendTFT(0.0,0x01,0x05);
    }
    if(send_timer==(timerNum*6))
    {
        sendTFT(0.0,0x01,0x06);
        //sendTFT(RMS.a_Prms*3.0f,0x01,0x06);
    }
    if(send_timer==(timerNum*7))
    {
        sendTFT(Vdcref,0x01,0x07);
    }
    if(send_timer==(timerNum*8))
    {
        //sendTFT(inv_Iset_ref,0x01,0x08);
        sendTFT(duty_flyback_pu2,0x01,0x08);
    }
    if(send_timer==(timerNum*9))
    {
        sendTFT(50.0,0x01,0x09);
        //sendTFT(DSOGI_dq0.fo_LPF1,0x01,0x09);
    }
    if(send_timer==(timerNum*10))
    {
        //发送的数据更新TFT LCD文本数据
        //EE B1 10 Screen_id Control_id     Strings      FF FC FF FF
        //EE B1 10   00 01     00 01       [Strings]     FF FC FF FF
        scic_xmit(COMM_ID);
        scic_xmit(0xB1);
        scic_xmit(0x10);
        //Screen_id
        scic_xmit(0x00);
        scic_xmit(0x01);
        //Control_id
        scic_xmit(0x00);
        scic_xmit(0x0A);
        //Strings
        send_msgc(cPV_State,strlen(cPV_State));
        //FF FC FF FF
        scic_xmit(0xFF);
        scic_xmit(0xFC);
        scic_xmit(0xFF);
        scic_xmit(0xFF);

        send_timer=0;
    }
    else
    {
        send_timer++;
    }
#endif
}

//************************************************************************************************
//************************************************************************************************
