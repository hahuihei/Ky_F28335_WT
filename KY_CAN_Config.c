/*
 * CAN操作函数
 *
 *  Created on: 2020年06月25日
 *      Author: kypine
 */

#include "KY_CAN_Config.h"
#include "KY-Settings.h"

// eCAN-A寄存器影子变量
struct ECAN_REGS ECanShadow;

CAN_MSG g_RxMsg;           // 接收消息存储
Uint16 g_RxFlag = 0;       // 接收标志位

void InitECanA(void)
{
    //
    // Create a shadow register structure for the CAN control registers. This
    // is needed, since only 32-bit access is allowed to these registers.
    // 16-bit access to these registers could potentially corrupt the register
    // contents or return false data. This is especially true while writing
    // to/reading from a bit (or group of bits) among bits 16 - 31
    //

    EALLOW;     // EALLOW enables access to protected bits
    // 2. 配置GPIO引脚为CAN功能，在中KY-DevInit_F28335.c已配置
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;                 // Enable pull-up for GPIO18 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;                 //Enable pull-up for GPIO19 (CANTXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;               // Asynch qual for GPIO18 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;                // Configure GPIO18 for CANRXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;                // Configure GPIO19 for CANTXA

    //
    // Configure eCAN RX and TX pins for CAN operation using eCAN regs
    //
    ECanShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanShadow.CANTIOC.all;

    ECanShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanShadow.CANRIOC.all;

    //
    // Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31)
    // HECC mode also enables time-stamping feature
    //
    ECanShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanShadow.CANMC.bit.SCB = 1;
    ECanaRegs.CANMC.all = ECanShadow.CANMC.all;

    //
    // Initialize all bits of 'Master Control Field' to zero
    // Some bits of MSGCTRL register come up in an unknown state. For proper
    // operation, all bits (including reserved bits) of MSGCTRL must be
    // initialized to zero
    //
    ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

    //
    // TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
    // as a matter of precaution.
    //
    ECanaRegs.CANTA.all = 0xFFFFFFFF;                   // Clear all TAn bits

    ECanaRegs.CANRMP.all = 0xFFFFFFFF;                  // Clear all RMPn bits

    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;                 // Clear all interrupt flag bits
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

    //
    // Configure bit timing parameters for eCANA
    //
    ECanShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanShadow.CANMC.bit.CCR = 1 ;                      // Set CCR = 1
    ECanaRegs.CANMC.all = ECanShadow.CANMC.all;

    ECanShadow.CANES.all = ECanaRegs.CANES.all;

    do
    {
        ECanShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanShadow.CANES.bit.CCE != 1 );            // Wait for CCE bit to be set

    ECanShadow.CANBTC.all = 0;

    //
    // CPU_FRQ_150MHz is defined in DSP2833x_Examples.h
    //
    #if (CPU_FRQ_150MHZ)
    //
    // The following block for all 150 MHz SYSCLKOUT
    // (75 MHz CAN clock) - default. Bit rate = 500 kbps See Note at End of File
    //
    // 位时间计算: (12+2+1) = 15 个时间量子
    // 时间量子周期: 1/7.5MHz = 133.33ns
    // 位时间: 15 * 133.33ns ≈ 1.99995μs -> 1/1.99995 ≈ 500kbps
    // 或
    // CAN时钟 = 150MHz / 2 / (BRP+1) = 7.5MHz
    // 位时间 = (TSEG1+1) + (TSEG2+1) + 1 = 15个时间量子
    // 波特率 = 7.5MHz / 15 = 500kbps

    ECanShadow.CANBTC.bit.BRPREG = 9;
    ECanShadow.CANBTC.bit.TSEG2REG = 2;
    ECanShadow.CANBTC.bit.TSEG1REG = 10;
    #endif

    //
    // CPU_FRQ_100MHz is defined in DSP2833x_Examples.h
    //
    #if (CPU_FRQ_100MHZ)
    //
    // The following block is only for 100 MHz SYSCLKOUT (50 MHz CAN clock).
    // Bit rate = 1 Mbps See Note at End of File
    //
    ECanaShadow.CANBTC.bit.BRPREG = 4;                  //10MHz
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;                //10MHz/(6+1+3)=1 Mbps
    ECanaShadow.CANBTC.bit.TSEG1REG = 6;
    #endif

    ECanShadow.CANBTC.bit.SAM = 1;
    ECanaRegs.CANBTC.all = ECanShadow.CANBTC.all;

    ECanShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanShadow.CANMC.bit.CCR = 0 ;                      // Set CCR = 0
    ECanaRegs.CANMC.all = ECanShadow.CANMC.all;

    ECanShadow.CANES.all = ECanaRegs.CANES.all;

    do
    {
        ECanShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanShadow.CANES.bit.CCE != 0 );            // Wait for CCE bit to be  cleared

    //
    // Disable all Mailboxes
    //
    ECanaRegs.CANME.all = 0;                            // Required before writing the MSGIDs

    EDIS;

    EALLOW;
    //
    // Write to the MSGID field
    //
    // 配置邮箱0消息ID
    ECanaMboxes.MBOX0.MSGID.all = 0;
#if(EXPF==1)
    ECanaMboxes.MBOX0.MSGID.bit.IDE = 1;                // 扩展帧
    ECanaMboxes.MBOX0.MSGID.all = 0x95555555;           // 扩展帧ID 0x15555555 | 0x80000000
#else
    ECanaMboxes.MBOX0.MSGID.bit.IDE = 0;                // 标准帧
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = 0x111;       // 标准ID
#endif
    //
    // Configure Mailbox under test as a Transmit mailbox
    //
    ECanShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanShadow.CANMD.bit.MD0 = 0;
    ECanaRegs.CANMD.all = ECanShadow.CANMD.all;

    //
    // Enable Mailbox under test
    //
    ECanShadow.CANME.all = ECanaRegs.CANME.all;
    ECanShadow.CANME.bit.ME0 = 1;
    ECanaRegs.CANME.all = ECanShadow.CANME.all;

    //
    // Write to DLC field in Master Control reg
    //
    ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;

    //
    // Write to the mailbox RAM field
    //
    ECanaMboxes.MBOX0.MDL.all = 0x55555555;
    ECanaMboxes.MBOX0.MDH.all = 0x55555555;
    EDIS;

    //配置接收邮箱 (邮箱1)
    EALLOW;
    // 清除邮箱1配置
    //ECanaRegs.CANME.bit.ME1 = 0;                  // 禁用邮箱1
    ECanaRegs.CANMD.bit.MD1 = 1;                    // 配置为接收模式

    // 配置邮箱1消息ID
    ECanaMboxes.MBOX1.MSGID.all = 0;
#if(EXPF==1)
    ECanaMboxes.MBOX1.MSGID.bit.IDE = 1;            // 扩展帧
    ECanaMboxes.MBOX1.MSGID.all = 0x829A00F0;       // 扩展帧ID 0x029A00F0 | 0x80000000
#else
    ECanaMboxes.MBOX1.MSGID.bit.IDE = 0;            // 标准帧
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = 0x222;   // 标准帧ID
#endif
    //ECanaMboxes.MBOX1.MSGID.bit.AME = 1;          // 使能接受掩码

    // 配置本地接受掩码
    //ECanaLAMRegs.LAM1.all = 0xFFFFFFFF;           // 接受所有ID

    // 配置消息控制寄存器
    ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;          // 数据长度8字节
    EDIS;
    // 配置中断
    EALLOW;
    ECanaRegs.CANMIM.bit.MIM1 = 1;                  // 使能邮箱1中断
    ECanaRegs.CANMIL.bit.MIL1 = 0;                  // 邮箱1中断连接到中断线0 ECAN0INT (INT9.5)
    ECanaRegs.CANGIM.bit.I0EN = 1;                  // 使能中断级别0
    EDIS;

    // 使能邮箱
    EALLOW;
    ECanaRegs.CANME.bit.ME1 = 1;                    // 使能接收邮箱1
    ECanaRegs.CANME.bit.ME0 = 1;                    // 必须再次使能发送邮箱0
    EDIS;

}

// CAN发送消息函数
void CAN_SendMessageA(Uint16 Mailbox, CAN_MSG *Msg) {

    EALLOW;
    //配置ID前一定要先禁用邮箱，配置好后再使能
    ECanaRegs.CANME.bit.ME1 = 0;                    // 禁用接收邮箱1
    ECanaRegs.CANME.bit.ME0 = 0;                    // 禁用发送邮箱0
    // 1. 配置消息ID
    if(Msg->msgID_IDE == 0) {
        // 标准帧 (11位ID)
        ECanaRegs.CANMD.bit.MD0 = 0;                // 发送模式
        ECanaMboxes.MBOX0.MSGID.bit.IDE = 0;
        ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = Msg->msgID;
    } else {
        // 扩展帧 (29位ID)
        ECanaMboxes.MBOX0.MSGID.bit.IDE = 1;
        ECanaMboxes.MBOX0.MSGID.all = (Msg->msgID | 0x80000000);
    }

    // 2. 配置数据长度
    ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = Msg->msgDLC;

    ECanaRegs.CANME.bit.ME1 = 1;                    // 使能接收邮箱1
    ECanaRegs.CANME.bit.ME0 = 1;                    // 使能发送邮箱0
    EDIS;

    // 3. 写入数据 (低32位)
    ECanaMboxes.MBOX0.MDL.all = 0;
    ECanaMboxes.MBOX0.MDL.byte.BYTE0 = Msg->msgData[0];
    ECanaMboxes.MBOX0.MDL.byte.BYTE1 = Msg->msgData[1];
    ECanaMboxes.MBOX0.MDL.byte.BYTE2 = Msg->msgData[2];
    ECanaMboxes.MBOX0.MDL.byte.BYTE3 = Msg->msgData[3];

    // 4. 写入数据 (高32位)
    ECanaMboxes.MBOX0.MDH.all = 0;
    ECanaMboxes.MBOX0.MDH.byte.BYTE4 = Msg->msgData[4];
    ECanaMboxes.MBOX0.MDH.byte.BYTE5 = Msg->msgData[5];
    ECanaMboxes.MBOX0.MDH.byte.BYTE6 = Msg->msgData[6];
    ECanaMboxes.MBOX0.MDH.byte.BYTE7 = Msg->msgData[7];

    // 5. 请求发送
    ECanShadow.CANTRS.all = 0;
    ECanShadow.CANTRS.bit.TRS0 = 1;                     // 发送请求
    ECanaRegs.CANTRS.all = ECanShadow.CANTRS.all;

    if(ECanShadow.CANTA.bit.TA0 == 0)
    {
        ECanShadow.CANTA.all = ECanaRegs.CANTA.all;
    }
    //使用下面方法DSP会死机
    //do
    //{
    //    ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
    //} while(ECanaShadow.CANTA.bit.TA25 == 0 );        // Wait for TA5 bit to be set

    ECanShadow.CANTA.all = 0;
    ECanShadow.CANTA.bit.TA0 = 1;                       // Clear TA0
    ECanaRegs.CANTA.all = ECanShadow.CANTA.all;
}

// CAN接收消息函数
void CAN_ReceiveMessageA(Uint16 Mailbox, CAN_MSG *Msg) {
    if(Mailbox != 1) return;

    // 读取消息ID
    if(ECanaMboxes.MBOX1.MSGID.bit.IDE == 0) {
        Msg->msgID_IDE = 0;
        Msg->msgID = ECanaMboxes.MBOX1.MSGID.bit.STDMSGID;
    } else {
        Msg->msgID_IDE = 1;
        Msg->msgID = ECanaMboxes.MBOX1.MSGID.all & 0x1FFFFFFF;
    }

    // 读取数据长度
    Msg->msgDLC = ECanaMboxes.MBOX1.MSGCTRL.bit.DLC;

    // 读取数据
    Msg->msgData[0] = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
    Msg->msgData[1] = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
    Msg->msgData[2] = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
    Msg->msgData[3] = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
    Msg->msgData[4] = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
    Msg->msgData[5] = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
    Msg->msgData[6] = ECanaMboxes.MBOX1.MDH.byte.BYTE6;
    Msg->msgData[7] = ECanaMboxes.MBOX1.MDH.byte.BYTE7;
}

// eCAN中断服务程序
__interrupt void eCAN_ISRA(void) {
    // 读取中断标志
    if(ECanaRegs.CANRMP.bit.RMP1 == 1) {
        // 接收消息到邮箱1
        CAN_ReceiveMessageA(1, (CAN_MSG*)&g_RxMsg);
        g_RxFlag = 1;

        // 清除接收消息挂起标志
        ECanaRegs.CANRMP.bit.RMP1 = 1;
    }

    // 清除全局中断标志
    ECanaRegs.CANGIF0.bit.GMIF0 = 1;

    // 应答PIE中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
