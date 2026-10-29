/*
 * CAN操作函数
 *
 *  Created on: 2020年06月25日
 *      Author: kypine
 */

#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

#include "DSP28x_Project.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "DSP2833x_ECan.h"

// CAN消息结构体
typedef struct {
    Uint32 msgID;                   // 消息标识符
    Uint16 msgID_IDE;               // 标识符扩展位 (0:标准帧, 1:扩展帧)
    Uint16 msgDLC;                  // 数据长度码 (0-8)
    Uint16 msgData[8];              // 消息数据 (8字节)
} CAN_MSG;

// 函数声明
void InitECanA(void);
void CAN_SendMessageA(Uint16 Mailbox, CAN_MSG *Msg);
void CAN_ReceiveMessageA(Uint16 Mailbox, CAN_MSG *Msg);
__interrupt void eCAN_ISRA(void);

#endif

//---------------------------------------------------------------------
//can end
//---------------------------------------------------------------------
