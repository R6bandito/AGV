#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#ifndef __cplusplus
  #define bool  _Bool
  #define true  1
  #define false 0
#endif

#include "kinematics.h"

#define FRAME_HEADER1  0xAA  //帧头1（固定0xAA）
#define FRAME_HEADER2  0x55  //帧头2（固定0x55）
#define CMD_MOTION     0x01  //运动指令码
#define CMD_ESTOP      0x02  //急停指令码

typedef struct __attribute__((packed)) //运动指令帧结构体，取消字节对齐，保证帧长度固定
{
    uint8_t  header1;  //帧头1
    uint8_t  header2;  //帧头2
    uint8_t  cmd;      //指令码（0x01=运动，0x02=急停）
    float    vx;       //前进速度
    float    vy;       //横向速度
    float    omega;    //旋转角速度
    uint8_t  crc;      //校验位
} MotionFrame_t;

typedef enum //UART接受状态机枚举，解析帧的不同状态
{
    PROT_WAIT_H1 = 0,   //等待帧头1
    PROT_WAIT_H2,       //等待帧头2
    PROT_WAIT_CMD,      //等待指令码
    PROT_WAIT_DATA,     //等待数据段（Vx，vy,omega）
    PROT_WAIT_CRC       //等待校验位
} UART_RxState_t;

void Protocol_Init(void);
void Protocol_FeedByte(uint8_t ch);
bool Protocol_NewFrameReady(void);
bool Protocol_ParseCommand(AGV_Command_t *cmd);

#endif

