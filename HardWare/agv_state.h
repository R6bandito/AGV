#ifndef __AGV_STATE_H
#define __AGV_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum 
{
    AGV_STATE_IDLE = 0, //空闲态（初始/未运行）
    AGV_STATE_RUNNING,  //运行态 执行指令
    AGV_STATE_ESTOP,    //急停态 指令超时/舵机异常触发
    AGV_STATE_ERROR     //错误态 转速超限等严重异常
} AGV_State_t;

void AGV_State_Init(void);
void AGV_SafetyCheck(void);
AGV_State_t AGV_GetState(void);
void AGV_EmergencyStop(void);

void AGV_UpdateLastCmdTick(void);
#endif

