
#ifndef __DEMO_MODE_H
#define __DEMO_MODE_H

#include "kinematics.h"   // 需要 AGV_Command_t

typedef enum 
{
    DEMO_MODE_STRAIGHT = 0,   // 直行
    DEMO_MODE_LATERAL,        // 横移
    DEMO_MODE_ROTATE,         // 原地旋转
    DEMO_MODE_COMPOUND,       // 复合运动
    DEMO_MODE_PARKING,        // 倒车入库
    DEMO_MODE_MODBUS,         // 上位机Modbus实时指令
    DEMO_MODE_COUNT
} DemoMode_t;

extern DemoMode_t demo_mode;   // 全局模式变量，在 main.c 中定义

void Demo_GetCommand(AGV_Command_t *cmd);   // 根据当前模式和运行时间生成指令

#endif

