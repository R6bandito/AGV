#ifndef __WHEEL_MANAGER_H
#define __WHEEL_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// 实际舵轮状态（从CAN反馈更新） 
typedef struct 
{
    float actual_steering_angle_deg;  //实际舵角（度）
    float actual_wheel_rpm;           //实际转速（rpm）
    bool  is_online;                  //舵轮是否在线
    uint32_t last_update_tick;        //最后一次更新时间戳
} WheelActual_t;

void WheelManager_Init(void);

// 发送所有轮的舵角和转速 （到can总线）
void WheelManager_SendAllTargets(void);

// CAN 接收回调调用，更新舵轮实际状态（模拟/真实can接收调用）
void WheelManager_UpdateActual(uint8_t wheel_id, float angle_deg, float rpm);

//  PID 修正接口，修正指定舵轮的转速
void WheelManager_ApplyPIDCorrection(uint8_t wheel_id, float correction_rpm);

// 急停，停止所有舵轮 
void WheelManager_StopAll(void);

//查询舵轮是否在线 
bool WheelManager_IsOnline(uint8_t wheel_id);
uint32_t WheelManager_GetLastUpdateTick(uint8_t wheel_id);
const WheelActual_t* WheelManager_GetActual(uint8_t wheel_id);

#endif

