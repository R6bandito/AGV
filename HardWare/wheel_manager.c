#include "wheel_manager.h"
#include "kinematics.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>    // 调试输出用


// 调试模式：模拟CAN发送（真实环境替换为实际CAN驱动） 
// 真实环境由角色3实现，这里仅打印并模拟成功 
#ifndef CAN_SIMPLE_MODE
#define CAN_SIMPLE_MODE 1   // 默认开启调试模式

#endif

#if CAN_SIMPLE_MODE
  #include "debug_uart.h"    // 你的工程中已有 debug_uart 用于 printf
  
  // 模拟CAN发送舵角（打印日志）
  #define CAN_SEND_ANGLE(id, angle)   printf("[CAN] Wheel%d Angle=%.2f deg\r\n", id, (double)(angle))  
  // 模拟CAN发送转速（打印日志）
  #define CAN_SEND_RPM(id, rpm)       printf("[CAN] Wheel%d Speed=%.2f rpm\r\n", id, (double)(rpm))
#else
  // 角色3 的真实函数声明（假设已提供）
  extern void CAN_SendSteeringAngle(uint8_t wheel_id, float angle_deg);
  extern void CAN_SendWheelSpeed(uint8_t wheel_id, float rpm);
  #define CAN_SEND_ANGLE CAN_SendSteeringAngle
  #define CAN_SEND_RPM   CAN_SendWheelSpeed
#endif

// 静态全局变量：4个舵轮的实际状态
static WheelActual_t wheel_actuals[4];

void WheelManager_Init(void)// 初始化舵轮管理器：清空所有舵轮状态
{
    for (int i = 0; i < 4; i++) 
    {
        wheel_actuals[i].actual_steering_angle_deg = 0; //初始舵角0度
        wheel_actuals[i].actual_wheel_rpm = 0;          //初始转速0
        wheel_actuals[i].is_online = false;             //初始离线
        wheel_actuals[i].last_update_tick = 0;          //初始时间戳0
    }
}

void WheelManager_SendAllTargets(void) //发送所有舵轮的目标舵角和转速（调用can接口）
{
    for (uint8_t i = 0; i < 4; i++) 
    {
        //获取第i个舵轮的目标状态
        const WheelTarget_t *wt = Kinematics_GetWheelTarget(i);
        if (wt == NULL) continue;
        CAN_SEND_ANGLE(i, wt->steering_angle_deg);  //发送目标舵角
        CAN_SEND_RPM(i, wt->wheel_rpm);             //发送目标转速
    }
}

//更新舵轮实际状态（can接受回调调用）
void WheelManager_UpdateActual(uint8_t wheel_id, float angle_deg, float rpm)
{
    if (wheel_id >= 4) return;//舵轮id越界则返回
    //更新实际舵角/转速
    wheel_actuals[wheel_id].actual_steering_angle_deg = angle_deg;
    wheel_actuals[wheel_id].actual_wheel_rpm = rpm;
    wheel_actuals[wheel_id].is_online = true;         //标记为在线
    wheel_actuals[wheel_id].last_update_tick = HAL_GetTick();//记录更新时间
}


// PID修正转速：在目标转速基础上叠加修正值（限幅后发送）
void WheelManager_ApplyPIDCorrection(uint8_t wheel_id, float correction_rpm)
{
    if (wheel_id >= 4) return;//舵轮id越界则返回
     // 获取目标转速
    const WheelTarget_t *wt = Kinematics_GetWheelTarget(wheel_id);
    if (wt == NULL) return;
    
    //计算修正后的转速
    float corrected = wt->wheel_rpm + correction_rpm;

    // 限幅（与安全模块中的 MAX_RPM 一致） 
    #define MAX_RPM 3000.0f
    if (corrected >  MAX_RPM) corrected =  MAX_RPM;
    if (corrected < -MAX_RPM) corrected = -MAX_RPM;

    // 直接发送修正后的转速，不改变内存中的目标值 
    CAN_SEND_RPM(wheel_id, corrected);
}

void WheelManager_StopAll(void)//急停：所有舵轮舵角归0，转速归0
{
    for (uint8_t i = 0; i < 4; i++) 
    {
        CAN_SEND_ANGLE(i, 0.0f);
        CAN_SEND_RPM(i, 0.0f);
    }
}

bool WheelManager_IsOnline(uint8_t wheel_id) //查询舵轮是否在线
{
    if (wheel_id >= 4) return false;   //id越界返回false
    return wheel_actuals[wheel_id].is_online;
}

uint32_t WheelManager_GetLastUpdateTick(uint8_t wheel_id)//查询舵轮最后时间戳
{
    if (wheel_id >= 4) return 0;
    return wheel_actuals[wheel_id].last_update_tick;
}

const WheelActual_t* WheelManager_GetActual(uint8_t wheel_id)// 获取舵轮实际状态（返回const指针，防止外部修改）
{
    if (wheel_id >= 4) return NULL;
    return &wheel_actuals[wheel_id];
}

