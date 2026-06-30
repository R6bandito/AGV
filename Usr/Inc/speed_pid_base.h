#ifndef __SPEED_PID_BASE_H
#define __SPEED_PID_BASE_H

#include <stdint.h>
#include <stdbool.h>

/* 通用速度PID配置结构体 */
typedef struct {
    // 三段PI参数
    float kp0;    // 零速段KP
    float ki0;    // 零速段KI
    float kp1;    // 低速段KP
    float ki1;    // 低速段KI
    float kp2;    // 高速段KP
    float ki2;    // 高速段KI

    // 切换阈值
    float switch_speed;   // 低速→高速切换转速(rpm)
    uint32_t zero_wait_ms;// 零速段切入延时(ms)

    // 功能开关
    bool ramp_enable;     // 是否启用加减速斜坡
    float ramp_up_rate;   // 加速速率(rpm/ms)
    float ramp_dn_rate;   // 减速速率(rpm/ms)

    float filter_tau;     // 反馈滤波时间常数(ms)
    float output_max;     // 输出上限
    float output_min;     // 输出下限
} SpeedPID_Config_t;

/* 通用速度PID运行状态结构体 */
typedef struct {
    SpeedPID_Config_t config;

    // 内部状态
    float target_raw;
    float target_ramp;
    float actual_raw;
    float actual_filter;

    float error;
    float last_error;
    float integral;
    float output;

    uint32_t zero_timer;
    uint8_t  pi_stage;  // 0=零速段 1=低速段 2=高速段
} SpeedPID_t;

/* 初始化速度PID */
void SpeedPID_Init(SpeedPID_t *pid, const SpeedPID_Config_t *config);

/* 重置速度PID状态 */
void SpeedPID_Reset(SpeedPID_t *pid);

/* 单周期速度PID计算，dt_ms为调用周期(固定10ms) */
float SpeedPID_Calc(SpeedPID_t *pid, float target_rpm, float actual_rpm, uint32_t dt_ms);

#endif