#include "steer_pos_pid.h"
#include "speed_pid_base.h"
#include "kinematics.h"
#include "wheel_manager.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ========== 底层发送接口（保持原有兼容） ========== */
#ifndef CAN_SIMPLE_MODE
#define CAN_SIMPLE_MODE 1
#endif

#if CAN_SIMPLE_MODE
  #include "debug_uart.h"
  #define STEER_SEND_RPM(id, rpm)   printf("[SteerPID] Motor%d Cmd=%.2f rpm\r\n", id, (double)(rpm))
#else
  extern void CAN_SendWheelSpeed(uint8_t motor_id, float rpm);
  #define STEER_SEND_RPM(id, rpm)   CAN_SendWheelSpeed(id, rpm)
#endif

/* ========== 位置环PID结构体（外环） ========== */
typedef struct {
    float kp;
    float ki;
    float kd;

    float error;
    float last_error;
    float integral;

    float output_max;
    float output_min;
    float output;
} PosPID_t;

/* ========== 内部实例 ========== */
static PosPID_t pos_pid[4];        // 4路位置外环
static SpeedPID_t steer_spd_pid[4];// 4路速度内环（复用通用速度PID模块）

/* ========== 参数配置（调试改这里） ========== */
// 位置外环参数
#define POS_KP        25.0f
#define POS_KI         0.8f
#define POS_KD         3.0f
#define POS_MAX_RPM  300.0f

// 速度内环参数（与行走速度环架构完全一致，仅参数不同）
#define STEER_SPD_KP0    1.2f
#define STEER_SPD_KI0    0.15f
#define STEER_SPD_KP1    1.0f
#define STEER_SPD_KI1    0.12f
#define STEER_SPD_KP2    0.6f
#define STEER_SPD_KI2    0.05f
#define STEER_SWITCH_SPD 100.0f
#define STEER_ZERO_WAIT  2000
#define STEER_FILTER_TAU 5.0f
#define STEER_MAX_RPM    300.0f

/* ========== 内部：位置PID计算 ========== */
static float PosPID_Calc(PosPID_t *pid, float target, float actual)
{
    pid->last_error = pid->error;
    pid->error = target - actual;

    float p_out = pid->kp * pid->error;

    // 抗积分饱和
    if (pid->output > pid->output_min && pid->output < pid->output_max) {
        pid->integral += pid->error;
    }
    float i_out = pid->ki * pid->integral;

    float d_out = pid->kd * (pid->error - pid->last_error);

    pid->output = p_out + i_out + d_out;
    // 限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < pid->output_min) pid->output = pid->output_min;

    return pid->output;
}

/* ========== 对外接口：初始化 ========== */
void SteerPosPID_Init(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        // 初始化位置外环
        pos_pid[i].kp = POS_KP;
        pos_pid[i].ki = POS_KI;
        pos_pid[i].kd = POS_KD;
        pos_pid[i].output_max = POS_MAX_RPM;
        pos_pid[i].output_min = -POS_MAX_RPM;
        pos_pid[i].error = 0;
        pos_pid[i].last_error = 0;
        pos_pid[i].integral = 0;
        pos_pid[i].output = 0;

        // 初始化速度内环（复用通用模块，关闭斜坡，保证位置响应）
        SpeedPID_Config_t cfg = {0};
        cfg.kp0 = STEER_SPD_KP0;
        cfg.ki0 = STEER_SPD_KI0;
        cfg.kp1 = STEER_SPD_KP1;
        cfg.ki1 = STEER_SPD_KI1;
        cfg.kp2 = STEER_SPD_KP2;
        cfg.ki2 = STEER_SPD_KI2;
        cfg.switch_speed = STEER_SWITCH_SPD;
        cfg.zero_wait_ms = STEER_ZERO_WAIT;
        cfg.ramp_enable = false;  // 内环关闭斜坡，避免和位置环减速叠加
        cfg.filter_tau = STEER_FILTER_TAU;
        cfg.output_max = STEER_MAX_RPM;
        cfg.output_min = -STEER_MAX_RPM;

        cfg.ramp_up_rate = 10.0f;      // 加速速率：10 rpm/ms
        cfg.ramp_down_rate = 10.0f;    // 减速速率：10 rpm/ms

        SpeedPID_Init(&steer_spd_pid[i], &cfg);
    }
}

/* ========== 对外接口：周期运算（核心串级控制） ========== */
void SteerPosPID_Cycle(void)
{
    const uint32_t dt_ms = 10;

    for (uint8_t i = 0; i < 4; i++) {
        const WheelTarget_t *target = Kinematics_GetWheelTarget(i);
        const WheelActual_t *actual = WheelManager_GetActual(i);

        if (target == NULL || actual == NULL || !actual->is_online) {
            continue;
        }

        // 第一步：外环位置PID计算，输出目标转速
        float target_spd = PosPID_Calc(&pos_pid[i],
                                       target->steering_angle_deg,
                                       actual->actual_steering_angle_deg);

        // 第二步：内环速度PID计算（复用通用模块，与行走速度环逻辑完全一致）
        float final_cmd = SpeedPID_Calc(&steer_spd_pid[i],
                                        target_spd,
                                        actual->actual_wheel_rpm,
                                        dt_ms);

        // 第三步：下发执行
        uint8_t motor_id = i * 2;
        STEER_SEND_RPM(motor_id, final_cmd);
    }
}

/* ========== 对外接口：重置所有PID ========== */
void SteerPosPID_ResetAll(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        // 重置位置环
        pos_pid[i].error = 0;
        pos_pid[i].last_error = 0;
        pos_pid[i].integral = 0;
        pos_pid[i].output = 0;
        // 重置速度内环
        SpeedPID_Reset(&steer_spd_pid[i]);
    }
}

/* ========== 对外接口：全部急停 ========== */
void SteerPosPID_StopAll(void)
{
    SteerPosPID_ResetAll();
    for (uint8_t i = 0; i < 4; i++) {
        STEER_SEND_RPM(i * 2, 0.0f);
    }
}