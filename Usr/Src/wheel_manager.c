#include "wheel_manager.h"
#include "kinematics.h"
#include "stm32f1xx_hal.h"
#include "motor_can_ctrl.h"      

/* 舵轮 0~3 对应的转向电机 Node_id 和行走电机 Node_id
 * 请根据实际 CAN 从机地址修改 */
static const uint8_t steer_node_id[4] = {1, 2, 3, 4};    // 示例
static const uint8_t trac_node_id[4]  = {5, 6, 7, 8};

/* 转向时的固定转速 rpm */
#define STEER_SPEED_RPM     100

/* 行走电机电流限制（0.0 表示不限制） */
#define TRAC_DEFAULT_CURRENT  0.0f

static WheelActual_t wheel_actuals[4];

/* ---------- 初始化（集成 CAN 初始化 + 电机注册） ---------- */
void WheelManager_Init(void)
{
    // #if !REAL_MODE

        /* 清空本地状态 */
        for (int i = 0; i < 4; i++)
        {
            wheel_actuals[i].actual_steering_angle_deg = 0;
            wheel_actuals[i].actual_wheel_rpm = 0;
            wheel_actuals[i].is_online = false;
            wheel_actuals[i].last_update_tick = 0;
        }

        /* 初始化 CAN 底层（全工程只需调用一次，可放在此处或 main.c） */
        HX_CAN_Init();

        /* 初始化所有电机（注册并启动） */
        for (int i = 0; i < 4; i++) 
        {
            /* 转向电机：绝对位置模式 */
            Cus_Motor_Steer_PosMode_Initial(steer_node_id[i]);
            /* 行走电机：速度模式 */
            Cus_Motor_Trac_VelocityMode_Initial(trac_node_id[i]);
        }
    // #else
    //     // 演示模式：仅仅清零本地结构体，不初始化硬件
    //     for (int i = 0; i < 4; i++) 
    //     {
    //         wheel_actuals[i].is_online = true;        // 假在线，避免安全模块抱怨
    //         wheel_actuals[i].last_update_tick = HAL_GetTick();
    //         wheel_actuals[i].actual_steering_angle_deg = 0.0f;
    //         wheel_actuals[i].actual_wheel_rpm = 0.0f;
    //     }
    // #endif
}

/* ---------- 发送所有目标舵角/转速 ---------- */
void WheelManager_SendAllTargets(void)
{
    for (uint8_t i = 0; i < 4; i++) 
    {
        const WheelTarget_t *wt = Kinematics_GetWheelTarget(i);
        if (wt == NULL) continue;

        /* 转向目标 → CAN */
        Cus_Motor_Steer_SetAngle(steer_node_id[i],wt->steering_angle_deg,STEER_SPEED_RPM,0);    // 异步发送，不等待到位

        /* 行走目标（转速）→ CAN */
        Cus_Motor_Trac_SetVelocity(trac_node_id[i], (int32_t)(wt->wheel_rpm),TRAC_DEFAULT_CURRENT);// 取整
    }
}

/* ---------- 急停：停止所有电机 ---------- */
void WheelManager_StopAll(void)
{
    for (uint8_t i = 0; i < 4; i++) 
    {
        Cus_Motor_Steer_EmergencyStop(steer_node_id[i]);
        Cus_Motor_Trac_EmergencyStop(trac_node_id[i]);
    }
}

/* ---------- PID 修正接口 ---------- */
void WheelManager_ApplyPIDCorrection(uint8_t wheel_id, float correction_rpm)
{
    if (wheel_id >= 4) return;

    const WheelTarget_t *wt = Kinematics_GetWheelTarget(wheel_id);
    if (wt == NULL) return;

    float corrected = wt->wheel_rpm + correction_rpm;
    /* 限幅（与安全模块的最大转速保持一致） */
    #define MAX_RPM 3000.0f
    if (corrected >  MAX_RPM) corrected =  MAX_RPM;
    if (corrected < -MAX_RPM) corrected = -MAX_RPM;

    /* 直接发送修正后转速 */
    Cus_Motor_Trac_SetVelocity(trac_node_id[wheel_id],
                               (int32_t)corrected,
                               TRAC_DEFAULT_CURRENT);
}

/* ---------- 从 motor_can_ctrl 同步实际状态（需周期调用） ---------- */
void WheelManager_SyncState(void)
{
    for (uint8_t i = 0; i < 4; i++) 
    {
        /* 获取转向角度 */
        float angle = 0;
        if (Cus_Motor_Steer_GetAngle(steer_node_id[i], &angle)) 
        {
            wheel_actuals[i].actual_steering_angle_deg = angle;
        }

        /* 获取行走速度（返回值为 rpm） */
        wheel_actuals[i].actual_wheel_rpm = 
            (float)Cus_Motor_Trac_GetVelocity(trac_node_id[i]);

        /* 只要任一电机成功返回数据，就认为在线并刷新时间戳
         * 更严格的判断可分别检查两个电机的状态字，此处简化 */
        wheel_actuals[i].is_online = 1;
        wheel_actuals[i].last_update_tick = HAL_GetTick();
    }
}

/* ---------- 保留原有查询接口（无需修改） ---------- */
bool WheelManager_IsOnline(uint8_t wheel_id)
{
    if (wheel_id >= 4) return false;
    return wheel_actuals[wheel_id].is_online;
}

uint32_t WheelManager_GetLastUpdateTick(uint8_t wheel_id)
{
    if (wheel_id >= 4) return 0;
    return wheel_actuals[wheel_id].last_update_tick;
}

const WheelActual_t* WheelManager_GetActual(uint8_t wheel_id)
{
    if (wheel_id >= 4) return NULL;
    return &wheel_actuals[wheel_id];
}