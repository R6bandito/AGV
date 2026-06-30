#include "speed_pid_base.h"
#include <math.h>

void SpeedPID_Init(SpeedPID_t *pid, const SpeedPID_Config_t *config)
{
    pid->config = *config; //解引用

    // 状态变量清零
    pid->target_raw = 0.0f;      //原始目标转速
    pid->target_ramp = 0.0f;     //经过加减速斜坡平滑后的目标转速，不会突变
    pid->actual_raw = 0.0f;      //从编码器读回来的原始实际转速
    pid->actual_filter = 0.0f;   // 经过低通滤波后的实际转速
    pid->error = 0.0f;           //当前转速误差
    pid->last_error = 0.0f;      //上一个周期的转速误差
    pid->integral = 0.0f;        //积分累积量
    pid->output = 0.0f;          //PID 最终输出的转速
    pid->zero_timer = 0;         //零速计时器
    pid->pi_stage = 1;           // 默认低速段
}

void SpeedPID_Reset(SpeedPID_t *pid)
{
    //运行状态全部清空
    pid->target_ramp = 0.0f;
    pid->actual_filter = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->zero_timer = 0;
    pid->pi_stage = 1;
}

/* 内部：加减速斜坡处理 ，加了斜坡之后，目标转速会从 0 开始，按固定速率涨，电机平稳加速，没有冲击。*/
static void RampProcess(SpeedPID_t *pid, float target, uint32_t dt_ms)
{
    //未使能不做处理
    if (!pid->config.ramp_enable) 
    {
        pid->target_ramp = target;
        return;
    }

    float step_up = pid->config.ramp_up_rate * dt_ms; //计算本周期内允许增加的最大转速
    float step_dn = pid->config.ramp_dn_rate * dt_ms; //允许减少的最大转速

    if (target > pid->target_ramp) //目标转速比当前斜坡值大，说明是加速过程
    {
        pid->target_ramp += step_up; //慢慢往目标靠
        if (pid->target_ramp > target) //超过了目标值，就直接等于目标值，防止超调
        pid->target_ramp = target; 
    }
    else if (target < pid->target_ramp) 
    {
        pid->target_ramp -= step_dn; //慢降到目标
        if (pid->target_ramp < target) pid->target_ramp = target;
    }
}

/* 内部：一阶低通滤波 ，平滑处理带噪声的转速信号*/
static void FilterProcess(SpeedPID_t *pid, float actual, uint32_t dt_ms)
{
    //计算滤波系数
    float alpha = (float)dt_ms / (pid->config.filter_tau + (float)dt_ms);
    //新滤波值 = 旧滤波值 + 系数 × (新采样值 - 旧滤波值)
    pid->actual_filter += alpha * (actual - pid->actual_filter);
}

/* 内部：三段PI计算 */
static void ThreeStagePI(SpeedPID_t *pid, uint32_t dt_ms)
{
    // 1. 判断当前PI段
    if (fabsf(pid->actual_filter) < 1.0f) {
        pid->zero_timer += dt_ms;
        if (pid->zero_timer >= pid->config.zero_wait_ms) {
            pid->pi_stage = 0;
        } else {
            pid->pi_stage = 1;
        }
    } else {
        pid->zero_timer = 0;
        if (fabsf(pid->actual_filter) > pid->config.switch_speed) {
            pid->pi_stage = 2;
        } else {
            pid->pi_stage = 1;
        }
    }

    // 2. 选择对应PI参数
    float kp, ki;
    switch (pid->pi_stage) {
        case 0:  kp = pid->config.kp0;  ki = pid->config.ki0;  break;
        case 2:  kp = pid->config.kp2;  ki = pid->config.ki2;  break;
        default: kp = pid->config.kp1;  ki = pid->config.ki1;  break;
    }

    // 3. 计算误差
    pid->last_error = pid->error;
    pid->error = pid->target_ramp - pid->actual_filter;

    // 4. 比例项
    float p_out = kp * pid->error;

    // 5. 积分项（抗积分饱和）
    if (pid->output > pid->config.output_min && pid->output < pid->config.output_max) {
        pid->integral += pid->error;
    }
    float i_out = ki * pid->integral;

    // 6. 总输出+限幅
    pid->output = p_out + i_out;
    if (pid->output > pid->config.output_max) pid->output = pid->config.output_max;
    if (pid->output < pid->config.output_min) pid->output = pid->config.output_min;
}

float SpeedPID_Calc(SpeedPID_t *pid, float target_rpm, float actual_rpm, uint32_t dt_ms)
{
    pid->target_raw = target_rpm;
    pid->actual_raw = actual_rpm;

    // 步骤1：目标转速斜坡处理
    RampProcess(pid, target_rpm, dt_ms);
    // 步骤2：实际转速滤波
    FilterProcess(pid, actual_rpm, dt_ms);
    // 步骤3：三段PI运算
    ThreeStagePI(pid, dt_ms);

    return pid->output;
}