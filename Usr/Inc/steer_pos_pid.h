#ifndef __STEER_POS_PID_H
#define __STEER_POS_PID_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  初始化4路转向电机的位置环+速度内环串级PID
 * @note   上电调用1次，自动适配4个舵轮的转向电机
 */
void SteerPosPID_Init(void);

/**
 * @brief  转向位置环周期运算函数，每10ms调用1次
 * @note   自动读取目标角度、实际角度/转速，串级计算后下发转速指令
 */
void SteerPosPID_Cycle(void);

/**
 * @brief  重置所有转向PID状态（清除积分、误差历史）
 * @note   急停、模式切换时调用，防止积分饱和
 */
void SteerPosPID_ResetAll(void);

/**
 * @brief  转向电机全部急停：输出0转速 + 重置PID状态
 */
void SteerPosPID_StopAll(void);

#endif