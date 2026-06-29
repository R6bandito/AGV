/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
extern TIM_HandleTypeDef htim7;

/* ----------------------- static functions ---------------------------------*/
void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
/**
 * @brief Modbus定时器底层初始化 HAL库版本，TIM6基本定时器
 * @param usTim1Timerout50us 50us单位溢出计数
 * @return BOOL 初始化成功返回TRUE
 */
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    // 开启TIM7外设时钟
    __HAL_RCC_TIM7_CLK_ENABLE();

    // 绑定TIM7外设
    htim7.Instance = TIM7;

    // 直接给句柄内部Init结构体赋值（标准HAL规范写法）
    htim7.Init.Prescaler = 3600 - 1;                // 预分频72M/3600=20KHz，单周期50us
    htim7.Init.Period = usTim1Timerout50us;         // 溢出计数值
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;

    if(HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        return FALSE;
    }

    // 开启更新中断
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

    // NVIC中断配置
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    return TRUE;
}


inline void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
	  // 清除定时器更新中断标志
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    // 将计数器清零
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    // 开启定时器中断模式
    HAL_TIM_Base_Start_IT(&htim7);
}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    // 停止定时器并关闭更新中断
    HAL_TIM_Base_Stop_IT(&htim7);
    // 清除更新中断标志
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    // 计数器清零
    __HAL_TIM_SET_COUNTER(&htim7, 0);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

