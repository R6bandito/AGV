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

#include "port.h"
#include "stm32f1xx_hal.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

// 全局串口句柄（main.c中定义extern UART_HandleTypeDef huart1;）
UART_HandleTypeDef huart2;

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	// 接收中断开关
    if (xRxEnable == TRUE)
    {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
        }
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
    }

    // 发送完成中断开关
    if (xTxEnable == TRUE)
    {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
        }
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity, UCHAR ucStopBits )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 开启GPIOA、USART2时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    // PA2 TX 复用推挽输出
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA3 RX 浮空输入
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 串口参数配置
    huart2.Instance = USART2;
    huart2.Init.BaudRate = ulBaudRate;
    // 数据位
    huart2.Init.WordLength = (ucDataBits == 9) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    // 停止位
    huart2.Init.StopBits = (ucStopBits == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    // 校验位
    switch(eParity)
    {
        case MB_PAR_ODD:  huart2.Init.Parity = UART_PARITY_ODD; break;
        case MB_PAR_EVEN: huart2.Init.Parity = UART_PARITY_EVEN; break;
        default:          huart2.Init.Parity = UART_PARITY_NONE; break;
    }
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&huart2) != HAL_OK)
    {
        return FALSE;
    }

    // 中断配置
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    // 开启接收中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    // 直接写入串口数据寄存器（非阻塞），由TC中断触发下一次发送
    huart2.Instance->DR = (uint8_t)ucByte;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	  // 读取串口接收寄存器缓存数据
    *pucByte = (CHAR)huart2.Instance->DR;
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}


void USART2_IRQHandler(void)
{
    uint32_t temp_flag = 0;
    uint32_t temp_data = 0;

    temp_flag = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE);
    if((temp_flag != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE) != RESET))
    {
        prvvUARTRxISR(); // 触发接收回调
        temp_data = huart2.Instance->DR; // 读DR寄存器清除RXNE标志
        UNUSED(temp_data);
    }

    temp_flag = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC);
    if((temp_flag != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TC) != RESET))
    {
        prvvUARTTxReadyISR(); // 触发发送回调
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC); // 清除TC标志
    }
}

