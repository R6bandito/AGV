#include "main.h"
#include "stm32f1xx_hal.h"
/* 用户模块 */
#include "debug_uart.h"   // 调试串口
#include "kinematics.h"
#include "protocol.h"
#include "agv_state.h"
#include "wheel_manager.h"

#include "Cus_ILI9341.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "debug_uart.h"
#include "Cus_ILI9341.h"
#include "motor_can_ctrl.h"


/* ------------------------- 时基分离 -------------------------- */
UART_HandleTypeDef huart1;  // 与角色1通信的串口#include "mb.h"
HAL_StatusTypeDef HAL_InitTick( uint32_t TickPriority );
tftDevice_HandleTypeDef lcd_device;
/* --------------------------------------------------------------- */


/* --------------------------------------------------------------- */
static void motor_test( void );
void CAN_Force_Start( void );
void CAN_forceStart( void );
/* --------------------------------------------------------------- */

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);

/* ---------- UART 中断回调 ---------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t ch = (uint8_t)(huart->Instance->DR);
        Protocol_FeedByte(ch);       // 喂给协议解析器
        // 重新开启单字节中断接收
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&ch, 1); // 注意需临时变量
    }
}

//---------- LCD 辅助：指定位置画浮点数 ---------- 
static void LCD_DrawFloat(uint16_t x, uint16_t y, float val, uint16_t color, uint16_t bg)
{
    char buf[24];
    sprintf(buf, "%.2f", (double)val);
    lcd_device.lcd_drawString(&lcd_device, x, y, buf,CUS_FONT_SIZE_12, color, bg);
}

static void SimulateFrame(float vx, float vy, float omega)
{
    uint8_t raw[16];
    raw[0] = 0xAA;
    raw[1] = 0x55;
    raw[2] = 0x01;   // CMD_MOTION
    memcpy(&raw[3],  &vx,    4);
    memcpy(&raw[7],  &vy,    4);
    memcpy(&raw[11], &omega, 4);
    raw[15] = 0x00;  // 假 CRC（因校验被注释）
    for (int i = 0; i < 16; i++) {
        Protocol_FeedByte(raw[i]);
    }
}
int main( void )
{
  HAL_Init();

  SystemClock_Config_72Mhz();

  debug_uart_Init();
  
  // MX_USART1_UART_Init();
  Kinematics_Init(AGV_WHEELBASE_X, AGV_WHEELBASE_Y, AGV_WHEEL_RADIUS);
  Protocol_Init();
  AGV_State_Init();
  WheelManager_Init();

     /* 启动串口单字节中断接收 */
  uint8_t rx_ch;
  HAL_UART_Receive_IT(&huart1, &rx_ch, 1);

  /* LCD初始化. */
  Cus_ILI9341_InitHandle(&lcd_device);
  lcd_device.displayInit(&lcd_device, ILI9341_ROTATION_0);

  while(1)
  {
        
  }
}


HAL_StatusTypeDef HAL_InitTick( uint32_t TickPriority )
{
  /* 重定义 HAL_InitTick ，进行时基分离. Systick交由FreeRTOS配置. */
  /* 初始化基本定时器 TIM6 作为HAL库的tick时基基准. */
  __HAL_RCC_TIM6_CLK_ENABLE();
  
  htim6.Instance = TIM6;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

  uint32_t pclk1_freq = HAL_RCC_GetPCLK1Freq();
  uint32_t presc = (2 * pclk1_freq / 1000000 == 0) ? (2 * pclk1_freq / 100000) : (2 * pclk1_freq / 1000000);

  // Period = (1MHz * 0.001s) - 1 = 1000 - 1 = 999. (0.001s = 1ms)
  htim6.Init.Period = 1000 - 1;
  htim6.Init.Prescaler = presc - 1;
  htim6.Init.RepetitionCounter = 0;

  if ( HAL_TIM_Base_Init(&htim6) != HAL_OK )
  {
    for( ; ; );
  }

  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  HAL_NVIC_SetPriority(TIM6_IRQn, TickPriority, 0);

  HAL_TIM_Base_Start_IT(&htim6);
  return HAL_OK;
}


void CAN_forceStart( void )
{
  // 先清除 WKUI
  if ( CAN1->MSR & CAN_MSR_WKUI ) 
  {
    CAN1->MSR = CAN_MSR_WKUI;
  }

  // 确保不在睡眠模式
  CAN1->MCR &= ~CAN_MCR_SLEEP;

  // 请求退出初始化（清除 INRQ）
  CAN1->MCR &= ~CAN_MCR_INRQ;

  // 等待 INAK 清零，但给一个短超时（5000 次循环）
  uint32_t timeout = 5000;
  while ( (CAN1->MSR & CAN_MSR_INAK) && timeout ) 
  {
    timeout--;
  }

  // 如果超时了（INAK 仍为 1），发送一帧空数据来强制唤醒
  if ( CAN1->MSR & CAN_MSR_INAK ) 
  {
    // 等待邮箱空闲
    while ((CAN1->TSR & CAN_TSR_TME0) == 0);

    // 发送一个“虚拟帧”（DLC=0，ID=0x7，不影响总线）
    CAN1->sTxMailBox[0].TIR = (0x7 << 21) | CAN_TI0R_TXRQ;
    CAN1->sTxMailBox[0].TDTR = 0;            // DLC=0，无数据
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    // 再次等待 INAK 清零（这次应该很快）
    timeout = 1000;
    while ((CAN1->MSR & CAN_MSR_INAK) && timeout) 
    {
      timeout--;
    }
  }
}


static void motor_test( void )
{
  /* 速度控制模式. */
  Cus_Motor_Trac_VelocityMode_Initial(0x0A);

  /* 绝对位置模式. (转向电机) */
  Cus_Motor_Steer_PosMode_Initial(0x0B);

  /* 设置节点0x0A. 速度1500rpm. 驱动电流2A. */
  Cus_Motor_Trac_SetVelocity(0x0A, 1500, 2.0);

  /* 设置节点0x0A. 转向速度400Rpm. 角度+75°. 异步. */
  Cus_Motor_Steer_SetAngle(0x0B, 75.0, 400, 0);
}


