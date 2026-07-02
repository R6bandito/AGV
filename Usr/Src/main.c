#include "main.h"
#include "stm32f1xx_hal.h"


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


//---------- LCD 辅助：指定位置画浮点数 ---------- 
static void LCD_DrawFloat(uint16_t x, uint16_t y, float val, uint16_t color, uint16_t bg)
{
    char buf[24];
    sprintf(buf, "%.2f", (double)val);
    lcd_device.lcd_drawString(&lcd_device, x, y, buf,CUS_FONT_SIZE_12, color, bg);
}

static void LCD_Clear(void)
{
    lcd_device.setWindow(&lcd_device, 0, 0, 240, 320);
    lcd_device.lcd_fill(&lcd_device, CUS_COLOR_WHITE);
}

static void LCD_ShowNormal(const char *mode_str, const AGV_Command_t *cmd)
{
    lcd_device.lcd_drawString(&lcd_device, 10, 10, mode_str,
                              CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
    if (cmd->is_valid)
    {
        lcd_device.lcd_drawString(&lcd_device, 10, 40, "Vx=", CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
        LCD_DrawFloat(40, 40, cmd->vx, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
        lcd_device.lcd_drawString(&lcd_device, 90, 40, "Vy=", CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
        LCD_DrawFloat(120, 40, cmd->vy, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
        lcd_device.lcd_drawString(&lcd_device, 170, 40, "W=", CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
        LCD_DrawFloat(200, 40, cmd->omega, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
    }
    const char* wheel_name[] = {"FL","FR","RL","RR"};
    for (int i = 0; i < 4; i++)
    {
        const WheelTarget_t *wt = Kinematics_GetWheelTarget(i);
        if (wt)
        {
            uint16_t y = 100 + i * 35;
            lcd_device.lcd_drawString(&lcd_device, 10, y, wheel_name[i],
                                      CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
            lcd_device.lcd_drawString(&lcd_device, 45, y, "ang:",
                                      CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
            LCD_DrawFloat(85, y, wt->steering_angle_deg, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
            lcd_device.lcd_drawString(&lcd_device, 140, y, "rpm:",
                                      CUS_FONT_SIZE_16, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
            LCD_DrawFloat(180, y, wt->wheel_rpm, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
        }
    }
}

DemoMode_t demo_mode = DEMO_MODE_MODBUS;   // 当前演示模式，可手动修改

int main( void )
{
  HAL_Init();

  SystemClock_Config_72Mhz();

  debug_uart_Init();
  
  // MX_USART1_UART_Init();
  Kinematics_Init(AGV_WHEELBASE_X, AGV_WHEELBASE_Y, AGV_WHEEL_RADIUS);

  AGV_State_Init();
  WheelManager_Init();

  HX_CAN_Init();

  /* LCD初始化. */
  Cus_ILI9341_InitHandle(&lcd_device);
  lcd_device.displayInit(&lcd_device, ILI9341_ROTATION_0);

      // Modbus初始化：模式、从站地址、串口号、波特率、校验、停止位
  eMBErrorCode eStatus = eMBInit(MB_RTU, 0x01, 2, 115200, MB_PAR_NONE, 1);
  if(eStatus == MB_ENOERR)
    {
        eMBEnable();
    }

  while(1)
  {
    eMBPoll();   // Modbus 协议栈轮询（两种模式均需要）

            /* ---------- 急停检测 ---------- */
            if (usRegHoldingBuf[MODBUS_REG_ESTOP] != 0)
            {
                AGV_EmergencyStop();
                LCD_Clear();
                
            }
            else
            {
                /* 1. 从 Modbus 读取指令并解算 */
                Kinematics_UpdateFromModbus();
                /* 2. 刷新指令时间戳 + 安全检查 */
                AGV_ForceValidCmd();
                AGV_SafetyCheck();
                
                /* 3. 真实模式：将解算结果通过 CAN 发送给电机 */
                 WheelManager_SendAllTargets();

                /* 4. LCD 显示（统一接口） */
                const AGV_Command_t *cmd = Kinematics_GetCurrentCommand();
                #if !REAL_MODE
                    LCD_ShowNormal("Mode: SIMULATE", cmd);
                #else
                    LCD_ShowNormal("Mode: REAL", cmd);
                #endif
            }
            HAL_Delay(50);
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


