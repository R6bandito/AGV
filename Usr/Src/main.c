#include "main.h"
#include "stm32f1xx_hal.h"
/* 用户模块 */
#include "debug_uart.h"   // 调试串口
#include "kinematics.h"
#include "protocol.h"
#include "agv_state.h"
#include "wheel_manager.h"

UART_HandleTypeDef huart1;  // 与角色1通信的串口

/* ------------------------- 时基分离 ----------------------------- */
TIM_HandleTypeDef htim6;
HAL_StatusTypeDef HAL_InitTick( uint32_t TickPriority );
tftDevice_HandleTypeDef lcd_device;
/* --------------------------------------------------------------- */

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



  lcd_device.lcd_drawString(&lcd_device,10,10,"Waiting Cmd...",CUS_FONT_SIZE_12,CUS_COLOR_RED,CUS_COLOR_WHITE);
  
  SimulateFrame(0.5,0.0,0.0);
  AGV_UpdateLastCmdTick();

  uint32_t last_sec_tick = HAL_GetTick();
  int seconds = 0;

  while (1) 
  {

    /* ===== 1. 安全检测 ===== */
        AGV_SafetyCheck();
        /* ===== 2. 处理协议帧（若收到新帧） ===== */
        if (Protocol_NewFrameReady()) 
        {
            AGV_Command_t cmd;
            if (Protocol_ParseCommand(&cmd)) 
            {
                AGV_UpdateLastCmdTick();     // ★ 刷新指令时间戳
                Kinematics_Inverse(&cmd);
                // 这里可以顺便更新 LCD 运动学数据（阶段3的显示代码）
            }
        }
        /* ===== 3. 每 1 秒更新一次 LCD 状态显示 ===== */
        if (HAL_GetTick() - last_sec_tick >= 1000) 
        {
            last_sec_tick = HAL_GetTick();
            seconds++;
            // 获取当前状态
            AGV_State_t st = AGV_GetState();
            const char *state_str = "???";
            uint16_t state_color = CUS_COLOR_WHITE;
            switch (st) 
            {
                case AGV_STATE_IDLE:    state_str = "IDLE";    state_color = CUS_COLOR_RED;  break;
                case AGV_STATE_RUNNING: state_str = "RUNNING"; state_color = CUS_COLOR_RED;  break;
                case AGV_STATE_ESTOP:   state_str = "ESTOP !"; state_color = CUS_COLOR_RED;  break;
                case AGV_STATE_ERROR:   state_str = "ERROR !"; state_color = CUS_COLOR_RED;  break;
            }
            // 显示状态
            char buf[40];
            snprintf(buf, sizeof(buf), "State: %s", state_str);
            lcd_device.lcd_drawString(&lcd_device, 10, 30, buf,
                                      CUS_FONT_SIZE_12, state_color, CUS_COLOR_BLACK);
            // 显示计时（用于观察超时）
            snprintf(buf, sizeof(buf), "Time: %d s", seconds);
            lcd_device.lcd_drawString(&lcd_device, 10, 50, buf,
                                      CUS_FONT_SIZE_12, CUS_COLOR_WHITE, CUS_COLOR_BLACK);
            // 如果是 ESTOP，给出提示
            if (st >= AGV_STATE_ESTOP) 
            {
                lcd_device.lcd_drawString(&lcd_device, 10, 90,
                    "(No cmd > 500ms)", CUS_FONT_SIZE_16, CUS_COLOR_RED, CUS_COLOR_BLACK);
            }
        }
        HAL_Delay(20);
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

