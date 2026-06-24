#include "main.h"


/* ------------------------- 时基分离 ----------------------------- */
TIM_HandleTypeDef htim6;
HAL_StatusTypeDef HAL_InitTick( uint32_t TickPriority );
tftDevice_HandleTypeDef lcd_device;
/* --------------------------------------------------------------- */


int main( void )
{
  HAL_Init();

  SystemClock_Config_72Mhz();

//  debug_uart_Init();
  
  printf("New Project Test OK! \n\n");

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

