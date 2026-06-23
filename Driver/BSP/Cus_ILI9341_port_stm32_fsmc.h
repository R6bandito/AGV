#ifndef __CUS_ILI9341_FSMC_H__
#define __CUS_ILI9341_FSMC_H__


#include "stm32f1xx_hal.h"


/* ***************** LCD_Conf ***************** */
  /* 以下配置 LCD 的 FSMC 地址映射.请根据原理图中的片选连接，正确设置基地址及命令/数据信号位，*/
  /* 确保 FSMC 能够正确控制片选引脚并区分命令与数据的访问周期. */
  #define LCD_BASE                         (0x6C000000UL)
  #define LCD_CMD_ADDR                     (*(volatile uint16_t *)LCD_BASE)
  #define LCD_DAT_ADDR                     (*(volatile uint16_t *)(LCD_BASE | (0x01 << 11)))

/* ***************** LCD_Conf ***************** */


/* ***************** FSMC_Pin_Conf ***************** */
  /* 以下是 LCD 的 CS/WR/RS/RD 引脚所对应的 FSMC接口引脚配置(FSMC_NEx/FSMC_NWE/FSMC_Ax/FSMC_NOE). */
  /* 请务必仔细参考所用芯片原理图以及所用LCD原理图. 确保以下引脚映射到正确位置. */
  #define LCD_CS_PORT                      (GPIOG)
  #define LCD_CS_PIN                       (GPIO_PIN_12)

  #define LCD_WR_PORT                      (GPIOD)
  #define LCD_WR_PIN                       (GPIO_PIN_5)

  #define LCD_RS_PORT                      (GPIOG)
  #define LCD_RS_PIN                       (GPIO_PIN_0)

  #define LCD_RD_PORT                      (GPIOD)
  #define LCD_RD_PIN                       (GPIO_PIN_4)

  /* 背光控制引脚. */
  #define LCD_BL_PORT                      (GPIOB)
  #define LCD_BL_PIN                       (GPIO_PIN_0)

  /* 以下是基于STM32F103ZET6的FSMC数据总线接口引脚配置. */
  /* 请务必仔细核实每一位所对应引脚是否与所使用芯片相同. */
  #define LCD_D0_PORT                      (GPIOD)
  #define LCD_D0_PIN                       (GPIO_PIN_14)
  #define LCD_D1_PORT                      (GPIOD)
  #define LCD_D1_PIN                       (GPIO_PIN_15)
  #define LCD_D2_PORT                      (GPIOD)
  #define LCD_D2_PIN                       (GPIO_PIN_0)
  #define LCD_D3_PORT                      (GPIOD)
  #define LCD_D3_PIN                       (GPIO_PIN_1)
  #define LCD_D4_PORT                      (GPIOE)
  #define LCD_D4_PIN                       (GPIO_PIN_7)
  #define LCD_D5_PORT                      (GPIOE)
  #define LCD_D5_PIN                       (GPIO_PIN_8)
  #define LCD_D6_PORT                      (GPIOE)
  #define LCD_D6_PIN                       (GPIO_PIN_9)
  #define LCD_D7_PORT                      (GPIOE)
  #define LCD_D7_PIN                       (GPIO_PIN_10)
  #define LCD_D8_PORT                      (GPIOE)
  #define LCD_D8_PIN                       (GPIO_PIN_11)
  #define LCD_D9_PORT                      (GPIOE)
  #define LCD_D9_PIN                       (GPIO_PIN_12)
  #define LCD_D10_PORT                     (GPIOE)
  #define LCD_D10_PIN                      (GPIO_PIN_13)
  #define LCD_D11_PORT                     (GPIOE)
  #define LCD_D11_PIN                      (GPIO_PIN_14)
  #define LCD_D12_PORT                     (GPIOE)
  #define LCD_D12_PIN                      (GPIO_PIN_15)
  #define LCD_D13_PORT                     (GPIOD)
  #define LCD_D13_PIN                      (GPIO_PIN_8)
  #define LCD_D14_PORT                     (GPIOD)
  #define LCD_D14_PIN                      (GPIO_PIN_9)
  #define LCD_D15_PORT                     (GPIOD)
  #define LCD_D15_PIN                      (GPIO_PIN_10)
/* ***************** FSMC_Pin_Conf ***************** */


/* ***************** FSMC_Param_Conf ***************** */
  #define FSMC_READ_ADDR_SETUP_TIME        (0x00)
  #define FSMC_READ_ADDR_HOLD_TIME         (0x00)
  #define FSMC_READ_DATA_SETUP_TIME        (0x0F)

  #define FSMC_WRITE_ADDR_SETUP_TIME       (0x00)
  #define FSMC_WRITE_ADDR_HOLD_TIME        (0x00)
  #define FSMC_WRITE_DATA_SETUP_TIME       (0x01)

  #define FSMC_ACCESS_MODE                 (FSMC_ACCESS_MODE_A)
  #define FSMC_BUS_AROUND_DURATION         (0x00)
  #define FSMC_NSBANK                      (FSMC_NORSRAM_BANK4)
/* ***************** FSMC_Param_Conf ***************** */ 


#endif /* __CUS_ILI9341_FSMC_H__ */

