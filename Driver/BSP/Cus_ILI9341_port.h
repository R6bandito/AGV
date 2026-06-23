#ifndef __CUS_ILI9341_PORT_H__
#define __CUS_ILI9341_PORT_H__


#include <stdint.h>



/* ****************** ILI9341_PORT ****************** */
/* 通信原语层. 上层通过该基本API与LCD设备进行通信. */
/* 请根据实际运行环境及通信接口(FSMC / SPI). 正确实现以下方法. */
void lcd_write_cmd( uint8_t cmd );


void lcd_write_data( uint8_t data );


void lcd_write_data16( uint16_t data );


uint8_t lcd_read_data( void );


uint16_t lcd_read_data16( void );


/* 背光控制. */
void lcd_enable_backlight( void );


/* 显存批量写入(选择实现). */
void lcd_write_buffer( const uint16_t *buffer, uint32_t len );


/* LCD初始化需要用到延时. */
void lcd_delay_ms( uint32_t ms );
/* ****************** ILI9341_PORT ****************** */


/* ****************** ILI9341_PORT ****************** */
/* 底层通信接口驱动. 此函数只配置通信环境，不发送 LCD 初始化命令。*/
void lcd_driver_init( void );
/* ****************** ILI9341_PORT ****************** */

#endif /* __CUS_ILI9341_PORT_H__ */


