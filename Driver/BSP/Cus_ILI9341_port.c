#include "Cus_ILI9341_port.h"



__weak void lcd_write_cmd( uint8_t cmd )
{
  (void) cmd;
}


__weak void lcd_write_data( uint8_t data )
{
  (void) data;
}


__weak void lcd_write_data16( uint16_t data )
{
  (void) data;
}


__weak uint8_t lcd_read_data( void )
{
  return 0;
}


__weak uint16_t lcd_read_data16( void )
{
  return 0;
}


__weak void lcd_enable_backlight( void )
{
  /* NULL */
}


__weak void lcd_driver_init( void )
{
  /* NULL */
}


__weak void lcd_write_buffer( const uint16_t *buffer, uint32_t len )
{
  (void)buffer;
  (void)len;
}


__weak void lcd_delay_ms( uint32_t ms )
{
  (void)ms;
}
