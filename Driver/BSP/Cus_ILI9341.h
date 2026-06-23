#ifndef __CUS_ILI9341_H__
#define __CUS_ILI9341_H__

#include <stdint.h>
#include <stdbool.h>


/* **************** Font Conf **************** */
  /* 以下宏用于决定是否启用对应字体. */
  #define ILI9341_FONT_12                 (1)           // 默认字体保持开启.(不建议关闭该字体)
  #define ILI9341_FONT_16                 (0)
  #define ILI9341_FONT_24                 (0)
  #define ILI9341_FONT_32                 (0)
/* **************** Font Conf **************** */

/* **************** Resolution Conf **************** */
  /* 屏幕分辨率. 一般无需手动改动. */
  #define LCD_SCREEN_WIDTH                (240)
  #define LCD_SCREEN_HEIGHT               (320)
/* **************** Resolution Conf **************** */

/* **************** Defines **************** */
  /* 以下旋转方向均按顺时针进行组织. */
  #define ILI9341_ROTATION_0               (0)
  #define ILI9341_ROTATION_90              (1)
  #define ILI9341_ROTATION_180             (2)
  #define ILI9341_ROTATION_270             (3)

  #define ID_ILI9341                       (0x9341UL)
  #define ID_ST7789                        (0x7789UL)

  #define CUS_COLOR_WHITE                  (0xFFFFUL)
  #define CUS_COLOR_BLACK                  (0x0000UL)
  #define CUS_COLOR_RED                    (0xF800UL)

  #define CUS_LINE_THIN                    (1)        // 默认细线.
  #define CUS_LINE_NORMAL                  (2)        // 常规加粗.
  #define CUS_LINE_THICK                   (3)        // 中等粗线.
  #define CUS_LINE_BOLD                    (4)        // 粗线.

  /* 以下宏用于作为字体大小参数传入对应API.(所用字体需保证已开启) */
  #define CUS_FONT_SIZE_12                 (0)
  #define CUS_FONT_SIZE_16                 (1)
  #define CUS_FONT_SIZE_24                 (2)
  #define CUS_FONT_SIZE_32                 (3)

  /* 默认主题背景色. */
  #define CUS_THEME_DEFAULT_BG             CUS_COLOR_WHITE
/* **************** Defines **************** */

/* *********************** ILI9341_Device_Struct *********************** */
typedef struct tftDevice tftDevice_HandleTypeDef;
struct tftDevice
{
  /* 基本属性(不对外开放) */
  /* 若需获取当前设备属性. 请使用公共API. */
  void *Attr;

  int8_t (*displayInit)( tftDevice_HandleTypeDef *dev, uint8_t rotation );
  void (*displayOn)( void );
  void (*setWindow)( tftDevice_HandleTypeDef *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 );      // 开窗回调.
  void (*lcd_fill)( tftDevice_HandleTypeDef *dev, uint16_t color );
  void (*lcd_drawPixel)( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, uint16_t color );    // 画点函数(频繁开窗.性能低).
  void (*lcd_drawHLine)( tftDevice_HandleTypeDef *dev, uint16_t pos_Y, uint16_t start_x, uint16_t len, uint8_t thickness, uint16_t color );         // 画水平线.
  void (*lcd_drawVLine)( tftDevice_HandleTypeDef *dev, uint16_t pos_X, uint16_t start_y, uint16_t len, uint8_t thickness, uint16_t color );         // 画垂线.
  void (*lcd_drawLine)( tftDevice_HandleTypeDef *dev, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t thickness, uint16_t color );          // 画任意斜率斜线.
  void (*lcd_drawRect)( tftDevice_HandleTypeDef *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t thickness, uint16_t color );      // 画矩形边框(无填充).
  void (*lcd_drawFillCircle)( tftDevice_HandleTypeDef *dev, int16_t x, int16_t y, int16_t r, uint16_t color );                                      // 画实心圆.
  void (*lcd_drawCircle)( tftDevice_HandleTypeDef *dev, int16_t x, int16_t y, int16_t r, uint8_t thickness, uint16_t color, uint16_t bg_color );    // 画圆框.
  void (*lcd_drawChar)( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, char chr, uint8_t font_size, uint16_t fg_color, uint16_t bg_color );  // 字符显示.
  void (*lcd_drawString)( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, const char *str, uint8_t font_size, uint16_t fg_color, uint16_t bg_color );   // 字符串显示.
};
/* *********************** ILI9341_Device_Struct *********************** */


/* *********************** Public API *********************** */
void Cus_ILI9341_InitHandle( tftDevice_HandleTypeDef *dev );
/* *********************** Public API *********************** */

#endif /* __CUS_ILI9341_H__ */

