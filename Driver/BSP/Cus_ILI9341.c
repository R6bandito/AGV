#include "Cus_ILI9341.h"
#include "Cus_ILI9341_port.h"
#include "Cus_ILI9341_font.h"
#include "Cus_ILI9341_commands.h"
#include <string.h>


/* *********************** Public API *********************** */
void Cus_ILI9341_InitHandle( tftDevice_HandleTypeDef *dev );
/* *********************** Public API *********************** */


/* *********************** CallBack *********************** */
static inline int8_t cus_tft_init( tftDevice_HandleTypeDef *dev, uint8_t rotation );                                  // 设备初始化.
static void cus_tft_setWindow( tftDevice_HandleTypeDef *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 );    // 开窗.
static void cus_tft_fill( tftDevice_HandleTypeDef *dev, uint16_t color );
static inline void cus_tft_displayOn( void );
static void cus_tft_drawPixel( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, uint16_t color );
static void cus_tft_drawHLine( tftDevice_HandleTypeDef *dev, uint16_t pos_Y, uint16_t start_x, uint16_t len, uint8_t thickness, uint16_t color );
static void cus_tft_drawVLine( tftDevice_HandleTypeDef *dev, uint16_t pos_X, uint16_t start_y, uint16_t len, uint8_t thickness, uint16_t color );
static void cus_tft_drawLine( tftDevice_HandleTypeDef *dev, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t thickness, uint16_t color );
static void cus_tft_drawRect( tftDevice_HandleTypeDef *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t thickness, uint16_t color );
static void cus_tft_drawFillCircle( tftDevice_HandleTypeDef *dev, int16_t x, int16_t y, int16_t r, uint16_t color );
static void cus_tft_drawChar( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, char chr, uint8_t font_size, uint16_t fg_color, uint16_t bg_color );
static void cus_tft_drawString( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, const char *str, uint8_t font_size, uint16_t fg_color, uint16_t bg_color );
static void cus_tft_drawCircle( tftDevice_HandleTypeDef *dev, int16_t x, int16_t y, int16_t r, uint8_t thickness, uint16_t color, uint16_t bg_color );
/* *********************** CallBack *********************** */


/* *********************** Restriction API *********************** */
static void coord_convert( tftDevice_HandleTypeDef *dev, uint16_t *x_out, uint16_t *y_out );        // 坐标转换API.
static inline void cus_tft_getChipID( tftDevice_HandleTypeDef *dev );                               // 读取芯片ID.
static inline void cus_tft_init_reg( void );                                                        // 初始化配置.
static inline int8_t cus_tft_setRotation( tftDevice_HandleTypeDef *dev, uint8_t rotation );         // 显示旋转方向设置.
/* *********************** Restriction API *********************** */

/* ********************************** */
typedef struct 
{
  /* 基本属性. */
  const char *dev_name;                // 设备名称.
  uint16_t width;                      // 当前显示宽度（像素）.
  uint16_t height;                     // 当前显示高度（像素）.
  uint8_t pixel_format;                // 像素格式 (0x55 = RGB565, 0x66 = RGB666 等).
  uint8_t MADCTL;                      // 内存访问控制寄存器状态.
  uint16_t xo, yo;                     // 当前逻辑原点相对于物理原点坐标.
  uint8_t rotation;                    // 显示方向控制. (0=默认显示, 1=顺时针90°,2=顺时针180°,3=顺时针270°)

  /* 窗口状态.(注意：窗口的 x0,y0 点与 x1,y1 点为逻辑坐标点. 而不是物理坐标点！) */
  uint16_t windows_x0, windows_y0;     // 当前窗口左上角坐标.
  uint16_t windows_x1, windows_y1;     // 当前窗口右下角坐标.

  /* 区域刷新模式相关. */
  bool is_partialMode_enable;          // 区域刷新模式是否开启.
  uint16_t partial_startRow;           // 部分刷新起始行.
  uint16_t partial_endRow;             // 部分刷新结束行.

  bool is_Init;                        // 是否经过初始化标志.
  
} deviceAttr_t;

static deviceAttr_t deviceAttrInstance;

#define DevATTR(attr)   (*((deviceAttr_t *)(attr)))
#define ABS(x)          ((x) < 0 ? -(x) : (x))                // 求取绝对值.(不应传入 x++ 此类表达式)
#define SIGN(x)         ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))    // 符号解析.
/* ********************************** */


/* ********************************** */
typedef struct 
{
  uint8_t width;             // 字宽.
  uint8_t height;            // 字高.
  uint8_t bytes_per_char;    // 每个字符占用的字节数.
  const uint8_t *font_data;  // 二维字模一维化.

} font_info_t;

/* 默认字号12. */
static const font_info_t font_12 = {
  .width = ILI9341_FONT_12_WIDTH,
  .height = ILI9341_FONT_12_HEIGHT,
  .bytes_per_char = ILI9341_FONT_12_SIZE,
  .font_data = (const uint8_t *)ili9341_font_12x6
};

#if (ILI9341_FONT_16)
/* 字号16 */
static const font_info_t font_16 = {
  .width = ILI9341_FONT_16_WIDTH,
  .height = ILI9341_FONT_16_HEIGHT,
  .bytes_per_char = ILI9341_FONT_16_SIZE,
  .font_data = (const uint8_t *)ili9341_font_16x8
};
#endif /* ILI9341_FONT_16 */

#if (ILI9341_FONT_24)
/* 字号24 */
static const font_info_t font_24 = {
  .width = ILI9341_FONT_24_WIDTH,
  .height = ILI9341_FONT_24_HEIGHT,
  .bytes_per_char = ILI9341_FONT_24_SIZE,
  .font_data = (const uint8_t *)ili9341_font_24x12
};
#endif /* ILI9341_FONT_24 */

#if (ILI9341_FONT_32)
/* 字号32 */
static const font_info_t font_32 = {
  .width = ILI9341_FONT_32_WIDTH,
  .height = ILI9341_FONT_32_HEIGHT,
  .bytes_per_char = ILI9341_FONT_32_SIZE,
  .font_data = (const uint8_t *)ili9341_font_32x16
};
#endif /* ILI9341_FONT_32 */

static const font_info_t *cus_tft_getFont( uint8_t font_size );
/* ********************************** */


/* *********************** Public API *********************** */
void Cus_ILI9341_InitHandle( tftDevice_HandleTypeDef *dev )
{
  if ( !dev )   return;

  dev->Attr = (void *)&deviceAttrInstance;
  deviceAttr_t *attr = &DevATTR(dev->Attr);
  attr->dev_name            = " ";
  attr->width               = LCD_SCREEN_WIDTH;
  attr->height              = LCD_SCREEN_HEIGHT;
  attr->pixel_format        = 0x66;    // 芯片默认18bit模式.
  attr->MADCTL              = 0x00;
  attr->xo                  = 0;
  attr->yo                  = 0;
  attr->rotation            = 0;

  attr->windows_x0          = 0;
  attr->windows_y0          = 0;
  attr->windows_x1          = LCD_SCREEN_WIDTH - 1;
  attr->windows_y1          = LCD_SCREEN_HEIGHT - 1;

  attr->is_partialMode_enable     = false;
  attr->partial_startRow          = 0;
  attr->partial_endRow            = 0;

  attr->is_Init                   = true;

  dev->displayInit    = cus_tft_init;
  dev->setWindow      = cus_tft_setWindow;
  dev->displayOn      = cus_tft_displayOn;
  dev->lcd_fill       = cus_tft_fill;
  dev->lcd_drawPixel  = cus_tft_drawPixel;
  dev->lcd_drawHLine  = cus_tft_drawHLine;
  dev->lcd_drawVLine  = cus_tft_drawVLine;
  dev->lcd_drawRect   = cus_tft_drawRect;
  dev->lcd_drawChar   = cus_tft_drawChar;
  dev->lcd_drawLine   = cus_tft_drawLine;
  dev->lcd_drawString = cus_tft_drawString;
  dev->lcd_drawFillCircle = cus_tft_drawFillCircle;
  dev->lcd_drawCircle = cus_tft_drawCircle;
}



/* *********************** Public API *********************** */


/* *********************** CallBack *********************** */
static inline int8_t cus_tft_init( tftDevice_HandleTypeDef *dev, uint8_t rotation )
{
  if ( !dev || !DevATTR(dev->Attr).is_Init )   return -1;

  /* 初始化由用户提供的底层接口驱动. */
  lcd_driver_init();

  /* 获取设备ID. */
  cus_tft_getChipID(dev);
  if ( strcmp(DevATTR(dev->Attr).dev_name, "ILI9341") != 0 )  
  {
    /* 本驱动只面向ILI9341芯片. */
    return -1;
  }

  /* 驱动芯片初始参数配置. */
  cus_tft_init_reg();

  /* 设置显示旋转方向. */
  int8_t res = cus_tft_setRotation(dev, rotation);
  if ( res < 0 )    return -1;

  /* 清屏. */
  dev->lcd_fill(dev, CUS_THEME_DEFAULT_BG);

  /* 开显示. */
  dev->displayOn();

  /* 开背光. */
  lcd_enable_backlight();

  return 1;
}


static void cus_tft_setWindow( tftDevice_HandleTypeDef *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 )
{
  if ( !dev )   return;

  uint16_t start_colmn, end_colmn;
  uint16_t start_page, end_page;

  if ( DevATTR(dev->Attr).rotation != ILI9341_ROTATION_0 )
  {
    /* 旋转情况. 需要坐标转换. */
    coord_convert(dev, &x1, &y1);
    coord_convert(dev, &x2, &y2);
  }

  start_colmn = (x1 < x2) ? x1 : x2;
  end_colmn = (x1 < x2) ? x2 : x1;
  start_page = (y1 < y2) ? y1 : y2;
  end_page = (y1 < y2) ? y2 : y1;

  /* 设置列起始地址. */
  lcd_write_cmd(ILI9341_COL_ADDR_SET);
  lcd_write_data(start_colmn >> 8);
  lcd_write_data(start_colmn & 0xFF);

  /* 设置列结束地址. */
  lcd_write_data(end_colmn >> 8);
  lcd_write_data(end_colmn & 0xFF);

  /* 设置行起始地址. */
  lcd_write_cmd(ILI9341_PAGES_ADDR_SET);
  lcd_write_data(start_page >> 8);
  lcd_write_data(start_page & 0xFF);

  /* 设置行结束地址. */
  lcd_write_data(end_page >> 8);
  lcd_write_data(end_page & 0xFF);

  /* 将窗口信息更新入设备结构. */
  DevATTR(dev->Attr).windows_x0 = start_colmn;
  DevATTR(dev->Attr).windows_y0 = start_page;
  DevATTR(dev->Attr).windows_x1 = end_colmn;
  DevATTR(dev->Attr).windows_y1 = end_page;

}


static void cus_tft_fill( tftDevice_HandleTypeDef *dev, uint16_t color )
{
  if ( !dev )   return;

  uint16_t x_index;
  uint16_t y_index;

  /* 写显存指令. */
  lcd_write_cmd(ILI9341_MEM_WRITE);

  /* 根据当前窗口进行颜色填充. */
  for( x_index = DevATTR(dev->Attr).windows_x0; x_index <= DevATTR(dev->Attr).windows_x1; x_index++ )
  {
    for( y_index = DevATTR(dev->Attr).windows_y0; y_index <= DevATTR(dev->Attr).windows_y1; y_index++ )
    {
      lcd_write_data16(color);
    }
  }
}


static void cus_tft_drawPixel( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, uint16_t color )
{
  if ( !dev )   return;

  /* 开 1x1 窗口. */
  dev->setWindow(dev, x, y, x, y);

  /* 写显存. */
  lcd_write_cmd(ILI9341_MEM_WRITE);
  lcd_write_data16(color);
}


static void cus_tft_drawHLine( tftDevice_HandleTypeDef *dev, uint16_t pos_Y, uint16_t start_x, uint16_t len, uint8_t thickness, uint16_t color )
{
  if ( !dev || thickness == 0 )   return;

  /* 开窗. */
  dev->setWindow(dev, start_x, pos_Y, (start_x + len - 1), (pos_Y + thickness - 1));

  /* 批量写入. */
  lcd_write_cmd(ILI9341_MEM_WRITE);
  for( uint16_t idx = 0; idx < len * thickness; idx++ )
  {
    lcd_write_data16(color);
  }
}


static void cus_tft_drawVLine( tftDevice_HandleTypeDef *dev, uint16_t pos_X, uint16_t start_y, uint16_t len, uint8_t thickness, uint16_t color )
{
  if ( !dev || thickness == 0 )   return;

  /* 开窗. */
  dev->setWindow(dev, pos_X, start_y, (pos_X + thickness - 1), (start_y + len - 1));

  /* 批量写入. */
  lcd_write_cmd(ILI9341_MEM_WRITE);
  for( uint16_t idx = 0; idx < len * thickness; idx++ )
  {
    lcd_write_data16(color);
  }
}


static void cus_tft_drawRect( tftDevice_HandleTypeDef *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t thickness, uint16_t color )
{
  if ( !dev )   return;

  /* 将两点重新进行映射. 防止后续减出负数. */
  uint16_t in_x1, in_y1, in_x2, in_y2;
  in_x1 = (x1 > x2) ? x2 : x1;
  in_x2 = (x2 > x1) ? x2 : x1;
  in_y1 = (y1 > y2) ? y2 : y1;
  in_y2 = (y2 > y1) ? y2 : y1;

  if ( in_x1 == in_x2 && in_y1 == in_y2 )
  {
    /* 起始点与结束点重合？不存在矩形. 退化为画点. */
    dev->lcd_drawPixel(dev, in_x1, in_y1, color);
    return;
  }

  if ( in_x1 == in_x2 && in_y1 != in_y2 )
  {
    /* 退化为画竖直线. */
    dev->lcd_drawVLine(dev, in_x1, in_y1, (in_y2 - in_y1 + 1), thickness, color);
    return;
  }

  if ( in_x1 != in_x2 && in_y1 == in_y2 )
  {
    /* 退化为画水平线. */
    dev->lcd_drawHLine(dev, in_y1, in_x1, (in_x2 - in_x1 + 1), thickness, color);
    return;
  }

  /* 根据厚度画矩形. */
  dev->lcd_drawHLine(dev, in_y1, in_x1, (in_x2 - in_x1 + 1), thickness, color);
  dev->lcd_drawVLine(dev, in_x1, in_y1, (in_y2 - in_y1 + 1), thickness, color);
  dev->lcd_drawHLine(dev, (in_y2 - thickness + 1), in_x1, (in_x2 - in_x1 + 1), thickness, color);
  dev->lcd_drawVLine(dev, (in_x2 - thickness + 1), in_y1, (in_y2 - in_y1 + 1), thickness, color);
}


static void cus_tft_drawChar( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, char chr, uint8_t font_size, uint16_t fg_color, uint16_t bg_color )
{
  if ( !dev )  return;

  const font_info_t *font = cus_tft_getFont(font_size);

  /* 先处理背景色. */
  /* 注： 字符是以传入坐标点作为左上基准点的. */
  dev->setWindow(dev, x, y, (x + font->width - 1), (y + font->height - 1));
  dev->lcd_fill(dev, bg_color);

  /* 从一维化的字模数组中定位到指定字符的第一个字节. */
  const uint8_t *char_pixel = font->font_data + ((chr - ' ') * font->bytes_per_char);

  /* 扫描字模. */
  for( uint8_t col = 0; col < font->width; col++ )
  {
    /* 字体每列最大字节数. */
    uint8_t bytes_per_col = (font->height + 7) / 8;

    for( uint8_t row = 0; row < font->height; row++ )
    {
      uint8_t byte_idx = row / 8;
      uint8_t bit_idx = row % 8;
      uint8_t byte = char_pixel[col * bytes_per_col + byte_idx];
      uint8_t bit = (byte >> (7 - bit_idx)) & 0x01;
      if ( bit )
      {
        dev->lcd_drawPixel(dev, (x + col), (y + row), fg_color);
      }
    }
  }
}


static void cus_tft_drawLine( tftDevice_HandleTypeDef *dev, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t thickness, uint16_t color )
{
  /* 不允许传入负数. 之所以采用int类型只是为了后续计算考虑. */
  if ( x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 )   return;

  /* 计算变化量. */
  int16_t dx = ABS(x2 - x1);
  int16_t dy = ABS(y2 - y1);

  /* 检查是否为水平线或垂线. 若是则调用对应方法而不进入 Bresenham 循环. */
  if ( dx == 0 )  
  { 
    uint16_t start = (y1 < y2) ? y1 : y2;
    dev->lcd_drawVLine(dev, x1, start, (dy + 1), thickness, color); 
    return; 
  }
  if ( dy == 0 )  
  { 
    uint16_t start = (x1 < x2) ? x1 : x2;
    dev->lcd_drawHLine(dev, y1, start, (dx + 1), thickness, color); 
    return; 
  }

  /* 计算直线步进方向. */
  int8_t sx = SIGN(x2 - x1);
  int8_t sy = SIGN(y2 - y1);

  /* 根据直线大致走向决定加粗方向. */
  bool thick_oritation;
  if ( dx >= dy )  
    thick_oritation = true;     // 较平缓直线. 垂直加粗.
  else 
    thick_oritation = false;    // 较陡峭直线. 水平加粗.

  /* 初始化累积参数. */
  int16_t Pi = dx - dy;

  /* 循环画点. */
  int16_t xi = x1;
  int16_t yi = y1;
  int8_t half = thickness / 2;
  while( (xi != x2 + 1) && (yi != y2 + 1) )
  {
    if( thick_oritation )
    {
      /* 平缓直线 垂直加粗. 画垂线. */
      dev->lcd_drawVLine(dev, xi, yi - half, thickness + 1, 1, color);
    }
    else 
    {
      /* 陡峭直线. 水平加粗. */
      dev->lcd_drawHLine(dev, yi, xi - half, thickness + 1, 1, color);
    }

    int16_t err = 2 * Pi;

    if ( err < dx )
    {
      /* y方向需要步进. */
      Pi += dx;
      yi += sy;
    }

    if ( err > -dy )
    {
      /* x方向需要步进. */
      Pi -= dy;
      xi += sx; 
    }

    /* 循环画点直到结束. */
  }
}


static void cus_tft_drawString( tftDevice_HandleTypeDef *dev, uint16_t x, uint16_t y, const char *str, uint8_t font_size, uint16_t fg_color, uint16_t bg_color )
{
  if ( !dev )   return;

  /* 拿到font参数. */
  const font_info_t *font = cus_tft_getFont(font_size);

  uint16_t cur_x = x;
  uint16_t cur_y = y;
  /* 逐字符绘制. */
  while( *str )
  {
    if ( *str == '\n' )
    {
      /* 换行. */
      cur_x = x;
      cur_y += font->height;
      str++;
      continue;
    }

    if ( *str == '\r' )
    {
      /* 回车. */
      cur_x = x;
      str++;
      continue;
    }

    if ( cur_x + font->width > DevATTR(dev->Attr).width )
    {
      /* 自动换行. */
      cur_x = x;
      cur_y += font->height;
    }

    dev->lcd_drawChar(dev, cur_x, cur_y, *str, font_size, fg_color, bg_color);
    cur_x += font->width;
    str++;
  }
}


static void cus_tft_drawFillCircle( tftDevice_HandleTypeDef *dev, int16_t x, int16_t y, int16_t r, uint16_t color )
{
  if ( !dev || r <= 0 )   return;

  /* 圆上(0,r)点. */
  int16_t cx = 0;
  int16_t cy = r;

  /* 初始决策参数. */
  int16_t Pi = 3 - 2 * r;

  while ( cx <= cy )
  {
    /* 每一个主点. 分别作水平线（填充） */
    dev->lcd_drawHLine(dev, y - cy, x - cx, 2 * cx + 1, 1, color);
    dev->lcd_drawHLine(dev, y + cy, x - cx, 2 * cx + 1, 1, color);
    dev->lcd_drawHLine(dev, y - cx, x - cy, 2 * cy + 1, 1, color);
    dev->lcd_drawHLine(dev, y + cx, x - cy, 2 * cy + 1, 1, color);

    cx++;
    if ( Pi < 0 )
    {
      /* y不动. */
      Pi += 4 * cx + 6;
    }
    else 
    {
      cy--;
      Pi += 4 * (cx -cy) + 10;
    }
  }
}


static void cus_tft_drawCircle( tftDevice_HandleTypeDef *dev, int16_t x, int16_t y, int16_t r, uint8_t thickness, uint16_t color, uint16_t bg_color )
{
  if ( !dev || r <= 0 )   return;

  /* 先画一个实心圆. */
  dev->lcd_drawFillCircle(dev, x, y, r, color);

  /* 再画一个背景色实心圆. */
  dev->lcd_drawFillCircle(dev, x, y, r - thickness, bg_color);
}


static inline void cus_tft_displayOn( void )
{
  lcd_write_cmd(ILI9341_DISPLAY_ON);
}
/* *********************** CallBack *********************** */


/* *********************** Restriction API *********************** */
static void coord_convert( tftDevice_HandleTypeDef *dev, uint16_t *x, uint16_t *y )
{
  if ( !dev || !x || !y )   return;

  uint16_t in_x = *x;
  uint16_t in_y = *y;

  switch (DevATTR(dev->Attr).rotation)
  {
    case ILI9341_ROTATION_0:      // 旋转角度为0. 无需转换.
    {
      *x = in_x;
      *y = in_y;

      break;
    }   

    case ILI9341_ROTATION_90:     // 顺时针旋转90度.
    {
      /* 将逻辑点映射到物理点. 例如逻辑点(2,1) 对应物理点(238,2). 计算公式如下 */
      /* 注意：90度情况存在一个 逻辑xy轴与物理xy轴的换序！逻辑x对应物理y. 逻辑y对应物理x. */
      *x = DevATTR(dev->Attr).xo - in_y;
      *y = DevATTR(dev->Attr).yo + in_x;
      break;
    }

    case ILI9341_ROTATION_180:    // 顺时针旋转180度.
    {
      /* 例如逻辑点(2,1) 对应物理点(237,318). 计算公式如下. */
      /* 注意：180度情况不存在xy换序. */
      *x = DevATTR(dev->Attr).xo - in_x;
      *y = DevATTR(dev->Attr).yo - in_y;
      break;
    }

    case ILI9341_ROTATION_270:    // 顺时针旋转270度.
    {
      /* 例如逻辑点(2,1) 对应物理点(1,317). 计算公式如下. */
      /* 注意：270度情况也存在xy换序. */
      *x = DevATTR(dev->Attr).xo + in_y;
      *y = DevATTR(dev->Attr).yo - in_x;
      break;
    }
    
    default:    break;
  }
}


static inline void cus_tft_getChipID( tftDevice_HandleTypeDef *dev )
{
  if ( !dev )   return;

  uint8_t id1,id2;
  uint16_t id_f;
  lcd_write_cmd(ILI9341_READ_LCD_MODULE);

  /* 读掉dummy数据. */
  lcd_read_data();
  lcd_read_data();

  /* 读取ID高8位与低8位. */
  id1 = lcd_read_data();
  id2 = lcd_read_data();

  id_f = ((id1 << 8) | id2);
  switch (id_f)
  {
    case ID_ILI9341: DevATTR(dev->Attr).dev_name = "ILI9341";   break;
    case ID_ST7789:  DevATTR(dev->Attr).dev_name = "ST7789";    break;
    
    default:    break;
  }
}


static inline int8_t cus_tft_setRotation( tftDevice_HandleTypeDef *dev, uint8_t rotation )
{
  if ( rotation != ILI9341_ROTATION_0 && 
        rotation != ILI9341_ROTATION_90 && 
        rotation != ILI9341_ROTATION_180 && 
        rotation != ILI9341_ROTATION_270  )
  {
    return -1;
  }

  if ( !dev )    return -1;

  switch (rotation)
  {
    case ILI9341_ROTATION_0:  
    {
      /* 无旋转情况. 按照初始化数据运行.直接返回.*/
      break;
    }

    case ILI9341_ROTATION_90: 
    {
      /* 顺时针旋转90度. */
      /* 更新逻辑原点对应的物理坐标. (239,0) */
      DevATTR(dev->Attr).xo = LCD_SCREEN_WIDTH - 1;
      DevATTR(dev->Attr).yo = 0;

      /* 宽高互换. */
      DevATTR(dev->Attr).width = LCD_SCREEN_HEIGHT;
      DevATTR(dev->Attr).height = LCD_SCREEN_WIDTH;
      break;
    }

    case ILI9341_ROTATION_180: 
    {
      /* 顺时针旋转180度. */
      /* 更新逻辑原点对应的物理坐标. (239,319) */
      DevATTR(dev->Attr).xo = LCD_SCREEN_WIDTH - 1;
      DevATTR(dev->Attr).yo = LCD_SCREEN_HEIGHT - 1;

      /* 无需宽高互换. */
      DevATTR(dev->Attr).width = LCD_SCREEN_WIDTH;
      DevATTR(dev->Attr).height = LCD_SCREEN_HEIGHT;
      break;
    }

    case ILI9341_ROTATION_270: 
    {
      /* 顺时针旋转270度. */
      /* 更新逻辑原点物理坐标. (0,319) */
      DevATTR(dev->Attr).xo = 0;
      DevATTR(dev->Attr).yo = LCD_SCREEN_HEIGHT - 1; 

      /* 宽高互换. */
      DevATTR(dev->Attr).width = LCD_SCREEN_HEIGHT;
      DevATTR(dev->Attr).height  =LCD_SCREEN_WIDTH;
      break;
    }
    
    default: return -1;
  }

  DevATTR(dev->Attr).rotation = rotation;
  DevATTR(dev->Attr).windows_x1 = DevATTR(dev->Attr).width - 1;
  DevATTR(dev->Attr).windows_y1 = DevATTR(dev->Attr).height - 1;

  return 1;
}


static const font_info_t *cus_tft_getFont( uint8_t font_size )
{
  switch (font_size)
  {
    case CUS_FONT_SIZE_12: 
    {
      #if(ILI9341_FONT_12)
        return &font_12;
      #endif 
    }

    case CUS_FONT_SIZE_16: 
    {
      #if(ILI9341_FONT_16)
        return &font_16;
      #else
        return &font_12;
      #endif 
    } 

    case CUS_FONT_SIZE_24: 
    {
      #if(ILI9341_FONT_24)
        return &font_24;
      #else 
        return &font_12;
      #endif 
    } 

    case CUS_FONT_SIZE_32: 
    {
      #if(ILI9341_FONT_32)
        return &font_32;
      #else 
        return &font_12;
      #endif 
    }
    
    default:  return &font_12;  
  }
}


static inline void cus_tft_init_reg( void )
{
  {
    lcd_write_cmd(ILI9341_PWR_CTRL_B); 
    lcd_write_data(0x00); 
    lcd_write_data(0xC1); 
    lcd_write_data(0x30);
  }

  {
    /* 上电时序控制. */
    lcd_write_cmd(ILI9341_PWR_SEQ_CTRL); 
    lcd_write_data(0x64); lcd_write_data(0x03); lcd_write_data(0x12); lcd_write_data(0x81);
  }

  {
    /* 调整驱动波形. */
    lcd_write_cmd(ILI9341_DRI_TIMING_CTRL_A); 
    lcd_write_data(0x85); 
    lcd_write_data(0x10); 
    lcd_write_data(0x7A);
  }

  {
    /* 配置核心电压. */
    lcd_write_cmd(ILI9341_PWR_CTRL_A); 
    lcd_write_data(0x39); lcd_write_data(0x2C); lcd_write_data(0x00); lcd_write_data(0x34); lcd_write_data(0x02);
  }

  {
    /* 设置内部电荷泵倍率. */
    lcd_write_cmd(ILI9341_PUMP_RATIO_CTRL); 
    lcd_write_data(0x20);
  }

  {
    /* 配置栅极驱动器EQ时序. */
    lcd_write_cmd(ILI9341_DRI_TIMING_CTRL_B); 
    lcd_write_data(0x00); 
    lcd_write_data(0x00);
  }

  {
    /* 设置 GVDD 电压. */
    lcd_write_cmd(ILI9341_PWR_CTRL_1); 
    lcd_write_data(0x1B);
  }

  {
    /* 设置升压电路的倍率因子. */
    lcd_write_cmd(ILI9341_PWR_CTRL_2); 
    lcd_write_data(0x01);
  }

  {
    /* 设置VCOMH和VCOML的电压. */
    lcd_write_cmd(ILI9341_VCOM_CTRL); 
    lcd_write_data(0x30); 
    lcd_write_data(0x30);
  }

  {
    /* 控制VCOM偏移电压的调整方式. */
    lcd_write_cmd(ILI9341_VCOM_CTRL_2); 
    lcd_write_data(0xB7);
  }

  {
    /* 设置内存读取扫描方向. */
    lcd_write_cmd(ILI9341_MEM_ACCESS_CTRL); 
    lcd_write_data(0x08);
  }

  {
    /* RGB565. */
    lcd_write_cmd(ILI9341_PIXEL_FORMAT_SET); 
    lcd_write_data(0x55);
  }

  {
    /* 设置正常模式（全彩）下的帧率. */
    lcd_write_cmd(ILI9341_FRAME_RATE_CTRL); 
    lcd_write_data(0x00); 
    lcd_write_data(0x1A);
  }

  {
    /* 配置显示功能. */
    lcd_write_cmd(ILI9341_DISPLAY_FUNC_CTRL); 
    lcd_write_data(0x0A); 
    lcd_write_data(0xA2);
  }

  {
    /* 3伽马控制. */
    lcd_write_cmd(ILI9341_ENABLE_3_GAMMA); 
    lcd_write_data(0x00);
  }

  {
    /* 伽马曲线设置. */
    lcd_write_cmd(ILI9341_GAMMA_SET); 
    lcd_write_data(0x01);
  }

  {
    /* 设置正极性伽马校正. */
    lcd_write_cmd(ILI9341_POSITIVE_GAMMA_CORRECTION); 
    lcd_write_data(0x0F); lcd_write_data(0x2A); lcd_write_data(0x28); lcd_write_data(0x08);
    lcd_write_data(0x0E); lcd_write_data(0x08); lcd_write_data(0x54); lcd_write_data(0xA9);
    lcd_write_data(0x43); lcd_write_data(0x0A); lcd_write_data(0x0F); lcd_write_data(0x00);
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x00);
  }

  {
    /* 设置负极性伽马校正. */
    lcd_write_cmd(ILI9341_NEGTIVE_GAMMA_CORRECTION); 
    lcd_write_data(0x00); lcd_write_data(0x15); lcd_write_data(0x17); lcd_write_data(0x07);
    lcd_write_data(0x11); lcd_write_data(0x06); lcd_write_data(0x2B); lcd_write_data(0x56);
    lcd_write_data(0x3C); lcd_write_data(0x05); lcd_write_data(0x10); lcd_write_data(0x0F);
    lcd_write_data(0x3F); lcd_write_data(0x3F); lcd_write_data(0x0F);
  }

  {
    /* 列范围. */
    lcd_write_cmd(ILI9341_COL_ADDR_SET); 
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0xEF); 
  }

  {
    /* 行范围. */
    lcd_write_cmd(ILI9341_PAGES_ADDR_SET); 
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x01); lcd_write_data(0x3F); 
  }

  /* 退出睡眠. */
  lcd_write_cmd(ILI9341_SLEEP_OUT);   

  lcd_delay_ms(120);
}
/* *********************** Restriction API *********************** */


