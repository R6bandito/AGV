#ifndef __CUS_ILI9341_CMDS_H__
#define __CUS_ILI9341_CMDS_H__



/* **************** ILI9341_Write_Command **************** */
  #define ILI9341_SOFTWARE_RESET                    (0x01)            // 软件复位.
  #define ILI9341_MEM_ACCESS_CTRL                   (0x36)            // 内存访问控制（方向/颜色顺序）.
  #define ILI9341_COL_ADDR_SET                      (0x2A)            // 列地址范围设置.
  #define ILI9341_PAGES_ADDR_SET                    (0x2B)            // 页地址范围设置.
  #define ILI9341_MEM_WRITE                         (0x2C)            // 写入显存.
  #define ILI9341_SLEEP_OUT                         (0x11)            // 退出睡眠模式.
  #define ILI9341_PIXEL_FORMAT_SET                  (0x3A)            // 设置像素格式.
  #define ILI9341_DISPLAY_ON                        (0x29)            // 开启显示.
  #define ILI9341_DISPLAY_OFF                       (0x28)            // 关闭显示.
  #define ILI9341_PWR_CTRL_B                        (0xCF)            // 内部电源控制B命令.
  #define ILI9341_PWR_SEQ_CTRL                      (0xED)            // 上电时序控制.
  #define ILI9341_DRI_TIMING_CTRL_A                 (0xE8)            // 调整栅极驱动器的时序参数命令.
  #define ILI9341_DRI_TIMING_CTRL_B                 (0xEA)            // 配置栅极驱动器EQ时序.
  #define ILI9341_PWR_CTRL_A                        (0xCB)            // 内部电源控制A命令.
  #define ILI9341_PUMP_RATIO_CTRL                   (0xF7)            // 设置内部电荷泵的倍率.
  #define ILI9341_PWR_CTRL_1                        (0xC0)            // 设置 GVDD 电压（即 VCOM 和灰度电压的参考电平）.
  #define ILI9341_PWR_CTRL_2                        (0xC1)            // 设置升压电路的倍率因子.
  #define ILI9341_VCOM_CTRL                         (0xC5)            // 设置VCOMH和VCOML的电压，以优化液晶显示器的对比度和闪烁抑制.
  #define ILI9341_VCOM_CTRL_2                       (0xC7)            // 控制VCOM偏移电压的调整方式.
  #define ILI9341_FRAME_RATE_CTRL                   (0xB1)            // 设置正常模式（全彩）下的帧率.
  #define ILI9341_DISPLAY_FUNC_CTRL                 (0xB6)            // 配置显示功能.
  #define ILI9341_ENABLE_3_GAMMA                    (0xF2)            // 使能3伽马控制.
  #define ILI9341_GAMMA_SET                         (0x26)            // gamma曲线设置.
  #define ILI9341_POSITIVE_GAMMA_CORRECTION         (0xE0)            // 设置正极性伽马校正曲线.
  #define ILI9341_NEGTIVE_GAMMA_CORRECTION          (0xE1)            // 设置负极性伽马校正曲线.
/* **************** ILI9341_Write_Command **************** */


/* **************** ILI9341_Read_Command **************** */
  #define ILI9341_READ_IDENTIFICATION_INFO          (0x04)              // 读取芯片 ID.
  #define ILI9341_READ_DISPLAY_STATUS               (0x09)              // 读取显示状态.
  #define ILI9341_READ_DISPLAY_PWR_MODE             (0x0A)              // 读取电源模式.
  #define ILI9341_READ_DISPLAY_MADCTL               (0x0B)              // 读取 MADCTL 值.
  #define ILI9341_READ_DISPLAY_PIXEL_FORMAT         (0x0C)              // 读取读取像素格式.
  #define ILI9341_READ_LCD_MODULE                   (0xD3)              // 读取模块信息.
  #define ILI9341_READ_MEM                          (0x2E)              // 读取显存.


/* **************** ILI9341_Read_Command **************** */

#endif /* __CUS_ILI9341_CMDS_H__ */
