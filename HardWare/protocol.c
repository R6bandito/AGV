#include "stm32f1xx_hal.h"
#include "protocol.h"
#include <string.h>



static uint8_t crc8_table[256];  // 静态全局变量：crc8校验表（预计算）

static void crc8_init(void)  // 初始化crc8校验表（预计算256个值，提升校验效率）
{
    for (int i = 0; i < 256; i++)
    {
        uint8_t crc = (uint8_t)i;
        for (int j = 0; j < 8; j++) 
        {
             // crc8算法：多项式0x07
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc <<= 1;
        }
        crc8_table[i] = crc;
    }
}

static uint8_t crc8_calc(const uint8_t *buf, int len)  // 计算crc8校验值（基于预计算表）
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) 
    {
        crc = crc8_table[crc ^ buf[i]];  // 查表更新crc
    }
    return crc;
}

static UART_RxState_t rx_state;  // 静态全局变量：接收状态机当前状态
static uint8_t        rx_buffer[16];  // 静态全局变量：接收缓冲区（存储整帧数据）
static uint8_t        rx_index;  // 静态全局变量：接收缓冲区索引
static volatile bool  frame_ready = false;  // 静态全局变量：新帧就绪标志（volatile：防止编译器优化，中断/主函数共享）
static MotionFrame_t  last_frame;  // 静态全局变量：最后解析完成的帧

void Protocol_Init(void)  // 协议初始化：初始化校验表、状态机、缓冲区
{
    crc8_init();      
    rx_state    = PROT_WAIT_H1; // 初始状态：等待帧头1
    rx_index    = 0;            // 缓冲区索引清零
    frame_ready = false;        // 无新帧
    memset(&last_frame, 0, sizeof(last_frame)); // 清空最后帧
}

void Protocol_FeedByte(uint8_t ch)  // 字节喂入函数（UART接收中断中调用，逐字节处理）
{
    switch (rx_state) 
    {
        case PROT_WAIT_H1:   //等待帧头1
            if (ch == FRAME_HEADER1) 
            {
                rx_buffer[0] = ch;  //存储帧头1
                rx_index = 1;       //索引移到下一位
                rx_state = PROT_WAIT_H2;//切换等待帧头2
            }
            break;

        case PROT_WAIT_H2: //等待帧头2
            if (ch == FRAME_HEADER2) 
            {
                rx_buffer[1] = ch; 
                rx_index = 2;
                rx_state = PROT_WAIT_CMD;//切换到等待指令码
            } 
            else 
            {
                rx_state = PROT_WAIT_H1; //帧头2错误，回到等待帧头1
            }
            break;

        case PROT_WAIT_CMD: //等待指令码
            rx_buffer[2] = ch; //存储指令码
            rx_index = 3;      //索引移到下一位
            rx_state = PROT_WAIT_DATA; //切换到等待数据段
            break;

        case PROT_WAIT_DATA: //等待数据段
            rx_buffer[rx_index++] = ch; //存储字节，索引自增
            if (rx_index >= 15) // 数据段接收完成（header1+header2+cmd+data=15字节）
            {
                rx_state = PROT_WAIT_CRC;  //切换到等待校验位
            }
            break;

        case PROT_WAIT_CRC: //等待校验位
        {
            // rx_buffer[15] = ch; //存储校验位
            
            // uint8_t crc = crc8_calc(rx_buffer, 15); //计算前15字节的crc8，与接收的校验位对比

            // if (crc == rx_buffer[15]) 
            // {
                memcpy(&last_frame, rx_buffer, sizeof(MotionFrame_t)); // 校验通过：复制到last_frame，置新帧就绪标志
                frame_ready = true;
            // }
            rx_state = PROT_WAIT_H1; // 重置状态机，等待下一帧
            break;
        }

        default: //// 异常状态：重置为等待帧头1
            rx_state = PROT_WAIT_H1;
            break;
    }
}

bool Protocol_NewFrameReady(void) // 检查是否有新帧就绪（对外接口）
{
    return frame_ready;
}

bool Protocol_ParseCommand(AGV_Command_t *cmd) // 解析帧为AGV指令（对外接口）
{
     // 无新帧/指令指针为空 → 返回失败
    if (!frame_ready || cmd == NULL) 
    return false;
    frame_ready = false; // 清零就绪标志（避免重复解析）

    if (last_frame.cmd == CMD_MOTION) //运动指令
    {
        cmd->vx       = last_frame.vx; //前进速度
        cmd->vy       = last_frame.vy; //横向速度
        cmd->omega    = last_frame.omega; //旋转角速度
        cmd->is_valid = true;           //指令有效
        return true;
    }
    else if (last_frame.cmd == CMD_ESTOP)  //急停指令
    {
        cmd->vx       = 0;          //速度清零
        cmd->vy       = 0;
        cmd->omega    = 0;
        cmd->is_valid = true;       //指令有效
        return true;
    }
    return false;    //未知指令码，返回失败
}

