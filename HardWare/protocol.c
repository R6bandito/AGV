#include "protocol.h"
#include <string.h>

/* CRC8 校验表（多项式 0x07） */
static uint8_t crc8_table[256];
static void crc8_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
        crc8_table[i] = crc;
    }
}

static uint8_t crc8_calc(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crc8_table[crc ^ buf[i]];
    }
    return crc;
}

/* 接收状态机变量 */
static UART_RxState_t rx_state;
static uint8_t rx_buffer[16];
static uint8_t rx_index;
static volatile bool frame_ready = false;  // 由中断/喂字节设为true
static MotionFrame_t last_frame;

void Protocol_Init(void)
{
    crc8_init();
    rx_state = WAIT_HEADER1;
    rx_index = 0;
    frame_ready = false;
    memset(&last_frame, 0, sizeof(last_frame));
}

void Protocol_FeedByte(uint8_t ch)
{
    switch (rx_state) {
        case WAIT_HEADER1:
            if (ch == FRAME_HEADER1) {
                rx_buffer[0] = ch;
                rx_index = 1;
                rx_state = WAIT_HEADER2;
            }
            break;

        case WAIT_HEADER2:
            if (ch == FRAME_HEADER2) {
                rx_buffer[1] = ch;
                rx_index = 2;
                rx_state = WAIT_CMD;
            } else {
                rx_state = WAIT_HEADER1;  // 失步，重新同步
            }
            break;

        case WAIT_CMD:
            rx_buffer[2] = ch;
            rx_index = 3;
            rx_state = WAIT_DATA;
            break;

        case WAIT_DATA:
            rx_buffer[rx_index++] = ch;
            if (rx_index >= 15) {   // 收到 2(帧头)+1(CMD)+12(3个float) = 15字节
                rx_state = WAIT_CRC;
            }
            break;

        case WAIT_CRC:
            rx_buffer[15] = ch;
            // 校验 CRC
            uint8_t crc = crc8_calc(rx_buffer, 15);
            if (crc == rx_buffer[15]) {
                memcpy(&last_frame, rx_buffer, sizeof(MotionFrame_t));
                frame_ready = true;
            }
            // 无论校验成功与否，回到等待帧头
            rx_state = WAIT_HEADER1;
            break;

        default:
            rx_state = WAIT_HEADER1;
            break;
    }
}

bool Protocol_NewFrameReady(void)
{
    return frame_ready;
}

bool Protocol_ParseCommand(AGV_Command_t *cmd)
{
    if (!frame_ready || cmd == NULL) return false;

    frame_ready = false;   // 清标志

    if (last_frame.cmd == CMD_MOTION) {
        cmd->vx       = last_frame.vx;
        cmd->vy       = last_frame.vy;
        cmd->omega    = last_frame.omega;
        cmd->is_valid = true;
        return true;
    }
    else if (last_frame.cmd == CMD_ESTOP) {
        cmd->vx       = 0;
        cmd->vy       = 0;
        cmd->omega    = 0;
        cmd->is_valid = true;
        return true;
    }
    return false;
}