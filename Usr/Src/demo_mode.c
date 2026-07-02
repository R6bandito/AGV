#include "demo_mode.h"

void Demo_GetCommand(AGV_Command_t *cmd)
{
    static DemoMode_t prev_mode = DEMO_MODE_PARKING; // 记录上一次模式
    static uint32_t parking_start_tick = 0;
    // 如果切换到倒车入库，重置计时
    if (demo_mode == DEMO_MODE_PARKING && prev_mode != DEMO_MODE_PARKING) 
    {
        parking_start_tick = HAL_GetTick();
    }
    prev_mode = demo_mode;
    switch (demo_mode) 
    {
        case DEMO_MODE_STRAIGHT:
            cmd->vx    =  500.0f / MODBUS_SCALE;
            cmd->vy    =  0;
            cmd->omega =  0;
            break;
        case DEMO_MODE_LATERAL:
            cmd->vx    =  0;
            cmd->vy    =  500.0f / MODBUS_SCALE;
            cmd->omega =  0;
            break;
        case DEMO_MODE_ROTATE:
            cmd->vx    =  0;
            cmd->vy    =  0;
            cmd->omega =  200.0f / MODBUS_SCALE;
            break;
        case DEMO_MODE_COMPOUND:
            cmd->vx    =  500.0f / MODBUS_SCALE;
            cmd->vy    =  300.0f / MODBUS_SCALE;
            cmd->omega =  100.0f / MODBUS_SCALE;
            break;
        case DEMO_MODE_PARKING:
        {
            uint32_t now = HAL_GetTick();
            if (parking_start_tick == 0) parking_start_tick = now;
            uint32_t t = now - parking_start_tick;
            // 三阶段倒车入库轨迹（循环演示）
            if (t < 3000)
            {                                 // 阶段1: 斜后退
                cmd->vx = -0.5f; cmd->vy = 0.3f; cmd->omega = 0;
            } 

            else if (t < 6000) 
            {              // 阶段2: 后退+转向
                cmd->vx = -0.3f; cmd->vy = 0;   cmd->omega = -0.2f;
            } 

            else if (t < 9000) 
            {              // 阶段3: 直线后退
                cmd->vx = -0.5f; cmd->vy = 0;   cmd->omega = 0;
            } 

            else 
            {
                parking_start_tick = now;       // 重新开始
            }
            break;
        }
        default:    break;
    }
    cmd->is_valid = true;
}

