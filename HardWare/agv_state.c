#include"agv_state.h"
#include "kinematics.h"
#include "wheel_manager.h"
#include"stm32f1xx_hal.h"

#define CMD_TIMEOUT_MS  500
#define WHEEL_TIMEOUT_MS  200
#define MAX_RPM         3000.0F

static AGV_State_t state=AGV_STATE_IDLE;
static uint32_t last_cmd_tick=0;

void AGV_State_Init(void)
{
    state = AGV_STATE_IDLE;
    last_cmd_tick = HAL_GetTick();
}

void AGV_SafetyCheck(void)
{
    uint32_t now = HAL_GetTick();
    
}