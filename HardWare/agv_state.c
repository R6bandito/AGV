#include "stm32f1xx_hal.h"
#include "agv_state.h"
#include "kinematics.h"
#include "wheel_manager.h"
#include <math.h>

#define CMD_TIMEOUT_MS      500
#define WHEEL_TIMEOUT_MS    200
#define MAX_RPM             3000.0f

static AGV_State_t state = AGV_STATE_IDLE;
static uint32_t    last_cmd_tick = 0;

void AGV_State_Init(void)
{
    state = AGV_STATE_IDLE;
    last_cmd_tick = HAL_GetTick();
}

void AGV_SafetyCheck(void)
{
    uint32_t now = HAL_GetTick();

    if (state == AGV_STATE_ESTOP || state == AGV_STATE_ERROR)
        return;

    if (now - last_cmd_tick > CMD_TIMEOUT_MS) {
        AGV_EmergencyStop();
        state = AGV_STATE_ESTOP;
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (!WheelManager_IsOnline(i)) {
            AGV_EmergencyStop();
            state = AGV_STATE_ESTOP;
            return;
        }
        uint32_t wheel_last = WheelManager_GetLastUpdateTick(i);
        if (now - wheel_last > WHEEL_TIMEOUT_MS) {
            AGV_EmergencyStop();
            state = AGV_STATE_ESTOP;
            return;
        }
    }

    for (int i = 0; i < 4; i++) {
        const WheelTarget_t *t = Kinematics_GetWheelTarget(i);
        if (t && (fabsf(t->wheel_rpm) > MAX_RPM)) {
            AGV_EmergencyStop();
            state = AGV_STATE_ERROR;
            return;
        }
    }

    if (state != AGV_STATE_RUNNING) {
        state = AGV_STATE_RUNNING;
    }
}

void AGV_EmergencyStop(void)
{
    WheelManager_StopAll();
    state = AGV_STATE_ESTOP;
}

AGV_State_t AGV_GetState(void)
{
    return state;
}