#include "stm32f1xx_hal.h"
#include "agv_state.h"
#include "kinematics.h"
#include "wheel_manager.h"
#include <math.h>

#define CMD_TIMEOUT_MS      500 //指令超时阈值（500ms无指令则急停）
#define WHEEL_TIMEOUT_MS    200 //舵机状态更新超时阈值（200ms无反应则急停）
#define MAX_RPM             3000.0f //最大允许转速 超限则触发错误

static AGV_State_t state = AGV_STATE_IDLE;
static uint32_t    last_cmd_tick = 0;  //最后一次有效指令的时间戳 ms

void AGV_State_Init(void) //初始化AGV状态：设置初始态为空闲，记录当前之间戳
{
    state = AGV_STATE_IDLE;  
    last_cmd_tick = HAL_GetTick();  //记录初始化时刻的系统tick
}

void AGV_SafetyCheck(void)  //安全检查函数（需要周期性调用）：监控指令，舵轮，转速状态
{
    uint32_t now = HAL_GetTick();    //获取当前系统tick

    if (state == AGV_STATE_ESTOP || state == AGV_STATE_ERROR)  //如果已是急停/错误态，无需检查（避免重复处理）
        return;

    if (now - last_cmd_tick > CMD_TIMEOUT_MS) //检查1：指令超时（500ms无新指令），则触发急停+设为急停态
    {
        AGV_EmergencyStop();
        state = AGV_STATE_ESTOP;
        return;
    }

    for (int i = 0; i < 4; i++)  //检查2：遍历4个舵轮，检查在线状态+更新超时
    {
        if (!WheelManager_IsOnline(i)) //舵轮离线，则急停+急停态
        {
            AGV_EmergencyStop();
            state = AGV_STATE_ESTOP;
            return;
        }

        uint32_t wheel_last = WheelManager_GetLastUpdateTick(i); //舵轮最后更新时间超时（200ms无反馈），则急停+急停态
        if (now - wheel_last > WHEEL_TIMEOUT_MS) 
        {
            AGV_EmergencyStop();
            state = AGV_STATE_ESTOP;
            return;
        }
    }

    for (int i = 0; i < 4; i++) //检查3.遍历4个舵轮，检查目标转速是否超限
    {//获取第i个舵轮的目标状态

        const WheelTarget_t *t = Kinematics_GetWheelTarget(i);
        //目标转速绝对值超过MAX_RPM,则急停加急停态
        if (t && (fabsf(t->wheel_rpm) > MAX_RPM)) 
        {
            AGV_EmergencyStop();
            state = AGV_STATE_ERROR;
            return;
        }
    }

    if (state != AGV_STATE_RUNNING) //所有检查通过：若当前状态非运行态，则切换为运行态
    {
        state = AGV_STATE_RUNNING;
    }
}

void AGV_EmergencyStop(void) //急停函数，停止所有舵轮，设置急停态
{
    WheelManager_StopAll();  //调用舵轮管理器的全局停止接口
    state = AGV_STATE_ESTOP; //设为急停态
}

AGV_State_t AGV_GetState(void)  //获取当前AGV状态（对外接口）
{
    return state;  //返回静态全局变量state
}

void AGV_UpdateLastCmdTick(void) 
{
    last_cmd_tick = HAL_GetTick();
}

