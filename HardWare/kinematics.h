#ifndef __KINEMATICS_H
#define __KINEMATICS_H
#include <stdint.h>

// ----- ARMCC v5 C99 兼容 bool 定义 ----- 
#ifndef __cplusplus
  #define bool  _Bool
  #define true  1
  #define false 0
#endif

//AGV几何参数 单位：米
#define AGV_WHEELBASE_X   1.2f //前后轮距的一半
#define AGV_WHEELBASE_Y   0.8f //左右轮距的一半
#define AGV_WHEEL_RADIUS  0.1f //轮子半径
#define PI 3.14159265358979  //圆周率近似值

#ifndef NULL
  #define NULL  (void *)0
#endif

//单个舵轮的目标状态
typedef struct 
{
    float steering_angle_rad; //目标舵角（弧度）
    float steering_angle_deg; //目标舵角（度）
    float wheel_rpm;          //目标轮转速（rpm）
    float wheel_linear_speed; //目标轮线速度（m/s）
}WheelTarget_t;

//AGV整体运动指令
typedef struct 
{
    float vx;        //前进速度m/s
    float vy;        //横向速度m/s
    float omega;     //旋转角速度rad/s
    bool is_valid;   //指令否有效
}AGV_Command_t;

void Kinematics_Init(float wb_x, float wb_y, float r);
bool Kinematics_Inverse(const AGV_Command_t *cmd);
const WheelTarget_t*Kinematics_GetWheelTarget(uint8_t wheel_index);
const AGV_Command_t*Kinematics_GetCurrentCommand(void);

#endif

