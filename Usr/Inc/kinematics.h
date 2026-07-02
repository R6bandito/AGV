#ifndef __KINEMATICS_H
#define __KINEMATICS_H
#include <stdint.h>
#include "mb_app.h"
#include "mb.h"

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


#define MODBUS_SCALE           100.0f       // 缩放因子：整数/1000 = 实际值
#define MODBUS_REG_VX          0
#define MODBUS_REG_VY          1
#define MODBUS_REG_OMEGA       2
#define MODBUS_REG_ESTOP       3             // 0=正常, 1=急停

extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];



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

bool Kinematics_UpdateFromModbus(void);      

const WheelTarget_t* Kinematics_GetWheelTarget(uint8_t wheel_index);
const AGV_Command_t* Kinematics_GetCurrentCommand(void);
#endif

