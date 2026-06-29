#include "stm32f1xx_hal.h"  
#include"kinematics.h"
#include<math.h>

static struct //静态全局结构体，存储AGV几何参数，当前指令，舵轮目标状态
{
    float wheel_base_x;
    float wheel_base_y;
    float wheel_radius;
    AGV_Command_t command;
    WheelTarget_t  wheels[4]; //4个舵轮的目标状态
}agv;

//初始化运动学模型
void Kinematics_Init(float wb_x, float wb_y, float r)
{
    agv.wheel_base_x=wb_x;
    agv.wheel_base_y=wb_y;
    agv.wheel_radius=r;
    agv.command.is_valid=false;

    //初始化4个舵轮的目标状态
    for (int i = 0; i < 4; i++)
    {
        agv.wheels[i].steering_angle_rad=0;
        agv.wheels[i].steering_angle_deg=0;
        agv.wheels[i].wheel_rpm=0;
        agv.wheels[i].wheel_linear_speed=0;
    }
    
}

//逆运动学解算：输入指令，计算出4个轮子的舵角和转速

bool Kinematics_Inverse(const AGV_Command_t *cmd)
{
    if (cmd==NULL||!cmd->is_valid)//指令为空/无效，返回失败
    return false;

    agv.command=*cmd;
    
    // 轮子坐标（AGV中心为原点，4个轮子的(Xi,Yi)）：
    //轮子顺序：0=前左，1=前右，2=后左，3=后右
    const float Xi[4]={agv.wheel_base_x,agv.wheel_base_x,
                       -agv.wheel_base_x,-agv.wheel_base_x};
    const float Yi[4]={agv.wheel_base_y,-agv.wheel_base_y,
                       agv.wheel_base_y,-agv.wheel_base_y};
    
    //提取AGV指令的速度分量
    float Vx = cmd->vx; //前进速度
    float Vy = cmd->vy; //横向速度
    float omega = cmd->omega; //旋转角速度（rad/s）

    //遍历四个舵轮，计算每个轮的目标状态
    for (int i = 0; i < 4; i++)
    {
        float Vxi = Vx-omega*Yi[i];
        float Vyi = Vy+omega*Xi[i];//舵角计算

        if (fabsf(Vxi)<0.0001f && fabsf(Vyi)<0.0001f)
        {//零速度指令，保持上一次舵角
            continue;
        }

        // 计算舵角（弧度）：atan2f(Vyi, Vxi) → 反正切，范围(-π, π)
        float angle_rad=atan2f(Vyi,Vxi);
         
         // 舵角归一化到 (-π, π) 范围（防止角度溢出）
        while (angle_rad>PI)
        {
            angle_rad-=2.0f*PI;
        }
        while (angle_rad<-PI)
        {
            angle_rad+=2.0f*PI;
        }

        // 计算舵轮线速度：合速度 = √(Vxi² + Vyi²)
        float speed = sqrtf(Vxi*Vxi + Vyi*Vyi);

        // 线速度转转速（rpm）：rpm = (speed * 60) / (2πr)
        float rpm = speed *60.0f/(2.0f*PI*agv.wheel_radius);

        agv.wheels[i].steering_angle_rad=angle_rad;
        agv.wheels[i].steering_angle_deg=angle_rad*180.0f/PI;
        agv.wheels[i].wheel_linear_speed=speed;
        agv.wheels[i].wheel_rpm=rpm;    
    }
    return true;
    
}


// 获取指定舵轮的目标状态（对外接口）
const WheelTarget_t*Kinematics_GetWheelTarget(uint8_t wheel_index)
{
    if(wheel_index>=4)// ID越界返回NULL
    return NULL;
    return &agv.wheels[wheel_index];
}

//获取当前指令（对外接口）
const AGV_Command_t*Kinematics_GetCurrentCommand(void)
{
    return &agv.command;
}

