#include"kinematics.h"
#include<math.h>

#ifndef NULL
    #define NULL  (void *)0
#endif
static struct 
{
    float wheel_base_x;
    float wheel_base_y;
    float wheel_radius;
    AGV_Command_t command;
    WheelTaget_t  wheels[4];
}agv;

//初始化运动学模型
void Kinematics_Init(float wb_x, float wb_y, float r)
{
    agv.wheel_base_x=wb_x;
    agv.wheel_base_y=wb_y;
    agv.wheel_radius=r;
    agv.command.is_valid=false;
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
    if (cmd==NULL||!cmd->is_valid)
    return false;
    agv.command=*cmd;
    //轮子顺序：0=前左，1=前右，2=后左，3=后右
    const float Xi[4]={agv.wheel_base_x,agv.wheel_base_x,
                       -agv.wheel_base_x,-agv.wheel_base_x};
    const float Yi[4]={agv.wheel_base_y,-agv.wheel_base_y,
                       agv.wheel_base_y,-agv.wheel_base_y};
    
    float Vx = cmd->Vx;
    float Vy = cmd->Vy;//以下看不懂
    float omega = cmd->omega;
    for (int i = 0; i < 4; i++)
    {
        float Vxi = Vx-omega*Yi[i];
        float Vyi = Vy+omega*Xi[i];//舵角计算

        if (fabsf(Vxi)<0.0001f && fabsf(Vyi)<0.0001f)
        {//零速度指令，保持上一次舵角
            continue;
        }

        float angle_rad=atan2f(Vyi,Vxi);
         
        //归一化到 -PI~PI
        while (angle_rad>PI)
        {
            angle_rad-=2.0f*PI;
        }
        while (angle_rad<-PI)
        {
            angle_rad+=2.0f*PI;
        }

        //线速度
        float speed = sqrtf(Vxi*Vxi + Vyi*Vyi);

        //转速rpm = speed/(2*PI*r)*60
        float rpm = speed *60.0f/(2.0f*PI*agv.wheel_radius);

        agv.wheels[i].steering_angle_rad=angle_rad;
        agv.wheels[i].steering_angle_deg=angle_rad*180.0f/PI;
        agv.wheels[i].wheel_linear_speed=speed;
        agv.wheels[i].wheel_rpm=rpm;    
    }
    return true;
    


    
}

//获取指定轮的目标状态
const WheelTaget_t*Kinematics_GetWheelTarget(uint8_t wheel_index)
{
    if(wheel_index>=4)
    return NULL;
    return &agv.wheels[wheel_index];
}

//获取当前指令
const AGV_Command_t*Kinematics_GetCurrentCommand(void)
{
    return &agv.command;
}

