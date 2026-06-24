#ifndef __MOTOR_CAN_CTRL_H__
#define __MOTOR_CAN_CTRL_H__


#include "stm32f1xx_hal.h"


/* —————————————————————————————————————————————————————— */
typedef enum                        // 错误码枚举.
{
    MOTOR_OK                = 0,
    MOTOR_ERR_STATUS_WORD   = 1,    // 状态字等待超时
    MOTOR_ERR_NOT_FOUND     = 2,    // Node-ID 未在状态表中注册
    MOTOR_ERR_NO_SLOT       = 3,    // 状态表已满，无空闲槽位
    MOTOR_ERR_COMM_LOST     = 4,    // 运行中心跳丢失
    MOTOR_ERR_HW_FAULT      = 5,    // 从机状态字报告硬件故障
    MOTOR_ERR_BUS_OFF       = 6,    // CAN 总线关闭

} MotorError_t;

typedef enum                        // 转向驱动电机状态枚举.
{
    STEER_POSI_MODE,                // 绝对位置模式.
    STEER_VEL_MODE,                 // 速度模式.
    STEER_TORQUE_MODE,              // 转矩模式.
    STEER_INTER_POS_MODE            // 插值位置模式.

} MotorMode_t;


typedef struct 
{
    uint8_t nodeId;                /* 节点ID. */
    uint16_t statusWord;           /* 状态字. */
    MotorMode_t runingMode;        /* 当前运行模式. */
    uint8_t is_Error;              /* 错误状态指示. 0=无错误, !0=有错误. */

} MotorStatus_t;
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* DEFINES */
    #define MOTOR_MAX_SUPPORT_STEER               (4)       // 最大支持4个转向舵机.
    #define MOTOR_MAX_SUPPORT_TRAC                (4)       // 最大支持4个直行舵机.

    #define MOTOR_TRAC_TYPE                       (0)
    #define MOTOR_STEER_TYPE                      (1)

    /* 转向电机模式宏定义. */
    #define STEER_MODE_POSITION                   (0x01)
    #define STEER_MODE_VELOCITY                   (0x03)
    #define STEER_MODE_TORQUE                     (0x04)
    #define STEER_MODE_INTER_POS                  (0x07)

    #define RPDO1_BASE                            (0x200)
    #define RPDO2_BASE                            (0x300)
    #define RPDO3_BASE                            (0x400)
    #define RPDO4_BASE                            (0x500)

    #define TPDO1_BASE                            (0x180)
    #define TPDO2_BASE                            (0x280)
    #define TPDO3_BASE                            (0x380)
    #define TPDO4_BASE                            (0x480)
/* —————————————————————————————————————————————————————— */

/* —————————————————————————————————————————————————————— */
/* CAN 底层Init. */
void HX_CAN_Init( void );

/* 核心驱动状态机. 需要周期性调用. */
/* 内部轮询接收缓冲区. 取出CAN报文后进行解包处理. */
void Cus_Motor_MainFunction( void );

/* 上层错误通知钩子. */
__WEAK void Cus_Motor_ErrorHook( uint8_t Node_id, uint8_t Type, MotorError_t ErrFlag );
/* —————————————————————————————————————————————————————— */


/* ================================================ */
/*                   转向驱动 (STEER)                */
/* ================================================ */

/* 转向电机初始化.(电机快速启动API) */
uint8_t Cus_Motor_Steer_Initial( uint8_t Node_id );

/* 转向电机失能. */
void Cus_Motor_Steer_Disable( uint8_t Node_id );

/* 转向电机模式选择. */
void Cus_Motor_Steer_ModeSelect( uint8_t Node_id, uint8_t Mode );

/* 
   转向电机控制（角度制）
   angle_deg: 角度值，范围 -360.0 ~ +360.0（正负代表方向）
   speed_rpm: 转向时的旋转速度（决定转的快慢，单位 rpm）
*/
void Cus_Motor_SetPosition( uint8_t Node_id, float angle_deg, uint16_t speed_rpm );


#endif /* __MOTOR_CAN_CTRL_H__ */
