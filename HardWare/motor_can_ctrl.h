#ifndef __MOTOR_CAN_CTRL_H__
#define __MOTOR_CAN_CTRL_H__


#include "stm32f1xx_hal.h"


/* —————————————————————————————————————————————————————— */
typedef enum                        // 错误码枚举.
{
    MOTOR_OK                    = 0,
    MOTOR_ERR_STATUS_WORD       = 1,    // 状态字等待超时.
    MOTOR_ERR_NOT_FOUND         = 2,    // Node-ID 未在状态表中注册.
    MOTOR_ERR_NO_SLOT           = 3,    // 状态表已满，无空闲槽位.
    MOTOR_ERR_COMM_LOST         = 4,    // 运行中心跳丢失.
    MOTOR_ERR_HW_FAULT          = 5,    // 从机状态字报告硬件故障.
    MOTOR_ERR_MODE_DISMATCH     = 6,    // 调用API与当前模式不匹配.
    MOTOR_ERR_BUS_OFF           = 7,    // CAN 总线关闭.
    MOTOR_ERR_PARAM_ILLEGAL     = 8,    // 用户参数非法.

} MotorError_t;

typedef enum                         // 转向驱动电机状态枚举.
{
    MOTOR_POSI_MODE             = 0x01,              // 绝对位置模式.
    MOTOR_VEL_MODE              = 0x03,              // 速度模式.
    MOTOR_TORQUE_MODE           = 0x04,              // 转矩模式.
    MOTOR_INTER_POS_MODE        = 0x07,              // 插值位置模式.
    MOTOR_UNKNOWN_MODE          = 0xFF,              // 未知位置.(用于指示模式初始化未完成前.默认值)    

} MotorMode_t;


typedef struct 
{
    uint8_t nodeId;                /* 节点ID. */
    uint16_t statusWord;           /* 状态字. */
    MotorMode_t runingMode;        /* 当前运行模式. */
    uint16_t lastErrorCode;        /* 上一次返回的错误码. */

    uint8_t is_Enable;             /* 判断电机是否上电. */
    uint8_t is_Fault;              /* 判断是否发生故障. */
    uint8_t is_TargetReach;        /* 位置是否到达标志. */
    uint8_t is_InternalLimit;      /* 限位触发标志位. */

    /* 用于转向驱动电机. */
    uint16_t actual_AngleUnits;     /* 0~65535. (0~360°) */
    float actual_AngleDeg;          /* 当前实际角度. */

    /* 用于行走电机. */
    int32_t total_Revolutions;      /* 轮子运转总圈数. */
    float actual_DistanceMM;        /* 当前实际行走距离. 单位mm */   
    int32_t current_Rpm;            /* 直行电机当前运行速度. */
    int16_t current_Apm;            /* 当前的直行电机电流值. 单位0.01A. */   

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

    /* 转向电机TPDO RPDO偏移映射. */
    #define RPDO1_BASE                            (0x200)
    #define RPDO2_BASE                            (0x300)
    #define RPDO3_BASE                            (0x400)
    #define RPDO4_BASE                            (0x500)

    #define TPDO1_BASE                            (0x180)
    #define TPDO2_BASE                            (0x280)
    #define TPDO3_BASE                            (0x380)
    #define TPDO4_BASE                            (0x480)

    /* 转向电机部分参数映射. */
    #define MOTOR_STEER_ANGLE_LIMIT               (180)     // 限幅.(+-180°)
    #define MOTOR_STEER_SPEED_RPM_LIMIT           (500)     // 限幅.(500rpm)

    /* 
        位置模式下的每圈脉冲数（位置当量）. 
        该值定义 1 圈（360°）对应的 CANopen 位置指令增量. 
        出厂默认为65535. 若修改了电子齿轮比等参数进而影响到该参数，请于此处进行正确映射.
    */
    #define MOTOR_STEER_POS_UNIT_PER_REVOLUTION   (UINT16_MAX)

    /* 电机致命错误掩码. */
    #define MOTOR_FAULT_OVERCURRENT_MASK          (0x0004)        // 电机过流.
    #define MOTOR_FAULT_OVERSPEED_MASK            (0x0400)        // 电机过速.
    #define MOTOR_FAULT_ENCODE_FAIL_MASK          (0x0008)        // 编码器故障.
    #define MOTOR_FAULT_POWER_MOD_OVERC_MASK      (0x2000)        // 功率模块过流.

    /* 电机错误掩码. 非致命. 通知上层. */
    #define MOTOR_FAULT_DRIVER_OVERHEAT           (0x0020)        // 驱动过热.
    #define MOTOR_FAULT_OVERHEAT                  (0x0040)        // 电机过热.
    #define MOTOR_FAULT_OVERLOAD                  (0x0080)        // 电机过载.
    #define MOTOR_FAULT_HALL_SIGN_ERR             (0x0100)        // 霍尔信号异常.

    /* 直行电机一圈周长. 用于计算路程. */
    #define MOTOR_TRAC_CIRCUMFERENCE              (314)           // 直径 100ms * 3.14.(请根据轮子实际大小映射.)

    #define MOTOR_TRAC_MAX_SPEED_RPM              (2000)           // 直行电机最大转速.
    #define MOTOR_TRAC_MAX_CURRENT                (500)            // 直行电机最大承载电流.(最大5A)
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
/*                   直行驱动 (TRAC)                 */
/* ================================================ */

/* 直行电机以速度控制模式初始化. */
uint8_t Cus_Motor_Trac_VelocityMode_Initial( uint8_t Node_id );

/* 直行电机失能. */
void Cus_Motor_Trac_Disable( uint8_t Node_id );

/* 直行电机紧急停机. */
void Cus_Motor_Trac_EmergencyStop( uint8_t Node_id );

/*  
   直行电机控制（角度制）
   rpm: 速度值，范围 +- MOTOR_TRAC_MAX_SPEED_RPM（正负代表方向）
   current: 电流.（单位 A. 0 ~ MOTOR_TRAC_MAX_CURRENT / 100.0）
*/
void Cus_Motor_Trac_SetVelocity( uint8_t Node_id, int32_t rpm, float current );

/* 获取当前直行电机速度. */
int32_t Cus_Motor_Trac_GetVelocity( uint8_t Node_id );

/* 获取当前里程. 单位:cm. */
int32_t Cus_Motor_Trac_GetDistanceCM( uint8_t Node_id );

/* 返回完整状态字. 供上层调试用. */
uint16_t Cus_Motor_Trac_StatusWord( uint8_t Node_id );

/* 检查直行电机是否有错误状态置起. */
uint8_t Cus_Motor_Trac_IsFault( uint8_t Node_id );

/* 检查直行电机使能状态. */
uint8_t Cus_Motor_Trac_IsEnable( uint8_t Node_id );

/* ================================================ */
/*                   转向驱动 (STEER)                */
/* ================================================ */

/* 转向电机以绝对位置模式初始化. */
uint8_t Cus_Motor_Steer_PosMode_Initial( uint8_t Node_id );

/* 转向电机失能. */
void Cus_Motor_Steer_Disable( uint8_t Node_id );

/* 
   转向电机控制（角度制）
   angle_deg: 角度值，范围 -360.0 ~ +360.0（正负代表方向）
   speed_rpm: 转向时的旋转速度（决定转的快慢，单位 rpm）
   forceSync: 是否强制同步. (0=异步.不等待电机就位直接返回. 1=同步. 阻塞直到电机就位.)
*/
void Cus_Motor_Steer_SetAngle( uint8_t Node_id, float angle_deg, uint32_t speed_rpm, uint8_t forceSync );

/* 转向电机紧急停机. */
void Cus_Motor_Steer_EmergencyStop( uint8_t Node_id );

/* 获取转向电机当前位置. */
uint8_t Cus_Motor_Steer_GetAngle( uint8_t Node_id, float *pAngle );

/* 检查电机是否已到达目的位置. */
uint8_t Cus_Motor_Steer_IsTargetReached( uint8_t Node_id );

/* 检查电机是否有错误状态置起. */
uint8_t Cus_Motor_Steer_IsFault( uint8_t Node_id );

/* 返回完整状态字. 供上层调试用. */
uint16_t Cus_Motor_Steer_StatusWord( uint8_t Node_id );

/* 检查转向电机使能状态. */
uint8_t Cus_Motor_Steer_IsEnable( uint8_t Node_id );


#endif /* __MOTOR_CAN_CTRL_H__ */
