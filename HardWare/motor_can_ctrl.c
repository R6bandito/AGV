#include "motor_can_ctrl.h"
#include "debug_uart.h"
#include "CAN_Cus.h"


/* —————————————————————————————————————————————————————— */
                    /* 全局Static变量 */
static const Cus_CAN_GPIO_t s_gpio = 
{ 
    .Alternate          = 0, 
    .CAN_GPIO_RX        = GPIO_PIN_11, 
    .CAN_GPIO_TX        = GPIO_PIN_12,
    .CAN_GPIOPort_x     = GPIOA
};

static MotorStatus_t StatusSteerArr[MOTOR_MAX_SUPPORT_STEER];       // 转向驱动设备的状态结构数组. 
static MotorStatus_t StatusTracArr[MOTOR_MAX_SUPPORT_TRAC];         // 行走驱动设备的状态结构数组.

static Cus_CAN_Device_t *pDev;      // 全局CAN设备指针.
static Cus_CAN_RxMsg_t RxBuf[64];   // 接收缓冲区.
static Cus_CAN_RxMsg_t BkpBuf[32];  // 备份缓冲区.
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* 全局变量 */

/* 驱动设备的状态结构数组掩码. 1=该槽空闲. 0=该槽被占用(已被注册). 低4位为Steer占用指示. 高4位为Trac占用指示. */
static RMOTOR_U8 DeviceMask = 0xFF;       
extern void CAN_forceStart( void );
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* Public API */
void HX_CAN_Init( void );
void Cus_Motor_MainFunction( void );

/* 错误通知钩子函数. */
__WEAK void Cus_Motor_ErrorHook( RMOTOR_U8 Node_id, RMOTOR_U8 Type, MotorError_t ErrFlag );
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* Public API Steer */

RMOTOR_I8 Cus_Motor_Steer_PosMode_Initial( RMOTOR_U8 Node_id );
void Cus_Motor_Steer_Disable( RMOTOR_U8 Node_id );
void Cus_Motor_Steer_SetAngle( RMOTOR_U8 Node_id, RMOTOR_FLOT angle_deg, RMOTOR_U32 speed_rpm, RMOTOR_U8 forceSync );
RMOTOR_U8 Cus_Motor_Steer_GetAngle( RMOTOR_U8 Node_id, RMOTOR_FLOT *pAngle );
void Cus_Motor_Steer_EmergencyStop( RMOTOR_U8 Node_id );
RMOTOR_U8 Cus_Motor_Steer_IsTargetReached( RMOTOR_U8 Node_id );
RMOTOR_U8 Cus_Motor_Steer_IsFault( RMOTOR_U8 Node_id );
RMOTOR_U16 Cus_Motor_Steer_StatusWord( RMOTOR_U8 Node_id );
RMOTOR_U8 Cus_Motor_Steer_IsEnable( RMOTOR_U8 Node_id );
/* —————————————————————————————————————————————————————— */

/* —————————————————————————————————————————————————————— */
                    /* Public API Trac */

RMOTOR_I8 Cus_Motor_Trac_VelocityMode_Initial( RMOTOR_U8 Node_id );
void Cus_Motor_Trac_Disable( RMOTOR_U8 Node_id );
void Cus_Motor_Trac_EmergencyStop( RMOTOR_U8 Node_id );
void Cus_Motor_Trac_SetVelocity( RMOTOR_U8 Node_id, RMOTOR_I32 rpm, RMOTOR_FLOT current );
RMOTOR_I32 Cus_Motor_Trac_GetVelocity( RMOTOR_U8 Node_id );
RMOTOR_I32 Cus_Motor_Trac_GetDistanceCM( RMOTOR_U8 Node_id );
RMOTOR_U16 Cus_Motor_Trac_StatusWord( RMOTOR_U8 Node_id );
RMOTOR_U8 Cus_Motor_Trac_IsFault( RMOTOR_U8 Node_id );
RMOTOR_U8 Cus_Motor_Trac_IsEnable( RMOTOR_U8 Node_id );
/* —————————————————————————————————————————————————————— */

/* —————————————————————————————————————————————————————— */
                    /* Static API */
static void motor_NMT_start( RMOTOR_U8 Node_id );
static RMOTOR_I8 motor_findSlotByNodeId( RMOTOR_U8 Node_id, RMOTOR_U8 Type );               // 根据节点ID查表查找对应的索引.
static RMOTOR_I8 motor_findFreeSlot( RMOTOR_U8 Type );                                      // 查找空闲槽位.
static RMOTOR_I8 motor_waitStatusWord( RMOTOR_U8 Node_id, RMOTOR_U8 Type, RMOTOR_U16 expect, RMOTOR_U32 timeouts_ms );       // 等待状态字.
static RMOTOR_U8 motor_canSend( RMOTOR_U16 cob_id, RMOTOR_U8 *data, RMOTOR_U8 dlc );        // 底层统一发送API.
static void motor_modeSelect( RMOTOR_U8 Node_id, RMOTOR_U8 Mode );                          // 模式切换API. 仅作辅助用途.内部不含完整切换的标准流程.

/* TPDO接收处理逻辑. */
static void motor_steerDispatch( RMOTOR_U16 tpdo_id, RMOTOR_U8 *RxB, MotorStatus_t *device );
static void motor_tracDispatch( RMOTOR_U16 tpdo_id, RMOTOR_U8 *RxB, MotorStatus_t *device );

static void CAN_CTRL_Init_Error_Hook( void );     // 初始化错误回调.     
static void CAN_CTRL_DevGet_Error_Hook( void );               

#if (MOTOR_USE_SYS)
static RMOTOR_I8 motorOS_GetLockSlot( RMOTOR_U8 Node_id, RMOTOR_U8 Type );
static RMOTOR_B motorOS_Lock( RMOTOR_U8 Node_id, RMOTOR_U8 Type );
static RMOTOR_B motorOS_UnLock( RMOTOR_U8 Node_id, RMOTOR_U8 Type );
#endif /* MOTOR_USE_SYS */
/* —————————————————————————————————————————————————————— */


/* ———————————————————— Static ————————————————————————————— */
#if (MOTOR_USE_SYS)

static RMOTOR_I8 
motorOS_GetLockSlot( RMOTOR_U8 Node_id, RMOTOR_U8 Type )
{
    /* 遍历表获取索引. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, Type);
    if ( index < 0 )
    {
        /* 设备不在表中. 返回. */
        return -1;
    }

    /* 返回锁数组的槽索引. 用户于外部定义的锁数组，低位用于STEER. 高位用于TRAC. */
    return (Type == MOTOR_STEER_TYPE) ? index : (index + MOTOR_MAX_SUPPORT_STEER);
}


static RMOTOR_B 
motorOS_Lock( RMOTOR_U8 Node_id, RMOTOR_U8 Type )
{
    RMOTOR_I8 slots = motorOS_GetLockSlot(Node_id, Type);
    if ( slots < 0 )
    {
        /* 设备无效返回. */
        return 0;
    }

    return MotorOS_Lock(slots, MOTOR_OS_MUTEX_TIMEOUT_MS);
}


static RMOTOR_B 
motorOS_UnLock( RMOTOR_U8 Node_id, RMOTOR_U8 Type )
{
    RMOTOR_I8 slots = motorOS_GetLockSlot(Node_id, Type);
    if ( slots < 0 )
    {
        return 0;
    }

    return MotorOS_Unlock(slots);
}

#endif /* MOTOR_USE_SYS */

static void 
CAN_CTRL_Init_Error_Hook( void )
{
    /* 暂时死循环处理. */
    while(1);
}


static void 
CAN_CTRL_DevGet_Error_Hook( void )
{
    while(1);
}


/* 发送 NMT 启动报文. */
static void 
motor_NMT_start( RMOTOR_U8 Node_id )
{
    /* 组装NMT帧. */
    RMOTOR_U8 Buf[8] = { 0 };

    RMOTOR_U16 NMT_id = 0x00;

    /* Byte0: 启动命令. Byte1: 节点ID. */
    Buf[0] = 0x01;
    Buf[1] = Node_id;

    motor_canSend(NMT_id, Buf, MOTOR_NMT_DLC);
}


/* 根据节点ID查状态对应表. Type为表类型：0=行走驱动电机, !0=转向驱动电机 */
static RMOTOR_I8 
motor_findSlotByNodeId( RMOTOR_U8 Node_id, RMOTOR_U8 Type )
{
    if ( Type )
    {
        /* 查转向驱动电机表. */
        for( RMOTOR_U8 index = 0; index < MOTOR_MAX_SUPPORT_STEER; index++ )
        {
            if ( StatusSteerArr[index].nodeId == Node_id )
            {
                /* 找到索引. 返回. */
                return index;
            }
        }
    }
    else 
    {
        /* 查行走驱动电机表. */
        for( RMOTOR_U8 index = 0; index < MOTOR_MAX_SUPPORT_TRAC; index++ )
        {
            if ( StatusTracArr[index].nodeId == Node_id )
            {
                return index;
            }
        }  
    }

    /* 未找到索引. 返回-1. */
    return -1;
}


/* 根据传入的表类型检查对应的表单中是否还有空位. Type为表类型：0=行走驱动电机, !0=转向驱动电机. */
static RMOTOR_I8 
motor_findFreeSlot( RMOTOR_U8 Type )
{
    if ( Type )
    {
        for( RMOTOR_U8 index = 0; index < MOTOR_MAX_SUPPORT_STEER; index++ ) 
        {
            if ( DeviceMask & (0x01 << index) )   // bit=1 表示空闲
            {
                return (RMOTOR_I8)index;
            }
        }
    }
    else 
    {
        for( RMOTOR_U8 index = 0; index < MOTOR_MAX_SUPPORT_TRAC; index++ )
        {
            if ( DeviceMask & ((0x01 << MOTOR_MAX_SUPPORT_STEER) << index) )
            {
                return (RMOTOR_I8)index;
            }
        }
    }

    /* 无空闲槽位分配. 返回-1. */
    return -1;
}


static RMOTOR_I8 
motor_waitStatusWord( RMOTOR_U8 Node_id, RMOTOR_U8 Type, RMOTOR_U16 expect, RMOTOR_U32 timeouts_ms )
{
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, Type);
    if ( index < 0 )
    {
        return -1;
    }

    MotorStatus_t *pSta = NULL;
    if ( Type == MOTOR_STEER_TYPE )    pSta = &StatusSteerArr[index];
    if ( Type == MOTOR_TRAC_TYPE )     pSta = &StatusTracArr[index];

    RMOTOR_U32 startTick = HAL_GetTick();
    while( (pSta->statusWord != expect) && (RMOTOR_U32)(HAL_GetTick() - startTick) < timeouts_ms )
    {
        /* 主动推动状态机. */
        Cus_Motor_MainFunction();

        #if (MOTOR_USE_SYS)
        MotorOS_Delay(5);       // OS下延时CPU不会空转. 因此多放出一些时间片供外部运行避免频繁切换任务.
        #else 
        HAL_Delay(1);
        #endif /* MOTOR_USE_SYS */
    }
    if ( pSta->statusWord != expect )   return -2;      // 超时返回.

    /* 成功等到状态字. */
    return 1;
}


static RMOTOR_U8 
motor_canSend( RMOTOR_U16 cob_id, RMOTOR_U8 *data, RMOTOR_U8 dlc )
{
    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.StdId = cob_id;
    txHeader.DLC   = dlc;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;

    for (RMOTOR_U8 retry = 0; retry < 3; retry++)
    {
        if (pDev->Send_IT(pDev, txHeader, data) == HAL_OK)
            return 0;

        #if (MOTOR_USE_SYS)
        MotorOS_Delay(2);
        #else 
        HAL_Delay(2);
        #endif /* MOTOR_USE_SYS */
    }
    return 0xFF;
}


static void 
motor_modeSelect( RMOTOR_U8 Node_id, RMOTOR_U8 Mode )
{
    RMOTOR_U16 cob_id = RPDO2_BASE + Node_id;

    RMOTOR_U8 txBuf[8] = { 0 };
    txBuf[0] = Mode;

    motor_canSend(cob_id, txBuf, MOTOR_RPDO2_DLC);
}


static void 
motor_steerDispatch( RMOTOR_U16 tpdo_id, RMOTOR_U8 *RxB, MotorStatus_t *device )
{
    /* 转向驱动电机 TPDO 处理逻辑. */
    RMOTOR_U16 func_code = tpdo_id & 0x780;   
    RMOTOR_U8 Node_id = tpdo_id & 0x7F;

    switch (func_code)
    {
        case TPDO1_BASE:
        {
            /* 控制字. */
            RMOTOR_U16 controlWord_1 = (RMOTOR_U16)RxB[0];
            RMOTOR_U16 controlWord_2 = (RMOTOR_U16)RxB[1];
            RMOTOR_U16 stateW = ((controlWord_2 << 8) | controlWord_1);

            /* 更新状态. */
            device->statusWord          = stateW;
            device->is_Enable           = (stateW >> 2) & 0x01;
            device->is_TargetReach      = (stateW >> 10) & 0x01;
            device->is_InternalLimit    = (stateW >> 11) & 0x01;

            RMOTOR_U8 newFault = (stateW >> 3) & 0x01;
            if ( !device->is_Fault && newFault )
            {
                /* 检测到错误边沿置起. 通知上层并更新状态. */
                Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_HW_FAULT);
                device->is_Fault = (stateW >> 3) & 0x01;
            }
            device->is_Fault = newFault;

            break;
        }

        case TPDO2_BASE:
        {
            /* 电机异常状态报告. */
            RMOTOR_U16 errWord_1 = (RMOTOR_U16)RxB[0];
            RMOTOR_U16 errWord_2 = (RMOTOR_U16)RxB[1];
            RMOTOR_U16 errWord = ((errWord_2 << 8) | errWord_1);

            RMOTOR_U16 fatal_err_Mask = MOTOR_FAULT_OVERCURRENT_MASK | MOTOR_FAULT_OVERSPEED_MASK | MOTOR_FAULT_ENCODE_FAIL_MASK | MOTOR_FAULT_POWER_MOD_OVERC_MASK;
            RMOTOR_U16 err_Mask = MOTOR_FAULT_DRIVER_OVERHEAT | MOTOR_FAULT_OVERHEAT | MOTOR_FAULT_OVERLOAD | MOTOR_FAULT_HALL_SIGN_ERR;

            device->lastErrorCode = errWord;
            if ( device->lastErrorCode & fatal_err_Mask )
            {
                /* 出现致命错误. 通知上层并紧急停机. */
                device->is_Fault = 1;
                Cus_Motor_Steer_EmergencyStop(Node_id);
                Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_HW_FAULT);
            }
            else if ( device->lastErrorCode & err_Mask )
            {
                /* 出现错误但非致命. 通知上层. */
                device->is_Fault = 1;
                Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_HW_FAULT);
            }
            else 
            {
                /* 正常情况. */
                device->is_Fault = 0;
            }

            break;
        }

        case TPDO4_BASE:
        {
            /* 获取当前转向电机的实时状态. */
            RMOTOR_I32 anglePosFeedback = (RMOTOR_I32)(RxB[4] | (RxB[5] << 8) | (RxB[6] << 16) | (RxB[7] << 24));

            device->actual_AngleUnits = (RMOTOR_U16)(anglePosFeedback & 0xFFFF);    // 低16位.角度. 
            device->actual_AngleDeg = ((RMOTOR_FLOT)(device->actual_AngleUnits) / (RMOTOR_FLOT)MOTOR_STEER_POS_UNIT_PER_REVOLUTION) * 360.0f;     // 换算成角度.

            break;
        }

        default:    break;
    }
}


static void 
motor_tracDispatch( RMOTOR_U16 tpdo_id, RMOTOR_U8 *RxB, MotorStatus_t *device )
{
    /* 直行驱动电机 TPDO 处理逻辑. */
    RMOTOR_U16 func_code = tpdo_id & 0x780;   
    RMOTOR_U8 Node_id = tpdo_id & 0x7F;

    switch (func_code)
    {
        case TPDO1_BASE:
        {
            /* 控制字. */
            RMOTOR_U16 controlWord_1 = (RMOTOR_U16)RxB[0];
            RMOTOR_U16 controlWord_2 = (RMOTOR_U16)RxB[1];
            RMOTOR_U16 stateW = ((controlWord_2 << 8) | controlWord_1);

            /* 更新状态. */
            device->statusWord          = stateW;
            device->is_Enable           = (stateW >> 2) & 0x01;
            device->is_TargetReach      = (stateW >> 10) & 0x01;
            device->is_InternalLimit    = (stateW >> 11) & 0x01;

            RMOTOR_U8 newFault = (stateW >> 3) & 0x01;
            if ( !device->is_Fault && newFault )
            {
                /* 检测到错误边沿置起. 通知上层并更新状态. */
                Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_HW_FAULT); 
                device->is_Fault = (stateW >> 3) & 0x01;
            }
            device->is_Fault = newFault;

            break;
        }

        case TPDO2_BASE:
        {
            /* 电机异常状态报告. */
            RMOTOR_U16 errWord_1 = (RMOTOR_U16)RxB[0];
            RMOTOR_U16 errWord_2 = (RMOTOR_U16)RxB[1];
            RMOTOR_U16 errWord = ((errWord_2 << 8) | errWord_1);

            RMOTOR_U16 fatal_err_Mask = MOTOR_FAULT_OVERCURRENT_MASK | MOTOR_FAULT_OVERSPEED_MASK | MOTOR_FAULT_ENCODE_FAIL_MASK | MOTOR_FAULT_POWER_MOD_OVERC_MASK;
            RMOTOR_U16 err_Mask = MOTOR_FAULT_DRIVER_OVERHEAT | MOTOR_FAULT_OVERHEAT | MOTOR_FAULT_OVERLOAD | MOTOR_FAULT_HALL_SIGN_ERR;

            device->lastErrorCode = errWord;
            if ( device->lastErrorCode & fatal_err_Mask )
            {
                /* 出现致命错误. 通知上层并紧急停机. */
                device->is_Fault = 1;
                Cus_Motor_Trac_EmergencyStop(Node_id);
                Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_HW_FAULT);
            }
            else if ( device->lastErrorCode & err_Mask )
            {
                /* 出现错误但非致命. 通知上层. */
                device->is_Fault = 1;
                Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_HW_FAULT);
            }
            else 
            {
                /* 正常情况. */
                device->is_Fault = 0;
            }

            break;
        }

        case TPDO3_BASE:
        {
            /* 直行电机速度信息报文处理. */
            RMOTOR_I32 speed_raw = (RxB[0] | (RxB[1] << 8) | (RxB[2] << 16) | (RxB[3] << 24));
            int16_t current = (RxB[4] | (RxB[5] << 8));

            device->current_Rpm = speed_raw;
            device->current_Apm = current;

            break;
        }

        case TPDO4_BASE:
        {
            /* 直行电机圈数与里程. */
            RMOTOR_I32 circulNum = (RxB[0] | (RxB[1] << 8) | (RxB[2] << 16) | (RxB[3] << 24));

            device->total_Revolutions = circulNum;
            device->actual_DistanceMM = device->total_Revolutions * MOTOR_TRAC_CIRCUMFERENCE;
        }

        default:    break;
    }
}


/* ———————————————————— Public ————————————————————————————— */
void 
HX_CAN_Init( void )
{
    CANInitConfig_t *pInit;
    CANFilterConfig_t *pFilter;

    #if (MOTOR_USE_SYS)
    /* 初始化锁. */
    MotorOS_Init( MOTOR_OS_MUTEX_NUM );
    #endif /* MOTOR_USE_SYS */

    RMOTOR_U8 iReturn = Factory_CANInitConfig_t(&pInit);
    if ( iReturn )
    {
        printf("CAN InitCFG Config Failed!\n");
        CAN_CTRL_Init_Error_Hook();
    }

    RMOTOR_U8 fReturn = Factory_CANFilterConfig_t(&pFilter);
    if ( fReturn )
    {
        printf("CAN InitCFG Config Failed!\n");
        CAN_CTRL_Init_Error_Hook();
    }

    /* bxCAN设备配置. */
    pInit->CAN_gpio     = s_gpio;
    pInit->baudrate     = CAN_BAUDRATE_500K;
    pInit->Instance     = CAN1;
    pInit->Mode         = MODE_NORMAL;        // 暂时使用回环模式进行自测.
    pInit->SJW          = Cus_CAN_SJW_1Tq;

    pInit->is_AutoBusOff            = false;
    pInit->is_AutoRestransmission   = false;
    pInit->is_AutoWakeUP            = false;
    pInit->is_ReceiveFifoLocked     = false;
    pInit->is_TimeTriggeredMode     = false;
    pInit->is_TransmitFifoPriority  = false;

    /* 初始化及释放. */
    pInit->Cus_CAN_Init(pInit);
    pInit->Self_Release(&pInit);

    /* 过滤器配置. */
    pFilter->FIFOAssignment     = Cus_CAN_FIFOASSIGNMENT_FIFO0;     // FIFO0.
    pFilter->FilterBank         = 0;                                // 过滤器0.
    pFilter->Mode               = Cus_CAN_FILTERMODE_IDMASK;        // 掩码模式.
    pFilter->Scale              = Cus_CAN_SCALE_32BIT;              // 过滤器32位位宽.
    pFilter->is_Activation      = Cus_FILTER_Enable;

    /* 配置全通过滤器. */
    Cus_CAN_Filter_SetStdMask32(pFilter, CAN_FILTER_RTR_NONE, 0x00, 0x00);

    pFilter->Cus_CAN_FilterInit(pFilter, CAN1);
    pFilter->Self_Release(&pFilter);

    /* bxCAN启动补丁. */
    CAN_forceStart();

    /* 启动bxCAN外设. */
    Cus_CAN_Start(CAN1);

    /* 获取CAN1的设备实例. */
    pDev = Cus_CAN_getControlBlock(CAN1);
    if ( !pDev )
    {
        printf("Device Accquired Failed.\n");
        CAN_CTRL_DevGet_Error_Hook();
    }

    /* 注册接收缓冲区. */
    pDev->registerRxBuffer(pDev, (void *)RxBuf, sizeof(RxBuf), FIFO_IDX_0);

    /* 注册备份缓冲区. */
    pDev->registerBackUPBuffer(pDev, (void *)BkpBuf, sizeof(BkpBuf));

    /* 开中断. */
    pDev->EnableInterrupt(pDev, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING);
}



void 
Cus_Motor_MainFunction( void )
{
    CAN_RxHeaderTypeDef rxHeader;
    RMOTOR_U8 RxData[8] = { 0 };

    /* 只要缓冲区还有CAN报文就一直取. */
    while( pDev->Receive_IT(pDev, &rxHeader, RxData, FIFO_IDX_0) == HAL_OK )
    {
        /* 解析报文ID. */
        RMOTOR_U16 tpdo_id = rxHeader.StdId;
        RMOTOR_U8 Node_id = tpdo_id & 0x7F;

        /* 查表. 检查该节点是否在驱动库有注册记录. */
        RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
        RMOTOR_U8 belong = MOTOR_STEER_TYPE;     // belong=1. 该节点来自转向电机表.
        if ( index < 0 )
        {
            /* 不在转向电机设备表. 查直行电机设备表. */
            index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
            if ( index < 0 )
            {
                /* 不在两张表中. 未知设备. 不派发给处理逻辑. 返回. */
                continue;
            }

            belong = MOTOR_TRAC_TYPE;   // belong-0. 该节点来自直行电机表.
        }

        /* 查到设备信息. 对报文进行分发处理. */
        switch (belong)
        {
            case MOTOR_STEER_TYPE: motor_steerDispatch(tpdo_id, RxData, &StatusSteerArr[index]);  break;
            case MOTOR_TRAC_TYPE:  motor_tracDispatch(tpdo_id, RxData, &StatusTracArr[index]);  break;

            default:    break;
        }
    }
}



/* ——————————————————— Public API Steer —————————————————————————— */

RMOTOR_I8 
Cus_Motor_Steer_PosMode_Initial( RMOTOR_U8 Node_id )
{
    MOTOR_API_ENTER_RET(Node_id, MOTOR_STEER_TYPE);

    /* 检查该设备是否已在状态表中. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index >= 0 )
    {
        /* 该设备已存在于状态表中. 检查该设备是否因为Error需要重新Initial. */
        if ( StatusSteerArr[index].is_Fault == 0 )
        {
            /* 无错误状态. 不允许重复初始化. */
            MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
            return MOTOR_ERROR;
        }
        else 
        { 
            /* 发送清除报警信息. */
            RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;
            RMOTOR_U8 Buf[8] = { 0 };
            Buf[0] = 0x80;
            Buf[1] = 0x00;      // 0x0080. 清除报警信息.

            motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);

            RMOTOR_U32 startTick = HAL_GetTick();
            while( (RMOTOR_U32)(HAL_GetTick() - startTick) < 500 )
            {
                /* 轮询ERROR标志.  */
                if ( StatusSteerArr[index].is_Fault == 0 )  break;
                Cus_Motor_MainFunction();

                #if (MOTOR_USE_SYS)
                MotorOS_Delay(5);
                #else 
                HAL_Delay(1);
                #endif /* MOTOR_USE_SYS */
            }
            /* 等待超时. ERROR标志未被消去. 错误. */
            if ( StatusSteerArr[index].is_Fault )   goto ERROR;

            /* ERROR标志被成功消去. 重走流程. */
        }   
    }
    else 
    {
        /* 该设备不在列表中. 加入列表进行状态管理. */
        index = motor_findFreeSlot(MOTOR_STEER_TYPE);
        if ( index < 0 )   
        {
            /* 状态表无更多空间分配. 返回. */ 
            Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_NO_SLOT);
            MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
            return MOTOR_ERROR;  
        }  

        StatusSteerArr[index].nodeId = Node_id;
        StatusSteerArr[index].runingMode = MOTOR_UNKNOWN_MODE;
        StatusSteerArr[index].statusWord = 0;
        DeviceMask &= ~(1 << index);     // 设置位图占用.
    }

    /* 发送NMT启动命令. */
    motor_NMT_start(Node_id);

    /* 等待状态字. */
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0050, 500) < 0 )  goto ERROR;

    /* 模式设置为绝对位置模式. */
    motor_modeSelect(Node_id, STEER_MODE_POSITION);

    /* 校准回零. */
    RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;
    RMOTOR_U8 Buf[8] = { 0 };
    Buf[0] = 0x06;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0031, 500) < 0 )  goto ERROR;

    Buf[0] = 0x07;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0033, 500) < 0 )  goto ERROR;

    Buf[0] = 0x0F;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0437, 500) < 0 )  goto ERROR;

    Buf[0] = 0x0F;
    Buf[1] = 0x80;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0437, 500) < 0 )  goto ERROR;

    /* 等待回零完毕. */
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x8437, 5000) < 0 )  goto ERROR;

    /* 回零结束. 绝对位置模式初始化完毕. 更新状态返回. */
    StatusSteerArr[index].runingMode = MOTOR_POSI_MODE;
    MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
    return MOTOR_OK;

ERROR:
    /* 状态字等待失败. 释放占用，通知钩子. */
    StatusSteerArr[index].nodeId = 0;
    DeviceMask |= (0x01 << index);      // 槽位归还.
    StatusSteerArr[index].runingMode = MOTOR_UNKNOWN_MODE;
    Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);

    MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
    return MOTOR_ERROR;
}


void 
Cus_Motor_Steer_Disable( RMOTOR_U8 Node_id )
{
    MOTOR_API_ENTER_VOID(Node_id, MOTOR_STEER_TYPE);

    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        /* 未找到设备. 返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_NOT_FOUND);
        MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
        return;
    }

    /* 计算 COB_ID. */
    RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;

    RMOTOR_U8 txBuf[8] = { 0 };
    txBuf[0] = 0x05;
    txBuf[1] = 0x00;

    motor_canSend(cob_id, txBuf, MOTOR_RPDO1_DLC);

    /* 等待状态字. */
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x9070, 500) < 0 )
    {
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);
    }

    MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
}


void 
Cus_Motor_Steer_EmergencyStop( RMOTOR_U8 Node_id )
{
    MOTOR_API_ENTER_VOID(Node_id, MOTOR_STEER_TYPE);

    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        /* 未找到设备. 返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_NOT_FOUND);
        MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
        return;
    }

    RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;

    RMOTOR_U8 Buf[8] = { 0 };
    Buf[0] = 0x02;
    Buf[1] = 0x00;

    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);

    MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
}


void 
Cus_Motor_Steer_SetAngle( RMOTOR_U8 Node_id, RMOTOR_FLOT angle_deg, RMOTOR_U32 speed_rpm, RMOTOR_U8 forceSync )
{
    MOTOR_API_ENTER_VOID(Node_id, MOTOR_STEER_TYPE);

    RMOTOR_I8 slots = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( slots < 0 )
    {
        /* 表中未查到设备信息. 通知上层后返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_NOT_FOUND);
        goto END;
    }

    if ( StatusSteerArr[slots].runingMode != MOTOR_POSI_MODE )
    {
        /* 该API必须工作于绝对位置模式. 当前模式不匹配. 通知并返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_MODE_DISMATCH);
        goto END;
    }

    /* 检查参数是否合法. */
    if ( angle_deg > (RMOTOR_FLOT)MOTOR_STEER_ANGLE_LIMIT || angle_deg < -MOTOR_STEER_ANGLE_LIMIT )
    {
        /* 传入的度数不合法. 保持当前状态并上报上层. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_PARAM_ILLEGAL);
        goto END;
    }

    if ( speed_rpm > MOTOR_STEER_SPEED_RPM_LIMIT )
    {
        /* 速度超限. 通知上层并将速度钳位至当前最大. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_PARAM_ILLEGAL);
        speed_rpm = MOTOR_STEER_SPEED_RPM_LIMIT;
    }

    /* 计算角度脉冲. */
    RMOTOR_FLOT circle = angle_deg / 360.0f;
    RMOTOR_I32 angleValue = (RMOTOR_I32)(circle * MOTOR_STEER_POS_UNIT_PER_REVOLUTION);
    
    RMOTOR_U16 cob_id = RPDO4_BASE + Node_id;
    RMOTOR_U8 Buf[8] = { 0 };

    /* 角度设置. */
    Buf[0] = (RMOTOR_U8)(angleValue);           // LSB序
    Buf[1] = (RMOTOR_U8)(angleValue >> 8);
    Buf[2] = (RMOTOR_U8)(angleValue >> 16);
    Buf[3] = (RMOTOR_U8)(angleValue >> 24);    

    /* 速度设置. */
    Buf[4] = (RMOTOR_U8)(speed_rpm);            // LSB
    Buf[5] = (RMOTOR_U8)(speed_rpm >> 8);
    Buf[6] = (RMOTOR_U8)(speed_rpm >> 16);
    Buf[7] = (RMOTOR_U8)(speed_rpm >> 24);      

    /* 发送位置指令. */
    motor_canSend(cob_id, Buf, MOTOR_RPDO4_DLC);
    memset(Buf, 0, sizeof(Buf));

    Buf[0] = 0x3F;
    Buf[1] = 0x00;
    cob_id = RPDO1_BASE + Node_id;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);      // 发送执行指令.

    if ( forceSync )
    {
        if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x9437, 5000) < 0 ) 
        {
            /* 没等到状态字？可能由于超时过短. 而当前轮子依然在移动. 通知上层但不修改设备任何状态. */
            Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);
        }
    }

END:
    MOTOR_API_EXIT(Node_id, MOTOR_STEER_TYPE);
    return;
}


RMOTOR_U8 
Cus_Motor_Steer_GetAngle( RMOTOR_U8 Node_id, RMOTOR_FLOT *pAngle )
{
    /* 查表. 获取设备. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        /* 未找到设备. 返回. */
        return 0;
    }

    /* 找到设备. 返回当前角度. */
    *pAngle = StatusSteerArr[index].actual_AngleDeg;

    return 1;
}


RMOTOR_U8 
Cus_Motor_Steer_IsTargetReached( RMOTOR_U8 Node_id )
{
    /* 查表. 获取设备. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return (StatusSteerArr[index].is_TargetReach) ? 1 : 0;
}


RMOTOR_U8 
Cus_Motor_Steer_IsFault( RMOTOR_U8 Node_id )
{
    /* 查表. 获取设备. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return (StatusSteerArr[index].is_Fault) ? 1 : 0;
}


RMOTOR_U16 
Cus_Motor_Steer_StatusWord( RMOTOR_U8 Node_id )
{
    /* 查表. 获取设备. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return StatusSteerArr[index].statusWord;
}


RMOTOR_U8 
Cus_Motor_Steer_IsEnable( RMOTOR_U8 Node_id )
{
    /* 查表. 获取设备. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return StatusSteerArr[index].is_Enable;
}


/* ——————————————————— Public API Trac —————————————————————————— */

RMOTOR_I8 
Cus_Motor_Trac_VelocityMode_Initial( RMOTOR_U8 Node_id )
{
    MOTOR_API_ENTER_RET(Node_id, MOTOR_TRAC_TYPE);

    /* 检查该设备是否已在状态表中. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index >= 0 )
    {
        /* 该设备已存在于状态表中. 检查该设备是否因为Error需要重新Initial. */
        if ( StatusTracArr[index].is_Fault == 0 )
        {
            /* 无错误状态. 不允许重复初始化. */
            MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
            return MOTOR_ERROR;
        }
        else 
        { 
            /* 发送清除报警信息. */
            RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;
            RMOTOR_U8 Buf[8] = { 0 };
            Buf[0] = 0x80;
            Buf[1] = 0x00;      // 0x0080. 清除报警信息.

            motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);

            RMOTOR_U32 startTick = HAL_GetTick();
            while( (RMOTOR_U32)(HAL_GetTick() - startTick) < 500 )
            {
                /* 轮询ERROR标志.  */
                if ( StatusTracArr[index].is_Fault == 0 )  break;
                Cus_Motor_MainFunction();

                #if (MOTOR_USE_SYS)
                MotorOS_Delay(5);
                #else 
                HAL_Delay(1);
                #endif /* MOTOR_USE_SYS */
            }
            /* 等待超时. ERROR标志未被消去. 错误. */
            if ( StatusTracArr[index].is_Fault )   goto ERROR;

            /* ERROR标志被成功消去. 重走流程. */
        }   
    }
    else 
    {
        /* 该设备不在列表中. 加入列表进行状态管理. */
        index = motor_findFreeSlot(MOTOR_TRAC_TYPE);
        if ( index < 0 )   
        {
            /* 状态表无更多空间分配. 返回. */ 
            Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_NO_SLOT);
            MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
            return MOTOR_ERROR;  
        }  

        StatusTracArr[index].nodeId = Node_id;
        StatusTracArr[index].runingMode = MOTOR_UNKNOWN_MODE;
        StatusTracArr[index].statusWord = 0;
        DeviceMask &= ~((0x01 << MOTOR_MAX_SUPPORT_STEER) << index);     // 设置位图占用.
    }

    motor_NMT_start(Node_id);

    if ( motor_waitStatusWord(Node_id, MOTOR_TRAC_TYPE, 0x0050, 500) < 0 )  goto ERROR;

    /* 设置速度控制模式及上电使能. */
    motor_modeSelect(Node_id, MOTOR_VEL_MODE);

    RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;

    RMOTOR_U8 Buf[8] = { 0 };
    Buf[0] = 0x06;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_TRAC_TYPE, 0x0031, 500) < 0 )  goto ERROR;

    Buf[0] = 0x07;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_TRAC_TYPE, 0x0033, 500) < 0 )  goto ERROR;

    Buf[0] = 0x0F;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);
    if ( motor_waitStatusWord(Node_id, MOTOR_TRAC_TYPE, 0x8037, 500) < 0 )  goto ERROR;

    StatusTracArr[index].runingMode = MOTOR_VEL_MODE;
    MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
    return MOTOR_OK;


ERROR:
    /* 状态字等待失败. 释放占用，通知钩子. */
    StatusTracArr[index].nodeId = 0;
    DeviceMask |= ((0x01 << MOTOR_MAX_SUPPORT_STEER) << index);      // 槽位归还.
    StatusTracArr[index].runingMode = MOTOR_UNKNOWN_MODE;
    Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_STATUS_WORD);
    
    MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
    return MOTOR_ERROR;
}


void 
Cus_Motor_Trac_Disable( RMOTOR_U8 Node_id )
{
    MOTOR_API_ENTER_VOID(Node_id, MOTOR_TRAC_TYPE);

    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        /* 表中未找到设备. 上报返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_NOT_FOUND);
        MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
        return;
    }

    RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;

    RMOTOR_U8 Buf[8] = { 0 };
    Buf[0] = 0x05;
    Buf[1] = 0x00;

    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);

    if ( motor_waitStatusWord(Node_id, MOTOR_TRAC_TYPE, 0x8070, 500) < 0 )
    {
        /* 未等到状态字. 上报上层并返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_STATUS_WORD);
    }

    MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
}


void 
Cus_Motor_Trac_EmergencyStop( RMOTOR_U8 Node_id )
{
    MOTOR_API_ENTER_VOID(Node_id, MOTOR_TRAC_TYPE);

    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        /* 表中未找到设备. 上报返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_NOT_FOUND);
        MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
        return;
    }

    RMOTOR_U16 cob_id = RPDO1_BASE + Node_id;

    RMOTOR_U8 Buf[8] = { 0 };
    Buf[0] = 0x02;
    Buf[1] = 0x00;

    motor_canSend(cob_id, Buf, MOTOR_RPDO1_DLC);

    MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
}


void 
Cus_Motor_Trac_SetVelocity( RMOTOR_U8 Node_id, RMOTOR_I32 rpm, RMOTOR_FLOT current )
{
    MOTOR_API_ENTER_VOID(Node_id, MOTOR_TRAC_TYPE);

    /* 查表. */
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        /* 表中未找到设备. 上报返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_NOT_FOUND);
        goto END;
    }

    if ( StatusTracArr[index].runingMode != MOTOR_VEL_MODE )
    {
        /* 模式不匹配. 上报上层返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_MODE_DISMATCH);
        goto END;
    }

    if ( rpm > MOTOR_TRAC_MAX_SPEED_RPM || rpm < -MOTOR_TRAC_MAX_SPEED_RPM )
    {
        /* 速度超限. 上报返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_PARAM_ILLEGAL);
        goto END;
    }

    /* 取电流绝对值判断. */
    if ( current < 0.0f )   current = -current;
    if ( current > (RMOTOR_FLOT)(MOTOR_TRAC_MAX_CURRENT / 100.0f) )
    {
        /* 电流超限. 上报返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_TRAC_TYPE, MOTOR_ERR_PARAM_ILLEGAL);
        goto END;
    }

    RMOTOR_U16 cob_id = RPDO3_BASE + Node_id;
    RMOTOR_I32 velocity_value = rpm * 10;

    RMOTOR_U8 Buf[8] = { 0 };
    Buf[0] = velocity_value & 0xFF;
    Buf[1] = (velocity_value >> 8) & 0xFF;
    Buf[2] = (velocity_value >> 16) & 0xFF;
    Buf[3] = (velocity_value >> 24) & 0xFF;

    int16_t cur = (int16_t)(current * 100.0f);

    Buf[4] = cur & 0xFF;
    Buf[5] = (cur >> 8) & 0xFF;

    motor_canSend(cob_id, Buf, MOTOR_RPDO3_DLC);

END:
    MOTOR_API_EXIT(Node_id, MOTOR_TRAC_TYPE);
    return;
}


RMOTOR_I32 
Cus_Motor_Trac_GetVelocity( RMOTOR_U8 Node_id )
{
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return (StatusTracArr[index].current_Rpm / 10);  // 0.1r/min.
}


RMOTOR_I32 
Cus_Motor_Trac_GetDistanceCM( RMOTOR_U8 Node_id )
{
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return (StatusTracArr[index].actual_DistanceMM / 10);
}


RMOTOR_U16 
Cus_Motor_Trac_StatusWord( RMOTOR_U8 Node_id )
{
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return StatusTracArr[index].statusWord;
}


RMOTOR_U8 
Cus_Motor_Trac_IsFault( RMOTOR_U8 Node_id )
{
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        return 0;
    }

    return StatusTracArr[index].is_Fault;
}


RMOTOR_U8 
Cus_Motor_Trac_IsEnable( RMOTOR_U8 Node_id )
{
    RMOTOR_I8 index = motor_findSlotByNodeId(Node_id, MOTOR_TRAC_TYPE);
    if ( index < 0 )
    {
        /* 未找到设备.  */
        return 0;
    }

    return StatusTracArr[index].is_Enable;
}


__WEAK void 
Cus_Motor_ErrorHook( RMOTOR_U8 Node_id, RMOTOR_U8 Type, MotorError_t ErrFlag )
{
    /* 钩子函数默认空实现. 等待用户自行实现. */
    (void)Node_id;
    (void)Type;
    (void)ErrFlag;
}


/* ——————————————————— RTOS Default —————————————————————————— */
#if (MOTOR_USE_SYS)

__WEAK void 
MotorOS_Init( RMOTOR_U8 needMutexNum )
{
    (void)needMutexNum;
}


__WEAK RMOTOR_B 
MotorOS_Lock( RMOTOR_U8 slots, RMOTOR_U32 timeouts )
{
    (void)slots;
    (void)timeouts;

    return 1;
}


__WEAK RMOTOR_B 
MotorOS_Unlock( RMOTOR_U8 slots )
{
    (void)slots;

    return 1;
}


__WEAK void 
MotorOS_Delay( RMOTOR_U32 delay_ms )
{
    HAL_Delay(delay_ms);
}


#endif /* MOTOR_USE_SYS */


