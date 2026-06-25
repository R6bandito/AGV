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
uint8_t DeviceMask = 0xFF;       
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* Public API */
void HX_CAN_Init( void );
void Cus_Motor_MainFunction( void );

/* 错误通知钩子函数. */
__WEAK void Cus_Motor_ErrorHook( uint8_t Node_id, uint8_t Type, MotorError_t ErrFlag );
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* Public API Steer */

uint8_t Cus_Motor_Steer_PosMode_Initial( uint8_t Node_id );
void Cus_Motor_Steer_Disable( uint8_t Node_id );
void Cus_Motor_Steer_SetAngle( uint8_t Node_id, float angle_deg, uint32_t speed_rpm );
void Cus_Motor_Steer_EmergencyStop( uint8_t Node_id );
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* Static API */
static void motor_NMT_start( uint8_t Node_id );
static int8_t motor_findSlotByNodeId( uint8_t Node_id, uint8_t Type );          // 根据节点ID查表查找对应的索引.
static int8_t motor_findFreeSlot( uint8_t Type );                               // 查找空闲槽位.
static int8_t motor_waitStatusWord( uint8_t Node_id, uint8_t Type, uint16_t expect, uint32_t timeouts_ms );       // 等待状态字.
static uint8_t motor_canSend( uint16_t cob_id, uint8_t *data, uint8_t dlc );     // 底层统一发送API.
static void motor_modeSelect( uint8_t Node_id, uint8_t Mode );                  // 模式切换API. 仅作辅助用途.内部不含完整切换的标准流程.

/* TPDO接收处理逻辑. */
static void motor_steerDispatch( uint16_t tpdo_id, uint8_t *RxB, MotorStatus_t *device );
static void motor_tracDispatch( uint16_t tpdo_id, uint8_t *RxB, MotorStatus_t *device );

static void CAN_CTRL_Init_Error_Hook( void );     // 初始化错误回调.     
static void CAN_CTRL_DevGet_Error_Hook( void );               
/* —————————————————————————————————————————————————————— */


/* ———————————————————— Static ————————————————————————————— */
static void CAN_CTRL_Init_Error_Hook( void )
{
    /* 暂时死循环处理. */
    while(1);
}


static void CAN_CTRL_DevGet_Error_Hook( void )
{
    while(1);
}


/* 发送 NMT 启动报文. */
static void motor_NMT_start( uint8_t Node_id )
{
    /* 组装NMT帧. */
    uint8_t Buf[8] = { 0 };

    uint16_t NMT_id = 0x00;

    /* Byte0: 启动命令. Byte1: 节点ID. */
    Buf[0] = 0x01;
    Buf[1] = Node_id;

    motor_canSend(NMT_id, Buf, 8);
}


/* 根据节点ID查状态对应表. Type为表类型：0=行走驱动电机, !0=转向驱动电机 */
static int8_t motor_findSlotByNodeId( uint8_t Node_id, uint8_t Type )
{
    if ( Type )
    {
        /* 查转向驱动电机表. */
        for( uint8_t index = 0; index < MOTOR_MAX_SUPPORT_STEER; index++ )
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
        for( uint8_t index = 0; index < MOTOR_MAX_SUPPORT_TRAC; index++ )
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
static int8_t motor_findFreeSlot( uint8_t Type )
{
    if ( Type )
    {
        for( uint8_t index = 0; index < MOTOR_MAX_SUPPORT_STEER; index++ ) 
        {
            if ( DeviceMask & (0x01 << index) )   // bit=1 表示空闲
            {
                return (int8_t)index;
            }
        }
    }
    else 
    {
        for( uint8_t index = 0; index < MOTOR_MAX_SUPPORT_TRAC; index++ )
        {
            if ( DeviceMask & ((0x01 << MOTOR_MAX_SUPPORT_STEER) << index) )
            {
                return (int8_t)index;
            }
        }
    }

    /* 无空闲槽位分配. 返回-1. */
    return -1;
}


static int8_t motor_waitStatusWord( uint8_t Node_id, uint8_t Type, uint16_t expect, uint32_t timeouts_ms )
{
    int8_t index = motor_findSlotByNodeId(Node_id, Type);
    if ( index < 0 )
    {
        return -1;
    }

    MotorStatus_t *pSta = NULL;
    if ( Type == MOTOR_STEER_TYPE )    pSta = &StatusSteerArr[index];
    if ( Type == MOTOR_TRAC_TYPE )     pSta = &StatusTracArr[index];

    uint32_t startTick = HAL_GetTick();
    while( (pSta->statusWord != expect) && (uint32_t)(HAL_GetTick() - startTick) < timeouts_ms )
    {
        /* 主动推动状态机. */
        Cus_Motor_MainFunction();

        HAL_Delay(1);
    }
    if ( pSta->statusWord != expect )   return -2;      // 超时返回.

    /* 成功等到状态字. */
    return 1;
}


static uint8_t motor_canSend( uint16_t cob_id, uint8_t *data, uint8_t dlc )
{
    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.StdId = cob_id;
    txHeader.DLC   = dlc;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;

    for (uint8_t retry = 0; retry < 3; retry++)
    {
        if (pDev->Send_IT(pDev, txHeader, data) == HAL_OK)
            return 0;
        HAL_Delay(2);
    }
    return 0xFF;
}


static void motor_modeSelect( uint8_t Node_id, uint8_t Mode )
{
    uint16_t cob_id = RPDO2_BASE + Node_id;

    uint8_t txBuf[8] = { 0 };
    txBuf[0] = Mode;

    motor_canSend(cob_id, txBuf, 8);
}


static void motor_steerDispatch( uint16_t tpdo_id, uint8_t *RxB, MotorStatus_t *device )
{
    /* 转向驱动电机 TPDO 处理逻辑. */
    uint16_t func_code = tpdo_id & 0x780;   
    uint8_t Node_id = tpdo_id & 0x7F;

    switch (func_code)
    {
        case TPDO1_BASE:
        {
            /* 控制字. */
            uint16_t controlWord_1 = (uint16_t)RxB[0];
            uint16_t controlWord_2 = (uint16_t)RxB[1];
            uint16_t stateW = ((controlWord_2 << 8) | controlWord_1);

            /* 更新状态. */
            device->statusWord          = stateW;
            device->is_Enable           = (stateW >> 2) & 0x01;
            device->is_TargetReach      = (stateW >> 10) & 0x01;
            device->is_InternalLimit    = (stateW >> 11) & 0x01;

            uint8_t newFault = (stateW >> 3) & 0x01;
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
            uint16_t errWord_1 = (uint16_t)RxB[0];
            uint16_t errWord_2 = (uint16_t)RxB[1];
            uint16_t errWord = ((errWord_2 << 8) | errWord_1);

            uint16_t fatal_err_Mask = MOTOR_FAULT_OVERCURRENT_MASK | MOTOR_FAULT_OVERSPEED_MASK | MOTOR_FAULT_ENCODE_FAIL_MASK | MOTOR_FAULT_POWER_MOD_OVERC_MASK;
            uint16_t err_Mask = MOTOR_FAULT_DRIVER_OVERHEAT | MOTOR_FAULT_OVERHEAT | MOTOR_FAULT_OVERLOAD | MOTOR_FAULT_HALL_SIGN_ERR;

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

        case TPDO3_BASE:
        {

            break;
        }

        case TPDO4_BASE:
        {

            break;
        }

        default:    break;
    }
}


static void motor_tracDispatch( uint16_t tpdo_id, uint8_t *RxB, MotorStatus_t *device )
{

}


/* ———————————————————— Public ————————————————————————————— */
void HX_CAN_Init( void )
{
    CANInitConfig_t *pInit;
    CANFilterConfig_t *pFilter;


    uint8_t iReturn = Factory_CANInitConfig_t(&pInit);
    if ( iReturn )
    {
        printf("CAN InitCFG Config Failed!\n");
        CAN_CTRL_Init_Error_Hook();
    }

    uint8_t fReturn = Factory_CANFilterConfig_t(&pFilter);
    if ( fReturn )
    {
        printf("CAN InitCFG Config Failed!\n");
        CAN_CTRL_Init_Error_Hook();
    }

    /* bxCAN设备配置. */
    pInit->CAN_gpio     = s_gpio;
    pInit->baudrate     = CAN_BAUDRATE_500K;
    pInit->Instance     = CAN1;
    pInit->Mode         = MODE_LOOPBACK;        // 暂时使用回环模式进行自测.
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



uint8_t Cus_Motor_Steer_PosMode_Initial( uint8_t Node_id )
{
    /* 检查该设备是否已在状态表中. */
    int8_t index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index >= 0 )
    {
        /* 该设备已存在于状态表中. 检查该设备是否因为Error需要重新Initial. */
        if ( StatusSteerArr[index].is_Fault == 0 )
        {
            /* 无错误状态. 不允许重复初始化. */
            return (uint8_t)index;
        }
        else 
        { 
            /* 发送清除报警信息. */
            uint16_t cob_id = RPDO1_BASE + Node_id;
            uint8_t Buf[8] = { 0 };
            Buf[0] = 0x80;
            Buf[1] = 0x00;      // 0x0080. 清除报警信息.

            motor_canSend(cob_id, Buf, 8);

            uint32_t startTick = HAL_GetTick();
            while( (uint32_t)(HAL_GetTick() - startTick) < 500 )
            {
                /* 轮询ERROR标志.  */
                if ( StatusSteerArr[index].is_Fault == 0 )  break;
                Cus_Motor_MainFunction();
                HAL_Delay(1);
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
            return 0xFF;  
        }  

        StatusSteerArr[index].nodeId = Node_id;
        StatusSteerArr[index].runingMode = STEER_UNKNOWN_MODE;
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
    uint16_t cob_id = RPDO1_BASE + Node_id;
    uint8_t Buf[8] = { 0 };
    Buf[0] = 0x06;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, 8);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0031, 500) < 0 )  goto ERROR;

    Buf[0] = 0x07;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, 8);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0033, 500) < 0 )  goto ERROR;

    Buf[0] = 0x0F;
    Buf[1] = 0x00;
    motor_canSend(cob_id, Buf, 8);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0437, 500) < 0 )  goto ERROR;

    Buf[0] = 0x0F;
    Buf[1] = 0x80;
    motor_canSend(cob_id, Buf, 8);
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0437, 500) < 0 )  goto ERROR;

    /* 等待回零完毕. */
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x8437, 5000) < 0 )  goto ERROR;

    /* 回零结束. 绝对位置模式初始化完毕. 更新状态返回. */
    StatusSteerArr[index].runingMode = STEER_POSI_MODE;
    return 0;

ERROR:
    /* 状态字等待失败. 释放占用，通知钩子. */
    StatusSteerArr[index].nodeId = 0;
    DeviceMask |= (0x01 << index);      // 槽位归还.
    StatusSteerArr[index].runingMode = STEER_UNKNOWN_MODE;
    Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);

    return 0xFF;
}


void Cus_Motor_MainFunction( void )
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t RxData[8] = { 0 };

    /* 只要缓冲区还有CAN报文就一直取. */
    while( pDev->Receive_IT(pDev, &rxHeader, RxData, FIFO_IDX_0) == HAL_OK )
    {
        /* 解析报文ID. */
        uint16_t tpdo_id = rxHeader.StdId;
        uint8_t Node_id = tpdo_id & 0x7F;

        /* 查表. 检查该节点是否在驱动库有注册记录. */
        int8_t index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
        uint8_t belong = MOTOR_STEER_TYPE;     // belong=1. 该节点来自转向电机表.
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
void Cus_Motor_Steer_Disable( uint8_t Node_id )
{
    /* 计算 COB_ID. */
    uint16_t cob_id = RPDO1_BASE + Node_id;

    uint8_t txBuf[8] = { 0 };
    txBuf[0] = 0x05;
    txBuf[1] = 0x00;

    motor_canSend(cob_id, txBuf, 8);

    /* 等待状态字. */
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x9070, 500) < 0 )
    {
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);
        return;
    }
}


void Cus_Motor_Steer_EmergencyStop( uint8_t Node_id )
{
    uint16_t cob_id = RPDO1_BASE + Node_id;

    uint8_t Buf[8] = { 0 };
    Buf[0] = 0x02;
    Buf[1] = 0x00;

    motor_canSend(cob_id, Buf, 8);
}


void Cus_Motor_Steer_SetAngle( uint8_t Node_id, float angle_deg, uint32_t speed_rpm )
{
    int8_t slots = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( slots < 0 )
    {
        /* 表中未查到设备信息. 通知上层后返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_NOT_FOUND);
        return;
    }

    if ( StatusSteerArr[slots].runingMode != STEER_POSI_MODE )
    {
        /* 该API必须工作于绝对位置模式. 当前模式不匹配. 通知并返回. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_MODE_DISMATCH);
        return;
    }

    /* 检查参数是否合法. */
    if ( angle_deg > (float)MOTOR_STEER_ANGLE_LIMIT || angle_deg < -MOTOR_STEER_ANGLE_LIMIT )
    {
        /* 传入的度数不合法. 保持当前状态并上报上层. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_PARAM_ILLEGAL);
        return;
    }

    if ( speed_rpm > MOTOR_STEER_SPEED_RPM_LIMIT )
    {
        /* 速度超限. 通知上层并将速度钳位至当前最大. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_PARAM_ILLEGAL);
        speed_rpm = MOTOR_STEER_SPEED_RPM_LIMIT;
    }

    /* 计算角度脉冲. */
    float circle = angle_deg / 360.0f;
    int32_t angleValue = (int32_t)(circle * MOTOR_STEER_POS_UNIT_PER_REVOLUTION);
    
    uint16_t cob_id = RPDO4_BASE + Node_id;
    uint8_t Buf[8] = { 0 };

    /* 角度设置. */
    Buf[0] = (uint8_t)(angleValue);           // LSB序
    Buf[1] = (uint8_t)(angleValue >> 8);
    Buf[2] = (uint8_t)(angleValue >> 16);
    Buf[3] = (uint8_t)(angleValue >> 24);    

    /* 速度设置. */
    Buf[4] = (uint8_t)(speed_rpm);            // LSB
    Buf[5] = (uint8_t)(speed_rpm >> 8);
    Buf[6] = (uint8_t)(speed_rpm >> 16);
    Buf[7] = (uint8_t)(speed_rpm >> 24);      

    /* 发送位置指令. */
    motor_canSend(cob_id, Buf, 8);
    memset(Buf, 0, sizeof(Buf));

    Buf[0] = 0x3F;
    Buf[1] = 0x00;
    cob_id = RPDO1_BASE + Node_id;
    motor_canSend(cob_id, Buf, 8);      // 发送执行指令.
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x9437, 5000) < 0 ) 
    {
        /* 没等到状态字？可能由于超时过短. 而当前轮子依然在移动. 通知上层但不修改设备任何状态. */
        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);
    }
}


__WEAK void Cus_Motor_ErrorHook( uint8_t Node_id, uint8_t Type, MotorError_t ErrFlag )
{
    /* 钩子函数默认空实现. 等待用户自行实现. */
    (void)Node_id;
    (void)Type;
    (void)ErrFlag;
}


