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

uint8_t Cus_Motor_Steer_Initial( uint8_t Node_id );
void Cus_Motor_Steer_Disable( uint8_t Node_id );
void Cus_Motor_Steer_ModeSelect( uint8_t Node_id, uint8_t Mode );
/* —————————————————————————————————————————————————————— */


/* —————————————————————————————————————————————————————— */
                    /* Static API */
static void motor_NMT_start( uint8_t Node_id );
static int8_t motor_findSlotByNodeId( uint8_t Node_id, uint8_t Type );          // 根据节点ID查表查找对应的索引.
static int8_t motor_findFreeSlot( uint8_t Type );                               // 查找空闲槽位.
static int8_t motor_waitStatusWord( uint8_t Node_id, uint8_t Type, uint16_t expect, uint32_t timeouts_ms );       // 等待状态字.
static uint8_t motor_canSend( uint16_t cob_id, uint8_t *data, uint8_t dlc );        // 底层统一发送API.

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

    CAN_TxHeaderTypeDef txHeader;
    txHeader.DLC = 8;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = NMT_id;

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



uint8_t Cus_Motor_Steer_Initial( uint8_t Node_id )
{
    /* 检查该设备是否已在状态表中. */
    int8_t index = motor_findSlotByNodeId(Node_id, MOTOR_STEER_TYPE);
    if ( index >= 0 )
    {
        /* 该设备已存在于状态表中. 检查该设备是否因为Error需要重新Initial. */
        if ( StatusSteerArr[index].is_Error == 0 )
        {
            /* 无错误状态. 不允许重复初始化. */
            return (uint8_t)index;
        }
        else { StatusSteerArr[index].is_Error = 0; }   /* 有错误. 重走初始化流程. */
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
        StatusSteerArr[index].statusWord = 0;
        DeviceMask &= ~(1 << index);     // 设置位图占用.
    }

    /* 发送NMT启动命令. */
    motor_NMT_start(Node_id);

    /* 等待状态字. */
    if ( motor_waitStatusWord(Node_id, MOTOR_STEER_TYPE, 0x0050, 500) < 0 ) 
    {
        /* 状态字等待失败. 释放占用，通知钩子. */
        StatusSteerArr[index].nodeId = 0;
        DeviceMask |= (0x01 << index);      // 槽位归还.

        Cus_Motor_ErrorHook(Node_id, MOTOR_STEER_TYPE, MOTOR_ERR_STATUS_WORD);
        return 0xFF;
    }
    
    return 0;
}


void Cus_Motor_MainFunction( void )
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t RxData[8] = { 0 };

    /* 只要缓冲区还有CAN报文就一直取. */
    while( pDev->Receive_IT(pDev, &rxHeader, RxData, FIFO_IDX_0) == HAL_OK )
    {

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


void Cus_Motor_Steer_ModeSelect( uint8_t Node_id, uint8_t Mode )
{
    uint16_t cob_id = RPDO2_BASE + Node_id;

    uint8_t txBuf[8] = { 0 };
    txBuf[0] = Mode;

    motor_canSend(cob_id, txBuf, 8);
}




__WEAK void Cus_Motor_ErrorHook( uint8_t Node_id, uint8_t Type, MotorError_t ErrFlag )
{
    /* 钩子函数默认空实现. 等待用户自行实现. */
    (void)Node_id;
    (void)Type;
    (void)ErrFlag;
}


