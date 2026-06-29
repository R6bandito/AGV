#include "mb.h"

//保持寄存器变量
#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 100
static USHORT usRegHoldingStart = REG_HOLDING_START;
USHORT usRegHoldingBuf[REG_HOLDING_NREGS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode
eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;

    /* Modbus地址从1开始，数组从0开始，地址减1 */
    usAddress--;

    if ((usAddress >= usRegHoldingStart) && (usAddress + usNRegs <= usRegHoldingStart + REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - usRegHoldingStart;
        switch (eMode)
        {
        /* 读保持寄存器 */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* 写保持寄存器 */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

#define REG_COILS_START 0
#define REG_COILS_NCOILS 100
static UCHAR ucCoilBuf[REG_COILS_NCOILS / 8 + 1] = {0}; // 1字节存8个线圈
eMBErrorCode
eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode)
{
 (void)pucRegBuffer;
	
	return MB_ENOREG;
}

// 离散输入配置（只读）
#define REG_DISC_START 0
#define REG_DISC_NINPUTS 100
static UCHAR ucDiscBuf[REG_DISC_NINPUTS / 8 + 1] = {0};
eMBErrorCode
eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
		(void)pucRegBuffer;
    return MB_ENOREG;
}

// 输入寄存器配置（只读）
#define REG_INPUT_START 0
#define REG_INPUT_NREGS 100
static USHORT usRegInputBuf[REG_INPUT_NREGS] = {0};
eMBErrorCode
eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{ 
    (void)pucRegBuffer;
		return MB_ENOREG;
}