/*
 * Simplified Modbus RTU Implementation for HG2 System
 * Based on FreeModbus Library - Minimized for essential features only
 */

#include "string.h"
#include "modbus.h"

// Timer control macros
#define vMBPortTimersEnable() \
    CpuTimer1Regs.TCR.all = 0xC420

#define vMBPortTimersDisable() \
    CpuTimer1Regs.TCR.all = 0x0430

// Static variables
static UCHAR ucMBAddress;
static eMBEventType eQueuedEvent;
static BOOL xEventInQueue;

// Register buffers
USHORT usRegInputStart = REG_INPUT_START;
USHORT usRegInputBuf[REG_INPUT_NREGS];
USHORT usRegHoldingStart = REG_HOLDING_START;
USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

// State machines
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

// Communication buffers
static volatile UCHAR ucRTUBuf[MB_SER_PDU_SIZE_MAX];
static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;
static volatile USHORT usRcvBufferPos;

// System state
enum {
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

// Function handlers
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
    {0, NULL}
};

// Function pointers
static BOOL (*pxMBFrameCBByteReceived)(void);
static BOOL (*pxMBFrameCBTransmitterEmpty)(void);
static BOOL (*pxMBPortCBTimerExpired)(void);

// CRC table
#if TABLE_CRC > 0
const UCHAR aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

const UCHAR aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};
#endif

//=============================================================================
// Hardware abstraction layer
//=============================================================================

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
    // Suppress unused parameter warnings
    (void)ucPORT;
    (void)ulBaudRate;
    (void)ucDataBits;
    (void)eParity;

    ScibRegs.SCIFFTX.all = 0xC040;
    ScibRegs.SCIFFRX.all = 0x4041;
    ScibRegs.SCIFFCT.all = 0x0;

    ScibRegs.SCICCR.all = 0x0007;
    ScibRegs.SCICTL1.all = 0x0043;

    ScibRegs.SCIHBAUD = SCI_PRD >> 8;
    ScibRegs.SCILBAUD = SCI_PRD & 0xFF;

    ScibRegs.SCICTL1.all = 0x0063;

    vMBPortSerialEnable(FALSE, FALSE);

    return TRUE;
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    if (xRxEnable == TRUE) {
        ScibRegs.SCIFFRX.bit.RXFFIENA = 1;
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
        ScibRegs.SCIFFRX.bit.RXFFIENA = 0;
    }

    if (xTxEnable == TRUE) {
        ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
        ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    } else {
        ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 0;
        ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
    }
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    ScibRegs.SCITXBUF = ucByte;
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
    *pucByte = (UCHAR)ScibRegs.SCIRXBUF.all;
    return TRUE;
}

BOOL xMBPortTimersInit(USHORT usTimerOutT35)
{
    ConfigCpuTimer(&CpuTimer1, 90, 90UL * usTimerOutT35);
    return TRUE;
}

//=============================================================================
// Interrupt service routines
//=============================================================================

__interrupt void scibTxEmptyISR(void)
{
    EINT;
    pxMBFrameCBTransmitterEmpty();
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

__interrupt void scibRxReadyISR(void)
{
    EINT;

    if (ScibRegs.SCIRXST.bit.RXERROR == 1) {
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
    } else {
        pxMBFrameCBByteReceived();
    }

    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

extern Uint16 cpu_timer1_cnt;
__interrupt void cpuTimer1ExpiredISR(void)
{
    EINT;
    pxMBPortCBTimerExpired();
}

//=============================================================================
// Event handling
//=============================================================================

BOOL xMBPortEventInit(void)
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
    xEventInQueue = TRUE;
    eQueuedEvent = eEvent;
    return TRUE;
}

BOOL xMBPortEventGet(eMBEventType *eEvent)
{
    BOOL xEventHappened = FALSE;

    if (xEventInQueue) {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

//=============================================================================
// Core Modbus functions
//=============================================================================

eMBErrorCode eMBInit(eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity)
{
    eMBErrorCode eStatus = MB_ENOERR;

    if ((ucSlaveAddress == MB_ADDRESS_BROADCAST) ||
        (ucSlaveAddress < MB_ADDRESS_MIN) || (ucSlaveAddress > MB_ADDRESS_MAX)) {
        eStatus = MB_EINVAL;
    } else {
        ucMBAddress = ucSlaveAddress;

        if (eMode == MB_RTU) {
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;

            eStatus = eMBRTUInit(ucMBAddress, ucPort, ulBaudRate, eParity);
        } else {
            eStatus = MB_EINVAL;
        }

        if (eStatus == MB_ENOERR) {
            if (!xMBPortEventInit()) {
                eStatus = MB_EPORTERR;
            } else {
                eMBState = STATE_DISABLED;
            }
        }
    }
    return eStatus;
}

eMBErrorCode eMBEnable(void)
{
    eMBErrorCode eStatus = MB_ENOERR;

    if (eMBState == STATE_DISABLED) {
        eMBRTUStart();
        eMBState = STATE_ENABLED;
    } else {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBDisable(void)
{
    eMBErrorCode eStatus;

    if (eMBState == STATE_ENABLED) {
        eMBRTUStop();
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    } else if (eMBState == STATE_DISABLED) {
        eStatus = MB_ENOERR;
    } else {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBPoll(void)
{
    static UCHAR *ucMBFrame;
    static UCHAR ucRcvAddress;
    static UCHAR ucFunctionCode;
    static USHORT usLength;
    static eMBException eException;

    int i;
    eMBErrorCode eStatus = MB_ENOERR;
    eMBEventType eEvent;

    if (eMBState != STATE_ENABLED) {
        return MB_EILLSTATE;
    }

    if (xMBPortEventGet(&eEvent) == TRUE) {
        switch (eEvent) {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            eStatus = eMBRTUReceive(&ucRcvAddress, &ucMBFrame, &usLength);
            if (eStatus == MB_ENOERR) {
                if ((ucRcvAddress == ucMBAddress) || (ucRcvAddress == MB_ADDRESS_BROADCAST)) {
                    (void)xMBPortEventPost(EV_EXECUTE);
                    cpu_timer1_cnt++;
                }
            }
            break;

        case EV_EXECUTE:
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;

            for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++) {
                if (xFuncHandlers[i].ucFunctionCode == 0) {
                    break;
                } else if (xFuncHandlers[i].ucFunctionCode == ucFunctionCode) {
                    eException = xFuncHandlers[i].pxHandler(ucMBFrame, &usLength);
                    break;
                }
            }

            if (ucRcvAddress != MB_ADDRESS_BROADCAST) {
                if (eException != MB_EX_NONE) {
                    usLength = 0;
                    ucMBFrame[usLength++] = (UCHAR)(ucFunctionCode | MB_FUNC_ERROR);
                    ucMBFrame[usLength++] = eException;
                }
                eStatus = eMBRTUSend(ucMBAddress, ucMBFrame, usLength);
            }
            break;

        case EV_FRAME_SENT:
            eSndState = STATE_TX_IDLE;
            break;
        }
    }

    return MB_ENOERR;
}

//=============================================================================
// RTU functions
//=============================================================================

eMBErrorCode eMBRTUInit(UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity)
{
    // Suppress unused parameter warning
    (void)ucSlaveAddress;

    eMBErrorCode eStatus = MB_ENOERR;
    ULONG usTimerT35_50us;

    ENTER_CRITICAL_SECTION();

    if (xMBPortSerialInit(ucPort, ulBaudRate, 8, eParity) != TRUE) {
        eStatus = MB_EPORTERR;
    } else {
        if (ulBaudRate > 19200) {
            usTimerT35_50us = 35;
        } else {
            usTimerT35_50us = (7UL * 220000UL) / (2UL * ulBaudRate);
        }
        if (xMBPortTimersInit((USHORT)usTimerT35_50us) != TRUE) {
            eStatus = MB_EPORTERR;
        }
    }
    EXIT_CRITICAL_SECTION();

    return eStatus;
}

void eMBRTUStart(void)
{
    ENTER_CRITICAL_SECTION();
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable(TRUE, FALSE);
    vMBPortTimersEnable();
    EXIT_CRITICAL_SECTION();
}

void eMBRTUStop(void)
{
    ENTER_CRITICAL_SECTION();
    vMBPortSerialEnable(FALSE, FALSE);
    vMBPortTimersDisable();
    EXIT_CRITICAL_SECTION();
}

eMBErrorCode eMBRTUReceive(UCHAR *pucRcvAddress, UCHAR **pucFrame, USHORT *pusLength)
{
    eMBErrorCode eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION();

    if ((usRcvBufferPos >= MB_SER_PDU_SIZE_MIN) && (usMBCRC16((UCHAR *)ucRTUBuf, usRcvBufferPos) == 0)) {
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];
        *pusLength = (USHORT)(usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC);
        *pucFrame = (UCHAR *)&ucRTUBuf[MB_SER_PDU_PDU_OFF];
    } else {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION();
    return eStatus;
}

eMBErrorCode eMBRTUSend(UCHAR ucSlaveAddress, const UCHAR *pucFrame, USHORT usLength)
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT usCRC16;

    ENTER_CRITICAL_SECTION();

    if (eRcvState == STATE_RX_IDLE) {
        pucSndBufferCur = (UCHAR *)pucFrame - 1;
        usSndBufferCount = 1;

        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        usCRC16 = usMBCRC16((UCHAR *)pucSndBufferCur, usSndBufferCount);
        ucRTUBuf[usSndBufferCount++] = (UCHAR)(usCRC16 & 0xFF);
        ucRTUBuf[usSndBufferCount++] = (UCHAR)(usCRC16 >> 8);

        eSndState = STATE_TX_XMIT;
        vMBPortSerialEnable(FALSE, TRUE);
    } else {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION();
    return eStatus;
}

BOOL xMBRTUReceiveFSM(void)
{
    BOOL xTaskNeedSwitch = FALSE;
    UCHAR ucByte;

    (void)xMBPortSerialGetByte((CHAR *)&ucByte);

    switch (eRcvState) {
    case STATE_RX_INIT:
        vMBPortTimersEnable();
        break;

    case STATE_RX_ERROR:
        vMBPortTimersEnable();
        break;

    case STATE_RX_IDLE:
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;
        vMBPortTimersEnable();
        break;

    case STATE_RX_RCV:
        if (usRcvBufferPos < MB_SER_PDU_SIZE_MAX) {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
        } else {
            eRcvState = STATE_RX_ERROR;
        }
        vMBPortTimersEnable();
        break;
    }

    return xTaskNeedSwitch;
}

BOOL xMBRTUTransmitFSM(void)
{
    BOOL xNeedPoll = FALSE;

    switch (eSndState) {
    case STATE_TX_IDLE:
        vMBPortSerialEnable(TRUE, FALSE);
        break;

    case STATE_TX_XMIT:
        if (usSndBufferCount != 0) {
            xMBPortSerialPutByte((CHAR)*pucSndBufferCur);
            pucSndBufferCur++;
            usSndBufferCount--;
        } else {
            vMBPortSerialEnable(TRUE, FALSE);
            eSndState = STATE_TX_SENT;
            vMBPortTimersEnable();
        }
        break;
    }

    return xNeedPoll;
}

BOOL xMBRTUTimerT35Expired(void)
{
    BOOL xNeedPoll = FALSE;

    switch (eRcvState) {
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost(EV_READY);
        break;

    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost(EV_FRAME_RECEIVED);
        break;

    case STATE_RX_ERROR:
        break;

    case STATE_RX_IDLE:
        xNeedPoll = xMBPortEventPost(EV_FRAME_SENT);
        break;
    }

    vMBPortTimersDisable();
    eRcvState = STATE_RX_IDLE;

    return xNeedPoll;
}

//=============================================================================
// Register callbacks
//=============================================================================

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_INPUT_START) &&
        (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS)) {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while (usNRegs > 0) {
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    } else {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_HOLDING_START) &&
        (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
        iRegIndex = (int)(usAddress - usRegHoldingStart);
        switch (eMode) {
        case MB_REG_READ:
            while (usNRegs > 0) {
                *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while (usNRegs > 0) {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

//=============================================================================
// Function code implementations
//=============================================================================

eMBException eMBFuncReadHoldingRegister(UCHAR *pucFrame, USHORT *usLen)
{
    USHORT usRegAddress;
    USHORT usRegCount;
    UCHAR *pucFrameCur;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

    if (*usLen == (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (USHORT)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
        usRegAddress |= (USHORT)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);

        usRegCount = (USHORT)(pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
        usRegCount |= (USHORT)(pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);

        if ((usRegCount >= 1) && (usRegCount <= MB_PDU_FUNC_READ_REGCNT_MAX)) {
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            *pucFrameCur++ = MB_FUNC_READ_HOLDING_REGISTER;
            *usLen += 1;

            *pucFrameCur++ = (UCHAR)(usRegCount * 2);
            *usLen += 1;

            eRegStatus = eMBRegHoldingCB(pucFrameCur, usRegAddress, usRegCount, MB_REG_READ);
            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            } else {
                *usLen += usRegCount * 2;
            }
        } else {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

eMBException eMBFuncReadInputRegister(UCHAR *pucFrame, USHORT *usLen)
{
    USHORT usRegAddress;
    USHORT usRegCount;
    UCHAR *pucFrameCur;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

    if (*usLen == (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (USHORT)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
        usRegAddress |= (USHORT)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);

        usRegCount = (USHORT)(pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
        usRegCount |= (USHORT)(pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);

        if ((usRegCount >= 1) && (usRegCount < MB_PDU_FUNC_READ_REGCNT_MAX)) {
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            *pucFrameCur++ = MB_FUNC_READ_INPUT_REGISTER;
            *usLen += 1;

            *pucFrameCur++ = (UCHAR)(usRegCount * 2);
            *usLen += 1;

            eRegStatus = eMBRegInputCB(pucFrameCur, usRegAddress, usRegCount);
            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            } else {
                *usLen += usRegCount * 2;
            }
        } else {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

eMBException eMBFuncWriteHoldingRegister(UCHAR *pucFrame, USHORT *usLen)
{
    USHORT usRegAddress;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

    if (*usLen == (MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN)) {
        usRegAddress = (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
        usRegAddress |= (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);

        eRegStatus = eMBRegHoldingCB(&pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF],
                                     usRegAddress, 1, MB_REG_WRITE);

        if (eRegStatus != MB_ENOERR) {
            eStatus = prveMBError2Exception(eRegStatus);
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

eMBException eMBFuncWriteMultipleHoldingRegister(UCHAR *pucFrame, USHORT *usLen)
{
    USHORT usRegAddress;
    USHORT usRegCount;
    UCHAR ucRegByteCount;
    eMBException eStatus = MB_EX_NONE;
    eMBErrorCode eRegStatus;

    if (*usLen >= (MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN)) {
        usRegAddress = (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
        usRegAddress |= (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);

        usRegCount = (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8);
        usRegCount |= (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1]);

        ucRegByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        if ((usRegCount >= 1) &&
            (usRegCount <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX) &&
            (ucRegByteCount == (UCHAR)(2 * usRegCount))) {

            eRegStatus = eMBRegHoldingCB(&pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                                         usRegAddress, usRegCount, MB_REG_WRITE);

            if (eRegStatus != MB_ENOERR) {
                eStatus = prveMBError2Exception(eRegStatus);
            } else {
                *usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
        } else {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

//=============================================================================
// Utility functions
//=============================================================================

eMBException prveMBError2Exception(eMBErrorCode eErrorCode)
{
    eMBException eStatus;

    switch (eErrorCode) {
    case MB_ENOERR:
        eStatus = MB_EX_NONE;
        break;
    case MB_ENOREG:
        eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
        break;
    case MB_ETIMEDOUT:
        eStatus = MB_EX_SLAVE_BUSY;
        break;
    default:
        eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
        break;
    }
    return eStatus;
}

USHORT usMBCRC16(UCHAR *pucFrame, USHORT usLen)
{
#if TABLE_CRC > 0
    UCHAR ucCRCHi = 0xFF;
    UCHAR ucCRCLo = 0xFF;
    int iIndex;

    while (usLen--) {
        iIndex = ucCRCLo ^ *(pucFrame++);
        ucCRCLo = (UCHAR)(ucCRCHi ^ aucCRCHi[iIndex]);
        ucCRCHi = aucCRCLo[iIndex];
    }
    return (USHORT)(ucCRCHi << 8 | ucCRCLo);
#else
    USHORT usCRC = 0xFFFF, poly = 0xA001;
    UCHAR i, j, *byte_point;
    for (i = 0, byte_point = pucFrame; i < usLen; ++i, ++byte_point) {
        usCRC = usCRC ^ *byte_point;
        for (j = 0; j < 8; ++j) {
            if (usCRC & 0x01) {
                usCRC = (usCRC >> 1) ^ poly;
            } else {
                usCRC = (usCRC >> 1);
            }
        }
    }
    return usCRC;
#endif
}
