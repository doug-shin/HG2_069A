/*
 * Simplified Modbus RTU Header for HG2 System
 * Based on FreeModbus Library
 *
 * Simplified to include only essential features:
 * - RTU mode only
 * - Basic function codes (03, 04, 06, 16)
 * - Essential register handling
 */

#ifndef _MODBUS_H_
#define _MODBUS_H_

#include "DSP28x_Project.h"

//=============================================================================
// Configuration Constants
//=============================================================================

#define CPU_FREQ    (90E6)        // 90.0MHz
#define LSPCLK_FREQ (CPU_FREQ)    // 90MHz
#define SCI_FREQ    (38400)       // BAUD
#define SCI_PRD     (Uint16)((LSPCLK_FREQ/(SCI_FREQ*8))-1)

// SCI-A configuration (for compatibility with HG2_setting.c)
#define SCIA_PRD    (0)           // 5625000 bps

// GPIO control macros
#define En485_ON()  (GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1)
#define Tx485_ON()  (GpioDataRegs.GPBSET.bit.GPIO51 = 1)
#define Rx485_ON()  (GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1)

// Critical section macros (simplified)
#define ENTER_CRITICAL_SECTION()
#define EXIT_CRITICAL_SECTION()

// Register configuration
#define REG_INPUT_START    400
#define REG_INPUT_NREGS    100
#define REG_HOLDING_START  200
#define REG_HOLDING_NREGS  100

// Protocol constants
#define MB_SER_PDU_SIZE_MIN     4
#define MB_SER_PDU_SIZE_MAX     256
#define MB_SER_PDU_SIZE_CRC     2
#define MB_SER_PDU_ADDR_OFF     0
#define MB_SER_PDU_PDU_OFF      1

#define MB_PDU_SIZE_MAX     253
#define MB_PDU_SIZE_MIN     1
#define MB_PDU_FUNC_OFF     0
#define MB_PDU_DATA_OFF     1

// Address limits
#define MB_ADDRESS_BROADCAST    0
#define MB_ADDRESS_MIN          1
#define MB_ADDRESS_MAX          247

// Function codes
#define MB_FUNC_NONE                          0
#define MB_FUNC_READ_HOLDING_REGISTER         3
#define MB_FUNC_READ_INPUT_REGISTER           4
#define MB_FUNC_WRITE_REGISTER                6
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS      16
#define MB_FUNC_ERROR                         128

// PDU offsets for different functions
#define MB_PDU_FUNC_READ_ADDR_OFF           (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_READ_SIZE               4
#define MB_PDU_FUNC_READ_REGCNT_OFF         (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READ_REGCNT_MAX         0x007D

#define MB_PDU_FUNC_WRITE_ADDR_OFF          (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_WRITE_VALUE_OFF         (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE              4

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF      (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF    (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF   (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF    (MB_PDU_DATA_OFF + 5)
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN      5
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX    0x0078

// Configuration
#define MB_FUNC_HANDLERS_MAX                16
#define TABLE_CRC                           1
#define DYNAMIC_CRC                         0

//=============================================================================
// Type Definitions
//=============================================================================

typedef char BOOL;
typedef unsigned char UCHAR;
typedef char CHAR;
typedef unsigned short USHORT;
typedef short SHORT;
typedef unsigned long ULONG;
typedef long LONG;

#ifndef TRUE
#define TRUE            1
#endif
#ifndef FALSE
#define FALSE           0
#endif

// Modbus modes
typedef enum {
    MB_RTU,
    MB_ASCII
} eMBMode;

// Register access mode
typedef enum {
    MB_REG_READ,
    MB_REG_WRITE
} eMBRegisterMode;

// Error codes
typedef enum {
    MB_ENOERR,
    MB_ENOREG,
    MB_EINVAL,
    MB_EPORTERR,
    MB_ENORES,
    MB_EIO,
    MB_EILLSTATE,
    MB_ETIMEDOUT
} eMBErrorCode;

// Event types
typedef enum {
    EV_READY,
    EV_FRAME_RECEIVED,
    EV_EXECUTE,
    EV_FRAME_SENT
} eMBEventType;

// Parity modes
typedef enum {
    MB_PAR_NONE,
    MB_PAR_ODD,
    MB_PAR_EVEN
} eMBParity;

// Exception codes
typedef enum {
    MB_EX_NONE = 0x00,
    MB_EX_ILLEGAL_FUNCTION = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE = 0x05,
    MB_EX_SLAVE_BUSY = 0x06,
    MB_EX_MEMORY_PARITY_ERROR = 0x08,
    MB_EX_GATEWAY_PATH_FAILED = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED = 0x0B
} eMBException;

// RTU state machines
typedef enum {
    STATE_RX_INIT,
    STATE_RX_IDLE,
    STATE_RX_RCV,
    STATE_RX_ERROR
} eMBRcvState;

typedef enum {
    STATE_TX_IDLE,
    STATE_TX_SENT,
    STATE_TX_XMIT
} eMBSndState;

// Function handler type
typedef eMBException (*pxMBFunctionHandler)(UCHAR *pucFrame, USHORT *pusLength);

// Function handler structure
typedef struct {
    UCHAR ucFunctionCode;
    pxMBFunctionHandler pxHandler;
} xMBFunctionHandler;

//=============================================================================
// External Variable Declarations
//=============================================================================

extern USHORT usRegInputBuf[REG_INPUT_NREGS];
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

//=============================================================================
// Function Prototypes
//=============================================================================

// Core Modbus functions
eMBErrorCode eMBInit(eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity);
eMBErrorCode eMBEnable(void);
eMBErrorCode eMBDisable(void);
eMBErrorCode eMBPoll(void);

// Register callback functions
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs);
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode);

// Function code implementations
eMBException eMBFuncReadHoldingRegister(UCHAR *pucFrame, USHORT *usLen);
eMBException eMBFuncReadInputRegister(UCHAR *pucFrame, USHORT *usLen);
eMBException eMBFuncWriteHoldingRegister(UCHAR *pucFrame, USHORT *usLen);
eMBException eMBFuncWriteMultipleHoldingRegister(UCHAR *pucFrame, USHORT *usLen);

// RTU specific functions
eMBErrorCode eMBRTUInit(UCHAR slaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity);
void eMBRTUStart(void);
void eMBRTUStop(void);
eMBErrorCode eMBRTUReceive(UCHAR *pucRcvAddress, UCHAR **pucFrame, USHORT *pusLength);
eMBErrorCode eMBRTUSend(UCHAR slaveAddress, const UCHAR *pucFrame, USHORT usLength);
BOOL xMBRTUReceiveFSM(void);
BOOL xMBRTUTransmitFSM(void);
BOOL xMBRTUTimerT35Expired(void);

// Hardware abstraction layer
BOOL xMBPortSerialInit(UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);
void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable);
BOOL xMBPortSerialGetByte(CHAR *pucByte);
BOOL xMBPortSerialPutByte(CHAR ucByte);
BOOL xMBPortTimersInit(USHORT usTimerOutT35);

// Event handling
BOOL xMBPortEventInit(void);
BOOL xMBPortEventPost(eMBEventType eEvent);
BOOL xMBPortEventGet(eMBEventType *eEvent);

// Interrupt service routines
__interrupt void scibRxReadyISR(void);
__interrupt void scibTxEmptyISR(void);
__interrupt void cpuTimer1ExpiredISR(void);

// Utility functions
eMBException prveMBError2Exception(eMBErrorCode eErrorCode);
USHORT usMBCRC16(UCHAR *pucFrame, USHORT usLen);

#endif // _MODBUS_H_
