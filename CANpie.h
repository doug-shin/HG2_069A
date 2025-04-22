//****************************************************************************//
// File:          CANpie.h                                                    //
// Description:   Compiler independent data types for embedded solutions      //
//                Definitions for CAN controller chips / targets              //
//                CANpie architecture definitions                             //
//                General CAN driver definitions and structures               //
//                CANpie core functions                                       //
//                CANpie message access functions                             //
//                CANpie network functions                                    //
//                FIFO for CAN messages                                       //
// Author:        Uwe Koppe                                                   //
// e-mail:        koppe@microcontrol.net                                      //
//                                                                            //
// Copyright (C) MicroControl GmbH & Co. KG                                   //
// 53842 Troisdorf - Germany                                                  //
// www.microcontrol.net                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
// Redistribution and use in source and binary forms, with or without         //
// modification, are permitted provided that the following conditions         //
// are met:                                                                   //
// 1. Redistributions of source code must retain the above copyright          //
//    notice, this list of conditions, the following disclaimer and           //
//    the referenced file 'COPYING'.                                          //
// 2. Redistributions in binary form must reproduce the above copyright       //
//    notice, this list of conditions and the following disclaimer in the     //
//    documentation and/or other materials provided with the distribution.    //
// 3. Neither the name of MicroControl nor the names of its contributors      //
//    may be used to endorse or promote products derived from this software   //
//    without specific prior written permission.                              //
//                                                                            //
// Provided that this notice is retained in full, this software may be        //
// distributed under the terms of the GNU Lesser General Public License       //
// ("LGPL") version 3 as distributed in the 'COPYING' file.                   //
//                                                                            //
//----------------------------------------------------------------------------//
//                                                                            //
// Date        History                                                        //
// ----------  -------------------------------------------------------------- //
// 04.12.1998  Initial version                                                //
// 04.29.1999  changed structures, new data type definitions                  //
// 06.15.2000  moved definitions from cpmsg.h                                 //
// 06.15.2000  added fixed FIFO size support                                  //
// 11.06.2000  added new error codes, added missing buffer number             //
// 09.07.2000  new symbol for const/code                                      //
// 02.14.2001  complete rewrite, check FIFO status                            //
// 06.12.2007  update HDI and statistic structure                             //
// 06.12.2007  rename file, change FIFO size to 32 bit                        //
// 12.18.2008  use 'typedef' instead of '#define', introduce MC_COMPILER      //
// 05.20.2010  collected and revised                                          //
//                                                                            //
//****************************************************************************//


#ifndef  _CANPIE_H_
#define  _CANPIE_H_


//-----------------------------------------------------------------------------
/*!
** \file    compiler.h
** \brief   Definition of compiler independent data types
**
** Due to different implementations of data types in the world of C compilers,
** the following data types are used for CANpie API:
** \li _BIT :   Boolean value, True or False
** \li _U08 :   1 Byte value, value range 0 .. 2^8 - 1 (0 .. 255)
** \li _S08 :   1 Byte value, value range -2^7 .. 2^7 - 1 (-128 .. 127)
** \li _U16 :   2 Byte value, value range 0 .. 2^16 - 1 (0 .. 65535)
** \li _S16 :   2 Byte value, value range -2^15 .. 2^15 - 1
** \li _U32 :   4 Byte value, value range 0 .. 2^32 - 1
** \li _S32 :   4 Byte value, value range -2^31 .. 2^31 - 1
**
** The compiler type/version is evaluated via its internal predefined
** symbols. If your compiler is not (yet) supported, you will get the
** following error message upon compilation:
** <p>
** <b>Data types are not defined! Please check compiler definition.</b>
** <p>
*/


//-----------------------------------------------------------------------------
/*!   
** \file    cp_cc.h
** \brief   CANpie constant values for targets
**
** This file defines values for the available silicon with CAN
** functionality. Each CAN controller (silicon) has its unique
** number, as shown in the following snippet:
**
** \code
** //--- CANary, AT89C51CC01 -------------
** #define CP_CC_CC01            0x1000
**
** //--- CANary, AT89C51CC02 -------------
** #define CP_CC_CC02            0x1001
**
** //--- CANary, AT89C51CC03 -------------
** #define CP_CC_CC03            0x1002
** 
** \endcode
**
** The number must be assigned to the symbol CP_TARGET for the
** compilation process. This can be done by setting it in the
** file cp_cc.h (at the end) or within the Makefile / compiler
** setup. If you forget to set the symbol CP_TARGET, you will
** get the following error message: <br>
** 
** <b>Target (Symbol CP_TARGET) is not defined! Check file cp_cc.h!</b>
*/


//-----------------------------------------------------------------------------
/*!   \file    cp_arch.h
**    \brief   CANpie architecture definitions
**
**
*/


//-----------------------------------------------------------------------------
/*!
** \file    CANpie.h
** \brief   CANpie constants, structures and enumerations
**
** This file holds constants and structures used within CANpie.
** All functions, structures, defines and constants in CANpie have
** the prefix "Cp". The following table shows the used nomenclature:
**
** <table border=0>
** <tr class="memlist">
** <td width=200 class="memItemLeft"> <b>CANpie code</b></td>
** <td width=200 class="memItemRight"><b>Prefix</b></td>
** </tr>
** <tr>
** <td width=200 class="memItemLeft">Core functions</td>
** <td width=200 class="memItemRight">CpCore</td>
** </tr>
** <tr>
** <td width=200 class="memItemLeft">Message access functions</td>
** <td width=200 class="memItemRight">CpMsg</td>
** </tr>
** <tr>
** <td width=200 class="memItemLeft">Structures</td>
** <td width=200 class="memItemRight">_TsCp</td>
** </tr>
** <tr>
** <td width=200 class="memItemLeft">Constants / Defines</td>
** <td width=200 class="memItemRight">CP</td>
** </tr>
** <tr>
** <td width=200 class="memItemLeft">Error Codes</td>
** <td width=200 class="memItemRight">CpErr</td>
** </tr>
** </table>
*/


//-----------------------------------------------------------------------------
/*!
** \file    cp_core.h
** \brief   CANpie core functions
**
** The core functions provide the direct interface to the CAN controller
** (hardware). Please note that due to hardware limitations maybe certain
** functions are not implemented on the target platform. A call to an
** unsupported function will always return the error code
** #CpErr_NOT_SUPPORTED.
** <p>
** For a "FullCAN" controller (i.e. a CAN controller that has several message
** buffers) an extended set of powerful functions (CpCoreBuffer..())is provided.
** <p>
** <h3>Initialisation Process</h3>
** <p>
** The CAN driver is initialised with the function CpCoreDriverInit(). This
** routine will setup the CAN controller, but not configure a certain bitrate
** nor switch the CAN controller to active operation. The following core
** functions must be called in that order:
** \li CpCoreDriverInit()
** \li CpCoreBaudrate() / CpCoreBittiming()
** \li CpCoreCanMode()
**
** The function CpCoreDriverInit() must be called before any other core
** function in order to have a valid handle / pointer to the open CAN interface.
**
** \b Example
** \code
** void MyCanInit(void)
** {
**    _TsCpPort tsCanPortT; // logical CAN port
**    //---------------------------------------------------
**    // setup the CAN controller / open a physical CAN port
**    //
**    CpCoreDriverInit(CP_CHANNEL_1, &tsCanPortT);
**    //---------------------------------------------------
**    // setup 500 kBit/s
**    //
**    CpCoreBaudrate(&tsCanPortT, CP_BAUD_500K);
**    //---------------------------------------------------
**    // start CAN operation
**    //
**    CpCoreCanMode(&tsCanPortT, CP_MODE_START);
**    //.. now we are operational
** }
** \endcode
**
** \code
** void SetCustomBaudrate(void)
** {
**    _TsCpPort tsCanPortT; // logical CAN port
**    _TsCpBitTiming tsBitTimeT;
**    //---------------------------------------------------
**    // setup the CAN controller / open a physical CAN port
**    //
**    CpCoreDriverInit(CP_CHANNEL_1, &tsCanPortT);
**    //---------------------------------------------------
**    // setup Btr0 and Btr1 with user defined values
**    //
**    tsBitTimeT.ubBtr0 = 0x3F;
**    tsBitTimeT.ubBtr1 = 0x1C;
**    //
**    CpCoreBittiming(&tsCanPortT, &tsBitTimeT);
**    //.. now we have a new baudrate setting
** }
** \endcode
*/


//-----------------------------------------------------------------------------
/*!
** \file    cp_msg.h
** \brief   CANpie message access
**
** &nbsp;<p>
** In order to create small and fast code, CANpie supplies a set of
** macros to access the CAN message structure (#_TsCpCanMsg). These
** macros can be used instead of the functions defined in the cp_msg.h
** header file. However keep in mind that macros can't be used to check
** for value ranges or parameter consistence. Usage of macros is enabled
** via the symbol #CP_CAN_MSG_MACRO.<p>
** \b Example
** \code
** //--- setup a CAN message ----------------------------------------
** _TsCpCanMsg   myMessage;
** CpMsgClear(&myMessage);                // clear the message
** CpMsgSetStdId(&myMessage, 100, 0);     // identifier is 100 dec, no RTR
** CpMsgSetDlc(&myMessage, 2);            // data length code is 2
** CpMsgSetData(&myMessage, 0, 0x11);     // byte 0 has the value 0x11
** CpMsgSetData(&myMessage, 1, 0x22);     // byte 1 has the value 0x22
** //... do something with it ....
**
** //--- evaluate a message that was received -----------------------
** _TsCpCanMsg   receiveMsg;
** //... receive the stuff ....
**
** if(CpMsgIsExtended(&receiveMsg))
** {
**    //--- this is an Extended Frame ---------------------
**    DoExtendedMessageService();
**    return;
** }
**
** if(CpMsgIsRemote(&receiveMsg))
** {
**    //... do something with RTR frames
** }
** \endcode
*/


//-----------------------------------------------------------------------------
/*!   \file    cp_fifo.h
**    \brief   CANpie FIFO functions
**
**    The FIFO functions care for the handling of messages between the user
**    interface (user functions) and the specific hardware implementation.
**    The are currently two implementations for the FIFO: one with dynamic
**    memory allocation (using malloc() and free() functions) and a second
**    with static memory size. The latter is typically used for microcontroller
**    implementations with low memory resources.
*/


//-----------------------------------------------------------------------------
/*!
** \file    cp_net.h
** \brief   CANpie network functions
**
** The network functions provide the */


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*\
** Definitions & Enumerations                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/


#ifndef  FALSE
#define  FALSE  0
#endif

#ifndef  TRUE
#define  TRUE   1
#endif


//----------------------------------------------------------------------------//
// Target Machine / Compiler dependant definitions                            //
//                                                                            //
//----------------------------------------------------------------------------//

//-------------------------------------------------------//
// Texas Instruments                         (0x48xx)    //
//-------------------------------------------------------//

#define CP_CC_TMS320F28xx     0x4800


/*---------------------------------------------------------
** TI Code Composer
**
*/
#ifdef __TMS320C2000__

#define  MC_COMPILER          1
#define  _CON                 const

typedef  unsigned char        _BIT;
typedef  unsigned char        _U08;
typedef  signed char          _S08;
typedef  unsigned int         _U16;
typedef  int                  _S16;
typedef  unsigned long        _U32;
typedef  long                 _S32;

#endif
/* End of definition:  __TMS320C2000__
**-------------------------------------------------------*/


//----------------------------------------------------------------------------//
//                                                                            //
// CP_TARGET symbol defines target controller for the CANpie driver.          //
// The value can be set in a Makefile or in the cp_cc.h file.                 //
//                                                                            //
//----------------------------------------------------------------------------//
#ifndef  CP_TARGET
#define  CP_TARGET   CP_CC_TMS320F28xx
#endif


//---------------------------------------------------------------------
// Architecture definitions for Texas Instruments TMS320F28xx
//
#if CP_TARGET == CP_CC_TMS320F28xx

#define CP_BUFFER_MAX            32
#define CP_CAN_MSG_MACRO         1
#define CP_SMALL_CODE            0
#define CP_STATISTIC             1

#define CP_GLOBAL_RCV_ENABLE     0
#define CP_GLOBAL_RCV_BUFFER     31
#define CP_GLOBAL_RCV_MASK       0x00000000
#endif


//-------------------------------------------------------------------
// The symbol CP_TARGET defines the target for the CANpie sources.
//
#ifndef  CP_TARGET
#error   Target (Symbol CP_TARGET) is not defined! Check file cp_cc.h!
#endif


//-------------------------------------------------------------------
// The symbol MC_COMPILER defines supported compiler platform.
//
#ifndef  MC_COMPILER
#error   Data types are not defined! Please check compiler definition.
#endif


//-----------------------------------------------------------------------------
/*!
** \defgroup CP_CONF  CANpie configuration options
**
** The CANpie driver can be configured during compile time via
** several configuration options. They are typically defined in
** the \c cp_arch.h architecture file. The symbol #CP_TARGET is used
** to select an existing definition scheme from this file.
** <p>
** If symbols are not defined, they get a default value which is
** assigned in the \c canpie.h header file.
*/

/*-------------------------------------------------------------------*/
/*!
** \def  CP_BUFFER_MAX
** \ingroup CP_CONF
**
** This symbol defines the number of message buffers (mailboxes)
** of a CAN controller. In case the controller has no message buffers,
** it is also possible to emulate these. A value of 0 denotes that
** there are no message buffers available. This also means all buffer
** functions (e.g. CpCoreBufferInit(), etc.) return the error code
** #CpErr_NOT_SUPPORTED.
*/
#ifndef  CP_BUFFER_MAX
#define  CP_BUFFER_MAX              0
#endif

/*-------------------------------------------------------------------*/
/*!
** \def  CP_CAN_MSG_MACRO
** \ingroup CP_CONF
**
** This symbol defines if access to the CAN message structure
** CpCanMsg_s is done via macros or via functions.
** - 0 = access via functions
** - 1 = access via macros
*/
#ifndef  CP_CAN_MSG_MACRO
#define  CP_CAN_MSG_MACRO           0
#endif

/*-------------------------------------------------------------------*/
/*!
** \def  CP_CAN_MSG_TIME
** \ingroup CP_CONF
**
** This symbol defines if the CAN message structure CpCanMsg_s
** has a timestamp field.
** - 0 = no timestamp field (not supported by hardware / driver)
** - 1 = include timestamp field
*/
#ifndef  CP_CAN_MSG_TIME
#define  CP_CAN_MSG_TIME            1
#endif

/*-------------------------------------------------------------------*/
/*!
** \def  CP_CAN_MSG_USER
** \ingroup CP_CONF
**
** This symbol defines if the CAN message structure CpCanMsg_s
** has a user-defined field.
** - 0 = no user-defined field (not supported by driver)
** - 1 = include user-defined field
*/
#ifndef  CP_CAN_MSG_USER
#define  CP_CAN_MSG_USER            1
#endif

/*-------------------------------------------------------------------*/
/*!
** \def  CP_CHANNEL_MAX
** \ingroup CP_CONF
**
** This symbol defines the total number of physical CAN interfaces
** supported by the driver. For a LPC2294 microcontroller the value
** would be 4 (4 CAN channels).
*/
#ifndef  CP_CHANNEL_MAX
#define  CP_CHANNEL_MAX             1
#endif

/*-------------------------------------------------------------------*/
/*!
** \def  CP_SMALL_CODE
** \ingroup CP_CONF
**
** This symbol is used to control the usage of the \c ptsPortV
** parameter in the core functions during compilation time. For
** microcontrollers with small resources and only one CAN channel
** the port parameter can be omitted.
** - 0 = use \c ptsPortV parameter in core functions
** - 1 = do not use \c ptsPortV parameter in core functions
*/
#ifndef  CP_SMALL_CODE
#define  CP_SMALL_CODE              0
#endif

/*-------------------------------------------------------------------*/
/*!
** \def  CP_STATISTIC
** \ingroup CP_CONF
**
** This symbol defines if the driver support statistic information.
** A value of 0 denotes that no statistic information is available.
** This also means that the function CpCoreStatistic() returns the
** error code #CpErr_NOT_SUPPORTED.
** - 0 = no statistic information (not supported by driver)
** - 1 = enable statistic information
*/
#ifndef  CP_STATISTIC
#define  CP_STATISTIC               0
#endif

#define  CP_VERSION_MAJOR           1
#define  CP_VERSION_MINOR           94


//-----------------------------------------------------------------------------
/*!
** \defgroup CP_MASK Mask values for CAN messages
**
** The following definitions are used in combination with the
** structure CpCanMsg_s.
*/

/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_STD_FRAME
** \ingroup CP_MASK
**
** Mask for standard frame (11 bits), used in combination with
** the CpCanMsg_s::ulExt.
*/
#define  CP_MASK_STD_FRAME 0x000007FF

/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_EXT_FRAME
** \ingroup CP_MASK
**
** Mask for extended frame (29 bits), used in combination with
** the CpCanMsg_s::ulExt.
*/
#define  CP_MASK_EXT_FRAME 0x1FFFFFFF

/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_EXT_BIT
** \ingroup CP_MASK
**
** Set the EXT bit (extended frame) in the \c ubMsgCtrl field of
** the _TsCpCanMsg structure (CpCanMsg_s::ubMsgCtrl).
*/
#define  CP_MASK_EXT_BIT   0x01

/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_RTR_BIT
** \ingroup CP_MASK
**
** Set the RTR bit (remote frame) in the ubMsgCtrl field of
** the _TsCpCanMsg structure (CpCanMsg_s::ubMsgCtrl).
*/
#define  CP_MASK_RTR_BIT   0x02

/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_OVR_BIT
** \ingroup CP_MASK
**
** Set the OVR bit (overrun) in the ubMsgCtrl field of
** the _TsCpCanMsg structure (CpCanMsg_s::ubMsgCtrl).
*/
#define  CP_MASK_OVR_BIT   0x04


/*-------------------------------------------------------------------------
** A driver with only one channel and small memory resources does not need
** the 'channel' parameter.
** The definition CP_SMALL_CODE is checked for the value '1' and the
** function prototypes are converted then. The function call in the
** application stays the same (with 'channel' parameter).
**
*/
#if   CP_SMALL_CODE == 1

#define  CpCoreAutobaud(CH, A, B)            CpCoreAutobaud(A, B)
#define  CpCoreBaudrate(CH, A)               CpCoreBaudrate(A)
#define  CpCoreBittiming(CH, A)              CpCoreBittiming(A)

#define  CpCoreBufferGetData(CH, A, B)       CpCoreBufferGetData(A, B)
#define  CpCoreBufferGetDlc(CH, A, B)        CpCoreBufferGetDlc(A, B)
#define  CpCoreBufferInit(CH, A, B, C)       CpCoreBufferInit(A, B, C)
#define  CpCoreBufferRelease(CH, A)          CpCoreBufferRelease(A)
#define  CpCoreBufferSetData(CH, A, B)       CpCoreBufferSetData(A, B)
#define  CpCoreBufferSetDlc(CH, A, B)        CpCoreBufferSetDlc(A, B)
#define  CpCoreBufferSend(CH, A)             CpCoreBufferSend(A)

#define  CpCoreCanMode(CH, A)                CpCoreCanMode(A)
#define  CpCoreCanState(CH, A)               CpCoreCanState(A)

#define  CpCoreFilterAll(CH, A)              CpCoreFilterAll(A)
#define  CpCoreFilterMsg(CH, A, B)           CpCoreFilterMsg(A, B)

#define  CpCoreHDI(CH, A)                    CpCoreHDI(A)

#define  CpCoreIntFunctions(CH, A, B, C)     CpCoreIntFunctions(A, B, C)

#define  CpCoreMsgRead(CH, A, B)             CpCoreMsgRead(A, B)
#define  CpCoreMsgWrite(CH, A, B)            CpCoreMsgWrite(A, B)

#define  CpCoreStatistic(CH, A)              CpCoreStatistic(A)

#endif


//-------------------------------------------------------------------
// When the option CP_SMALL_CODE is set, the following function
// has no parameters. Inside the header file it must have the
// parameter type *void* then. The function is re-defined after-
// wards!
//
#if   CP_SMALL_CODE == 1
#define  CpCoreDriverInit(A, CH)             CpCoreDriverInit(void)
#endif


//-------------------------------------------------------------------
// Re-define the function for proper compilation.
//
#if   CP_SMALL_CODE == 1
#undef   CpCoreDriverRelease
#define  CpCoreDriverRelease(CH)             CpCoreDriverRelease(void)
#endif


//-------------------------------------------------------------------//
// Macros for CpMsgXXX() commands                                    //
//-------------------------------------------------------------------//
#if   CP_CAN_MSG_MACRO == 1

#define  CpMsgClrRemote(MSG_PTR)          ( (MSG_PTR)->ubMsgCtrl &= ~CP_MASK_RTR_BIT   );

#define  CpMsgClear(MSG_PTR)              ( (MSG_PTR)->tuMsgId.ulExt = 0); \
                                          ( (MSG_PTR)->ubMsgCtrl = 0);  \
                                          ( (MSG_PTR)->ubMsgDLC  = 0);

#define  CpMsgGetData(MSG_PTR, POS)       ( (MSG_PTR)->tuMsgData.aubByte[POS] )
#define  CpMsgGetDlc(MSG_PTR)             ( (MSG_PTR)->ubMsgDLC)

#define  CpMsgGetExtId(MSG_PTR)           ( (MSG_PTR)->tuMsgId.ulExt)
#define  CpMsgGetStdId(MSG_PTR)           ( (MSG_PTR)->tuMsgId.uwStd)

#define  CpMsgIsExtended(MSG_PTR)         ( (MSG_PTR)->ubMsgCtrl & CP_MASK_EXT_BIT )
#define  CpMsgIsRemote(MSG_PTR)           ( (MSG_PTR)->ubMsgCtrl & CP_MASK_RTR_BIT )
#define  CpMsgIsOverrun(MSG_PTR)          ( (MSG_PTR)->ubMsgCtrl & CP_MASK_OVR_BIT )

#define  CpMsgSetData(MSG_PTR, POS, VAL)  ( (MSG_PTR)->tuMsgData.aubByte[POS] = VAL )
#define  CpMsgSetDlc(MSG_PTR, DLC)        ( (MSG_PTR)->ubMsgDLC = DLC )

#define  CpMsgSetExtId(MSG_PTR, VAL)      ( (MSG_PTR)->tuMsgId.ulExt = VAL & CP_MASK_EXT_FRAME); \
                                          ( (MSG_PTR)->ubMsgCtrl |= CP_MASK_EXT_BIT   );

#define  CpMsgSetRemote(MSG_PTR)          ( (MSG_PTR)->ubMsgCtrl |= CP_MASK_RTR_BIT   );

#define  CpMsgSetOverrun(MSG_PTR)         ( (MSG_PTR)->ubMsgCtrl |= CP_MASK_OVR_BIT   );

#define  CpMsgSetStdId(MSG_PTR, VAL)      ( (MSG_PTR)->tuMsgId.uwStd = VAL & CP_MASK_STD_FRAME); \
                                          ( (MSG_PTR)->ubMsgCtrl &= ~CP_MASK_EXT_BIT   );

#define  CpMsgSetTime(MSG_PTR, VAL)       ( (MSG_PTR)->tsMsgTime.ulSec1970 = (VAL)->ulSec1970); \
                                          ( (MSG_PTR)->tsMsgTime.ulNanoSec = (VAL)->ulNanoSec);

#endif


/*----------------------------------------------------------------------------*/
/*!
** \enum    CpErr
** \brief   CANpie Error codes
**
** All functions that may cause an error condition will return an
** error code. The CANpie error codes are within the value range from
** 0 to 127. The designer of the core functions might extend the error code
** table with hardware specific error codes, which must be in the range
** from 128 to 255.
*/
enum CpErr {

   /*!   No error (00dec / 00hex)
   */
   CpErr_OK = 0,

   /*!   Error not specified (01dec / 01hex)
   */
   CpErr_GENERIC,

   /*!   Hardware failure (02dec / 02hex)
   */
   CpErr_HARDWARE,

   /*!   Initialisation failure (03dec / 03hex)
   */
   CpErr_INIT_FAIL,

   /*!   Channel is initialised, ready to run (04dec / 04hex)
   */
   CpErr_INIT_READY,

   /*!    CAN channel was not initialised (05dec / 05hex)
   */
   CpErr_INIT_MISSING,

   /*!   Receive buffer is empty (05dec / 05hex)
   */
   CpErr_RCV_EMPTY,

   /*!   Receive buffer overrun (06dec / 06hex)
   */
   CpErr_OVERRUN,

   /*!   Transmit buffer is full (07dec / 07hex)
   */
   CpErr_TRM_FULL,

   /*!   CAN message has wrong format (10dec / 0Ahex)
   */
   CpErr_CAN_MESSAGE = 10,

   /*!   CAN identifier not valid (11dec / 0Bhex)
   */
   CpErr_CAN_ID,

   /*!   CAN data length code not valid (12dec / 0Chex)
   */
   CpErr_CAN_DLC,

   /*!   FIFO is empty (20dec / 14hex)
   */
   CpErr_FIFO_EMPTY = 20,

   /*!   Message is waiting in FIFO (21dec / 15hex)
   */
   CpErr_FIFO_WAIT,

   /*!   FIFO is full (22dec / 16hex)
   */
   CpErr_FIFO_FULL,

   /*!   FIFO size is out of range (23dec / 17hex)
   */
   CpErr_FIFO_SIZE,

   /*!   Parameter of FIFO function is out of range (24dec / 18hex)
   */
   CpErr_FIFO_PARM,

   /*!   Controller is in error passive (30dec / 1Ehex)
   */
   CpErr_BUS_PASSIVE = 30,

   /*!   Controller is in bus off (31dec / 1Fhex)
   */
   CpErr_BUS_OFF,

   /*!   Controller is in warning status (32dec / 20hex)
   */
   CpErr_BUS_WARNING,

   /*!   Channel out of range (40dec / 28hex)
   */
   CpErr_CHANNEL = 40,

   /*!   Register address out of range (41dec / 29hex)
   */
   CpErr_REGISTER,

   /*!   Baudrate out of range (42dec / 2Ahex)
   */
   CpErr_BAUDRATE,

   /*!   Buffer number out of range (43dec / 2Bhex)
   */
   CpErr_BUFFER,

   /*!   Parameter number out of range (44dec / 2Chex)
   */
   CpErr_PARAM,

   /*!   Function is not supported (50dec / 32hex)
   */
   CpErr_NOT_SUPPORTED = 50
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_FIFO
** \brief   FIFO Buffer numbers
*/
enum CP_FIFO {

   CP_FIFO_RCV = 0,

   CP_FIFO_TRM
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_CALLBACK
** \brief   Callback Return Codes
**
** These return values are used by the callback functions that can be
** installed by the function CpCoreIntFunctions(). <p>
** \b Example
** \code
** _U08 MyCallback(_TsCpCanMsg * ptsCanMsgV, _U08 ubBufferIdxV)
** {
**    // Do something with IDs < 100
**    if( CpMsgGetStdId(ptsCanMsgV) < 100)
**    {
**       //.....
**       return(CP_CALLBACK_PROCESSED)
**    }
**
**    // Put all other messages into the FIFO
**    return (CP_CALLBACK_PUSH_FIFO);
** }
** \endcode
** <br>
**
*/
enum CP_CALLBACK {

   /*!
   ** Message was processed by callback and is not inserted in the FIFO
   */
   CP_CALLBACK_PROCESSED = 0,

   /*!
   ** Message was processed by callback and is inserted in the FIFO
   */
   CP_CALLBACK_PUSH_FIFO
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_CHANNEL
** \brief   Channel definition
**
** The physical CAN interfaces are numbered from 0 .. N-1 (N: total
** number of physical CAN interfaces on the target system). The enumeration
** CP_CHANNEL lists up to 8 physical interfaces.
*/
enum CP_CHANNEL {

   /*! CAN interface 1              */
   CP_CHANNEL_1 = 0,

   /*! CAN interface 2              */
   CP_CHANNEL_2,

   /*! CAN interface 3              */
   CP_CHANNEL_3,

   /*! CAN interface 4              */
   CP_CHANNEL_4,

   /*! CAN interface 5              */
   CP_CHANNEL_5,

   /*! CAN interface 6              */
   CP_CHANNEL_6,

   /*! CAN interface 7              */
   CP_CHANNEL_7,

   /*! CAN interface 8              */
   CP_CHANNEL_8
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_MODE
** \brief   Mode of CAN controller
**
** These values are used as parameter for the function CpCoreCanMode() in
** order to change the state of the CAN controller.
*/
enum CP_MODE {

   /*!   Set controller in Stop mode (no reception / transmission possible)
   */
   CP_MODE_STOP = 0,

   /*!   Set controller into normal operation
   */
   CP_MODE_START,

   /*!   Set controller into listen-only mode
   */
   CP_MODE_LISTEN_ONLY,

   /*!   Set controller into Sleep mode
   */
   CP_MODE_SLEEP
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_STATE
** \brief   State of CAN controller
**
** These values are used as return value for the function CpCoreCanState().
*/
enum CP_STATE {

   /*!
   ** CAN controller is in stopped mode
   */
   CP_STATE_STOPPED  = 0,

   /*!
   ** CAN controller is in Sleep mode
   */
   CP_STATE_SLEEPING,

   /*!
   ** CAN controller is error active
   */
   CP_STATE_BUS_ACTIVE,

   /*!
   ** CAN controller is active, warning level is reached
   */
   CP_STATE_BUS_WARN,

   /*!
   ** CAN controller is error passive
   */
   CP_STATE_BUS_PASSIVE,

   /*!
   ** CAN controller went into Bus Off
   */
   CP_STATE_BUS_OFF,

   /*!
   ** General failure of physical layer detected (if supported by hardware)
   */
   CP_STATE_PHY_FAULT = 10,

   /*!
   ** Fault on CAN-H detected (Low Speed CAN)
   */
   CP_STATE_PHY_H,

   /*!
   ** Fault on CAN-L detected (Low Speed CAN)
   */
   CP_STATE_PHY_L
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_ERR_TYPE
** \brief   Error type
**
** These values are used as return value for the function CpCoreCanError().
*/
enum CP_ERR_TYPE {
   CP_ERR_TYPE_NONE = 0,
   CP_ERR_TYPE_BIT0,
   CP_ERR_TYPE_BIT1,
   CP_ERR_TYPE_STUFF,
   CP_ERR_TYPE_FORM,
   CP_ERR_TYPE_CRC,
   CP_ERR_TYPE_ACK
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_BUFFER
** \brief   Buffer definition
**
** The enumeration CP_BUFFER is used to define a message buffer inside a
** FullCAN controller. The index for the first buffer starts at 1.
*/
enum CP_BUFFER {
   CP_BUFFER_1 = 1,
   CP_BUFFER_2,
   CP_BUFFER_3,
   CP_BUFFER_4,
   CP_BUFFER_5,
   CP_BUFFER_6,
   CP_BUFFER_7,
   CP_BUFFER_8,
   CP_BUFFER_9,
   CP_BUFFER_10,
   CP_BUFFER_11,
   CP_BUFFER_12,
   CP_BUFFER_13,
   CP_BUFFER_14,
   CP_BUFFER_15,
   CP_BUFFER_16,
   CP_BUFFER_17,
   CP_BUFFER_18,
   CP_BUFFER_19,
   CP_BUFFER_20,
   CP_BUFFER_21,
   CP_BUFFER_22,
   CP_BUFFER_23,
   CP_BUFFER_24,
   CP_BUFFER_25,
   CP_BUFFER_26,
   CP_BUFFER_27,
   CP_BUFFER_28,
   CP_BUFFER_29,
   CP_BUFFER_30,
   CP_BUFFER_31,
   CP_BUFFER_32
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_BUFFER_DIR
** \brief   Buffer direction definition
*/
enum CP_BUFFER_DIR {

   /*!
   **    Buffer direction is receive
   */
   CP_BUFFER_DIR_RX = 0,

   /*!
   **    Buffer direction is transmit
   */
   CP_BUFFER_DIR_TX
};


/*----------------------------------------------------------------------------*\
** Structures                                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/


struct CpTime_s {
   _U32  ulSec1970;
   _U32  ulNanoSec;
};

typedef struct CpTime_s  _TsCpTime;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpPortEmbedded_s   cp_arch.h
** \brief   Port structure for embedded target
**
*/
struct CpPortEmbedded_s {

   /*!   physical CAN interface number,
   **    first CAN channel (index) is 0
   */
   _U08     ubPhyIf;

   /*!   logical CAN interface number,
   **    first index is 0
   */
   _U08     ubLogIf;

};


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpCanMsg_s   canpie.h
** \brief   CAN message structure
**
** For transmission and reception of CAN messages a structure which holds
** all necessary informations is used. The structure has the following
** data fields:
*/
struct CpCanMsg_s {

   /*!
   ** The identifier field may have 11 bits for standard frames
   ** (CAN specification 2.0A) or 29 bits for extended frames
   ** (CAN specification 2.0B). The three most significant bits
   ** are reserved (always read 0).
   ** \see CP_MASK
   */
   union {
      /*! identifier for Standard Frame      */
      _U16  uwStd;
      /*! identifier for Extended Frame      */
      _U32  ulExt;
   } tuMsgId;

   /*!
   ** The data field has up to 8 bytes (64 bit) of message data.
   ** The number of used bytes is described via the structure
   ** member \c ubMsgDLC.
   */
   union {
      /*!   byte access, array of 8 byte     */
      _U08  aubByte[8];

      /*!   16 bit access, array of 4 words  */
      _U16  auwWord[4];

      /*!   32 bit access, array of 2 longs  */
      _U32  aulLong[2];
   } tuMsgData;

   /*!
   ** The data length code denotes the number of data bytes
   ** which are transmitted by a message.
   ** The possible value range for the data length code is
   ** from 0 to 8 (bytes).
   */
   _U08  ubMsgDLC;

   /*!
   ** The structure member \c ubMsgCtrl defines the
   ** different data frames (2.0A / 2.0B) and the RTR frames.
   ** <ul>
   ** <li>Bit 0: Extended Frame if set to 1, else Standard Frame
   ** <li>Bit 1: Remote Frame if set to 1, else Data Frame
   ** <li>Bit 2: Receiver Overrun if set to 1, else normal reception
   ** </ul>
   ** \see CP_MASK
   */
   _U08  ubMsgCtrl;

#if CP_CAN_MSG_TIME == 1
   /*!   The time stamp field defines the time when a CAN message
   **    was received by the CAN controller. This is an optional
   **    field (available if #CP_CAN_MSG_TIME is set to 1).
   */
   _TsCpTime tsMsgTime;
#endif

#if CP_CAN_MSG_USER == 1

   /*!   The field user data can hold a 32 bit value, which is
   **    defined by the user. This is an optional field
   **    (available if #CP_CAN_MSG_USER is set to 1).
   */
   _U32  ulMsgUser;
#endif

};


/*----------------------------------------------------------------------------*/
/*!
** \typedef _TsCpCanMsg
** \brief   CAN message structure
**
** For transmission and reception of CAN messages the structure CpCanMsg_s
** is used.
*/
typedef struct CpCanMsg_s  _TsCpCanMsg;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpStruct_HDI   CANpie.h
** \brief   Hardware description interface
**
** The Hardware Description Interface provides a method to gather
** information about the CAN hardware and the functionality of the driver.
** All items in the structure CpStruct_HDI are constant and must be
** supplied by the designer of the CAN driver. The hardware description
** structure is available for every physical CAN channel.
*/
struct CpHdi_s {

  _U16 uwVersionNumber;

   /*!   Bit coded value that describes the features of the CAN driver.
   **    <ul>
   **    <li>Bit 0/1: 0 = Standard Frame, 1 = Extended Frame passive,
   **                 2 = Extended Frame active
   **    <li>Bit 2: 0 = BasicCAN, 1 = FullCAN
   **    <li>Bit 3: 0 = No IRQ Handler, 1 = Has IRQ Handler
   **    <li>Bit 4: 0 = No identifier filter, 1 = software identifier filter
   **    <li>Bit 5: 0 = No timestamp, 1 = has timestamp
   **    <li>Bit 6: 0 = No user data field, 1 = has user data field
   **    <li>Bit 7: reserved
   **    </ul>
   */
  _U16 uwSupportFlags;

   /*!   Constant value that identifies the used CAN controller
   **    chip. Possible values for this member are listed
   **    in the header file cp_cc.h
   */
  _U16 uwControllerType;

   /*!   Defines the number of the interrupt which is used.
   **    If the flag IRQHandler is set to 0, the value of
   **    IRQNumber will be undefined.
   */
  _U16 uwIRQNumber;

  _U16 uwBufferMax;
  _U16 uwRes;

  _U32 ulTimeStampRes;
  _U32 ulCanClock;
  _U32 ulBitrate;
};


/*----------------------------------------------------------------------------*/
/*!
** \typedef _TsCpHdi
** \brief   Hardware description interface structure
**
** The structure CpHdi_s provides fields to gather information about
** the CAN hardware.
*/
typedef struct CpHdi_s  _TsCpHdi;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpBitTiming_s   canpie.h
** \brief   Bit timing structure
**
*/
struct CpBitTiming_s {

   /*!   Bit timing register 0
   */
   _U08     ubBtr0;

   /*!   Bit timing register 1
   */
   _U08     ubBtr1;

   /*!   Synchronisation jump width
   */
   _U08     ubSjw;

   /*!   Baudrate prescaler
   */
   _U08     ubBrp;
};

typedef struct CpBitTiming_s  _TsCpBitTiming;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpStatistic_s   canpie.h
** \brief   CAN statistic structure
**
*/
struct CpStatistic_s {

   /*!   Total number of received data & remote frames
   */
   _U32     ulRcvMsgCount;

   /*!   Total number of transmitted data & remote frames
   */
   _U32     ulTrmMsgCount;

   /*!   Total number of error frames
   */
   _U32     ulErrMsgCount;
};

typedef struct CpStatistic_s _TsCpStatistic;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpState_s   canpie.h
** \brief   CAN state structure
**
*/
struct CpState_s {

   /*!   CAN error state
   */
   _U08     ubCanErrState;

   /*!   Last error type occurred
   */
   _U08     ubCanErrType;

   /*!   receive error counter
   */
   _U08     ubCanRcvErrCnt;

   /*!   transmit error counter
   */
   _U08     ubCanTrmErrCnt;
};

typedef struct CpState_s _TsCpState;


struct CpFifo_s {

   /*! This pointer is the first entry of an array of messages. The
   **  total number of messages is given by the field <b>uwFifoSize</b>.
   */
   _TsCpCanMsg *   ptsMsgList;

   /*! The 'uwFifoSize' field holds the number of maximum messages
   **  that can be stored in the FIFO.
   */
   _U32            ulFifoSize;

   _U32            ulHeadPos;
   _U32            ulTailPos;
};

typedef struct CpFifo_s   _TsCpFifo;


//---------------------------------------------------------------------
// Architecture structures for Texas Instruments TMS320F28xx
//
#if CP_TARGET == CP_CC_TMS320F28xx

typedef struct CpPortEmbedded_s  _TsCpPort;
typedef _U16                     _TvCpStatus;

#endif


/*----------------------------------------------------------------------------*\
** Function prototypes                                                        **
**                                                                            **
\*----------------------------------------------------------------------------*/


/*!
** \brief   Set bitrate of CAN controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBaudSelV     Bitrate selection
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function directly writes to the bit timing registers of the CAN
** controller. The value for the parameter \e ubBaudSelV is taken
** from the CP_BAUD enumeration.
**
*/
_TvCpStatus CpCoreBaudrate(_TsCpPort * ptsPortV, _U08 ubBaudSelV);


/*!
** \brief   Set bitrate of CAN controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsBitrateV    Pointer to bit timing structure
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function directly writes to the bit timing registers of the CAN
** controller. Usage of the function requires a detailed knowledge of
** the used CAN controller hardware.
**
*/
_TvCpStatus CpCoreBittiming(_TsCpPort * ptsPortV, _TsCpBitTiming * ptsBitrateV);


/*!
** \brief   Enable / Disable a message buffer
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Buffer number
** \param   ubEnableV      Flag to enable/disable message Buffer
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** The functions enables or disables a message buffer for reception and
** transmission of messages. The message buffer has to be configured by
** CpCoreBufferInit() in advance.
** In contrast to the CpCoreBufferRelease() function the contents
** message buffer is not deleted, but message reception and transmission
** can be suppressed by setting \c ubEnableV to 0.
**
*/
_TvCpStatus CpCoreBufferEnable(_TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                               _U08 ubEnableV);


/*!
** \brief   Get data from message buffer
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Buffer number
** \param   pubDataV       Buffer for data
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function is the fastest method to get data from a FullCAN message buffer.
** The message buffer has to be configured by CpCoreBufferInit() in advance.
**
*/
_TvCpStatus CpCoreBufferGetData( _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 * pubDataV);


/*!
** \brief   Get DLC of specified buffer
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Buffer number
** \param   pubDlcV        Data Length Code
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function retrieves the Data Length Code (DLC) of the selected buffer
** \c ubBufferIdxV.
*/
_TvCpStatus CpCoreBufferGetDlc(  _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 * pubDlcV);


/*!
** \brief   Initialise buffer in FullCAN controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsCanMsgV     Pointer to a CAN message structure
** \param   ubBufferIdxV   Buffer number
** \param   ubDirectionV   Direction of message
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** \see     CpCoreBufferRelease()
**
** This function allocates the message buffer in a FullCAN controller.
** The number of the message buffer inside the FullCAN controller is
** denoted via the parameter \c ubBufferIdxV.
** The parameter \c ubDirectionV can have the following values:
** \li CP_BUFFER_DIR_RX: receive
** \li CP_BUFFER_DIR_TX: transmit
**
** The following example shows the setup of a transmit buffer
** \dontinclude buf_init.c
** \skip    void AllocateTrmBuffer
** \until   }
**
** An allocated transmit buffer can be sent via the function
** CpCoreBufferSend().
*/
_TvCpStatus CpCoreBufferInit( _TsCpPort * ptsPortV, _TsCpCanMsg * ptsCanMsgV,
                              _U08 ubBufferIdxV, _U08 ubDirectionV);


/*!
** \brief   Release message buffer of FullCAN controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Buffer number
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** \see     CpCoreBufferInit()
*/
_TvCpStatus CpCoreBufferRelease( _TsCpPort * ptsPortV, _U08 ubBufferIdxV);


/*!
** \brief   Send message from message buffer
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Index of message buffer
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function transmits a message from the specified message buffer
** \c ubBufferIdxV. The message buffer has to be configured by a call to
** CpCoreBufferInit() in advance. The first message buffer starts at
** the index CP_BUFFER_1.
**
*/
_TvCpStatus CpCoreBufferSend(_TsCpPort * ptsPortV, _U08 ubBufferIdxV);


/*!
** \brief   Set data of message buffer
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Buffer number
** \param   pubDataV       Pointer to data buffer
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function is the fastest method to set the data bytes of a CAN message.
** It can be used in combination with the function CpCoreBufferSend(). It
** will write 8 data bytes into the buffer defined by \e ubBufferIdxV. The
** buffer has to be configured by CpCoreBufferInit() in advance. The size
** of the data buffer \e pubDataV must have a size of 8 bytes.
**
** The following example demonstrates the access to the data bytes of a CAN
** message:
** \code
**  _U08 aubDataT[8];   // buffer for 8 bytes
**
** aubDataT[0] = 0x11;  // byte 0: set to 11hex
** aubDataT[1] = 0x22;  // byte 1: set to 22hex

** //--- copy the stuff to message buffer 1 ---------------
** CpCoreBufferSetData(CP_CHANNEL_1, CP_BUFFER_1, &aubDataT);
**
** //--- send this message out ----------------------------
** CpCoreBufferSend(CP_CHANNEL_1, CP_BUFFER_1);
** \endcode
**
*/
_TvCpStatus CpCoreBufferSetData( _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 * pubDataV);


/*!
** \brief   Set DLC of specified buffer
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubBufferIdxV   Buffer number
** \param   ubDlcV         Data Length Code
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function sets the Data Length Code (DLC) of the selected buffer
** ubBufferIdxV. The DLC must be in the range from 0 to 8.
*/
_TvCpStatus CpCoreBufferSetDlc(  _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 ubDlcV);


/*!
** \brief   Set state of CAN controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ubModeV        Mode selection
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function changes the operating mode of the CAN controller.
** Possible values for mode are defined in the #CP_MODE enumeration.
*/
_TvCpStatus CpCoreCanMode(_TsCpPort * ptsPortV, _U08 ubModeV);


/*!
** \brief   Retrieve status of CAN controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsStateV      Pointer to CAN state structure
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function retrieved the present state of the CAN controller. Possible
** values are defined in the #CP_STATE enumeration. The state of the CAN
** controller is copied to the variable pointer 'ptsStateV'.
*/
_TvCpStatus CpCoreCanState(_TsCpPort * ptsPortV, _TsCpState * ptsStateV);


/*!
** \brief   Initialise the CAN driver
** \param   ubPhyIfV     CAN channel of the hardware
** \param   ptsPortV     Pointer to CAN port structure
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** \see     CpCoreDriverRelease()
**
** The functions opens the physical CAN interface defined by the
** parameter \c ubPhyIfV. The value for \c ubPhyIfV is taken from
** the enumeration CP_CHANNEL. The function sets up the field
** members of the CAN port handle \c ptsPortV. On success, the
** function returns CpErr_OK. On failure, the function can return
** the following values:
** <ul>
** <li>#CpErr_HARDWARE
** <li>#CpErr_INIT_FAIL
** </ul>
** An opened handle to a CAN port must be closed via the CpCoreDriverRelease()
** function.
*/
_TvCpStatus CpCoreDriverInit(_U08 ubPhyIfV, _TsCpPort * ptsPortV);


/*!
** \brief   Release the CAN driver
** \param   ptsPortV       Pointer to CAN port structure
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** \see     CpCoreDriverInit()
**
** The implementation of this function is dependent on the operating
** system. Typical tasks might be:
** <ul>
** <li>clear the interrupt vector / routine
** <li>close all open paths to the hardware
** </ul>
**
*/
_TvCpStatus CpCoreDriverRelease(_TsCpPort * ptsPortV);


/*!
** \brief   Get hardware description information
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsHdiV        Pointer to the Hardware Description Interface
**                         structure (_TsCpHdi)
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function retrieves information about the used hardware.
**
*/
_TvCpStatus CpCoreHDI(_TsCpPort * ptsPortV, _TsCpHdi * ptsHdiV);


/*!
** \brief   Install callback functions
** \param   ptsPortV       Pointer to CAN port structure
** \param   pfnRcvHandler  Pointer to callback function for receive interrupt
** \param   pfnTrmHandler  Pointer to callback function for transmit interrupt
** \param   pfnErrHandler  Pointer to callback function for error interrupt
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function will install three different callback routines in the
** interrupt handler. If you do not want to install a callback for a
** certain interrupt condition the parameter must be set to NULL.
** The callback functions for receive and transmit interrupt have the
** following syntax:
** <code>
** _U08 Handler(_TsCpCanMsg * ptsCanMsgV, _U08 ubBufferIdxV)
** </code>
** <p>
** The callback function for the CAN status-change / error interrupt has
** the following syntax:
** <code>
** _U08 Handler(_U08 ubStateV)
** </code>
** <p>
*/
_TvCpStatus CpCoreIntFunctions(  _TsCpPort * ptsPortV,
                                 _U08 (* pfnRcvHandler) (_TsCpCanMsg *, _U08),
                                 _U08 (* pfnTrmHandler) (_TsCpCanMsg *, _U08),
                                 _U08 (* pfnErrHandler) (_U08)                );


/*!
** \brief   Read a CAN message from controller
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsBufferV     Pointer to a CAN message structure
** \param   pulBufferSizeV Pointer to size variable
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function reads the receive queue from a CAN controller.
*/
_TvCpStatus CpCoreMsgRead( _TsCpPort * ptsPortV, _TsCpCanMsg * ptsBufferV,
                           _U32 * pulBufferSizeV);


/*!
** \brief   Transmit a CAN message
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsBufferV     Pointer to a CAN message structure
** \param   pulBufferSizeV Pointer to size variable
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function writes to the transmit queue of a CAN controller.
*/
_TvCpStatus CpCoreMsgWrite(_TsCpPort * ptsPortV, _TsCpCanMsg * ptsBufferV,
                           _U32 * pulBufferSizeV);


/*!
** \brief   Read CAN controller statistics
** \param   ptsPortV       Pointer to CAN port structure
** \param   ptsStatsV      Pointer to statistic data structure
**
** \return  Error code taken from the #CpErr enumeration. If no error
**          occurred, the function will return \c CpErr_OK.
**
** This function copies CAN statistic information to the structure
** pointed by ptsStatsV.
**
*/
_TvCpStatus CpCoreStatistic(_TsCpPort * ptsPortV, _TsCpStatistic * ptsStatsV);


#if   CP_CAN_MSG_MACRO == 0

/*!
** \brief   Clear RTR bit
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \see     CpMsgSetRemote()
**
** This function clears the Remote Transmission bit (RTR).
*/
void  CpMsgClrRemote(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Clear message structure
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
**
** This macro sets the identifier field and the flags field
** of a CAN message structure to 0. It is recommended to use
** this function when a message structure is assigned in memory.
*/
void  CpMsgClear(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Get Data
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \param   ubPosV      Zero based index of byte position
** \return  Value of data at position ubPosV
**
** This functions retrieves the data of a CAN message. The parameter
** \c ubPosV must be within the range 0 .. 7. Please note that the
** macro implementation does not check the value range of the parameter
** \c ubPosV.
*/
_U08  CpMsgGetData(_TsCpCanMsg * ptsCanMsgV, _U08 ubPosV);


/*!
** \brief   Get Data Length Code
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \return  Data Length Code (DLC) of CAN message
**
** This function retrieves the data length code (DLC) of a CAN message.
** The return value range is between 0 and 8.
*/
_U08  CpMsgGetDlc(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Get 29 Bit Identifier Value
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \return  Extended Identifier value
** \see     CpMsgGetStdId()
**
** This function retrieves the value for the identifier of an
** extended frame (CAN 2.0B). The frame format of the CAN message
** can be tested with the CpMsgIsExtended() function.
*/
_U32  CpMsgGetExtId(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Get 11 Bit Identifier Value
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \return  Standard Identifier value
** \see     CpMsgGetExtId()
**
** This macro retrieves the value for the identifier of an
** standard frame (CAN 2.0A). The frame format of the CAN message
** can be tested with the CpMsgIsExtended() function.
*/
_U16  CpMsgGetStdId(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Check the frame type
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \return  0 for CAN 2.0A, 1 for CAN 2.0B
**
** This function checks the frame type. If the frame is CAN 2.0A
** (Standard Frame), the value 0 is returned. If the frame is
** CAN 2.0B (Extended Frame), the value 1 is returned.
*/
_U08  CpMsgIsExtended(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Check for remote frame
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \return  0 for data frame, 1 for remote frame
**
** This function checks if the Remote Transmission bit (RTR) is
** set. If the RTR bit is set, the function will return 1, otherwise
** 0.
*/
_U08  CpMsgIsRemote(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Set Data
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \param   ubPosV      Zero based index of byte position
** \param   ubValueV    Value of data byte in CAN message
**
** This function sets the data in a CAN message. The parameter
** \c ubPosV must be within the range 0 .. 7. Please note that the
** macro implementation does not check the value range of the parameter
** \c ubPosV.
*/
void  CpMsgSetData(_TsCpCanMsg * ptsCanMsgV, _U08 ubPosV, _U08 ubValueV);


/*!
** \brief   Set Data Length Code
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \param   ubDlcV      Data Length Code
**
** This function sets the Data Length Code (DLC) of a CAN message.
** The parameter ubDlcV must be within the range from 0..8.
** Please note that the macro implementation does not check the value
** range of the parameter \c ubDlcV.
*/
void  CpMsgSetDlc(_TsCpCanMsg * ptsCanMsgV, _U08 ubDlcV);


/*!
** \brief   Set 29 Bit Identifier Value
** \param   ptsCanMsgV     Pointer to a _TsCpCanMsg message
** \param   ulExtIdV       Identifier value
** \see     CpMsgSetStdFrame()
**
** This function sets the identifer value for an
** Extended Frame (CAN 2.0B) and marks the frame format as
** accordingly. The value of \c ulExtIdV is limited to
** #CP_MASK_EXT_FRAME.
*/
void  CpMsgSetExtId(_TsCpCanMsg * ptsCanMsgV, _U32 ulExtIdV);


/*!
** \brief   Set RTR bit
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
**
** This function sets the Remote Transmission bit (RTR).
*/
void  CpMsgSetRemote(_TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Set 11 Bit Identifier Value
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \param   uwStdIdV    Identifier value
** \see     CpMsgSetExtFrame()
**
** This function sets the identifer value for a Standard frame (CAN 2.0A).
** The value of \c uwStdIdV is limited to #CP_MASK_STD_FRAME.
*/
void  CpMsgSetStdId(_TsCpCanMsg * ptsCanMsgV, _U16 uwStdIdV);


/*!
** \brief   Set timestamp value
** \param   ptsCanMsgV  Pointer to a _TsCpCanMsg message
** \param   ptsTimeV    Pointer to timestamp structure
**
** This function sets the timestamp value for a CAN frame.
*/
void  CpMsgSetTime(_TsCpCanMsg * ptsCanMsgV, _TsCpTime * ptsTimeV);

#endif


/*!
** \brief   Setup FIFO
** \param   ptsFifoV      pointer to FIFO
**
** \return  Error code taken from the CpErr enumeration. If no error
**          occured, the function will return CpErr_OK.
**
** This function reserves memory for the specified FIFO buffer. If
** a static FIFO is used (CP_FIFO_TYPE == 1) the parameter 'ulSizeV'
** is not evaluated.
**
*/
_TvCpStatus CpFifoInit(_TsCpFifo * ptsFifoV, _U32 ulSizeV);


/*!
** \brief   Remove FIFO
** \param   ptsFifoV      pointer to FIFO
**
** \return  Error code taken from the CpErr enumeration. If no error
**          occured, the function will return CpErr_OK.
**
** This function frees the reserved memory for the specified FIFO
** buffer. If a static FIFO is used (CP_FIFO_TYPE == 1) this function
** simply returns CpErr_OK.
**
*/
_TvCpStatus CpFifoRelease(_TsCpFifo * ptsFifoV);


/*!
** \brief   Clear FIFO contents
** \param   ptsFifoV      pointer to FIFO
**
** This function clears the specified FIFO buffer. All messages inside
** the FIFO are erased.
**
*/
void CpFifoClear(_TsCpFifo * ptsFifoV );


/*!
** \brief   Check if FIFO is empty
** \param   ptsFifoV      pointer to FIFO
**
** \return  number of messages stored in FIFO
*/
_U32 CpFifoMsgCount(_TsCpFifo * ptsFifoV);


/*!
** \brief   Push message into FIFO
** \param   ptsFifoV      pointer to FIFO
** \param   ptsCanMsgV    pointer to CAN message
**
** \return  Error code taken from the CpErr enumeration. If no error
**          occured, the function will return CpErr_OK.
**
** Push a message into the FIFO buffer, given by the parameter
** 'ptsFifoV'. If the buffer is full, the function will return
** 'CpErr_FIFO_FULL'.
**
*/
_TvCpStatus CpFifoPush(_TsCpFifo * ptsFifoV, _TsCpCanMsg * ptsCanMsgV);


/*!
** \brief   Pop message from FIFO
** \param   ptsFifoV      pointer to FIFO
** \param   ptsCanMsgV    pointer to CAN message
**
** \return  Error code taken from the CpErr enumeration. If no error
**          occured, the function will return CpErr_OK.
**
** Pop a message from the FIFO buffer, given by the parameter
** 'ptsFifoV'. If the buffer is empty, the function will
** return 'CpErr_FIFO_EMPTY'.
**
*/
_TvCpStatus CpFifoPop(_TsCpFifo * ptsFifoV, _TsCpCanMsg * ptsCanMsgV);


#endif   /* _CANPIE_H_   */

