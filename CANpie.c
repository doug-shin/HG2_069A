//****************************************************************************//
// File:          CANpie.c                                                    //
// Description:   CANpie core, message access, and buffer functions           //
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
// 09.09.1999  Initial version                                                //
// 05.20.2010  collected and revised                                          //
//                                                                            //
//****************************************************************************//


#include <string.h>        // for memcpy() function

#include "CANpie.h"

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


//-------------------------------------------------------------------
// test the configuration
//
#ifndef CP_CAN_MSG_USER
#error  Driver must be compiled with CP_CAN_MSG_USER=1
#endif

#if     CP_CAN_MSG_USER!=1
#error  Driver must be compiled with CP_CAN_MSG_USER=1
#endif


/*----------------------------------------------------------------------------*\
** Definitions                                                                **
**                                                                            **
\*----------------------------------------------------------------------------*/


//-------------------------------------------------------------------
// buffer status information, it is stored in the 'ulMsgUser'
// element of the _TsCpCanMsg structure
//
#define  CP_BUFFER_VAL        0x0001   // buffer is valid
#define  CP_BUFFER_TRM        0x0002   // buffer is for transmit
#define  CP_BUFFER_RCV        0x0004   // buffer is for receive
#define  CP_BUFFER_PND        0x0020   // transmit buffer pending
#define  CP_BUFFER_UPD        0x0040   // receive buffer update


//-------------------------------------------------------------------
// Debugging
// set CP_DBG_CAN_IRQ to 1 for measuring the CAN IRQ time
//
#define  CP_DBG_CAN_IRQ        0


/*----------------------------------------------------------------------------*\
** Variables of module                                                        **
**                                                                            **
\*----------------------------------------------------------------------------*/


/* Create a shadow register structure for the CAN control registers.
 * This is needed, since, only 32-bit access is allowed to these registers.
 * 16-bit access to these registers could potentially corrupt the register contents.
 * This is especially true while writing to a bit (or group of bits) among bits 16 - 31.
 */
struct ECAN_REGS ECanaShadow;
struct ECAN_REGS ECanbShadow;

//-------------------------------------------------------------------
// these pointers store the callback handlers
//
_U08           (* pfnRcvIntHandler) (_TsCpCanMsg *, _U08);
_U08           (* pfnTrmIntHandler) (_TsCpCanMsg *, _U08);
_U08           (* pfnErrIntHandler) (_U08);


//-------------------------------------------------------------------
// mailboxes for up to CP_BUFFER_MAX CAN messages. These messages
// can be either transmit or receive
//
_TsCpCanMsg atsCan1MsgS[CP_BUFFER_MAX];


/*----------------------------------------------------------------------------*\
** Functions                                                                  **
**                                                                            **
\*----------------------------------------------------------------------------*/


//----------------------------------------------------------------------------//
// CpCoreBufferGetData()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferGetData( _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 * pubDataV)
{
   _U08  ubDataCntT;

   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);

   //----------------------------------------------------------------
   // align buffer number to index 0 for atsCanMsgS[]
   //
   ubBufferIdxV--;

   //----------------------------------------------------------------
   // copy data from buffer
   //
   for(ubDataCntT = 0; ubDataCntT < 8; ubDataCntT++)
   {
      *pubDataV = atsCan1MsgS[ubBufferIdxV].tuMsgData.aubByte[ubDataCntT];
      pubDataV++;
   }

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreBufferGetDlc()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferGetDlc(  _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 * pubDlcV)
{
   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);

   //----------------------------------------------------------------
   // align buffer number to index 0 for atsCanMsgS[]
   //
   ubBufferIdxV--;

   *pubDlcV = atsCan1MsgS[ubBufferIdxV].ubMsgDLC;

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreBufferInit()                                                         //
// initialize CAN message buffer                                              //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferInit( _TsCpPort * ptsPortV, _TsCpCanMsg * ptsCanMsgV,
                              _U08 ubBufferIdxV, _U08 ubDirectionV)
{
   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);

   //----------------------------------------------------------------
   // align buffer number to index 0 for atsCanMsgS[]
   //
   ubBufferIdxV--;

   //----------------------------------------------------------------
   // copy the data from the pCanMsgV pointer to the main buffer
   //
   memcpy(&atsCan1MsgS[ubBufferIdxV], ptsCanMsgV, sizeof(_TsCpCanMsg));

   //----------------------------------------------------------------
   // setup the buffer for Transmit / Receive operation
   //
   if(ubDirectionV == CP_BUFFER_DIR_TX)
   {
      //---------------------------------------------------
      // direction is transmit
      //
      atsCan1MsgS[ubBufferIdxV].ulMsgUser = CP_BUFFER_VAL | CP_BUFFER_TRM;
   }
   else
   {
      //---------------------------------------------------
      // direction is receive
      //
      atsCan1MsgS[ubBufferIdxV].ulMsgUser = CP_BUFFER_VAL | CP_BUFFER_RCV;
   }

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreBufferRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferRelease( _TsCpPort * ptsPortV, _U08 ubBufferIdxV)
{
   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);

   //----------------------------------------------------------------
   // align buffer number to index 0 for atsCanMsgS[]
   //
   ubBufferIdxV--;

   //----------------------------------------------------------------
   // clear the message buffer structure
   // clearing bit 0 of ulMsgUser disables the buffer
   //
   memset(&atsCan1MsgS[ubBufferIdxV], 0x00, sizeof(_TsCpCanMsg));

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreBufferSend()                                                         //
// send message out of the CAN controller                                     //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferSend(_TsCpPort * ptsPortV, _U08 ubBufferIdxV)
{
//    _TsCpCanMsg * ptsCanMsgT;

   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);

   //----------------------------------------------------------------
   // internal buffer count starts at 0, the supplied buffer
   // number starts at 1: we adjust it here
   //
   ubBufferIdxV = ubBufferIdxV - 1;
//   ptsCanMsgT = &atsCan1MsgS[ubBufferIdxV];

   // write transmission request  (send the message)

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreBufferSetData()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferSetData( _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 * pubDataV)
{
   _U08  ubDataCntT;

   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);

   //----------------------------------------------------------------
   // align buffer number to index 0 for atsCanMsgS[]
   //
   ubBufferIdxV--;

   //----------------------------------------------------------------
   // copy data to buffer
   //
   for(ubDataCntT = 0; ubDataCntT < 8; ubDataCntT++)
   {
      atsCan1MsgS[ubBufferIdxV].tuMsgData.aubByte[ubDataCntT] = *pubDataV;
      pubDataV++;
   }

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreBufferSetDlc()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreBufferSetDlc(  _TsCpPort * ptsPortV, _U08 ubBufferIdxV,
                                 _U08 ubDlcV)
{
   //----------------------------------------------------------------
   // check for valid buffer number
   //
   if(ubBufferIdxV < CP_BUFFER_1  ) return(CpErr_BUFFER);
   if(ubBufferIdxV > CP_BUFFER_MAX) return(CpErr_BUFFER);


   //----------------------------------------------------------------
   // align buffer number to index 0 for atsCanMsgS[]
   //
   ubBufferIdxV--;

   //----------------------------------------------------------------
   // copy DLC to buffer
   //
   atsCan1MsgS[ubBufferIdxV].ubMsgDLC = ubDlcV;

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreCanMode()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreCanMode(_TsCpPort * ptsPortV, _U08 ubModeV)
{
   _U08  ubStatusT = 0;

   switch(ubModeV)
   {
      case CP_MODE_STOP:
         //------------------------------------------------
         // Mode register

         ubStatusT = CpErr_OK;
         break;

      case CP_MODE_START:
         //------------------------------------------------
         // Mode register:

         ubStatusT = CpErr_OK;
         break;

      default:
         ubStatusT = CpErr_NOT_SUPPORTED;
         break;
   }

   return(ubStatusT);

}


//----------------------------------------------------------------------------//
// CpCoreCanState()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreCanState(_TsCpPort * ptsPortV, _TsCpState * ptsStateV)
{
   //----------------------------------------------------------------
   // read global status register, e.g.:
   //
   //    _U32  ulStatusT = 0;
   //    ulStatusT = Register_of_CAN_controller
   //
   // set the bits of ptsStateV according to this status
   // here you see an example:
   //
   ptsStateV->ubCanErrState   = CP_STATE_BUS_ACTIVE;
   ptsStateV->ubCanErrType    = CP_ERR_TYPE_STUFF;

   //----------------------------------------------------------------
   // if the CAN controller grants access to its internal
   // error counters, copy it here:
   //
   ptsStateV->ubCanRcvErrCnt  = 8;
   ptsStateV->ubCanTrmErrCnt  = 0;

   return(CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreDriverInit()                                                         //
// init CAN controller                                                        //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreDriverInit(_U08 ubPhyIfV, _TsCpPort * ptsPortV)
{
   //----------------------------------------------------------------
   // initialize the CAN controller here
   //

// Initialize eCAN module

   EALLOW;  // This is needed to write to EALLOW protected registers

   if( ubPhyIfV == CP_CHANNEL_1 )
   {
   ptsPortV->ubPhyIf = CP_CHANNEL_1;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
// GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;     // Enable pull-up for GPIO18 (CANRXA)

   GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     // Enable pull-up for GPIO31 (CANTXA)
// GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     // Enable pull-up for GPIO19 (CANTXA)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

   GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   // Asynch qual for GPIO18 (CANRXA)

/* Configure eCAN-A/B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

   GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;   // Configure GPIO30 for CANRXA operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;  // Configure GPIO18 for CANRXA operation

   GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;   // Configure GPIO31 for CANTXA operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;  // Configure GPIO19 for CANTXA operation

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

   ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
   ECanaShadow.CANTIOC.bit.TXFUNC = 1;
   ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

   ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
   ECanaShadow.CANRIOC.bit.RXFUNC = 1;
   ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (required to access mailboxes 16 thru 31) */
                   // HECC mode also enables time-stamping feature

   ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
   ECanaShadow.CANMC.bit.SCB = 1;
   ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

/* Initialize all bits of 'Master Control Field' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

   ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
// as a matter of precaution.

   ECanaRegs.CANTA.all  = 0xFFFFFFFF;  /* Clear all TAn bits */

   ECanaRegs.CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */

   ECanaRegs.CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
   ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

/* Configure bit timing parameters for eCANA*/

   ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
   ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
   ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

   ECanaShadow.CANES.all = ECanaRegs.CANES.all;

   do {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
   } while(ECanaShadow.CANES.bit.CCE != 1 );  // Wait for CCE bit to be set..

   ECanaShadow.CANBTC.all = 0;

   /* The following block for all 150 MHz SYSCLKOUT (75 MHz CAN clock) - default.
    * Bit rate = 1 Mbps. See Note at End of File */

//  SPRUH18 기술문서


// ECanaShadow.CANBTC.bit.BRPREG = 4;   //   1 Mbps
   ECanaShadow.CANBTC.bit.BRPREG = 9;   // 500 kbps
// ECanaShadow.CANBTC.bit.BRPREG = 19;  // 250 kbps
// ECanaShadow.CANBTC.bit.BRPREG = 39;  // 125 kbps
   ECanaShadow.CANBTC.bit.TSEG2REG = 2;
   ECanaShadow.CANBTC.bit.TSEG1REG = 10;
 //  I am using PCAN to transmit message which is set at 1M bits/sec  and I am using ECAN on F28069 as Receiver end
   ECanaShadow.CANBTC.bit.BRPREG = 1; // 1M
   ECanaShadow.CANBTC.bit.TSEG2REG = 4;
   ECanaShadow.CANBTC.bit.TSEG1REG = 13;


   ECanaShadow.CANBTC.all = 0x00020059;


   ECanaShadow.CANBTC.bit.BRPREG = 11; // 2 for 1 Mbps, 5 for 500 kbps, 11 for 250 kbps, 23 for 125 kbps, 29 for 100 kbps
   ECanaShadow.CANBTC.bit.TSEG2REG = 3; // Time Segement 2 for the Bit Timing Configuration Register (length of the phase in TQ units) TSEG2 = TSEG2reg + 1
   ECanaShadow.CANBTC.bit.TSEG1REG = 9; // Time Segement 1 for the Bit Timing Configuration Register (length of a bit on the CAN bus) TSEG1 = TSEG1reg + 1

   ECanaShadow.CANBTC.bit.SAM = 1;
   ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

   ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
   ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
   ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

   ECanaShadow.CANES.all = ECanaRegs.CANES.all;

   do {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
   } while(ECanaShadow.CANES.bit.CCE != 0 );  // Wait for CCE bit to be cleared..

/* Disable all Mailboxes  */
   ECanaRegs.CANME.all = 0;      // Required before writing the MSGIDs

   }

   else if( ubPhyIfV == CP_CHANNEL_2 )
   {
   ptsPortV->ubPhyIf = CP_CHANNEL_2;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//   GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up for GPIO8  (CANTXB)
  GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pull-up for GPIO12 (CANTXB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up for GPIO16 (CANTXB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up for GPIO20 (CANTXB)

//   GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;   // Enable pull-up for GPIO10 (CANRXB)
  GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pull-up for GPIO13 (CANRXB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up for GPIO17 (CANRXB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up for GPIO21 (CANRXB)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

//   GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 3; // Asynch qual for GPIO10 (CANRXB)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3; // Asynch qual for GPIO13 (CANRXB)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch qual for GPIO17 (CANRXB)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3; // Asynch qual for GPIO21 (CANRXB)

/* Configure eCAN-A/B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

//   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 2;   // Configure GPIO8 for CANTXB operation
  GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;  // Configure GPIO12 for CANTXB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 2;  // Configure GPIO16 for CANTXB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 3;  // Configure GPIO20 for CANTXB operation

//   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 2;  // Configure GPIO10 for CANRXB operation
  GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 2;  // Configure GPIO13 for CANRXB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 2;  // Configure GPIO17 for CANRXB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 3;  // Configure GPIO21 for CANRXB operation

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

   ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
   ECanaShadow.CANTIOC.bit.TXFUNC = 1;
   ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

   ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
   ECanaShadow.CANRIOC.bit.RXFUNC = 1;
   ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (required to access mailboxes 16 thru 31) */
                   // HECC mode also enables time-stamping feature

   ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
   ECanaShadow.CANMC.bit.SCB = 1;
   ECanaShadow.CANMC.bit.ABO = 1;
   ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

/* Initialize all bits of 'Master Control Field' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

   ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
   ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
// as a matter of precaution.

   ECanaRegs.CANTA.all  = 0xFFFFFFFF;  /* Clear all TAn bits */

   ECanaRegs.CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */

   ECanaRegs.CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
   ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

/* Configure bit timing parameters for eCANA*/

   ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
   ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
   ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

   ECanaShadow.CANES.all = ECanaRegs.CANES.all;

   do {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
   } while(ECanaShadow.CANES.bit.CCE != 1 );  // Wait for CCE bit to be set..

   ECanaShadow.CANBTC.all = 0;

   /* The following block for all 150 MHz SYSCLKOUT (75 MHz CAN clock) - default.
    * Bit rate = 1 Mbps. See Note at End of File */
//   ECanbShadow.CANBTC.bit.BRPREG = 4;   //   1 Mbps
 ECanaShadow.CANBTC.bit.BRPREG = 9;   // 500 kbps
 // ECanbShadow.CANBTC.bit.BRPREG = 19;  // 250 kbps
// ECanbShadow.CANBTC.bit.BRPREG = 39;  // 125 kbps
// ECanbShadow.CANBTC.bit.BRPREG = 49;  // 100 kbps
    ECanaShadow.CANBTC.bit.TSEG2REG = 2;
   ECanaShadow.CANBTC.bit.TSEG1REG = 10;

   /* The following block is for 90 MHz SYSCLKOUT. (45 MHz CAN module clock Bit rate = 1 Mbps */

   ECanaShadow.CANBTC.bit.BRPREG = 11; // 2 for 1 Mbps, 5 for 500 kbps, 11 for 250 kbps, 23 for 125 kbps, 29 for 100 kbps
   ECanaShadow.CANBTC.bit.TSEG2REG = 3; // Time Segement 2 for the Bit Timing Configuration Register (length of the phase in TQ units) TSEG2 = TSEG2reg + 1
   ECanaShadow.CANBTC.bit.TSEG1REG = 9; // Time Segement 1 for the Bit Timing Configuration Register (length of a bit on the CAN bus) TSEG1 = TSEG1reg + 1

   ECanaShadow.CANBTC.bit.SAM = 1;
   ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

   ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
   ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
   ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

   ECanaShadow.CANES.all = ECanaRegs.CANES.all;

   do {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
   } while(ECanaShadow.CANES.bit.CCE != 0 );      // Wait for CCE bit to be cleared..

/* Disable all Mailboxes  */
 	ECanaRegs.CANME.all = 0;		// Required before writing the MSGIDs

   }

   EDIS;    // This is needed to disable write to EALLOW protected registers

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreDriverRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreDriverRelease(_TsCpPort * ptsPortV)
{
   CpCoreCanMode(ptsPortV, CP_MODE_STOP);
   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreHDI()                                                                //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreHDI(_TsCpPort * ptsPortV, _TsCpHdi * ptsHdiV)
{
   ptsHdiV->ulBitrate  = 0;

   ptsHdiV->ulCanClock = 75;

   ptsHdiV->ulTimeStampRes = 1;

   ptsHdiV->uwBufferMax = CP_BUFFER_MAX;

   ptsHdiV->uwControllerType = CP_TARGET;

   ptsHdiV->uwIRQNumber = 0;

   ptsHdiV->uwSupportFlags = 0;

   ptsHdiV->uwVersionNumber = 1;

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreIntFunctions()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreIntFunctions(  _TsCpPort * ptsPortV,
                                 _U08 (* pfnRcvHandler) (_TsCpCanMsg *, _U08),
                                 _U08 (* pfnTrmHandler) (_TsCpCanMsg *, _U08),
                                 _U08 (* pfnErrHandler) (_U08)                )
{
   //----------------------------------------------------------------
   // store the new callbacks
   //
   pfnRcvIntHandler = pfnRcvHandler;
   pfnTrmIntHandler = pfnTrmHandler;
   pfnErrIntHandler = pfnErrHandler;

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreMsgRead()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreMsgRead( _TsCpPort * ptsPortV, _TsCpCanMsg * ptsBufferV,
                           _U32 * pulBufferSizeV)
{

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreMsgWrite()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreMsgWrite(_TsCpPort * ptsPortV, _TsCpCanMsg * ptsBufferV,
                           _U32 * pulBufferSizeV)
{

   return (CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpCoreStatistic()                                                          //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpCoreStatistic(_TsCpPort * ptsPortV, _TsCpStatistic * ptsStatsV)
{

   return (CpErr_OK);
}


#if   CP_CAN_MSG_MACRO == 0

//----------------------------------------------------------------------------//
// CpMsgClear()                                                               //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgClear(_TsCpCanMsg * ptsCanMsgV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;

   //----------------------------------------------------------------
   // clear all fields of the structure
   //
   ptsCanMsgV->tuMsgId.ulExt        = 0;

   ptsCanMsgV->tuMsgData.aulLong[0] = 0;
   ptsCanMsgV->tuMsgData.aulLong[1] = 0;

   ptsCanMsgV->ubMsgDLC             = 0;

   ptsCanMsgV->ubMsgCtrl            = 0;

   #if CP_CAN_MSG_TIME == 1
   ptsCanMsgV->tsMsgTime.ulSec1970  = 0;
   ptsCanMsgV->tsMsgTime.ulNanoSec  = 0;
   #endif

   #if CP_CAN_MSG_USER == 1
   ptsCanMsgV->ulMsgUser            = 0;
   #endif
}


//----------------------------------------------------------------------------//
// CpMsgGetData()                                                             //
//                                                                            //
//----------------------------------------------------------------------------//
_U08  CpMsgGetData(_TsCpCanMsg * ptsCanMsgV, _U08 ubPosV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return(0);

   //----------------------------------------------------------------
   // test ubPosV, range from 0 to 7
   //
   if(ubPosV > 7) return (0);

   //----------------------------------------------------------------
   // return data
   //
   return (ptsCanMsgV->tuMsgData.aubByte[ubPosV]);
}


//----------------------------------------------------------------------------//
// CpMsgGetDlc()                                                              //
//                                                                            //
//----------------------------------------------------------------------------//
_U08  CpMsgGetDlc(_TsCpCanMsg * ptsCanMsgV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return(0);

   return(ptsCanMsgV->ubMsgDLC);
}


//----------------------------------------------------------------------------//
// CpMsgGetExtId()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
_U32  CpMsgGetExtId(_TsCpCanMsg * ptsCanMsgV)
{
   _U32  ulExtIdT;

   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return(0);

   //----------------------------------------------------------------
   // mask the lower 29 bits
   //
   ulExtIdT = ptsCanMsgV->tuMsgId.ulExt & CP_MASK_EXT_FRAME;

   return(ulExtIdT);
}


//----------------------------------------------------------------------------//
// CpMsgGetStdId()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
_U16  CpMsgGetStdId(_TsCpCanMsg * ptsCanMsgV)
{
   _U16  uwStdIdT;

   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return(0);

   //----------------------------------------------------------------
   // mask the lower 11 bits
   //
   uwStdIdT = ptsCanMsgV->tuMsgId.uwStd & CP_MASK_STD_FRAME;

   return(uwStdIdT);
}


//----------------------------------------------------------------------------//
// CpMsgIsExtended()                                                          //
//                                                                            //
//----------------------------------------------------------------------------//
_U08  CpMsgIsExtended(_TsCpCanMsg * ptsCanMsgV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return(0);

   if(ptsCanMsgV->ubMsgCtrl & CP_MASK_EXT_BIT)
   {
      return(1);
   }

   return(0);
}


//----------------------------------------------------------------------------//
// CpMsgIsRemote()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
_U08  CpMsgIsRemote(_TsCpCanMsg * ptsCanMsgV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return(0);

   if(ptsCanMsgV->ubMsgCtrl & CP_MASK_RTR_BIT)
   {
      return(1);
   }

   return(0);
}


//----------------------------------------------------------------------------//
// CpMsgSetData()                                                             //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgSetData(_TsCpCanMsg * ptsCanMsgV, _U08 ubPosV, _U08 ubValueV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;

   //----------------------------------------------------------------
   // test ubPosV, range from 0 to 7
   //
   if(ubPosV > 7) return;

   ptsCanMsgV->tuMsgData.aubByte[ubPosV] = ubValueV;
}


//----------------------------------------------------------------------------//
// CpMsgSetDlc()                                                              //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgSetDlc(_TsCpCanMsg * ptsCanMsgV, _U08 ubDlcV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;

   //----------------------------------------------------------------
   // make sure the Data Length Code is in range
   //
   if(ubDlcV > 8) return;

   ptsCanMsgV->ubMsgDLC = ubDlcV;
}


//----------------------------------------------------------------------------//
// CpMsgSetExtId()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgSetExtId(_TsCpCanMsg * ptsCanMsgV, _U32 ulExtIdV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;

   //----------------------------------------------------------------
   // use only lower 29 bits
   //
   ulExtIdV = ulExtIdV & CP_MASK_EXT_FRAME;

   //----------------------------------------------------------------
   // mark as extended frame
   //
   ptsCanMsgV->ubMsgCtrl |= CP_MASK_EXT_BIT;

   ptsCanMsgV->tuMsgId.ulExt = ulExtIdV;
}


//----------------------------------------------------------------------------//
// CpMsgSetRemote()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgSetRemote(_TsCpCanMsg * ptsCanMsgV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;
   ptsCanMsgV->ubMsgCtrl |= CP_MASK_RTR_BIT;
}


//----------------------------------------------------------------------------//
// CpMsgSetStdId()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgSetStdId(_TsCpCanMsg * ptsCanMsgV, _U16 uwStdIdV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;

   //----------------------------------------------------------------
   // use only lower 11 bits
   //
   uwStdIdV = uwStdIdV & CP_MASK_STD_FRAME;

   //----------------------------------------------------------------
   // mark as standard frame
   //
   ptsCanMsgV->ubMsgCtrl &= ~CP_MASK_EXT_BIT;

   ptsCanMsgV->tuMsgId.uwStd = uwStdIdV;
}


//----------------------------------------------------------------------------//
// CpMsgSetTime()                                                             //
//                                                                            //
//----------------------------------------------------------------------------//
void  CpMsgSetTime(_TsCpCanMsg * ptsCanMsgV, _TsCpTime * ptsTimeV)
{
   //----------------------------------------------------------------
   // check for valid pointer
   //
   if(ptsCanMsgV == 0L) return;

   //----------------------------------------------------------------
   // set timestamp value
   //
   ptsCanMsgV->tsMsgTime.ulSec1970 = ptsTimeV->ulSec1970;

   ptsCanMsgV->tsMsgTime.ulNanoSec = ptsTimeV->ulNanoSec;
}

#endif


//----------------------------------------------------------------------------//
// AllocateRcvBuffer()                                                        //
//                                                                            //
//----------------------------------------------------------------------------//
void AllocateRcvBuffer(_TsCpPort * ptsCanPortV)
{
   _TsCpCanMsg tsCanMsgT;    // temporary CAN message

   //----------------------------------------------------------------
   // set message buffer 2 as receive buffer,
   // ID = 211, DLC = 2
   //
   CpMsgClear(&tsCanMsgT);
   CpMsgSetStdId(&tsCanMsgT, 211); // ID = 211
   CpMsgSetDlc(&tsCanMsgT, 2);
   CpCoreBufferInit(ptsCanPortV, &tsCanMsgT,
                    CP_BUFFER_2, CP_BUFFER_DIR_RX);
}


//----------------------------------------------------------------------------//
// AllocateTrmBuffer()                                                        //
//
//----------------------------------------------------------------------------//
void AllocateTrmBuffer(_TsCpPort * ptsCanPortV)
{
   _TsCpCanMsg tsCanMsgT;    // temporary CAN message

   //----------------------------------------------------------------
   // set message buffer 1 as transmit buffer,
   // ID = 120, DLC = 2
   //
   CpMsgClear(&tsCanMsgT);
   CpMsgSetStdId(&tsCanMsgT, 120); // ID = 120
   CpMsgSetDlc(&tsCanMsgT, 2);
   CpCoreBufferInit(ptsCanPortV, &tsCanMsgT,
                    CP_BUFFER_1, CP_BUFFER_DIR_TX);
}

