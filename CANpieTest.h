//****************************************************************************//
// File:          CANpieTest.h                                                //
// Description:   CANpie testing functions                                    //
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
// 13.03.2008  Initial version                                                //
//                                                                            //
//****************************************************************************//


#ifndef  _CP_TEST_H_
#define  _CP_TEST_H_


//-----------------------------------------------------------------------------
// SVN  $Date: 2008-07-04 23:03:11 +0200 (Fr, 04 Jul 2008) $
// SVN  $Rev: 140 $ --- $Author: microcontrol $
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
/*!
** \file    CANpieTest.h
** \brief   CANpie test functions
**
** The CANpie test functions are used to verify the driver functionality
** on the CAN controller.
** <p>
** The following tests need to be defined:
** <ul>
** <li>remote frame reception / transmission
** <li>test for autobaud detection
** <li>test for error detection
** </ul>
*/


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/

#include "CANpie.h"



/*----------------------------------------------------------------------------*\
** Definitions                                                                **
**                                                                            **
\*----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------*/
/*!
** \def  CP_TEST_TRM_ID_STD
**
** Defines the initial identifier that is used for Standard Frame
** message transmission.
*/
#define  CP_TEST_TRM_ID_STD   0x100

/*-------------------------------------------------------------------*/
/*!
** \def  CP_TEST_TRM_ID_EXT
**
** Defines the initial identifier that is used for Extended Frame
** message transmission.
*/
#define  CP_TEST_TRM_ID_EXT   0x180A0100

/*-------------------------------------------------------------------*/
/*!
** \def  CP_TEST_RCV_ID_STD
**
** Defines the initial identifier that is used for Standard Frame
** message reception.
*/
#define  CP_TEST_RCV_ID_STD   0x200

/*-------------------------------------------------------------------*/
/*!
** \def  CP_TEST_RCV_ID_EXT
**
** Defines the initial identifier that is used for Extended Frame
** message reception.
*/
#define  CP_TEST_RCV_ID_EXT   0x182C0200


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_TEST_CASE
** \brief   Defines test cases
**
** The values of the enumeration CP_TEST_CASE are used to start and identify
** a specific test funtion.
*/
enum CP_TEST_CASE {

   /*!
   ** Test case #1: check hardware description
   */
   CP_TEST_BUFFER_TRM_HDI = 1,

   /*!
   ** Test case #2: send on multiple buffers
   */
   CP_TEST_BUFFER_TRM_ALL,

   /*!
   ** Test case #3: send Extended Frames
   */
   CP_TEST_BUFFER_TRM_EXT,

   /*!
   ** Test case #4: mirror Standard Frames
   */
   CP_TEST_BUFFER_MIRROR_STD,

   /*!
   ** Test case #5: mirror Extended Frames
   */
   CP_TEST_BUFFER_MIRROR_EXT,

   /*!
   ** Test case #6: count Standard Frames
   */
   CP_TEST_BUFFER_COUNT_STD

};

/*----------------------------------------------------------------------------*\
** Function prototypes                                                        **
**                                                                            **
\*----------------------------------------------------------------------------*/


/*!
** \brief   Report test failure
**
**
**
*/
void CpTestFail(_U08 ubTestCaseV, _TvCpStatus tvStatusV);


/*!
** \brief   Report test success
**
**
**
*/
void CpTestSuccess(_U08 ubTestCaseV);


/*!
** \brief   Test number of message buffers
** \param   ptsPortV       Pointer to CAN port structure
**
** The function evaluates the total number of available message buffers
** and reports the result via a CAN message. The identifier is defined
** by #CP_TEST_TRM_ID_STD.
*/
void CpTest_01(_TsCpPort * ptsPortV);


/*!
** \brief   Test CAN 2.0A message transmission on multiple buffers
** \param   ptsPortV       Pointer to CAN port structure
**
** The function sends CAN 2.0A messages on all available message buffers.
** The first message buffer uses the identifier #CP_TEST_TRM_ID_STD, for
** the following an offset of 1 is added.
*/
void CpTest_02(_TsCpPort * ptsPortV);


/*!
** \brief   Test CAN 2.0B message transmission on multiple buffers
** \param   ptsPortV       Pointer to CAN port structure
**
** The function sends CAN 2.0B messages on all available message buffers.
** The first message buffer uses the identifier #CP_TEST_TRM_ID_EXT, for
** the following an offset of 1 is added.
*/
void CpTest_03(_TsCpPort * ptsPortV);


/*!
** \brief   Test CAN 2.0A message reception
** \param   ptsPortV       Pointer to CAN port structure
**
** The function receives a CAN 2.0A message on the identifier defined
** by #CP_TEST_RCV_ID_STD and mirrors the incoming data on the transmit
** message with the identifier #CP_TEST_TRM_ID_STD.
*/
void CpTest_04(_TsCpPort * ptsPortV);


/*!
** \brief   Test CAN 2.0B message reception
** \param   ptsPortV       Pointer to CAN port structure
**
** The function receives a CAN 2.0B message on the identifier defined
** by #CP_TEST_RCV_ID_EXT and mirrors the incoming data on the transmit
** message with the identifier #CP_TEST_TRM_ID_EXT.
*/
void CpTest_05(_TsCpPort * ptsPortV);


/*!
** \brief   Test CAN 2.0A message reception on multiple buffers
** \param   ptsPortV       Pointer to CAN port structure
**
** The function receives CAN 2.0A messages on multiple identifiers.
** The total number of received messages is counted and reported.
**
*/
void CpTest_06(_TsCpPort * ptsPortV);


#endif   // _CP_TEST_H_
