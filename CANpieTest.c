//****************************************************************************//
// File:          CANpieTest.c                                                //
// Description:   CANpie testing                                              //
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


//-----------------------------------------------------------------------------
// SVN  $Date: 2008-06-10 00:12:01 +0200 (Di, 10 Jun 2008) $
// SVN  $Rev: 137 $ --- $Author: microcontrol $
//-----------------------------------------------------------------------------


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/

#include "CANpieTest.h"

#include "CANpie.h"


//----------------------------------------------------------------------------//
// CpTest_01()                                                                //
//                                                                            //
//----------------------------------------------------------------------------//
void CpTest_01(_TsCpPort * ptsCanPortV)
{

   _TsCpCanMsg    tsCanMsgT;    // temporary CAN message
   _TsCpHdi       tsCanHdiT;
   _TvCpStatus    tvStatusT;
   _U08           aubDataT[8];


   //----------------------------------------------------------------
   // get hardware information
   //
   tvStatusT = CpCoreHDI(ptsCanPortV, &tsCanHdiT);
   if(tvStatusT != CpErr_OK)
   {
      CpTestFail(CP_TEST_BUFFER_TRM_HDI, tvStatusT);
      return;
   }

   if( tsCanHdiT.uwBufferMax == 0 )
   {
      CpTestFail(CP_TEST_BUFFER_TRM_HDI, CpErr_BUFFER);
      return;
   }


   //----------------------------------------------------------------
   // set message buffer 1 as transmit buffer,
   // ID = CP_TEST_TRM_ID, DLC = 8
   //
   CpMsgClear(&tsCanMsgT);
   CpMsgSetStdId(&tsCanMsgT, CP_TEST_TRM_ID_STD);
   CpMsgSetDlc(&tsCanMsgT, 8);
   tvStatusT = CpCoreBufferInit(ptsCanPortV, &tsCanMsgT,
                           CP_BUFFER_1,
                           CP_BUFFER_DIR_TX);

   if(tvStatusT != CpErr_OK)
   {
      CpTestFail(CP_TEST_BUFFER_TRM_HDI, tvStatusT);
      return;
   }

   //----------------------------------------------------------------
   // setup info data (Motorola format)
   // - controller type (16 bit)
   // - buffer max (16 bit)
   // - CAN clock (32 bit)
   //
   aubDataT[0] = (_U08) (tsCanHdiT.uwControllerType >> 8);
   aubDataT[1] = (_U08) (tsCanHdiT.uwControllerType);
   aubDataT[2] = (_U08) (tsCanHdiT.uwBufferMax >> 8);
   aubDataT[3] = (_U08) (tsCanHdiT.uwBufferMax);
   aubDataT[4] = (_U08) (tsCanHdiT.ulCanClock >> 24);
   aubDataT[5] = (_U08) (tsCanHdiT.ulCanClock >> 16);
   aubDataT[6] = (_U08) (tsCanHdiT.ulCanClock >> 8);
   aubDataT[7] = (_U08) (tsCanHdiT.ulCanClock);

   CpCoreBufferSetData(ptsCanPortV, CP_BUFFER_1, &aubDataT[0]);
   CpCoreBufferSend(ptsCanPortV, CP_BUFFER_1);

   //----------------------------------------------------------------
   // report success
   //
   CpTestSuccess(CP_TEST_BUFFER_TRM_HDI);
}


//----------------------------------------------------------------------------//
// CpTest_02()                                                                //
//                                                                            //
//----------------------------------------------------------------------------//
void CpTest_02(_TsCpPort * ptsCanPortV)
{

   _TsCpCanMsg    tsCanMsgT;    // temporary CAN message
   _TsCpHdi       tsCanHdiT;
   _TvCpStatus    tvStatusT;
   _U08           aubDataT[8];
   _U16           uwBufferIdxT;

   //----------------------------------------------------------------
   // get hardware information
   //
   tvStatusT = CpCoreHDI(ptsCanPortV, &tsCanHdiT);
   if(tvStatusT != CpErr_OK)
   {
      CpTestFail(CP_TEST_BUFFER_TRM_ALL, tvStatusT);
      return;
   }

   if( tsCanHdiT.uwBufferMax == 0 )
   {
      CpTestFail(CP_TEST_BUFFER_TRM_ALL, CpErr_BUFFER);
      return;
   }

   //----------------------------------------------------------------
   // setup message buffers
   // the access to message buffer with index 0 must fail
   //
   aubDataT[0] = 0x00;
   aubDataT[1] = 0x01;
   aubDataT[2] = 0x02;
   aubDataT[3] = 0x03;
   aubDataT[4] = 0x04;
   aubDataT[5] = 0x05;
   aubDataT[6] = 0x06;
   aubDataT[7] = 0x07;

   for(uwBufferIdxT = 0; uwBufferIdxT < CP_BUFFER_MAX; uwBufferIdxT++)
   {
      CpMsgClear(&tsCanMsgT);
      CpMsgSetStdId(&tsCanMsgT, (CP_TEST_TRM_ID_STD + uwBufferIdxT));
      CpMsgSetDlc(&tsCanMsgT, uwBufferIdxT % 8);
      tvStatusT = CpCoreBufferInit( ptsCanPortV, &tsCanMsgT,
                                    uwBufferIdxT,
                                    CP_BUFFER_DIR_TX);

      CpCoreBufferSetData(ptsCanPortV, uwBufferIdxT, &aubDataT[0]);

   }

   //----------------------------------------------------------------
   // send all messages
   //
   for(uwBufferIdxT = 0; uwBufferIdxT < CP_BUFFER_MAX; uwBufferIdxT++)
   {
      CpCoreBufferSend(ptsCanPortV, uwBufferIdxT);
   }

   //----------------------------------------------------------------
   // report success
   //
   CpTestSuccess(CP_TEST_BUFFER_TRM_ALL);
}


//----------------------------------------------------------------------------//
// CpTest_03()                                                                //
//                                                                            //
//----------------------------------------------------------------------------//
void CpTest_03(_TsCpPort * ptsCanPortV)
{

   _TsCpCanMsg    tsCanMsgT;    // temporary CAN message
   _TsCpHdi       tsCanHdiT;
   _TvCpStatus    tvStatusT;
   _U08           aubDataT[8];
   _U16           uwBufferIdxT;

   //----------------------------------------------------------------
   // get hardware information
   //
   tvStatusT = CpCoreHDI(ptsCanPortV, &tsCanHdiT);
   if(tvStatusT != CpErr_OK)
   {
      CpTestFail(CP_TEST_BUFFER_TRM_EXT, tvStatusT);
      return;
   }

   if( tsCanHdiT.uwBufferMax == 0 )
   {
      CpTestFail(CP_TEST_BUFFER_TRM_EXT, CpErr_BUFFER);
      return;
   }

   //----------------------------------------------------------------
   // setup message buffers
   // the access to message buffer with index 0 must fail
   //
   aubDataT[0] = 0x00;
   aubDataT[1] = 0x01;
   aubDataT[2] = 0x02;
   aubDataT[3] = 0x03;
   aubDataT[4] = 0x04;
   aubDataT[5] = 0x05;
   aubDataT[6] = 0x06;
   aubDataT[7] = 0x07;

   for(uwBufferIdxT = 0; uwBufferIdxT < CP_BUFFER_MAX; uwBufferIdxT++)
   {
      CpMsgClear(&tsCanMsgT);
      CpMsgSetExtId(&tsCanMsgT, (CP_TEST_TRM_ID_EXT + uwBufferIdxT));
      CpMsgSetDlc(&tsCanMsgT, uwBufferIdxT % 8);
      tvStatusT = CpCoreBufferInit( ptsCanPortV, &tsCanMsgT,
                                    uwBufferIdxT,
                                    CP_BUFFER_DIR_TX);

      CpCoreBufferSetData(ptsCanPortV, uwBufferIdxT, &aubDataT[0]);

   }

   //----------------------------------------------------------------
   // send all messages
   //
   for(uwBufferIdxT = 0; uwBufferIdxT < CP_BUFFER_MAX; uwBufferIdxT++)
   {
      CpCoreBufferSend(ptsCanPortV, uwBufferIdxT);
   }

   //----------------------------------------------------------------
   // report success
   //
   CpTestSuccess(CP_TEST_BUFFER_TRM_EXT);
}


//----------------------------------------------------------------------------//
// CpTestFail()                                                               //
//                                                                            //
//----------------------------------------------------------------------------//
void CpTestFail(_U08 ubTestCaseV, _TvCpStatus tvStatusV)
{
   //----------------------------------------------------------------
   // display failure of test #ubTestCaseV with status 'tvStatusV'
   //
   // e.g. printf("Test %d failed with status %d \n", ubTestCaseV, tvStatusV);
}


//----------------------------------------------------------------------------//
// CpTestSuccess()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
void CpTestSuccess(_U08 ubTestCaseV)
{
   //----------------------------------------------------------------
   // display success of test #ubTestCaseV
   //
   // e.g. printf("Test %d succeeded! \n", ubTestCaseV);
}

