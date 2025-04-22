/*
 * sicDCDC40kw_current_controller.h
 *
 *  Created on: Jun 26, 2019
 *      Author: joas1
 */

#ifndef SICDCDC40KW_CURRENT_CONTROLLER_H_
#define SICDCDC40KW_CURRENT_CONTROLLER_H_



#endif /* SICDCDC40KW_CURRENT_CONTROLLER_H_ */


//###########################################################################
// Description:
//
// Declare shared memory variables and assign them to specific CLA-accessible
// memory locations
//
//! \addtogroup f2806x_example_cla_list
//! \b Memory \b Allocation \n
//!  - CLA1 to CPU Message RAM
//!    - z - Result of the matrix multiplication
//!  - CPU to CLA1 Message RAM
//!    - x - 3X3 Input Matrix
//!    - y - 3X3 Input Matrix
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"
// Include the test header file whose name is based on the test name
// which is defined by the macro TEST on the command line
//#include XSTRINGIZE(XCONCAT(TEST_NAME,_shared.h))
#include "sicDCDC35kw_setting.h"


//GLobal Data
 //Ensure that all data is placed in the data rams

//Task 1 (C) Variables
static inline void current_controller ( void );

float v1Meas,v2Meas,v3Meas;
float vRmsMeasAvg;
float vBusHalfMeas,vBusMeas;

float vBus_command,iLref_actual;

Uint16 closeGiLoop;


Uint16 closeGsLoop;

float Gs_Out;


Uint16 PWM_trip;

float OCP_trip_threshold;

Uint16 protection_enable;



PARA_OVP para_OVP;




float iL1Meas;

float iL2Meas;

float iL3Meas;

float iL1Ref;

float iL2Ref;

float iL3Ref;

float duty1;

float duty2;

float duty3;

float duty1PU;

float duty2PU;

float duty3PU;

float vcon1_temp;

float vcon2_temp;

float vcon3_temp;


float iL1_err;

float iL2_err;

float iL3_err;


float Gi1_Out_display;

float Gi2_Out_display;

float Gi3_Out_display;



PARA_OCP para_OCP;


//#pragma DATA_SECTION(vin_peak_limit,"Cla1ToCpuMsgRAM")
//float vin_peak_limit;
//#pragma DATA_SECTION(vin_RMS_limit,"Cla1ToCpuMsgRAM")
//float vin_RMS_limit;
//#pragma DATA_SECTION(vBusOVP,"Cla1ToCpuMsgRAM")
//float vBusOVP;
//
//#pragma DATA_SECTION(vin_too_high ,"Cla1ToCpuMsgRAM")
//Uint16 vin_too_high;




Uint16 indexCLA;


Uint16 flag_CLA;

//Task 2 (C) Variables

//Task 3 (C) Variables

//Task 4 (C) Variables

//Task 5 (C) Variables

//Task 6 (C) Variables

//Task 7 (C) Variables

//Task 8 (C) Variables

//Common (C) Variables


