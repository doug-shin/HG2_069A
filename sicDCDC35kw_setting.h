/* ==============================================================================
System Name:

File Name:      {ProjectName}-Settings.h

Target:

Author: Manish Bhardwaj, C2000 Systems Solutions Group

Description:

Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED*
=================================================================================  */
#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

#ifdef __cplusplus

extern "C" {
#endif
#include "DSP28x_Project.h" //for basic variable types


#define ACTIVE              (1) //only use when a DAC board is inserted

//////////////////////////////////////////////////////////////
// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);

void Gpio_select(void);
void AdcSetup(void);

void spi_fifo_init(void);   // Initialize the Spi FIFO
void spi_init(void);        // init SPI

void scia_init(void);
void scia_fifo_init(void);


#define PWM_PERIOD_10k              4500  //(FAN)        // TBPRD = HSCLK/(2*freq) = 90M/(2*10k) = 4500
#define PWM_PERIOD_100k              900          // TBPRD = HSCLK/(2*freq) = 90M/(2*10k) = 4500

#define EPWM_DB                 27            // deadtime = 300n --> DB = 300n/(1/HSCLK) = 0.3u*90M = 27

#define DUTY1_INIT      PWM_PERIOD_10k * 0.0
#define DUTY2_INIT      PWM_PERIOD_10k * 0.0
#define DUTY3_INIT      PWM_PERIOD_10k * 0.0

#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif //_PROJSETTINGS_H

