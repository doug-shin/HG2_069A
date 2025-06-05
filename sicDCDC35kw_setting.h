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

//=============================================================================
// GPIO 디지털 입력 변수들
//=============================================================================
extern Uint16 Board_ID;                    // DIP 스위치로부터 읽은 보드 ID (4비트)
extern Uint16 power_switch;                // 전원 스위치 상태 (GPIO54)

#define ACTIVE              (1) //only use when a DAC board is inserted

//////////////////////////////////////////////////////////////
// Prototype statements for functions found within this file.
void InitEPwm1(void);
void InitEPwm3(void);

void gpio_config(void);     // GPIO configuration
void AdcSetup(void);

void spi_fifo_init(void);   // Initialize the Spi FIFO
void spi_init(void);        // init SPI

void scia_init(void);
void scia_fifo_init(void);

void ReadGpioInputs(void);  // Read GPIO digital inputs
void eCana_config(void);    // CAN configuration


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

