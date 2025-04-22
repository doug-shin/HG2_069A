/*
 * pfc3phvienna30kw_isr.h
 *
 *  Created on: May 29, 2019
 *      Author: joas1
 */

#ifndef SICDCDC40KW_ISR_H_
#define SICDCDC40KW_ISR_H_



#endif /* SICDCDC40KW_ISR_H_ */

//#include "sicDCDC40kw_setting.h" //for enum def

//for FLASH operation
// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped using
// the linker cmd file.
//#pragma CODE_SECTION(epwm1_isr, "ramfuncs");
//#pragma CODE_SECTION(epwm2_isr, "ramfuncs");
////////////////////////////////////////////////////

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void  adc_isr(void);
__interrupt void cla1_task1_isr( void );
__interrupt void Cla1Task1();

//extern static inline void current_controller ( void );

inline void  Dac_out(void);

//inline void spi_xmit_TLV5638(Uint16 sdataA,Uint16 sdataB);

//inline float LPF_50Hz(float v_rec);

// Global variables used in this example
volatile Uint32  EPwm1TimerIntCount;
volatile Uint32  EPwm2TimerIntCount;


//for ADC
volatile Uint16 LoopCount;
volatile Uint16 ConversionCount;
//float Voltage1[200];
//float Voltage2[200];
//float Voltage3[200];
//float Current1[200];
//float Current2[200];

//debug SPI
Uint16 errorSPI;
Uint16 volatile Dacout_count;

//for simulated sine waves
Uint16 nstep;
//for plotting on DAC
Uint16 volatile test_val1;
Uint16 volatile test_val2;

float vBusDiff;

//voltage controller
typedef struct   {
   float   Ref;
   float   Fdbk;
   float   Errn;
   float   Errn_prev;
   float   Out;
   float   Out_prev;

}GV_VARS;


//voltage controller parameters
volatile GV_VARS Gv_vars;

//current controller parameters
volatile GI_VARS Gi_vars1;
volatile GI_VARS Gi_vars2;
volatile GI_VARS Gi_vars3;


Uint16 closeGvLoop,firstTimeGvLoop;
volatile float vBusRefSlewed,vBusRef;

volatile float iLref;
volatile Uint16 Iac_OC,DC_link_OV;

enum enum_boardStatus boardStatus;
Uint16 board_fault = 0;

volatile Uint16 timer1ms,timer1ms_flag;

//for DAC debugging
extern volatile Uint16 AdcCS0Dac,AdcCS1Dac,AdcCS2Dac,AdcCS3Dac;
extern volatile Uint16 test_val1,test_val2;
extern volatile Uint16 Dacout_count;

extern volatile Uint16  timer100ms;
extern volatile FAULT_TYPE fault;
extern volatile Uint16 faultAckGUI;
