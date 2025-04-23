/* ==============================================================================
System Name:
File Name:
Target:
Author:
Description:
Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED*
=================================================================================  */

//*****************************************************************************
// the includes
//*****************************************************************************

#define _MAIN_H_

#include "sicDCDC35kw_setting.h" //for enum def

//for DAC
#define DAC1_DS()        ( GpioDataRegs.GPBSET.bit.GPIO44 = 1)
#define DAC1_CS()        ( GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1)

#define ADC1_DS()        ( GpioDataRegs.GPASET.bit.GPIO7 = 1)
#define ADC1_CS()        ( GpioDataRegs.GPACLEAR.bit.GPIO7 = 1)

#define EEPROM_WP_EN()      ( GpioDataRegs.GPASET.bit.GPIO14 = 1)
#define EEPROM_WP_DIS()     ( GpioDataRegs.GPACLEAR.bit.GPIO14 = 1)

//#define Buck_EN             (GpioDataRegs.GPADAT.bit.GPIO17)


#define led2ON()         (GpioDataRegs.GPBSET.bit.GPIO57  = 1)
#define led2OFF()        (GpioDataRegs.GPBCLEAR.bit.GPIO57  = 1)
#define led2TOGGLE()     (GpioDataRegs.GPBTOGGLE.bit.GPIO57  = 1)

#define led3ON()         (GpioDataRegs.GPASET.bit.GPIO27  = 1)
#define led3OFF()        (GpioDataRegs.GPACLEAR.bit.GPIO27  = 1)
#define led3TOGGLE()     (GpioDataRegs.GPATOGGLE.bit.GPIO27  = 1)

#define RUN_LED_ON()         (GpioDataRegs.GPASET.bit.GPIO4  = 1)
#define RUN_LED_OFF()        (GpioDataRegs.GPACLEAR.bit.GPIO4  = 1)
#define RUN_LED_TOGGLE()     (GpioDataRegs.GPATOGGLE.bit.GPIO4  = 1)

#define FAULT_LED_ON()         (GpioDataRegs.GPASET.bit.GPIO5  = 1)
#define FAULT_LED_OFF()        (GpioDataRegs.GPACLEAR.bit.GPIO5  = 1)
#define FAULT_LED_TOGGLE()     (GpioDataRegs.GPATOGGLE.bit.GPIO5  = 1)

typedef union
{
    float fValue;
    unsigned long ulValue;
} UNIONFLOAT;


struct Write_Start {         // bits   description
   Uint16 Reserved00:1;             // 0      BIT0
   Uint16 Reserved01:1;             // 1      BIT1
   Uint16 Reserved02:1;             // 2      BIT2
   Uint16 Start     :1;             // 3      BIT3
   Uint16 Stop      :1;             // 4      BIT4
   Uint16 Reserved05:1;             // 5      BIT5
   Uint16 Reserved06:1;             // 6      BIT6
   Uint16 Reserved07:1;             // 7      BIT7
   Uint16 Reserved08:1;             // 8      BIT8
   Uint16 Reserved09:1;             // 9      BIT9
   Uint16 Reserved10:1;            // 10     BIT10
   Uint16 Reserved11:1;            // 11     BIT11
   Uint16 Reserved12:1;            // 12     BIT12
   Uint16 Reserved13:1;            // 13     BIT13
   Uint16 Reserved14:1;            // 14     BIT14
   Uint16 Reserved15:1;            // 15     BIT15
};


typedef union {
   Uint16               all;
   struct Write_Start  bit;
} _Write_Start;

typedef enum
{
    No_Selection                   =   0,
    ElectronicLoad_CV_Mode         =   2,
    ElectronicLoad_CC_Mode         =   4,
    ElectronicLoad_CR_Mode         =   8,
    PowerSupply_CV_Mode            =  16,
    PowerSupply_CC_Mode            =  32,
    Battery_Charg_Discharg_CC_Mode =  64,
    As_a_Battery_CV_Mode           = 128
} eCharge_DisCharge_Mode;



void modbus_parse(void);

struct DIGITAL_BITS {         // bits   description
   Uint16 Bit0:1;             // 0      BIT0
   Uint16 Bit1:1;             // 1      BIT1
   Uint16 Bit2:1;             // 2      BIT2
   Uint16 Bit3:1;             // 3      BIT3
   Uint16 Bit4:1;             // 4      BIT4
   Uint16 Bit5:1;             // 5      BIT5
   Uint16 Bit6:1;             // 6      BIT6
   Uint16 Bit7:1;             // 7      BIT7
   Uint16 Bit8:1;             // 8      BIT8
   Uint16 Bit9:1;             // 9      BIT9
   Uint16 Bit10:1;            // 10     BIT10
   Uint16 Bit11:1;            // 11     BIT11
   Uint16 Bit12:1;            // 12     BIT12
   Uint16 Bit13:1;            // 13     BIT13
   Uint16 Bit14:1;            // 14     BIT14
   Uint16 Bit15:1;            // 15     BIT15
};

typedef union {
   Uint16               all;
   struct DIGITAL_BITS  bit;
} DIGITAL_REG;




typedef enum // code composer studio 개발툴은 _로 시작하는 변수는 와치 변수에서 바뀌지 않음. 컴파일은 됨.
{
    e1000ms  = 100000, // 1hz
    e100ms   = 10000,  // 10hz
    e10ms    = 1000,   // 100hz
    e1ms     = 100,    // 1khz
    e0_5ms   = 50,     // 2khz
    e0_2ms   = 20,     // 5khz
    e100us   = 10,     // 10khz
    e50us    = 5,      // 20khz
    e0_01ms  = 1       // 100khz, 10us
} eConfig_USART_send_period;


#define VOUT_SCALE      ((float)1255.)   // [V]
#define AD7980_REF_SCALE   ((float)5.)   // [V]
#define AD7980_REF_SCALE_REV   ((float)1/5.)   // [V]









#ifdef _MAIN_C_




eCharge_DisCharge_Mode eChargeMode;

int deg_sDacTmp;

Uint16 prabhu_test = 0;
Uint32 mainLoopCount = 0;
// Uint16 IcomTemp, I_com_set = 0;
Uint16 IcomTemp = 0, spi_tx_temp = 0;
float32 I_com_set = 0;
Uint16 Run = 0;
Uint16 dummy;
Uint16 Vo_m=0;
Uint16 dac_count = 0;
Uint16 dac_out_a = 0;
Uint16 dac_out_b = 0;
Uint16 Average_count = 0;
Uint16 Current_Average = 0;



Uint16 dac_a_int = 0;

float32 dutyManualGUI = 0.2;
float32 Icom = 0;
float32 Icom_ss_old = 0;
float32 Icom_ss = 0;
float32 I_com= 0, I_ss_old, Voh1, Vol1, I_ss, I_ss_old2;
float32 V_com = 0;
float32 Voh_com = 0; // high
float32 Vol_com = 0; // low

float32 KI_out_old = 0;
float32 KI_out=0;
float32 Voh_KI_out=0;
float32 Vol_KI_out=0;

float32 In_Temp = 0;
float32 Vo = 0;

float32 Vo_sen_sum = 0;
float32 Vo_Err = 0;
float32 Voh_Err = 0;
float32 Vol_Err = 0;

float32 Vo_err_PI_out = 0;
float32 Voh_err_PI_out = 0;
float32 Vol_err_PI_out = 0;

float32 KP_out = 0;
float32 Voh_KP_out = 0;
float32 Vol_KP_out = 0;

// UI에서 마이너스 전류 지령을 줄 수 없도록 되어있음
// Icom_Pos +전류지령(충전모드)
// Icom_Neg -전류지령(회생모드)
// V_com 전압지령


float Icom_temp;

float32 Icom_Pos = 0, Icom_Pos1 = 0;
float32 Icom_Neg = 0, Icom_Neg1 = 0;

float32 Io_avg = 0;
float Io_ad_avg = 0;
//float32 Io_sen = 0;
Uint32 Io_ad_sum = 0;
Uint32 Io_sen_sum = 0;

float32 Temp_ad_sen = 0, fan_pwm_duty = 0.15, fan_pwm_duty_temp = 0.;
float32 Temp_ad_sen_1 = 0;
float32 Voffset = 0;
float32 Vscale = 0;

float32 Kp = 1;
float32 Ki = 3000;
float32 Tsampl=50E-6;    //Tsampl=50E-6 (50us)

//volatile float32 iL1,iL2,iL3;
volatile float32 Temp_ad;
Uint16 Io_ad = 0;
volatile float32 Vo_ad;

//시험용
Uint16 Test_Temp = 0;
Uint16 Test_Temp1 = 0;
float32 Test_Temp2 = 0;
float Io_sen_real;

float32 I_manual = 0;

UNIONFLOAT Vin_monitor, Vout_monitor, Iout_monitor, uf_resistance, Iout_command, Io_sense1, Io_sense2, Io_sense3, Io_sense4, Io_sense5, Io_sense6, Io_sense7, Io_sense8, Io_sense9, UI_Iout_command, Io_sen, Io_sen_total, Vo_sen_avg;
Uint16 Vout_Reference;
int16  Iout_Reference;

Uint16 V_high_limit, V_low_limit;

Uint16 fan_speed = 0, temperature1 = 10, temperature2 = 20, temperature3 = 40, start_stop = 0;
Uint16 fan_pwm_out_trip = 0, fan_pwm_out_trip_old = 0;

char *msg;
Uint16 SendChar = 65; //'A'
float fADC_voltage, Vo_sen;
Uint16 over_voltage_flag = 0;

Uint32 _100khz_count = 0, __100ms_flag, __1000ms_flag;
Uint16 _10ms_flag, _1ms_flag, _100us_flag, _50us_flag, _0_1ms_count;

float Power;
int16 Iref_temp;

volatile DIGITAL_REG DigitalIn, DigitalIn_old, DigitalIn_old_old;
void Digital_Input( void );
void PI_Controller_high(void);
void PI_Controller_low(void);
void Calculating_current_average_and_monitoring_average(void);
void Calculating_voltage_average_and_monitoring_average(void);
void FAN_pwm_service(void);



Uint16 Board_ID;

eConfig_USART_send_period USART_send_period = e50us;//e0_1ms ;//e0_1ms e1ms;

Uint32 setup_off_timer = 0, setup_on_timer = 0;
Uint16 dab_ok_fault = 0, dab_ok = 0;


#else





extern Uint16 V_high_limit, V_low_limit;
extern eCharge_DisCharge_Mode eChargeMode;
extern Uint16 Vout_Reference;
extern int16  Iout_Reference;

extern char *msg;
extern Uint16 SendChar;

extern float32 In_Temp;
extern float fADC_voltage;

extern float32 Vo, Io_avg;
extern Uint16 start_stop;


extern Uint32 _100khz_count, __100ms_flag, __1000ms_flag;
extern float Power;
extern int16 Iref_temp;
extern Uint16 Board_ID;

extern eConfig_USART_send_period USART_send_period;
#endif
