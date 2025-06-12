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

#ifndef _MAIN_H_
#define _MAIN_H_

#include "sicDCDC35kw_setting.h" //for enum def

//*****************************************************************************
// System Constants and Configuration
//*****************************************************************************
#define I_MAX (80)                    // 최대 전류 제한값 (A)
#define MODULE_NUM (1)                // 모듈 개수
#define OVER_VOLTAGE (1100)           // 과전압 보호 임계값 (V)

#define MON_MAXCNT        (10000.)    // 모니터링 카운트 (평균 계산용)
#define MON_MAXCNT_REV    ((float)((1)/(MON_MAXCNT)))  // 평균 계산 최적화용

//*****************************************************************************
// GPIO Macros and Hardware Control
//*****************************************************************************

//for DAC
#define DAC1_DS()          (GpioDataRegs.GPBSET.bit.GPIO44 = 1)
#define DAC1_CS()          (GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1)

#define ADC1_DS()          (GpioDataRegs.GPASET.bit.GPIO7 = 1)
#define ADC1_CS()          (GpioDataRegs.GPACLEAR.bit.GPIO7 = 1)

#define EEPROM_WP_EN()     (GpioDataRegs.GPASET.bit.GPIO14 = 1)
#define EEPROM_WP_DIS()    (GpioDataRegs.GPACLEAR.bit.GPIO14 = 1)

#define BUCK_EN            (GpioDataRegs.GPADAT.bit.GPIO17)

#define LED2_ON()          (GpioDataRegs.GPBSET.bit.GPIO57 = 1)
#define LED2_OFF()         (GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1)
#define LED2_TOGGLE()      (GpioDataRegs.GPBTOGGLE.bit.GPIO57 = 1)

#define LED3_ON()          (GpioDataRegs.GPASET.bit.GPIO27 = 1)
#define LED3_OFF()         (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1)
#define LED3_TOGGLE()      (GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1)

#define RUN_LED_ON()       (GpioDataRegs.GPASET.bit.GPIO4 = 1)
#define RUN_LED_OFF()      (GpioDataRegs.GPACLEAR.bit.GPIO4 = 1)
#define RUN_LED_TOGGLE()   (GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1)

#define FAULT_LED_ON()     (GpioDataRegs.GPASET.bit.GPIO5 = 1)
#define FAULT_LED_OFF()    (GpioDataRegs.GPACLEAR.bit.GPIO5 = 1)
#define FAULT_LED_TOGGLE() (GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1)

// float32 <-> Uint32 변환용 union 타입 정의
typedef union {
    float32 f;
    Uint32 u;
} UNIONFLOAT;

// CAN 쉐도우 레지스터 정의
extern volatile struct ECAN_REGS ECanaShadow;

// 충전/방전 모드 열거형
typedef enum {
    No_Selection                   =   0,
    ElectronicLoad_CV_Mode         =   2,
    ElectronicLoad_CC_Mode         =   4,
    ElectronicLoad_CR_Mode         =   8,
    PowerSupply_CV_Mode            =  16,
    PowerSupply_CC_Mode            =  32,
    Battery_Charg_Discharg_CC_Mode =  64,
    As_a_Battery_CV_Mode           = 128
} eCharge_DisCharge_Mode;

// USART 전송 주기 열거형
typedef enum {
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

void modbus_parse(void);

#ifdef _MAIN_C_

//=============================================================================
// 1. System Control and Status Variables
//=============================================================================
eCharge_DisCharge_Mode eChargeMode;
Uint16 mainLoopCount = 0;
Uint16 Run = 0;
Uint16 start_stop = 0;
eConfig_USART_send_period USART_send_period = e50us;

//=============================================================================
// 2. Current Control and Sensing Variables
//=============================================================================
// Current Commands (전류 지령)
float32 I_com_set = 0;           // Current command set value
float32 I_com = 0;               // Current command
float32 Icom = 0;                // Current command (실제 사용)
float32 Icom_Pos = 0, Icom_Pos1 = 0;  // +전류지령(충전모드)
float32 Icom_Neg = 0, Icom_Neg1 = 0;  // -전류지령(회생모드)
float32 I_manual = 0;            // Manual current command

// Current Sensing and Feedback  
float32 currentAvg = 0;              // Average output current (이전 Io_avg)
float32 currentSensorSum = 0;        // Current sensor sum for monitoring (이전 Io_sen_sum)
Uint32 currentAdcSum = 0;            // ADC sum for averaging (이전 Io_ad_sum)
Uint16 currentAdc = 0;               // Current ADC value (이전 Io_ad)
Uint16 currentAvgCount = 0;          // Current averaging counter (이전 Current_Average)

// PI Controller Current Variables
float32 Icom_ss_old = 0;         // Current steady state old value
float32 Icom_ss = 0;             // Current steady state
float32 I_ss_old, I_ss, I_ss_old2;  // Current steady state variables
float I_com_1;                   // Current command intermediate value (PI 제어용)

//=============================================================================
// 3. Voltage Control and Sensing Variables  
//=============================================================================
// Voltage Commands (전압 지령)
float32 V_com = 0;               // Voltage command
float32 Voh_com = 0;             // High voltage command
float32 Vol_com = 0;             // Low voltage command
float32 Bat_Mean = 0;            // Battery Mean voltage

// Voltage Sensing and Feedback
float32 Vo = 0;                  // Output voltage
volatile float32 Vo_ad;          // Voltage ADC value
float32 Vo_sen_sum = 0;          // Voltage sensor sum
float32 Voffset = 0;             // Voltage offset
float32 Vscale = 0;              // Voltage scale

// Voltage Monitoring Variables
unsigned int MonitoringCount = 0; // Voltage monitoring counter
float Vo_sen_sum_mon = 0.;       // Voltage sensor sum for monitoring
float Vo_Mean = 0.;              // Voltage mean value

// PI Controller Voltage Variables
float32 Vo_Err = 0;              // Voltage error
float32 Voh_Err = 0;             // High voltage error 
float32 Vol_Err = 0;             // Low voltage error
float32 Vo_err_PI_out = 0;       // Voltage error PI output
float32 Voh_err_PI_out = 0;      // High voltage error PI output
float32 Vol_err_PI_out = 0;      // Low voltage error PI output

// Reference Values
Uint16 Vout_Reference;           // Voltage reference
int16  Iout_Reference;           // Current reference
Uint16 V_high_limit, V_low_limit; // Voltage limits

//=============================================================================
// 4. PI Controller Parameters and Outputs
//=============================================================================
// PI Parameters
float32 Kp = 1;                  // Proportional gain
float32 Ki = 3000;               // Integral gain  
float32 Tsampl = 50E-6;          // Sampling time (50us)

// PI Outputs - Proportional Term
float32 KP_out = 0;              // General KP output
float32 Voh_KP_out = 0;          // High voltage KP output
float32 Vol_KP_out = 0;          // Low voltage KP output

// PI Outputs - Integral Term
float32 KI_out_old = 0;          // Integral output old value
float32 KI_out = 0;              // General integral output
float32 Voh_KI_out = 0;          // High voltage integral output
float32 Vol_KI_out = 0;          // Low voltage integral output

//=============================================================================
// 5. Temperature Sensing and Fan Control
//=============================================================================
// Temperature Sensing (온도 센싱)
float32 In_Temp = 0;             // Input temperature (실제 온도값)
volatile float32 Temp_ad;        // Temperature ADC raw value
float32 Temp_ad_sen = 0;         // Temperature ADC sensor value
float32 Temp_ad_sen_1 = 0;       // Temperature ADC sensor 1

// Fan Control
float32 fan_pwm_duty = 0.15;     // Fan PWM duty cycle
float32 fan_pwm_duty_temp = 0.;  // Fan PWM duty temporary calculation

//=============================================================================
// 6. DAC and SPI Control Variables
//=============================================================================
int deg_sDacTmp;                 // DAC temporary variable (임시 변수)

// SPI Communication
Uint16 IcomTemp = 0, spi_tx_temp = 0;  // SPI temporary variables (임시 변수)

//=============================================================================  
// 7. Timing and Control Flags
//=============================================================================
// High frequency counters and flags
Uint32 controlPhase = 0;        // 100kHz counter
Uint16 _10ms_flag, _1ms_flag, _100us_flag, _50us_flag, _0_1ms_count; // Timing flags
Uint32 __100ms_flag, __1000ms_flag; // Low frequency flags

// Setup timers
Uint32 setup_off_timer = 0, setup_on_timer = 0;
Uint16 dab_ok_fault = 0, dab_ok = 0;

//=============================================================================
// 8. Digital I/O and Status Variables  
//=============================================================================

Uint16 dummy;                    // Dummy variable
Uint16 Vo_m = 0;                 // Voltage measurement
Uint16 Average_count = 0;        // General averaging counter
Uint16 over_voltage_flag = 0;    // Over voltage flag


//=============================================================================
// 9. Communication and Monitoring Variables
//=============================================================================
// Modbus and Communication
UNIONFLOAT loadResistance, uiCurrentCommand;
UNIONFLOAT currentSense[10];        // 전류 센서 배열 (인덱스 0은 사용 안함, 1~9 사용)
UNIONFLOAT currentSensor, totalCurrentSensor, voltageSensorAvg;

// Serial Communication
float fADC_voltage, Vo_sen;      // ADC voltage variables

//=============================================================================
// 10. Calculation and Control Variables  
//=============================================================================
// Calculation Variables
float currentCmdTemp;            // Current command temporary calculation (임시 계산 변수)
float32 Voh1, Vol1;              // Voltage high/low temporary values (임시값)
float Power;                     // Power calculation
int16 irefTemp;                  // Current reference temporary (임시 변수)

// Voltage Monitoring Variables (TI style naming)
unsigned int voltageCount = 0;   // Voltage monitoring counter
float voltageSumMonitor = 0.;    // Voltage sensor sum for monitoring
float voltageMean = 0.;          // Voltage mean value

//=============================================================================
// 11. Function Declarations
//=============================================================================

//-----------------------------------------------------------------------------
// A. Interrupt Service Routines (인터럽트 서비스 루틴)
//-----------------------------------------------------------------------------
__interrupt void adc_isr(void);                       // ADC 인터럽트 서비스 루틴
__interrupt void cpu_timer0_isr(void);                // CPU Timer 0 인터럽트 (100kHz)
__interrupt void cpu_timer2_isr(void);                // CPU Timer 2 인터럽트 (1 Second)
__interrupt void epwm1_isr(void);                     // ePWM1 인터럽트
__interrupt void epwm3_isr(void);                     // ePWM3 인터럽트 (100kHz)
__interrupt void spi_isr(void);                       // SPI 인터럽트
__interrupt void scia_txFifo_isr(void);               // SCI-A 송신 FIFO 인터럽트
__interrupt void ecan0_isr(void);                     // CAN 인터럽트

__interrupt void scibRxReadyISR(void);                // SCI-B 수신 인터럽트
__interrupt void scibTxEmptyISR(void);                // SCI-B 송신 인터럽트
__interrupt void cpuTimer1ExpiredISR(void);           // CPU Timer 1 만료 인터럽트

//-----------------------------------------------------------------------------
// B. Control Algorithm Functions (제어 알고리즘 함수)
//-----------------------------------------------------------------------------
void PIControlHigh(void);                             // 고전압 PI 컨트롤러
void PIControlLow(void);                              // 저전압 PI 컨트롤러
void PIControlUnified(void);                          // 통합 PI 컨트롤러 (충전/방전 자동 선택)

//-----------------------------------------------------------------------------
// C. Sensing and Calculation Functions (센싱 및 계산 함수)
//-----------------------------------------------------------------------------
void ReadGpioInputs(void);                         // GPIO 디지털 입력 읽기
void CalcCurrentAverage(void);                     // 전류 평균 계산
void CalcVoltageAverage(void);                     // 전압 평균 계산

//-----------------------------------------------------------------------------
// D. System Control Functions (시스템 제어 함수)
//-----------------------------------------------------------------------------
void ControlFanPwm(void);                          // 팬 PWM 서비스

//-----------------------------------------------------------------------------
// E. Communication Functions (통신 함수)
//-----------------------------------------------------------------------------
void ParseModbusData(void);                        // Modbus 데이터 파싱
void stra_xmit(Uint8 *buff, Uint16 Length);        // 문자열 전송

//-----------------------------------------------------------------------------
// F. System Initialization Functions (시스템 초기화 함수)
//-----------------------------------------------------------------------------
void InitEPwm1(void);                       // ePWM1 초기화
void InitEPwm3(void);                       // ePWM3 초기화
void SpiInit(void);                                // SPI 초기화
void eCanaConfig(void);                            // CAN 설정

//-----------------------------------------------------------------------------
// G. Protocol and Communication Functions (프로토콜 및 통신 함수)
//-----------------------------------------------------------------------------
void InitProtocol(void);                           // 프로토콜 초기화
void ProcessCANCommand(Uint32 rxMbox, Uint32 txMbox);  // CAN 명령 처리
void UpdateCANFeedbackValues(void);                // CAN 피드백 값 업데이트
void TransitionToRunning(void);                    // 운전 상태로 전환
void TransitionToIdle(void);                       // 대기 상태로 전환

#else

//=============================================================================
// External Variable Declarations
//=============================================================================
extern Uint16 V_high_limit, V_low_limit;
extern eCharge_DisCharge_Mode eChargeMode;
extern Uint16 Vout_Reference;
extern int16  Iout_Reference;

extern float32 In_Temp;
extern float fADC_voltage;
extern float32 Vo, currentAvg;          // Output voltage and current average
extern Uint16 start_stop;

extern Uint32 controlPhase, __100ms_flag, __1000ms_flag;
extern float Power;
extern int16 irefTemp;
extern eConfig_USART_send_period USART_send_period;

// Additional External Declarations
extern float I_com_1;                              // Current command intermediate value
extern unsigned int voltageCount;                  // Voltage monitoring counter
extern float voltageMean;                          // Voltage mean value
extern Uint16 Run;                                 // System run state

#endif   //_MAIN_C_

#endif   //_MAIN_H_
