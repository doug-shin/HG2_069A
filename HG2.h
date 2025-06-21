/**
 * @file HG2.h
 * @brief 35kW DC-DC 컨버터 제어 시스템 헤더 파일
 *
 * @details
 * 이 파일은 35kW DC-DC 컨버터 제어 시스템의 모든 변수, 상수, 함수 선언을 포함합니다.
 *
 * @section file_organization 파일 구성
 * 1. 시스템 상수 및 설정
 * 2. 하드웨어 매크로 정의
 * 3. 데이터 타입 정의
 * 4. 변수 선언 (기능별 그룹화)
 * 5. 함수 선언 (카테고리별 분류)
 * 6. 외부 변수 선언
 *
 * File Name: HG2.h
 * Target: TI F28069 Piccolo Microcontroller
 * Compiler: TI C2000 Code Generation Tools
 * Hardware: 35kW DC-DC Converter Control Board
 *
 * @author 개발팀
 * @date 2024
 * @version 2.0
 *
 * @copyright Copyright (c) 2024
 *
 * @note 이 헤더 파일은 메인 소스 파일과 설정 파일에서 공통으로 사용됩니다.
 */

#ifndef _MAIN_H_
#define _MAIN_H_

//*****************************************************************************
// 헤더 파일 포함
//*****************************************************************************
#include "DSP28x_Project.h"
#include "DCLF32.h"
#include "HG2_setting.h"
#include "modbus.h"
#include "protocol.h"

//*****************************************************************************
// System Constants and Configuration
//*****************************************************************************
#define I_MAX (80.0f)       ///< 최대 전류 제한값 (A)
#define MODULE_NUM (1)      ///< 모듈 개수
#define OVER_VOLTAGE (1100) ///< 과전압 보호 임계값 (V)

//*****************************************************************************
// GPIO Macros and Hardware Control
//*****************************************************************************

/**
 * @defgroup GPIO_Macros GPIO 제어 매크로
 * @brief 하드웨어 제어를 위한 GPIO 매크로 정의
 * @{
 */

// for DAC
#define DAC1_DS() (GpioDataRegs.GPBSET.bit.GPIO44 = 1)            ///< DAC1 비활성화
#define DAC1_CS() (GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1)          ///< DAC1 활성화

#define ADC1_DS() (GpioDataRegs.GPASET.bit.GPIO7 = 1)             ///< ADC1 비활성화
#define ADC1_CS() (GpioDataRegs.GPACLEAR.bit.GPIO7 = 1)           ///< ADC1 활성화

#define EEPROM_WP_EN() (GpioDataRegs.GPASET.bit.GPIO14 = 1)       ///< EEPROM 쓰기 보호 활성화
#define EEPROM_WP_DIS() (GpioDataRegs.GPACLEAR.bit.GPIO14 = 1)    ///< EEPROM 쓰기 보호 비활성화

#define BUCK_EN (GpioDataRegs.GPADAT.bit.GPIO17)                  ///< Buck 컨버터 인에이블 상태

#define LED2_ON() (GpioDataRegs.GPBSET.bit.GPIO57 = 1)            ///< LED2 켜기
#define LED2_OFF() (GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1)         ///< LED2 끄기
#define LED2_TOGGLE() (GpioDataRegs.GPBTOGGLE.bit.GPIO57 = 1)     ///< LED2 토글

#define LED3_ON() (GpioDataRegs.GPASET.bit.GPIO27 = 1)            ///< LED3 켜기
#define LED3_OFF() (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1)         ///< LED3 끄기
#define LED3_TOGGLE() (GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1)     ///< LED3 토글

#define LED_RUN_ON() (GpioDataRegs.GPASET.bit.GPIO4 = 1)          ///< RUN LED 켜기
#define LED_RUN_OFF() (GpioDataRegs.GPACLEAR.bit.GPIO4 = 1)       ///< RUN LED 끄기
#define LED_RUN_TOGGLE() (GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1)   ///< RUN LED 토글

#define LED_FAULT_ON() (GpioDataRegs.GPASET.bit.GPIO5 = 1)        ///< FAULT LED 켜기
#define LED_FAULT_OFF() (GpioDataRegs.GPACLEAR.bit.GPIO5 = 1)     ///< FAULT LED 끄기
#define LED_FAULT_TOGGLE() (GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1) ///< FAULT LED 토글

/** @} */

/**
 * @brief float32와 Uint32 간 변환을 위한 공용체
 * @details CAN 통신에서 float 데이터를 32bit 정수로 전송하기 위해 사용
 */
typedef union
{
    float32 f; ///< 부동소수점 값
    Uint32 u;  ///< 32비트 정수 값
} UNIONFLOAT;

// CAN 쉐도우 레지스터 정의
extern volatile struct ECAN_REGS ECanaShadow;

/**
 * @brief 시스템 운전 상태 열거형
 * @details Modbus 및 CAN을 통해 제어되는 시스템 운전/정지 상태
 */
typedef enum
{
    STATE_STOP = 0,   ///< 시스템 정지
    STATE_RUNNING = 1 ///< 시스템 운전
} SYSTEM_STATE;

/**
 * @brief 충전/방전 모드 열거형
 * @details Modbus를 통해 제어 컴퓨터에서 설정되는 운전 모드
 */
typedef enum
{
    No_Selection = 0,                    ///< 모드 선택 안함
    ElectronicLoad_CV_Mode = 2,          ///< 전자부하 정전압 모드
    ElectronicLoad_CC_Mode = 4,          ///< 전자부하 정전류 모드
    ElectronicLoad_CR_Mode = 8,          ///< 전자부하 정저항 모드
    PowerSupply_CV_Mode = 16,            ///< 전원공급 정전압 모드
    PowerSupply_CC_Mode = 32,            ///< 전원공급 정전류 모드
    Battery_Charg_Discharg_CC_Mode = 64, ///< 배터리 충방전 정전류 모드
    As_a_Battery_CV_Mode = 128           ///< 배터리 시뮬레이션 정전압 모드
} eCharge_DisCharge_Mode;

/**
 * @brief USART 전송 주기 열거형
 * @details 통신 주기 설정을 위한 열거형 (현재 미사용)
 */
typedef enum
{
    e1000ms = 100000, ///< 1Hz
    e100ms = 10000,   ///< 10Hz
    e10ms = 1000,     ///< 100Hz
    e1ms = 100,       ///< 1kHz
    e0_5ms = 50,      ///< 2kHz
    e0_2ms = 20,      ///< 5kHz
    e100us = 10,      ///< 10kHz
    e50us = 5,        ///< 20kHz
    e0_01ms = 1       ///< 100kHz, 10us
} eConfig_USART_send_period;

void modbus_parse(void);

#ifdef _MAIN_C_

//=============================================================================
// 1. SYSTEM CONTROL AND STATUS VARIABLES
//=============================================================================
/**
 * @defgroup System_Control_Variables 시스템 제어 및 상태 변수
 * @brief 시스템 운전 상태, 모드 제어 관련 변수들
 * @{
 */

// System State Control
eCharge_DisCharge_Mode eChargeMode = Battery_Charg_Discharg_CC_Mode; ///< 충전/방전 모드 (기본: CC 모드)
SYSTEM_STATE system_state = STATE_STOP;                              ///< 시스템 운전 상태 (정지/운전)
eConfig_USART_send_period USART_send_period = e50us;                 ///< USART 전송 주기 (50us)

// Debug and Monitoring
Uint32 mainLoopCount = 0; ///< 메인 루프 카운터 (디버깅용)

/** @} */

//=============================================================================
// 2. CURRENT CONTROL VARIABLES
//=============================================================================
/**
 * @defgroup Current_Control_Variables 전류 제어 관련 변수
 * @brief 전류 지령, 센싱, 제어 출력 관련 변수들
 * @{
 */

// Current Commands and Control Outputs
float32 I_cmd = 0.0f;                ///< 기본 전류 지령 (Modbus에서 수신)
float32 I_cmd_PI = 0.0f;             ///< PI 제어용 중간 전류 지령값
float32 I_cmd_control_output = 0.0f; ///< 최종 전류 지령값 (DAC 출력용)

// Current Sensing and ADC
Uint16 I_out_ADC = 0;         ///< 현재 전류 ADC 값 (100kHz)
Uint32 I_out_ADC_sum = 0;     ///< 전류 ADC 합계 (5회 평균용)
float32 I_out_ADC_avg = 0.0f; ///< 5회 평균된 ADC 전류값 (20kHz)

// Current Feedback and Monitoring
float32 I_fb_sum = 0.0f; ///< 전류 센서 합계 (장기 평균용)
float32 I_fb_avg = 0.0f; ///< 전류 피드백 평균 (모니터링용)
Uint16 I_cal_cnt = 0;    ///< 전류 계산 카운터

// Soft Start Control
float32 I_ss = 0.0f;          ///< 소프트 스타트 전류 제한
float32 I_ss_prev = 0.0f;     ///< 이전 소프트 스타트 값
float32 I_ss_prev2 = 0.0f;    ///< 이전 소프트 스타트 값 2
float32 I_cmd_ss = 0.0f;      ///< 정상상태 전류 지령
float32 I_cmd_ss_prev = 0.0f; ///< 이전 정상상태 전류 지령

// DAC Output
Uint16 I_cmd_DAC = 0; ///< 전류 지령 DAC 값 (0~4095)

/** @} */

//=============================================================================
// 3. VOLTAGE CONTROL VARIABLES
//=============================================================================
/**
 * @defgroup Voltage_Control_Variables 전압 제어 관련 변수
 * @brief 전압 지령, 센싱, 제어 관련 변수들
 * @{
 */

// Voltage Commands and Limits
float32 V_max_lim = 0.0f; ///< 고전압 지령 (충전 모드용)
float32 V_min_lim = 0.0f; ///< 저전압 지령 (방전 모드용)
float32 Bat_Mean = 0.0f;  ///< 배터리 평균 전압

// Voltage Sensing and ADC
volatile float32 V_out_ADC = 0.0f; ///< 전압 ADC 원시값
float32 V_out = 0.0f;              ///< 출력 전압 (PI 제어용)
float32 V_fb = 0.0f;               ///< 전압 피드백 (센싱용)

// Voltage Feedback and Monitoring
float32 V_fb_sum = 0.0f; ///< 전압 피드백 합계 (장기 평균용)
Uint32 V_cal_cnt = 0;    ///< 전압 계산 카운터

// Voltage Control Errors
float32 V_max_error = 0.0f; ///< 고전압 오차 (충전 모드)
float32 V_min_error = 0.0f; ///< 저전압 오차 (방전 모드)

// PI Control Outputs
float32 V_max_control_output = 0.0f; ///< 고전압 PI 출력 (충전 모드)
float32 V_min_control_output = 0.0f; ///< 저전압 PI 출력 (방전 모드)

// Protection Flags
Uint16 over_voltage_flag = 0; ///< 과전압 보호 플래그

/** @} */

//=============================================================================
// 4. PI CONTROLLER PARAMETERS AND OUTPUTS
//=============================================================================
/**
 * @defgroup PI_Controller_Variables PI 제어기 관련 변수
 * @brief PI 제어기 파라미터, 출력, DCL 컨트롤러 관련 변수들
 * @{
 */

// PI Control Parameters
float32 Kp = 1.0f;       ///< 비례 게인
float32 Ki = 3000.0f;    ///< 적분 게인
float32 Tsampl = 50E-6f; ///< 샘플링 시간 (50us, 20kHz)

// PI Outputs - Proportional Terms
float32 V_max_kP_out = 0.0f; ///< 고전압 비례 출력
float32 V_min_kP_out = 0.0f; ///< 저전압 비례 출력

// PI Outputs - Integral Terms
float32 kI_out_prev = 0.0f;  ///< 이전 적분 출력
float32 V_max_kI_out = 0.0f; ///< 고전압 적분 출력
float32 V_min_kI_out = 0.0f; ///< 저전압 적분 출력

// DCL PI Controllers (TI DCL Library)
DCL_PI dcl_pi_charge;    ///< DCL 충전용 PI 컨트롤러
DCL_PI dcl_pi_discharge; ///< DCL 방전용 PI 컨트롤러
DCL_CSS dcl_css_common;  ///< DCL 공통 지원 구조체

// DCL Control Flags
Uint16 use_dcl_controller = 0; ///< DCL 제어기 사용 플래그 (0: 기존, 1: DCL)

/** @} */

//=============================================================================
// 5. TEMPERATURE SENSING AND FAN CONTROL
//=============================================================================
/**
 * @defgroup Temperature_Control_Variables 온도 제어 관련 변수
 * @brief 온도 센싱 및 팬 PWM 제어 관련 변수들
 * @{
 */

// Temperature Sensing
volatile float32 temp_ADC = 0.0f; ///< 온도 ADC 원시값
float32 temp_in = 0.0f;           ///< 실제 온도값 (°C)
float32 temp_ADC_fb = 0.0f;       ///< 온도 ADC 피드백값
float32 temp_ADC_fb_alt = 0.0f;   ///< 온도 ADC 보조값

// Fan PWM Control
float32 fan_pwm_duty = 0.15f;    ///< 팬 PWM 듀티 사이클 (15~90%)
float32 fan_pwm_duty_tmp = 0.0f; ///< 팬 PWM 듀티 임시값

/** @} */

//=============================================================================
// 6. COMMUNICATION VARIABLES
//=============================================================================
/**
 * @defgroup Communication_Variables 통신 관련 변수
 * @brief Modbus, CAN, RS485 통신 관련 변수들
 * @{
 */

// Modbus Command Interface
Uint16 UI_V_cmd = 0; ///< 전압 기준값 (Modbus에서 수신)
UNIONFLOAT UI_I_cmd; ///< 전류 지령 (Modbus에서 수신)

// CAN Communication Data
UNIONFLOAT I_fb_array[10]; ///< 마스터, 슬레이브 전류 센서 배열 (인덱스 1~9는 슬레이브, 0은 마스터)
UNIONFLOAT I_fb_total;     ///< 총 전류 피드백 (Modbus 전송용)
UNIONFLOAT V_fb_avg;       ///< 평균 전압 (Modbus 전송용)

// Load Configuration
UNIONFLOAT load_resistance; ///< 부하 저항값 (CR 모드용)

/** @} */

//=============================================================================
// 7. TIMING AND CONTROL FLAGS
//=============================================================================
/**
 * @defgroup Timing_Control_Variables 타이밍 제어 관련 변수
 * @brief 시스템 타이밍 제어 및 플래그 관련 변수들
 * @{
 */

// High-Frequency Control Timing
Uint32 control_phase = 0; ///< 100kHz → 20kHz 분주 카운터 (0~4)

// System Flags
Uint16 parse_mb_flag = 0; ///< Modbus 파싱 플래그 (10ms 주기)

// Hardware Status Flags
Uint16 dab_ok_fault = 0; ///< DAB 폴트 상태
Uint16 dab_ok = 0;       ///< DAB OK 상태

/** @} */

//=============================================================================
// 9. FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// A. Interrupt Service Routines (인터럽트 서비스 루틴)
//-----------------------------------------------------------------------------
/**
 * @defgroup ISR_Functions 인터럽트 서비스 루틴
 * @brief 시스템 인터럽트 처리 함수들
 * @{
 */

__interrupt void adc_isr(void);             ///< ADC 변환 완료 인터럽트 (온도/전류/전압)
__interrupt void cpu_timer0_isr(void);      ///< CPU Timer 0 인터럽트 (100kHz, 워치독)
__interrupt void cpu_timer2_isr(void);      ///< CPU Timer 2 인터럽트 (저속 타이밍)
__interrupt void epwm1_isr(void);           ///< ePWM1 인터럽트 (팬 PWM 제어)
__interrupt void epwm3_isr(void);           ///< ePWM3 인터럽트 (100kHz, 메인 제어 루프)
__interrupt void spi_isr(void);             ///< SPI 통신 완료 인터럽트
__interrupt void scia_txFifo_isr(void);     ///< SCI-A 송신 FIFO 인터럽트 (RS485)
__interrupt void ecan0_isr(void);           ///< CAN 인터럽트 (슬레이브 + Protocol)
__interrupt void scibRxReadyISR(void);      ///< SCI-B 수신 인터럽트 (Modbus RTU)
__interrupt void scibTxEmptyISR(void);      ///< SCI-B 송신 인터럽트 (Modbus RTU)
__interrupt void cpuTimer1ExpiredISR(void); ///< CPU Timer 1 인터럽트 (Modbus 타이밍)

/** @} */

//-----------------------------------------------------------------------------
// B. Control Algorithm Functions (제어 알고리즘 함수)
//-----------------------------------------------------------------------------
/**
 * @defgroup Control_Functions 제어 알고리즘 함수
 * @brief PI 제어 및 제어 알고리즘 관련 함수들
 * @{
 */

// Traditional PI Controllers
void PIControlHigh(void);    ///< 고전압 PI 컨트롤러 (충전 모드, I_cmd >= 0)
void PIControlLow(void);     ///< 저전압 PI 컨트롤러 (방전 모드, I_cmd < 0)
void PIControlUnified(void); ///< 통합 PI 컨트롤러 (충전/방전 자동 선택)

// DCL-Based PI Controllers (TI DCL Library)
void InitDCLControllers(void);                                   ///< DCL PI 컨트롤러 초기화
void PIControlDCL(void);                                         ///< DCL 기반 PI 제어 (65% 성능 향상)
extern float32 DCL_runPI_C1(DCL_PI *pi, float32 rk, float32 yk); ///< DCL 어셈블리 함수

/** @} */

//-----------------------------------------------------------------------------
// C. Sensing and Calculation Functions (센싱 및 계산 함수)
//-----------------------------------------------------------------------------
/**
 * @defgroup Sensing_Functions 센싱 및 계산 함수
 * @brief 센서 데이터 처리 및 계산 관련 함수들
 * @{
 */

void ReadGpioInputs(void); ///< GPIO 디지털 입력 읽기 (DIP 스위치, 시작/정지)
void Calc_I_fb_avg(void);  ///< 전류 평균 계산 (10000회 평균)
void Calc_V_fb_avg(void);  ///< 전압 평균 계산 (10000회 평균)

/** @} */

//-----------------------------------------------------------------------------
// D. System Control Functions (시스템 제어 함수)
//-----------------------------------------------------------------------------
/**
 * @defgroup System_Control_Functions 시스템 제어 함수
 * @brief 시스템 제어 관련 함수들
 * @{
 */

void ControlFanPwm(void); ///< 팬 PWM 제어 (온도 비례, 15~90%)

/** @} */

//-----------------------------------------------------------------------------
// E. Communication Functions (통신 함수)
//-----------------------------------------------------------------------------
/**
 * @defgroup Communication_Functions 통신 함수
 * @brief 통신 프로토콜 처리 관련 함수들
 * @{
 */

void ParseModbusData(void);                 ///< Modbus 데이터 파싱 (전류/전압 지령, 모드 설정)
void stra_xmit(Uint8 *buff, Uint16 Length); ///< RS485 문자열 전송 (슬레이브 전류 지령)
void modbus_parse(void);                    ///< Modbus 파싱 (외부 함수)

/** @} */

//-----------------------------------------------------------------------------
// F. System Initialization Functions (시스템 초기화 함수)
//-----------------------------------------------------------------------------
/**
 * @defgroup Init_Functions 시스템 초기화 함수
 * @brief 시스템 초기화 관련 함수들
 * @{
 */

void InitEPwm1(void);   ///< ePWM1 초기화 (팬 PWM, 10kHz)
void InitEPwm3(void);   ///< ePWM3 초기화 (메인 제어 타이밍, 100kHz)
void SpiInit(void);     ///< SPI 초기화 (ADC/DAC 통신, 11.25MHz)
void eCanaConfig(void); ///< CAN 설정 (슬레이브 피드백, 500kbps)

/** @} */

#else // ifndef _MAIN_C_

//=============================================================================
// 10. EXTERNAL VARIABLE DECLARATIONS
//=============================================================================
/**
 * @brief 다른 파일에서 HG2.h를 include할 때 사용하는 외부 변수 선언
 * @details 컴파일 시 중복 선언을 방지하기 위해 extern으로 선언
 */

//-----------------------------------------------------------------------------
// System Control Variables
//-----------------------------------------------------------------------------
extern eCharge_DisCharge_Mode eChargeMode;          ///< 운전 모드
extern SYSTEM_STATE system_state;                   ///< 시스템 운전 상태
extern eConfig_USART_send_period USART_send_period; ///< USART 전송 주기

//-----------------------------------------------------------------------------
// Sensing Data Variables
//-----------------------------------------------------------------------------
extern float32 temp_in;  ///< 온도 센서값 (°C)
extern float32 V_out;    ///< 출력 전압 (V)
extern Uint32 V_cal_cnt; ///< 전압 계산 카운터

//-----------------------------------------------------------------------------
// Control Variables
//-----------------------------------------------------------------------------
extern float32 I_cmd_PI; ///< PI 제어용 전류 지령값 (A)

//-----------------------------------------------------------------------------
// DCL Controller Variables
//-----------------------------------------------------------------------------
extern DCL_PI dcl_pi_charge;      ///< DCL 충전용 PI 컨트롤러
extern DCL_PI dcl_pi_discharge;   ///< DCL 방전용 PI 컨트롤러
extern DCL_CSS dcl_css_common;    ///< DCL 공통 지원 구조체
extern Uint16 use_dcl_controller; ///< DCL 제어기 사용 플래그 (0: 기존, 1: DCL)

#endif //_MAIN_C_

#endif //_MAIN_H_
