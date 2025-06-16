/**
 * @file sicDCDC35kw.h
 * @brief 35kW DC-DC 컨버터 제어 시스템 헤더 파일
 * 
 * @details 시스템 구조:
 * - 마스터/슬레이브 구조의 병렬 운전 시스템
 * - 제어 컴퓨터 → 마스터: SCI Modbus RTU (38400bps, 전류/전압 지령)
 * - 마스터 → 슬레이브: SCI 485 (5.625Mbps, 전류지령 브로드캐스팅)
 * - 슬레이브 → 마스터: CAN (500kbps, 전류 피드백)
 * - 마스터 ↔ 제어 컴퓨터: CAN Protocol (500kbps, 상태 보고)
 * 
 * @author Original + Protocol Integration
 * @date 2025.02.26
 */

/* ==============================================================================
System Name: 35kW DC-DC Converter Control System
File Name: sicDCDC35kw.h
Target: TMS320F28069
Author: Original + Protocol Integration
Description: 
- Master/Slave parallel operation system
- Control Computer → Master: SCI Modbus RTU (current/voltage commands)
- Master → Slaves: SCI 485 broadcasting (current commands)
- Slaves → Master: CAN feedback (current sensing)
- Master ↔ Control Computer: CAN Protocol (status reporting)
Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED*
=================================================================================  */

//*****************************************************************************
// the includes
//*****************************************************************************

#ifndef _MAIN_H_
#define _MAIN_H_

#include <string.h>
#include <math.h>
#include "DSP28x_Project.h"
#include "DCLF32.h"
#include "sicDCDC35kw_setting.h"
#include "modbus.h"
#include "protocol.h"

//*****************************************************************************
// System Constants and Configuration
//*****************************************************************************
#define I_MAX (80.0f)                 ///< 최대 전류 제한값 (A)
#define MODULE_NUM (1)                ///< 모듈 개수
#define OVER_VOLTAGE (1100)           ///< 과전압 보호 임계값 (V)

#define MON_MAXCNT        (10000.)    ///< 모니터링 카운트 (평균 계산용)
#define MON_MAXCNT_REV    ((float)((1)/(MON_MAXCNT)))  ///< 평균 계산 최적화용

//*****************************************************************************
// GPIO Macros and Hardware Control
//*****************************************************************************

/**
 * @defgroup GPIO_Macros GPIO 제어 매크로
 * @brief 하드웨어 제어를 위한 GPIO 매크로 정의
 * @{
 */

//for DAC
#define DAC1_DS()          (GpioDataRegs.GPBSET.bit.GPIO44 = 1)      ///< DAC1 비활성화
#define DAC1_CS()          (GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1)    ///< DAC1 활성화

#define ADC1_DS()          (GpioDataRegs.GPASET.bit.GPIO7 = 1)       ///< ADC1 비활성화
#define ADC1_CS()          (GpioDataRegs.GPACLEAR.bit.GPIO7 = 1)     ///< ADC1 활성화

#define EEPROM_WP_EN()     (GpioDataRegs.GPASET.bit.GPIO14 = 1)      ///< EEPROM 쓰기 보호 활성화
#define EEPROM_WP_DIS()    (GpioDataRegs.GPACLEAR.bit.GPIO14 = 1)    ///< EEPROM 쓰기 보호 비활성화

#define BUCK_EN            (GpioDataRegs.GPADAT.bit.GPIO17)          ///< Buck 컨버터 인에이블 상태

#define LED2_ON()          (GpioDataRegs.GPBSET.bit.GPIO57 = 1)      ///< LED2 켜기
#define LED2_OFF()         (GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1)    ///< LED2 끄기
#define LED2_TOGGLE()      (GpioDataRegs.GPBTOGGLE.bit.GPIO57 = 1)   ///< LED2 토글

#define LED3_ON()          (GpioDataRegs.GPASET.bit.GPIO27 = 1)      ///< LED3 켜기
#define LED3_OFF()         (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1)    ///< LED3 끄기
#define LED3_TOGGLE()      (GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1)   ///< LED3 토글

#define LED_RUN_ON()       (GpioDataRegs.GPASET.bit.GPIO4 = 1)       ///< RUN LED 켜기
#define LED_RUN_OFF()      (GpioDataRegs.GPACLEAR.bit.GPIO4 = 1)     ///< RUN LED 끄기
#define LED_RUN_TOGGLE()   (GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1)    ///< RUN LED 토글

#define LED_FAULT_ON()     (GpioDataRegs.GPASET.bit.GPIO5 = 1)       ///< FAULT LED 켜기
#define LED_FAULT_OFF()    (GpioDataRegs.GPACLEAR.bit.GPIO5 = 1)     ///< FAULT LED 끄기
#define LED_FAULT_TOGGLE() (GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1)    ///< FAULT LED 토글

/** @} */

/**
 * @brief float32와 Uint32 간 변환을 위한 공용체
 * @details CAN 통신에서 float 데이터를 32bit 정수로 전송하기 위해 사용
 */
typedef union {
    float32 f;    ///< 부동소수점 값
    Uint32 u;     ///< 32비트 정수 값
} UNIONFLOAT;

// CAN 쉐도우 레지스터 정의
extern volatile struct ECAN_REGS ECanaShadow;

/**
 * @brief 시스템 운전 상태 열거형
 * @details Modbus 및 CAN을 통해 제어되는 시스템 운전/정지 상태
 */
typedef enum {
    STATE_STOP = 0,     ///< 시스템 정지
    STATE_RUNNING = 1   ///< 시스템 운전
} SYSTEM_STATE;

/**
 * @brief 충전/방전 모드 열거형
 * @details Modbus를 통해 제어 컴퓨터에서 설정되는 운전 모드
 */
typedef enum {
    No_Selection                   =   0,   ///< 모드 선택 안함
    ElectronicLoad_CV_Mode         =   2,   ///< 전자부하 정전압 모드
    ElectronicLoad_CC_Mode         =   4,   ///< 전자부하 정전류 모드
    ElectronicLoad_CR_Mode         =   8,   ///< 전자부하 정저항 모드
    PowerSupply_CV_Mode            =  16,   ///< 전원공급 정전압 모드
    PowerSupply_CC_Mode            =  32,   ///< 전원공급 정전류 모드
    Battery_Charg_Discharg_CC_Mode =  64,   ///< 배터리 충방전 정전류 모드
    As_a_Battery_CV_Mode           = 128    ///< 배터리 시뮬레이션 정전압 모드
} eCharge_DisCharge_Mode;

/**
 * @brief USART 전송 주기 열거형
 * @details 통신 주기 설정을 위한 열거형 (현재 미사용)
 */
typedef enum {
    e1000ms  = 100000, ///< 1Hz
    e100ms   = 10000,  ///< 10Hz
    e10ms    = 1000,   ///< 100Hz
    e1ms     = 100,    ///< 1kHz
    e0_5ms   = 50,     ///< 2kHz
    e0_2ms   = 20,     ///< 5kHz
    e100us   = 10,     ///< 10kHz
    e50us    = 5,      ///< 20kHz
    e0_01ms  = 1       ///< 100kHz, 10us
} eConfig_USART_send_period;

void modbus_parse(void);

#ifdef _MAIN_C_

//=============================================================================
// 1. System Control and Status Variables
//=============================================================================
eCharge_DisCharge_Mode eChargeMode;  ///< 현재 충전/방전 모드
Uint32 mainLoopCount = 0;            ///< 메인 루프 카운터 (디버깅용)
SYSTEM_STATE system_state = STATE_STOP;     ///< 시스템 운전 상태 (Modbus/CAN 통합)
eConfig_USART_send_period USART_send_period = e50us; ///< USART 전송 주기

//=============================================================================
// 2. Current Control and Sensing Variables
//=============================================================================

/**
 * @defgroup Current_Variables 전류 관련 변수
 * @brief 전류 지령, 센싱, 피드백 관련 변수들
 * @{
 */

// Current Commands (전류 지령)
float32 I_com_set = 0;           ///< 최종 전류 지령값 (DAC 출력용)
float32 I_com = 0;               ///< 기본 전류 지령 (Modbus에서 수신)

// Current Sensing and Feedback  
float32 I_avg = 0;               ///< 마스터 모듈 평균 출력 전류
float32 I_fb_sum = 0;            ///< 전류 센서 합계 (모니터링용)
Uint32 I_ADC_sum = 0;            ///< ADC 합계 (평균 계산용)
Uint16 I_ADC = 0;                ///< 현재 전류 ADC 값
Uint16 I_avg_count = 0;          ///< 전류 평균 계산 카운터

// PI Controller Current Variables
float32 I_cmd_ss_prev = 0;       ///< 이전 정상상태 전류 지령
float32 I_cmd_ss = 0;            ///< 정상상태 전류 지령
float32 I_ss, I_ss_prev, I_ss_prev2; ///< 소프트 스타트 관련 변수
float32 I_cmd_PI;                ///< PI 제어용 중간 전류 지령값

/** @} */

//=============================================================================
// 3. Voltage Control and Sensing Variables  
//=============================================================================

/**
 * @defgroup Voltage_Variables 전압 관련 변수
 * @brief 전압 지령, 센싱, 피드백 관련 변수들
 * @{
 */

// Voltage Commands (전압 지령)
float32 V_com = 0;               ///< 기본 전압 지령
float32 V_lim_max = 0;           ///< 고전압 지령 (충전 모드용)
float32 V_lim_min = 0;           ///< 저전압 지령 (방전 모드용)
float32 Bat_Mean = 0;            ///< 배터리 평균 전압

// Voltage Sensing and Feedback
float32 V_out = 0;               ///< 출력 전압 (PI 제어용)
volatile float32 V_out_ADC;      ///< 전압 ADC 원시값
float32 V_out_fb_sum = 0;        ///< 전압 센서 합계
float32 V_offset = 0;            ///< 전압 오프셋
float32 V_scale = 0;             ///< 전압 스케일

// Voltage Monitoring Variables
Uint32 mon_count = 0;            ///< 전압 모니터링 카운터
float32 V_out_fb_mon = 0.;       ///< 모니터링용 전압 센서 합계
float32 V_out_avg = 0.;          ///< 전압 평균값

// PI Controller Voltage Variables
float32 V_out_error = 0;         ///< 전압 오차
float32 V_lim_max_error = 0;     ///< 고전압 오차 (충전 모드)
float32 V_lim_min_error = 0;     ///< 저전압 오차 (방전 모드)
float32 V_out_error_PI_out = 0;  ///< 전압 오차 PI 출력
float32 V_lim_max_error_PI_out = 0; ///< 고전압 오차 PI 출력 (충전 모드)
float32 V_lim_min_error_PI_out = 0; ///< 저전압 오차 PI 출력 (방전 모드)

// Reference Values
Uint16 Vout_Reference;           ///< 전압 기준값 (Modbus에서 수신)
int16  Iout_Reference;           ///< 전류 기준값 (Modbus에서 수신)

/** @} */

//=============================================================================
// 4. PI Controller Parameters and Outputs
//=============================================================================

/**
 * @defgroup PI_Controller PI 제어기 관련 변수
 * @brief PI 제어기 파라미터 및 출력 변수들
 * @{
 */

// PI Parameters
float32 Kp = 1;                  ///< 비례 게인
float32 Ki = 3000;               ///< 적분 게인  
float32 Tsampl = 50E-6;          ///< 샘플링 시간 (50us, 20kHz)

// PI Outputs - Proportional Term
float32 kP_out = 0;              ///< 일반 비례 출력
float32 V_lim_max_kP_out = 0;    ///< 고전압 비례 출력
float32 V_lim_min_kP_out = 0;    ///< 저전압 비례 출력

// PI Outputs - Integral Term
float32 kI_out_prev = 0;         ///< 이전 적분 출력
float32 kI_out = 0;              ///< 일반 적분 출력
float32 V_lim_max_kI_out = 0;    ///< 고전압 적분 출력
float32 V_lim_min_kI_out = 0;    ///< 저전압 적분 출력

/** @} */

//=============================================================================
// 4.1 DCL PI Controller Variables (DCL 라이브러리 기반 PI 제어)
//=============================================================================

/**
 * @defgroup DCL_Controller DCL PI 제어기 관련 변수
 * @brief TI DCL 라이브러리 기반 최적화된 PI 제어기
 * @{
 */

// DCL PI Controllers
DCL_PI dcl_pi_charge;            ///< DCL 충전용 PI 컨트롤러
DCL_PI dcl_pi_discharge;         ///< DCL 방전용 PI 컨트롤러
DCL_CSS dcl_css_common;          ///< DCL 공통 지원 구조체

// DCL Control Flags
Uint16 use_dcl_controller = 1;   ///< DCL 제어기 사용 플래그 (1: DCL 사용, 0: 기존 사용)

/** @} */

//=============================================================================
// 5. Temperature Sensing and Fan Control
//=============================================================================

/**
 * @defgroup Temperature_Control 온도 센싱 및 팬 제어
 * @brief 온도 측정 및 팬 PWM 제어 관련 변수들
 * @{
 */

// Temperature Sensing (온도 센싱)
float32 temp_in = 0;             ///< 실제 온도값 (°C)
volatile float32 temp_ADC;       ///< 온도 ADC 원시값
float32 temp_ADC_fb = 0;         ///< 온도 ADC 센서값
float32 temp_ADC_fb_alt = 0;     ///< 온도 ADC 센서값 1

// Fan Control
float32 fan_pwm_duty = 0.15;     ///< 팬 PWM 듀티 사이클 (15~90%)
float32 fan_pwm_duty_tmp = 0.;   ///< 팬 PWM 듀티 임시 계산값

/** @} */

//=============================================================================
// 6. DAC and SPI Control Variables
//=============================================================================

/**
 * @defgroup SPI_DAC SPI 및 DAC 제어
 * @brief SPI 통신 및 DAC 출력 관련 변수들
 * @{
 */

int16 deg_sDacTmp;               ///< DAC 임시 변수

// SPI Communication
Uint16 spi_tx_tmp = 0;   ///< SPI 전송용 임시 변수

/** @} */

//=============================================================================  
// 7. Timing and Control Flags
//=============================================================================

/**
 * @defgroup Timing_Control 타이밍 및 제어 플래그
 * @brief 시스템 타이밍 제어를 위한 플래그들
 * @{
 */

// High frequency counters and flags
Uint32 controlPhase = 0;        ///< 100kHz → 20kHz 분주 카운터 (0~4)
Uint16 _10ms_flag, _1ms_flag, _100us_flag, _50us_flag, _0_1ms_count; ///< 타이밍 플래그들
Uint32 __100ms_flag, __1000ms_flag; ///< 저주파 플래그들

// Setup timers
Uint32 setup_off_timer = 0, setup_on_timer = 0; ///< 설정 타이머들
Uint16 dab_ok_fault = 0, dab_ok = 0;            ///< DAB 상태 플래그들

/** @} */

//=============================================================================
// 8. Digital I/O and Status Variables  
//=============================================================================

/**
 * @defgroup Status_Variables 상태 및 디지털 I/O 변수
 * @brief 시스템 상태 및 디지털 입출력 관련 변수들
 * @{
 */

Uint16 dummy;                    ///< 더미 변수
Uint16 V_out_measured = 0;       ///< 전압 측정값
Uint16 average_count = 0;        ///< 일반 평균 카운터
Uint16 over_voltage_flag = 0;    ///< 과전압 플래그

/** @} */

//=============================================================================
// 9. Communication and Monitoring Variables
//=============================================================================

/**
 * @defgroup Communication 통신 및 모니터링 변수
 * @brief Modbus, CAN 통신 및 모니터링 관련 변수들
 * @{
 */

// Modbus and Communication
UNIONFLOAT load_resistance, UI_I_cmd;   ///< 부하 저항, UI 전류 지령
UNIONFLOAT I_fb_array[10];       ///< 슬레이브 전류 센서 배열 (인덱스 1~9 사용)
UNIONFLOAT I_sensor, I_fb_total, V_fb_avg; ///< 센서 데이터

// Serial Communication
float32 V_ADC, V_out_fb;         ///< ADC 전압 변수들

/** @} */

//=============================================================================
// 10. Calculation and Control Variables  
//=============================================================================

/**
 * @defgroup Calculation 계산 및 제어 변수
 * @brief 각종 계산 및 제어용 임시 변수들
 * @{
 */

// Calculation Variables
float32 I_cmd_tmp;               ///< 전류 지령 임시 계산 변수
float32 V_lim_max_tmp, V_lim_min_tmp; ///< 전압 고/저 임시값
float32 power;                   ///< 전력 계산값
int16 I_ref_tmp;                 ///< 전류 기준 임시 변수

// Voltage Monitoring Variables (TI style naming)
Uint32 V_count = 0;              ///< 전압 모니터링 카운터
float32 V_sum_mon = 0.;          ///< 모니터링용 전압 센서 합계
float32 V_avg = 0.;              ///< 전압 평균값

/** @} */

//=============================================================================
// 11. Function Declarations
//=============================================================================

//-----------------------------------------------------------------------------
// A. Interrupt Service Routines (인터럽트 서비스 루틴)
//-----------------------------------------------------------------------------

/**
 * @defgroup ISR_Functions 인터럽트 서비스 루틴
 * @brief 시스템 인터럽트 처리 함수들
 * @{
 */

/**
 * @brief ADC 인터럽트 서비스 루틴
 * @details 온도, 전류, 전압 ADC 변환 완료 시 호출
 */
__interrupt void adc_isr(void);

/**
 * @brief CPU Timer 0 인터럽트 (100kHz)
 * @details 워치독 서비스 및 고속 타이밍 제어
 */
__interrupt void cpu_timer0_isr(void);

/**
 * @brief CPU Timer 2 인터럽트 (1초)
 * @details 저속 타이밍 제어
 */
__interrupt void cpu_timer2_isr(void);

/**
 * @brief ePWM1 인터럽트
 * @details 팬 PWM 제어용
 */
__interrupt void epwm1_isr(void);

/**
 * @brief ePWM3 인터럽트 (100kHz)
 * @details 메인 제어 루프 - PI 제어, 통신, 센싱
 */
__interrupt void epwm3_isr(void);

/**
 * @brief SPI 인터럽트
 * @details SPI 통신 완료 시 호출
 */
__interrupt void spi_isr(void);

/**
 * @brief SCI-A 송신 FIFO 인터럽트
 * @details RS485 통신 송신 완료 시 호출
 */
__interrupt void scia_txFifo_isr(void);

/**
 * @brief CAN 인터럽트
 * @details CAN 메시지 수신 시 호출 (슬레이브 피드백 + Protocol)
 */
__interrupt void ecan0_isr(void);

/**
 * @brief SCI-B 수신 인터럽트
 * @details Modbus RTU 데이터 수신 시 호출
 */
__interrupt void scibRxReadyISR(void);

/**
 * @brief SCI-B 송신 인터럽트
 * @details Modbus RTU 데이터 송신 완료 시 호출
 */
__interrupt void scibTxEmptyISR(void);

/**
 * @brief CPU Timer 1 만료 인터럽트
 * @details Modbus 타이밍 제어용
 */
__interrupt void cpuTimer1ExpiredISR(void);

/** @} */

//-----------------------------------------------------------------------------
// B. Control Algorithm Functions (제어 알고리즘 함수)
//-----------------------------------------------------------------------------

/**
 * @defgroup Control_Functions 제어 알고리즘 함수
 * @brief PI 제어 및 제어 알고리즘 관련 함수들
 * @{
 */

/**
 * @brief 고전압 PI 컨트롤러 (충전 모드용)
 * @details I_com >= 0일 때 사용되는 PI 제어기
 */
void PIControlHigh(void);

/**
 * @brief 저전압 PI 컨트롤러 (방전 모드용)
 * @details I_com < 0일 때 사용되는 PI 제어기
 */
void PIControlLow(void);

/**
 * @brief 통합 PI 컨트롤러 (충전/방전 자동 선택)
 * @details 전류 지령에 따라 충전/방전 모드를 자동 선택하는 통합 PI 제어기
 * @note 50% 계산량 절약, bumpless transfer 구현
 */
void PIControlUnified(void);

/**
 * @brief DCL PI 컨트롤러 초기화
 * @details TI DCL 라이브러리 기반 PI 컨트롤러 초기화
 */
void InitDCLControllers(void);

/**
 * @brief DCL 기반 PI 제어 함수
 * @details DCL_runPI_C1 어셈블리 함수를 사용한 최적화된 PI 제어
 * @note 약 65% 성능 향상 (40-50 cycles → 12-15 cycles)
 */
void PIControlDCL(void);

/**
 * @brief DCL 어셈블리 함수 선언
 * @param pi PI 컨트롤러 구조체 포인터
 * @param rk 기준값 (reference)
 * @param yk 피드백값 (feedback)
 * @return PI 제어 출력값
 */
extern float32 DCL_runPI_C1(DCL_PI *pi, float32 rk, float32 yk);

/** @} */

//-----------------------------------------------------------------------------
// C. Sensing and Calculation Functions (센싱 및 계산 함수)
//-----------------------------------------------------------------------------

/**
 * @defgroup Sensing_Functions 센싱 및 계산 함수
 * @brief 센서 데이터 처리 및 계산 관련 함수들
 * @{
 */

/**
 * @brief GPIO 디지털 입력 읽기
 * @details DIP 스위치, 시작/정지 스위치 상태 읽기
 */
void ReadGpioInputs(void);

/**
 * @brief 전류 평균 계산
 * @details SPI ADC로부터 읽은 전류값의 평균 계산 (10000회 평균)
 */
void CalcCurrentAverage(void);

/**
 * @brief 전압 평균 계산
 * @details 전압 센서값의 평균 계산 (10000회 평균)
 */
void CalcVoltageAverage(void);

/** @} */

//-----------------------------------------------------------------------------
// D. System Control Functions (시스템 제어 함수)
//-----------------------------------------------------------------------------

/**
 * @defgroup System_Control 시스템 제어 함수
 * @brief 시스템 제어 관련 함수들
 * @{
 */

/**
 * @brief 팬 PWM 제어
 * @details 온도에 따른 팬 PWM 듀티 사이클 제어 (15~90%)
 */
void ControlFanPwm(void);

/** @} */

//-----------------------------------------------------------------------------
// E. Communication Functions (통신 함수)
//-----------------------------------------------------------------------------

/**
 * @defgroup Communication_Functions 통신 함수
 * @brief 통신 프로토콜 처리 관련 함수들
 * @{
 */

/**
 * @brief Modbus 데이터 파싱
 * @details 제어 컴퓨터로부터 수신한 Modbus 데이터 파싱 및 처리
 * @note 전류/전압 지령, 운전 모드 설정 등 처리
 */
void ParseModbusData(void);

/**
 * @brief 문자열 전송 (SCI-A)
 * @param buff 전송할 데이터 버퍼
 * @param Length 전송할 데이터 길이
 * @details RS485를 통한 슬레이브 전류 지령 브로드캐스팅
 */
void stra_xmit(Uint8 *buff, Uint16 Length);

/** @} */

//-----------------------------------------------------------------------------
// F. System Initialization Functions (시스템 초기화 함수)
//-----------------------------------------------------------------------------

/**
 * @defgroup Init_Functions 시스템 초기화 함수
 * @brief 시스템 초기화 관련 함수들
 * @{
 */

/**
 * @brief ePWM1 초기화
 * @details 팬 PWM 제어용 ePWM1 초기화
 */
void InitEPwm1(void);

/**
 * @brief ePWM3 초기화
 * @details 메인 제어 타이밍용 ePWM3 초기화 (100kHz)
 */
void InitEPwm3(void);

/**
 * @brief SPI 초기화
 * @details ADC/DAC 통신용 SPI 초기화
 */
void SpiInit(void);

/**
 * @brief CAN 설정
 * @details CAN 통신 설정 (슬레이브 피드백용)
 */
void eCanaConfig(void);

/** @} */

#else // ifndef _MAIN_C_

//=============================================================================
// External Variable Declarations
//=============================================================================
extern eCharge_DisCharge_Mode eChargeMode;
extern Uint16 Vout_Reference;
extern int16  Iout_Reference;

extern float32 temp_in;
extern float32 V_ADC;
extern float32 V_out, I_avg;          // Output voltage and current average
extern SYSTEM_STATE system_state;              // System operation state (unified)

extern Uint32 controlPhase, __100ms_flag, __1000ms_flag;
extern float Power;
extern int16 iref_tmp;
extern eConfig_USART_send_period USART_send_period;

// Additional External Declarations
extern float32 I_cmd_PI;                              // Current command intermediate value
extern Uint32 V_count;                  // Voltage monitoring counter
extern float32 V_avg;                          // Voltage mean value

// DCL Controller External Declarations
extern DCL_PI dcl_pi_charge, dcl_pi_discharge;     // DCL PI 컨트롤러들
extern DCL_CSS dcl_css_common;                     // DCL 공통 지원 구조체
extern Uint16 use_dcl_controller;                  // DCL 제어기 사용 플래그

#endif   //_MAIN_C_

#endif   //_MAIN_H_
