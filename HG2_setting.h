/**
 * @file HG2_setting.h
 * @brief 시스템 하드웨어 초기화 헤더 파일
 * 
 * @author 김은규 (원작자), 신덕균 (수정자)
 * @date 2025
 * @version 1.0
 * @copyright Copyright (c) 2025
 * 
 * @details
 * 35kW DC-DC 컨버터 하드웨어 초기화 함수 선언 및 상수 정의
 * 
 * @section functions 초기화 함수
 * - PWM 모듈 초기화 (메인 타이머, 팬 제어)
 * - ADC 센싱 설정 (온도, 전류, 전압)
 * - 통신 인터페이스 (SPI, CAN, SCI)
 * - GPIO 핀 설정 및 디지털 I/O
 * 
 * @section constants 주요 상수
 * - PWM 주기: 10kHz (팬), 100kHz (메인)
 * - 데드타임: 300ns (27 클럭)
 * - 클럭 설정: 90MHz 시스템 클럭
 * 
 * @section history 변경 이력
 * - v1.0: 김은규 - 기본 설정
 * - v1.1: 신덕균 - protocol 추가 및 주석 개선
 */

#ifndef _HG2_SETTING_H_
#define _HG2_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "DSP28x_Project.h" //for basic variable types

    //=============================================================================
    // GPIO 디지털 입력 변수들
    //=============================================================================
    extern Uint16 board_id;     // DIP 스위치로부터 읽은 보드 ID (4비트)
    extern Uint16 run_switch; // 운전 스위치 상태 (GPIO54)

#define ACTIVE (1) // only use when a DAC board is inserted

    //////////////////////////////////////////////////////////////
    // Prototype statements for functions found within this file.
    void InitEPwm1(void);
    void InitEPwm3(void);

    void GpioConfig(void); // GPIO configuration
    void AdcSetup(void);

    void SpiFifoConfig(void); // Initialize the Spi FIFO
    void SpiConfig(void);      // init SPI

    void InitScia(void);
    void SciaFifoConfig(void);

    void ReadGpioInputs(void); // Read GPIO digital inputs
    void ECanaConfig(void);   // CAN configuration

#define PWM_PERIOD_10k 4500 //(FAN)        // TBPRD = HSCLK/(2*freq) = 90M/(2*10k) = 4500
#define PWM_PERIOD_100k 900 // TBPRD = HSCLK/(2*freq) = 90M/(2*10k) = 4500

#define EPWM_DB 27 // deadtime = 300n --> DB = 300n/(1/HSCLK) = 0.3u*90M = 27

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif //_HG2_SETTING_H_
