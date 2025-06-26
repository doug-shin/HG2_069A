/**
 * @file HG2_setting.c
 * @brief 시스템 하드웨어 초기화 구현 파일
 * 
 * @author 김은규 (원작자), 신덕균 (수정자)
 * @date 2025
 * @version 1.1
 * @copyright Copyright (c) 2025
 * 
 * @details
 * TI F28069 기반 35kW DC-DC 컨버터 하드웨어 초기화 구현
 * 
 * @section modules 초기화 모듈
 * **PWM 시스템**
 * - ePWM1: 팬 PWM 제어 (10kHz, 데드타임 300ns)
 * - ePWM3: 메인 제어 타이머 (100kHz 인터럽트)
 * 
 * **통신 인터페이스**
 * - SPI-A: 외부 ADC 통신 (11.25MHz)
 * - CAN-A: 프로토콜 통신 (500kbps)
 * - SCI-A/B: RS485/RS232 통신
 * 
 * **센싱 시스템**
 * - 내장 ADC: 온도, 전류, 배터리 전압
 * - GPIO: 디지털 입력 (DIP 스위치, 운전 스위치)
 * 
 * @section timing 타이밍 설정
 * - 시스템 클럭: 90MHz
 * - PWM 클럭: 90MHz (1:1 분주)
 * - SPI 클럭: 11.25MHz (8:1 분주)
 * - 제어 주기: 100kHz (10μs)
 * 
 * @section history 변경 이력
 * - v1.0: 김은규 - 기본 설정
 * - v1.1: 신덕균 - protocol 추가 및 주석 개선
 * 
 * @note 시스템 부팅 시 한 번만 실행되는 초기화 함수들
 */

#include "DSP28x_Project.h"
#include "HG2_setting.h"
#include "modbus.h"

//=============================================================================
// GPIO 디지털 입력 변수 정의
// 주요 변수들은 HG2.h에서 선언됨 (extern 선언)
//=============================================================================

//=============================================================================
// PWM 초기화 함수들
//=============================================================================

/**
 * @brief ePWM1 초기화 - 팬 PWM 제어용 (10kHz)
 * - 용도: 냉각팬 속도 제어
 * - 주파수: 10kHz (TBPRD = 4500)
 * - 모드: UP-DOWN 카운트, 데드타임 적용
 * - 보호: Trip Zone (TZ1, TZ2) 설정
 */
void InitEPwm1()
{
    // 타이머 기본 설정
    EPwm1Regs.TBPRD = PWM_PERIOD_10k;    // 타이머 주기 설정 (10kHz)
    EPwm1Regs.TBCTR = 0x0000;            // 카운터 초기화
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000; // 위상 0도

    // 타이머 제어 설정
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // UP-DOWN 카운트
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // 위상 로딩 비활성화 (마스터 모듈)
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;         // Shadow 모드에서 로드
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // 동기 신호 설정
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 클럭 분주비 1:1
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // 비교값 제어 설정
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Shadow 모드
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Zero에서 로드
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // 비교값 설정
    EPwm1Regs.CMPA.half.CMPA = 0;        // 초기 듀티 0% (팬 OFF)
    EPwm1Regs.CMPB = PWM_PERIOD_10k;     // PWM1B 초기 OFF

    // 액션 설정 (NON-inverted PWM)
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR; // CAU에서 Clear
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;   // CAD에서 Set
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;   // CBU에서 Set
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR; // CBD에서 Clear

    // 데드타임 설정
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // 극성 설정
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;    // 입력 모드
    EPwm1Regs.DBRED = EPWM_DB;                // Rising edge 데드타임
    EPwm1Regs.DBFED = EPWM_DB;                // Falling edge 데드타임

    // Trip Zone 보호 설정
    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;         // TZ1 One-shot 트립 활성화
    EPwm1Regs.TZSEL.bit.OSHT2 = 1;         // TZ2 One-shot 트립 활성화
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // 트립 시 PWM1A LOW
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // 트립 시 PWM1B LOW
    EDIS;
}

/**
 * @brief ePWM3 초기화 - 시스템 메인 타이머 (100kHz)
 * - 용도: 100kHz 인터럽트 생성 (시스템 제어 루프)
 * - 주파수: 100kHz (TBPRD = 899)
 * - 모드: UP 카운트
 * - 인터럽트: Period에서 발생
 */
void InitEPwm3()
{
    // 타이머 기본 설정
    EPwm3Regs.TBPRD = PWM_PERIOD_100k - 1; // 타이머 주기 설정 (100kHz)
    EPwm3Regs.TBCTR = 0x0000;              // 카운터 초기화
    EPwm3Regs.TBPHS.half.TBPHS = 0;        // 위상 0도

    // 타이머 제어 설정
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // UP 카운트
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // 위상 로딩 비활성화
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // 클럭 분주비 1:1
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // 동기 비활성화

    // 비교값 제어 설정
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Shadow 모드
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Zero에서 로드
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // 비교값 설정 (FPGA Chip Select용)
    EPwm3Regs.CMPA.half.CMPA = (PWM_PERIOD_100k - 1) * 0.9f; // 10% 듀티
    EPwm3Regs.CMPB = 0;                                      // PWM3B OFF

    // 액션 설정 (NON-inverted)
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;   // CAU에서 Set
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // Zero에서 Clear

    // 인터럽트 설정 (100kHz 시스템 타이머)
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_PRD; // Period에서 인터럽트
    EPwm3Regs.ETSEL.bit.INTEN = 1;           // 인터럽트 활성화
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;      // 매 주기마다 인터럽트
}

//=============================================================================
// ADC 및 GPIO 초기화 함수들
//=============================================================================

/**
 * @brief ADC 초기화 설정
 * - SOC0: ADCINB0 (Ch8) - 온도 센서
 * - SOC1: ADCINB1 (Ch9) - 전류 센서  
 * - SOC2: ADCINB2 (Ch10) - 배터리 전압 센서 (V_batt_avg)
 * - 트리거: EPWM6A SOCA
 * - 샘플링: 7 ADC 클럭 사이클
 * 
 * @note 주 전압 센싱은 SPI ADC(SpiaRegs.SPIRXBUF)를 사용
 */
void AdcSetup(void)
{
    EALLOW;

    // ADC 기본 설정
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Non-overlap 모드 활성화
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // AdcResults 래치 후 ADCINT1 트리거

    // 인터럽트 설정
    AdcRegs.INTSEL1N2.bit.INT1E = 1;    // ADCINT1 활성화
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0; // ADCINT1 연속 모드 비활성화
    AdcRegs.INTSEL1N2.bit.INT1SEL = 0;  // EOC8에서 ADCINT1 트리거

    // ADC 채널 매핑
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 8;  // SOC0 -> ADCINB0 (온도)
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 9;  // SOC1 -> ADCINB1 (전류)
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 10; // SOC2 -> ADCINB2 (배터리 전압)

    // 트리거 소스 선택 (Round-robin: SOC0 -> SOC1 -> SOC2)
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0F; // EPWM6A SOCA 트리거
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0F; // Round-robin: SOC0 다음
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0F; // Round-robin: SOC1 다음

    // 샘플 & 홀드 윈도우 설정 (7 ADC 클럭)
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6; // 6 ACQPS + 1 = 7 클럭
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;

    EDIS;
}

/**
 * @brief GPIO 핀 기능 및 방향 설정
 * - 디지털 입력: DIP 스위치 4개, 외부 스위치, 센싱 신호 3개
 * - 디지털 출력: LED 2개, Buck Enable, Discharge, CS 신호들
 * - Pull-up: DIP 스위치는 비활성화 (외부 풀업 사용)
 * - Input Qualification: GPIO54(Run Switch)에 노이즈 필터 적용
 */
void GpioConfig(void)
{
    EALLOW;

    // 모든 GPIO를 입력으로 설정 후 필요한 것만 출력으로 변경
    GpioCtrlRegs.GPADIR.all = 0x0000;
    GpioCtrlRegs.GPBDIR.all = 0x0000;

    // PWM 출력 핀들 설정
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1; // ePWM1A
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1; // ePWM1B
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1; // ePWM3A
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1; // ePWM3B

    // SPI 관련 출력 핀들 설정
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;  // ADC CNV CS (출력)
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1; // EEPROM WP (출력)

    // 제어 출력 핀들 설정
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1; // Buck Enable (출력)
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1; // Discharge FET (출력)
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1; // Debug LED3 (출력)

    // LED 및 센싱 핀들 설정
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1; // DAC CS (출력)
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1; // Debug LED2 (출력)

    // DIP 스위치 입력 핀들 설정 (기본적으로 입력이므로 방향은 설정 안함)
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 0; // DIP Switch_1 (입력)
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0; // DIP Switch_2 (입력)
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0; // DIP Switch_3 (입력)
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0; // DIP Switch_4 (입력)
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0; // 외부 센싱 1 (입력)
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0; // 외부 센싱 2 (DAB 상태) (입력)
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0; // 외부 센싱 3 (입력)
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0; // 외부 스위치 (Run Switch) (입력)

    // GPIO 핀 기능 설정 (MUX = 0: GPIO 모드, MUX = 1: 주변장치 모드)
    // SPI 관련
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;  // ADC CNV CS (GPIO 모드)
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0; // EEPROM WP (GPIO 모드)

    // DIP 스위치 (GPIO 모드)
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // DIP Switch_1
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // DIP Switch_2
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0; // DIP Switch_3
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // DIP Switch_4

    // 외부 신호 입력 (GPIO 모드)
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0; // 외부 센싱 1
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0; // 외부 센싱 2 (DAB 상태)
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; // 외부 센싱 3
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0; // 외부 스위치 (Run Switch)

    // 출력 신호 (GPIO 모드)
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0; // Buck Enable
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // Discharge FET
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0; // DAC CS
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0; // Debug LED2
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; // Debug LED3

    // PWM Mux 설정 (주변장치 모드)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // ePWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // ePWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // ePWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; // ePWM3B

    // DIP 스위치 핀들 설정 (하드웨어 qualification 적용)
    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF; // GPIO0-7 qualification
    GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0xFF; // GPIO8-15 qualification
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD1 = 0xFF; // GPIO40-47 qualification
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 0xFF; // GPIO48-55 qualification

    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 2; // DIP Switch_1 (6 sample qualification)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 2; // DIP Switch_2 (6 sample qualification)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // DIP Switch_3 (6 sample qualification)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 2; // DIP Switch_4 (6 sample qualification)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Run Switch (6 sample qualification)

    // Pull-up 설정 (DIP 스위치들은 외부 풀업 사용으로 비활성화)
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1; // DIP Switch_2 pull-up 비활성화
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1; // DIP Switch_1 pull-up 비활성화
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1; // DIP Switch_4 pull-up 비활성화
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // DIP Switch_3 pull-up 비활성화

    // Input Qualification 설정 (노이즈 필터링)
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 0xFF; // Qualification 주기 = SYSCLKOUT/510
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;     // GPIO54(Run Switch): 6 샘플 필터링

    EDIS;
}

// GPIO 디지털 입력 읽기 함수 (하드웨어 qualification으로 글리치 제거됨)
void ReadGpioInputs(void)
{
    // Board ID 읽기 (DIP 스위치 4개를 비트 연산으로 조합)
    board_id = (GpioDataRegs.GPBDAT.bit.GPIO41 << 3) | // DIP Switch_4 (Bit3)
               (GpioDataRegs.GPBDAT.bit.GPIO55 << 2) | // DIP Switch_3 (Bit2)
               (GpioDataRegs.GPADAT.bit.GPIO10 << 1) | // DIP Switch_2 (Bit1)
               (GpioDataRegs.GPADAT.bit.GPIO11);       // DIP Switch_1 (Bit0)

    // 운전 스위치 읽기 (하드웨어에서 이미 필터링된 값)
    run_switch = GpioDataRegs.GPBDAT.bit.GPIO54;
}

//=============================================================================
// SPI 및 SCI 통신 초기화 함수들
//=============================================================================

/**
 * @brief SPI-A 초기화 (DAC/ADC 통신용)
 * - 모드: 마스터, 16비트 데이터
 * - 클럭: LSPCLK / (SPIBRR + 1) = 90MHz / 8 = 11.25MHz
 * - 극성: CPOL=0, CPHA=0 (Mode 0)
 * - 용도: DAC 제어 및 ADC 데이터 읽기
 */
void SpiConfig()
{
    // SPI 소프트웨어 리셋
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    // SPI 기본 설정 (16비트, Rising edge)
    SpiaRegs.SPICCR.all = 0x000F; // Reset, Rising edge, 16-bit
    SpiaRegs.SPICTL.all = 0x0006; // Master mode, Normal phase

    // SPI 제어 설정
    SpiaRegs.SPICTL.bit.TALK = 1;      // Master/Slave 송신 활성화
    SpiaRegs.SPICTL.bit.SPIINTENA = 0; // SPI 인터럽트 비활성화
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0; // SPICLK 지연 없음

    // SPI 클럭 설정
    SpiaRegs.SPIBRR = 0x0007; // Baud Rate = LSPCLK / (7+1)

    // SPI 최종 설정
    SpiaRegs.SPICCR.all = 0x00DF;        // Reset 해제
    SpiaRegs.SPICCR.bit.SPILBK = 0;      // Loopback 비활성화
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; // 클럭 극성 설정
    SpiaRegs.SPIPRI.bit.FREE = 1;        // 브레이크포인트 방해 방지

    // SPI 활성화
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}

/**
 * @brief SPI FIFO 초기화
 * - TX FIFO: 인터럽트 플래그 클리어, FIFO 활성화
 * - RX FIFO: 4워드 후 인터럽트, FIFO 활성화
 * - 지연: 없음
 */
void SpiFifoConfig()
{
    SpiaRegs.SPIFFTX.all = 0xE040; // TX FIFO: 클리어 & 활성화
    SpiaRegs.SPIFFRX.all = 0x2044; // RX FIFO: 4워드 인터럽트 & 활성화
    SpiaRegs.SPIFFCT.all = 0x0;    // FIFO 송신 지연 없음
}

/**
 * @brief SCI-A FIFO 초기화 (RS232/RS485 통신용)
 * - F28069 전용 설정 (4-level FIFO)
 * - TX/RX FIFO 활성화
 */
void SciaFifoConfig(void)
{
    SciaRegs.SCIFFTX.all = 0xE040; // TX FIFO 설정
    SciaRegs.SCIFFRX.all = 0x2041; // RX FIFO 설정
    SciaRegs.SCIFFCT.all = 0x0;    // FIFO 제어 설정
}

Uint16 sci_baud_capture = 0;

/**
 * @brief SCI-A 초기화 (RS232/RS485 Modbus 통신용)
 * - 설정: 38400bps, 8비트, Even 패리티, 1 정지비트
 * - 모드: 비동기, Idle-line 프로토콜
 * - 용도: Modbus RTU 통신
 */
void InitScia(void)
{
    // SCI 기본 설정
    SciaRegs.SCICCR.all = 0x0007;      // 1 정지비트, 루프백 없음
                                       // 8비트 데이터, 비동기 모드
    SciaRegs.SCICCR.bit.PARITYENA = 1; // 패리티 활성화
    SciaRegs.SCICCR.bit.PARITY = 1;    // Even 패리티

    // SCI 제어 설정
    SciaRegs.SCICTL1.all = 0x0003;       // TX, RX 활성화, 내부 SCICLK
    SciaRegs.SCICTL2.bit.TXINTENA = 0;   // TX 인터럽트 비활성화
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0; // RX 브레이크 인터럽트 비활성화

    // Baud Rate 설정 (38400bps)
    sci_baud_capture = SCIA_PRD;
    SciaRegs.SCIHBAUD = SCIA_PRD >> 8;   // High byte
    SciaRegs.SCILBAUD = SCIA_PRD & 0xFF; // Low byte

    // SCI 활성화
    SciaRegs.SCICTL1.bit.SWRESET = 1; // 소프트웨어 리셋 해제
    SciaRegs.SCICTL1.all = 0x0023;    // SCI 리셋 해제 및 활성화
}

extern struct ECAN_REGS ECanaShadow;

void ECanaConfig(void)
{
    EALLOW;

    // CAN 모드 설정
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.SCB = 1; // eCAN-B 모드 활성화 (32개 메일박스 사용)
    ECanaShadow.CANMC.bit.DBO = 0; // Data Byte Order (0: 표준)
    ECanaShadow.CANMC.bit.PDR = 0; // Power-down mode request (0: 정상 동작)
    ECanaShadow.CANMC.bit.ABO = 1; // Auto Bus-On (1: 자동 버스 온)
    ECanaShadow.CANMC.bit.STM = 0; // Self-Test Mode (0: 비활성화)
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Configuration Change Request
    ECanaRegs.CANMC.bit.CCR = 1;
    while (ECanaRegs.CANES.bit.CCE != 1)
    {
    }

    /* CAN 통신 속도를 500kbps로 설정 */
    /* CAN 클럭 = 45MHz (SYSCLKOUT/2) */
    /* Time Quantum(Tq) = (BRP + 1) / 45MHz = 5 / 45MHz = 111.11ns */
    /* Bit Time = (SYNC_SEG + TSEG1 + TSEG2) = (1 + 14 + 3) = 18 Tq */
    /* 전송 속도 = 45MHz / (5 * 18) = 500kbps */
    /* 샘플링 포인트 = (SYNC_SEG + TSEG1) / Bit Time * 100% = 15/18 * 100% = 83.33% */
    ECanaRegs.CANBTC.all = 0;
    ECanaRegs.CANBTC.bit.BRPREG = 4;    // BRP = 4 (실제값 = 5)
    ECanaRegs.CANBTC.bit.TSEG1REG = 13; // TSEG1 = 13 (실제값 = 14, Propagation + Phase1)
    ECanaRegs.CANBTC.bit.TSEG2REG = 2;  // TSEG2 = 2 (실제값 = 3, Phase2)
    ECanaRegs.CANBTC.bit.SJWREG = 0;    // SJW = 0 (실제값 = 1, Synchronization Jump Width)

    ECanaRegs.CANMC.bit.CCR = 0;
    while (ECanaRegs.CANES.bit.CCE != 0)
    {
    }

    EDIS;

    // 모든 메일박스 비활성화
    ECanaRegs.CANME.all = 0;
    ECanaShadow.CANMD.all = 0;
    ECanaShadow.CANME.all = 0;

    // MD (Message Direction) 설정 - RX 모드
    ECanaShadow.CANMD.bit.MD1 = 1;  // MBOX1 RX
    ECanaShadow.CANMD.bit.MD2 = 1;  // MBOX2 RX
    ECanaShadow.CANMD.bit.MD3 = 1;  // MBOX3 RX
    ECanaShadow.CANMD.bit.MD4 = 1;  // MBOX4 RX
    ECanaShadow.CANMD.bit.MD5 = 1;  // MBOX5 RX
    ECanaShadow.CANMD.bit.MD6 = 1;  // MBOX6 RX
    ECanaShadow.CANMD.bit.MD7 = 1;  // MBOX7 RX
    ECanaShadow.CANMD.bit.MD8 = 1;  // MBOX8 RX
    ECanaShadow.CANMD.bit.MD9 = 1;  // MBOX9 RX
    ECanaShadow.CANMD.bit.MD29 = 1; // MBOX29 RX
    ECanaShadow.CANMD.bit.MD30 = 1; // MBOX30 RX
    ECanaShadow.CANMD.bit.MD31 = 1; // MBOX31 RX

    // ME (Mailbox Enable) 설정
    ECanaShadow.CANME.bit.ME16 = 1;
    ECanaShadow.CANME.bit.ME17 = 1;
    ECanaShadow.CANME.bit.ME18 = 1;
    ECanaShadow.CANME.bit.ME19 = 1;
    ECanaShadow.CANME.bit.ME20 = 1;
    ECanaShadow.CANME.bit.ME21 = 1;
    ECanaShadow.CANME.bit.ME22 = 1;
    ECanaShadow.CANME.bit.ME23 = 1;
    ECanaShadow.CANME.bit.ME24 = 1;
    ECanaShadow.CANME.bit.ME25 = 1;
    ECanaShadow.CANME.bit.ME29 = 1;
    ECanaShadow.CANME.bit.ME30 = 1;
    ECanaShadow.CANME.bit.ME31 = 1;

    // MSGID 설정 - 11bit ID (MBOX 0~9)
    ECanaMboxes.MBOX0.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = 0xF0;
    ECanaMboxes.MBOX1.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = 0xF1;
    ECanaMboxes.MBOX2.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX2.MSGID.bit.STDMSGID = 0xF2;
    ECanaMboxes.MBOX3.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX3.MSGID.bit.STDMSGID = 0xF3;
    ECanaMboxes.MBOX4.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX4.MSGID.bit.STDMSGID = 0xF4;
    ECanaMboxes.MBOX5.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX5.MSGID.bit.STDMSGID = 0xF5;
    ECanaMboxes.MBOX6.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX6.MSGID.bit.STDMSGID = 0xF6;
    ECanaMboxes.MBOX7.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX7.MSGID.bit.STDMSGID = 0xF7;
    ECanaMboxes.MBOX8.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX8.MSGID.bit.STDMSGID = 0xF8;
    ECanaMboxes.MBOX9.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX9.MSGID.bit.STDMSGID = 0xF9;

    // MSGID 설정 - 29bit ID (MBOX 16~31)
    ECanaMboxes.MBOX16.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX16.MSGID.bit.EXTMSGID_L = 0x117;
    ECanaMboxes.MBOX17.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX17.MSGID.bit.EXTMSGID_L = 0x116;
    ECanaMboxes.MBOX18.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX18.MSGID.bit.EXTMSGID_L = 0x115;
    ECanaMboxes.MBOX19.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX19.MSGID.bit.EXTMSGID_L = 0x114;
    ECanaMboxes.MBOX20.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX20.MSGID.bit.EXTMSGID_L = 0x113;
    ECanaMboxes.MBOX21.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX21.MSGID.bit.EXTMSGID_L = 0x112;
    ECanaMboxes.MBOX22.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX22.MSGID.bit.EXTMSGID_L = 0x111;
    ECanaMboxes.MBOX23.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX23.MSGID.bit.EXTMSGID_L = 0x110;
    ECanaMboxes.MBOX24.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX24.MSGID.bit.EXTMSGID_L = 0x2200;
    ECanaMboxes.MBOX25.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX25.MSGID.bit.EXTMSGID_L = 0x2201;
    ECanaMboxes.MBOX29.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX29.MSGID.bit.EXTMSGID_L = 0x360;
    ECanaMboxes.MBOX30.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX30.MSGID.bit.EXTMSGID_L = 0x200;
    ECanaMboxes.MBOX31.MSGID.bit.IDE = 1;
    ECanaMboxes.MBOX31.MSGID.bit.EXTMSGID_L = 0x201;

    // DLC 설정 - MBOX 0~15: DLC=4
    ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX3.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX5.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX6.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX7.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX8.MSGCTRL.bit.DLC = 4;
    ECanaMboxes.MBOX9.MSGCTRL.bit.DLC = 4;

    // DLC 설정 - MBOX 16~31: DLC=8
    ECanaMboxes.MBOX16.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX17.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX19.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX20.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX21.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX22.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX23.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX24.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX29.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX30.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC = 8;

    // AME (Acceptance Mask Enable) 설정
    ECanaMboxes.MBOX30.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX31.MSGID.bit.AME = 1;

    // LAM(Local Acceptance Mask) 설정 (MBOX30, MBOX31)
    ECanaLAMRegs.LAM30.all = 0;
    ECanaLAMRegs.LAM30.bit.LAMI = 1;
    ECanaLAMRegs.LAM30.bit.LAM_L = 0xFE;
    ECanaLAMRegs.LAM31.all = 0;
    ECanaLAMRegs.LAM31.bit.LAMI = 1;
    ECanaLAMRegs.LAM31.bit.LAM_L = 0xFE;

    // 전역 마스크 설정
    ECanaShadow.CANGAM.all = 0;
    ECanaShadow.CANGAM.bit.AMI = 0; // 전역 마스크 적용 (0: 로컬 마스크 사용)
    ECanaRegs.CANGAM.all = ECanaShadow.CANGAM.all;

    // 설정 적용
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;

    // 인터럽트 설정
    EALLOW;
    ECanaRegs.CANMIM.all = 0x0; // 모든 메일박스 인터럽트 비활성화

    // 특정 메일박스 인터럽트 활성화 (MBOX30, MBOX31)
    ECanaRegs.CANMIM.bit.MIM30 = 1; // MBOX30 인터럽트 활성화
    ECanaRegs.CANMIM.bit.MIM31 = 1; // MBOX31 인터럽트 활성화

    // 인터럽트 라인 할당 (ECAN0INTA로 할당)
    ECanaRegs.CANMIL.bit.MIL30 = 0; // MBOX30을 INT9.5에 할당
    ECanaRegs.CANMIL.bit.MIL31 = 0; // MBOX31을 INT9.5에 할당

    // 전역 인터럽트 설정
    ECanaRegs.CANGIM.all = 0;
    ECanaRegs.CANGIM.bit.GIL = 0;  // 모든 인터럽트를 INT9.5에 할당
    ECanaRegs.CANGIM.bit.I0EN = 1; // 인터럽트0 활성화
    ECanaRegs.CANGIM.bit.I1EN = 0; // 인터럽트1 비활성화
    ECanaRegs.CANGIM.bit.AAIM = 0; // Abort Acknowledge 인터럽트 비활성화
    ECanaRegs.CANGIM.bit.WDIM = 0; // Write Denied 인터럽트 비활성화
    ECanaRegs.CANGIM.bit.WUIM = 0; // Wake-up 인터럽트 비활성화
    ECanaRegs.CANGIM.bit.BOIM = 0; // Bus-off 인터럽트 비활성화

    EDIS;
}
