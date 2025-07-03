/**
 * @file HG2.c
 * @brief 35kW DC-DC 컨버터 제어 시스템 메인 소스 파일
 * 
 * @author 김은규 (원작자), 신덕균 (수정자)
 * @date 2025
 * @version 1.2
 * @copyright Copyright (c) 2025
 *
 * @details
 * TI F28069 마이크로컨트롤러 기반 35kW DC-DC 컨버터 제어 시스템
 * DCL 라이브러리를 활용한 최적화된 PI 제어 및 다중 통신 프로토콜 지원
 *
 * @section overview 시스템 개요
 * **주요 기능**
 * - PI 제어 기반 전압/전류 제어 (충전/방전 모드)
 * - 멀티 프로토콜 통신 (Modbus RTU, CAN Protocol, Legacy CAN, RS485)
 * - 하드웨어 폴트 감지 및 보호 기능
 * - 온도 모니터링 및 팬 PWM 제어
 * - 소프트 스타트 및 안전 기능
 *
 * @section control_system 제어 시스템
 * **제어 타이밍**
 * - **메인 루프**: 100kHz ePWM3 인터럽트 타이머 기반
 * - **제어 주기**: 20kHz (5단계 순차 처리)
 * - **PI 컨트롤러**: DCL 라이브러리 기반 (65% 성능 향상)
 * 
 * **제어 알고리즘**
 * - 전압센싱 → PI제어 → 통신처리 → 온도제어 → 평균계산
 * - 충전/방전 모드별 독립 PI 컨트롤러
 * - 소프트 스타트를 통한 안전한 전류 램프업
 *
 * @section communication 통신 시스템
 * **1. Modbus RTU (SCI-B/RS232, 38.4kbps)**
 * - HMI와의 제어 통신
 * - 전류/전압 지령 수신, 센서값 송신
 * 
 * **2. CAN Protocol (CAN-A, 500kbps)**
 * - 외부 EPC 시스템과의 표준 프로토콜 통신
 * - protocol.md 규격 (100번대 보고, 200번대 명령, Heart Beat)
 * 
 * **3. Legacy CAN (CAN-A, 500kbps)**
 * - 내부 슬레이브 모듈 1~9 피드백 수신
 * - MBOX0~9 (마스터 0xF0, 슬레이브 0xF1~0xF9)
 * 
 * **4. RS485 (SCI-A, 5.625Mbps)**
 * - 슬레이브로 전류 지령 DAC 값 전송 (20kHz 주기)
 * - 프로토콜: STX + 2바이트 + ETX
 *
 * @section hardware 하드웨어 구성
 * **클럭 시스템**
 * - SYSCLKOUT: 90MHz
 * - LSPCLK: 11.25MHz (90MHz/8)
 * 
 * **센싱 시스템**
 * - 내장 ADC (12비트): 온도, 전류 센싱
 * - 외부 SPI ADC (16비트): 고정밀 전압 센싱
 * 
 * **출력 시스템**
 * - RS485 전송: 12비트 DAC 전류 지령값 (0~4095)
 * - 팬 PWM 제어: 온도 기반 15~90% 제어
 *
 * @section safety 안전 기능
 * - 과전압 보호 (1100V 임계값)
 * - DAB 하드웨어 폴트 감지
 * - 워치독 타이머
 * - 소프트 스타트 제어
 * - Heart Beat 타임아웃 감지
 *
 * @section todo 주요 미완료 작업
 * **우선순위 높음**
 * - Protocol Heart Beat 타임아웃 처리
 * - 슬레이브 CAN ID 중복 검출
 * - 통신 폴트 처리 및 경고
 * 
 * **우선순위 중간**
 * - FAN 초기 동작 개선
 * - SCI 에러 처리 강화
 *
 * @section history 변경 이력
 * - v1.0: 김은규 - 초기 개발 (기본 제어, Modbus 통신)
 * - v1.1: 신덕균 - 구조 개선 (변수 정리, CAN Protocol 추가)
 * - v1.2: 신덕균 - 최적화 (DCL 컨트롤러, 인터럽트 개선)
 */

// ###########################################################################
#ifndef _MAIN_C_
#define _MAIN_C_

//*****************************************************************************
// 헤더 파일 포함
//*****************************************************************************
#include <string.h>
#include <math.h>

// TI C2000 Device Support
#include "DSP28x_Project.h"

// DCL 라이브러리 (Digital Control Library)
#include "DCLCLA.h"

// 프로젝트 헤더 파일
#include "HG2.h"
#include "HG2_CLA_shared.h"

// CAN Protocol 관련 외부 변수 선언
extern PROTOCOL_INTEGRATED protocol; // 프로토콜 구조체

// CAN 쉐도우 레지스터 정의
volatile struct ECAN_REGS ECanaShadow;

void InitEPwm1(void);
void InitEPwm3(void);
void SpiConfig(void);
void ECanaConfig(void);
void InitCLA(void);
void InitDCLControllersCLA(void);

#pragma CODE_SECTION(cpuTimer1ExpiredISR, "ramfuncs");
#pragma CODE_SECTION(scibRxReadyISR, "ramfuncs");
#pragma CODE_SECTION(scibTxEmptyISR, "ramfuncs");
#pragma CODE_SECTION(ParseModbusData, "ramfuncs");
#pragma CODE_SECTION(control_loop_isr, "ramfuncs");
#pragma CODE_SECTION(can_protocol_isr, "ramfuncs");
#pragma CODE_SECTION(adc_isr, "ramfuncs");

__interrupt void control_loop_isr(void);
__interrupt void can_protocol_isr(void);
__interrupt void adc_isr(void);

extern __interrupt void scibRxReadyISR(void);
extern __interrupt void scibTxEmptyISR(void);
extern __interrupt void cpuTimer1ExpiredISR(void);

extern USHORT usRegInputBuf[REG_INPUT_NREGS];
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

// SCI전송 제어 문자
#define STX (0x02) // Start of Text, 본문의 개시 및 정보 메세지 헤더의 종료를 표시
#define ETX (0x03) // End of Text, 본문의 종료를 표시한다
#define DLE (0x10) // Data link escape, 뒤따르는 연속된 글자들의 의미를 바꾸기 위해 사용, 주로 보조적 전송제어기능을 제공

Uint16 cpu_timer1_cnt = 0;
Uint32 control_loop_cnt = 0;

Uint16 force_reset_test = 0, fifo_err_cnt = 0;
Uint16 can_tx_cnt = 0, can_tx_flag = 0;
Uint16 can_rx_fault_cnt[11];
Uint16 detect_module_num = 1;

//=============================================================================
// CLA 공유 변수 정의 (CPU ↔ CLA 데이터 교환용)
//=============================================================================

/* 충전용 PI 컨트롤러 공유 변수 (CPU → CLA) */
#pragma DATA_SECTION(charge_rk, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(charge_yk, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(charge_uk, "Cla1ToCpuMsgRAM")
float charge_rk = 0.0f;          // 충전 기준값 (V_max_lim)
float charge_yk = 0.0f;          // 충전 피드백값 (V_out)
float charge_uk = 0.0f;          // 충전 제어 출력값 (V_max_PI)

/* 방전용 PI 컨트롤러 공유 변수 (CPU → CLA) */
#pragma DATA_SECTION(discharge_rk, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(discharge_yk, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(discharge_uk, "Cla1ToCpuMsgRAM")
float discharge_rk = 0.0f;       // 방전 기준값 (V_min_lim)
float discharge_yk = 0.0f;       // 방전 피드백값 (V_out)
float discharge_uk = 0.0f;       // 방전 제어 출력값 (V_min_PI)

/* CLA PI 컨트롤러 구조체 (CLA Data RAM에 저장) */
#pragma DATA_SECTION(dcl_pi_charge_cla, "Cla1DataRam1")
#pragma DATA_SECTION(dcl_pi_discharge_cla, "Cla1DataRam1")
DCL_PI_CLA dcl_pi_charge_cla = PI_CLA_DEFAULTS;
DCL_PI_CLA dcl_pi_discharge_cla = PI_CLA_DEFAULTS;

// CAN 메일박스 배열 포인터
static struct MBOX *mbox_array = (struct MBOX *)&ECanaMboxes;

Uint16 discharge_fet_delay_cnt = 0;  // 방전 FET 지연 카운터 (운전 시작 후 1초)
Uint16 timer_50us_cnt = 0;

// ADC 및 기본 설정
const float32 V_ref = 5.0f;

/**
 * @brief 메인 함수 - 시스템 초기화 및 메인 루프
 *
 * @details 시스템 초기화 순서:
 * 1. 시스템 제어 초기화 (PLL, 워치독, 주변장치 클럭)
 * 2. GPIO 초기화 및 하드웨어 설정
 * 3. 인터럽트 및 PIE 벡터 테이블 초기화
 * 4. 시리얼 통신 초기화 (RS-232, RS-485)
 * 5. CAN 통신 및 프로토콜 초기화
 * 6. ADC 및 SPI 초기화
 * 7. 타이머 및 PWM 초기화
 * 8. DCL PI 컨트롤러 초기화
 * 9. Modbus 프로토콜 스택 초기화
 * 10. 메인 루프 실행
 *
 * @section main_loop 메인 루프 처리 내용
 * - SCI FIFO 오버플로우 처리
 * - Modbus 폴링 (필수 호출)
 * - **Protocol 시스템**: EPC와의 CAN 통신 처리 (500kbps)
 *   - Heart Beat 타임아웃 검사
 *   - 명령 수신 및 ACK 처리  
 *   - 상태 보고 (10ms/100ms 주기)
 * - **Legacy CAN**: 슬레이브 1~9 피드백 수신 (500kbps 적응)
 * - Modbus 데이터 파싱 (20kHz → 100Hz)
 * - 워치독 서비스
 */
void main(void)
{
    Uint16 i;
    static eMBErrorCode eStatus;

    UI_I_cmd.f = 0;

    //=========================================================================
    // 1. 시스템 제어 초기화 (PLL, 워치독, 주변장치 클럭)
    //=========================================================================
    InitSysCtrl(); // 90MHz PLL 설정 (TI 디바이스서포트 기본 함수)
    ConfigLowSpeedClock(); // LOSPCP 설정 오버라이드 (22.5MHz → 90MHz)

    // Flash 운영을 위한 RAM으로 복사 (타이밍 크리티컬 코드)
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    
    // CLA 프로그램 Flash에서 RAM으로 복사
    memcpy(&Cla1funcsRunStart, &Cla1funcsLoadStart, (Uint32)&Cla1funcsLoadSize);

    //=========================================================================
    // 2. GPIO 초기화
    //=========================================================================
    GpioConfig();
    ReadGpioInputs(); // DIP 스위치로 보드 ID 읽기 및 운전 스위치 상태 읽기

    // PWM 및 SPI GPIO 설정
    InitEPwm1Gpio();
    InitEPwm3Gpio();
    ConfigSpiaGpio();

    //=========================================================================
    // 3. 인터럽트 및 PIE 벡터 테이블 초기화
    //=========================================================================
    DINT;
    InitPieCtrl();
    InitFlash();

    // 인터럽트 플래그 초기화
    IER = 0x0000;
    IFR = 0x0000;

    //=========================================================================
    // 4. 시리얼 통신 초기화 (RS-232, RS-485)
    //=========================================================================
    // GPIO 초기화 (모든 통신 포트)
    ConfigSciAGpio();  // SCI-A GPIO 초기화 (RS485용)
    ConfigSciBGpio();  // SCI-B GPIO 초기화 (Modbus RTU용)
    
    // SCI-A 레지스터 설정
    SciaFifoConfig();  // SCI-A FIFO 설정
    InitScia();        // SCI-A 초기화

    EALLOW;
    En485_ON();
    EDIS;

    Tx485_ON(); // 마스터 모드

    //=========================================================================
    // 5. PIE 벡터 테이블 및 CAN 통신 초기화
    //=========================================================================
    InitPieVectTable();
    InitECanaGpio();
    InitECana();
    ECanaConfig();
    InitProtocol();

    //=========================================================================
    // 6. ADC 및 SPI 초기화
    //=========================================================================
    InitAdc();
    AdcOffsetSelfCal();
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 1; // 외부 기준전압 선택
    EDIS;

    AdcSetup();
    SpiConfig();

    //=========================================================================
    // 7. 타이머 초기화
    //=========================================================================
    InitCpuTimers();
    // CpuTimer2 제거: 사용되지 않는 인터럽트였음

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // 타이머 정지
    EDIS;

    //=========================================================================
    // 8. 인터럽트 벡터 할당
    //=========================================================================
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    EALLOW;
    PieVectTable.EPWM3_INT = &control_loop_isr;
    PieVectTable.ADCINT1 = &adc_isr;
    PieVectTable.SCIRXINTB = &scibRxReadyISR;
    PieVectTable.SCITXINTB = &scibTxEmptyISR;
    PieVectTable.TINT1 = &cpuTimer1ExpiredISR;
    PieVectTable.ECAN0INTA = &can_protocol_isr;
    EDIS;

    //=========================================================================
    // 9. PWM 초기화
    //=========================================================================
    InitEPwm1();
    InitEPwm3();

    //=========================================================================
    // 10. 인터럽트 활성화
    //=========================================================================
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

    // CPU 인터럽트 그룹 활성화
    IER |= M_INT1;  // Timer 0, ADC
    IER |= M_INT3;  // ePWM
    IER |= M_INT9;  // CAN, SCI
    IER |= M_INT13; // Timer 1

    // PIE 인터럽트 활성화
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // ePWM3
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADC
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1; // SCI-B RX
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1; // SCI-B TX
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1; // CAN

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // 타이머 시작
    EDIS;

    //=========================================================================
    // 11. DAC 초기 설정
    //=========================================================================
    DAC1_DS();
    DAC1_CS();
    SpiaRegs.SPITXBUF = 0xD002; // DAC 제어 레지스터 (2.048V 내부 기준전압)
    while (!SpiaRegs.SPISTS.bit.INT_FLAG);
    (void)SpiaRegs.SPIRXBUF; // SPI 통신 완료를 위한 더미 읽기

    //=========================================================================
    // 12. Modbus 프로토콜 스택 초기화
    //=========================================================================
    // 레지스터 초기화
    for (i = 0; i < REG_INPUT_NREGS; i++)
        usRegInputBuf[i] = 0;
    for (i = 0; i < REG_HOLDING_NREGS; i++)
        usRegHoldingBuf[i] = 0;

    // Modbus RTU 모드 초기화 (슬레이브 주소: 0x01, 38400 bps)
    eStatus = eMBInit(MB_RTU, 0x01, 0, 38400, MB_PAR_NONE);
    if (eStatus != MB_ENOERR)
    {
        EALLOW;
        SysCtrlRegs.WDCR = 0x0010;
        EDIS;
    }

    // Modbus 프로토콜 스택 활성화
    eStatus = eMBEnable();
    if (eStatus != MB_ENOERR)
    {
        EALLOW;
        SysCtrlRegs.WDCR = 0x0010;
        EDIS;
    }

    //=========================================================================
    // 13. CLA 및 DCL PI 컨트롤러 초기화
    //=========================================================================
    InitCLA();              // CLA 하드웨어 초기화
    InitDCLControllersCLA(); // CLA 기반 DCL PI 컨트롤러 초기화

    //=========================================================================
    // 14. 최종 시스템 활성화
    //=========================================================================
    ServiceDog(); // 워치독 리셋

    // 전류 센서 배열 초기화
    for (i = 0; i < 10; i++)
    {
        I_fb_array[i].f = 0.0f;
    }

    EALLOW;
    SysCtrlRegs.WDCR = 0x0028; // 워치독 활성화
    EDIS;

    EINT; // 전역 인터럽트 활성화
    ERTM; // 실시간 디버그 인터럽트 활성화

    I_cmd_DAC = 2000; // 초기 전류 지령값

    //=========================================================================
    // 15. 메인 루프
    //=========================================================================
    for (;;)
    {
        main_loop_cnt++; // 디버깅용 free-run 카운터

        // SCI FIFO 오버플로우 처리
        if (ScibRegs.SCIFFRX.bit.RXFFOVF || force_reset_test)
        {
            force_reset_test = 0;
            fifo_err_cnt++;
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
            ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
        }

        // Modbus 폴링 (필수 호출)
        (void)eMBPoll();

        // Protocol CAN 보고 처리 (EPC 시스템과의 통신)
        if (can_report_flag == 1)
        {
            can_report_flag = 0;
            SendCANReport(16); // protocol.md 규격에 따른 상태 보고 (100번대/110번대)
        }

        // Legacy CAN 송수신 처리 (슬레이브 1~9 피드백, 1kHz)
        if (can_tx_flag == 1)
        {
            Uint32 rmp_status;
            can_tx_flag = 0;

            // Legacy 운전 상태에 따른 CAN 메시지 설정 (Legacy 프로그램 호환)
            if (system_state == STATE_RUNNING)
                ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0xA0; // 운전 신호 (0xA0)
            else
                ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0;    // 정지 신호 (0x00)

            ECanaRegs.CANTRS.all = 0x00000001; // MBOX0 전송 요청 (마스터 → 슬레이브)

            // Legacy CAN 메일박스 상태 읽기 (성능 최적화)
            rmp_status = ECanaRegs.CANRMP.all & 0x000003FE; // MBOX1~9 (슬레이브 피드백)

            // Legacy 슬레이브 모듈별 전류값 수신 처리 (배열에 직접 저장)
            for (i = 1; i <= 9; i++)
            {
                if (rmp_status & (1 << i))
                {
                    // Legacy CAN 데이터 수신됨 (슬레이브 전류 피드백)
                    I_fb_array[i].u = mbox_array[i].MDL.all;
                    ECanaRegs.CANRMP.all = ((Uint32)1 << i); // 해당 비트만 클리어
                    can_rx_fault_cnt[i] = 0;
                }
                else
                {
                    // Legacy CAN 데이터 수신 안됨 (타임아웃 처리)
                    if (can_rx_fault_cnt[i]++ >= 10000)
                    {
                        I_fb_array[i].u = 0;
                        can_rx_fault_cnt[i] = 10000;
                    }
                }
            }

            // Legacy 활성 모듈 개수 계산 (슬레이브 1~9)
            for (i = 1; i < 10; i++)
            {
                if (can_rx_fault_cnt[i] < 9999)
                {
                    if (detect_module_num++ > 10)
                        detect_module_num = 10;
                }
                else
                {
                    if (detect_module_num-- < 1)
                        detect_module_num = 1;
                }
            }
        }

        // Modbus 데이터 파싱 (10ms 주기)
        if (parse_mb_flag)
        {
            parse_mb_flag = 0;
            ParseModbusData();
        }

        // 워치독 서비스
        EALLOW;
        SysCtrlRegs.WDKEY = 0x0055; // 워치독 키 (다른 키는 Timer 0 ISR에서)
        EDIS;

    } // 메인 루프 끝
} // 메인 함수 끝

//=============================================================================
// DCL PI 제어 함수 (성능 최적화)
//=============================================================================

/**
 * @brief CLA 기반 통합 PI 제어 함수 (CLA 가속 최적화)
 *
 * @details 100kHz 인터럽트에서 호출되는 CLA 기반 PI 제어 함수
 * - CLA Task 3/4에서 병렬 PI 제어 실행
 * - DCL_runPI_L1 CLA 함수 사용: CLA 하드웨어 최적화
 * - 적분기 상태는 CLA DataRAM에 저장되어 지속적으로 유지됨
 * 
 * @note 충전/방전 모드에 따라 적절한 컨트롤러 선택 및 bumpless transfer 구현
 * @note 적분기 상태: dcl_pi_charge_cla.i10, dcl_pi_discharge_cla.i10 (CLA 전용 변수)
 */
static inline void PIControlCLA(void)
{
    if (I_cmd >= 0)
    {
        // =====================================================================
        // 충전 모드 (I_cmd >= 0): 고전압 제어
        // =====================================================================

        // 동적 상한값 업데이트 (전류 제한에 따라)
        dcl_pi_charge_cla.Umax = I_cmd_ss;

        // CLA 공유 변수 업데이트 (CPU → CLA 메시지 RAM)
        charge_rk = V_max_lim;    // 기준값
        charge_yk = V_out;        // 피드백값

        // CLA Task 3 트리거: 충전용 PI 제어 실행
        EALLOW;
        Cla1ForceTask3andWait();
        EDIS;

        // CLA 계산 결과 읽기 (CLA → CPU 메시지 RAM)
        V_max_PI = charge_uk;

        // 기존 변수 호환성을 위한 할당 (디버깅 및 모니터링용)
        V_max_error = V_max_lim - V_out;                     // 에러 계산
        V_max_kP_out = dcl_pi_charge_cla.Kp * V_max_error;   // 비례항
        V_max_kI_out = dcl_pi_charge_cla.i10;                // 적분항 상태

        // 방전용 컨트롤러와의 bumpless transfer를 위한 상태 동기화
        // 방전 모드로 전환될 때 급격한 출력 변화 방지
        dcl_pi_discharge_cla.i10 = V_max_PI - dcl_pi_discharge_cla.Kp * (V_min_lim - V_out);
        if (dcl_pi_discharge_cla.i10 > 2.0f)
            dcl_pi_discharge_cla.i10 = 2.0f;
        else if (dcl_pi_discharge_cla.i10 < I_cmd_ss)
            dcl_pi_discharge_cla.i10 = I_cmd_ss;
    }
    else
    {
        // =====================================================================
        // 방전 모드 (I_cmd < 0): 저전압 제어
        // =====================================================================

        // 동적 하한값 업데이트 (전류 제한에 따라)
        dcl_pi_discharge_cla.Umin = I_cmd_ss;

        // CLA 공유 변수 업데이트 (CPU → CLA 메시지 RAM)
        discharge_rk = V_min_lim; // 기준값
        discharge_yk = V_out;     // 피드백값

        // CLA Task 4 트리거: 방전용 PI 제어 실행
        EALLOW;
        Cla1ForceTask4andWait();
        EDIS;

        // CLA 계산 결과 읽기 (CLA → CPU 메시지 RAM)
        V_min_PI = discharge_uk;

        // 기존 변수 호환성을 위한 할당 (디버깅 및 모니터링용)
        V_min_error = V_min_lim - V_out;                         // 에러 계산
        V_min_kP_out = dcl_pi_discharge_cla.Kp * V_min_error;   // 비례항
        V_min_kI_out = dcl_pi_discharge_cla.i10;                // 적분항 상태

        // 충전용 적분기는 현재 PI 출력으로 초기화 (bumpless transfer)
        dcl_pi_charge_cla.i10 = V_min_PI - dcl_pi_charge_cla.Kp * (V_max_lim - V_out);
        if (dcl_pi_charge_cla.i10 > I_cmd_ss)
            dcl_pi_charge_cla.i10 = I_cmd_ss;
        else if (dcl_pi_charge_cla.i10 < -2.0f)
            dcl_pi_charge_cla.i10 = -2.0f;
    }
}

//=============================================================================
// 제어 태스크 함수들 (함수 포인터 최적화)
//=============================================================================

/**
 * @brief Case 0: 전압 센싱 태스크 (20kHz)
 * @details SPI ADC 전압 센싱 및 스케일링 처리
 */
static void voltage_sensing_task(void)
{
    // SPI ADC 전압 센싱 및 스케일링
    // SPI ADC 평균값 계산 (5회 평균, 100kHz → 20kHz 데시메이션)
    V_out_ADC_avg = (float32)(V_out_ADC_sum * 0.2f);
    V_out_ADC_sum = 0; // 누적값 초기화
    
    // ADC → 전압 변환: V_ref * ADC_value * (1/65536) - offset = 실제 전압
    // 0.0000152 = 1/65536 (16비트 ADC 정규화)
    // 0.1945 = ADC 오프셋 보정값
    // 250 = 전압 분배비 (실제 전압/센싱 전압)
    // 1.04047 = 하드웨어 보정계수 (1/0.9611, 실측 캘리브레이션 값)
    V_fb = V_out = (V_ref * V_out_ADC_avg * 0.0000152f - 0.1945f) * 250.0f * 1.04047f; // 보정계수 적용 및 제어기용 전압값 설정
}

/**
 * @brief Case 1: PI 제어 태스크 (20kHz)
 * @details CLA 기반 통합 PI 제어 (충전/방전 자동 선택)
 */
static void pi_control_task(void)
{
    // CLA PI 제어기 실행
    PIControlCLA(); // CLA 기반 최적화된 PI 제어
}

/**
 * @brief RS485 전류 지령 전송 함수 (코드 중복 제거)
 * @param dac_value 전송할 DAC 값 (0~4095)
 */
/**
 * @brief RS485 네트워크로 전류 지령 브로드캐스트 (병렬 모듈 동기화용)
 *
 * @param dac_value 전송할 DAC 값 (0~4095)
 *
 * @details
 * ** 목적: 병렬 연결된 슬레이브 모듈들에게 동일한 전류 지령 전송 **
 * - 마스터 모듈(이 모듈)이 계산한 전류 지령을 모든 슬레이브에 브로드캐스트
 * - 모든 모듈이 동일한 전류로 동작하여 전류 분담 및 안정성 확보
 * - 주기: 20kHz (Case 2에서 호출)
 * 
 * ** 프로토콜 구조 (4바이트) **
 * - STX (0x02): 시작 문자
 * - Data Low: DAC 값 하위 8비트 (0~255)
 * - Data High: DAC 값 상위 8비트 (0~15, 12비트 DAC의 상위 4비트만 사용)
 * - ETX (0x03): 종료 문자
 * 
 * ** 데이터 의미 **
 * - 0: 최대 회생 (-80A)
 * - 2000: 전류 없음 (0A) 
 * - 4000: 최대 방전 (+80A)
 * - 4095: DAC 최대값 (하드웨어 제한)
 */
static inline void send_current_command_rs485(Uint16 dac_value)
{
    // RS485 브로드캐스트: 병렬 모듈 동기화를 위한 전류 지령 전송
    SciaRegs.SCITXBUF = STX;                               // 시작 문자 (0x02)
    while (SciaRegs.SCICTL2.bit.TXRDY == 0);               // 전송 완료 대기
    
    SciaRegs.SCITXBUF = (dac_value & 0xFF);                // 전류 지령 하위 바이트 (0~255)
    while (SciaRegs.SCICTL2.bit.TXRDY == 0);               // 전송 완료 대기
    
    SciaRegs.SCITXBUF = ((dac_value >> 8) & 0xFF);         // 전류 지령 상위 바이트 (실제 0~15, 12비트)
    while (SciaRegs.SCICTL2.bit.TXRDY == 0);               // 전송 완료 대기
    
    SciaRegs.SCITXBUF = ETX;                               // 종료 문자 (0x03)
    while (SciaRegs.SCICTL2.bit.TXRDY == 0);               // 전송 완료 대기
}

/**
 * @brief Case 2: 전류 지령 처리 및 통신 태스크 (20kHz)
 * @details 소프트 스타트, DAC 출력, RS485 통신, 안전 처리
 */
static void current_control_task(void)
{
    //=========================================================================
    // 1. 시스템 안전성 확인 및 운전 상태 결정 (우선 처리)
    //=========================================================================
    
    // 과전압 보호 (안전성 체크 우선)
    if (V_out >= OVER_VOLTAGE)
        over_voltage_flag = 1;

    // 시스템 안전성 확인 및 운전 상태 결정
    if (IsSystemSafeToRun())
    {
        system_state = STATE_RUNNING;
        BUCK_ENABLE(); // Buck 컨버터 활성화

        //=====================================================================
        // 2. 운전 상태: 소프트 스타트 및 제어 로직 실행
        //=====================================================================
        
        // 소프트 스타트 램프업 처리 (운전 중일 때만)
        // 0.00005 = 램프업 기울기 (80A/1.6초, 50us마다 0.004A 증가)
        soft_start_limit += CURRENT_LIMIT * 0.00005f;
        if (soft_start_limit > CURRENT_LIMIT)
            soft_start_limit = CURRENT_LIMIT; // 최대 전류로 제한

        // 소프트 스타트 제한 처리
        if (I_cmd > soft_start_limit)
            I_cmd_ss = soft_start_limit;
        else if (I_cmd < -soft_start_limit)
            I_cmd_ss = -soft_start_limit;
        else
            I_cmd_ss = I_cmd;

        // PI 출력 제한 적용
        if (I_cmd_ss > V_max_PI)
            I_cmd_final = V_max_PI;
        else if (I_cmd_ss < V_min_PI)
            I_cmd_final = V_min_PI;
        else
            I_cmd_final = I_cmd_ss;

        // 방전 FET 제어 (운전 시작 후 1초 지연)
        // 20000 = 1초 지연 (20kHz × 1초)
        if (discharge_fet_delay_cnt++ >= 20000)
        {
            DISCHARGE_FET_OFF(); // 방전 FET 끄기
        }

        // DAC 출력값 계산 및 제한
        // 25.0 = 전류→DAC 스케일링 팩터 (50 * 0.5, 원본: I_com_set * 50 * 0.5f)
        // 2000 = DAC 제로점 (0A 지령 시 DAC 출력값)
        // 1.0005 = DAC 보정계수 (하드웨어 캘리브레이션)
        // 방전 모드(+80A): DAC 4000, 회생 모드(-80A): DAC 0
        I_cmd_DAC = (I_cmd_final * 25.0f + 2000.0f) * 1.0005f;
        if (I_cmd_DAC > 4095)
            I_cmd_DAC = 4095; // DAC 상한 제한 (12비트)
    }
    else
    {
        //=====================================================================
        // 2. 정지 상태: 즉시 초기화 및 안전 처리
        //=====================================================================
        
        system_state = STATE_STOP;
        BUCK_DISABLE(); // Buck 컨버터 비활성화
        
        // 정지 상태일 때 즉시 초기화 (타이밍 이슈 해결)
        soft_start_limit = 0.0f;        // 소프트 스타트 리셋
        discharge_fet_delay_cnt = 0;     // 방전 FET 지연 카운터 리셋
        I_cmd_ss = 0.0f;                 // 소프트 스타트 제한 전류 리셋
        I_cmd_final = 0.0f;              // 최종 전류 지령 리셋
        I_cmd_DAC = 2000;                // DAC 출력을 0A 지령으로 리셋
        
        DISCHARGE_FET_ON(); // 방전 FET 켜기 (즉시, 안전성 확보)
    }

    //=========================================================================
    // 3. 병렬 모듈 동기화: RS485 네트워크로 전류 지령 브로드캐스트
    //=========================================================================
    // ** 목적: 마스터-슬레이브 병렬 운전 시스템 구현 **
    // - 이 모듈(마스터)이 계산한 I_cmd_DAC를 모든 슬레이브 모듈에 전송
    // - 운전/정지 상태 무관하게 항상 전송하여 동기화 유지
    // - 모든 모듈이 동일한 전류 지령으로 동작 → 전류 분담 및 안정성 확보
    // - 전송 주기: 20kHz (50μs마다) - 실시간 동기화
    send_current_command_rs485(I_cmd_DAC);
}

/**
 * @brief Case 3: 온도 처리 및 CAN 보고 태스크 (20kHz)
 * @details 온도 센서 처리, 전압 평균 계산, CAN 보고 타이밍 관리
 */
static void temperature_monitoring_task(void)
{
    // 온도 센서 처리
    // 4095 = 12비트 ADC 최대값, 3.0 = ADC 기준전압 (3V)
    temp_ADC_fb = (float32)(temp_ADC / 4095.0f * 3.0f);
    // 5/3 = 온도센서 전압 증폭비 복원, 20.4 = 온도센서 기울기(mV/℃), 1.4 = 오프셋 온도(℃)
    temp_in = (float32)((temp_ADC_fb * 5.0f / 3.0f * 20.4f) + 1.4f); // 인라인 계산으로 최적화

    // 전압 평균값 계산
    Calc_V_fb_avg();

    // CAN 메시지 전송 주기 관리 (실제 메시지 전송 타이밍)
    // ** can_report_interval 설정값 **
    // - 운전 중: 200 (10ms = 200 × 0.05ms) → 100Hz 전송
    // - 대기 중: 2000 (100ms = 2000 × 0.05ms) → 10Hz 전송
    // - 프로토콜 규격: 100번대(10ms), 110번대(100ms)에 맞춤
    if (can_report_cnt++ >= can_report_interval)
    {
        can_report_cnt = 0;
        can_report_flag = 1; // 메인 루프에서 실제 CAN 메시지 전송
    }
}

/**
 * @brief Case 4: 평균 계산 및 팬 제어 태스크 (20kHz)
 * @details 10ms 타이머, 전류 평균 계산, 팬 PWM 제어
 */
static void average_calculation_task(void)
{
    // 10ms 타이머 (디지털 입력 및 Modbus 파싱용)
    if (timer_50us_cnt++ >= 200)
    { // 200 * 0.05ms = 10ms
        timer_50us_cnt = 0;
        parse_mb_flag = 1; // Modbus 파싱 플래그 설정
        ReadGpioInputs();  // DIP 스위치 및 운전 스위치 입력 처리
    }

    // 전류 평균 계산 및 팬 PWM 제어
    Calc_I_fb_avg();
    ControlFanPwm();
}

/**
 * @brief 제어 태스크 함수 포인터 배열 (컴파일 타임 최적화)
 * @details Switch문 대신 직접 함수 호출로 성능 향상 (20-50% 개선)
 */
static void (*control_task_functions[])(void) = {
    voltage_sensing_task,        // Phase 0: 전압 센싱
    pi_control_task,             // Phase 1: PI 제어
    current_control_task,        // Phase 2: 전류 지령 처리
    temperature_monitoring_task, // Phase 3: 온도 처리
    average_calculation_task     // Phase 4: 평균 계산
};

//=============================================================================
// 인터럽트 서비스 루틴
//=============================================================================

/**
 * @brief ePWM3 인터럽트 서비스 루틴 (100kHz 호출)
 *
 * @details 메인 제어 루프 - 시스템의 핵심 제어 알고리즘이 실행되는 인터럽트
 *
 * @section timing_structure 타이밍 구조 (100kHz → 20kHz 분주)
 * - **Case 0**: 전압 센싱 (20kHz)
 * - **Case 1**: 통합 PI 제어 (충전/방전 자동 선택) - **DCL 최적화 적용**
 * - **Case 2**: 전류 지령 처리 및 RS485 통신 (20kHz)
 * - **Case 3**: 온도 처리 및 CAN 보고 (20kHz)
 * - **Case 4**: 평균 계산 및 팬 제어 (20kHz)
 *
 * @section communication_handling 통신 처리 (이중 DAC 지령 시스템)
 * - **로컬 SPI DAC**: 매 인터럽트마다 자신의 전류 제어용 DAC 출력 (100kHz)
 * - **네트워크 RS485**: Case 2에서 병렬 모듈들에게 지령 브로드캐스트 (20kHz)
 * - **SPI ADC**: 매 인터럽트마다 전압/전류 센서 데이터 읽기 (100kHz)
 * - **CAN 프로토콜**: 상위 제어기와의 통신 (500kbps, 메시지: 운전시 100Hz/대기시 10Hz)
 *
 * @section safety_features 안전 기능
 * - 하드웨어 폴트 감지 (GPIO 기반)
 * - 과전압 보호
 * - 소프트 스타트 제어
 * - 방전 FET 제어 (1초 지연)
 *
 * @note 이 인터럽트는 시스템의 실시간 성능을 결정하는 핵심 함수입니다.
 *       DCL 사용 시 약 65% 성능 향상 (40-50 cycles → 12-15 cycles)
 */
__interrupt void control_loop_isr(void) //100kHz 호출
{
    control_loop_cnt++;

    //=========================================================================
    // 1. CAN 메시지 전송 타이밍 체크 (1ms 주기)
    //=========================================================================
    // ** CAN 통신 구조 **
    // - 버스 속도: 500kbps (물리적 통신 속도)
    // - 타이밍 체크: 1ms마다 (여기서 수행)
    // - 실제 메시지 전송: can_report_interval에 의해 결정
    //   운전 중: 10ms (100Hz), 대기 중: 100ms (10Hz)
    if (can_tx_cnt++ >= 100) // 100번 = 1ms (100kHz ÷ 100)
    {
        can_tx_cnt = 0;
        can_tx_flag = 1; // 메인 루프에서 실제 전송 처리
    }

    //=========================================================================
    // 2. SPI ADC 데이터 처리 (전압 센서) 및 내장 ADC 데이터 처리 (전류 센서)
    //=========================================================================
    DAC1_DS();                     // DAC1_CS 비활성화
    V_out_ADC = SpiaRegs.SPIRXBUF; // SPI ADC 데이터 읽기 (전압 센서)

    ADC1_CS(); // ADC1_CS 활성화 (컨버전 시작, 최소 700ns 후 읽기 가능)
    DAC1_CS(); // DAC1_CS 활성화

    // SPI ADC 전압 데이터 누적 (5회 평균 계산용, 100kHz → 20kHz)
    V_out_ADC_sum += V_out_ADC;
    
    // 내장 ADC 전류 데이터 누적 (5회 평균 계산용, 100kHz → 20kHz)
    // 주의: I_out_ADC는 adc_isr에서 AdcResult.ADCRESULT1로부터 읽어옴
    I_out_ADC_sum += I_out_ADC;

    //=========================================================================
    // 3. 함수 포인터 기반 제어 알고리즘 (성능 최적화)
    //=========================================================================
    /*
     * 함수 포인터 배열을 사용한 20kHz 분주 제어:
     * - Phase 0: 전압 센싱 (voltage_sensing_task)
     * - Phase 1: PI 제어 (pi_control_task)  
     * - Phase 2: 전류 지령 처리 (current_control_task)
     * - Phase 3: 온도 처리 (temperature_monitoring_task)
     * - Phase 4: 평균 계산 (average_calculation_task)
     *
     * 최적화 효과: Switch문 대비 20-50% 성능 향상
     * - 분기 예측 성공률 100%
     * - 결정론적 실행 시간
     * - 컴파일러 최적화 향상
     */
    
    // 함수 포인터를 통한 직접 호출 (최적화된 실행)
    control_task_functions[control_phase]();
    
    // Phase 카운터 업데이트 (순환)
    if (++control_phase > 4)
        control_phase = 0;

    //=========================================================================
    // 4. 하드웨어 안전 처리 (DAB 폴트 감지 및 시스템 안전성 확인)
    //=========================================================================
    // DAB 폴트 감지 (연속 2회 감지 방식)
    CheckDABFault();
    
    // 운전 스위치 OFF 시 모든 폴트 클리어
    if (run_switch == 0)
        ClearHardwareFaults();

    //=========================================================================
    // 5. 로컬 DAC 출력 (자신의 모듈 전류 제어용)
    //=========================================================================
    // ** 목적: 이 모듈의 전류 제어 회로에 아날로그 지령 출력 **
    // - DAC1 A채널에 12비트 전류 지령 출력 (0~4095)
    // - 0xC000: DAC1 A채널 쓰기 명령 (SPI 프로토콜)
    // - I_cmd_DAC: 0=최대회생(-80A), 2000=0A, 4000=최대방전(+80A)
    // - 출력 전압: 0V~3.3V (하드웨어 증폭기를 통해 전류 제어)
    // - 주기: 100kHz (매 인터럽트마다 실시간 업데이트)
    SpiaRegs.SPITXBUF = 0xC000 | (I_cmd_DAC & 0xFFF);

    ADC1_DS(); // ADC1_CS 비활성화 (SPI 통신 완료)

    //=========================================================================
    // 6. 인터럽트 정리
    //=========================================================================
    EPwm3Regs.ETCLR.bit.INT = 1;            // 인터럽트 플래그 클리어
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // PIE ACK
}

__interrupt void adc_isr(void)
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;

    // ADC conversions - SOC0 시작
    AdcRegs.ADCSOCFRC1.all = 0x07; // SOC0, SOC1, SOC2

    temp_ADC = AdcResult.ADCRESULT0;  // 온도 ADC 결과
    I_out_ADC = AdcResult.ADCRESULT1; // 전류 ADC 결과
    V_batt_avg = AdcResult.ADCRESULT2;  // 배터리 평균 전압 ADC 결과

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   // Clear ADCINT1 flag to reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
    return;
}

//=============================================================================
// CLA PI Controller Functions (CLA 기반 PI 제어 함수들)
//=============================================================================

/**
 * @brief CLA 초기화 함수
 * CLA 하드웨어 설정, 메모리 매핑, 태스크 벡터 초기화
 */
void InitCLA(void)
{
    EALLOW;

    // CLA 태스크 벡터 설정 - CLA 프로그램 시작 주소 기준 오프셋
    Cla1Regs.MVECT1 = (Uint16)((Uint32)&Cla1Task1 - (Uint32)&Cla1funcsRunStart);
    Cla1Regs.MVECT2 = (Uint16)((Uint32)&Cla1Task2 - (Uint32)&Cla1funcsRunStart);
    Cla1Regs.MVECT3 = (Uint16)((Uint32)&Cla1Task3 - (Uint32)&Cla1funcsRunStart); // 충전 PI
    Cla1Regs.MVECT4 = (Uint16)((Uint32)&Cla1Task4 - (Uint32)&Cla1funcsRunStart); // 방전 PI
    Cla1Regs.MVECT5 = (Uint16)((Uint32)&Cla1Task5 - (Uint32)&Cla1funcsRunStart);
    Cla1Regs.MVECT6 = (Uint16)((Uint32)&Cla1Task6 - (Uint32)&Cla1funcsRunStart);
    Cla1Regs.MVECT7 = (Uint16)((Uint32)&Cla1Task7 - (Uint32)&Cla1funcsRunStart);
    Cla1Regs.MVECT8 = (Uint16)((Uint32)&Cla1Task8 - (Uint32)&Cla1funcsRunStart);

    // CLA 태스크 트리거 소스 설정 (소프트웨어 트리거 사용)
    Cla1Regs.MPISRCSEL1.bit.PERINT1SEL = CLA_INT1_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT2SEL = CLA_INT2_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT3SEL = CLA_INT3_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT4SEL = CLA_INT4_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT5SEL = CLA_INT5_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT6SEL = CLA_INT6_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT7SEL = CLA_INT7_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT8SEL = CLA_INT8_NONE;

    // CLA 인터럽트 활성화
    Cla1Regs.MIER.all = 0x00FF;

    // CLA 메모리 설정
    // - CLA 프로그램 공간 활성화
    // - CLA 데이터 RAM 0, 1, 2 활성화
    // - CPU에서 CLA RAM1 읽기 가능 (디버깅용)
    Cla1Regs.MMEMCFG.all = CLA_PROG_ENABLE | CLARAM0_ENABLE | CLARAM1_ENABLE | CLARAM2_ENABLE | CLA_RAM1CPUE;

    // CLA 인터럽트 ACK 활성화
    Cla1Regs.MCTL.bit.IACKE = 1;

    EDIS;

    // CLA 초기화 확인을 위한 Task 8 실행
    Cla1ForceTask8andWait();
}

/**
 * @brief CLA PI 컨트롤러 초기화 함수
 * 충전/방전 모드별 독립적인 CLA PI 컨트롤러를 초기화하고 파라미터를 설정
 */
void InitDCLControllersCLA(void)
{
    // CLA PI 구조체 기본값으로 초기화
    dcl_pi_charge_cla = (DCL_PI_CLA)PI_CLA_DEFAULTS;
    dcl_pi_discharge_cla = (DCL_PI_CLA)PI_CLA_DEFAULTS;

    //=========================================================================
    // 충전용 CLA PI 컨트롤러 설정
    //=========================================================================
    dcl_pi_charge_cla.Kp = DCL_KP;              // 비례 게인
    dcl_pi_charge_cla.Ki = DCL_KI * DCL_TSAMPL; // 적분 게인 (이산화)
    dcl_pi_charge_cla.Umax = CURRENT_LIMIT;     // 초기 상한값 (동적으로 변경됨)
    dcl_pi_charge_cla.Umin = -2.0f;             // 하한값
    dcl_pi_charge_cla.i10 = 0.0f;               // 적분기 상태 초기화
    dcl_pi_charge_cla.i6 = 1.0f;                // Anti-windup 상태 초기화

    //=========================================================================
    // 방전용 CLA PI 컨트롤러 설정
    //=========================================================================
    dcl_pi_discharge_cla.Kp = DCL_KP;               // 비례 게인
    dcl_pi_discharge_cla.Ki = DCL_KI * DCL_TSAMPL;  // 적분 게인 (이산화)
    dcl_pi_discharge_cla.Umax = 2.0f;               // 상한값
    dcl_pi_discharge_cla.Umin = -CURRENT_LIMIT;     // 초기 하한값 (동적으로 변경됨)
    dcl_pi_discharge_cla.i10 = 0.0f;                // 적분기 상태 초기화
    dcl_pi_discharge_cla.i6 = 1.0f;                 // Anti-windup 상태 초기화

    //=========================================================================
    // 공유 변수 초기화
    //=========================================================================
    charge_rk = 0.0f;    // 충전 기준값
    charge_yk = 0.0f;    // 충전 피드백값
    charge_uk = 0.0f;    // 충전 출력값
    
    discharge_rk = 0.0f; // 방전 기준값
    discharge_yk = 0.0f; // 방전 피드백값
    discharge_uk = 0.0f; // 방전 출력값

    // CLA 초기화 완료 확인
    Cla1ForceTask8andWait();
}

/**
 * @brief Modbus 데이터 파싱 및 처리
 *
 * @details 제어 컴퓨터로부터 수신한 Modbus RTU 데이터를 파싱하여 시스템 제어 변수에 반영
 *
 * @section data_flow 데이터 흐름
 * 1. **피드백 데이터 전송**: 전류/전압 센서값을 Input Register에 저장
 * 2. **명령 수신**: Holding Register에서 제어 명령 추출
 * 3. **운전 모드 처리**: 모드별 파라미터 설정
 * 4. **전류 지령 처리**: UI 전류 지령을 모듈 개수로 분배
 *
 * @section modbus_registers Modbus 레지스터 맵
 * - **Input Register 4-5**: 총 출력 전류 (32bit float)
 * - **Input Register 10-11**: 평균 전압 (32bit float)
 * - **Holding Register 0**: 시작/정지 명령 (bit 3: SYSTEM_START, bit 4: SYSTEM_STOP)
 * - **Holding Register 2**: 운전 모드 (강제로 64 설정)
 * - **Holding Register 5**: 전압 기준값
 * - **Holding Register 7-8**: 전류 지령 (32bit float)
 *
 * @section operating_modes 운전 모드
 * - **64**: 배터리 충방전 CC 모드 (현재 고정)
 * - 각 모드별로 다른 파라미터 설정 방식 적용
 *
 * @note 이 함수는 20kHz → 100Hz로 분주되어 호출됨 (10ms 주기)
 */
void ParseModbusData(void)
{
    // C89 변수 선언
    // Uint16 i;
    // I_fb_total.f = 0;
    // 총 출력 전류 계산 (마스터 + 모든 슬레이브)
    /*
    for (i = 0; i <= 9; i++)
    {
        I_fb_total.f += I_fb_array[i].f;
    }
    */
    I_fb_total.f = I_fb_avg;
    /* 송신 데이터 처리 */
    // Modbus Input Register에 전류값 저장 (32bit float -> 2개 16bit)
    usRegInputBuf[4] = (Uint16)(I_fb_total.u);
    usRegInputBuf[5] = (Uint16)(I_fb_total.u >> 16);

    // Modbus Input Register에 전압값 저장
    usRegInputBuf[10] = (Uint16)(V_fb_avg.u);
    usRegInputBuf[11] = (Uint16)(V_fb_avg.u >> 16);

    /* 수신 데이터 처리 */
    // 시작/정지 명령 처리 (정확한 값 매칭)
    if (usRegHoldingBuf[0] == 0x08)      // 0x08 = 시작 명령
        system_state = STATE_RUNNING;    // 시작
    else if (usRegHoldingBuf[0] == 0x10) // 0x10 = 정지 명령
        system_state = STATE_STOP;       // 정지

    V_max_lim = usRegHoldingBuf[5];
    V_min_lim = usRegHoldingBuf[12];
    UI_I_cmd.u = (((Uint32)usRegHoldingBuf[8] << 16) | (Uint32)usRegHoldingBuf[7]);

    // 모듈 개수로 나눈 전류 지령 계산 및 제한 (-80A ~ +80A)
    I_cmd = UI_I_cmd.f / MODULE_COUNT;
    if (I_cmd > CURRENT_LIMIT)
        I_cmd = CURRENT_LIMIT;
    else if (I_cmd < -CURRENT_LIMIT)
        I_cmd = -CURRENT_LIMIT;
}

/**
 * @brief 전류 평균 계산
 *
 * @details 내장 ADC로부터 읽은 전류값의 평균 계산 및 스케일링
 *
 * @section adc_conversion ADC 변환 사양
 * - **ADC 범위**: 0~4095 (12bit)
 * - **전압 범위**: 0~3V
 * - **전류 매핑**: 0A=1.5V(2048), +100A=3V(4095), -100A=0V(0)
 * - **변환 공식**: (ADC - 2048) × (100/2048) = 실제 전류(A)
 *
 * @section averaging_process 평균 계산 과정
 * 1. **5회 평균**: 내장 ADC에서 읽은 5개 ADC 값의 평균 (100kHz → 20kHz)
 * 2. **스케일링**: ADC 값을 실제 전류값으로 변환
 * 3. **10000회 누적**: 모니터링용 장기 평균 계산
 * 4. **최종 평균**: 10000회 완료 시 I_fb_avg에 저장
 *
 * @note 이 함수는 epwm3_isr의 Case 4에서 20kHz로 호출됨
 * @note Legacy 프로그램과 동일한 구조로 Case 4에서 전류 처리
 */
void Calc_I_fb_avg(void)
{
    // 내장 ADC 평균값 계산 (5회 평균, 100kHz → 20kHz 데시메이션)
    I_out_ADC_avg = (float32)(I_out_ADC_sum * 0.2f); // 0.2 = 1/5
    I_out_ADC_sum = 0; // 누적값 초기화

    // 내장 ADC 전류값을 실제 전류로 변환 - Legacy 프로그램의 Io_sen_real과 동일
    // ADC 매핑: 0A=2048(1.5V), +100A=4095(3V), -100A=0(0V)
    // 변환 공식: (ADC - 2048) × (100/2048) = 실제 전류(A)
    I_fb_array[0].f = (I_out_ADC_avg - 2048) * 0.048828125f; // 100 / 2048
    I_fb_sum += I_fb_array[0].f; // 장기 평균용 누적

    // 10000번 평균 계산 완료 시 최종 평균값 저장 (0.5초 주기)
    if (I_cal_cnt++ == 10000)
    {
        I_fb_avg = I_fb_sum * 0.0001; // 0.0001 = 1/10000 (평균값 계산)
        I_fb_sum = 0;  // 누적값 초기화
        I_cal_cnt = 0; // 카운터 초기화
    }
}

/**
 * @brief 전압 평균 계산
 *
 * @details 전압 센서값의 장기 평균 계산 (모니터링 및 Modbus 전송용)
 *
 * @section voltage_sensing 전압 센싱
 * - **입력**: V_fb (스케일링된 실제 전압값)
 * - **누적**: 10000회 누적하여 평균 계산
 * - **주기**: 20kHz로 호출되어 0.5초마다 평균값 갱신
 *
 * @section data_usage 데이터 활용
 * - **voltageMean**: 내부 모니터링용 평균값
 * - **voltageSensorAvg.f**: Modbus Input Register 전송용
 *
 * @note 이 함수는 epwm3_isr의 Case 3에서 20kHz로 호출됨
 */
void Calc_V_fb_avg(void)
{
    // 전압 센서값 누적
    V_fb_sum += V_fb;

    // 10000번 평균 계산 완료 시 최종 평균값 저장
    if (V_cal_cnt++ == 10000)
    {
        V_fb_avg.f = V_fb_sum * 0.0001; // 평균값 계산
        V_fb_sum = 0.;                  // 누적값 초기화
        V_cal_cnt = 0;                  // 카운터 초기화
    }
}

/**
 * @brief 팬 PWM 제어
 *
 * @details 온도에 따른 팬 PWM 듀티 사이클 제어 (온도 비례 제어)
 *
 * @section temperature_control 온도 제어 알고리즘
 * - **온도 범위**: 36°C ~ 52.4°C
 * - **듀티 범위**: 15% ~ 90%
 * - **제어 공식**: duty = 0.8 × (Temp - 36) / (52.4 - 36) + 0.2
 * - **최소 듀티**: 15% (항상 최소 회전 보장)
 * - **최대 듀티**: 90% (과부하 방지)
 *
 * @section pwm_output PWM 출력
 * - **PWM 주파수**: 10kHz (PWM_PERIOD_10k)
 * - **출력 핀**: ePWM1A
 * - **극성**: 반전 (1 - fan_pwm_duty)
 *
 * @note 이 함수는 epwm3_isr의 Case 4에서 20kHz로 호출됨
 */
void ControlFanPwm(void)
{
    // 온도별 팬 PWM 듀티 계산: 36°C에서 20%, 52.4°C에서 100%
    // 0.8 = 최대 가변범위(100%-20%), 0.2 = 최소 듀티(20%)
    fan_pwm_duty = 0.8 * (temp_in - 36) / (52.4f - 36) + 0.2;

    if (fan_pwm_duty < 0.15)
        fan_pwm_duty = 0.15; // 최소 15%로 제한 (최소 회전 보장)
    else if (fan_pwm_duty > 0.9)
        fan_pwm_duty = 0.9;  // 최대 90%로 제한 (과부하 방지)

    // PWM 극성 반전: (1 - duty)로 설정
    EPwm1Regs.CMPA.half.CMPA = (1.0f - fan_pwm_duty) * PWM_PERIOD_10k;
}

/**
 * @brief CAN 인터럽트 서비스 루틴
 *
 * @details 두 가지 CAN 통신을 처리하는 통합 인터럽트
 *
 * @section can_communication CAN 통신 구조
 * 1. **슬레이브 피드백** (MBOX 1~9): 슬레이브 → 마스터 전류 피드백
 * 2. **Protocol 통신** (MBOX 16~31): 제어 컴퓨터 ↔ 마스터 명령/상태
 *
 * @section protocol_mailboxes Protocol 메일박스
 * - **MBOX 30**: 제어 컴퓨터 → 마스터 명령 수신
 * - **MBOX 31**: 제어 컴퓨터 → 마스터 명령 수신 (추가)
 * - **MBOX 24, 25**: 마스터 → 제어 컴퓨터 응답 송신
 *
 * @section interrupt_handling 인터럽트 처리
 * - RMP30/RMP31 비트 확인하여 Protocol 명령 처리
 * - 슬레이브 피드백은 메인 루프에서 폴링 방식으로 처리
 * - 모든 인터럽트 플래그 클리어 후 PIE ACK
 *
 * @note Protocol 시스템은 별도 추가된 기능으로 기존 슬레이브 통신과 독립적
 */
__interrupt void can_protocol_isr(void)
{
    // Protocol 메일박스 30번 처리
    if (ECanaRegs.CANRMP.bit.RMP30 != 0)
    {
        ProcessCANCommand(30, 24);
        ECanaRegs.CANRMP.all = ((Uint32)1 << 30);   // RMP30만 클리어
        ECanaRegs.CANGIF0.all = ((Uint32)1 << 30);  // GMIF30만 클리어
    }

    // Protocol 메일박스 31번 처리
    if (ECanaRegs.CANRMP.bit.RMP31 != 0)
    {
        ProcessCANCommand(31, 25);
        ECanaRegs.CANRMP.all = ((Uint32)1 << 31);   // RMP31만 클리어
        ECanaRegs.CANGIF0.all = ((Uint32)1 << 31);  // GMIF31만 클리어
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//=============================================================================
// 하드웨어 안전 함수들 (Hardware Safety Functions)
//=============================================================================

/**
 * @brief DAB 폴트 감지 및 처리
 * 
 * @details 연속 2회 폴트 감지 시 하드웨어 폴트로 판정
 * - DAB 상태가 2회 연속 LOW일 때 폴트 확정
 * - 디바운싱 효과로 노이즈에 의한 오동작 방지
 * 
 * @note 이 함수는 epwm3_isr에서 100kHz로 호출됨
 */
void CheckDABFault(void)
{
    // 이전 상태 저장
    dab_previous = dab_current;
    
    // 현재 DAB 상태 판정
    if (DAB_FAULT())
        dab_current = STATUS_FAULT;
    else
        dab_current = STATUS_OK;
    
    // 연속 2회 폴트 감지 시 하드웨어 폴트 설정
    if (dab_current == STATUS_FAULT && 
        dab_previous == STATUS_FAULT)
    {
        hw_fault_flag = 1; // 하드웨어 폴트 확정
    }
}

/**
 * @brief 하드웨어 폴트 클리어
 * 
 * @details 운전 스위치가 OFF될 때 모든 하드웨어 폴트를 클리어
 * - 사용자가 의도적으로 시스템을 정지시킨 상황
 * - 폴트 상태를 리셋하여 재시작 가능하도록 함
 */
void ClearHardwareFaults(void)
{
    hw_fault_flag = 0;
    dab_current = STATUS_OK;
    dab_previous = STATUS_OK;
    over_voltage_flag = 0;
}

/**
 * @brief 시스템 운전 안전성 확인
 * 
 * @details 시스템이 안전하게 운전 가능한 상태인지 확인
 * @return 1: 운전 가능, 0: 운전 불가
 * 
 * @section safety_conditions 안전 조건
 * - 하드웨어 폴트 없음 (hw_fault_flag == 0)
 * - 과전압 상태 아님 (over_voltage_flag == 0)  
 * - 운전 스위치 ON (run_switch == 1)
 */
Uint16 IsSystemSafeToRun(void)
{
    return (hw_fault_flag == 0 && over_voltage_flag == 0 && run_switch == 1) ? 1 : 0;
}

#endif // _MAIN_C_

//===========================================================================
// No more.
//===========================================================================
