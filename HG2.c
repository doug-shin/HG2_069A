/**
 * @file HG2.c
 * @brief 35kW DC-DC 컨버터 제어 시스템 메인 소스 파일
 *
 * @details
 * 이 파일은 35kW DC-DC 컨버터의 메인 제어 로직을 포함합니다.
 * 주요 기능:
 * - Modbus RTU 통신을 통한 전류/전압 지령 수신
 * - PI 제어를 통한 전압 제어 (충전/방전 모드)
 * - CAN 통신을 통한 슬레이브 모듈 관리
 * - 과전압/하드웨어 폴트 보호
 * - 온도 모니터링 및 팬 제어
 * - RS485 통신을 통한 슬레이브 전류 지령 전송
 *
 * @section control_algorithm 제어 알고리즘
 * - **100kHz ePWM3 인터럽트**: 메인 제어 루프
 * - **20kHz 분주**: 5단계 순차 처리 (전압센싱, PI제어, 통신, 온도, 평균계산)
 * - **PI 제어**: 충전/방전 모드별 독립적인 PI 컨트롤러
 * - **소프트 스타트**: 전류 램프업을 통한 안전한 기동
 *
 * @section communication 통신 시스템
 * - **Modbus RTU**: 제어 컴퓨터와의 명령/상태 통신 (38.4kbps)
 * - **CAN**: 슬레이브 모듈 피드백 수신 (500kbps)
 * - **RS485**: 슬레이브 전류 지령 브로드캐스팅 (5.625Mbps)
 *
 * @section safety_features 안전 기능
 * - 과전압 보호 (1100V 이상)
 * - 하드웨어 폴트 감지 (GPIO 기반)
 * - 워치독 타이머
 * - 소프트 스타트 제어
 *
 * @author 개발팀
 * @date 2024
 * @version 2.0
 *
 * @copyright Copyright (c) 2024
 *
 * @note 이 파일은 TI F28069 마이크로컨트롤러용으로 작성되었습니다.
 *       DCL 라이브러리를 사용한 최적화된 PI 제어기를 포함합니다.
 */

/*===========================================================================
 * 미완료 작업 (우선순위별)
 *===========================================================================
 * [우선순위 중]
 * - 슬레이브 보드 CAN ID 중복 검출하여 마스터로 전송
 * - 슬레이브에서 마스터의 485 수신 실패 시 CAN 폴트 전송
 *
 * [우선순위 저]
 * - 리셋 후 초기 FAN 미동작 원인 검토
 * - 터미널에 전류값 소수점 표시
 * - SCI 에러 처리: if(SciaRegs.SCIRXST.bit.PE) // 패리티 에러 등
 * - 에러 발생 시 카운트하여 저장
 * - Flash vs RAM 실행시간 비교 분석
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
#include "DCLF32.h"

// 프로젝트 헤더 파일
#include "HG2.h"

// CAN Protocol 관련 외부 변수 선언
extern PROTOCOL_INTEGRATED protocol; // 프로토콜 구조체

// CAN 쉐도우 레지스터 정의
volatile struct ECAN_REGS ECanaShadow;

void InitEPwm1(void);
void InitEPwm3(void);
void spi_init(void);
void eCana_config(void);
void stra_xmit(Uint8 *buff, Uint16 Length);

#pragma CODE_SECTION(scia_txFifo_isr, "ramfuncs");
#pragma CODE_SECTION(cpuTimer1ExpiredISR, "ramfuncs");
#pragma CODE_SECTION(scibRxReadyISR, "ramfuncs");
#pragma CODE_SECTION(scibTxEmptyISR, "ramfuncs");
#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs");
#pragma CODE_SECTION(cpu_timer2_isr, "ramfuncs");
#pragma CODE_SECTION(ParseModbusData, "ramfuncs");
#pragma CODE_SECTION(stra_xmit, "ramfuncs");
#pragma CODE_SECTION(epwm3_isr, "ramfuncs");
#pragma CODE_SECTION(ecan0_isr, "ramfuncs");
#pragma CODE_SECTION(adc_isr, "ramfuncs");

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void spi_isr(void);
__interrupt void scia_txFifo_isr(void);
__interrupt void ecan0_isr(void);
__interrupt void adc_isr(void);

extern __interrupt void scibRxReadyISR(void);
extern __interrupt void scibTxEmptyISR(void);
extern __interrupt void cpuTimer1ExpiredISR(void);

extern USHORT usRegInputBuf[REG_INPUT_NREGS];
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

//*****************************************************************************
// DAC Reference Voltage Configuration
// DAC output range is 0V to 2*Vref (e.g., 1.024V ref → 0~2.048V output)
//*****************************************************************************
#define FAST_REF1 0xD001 // 내부 1.024V
#define SLOW_REF1 0x9001
#define FAST_REF2 0xD002 // 내부 2.048V
#define SLOW_REF2 0x9002
#define FAST_REF_E 0xD000 // 외부
#define SLOW_REF_E 0x9000

// 전송 제어 문자
#define STX (0x02) // Start of Text, 본문의 개시 및 정보 메세지 헤더의 종료를 표시
#define ETX (0x03) // End of Text, 본문의 종료를 표시한다
#define DLE (0x10) // Data link escape, 뒤따르는 연속된 글자들의 의미를 바꾸기 위해 사용, 주로 보조적 전송제어기능을 제공
Uint16 gSciTxBuf[4] = {STX, 0, 0, ETX};

Uint16 cpu_timer0_cnt = 0, cpu_timer1_cnt = 0, cpu_timer2_cnt = 0;
Uint16 spi_interrupt_cnt = 0;
Uint32 epwm3_isr_cnt = 0;

Uint16 force_reset_test = 0, fifo_err_cnt = 0;
Uint16 can_tx_cnt = 0, can_tx_flag = 0;
Uint16 can_rx_fault_cnt[11];
Uint16 detect_module_num = 1;

// CAN 메일박스 배열 포인터
static struct MBOX *mbox_array = (struct MBOX *)&ECanaMboxes;

Uint16 Run_on_count = 0;
Uint16 timer_10ms = 0;

// ADC 및 기본 설정
float32 V_ref = 5.0f;

float32 V_out_delta = 0., V_out_monitor = 0.;
Uint16 hw_fault = 0, gpio_dab_fault = 0, gpio_dab_fault_prev = 0;

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
 * - CAN 보고 처리 (Protocol 시스템)
 * - CAN 송수신 처리 (슬레이브 피드백, 1kHz)
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
    InitSysCtrl(); // 90MHz PLL 설정

    // Flash 운영을 위한 RAM으로 복사 (타이밍 크리티컬 코드)
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

    //=========================================================================
    // 2. GPIO 초기화
    //=========================================================================
    gpio_config();
    ReadGpioInputs(); // Board ID 읽기 및 초기 CAN ID 설정

    // PWM 및 SPI GPIO 설정
    InitEPwm1Gpio();
    InitEPwm3Gpio();
    InitSpiaGpio();

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
    InitSciaGpio();
    scia_fifo_init();
    scia_init();

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
    eCana_config();
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
    spi_init();

    //=========================================================================
    // 7. 타이머 초기화
    //=========================================================================
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer2, 90, 50); // 20kHz 인터럽트

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // 타이머 정지
    EDIS;

    //=========================================================================
    // 8. 인터럽트 벡터 할당
    //=========================================================================
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.ADCINT1 = &adc_isr;
    PieVectTable.SPIRXINTA = &spi_isr;
    PieVectTable.SCIRXINTB = &scibRxReadyISR;
    PieVectTable.SCITXINTB = &scibTxEmptyISR;
    PieVectTable.TINT1 = &cpuTimer1ExpiredISR;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.TINT2 = &cpu_timer2_isr;
    PieVectTable.ECAN0INTA = &ecan0_isr;
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
    IER |= M_INT6;  // SPI
    IER |= M_INT9;  // CAN, SCI
    IER |= M_INT13; // Timer 1

    // PIE 인터럽트 활성화
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // ePWM3
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADC
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1; // SPI
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
    // 13. DCL PI 컨트롤러 초기화
    //=========================================================================
    InitDCLControllers(); // DCL 기반 PI 컨트롤러 초기화

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
        mainLoopCount++; // 디버깅용 free-run 카운터

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

        // CAN 보고 처리
        if (can_report_flag == 1)
        {
            can_report_flag = 0;
            SendCANReport(16);
        }

        // CAN 송수신 처리 (1kHz)
        if (can_tx_flag == 1)
        {
            Uint32 rmp_status;
            can_tx_flag = 0;

            // 운전 상태에 따른 CAN 메시지 설정
            if (system_state == STATE_RUNNING)
                ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0xA0;
            else
                ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0;

            ECanaRegs.CANTRS.all = 0x00000001; // 전송 요청

            // CAN 메일박스 상태 읽기 (성능 최적화)
            rmp_status = ECanaRegs.CANRMP.all & 0x000003FE; // MBOX1~9

            // 슬레이브 모듈별 전류값 수신 처리 (배열에 직접 저장)
            for (i = 1; i <= 9; i++)
            {
                if (rmp_status & (1 << i))
                {
                    // CAN 데이터 수신됨 (배열에 직접 저장)
                    I_fb_array[i].u = mbox_array[i].MDL.all;
                    ECanaRegs.CANRMP.all = ((Uint32)1 << i); // 해당 비트만 클리어
                    can_rx_fault_cnt[i] = 0;
                }
                else
                {
                    // CAN 데이터 수신 안됨 (타임아웃 처리)
                    if (can_rx_fault_cnt[i]++ >= 10000)
                    {
                        I_fb_array[i].u = 0;
                        can_rx_fault_cnt[i] = 10000;
                    }
                }
            }

            // 활성 모듈 개수 계산
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
// 유틸리티 함수
//=============================================================================

/**
 * @brief 문자열 전송 함수 (SCI-A, RS485)
 *
 * @param buff 전송할 데이터 버퍼
 * @param Length 전송할 데이터 길이
 *
 * @details RS485를 통한 슬레이브 전류 지령 브로드캐스팅
 *
 * @section rs485_communication RS485 통신 사양
 * - **속도**: 5.625Mbps
 * - **주기**: 0.1ms (10kHz)
 * - **데이터**: 4바이트 (STX, 전류지령 Low, 전류지령 High, ETX)
 * - **방식**: 브로드캐스팅 (모든 슬레이브가 동일한 지령 수신)
 *
 * @section data_format 데이터 포맷
 * - **STX (0x02)**: 시작 문자
 * - **Data Low**: 전류 지령 하위 바이트
 * - **Data High**: 전류 지령 상위 바이트
 * - **ETX (0x03)**: 종료 문자
 *
 * @note 이 함수는 epwm3_isr의 Case 2에서 20kHz로 호출됨
 */
void stra_xmit(Uint8 *buff, Uint16 Length)
{
    Uint16 i;

    for (i = 0; i < Length; i++)
    {
        SciaRegs.SCITXBUF = buff[i];
        while (SciaRegs.SCICTL2.bit.TXRDY == 0)
            ;
    }
}

//=============================================================================
// 인터럽트 서비스 루틴
//=============================================================================

/**
 * @brief CPU Timer 0 인터럽트 (100kHz)
 */
__interrupt void cpu_timer0_isr(void)
{
    cpu_timer0_cnt++;
    CpuTimer0Regs.TCR.bit.TIF = 1; // 인터럽트 플래그 클리어
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/**
 * @brief CPU Timer 2 인터럽트 (50us, 20kHz)
 */
__interrupt void cpu_timer2_isr(void)
{
    cpu_timer2_cnt++;
}

/**
 * @brief SPI 인터럽트 서비스 루틴
 */
__interrupt void spi_isr(void)
{
    spi_interrupt_cnt++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

/**
 * @brief ePWM1 인터럽트 서비스 루틴 (팬 PWM 제어용)
 */
__interrupt void epwm1_isr(void)
{
    EPwm1Regs.ETCLR.bit.INT = 1; // 인터럽트 플래그 클리어
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

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
 * @section communication_handling 통신 처리
 * - **SPI ADC**: 매 인터럽트마다 전류 ADC 데이터 읽기 (100kHz)
 * - **RS485**: Case 2에서 전류 지령 브로드캐스팅 (20kHz)
 * - **CAN 타이밍**: 1kHz 전송 타이밍 관리
 * - **DAC 출력**: 매 인터럽트마다 전류 지령 DAC 출력 (100kHz)
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
__interrupt void epwm3_isr(void) //100kHz 호출
{
    // C89 변수 선언
    Uint16 i;

    epwm3_isr_cnt++;

    //=========================================================================
    // 1. CAN 전송 타이밍 관리 (1kHz)
    //=========================================================================
    if (can_tx_cnt++ >= 100) // 1ms 마다 CAN 전송
    {
        can_tx_cnt = 0;
        can_tx_flag = 1; // 메인 루프에서 처리
    }

    //=========================================================================
    // 2. SPI ADC 데이터 처리
    //=========================================================================
    DAC1_DS();                     // DAC1_CS 비활성화
    I_out_ADC = SpiaRegs.SPIRXBUF; // ADC 데이터 읽기

    ADC1_CS(); // ADC1_CS 활성화 (컨버전 시작)
    DAC1_CS(); // DAC1_CS 활성화

    // ADC 데이터 누적 (평균 계산용)
    I_out_ADC_sum += I_out_ADC;

    //=========================================================================
    // 3. 100kHz -> 20kHz 분주 제어 알고리즘
    //=========================================================================
    /*
     * 20kHz로 PI 제어 실행:
     * - case 0: 전압 센싱 (20kHz)
     * - case 1: 통합 PI 제어 (충전/방전 자동 선택) - **테스트용**
     * - case 2: 전류 지령 처리 및 통신
     * - case 3: 온도 처리 및 CAN 보고
     * - case 4: 평균 계산 및 팬 제어
     *
     * 총 처리 시간: 기존과 동일, case 1에서만 최적화 테스트
     */
    switch (control_phase)
    {
    //---------------------------------------------------------------------
    // Case 0: 전압 센싱 (20kHz)
    //---------------------------------------------------------------------
    case 0:
        // ADC 평균값 계산 (5회 평균)
        I_fb_array[0].f = I_out_ADC_avg = (float32)(I_out_ADC_sum * 0.2f);
        I_out_ADC_sum = 0;

        // 전압 센싱 및 스케일링
        V_fb = V_out = (V_ref * (float32)V_out_ADC * 0.0000152f - 0.1945f) * 250.0f * 1.04047f; // 보정계수 적용 및 제어기용 전압값 설정

        control_phase++;
        break;

    //---------------------------------------------------------------------
    // Case 1: 통합 PI 제어 (충전/방전 자동 선택)
    //---------------------------------------------------------------------
    case 1:
        // PI 제어기 선택 실행 (플래그에 따라 DCL 또는 기존 방식)
        if (use_dcl_controller)
        {
            PIControlDCL(); // DCL 기반 최적화된 PI 제어 (권장)
        }
        else
        {
            PIControlUnified(); // 기존 PI 제어 (호환성용)
        }

        control_phase++;
        break;

    //---------------------------------------------------------------------
    // Case 2: 전류 지령 처리 및 통신 (20kHz)
    //---------------------------------------------------------------------
    case 2:
        // 전류 지령 제한 처리
        if (I_cmd > I_ss)
            I_cmd_PI = I_ss;
        else if (I_cmd < -I_ss)
            I_cmd_PI = -I_ss;
        else
            I_cmd_PI = I_cmd;

        // PI 출력 제한 적용
        if (I_cmd_PI > V_max_control_output)
            I_cmd_control_output = V_max_control_output;
        else if (I_cmd_PI < V_min_control_output)
            I_cmd_control_output = V_min_control_output;
        else
            I_cmd_control_output = I_cmd_PI;

        // 소프트 스타트 처리
        I_ss += I_MAX * 0.00005f;
        if (I_ss > I_MAX)
            I_ss = I_MAX;

        // 방전 FET 제어 (system_state에 따른 1초 지연)
        if (system_state == STATE_STOP)
        {
            I_ss = 0;
        }

        if (system_state == STATE_RUNNING)
        {
            if (Run_on_count++ >= 20000)
            { // 1초 후 방전 FET OFF
                Run_on_count = 20000;
                GpioDataRegs.GPADAT.bit.GPIO19 = 1; // 방전 FET OFF
            }
        }
        else
        {
            Run_on_count = 0;
            GpioDataRegs.GPADAT.bit.GPIO19 = 0; // 방전 FET ON (지연 없음)
        }

        // DAC 출력값 계산 및 제한
        I_cmd_DAC = (I_cmd_control_output * 25.0f + 2000.0f) * 1.0005f;
        if (I_cmd_DAC > 4095)
            I_cmd_DAC = 4095; // DAC 상한 제한

        // RS485 통신 데이터 준비 및 전송
        gSciTxBuf[1] = (UCHAR)(I_cmd_DAC & 0x00FF);
        gSciTxBuf[2] = (UCHAR)((I_cmd_DAC >> 8) & 0xFF);

        for (i = 0; i < 4; i++)
        {
            SciaRegs.SCITXBUF = gSciTxBuf[i] & 0x00FF; // 모든 슬레이브로 전류 지령 전송
        }

        // 과전압 보호
        if (V_out >= OVER_VOLTAGE)
            over_voltage_flag = 1;

        // 운전 조건 확인 (하드웨어 폴트, 과전압, 스위치 입력)
        if (hw_fault == 0 && over_voltage_flag == 0 && power_switch == 1)
        {
            system_state = STATE_RUNNING;
            GpioDataRegs.GPADAT.bit.GPIO17 = 0; // Buck Enable
        }
        else
        {
            system_state = STATE_STOP;
            GpioDataRegs.GPADAT.bit.GPIO17 = 1; // Buck Disable
        }

        control_phase++;
        break;

    //---------------------------------------------------------------------
    // Case 3: 온도 처리 및 CAN 보고 (20kHz)
    //---------------------------------------------------------------------
    case 3:
        // 온도 센서 처리
        temp_ADC_fb = (float32)(temp_ADC / 4095.0f * 3.0f);
        temp_ADC_fb_alt = (float32)(temp_ADC_fb * 5.0f / 3.0f); // 전압 변환
        temp_in = (float32)((temp_ADC_fb_alt * 20.4f) + 1.4f);  // 온도 변환

        // 전압 평균값 계산
        Calc_V_fb_avg();

        // CAN 보고 타이밍 관리 (0.05ms 단위로 카운트)
        if (can_report_counter++ >= can_report_interval)
        {
            can_report_counter = 0;
            can_report_flag = 1; // 메인 루프에서 처리
        }

        control_phase++;
        break;

    //---------------------------------------------------------------------
    // Case 4: 평균 계산 및 팬 제어 (20kHz)
    //---------------------------------------------------------------------
    case 4:
        // 10ms 타이머 (디지털 입력 및 Modbus 파싱용)
        if (timer_10ms++ >= 200)
        { // 200 * 0.05ms = 10ms
            timer_10ms = 0;
            parse_mb_flag = 1; // Modbus 파싱 플래그 설정
            ReadGpioInputs();  // DIP 스위치 및 스위치 입력 처리
        }

        // 전류 평균 계산 및 팬 PWM 제어
        Calc_I_fb_avg();
        ControlFanPwm();

        control_phase = 0; // 카운터 리셋
        break;

    //---------------------------------------------------------------------
    // Default: 에러 방지
    //---------------------------------------------------------------------
    default:
        control_phase = 0; // 안전을 위한 리셋
        break;
    }

    // 카운터 범위 보호
    if (control_phase > 4)
        control_phase = 0;

    //=========================================================================
    // 4. 하드웨어 폴트 감지 (GPIO 기반)
    //=========================================================================
    gpio_dab_fault_prev = gpio_dab_fault;
    gpio_dab_fault = !GpioDataRegs.GPBDAT.bit.GPIO39; // DAB_OK 신호 반전

    if (gpio_dab_fault == 1 && gpio_dab_fault_prev == 1)
        hw_fault = 1; // 연속 폴트 감지
    else if (power_switch == 0)
        hw_fault = 0; // 스위치 OFF시 폴트 해제

    //=========================================================================
    // 5. DAC 출력 및 SPI 정리
    //=========================================================================
    // DAC1 A채널에 전류 지령 출력 (12비트, 0~4095)
    SpiaRegs.SPITXBUF = 0xC000 | (I_cmd_DAC & 0xFFF);

    ADC1_DS(); // ADC1_CS 비활성화

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
    V_out_ADC = AdcResult.ADCRESULT2; // 전압 ADC 결과

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   // Clear ADCINT1 flag to reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
    return;
}

/**
 * @brief 고전압 PI 컨트롤러 (충전 모드용)
 *
 * @details 충전 모드(I_cmd >= 0)에서 사용되는 PI 제어기
 * - 고전압 지령(V_max_lim)과 실제 전압(V_out)의 차이를 PI 제어로 보상
 * - 출력은 양의 전류 지령으로 제한됨 (충전 방향)
 *
 * @section control_algorithm 제어 알고리즘
 * - **비례항**: Kp × (V_max_lim - V_out)
 * - **적분항**: Ki × Tsampl × ∑(V_max_lim - V_out)
 * - **출력 제한**: -2A ~ I_cmd_PI (전류 지령 상한)
 * - **Anti-windup**: 적분항 포화 방지
 *
 * @note 이 함수는 기존 호환성을 위해 유지되며, 새로운 통합 PI 제어기 사용 권장
 */
void PIControlHigh(void)
{
    V_max_error = V_max_lim - V_out; // 고전압 에러 계산
    V_max_kP_out = Kp * V_max_error; // P term (비례 제어)

    kI_out_prev = Ki * Tsampl * V_max_error; // I term (적분 제어) Tsampl=50E-6 (50us)
    V_max_kI_out = V_max_kI_out + kI_out_prev;

    // 적분 출력 제한 (Anti-windup)
    if (V_max_kI_out > I_cmd_PI)
        V_max_kI_out = I_cmd_PI;
    else if (V_max_kI_out < -2)
        V_max_kI_out = -2;

    // PI 출력 계산
    V_max_control_output = V_max_kP_out + V_max_kI_out;

    // PI 출력 제한
    if (V_max_control_output > I_cmd_PI)
        V_max_control_output = I_cmd_PI;
    else if (V_max_control_output < -2)
        V_max_control_output = -2;
}

/**
 * @brief 저전압 PI 컨트롤러 (방전 모드용)
 *
 * @details 방전 모드(I_cmd < 0)에서 사용되는 PI 제어기
 * - 저전압 지령(V_min_lim)과 실제 전압(V_out)의 차이를 PI 제어로 보상
 * - 출력은 음의 전류 지령으로 제한됨 (방전 방향)
 *
 * @section control_algorithm 제어 알고리즘
 * - **비례항**: Kp × (V_min_lim - V_out)
 * - **적분항**: Ki × Tsampl × ∑(V_min_lim - V_out)
 * - **출력 제한**: I_cmd_PI (전류 지령 하한) ~ 2A
 * - **Anti-windup**: 적분항 포화 방지
 *
 * @note 이 함수는 기존 호환성을 위해 유지되며, 새로운 통합 PI 제어기 사용 권장
 */
void PIControlLow(void)
{
    V_min_error = V_min_lim - V_out; // 저전압 에러 계산
    V_min_kP_out = Kp * V_min_error; // P term (비례 제어)

    kI_out_prev = Ki * Tsampl * V_min_error; // I term (적분 제어) Tsampl=50E-6 (50us)
    V_min_kI_out = V_min_kI_out + kI_out_prev;

    // 적분 출력 제한 (Anti-windup)
    if (V_min_kI_out > 2)
        V_min_kI_out = 2;
    else if (V_min_kI_out < I_cmd_PI)
        V_min_kI_out = I_cmd_PI;

    // PI 출력 계산
    V_min_control_output = V_min_kP_out + V_min_kI_out;

    // PI 출력 제한
    if (V_min_control_output > 2)
        V_min_control_output = 2;
    else if (V_min_control_output < I_cmd_PI)
        V_min_control_output = I_cmd_PI;
}

/**
 * @brief 통합 PI 제어 함수
 * 충전/방전 모드를 자동 선택하여 전압 제어 수행
 * 50% 계산량 절약 및 코드 중복 제거
 * 충전/방전별 독립적인 적분기 상태 유지로 bumpless transfer 구현
 */
void PIControlUnified(void)
{
    float32 error, kp_out, pi_out;
    float32 voltage_cmd, upper_limit, lower_limit;
    static float32 ki_accumulator_charge = 0;    // 충전용 적분기 상태
    static float32 ki_accumulator_discharge = 0; // 방전용 적분기 상태

    if (I_cmd >= 0)
    {
        // =====================================================================
        // 충전 모드 (I_cmd >= 0): 기존 PIControlHigh 로직
        // =====================================================================
        voltage_cmd = V_max_lim; // 고전압 지령
        upper_limit = I_cmd_PI;  // 상한: 양의 전류 제한
        lower_limit = -2;        // 하한: 최소 음의 값

        // 기존 V_max_ 변수들 업데이트 (호환성 유지)
        V_max_error = error = voltage_cmd - V_out;
        V_max_kP_out = kp_out = Kp * error;

        kI_out_prev = Ki * Tsampl * error;
        ki_accumulator_charge += kI_out_prev;
        V_max_kI_out = ki_accumulator_charge;

        // 적분 출력 제한 (Anti-windup)
        if (ki_accumulator_charge > upper_limit)
            ki_accumulator_charge = upper_limit;
        else if (ki_accumulator_charge < lower_limit)
            ki_accumulator_charge = lower_limit;
        V_max_kI_out = ki_accumulator_charge;

        // PI 출력 계산 및 제한
        V_max_control_output = pi_out = kp_out + ki_accumulator_charge;
        if (pi_out > upper_limit)
            V_max_control_output = upper_limit;
        else if (pi_out < lower_limit)
            V_max_control_output = lower_limit;

        // 방전용 적분기는 현재 PI 출력으로 초기화 (bumpless transfer)
        ki_accumulator_discharge = V_max_control_output - kp_out;
        if (ki_accumulator_discharge > 2.0f)
            ki_accumulator_discharge = 2.0f;
        else if (ki_accumulator_discharge < I_cmd_PI)
            ki_accumulator_discharge = I_cmd_PI;
    }
    else
    {
        // =====================================================================
        // 방전 모드 (I_cmd < 0): 기존 PIControlLow 로직
        // =====================================================================
        voltage_cmd = V_min_lim; // 저전압 지령
        upper_limit = 2;         // 상한: 최대 양의 값
        lower_limit = I_cmd_PI;  // 하한: 음의 전류 제한

        // 기존 V_min_ 변수들 업데이트 (호환성 유지)
        V_min_error = error = voltage_cmd - V_out;
        V_min_kP_out = kp_out = Kp * error;

        kI_out_prev = Ki * Tsampl * error;
        ki_accumulator_discharge += kI_out_prev;
        V_min_kI_out = ki_accumulator_discharge;

        // 적분 출력 제한 (Anti-windup)
        if (ki_accumulator_discharge > upper_limit)
            ki_accumulator_discharge = upper_limit;
        else if (ki_accumulator_discharge < lower_limit)
            ki_accumulator_discharge = lower_limit;
        V_min_kI_out = ki_accumulator_discharge;

        // PI 출력 계산 및 제한
        V_min_control_output = pi_out = kp_out + ki_accumulator_discharge;
        if (pi_out > upper_limit)
            V_min_control_output = upper_limit;
        else if (pi_out < lower_limit)
            V_min_control_output = lower_limit;

        // 충전용 적분기는 현재 PI 출력으로 초기화 (bumpless transfer)
        ki_accumulator_charge = V_min_control_output - kp_out;
        if (ki_accumulator_charge > I_cmd_PI)
            ki_accumulator_charge = I_cmd_PI;
        else if (ki_accumulator_charge < -2.0f)
            ki_accumulator_charge = -2.0f;
    }
}

//=============================================================================
// DCL PI Controller Functions (DCL 기반 PI 제어 함수들)
//=============================================================================

/**
 * @brief DCL PI 컨트롤러 초기화 함수
 * 충전/방전 모드별 독립적인 PI 컨트롤러를 초기화하고 파라미터를 설정
 */
void InitDCLControllers(void)
{
    // DCL PI 구조체 기본값으로 초기화
    dcl_pi_charge = (DCL_PI)PI_DEFAULTS;
    dcl_pi_discharge = (DCL_PI)PI_DEFAULTS;

    // 공통 지원 구조체 초기화
    dcl_css_common.T = Tsampl; // 샘플링 시간 50us
    dcl_css_common.err = 0;    // 에러 플래그 초기화
    dcl_css_common.sts = 0;    // 상태 플래그 초기화

    // 공통 지원 구조체 연결
    dcl_pi_charge.css = &dcl_css_common;
    dcl_pi_discharge.css = &dcl_css_common;

    //=========================================================================
    // 충전용 PI 컨트롤러 설정
    //=========================================================================
    dcl_pi_charge.Kp = Kp;          // 비례 게인
    dcl_pi_charge.Ki = Ki * Tsampl; // 적분 게인 (이산화)
    dcl_pi_charge.Umax = I_MAX;     // 초기 상한값 (동적으로 변경됨)
    dcl_pi_charge.Umin = -2.0f;     // 하한값
    dcl_pi_charge.i10 = 0.0f;       // 적분기 상태 초기화
    dcl_pi_charge.i6 = 1.0f;        // Anti-windup 상태 초기화

    //=========================================================================
    // 방전용 PI 컨트롤러 설정
    //=========================================================================
    dcl_pi_discharge.Kp = Kp;          // 비례 게인
    dcl_pi_discharge.Ki = Ki * Tsampl; // 적분 게인 (이산화)
    dcl_pi_discharge.Umax = 2.0f;      // 상한값
    dcl_pi_discharge.Umin = -I_MAX;    // 초기 하한값 (동적으로 변경됨)
    dcl_pi_discharge.i10 = 0.0f;       // 적분기 상태 초기화
    dcl_pi_discharge.i6 = 1.0f;        // Anti-windup 상태 초기화
}

/**
 * @brief DCL 기반 통합 PI 제어 함수
 *
 * DCL_runPI_C1 어셈블리 함수를 사용하여 최적화된 PI 제어 수행
 * 충전/방전 모드에 따라 적절한 컨트롤러 선택 및 bumpless transfer 구현
 */
void PIControlDCL(void)
{
    float32_t pi_output;

    if (I_cmd >= 0)
    {
        // =====================================================================
        // 충전 모드 (I_cmd >= 0): 고전압 제어
        // =====================================================================

        // 동적 상한값 업데이트 (전류 제한에 따라)
        dcl_pi_charge.Umax = I_cmd_PI;

        // DCL_runPI_C1 어셈블리 함수 실행 (최고 성능)
        pi_output = DCL_runPI_C1(&dcl_pi_charge, V_max_lim, V_out);

        // 기존 변수 호환성을 위한 할당 (디버깅 및 모니터링용)
        V_max_error = V_max_lim - V_out;               // 에러 계산
        V_max_kP_out = dcl_pi_charge.Kp * V_max_error; // 비례항
        V_max_kI_out = dcl_pi_charge.i10;              // 적분항 상태
        V_max_control_output = pi_output;              // PI 출력

        // 방전용 컨트롤러와의 bumpless transfer를 위한 상태 동기화
        // 방전 모드로 전환될 때 급격한 출력 변화 방지
        dcl_pi_discharge.i10 = pi_output - dcl_pi_discharge.Kp * (V_min_lim - V_out);
        if (dcl_pi_discharge.i10 > 2.0f)
            dcl_pi_discharge.i10 = 2.0f;
        else if (dcl_pi_discharge.i10 < I_cmd_PI)
            dcl_pi_discharge.i10 = I_cmd_PI;
    }
    else
    {
        // =====================================================================
        // 방전 모드 (I_cmd < 0): 저전압 제어
        // =====================================================================

        // 동적 하한값 업데이트 (전류 제한에 따라)
        dcl_pi_discharge.Umin = I_cmd_PI;

        // DCL_runPI_C1 어셈블리 함수 실행 (최고 성능)
        pi_output = DCL_runPI_C1(&dcl_pi_discharge, V_min_lim, V_out);

        // 기존 변수 호환성을 위한 할당 (디버깅 및 모니터링용)
        V_min_error = V_min_lim - V_out;                  // 에러 계산
        V_min_kP_out = dcl_pi_discharge.Kp * V_min_error; // 비례항
        V_min_kI_out = dcl_pi_discharge.i10;              // 적분항 상태
        V_min_control_output = pi_output;                 // PI 출력

        // 충전용 적분기는 현재 PI 출력으로 초기화 (bumpless transfer)
        dcl_pi_charge.i10 = pi_output - dcl_pi_charge.Kp * (V_max_lim - V_out);
        if (dcl_pi_charge.i10 > I_cmd_PI)
            dcl_pi_charge.i10 = I_cmd_PI;
        else if (dcl_pi_charge.i10 < -2.0f)
            dcl_pi_charge.i10 = -2.0f;
    }
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
    Uint16 i;
    /* 송신 데이터 처리 */
    I_fb_total.f = 0;
    // 총 출력 전류 계산 (마스터 + 모든 슬레이브)
    for (i = 0; i <= 9; i++)
    {
        I_fb_total.f += I_fb_array[i].f;
    }
    // Modbus Input Register에 전류값 저장 (32bit float -> 2개 16bit)
    usRegInputBuf[4] = (Uint16)(I_fb_total.u);
    usRegInputBuf[5] = (Uint16)(I_fb_total.u >> 16);

    // Modbus Input Register에 전압값 저장
    usRegInputBuf[10] = (Uint16)(V_fb_avg.u);
    usRegInputBuf[11] = (Uint16)(V_fb_avg.u >> 16);

    /* 수신 데이터 처리 */
    // 시작/정지 명령 처리 (bit 3: SYSTEM_START, bit 4: SYSTEM_STOP)
    if (usRegHoldingBuf[0] & (1U << 3))
        system_state = STATE_RUNNING; // 시작
    else if (usRegHoldingBuf[0] & (1U << 4))
        system_state = STATE_STOP; // 정지

    // 충전/방전 모드 설정 (강제로 64 설정)
    usRegHoldingBuf[2] = 64; // 배터리 충전/방전 CC 모드 (64) 강제 설정
    eChargeMode = (eCharge_DisCharge_Mode)usRegHoldingBuf[2];

    // 운전 모드에 따른 설정값 처리
    switch (eChargeMode)
    {
    case ElectronicLoad_CV_Mode: // 전자부하 CV 모드          (2)
    case ElectronicLoad_CC_Mode: // 전자부하 CC 모드          (4)
    case PowerSupply_CV_Mode:    // 전원공급 CV 모드         (16)
    case PowerSupply_CC_Mode:    // 전원공급 CC 모드         (32)
    case As_a_Battery_CV_Mode:   // 배터리 시뮬레이션 CV 모드 (128)
        UI_V_cmd = usRegHoldingBuf[5];
        UI_I_cmd.u = (((Uint32)usRegHoldingBuf[8] << 16) | (Uint32)usRegHoldingBuf[7]);
        break;

    case ElectronicLoad_CR_Mode: // 전자부하 CR 모드          (8)
        UI_V_cmd = usRegHoldingBuf[5];
        UI_I_cmd.u = (((Uint32)usRegHoldingBuf[8] << 16) | (Uint32)usRegHoldingBuf[7]);
        load_resistance.u = (((Uint32)usRegHoldingBuf[10] << 16) | (Uint32)usRegHoldingBuf[9]);
        break;

    case Battery_Charg_Discharg_CC_Mode: // 배터리 충전/방전 CC 모드  (64)
        V_max_lim = usRegHoldingBuf[5];
        V_min_lim = usRegHoldingBuf[12];
        UI_I_cmd.u = (((Uint32)usRegHoldingBuf[8] << 16) | (Uint32)usRegHoldingBuf[7]);
        break;

    default:
        break;
    }

    // 모듈 개수로 나눈 전류 지령 계산 및 제한 (-80A ~ +80A)
    I_cmd = UI_I_cmd.f / MODULE_NUM;
    if (I_cmd > I_MAX)
        I_cmd = I_MAX;
    else if (I_cmd < -I_MAX)
        I_cmd = -I_MAX;
}

/**
 * @brief 전류 평균 계산
 *
 * @details SPI ADC로부터 읽은 전류값의 평균 계산 및 스케일링
 *
 * @section adc_conversion ADC 변환 사양
 * - **ADC 범위**: 0~4095 (12bit)
 * - **전압 범위**: 0~3V
 * - **전류 매핑**: 0A=1.5V(2048), +100A=3V(4095), -100A=0V(0)
 * - **변환 공식**: (ADC - 2048) × (100/2048) = 실제 전류(A)
 *
 * @section averaging_process 평균 계산 과정
 * 1. **5회 평균**: SPI에서 읽은 5개 ADC 값의 평균 (100kHz → 20kHz)
 * 2. **스케일링**: ADC 값을 실제 전류값으로 변환
 * 3. **10000회 누적**: 모니터링용 장기 평균 계산
 * 4. **최종 평균**: 10000회 완료 시 I_fb_avg에 저장
 *
 * @note 이 함수는 epwm3_isr의 Case 4에서 20kHz로 호출됨
 */
void Calc_I_fb_avg(void)
{
    // ADC 값을 실제 전류값으로 변환하여 직접 누적
    // ADC 3V -> 4095, 0A : 1.5V, +100A : 3V, -100A : 0V
    I_fb_sum += (I_out_ADC_avg - 2048) * 0.048828125f; // 100 / 2048

    // 10000번 평균 계산 완료 시 최종 평균값 저장
    if (I_cal_cnt++ == 10000)
    {
        I_fb_avg = I_fb_sum * 0.0001; // 10000번 평균
        I_fb_sum = 0;
        I_cal_cnt = 0;
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
    fan_pwm_duty = 0.8 * (temp_in - 36) / (52.4f - 36) + 0.2;

    if (fan_pwm_duty < 0.15)
        fan_pwm_duty = 0.15;
    else if (fan_pwm_duty > 0.9)
        fan_pwm_duty = 0.9;

    EPwm1Regs.CMPA.half.CMPA = (1. - fan_pwm_duty) * PWM_PERIOD_10k;
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
__interrupt void ecan0_isr(void)
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

#endif // _MAIN_C_

//===========================================================================
// No more.
//===========================================================================
