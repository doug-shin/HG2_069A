//###########################################################################
// 2025.02.26 final 정상 동작 확인

/*
 * HMI와 마스터 보드 RS232 Modbus RTU로 통신한다. 38400bps, 8bit, 1bit, none, HMI에서 100ms polling
 * 마스터 보드에서 계산한 PI 출력 값을 DAC값으로 환산하여 485 통신으로 전송한다(전류지령 전송). 통신속도는 5.625Mbps이고, 0.1ms 마다 한번에 9byte를 전송한다.
 * 마스터 보드와 슬레이브 보드는 프로세서가 같아서 baud 에러율이 없다.
 * 마스터 보드는 DIP 스위치와 관계없이 CAN ID가 0xF0로 고정되어 있다.
 * 슬레이브 보드는 DIP 스위치를 1~9까지 설정하여 ID를 정한다. 정해진 설정 ID에 0xF0를 더하여 슬레이브 CAN ID가 설정된다. 0xF1~0xF9
 * CAN과 485 통신 종단 저항은 맨 처음과 맨 끝에만 달아야 한다.
 * 485의 종단 저항 양단에 전압은 200mV이상이 걸려야 한다.
 * 마스터 보드는 ON/OFF 신호를 슬레이브 보드에 CAN 2.0A 방식으로 전송한다. 통신속도 1Mbps, 1ms polling, 1 frame 당 CAN BUS 로드율은 약 8~9% 이다.
 * 슬레이브 보드들은 CAN 우선 순위대로 마스터로 센싱 전류(출력 전류)를 전송한다. 각 슬레이브 보드가 1ms마다 마스터로 데이터를 전송한다. 마스터에서는 10ms동안에 각각의 모든 슬레이브의 전류 데이터를 수신한다.
 * 슬레이브 보드  RS-232(SCI-B)   슬레이브 보드의 내부 변수 모니터링 Termial(TeraTerm)   100ms       230400bps
 *
 */

// 20khz(50us)로 PI동작, RS485 Tx, CAN Tx는 main에서, SPI는 100khz 마다
// OFF 상태일 때 DAC 값 약 2000 확인
// -> 3으로 설정해야 LSPCLK 90Mhz/(3+1) = 22.5Mhz가 되어서 맞는 듯. 소장님께 확인 더 느려도 되는지? 아니면 22.5Mhz가 맞는지?
// -> 7로 설정하면  LSPCLK 90Mhz/(7+1) = 11.25Mhz가 됨. 언제 부터 바뀌었는지 확인 필요.

// GPIO5,6,7,8  DIP 스위치로 CAN ID를 설정하여 확인한다.
// master에서는 시작/정지 명령과 전류 지령을 slave로 전달한다.
// master에서는 PI만 하고 그 결과값을 slave1,2로 전달하여 slave1,2에서 dac를 출력한다. 2개 슬레이브 병렬이므로 160A 일때 slave1과 slave2는 각각 4V를 출력한다. -160A는 각각 0V를 출력한다.
// slave는 can으로 응답
// slave는 232를 사용하지 않으므로 터미널을 살린다. CAN ID 표시, master에서 수신받은 지령 표시


// CAN 2.0B Bus Load Rate -> 2.0A로 하면 100us 소요될듯
/*
1000kbps로 1 bit를 전송하는데 1us가 걸린다.
130bit(CAN 1 Frame)을 전송하는데 걸리는 시간은 130us이다.
1ms 마다 1 Frame씩 매번 전송한다고 가정하면
1ms 시간동안 bus는 130us동안만 차지될 것이고

130/1000 //  1ms 마다 버스 로드는 13%로 계산된다. 1khz
130/200  // 0.2ms 마다 버스 로드는 65%로 계산된다.

실험결과 25k마다 보냈는데 수신 프로그램에서는 0.15ms 마다 수신됐음.

130/100  // 0.1ms 마다는 안 됨. 100%를 넘어 감. 10khz
 */

//100 1khz = 13%    1ms
//50 2khz = 25%
//10 10khz = 60%    0.1ms -> 수신 프로그램에서는 0.22ms로 수신됨
//9 11.11khz = 66%
//8 12.5khz = 73%
//7 14.2857khz = 82%
//6 16.6khz = 95%
//5 20khz = 73%     0.2ms
//4 25khz = 88%     0.4ms
//3 33.3khz = 82%
//2 50khz = 88%
//1 100khz = 96%

//
/*
 * RXERR INT ENA(SCICTL1의 비트 6) 비트를 활성화하여 수신 오류가 발생할 때 인터럽트를 생성할 수 있습니다.
 * 이 비트를 활성화하면 중단이나 수신 오류가 발생할 때 인터럽트(PIE 및 SCI를 올바르게 구성했다고 가정)가 생성됩니다.
 * SCI RX INT ISR에서 SCIRXST 레지스터를 확인하여 어떤 오류(중단 감지 오류, 프레이밍 오류, 오버런 오류 및 패리티 오류)가 발생했는지 알 수 있습니다.
 * SCICTL1 레지스터의 SW RESET 비트를 사용하여 오류 플래그를 삭제할 수 있습니다.
 * 중단 감지 오류 발생 시 SCI는 데이터 수신을 중지합니다. 데이터 수신을 시작하려면 SW RESET 비트를 토글하여 SCI를 재설정해야 합니다.
 *  SCI RX ISR에서 SCIRXST 레지스터를 확인하여 어떤 오류가 발생했는지 확인할 수 있습니다.
 */

// LED11, LED12, LED13 (PIN 31, 32, 33)
// ADC0 - 온도
// ADC1 - 100A 3V, 0A 1.5V, -100A 0V (100A 5mA 5V -> 100A 5mA 3V, -100A -5mA -5V -> -100A -5mA -3V)
// 24LC128
// EEROM-SDA GPIO32
// EEROM-SCL GPIO33
// EEROM-WP  GPIO14
// CAN , https://m.blog.naver.com/cyjrntwkd/70120007780
// 232
// I2C
// SPI (ADC chip)


/* 해야 할일
FAN 온도에 따른 PWM Duty 가변 안 됨. 0.15 fix
슬레이브 보드 CAN ID 중복 검출하여 마스터로 전송
마스터에서 슬레이브 CAN 수신 안 되면 통신 폴트 표시
슬레이브에서 마스터의 485수신을 못 받으면 CAN으로 폴트 전송
리셋 후 초기 FAN 안 돌아가는 원인 검토
터미널에 전류값 소수점 표시
통신 폴트 발생시 처리 및 경고
if(SciaRegs.SCIRXST.bit.PE) // error 처리 할 것
에러 발생 시 카운트해서 저장해 볼 것
flash 에서 실행과 ram에서 실행시간 관찰

Run 신호를 슬레이브로 전송시 0,1이 아닌 특정값으로 Run = 0xA0, !Run = x00;, 슬레이브에서도 프로그램 코드 수정할 것

*/

//###########################################################################
#define _MAIN_C_

#define I_MAX (80)
#define MODULE_NUM (1)   // 모듈 개수

#define MON_MAXCNT        (10000.)                        // Monitoring Count for Summing data.
#define MON_MAXCNT_REV    ((float)((1)/(MON_MAXCNT)))  // 평균 구할 때 나눗샘을 곱샘으로 하기 위해서

#ifndef _CAN_BUS_
#define _CAN_BUS_      ( 1 )
#endif

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2806x_Cla_defines.h"
#include "string.h"

#include "sicDCDC35kw.h"


#include "sicDCDC35kw_setting.h" //@prabhu changed sicDCDC35kw_setting 20210811

#include "modbus.h"
#include "math.h"
#include "protocol.h"
#include "Terminal.h"


void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);

void spi_init(void);
void eCana_config (void);

void SaveCommand(Uint16 mbox_num);
void ProcessCommand(Uint16 mbox_num);
void SendParameters(void);  // 추가된 함수 선언

void stra_xmit(UCHAR  *buff, Uint16 Length);


#if _CAN_BUS_
#include "CANpie.h"
#include "CANpieTest.h"
//extern struct ECAN_REGS ECanaShadow;
unsigned long ulCanTRS = 0;  // Transmission Request Set
unsigned long ulCanTRS_Prev = 0;
#endif

#pragma CODE_SECTION(scia_txFifo_isr, "ramfuncs");
//#pragma CODE_SECTION(scia_rxFifo_isr, "ramfuncs");
#pragma CODE_SECTION(cpuTimer1ExpiredISR, "ramfuncs");
#pragma CODE_SECTION(scibRxReadyISR, "ramfuncs");
#pragma CODE_SECTION(scibTxEmptyISR, "ramfuncs");
#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs");   //copy to FLash to ram
#pragma CODE_SECTION(cpu_timer2_isr, "ramfuncs");   //copy to FLash to ram

//#pragma CODE_SECTION(USART_A_FUNC, "ramfuncs");
#pragma CODE_SECTION(modbus_parse, "ramfuncs");


#pragma CODE_SECTION(stra_xmit, "ramfuncs");
#pragma CODE_SECTION(epwm3_isr, "ramfuncs");
#pragma CODE_SECTION(ecan0_isr, "ramfuncs");

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void epwm1_isr(void);
//__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void spi_isr(void);
//__interrupt void scia_rxFifo_isr(void);
__interrupt void scia_txFifo_isr(void);
__interrupt void ecan0_isr(void);


#pragma CODE_SECTION(adc_isr, "ramfuncs");   //copy to Flash to Ram
__interrupt void adc_isr(void);

extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadEnd;
extern Uint16 Cla1funcsRunStart;
extern Uint16 Cla1funcsLoadSize;

extern __interrupt void scibRxReadyISR( void );
extern __interrupt void scibTxEmptyISR( void );
extern __interrupt void cpuTimer1ExpiredISR( void );

extern USHORT usRegInputBuf[REG_INPUT_NREGS];
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

Uint16 type_size = 0;
Uint16 cpu_timer0_cnt = 0, cpu_timer1_cnt = 0, cpu_timer2_cnt = 0;



//PI
Uint16 rx_adc_data;
int32 rx_adc_data_buf;
int32 rx_adc_data_buf_sum = 0;
float32 rx_adc_data_buf_ave = 0.0f;

float v_ref = 5.;
Uint16 test_dac = 2000;
Uint16 test_en, test_en2 =0;
Uint16 hw_switch = 0;
Uint16 Buck_EN = 0;

Uint16 board2_voltage;
Uint16 board2_current;
Uint16 tx_count = 0, count_num = 100;
Uint8 test_data[8] = {0};



#define STOP (0)
#define START (1)

#define OVER_VOLTAGE (1100)


//기준 전압 전체 범위 출력 전압은 기준 전압의 두 배입니다.
//SpiaRegs.SPITXBUF =
#define FAST_REF1 0xD001 //내부 1.024V
#define SLOW_REF1 0x9001
#define FAST_REF2 0xD002 //내부 2.048V
#define SLOW_REF2 0x9002
#define FAST_REF_E 0xD000 //외부
#define SLOW_REF_E 0x9000

Uint16 fifo_err_cnt = 0;

Uint16 LoopCount = 0;
Uint16 ErrorCount= 0;

Uint16 _1second_flag = 0, force_reset_test = 0;


// 전송 제어 문자
#define STX (0x02) // Start of Text, 본문의 개시 및 정보 메세지 헤더의 종료를 표시
#define ETX (0x03) // End of Text, 본문의 종료를 표시한다
#define DLE (0x10) // Data link escape, 뒤따르는 연속된 글자들의 의미를 바꾸기 위해 사용, 주로 보조적 전송제어기능을 제공


Uint16 gSciTxBuf[4] = {STX,0,0,ETX};


int16 slop = 0;
Uint16 can_tx_flag = 0;

Uint16 can_rx_fault_cnt[11], detect_module_num = 1;

Uint16 size_check = 0;

// SD2 //GPIO3   SPISOMIA(기존과 동일, 네트 이름만 변경)
// SCK2 //GPIO18 SPICLKA (기존과 동일,  네트 이름만 변경) DAC1-CLK
// CS_100K // GPIO44 (chip select) 기존 그대로 사용

// Gpio_select() 수정 부분

// ADC-CNV (GPIO7)은 삭제
float adc_ramp = 1.;
Uint16 adc_offset = 2500;
Uint32 test_val = 0;
Uint16 count_num2 = 100000; //100000 = 1sec. 10000 = 100ms
Uint16 send_start_flag = 1;
Uint16 test_out_A = 0;
Uint16 Run_on_count = 0;
Uint16 switch_1_old_old = 0, switch_1_old = 0, switch_1 = 0, filtered_switch_input = 0;
Uint16 _10ms_timer = 0;
float I_com_1;  // Declare before usage



unsigned int MonitoringCount = 0;
float Vo_delta = 0.;
float Vo_sen_sum_mon = 0.;
float Vo_Monitor = 0.;
float Vo_Mean = 0.;

Uint16 hw_fault = 0, GPIO_in = 0, GPIO_in_1 = 0;


void main(void)
 {

    Uint16 i;
    UI_Iout_command.fValue = 0; // Iout_command.fValue = 80;


// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks (PLL 18/2*10M (INOSC1)
// This example function is found in the F2806x_SysCtrl.c file.
    InitSysCtrl(); // 90Mhz/1  sci_clk/16 = 5.625Mbps(Max),  90Mhz/4 = 22.5Mhz, sci_clk/16 = 1.40625Mbps(Max)
//Control Suite 새로 설치시에는 SysCtrlRegs.LOSPCP.all = 0x0000; 으로 변경해줘야 한다. 20247.10.10

    ///////////////////////////////////////////////////////////////////////////////////////////
    // for FLASH operation
    // 1. Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions: cpu_timer0_isr(), adc_isr()
    // and InitFlash() - (default);
    // 2. The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the F28069.cmd file.
       memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

    // 3. Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM (default configuration)
    //  InitFlash();
    ///////////////////////////////////////////////////////////////////////////////////////////

//   DELAY_US(100000); // 1 second, (SMPS Setup Rise Time margin)

   static eMBErrorCode eStatus;

// Step 2. Initalize GPIO: 
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
    Gpio_select();

//    led2ON(); //GpioDataRegs.GPBSET.bit.GPIO57  = 1

    DigitalIn.all = 0;

    Digital_Input(); // board의 id를 읽어서 초기 can id를 설정하기 위함

// For this case just init GPIO pins for ePWM1 ~ ePWM8
// These functions are in the F2806x_EPwm.c file
    InitEPwm1Gpio(); //dont use for a 3-phase interleaved version
    InitEPwm3Gpio();

// Setup only the GP I/O only for SPI-A functionality
    InitSpiaGpio();


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
    DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the F2806x_PieCtrl.c file.
    InitPieCtrl();

//    size_check = sizeof(eChargeMode);

    //
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    //
    InitFlash();

//    DINT;
    // Disable interrupts again (for now)
    // Note that InitPieCtrl() enables interrupts
// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

   InitSciaGpio(); // RS-232, PCB ref. CN4 , for Parameter Monitoring
   scia_fifo_init();      // Initialize the SCI FIFO
   scia_init();
   InitProtocol();

   EALLOW;   // This is needed to write to EALLOW protected registers
   En485_ON( );   // GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
   EDIS;     // This is needed to disable write to EALLOW protected registers

   Tx485_ON( ); // for master
//   Rx485_ON( ); // for slave

//   msg = "Hello World\0";
//   scia_msg(msg);
////   msg = "\r\nDSP USART(SCI-B) port init...  \n\0";
//   scib_msg(msg);
//
   //   msg = "\r\n\0";
//   scib_msg(msg);


// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
    InitPieVectTable();

    InitECanaGpio();         // GPIO를 CAN 통신 용으로 설정
    InitECana();          // CAN 초기화 : CAN2.0A 모드, 1Mbps 속도
    eCana_config();

    InitAdc();  // For this example, init the ADC
    AdcOffsetSelfCal();
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 1; // Select external ref. after self-calib. proc.
    EDIS;

    AdcSetup();
    spi_init();        // init SPI

    InitCpuTimers();   //  initialize the Cpu Timers ( and stop timer)

    ConfigCpuTimer(&CpuTimer2, 90, 50); //20kHz interrupt

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // stop timer
    EDIS;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable vector fetching from PIE block

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_isr;     // dont use
    PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.ADCINT1 = &adc_isr; // to update ADC results, at the last ADC conversion in one PWM period

    PieVectTable.SPIRXINTA = &spi_isr;

    PieVectTable.SCIRXINTB = &scibRxReadyISR; // modbus
    PieVectTable.SCITXINTB = &scibTxEmptyISR; // modbus
    PieVectTable.TINT1 = &cpuTimer1ExpiredISR; // modbus

    PieVectTable.TINT0 = &cpu_timer0_isr; // to calculate controllers
    PieVectTable.TINT2 = &cpu_timer2_isr; //

    PieVectTable.SPIRXINTA = &spi_isr;

    PieVectTable.ECAN0INTA = &ecan0_isr; // CAN-A 인터럽트 핸들러 등록

    EDIS;
    // This is needed to disable write to EALLOW protected registers

    InitEPwm1Example();    //Initialize ePWM1
    InitEPwm3Example();    //Initialize ePWM3

    /* Configure PIE interrupts */
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable vector fetching from PIE block
    PieCtrlRegs.PIEACK.bit.ACK9 = 1; // Enables PIE to drive a pulse into the CPU

    IER |= M_INT1; // Enable CPU Interrupt 1 (timer 0, INT0-INT1.7) and (ADCINT1, INT1.1)
    IER |= M_INT3;            // Enable CPU Interrupt 3 (ePWM1~ePWM8) - reserved
    IER |= M_INT6;                      // Enable INT6 of CPU for SPIRXINTA
    IER |= M_INT9; // Enable INT9 of CPU for CAN, Enable CPU INT9 which is connected to SCI-A/B:

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
// PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
 PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // <<-- ePWM3, kcs

// Enable TINT0 in the PIE: Group 1 interrupt 7
//    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
// Enable ADCINT1 in PIE
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

// Enable SPIRXINTA in the PIE: Group 6 interrupt 1
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;

    // SCI-A/B related interrupt
    // Enable SCIRxINTA & SCITxINTA in the PIE: Group 9 interrupt 1 & 2 respectively
//    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;     // PIE Group 9, INT1 : SCIRxINTA
//    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;     // PIE Group 9, INT2 : SCITxINTA

    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;     // PIE Group 9, INT1 : SCIRxINTB
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;     // PIE Group 9, INT2 : SCITxINTB

    IER |= M_INT13;                     // Timer 1
//  IER |= M_INT14;                     // Timer 2

    // CAN 인터럽트 활성화
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;     // PIE Group 9, INT5 : ECAN0INTA
    PieCtrlRegs.PIEIER9.bit.INTx6 = 0;     // PIE Group 9, INT6 : ECAN1INTA

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//DAC SETTING
    DAC1_DS();
    DAC1_CS(); // DAC1_CS Activate w Falling Edge
    SpiaRegs.SPITXBUF = 0xD002; // Write DAC Control Register for 2.048v internal ref.
    while (!SpiaRegs.SPISTS.bit.INT_FLAG);
    deg_sDacTmp = SpiaRegs.SPIRXBUF;



    /* Initialize the input register values before starting the Modbus stack. */
    for (i = 0; i < REG_INPUT_NREGS; i++) usRegInputBuf[i] = 0;

    /* Initialize the holding register values before starting the Modbus stack. */
    for (i = 0; i < REG_HOLDING_NREGS; i++) usRegHoldingBuf[i] = 0;

    // Initialize protocol stack in RTU mode for a slave with address 01 = 0x01
    eStatus = eMBInit(MB_RTU, 0x01, 0, 38400, MB_PAR_NONE);

    if (eStatus != MB_ENOERR)
    {
        EALLOW;
        // This is needed to write to EALLOW protected registers
        SysCtrlRegs.WDCR = 0x0010;
        EDIS;
        // This is needed to disable write to EALLOW protected registers
    }

    // Enable the Modbus Protocol Stack.
    eStatus = eMBEnable();
    if (eStatus != MB_ENOERR)
    {
        EALLOW;
        // This is needed to write to EALLOW protected registers
        SysCtrlRegs.WDCR = 0x0010;
        EDIS;
        // This is needed to disable write to EALLOW protected registers
    }


//    type_size = sizeof(UCHAR);
//    Vin_monitor.fValue = 0;
//    Vout_monitor.fValue = 0;
//    Iout_monitor.fValue = 0;

    // Reset the watchdog counter
    ServiceDog();
//    // Enable the watchdog
    EALLOW;
    SysCtrlRegs.WDCR = 0x0028;
    EDIS;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

//    Tx485_ON();

    IcomTemp = 2000;

    for (;;)
    {
        mainLoopCount++; //free run counter, for debugging
        if(ScibRegs.SCIFFRX.bit.RXFFOVF || force_reset_test) // 리셋 시 usRegHoldingBuf는 초기화 되지 않아야한다.
        {
            force_reset_test = 0;
            fifo_err_cnt++;
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
            ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
        }

        (void) eMBPoll(); // 2024.06.04 반드시 메인 무한 루프에서 호출할 것.

        if(can_tx_flag == 1)
        {
            can_tx_flag = 0;
            if(Run == 1) ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0xA0; // 2025.02.17
            else         ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0;

            ECanaRegs.CANTRS.all = 0x00000001;   // Transmit Request Set // 약 1초에 6000 프레임 전송)

            if (ECanaRegs.CANRMP.bit.RMP1 != 0) {Io_sense1.ulValue  = ECanaMboxes.MBOX1.MDL.all; ECanaRegs.CANRMP.bit.RMP1 = 1; can_rx_fault_cnt[1] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[1]++ >= 10000) {Io_sense1.ulValue = 0; can_rx_fault_cnt[1] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP2 != 0) {Io_sense2.ulValue  = ECanaMboxes.MBOX2.MDL.all; ECanaRegs.CANRMP.bit.RMP2 = 1; can_rx_fault_cnt[2] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[2]++ >= 10000) {Io_sense2.ulValue = 0; can_rx_fault_cnt[2] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP3 != 0) {Io_sense3.ulValue  = ECanaMboxes.MBOX3.MDL.all; ECanaRegs.CANRMP.bit.RMP3 = 1; can_rx_fault_cnt[3] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[3]++ >= 10000) {Io_sense3.ulValue = 0; can_rx_fault_cnt[3] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP4 != 0) {Io_sense4.ulValue  = ECanaMboxes.MBOX4.MDL.all; ECanaRegs.CANRMP.bit.RMP4 = 1; can_rx_fault_cnt[4] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[4]++ >= 10000) {Io_sense4.ulValue = 0; can_rx_fault_cnt[4] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP5 != 0) {Io_sense5.ulValue  = ECanaMboxes.MBOX5.MDL.all; ECanaRegs.CANRMP.bit.RMP5 = 1; can_rx_fault_cnt[5] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[5]++ >= 10000) {Io_sense5.ulValue = 0; can_rx_fault_cnt[5] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP6 != 0) {Io_sense6.ulValue  = ECanaMboxes.MBOX6.MDL.all; ECanaRegs.CANRMP.bit.RMP6 = 1; can_rx_fault_cnt[6] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[6]++ >= 10000) {Io_sense6.ulValue = 0; can_rx_fault_cnt[6] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP7 != 0) {Io_sense7.ulValue  = ECanaMboxes.MBOX7.MDL.all; ECanaRegs.CANRMP.bit.RMP7 = 1; can_rx_fault_cnt[7] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[7]++ >= 10000) {Io_sense7.ulValue = 0; can_rx_fault_cnt[7] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP8 != 0) {Io_sense8.ulValue  = ECanaMboxes.MBOX8.MDL.all; ECanaRegs.CANRMP.bit.RMP8 = 1; can_rx_fault_cnt[8] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[8]++ >= 10000) {Io_sense8.ulValue = 0; can_rx_fault_cnt[8] = 10000;}}
            if (ECanaRegs.CANRMP.bit.RMP9 != 0) {Io_sense9.ulValue  = ECanaMboxes.MBOX9.MDL.all; ECanaRegs.CANRMP.bit.RMP9 = 1; can_rx_fault_cnt[9] = 0;}
            else                                {                                                                               if(can_rx_fault_cnt[9]++ >= 10000) {Io_sense9.ulValue = 0; can_rx_fault_cnt[9] = 10000;}}

            for(i = 1; i < 10; i++)
            {
                if(can_rx_fault_cnt[i] < 9999) if(detect_module_num++ >  10) detect_module_num = 10;
                else                           if(detect_module_num-- <   1) detect_module_num = 1;
            }
        }

        // slave에서 중복된 can id를 설정하여 마스터나 슬레이브의 can 레지스터에 어떤 에러가 발생하는지 관찰한다.
        // 또는 peak can을 연결하여 busoff 등 에러를 관찰하여 처리 방법을 찾는다.

        if(_50us_flag )
        {
            _50us_flag = 0;
            if(_0_1ms_count++ >= 200){ // 10ms loop, 20khz/100hz = 200
                _0_1ms_count = 0;
                modbus_parse(); // RS-232 modbus parse
            }
        }

        // Watch dog service
        EALLOW;
        SysCtrlRegs.WDKEY = 0x0055; // Feed Dog with one Key, the other in Timer 0 ISR
                                    // Dog runs with the highest rate of OSC/512/2^8 = 686 Hz
        //SysCtrlRegs.WDKEY = 0x00AA;
        EDIS;

    } //end (for loop)

} //end main


void stra_xmit(UCHAR  *buff, Uint16 Length)
{
    Uint16  i;
//    DELAY_US ( 10 );
//    Tx485_ON( );
    for(i=0 ; i<Length ; i++)
    {
        SciaRegs.SCITXBUF = buff[i];
        while(SciaRegs.SCICTL2.bit.TXRDY == 0);
    }
//    DELAY_US ( 2 );
//    Rx485_ON( ); // for slave
}


__interrupt void cpu_timer0_isr(void) //100khz
{
    cpu_timer0_cnt++;

    CpuTimer0Regs.TCR.bit.TIF = 1;   // Clear Interrupt flag

// Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



__interrupt void cpu_timer2_isr(void) //1 Second
{
//    _1second_flag = 1;
//    led3ON();
    cpu_timer2_cnt++;
//    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;      /* SCI 송신 FIFO 인터럽트 Enable */
//    led3OFF();
//    led3TOGGLE();
}


Uint16 spi_interrupt_cnt;
__interrupt void spi_isr(void)
{
    spi_interrupt_cnt++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
}

Uint16 epwm1_isr_cnt, led2_on;
__interrupt void epwm1_isr(void)
{
//    epwm1_isr_cnt++;
    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

Uint16 epwm3_isr_cnt, check_counter;
__interrupt void epwm3_isr(void) // 100KHz
{
    Uint16 i; epwm3_isr_cnt++;
//    led2ON();
//    led3ON();
    if(tx_count++ >= count_num) // 1khz(1ms) 마다
    {
        tx_count = 0;
        can_tx_flag = 1; // main loop 에서 처리
    }

    DAC1_DS(); // DAC1_CS Deactivate
    rx_adc_data = SpiaRegs.SPIRXBUF; // Read ADC data

    ADC1_CS(); // ADC1_CS Activate w Falling Edge 컨버전 시작 후 최소 700ns 후에 읽어야 함.
    DAC1_CS(); // DAC1_CS Activate w Falling Edge

    rx_adc_data_buf_sum += rx_adc_data;
    if(rx_adc_data_buf_sum < 1) rx_adc_data_buf_sum = 1;

    Io_ad_sum += Io_ad;

/********************************
// 20khz loop
 *
 * 20khz 에 PI_L를 하고, 다시 20khz 후에 PI_H를 수행한다.
 * 그래서 PI_L과 PI_H의 연산(수행)시간을 각각 체크해본다. 두개가 합쳐서 100Khz 이내에 가능한지.
 * 3.5uS + 1.666uS = 5.1666us
*********************************/
    switch(_100khz_count)
    {
    case 0: //20KHz
        rx_adc_data_buf_ave = (float32)(rx_adc_data_buf_sum * 0.2f);
        rx_adc_data_buf_sum = 0;
        fADC_voltage = v_ref * (float) rx_adc_data_buf_ave * (float)(0.0000152) - 0.1945;// (float)((1ul << 16))
        Vo_sen = fADC_voltage * 250 * 1.04047;// 1.04047 = 1/0.9611;
        Vo = Vo_sen; // Vo 변수는 제어기에 사용

        PI_Controller_high();  // PI 제어기,  battery charger/discharger mode

//        for test code, 구형파
//        if  (IcomTemp >= 2000) slop = -100;
//        if  (IcomTemp <= 1000) slop = +100;
//        IcomTemp = IcomTemp + slop;

//        for test code, 삼각파
//        if     (slop <  90) IcomTemp = 0;
//        else if(slop < 100) IcomTemp = 4000;
//        else                slop = 0;
//        slop++;

//        IcomTemp = (((I_com_set - 0.5) * 0.5 * 50)* 0.5 + 2000) * 1.0005; // 방전 모드(Icom_Pos 80A) DA 출력 4.000V. 회생 모드(Icom_Neg -80A) DA 출력 0.000V
//        SpiaRegs.SPITXBUF = 0xC000 | (test_out_A & 0xfff); // test code
//        SciaRegs.SCIFFTX.bit.TXFFIENA = 1;      // SCI 송신 FIFO 인터럽트 Enable
         _100khz_count++;
        break;

    case 1:
        PI_Controller_low();  // PI 제어기,  battery charger/discharger mode
        _100khz_count++;
        break;

    case 2: // 온도, 20khz 마다
        //               if(GpioDataRegs.GPBDAT.bit.GPIO39) // DAB_ok 일때만 buck_on 되어야 한다.
        if      (I_com >  I_ss) I_com_1 = I_ss;
        else if (I_com < -I_ss) I_com_1 = -I_ss;
        else                    I_com_1 = I_com;

        if      (I_com_1 > Voh_err_PI_out) I_com_set = Voh_err_PI_out;
        else if (I_com_1 < Vol_err_PI_out) I_com_set = Vol_err_PI_out;
        else                               I_com_set = I_com_1;

        I_ss += I_MAX * 0.00005f;
        if (I_ss > I_MAX ) I_ss = I_MAX;



//            if (!Run) {
//
//                I_ss = 0;
//                Run_on_count = 0;    // Reset Run-on counter
//                GpioDataRegs.GPADAT.bit.GPIO19 = 0; // FET ON immediately
//
//            }
//
//            else {  // When Run == 1
//                Run_on_count++; // Increase Run-on counter
//
//                if (Run_on_count >= 20000) { // After 1 sec
//                    Run_on_count = 20000;  // Limit max value
//                    GpioDataRegs.GPADAT.bit.GPIO19 = 1; // FET OFF after delay
//
//                }
//            }


            if (!Run) I_ss = 0;

            if (Run)
            {
                if(Run_on_count++>= 20000) // Discharge FET OFF After 1sec From Run ON
                {
                    Run_on_count = 20000;
                    GpioDataRegs.GPADAT.bit.GPIO19 = 1; //GPIO19 = 1 : Discharge FET OFF
                }
            }
            else
            {
                Run_on_count = 0;
                GpioDataRegs.GPADAT.bit.GPIO19 = 0; //GPIO19 = 0 : Discharge FET ON, NO Delay
            }

//     temp = (((I_com_set - 0.5f) * 0.5f * 50)* 0.5f + 2000) * 1.0005f;  // 20khz
       IcomTemp = (I_com_set * 50 * 0.5f + 2000) * 1.0005f;  // 20khz

       if (IcomTemp > 4095) IcomTemp = 4095;    // DA 출력 상한 제한
       spi_tx_temp = IcomTemp;     //20khz 마다 업데이트

       gSciTxBuf[1] = (UCHAR)(IcomTemp & 0x00FF);;
       gSciTxBuf[2] = (UCHAR)((IcomTemp >> 8) & 0xFF);

       for (i = 0; i < 4; i++)
       {
           SciaRegs.SCITXBUF = gSciTxBuf[i] & 0x00FF;  // 모든 슬레이브 보드로 전류 지령 전송
       }
        // fan_pwm을 off 하고 DAB_ok는 약간의 지연후에 off된다.
    //softstart 함수로
       if(Vo >= OVER_VOLTAGE) over_voltage_flag = 1; // 평균한 값을 20khz 마다 체크. fault 표시 필요

       if (hw_fault == 0 && over_voltage_flag == 0 && filtered_switch_input == 1) Run = 1;
//       if ( !hw_fault && !over_voltage_flag && filtered_switch_input) Run = 1;
//       if ( !(hw_fault || over_voltage_flag) && filtered_switch_input) Run = 1;

       else Run = 0;

      if(Vo >= OVER_VOLTAGE) over_voltage_flag = 1; // 평균한 값을 20khz 마다 체크. fault 표시 필요
      else if (filtered_switch_input == 0 ) over_voltage_flag = 0;


//       if (dab_ok)
//       {
//           if (filtered_switch_input) // 외부 ON/OFF 스위치
//           {
//               if(over_voltage_flag) Run = 0; // fault
//               else
//               {
//                   Run = 1;
//                }
//
//           } // CN2, Switch
//           else
//           {
//               Run = 0;
//               over_voltage_flag = 0; //fault clear
//           }
//       }
//       else
//       {
//           Run = 0;
//       }


       if(Run)     GpioDataRegs.GPADAT.bit.GPIO17 = 0;
       else        GpioDataRegs.GPADAT.bit.GPIO17 = 1; //Buck_Enable

        _100khz_count++;
        break;

    case 3:
        Temp_ad_sen = (float32) (Temp_ad / 4095 * 3.0);
        Temp_ad_sen_1 = (float32) (Temp_ad_sen * 5 / 3);  // Temp_ad_sen_1 == V
        In_Temp = (float32) ((Temp_ad_sen_1 * 20.4) + 1.4);

        Calculating_voltage_average_and_monitoring_average();
        _100khz_count++;
        break;

    case 4:
        _50us_flag = 1; // 20khz flag on

        if(_10ms_timer++ >= 200)
        {
           _10ms_timer = 0;

           Digital_Input();

        }

        Calculating_current_average_and_monitoring_average();
        FAN_pwm_service();
        _100khz_count = 0;
        break;

    default:
        break;

    }

if ( _100khz_count > 4 ) _100khz_count = 0;

// Practice

         GPIO_in_1 = GPIO_in;
         GPIO_in = !GpioDataRegs.GPBDAT.bit.GPIO39;

         if      (GPIO_in == 1 && GPIO_in_1 == 1) hw_fault = 1;
         else if (filtered_switch_input     == 0) hw_fault = 0;

    //증폭 관련 스케일 저항 부분은 0.1% 급을 사용해야 한다.
    SpiaRegs.SPITXBUF = 0xC000 | (spi_tx_temp & 0xfff); // DAC1 A write, fast 100khz 마다
//    SpiaRegs.SPITXBUF = 0xC000 | (test_dac & 0xfff); // DAC1 A write, fast 100khz 마다

//    Vo_sen = (float32) (Vo_ad - 162) * 1200 / 4095; // adc 3v -> 4095, adc 3v : 1200

    ADC1_DS(); // ADC1_CS Deactivate

    // Clear INT flag for this timer
    EPwm3Regs.ETCLR.bit.INT = 1;
//    led3OFF();
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void adc_isr(void)
{
    EALLOW;
    //SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA; // Feed Dog with one Key, the other in Main
                                // Dog runs with the highest rate of OSC/512/2^8 = 686 Hz
    EDIS;
// ADC conversions
// SOC0 시작
    AdcRegs.ADCSOCFRC1.all = 0x07; // SOC0, SOC1, SOC2

    Temp_ad = AdcResult.ADCRESULT0;
    Io_ad = AdcResult.ADCRESULT1;
    Vo_ad = AdcResult.ADCRESULT2;

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Clear ADCINT1 flag to reinitialize for next SOC

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
    return;
}


void modbus_parse(void)
{
    Io_sen_total.fValue = ((Io_avg + Io_sense1.fValue + Io_sense2.fValue + Io_sense3.fValue + Io_sense4.fValue + Io_sense5.fValue + Io_sense6.fValue + Io_sense7.fValue + Io_sense8.fValue + Io_sense9.fValue));// 출력 전류

    usRegInputBuf[4] = (Uint16) (Io_sen_total.ulValue);
    usRegInputBuf[5] = (Uint16) (Io_sen_total.ulValue >> 16);

//    usRegInputBuf[11] =  (Uint16)Vo_sen.ulValue;// 출력 전압

    usRegInputBuf[10] = (Uint16) (Vo_sen_avg.ulValue);
    usRegInputBuf[11] = (Uint16) (Vo_sen_avg.ulValue >> 16);

   if     (usRegHoldingBuf[0] & 0x0008)  start_stop = START; //
   else if(usRegHoldingBuf[0] & 0x0010)  start_stop = STOP;  //

   usRegHoldingBuf[2] = 64; // force value
   eChargeMode = (eCharge_DisCharge_Mode)usRegHoldingBuf[2];

   switch(eChargeMode){                // 전류는 현재 UI가 -지령을 받을 수 없으므로 최대 160A를 받아서 -80A 해서 사용(임시로)
   case ElectronicLoad_CV_Mode        : Vout_Reference = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]); break; // 2
   case ElectronicLoad_CC_Mode        : Vout_Reference = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]); break; // 4
   case ElectronicLoad_CR_Mode        : Vout_Reference = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]);
                                        uf_resistance.ulValue = ( (Uint32)usRegHoldingBuf[9] + ((Uint32)usRegHoldingBuf[10]<<16) ); break; // 8
   case PowerSupply_CV_Mode           : Vout_Reference = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]); break; // 16
   case PowerSupply_CC_Mode           : Vout_Reference = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]); break; // 32
   case Battery_Charg_Discharg_CC_Mode: V_high_limit   = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]); V_low_limit = usRegHoldingBuf[12];
                                        Voh_com = V_high_limit; Vol_com = V_low_limit; break; // 64
   case As_a_Battery_CV_Mode          : Vout_Reference = usRegHoldingBuf[5]; Iout_Reference = ((int16)usRegHoldingBuf[7]); break; // 128
   default :
       break;
   }

//       uf_resistance = uf_resistance.fValue;
    UI_Iout_command.ulValue = ( (Uint32)usRegHoldingBuf[7] + ((Uint32)usRegHoldingBuf[8]<<16) );

    Icom_temp = UI_Iout_command.fValue / MODULE_NUM;
//    Icom_temp = Iref_temp;

    if     (Icom_temp >  I_MAX) Icom_temp =  I_MAX; // detect_module_num * 80
    else if(Icom_temp < -I_MAX) Icom_temp = -I_MAX;

    I_com = Icom_temp;
}


void PI_Controller_high(void)
{
    Voh_Err = Voh_com - Vo;
    Voh_KP_out  = Kp * Voh_Err;             //P term

    KI_out_old = Ki * Tsampl * Voh_Err;     //I term  Tsampl=50E-6 (50us)
    Voh_KI_out = Voh_KI_out + KI_out_old;

    if      (Voh_KI_out >  I_com_1) Voh_KI_out = I_com_1; // +
    else if (Voh_KI_out <       -2) Voh_KI_out = -2;

    Voh_err_PI_out = Voh_KP_out + Voh_KI_out;

    if      (Voh_err_PI_out >  I_com_1) Voh_err_PI_out = I_com_1; // 80 * detect_module_num
    else if (Voh_err_PI_out <       -2) Voh_err_PI_out =  -2;
}

void PI_Controller_low(void)
{
    Vol_Err = Vol_com - Vo;
    Vol_KP_out = Kp * Vol_Err;              //P term

    KI_out_old = Ki * Tsampl * Vol_Err;     //I term  Tsampl=50E-6 (50us)
    Vol_KI_out = Vol_KI_out + KI_out_old;

    if      (Vol_KI_out >       2) Vol_KI_out = 2;
    else if (Vol_KI_out < I_com_1) Vol_KI_out = I_com_1;

    Vol_err_PI_out = Vol_KP_out + Vol_KI_out;

    if      (Vol_err_PI_out >       2) Vol_err_PI_out = 2;
    else if (Vol_err_PI_out < I_com_1) Vol_err_PI_out = I_com_1;
}


__interrupt void scia_txFifo_isr(void)
{
    Uint32 i;
    for (i = 0; i < 4; i++)
        SciaRegs.SCITXBUF = gSciTxBuf[i] & 0x00FF;

    SciaRegs.SCIFFTX.bit.TXFFIENA   = 0;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;        /* Clear Interrupt flag */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;     /* Acknowledge interrupt to PIE */
    return ;
}

void Digital_Input(void) // 글리치 제거 프로그램 할 것
{
//    DigitalIn.all = 0;
    DigitalIn.bit.Bit0 = GpioDataRegs.GPADAT.bit.GPIO11;   // GPIO for DIP Switch_1 net_name GPIO5
    DigitalIn.bit.Bit1 = GpioDataRegs.GPADAT.bit.GPIO10;   // GPIO for DIP Switch_2 net_name GPIO6
    DigitalIn.bit.Bit2 = GpioDataRegs.GPBDAT.bit.GPIO55;   // GPIO for DIP Switch_3 net_name GPIO7
    DigitalIn.bit.Bit3 = GpioDataRegs.GPBDAT.bit.GPIO41;   // GPIO for DIP Switch_4 net_name GPIO8

    DigitalIn_old_old.all = DigitalIn_old.all;
    DigitalIn_old.all = DigitalIn.all;

    //글리치 판단해서 세이브
    if(DigitalIn_old_old.all == DigitalIn_old.all == DigitalIn.all)
    Board_ID = DigitalIn.all;

    switch_1_old_old = switch_1_old; switch_1_old = switch_1; switch_1 = GpioDataRegs.GPBDAT.bit.GPIO54; // 30ms
    if     (switch_1_old_old == 1 && switch_1_old == 1 && switch_1 == 1) filtered_switch_input = 1;
    else if(switch_1_old_old == 0 && switch_1_old == 0 && switch_1 == 0) filtered_switch_input = 0;
}


void Calculating_current_average_and_monitoring_average(void)
{
    Io_ad_avg = Io_ad_sum * 0.2f;
    Io_ad_sum = 0;

    Io_sen_real =  (Io_ad_avg - 2048) * 0.048828125f;//0.048851978f; // 100 / 2048; // adc 3v -> 4095, 0A : 1.5V, +100A : 3V, -100A : 0V
    Io_sen_sum += Io_sen_real;

    if (Current_Average++ == MON_MAXCNT) //  FOR MONITORING
    {
      Io_sen.fValue = Io_sen_sum * MON_MAXCNT_REV; //200번 평균;
      Io_sen_sum = 0;
      Current_Average = 0;
    }
}

void Calculating_voltage_average_and_monitoring_average(void)
{
    Vo_sen_sum_mon += Vo_sen;

    if (MonitoringCount++ == 10000) //  FOR MONITORING
    {
        Vo_Mean = Vo_sen_sum_mon * 0.0001;
        Vo_sen_sum_mon = 0.;
        MonitoringCount = 0;
        Vo_sen_avg.fValue = Vo_Mean; // Monitor value
    }
}


void FAN_pwm_service(void)
{
    //fan pwm service
//            unsigned int fan_pwm_duty1 = 0;

            fan_pwm_duty =  0.8 * (In_Temp - 36)/(52.4f - 36) + 0.2; // In_Temp가 36도씨 일때 20%, 52.4도씨 일때 100%

            if     (fan_pwm_duty < 0.15) fan_pwm_duty = 0.15; //최소 15%로 제한
            else if(fan_pwm_duty > 0.9 ) fan_pwm_duty = 0.9; //최대 90%로 제한

            EPwm1Regs.CMPA.half.CMPA = (1. - fan_pwm_duty) * PWM_PERIOD_10k; //fan, PWM_PERIOD_10k

//            fan_pwm_out_trip_old = fan_pwm_out_trip;
}

void error(void)
{
   __asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

//###########################################################################
// CLA ISRs
//###########################################################################
__interrupt void cla1_task1_isr(void)
{
    // GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;   //LED profiling
    PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

//===========================================================================
// No more.
//===========================================================================

__interrupt void ecan0_isr(void)
{
    Uint16 mbox30cnt = 0;
    
    // 테스트용으로 MBOX31만 처리
    if (ECanaRegs.CANRMP.bit.RMP30)
    {
        mbox30cnt++;
        test_data[0] = ECanaMboxes.MBOX30.MDL.byte.BYTE0;
        test_data[1] = ECanaMboxes.MBOX30.MDL.byte.BYTE1;
        test_data[2] = ECanaMboxes.MBOX30.MDL.byte.BYTE2;
        test_data[3] = ECanaMboxes.MBOX30.MDL.byte.BYTE3;
        test_data[4] = ECanaMboxes.MBOX30.MDH.byte.BYTE4;
        test_data[5] = ECanaMboxes.MBOX30.MDH.byte.BYTE5;
        test_data[6] = ECanaMboxes.MBOX30.MDH.byte.BYTE6;
        test_data[7] = ECanaMboxes.MBOX30.MDH.byte.BYTE7;
        
        
        // ID 확인 후 적절한 함수 호출 (하위 비트가 0x210인지 확인)
        if (ECanaMboxes.MBOX31.MSGID.bit.EXTMSGID_L == 0x210)
        {
            ProcessCommand(30);
        }
        else
        {
            SaveCommand(30);
        }
        
        ECanaRegs.CANRMP.bit.RMP30 = 1;
    }
    
    // 인터럽트 플래그 초기화
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;
    
    // PIE 인터럽트 응답
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

