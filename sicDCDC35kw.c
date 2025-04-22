 //###########################################################################
// 2025.02.26 final ���� ���� Ȯ��

/*
 * HMI�� ������ ���� RS232 Modbus RTU�� ����Ѵ�. 38400bps, 8bit, 1bit, none, HMI���� 100ms polling
 * ������ ���忡�� ����� PI ��� ���� DAC������ ȯ���Ͽ� 485 ������� �����Ѵ�(�������� ����). ��żӵ��� 5.625Mbps�̰�, 0.1ms ���� �ѹ��� 9byte�� �����Ѵ�.
 * ������ ����� �����̺� ����� ���μ����� ���Ƽ� baud �������� ����.
 * ������ ����� DIP ����ġ�� ������� CAN ID�� 0xF0�� �����Ǿ� �ִ�.
 * �����̺� ����� DIP ����ġ�� 1~9���� �����Ͽ� ID�� ���Ѵ�. ������ ���� ID�� 0xF0�� ���Ͽ� �����̺� CAN ID�� �����ȴ�. 0xF1~0xF9
 * CAN�� 485 ��� ���� ������ �� ó���� �� ������ �޾ƾ� �Ѵ�.
 * 485�� ���� ���� ��ܿ� ������ 200mV�̻��� �ɷ��� �Ѵ�.
 * ������ ����� ON/OFF ��ȣ�� �����̺� ���忡 CAN 2.0A ������� �����Ѵ�. ��żӵ� 1Mbps, 1ms polling, 1 frame �� CAN BUS �ε����� �� 8~9% �̴�.
 * �����̺� ������� CAN �켱 ������� �����ͷ� ���� ����(��� ����)�� �����Ѵ�. �� �����̺� ���尡 1ms���� �����ͷ� �����͸� �����Ѵ�. �����Ϳ����� 10ms���ȿ� ������ ��� �����̺��� ���� �����͸� �����Ѵ�.
 * �����̺� ����  RS-232(SCI-B)   �����̺� ������ ���� ���� ����͸� Termial(TeraTerm)   100ms       230400bps
 *
 */

// 20khz(50us)�� PI����, RS485 Tx, CAN Tx�� main����, SPI�� 100khz ����
// OFF ������ �� DAC �� �� 2000 Ȯ��
// -> 3���� �����ؾ� LSPCLK 90Mhz/(3+1) = 22.5Mhz�� �Ǿ �´� ��. ����Բ� Ȯ�� �� ������ �Ǵ���? �ƴϸ� 22.5Mhz�� �´���?
// -> 7�� �����ϸ�  LSPCLK 90Mhz/(7+1) = 11.25Mhz�� ��. ���� ���� �ٲ������ Ȯ�� �ʿ�.

// GPIO5,6,7,8  DIP ����ġ�� CAN ID�� �����Ͽ� Ȯ���Ѵ�.
// master������ ����/���� ��ɰ� ���� ������ slave�� �����Ѵ�.
// master������ PI�� �ϰ� �� ������� slave1,2�� �����Ͽ� slave1,2���� dac�� ����Ѵ�. 2�� �����̺� �����̹Ƿ� 160A �϶� slave1�� slave2�� ���� 4V�� ����Ѵ�. -160A�� ���� 0V�� ����Ѵ�.
// slave�� can���� ����
// slave�� 232�� ������� �����Ƿ� �͹̳��� �츰��. CAN ID ǥ��, master���� ���Ź��� ���� ǥ��


// CAN 2.0B Bus Load Rate -> 2.0A�� �ϸ� 100us �ҿ�ɵ�
/*
1000kbps�� 1 bit�� �����ϴµ� 1us�� �ɸ���.
130bit(CAN 1 Frame)�� �����ϴµ� �ɸ��� �ð��� 130us�̴�.
1ms ���� 1 Frame�� �Ź� �����Ѵٰ� �����ϸ�
1ms �ð����� bus�� 130us���ȸ� ������ ���̰�

130/1000 //  1ms ���� ���� �ε�� 13%�� ���ȴ�. 1khz
130/200  // 0.2ms ���� ���� �ε�� 65%�� ���ȴ�.

������ 25k���� ���´µ� ���� ���α׷������� 0.15ms ���� ���ŵ���.

130/100  // 0.1ms ���ٴ� �� ��. 100%�� �Ѿ� ��. 10khz
 */

//100 1khz = 13%    1ms
//50 2khz = 25%
//10 10khz = 60%    0.1ms -> ���� ���α׷������� 0.22ms�� ���ŵ�
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
 * RXERR INT ENA(SCICTL1�� ��Ʈ 6) ��Ʈ�� Ȱ��ȭ�Ͽ� ���� ������ �߻��� �� ���ͷ�Ʈ�� ������ �� �ֽ��ϴ�.
 * �� ��Ʈ�� Ȱ��ȭ�ϸ� �ߴ��̳� ���� ������ �߻��� �� ���ͷ�Ʈ(PIE �� SCI�� �ùٸ��� �����ߴٰ� ����)�� �����˴ϴ�.
 * SCI RX INT ISR���� SCIRXST �������͸� Ȯ���Ͽ� � ����(�ߴ� ���� ����, �����̹� ����, ������ ���� �� �и�Ƽ ����)�� �߻��ߴ��� �� �� �ֽ��ϴ�.
 * SCICTL1 ���������� SW RESET ��Ʈ�� ����Ͽ� ���� �÷��׸� ������ �� �ֽ��ϴ�.
 * �ߴ� ���� ���� �߻� �� SCI�� ������ ������ �����մϴ�. ������ ������ �����Ϸ��� SW RESET ��Ʈ�� ����Ͽ� SCI�� �缳���ؾ� �մϴ�.
 *  SCI RX ISR���� SCIRXST �������͸� Ȯ���Ͽ� � ������ �߻��ߴ��� Ȯ���� �� �ֽ��ϴ�.
 */

// LED11, LED12, LED13 (PIN 31, 32, 33)
// ADC0 - �µ�
// ADC1 - 100A 3V, 0A 1.5V, -100A 0V (100A 5mA 5V -> 100A 5mA 3V, -100A -5mA -5V -> -100A -5mA -3V)
// 24LC128
// EEROM-SDA GPIO32
// EEROM-SCL GPIO33
// EEROM-WP  GPIO14
// CAN , https://m.blog.naver.com/cyjrntwkd/70120007780
// 232
// I2C
// SPI (ADC chip)


/* �ؾ� ����
FAN �µ��� ���� PWM Duty ���� �� ��. 0.15 fix
�����̺� ���� CAN ID �ߺ� �����Ͽ� �����ͷ� ����
�����Ϳ��� �����̺� CAN ���� �� �Ǹ� ��� ��Ʈ ǥ��
�����̺꿡�� �������� 485������ �� ������ CAN���� ��Ʈ ����
���� �� �ʱ� FAN �� ���ư��� ���� ����
�͹̳ο� ������ �Ҽ��� ǥ��
��� ��Ʈ �߻��� ó�� �� ���
if(SciaRegs.SCIRXST.bit.PE) // error ó�� �� ��
���� �߻� �� ī��Ʈ�ؼ� ������ �� ��
flash ���� ����� ram���� ����ð� ����

Run ��ȣ�� �����̺�� ���۽� 0,1�� �ƴ� Ư�������� Run = 0xA0, !Run = x00;, �����̺꿡���� ���α׷� �ڵ� ������ ��

*/

//###########################################################################
#define _MAIN_C_

#define I_MAX (80)
#define MODULE_NUM (1)   // ��� ����

#define MON_MAXCNT        (10000.)                        // Monitoring Count for Summing data.
#define MON_MAXCNT_REV    ((float)((1)/(MON_MAXCNT)))  // ��� ���� �� �������� �������� �ϱ� ���ؼ�

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

#include "Terminal.h"


void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);

void spi_init(void);
void eCana_config (void);


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

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void epwm1_isr(void);
//__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void spi_isr(void);
//__interrupt void scia_rxFifo_isr(void);
__interrupt void scia_txFifo_isr(void);


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




#define STOP (0)
#define START (1)

#define OVER_VOLTAGE (1100)


//���� ���� ��ü ���� ��� ������ ���� ������ �� ���Դϴ�.
//SpiaRegs.SPITXBUF =
#define FAST_REF1 0xD001 //���� 1.024V
#define SLOW_REF1 0x9001
#define FAST_REF2 0xD002 //���� 2.048V
#define SLOW_REF2 0x9002
#define FAST_REF_E 0xD000 //�ܺ�
#define SLOW_REF_E 0x9000

Uint16 fifo_err_cnt = 0;


Uint16 LoopCount = 0;
Uint16 ErrorCount= 0;

Uint16 _1second_flag = 0, force_reset_test = 0;


// ���� ���� ����
#define STX (0x02) // Start of Text, ������ ���� �� ���� �޼��� ����� ���Ḧ ǥ��
#define ETX (0x03) // End of Text, ������ ���Ḧ ǥ���Ѵ�
#define DLE (0x10) // Data link escape, �ڵ����� ���ӵ� ���ڵ��� �ǹ̸� �ٲٱ� ���� ���, �ַ� ������ ������������ ����


Uint16 gSciTxBuf[4] = {STX,0,0,ETX};


int16 slop = 0;
Uint16 can_tx_flag = 0;

Uint16 can_rx_fault_cnt[11], detect_module_num = 1;

Uint16 size_check = 0;

// SD2 //GPIO3   SPISOMIA(������ ����, ��Ʈ �̸��� ����)
// SCK2 //GPIO18 SPICLKA (������ ����,  ��Ʈ �̸��� ����) DAC1-CLK
// CS_100K // GPIO44 (chip select) ���� �״�� ���

// Gpio_select() ���� �κ�

// ADC-CNV (GPIO7)�� ����
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
//Control Suite ���� ��ġ�ÿ��� SysCtrlRegs.LOSPCP.all = 0x0000; ���� ��������� �Ѵ�. 20247.10.10

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

    Digital_Input(); // board�� id�� �о �ʱ� can id�� �����ϱ� ����

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

   EALLOW;   // This is needed to write to EALLOW protected registers
   En485_ON( );   // GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
   EDIS;     // This is needed to disable write to EALLOW protected registers

   Tx485_ON( ); // for master
//   Rx485_ON( ); // for slave

   msg = "Hello World\0";
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

    InitECanaGpio();         // GPIO�� CAN ��� ������ ����
    InitECana();          // CAN �ʱ�ȭ : CAN2.0A ���, 1Mbps �ӵ�
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
        if(ScibRegs.SCIFFRX.bit.RXFFOVF || force_reset_test) // ���� �� usRegHoldingBuf�� �ʱ�ȭ ���� �ʾƾ��Ѵ�.
        {
            force_reset_test = 0;
            fifo_err_cnt++;
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
            ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
        }

        (void) eMBPoll(); // 2024.06.04 �ݵ�� ���� ���� �������� ȣ���� ��.

        if(can_tx_flag == 1)
        {
            can_tx_flag = 0;
            if(Run == 1) ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0xA0; // 2025.02.17
            else         ECanaMboxes.MBOX0.MDL.byte.BYTE0 = 0;

            ECanaRegs.CANTRS.all = 0x00000001;   // Transmit Request Set // �� 1�ʿ� 6000 ������ ����)

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

        // slave���� �ߺ��� can id�� �����Ͽ� �����ͳ� �����̺��� can �������Ϳ� � ������ �߻��ϴ��� �����Ѵ�.
        // �Ǵ� peak can�� �����Ͽ� busoff �� ������ �����Ͽ� ó�� ����� ã�´�.

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
//    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;      /* SCI �۽� FIFO ���ͷ�Ʈ Enable */
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
    if(tx_count++ >= count_num) // 1khz(1ms) ����
    {
        tx_count = 0;
        can_tx_flag = 1; // main loop ���� ó��
    }

    DAC1_DS(); // DAC1_CS Deactivate
    rx_adc_data = SpiaRegs.SPIRXBUF; // Read ADC data

    ADC1_CS(); // ADC1_CS Activate w Falling Edge ������ ���� �� �ּ� 700ns �Ŀ� �о�� ��.
    DAC1_CS(); // DAC1_CS Activate w Falling Edge

    rx_adc_data_buf_sum += rx_adc_data;
    if(rx_adc_data_buf_sum < 1) rx_adc_data_buf_sum = 1;

    Io_ad_sum += Io_ad;

/********************************
// 20khz loop
 *
 * 20khz �� PI_L�� �ϰ�, �ٽ� 20khz �Ŀ� PI_H�� �����Ѵ�.
 * �׷��� PI_L�� PI_H�� ����(����)�ð��� ���� üũ�غ���. �ΰ��� ���ļ� 100Khz �̳��� ��������.
 * 3.5uS + 1.666uS = 5.1666us
*********************************/
    switch(_100khz_count)
    {
    case 0: //20KHz
        rx_adc_data_buf_ave = (float32)(rx_adc_data_buf_sum * 0.2f);
        rx_adc_data_buf_sum = 0;
        fADC_voltage = v_ref * (float) rx_adc_data_buf_ave * (float)(0.0000152) - 0.1945;// (float)((1ul << 16))
        Vo_sen = fADC_voltage * 250 * 1.04047;// 1.04047 = 1/0.9611;
        Vo = Vo_sen; // Vo ������ ����⿡ ���

        PI_Controller_high();  // PI �����,  battery charger/discharger mode

//        for test code, ������
//        if  (IcomTemp >= 2000) slop = -100;
//        if  (IcomTemp <= 1000) slop = +100;
//        IcomTemp = IcomTemp + slop;

//        for test code, �ﰢ��
//        if     (slop <  90) IcomTemp = 0;
//        else if(slop < 100) IcomTemp = 4000;
//        else                slop = 0;
//        slop++;

//        IcomTemp = (((I_com_set - 0.5) * 0.5 * 50)* 0.5 + 2000) * 1.0005; // ���� ���(Icom_Pos 80A) DA ��� 4.000V. ȸ�� ���(Icom_Neg -80A) DA ��� 0.000V
//        SpiaRegs.SPITXBUF = 0xC000 | (test_out_A & 0xfff); // test code
//        SciaRegs.SCIFFTX.bit.TXFFIENA = 1;      // SCI �۽� FIFO ���ͷ�Ʈ Enable
         _100khz_count++;
        break;

    case 1:
        PI_Controller_low();  // PI �����,  battery charger/discharger mode
        _100khz_count++;
        break;

    case 2: // �µ�, 20khz ����
        //               if(GpioDataRegs.GPBDAT.bit.GPIO39) // DAB_ok �϶��� buck_on �Ǿ�� �Ѵ�.
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

       if (IcomTemp > 4095) IcomTemp = 4095;    // DA ��� ���� ����
       spi_tx_temp = IcomTemp;     //20khz ���� ������Ʈ

       gSciTxBuf[1] = (UCHAR)(IcomTemp & 0x00FF);;
       gSciTxBuf[2] = (UCHAR)((IcomTemp >> 8) & 0xFF);

       for (i = 0; i < 4; i++)
       {
           SciaRegs.SCITXBUF = gSciTxBuf[i] & 0x00FF;  // ��� �����̺� ����� ���� ���� ����
       }
        // fan_pwm�� off �ϰ� DAB_ok�� �ణ�� �����Ŀ� off�ȴ�.
    //softstart �Լ���
       if(Vo >= OVER_VOLTAGE) over_voltage_flag = 1; // ����� ���� 20khz ���� üũ. fault ǥ�� �ʿ�

       if (hw_fault == 0 && over_voltage_flag == 0 && filtered_switch_input == 1) Run = 1;
//       if ( !hw_fault && !over_voltage_flag && filtered_switch_input) Run = 1;
//       if ( !(hw_fault || over_voltage_flag) && filtered_switch_input) Run = 1;

       else Run = 0;

      if(Vo >= OVER_VOLTAGE) over_voltage_flag = 1; // ����� ���� 20khz ���� üũ. fault ǥ�� �ʿ�
      else if (filtered_switch_input == 0 ) over_voltage_flag = 0;


//       if (dab_ok)
//       {
//           if (filtered_switch_input) // �ܺ� ON/OFF ����ġ
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

    //���� ���� ������ ���� �κ��� 0.1% ���� ����ؾ� �Ѵ�.
    SpiaRegs.SPITXBUF = 0xC000 | (spi_tx_temp & 0xfff); // DAC1 A write, fast 100khz ����
//    SpiaRegs.SPITXBUF = 0xC000 | (test_dac & 0xfff); // DAC1 A write, fast 100khz ����

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
// SOC0 ����
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
    Io_sen_total.fValue = ((Io_avg + Io_sense1.fValue + Io_sense2.fValue + Io_sense3.fValue + Io_sense4.fValue + Io_sense5.fValue + Io_sense6.fValue + Io_sense7.fValue + Io_sense8.fValue + Io_sense9.fValue));// ��� ����

    usRegInputBuf[4] = (Uint16) (Io_sen_total.ulValue);
    usRegInputBuf[5] = (Uint16) (Io_sen_total.ulValue >> 16);

//    usRegInputBuf[11] =  (Uint16)Vo_sen.ulValue;// ��� ����

    usRegInputBuf[10] = (Uint16) (Vo_sen_avg.ulValue);
    usRegInputBuf[11] = (Uint16) (Vo_sen_avg.ulValue >> 16);

   if     (usRegHoldingBuf[0] & 0x0008)  start_stop = START; //
   else if(usRegHoldingBuf[0] & 0x0010)  start_stop = STOP;  //

   usRegHoldingBuf[2] = 64; // force value
   eChargeMode = (eCharge_DisCharge_Mode)usRegHoldingBuf[2];

   switch(eChargeMode){                // ������ ���� UI�� -������ ���� �� �����Ƿ� �ִ� 160A�� �޾Ƽ� -80A �ؼ� ���(�ӽ÷�)
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

void Digital_Input(void) // �۸�ġ ���� ���α׷� �� ��
{
//    DigitalIn.all = 0;
    DigitalIn.bit.Bit0 = GpioDataRegs.GPADAT.bit.GPIO11;   // GPIO for DIP Switch_1 net_name GPIO5
    DigitalIn.bit.Bit1 = GpioDataRegs.GPADAT.bit.GPIO10;   // GPIO for DIP Switch_2 net_name GPIO6
    DigitalIn.bit.Bit2 = GpioDataRegs.GPBDAT.bit.GPIO55;   // GPIO for DIP Switch_3 net_name GPIO7
    DigitalIn.bit.Bit3 = GpioDataRegs.GPBDAT.bit.GPIO41;   // GPIO for DIP Switch_4 net_name GPIO8

    DigitalIn_old_old.all = DigitalIn_old.all;
    DigitalIn_old.all = DigitalIn.all;

    //�۸�ġ �Ǵ��ؼ� ���̺�
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
      Io_sen.fValue = Io_sen_sum * MON_MAXCNT_REV; //200�� ���;
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

            fan_pwm_duty =  0.8 * (In_Temp - 36)/(52.4f - 36) + 0.2; // In_Temp�� 36���� �϶� 20%, 52.4���� �϶� 100%

            if     (fan_pwm_duty < 0.15) fan_pwm_duty = 0.15; //�ּ� 15%�� ����
            else if(fan_pwm_duty > 0.9 ) fan_pwm_duty = 0.9; //�ִ� 90%�� ����

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
