#include "sicDCDC35kw_setting.h"

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "modbus.h"

void InitEPwm1Example() //PWM1 is reserved for 2nd year design (only 3-phase interleaved)
{
    EPwm1Regs.TBPRD = PWM_PERIOD_10k;                  // Set timer period
    EPwm1Regs.TBCTR = 0x0000;                      // Clear counter
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up-down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading - master module
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;         // Load on shadow mode
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync down-stream module

    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm1Regs.CMPA.half.CMPA = DUTY1_INIT;
    EPwm1Regs.CMPB = PWM_PERIOD_10k;                    //turn-off PWM1B at 1st run

    // Set actions - PWM1 is duty NON inverted, due to the buck switches and control PWMs
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;              // ReSet PWM1A on CAU
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;               // Set PWM1B on CBU
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = EPWM_DB;
    EPwm1Regs.DBFED = EPWM_DB;
    //EPwm1_DB_Direction = DB_UP;

    //Trip zone config
    // Enable TZ1 and TZ2 as one shot trip sources
    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm1Regs.TZSEL.bit.OSHT2 = 1;
    // What do we want the TZ1 and TZ2 to do?
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
/*
    // Interrupt where we will change the Deadband
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event
*/
}

void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = PWM_PERIOD_100k;                   // Set timer period
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm2Regs.TBPHS.half.TBPHS = 0; // Phase is 0 degree = degree/180*PERIOD (for UP/DOWN mode) respected to PWM1

    // Setup TBCLK
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up-DOWN
//kcs    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Enable phase loading - slave module
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading - master module

    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;           //

    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
//kcs    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;      // sync flow-through
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;    // Sync down-stream module

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm2Regs.CMPA.half.CMPA = DUTY2_INIT;
    EPwm2Regs.CMPB = PWM_PERIOD_100k;                    // turn-off PWM2B at 1st run

    // Set actions, PWM2 is duty NON inverted
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;              // ReSet PWM2A on CAU
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;               // ReSet PWM2B on CBU
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = EPWM_DB;
    EPwm2Regs.DBFED = EPWM_DB;
    //EPwm2_DB_Direction = DB_UP;

    //Trip zone config
    // Enable TZ1 and TZ2 as one shot trip sources
    EALLOW;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
    EPwm2Regs.TZSEL.bit.OSHT2 = 1;
    // What do we want the TZ1 and TZ2 to do?
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
/*kcs
    // Interrupt where we will modify the deadband
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;        // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event
*/
}
void InitEPwm3Example()
{
    EPwm3Regs.TBPRD = PWM_PERIOD_100k-1;//1799;//899; //PWM_PERIOD_100k;                    // Set timer period
    EPwm3Regs.TBCTR = 0x0000;                        // Clear counter
//kcs    EPwm3Regs.TBPHS.half.TBPHS = 2 * PWM_PERIOD / 3; // Phase is 120 degree = degree/180*PERIOD (for UP/DOWN mode) respected to PWM1
    EPwm3Regs.TBPHS.half.TBPHS = 0; // Phase is 120 degree = degree/180*PERIOD (for UP/DOWN mode) respected to PWM1

    // Setup TBCLK
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;   // Count up-DOWN
//kcs    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;           // Enable phase loading
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;           // Enable phase loading

    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;           //

//kcs    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // sync flow-through
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sync flow-through

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;    // load on CTR=Zero
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;    // load on CTR=Zero

    // Setup compare
    EPwm3Regs.CMPA.half.CMPA = (PWM_PERIOD_100k-1) * (1 - (float)0.1); //  10% duty, FPGA Chip select
    EPwm3Regs.CMPB = 0;                    // turn-off PWM3B at 1st run

    // Set actions - PWM3 is NON inverted
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;               // ReSet PWM3A on CAU
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;

//    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;                 // Set PWM3B on CBU
//    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;




    // Interrupt where we will change the deadband
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;        // Select INT on Prd event
    EPwm3Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate INT
}

void AdcSetup(void)
{
    // Configure ADC
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;    // Enable non-overlap mode
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; // ADCINT1 trips after AdcResults latching
    AdcRegs.INTSEL1N2.bit.INT1E = 1;      // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;      // Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL = 0;  // Setup EOC8 to trigger ADCINT1 to fire

    //Map the ADC channels
    //HW pin and header file
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 8;      // Set SOC0 channel select to ADCINB0
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 9;      // Set SOC1 channel select to ADCINB1
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 10;     // Set SOC2 channel select to ADCINB2

    //Select the trigger sources
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0F; // Set SOC0 triggered on EPWM6A SOCA for inductor current IL2, due to round-robin SOC0 converts first then SOC1, SOC2
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0F; // Due to round-robin SOC1 is after SOC0
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0F; // Due to round-robin SOC2 is after SOC1

    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6; // Set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;

    EDIS;

}

void Gpio_select(void)
{

    EALLOW;

//kcs    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // GPIO for RUN_LED
//kcs    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;   // GPIO for FLT_LED

    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;   // GPIO for adc cnv cs

    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;   // GPIO for DIP Switch_1
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;   // GPIO for DIP Switch_2
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;   // GPIO for DIP Switch_3
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;   // GPIO for DIP Switch_4

    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;   // GPIO for External "S" Sensing1
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;   // GPIO for External "S" Sensing2
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;   // GPIO for External "S" Sensing3
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;   // GPIO for External Switch
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;   // GPIO for BUCK Enable
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;   // GPIO for Discharge
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;   // GPIO for dac cs
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;   // GPIO for debugging LED2
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;   // GPIO for debugging LED3

//kcs    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;   // GPIO for RUN_LED
//kcs    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;   // GPIO for FLT_LED

    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;   // GPIO for adc cnv cs
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;   // GPIO for adc cnv cs

    GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;   // GPIO for DIP Switch_1_input
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;   // GPIO for DIP Switch_2_input
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;   // GPIO for DIP Switch_3_input
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;   // GPIO for DIP Switch_4_input

    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;  // GPIO for External "S" Sensing1_input
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0;  // GPIO for External "S" Sensing2_input
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;  // GPIO for External "S" Sensing3_input
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0;   // GPIO for External Switch_input
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;   // GPIO for BUCK Enable_output
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;   // GPIO for Discharge_output
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;   // GPIO for dac cs output
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;   // GPIO for debugging LED2_output
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;   // GPIO for debugging LED3_output


    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1; // pullup disable
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1; // pullup disable
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // pullup disable
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1; // pullup disable

    // Set input qualifcation period for GPIO54
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 0xFF;  // Qual period = SYSCLKOUT/510
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;      // 6 samples

    EDIS;

}
//SPI-A initialization
//void spi_init()
//{
//    SpiaRegs.SPICCR.all =0x000F;          // Reset on, rising edge, 16-bit char bits
//    SpiaRegs.SPICTL.all =0x0006;          // Enable master mode, normal phase,
//                                          // Enable talk, and SPI int disabled.
//    SpiaRegs.SPIBRR =0x0009;              // SPI baud rate = LSCLK/(SPIBRR+1) = LSCLK/10
//                                          //               = (90M/4)/10 = 2.2M
//                                          // 16-bit = 178kHz
//
//    SpiaRegs.SPICCR.all =0x009F;          // Relinquish SPI from Reset
//    SpiaRegs.SPIPRI.bit.FREE = 1;         // Set so breakpoints don't disturb xmission
//}

void spi_init()
{
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    // CPOL = 0, CPHA = 1
    SpiaRegs.SPICCR.all = 0x000F;     // Reset on, rising edge, 16-bit char bits
    SpiaRegs.SPICTL.all = 0x0006;          // Enable master mode, normal phase,
                                           // Enable talk, and SPI int disabled.

    SpiaRegs.SPICTL.bit.TALK = 1;
//    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
    SpiaRegs.SPIBRR = 0x0007;           // SPI Baud Rate = LSPCLK / (SPIBRR + 1)
//    SpiaRegs.SPICCR.all =0x009F;          // Relinquish SPI from Reset
            // 변경레지스터
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;        // SPI Interrupt disable
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0; // SPICLK signal delayed by one half-cycle

    //   Uint16  TALK:1;             // 1    Master/Slave Transmit Enable
//    Uint16  MASTER_SLAVE:1;     // 2    SPI Network Mode Control
//    Uint16  CLK_PHASE:1;        // 3    SPI Clock Phase

    SpiaRegs.SPICCR.all = 0x00DF;          // Relinquish SPI from Reset
    SpiaRegs.SPICCR.bit.SPILBK = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPIPRI.bit.FREE = 1;   // Set so breakpoints don't disturb xmission

    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

#if 0

    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;

    SpiaRegs.SPICCR.bit.SPILBK = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = 15;

    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    SpiaRegs.SPIBRR = 0x0003;

    SpiaRegs.SPICTL.bit.SPIINTENA = 0; //인터럽트 비사용

       SpiaRegs.SPICTL.bit.TALK = 1;
       SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
       SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
       SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;

       SpiaRegs.SPIPRI.bit.FREE = 1;

#endif

}

//void spi_init()
//{
//    SpiaRegs.SPICCR.bit.SPISWRESET   = 0;        // SPI Software RESET for changing configuration
//    SpiaRegs.SPICCR.all              = 0x004F;   // Reset on, falling edge, 16-bit char bits
//    SpiaRegs.SPICTL.all              = 0x0006;   // Enable master mode, normal phase,
//                                                 // enable talk, and SPI int disabled.
// // SpiaRegs.SPICCR.bit.CLKPOLARITY  = 0;        // Data is output on rising edge and input on falling edge
// // SpiaRegs.SPICTL.bit.CLK_PHASE    = 0;        // SPICLK signal is not delayed
// // SpiaRegs.SPICCR.bit.CLKPOLARITY  = 1;        // Data is output on falling edge and input on rising edge
// // SpiaRegs.SPICTL.bit.CLK_PHASE    = 0;        // SPICLK signal is not delayed
// // SpiaRegs.SPICCR.bit.CLKPOLARITY  = 0;        // Data is output on rising edge and input on falling edge
// // SpiaRegs.SPICTL.bit.CLK_PHASE    = 1;        // SPICLK signal delayed by one half-cycle
//    SpiaRegs.SPICTL.bit.SPIINTENA    = 1;        // SPI Interrupt Enable
//    SpiaRegs.SPICTL.bit.CLK_PHASE    = 1;        // SPICLK signal delayed by one half-cycle
//    SpiaRegs.SPIBRR                  = 5;        // SPI Baud Rate = LSPCLK / (SPIBRR + 1)
//    SpiaRegs.SPIPRI.bit.FREE         = 1;        // Set so breakpoints don't disturb xmission
//    SpiaRegs.SPICCR.bit.CLKPOLARITY  = 1;        // Data is output on falling edge and input on rising edge
//    SpiaRegs.SPICCR.bit.SPISWRESET   = 1;        // SPI Software RESET release
//}

void spi_fifo_init()
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all = 0xE040; // FIFO transmit: clear TXFIFINT flag, enable FIFO
    SpiaRegs.SPIFFRX.all = 0x2044; // FIFO receive: interrupts after 4 words, clear RXFIFO flag, enable FIFO
    SpiaRegs.SPIFFCT.all = 0x0;             // No FIFO transmit delay

//    SpiaRegs.SCIFFTX.all = 0xa000; // FIFO reset
//    SpiaRegs.SCIFFCT.all = 0x4000; // Clear ABD(Auto baud bit)
}

void scia_fifo_init(void)
{
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2041;
    SciaRegs.SCIFFCT.all = 0x0;
// 28069 아래 셋팅은 안 됨
//    SciaRegs.SCIFFTX.all = 0xC040;// SCI FIFO Reset, Enhancement Enable, TxFIFO Reset, Clear TxFFINT flag, TxFFIL=00000b;
//    SciaRegs.SCIFFRX.all = 0x4041;// Clear RxFFOVF flag, RxFIFO Reset, Clear RxFFINT flag, RxFFIL=1
    SciaRegs.SCIFFCT.all = 0x0;

//    16-level transmit/receive FIFO in F28232 vs 4-level transmit/receive FIFO in F28034
//    Data-word format has an extra bit to distinguish addresses from data (address bit mode only) in F28034 which is not there in F28232.
 //   정확한 타이밍과 데이터 무결성을 보장하기 위해 에코백 및 ACK/NAK 프로토콜을 구현한 controlSUITE의 예가 있습니다.
 //   device_support/F2837xD/v150/F2837xD_examples_Dual에서 F2837xD_sci_flash_kernels를 살펴보세요.
    //플래시 커널은 SCI_GetFunction.c에 있는 SCI_GetWordData() 함수의 모든 바이트를 반향합니다.
 //   또한 SCI_GetFunction.c에서 SendNAK(), SendACK(), SCI_Flush() 함수를 볼 수 있다.
/*
문자를 전송하는 호스트에 반환된 값을 모니터링하여 데이터 무결성을 보장하는 데 도움이 될 수 있습니다. 당신이 올바른지. 호스트는 다시 전송되는 내용을 모니터링해야 합니다.
RX/BK 인터럽트 수신 및 활성화를 활성화하려면 SCICTL1.bit.RXENA = 1 및 SCICTL2.bit.RXBKINTENA =1을 구성해야 합니다. 그런 다음 SCIFFRX.bit.RXFFIENA = 1을 구성합니다. SCIFFRX.bit.RXFFIL = 4이므로 FIFO가 가득 찼을 때(최대 4레벨) 인터럽트를 생성합니다.
한 번에 45바이트를 얻을 수 없습니다. FIFO로는 최대 4바이트만 수신할 수 있습니다. 데이터가 수신되면 모듈은 RXRDY 비트를 설정하고 SCIRXBUF에서 데이터를 읽어야 합니다.
인터럽트를 구성하지 않고도 SCI에서 데이터를 수신할 수 있습니다.
RXDY를 폴링하고 SCIRXBUF에서 한 번에 한 바이트씩 읽을 때까지 기다렸다가 모든 데이터를 읽을 때까지 계속할 수 있습니다. 아래 의사코드를 참조하세요...
for i < 45
while(SciaRegs.SCIRXST.bit.RXRDY != 1) // 수신 완료 플래그
data = SCIRXBUF.bit.RXDT; //


데이터가 손실되지 않도록 데이터 전송 및 수신 타이밍을 조정해야 할 수도 있습니다. 도움이 되었기를 바랍니다. *
 * */
}

Uint16     sci_baud_capture = 0;
void scia_init(void)
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    SciaRegs.SCICCR.bit.PARITYENA = 1;// Parity enable
    SciaRegs.SCICCR.bit.PARITY = 1;   // Even parity

    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
//    ScibRegs.SCICTL2.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =0;
    SciaRegs.SCICTL2.bit.RXBKINTENA =0;

    sci_baud_capture = SCIA_PRD;
    SciaRegs.SCIHBAUD = SCIA_PRD >> 8;  //  @LSPCLK = 22.5MHz.
    SciaRegs.SCILBAUD = SCIA_PRD & 0xFF;

//    ScibRegs.SCICTL1.all =0x0063;  // Relinquish SCI from Reset

    SciaRegs.SCICTL1.bit.SWRESET = 1;
//    ScibRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset

}

extern struct ECAN_REGS ECanaShadow;
void eCana_config (void)
{
    //
    // Mailboxs can be written to 16-bits or 32-bits at a time
    // Write to the MSGID field of TRANSMIT mailboxes MBOX0 - 15
    //
    //           ECanaMboxes.MBOX0.MSGID.bit.IDE = 1 // 0 = 2.0A 기준 11bit, 1= 2.0B 기준 29bit
    //  can 설정 참고 https://m.blog.naver.com/cyjrntwkd/70120007780

    ECanaMboxes.MBOX0.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX0.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = 0xF0; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX1.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX1.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = 0xF0 + 1; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX2.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX2.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX2.MSGID.bit.STDMSGID = 0xF0 + 2; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX3.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX3.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX3.MSGID.bit.STDMSGID = 0xF0 + 3; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX4.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX4.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX4.MSGID.bit.STDMSGID = 0xF0 + 4; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX5.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX5.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX5.MSGID.bit.STDMSGID = 0xF0 + 5; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX6.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX6.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX6.MSGID.bit.STDMSGID = 0xF0 + 6; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX7.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX7.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX7.MSGID.bit.STDMSGID = 0xF0 + 7; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX8.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX8.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX8.MSGID.bit.STDMSGID = 0xF0 + 8; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX9.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX9.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX9.MSGID.bit.STDMSGID = 0xF0 + 9; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID

    ECanaMboxes.MBOX10.MSGID.bit.IDE = 0;   // ID 확장 여부 설정 : 11Bit ID 사용
    ECanaMboxes.MBOX10.MSGID.bit.AAM = 0;   // 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
    ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = 0xF0 + 10; // Mailbox ID 설정 : CAN2.0A 기준 11Bit ID
    //
    // Configure Mailboxes 0-15 as Tx, 16-31 as Rx
    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.
    //
    ECanaRegs.CANMD.all = 0xFFFFFFFE; // 0 : 송신모드, 1 : 수신모드  -> 이 부분 레지스터와 쉐도우 레지스터 공부할것
    ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;

    ECanaShadow.CANMD.bit.MD0 = 0;     // MailBox 0번 : 송신

    ECanaShadow.CANMD.bit.MD1 = 1;     // MailBox 1번 : 수신
    ECanaShadow.CANMD.bit.MD2 = 1;
    ECanaShadow.CANMD.bit.MD3 = 1;
    ECanaShadow.CANMD.bit.MD4 = 1;
    ECanaShadow.CANMD.bit.MD5 = 1;
    ECanaShadow.CANMD.bit.MD6 = 1;
    ECanaShadow.CANMD.bit.MD7 = 1;
    ECanaShadow.CANMD.bit.MD8 = 1;
    ECanaShadow.CANMD.bit.MD9 = 1;
    ECanaShadow.CANMD.bit.MD10 = 1;
    //
    // Enable all Mailboxes
    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.
    //
    ECanaRegs.CANME.all = 0xFFFFFFFF; // Enable all Mailboxes   // 메일박스 Enable/Disable 설정

    //
    // Specify that 8 bits will be sent/received
    //
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
    ECanaMboxes.MBOX10.MSGCTRL.bit.DLC = 4;

    //
    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.
    //
    EALLOW;
    ECanaRegs.CANMIM.all = 0xFFFFFFFF; // 메일박스 인터럽트 마스크 설정

 //  메일박스 MBOXx 레지스터 의 내용 초기화와 송수신 데이터 크기 설정, 모드 설정을 함.
 //  인터럽트설정 CANMIM, CANMIL, CANGIM 레지스터) 관련 PIE, CPU 인터럽트 라인 설정.
    EDIS;

}
