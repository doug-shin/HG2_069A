

#define __TERMINAL_C

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
//#include "F2806x_Cla_defines.h"
//#include "string.h"
//#include "math.h"
//#include "stdint.h"
//#include "stm32f10x.h"
//#include "HMI_modbus.h"
//#include "main.h"
//#include "stm32f10x_it.h"
//#include "FirmwareUpdate.h"
//#include "io.h"
//#include "infy_module.h"
//#include "PLC_3_0.h"
#include "ctype.h"

#include "Terminal.h"
//#include "sicDCDC35kw.h"

//#include "iostream.h"

//#include "stdlib.h"
//#include "time.h"

//#include "sicDCDC35kw_setting.h"


#include "sicDCDC35kW.h"

Uint16 test_variable = 20;


// 232 로그 출력 ON/OFF - UI에 플래그 만들어서 관리
Uint16 user_test_val = 0;
Uint16 test_Vout_val_1 = 100, test_Vout_val_2 = 200, test_Vout_val_3 = 300, test_Vout_val_4 = 400, test_Vout_val_5 = 500;
Uint8 test_Iout_val_1 = 0, test_Iout_val_2 = 10, test_Iout_val_3 = 20, test_Iout_val_4 = 30, test_Iout_val_5 = 40;

Uint32 elapsed_time = 0; // 100ms
float elapsed_time_s = 0;


Uint16 minute;
Uint16 hour;
Uint16 second;


Uint32 g_elapsed_time = 0; // 100ms
float g_elapsed_time_s = 0;

Uint16 g_day;
Uint16 g_minute;
Uint16 g_hour;
Uint16 g_second;

Uint8 _1s_timer = 0;
//Uint8 _1sec_flag = 0;
//Int8 count_down = 10;

Uint8 charger_status;


StructUart Uart1, Uart2;


/*******************************************************************************
 * Function Name : char HexToAscii(u8 hex)
 * Description   : 16진수를 아스키코드로 변환
 * Parameters    : 1byte
 * Return        : 아스키코드
 *******************************************************************************/
char HexToAscii(Uint8 hex)
{
    return hex < 10 ? hex + '0' : hex - 10 + 'A';
}

/*******************************************************************************
 * Function Name : void UART1_PutChar(u8 data)
 * Description   : UART1 1문자 출력, Transmit one byte
 * Parameters    : 아스키코드
 * Return        : None
 *******************************************************************************/
void UART1_PutChar(Uint8 data)
{
//   USART_SendData(USART1,data);
//   while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0){}
    SciaRegs.SCITXBUF = data;
}

void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;
}

void scia_msg(char *msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}





/*******************************************************************************
 * Function Name : u8 UART1_GetChar(u8 *pData)
 * Description   : UART1로 수신버퍼에서 데이타(1byte) 가져오기
 * Parameters    : none
 * Return        : 데이타 수신 여부(0: 수신버퍼가 비었음, 1: 수신버퍼에 데이타가 있음)
 *******************************************************************************/
Uint8 UART1_GetChar(Uint8 *pData)
{
    if (Uart1.RxInCnt == Uart1.RxOutCnt) return 0;
    else
        *pData = Uart1.RxBuf[Uart1.RxOutCnt];

    if (Uart1.RxOutCnt < RBUF_SIZE - 1) Uart1.RxOutCnt++;
    else                                Uart1.RxOutCnt = 0;

    return 1;
}

/*******************************************************************************
 * Function Name : void UART1_PutStr(char *pStr)
 * Description   : UART1 믄자열 출력
 * Parameters    : 문자열
 * Return        : 1byte
 *******************************************************************************/
void UART1_PutStr(char *pStr)
{
    while (*pStr != '\0')
    {
        UART1_PutChar(*(pStr++));
    }
}

/*******************************************************************************
 * Function Name : void UART1_PutHex(u8 hex)
 * Description   : UART1 16진수값 1byte출력
 * Parameters    : 1byte
 * Return        : none
 *******************************************************************************/
void UART1_PutHex(Uint8 hex)
{
    UART1_PutChar(HexToAscii(hex >> 4));
    UART1_PutChar(HexToAscii(hex & 0x0F));
}

/*******************************************************************************
 * Function Name : void UART1_Put2Hex(u16 hex)
 * Description   : UART1 16진수값 2byte 출력
 * Parameters    : 2byte
 * Return        : none
 *******************************************************************************/
void UART1_Put2Hex(Uint16 hex)
{
    UART1_PutHex(hex / 0x100);
    UART1_PutHex(hex % 0x100);
}

/*******************************************************************************
 * Function Name : void UART1_Put4Hex(u32 hex)
 * Description   : UART1 16진수값 4byte 출력
 * Parameters    : 4byte
 * Return        : none
 *******************************************************************************/
void UART1_Put4Hex(Uint32 hex)
{
    UART1_Put2Hex(hex / 0x10000);
    UART1_Put2Hex(hex % 0x10000);
}

/*******************************************************************************
 * Function Name : UART1_PutDec(u8 cnt,u16 dec)
 * Description   : UART1 10진수값 출력(1~5자리)
 * Parameters    : 출력 자리수, 2byte
 * Return        : none
 *******************************************************************************/
void UART1_PutDec(Uint8 cnt, Uint16 dec)
{
    if (cnt == 5) UART1_PutChar(HexToAscii(dec / 10000))       , cnt--;
    if (cnt == 4) UART1_PutChar(HexToAscii(dec % 10000 / 1000)), cnt--;
    if (cnt == 3) UART1_PutChar(HexToAscii(dec % 1000 / 100))  , cnt--;
    if (cnt == 2) UART1_PutChar(HexToAscii(dec % 100 / 10))    , cnt--;
    if (cnt == 1) UART1_PutChar(HexToAscii(dec % 10));
}

/*******************************************************************************
 * Function Name : u8 AsciiToHex(char ascii)
 * Description   : 아스키코드를 16진수로 변환
 * Parameters    : 아스키코드
 * Return        : 16진수
 *******************************************************************************/
Uint8 AsciiToHex(char ascii)
{
    ascii = toupper(ascii);
    return ascii >= 'A' ? ascii - 'A' + 10 : ascii - '0';
}

Uint8 Term_Send_RAMStr(Uint8 *str)
{
    Uint8 count = 0;            // Initialize byte counter

    while (*str != 0)                  // Reached zero byte ?
    {
        UART1_PutChar(*str);
        str++;                          // Increment ptr
        count++;                        // Increment byte counter
    }

    return count;                       // Return byte count
}

void total_operating_time(void) // 정확한 인터럽트 100ms 타이머에서 call 할 것
{
    if (g_elapsed_time++ >= 864000) // 1day = 24hour = 1440min = 86400sec, 100ms timer
    {
        g_day++;
        g_elapsed_time = 0;
    }
    g_elapsed_time_s = g_elapsed_time * 0.1F; // sec timer

    g_minute = (Uint16) (g_elapsed_time_s * 0.0166666667);
    g_hour   = (Uint16) (g_minute * 0.0166666667);
    g_minute = (Uint16) (g_minute % 60);
    g_second = (Uint16) ((Uint16) g_elapsed_time_s % 60);
}

Uint16 number = 0;
void Send_Terminal (void)
{
    //    elapsed_time++; // 100ms timer
    //    elapsed_time_s = elapsed_time * 0.1F; // sec timer
    //
    //    minute = (uint16_t)(elapsed_time_s * 0.0166666667);
    //    hour   = (uint16_t)(minute * 0.0166666667);
    //    minute = (uint16_t)(minute % 60);
    //    second = (uint16_t)((u16)elapsed_time_s % 60);

    //    scib_xmit(SendChar);
#if 0
    UART1_PutDec(5, number++);
    UART1_PutStr("ID: "); UART1_PutDec(2, Board_ID); UART1_PutStr(" ");

//    if(start_stop == 1) {UART1_PutStr(T_BrRED);  UART1_PutStr(" ON "); UART1_PutStr(T_RESET);}
//    else                {UART1_PutStr(T_BrGREEN);UART1_PutStr("OFF "); UART1_PutStr(T_RESET);}

    switch(eChargeMode){
    case ElectronicLoad_CV_Mode        :  msg = "전자로드    -CV \0"; break; //  2
    case ElectronicLoad_CC_Mode        :  msg = "전자로드    -CC \0"; break; //  4
    case ElectronicLoad_CR_Mode        :  msg = "전자로드    -CR \0"; break; //  8
    case PowerSupply_CV_Mode           :  msg = "전원공급장치-CV \0"; break; //  16
    case PowerSupply_CC_Mode           :  msg = "전원공급장치-CC \0"; break; //  32
    case Battery_Charg_Discharg_CC_Mode:  msg = "배터리      -CC \0"; break; //  64
    case As_a_Battery_CV_Mode          :  msg = "배터리      -CV \0"; break; // 128
    default                            :  msg = "UI모드 선택안됨 \0"; break;
    }
    scia_msg(msg);

    if(eChargeMode == Battery_Charg_Discharg_CC_Mode)
    {
        msg = "Vh_lim: "; scia_msg(msg); // 지령전압 high
        if     (V_high_limit >= 1000) {                    UART1_PutDec(4, (Uint16)V_high_limit);}
        else if(V_high_limit >=  100) {UART1_PutStr(" ");  UART1_PutDec(3, (Uint16)V_high_limit);}
        else if(V_high_limit >=   10) {UART1_PutStr("  "); UART1_PutDec(2, (Uint16)V_high_limit);}
        else                          {UART1_PutStr("   ");UART1_PutDec(1, (Uint16)V_high_limit);}
        UART1_PutStr(" [V] ");

        msg = "Vl_lim: "; scia_msg(msg); // 지령전압 low
        if     (V_low_limit >= 100) {                   UART1_PutDec(3, (Uint16)V_low_limit);}
        else if(V_low_limit >=  10) {UART1_PutStr(" "); UART1_PutDec(2, (Uint16)V_low_limit);}
        else                         {UART1_PutStr("  ");UART1_PutDec(1, (Uint16)V_low_limit);}
        UART1_PutStr(" [V] ");

        UART1_PutStr("I_Ref: ");       // 지령전류
        if     (Iout_Reference >= 100) {UART1_PutStr(" ");  UART1_PutDec(3, (Uint16)Iout_Reference);}
        else if(Iout_Reference >=  10) {UART1_PutStr("  "); UART1_PutDec(2, (Uint16)Iout_Reference);}
        else if(Iout_Reference <    0) {UART1_PutStr(T_BrRED);UART1_PutStr("-"); UART1_PutDec(3,       ~(Iout_Reference)+1); UART1_PutStr(T_RESET);} // two's complement
        else                           {UART1_PutStr("   ");UART1_PutDec(1, (Uint16)Iout_Reference);}
        UART1_PutStr(" [A] ");
    }
    else
    {
        msg = "V_Ref: "; scia_msg(msg); // 지령전압
        if     (Vout_Reference >= 1000) {                    UART1_PutDec(4, (Uint16)Vout_Reference);}
        else if(Vout_Reference >=  100) {UART1_PutStr(" ");  UART1_PutDec(3, (Uint16)Vout_Reference);}
        else if(Vout_Reference >=   10) {UART1_PutStr("  "); UART1_PutDec(2, (Uint16)Vout_Reference);}
        else                            {UART1_PutStr("   ");UART1_PutDec(1, (Uint16)Vout_Reference);}
        UART1_PutStr(" [V] ");

        UART1_PutStr("I_Ref: ");       // 지령전류
        if     (Iout_Reference >= 100) {UART1_PutStr(" ");  UART1_PutDec(3, (Uint16)Iout_Reference);}
        else if(Iout_Reference >=  10) {UART1_PutStr("  "); UART1_PutDec(2, (Uint16)Iout_Reference);}
        else if(Iout_Reference <    0) {UART1_PutStr(T_BrRED);UART1_PutStr("-"); UART1_PutDec(3,       ~(Iout_Reference)+1); UART1_PutStr(T_RESET);} // two's complement
        else                           {UART1_PutStr("   ");UART1_PutDec(1, (Uint16)Iout_Reference);}
        UART1_PutStr(" [A] ");
    }

    UART1_PutStr("AD7980: ");
    UART1_PutDec(1, (Uint16)(fADC_voltage));
    UART1_PutChar('.');
    UART1_PutDec(4, (Uint16)(fADC_voltage * 10000)); // 0.1953125
#endif

    UART1_PutStr("A");
//    if     (Vo >= 100) {                   UART1_PutDec(3, (Uint16)Vo);}
//    else if(Vo >=  10) {UART1_PutStr(" "); UART1_PutDec(2, (Uint16)Vo);}
//    else               {UART1_PutStr("  ");UART1_PutDec(1, (Uint16)Vo);}

//    UART1_PutStr(" Io: ");
//    if     (Io >= 100) {                   UART1_PutDec(3, (Uint16)Io);}
//    else if(Io >=  10) {UART1_PutStr(" "); UART1_PutDec(2, (Uint16)Io);}
//    else if(Io <    0) {UART1_PutStr("-"); UART1_PutDec(3,       ~((int16)Io)+1);} // two's complement
//    else               {UART1_PutStr("  ");UART1_PutDec(1, (Uint16)Io);}

    // RED, YELLOW, GREEN
//    UART1_PutStr(" 온도: ");
//    if     (In_Temp >= 100) {                   UART1_PutDec(3, (Uint16)In_Temp);}
//    else if(In_Temp >=  10) {UART1_PutStr(" "); UART1_PutDec(2, (Uint16)In_Temp);}
//    else                    {UART1_PutStr("  ");UART1_PutDec(1, (Uint16)In_Temp);}
//    UART1_PutStr(" [℃] ");
//
//    Power = Vo * Io * 0.001f; // kw
//
//    UART1_PutStr(" Power: ");
//    if     (Power <    0) {UART1_PutStr("-"); UART1_PutDec(3,       ~((int16)Power)+1);} // two's complement
//    else if(Power >= 100) {                   UART1_PutDec(3, (Uint16)Power);}
//    else if(Power >=  10) {UART1_PutStr(" "); UART1_PutDec(2, (Uint16)Power);}
//    else                  {UART1_PutStr("  ");UART1_PutDec(1, (Uint16)Power);}
//
//    UART1_PutChar('.');
//    UART1_PutDec(3, (int16)(Power * 1000));
//    UART1_PutStr(" [kW] ");




    //UART1_PutStr("\033[31m\n");   // RED
    //UART1_PutStr("\033[32m\n");   // GREEN
    //UART1_PutStr("\033[33m\n");   // YELLOW
    //UART1_PutStr("\033[34m\n");   // BLUE
// 통신 에러
// EEPROM 저장 값

//    msg = "\r\n";
//    scia_msg(msg);
}


