
#define __TERMINAL_H

// wchar_t == unsigned short? 리터럴을 2바이트 문자열로 인식하게 해주는 접두어 L이 있다.
// c언어는 기본적으로 1바이트 형식으로 문자열을 인식하고, 리터럴마저도 그런 법칙에서 벗어나진 않기 때문에 이걸 앞에 붙여주지 않으면 문제가 생긴다.
// 로케일이란게 필요하다. 문자열 처리방식을 어떻게 해야 할까를 정하는 것이다.
// C언어의 로케일은 기본적으로 아스키코드만을 사용하기 때문에 이것도 확장을 해줘야 제대로 처리를 한다.
// 유니코드는 국제표준 문자표이고 UTF-8은 인코딩 방식이다. 

/* Defines -------------------------------------------------------------------*/
#define last_update_year  (23) // 년
#define last_update_month (2) // 월
#define last_update_day   (3)  // 일 , 앞에 08, 09을 입력하면 컴파일 안됨 -> 8진수로 인식

#define fw_version       (351) // 3.5.1

#define UART1           0
#define UART2           1
#define UART3           2
#define RBUF_SIZE       256
#define RBUF_SIZE7      7

#define T_CS1    "\033[1J"  //ESC[1J 은 현재 커서 위치부터 화면을 지운다.
#define T_CS2    "\033[2J"  //ESC[2J 은 전체 화면을 지운다.
#define T_CS3    "\033[3J"  //ESC[3J 은 Scroll-back을 포함한 전체 화면을 지운다.
#define T_CS4    "\033[K"   //ESC[K  은 현재 커서 위치부터 라인을 지운다.
#define T_CS5    "\033[1K"  //ESC[1K 은 라인 처음에서 현재 커서 위치까지 지운다.
#define T_CS6    "\033[2K"  //ESC[2K 은 전체 라인을 지운다.

#define T_RESET        "\033[0m" // All attributes off
#define T_BOLD         "\033[1m" // Bold or increased intensity
#define T_UNDERLINE    "\033[4m" // Underline
#define T_BELL         "\a "

#define T_BrBLACK   "\033[90m"
#define T_BrRED     "\033[91m"
#define T_BrGREEN   "\033[92m"
#define T_BrYELLOW  "\033[93m"
#define T_BrBLUE    "\033[94m"
#define T_BrMAGENTA "\033[95m"
#define T_BrCYAN    "\033[96m"
#define T_BrWHITE   "\033[97m"

#define T_BG_BrBLACK   "\033[100m"
#define T_BG_BrRED     "\033[101m"
#define T_BG_BrGREEN   "\033[102m"
#define T_BG_BrYELLOW  "\033[103m"
#define T_BG_BrBLUE    "\033[104m"
#define T_BG_BrMAGENTA "\033[105m"
#define T_BG_BrCYAN    "\033[106m"
#define T_BG_BrWHITE   "\033[107m"

#define T_BLACK   "\033[30m"
#define T_RED     "\033[31m"
#define T_GREEN   "\033[32m"
#define T_YELLOW  "\033[33m"
#define T_BLUE    "\033[34m"
#define T_MAGENTA "\033[35m"
#define T_CYAN    "\033[36m"
#define T_WHITE   "\033[37m"


/* Type declarations ---------------------------------------------------------*/
typedef struct
{

    Uint16 RxInCnt, RxOutCnt;
    Uint16 TxInCnt, TxOutCnt;
    Uint8 RxBuf[RBUF_SIZE];

} StructUart;


//#ifdef __TERMINAL_C






char HexToAscii(Uint8 hex);
void UART1_PutChar(Uint8 data);
Uint8 UART1_GetChar(Uint8 *pData);
void UART1_PutStr(char *pStr);
void UART1_PutHex(Uint8 hex);
void UART1_Put2Hex(Uint16 hex);
void UART1_Put4Hex(Uint32 hex);
void UART1_PutDec(Uint8 cnt, Uint16 dec);
Uint8 AsciiToHex(char ascii);
void Send_Terminal(void);

void scia_xmit(int a);
void scia_msg(char *msg);

//unsigned char   TX_Frame_a[7];

void scia_msg(char *msg);




