
#define __TERMINAL_H

// wchar_t == unsigned short? ���ͷ��� 2����Ʈ ���ڿ��� �ν��ϰ� ���ִ� ���ξ� L�� �ִ�.
// c���� �⺻������ 1����Ʈ �������� ���ڿ��� �ν��ϰ�, ���ͷ������� �׷� ��Ģ���� ����� �ʱ� ������ �̰� �տ� �ٿ����� ������ ������ �����.
// �������̶��� �ʿ��ϴ�. ���ڿ� ó������� ��� �ؾ� �ұ ���ϴ� ���̴�.
// C����� �������� �⺻������ �ƽ�Ű�ڵ常�� ����ϱ� ������ �̰͵� Ȯ���� ����� ����� ó���� �Ѵ�.
// �����ڵ�� ����ǥ�� ����ǥ�̰� UTF-8�� ���ڵ� ����̴�. 

/* Defines -------------------------------------------------------------------*/
#define last_update_year  (23) // ��
#define last_update_month (2) // ��
#define last_update_day   (3)  // �� , �տ� 08, 09�� �Է��ϸ� ������ �ȵ� -> 8������ �ν�

#define fw_version       (351) // 3.5.1

#define UART1           0
#define UART2           1
#define UART3           2
#define RBUF_SIZE       256
#define RBUF_SIZE7      7

#define T_CS1    "\033[1J"  //ESC[1J �� ���� Ŀ�� ��ġ���� ȭ���� �����.
#define T_CS2    "\033[2J"  //ESC[2J �� ��ü ȭ���� �����.
#define T_CS3    "\033[3J"  //ESC[3J �� Scroll-back�� ������ ��ü ȭ���� �����.
#define T_CS4    "\033[K"   //ESC[K  �� ���� Ŀ�� ��ġ���� ������ �����.
#define T_CS5    "\033[1K"  //ESC[1K �� ���� ó������ ���� Ŀ�� ��ġ���� �����.
#define T_CS6    "\033[2K"  //ESC[2K �� ��ü ������ �����.

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




