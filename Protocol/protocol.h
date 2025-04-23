#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "F2806x_Cla_typedefs.h"

// 모듈 상태 정의
typedef enum {
    STATE_IDLE = 0,    // 대기 상태
    STATE_RUNNING = 1  // 운전 상태
} STATE;

// 모듈 동작 상태 정의
typedef enum {
    READY = 0,         // 준비
    OPERATING = 1,     // 운전
    CAUTION = 2,       // 경알람
    ALARM = 3,         // 알람
    EMERGENCY = 4,     // 비상정지
    PAUSE = 5,         // 일시정지
    END = 6            // 종료
} STATUS;

// 운전 모드 정의
typedef enum {
    MODE_IDLE = 0,     // 대기
    MODE_CC = 1,       // CC
    MODE_CV = 2,       // CV
    MODE_CC_CV = 3,    // CC/CV
    MODE_CP_CV = 4,    // CP/CV
    MODE_PATTERN = 5,  // Pattern
    MODE_REST = 6,     // Rest
    MODE_PWR_START = 10, // PWR Start
    MODE_PWR_STOP = 50  // PWR Stop
} MODE;

// 제어 명령 코드 정의
typedef enum {
    CMD_START = 0x01,      // 시작
    CMD_STOP = 0x02,       // 정지
    CMD_PAUSE = 0x03,      // 일시정지
    CMD_PWR_START = 0x0A,  // PWR Start
    CMD_PWR_STOP = 0x32,   // PWR Stop
    CMD_ALARM_RESET = 0x0B,// 알람 리셋
    CMD_EMERGENCY = 0xD1   // 비상정지
} COMMAND;

// 상태 비트 유니온 정의
typedef union {
    Uint8 all;             // 전체 바이트 접근
    struct {
        Uint8 parallel_mode    : 1;  // Bit 0: 병렬 ON/OFF
        Uint8 output_relay     : 1;  // Bit 1: 출력 릴레이 ON/OFF
        Uint8 reserved_bits    : 5;  // Bit 2-6: 예약된 비트
        Uint8 pwr_status       : 1;  // Bit 7: PWR ON/OFF
    } bit;
} STATE_BITS_UNION;

// 패턴 데이터 타입 정의
typedef enum {
    PATTERN_DATA_STORE = 1,    // 시작 전 저장
    PATTERN_DATA_REQUEST = 2,  // 요청 데이터
    PATTERN_DATA_LAST = 3      // 마지막 데이터
} PATTERN_DATA_TYPE;

// 패턴 제어 타입 정의
typedef enum {
    PATTERN_CONTROL_CURRENT = 1,  // 전류
    PATTERN_CONTROL_POWER = 2     // Power
} PATTERN_CONTROL_TYPE;

// 통합 프로토콜 구조체 (protocol.md 순서 반영)
typedef struct {
    // 1. 기본 정보
    Uint8 channel;            // 채널 번호
    MODE mode;                // 운전 모드
    STATUS status;            // 상태
    STATE_BITS_UNION state_bits; // 상태 비트
    Uint16 event_code;        // 이벤트 코드
    
    // 2. 설정값
    MODE cmd_mode;            // 운전 모드
    Uint8 cmd_step;           // 스텝 번호
    float32 cmd_voltage;      // 전압 설정값
    float32 cmd_current;      // 전류 설정값
    float32 cmd_power;        // 파워 설정값
    
    // 3. 모니터링값
    float32 fb_voltage;          // 전압
    float32 fb_current;          // 전류
    float32 fb_t1_temp;          // 온도1
    float32 fb_t2_temp;          // 온도2
    float32 fb_charge_ah;        // 충전량 (Ah)
    float32 fb_discharge_ah;     // 방전량 (Ah)
    float32 fb_charge_wh;        // 충전량 (Wh)
    float32 fb_discharge_wh;     // 방전량 (Wh)
    Uint32 fb_operation_time;    // 운전 시간 (10ms 단위)
    Uint32 fb_cv_time;           // CV 모드 시간 (10ms 단위)
    
    // 4. 에러 상태
    Uint16 pwm_hw_error;      // PWM 하드웨어 에러
    Uint16 pwm_sw_error1;     // PWM 소프트웨어 에러1
    Uint16 pwm_sw_error2;     // PWM 소프트웨어 에러2
    Uint16 pwm_sw_warning;    // PWM 경고
    Uint16 dcdc_hw_error;     // DC/DC 하드웨어 에러
    Uint16 dcdc_sw_error1;    // DC/DC 소프트웨어 에러1
    Uint16 dcdc_sw_error2;    // DC/DC 소프트웨어 에러2
    Uint16 dcdc_sw_warning;   // DC/DC 경고
    
    // 5. 종료 조건
    float32 end_condition_voltage;      // 전압 종료 조건
    float32 end_condition_current;      // 전류 종료 조건
    float32 end_condition_capacity_ah;  // Ah 종료 조건
    float32 end_condition_capacity_wh;  // Wh 종료 조건
    Uint32 end_condition_cv_time;       // CV 타임 종료 조건
    
    // 6. 안전 제한
    float32 limit_voltage_min;       // 최소 전압 안전 조건
    float32 limit_voltage_max;       // 최대 전압 안전 조건
    float32 limit_current_charge;    // 충전 전류 안전 조건
    float32 limit_current_discharge; // 방전 전류 안전 조건
    float32 limit_capacity_charge;   // 충전 용량 안전 조건
    float32 limit_capacity_discharge;// 방전 용량 안전 조건
    
    // 7. 시간 설정
    Uint32 time_start;        // 시작 시간 (10ms 단위)
    Uint32 time_operation;    // 운전 시간 설정 (10ms 단위)
    
    // 8. 패턴 데이터
    Uint8 pattern_index;              // 패턴 인덱스 번호
    PATTERN_DATA_TYPE pattern_data_type;    // 패턴 데이터 형태
    PATTERN_CONTROL_TYPE pattern_control_type; // 제어 Type
    Uint16 pattern_time_unit;         // 패턴 단위 시간
    Uint32 pattern_index_value;       // 패턴 인덱스
    float32 pattern_current_or_power; // 전류 또는 파워 값
    
    // 9. 전역 안전 조건
    float32 global_voltage_min;         // 최소 전압 제한 (V)
    float32 global_voltage_max;         // 최대 전압 제한 (V)
    float32 global_current_charge;      // 최대 충전 전류 제한 (A)
    float32 global_current_discharge;   // 최대 방전 전류 제한 (A)
    float32 global_capacity_charge;     // 최대 충전 용량 제한 (Ah)
    float32 global_capacity_discharge;  // 최대 방전 용량 제한 (Ah)
    float32 global_temp_max;            // 최대 온도 제한 (°C)
    float32 global_temp_min;            // 최소 온도 제한 (°C)
    Uint16 global_voltage_change;       // 전압 변화량 (mV)
    Uint16 global_voltage_change_time;  // 전압 변화량 시간 (ms)
    Uint16 global_current_change;       // 전류 변화량 (mA)
    Uint16 global_current_change_time;  // 전류 변화량 시간 (ms)
} PROTOCOL_INTEGRATED;

// CAN 메시지 구조체
typedef struct {
    Uint32 id;      // CAN ID
    Uint8 dlc;      // 데이터 길이
    Uint8 data[8];  // 데이터
} CAN_MESSAGE;

// CAN 메시지 큐 설정
#define CAN_RX_QUEUE_SIZE 32  // 수신 큐 크기

// CAN 메시지 큐 구조체
typedef struct {
    CAN_MESSAGE messages[CAN_RX_QUEUE_SIZE];  // 메시지 배열
    Uint16 head;                              // 큐 헤드 인덱스
    Uint16 tail;                              // 큐 테일 인덱스
    Uint16 count;                             // 현재 메시지 개수
} CAN_QUEUE;

// 전역 변수 선언
extern CAN_QUEUE can_rx_queue;

// 엔디안 변환 함수 선언 (다른 파일에서도 사용 가능)
Uint16 Swap16(Uint16 value);
Uint32 Swap32(Uint32 value);
float32 SwapFloat(float32 value);

// 초기화 및 처리 함수
void InitProtocol(void);
void SaveCommand(Uint16 command_id, Uint8* data);
void ProcessCommand(Uint8* data);

// 메시지 전송 함수
Uint16 SendAck(Uint16 command_id);
Uint16 SendCommandAck(Uint8* data);
void SendEndReport(void);
void SendReport(Uint16 index);
Uint16 SendCANMessage(Uint32 id, const Uint8* data);

// 상태 전환 함수
void TransitionToRunning(void);
void TransitionToIdle(void);

// 타이머 및 업데이트 함수
void PeriodicTask(void);
void UpdateFeedbackValues(void);
void UpdateOperationTime(void);
void UpdateCVTime(void);
void UpdateCapacityValues(void);

// CAN 메시지 큐 관련 함수
void InitCanQueue(void);                           // CAN 큐 초기화
Uint16 CanQueuePush(const CAN_MESSAGE* msg);       // CAN 메시지를 큐에 추가
Uint16 CanQueuePop(CAN_MESSAGE* msg);              // CAN 메시지를 큐에서 가져옴
void ProcessCanMessage(const CAN_MESSAGE* msg);    // CAN 메시지 처리

// CAN 인터럽트 관련 함수
interrupt void Protocol_ECAN0INTA_ISR(void);                // CAN 인터럽트 서비스 루틴
void CheckHeartBitTimeout(void);                   // Heart Bit 타임아웃 체크 함수

// Heart Bit 관련 전역 변수 선언
extern Uint16 can_360_msg_count;       // Heart Bit 메시지 수신 횟수
extern Uint16 can_360_last_count;      // 마지막으로 수신한 Heart Bit 카운트 값
extern Uint16 can_360_timeout_counter; // Heart Bit 타임아웃 카운터
extern Uint16 can_360_timeout_flag;    // Heart Bit 타임아웃 플래그

#endif /* PROTOCOL_H_ */
