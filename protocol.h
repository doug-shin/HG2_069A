#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <string.h>
#include "sicDCDC35kw.h"

// Endian 변환 매크로
#define SWAP16(data, offset) ((((Uint16)(data)[(offset) + 1]) << 8) | \
                              ((Uint16)(data)[(offset)]))

#define SWAP32(data, offset) ((((Uint32)(data)[(offset) + 3]) << 24) | \
                              (((Uint32)(data)[(offset) + 2]) << 16) | \
                              (((Uint32)(data)[(offset) + 1]) << 8) | \
                              ((Uint32)(data)[(offset)]))

// float32 타입의 Endian 변환 매크로 (GCC 확장 문법)
#define SWAP_FLOAT(data, offset) \
    ({ \
        Uint32 _temp = SWAP32((data), (offset)); \
        *((float32*)&_temp); \
    })

// MBOX 구조체 전용 매크로들 (F28069 CAN 메일박스용)
// Uint8 값을 MBOX의 특정 바이트에 저장
#define PUT_MBOX_BYTE(mbox, byte_pos, value) \
    do { \
        if((byte_pos) == 0) (mbox).MDL.byte.BYTE0 = (Uint8)(value); \
        else if((byte_pos) == 1) (mbox).MDL.byte.BYTE1 = (Uint8)(value); \
        else if((byte_pos) == 2) (mbox).MDL.byte.BYTE2 = (Uint8)(value); \
        else if((byte_pos) == 3) (mbox).MDL.byte.BYTE3 = (Uint8)(value); \
        else if((byte_pos) == 4) (mbox).MDH.byte.BYTE4 = (Uint8)(value); \
        else if((byte_pos) == 5) (mbox).MDH.byte.BYTE5 = (Uint8)(value); \
        else if((byte_pos) == 6) (mbox).MDH.byte.BYTE6 = (Uint8)(value); \
        else if((byte_pos) == 7) (mbox).MDH.byte.BYTE7 = (Uint8)(value); \
    } while(0)

// Uint16 값을 MBOX에 리틀 엔디안으로 저장 (2바이트)
#define PUT_MBOX_UINT16(mbox, start_byte, value) \
    do { \
        PUT_MBOX_BYTE((mbox), (start_byte), (Uint8)((value) & 0xFF)); \
        PUT_MBOX_BYTE((mbox), (start_byte) + 1, (Uint8)(((value) >> 8) & 0xFF)); \
    } while(0)

// Uint32 값을 MBOX에 리틀 엔디안으로 저장 (4바이트)
#define PUT_MBOX_UINT32(mbox, start_byte, value) \
    do { \
        PUT_MBOX_BYTE((mbox), (start_byte), (Uint8)((value) & 0xFF)); \
        PUT_MBOX_BYTE((mbox), (start_byte) + 1, (Uint8)(((value) >> 8) & 0xFF)); \
        PUT_MBOX_BYTE((mbox), (start_byte) + 2, (Uint8)(((value) >> 16) & 0xFF)); \
        PUT_MBOX_BYTE((mbox), (start_byte) + 3, (Uint8)(((value) >> 24) & 0xFF)); \
    } while(0)

// float32 값을 MBOX에 저장 (4바이트, 전역 union 사용)
#define PUT_MBOX_FLOAT(mbox, start_byte, value) \
    do { \
        float_converter.f = (value); \
        PUT_MBOX_UINT32((mbox), (start_byte), float_converter.u); \
    } while(0)

// int16 값을 MBOX에 저장 (2바이트, 부호 있는 정수)
#define PUT_MBOX_INT16(mbox, start_byte, value) \
    do { \
        Uint16 _temp_int16 = (Uint16)(value); \
        PUT_MBOX_UINT16((mbox), (start_byte), _temp_int16); \
    } while(0)

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
    STATE state_machine;      // 모듈 상태 머신 (STATE_IDLE or STATE_RUNNING)
    Uint8 channel;            // 채널 번호
    MODE mode;                // 운전 모드
    STATUS status;            // 상태
    STATE_BITS_UNION state_bits; // 상태 비트
    Uint16 event_code;        // 이벤트 코드
    
    // 2. 설정값
    MODE cmd_mode;            // 운전 모드
    Uint16 cmd_step;           // 스텝 번호
    float32 cmd_voltage;      // 전압 설정값
    float32 cmd_current;      // 전류 설정값
    float32 cmd_power;        // 파워 설정값
    
    // 3. 모니터링값
    float32 fb_voltage;          // 전압
    float32 fb_current;          // 전류
    int16 fb_t1_temp;            // 온도1
    int16 fb_t2_temp;            // 온도2
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
    Uint32 pattern_index;              // 패턴 인덱스 번호
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

// 초기화 및 처리 함수
void InitProtocol(void);

// 메시지 전송 함수
Uint16 SendAck(Uint16 command_id);
Uint16 SendCommandAck(Uint8* data);
void SendCANEndReport(Uint32 mbox_num);
void SendCANReport(Uint32 mbox_num);

// 메일박스 기반 함수 선언 추가
void ProcessCANCommand(Uint32 rx_mbox, Uint32 tx_mbox);  // 메일박스 명령 처리 함수 (수신/송신 분리)

// 상태 전환 함수
void TransitionToRunning(void);
void TransitionToIdle(void);

// 타이머 및 업데이트 함수
void UpdateCANFeedbackValues(void);           // 모든 피드백 값 통합 업데이트

// 메일박스 관리 함수
void SetMBOXChannels(Uint16 start_mbox, Uint16 count);
void ChangeMBOXIDs(Uint16 base_id, Uint16 start_mbox, Uint16 count);

// CAN 인터럽트 관련 함수
void CheckCANHeartBitTimeout(void);                // Heart Bit 타임아웃 체크 함수

// Heart Bit 관련 전역 변수 선언
extern Uint16 can_360_msg_count;       // Heart Bit 메시지 수신 횟수
extern Uint16 can_360_last_count;      // 마지막으로 수신한 Heart Bit 카운트 값
extern Uint16 can_360_timeout_counter; // Heart Bit 타임아웃 카운터
extern Uint16 can_360_timeout_flag;    // Heart Bit 타임아웃 플래그

extern Uint16 hw_fault;
extern Uint16 Run;
extern float32 Vo_sen;
extern Uint16 over_voltage_flag;

// 전역 변수 선언
extern PROTOCOL_INTEGRATED protocol;  // 프로토콜 구조체

// CAN 보고 관련 변수 (protocol.c에서 정의됨)
extern Uint16 can_report_flag;           // CAN 보고 플래그
extern Uint16 can_report_counter;        // CAN 보고 카운터
extern Uint16 can_report_interval;       // CAN 보고 간격

// float32 <-> Uint32 변환용 union 타입 정의
typedef union {
    float32 f;
    Uint32 u;
} FLOAT_CONVERTER_UNION;

// float32 <-> Uint32 변환용 전역 union 변수들
extern FLOAT_CONVERTER_UNION float_converter;


#endif /* PROTOCOL_H_ */
