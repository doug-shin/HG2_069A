// protocol.c - 프로토콜 처리 모듈
// CAN 통신을 통한 충/방전 명령 처리 및 상태 보고 기능 제공

#include "protocol.h"
#include "F2806x_Cla_typedefs.h"
#include "F2806x_Device.h"
#include <string.h>
#include "sicDCDC35kw.h" // UNIONFLOAT 타입 정의를 위해 추가

#define MODULE_CHANNEL 0x01 // 모듈 채널번호

/**
 * float32 타입의 Endian 변환 함수
 * @param data 바이트 배열
 * @param offset 시작 위치
 * @return 변환된 float32 값
 */
float32 SwapFloat(Uint8* data, Uint16 offset) {
    Uint32 temp = SWAP32(data, offset);
    return *((float32*)&temp);
}

/**
 * Uint16 값을 CAN 메시지 바이트 배열에 엔디안 변환하여 저장하는 함수
 * @param msg 바이트 배열
 * @param offset 시작 위치
 * @param value 저장할 Uint16 값
 */
void CanPutUint16(Uint8* data, Uint16 offset, Uint16 value) {
    data[offset] = (Uint8)(value);    // 하위 바이트
    data[offset + 1] = (Uint8)(value >> 8);       // 상위 바이트
}

/**
 * Uint32 값을 CAN 메시지 바이트 배열에 엔디안 변환하여 저장하는 함수
 * @param msg 바이트 배열
 * @param offset 시작 위치
 * @param value 저장할 Uint32 값
 */
void CanPutUint32(Uint8* data, Uint16 offset, Uint32 value) {
    data[offset] = (Uint8)(value);            // 최하위 바이트
    data[offset + 1] = (Uint8)(value >> 8);   // 하위 중간 바이트
    data[offset + 2] = (Uint8)(value >> 16);  // 상위 중간 바이트
    data[offset + 3] = (Uint8)(value >> 24);  // 최상위 바이트
}

/**
 * float32 값을 CAN 메시지 바이트 배열에 엔디안 변환하여 저장하는 함수
 * @param msg 바이트 배열
 * @param offset 시작 위치
 * @param value 저장할 float32 값
 */
void CanPutFloat32(Uint8* data, Uint16 offset, float32 value) {
    Uint32* temp = (Uint32*)&value;    // float를 Uint32로 재해석
    CanPutUint32(data, offset, *temp);  // Uint32로 변환된 값을 저장
}

/*----------------------------------------------------------------------
 * 전역 변수 정의
 *----------------------------------------------------------------------*/
PROTOCOL_INTEGRATED protocol;  // 프로토콜 구조체
STATE module_state;           // 모듈 상태 (STATE_IDLE or STATE_RUNNING)

// Heart Bit 타임아웃 관련 변수 정의
Uint16 can_360_timeout_counter = 0;  // Heart Bit 타임아웃 카운터
Uint16 can_360_timeout_flag = 0;     // Heart Bit 타임아웃 플래그

// 프로토콜 초기화 함수
void InitProtocol(void) {
    // 기본 정보 초기화
    protocol.channel = MODULE_CHANNEL;
    protocol.mode = MODE_IDLE;
    protocol.status = READY;
    protocol.state_bits.all = 0;
    protocol.event_code = 0;
    
    // 설정값 초기화
    protocol.cmd_mode = MODE_IDLE;
    protocol.cmd_step = 0;
    protocol.cmd_voltage = 0.0f;
    protocol.cmd_current = 0.0f;
    protocol.cmd_power = 0.0f;
    
    // 모니터링값 초기화
    protocol.fb_voltage = 0.0f;
    protocol.fb_current = 0.0f;
    protocol.fb_t1_temp = 0.0f;
    protocol.fb_t2_temp = 0.0f;
    protocol.fb_charge_ah = 0.0f;
    protocol.fb_discharge_ah = 0.0f;
    protocol.fb_charge_wh = 0.0f;
    protocol.fb_discharge_wh = 0.0f;
    protocol.fb_operation_time = 0;
    protocol.fb_cv_time = 0;
    
    // 에러 상태 초기화
    protocol.pwm_hw_error = 0;
    protocol.pwm_sw_error1 = 0;
    protocol.pwm_sw_error2 = 0;
    protocol.pwm_sw_warning = 0;
    protocol.dcdc_hw_error = 0;
    protocol.dcdc_sw_error1 = 0;
    protocol.dcdc_sw_error2 = 0;
    protocol.dcdc_sw_warning = 0;
    
    // 종료 조건 초기화
    protocol.end_condition_voltage = 0.0f;
    protocol.end_condition_current = 0.0f;
    protocol.end_condition_capacity_ah = 0.0f;
    protocol.end_condition_capacity_wh = 0.0f;
    protocol.end_condition_cv_time = 0;
    
    // 안전 제한 초기화
    protocol.limit_voltage_min = 0.0f;
    protocol.limit_voltage_max = 0.0f;
    protocol.limit_current_charge = 0.0f;
    protocol.limit_current_discharge = 0.0f;
    protocol.limit_capacity_charge = 0.0f;
    protocol.limit_capacity_discharge = 0.0f;
    
    // 시간 설정 초기화
    protocol.time_start = 0;
    protocol.time_operation = 0;
    
    // 패턴 데이터 초기화
    protocol.pattern_index = 0;
    protocol.pattern_data_type = PATTERN_DATA_STORE;
    protocol.pattern_control_type = PATTERN_CONTROL_CURRENT;
    protocol.pattern_time_unit = 0;
    protocol.pattern_index_value = 0;
    protocol.pattern_current_or_power = 0.0f;
    
    // 전역 안전 조건 초기화
    protocol.global_voltage_min = 0.0f;
    protocol.global_voltage_max = 0.0f;
    protocol.global_current_charge = 0.0f;
    protocol.global_current_discharge = 0.0f;
    protocol.global_capacity_charge = 0.0f;
    protocol.global_capacity_discharge = 0.0f;
    protocol.global_temp_max = 0.0f;
    protocol.global_temp_min = 0.0f;
    protocol.global_voltage_change = 0;
    protocol.global_voltage_change_time = 0;
    protocol.global_current_change = 0;
    protocol.global_current_change_time = 0;
}

/**
 * 메일박스의 데이터를 프로토콜 구조체에 저장하는 함수
 * @param mbox_num 메일박스 번호
 */
void SaveCommand(Uint16 mbox_num)
{
    Uint16 command_id;
    Uint8 data[8];
    struct MBOX *mbox_array = (struct MBOX *)&ECanaMboxes;
    
    // MSGID에서 명령 ID 추출
    command_id = mbox_array[mbox_num].MSGID.all & 0xFFFF;
    
    // MDL과 MDH에서 데이터 추출
    data[0] = mbox_array[mbox_num].MDL.byte.BYTE0;
    data[1] = mbox_array[mbox_num].MDL.byte.BYTE1;
    data[2] = mbox_array[mbox_num].MDL.byte.BYTE2;
    data[3] = mbox_array[mbox_num].MDL.byte.BYTE3;
    data[4] = mbox_array[mbox_num].MDH.byte.BYTE4;
    data[5] = mbox_array[mbox_num].MDH.byte.BYTE5;
    data[6] = mbox_array[mbox_num].MDH.byte.BYTE6;
    data[7] = mbox_array[mbox_num].MDH.byte.BYTE7;
    
    // 명령 데이터를 통합 구조체에 저장
    switch (command_id) {
        case 0x201:  // 스텝 명령 #1
            protocol.cmd_step = SWAP16(data, 1);
            protocol.cmd_mode = (MODE)data[3];
            protocol.cmd_current = SWAP_FLOAT(data, 4);
            break;
            
        case 0x202:  // 스텝 명령 #2
            protocol.cmd_voltage = SWAP_FLOAT(data, 0);
            protocol.cmd_power = SWAP_FLOAT(data, 4);
            break;
            
        case 0x203:  // 스텝 명령 #3
            protocol.time_start = SWAP32(data, 0);
            protocol.time_operation = SWAP32(data, 4);
            break;
            
        case 0x204:  // 스텝 명령 #4
            protocol.end_condition_capacity_wh = SWAP_FLOAT(data, 0);
            protocol.end_condition_capacity_ah = SWAP_FLOAT(data, 4);
            break;
            
        case 0x205:  // 스텝 명령 #5
            protocol.end_condition_voltage = SWAP_FLOAT(data, 0);
            protocol.end_condition_current = SWAP_FLOAT(data, 4);
            break;
            
        case 0x206:  // 스텝 명령 #6
            protocol.end_condition_cv_time = SWAP32(data, 0);
            protocol.limit_voltage_min = SWAP_FLOAT(data, 4);
            break;
            
        case 0x207:  // 스텝 명령 #7
            protocol.limit_voltage_max = SWAP_FLOAT(data, 0);
            protocol.limit_current_charge = SWAP_FLOAT(data, 4);
            break;
            
        case 0x208:  // 스텝 명령 #8
            protocol.limit_current_discharge = SWAP_FLOAT(data, 0);
            protocol.limit_capacity_charge = SWAP_FLOAT(data, 4);
            break;
            
        case 0x209:  // 스텝 명령 #9
            protocol.limit_capacity_discharge = SWAP_FLOAT(data, 0);
            break;
            
        case 0x250:  // Global Safety #1
            protocol.global_current_charge = SWAP_FLOAT(data, 0);
            protocol.global_voltage_min = SWAP_FLOAT(data, 4);
            break;
            
        case 0x251:  // Global Safety #2
            protocol.global_voltage_max = SWAP_FLOAT(data, 0);
            break;
            
        case 0x252:  // Global Safety #3
            protocol.global_capacity_charge = SWAP_FLOAT(data, 0);
            protocol.global_capacity_discharge = SWAP_FLOAT(data, 4);
            break;
            
        case 0x253:  // Global Safety #4
            protocol.global_voltage_change = SWAP16(data, 0);
            protocol.global_voltage_change_time = SWAP16(data, 2);
            protocol.global_current_change = SWAP16(data, 4);
            protocol.global_current_change_time = SWAP16(data, 6);
            break;
            
        case 0x254:  // Global Safety #5
            protocol.global_current_discharge = SWAP_FLOAT(data, 0);
            break;
            
        default:
            return;  // 알 수 없는 서브 ID
    }
}

// MBOX의 명령을 처리하는 함수
void ProcessCommand(Uint16 mbox_num)
{
    Uint8 data[8];
    struct MBOX *mbox_array = (struct MBOX *)&ECanaMboxes;
    
    // MDL과 MDH에서 데이터 추출
    data[0] = mbox_array[mbox_num].MDL.byte.BYTE0;
    data[1] = mbox_array[mbox_num].MDL.byte.BYTE1;
    data[2] = mbox_array[mbox_num].MDL.byte.BYTE2;
    data[3] = mbox_array[mbox_num].MDL.byte.BYTE3;
    data[4] = mbox_array[mbox_num].MDH.byte.BYTE4;
    data[5] = mbox_array[mbox_num].MDH.byte.BYTE5;
    data[6] = mbox_array[mbox_num].MDH.byte.BYTE6;
    data[7] = mbox_array[mbox_num].MDH.byte.BYTE7;
    
    // 명령어 처리 로직
    // 예: 두 번째 바이트로 명령어 유형 판단
    switch (data[1])
    {
        case CMD_START:  // 시작 명령
            // 작동 시작 로직 구현
            Buck_EN = 1;
            Run = 1;
            TransitionToRunning();
            break;
            
        case CMD_STOP:  // 정지 명령
            // 작동 정지 로직 구현
            Buck_EN = 0;
            Run = 0;
            TransitionToIdle();
            break;
            
        default:
            // 알 수 없는 명령
            break;
    }
}

/**
 * 보고 메시지 전송 함수
 * 인덱스에 따라 다른 형식의 상태 보고 메시지를 생성하여 전송
 * @param index 보고 메시지 인덱스 (0~7)
 */
void SendReport(Uint16 index) {
    Uint8 data[8];
    Uint32 id;
    
    // 메시지 데이터 초기화
    memset(data, 0, sizeof(data));
    
    // 기본 ID 설정 (0xXX0100~0xXX0107 또는 0xXX0110~0xXX0117)
    // 모듈 상태에 따라 기본 ID 결정
    if (module_state == STATE_RUNNING) {
        id = 0x00000100 | (protocol.channel << 8) | index;
    } else {
        id = 0x00000110 | (protocol.channel << 8) | index;
    }
    
    // 인덱스별 메시지 구성
    switch (index) {
        case 0: // 0xXX0100/0xXX0110 - Online Data #1
            data[0] = protocol.channel;                           // 채널 번호
            CanPutUint16(data, 1, protocol.cmd_step);           // 스텝 번호
            data[3] = protocol.status;                            // 동작 상태
            data[4] = protocol.mode;                              // 운전 모드
            data[5] = protocol.state_bits.all;                    // 상태 비트
            CanPutUint16(data, 6, protocol.event_code);         // 알람/이벤트 코드
            break;
            
        case 1: // 0xXX0101/0xXX0111 - Online Data #2
            CanPutUint16(data, 0, (Uint16)protocol.fb_t1_temp); // T1 온도 (16비트 정수)
            CanPutUint16(data, 2, (Uint16)protocol.fb_t2_temp); // T2 온도 (16비트 정수)
            CanPutFloat32(data, 4, protocol.fb_voltage);        // 전압 (float)
            break;
            
        case 2: // 0xXX0102/0xXX0112 - Online Data #3
            // 전류 (float)
            CanPutFloat32(data, 0, protocol.fb_current);
            // CV 타임 (32비트 정수, 1 = 10ms)
            CanPutUint32(data, 4, protocol.fb_cv_time);
            break;
            
        case 3: // 0xXX0103/0xXX0113 - Online Data #4
            // 충전 Ah (float)
            CanPutFloat32(data, 0, protocol.fb_charge_ah);
            // 방전 Ah (float)
            CanPutFloat32(data, 4, protocol.fb_discharge_ah);
            break;
            
        case 4: // 0xXX0104/0xXX0114 - Online Data #5
            // 충전 Wh (float)
            CanPutFloat32(data, 0, protocol.fb_charge_wh);
            // 방전 Wh (float)
            CanPutFloat32(data, 4, protocol.fb_discharge_wh);
            break;
            
        case 5: // 0xXX0105/0xXX0115 - Online Data #6
            // 운전 시간 (32비트 정수, 1 = 10ms)
            CanPutUint32(data, 0, protocol.fb_operation_time);
            // Reserved
            break;
            
        case 6: // 0xXX0106/0xXX0116 - Online Data #7
            // PWM 오류 정보
            CanPutUint16(data, 0, protocol.pwm_hw_error);
            CanPutUint16(data, 2, protocol.pwm_sw_error1);
            CanPutUint16(data, 4, protocol.pwm_sw_error2);
            CanPutUint16(data, 6, protocol.pwm_sw_warning);
            break;
            
        case 7: // 0xXX0107/0xXX0117 - Online Data #8
            // DC/DC 오류 정보
            CanPutUint16(data, 0, protocol.dcdc_hw_error);
            CanPutUint16(data, 2, protocol.dcdc_sw_error1);
            CanPutUint16(data, 4, protocol.dcdc_sw_error2);
            CanPutUint16(data, 6, protocol.dcdc_sw_warning);
            break;
            
        default:
            // 알 수 없는 인덱스
            return;
    }
    
    // 메시지 전송
    SendCANMessage(id, data);
}

/**
 * 종료 보고 전송 함수
 * 모듈 종료 시 종합적인 상태 정보를 보고하는 CAN 메시지 전송
 */
void SendEndReport(void) {
    Uint8 data[8];
    Uint16 i;
    Uint32 id;
    
    // 총 6개 메시지 전송 (0xXX0130~0xXX0135)
    for (i = 0; i < 6; i++) {
        // 메시지 데이터 초기화
        memset(data, 0, sizeof(data));
        
        // 기본 ID 설정 (0xXX0130 ~ 0xXX0135)
        id = 0x00000130 | ((protocol.channel & 0xFF) << 8) | i;
        
        switch (i) {
            case 0: // 0xXX0130 - Step End Data #1
                data[0] = protocol.channel & 0xFF;                 // 채널 번호
                CanPutUint16(data, 1, protocol.cmd_step);        // 스텝 번호
                data[3] = protocol.status;                         // 동작 상태
                data[4] = protocol.mode;                           // 운전 모드
                CanPutUint16(data, 5, protocol.event_code);      // 알람/이벤트 코드
                data[7] = protocol.pattern_index;                  // Pattern Index No
                break;
                
            case 1: // 0xXX0131 - Step End Data #2
                CanPutUint16(data, 0, protocol.fb_t1_temp * 10); // T1 온도 (x10 스케일)
                CanPutUint16(data, 2, protocol.fb_t2_temp * 10); // T2 온도 (x10 스케일)
                CanPutFloat32(data, 4, protocol.fb_voltage);      // 전압
                break;
                
            case 2: // 0xXX0132 - Step End Data #3
                CanPutFloat32(data, 0, protocol.fb_current);      // 전류
                CanPutUint32(data, 4, protocol.fb_cv_time * 100); // CV 타임 (10ms 단위)
                break;
                
            case 3: // 0xXX0133 - Step End Data #4
                CanPutFloat32(data, 0, protocol.fb_charge_ah);    // 충전 Ah
                CanPutFloat32(data, 4, protocol.fb_discharge_ah); // 방전 Ah
                break;
                
            case 4: // 0xXX0134 - Step End Data #5
                CanPutFloat32(data, 0, protocol.fb_charge_wh);    // 충전 Wh
                CanPutFloat32(data, 4, protocol.fb_discharge_wh); // 방전 Wh
                break;
                
            case 5: // 0xXX0135 - Step End Data #6
                CanPutUint32(data, 0, protocol.fb_operation_time * 100); // 운전 시간 (10ms 단위)
                CanPutUint32(data, 4, protocol.pattern_index);           // Pattern Index
                break;
        }
        
        // CAN 메시지 전송
        SendCANMessage(id, data);
    }
}

// 주기적 태스크 함수
void PeriodicTask(void)
{
    static Uint16 can_report_timer = 0;
    static Uint16 report_index = 0;
    CAN_MESSAGE can_msg;
    Uint16 threshold;  // 변수 선언을 함수 시작 부분으로 이동
    
    // 피드백 값 업데이트 (전압, 전류, 온도 등)
    UpdateFeedbackValues();
    
    // 운전 시간 업데이트
    UpdateOperationTime();
    
    // CV 시간 업데이트
    UpdateCVTime();
    
    // 용량 값 업데이트 (Ah, Wh)
    UpdateCapacityValues();
    
    // HeartBit 타임아웃 체크 추가
    CheckHeartBitTimeout();
    
    // 보고 주기 카운터 증가
    can_report_timer++;
    
    // 현재 상태에 따라 카운터 임계값 설정
    threshold = (module_state == STATE_RUNNING) ? 1 : 10;  // 운전 중 1(10ms), 대기 중 10(100ms)
    
    // 임계값에 도달하면 메시지 전송
    if (can_report_timer >= threshold) {
        SendReport(report_index);
        
        // 다음 보고 인덱스로 이동
        report_index++;
        if (report_index >= 8) {
            report_index = 0;
        }
        
        // 카운터 리셋
        can_report_timer = 0;
    }
}

// 피드백 값 업데이트 함수
void UpdateFeedbackValues(void) {
    extern float32 Vo;      // 전압 (V)
    extern float32 Io_avg;  // 전류 (A)
    extern float32 t1_value; // 온도1 (°C)
    extern float32 In_Temp;  // 35kW 프로그램의 온도 값 (°C)
    
    // 프로토콜 구조체의 피드백 값들은 float32 타입으로 정의되어 있음
    // 단위 변환: V -> mV, A -> mA, °C -> 0.1°C
    protocol.fb_voltage = Vo * 1000.0f;       // V -> mV 변환
    protocol.fb_current = Io_avg * 1000.0f;   // A -> mA 변환
    
    // 35kW 프로그램의 In_Temp 값을 protocol.fb_t1_temp에 반영
    protocol.fb_t1_temp = In_Temp * 10.0f;    // °C -> 0.1°C 변환
}

// 운전 시간 업데이트 함수
void UpdateOperationTime(void) {
    static Uint16 tick_counter = 0;
    
    // 운전 중일 때만 시간 증가
    if (protocol.status == OPERATING) {
        tick_counter++;
        
        // 1초마다 운전 시간 증가 (10ms 단위)
        if (tick_counter >= 100) { // 10ms * 100 = 1초
            tick_counter = 0;
            protocol.fb_operation_time++;  // 10ms 단위로 저장
        }
    }
}

// CV 시간 업데이트 함수
void UpdateCVTime(void) {
    static Uint16 tick_counter = 0;
    
    // 운전 중일 때만 시간 업데이트
    if (protocol.status == OPERATING) {
        tick_counter++;
        
        // 10ms마다 CV 시간 업데이트
        if (tick_counter >= 1) {  // 10ms = 1 tick
            tick_counter = 0;
            
            // CV 모드 체크 (CC 모드이거나 CV 모드일 때)
            if (protocol.cmd_mode == MODE_CC || protocol.cmd_mode == MODE_CV) {
                // 전압이 설정값에 도달했는지 확인 (단위 변환: mV -> V)
                if (protocol.fb_voltage >= protocol.cmd_voltage) {
                    protocol.fb_cv_time++;
                }
            }
        }
    }
}

// 용량 값 업데이트 함수
void UpdateCapacityValues(void) {
    static Uint16 tick_counter = 0;
    extern float32 Io_avg;  // 전류 (A)
    float32 delta_capacity;  // 변수 선언을 함수 시작 부분으로 이동
    
    // 운전 중일 때만 용량 업데이트
    if (protocol.status == OPERATING) {
        tick_counter++;
        
        // 1초마다 용량 업데이트 (10ms 단위)
        if (tick_counter >= 100) {
            tick_counter = 0;
            
            // 전류값을 Ah로 변환 (A -> mAh)
            delta_capacity = (Io_avg * 1000.0f) / 3600.0f;  // A -> mA, 1시간 = 3600초
            
            // 충방전 용량 업데이트
            if (delta_capacity > 0) {
                protocol.fb_charge_ah += delta_capacity;  // mAh 단위로 누적
            } else {
                protocol.fb_discharge_ah += -delta_capacity;  // mAh 단위로 누적
            }
        }
    }
}

// 운전 상태로 전환 함수
void TransitionToRunning(void) {
    extern UNIONFLOAT UI_Iout_command; // 전류 지령 값 (A 단위)
    extern float32 V_com; // 전압 지령 값 (V 단위)
    extern float32 Power; // 파워 지령 값 (W 단위)
    
    // 상태 변경
    module_state = STATE_RUNNING;
    protocol.status = OPERATING;
    
    // 운전 시간 초기화 (필요한 경우)
    protocol.fb_operation_time = 0;
    
    // CV 타임 초기화 (필요한 경우)
    protocol.fb_cv_time = 0;
    
    // 패턴 인덱스 초기화 (패턴 모드인 경우)
    if (protocol.mode == MODE_PATTERN) {
        protocol.pattern_index = 0;
    }
    
    // 운전 모드 설정
    protocol.mode = protocol.cmd_mode;
    
    // 단위 변환: mV -> V, mA -> A, mW -> W
    // 전류 지령 설정 (외부 변수에 저장)
    UI_Iout_command.fValue = protocol.cmd_current / 1000.0f; // mA -> A 변환
    
    // 전압 지령 설정 (필요한 경우)
    V_com = protocol.cmd_voltage / 1000.0f; // mV -> V 변환
    
    // 파워 지령 설정 (필요한 경우)
    Power = protocol.cmd_power / 1000.0f; // mW -> W 변환
}

// 대기 상태로 전환 함수
void TransitionToIdle(void) {
    extern UNIONFLOAT UI_Iout_command; // 전류 지령 값
    
    // 전류 지령 0으로 설정
    UI_Iout_command.fValue = 0.0f;
    
    // 상태 변경
    module_state = STATE_IDLE;
    protocol.status = READY;
    
    // 종료 보고 전송
    SendEndReport();
}

// CAN 메시지 처리 함수
void ProcessCanMessage(const CAN_MESSAGE* msg)
{
    Uint8 data[8];
    Uint16 i;
    Uint16 command_id;
    
    // 데이터 복사
    for(i = 0; i < 8; i++)
    {
        data[i] = msg->data[i];
    }
    
    // 명령 ID 추출
    command_id = (Uint16)msg->id & 0xFFFF;
    
    // 210번 제어 명령인지 확인 (0x0210)
    if((command_id & 0xFFFF) == 0x0210)
    {
        // 210번 제어 명령 처리
        ProcessCommand(command_id);
    }
    // 210번을 제외한 모든 200번대 명령 처리 (200~209, 250~254, 260~261)
    else if((command_id & 0xFF00) == 0x0200)
    {
        // 명령 저장 및 ACK 전송
        SaveCommand(command_id);
    }
    // Heart Bit 메시지(0x360)인지 확인
    else if(msg->id == 0x360) {
        
        // 타임아웃 카운터 리셋
        can_360_timeout_counter = 0;
        can_360_timeout_flag = 0;
        
        return; // Heart Bit 메시지 처리 완료
    }
}

// Heart Bit 타임아웃 체크 함수
void CheckHeartBitTimeout(void)
{
    // 1ms마다 호출된다고 가정
    can_360_timeout_counter++;
    
    // 1초(1000ms) 이상 Heart Bit 메시지가 수신되지 않으면 타임아웃으로 처리
    if(can_360_timeout_counter >= 1000)
    {
        can_360_timeout_flag = 1;
        
        // 타임아웃 발생 시 필요한 처리 추가
        // 예: 오류 상태로 전환, 알람 발생 등
        if(module_state == STATE_RUNNING)
        {
            // 실행 중인 경우 안전을 위해 IDLE 상태로 전환
            TransitionToIdle();
            
            // 오류 상태 설정
            protocol.status = PAUSE;
        }
    }
}

void SendCANMessage(Uint32 id, Uint8 *data)
{
    // 빈 함수 - 구현 생략
}
