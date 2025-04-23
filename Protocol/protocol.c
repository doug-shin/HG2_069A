// protocol.c - 프로토콜 처리 모듈
// CAN 통신을 통한 충/방전 명령 처리 및 상태 보고 기능 제공

#include "protocol.h"
#include "F2806x_Cla_typedefs.h"
#include "F2806x_Device.h"
#include <string.h>
#include "sicDCDC35kw.h" // UNIONFLOAT 타입 정의를 위해 추가
#define MODULE_CHANNEL 0x01

// 전역 변수
PROTOCOL_INTEGRATED protocol;  // 프로토콜 구조체
STATE module_state;           // 모듈 상태 (STATE_IDLE or STATE_RUNNING)

// Heart Bit 타임아웃 관련 변수 정의
Uint16 can_360_timeout_counter = 0;  // Heart Bit 타임아웃 카운터
Uint16 can_360_timeout_flag = 0;     // Heart Bit 타임아웃 플래그

// 엔디안 변환 함수 - 16비트
Uint16 Swap16(Uint16 value) {
    return ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
}

// 엔디안 변환 함수 - 32비트
Uint32 Swap32(Uint32 value) {
    Uint32 result;
    result = ((value & 0xFF) << 24) | 
             ((value & 0xFF00) << 8) | 
             ((value & 0xFF0000) >> 8) | 
             ((value & 0xFF000000) >> 24);
    return result;
}

// 엔디안 변환 함수 - 부동소수점
float32 SwapFloat(float32 value) {
    union {
        float32 f;
        Uint32 i;
    } src;
    union {
        float32 f;
        Uint32 i;
    } dst;
    
    src.f = value;
    dst.i = Swap32(src.i);
    
    return dst.f;
}

// 엔디안 변환 함수 검증
void TestEndianConversion(void) {
    // 테스트 케이스 1: 양수
    float32 test1 = 123.456f;
    float32 swapped1 = SwapFloat(test1);
    float32 restored1 = SwapFloat(swapped1);
    
    // 테스트 케이스 2: 음수
    float32 test2 = -123.456f;
    float32 swapped2 = SwapFloat(test2);
    float32 restored2 = SwapFloat(swapped2);
    
    // 테스트 케이스 3: 0
    float32 test3 = 0.0f;
    float32 swapped3 = SwapFloat(test3);
    float32 restored3 = SwapFloat(swapped3);
    
    // 테스트 케이스 4: 매우 큰 수
    float32 test4 = 1.0e38f;
    float32 swapped4 = SwapFloat(test4);
    float32 restored4 = SwapFloat(swapped4);
    
    // 테스트 케이스 5: 매우 작은 수
    float32 test5 = 1.0e-38f;
    float32 swapped5 = SwapFloat(test5);
    float32 restored5 = SwapFloat(swapped5);
    
    // 검증: 원래 값과 두 번 스왑 후의 값이 같아야 함
    if (test1 != restored1 || 
        test2 != restored2 || 
        test3 != restored3 || 
        test4 != restored4 || 
        test5 != restored5) {
        // 에러 처리
        while(1); // 디버깅을 위한 무한 루프
    }
    
    // 검증: 스왑된 값이 원래 값과 다르야 함
    if (test1 == swapped1 || 
        test2 == swapped2 || 
        test3 == swapped3 || 
        test4 == swapped4 || 
        test5 == swapped5) {
        // 에러 처리
        while(1); // 디버깅을 위한 무한 루프
    }
}

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

// 명령 데이터 저장 함수
void SaveCommand(Uint16 command_id, Uint8* data) {
    // 명령 ID 파싱 (WXYZ 형식)
    Uint8 channel = (command_id >> 8) & 0xFF;  // 채널 번호 (W)
    Uint8 sub_id = command_id & 0xFF;           // 서브 ID (Z)
    
    // 채널 번호 확인
    if (channel != protocol.channel) {
        return;  // 다른 채널의 메시지는 무시
    }
    
    // 명령 데이터를 통합 구조체에 저장
    switch (sub_id) {
        case 0x01:  // 스텝 명령 #1
            protocol.cmd_step = (data[1] << 8) | data[2];
            protocol.cmd_mode = (MODE)data[3];  // 타입 캐스팅 추가
            protocol.cmd_current = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x02:  // 스텝 명령 #2
            protocol.cmd_voltage = SwapFloat(*(float*)&data[0]);
            protocol.cmd_power = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x03:  // 스텝 명령 #3
            protocol.time_start = Swap32(*(Uint32*)&data[0]);
            protocol.time_operation = Swap32(*(Uint32*)&data[4]);
            break;
            
        case 0x04:  // 스텝 명령 #4
            protocol.end_condition_capacity_wh = SwapFloat(*(float*)&data[0]);
            protocol.end_condition_capacity_ah = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x05:  // 스텝 명령 #5
            protocol.end_condition_voltage = SwapFloat(*(float*)&data[0]);
            protocol.end_condition_current = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x06:  // 스텝 명령 #6
            protocol.end_condition_cv_time = Swap32(*(Uint32*)&data[0]);
            protocol.limit_voltage_min = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x07:  // 스텝 명령 #7
            protocol.limit_voltage_max = SwapFloat(*(float*)&data[0]);
            protocol.limit_current_charge = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x08:  // 스텝 명령 #8
            protocol.limit_current_discharge = SwapFloat(*(float*)&data[0]);
            protocol.limit_capacity_charge = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x09:  // 스텝 명령 #9
            protocol.limit_capacity_discharge = SwapFloat(*(float*)&data[0]);
            break;
            
        case 0x50:  // Global Safety #1
            protocol.global_current_charge = SwapFloat(*(float*)&data[0]);
            protocol.global_voltage_min = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x51:  // Global Safety #2
            protocol.global_voltage_max = SwapFloat(*(float*)&data[0]);
            break;
            
        case 0x52:  // Global Safety #3
            protocol.global_capacity_charge = SwapFloat(*(float*)&data[0]);
            protocol.global_capacity_discharge = SwapFloat(*(float*)&data[4]);
            break;
            
        case 0x53:  // Global Safety #4
            protocol.global_voltage_change = Swap16(*(Uint16*)&data[0]);
            protocol.global_voltage_change_time = Swap16(*(Uint16*)&data[2]);
            protocol.global_current_change = Swap16(*(Uint16*)&data[4]);
            protocol.global_current_change_time = Swap16(*(Uint16*)&data[6]);
            break;
            
        case 0x54:  // Global Safety #5
            protocol.global_current_discharge = SwapFloat(*(float*)&data[0]);
            break;
            
        default:
            return;  // 알 수 없는 서브 ID
    }
}

/**
 * @brief 제어 명령을 처리하는 함수
 * 
 * @param data 명령 데이터
 */
void ProcessCommand(Uint8* data) {
    Uint8 command_code = data[1];  // 명령 코드
    Uint16 event_code = (data[2] << 8) | data[3];  // 이벤트 코드
    
    // 이벤트 코드가 있으면 프로토콜에 저장
    if (event_code != 0) {
        protocol.event_code = event_code;
    }
    
    // 명령 코드에 따른 처리
    switch (command_code) {
        case CMD_START:  // 시작 명령
            TransitionToRunning();
            break;
            
        case CMD_STOP:  // 정지 명령
            TransitionToIdle();
            break;
            
        case CMD_PAUSE:  // 일시정지 명령
            TransitionToIdle();
            break;
            
        case CMD_EMERGENCY:  // 비상정지 명령
            TransitionToIdle();
            break;
            
        case CMD_PWR_START:  // PWR Start 명령
            protocol.state_bits.bit.pwr_status = 1;
            break;
            
        case CMD_PWR_STOP:  // PWR Stop 명령
            protocol.state_bits.bit.pwr_status = 0;
            break;
            
        case CMD_ALARM_RESET:  // 알람 리셋 명령
            protocol.status = READY;
            protocol.event_code = 0;
            break;
            
        default:
            // 알 수 없는 명령 코드
            break;
    }
}

/**
 * @brief 일반 명령에 대한 ACK 메시지를 전송하는 함수
 * 
 * @param command_id 원본 명령 ID (WXYZ 형식)
 * @return Uint16 성공(0) 또는 실패(에러 코드)
 */
Uint16 SendAck(Uint16 command_id) {
    Uint8 ack_data[8] = {0};  // ACK 메시지 데이터
    Uint32 ack_id;            // ACK 메시지 ID
    
    // 명령 ID 파싱 (WXYZ 형식)
    Uint8 sub_id = command_id & 0xFF;           // 서브 ID (Z)
    
    // ACK 메시지 ID 설정 (22YZ 형식)
    // Y: 0x50-0x54는 0x5로, 나머지는 0x0
    // Z: 0x50-0x54는 하위 4비트만 사용, 나머지는 그대로 사용
    Uint8 ack_y = (sub_id >= 0x50) ? 0x5 : 0x0;
    Uint8 ack_z = (sub_id >= 0x50) ? (sub_id & 0x0F) : sub_id;
    ack_id = 0x22000000 | ((Uint32)(protocol.channel & 0xFF) << 16) | (ack_y << 8) | ack_z;
    
    // ACK 메시지 전송
    return SendCANMessage(ack_id, ack_data);
}

/**
 * @brief 210번 제어 명령에 대한 ACK 메시지를 전송하는 함수
 * 
 * @param data 원본 명령 데이터
 * @return Uint16 성공(0) 또는 실패(에러 코드)
 */
Uint16 SendCommandAck(Uint8* data) {
    Uint8 ack_data[8] = {0};  // ACK 메시지 데이터
    Uint32 ack_id;            // ACK 메시지 ID
    
    // ACK 메시지 ID 설정 (2210 형식)
    ack_id = 0x22000000 | ((Uint32)(protocol.channel & 0xFF) << 16) | 0x10;
    
    // ACK 메시지 데이터 설정
    ack_data[0] = protocol.channel;           // 채널 번호
    ack_data[1] = data[1];                   // 명령 코드
    ack_data[2] = data[2];                   // 이벤트 코드 (상위 바이트)
    ack_data[3] = data[3];                   // 이벤트 코드 (하위 바이트)
    ack_data[4] = 0x01;                      // 응답 ACK (0x01: OK)
    ack_data[5] = 0x00;                      // 이벤트 코드 (0x00: 정상)
    
    // ACK 메시지 전송
    return SendCANMessage(ack_id, ack_data);
}

/**
 * @brief CAN 메시지를 전송하는 함수 (35kW Master 프로그램용)
 * 
 * @param id CAN 메시지 ID (29비트 확장 ID)
 * @param data 전송할 데이터 (8바이트)
 * @return Uint16 성공(0) 또는 실패(에러 코드)
 */
Uint16 SendCANMessage(Uint32 id, const Uint8* data) {
    // 매개변수 검증
    if (data == NULL) {
        return 2; // 잘못된 매개변수
    }
    
    // 메일박스 상태 확인
    if (ECanaRegs.CANTRS.bit.TRS0) {
        return 1; // 이전 전송이 아직 진행 중
    }
    
    // 메시지 ID 설정 (29비트 확장 ID)
    ECanaMboxes.MBOX0.MSGID.all = id & 0x1FFFFFFF;
    
    // 메시지 제어 설정
    ECanaMboxes.MBOX0.MSGCTRL.all = 0x0000;  // 기본값
    ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;   // 8바이트 데이터 길이
    
    // 데이터 복사 (32비트 단위로 복사)
    // 하위 32비트 (0-3바이트)
    ECanaMboxes.MBOX0.MDL.all = ((Uint32)data[3] << 24) | ((Uint32)data[2] << 16) | ((Uint32)data[1] << 8) | data[0];
    
    // 상위 32비트 (4-7바이트)
    ECanaMboxes.MBOX0.MDH.all = ((Uint32)data[7] << 24) | ((Uint32)data[6] << 16) | ((Uint32)data[5] << 8) | data[4];
    
    // 전송 요청
    ECanaRegs.CANTRS.bit.TRS0 = 1;
    
    return 0; // 성공
}

// 보고 메시지 전송 함수
void SendReport(Uint16 index) {
    Uint8 message[8];
    Uint32 id;
    
    // 메시지 데이터 초기화
    memset(message, 0, sizeof(message));
    
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
            message[0] = protocol.channel;           // 채널 번호
            *(Uint16*)&message[1] = Swap16(protocol.cmd_step); // 스텝 번호
            message[3] = protocol.status;            // 동작 상태
            message[4] = protocol.mode;              // 운전 모드
            message[5] = protocol.state_bits.all;    // 상태 비트
            *(Uint16*)&message[6] = Swap16(protocol.event_code); // 알람/이벤트 코드
            break;
            
        case 1: // 0xXX0101/0xXX0111 - Online Data #2
            // T1 온도 (16비트 정수)
            *(Uint16*)&message[0] = Swap16((Uint16)protocol.fb_t1_temp);
            // T2 온도 (16비트 정수)
            *(Uint16*)&message[2] = Swap16((Uint16)protocol.fb_t2_temp);
            // 전압 (float)
            *(float32*)&message[4] = SwapFloat(protocol.fb_voltage);
            break;
            
        case 2: // 0xXX0102/0xXX0112 - Online Data #3
            // 전류 (float)
            *(float32*)&message[0] = SwapFloat(protocol.fb_current);
            // CV 타임 (32비트 정수, 1 = 10ms)
            *(Uint32*)&message[4] = Swap32(protocol.fb_cv_time);
            break;
            
        case 3: // 0xXX0103/0xXX0113 - Online Data #4
            // 충전 Ah (float)
            *(float32*)&message[0] = SwapFloat(protocol.fb_charge_ah);
            // 방전 Ah (float)
            *(float32*)&message[4] = SwapFloat(protocol.fb_discharge_ah);
            break;
            
        case 4: // 0xXX0104/0xXX0114 - Online Data #5
            // 충전 Wh (float)
            *(float32*)&message[0] = SwapFloat(protocol.fb_charge_wh);
            // 방전 Wh (float)
            *(float32*)&message[4] = SwapFloat(protocol.fb_discharge_wh);
            break;
            
        case 5: // 0xXX0105/0xXX0115 - Online Data #6
            // 운전 시간 (32비트 정수, 1 = 10ms)
            *(Uint32*)&message[0] = Swap32(protocol.fb_operation_time);
            // Reserved
            break;
            
        case 6: // 0xXX0106/0xXX0116 - Online Data #7
            // PWM 오류 정보
            *(Uint16*)&message[0] = Swap16(protocol.pwm_hw_error);
            *(Uint16*)&message[2] = Swap16(protocol.pwm_sw_error1);
            *(Uint16*)&message[4] = Swap16(protocol.pwm_sw_error2);
            *(Uint16*)&message[6] = Swap16(protocol.pwm_sw_warning);
            break;
            
        case 7: // 0xXX0107/0xXX0117 - Online Data #8
            // DC/DC 오류 정보
            *(Uint16*)&message[0] = Swap16(protocol.dcdc_hw_error);
            *(Uint16*)&message[2] = Swap16(protocol.dcdc_sw_error1);
            *(Uint16*)&message[4] = Swap16(protocol.dcdc_sw_error2);
            *(Uint16*)&message[6] = Swap16(protocol.dcdc_sw_warning);
            break;
            
        default:
            // 알 수 없는 인덱스
            return;
    }
    
    // 메시지 전송
    SendCANMessage(id, message);
}


// 종료 보고 전송 함수
void SendEndReport(void) {
    Uint8 message[8];
    Uint16 i;
    Uint32 id;
    
    // 메시지 ID 구성: 0xXX0130~0xXX0135 (XX는 채널 번호)
    // 총 6개 메시지 전송
    for (i = 0; i < 6; i++) {
        // 메시지 데이터 초기화
        memset(message, 0, sizeof(message));
        
        // 기본 ID 설정 (0xXX0130 ~ 0xXX0135)
        id = 0x00000130 | ((protocol.channel & 0xFF) << 8) | i;
        
        switch (i) {
            case 0: // 0xXX0130 - Step End Data #1
                message[0] = protocol.channel & 0xFF;    // 채널 번호
                *(Uint16*)&message[1] = Swap16(protocol.cmd_step); // 스텝 번호
                message[3] = protocol.status;            // 동작 상태
                message[4] = protocol.mode;              // 운전 모드
                *(Uint16*)&message[5] = Swap16(protocol.event_code); // 알람/이벤트 코드
                message[7] = protocol.pattern_index;     // Pattern Index No
                break;
                
            case 1: // 0xXX0131 - Step End Data #2
                *(Uint16*)&message[0] = Swap16(protocol.fb_t1_temp * 10); // T1 온도 (x10 스케일)
                *(Uint16*)&message[2] = Swap16(protocol.fb_t2_temp * 10); // T2 온도 (x10 스케일)
                *(float*)&message[4] = SwapFloat(protocol.fb_voltage); // 전압
                break;
                
            case 2: // 0xXX0132 - Step End Data #3
                *(float*)&message[0] = SwapFloat(protocol.fb_current); // 전류
                *(Uint32*)&message[4] = Swap32(protocol.fb_cv_time * 100); // CV 타임 (10ms 단위)
                break;
                
            case 3: // 0xXX0133 - Step End Data #4
                *(float*)&message[0] = SwapFloat(protocol.fb_charge_ah); // 충전 Ah
                *(float*)&message[4] = SwapFloat(protocol.fb_discharge_ah); // 방전 Ah
                break;
                
            case 4: // 0xXX0134 - Step End Data #5
                *(float*)&message[0] = SwapFloat(protocol.fb_charge_wh); // 충전 Wh
                *(float*)&message[4] = SwapFloat(protocol.fb_discharge_wh); // 방전 Wh
                break;
                
            case 5: // 0xXX0135 - Step End Data #6
                *(Uint32*)&message[0] = Swap32(protocol.fb_operation_time * 100); // 운전 시간 (10ms 단위)
                *(Uint32*)&message[4] = Swap32(protocol.pattern_index); // Pattern Index
                break;
        }
        
        // CAN 메시지 전송
        SendCANMessage(id, message);
    }
}

// 주기적 태스크 함수
void PeriodicTask(void)
{
    static Uint16 can_report_timer = 0;
    static Uint16 report_index = 0;
    CAN_MESSAGE can_msg;
    
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
    
    // CAN 메시지 큐 처리
    while (CanQueuePop(&can_msg) == 0) {
        // 큐에서 꺼낸 메시지 처리
        ProcessCanMessage(&can_msg);
    }
    
    // 보고 주기 카운터 증가
    can_report_timer++;
    
    // 현재 상태에 따라 카운터 임계값 설정
    Uint16 threshold = (module_state == STATE_RUNNING) ? 1 : 10;  // 운전 중 1(10ms), 대기 중 10(100ms)
    
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

// CAN 메시지 큐 전역 변수
CAN_QUEUE can_rx_queue;

// CAN 큐 초기화 함수
void InitCanQueue(void)
{
    // 큐 초기화
    can_rx_queue.head = 0;
    can_rx_queue.tail = 0;
    can_rx_queue.count = 0;
}

// CAN 메시지를 큐에 추가하는 함수
Uint16 CanQueuePush(const CAN_MESSAGE* msg)
{
    Uint16 i;
    
    // 큐가 가득 찼는지 확인
    if(can_rx_queue.count >= CAN_RX_QUEUE_SIZE)
    {
        return 1; // 큐가 가득 참
    }
    
    // 메시지 복사 (C89 스타일)
    can_rx_queue.messages[can_rx_queue.tail].id = msg->id;
    can_rx_queue.messages[can_rx_queue.tail].dlc = msg->dlc;
    
    for(i = 0; i < 8; i++)
    {
        can_rx_queue.messages[can_rx_queue.tail].data[i] = msg->data[i];
    }
    
    // 테일 인덱스 업데이트
    can_rx_queue.tail = (can_rx_queue.tail + 1) % CAN_RX_QUEUE_SIZE;
    can_rx_queue.count++;
    
    return 0; // 성공
}

// CAN 메시지를 큐에서 가져오는 함수
Uint16 CanQueuePop(CAN_MESSAGE* msg)
{
    Uint16 i;
    
    // 큐가 비어있는지 확인
    if(can_rx_queue.count == 0)
    {
        return 1; // 큐가 비어있음
    }
    
    // 메시지 복사 (C89 스타일)
    msg->id = can_rx_queue.messages[can_rx_queue.head].id;
    msg->dlc = can_rx_queue.messages[can_rx_queue.head].dlc;
    
    for(i = 0; i < 8; i++)
    {
        msg->data[i] = can_rx_queue.messages[can_rx_queue.head].data[i];
    }
    
    // 헤드 인덱스 업데이트
    can_rx_queue.head = (can_rx_queue.head + 1) % CAN_RX_QUEUE_SIZE;
    can_rx_queue.count--;
    
    return 0; // 성공
}

/**
 * @brief eCAN 메일박스에서 확장 ID CAN 메시지를 파싱하는 함수
 * @param mailbox_num 메일박스 번호 (0-31)
 * @param msg 파싱된 데이터를 저장할 CAN_MESSAGE 구조체 포인터
 * @return Uint16 성공(0) 또는 실패(1)
 */
Uint16 ParseExtendedCANMessage(Uint16 mailbox_num, CAN_MESSAGE* msg)
{
    // 매개변수 검증
    if (msg == NULL || mailbox_num > 31) {
        return 1; // 에러: 잘못된 매개변수
    }
    
    volatile struct MBOX* mbox; // 메일박스 포인터
    
    // 메일박스 번호에 따라 적절한 메일박스 선택
    switch(mailbox_num) {
        case 0:
            mbox = &ECanaMboxes.MBOX0;
            break;
        case 1:
            mbox = &ECanaMboxes.MBOX1;
            break;
        case 2:
            mbox = &ECanaMboxes.MBOX2;
            break;
        case 3:
            mbox = &ECanaMboxes.MBOX3;
            break;
        default:
            return 1; // 에러: 잘못된 메일박스 번호
    }
    
    // 확장 ID가 활성화 되어 있는지 확인
    if (mbox->MSGID.bit.IDE == 1) {
        // 확장 ID (29비트) 추출
        msg->id = (((Uint32)mbox->MSGID.bit.EXTMSGID_H) << 16) | 
                  (Uint32)mbox->MSGID.bit.EXTMSGID_L;
    } else {
        // 표준 ID (11비트) 추출
        msg->id = (Uint32)mbox->MSGID.bit.STDMSGID;
    }
    
    // 데이터 길이 코드 (DLC) 추출
    msg->dlc = (Uint8)mbox->MSGCTRL.bit.DLC;
    
    // 메시지 데이터 추출 (32비트 레지스터에서 바이트 단위로 추출)
    msg->data[0] = (Uint8)(mbox->MDL.byte.BYTE0);
    msg->data[1] = (Uint8)(mbox->MDL.byte.BYTE1);
    msg->data[2] = (Uint8)(mbox->MDL.byte.BYTE2);
    msg->data[3] = (Uint8)(mbox->MDL.byte.BYTE3);
    msg->data[4] = (Uint8)(mbox->MDH.byte.BYTE4);
    msg->data[5] = (Uint8)(mbox->MDH.byte.BYTE5);
    msg->data[6] = (Uint8)(mbox->MDH.byte.BYTE6);
    msg->data[7] = (Uint8)(mbox->MDH.byte.BYTE7);
    
    return 0; // 성공
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
        ProcessCommand(data);
        SendCommandAck(data);  // 제어 명령에 대한 ACK 전송
    }
    // 210번을 제외한 모든 200번대 명령 처리 (200~209, 250~254, 260~261)
    else if((command_id & 0xFF00) == 0x0200)
    {
        // 명령 저장 및 ACK 전송
        SaveCommand(command_id, data);
        SendAck(command_id);
    }
    // Heart Bit 메시지(0x360)인지 확인
    else if(msg->id == 0x360) {
        
        // 타임아웃 카운터 리셋
        can_360_timeout_counter = 0;
        can_360_timeout_flag = 0;
        
        return; // Heart Bit 메시지 처리 완료
    }
    
}

// CAN 인터럽트 서비스 루틴
interrupt void Protocol_ECAN0INTA_ISR(void)
{
    Uint32 mailbox_nr;
    Uint16 heart_bit_count = 0;
    
    // 어떤 메일박스에서 인터럽트가 발생했는지 확인
    CAN_MESSAGE rx_msg;
    mailbox_nr = ECanaRegs.CANGIF1.bit.MIV1;
    
    // 메시지 수신
    switch(mailbox_nr) {            
        case 2:  // 200번대 명령
            rx_msg.id = ECanaMboxes.MBOX2.MSGID.all & 0x1FFFFFFF;
            rx_msg.dlc = ECanaMboxes.MBOX2.MSGCTRL.bit.DLC;
            rx_msg.data[0] = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
            rx_msg.data[1] = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
            rx_msg.data[2] = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
            rx_msg.data[3] = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
            rx_msg.data[4] = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
            rx_msg.data[5] = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
            rx_msg.data[6] = ECanaMboxes.MBOX2.MDH.byte.BYTE6;
            rx_msg.data[7] = ECanaMboxes.MBOX2.MDH.byte.BYTE7;
            CanQueuePush(&rx_msg);
            break;
            
        case 3:  // Heart Bit 메시지
            heart_bit_count++;
            break;            
    }
    
    // 인터럽트 플래그 클리어
    ECanaRegs.CANRMP.all = (1UL << mailbox_nr);
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
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


