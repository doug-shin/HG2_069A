// protocol.c - 프로토콜 처리 모듈
// CAN 통신을 통한 충/방전 명령 처리 및 상태 보고 기능 제공

#include "protocol.h"
#include "F2806x_Cla_typedefs.h"
#include "F2806x_Device.h"
#include <string.h>
#include "sicDCDC35kw.h" // UNIONFLOAT 타입 정의를 위해 추가

#define MODULE_CHANNEL 0x01 // 모듈 채널번호

/*----------------------------------------------------------------------
 * 전역 변수 정의
 *----------------------------------------------------------------------*/
PROTOCOL_INTEGRATED protocol;  // 프로토콜 구조체
STATE module_state;           // 모듈 상태 (STATE_IDLE or STATE_RUNNING)

// float32 <-> Uint32 변환용 전역 union 변수들
FLOAT_CONVERTER_UNION float_converter;

// Heart Bit 타임아웃 관련 변수 정의
Uint16 can_360_timeout_counter = 0;  // Heart Bit 타임아웃 카운터
Uint16 can_360_timeout_flag = 0;     // Heart Bit 타임아웃 플래그
Uint16 can_report_flag = 0;           // CAN 보고 플래그
Uint16 can_report_counter = 0;        // CAN 보고 카운터
Uint16 can_report_interval = 2000;    // CAN 보고 간격 (기본값: 100ms = 2000 * 0.05ms, 20kHz 주기)

extern volatile struct ECAN_REGS ECanaShadow;

// CAN 메일박스 배열 포인터 (최적화를 위해 전역으로 선언)
static struct MBOX *mbox_array = (struct MBOX *)&ECanaMboxes;

extern float32 currentCmdTemp; // 전류 지령 값 (A 단위)
extern UNIONFLOAT uiCurrentCommand; // 전류 지령 값 (A 단위)
extern float32 currentAvg;  // 전류 (A)

// 프로토콜 초기화 함수 - CAN ID 및 채널 설정 때문에 CAN 초기화 이후에 호출해야 함
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
    
    // 모든 메일박스 채널 정보 설정 (MBOX16~31, 16개)
    SetMBOXChannels(16, 16);
    
    // 초기 ID 설정 (대기 모드, MBOX16~23)
    ChangeMBOXIDs(0x110, 16, 8);
}

/**
 * 메일박스의 데이터를 프로토콜 구조체에 저장하는 함수
 * @param isr_mbox 수신 메일박스 번호
 * @param ack_mbox 송신 메일박스 번호 (ACK 전송용)
 */
void ProcessCANCommand(Uint32 isr_mbox, Uint32 ack_mbox)
{
    Uint16 command_id;
    Uint16 ack_id;
    Uint8 data[8];

    // 수신 메일박스에서 MSGID와 데이터 추출
    command_id = mbox_array[isr_mbox].MSGID.all & 0xFFFF;
    
    // MDL과 MDH에서 데이터 추출
    data[0] = mbox_array[isr_mbox].MDL.byte.BYTE0;
    data[1] = mbox_array[isr_mbox].MDL.byte.BYTE1;
    data[2] = mbox_array[isr_mbox].MDL.byte.BYTE2;
    data[3] = mbox_array[isr_mbox].MDL.byte.BYTE3;
    data[4] = mbox_array[isr_mbox].MDH.byte.BYTE4;
    data[5] = mbox_array[isr_mbox].MDH.byte.BYTE5;
    data[6] = mbox_array[isr_mbox].MDH.byte.BYTE6;
    data[7] = mbox_array[isr_mbox].MDH.byte.BYTE7;
    
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

        case 0x210:  // Control Command
            switch (data[1]) {
                case CMD_START:  // 시작 명령
                    TransitionToRunning();
                    break;

                case CMD_STOP:  // 정지 명령
                    TransitionToIdle();
                    break;
                
                default:        // 알 수 없는 명령
                    break;            
            }
            
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
            // 수신 메일박스 RMP 플래그 클리어
            ECanaRegs.CANRMP.all = ((Uint32)1 << isr_mbox);
            return;  // 알 수 없는 명령 ID는 ACK 응답 없이 종료
    }

    // 각 명령 ID에 대한 ACK ID 계산 (0x02xx → 0x22xx)
    ack_id = 0x2200 | (command_id & 0xFF);
    
    EALLOW;
    // 송신 메일박스 비활성화
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.all &= ~((Uint32)1 << ack_mbox);
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    
    // 송신 메일박스에 ACK ID 설정
    mbox_array[ack_mbox].MSGID.bit.EXTMSGID_L = ack_id;
    mbox_array[ack_mbox].MSGID.bit.IDE = 1;  // 29비트 확장 ID 사용
    
    // 수신된 데이터를 송신 메일박스에 복사 (ACK 데이터)
    mbox_array[ack_mbox].MDL.byte.BYTE0 = data[0];
    mbox_array[ack_mbox].MDL.byte.BYTE1 = data[1];
    mbox_array[ack_mbox].MDL.byte.BYTE2 = data[2];
    mbox_array[ack_mbox].MDL.byte.BYTE3 = data[3];
    mbox_array[ack_mbox].MDH.byte.BYTE4 = data[4];
    mbox_array[ack_mbox].MDH.byte.BYTE5 = data[5];
    mbox_array[ack_mbox].MDH.byte.BYTE6 = data[6];
    mbox_array[ack_mbox].MDH.byte.BYTE7 = data[7];

    // 송신 메일박스 활성화
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.all |= ((Uint32)1 << ack_mbox);
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    EDIS;

    // 전송 요청
    ECanaRegs.CANTRS.all = ((Uint32)1 << ack_mbox);
    // 전송 완료 대기 (타임아웃 추가)
    {
        Uint32 timeout_counter = 0;
        while(!(ECanaRegs.CANTA.all & ((Uint32)1 << ack_mbox))) {
            timeout_counter++;
            if(timeout_counter > 10000) {  // 타임아웃 발생
                // 전송 요청 취소 (TRR 레지스터 사용)
                ECanaRegs.CANTRR.all = ((Uint32)1 << ack_mbox);
                break;
            }
        }
        // 전송 완료 플래그 클리어
        ECanaRegs.CANTA.all = (Uint32)1 << ack_mbox;
    }

    // 수신 메일박스 RMP 플래그 클리어
    ECanaRegs.CANRMP.all = ((Uint32)1 << isr_mbox);
}

/**
 * 통합 CAN 보고 함수 (일괄 전송 버전)
 * 모든 메시지 데이터를 먼저 준비한 후 한꺼번에 송신 요청
 * MBOX16~23 사용
 */
void SendCANReport(Uint32 mbox_num) {
    Uint32 wait_count;
    const Uint32 mbox_mask = 0x00FF0000;  // MBOX16~23 마스크 (비트 16~23)
    
    // 1. 모든 메시지 데이터 준비 (8개 메시지)
    
    // Online Data #1 - 기본 상태 정보
    PUT_MBOX_BYTE(mbox_array[mbox_num +7], 0, protocol.channel);
    PUT_MBOX_UINT16(mbox_array[mbox_num +7], 1, protocol.cmd_step);
    PUT_MBOX_BYTE(mbox_array[mbox_num +7], 3, protocol.status);
    PUT_MBOX_BYTE(mbox_array[mbox_num +7], 4, protocol.mode);
    PUT_MBOX_BYTE(mbox_array[mbox_num +7], 5, protocol.state_bits.all);
    PUT_MBOX_UINT16(mbox_array[mbox_num +7], 6, protocol.event_code);
    
    // Online Data #2 - 온도 및 전압
    PUT_MBOX_INT16(mbox_array[mbox_num +6], 0, protocol.fb_t1_temp);
    PUT_MBOX_INT16(mbox_array[mbox_num +6], 2, protocol.fb_t2_temp);
    PUT_MBOX_FLOAT(mbox_array[mbox_num +6], 4, protocol.fb_voltage);
    
    // Online Data #3 - 전류 및 CV 시간
    PUT_MBOX_FLOAT(mbox_array[mbox_num +5], 0, protocol.fb_current);
    PUT_MBOX_UINT32(mbox_array[mbox_num +5], 4, protocol.fb_cv_time);
    
    // Online Data #4 - 충방전 Ah
    PUT_MBOX_FLOAT(mbox_array[mbox_num +4], 0, protocol.fb_charge_ah);
    PUT_MBOX_FLOAT(mbox_array[mbox_num +4], 4, protocol.fb_discharge_ah);
    
    // Online Data #5 - 충방전 Wh
    PUT_MBOX_FLOAT(mbox_array[mbox_num +3], 0, protocol.fb_charge_wh);
    PUT_MBOX_FLOAT(mbox_array[mbox_num +3], 4, protocol.fb_discharge_wh);
    
    // Online Data #6 - 운전 시간
    PUT_MBOX_UINT32(mbox_array[mbox_num +2], 0, protocol.fb_operation_time);
    
    // Online Data #7 - PWM 오류 정보
    PUT_MBOX_UINT16(mbox_array[mbox_num +1], 0, protocol.pwm_hw_error);
    PUT_MBOX_UINT16(mbox_array[mbox_num +1], 2, protocol.pwm_sw_error1);
    PUT_MBOX_UINT16(mbox_array[mbox_num +1], 4, protocol.pwm_sw_error2);
    PUT_MBOX_UINT16(mbox_array[mbox_num +1], 6, protocol.pwm_sw_warning);
    
    // Online Data #8 - DC/DC 오류 정보
    PUT_MBOX_UINT16(mbox_array[mbox_num], 0, protocol.dcdc_hw_error);
    PUT_MBOX_UINT16(mbox_array[mbox_num], 2, protocol.dcdc_sw_error1);
    PUT_MBOX_UINT16(mbox_array[mbox_num], 4, protocol.dcdc_sw_error2);
    PUT_MBOX_UINT16(mbox_array[mbox_num], 6, protocol.dcdc_sw_warning);
    
    // 2. 이전 전송 완료 플래그 클리어
    ECanaRegs.CANTA.all = mbox_mask;
    
    // 3. 한꺼번에 송신 요청
    ECanaRegs.CANTRS.all = mbox_mask;
    
    // 4. 전송 완료 대기 중에 피드백 값 업데이트 (최적화)
    UpdateCANFeedbackValues();
    
    // 5. 모든 전송 완료 대기 (타임아웃 포함)
    wait_count = 0;
    while((ECanaRegs.CANTA.all & mbox_mask) != mbox_mask) {
        wait_count++;
        if(wait_count > 1000) break;  // 타임아웃
    }
    
    // 6. 전송 완료 플래그 클리어
    ECanaRegs.CANTA.all = mbox_mask;
}

/**
 * 종료 보고 전송 함수 (순차 전송 버전)
 * 모듈 종료 시 종합적인 상태 정보를 MBOX18~23에 순서대로 전송
 * 130번대 메시지 6개 전송
 */
void SendCANEndReport(Uint32 mbox_num) {
    Uint32 i;
    Uint32 wait_count;
    // mbox_num=18일 때: MBOX18~23 (6개) 마스크 생성
    // 마스크 = 0x00FC0000 (비트 18~23)
    static Uint32 mbox_mask = 0;
    for(i = 0; i < 6; i++) {
        mbox_mask |= ((Uint32)1 << (mbox_num + i));
    }
    
    // 1. 6개 메시지 데이터 구성 (한 번에 모든 데이터 준비)
    
    // Step End Data #1 - 기본 상태 정보
    PUT_MBOX_BYTE(mbox_array[mbox_num + 5], 0, protocol.channel);
    PUT_MBOX_UINT16(mbox_array[mbox_num + 5], 1, protocol.cmd_step);
    PUT_MBOX_BYTE(mbox_array[mbox_num + 5], 3, protocol.status);
    PUT_MBOX_BYTE(mbox_array[mbox_num + 5], 4, protocol.mode);
    PUT_MBOX_UINT16(mbox_array[mbox_num + 5], 5, protocol.event_code);
    PUT_MBOX_BYTE(mbox_array[mbox_num + 5], 7, protocol.pattern_index);
    
    // Step End Data #2 - 온도 및 전압
    PUT_MBOX_INT16(mbox_array[mbox_num + 4], 0, protocol.fb_t1_temp);
    PUT_MBOX_INT16(mbox_array[mbox_num + 4], 2, protocol.fb_t2_temp);
    PUT_MBOX_FLOAT(mbox_array[mbox_num + 4], 4, protocol.fb_voltage);
    
    // Step End Data #3 - 전류 및 CV 시간
    PUT_MBOX_FLOAT(mbox_array[mbox_num + 3], 0, protocol.fb_current);
    PUT_MBOX_UINT32(mbox_array[mbox_num + 3], 4, protocol.fb_cv_time);
    
    // Step End Data #4 - 충방전 Ah
    PUT_MBOX_FLOAT(mbox_array[mbox_num + 2], 0, protocol.fb_charge_ah);
    PUT_MBOX_FLOAT(mbox_array[mbox_num + 2], 4, protocol.fb_discharge_ah);
    
    // Step End Data #5 - 충방전 Wh
    PUT_MBOX_FLOAT(mbox_array[mbox_num + 1], 0, protocol.fb_charge_wh);
    PUT_MBOX_FLOAT(mbox_array[mbox_num + 1], 4, protocol.fb_discharge_wh);
    
    // Step End Data #6 - 운전 시간 및 Pattern Index
    PUT_MBOX_UINT32(mbox_array[mbox_num], 0, protocol.fb_operation_time);
    PUT_MBOX_UINT32(mbox_array[mbox_num], 4, protocol.pattern_index);
    
    // 2. 이전 전송 완료 플래그 클리어
    ECanaRegs.CANTA.all = mbox_mask;
    
    // 3. 한꺼번에 송신 요청 (mbox_mask 사용)
    ECanaRegs.CANTRS.all = mbox_mask;
    
    // 4. 모든 전송 완료 대기 (타임아웃 포함)
    wait_count = 0;
    while((ECanaRegs.CANTA.all & mbox_mask) != mbox_mask) {
        wait_count++;
        if(wait_count > 10000) break;  // 타임아웃
    }
    
    // 5. 전송 완료 플래그 클리어
    ECanaRegs.CANTA.all = mbox_mask;
}

/**
 * 모든 피드백 값을 한 번에 업데이트하는 통합 함수
 * 데이터 일관성을 위해 원자적으로 처리
 */
void UpdateCANFeedbackValues(void) {
    extern float32 Vo;      // 전압 (V)
    extern float32 In_Temp;  // 35kW 프로그램의 온도 값 (°C)
    static Uint16 operation_tick_counter = 0;
    static Uint16 cv_tick_counter = 0;
    static Uint16 capacity_tick_counter = 0;
    float32 delta_capacity;
    
    // 1. 기본 피드백 값 업데이트 (단위 변환: V->mV, A->mA, °C->0.1°C)
    protocol.fb_voltage = Vo * 1000.0f;       // V -> mV 변환
    protocol.fb_current = currentAvg * 1000.0f;   // A -> mA 변환
    protocol.fb_t1_temp = In_Temp * 10.0f;    // °C -> 0.1°C 변환
    
    // 2. 운전 시간 업데이트 (운전 중일 때만)
    if (protocol.status == OPERATING) {
        operation_tick_counter++;
        if (operation_tick_counter >= 100) { // 1초마다 (10ms * 100 = 1초)
            operation_tick_counter = 0;
            protocol.fb_operation_time++;  // 10ms 단위로 저장
        }
    }
    
    // 3. CV 시간 업데이트 (운전 중이고 CV 조건일 때)
    if (protocol.status == OPERATING) {
        cv_tick_counter++;
        if (cv_tick_counter >= 1) {  // 10ms마다
            cv_tick_counter = 0;
            
            // CV 모드 체크 (전압이 설정값에 도달했는지 확인)
            if ((protocol.cmd_mode == MODE_CC || protocol.cmd_mode == MODE_CV) &&
                (protocol.fb_voltage >= protocol.cmd_voltage)) {
                protocol.fb_cv_time++;
            }
        }
    }
    
    // 4. 용량 값 업데이트 (운전 중일 때만)
    if (protocol.status == OPERATING) {
        capacity_tick_counter++;
        if (capacity_tick_counter >= 100) { // 1초마다
            // C89 변수 선언을 블록 시작 부분으로 이동
            float32 power_w, delta_wh;
            
            capacity_tick_counter = 0;
            
            // 전류값을 Ah로 변환 (A -> mAh)
            delta_capacity = currentAvg * 0.2777778f;  // A -> mAh, 1000/3600 = 0.2777778
            
            // 충방전 용량 업데이트
            if (delta_capacity > 0) {
                protocol.fb_charge_ah += delta_capacity;  // mAh 단위로 누적
            } else {
                protocol.fb_discharge_ah += -delta_capacity;  // mAh 단위로 누적
            }
            
            // Wh 계산 (P = V * I)
            power_w = protocol.fb_voltage * 0.001f * currentAvg;  // W (mV -> V 변환)
            delta_wh = power_w * 0.0002777778f;  // Wh (1/3600 = 0.0002777778)
            
            if (delta_wh > 0) {
                protocol.fb_charge_wh += delta_wh;
            } else {
                protocol.fb_discharge_wh += -delta_wh;
            }
        }
    }
}

/**
 * 메일박스 채널 정보 설정 함수
 * MODULE_CHANNEL을 MSGID의 비트 16~23에 설정
 * @param start_mbox 시작 메일박스 번호
 * @param count 설정할 메일박스 개수
 */
void SetMBOXChannels(Uint16 start_mbox, Uint16 count) {
    Uint32 i;
    
    EALLOW;
    
    // 쉐도우 레지스터를 실제 레지스터와 동기화
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    
    for(i = 0; i < count; i++) {
        Uint32 mailbox = start_mbox + i;
        
        // 메일박스 비활성화
        ECanaShadow.CANME.all &= ~((Uint32)1 << mailbox);
        ECanaRegs.CANME.all = ECanaShadow.CANME.all;
                
        // MODULE_CHANNEL을 비트 16~23에 설정 (상위 8비트)
        mbox_array[mailbox].MSGID.bit.EXTMSGID_H = MODULE_CHANNEL & 0x03;        // 비트 17:16 (하위 2비트)
        mbox_array[mailbox].MSGID.bit.STDMSGID = (MODULE_CHANNEL >> 2) & 0x3F;   // 비트 23:18 (상위 6비트)
        
        // 메일박스 활성화
        ECanaShadow.CANME.all |= ((Uint32)1 << mailbox);
        ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    }
    
    EDIS;
}

/**
 * 메일박스 ID 변경 함수
 * @param base_id 시작 ID (예: 0x100, 0x110, 0x130)
 * @param start_mbox 시작 메일박스 번호 (예: 16, 18)
 * @param count 변경할 메일박스 개수 (예: 6, 8)
 * 
 * 주의: EXTMSGID_H와 STDMSGID는 변경하지 않음 (채널 정보 유지)
 *       EXTMSGID_L(16비트)만 변경하여 메시지 ID 설정
 */
void ChangeMBOXIDs(Uint16 base_id, Uint16 mbox_num, Uint16 count) {
    Uint32 i;
    EALLOW;
    
    // 지정된 범위의 메일박스 ID 설정
    for(i = 0; i < count; i++) {
        Uint32 mailbox = mbox_num + (count - 1 - i);  // 높은 번호부터 할당
        Uint16 msg_id = base_id + i;  // base_id부터 순차적으로 증가
        
        // 메일박스 비활성화
        ECanaShadow.CANME.all = ECanaRegs.CANME.all;
        ECanaShadow.CANME.all &= ~((Uint32)1 << mailbox);
        ECanaRegs.CANME.all = ECanaShadow.CANME.all;
        
        // EXTMSGID_L만 변경 (EXTMSGID_H와 STDMSGID는 채널 정보로 유지)
        mbox_array[mailbox].MSGID.bit.EXTMSGID_L = msg_id;
        
        // 메일박스 활성화
        ECanaShadow.CANME.all |= ((Uint32)1 << mailbox);
        ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    }
    
    EDIS;
}

// 운전 상태로 전환 함수
void TransitionToRunning(void) {
    extern UNIONFLOAT uiCurrentCommand; // 전류 지령 값 (A 단위)
    extern float32 currentCmdTemp; // 전류 지령 값 (A 단위)
    extern float32 V_com; // 전압 지령 값 (V 단위)
    extern float32 Power; // 파워 지령 값 (W 단위)
    extern float32 Voh_com, Vol_com; // 배터리 모드 전압 제한값
    extern Uint16 can_report_interval; // CAN 보고 간격
    
    // 하드웨어 제어 변수 설정
    Run = 1;
    
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
    
    // CAN 보고 메시지 ID를 운전 모드로 변경 (MBOX23=0x100, MBOX22=0x101, ..., MBOX16=0x107)
    ChangeMBOXIDs(0x100, 16, 8);
    
    // 단위 변환: mV -> V, mA -> A, mW -> W (곱셈 최적화)
    // 전류 지령 설정 (외부 변수에 저장)
    currentCmdTemp = protocol.cmd_current * 0.001f; // mA -> A 변환
    
    // 전압 지령 설정 (필요한 경우)
    V_com = protocol.cmd_voltage * 0.001f; // mV -> V 변환
    
    // 파워 지령 설정 (필요한 경우)
    Power = protocol.cmd_power * 0.001f; // mW -> W 변환
    
    // 배터리 모드 전압 목표값 설정 (부호 반전 방식)
    if (protocol.cmd_mode == MODE_CC_CV || protocol.cmd_mode == MODE_CP_CV) {
        Voh_com = protocol.cmd_voltage * 0.001f; // mV -> V 변환
        Vol_com = -protocol.cmd_voltage * 0.001f; // mV -> V 변환
    }
    
    // CAN 보고 간격 설정 (운전 상태: 10ms 간격 = 200 * 0.05ms, 20kHz 주기)
    can_report_interval = 200;
}

// 대기 상태로 전환 함수
void TransitionToIdle(void) {
    extern UNIONFLOAT uiCurrentCommand; // 전류 지령 값
    extern Uint16 can_report_interval; // CAN 보고 간격
    
    // 하드웨어 제어 변수 설정
    Run = 0;
    
    // 전류 지령 0으로 설정
    uiCurrentCommand.f = 0.0f;
    
    // 상태 변경
    module_state = STATE_IDLE;
    protocol.status = READY;
    
    // 종료 보고용 메일박스 활성화 (MBOX18~23, 6개)
    SetMBOXChannels(18, 6);
    
    // 종료 메시지 ID 설정 (MBOX23=0x130, MBOX22=0x131, ..., MBOX18=0x135)
    ChangeMBOXIDs(0x130, 18, 6);
    
    // 종료 보고 전송 (130번대 메시지는 별도 함수로 처리)
    SendCANEndReport(18);

    // CAN 보고 메시지 ID를 대기 모드로 변경 (MBOX23~16에 0x110~0x117)
    ChangeMBOXIDs(0x110, 16, 8);

    // CAN 보고 간격 설정 (대기 상태: 100ms 간격 = 2000 * 0.05ms, 20kHz 주기)
    can_report_interval = 2000;
}

// Heart Bit 타임아웃 체크 함수
void CheckCANHeartBitTimeout(void) {
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
