/*  HG2_CLA_shared.h
 *  CLA와 CPU 간 공유 변수 및 PI 컨트롤러 정의
 *  HG2 35kW DC-DC 컨버터 제어 시스템용
 */

#ifndef _HG2_CLA_SHARED_H
#define _HG2_CLA_SHARED_H

#include "DCLCLA.h"
#include "F2806x_Cla_typedefs.h"
#include "F2806x_Cla_defines.h"

// CLA 메모리 설정 플래그
#define CLA_PROG_ENABLE      0x0001
#define CLARAM0_ENABLE       0x0010
#define CLARAM1_ENABLE       0x0020
#define CLARAM2_ENABLE       0x0040
#define CLA_RAM0CPUE         0x0100
#define CLA_RAM1CPUE         0x0200
#define CLA_RAM2CPUE         0x0400

// CLA 인터럽트 소스는 F2806x_Cla_defines.h에서 정의됨

/* 충전용 PI 컨트롤러 공유 변수 */
extern float charge_rk;          // 충전 기준값 (V_max_lim)
extern float charge_yk;          // 충전 피드백값 (V_out)
extern float charge_uk;          // 충전 제어 출력값 (V_max_PI)
extern DCL_PI_CLA dcl_pi_charge_cla;

/* 방전용 PI 컨트롤러 공유 변수 */
extern float discharge_rk;       // 방전 기준값 (V_min_lim)
extern float discharge_yk;       // 방전 피드백값 (V_out)
extern float discharge_uk;       // 방전 제어 출력값 (V_min_PI)
extern DCL_PI_CLA dcl_pi_discharge_cla;

/* 제어 모드 플래그 */
extern float control_mode;       // 0: 충전 모드, 1: 방전 모드

// CLA 메모리 주소 심볼
extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadEnd;
extern Uint16 Cla1funcsLoadSize;
extern Uint16 Cla1funcsRunStart;

// CLA math table 심볼
extern Uint16 Cla1mathTablesLoadStart;
extern Uint16 Cla1mathTablesLoadEnd;
extern Uint16 Cla1mathTablesLoadSize;
extern Uint16 Cla1mathTablesRunStart;

// CLA Task 함수 선언
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();    // 충전용 PI 제어 태스크
__interrupt void Cla1Task4();    // 방전용 PI 제어 태스크
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

// CLA Force 함수는 F2806x_Cla_defines.h에서 매크로로 정의됨

#endif // _HG2_CLA_SHARED_H

/* end of file */
