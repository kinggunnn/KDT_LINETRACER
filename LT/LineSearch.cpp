//====================================================
// 작업자 : 임진효
// 최신화 일자 : 2026_02_23
// 용도 : 라인 상실 시 스파이럴 탐색 구현
// 기능 :
//   - 좌/중/우 IR 센서가 모두 라인을 감지하지 못했을 때
//     점점 회전 반경을 넓히는 나선형 탐색 수행
//   - 최대 15초 진행
//   - 라인 발견 즉시 종료
//====================================================

#include "Robot.h"

constexpr unsigned long SEARCH_LIMIT_MS = 10000; // 10초
// constexpr int SPEED_MIN = 80;
// constexpr int DIFF_MAX = 150;
// constexpr int DIFF_MIN = 40;

constexpr unsigned long STRAIGHT_BASE_MS = 500;   // 직진 기본 시간(1배)
constexpr unsigned long TURN_90_MS = 0;           // ★ 90도 회전 시간 = 보정값으로 채움

constexpr int SEARCH_SPEED = 120;                 // 탐색 직진 속도

// void spiral_search(int L, int C, int R){

//     static bool searching = false;
//     static unsigned long startTime = 0;

//     bool lineLost = (L == HIGH && C == HIGH && R == HIGH);

//     // 라인 발견 시 종료
//     if(!lineLost){
//         searching = false;
//         startTime = 0;
//         return;
//     }

//     if(!searching){
//         searching = true;
//         startTime = millis();
//     }

//     unsigned long elapsed = millis() - startTime;

//     // 제한시간 초과 시 정지
//     if(elapsed >= SPIRAL_LIMIT){
//         driveStop();
//         return;
//     }

//     // 시간 비율 계산
//     float ratio = (float)elapsed / (float)SPIRAL_LIMIT;

//     int diff = DIFF_MAX - (int)((DIFF_MAX - DIFF_MIN) * ratio);

//     int rightSpeed = SPEED_MIN;
//     int leftSpeed  = SPEED_MIN - diff;

//     if(leftSpeed < 0) leftSpeed = 0;

//     driveSetRaw(rightSpeed, leftSpeed);
// }


/*
  =====================================================
  square_search(L, C, R)
  -----------------------------------------------------
  역할:
    - 라인 유실(LOW,LOW,LOW) 상태에서만 "사각형 탐색" 수행
    - 직진 시간: 1배 → 2배 → 3배 ... 증가
    - 각 직진 뒤 90도 회전 후 다시 직진

  매개변수:
    L, C, R : IR 센서 상태 (HIGH=검정 라인, LOW=흰 바닥)

  주의:
    - 90도 회전 시간은 "180도 보정값"으로 계산해서 쓰는 걸 권장
      예) unsigned long t90 = getCalibrated180() / 2;
      아래 코드에서 TURN_90_MS 대신 t90를 넣어주면 됨.
  =====================================================
*/
void square_search(int L, int C, int R)
{
  // 라인 유실 조건: 모두 흰색(LOW)
  const bool lineLost = (L == LOW && C == LOW && R == LOW);

  // 탐색 상태 유지용 static
  static bool searching = false;
  static unsigned long searchStartMs = 0;

  // 단계 상태
  // phase=0: 직진 구간
  // phase=1: 90도 회전 구간
  static uint8_t phase = 0;
  static unsigned long phaseStartMs = 0;

  // 직진 시간 배수(1,2,3,...)
  static uint8_t straightMul = 1;

  // 라인 잡히면 즉시 탐색 종료(상태 리셋)
  if (!lineLost) {
    searching = false;
    searchStartMs = 0;
    phase = 0;
    phaseStartMs = 0;
    straightMul = 1;
    return;
  }

  // 탐색 시작 초기화
  if (!searching) {
    searching = true;
    searchStartMs = millis();
    phase = 0;
    phaseStartMs = millis();
    straightMul = 1;
  }

  unsigned long now = millis();

  // 전체 제한 시간 초과 -> 정지 및 종료
  if (now - searchStartMs >= SEARCH_LIMIT_MS) {
    driveStop();
    searching = false;
    return;
  }

  // ---- 90도 회전 시간 결정 ----
    unsigned long t90 = 0;
  t90 = getCalibrated180() / 2;

  if (t90 == 0) {
    driveStop();
    return;
  }

  // ---- 사각형 탐색 FSM ----
  if (phase == 0) {
    // [직진 구간]
    driveSetRaw(SEARCH_SPEED, SEARCH_SPEED);

    unsigned long straightTime = STRAIGHT_BASE_MS * (unsigned long)straightMul;

    if (now - phaseStartMs >= straightTime) {
      phase = 1;
      phaseStartMs = now;
    }
  }
  else {
    // [90도 회전 구간] (제자리 회전)
    driveSetRaw(SEARCH_SPEED, -SEARCH_SPEED);

    if (now - phaseStartMs >= t90) {
      phase = 0;
      phaseStartMs = now;

      // 직진 길이 증가 
      if (straightMul < 3) straightMul++;
    }
  }
}