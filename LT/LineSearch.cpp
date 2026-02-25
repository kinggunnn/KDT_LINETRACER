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
  // 라인 유실: 모두 흰색(LOW)  (검정=HIGH 기준 유지)
  const bool lineLost = (L == LOW && C == LOW && R == LOW);

  static bool searching = false;
  static unsigned long searchStartMs = 0;

  static uint8_t phase = 0; // 0=직진, 1=회전
  static unsigned long phaseStartMs = 0;
  static uint8_t straightMul = 1;

  static unsigned long t90 = 0;

  if (!lineLost) {
    searching = false;
    searchStartMs = 0;
    phase = 0;
    phaseStartMs = 0;
    straightMul = 1;
    t90 = 0;
    return;
  }

  if (!searching) {
    searching = true;
    searchStartMs = millis();
    phase = 0;
    phaseStartMs = searchStartMs;
    straightMul = 1;

    // 캘리브레이션 기반 90도 시간(회전 속도는 반드시 캘리브레이션과 동일하게 쓸 것)
    t90 = getCalibrated180() / 2;
    if (t90 < 150) t90 = 150; // 너무 짧으면 무조건 찔끔이라 하한 보험
  }

  unsigned long now = millis();

  if (now - searchStartMs >= SEARCH_LIMIT_MS) {
    driveStop();
    searching = false;
    return;
  }

  if (phase == 0) {
    // 직진
    driveSetRaw(SEARCH_SPEED, SEARCH_SPEED);

    unsigned long straightTime = STRAIGHT_BASE_MS * (unsigned long)straightMul;
    if (now - phaseStartMs >= straightTime) {
      phase = 1;
      phaseStartMs = now;
    }
  } else {
    // 90도 회전 (캘리브레이션과 같은 회전 속도 사용)
    const unsigned long KICK_MS = 80;
    int turn = SPEED_ROTATE;          // 캘리브레이션과 동일 속도
    if (now - phaseStartMs < KICK_MS) turn = SPEED_ROTATE ; // 시작 킥(상황에 따라 20~60 조절)

    driveSetRaw(turn, -turn);

    if (now - phaseStartMs >= t90) {
      phase = 0;
      phaseStartMs = now;
      if (straightMul < 3) straightMul++;
    }
  }
}