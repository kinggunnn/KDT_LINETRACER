//====================================================
// 작업자 : 임진효
// 최신화 : 2026_02_23
// 용도 : setup에서 1회 실행되는 회전 보정 로직
// 특징 :
//   - 중앙 IR 센서(C)만 사용
//   - 3회 동일값 들어와야 값 확정(노이즈 필터)
//   - BLACK→WHITE→BLACK 경계 기반 180도 측정
//   - 2회 반복 평균 계산
//====================================================

#include "Robot.h"

static unsigned long g_turn180_ms = 2000; // 기본 보험값


// 중앙 센서(C) 전용 구조체 선언
static StableDigitalFilter fc;

// ===============================
// 1) setup용 라인 획득
// - 중앙 센서가 BLACK(HIGH) 될 때까지
// - 제자리 회전 탐색==> 추후 탐색회전으로 발전시킬 예정
// - timeoutMs 초과 시 false
// ===============================
bool acquireLineInSetup(unsigned long timeoutMs)
{
  unsigned long start = millis();

  while (millis() - start < timeoutMs)
  {
    int stableC = readStableDigital(C_Line, fc, 3);
    if (stableC == -1) {
      // 아직 안정화 안 됨 → 계속 회전하며 샘플 쌓기
      driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);
      continue;
    }

    if (stableC == HIGH) {  // LINE_BLACK = HIGH 기준
      driveStop();
      return true;
    }

    // 라인 못 잡았으면 계속 회전
    driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);
  }

  driveStop();
  return false;
}

// ===============================
// 2) 180도 보정 함수
// - repeatCount=2 권장
// - timeoutMs는 "한 회차" 제한 시간
// ===============================
bool calibrateTurn180InSetup(int repeatCount, unsigned long timeoutMs)
{
  unsigned long measuredSum = 0;

  for (int round = 0; round < repeatCount; round++)
  {
    unsigned long start = millis();       // 라운드 전체 타임아웃 기준
    unsigned long firstEdge = 0;          // BLACK->WHITE (라인 이탈)
    unsigned long secondEdge = 0;         // WHITE->BLACK (라인 재진입)

    int prevStable = -1;

    // 0) 안정값 하나 확보(최대 500ms)
    while (millis() - start < 500) {
      int v = readStableDigital(C_Line, fc, 3);
      if (v != -1) { prevStable = v; break; }
      delay(5);
    }
    if (prevStable == -1) return false;

    // 1) 회전 시작
    unsigned long rotateStart = millis();

    while (millis() - rotateStart < timeoutMs)
    {
      driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

      int stable = readStableDigital(C_Line, fc, 3);
      if (stable == -1) continue;

      // [시작점] BLACK -> WHITE (HIGH -> LOW) : 라인에서 벗어나는 순간
      if (prevStable == HIGH && stable == LOW && firstEdge == 0) {
        firstEdge = millis();
      }

      // [종료점] WHITE -> BLACK (LOW -> HIGH) : 라인에 다시 올라타는 순간
      if (prevStable == LOW && stable == HIGH && firstEdge != 0) {
        secondEdge = millis();
        break;
      }

      prevStable = stable;
    }

    driveStop();

    // 타임아웃/실패 처리
    if (firstEdge == 0 || secondEdge == 0) {
      return false;
    }

    // 2) 한 바퀴 시간(라인 이탈->재진입)을 이용해 180도 시간 계산
    //    라인 두께가 두꺼워도 "경계~경계"는 상대적으로 안정적임
    unsigned long fullTurnMs = secondEdge - firstEdge;
    unsigned long measured180 = fullTurnMs / 2;

    measuredSum += measured180;

    delay(500); // 다음 라운드 전 안정화(센서 튐 방지)
  }

  // 3) 2회 평균
  g_turn180_ms = measuredSum / (unsigned long)repeatCount;

  // 4) (선택) 캘리브레이션 끝나고 라인 위(HIGH)로 복귀 시도: 출발 막힘 방지
  unsigned long fixStart = millis();
  const unsigned long FIX_TIMEOUT = 2000;
  while (millis() - fixStart < FIX_TIMEOUT) {
    int stableC = readStableDigital(C_Line, fc, 3);
    if (stableC == HIGH) break;
    driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);
  }
  driveStop();

  return true;
}
unsigned long getCalibrated180()
{
  return g_turn180_ms;
}