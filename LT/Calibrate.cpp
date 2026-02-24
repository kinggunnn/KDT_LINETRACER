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
    // 라운드 시작 시점에서 중앙 센서 안정값 확보
    unsigned long start = millis();
    unsigned long firstEdge = 0;
    unsigned long secondEdge = 0;

    int prevStable = -1;

    // prevStable을 안정값으로 만들기 위한 루프(짧게)
    while (millis() - start < 500) {
      int v = readStableDigital(C_Line, fc, 3);
      if (v != -1) { prevStable = v; break; }
      delay(5);
    }
    if (prevStable == -1) return false; // 센서 안정값 자체를 못 얻음

    // 회전 시작
    unsigned long rotateStart = millis();

    while (millis() - rotateStart < timeoutMs)
    {
      driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

      int stable = readStableDigital(C_Line, fc, 3);
      if (stable == -1) continue;

      // BLACK → WHITE (첫 경계)
      if (prevStable == LOW && stable == HIGH && firstEdge == 0) {
        firstEdge = millis();
      }

      // WHITE → BLACK (두 번째 경계) : 한 바퀴 돌아서 라인 재진입했다고 가정
      if (prevStable == HIGH && stable == LOW && firstEdge != 0) {
        secondEdge = millis();
        break;
      }

      prevStable = stable;
    }

    driveStop();

    if (firstEdge == 0 || secondEdge == 0) {
      return false;
    }

    // 180도 시간 = (회전 시작 → 첫 경계까지)
    unsigned long measured180 = firstEdge - rotateStart;
    measuredSum += measured180;

    delay(500); // 다음 측정 전 안정화
  }

  g_turn180_ms = measuredSum / (unsigned long)repeatCount;
  return true;
}

unsigned long getCalibrated180()
{
  return g_turn180_ms;
}