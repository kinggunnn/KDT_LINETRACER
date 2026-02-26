//====================================================
// 작업자 : 김유진
// 최신화 일자 : 2026_02_25
// 용도 : 도착 패턴 구현부
//====================================================

#include "Arrival.h"

//constexpr unsigned long ARRIVAL_TIME = 300; // 300ms 유지

// void ArrivalDetector::reset(){
//     holdTime = 0;
// }
// bool ArrivalDetector::update(const IRSample& ir, unsigned long deltaMs){

//     // 도착 패턴: 검정 - 흰색 - 검정
//     if(isBlack(ir.L) && isWhite(ir.C) && isBlack(ir.R)){
//         holdTime += deltaMs;
//         Serial.print("[ARR] holdTime=");
//         Serial.println(holdTime);
//         if(holdTime >= 300){
//             return true;
//         }
//     }
//     else{
//         if (holdTime > 0) Serial.println("[ARR] reset");
//         holdTime = 0;
//     }

//     return false;
// }



#include "Arrival.h"

// HIT: 몇 점 이상이면 도착 확정 
constexpr int8_t ARRIVAL_HIT = 4;

// 시간 창: 이 시간 안에 HIT 못 채우면 리셋 
constexpr unsigned long ARRIVAL_WINDOW_MS = 800;

// 101이면 +, 아니면 - (대각선 내성 조절)
constexpr int8_t SCORE_UP = 1;
constexpr int8_t SCORE_DOWN = 1;

void ArrivalDetector::reset() {
  score = 0;
  windowMs = 0;
}

bool ArrivalDetector::update(const IRSample& ir, unsigned long deltaMs) {

  bool is101 = isBlack(ir.L) && isWhite(ir.C) && isBlack(ir.R);

  // 판정 시간 창 누적
  windowMs += deltaMs;

  if (is101) {
    if (score < ARRIVAL_HIT) score += SCORE_UP;
  } else {
    if (score > 0) score -= SCORE_DOWN;   // 감쇠 (0 아래로는 안 내려가지 않도록)
  }

  // 디버그
  // Serial.print("[ARR] is101=");
  // Serial.print(is101);
  // Serial.print(" score=");
  // Serial.print(score);
  // Serial.print(" windowMs=");
  // Serial.println(windowMs);

  // 도착 확정
  if (score >= ARRIVAL_HIT) {
    return true;
  }

  // 시간 초과면 리셋 (오탐/잡음 방지)
  if (windowMs >= ARRIVAL_WINDOW_MS) {
    score = 0;
    windowMs = 0;
  }

  return false;
}