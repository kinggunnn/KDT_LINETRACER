//====================================================
// 작업자 : 김유진
// 최신화 일자 : 2026_02_25
// 용도 : 도착 패턴 판별 모듈
// 기능 : 101 패턴이 누적 4회 이상 관측되면 도착으로 확정 (중간 1~2회 끊김은 감쇠 방식으로 허용)
//====================================================

#ifndef ARRIVAL_H
#define ARRIVAL_H

#include "Robot.h"


class ArrivalDetector {
public:
    void reset();
    bool update(const IRSample& ir, unsigned long deltaMs);

private:
    //unsigned long holdTime = 0;
    int8_t score = 0;                 // 누적 점수 (0~HIT)
    unsigned long windowMs = 0;       // 판정 시간 창 누적
};
#endif