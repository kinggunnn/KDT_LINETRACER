//====================================================
// 작업자 : 임진효
// 최신화 일자 : 2026_02_20
// 용도 : 도착 패턴 판별 모듈
// 기능 :
//   - IR 센서 3개 중 2개 이상이 검정 상태가
//     300ms 이상 유지되면 도착 확정
//====================================================

#ifndef ARRIVAL_H
#define ARRIVAL_H

#include "Robot.h"


class ArrivalDetector {
public:
    void reset();
    bool update(const IRSample& ir, unsigned long deltaMs);

private:
    unsigned long holdTime = 0;
};
#endif