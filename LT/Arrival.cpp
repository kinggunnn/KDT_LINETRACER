//====================================================
// 작업자 : 임진효 
// 최신화 일자 : 2026_02_20
// 용도 : 도착 패턴 구현부
//====================================================

#include "Arrival.h"

constexpr unsigned long ARRIVAL_TIME = 300; // 300ms 유지

void ArrivalDetector::reset(){
    holdTime = 0;
}
bool ArrivalDetector::update(const IRSample& ir, unsigned long deltaMs){

    // 도착 패턴: 검정 - 흰색 - 검정
    if(isBlack(ir.L) && isWhite(ir.C) && isBlack(ir.R)){
        holdTime += deltaMs;
        Serial.print("[ARR] holdTime=");
        Serial.println(holdTime);
        if(holdTime >= 300){
            return true;
        }
    }
    else{
        if (holdTime > 0) Serial.println("[ARR] reset");
        holdTime = 0;
    }

    return false;
}