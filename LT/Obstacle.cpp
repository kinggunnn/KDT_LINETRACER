#include "Robot.h"
//====================================================
// 작업자 : 박서희
// 최신화 일자 : 2026_02_21
// 용도 : 
// 함수 기능 : 
// 매개변수 :
// return 값 : 
//====================================================

bool isObstacleStable(long dist)
{
    const int THRESHOLD = 30;   // cm
    const int REQUIRED = 3;     // 연속 횟수

    static int count = 0;

    if (dist > 0 && dist <= THRESHOLD) {
        count++;

        // Serial.print("거리: ");
        // Serial.println(dist);

        // Serial.print("장애물 카운트: ");
        // Serial.println(count);

        if (count >= REQUIRED){
            // Serial.println("장애물로 인한 정지");
            count = 0;
            return true;
        }
    }
    else {
        count = 0;
    }

    return false;
}