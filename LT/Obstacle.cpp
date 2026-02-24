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
    const int THRESHOLD = 30;   // cm 이하 = 장애물
    const int REQUIRED = 3;     // 연속 횟수
    static int count = 0;

    if (dist > 0 && dist <= THRESHOLD) {
        count++;

        if (count >= REQUIRED) {
            count = 0;
            return true;
        }
    } 
    else {
        count = 0;
    }

    return false;
}