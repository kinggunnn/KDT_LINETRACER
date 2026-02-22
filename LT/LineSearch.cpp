//====================================================
// 작업자 : 임진효
// 최신화 일자 : 2026_02_20
// 용도 : 라인 상실 시 스파이럴 탐색 구현
// 기능 :
//   - 좌/중/우 IR 센서가 모두 라인을 감지하지 못했을 때
//     점점 회전 반경을 넓히는 나선형 탐색 수행
//   - 최대 15초 진행
//   - 라인 발견 즉시 종료i
//====================================================

#include "Robot.h"

constexpr unsigned long SPIRAL_LIMIT = 15000; // 15초
constexpr int SPEED_MIN = 80;
constexpr int DIFF_MAX = 150;
constexpr int DIFF_MIN = 40;

void spiral_search(int L, int C, int R){

    static bool searching = false;
    static unsigned long startTime = 0;

    bool lineLost = (L == HIGH && C == HIGH && R == HIGH);

    // 라인 발견 시 종료
    if(!lineLost){
        searching = false;
        startTime = 0;
        return;
    }

    if(!searching){
        searching = true;
        startTime = millis();
    }

    unsigned long elapsed = millis() - startTime;

    // 제한시간 초과 시 정지
    if(elapsed >= SPIRAL_LIMIT){
        driveStop();
        return;
    }

    // 시간 비율 계산
    float ratio = (float)elapsed / (float)SPIRAL_LIMIT;

    int diff = DIFF_MAX - (int)((DIFF_MAX - DIFF_MIN) * ratio);

    int rightSpeed = SPEED_MIN;
    int leftSpeed  = SPEED_MIN - diff;

    if(leftSpeed < 0) leftSpeed = 0;

    driveSetRaw(rightSpeed, leftSpeed);
}