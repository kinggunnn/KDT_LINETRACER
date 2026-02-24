//====================================================
// 작업자 : 김유진
// 최신화 일자 : 2026_02_21
// 용도 : Ending 상태 표시 및 차량 정지 제어
//====================================================
#ifndef ENDING_H
#define ENDING_H

#include "Robot.h"
#include "Pitches.h"
class EndingController {
public:
	void start();
	void update(unsigned long deltaMs);
	bool isFinished() const;

private:
	void setLedBlink(unsigned long nowMs); 
	void setLedOn();                      
	void beep();                     
	void doReduce();   
	static void Play_MarioUW();                     

	// 진행 중 LED 깜빡임 주기
	static constexpr unsigned long BLINK_PERIOD_MS = 200;

	// 부저 소리 지속 시간(ms)
	static constexpr unsigned long BEEP_MS = 120;


};
#endif