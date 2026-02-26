//========================================================================================
// 작업자 : 김유진
// 최신화 일자 : 2026_02_21
// 용도 : 도착 시 Ending 상태 처리 구현
// 기능 :
//   - 도착 패턴이 인식되면 차량을 즉시 정지하지 않고 단계적으로 감속
//   - 감속 후 차량을 완전히 정지시키고 Ending 상태를 표시
//   - LED 켜짐 + 부저 울림으로  Ending 상태를 표시
//   - Ending 동작이 완료되면 차량을 정지 상태로 유지
//========================================================================================

#include "Ending.h"

//========================================================================================
// Ending 동작의 현재 단계를 저장하는 상태 변수
//========================================================================================
enum class State {
	Idle,
	Reduce,
	Hold,
	Done
};

// 현재 Ending 진행 단계
static State state = State::Idle;

// 현재 단계에서 경과된 시간(ms) : update()가 호출 될 때마다 delaMS 만큼 증가
//                                 단계 전환 조건 판단에 사용
static unsigned long stageTime = 0;

constexpr unsigned long REDUCE_TIME = 400;	// 감속 총 시간
constexpr unsigned long HOLD_TIME = 1500;	// 정지 유지 시간

// LED 타이머
static unsigned long blinkAcc = 0;
static bool ledOn = false;

// Done 진입 시 부저를 울리기
static bool beeped = false;




//========================================================================================
// [Ending 시작 함수]
// - 도착 패턴이 인식된 순간에 1회 호출
// - Ending 상태를 감속 단계로 설정
//========================================================================================
void EndingController::start() {
	// 이미 Ending이 진행 중이면 중복 시작 방지
	if (state != State::Idle)
		return;

	state = State::Reduce;
	stageTime = 0;

	pinMode(LED_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	digitalWrite(LED_PIN, LOW);
	digitalWrite(BUZZER_PIN, LOW);
	blinkAcc = 0;

	ledOn = false;

	beeped = false; // Done 들어갈 때 beep 1회 울리기 위해 초기화

}

//===========================================================
// 부저 멜로디 (2024.02.23)
//===========================================================
//############## "Mario underworld" ##############//
int MarioUW_note[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_AS3, NOTE_AS4, 0, 0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_AS3, NOTE_AS4, 0, 0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4, NOTE_DS3, NOTE_DS4, 0, 0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4, NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4, NOTE_DS4, NOTE_GS3, NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3, NOTE_AS3, NOTE_A3, NOTE_GS3, 0, 0, 0
};

int MarioUW_duration[] = {
  12,12,12,12,12,12,6,3,
  12,12,12,12,12,12,6,3,
  12,12,12,12,12,12,6,3,
  12,12,12,12,12,12,6,6,18,18,18,
  6,6,6,6,6,6,
  18,18,18,18,18,18,10,10,10,
  10,10,10,3,3,3
};
//########### End of Mario underworld ###########//

static void EndingController::Play_MarioUW()
{
  int total = sizeof(MarioUW_note) / sizeof(int);

  for (int i = 0; i < total; i++) {
    int noteDuration = 1000 / MarioUW_duration[i];
    if (MarioUW_note[i] == 0) {
      noTone(BUZZER_PIN);
    } else {
      tone(BUZZER_PIN, MarioUW_note[i], noteDuration);
    }

    int pauseBetweenNotes = (int)(noteDuration * 1.80);
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}




//========================================================================================
// [Ending 진행 단계]
// Idle : Ending 시작 전 대기 상태
// Reduce: 감속 단계
// Hold: 정지 유지 단계
// Done: Ending 완료 상태
// 
// 감속을 사용하는 이유: 
// (1) 급정지시,
// 관성으로 인해 RC카가 흔들릴 수 있음
// 모터에 순간적인 부하를 주어 하드웨어에 부담을 줄 수 있음
// 
// (2) 실제 차량과 같이 자연스럽고 안정적인 정지를 위함
//========================================================================================

void EndingController::update(unsigned long deltaMs) {

	// 현재 단계 경과 시간 누적
	stageTime += deltaMs;

	switch (state)
	{
	case State::Idle:
		break;

	case State::Reduce:
		// 진행 중 표시: LED 깜빡임 시작
		setLedBlink(deltaMs);

		doReduce();

		// 감속 완료 후 Hold 단계로 전환
		if (stageTime >= REDUCE_TIME) {
			state = State::Hold;
			stageTime = 0;

			// Hold 진입 시 완전 정지 1회
			driveStop();
		}
		break;

	case State::Hold:
		// 진행 중 표시: LED 깜빡
		setLedBlink(deltaMs);

		// 정지 유지
		driveStop();

		if (stageTime >= HOLD_TIME) {
			state = State::Done;
			stageTime = 0;

			// Done 진입 순간: LED OFF 고정 + 부저 1회
			setLedOff();
			//beep();
		
		}
		break;


	case State::Done:
		// 완료 표시: LED 항상 OFF
		setLedOff();

		// Done에서는 정지 유지
		driveStop();
	  if (!beeped) {
    beeped = true;
    Play_MarioUW();   
  	}
		break;

	}
}
//====================================================
// isFinished()
// - main에서 엔딩 완료 확인용
//====================================================
bool EndingController::isFinished() const {
	return state == State::Done;
}


//====================================================
// doReduce()
// - 단계적 감속(급정지 방지, 흔들림/충격 완화)
//====================================================
void EndingController::doReduce() {

	if (stageTime < 200) {
		driveSetRaw(120, 120);
	}
	else if (stageTime < 400) {
		driveSetRaw(90, 90);
	}
	else {
		driveStop();
	}
}


//====================================================
// setLedBlink(deltaMs)
// - 엔딩 진행 중(Decel/Hold)에 LED를 주기적으로 토글
//====================================================
void EndingController::setLedBlink(unsigned long deltaMs) {

	blinkAcc += deltaMs;

	if (blinkAcc >= BLINK_PERIOD_MS) {
		blinkAcc = 0;
		ledOn = !ledOn;
		digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
	}
}


//====================================================
// setLedOff()
// - Done(완료) 표시: LED 항상 OFF
//====================================================
void EndingController::setLedOff() {
	ledOn = false;
	digitalWrite(LED_PIN, HIGH);
}


//====================================================
// beep()
// - Done 진입 순간에 부저를 1번 울림
//====================================================
// void EndingController::beep() {

// 	if (beeped) return;
// 	beeped = true;

// 	digitalWrite(BUZZER_PIN, HIGH);
// 	delay(BEEP_MS);                
// 	digitalWrite(BUZZER_PIN, LOW);
// }

