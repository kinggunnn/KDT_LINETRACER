#include "Robot.h"
#include "Arrival.h"

/*
  =====================================================
  [main.ino 역할]
  - 전체 상태머신(FlowState)을 굴리는 메인 제어부
  - 센서(IR/초음파)를 주기적으로 읽고,
    조건에 따라 상태를 바꾸며 DriveControl의 모터 API를 호출한다.

  [중요]
  - driveLineFollow / driveSetRaw / driveStop 같은 "하드웨어 제어"는
    DriveControl.cpp에 구현되어 있어야 한다.
  - ArrivalDetector(도착 판정)는 Arrival.cpp에서 구현되어 있어야 한다.
  =====================================================
*/

// -----------------------------------------------------
// [상태 변수]
// - state: 현재 로봇이 어떤 모드로 동작 중인지 나타냄
//   WAIT_START    : 출발 대기(중앙 센서가 라인 감지하면 출발)
//   LINE_TRACE    : 라인 추적 주행
//   SEARCH_ROTATE : 라인 유실 시 제자리 회전 탐색
//   SEARCH_SPIRAL : 회전 탐색 실패 후 스파이럴 탐색
//   OBSTACLE      : 장애물 거리 판단(감속/정지)
//   STOP_HOLD     : 장애물 너무 가까울 때 2초 정지
//   ESCAPE        : 탈출(제자리 회전하면서 라인 재획득)
//   ENDING        : 도착/이탈/종료 -> 정지 유지
// -----------------------------------------------------
static FlowState state = FlowState::WAIT_START;

// -----------------------------------------------------
// [도착 판정 객체]
// - ArrivalDetector::update(ir, deltaMs)가 true를 반환하면 도착 확정
// - 내부에서 "패턴이 일정 시간 유지"를 카운트한다.
// -----------------------------------------------------
ArrivalDetector arrival;

// -----------------------------------------------------
// [시간/카운터 변수]
// - lostStart   : 라인 유실(흰,흰,흰)이 시작된 시각 기록
// - rotateStart : 회전 탐색(SEARCH_ROTATE)에서 회전 1바퀴 시간 카운트용 기준 시각
// - spiralStart : 스파이럴 탐색 시작 시간(15초 제한용)
// - stopStart   : STOP_HOLD(2초 정지) 시작 시간
// - rotateCount : 회전 탐색에서 몇 바퀴 돌았는지 카운트(시간 기준으로 근사) : 불필요시 제거 예정
// -----------------------------------------------------
static unsigned long lostStart   = 0;
static unsigned long rotateStart = 0;
static unsigned long spiralStart = 0;
static unsigned long stopStart   = 0;
static int rotateCount = 0;

// ---------------- 시간 상수 ----------------
// LOOP_PERIOD_MS : loop 주기(50ms). deltaMs를 고정값으로 update에 넣기 위해 사용
// LOST_TRIGGER   : 라인 유실이 200ms 이상 지속되면 탐색 모드로 전환
// ROTATE_TIME    : 제자리 회전 탐색 "1바퀴"를 시간으로 근사(현재 2000ms 임시)
// SPIRAL_MAX     : 스파이럴 탐색 최대 시간(10초). 초과하면 ENDING(정지)
// STOP_TIME      : 장애물이 너무 가까울 때 정지 유지 시간(2초)
constexpr unsigned long LOOP_PERIOD_MS = 50; // ★ 루프 주기(고정 deltaMs로 쓸 값)
constexpr unsigned long LOST_TRIGGER   = 200;
constexpr unsigned long ROTATE_TIME    = 2000;   // ★ 1회전 = 2000ms (임시값, 조정 필요)
constexpr unsigned long SPIRAL_MAX     = 10000;  // 10초 후 이탈
constexpr unsigned long STOP_TIME      = 2000;   // 장애물 정지 2초

void setup(){
  // ---------------------------------------------------
  // [시리얼 디버그]
    // ---------------------------------------------------
  Serial.begin(9600);
  Serial.println("BOOT OK");

  // ---------------------------------------------------
  // [센서 핀모드]
  // - IR 센서 3개: 디지털 입력
  // - 초음파: trig 출력 / echo 입력
  // ---------------------------------------------------
  pinMode(L_Line,INPUT);
  pinMode(C_Line,INPUT);
  pinMode(R_Line,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);

  // ---------------------------------------------------
  // [모터 초기화]
  // - DriveControl.cpp의 driveInit()에서 모터 핀 OUTPUT 설정 + 정지 초기화
  // ---------------------------------------------------
  driveInit();

  // ---------------------------------------------------
  // [도착 판정 초기화]
  // - 이전 테스트 값이 남는 것을 방지
  // ---------------------------------------------------
  arrival.reset();
}

void loop(){
  // ---------------------------------------------------
  // [루프 주기 제한]
  // - 50ms마다 한 번만 실행되도록 제한
  // - ArrivalDetector.update()에 deltaMs=50을 넣는 근거가 됨
  // - 50ms니깐 "1000/50=20" 즉 1초에 20번 실행됨 => 그러나 루프 한번이 몇초 걸리는지 모르기에 20번 이하일 확률이 높음. 확인해봐야함.
  // ---------------------------------------------------
  static unsigned long lastLoopMs = 0;
  unsigned long now = millis();
  if(now - lastLoopMs < LOOP_PERIOD_MS) return;
  lastLoopMs = now;

  // 서범 : 러닝 타임 확인하기 위한 코드
  //unsigned long t0 = micros();

  // ---------------------------------------------------
  // [센서 읽기]
  // - ir.L/C/R : 라인센서 상태(LOW=검정, HIGH=흰색)
  // - dist     : 초음파 거리(cm), 실패시 -1
  // ---------------------------------------------------
  IRSample ir = readIR();
  long dist = readUltrasonicCm();

  // ---------------------------------------------------
  // [상태머신]
  // - state 값에 따라 실행하는 동작이 달라짐
  // ---------------------------------------------------
  switch(state){

    // =================================================
    // 0) 출발 대기
    // - 중앙 센서가 라인을 감지하면(=검정 LOW) 출발
    // =================================================
    case FlowState::WAIT_START:
      if(isBlack(ir.C)){
        state = FlowState::LINE_TRACE;
      }
      break;

    // =================================================
    // 1) 라인 추적 주행
    // - driveLineFollow(): 기본 주행 및 좌/우 보정
    // - 라인 유실 200ms 이상이면 SEARCH_ROTATE로 전환
    // - 장애물 감지되면 OBSTACLE로 전환
    // - 도착(ArrivalDetector true)이면 ENDING으로 전환
    // =================================================
    case FlowState::LINE_TRACE:
    {
      Serial.print("STATE=");
      Serial.print((int)state);
      Serial.print(" IR=");
      Serial.print(ir.L);Serial.print(",");
      Serial.print(ir.C);Serial.print(",");
      Serial.print(ir.R);
      Serial.print(" dist=");
      Serial.println(dist);
      // -----------------------------
      // [IR 디버깅 출력]
      // - 200ms마다 한 번씩 IR 값과 blackCount 출력
      // - blackCount: 검정으로 판정된 센서 개수(0~3)
      // -----------------------------
      static unsigned long t = 0;
      //아래 if문은 디버깅용으러 넣어놨ㅅ므다~
      if(millis() - t >= 200){
        t = millis();
        Serial.print("IR=");
        Serial.print(ir.L); Serial.print(",");
        Serial.print(ir.C); Serial.print(",");
        Serial.print(ir.R);
        Serial.print(" blackCount=");
        int blackCount =
          (isBlack(ir.L)?1:0) + (isBlack(ir.C)?1:0) + (isBlack(ir.R)?1:0);
        Serial.println(blackCount);
      }

      // -----------------------------
      // [라인 추적 주행]
      // - DriveControl.cpp에서 구현된 라인 추적 함수
      // - 기본 속도 SPEED_BASE(=140)로 주행
      // -----------------------------
      driveLineFollow(ir, SPEED_BASE);

      // -----------------------------
      // [도착 판정 ]
       // -----------------------------
      if(arrival.update(ir, LOOP_PERIOD_MS)){
        Serial.println("GO ENDING: ARRIVAL");
        state = FlowState::ENDING;
        break;
      }

      // -----------------------------
      // [라인 유실 판정]
      // - L/C/R 모두 흰색(HIGH,HIGH,HIGH)이면 라인 유실로 판단
      // - 200ms 이상 지속되면 제자리 회전 탐색으로 전환
      // -----------------------------
      if(isWhite(ir.L) && isWhite(ir.C) && isWhite(ir.R)){
        Serial.println("LINE LOST");
        if(lostStart == 0) lostStart = now;

        if(now - lostStart >= LOST_TRIGGER){
          rotateCount = 0;
          rotateStart = now;
          state = FlowState::SEARCH_ROTATE;
        }
      }else{
        // 라인을 다시 잡으면 유실 타이머 리셋
        lostStart = 0;
      }

      // -----------------------------
      // [장애물]
      // - 30cm 이내면 장애물 상태로 진입(감속/정지 판단은 OBSTACLE에서)
      // -----------------------------
      if(dist > 0 && dist <= 10){
        state = FlowState::OBSTACLE;
      }

      // -----------------------------
      // [도착 판정 2회차 (중복)]
      // 님들 여기 잘 보고 적어야함~~ 혹시 너무 자주 멈추면 여기 주석처리하세요~
      //   여기서 또 호출하면 도착 판정이 "빨라질 수 있음"
            // -----------------------------
      // if(arrival.update(ir, LOOP_PERIOD_MS)){
      //   Serial.println("GO ENDING: ARRIVAL");
      //   state = FlowState::ENDING;
      //   break;
      // }

      break;
    }

    // =================================================
    // 2) 라인 탐색 - 제자리 회전
    // - 좌/우 바퀴를 반대방향으로 돌려 제자리 회전
    // - ROTATE_TIME(임시 2000ms)을 1회전으로 보고 rotateCount 증가
    // - 2회전 초과(rotateCount>2)면 SEARCH_SPIRAL로 전환
    // - 회전 중 라인 재획득하면 LINE_TRACE로 복귀
    // =================================================
    case FlowState::SEARCH_ROTATE:

      // 제자리 회전(오른쪽 전진 / 왼쪽 후진)
      driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

      // 회전 시간 기준으로 "한 바퀴" 카운트 증가
      if(now - rotateStart >= ROTATE_TIME){
        rotateCount++;
        rotateStart = now;
        Serial.print("RotateCount=");
        Serial.println(rotateCount);
      }

      // 라인 재획득 시 즉시 복귀
      if(!(isWhite(ir.L) && isWhite(ir.C) && isWhite(ir.R))){
        state = FlowState::LINE_TRACE;
      }

      // 2회전 초과 -> 스파이럴 탐색
      // (요구사항: 2회전 초과=3번째부터)
      if(rotateCount > 2){
        spiralStart = now;
        state = FlowState::SEARCH_SPIRAL;
      }

      break;

    // =================================================
    // 3) 라인 탐색 - 스파이럴
    // - LineSearch.cpp의 spiral_search()가 "나선형"으로 라인 탐색 수행
    // - 스파이럴은 최대 15초(SPIRAL_MAX) 진행, 초과하면 ENDING으로 정지
    // - 탐색 중 라인 재획득하면 LINE_TRACE로 복귀
    // =================================================
    case FlowState::SEARCH_SPIRAL:

      // 스파이럴 탐색(내부에서 속도차로 회전반경 확장)
      spiral_search(ir.L, ir.C, ir.R);

      // 라인 재획득 시 복귀
      if(!(isWhite(ir.L) && isWhite(ir.C) && isWhite(ir.R))){
        state = FlowState::LINE_TRACE;
      }

      // 스파이럴 제한시간 초과 -> 이탈 모드(정지 유지)
      if(now - spiralStart >= SPIRAL_MAX){
        Serial.println("GO ENDING: SPIRAL TIMEOUT");
        state = FlowState::ENDING;
      }

      break;

    // =================================================
    // 4) 장애물 처리
    // - dist <= 10cm : 정지 후 STOP_HOLD로 전환
    // - 10cm < dist <= 30cm : 감속 주행
    // =================================================
    case FlowState::OBSTACLE:

      if(dist <= 10){
        driveStop();
        stopStart = now;
        state = FlowState::STOP_HOLD;
      }
      else{
        // 감속 주행
        driveLineFollow(ir, SPEED_SLOW);
      }

      break;

    // =================================================
    // 5) 장애물 정지 유지(2초)
    // - STOP_TIME(2초) 지나면 ESCAPE로 전환
    // =================================================
    case FlowState::STOP_HOLD:

      if(now - stopStart >= STOP_TIME){
        state = FlowState::ESCAPE;
      }
      break;

    // =================================================
    // 6) 탈출 모드
    // - 제자리 회전하며 라인을 다시 찾음
    // - 라인 재획득하면 LINE_TRACE로 복귀
    // =================================================
    case FlowState::ESCAPE:
      driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

      if(!(isWhite(ir.L) && isWhite(ir.C) && isWhite(ir.R))){
        state = FlowState::LINE_TRACE;
      }
      break;

    // =================================================
    // 7) ENDING
    // - 도착/이탈 등 종료 상태
    // - 모터 정지 유지
    // =================================================
    case FlowState::ENDING:
      driveStop();
      break;
  }

  // ---------------------------------------------------
  // [디버깅: 상태 변경 순간만 출력]
  // - prevState와 다를 때만 출력해서 시리얼 스팸 방지
  // ---------------------------------------------------
  static int prevState = -1;
  int curState = (int)state;
  if(curState != prevState){
    Serial.print("STATE CHANGED => ");
    Serial.println(curState);
    prevState = curState;
  }

}