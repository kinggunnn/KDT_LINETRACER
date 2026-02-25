#include "Robot.h"
#include "Arrival.h"
#include "Ending.h"
#include <Servo.h>
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
//   WAIT_START         : 출발 대기(중앙 센서가 라인 감지하면 출발)
//   LINE_TRACE         : 라인 추적 주행
//   SEARCH_ROTATE      : 라인 유실 시 제자리 회전 탐색
//   SEARCH_SPIRAL      : 회전 탐색 실패 후 스파이럴 탐색
//   OBSTACLE           : 장애물 거리 판단(감속/정지)
//   STOP_HOLD          : 장애물 너무 가까울 때 2초 정지
//   ESCAPE             : 탈출(제자리 회전하면서 라인 재획득)
//   ENDING             : 도착/이탈/종료 -> 정지 유지
//   CROSS_TURN_LEFT    : 교차로에서 좌회전 하는 경우
//   CROSS_GO_STRAIGHT  : 교차로에서 직진하는 경우
// -----------------------------------------------------
static FlowState state = FlowState::WAIT_START;

// -----------------------------------------------------
// [도착 판정 객체]
// - ArrivalDetector::update(ir, deltaMs)가 true를 반환하면 도착 확정
// - 내부에서 "패턴이 일정 시간 유지"를 카운트한다.
// -----------------------------------------------------
ArrivalDetector arrival;

// -----------------------------------------------------
// [Ending 제어 객체]
// - EndingController::start()가 호출되면 엔딩 시퀀스(감속 -> 정지 유지)가 시작
// - EndingController::update(deltaMs)는 엔딩 단계 진행을 수행 / 내부에서 경과 시간을 누적해 단계 전환을 관리
// - EndingController::isFinished()가 true를 반환하면 엔딩 동작 완료로 판단
// -----------------------------------------------------
EndingController ending;

// -----------------------------------------------------
// [시간/카운터 변수]
// - lostStart   : 라인 유실(흰,흰,흰)이 시작된 시각 기록
// - rotateStart : 회전 탐색(SEARCH_ROTATE)에서 회전 1바퀴 시간 카운트용 기준 시각
// - spiralStart : 스파이럴 탐색 시작 시간(15초 제한용)
// - stopStart   : STOP_HOLD(2초 정지) 시작 시간
// - rotateCount : 회전 탐색에서 몇 바퀴 돌았는지 카운트(시간 기준으로 근사) : 불필요시 제거 예정
// -----------------------------------------------------
static unsigned long lostStart = 0;
static unsigned long rotateStart = 0;
static unsigned long spiralStart = 0;
static unsigned long stopStart = 0;
static int rotateCount = 0;


Servo myServo;
// ---------------- 장애물 관련 변수들 ----------------
// 장애물 감지 -> 정지/회피 -> 라인복귀 과정에서
// 상태가 불필요하게 반복 진입/무한 루프 방지 용도
// ex) ESCAPE 직후에도 장애물을 보고 있어서
//     LINE_TRACE 진입 후, 다시 OBSTACLE로 재진입
static bool obstacleMode = false;           //현재 장애물 처리 루틴에 들어와 있는 상태를 표시
static unsigned long obstacleCooldown = 0;  //장애물 회피가 끝난 시각 기록.  ESCAPE 직후 같은 장애물 다시 감지 방지
constexpr unsigned long COOLDOWN = 2000;    // 2초 재감지 방지
static unsigned long escapeStart = 0;       //ESCAPE 상태가 시작된 시각(ms)을 기록
static unsigned long waitStartTime = 0; //초기 waittime 대기 시간 


// // 서범 : 0224
// static unsigned long turnStart = 0;
// static uint8_t cnt111 = 0;    // 111 연속 카운트
// static uint8_t cnt011F = 0;   // (011 + F=1) 연속 카운트

// ---------------- 시간 상수 ----------------
// LOOP_PERIOD_MS : loop 주기(50ms). deltaMs를 고정값으로 update에 넣기 위해 사용
// LOST_TRIGGER   : 라인 유실이 200ms 이상 지속되면 탐색 모드로 전환
// ROTATE_TIME    : 제자리 회전 탐색 "1바퀴"를 시간으로 근사(현재 2000ms 임시)
// SPIRAL_MAX     : 스파이럴 탐색 최대 시간(10초). 초과하면 ENDING(정지)
// STOP_TIME      : 장애물이 너무 가까울 때 정지 유지 시간(2초)
constexpr unsigned long LOOP_PERIOD_MS = 40;  // ★ 루프 주기(고정 deltaMs로 쓸 값)
constexpr unsigned long LOST_TRIGGER = 50;
//constexpr unsigned long ROTATE_TIME    = 2000;   // ★ 1회전 = 2000ms (임시값, 조정 필요) ->getCalibrated180() *2로 리턴
constexpr unsigned long SPIRAL_MAX = 10000;  // 10초 후 이탈
constexpr unsigned long STOP_TIME = 2000;    // 장애물 정지 2초

// 서범 : 0224
constexpr unsigned long TURN_FORWARD_MS = 100;   // 회전 시 안정 전진
constexpr unsigned long TURN_TIMEOUT_MS = 1200;  // 좌회전 안전 타임아웃
constexpr unsigned long STRAIGHT_PASS_MS = 200;  // ㅏ자에서 직진 통과 시간
constexpr uint8_t HIT_111 = 3;
constexpr uint8_t HIT_011F = 2;
// 분기 처리 후 재판정 방지(락아웃) - 루프/연속 오판 방지
static unsigned long crossLockUntil = 0;
constexpr unsigned long CROSS_LOCK_MS = 350;

void setup() {
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
  pinMode(L_Line, INPUT);
  pinMode(C_Line, INPUT);
  pinMode(R_Line, INPUT);
  pinMode(FC_Line, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  /* ---------------------------------------------------
  // 2026.02.24
  // [ 초기 서보모터 각도 고정 ]
  * --------------------------------------------------- */
  myServo.attach(2);  // 서보 연결 핀
  myServo.write(90);  // 시작 각도 (0~180)
  // ---------------------------------------------------
  // [모터 초기화]
  // - DriveControl.cpp의 driveInit()에서 모터 핀 OUTPUT 설정 + 정지 초기화
  // ---------------------------------------------------
  driveInit();

  //디버깅용
  Serial.println("Waiting for line...");

  if (!acquireLineInSetup(10000)) {
    Serial.println("Line not found. STOP.");
    while (1)
      ;
  }

  //디버깅용
  Serial.println("Calibrating 180 turn...");

  //디버깅용
  if (!calibrateTurn180InSetup(2, 8000)) {
    Serial.println("Calibration failed.");
    while (1)
      ;
  }

  //디버깅용
  Serial.print("Calibrated 180 time = ");

  ///180도 회전용 !!!! 리턴값~
  Serial.println(getCalibrated180());


  // ---------------------------------------------------
  // [도착 판정 초기화]
  // - 이전 테스트 값이 남는 것을 방지
  // ---------------------------------------------------
  arrival.reset();
}


void loop() {
  // ---------------------------------------------------
  // [루프 주기 제한]
  // - 50ms마다 한 번만 실행되도록 제한
  // - ArrivalDetector.update()에 deltaMs=50을 넣는 근거가 됨
  // ---------------------------------------------------
  static unsigned long lastLoopMs = 0;
  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_PERIOD_MS) return;
  lastLoopMs = now;

  // ---------------------------------------------------
  // [센서 읽기]
  // - ir.L/C/R : 라인센서 상태(LOW=흰색, HIGH=검정색)
  // - dist     : 초음파 거리(cm), 실패시 -1
  // ---------------------------------------------------
  IRSample ir = readIR();
  long dist = readUltrasonicCm();


  // ---------------------------------------------------
  // [상태머신]
  // - st가 시작된 시각(ms)을 기록ate 값에 따라 실행하는 동작이 달라짐
  // ---------------------------------------------------
  switch (state) {

  //WAIT_START 수정 : 0226 임진효
  //setup이후 센서가 탈출하는 경우가 있음 이때 보정 후 주행
  case FlowState::WAIT_START: {

    // 처음 진입했을 때 시간 기록
    if (waitStartTime == 0)
      waitStartTime = now;

    // 중앙 센서가 검정이면 바로 출발
    if (isBlack(ir.C)) {
      waitStartTime = 0;   // 다음에 다시 쓸 수 있도록 초기화
      state = FlowState::LINE_TRACE;
      break;
    }

    // 100ms 동안 검정 못 보면 회전 탐색으로 전환
    if (now - waitStartTime >= 100) {
      waitStartTime = 0;
      rotateStart = now;      // SEARCH_ROTATE용 초기화
      rotateCount = 0;
      state = FlowState::SEARCH_ROTATE;
    }

    break;
  }

  case FlowState::LINE_TRACE: {
    // 1) 도착 판정
    if (arrival.update(ir, LOOP_PERIOD_MS)) {
      Serial.println("GO ENDING: ARRIVAL");
      ending.start();
      state = FlowState::ENDING;
      break;
    }

    // 2) 교차로(111) → 좌회전 모드
    if (isBlack(ir.L) && isBlack(ir.C) && isBlack(ir.R)) {
      state = FlowState::CROSS_TURN_LEFT;
      break;
    }

    // 3) 라인 추적 (라인 하나라도 있으면 주행)
    if (isBlack(ir.L) || isBlack(ir.C) || isBlack(ir.R)) {
      driveLineFollow_detail(ir, SPEED_BASE);
      prev_ir = ir;
      hasPrevIr = true;
    }

    // 4) 라인 유실 판정 (000이 200ms 지속되면 탐색)
    if (isWhite(ir.L) && isWhite(ir.C) && isWhite(ir.R)) {
      if (lostStart == 0) lostStart = now;

      if (now - lostStart >= LOST_TRIGGER) {
        rotateCount = 0;
        rotateStart = now;
        state = FlowState::SEARCH_ROTATE;
        break;
      }
    } else {
      lostStart = 0;
    }

    // 5) 장애물 판정
    if (!obstacleMode &&
        isObstacleStable(dist) &&
        (now - obstacleCooldown > COOLDOWN)) {
      obstacleMode = true;
      state = FlowState::OBSTACLE;
      break;
    }

    break;
  }

  case FlowState::CROSS_TURN_LEFT: {
    // 좌회전(제자리 회전)
    driveSetRaw(SPEED_BASE, -SPEED_BASE);

    // 라인 재획득하면 복귀
    if (isBlack(ir.L) || isBlack(ir.C) || isBlack(ir.R)) {
      state = FlowState::LINE_TRACE;
    }

    break; // ★ 필수
  }

  case FlowState::SEARCH_ROTATE: {
    driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

    // "1바퀴" 시간 기준 카운트 (현재: 360도 = 2*180)
    if (now - rotateStart >= 2 * getCalibrated180()) {
      rotateCount++;
      rotateStart = now;
    }

    // 라인 재획득 시 복귀
    if (!(isWhite(ir.L) && isWhite(ir.C) && isWhite(ir.R))) {
      state = FlowState::LINE_TRACE;
      break;
    }

    // 2회전 초과 → 사각 탐색(SEARCH_SPIRAL)
    if (rotateCount > 2) {
      spiralStart = now;
      state = FlowState::SEARCH_SPIRAL;
      break;
    }

    break;
  }

  case FlowState::SEARCH_SPIRAL: {
    square_search(ir.L, ir.C, ir.R);

    // 라인 재획득 시 복귀
    if (isBlack(ir.L) || isBlack(ir.C) || isBlack(ir.R)) {
      state = FlowState::LINE_TRACE;
      break;
    }

    // (원하면 시간 제한도 다시 넣어라)
    // if (now - spiralStart >= SPIRAL_MAX) state = FlowState::ENDING;

    break;
  }

  case FlowState::OBSTACLE: {
    // 장애물 해제
    if (dist > 30) {
      obstacleMode = false;
      state = FlowState::LINE_TRACE;
      break;
    }

    // 너무 가까우면 정지
    if (dist <= 10) {
      driveStop();
      stopStart = now;
      state = FlowState::STOP_HOLD;
      break;
    }

    // 그 외 감속 주행
    driveLineFollow_detail(ir, SPEED_SLOW);
    break;
  }

  case FlowState::STOP_HOLD: {
    if (now - stopStart >= STOP_TIME) {
      escapeStart = now;
      state = FlowState::ESCAPE;
    }
    break;
  }

  case FlowState::ESCAPE: {
    driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

    if ((now - escapeStart > 800) && isBlack(ir.C)) {
      obstacleMode = false;
      obstacleCooldown = now;
      state = FlowState::LINE_TRACE;
    }
    break;
  }

  case FlowState::ENDING: {
    ending.update(LOOP_PERIOD_MS);
    if (ending.isFinished()) driveStop();
    break;
  }
}

  // ---------------------------------------------------
  // [디버깅: 상태 변경 순간만 출력]
  // - prevState와 다를 때만 출력해서 시리얼 스팸 방지
  // ---------------------------------------------------
  static int prevState = -1;
  int curState = (int)state;
  if (curState != prevState) {
    Serial.print("STATE CHANGED => ");
    Serial.println(curState);
    prevState = curState;
  }
}