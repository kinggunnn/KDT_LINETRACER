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
static unsigned long lostStart   = 0;
static unsigned long rotateStart = 0;
static unsigned long spiralStart = 0;
static unsigned long stopStart   = 0;
static int rotateCount = 0;


Servo myServo;
// ---------------- 장애물 관련 변수들 ----------------
// 장애물 감지 -> 정지/회피 -> 라인복귀 과정에서 
// 상태가 불필요하게 반복 진입/무한 루프 방지 용도
// ex) ESCAPE 직후에도 장애물을 보고 있어서  
//     LINE_TRACE 진입 후, 다시 OBSTACLE로 재진입
static bool obstacleMode = false; //현재 장애물 처리 루틴에 들어와 있는 상태를 표시
static unsigned long obstacleCooldown = 0; //장애물 회피가 끝난 시각 기록.  ESCAPE 직후 같은 장애물 다시 감지 방지
constexpr unsigned long COOLDOWN = 2000;  // 2초 재감지 방지
static unsigned long escapeStart = 0; //ESCAPE 상태가 시작된 시각(ms)을 기록


// 서범 : 0224
static unsigned long turnStart = 0;
static uint8_t cnt111 = 0;    // 111 연속 카운트
static uint8_t cnt011F = 0;   // (011 + F=1) 연속 카운트

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

// 서범 : 0224
constexpr unsigned long TURN_FORWARD_MS = 100; // 회전 시 안정 전진
constexpr unsigned long TURN_TIMEOUT_MS = 1200; // 좌회전 안전 타임아웃
constexpr unsigned long STRAIGHT_PASS_MS = 200; // ㅏ자에서 직진 통과 시간
constexpr uint8_t HIT_111 = 3;
constexpr uint8_t HIT_011F = 2;
// 분기 처리 후 재판정 방지(락아웃) - 루프/연속 오판 방지
static unsigned long crossLockUntil = 0;
constexpr unsigned long CROSS_LOCK_MS = 350;

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
  pinMode(FC_Line, INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  /* ---------------------------------------------------
  // 2026.02.24
  // [ 초기 서보모터 각도 고정 ]
  * --------------------------------------------------- */
  myServo.attach(2);   // 서보 연결 핀
  myServo.write(90);   // 시작 각도 (0~180)
  // ---------------------------------------------------
  // [모터 초기화]
  // - DriveControl.cpp의 driveInit()에서 모터 핀 OUTPUT 설정 + 정지 초기화
  // ---------------------------------------------------
  driveInit();

  //디버깅용 
  Serial.println("Waiting for line...");

  if(!acquireLineInSetup(10000)) {
    Serial.println("Line not found. STOP.");
    while(1);
  }

  //디버깅용 
  Serial.println("Calibrating 180 turn...");

  //디버깅용 
  if(!calibrateTurn180InSetup(2, 8000)) {
    Serial.println("Calibration failed.");
    while(1);
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


void loop(){
  // ---------------------------------------------------
  // [루프 주기 제한]
  // - 50ms마다 한 번만 실행되도록 제한
  // - ArrivalDetector.update()에 deltaMs=50을 넣는 근거가 됨
  // ---------------------------------------------------
  static unsigned long lastLoopMs = 0;
  unsigned long now = millis();
  if(now - lastLoopMs < LOOP_PERIOD_MS) return;
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
      // Serial.print("STATE=");
      // Serial.print((int)state);
      // Serial.print(" IR=");
      // Serial.print(ir.L);Serial.print(",");
      // Serial.print(ir.C);Serial.print(",");
      // Serial.print(ir.R);
      // Serial.print(" dist=");
      // Serial.println(dist);
      // -----------------------------
      // [IR 디버깅 출력]
      // - 200ms마다 한 번씩 IR 값과 blackCount 출력
      // - blackCount: 검정으로 판정된 센서 개수(0~3)
      // -----------------------------
      //static unsigned long t = 0;
      //아래 if문은 디버깅용으러 넣어놨ㅅ므다~
      // if(millis() - t >= 200){
      //   t = millis();
      //   Serial.print("IR=");
      //   Serial.print(ir.L); Serial.print(",");
      //   Serial.print(ir.C); Serial.print(",");
      //   Serial.print(ir.R);
      //   Serial.print(" blackCount=");
      //   int blackCount =
      //     (isBlack(ir.L)?1:0) + (isBlack(ir.C)?1:0) + (isBlack(ir.R)?1:0);
      //   Serial.println(blackCount);
      // }

      // -----------------------------
      // [라인 추적 주행]
      // - DriveControl.cpp에서 구현된 라인 추적 함수
      // - 기본 속도 SPEED_BASE(=100)로 주행
      // -----------------------------
      // driveLineFollow_detail(ir, SPEED_BASE);

      // -----------------------------
      // [도착 판정 ]
       // -----------------------------
      bool arrived = arrival.update(ir, LOOP_PERIOD_MS);
      if(arrival.update(ir, LOOP_PERIOD_MS)){
        Serial.println("GO ENDING: ARRIVAL");
        ending.start();
        state = FlowState::ENDING;
        break;
      }


      if (now < crossLockUntil){
        cnt111 = 0;
        cnt011F = 0;
      } else{
        // 111 카운트
        if (isBlack(ir.L) && isBlack(ir.C) && isBlack(ir.R)) cnt111++;
        else cnt111 = 0;

        // 011 + F 카운트
        if (isBlack(ir.FC) && !isBlack(ir.L) && isBlack(ir.C) && isBlack(ir.R)) cnt011F++;
        else cnt011F = 0;

        // --------------------------------------
        // [규칙 1]
        // - 111이면 FC 상관없이 무조건 좌회전
        // --------------------------------------
        if (cnt111 >= HIT_111){ // 3번 이상 반복되면 실행
          cnt111 = 0;
          cnt011F = 0;
          crossLockUntil = now + CROSS_LOCK_MS;

          turnStart = now;
          state = FlowState::CROSS_TURN_LEFT;

          // 출력되는 값 확인용
          Serial.print("DECIDE(111)->LEFT  L,C,R,FC=");
          Serial.print(ir.L); Serial.print(",");
          Serial.print(ir.C); Serial.print(",");
          Serial.print(ir.R); Serial.print(",");
          Serial.println(ir.FC);
          break; // switch(state)에서 LINE_TRACE case 종료
        }

        // -------------------------------------------
        // [규칙 2]
        // - 011 + 전방감지(FC=1)이면 (ㅏ자형태) -> 직진
        // -------------------------------------------
        if (cnt011F >= HIT_011F) {
          cnt111 = 0;
          cnt011F = 0;
          crossLockUntil = now + CROSS_LOCK_MS;

          turnStart = now;
          state = FlowState::CROSS_GO_STRAIGHT;

          Serial.print("DECIDE(011F)->STRAIGHT  L,C,R,FC=");
          Serial.print(ir.L); Serial.print(",");
          Serial.print(ir.C); Serial.print(",");
          Serial.print(ir.R); Serial.print(",");
          Serial.println(ir.FC);

          break; // switch(state)에서 LINE_TRACE case 종료
        }
      }

      // -----------------------------
      // [라인 이전값 저장]
      // - 라인 유실이 아닐 때 이전 값 저장
      // -----------------------------
      if (isBlack(ir.L) || isBlack(ir.C) || isBlack(ir.R)) {
        driveLineFollow_detail(ir, SPEED_BASE); // 100

        // 정상일 때만 prev_ir 갱신
        prev_ir = ir;
        hasPrevIr = true;
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
      // 초음파 거리값(dist)이 일정 범위 이내에서 연속 3회 이상 안정적으로 감지되면 장애물로 판단
      // -----------------------------
      if(!obstacleMode &&
        isObstacleStable(dist) &&
        now - obstacleCooldown > COOLDOWN)
      {
          obstacleMode = true;  // 장애물 처리 모드 진입
          state = FlowState::OBSTACLE; // 장애물 처리 상태로 전환
      }
    

      break;
    }

    // ==============================================================
    // 서범 : 0224
    // 111일 때 -> 좌회전 (왼쪽을 우선시 하는 로직)
    // ==============================================================
    case FlowState::CROSS_TURN_LEFT:
    {
      unsigned long dt = now - turnStart;

      // 111 판단 됐을 때, 100ms 정도 살짝 전진
      if (dt < TURN_FORWARD_MS) {
        driveSetRaw(SPEED_BASE, SPEED_BASE); 
        break; // 약간 직진 하고 break걸어서 아래 코드 실행 안되게 방지
      }

      // 강한 좌회전
      driveSetRaw(SPEED_BASE+50, 50); // 오른쪽은 기본 스피스+50 , 왼쪽 바퀴는 거의 꺼져있는 값(50) 

      // 중앙 검정색 + 모두 111이 아니면 => 재흭득하면 종료
      if (isBlack(ir.C) && !(isBlack(ir.L) && isBlack(ir.C) && isBlack(ir.R))){
        state = FlowState::LINE_TRACE;
        break;
      }

      // 혹시나 무한 루프에 빠질 수도 있으니깐 처리해놓기 -> 1.2초 지나면 돌아서 가기
      if (dt > TURN_TIMEOUT_MS){
        rotateStart = now;
        rotateCount = 0;
        state = FlowState::SEARCH_ROTATE;
      }
      break;
    }

    // ==============================================================
    // 서범 : 0224
    // 011 때 -> 직진 (ㅏ자 형태에서 직진하는 로직)
    // ==============================================================
    case FlowState::CROSS_GO_STRAIGHT:
    {
       unsigned long dt = now - turnStart;

       driveSetRaw(SPEED_BASE, SPEED_BASE); // 직진

      // 교차로 통과 시간(200ms로 가정)만큼 직진 후 복귀
      if (dt > STRAIGHT_PASS_MS) {
        state = FlowState::LINE_TRACE;
      }
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

      // 사각형 탐색 수행
      square_search(ir.L, ir.C, ir.R);

      // 라인 재획득 시 즉시 복귀
      // (검정 = HIGH 기준이면)
      if(ir.L == HIGH || ir.C == HIGH || ir.R == HIGH){
        state = FlowState::LINE_TRACE;
      }

      break;

    // =================================================
    // 4) 장애물 처리
    // - dist <= 10cm : 정지 후 STOP_HOLD로 전환
    // - 10cm < dist <= 30cm : 감속 주행
    // =================================================
    case FlowState::OBSTACLE:

      // [장애물 해제 조건]
      // - 초음파 거리값이 30cm 초과이면 장애물이 사라졌다고 판단
      // - obstacleMode를 false로 내려서 다시 장애물 감지가 가능
      if(dist > 30){
        obstacleMode = false; // 장애물 처리 종료
        state = FlowState::LINE_TRACE;  // 정상 라인트레이싱으로 복귀
        break;
      }

      // [매우 가까운 장애물]
      // - 즉시 모터 정지 후 STOP_HOLD 상태로 전환
      // - stopStart에 현재 시각을 저장하여
      //   정지 유지 시간(STOP_TIME) 계산에 사용
      if(dist <= 10){
        // Serial.println("장애물->일시정지");
        driveStop();  // 모터 완전 정지
        stopStart = now;  // 정지 시작 시각 기록
        state = FlowState::STOP_HOLD;
      }
      // [중간 거리 장애물]
      // - 10cm < dist <= 30cm 구간
      // - 완전 정지 대신 감속 주행으로 대응
      // - 장애물과 일정 거리를 유지하면서 라인 추적 지속
      else{
        // 감속 주행
        driveLineFollow_detail(ir, SPEED_SLOW);
      }

      break;

    // =================================================
    // 5) 장애물 정지 유지(2초)
    // - STOP_TIME(2초) 지나면 ESCAPE로 전환
    // =================================================
    // [정지 유지 → 탈출 모드 전환]
    // - STOP_TIME(2초) 동안 정지 상태 유지
    // - 이후 ESCAPE 상태로 전환하여 장애물 회피 동작 수행
    // - escapeStart는 ESCAPE 시작 시각으로,
    //   최소 회전 시간 보장을 위해 사용된다.
    case FlowState::STOP_HOLD:
      if(now - stopStart >= STOP_TIME){
      escapeStart = now; 
      state = FlowState::ESCAPE;
    }
    break;

    // =================================================
    // 6) 탈출 모드
    // - 제자리 회전하며 라인을 다시 찾음
    // - 라인 재획득하면 LINE_TRACE로 복귀
    // =================================================
    // [탈출 완료 조건]
    // 1) 최소 회전 시간(800ms) 경과
    //    → 노이즈로 인한 조기 복귀 방지
    // 2) 중앙 IR 센서가 라인을 감지
    //    → 실제로 라인을 다시 찾았을 때만 복귀
    case FlowState::ESCAPE:
      // [탈출 모드: 제자리 회전]
      // - 좌/우 바퀴를 반대 방향으로 구동하여 제자리 회전
      // - 장애물을 피하면서 라인을 재탐색한다.
      driveSetRaw(SPEED_ROTATE, -SPEED_ROTATE);

      if(now - escapeStart > 800 && isBlack(ir.C)){
        obstacleMode = false;   // 장애물 처리 모드 해제
        obstacleCooldown = now; // 재감지 쿨다운 시작
        state = FlowState::LINE_TRACE;
      } 
      break;

    // =================================================
    // 7) ENDING
    // - 도착/이탈 등 종료 상태
    // - 모터 정지 유지
    // =================================================
    case FlowState::ENDING:
      ending.update(LOOP_PERIOD_MS);

      if(ending.isFinished()){
        driveStop();   // 기존 함수 사용
      }
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