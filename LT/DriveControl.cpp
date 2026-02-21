//====================================================
// 작업자 : 임진효(최초)
// 최신화 일자 : 2026_02_20
// 용도 : 모터 제어 및 센서 읽기 구현부 (하드웨어 레벨)
// 설명 :
//   - L298N 모터 드라이버를 통해 좌/우 모터 제어
//   - IR 센서 값 읽기
//   - 초음파 센서 거리 측정
//   - 상태머신(main.ino)에서 호출하는 저수준 API 제공
//====================================================

#include "Robot.h"

/*
  =====================================================
  [내부 전용 함수] setMotor
  -----------------------------------------------------
  역할:
    - L298N 한 채널(모터 1개 그룹)을 직접 제어
    - 방향(IN1/IN2) + 속도(PWM EN 핀)를 동시에 설정

  매개변수:
    in1, in2 : L298N 방향 제어 핀
    en       : L298N Enable(PWM) 핀
    speed    : -255 ~ 255
               + 값 → 전진 방향
               - 값 → 후진 방향
               0    → 정지

  주의:
    - 좌우 모터는 driveSetRaw()를 통해 이 함수를 간접 호출한다.
  =====================================================
*/
static void setMotor(uint8_t in1, uint8_t in2, uint8_t en, int speed){
  speed = constrain(speed,-255,255);  // 속도 범위 제한

  if(speed > 0){
    // 전진
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    analogWrite(en,speed);
  }
  else if(speed < 0){
    // 후진
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(en,-speed);
  }
  else{
    // 정지 (브레이크)
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    analogWrite(en,0);
  }
}

/*
  =====================================================
  driveInit()
  -----------------------------------------------------
  역할:
    - 모터 관련 핀을 OUTPUT으로 설정
    - 초기 상태에서 모터를 정지 상태로 만든다.

  호출 위치:
    - setup()에서 1회 호출
  =====================================================
*/
void driveInit(){
  pinMode(RightMotor_1_pin,OUTPUT);
  pinMode(RightMotor_2_pin,OUTPUT);
  pinMode(RightMotor_E_pin,OUTPUT);

  pinMode(LeftMotor_3_pin,OUTPUT);
  pinMode(LeftMotor_4_pin,OUTPUT);
  pinMode(LeftMotor_E_pin,OUTPUT);

  driveStop();  // 안전하게 정지 상태로 시작
}

/*
  =====================================================
  driveStop()
  -----------------------------------------------------
  역할:
    - 좌/우 모터를 모두 정지시킨다.
  =====================================================
*/
void driveStop(){
  driveSetRaw(0,0);
}

/*
  =====================================================
  driveSetRaw(rightSpeed, leftSpeed)
  -----------------------------------------------------
  역할:
    - 좌/우 모터 속도를 직접 지정
    - 차동주행(속도 차이)을 통해 방향 제어

  매개변수:
    rightSpeed : -255 ~ 255
    leftSpeed  : -255 ~ 255

  사용 예:
    driveSetRaw(120,120);     → 직진
    driveSetRaw(150,-150);    → 제자리 회전
    driveSetRaw(140,100);     → 좌측 회전
  =====================================================
*/
void driveSetRaw(int rightSpeed, int leftSpeed){
  setMotor(RightMotor_1_pin,RightMotor_2_pin,RightMotor_E_pin,rightSpeed);
  setMotor(LeftMotor_3_pin,LeftMotor_4_pin,LeftMotor_E_pin,leftSpeed);
}

/*
  =====================================================
  driveLineFollow()
  -----------------------------------------------------
  역할:
    - IR 센서 3개(L/C/R)를 기반으로 기본 라인 추적 수행
    - 중앙이 검정이면 직진
    - 좌측이 검정이면 좌 보정
    - 우측이 검정이면 우 보정
    - 모두 흰색이면 정지(라인 유실)

  매개변수:
    ir        : IR 센서 샘플 구조체
    baseSpeed : 기본 주행 속도 (예: 120)

  주의:
    - 정밀 제어(P제어 등)는 현재 적용되지 않음
    - 단순한 if-else 기반 보정 방식
  =====================================================
*/
void driveLineFollow(const IRSample& ir, uint8_t baseSpeed){

  if(isBlack(ir.C)){
    // 중앙이 라인 위 → 직진
    driveSetRaw(baseSpeed,baseSpeed);
  }
  else if (isBlack(ir.C)&&isBlack(ir.L)){
    driveSetRaw(baseSpeed,baseSpeed-20);
  }
  else if (isBlack(ir.C)&&isBlack(ir.R)){
    driveSetRaw(baseSpeed-20,baseSpeed);
  }
  else if(isBlack(ir.L)){
    // 왼쪽이 라인 위 → 왼쪽으로 붙어있음 → 우측 더 빠르게
    driveSetRaw(baseSpeed+40, baseSpeed-40);
  }
  else if(isBlack(ir.R)){
    // 오른쪽이 라인 위 → 오른쪽으로 붙어있음 → 좌측 더 빠르게
    driveSetRaw(baseSpeed-40, baseSpeed+40);
  }
  else{
    // 라인 없음 → 정지 (탐색은 main에서 처리)
    driveStop();
  }
}

/*
  =====================================================
  readIR()
  -----------------------------------------------------
  역할:
    - IR 라인센서 3개의 현재 디지털 값을 읽어 반환

  반환:
    IRSample 구조체
      L : 왼쪽 센서
      C : 중앙 센서
      R : 오른쪽 센서

  판정 기준:
    LINE_BLACK = LOW (Robot.h에서 정의)
  =====================================================
*/
IRSample readIR(){
  IRSample s;
  s.L = digitalRead(L_Line);
  s.C = digitalRead(C_Line);
  s.R = digitalRead(R_Line);
  return s;
}

/*
  =====================================================
  readUltrasonicCm()
  -----------------------------------------------------
  역할:
    - 초음파 센서(trig/echo)로 거리(cm) 측정

  동작 과정:
    1. trig 핀에 10us HIGH 펄스 출력
    2. echo 핀에서 HIGH 유지 시간 측정
    3. duration/58 → cm 변환

  매개변수:
    timeoutUs : echo 대기 최대 시간(us)

  반환:
    거리(cm)
    -1 : timeout(신호 없음)
  =====================================================
*/
long readUltrasonicCm(uint16_t timeoutUs){
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  long duration = pulseIn(echoPin,HIGH,timeoutUs);

  if(duration==0) return -1; // 측정 실패

  return duration/58; // us → cm 변환
}