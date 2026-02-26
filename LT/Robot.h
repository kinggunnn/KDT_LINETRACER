//====================================================
// 작업자 : 임진효
// 최신화 일자 : 2026_02_20
// 용도 : 핀 정의 + 상태 정의 + 공통 인터페이스 선언
//====================================================

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>

// ---------------- 핀 정의 ----------------
#define RightMotor_E_pin 5
#define RightMotor_1_pin 8
#define RightMotor_2_pin 9

#define LeftMotor_3_pin  10
#define LeftMotor_4_pin  11
#define LeftMotor_E_pin  6

#define trigPin 13
#define echoPin 12

#define L_Line A5
#define C_Line A4
#define R_Line A3
<<<<<<< Updated upstream
=======
#define FC_Line A2

>>>>>>> Stashed changes

// 김유진 - Ending 구현 (2026.02.22)
#define LED_PIN 4
#define BUZZER_PIN 7

// ---------------- 라인 기준 ----------------
constexpr uint8_t LINE_BLACK = HIGH;
inline bool isBlack(uint8_t v){ return v == LINE_BLACK; }
inline bool isWhite(uint8_t v){ return v != LINE_BLACK; }

struct IRSample{
  uint8_t L;
  uint8_t C;
  uint8_t R;
  uint8_t FC;
};

// 서범 : 이전값 유지하기위한
static IRSample prev_ir;      // 마지막 정상 IR
static bool hasPrevIr = false;

// ---------------- 속도 상수 ----------------
constexpr uint8_t SPEED_BASE = 110; // 빠때리풀충  : 100 좀썼다 110
constexpr uint8_t SPEED_SLOW = 80;
constexpr uint8_t SPEED_ROTATE = 110; //풀충 :115 

// ---------------- 상태 ----------------
enum class FlowState{
  WAIT_START, 
  LINE_TRACE, 
  SEARCH_ROTATE,
  SEARCH_SPIRAL,
  OBSTACLE,
  STOP_HOLD,
  ESCAPE,
  ENDING,

  CROSS_TURN_LEFT,
  CROSS_GO_STRAIGHT
};


// ===============================
// 세개씩 데이터 받아오는 보정용 구조체 선언
// 담당자 : 임진효
// ===============================
struct StableDigitalFilter {
  int lastRaw = -1;
  uint8_t stableCount = 0;
  int confirmed = -1;
};

// ===============================
// Setup용 보정 함수 선언
// 담당자 : 임진효
// ===============================
bool acquireLineInSetup(unsigned long timeoutMs);
bool calibrateTurn180InSetup(int repeatCount, unsigned long timeoutMs);
int readStableDigital(uint8_t pin, StableDigitalFilter &f, uint8_t required = 3);
unsigned long getCalibrated180();

// ---------------- DriveControl API ----------------
void driveInit();
void driveStop();
void driveSetRaw(int rightSpeed, int leftSpeed);
void driveLineFollow(const IRSample& ir, uint8_t baseSpeed); // 기본 직/좌/우 구현
void driveLineFollow_detail(const IRSample& ir, uint8_t baseSpeed); // 세분화된 좌좌/좌/직/우/우우 구현

IRSample readIR();
long readUltrasonicCm(uint16_t timeoutUs = 25000);
bool isObstacleStable(long dist);
//사각회전탐색
void square_search(int L, int C, int R);


void spiral_search(int L, int C, int R);  // LineSearch에서 구현

bool isObstacleStable(long dist);   // Obstacle.cpp에서 구현
#endif