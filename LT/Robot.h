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

#define LED_PIN 2
#define BUZZER_PIN 7

// ---------------- 라인 기준 ----------------
constexpr uint8_t LINE_BLACK = HIGH;
inline bool isBlack(uint8_t v){ return v == LINE_BLACK; }
inline bool isWhite(uint8_t v){ return v != LINE_BLACK; }

struct IRSample{
  uint8_t L;
  uint8_t C;
  uint8_t R;
};

// ---------------- 속도 상수 ----------------
constexpr uint8_t SPEED_BASE = 100;
constexpr uint8_t SPEED_SLOW = 90;
constexpr uint8_t SPEED_ROTATE = 150;

// ---------------- 상태 ----------------
enum class FlowState{
  WAIT_START, 
  LINE_TRACE, 
  SEARCH_ROTATE,
  SEARCH_SPIRAL,
  OBSTACLE,
  STOP_HOLD,
  ESCAPE,
  ENDING
};

// ---------------- DriveControl API ----------------
void driveInit();
void driveStop();
void driveSetRaw(int rightSpeed, int leftSpeed);
void driveLineFollow(const IRSample& ir, uint8_t baseSpeed);

IRSample readIR();
long readUltrasonicCm(uint16_t timeoutUs = 25000);

void spiral_search(int L, int C, int R);  // LineSearch에서 구현

bool isObstacleStable(long dist);   // Obstacle.cpp에서 구현
#endif