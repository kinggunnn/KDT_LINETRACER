#include "Robot.h"

//====================================================
// 작업자 : 이서범
// 최신화 일자 : 2026_02_22
// 용도 : 조향 방향 결정
// 함수 기능 : 
        // - IR 센서 3개(L/C/R)를 기반으로 기본 라인 추적 수행
        // - 중앙이 검정이면 직진
        // - 좌측이 검정이면 좌 보정
        // - 우측이 검정이면 우 보정
        // - 모두 흰색이면 정지(라인 유실)
// 매개변수 : IR 센서값(구조체), 기본 설정 스피드
// return 값 : 없음(바로 조향 진행)
//====================================================
void driveLineFollow_detail(const IRSample& ir, uint8_t baseSpeed) { // &주소로 값을 받되, const로 수정은 불가하게 작성
  bool Lb = isBlack(ir.L);
  bool Cb = isBlack(ir.C);
  bool Rb = isBlack(ir.R);

  // baseSpeed = 150
  // 직진 : 010
  if (!Lb && Cb && !Rb) { 
    driveSetRaw(baseSpeed, baseSpeed);
  } 
  // 약한 좌회전 : 110
  else if (Lb && Cb && !Rb) {
    driveSetRaw(baseSpeed-20, baseSpeed+20);
  }
  // 약한 우회전 : 011
  else if (!Lb && Cb && Rb) {
    driveSetRaw(baseSpeed+20, baseSpeed-20);
  }
  // 강한 좌회전 : 100
  else if (Lb && !Cb && !Rb) {
    driveSetRaw(baseSpeed-70, baseSpeed+70);
  }
  // 강한 우회전 : 001
  else if (!Lb && !Cb && Rb) {
    driveSetRaw(baseSpeed+70, baseSpeed-70);
  }
  // 교차로 혹은 두꺼운 라인 : 111 
  else if (Lb && Cb && Rb) {
    driveSetRaw(baseSpeed-70, baseSpeed+70); // 강한 좌회전
  }
  // 검출 안됨 : 000
  // 검출 안되는 상황은 main 안에서 처리됨 -> 이전 속도로 출력해야함
}


//====================================================
// 작업자 : 김효리/이서범
// 최신화 일자 : 2026_00_00
// 용도 : 
// 함수 기능 : 
// 매개변수 :
// return 값 : 
//====================================================