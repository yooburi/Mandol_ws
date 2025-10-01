  #include <Arduino.h>
#include <math.h>
#include <util/atomic.h>   // ★ AVR 원자적 블록

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// (ROS 연동 코드 삭제됨)

// ===== 펄스 범위/에러 =====
#define PULSE_MAX               2200
#define PULSE_MIN               800
#define DETECTION_ERR           -1

// ===== RC 입력 핀 =====
// #define STEERING_PULSE_PIN      2
// #define ACCEL_PULSE_PIN         3

// ===== 엔코더 핀 =====
#define ENCODER_A               18
#define ENCODER_B               19
volatile long encoderCount = 0;

// ===== 모드 핀 =====
// #define MANUAL_MODE_PIN         20
// #define AUTO_MODE_PIN           21

// // ===== 모드 기준값(펄스) =====
// #define BREAK_MODE              200
// #define MANUAL_MODE             1400
// #define AUTO_MODE               1700

// ===== 신호 스케일 =====
#define SIGNAL_THRESHOLD        0.05

// ===== 조향 포텐셔미터 =====
#define POT_MAX                 978
#define POT_MIN                 34
#define MAX_STEER_TIRE_DEG      20

// ===== PID 게인 =====
#define KP                      0.8
#define KI                      0.000
#define KD                      0.00

// ===== Dead Band 설정(센서/출력) =====
#define STEER_DEADBAND_DEG      1.0    // 포텐셔미터 각도 데드밴드(±도)
#define PID_DEADBAND            0.05   // PID 출력 데드밴드(±값, -1~+1 기준)

// ===== RC(리모컨) 데드밴드 =====
// 1) 펄스 중심부(µs) 데드밴드
// #define ACCEL_CENTER_US         1500
// #define STEER_CENTER_US         1500
// #define ACCEL_DB_US             25     // ±25us
// #define STEER_DB_US             25

// 2) 정규화(-1..+1) 데드밴드
#define RC_THROTTLE_DB_NORM     0.08   // 스로틀 소신호 무시
#define RC_STEER_DB_NORM        0.10   // 스티어 소신호 무시

// ===== 공유 변수(ISR에서 갱신) =====
// volatile long Steering_Edge_before_us= DETECTION_ERR;
// volatile long Steering_us            = DETECTION_ERR;

// volatile long Accel_Edge_before_us   = DETECTION_ERR;
// volatile long Accel_us               = DETECTION_ERR;

// volatile long Manual_Edge_before_us  = DETECTION_ERR;
// volatile long Manual_us              = DETECTION_ERR;

// volatile long Auto_Edge_before_us    = DETECTION_ERR;
// volatile long Auto_us                = DETECTION_ERR;

// ===== 모터 핀 =====
int DIR1 = 10;
int PWM1 = 11;
int DIR2 = 6;
int PWM2 = 7;
int DIR3 = 8;
int PWM3 = 9;

// ===== 포텐셔미터 =====
int POTval = 0;
int POTPin = A0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 시리얼 파서(TH/SA)

#define SERIAL_BUFFER_SIZE  48
char   serialBuffer[SERIAL_BUFFER_SIZE];
size_t bufferIndex = 0;

float  throttle_cmd = 0.0f;      // -1..+1 (시리얼 TH)
float  steer_auto_deg = 0.0f;    // -MAX..+MAX [deg] (시리얼 SA)

unsigned long last_serialMS = 0;
const unsigned long serialMS_TIMEOUT = 1000;
bool TIMEOUT = false;

void markFresh() {
  last_serialMS = millis();

  TIMEOUT = false;
}
// unsigned long nextDebugMs = 0;
// const unsigned long DEBUG_PERIOD_MS = 100;

void parseSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';

      // TH <float>
      if (strncmp(serialBuffer, "TH", 2) == 0 || strncmp(serialBuffer, "th", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float v = atof(p);
        if (v >  1.0f) v =  1.0f;
        if (v < -1.0f) v = -1.0f;
        throttle_cmd   = v;
        markFresh();
      }
      // SA <float>
      else if (strncmp(serialBuffer, "SA", 2) == 0 || strncmp(serialBuffer, "sa", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float a = atof(p);
        if (a >  MAX_STEER_TIRE_DEG) a =  MAX_STEER_TIRE_DEG;
        if (a < -MAX_STEER_TIRE_DEG) a = -MAX_STEER_TIRE_DEG;
        steer_auto_deg = a;
        markFresh();
      }

      bufferIndex = 0;  // 라인 종료 → 버퍼 리셋
    }
    else if (c != '\r') {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0; // 오버플로우 방지
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 유틸

static inline float Mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline double applyDeadband(double x, double band) {
  return (fabs(x) < band) ? 0.0 : x;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ISR

// void SteeringPulseInt() {
//   long now = micros();
//   Steering_us = now - Steering_Edge_before_us;
//   Steering_Edge_before_us = now;
// }

// void AccelPulseInt() {
//   long now = micros();
//   Accel_us = now - Accel_Edge_before_us;
//   Accel_Edge_before_us = now;
// }

// void ManualPulseInt() {
//   long now = micros();
//   Manual_us = now - Manual_Edge_before_us;
//   Manual_Edge_before_us = now;
// }

// void AutoPulseInt() {
//   long now = micros();
//   Auto_us = now - Auto_Edge_before_us;
//   Auto_Edge_before_us = now;
// }

void encoderISR() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderCount++;   // 예: 시계방향
  } else {
    encoderCount--;   // 예: 반시계방향
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID

double PID(double ref, double sense, unsigned long dt_us) {
  static double prev_err = 0.0;
  static double integral = 0.0;

  double dt_s = dt_us * 1.0e-6;     // ★ 초 단위
  if (dt_s <= 0.0) dt_s = 1e-6;     // 방어

  double err = ref - sense;

  integral += err * dt_s;

  double P = KP * err;
  double I = KI * integral;
  double D = KD * (err - prev_err) / dt_s;   // ★ 초 단위 미분

  prev_err = err;
  return P + I + D;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 구동 함수 */

void StopMotor() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
  digitalWrite(DIR3, HIGH);
  analogWrite(PWM3, 0);
}

void StopDriveOnly() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
}

void MoveForward(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < 0.0) throttle = 0.0;

  int in = (int)(Mapping(throttle, 0.0, 1.0, 0.0, 255.0));
  if (fabs(throttle) < SIGNAL_THRESHOLD) in = 0;

  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, in);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM2, in);
}

void MoveBackward(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < 0.0) throttle = 0.0;

  int in = (int)(Mapping(throttle, 0.0, 1.0, 0.0, 255.0));
  if (fabs(throttle) < SIGNAL_THRESHOLD) in = 0;

  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, in);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, in);
}

void Steer(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < -1.0) throttle = -1.0;

  int in = (int)(Mapping(throttle, 1.0, -1.0, -255.0, 255.0)); // 좌(+1)→우(-1) 기준
  if (fabs(throttle) < SIGNAL_THRESHOLD) in = 0;

  if (in >= 0) {
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, in/2);
  } else {
    in = -in;
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, in/2);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 셋업

void setup() {
  Serial.begin(115200);

  // RC 입력
  // pinMode(STEERING_PULSE_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);

  // pinMode(ACCEL_PULSE_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);

  // pinMode(MANUAL_MODE_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(MANUAL_MODE_PIN), ManualPulseInt, CHANGE);

  // pinMode(AUTO_MODE_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(AUTO_MODE_PIN), AutoPulseInt, CHANGE);

  // 엔코더
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // 포텐셔미터
  pinMode(POTPin, INPUT);

  // 모터
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM3, OUTPUT);

  StopMotor();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 루프

void loop() {
  // === 0) 시리얼 먼저 파싱 ===
  parseSerial();

  unsigned long nowMS = millis();
  if ((nowMS - last_serialMS) > serialMS_TIMEOUT) {
    if (!TIMEOUT) {
      StopMotor();
      throttle_cmd = 0.0f;
      steer_auto_deg = 0.0f;
      Steer(0.0);
      TIMEOUT = true;
    }
  }
  else {
    TIMEOUT = false;
  }

  if (TIMEOUT) {
    return;
  }

  // === 1) 시간 계산(µs) ===
  static unsigned long prev_t_us = 0;
  unsigned long t_us = micros();
  unsigned long dt_us = (prev_t_us == 0) ? 1000UL : (t_us - prev_t_us);
  prev_t_us = t_us;

  // === 2) 공유 변수 스냅샷(원자적 복사) ===
  long Steering_us_local, Accel_us_local, Manual_us_local, Auto_us_local;
  long encoderCount_local;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Steering_us_local = Steering_us;
    // Accel_us_local    = Accel_us;
    // Manual_us_local   = Manual_us;
    // Auto_us_local     = Auto_us;
    encoderCount_local= encoderCount;
  }

  // === 3) 기본값(안전) ===
  // int Steering_val = 1500;
  // int Accel_val    = 1500;
  // int Mode_val     = BREAK_MODE;

  // === 4) 유효 범위 내면 적용 ===
  // if ((Steering_us_local > PULSE_MIN) && (Steering_us_local < PULSE_MAX)) {
  //   Steering_val = Steering_us_local;
  // }
  // if ((Accel_us_local > PULSE_MIN) && (Accel_us_local < PULSE_MAX)) {
  //   Accel_val = Accel_us_local;
  // }

  // === 5) 모드 판정 ===
  // if ((Manual_us_local > 1900 && Auto_us_local <= 1100)) {
  //   Mode_val = MANUAL_MODE;
  // } 
  // else if ((Auto_us_local >= 1800 && Manual_us_local <= 1100)) {
  //   Mode_val = AUTO_MODE;
  // } 
  // else {
  //   Mode_val = BREAK_MODE;
  // }

  // === 6) RC → 내부 표준값 ===
  // (A) 펄스 중심부 데드밴드(µs)
  // if (abs(Accel_val   - ACCEL_CENTER_US) <= ACCEL_DB_US)   Accel_val   = ACCEL_CENTER_US;
  // if (abs(Steering_val- STEER_CENTER_US) <= STEER_DB_US)   Steering_val= STEER_CENTER_US;

  // // (B) 맵핑(-1..+1)
  // float Throttle_input = Mapping(Accel_val, 1060.0f, 1960.0f, -1.0f, 1.0f);
  // float Steer_input    = Mapping(Steering_val, 1036.0f, 2036.0f, -1.0f, 1.0f);

  // (C) 정규화 데드밴드(-1..+1)
  // Throttle_input = applyDeadband(Throttle_input, RC_THROTTLE_DB_NORM);
  // Steer_input    = applyDeadband(Steer_input,    RC_STEER_DB_NORM);

  // (D) RC 스티어 목표각도 생성
  // double ref_steer_deg_rc = Mapping(Steer_input, -1.0f, 1.0f, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  // === 7) 센서 각도 ===
  POTval = analogRead(POTPin);
  double deg = Mapping(POTval, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);
  deg = applyDeadband(deg, STEER_DEADBAND_DEG);

  // === 8) 목표각 선택 ===
  double ref_steer_deg = (double)steer_auto_deg;

  // === 9) PID ===
  // // === 10) 동작 모드 ===
  // if (Mode_val == BREAK_MODE) {
  //   StopMotor();
  // }
  // else if (Mode_val == MANUAL_MODE) {
  //   if (Throttle_input > 0.0)      MoveForward(Throttle_input);
  //   else if (Throttle_input < 0.0) MoveBackward(-Throttle_input);
  //   else                           StopDriveOnly();

  //   Steer(-u);
  // }
  // else if (Mode_val == AUTO_MODE) {
    // 스로틀: 시리얼 TH 값 사용
  float th = throttle_cmd;

  if (th > 0.0f) {
     MoveForward(th);
  } 
  else if (th < 0.0f)  {
     MoveBackward(-th);
  } 
  else {
     StopDriveOnly();
  }

    // 조향: 시리얼 SA 값 사용
  if (th < 0) {
    ref_steer_deg = -ref_steer_deg;       // ★ 후진 시 목표 조향각 부호 반전
  }

  double PID_return = PID(ref_steer_deg, deg, dt_us);
  PID_return = applyDeadband(PID_return, PID_DEADBAND);
  if (PID_return > 1.0) {
    PID_return = 1.0;
  }
  else if (PID_return < -1.0) {
    PID_return = -1.0;
  }

  Steer(-PID_return);

  Serial.print("throttle_cmd = ");
  Serial.print(throttle_cmd, 3);
  Serial.print(" SA = "); 
  Serial.print(steer_auto_deg, 3);
  Serial.println();
  
}
  // unsigned long nowMs = millis();
  // if (nowMs >= nextDebugMs) {
  //   nextDebugMs = nowMs + DEBUG_PERIOD_MS;
  //   Serial.print("Mode="); Serial.print(Mode_val); // 200=BREAK, 1400=MANUAL, 1700=AUTO
  //   Serial.print(" TH="); Serial.print(throttle_cmd, 3);
  //   Serial.print(" SA="); Serial.print(steer_auto_deg, 3);
  //   Serial.print(" deg="); Serial.print(deg, 2);
  //   Serial.print(" u=");   Serial.print(u, 2);
  //   Serial.println();
  // }
