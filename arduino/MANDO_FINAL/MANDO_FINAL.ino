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
#define STEERING_PULSE_PIN      2
#define ACCEL_PULSE_PIN         3

// ===== 엔코더 핀 =====
#define ENCODER_A               18
#define ENCODER_B               19
volatile long encoderCount = 0;

// ===== 모드 핀 =====
#define MANUAL_MODE_PIN         20
#define AUTO_MODE_PIN           21

// ===== 모드 기준값(펄스) =====
#define BREAK_MODE              200
#define MANUAL_MODE             1400
#define AUTO_MODE               1700

// ===== 스로틀/조향 신호 스케일 =====
#define TORQUE_MIN              -1
#define TORQUE_MAX              1
#define SERVO_MIN               -1
#define SERVO_MAX               1
#define SIGNAL_THRESHOLD        0.1

// ===== 조향 포텐셔미터 =====
#define POT_MAX                 980
#define POT_MIN                 120
#define MAX_STEER_TIRE_DEG      18

// ===== PID 게인 =====
#define KP                      0.5
#define KI                      0.000
#define KD                      0.00

// ===== 공유 변수(ISR에서 갱신) =====
volatile long Steering_Edge_now_us   = DETECTION_ERR;
volatile long Steering_Edge_before_us= DETECTION_ERR;
volatile long Steering_us            = DETECTION_ERR;

volatile long Accel_Edge_now_us      = DETECTION_ERR;
volatile long Accel_Edge_before_us   = DETECTION_ERR;
volatile long Accel_us               = DETECTION_ERR;


volatile long Manual_Edge_now_us     = DETECTION_ERR;
volatile long Manual_Edge_before_us  = DETECTION_ERR;
volatile long Manual_us              = DETECTION_ERR;

volatile long Auto_Edge_now_us       = DETECTION_ERR;
volatile long Auto_Edge_before_us    = DETECTION_ERR;
volatile long Auto_us                = DETECTION_ERR;

// ===== 모터 핀 =====
int DIR1 = 4;
int PWM1 = 5;
int DIR2 = 6;
int PWM2 = 7;
int DIR3 = 8;
int PWM3 = 9;

// ===== 포텐셔미터 =====
int POTval = 0;
int POTPin = A0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 유틸

static inline float Mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ISR: RC 펄스 폭 측정 (간단한 상승/하강 시간차 방식)
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
        throttleFresh  = true;
        lastThrottleMs = millis();
      }
      // SA <float>
      else if (strncmp(serialBuffer, "SA", 2) == 0 || strncmp(serialBuffer, "sa", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float a = atof(p);
        if (a >  MAX_STEER_TIRE_DEG) a =  MAX_STEER_TIRE_DEG;
        if (a < -MAX_STEER_TIRE_DEG) a = -MAX_STEER_TIRE_DEG;
        steer_cmd_deg = a;
        steerFresh    = true;
        lastSteerMs   = millis();
        // 조향 목표를 점프시킬 때는 윈드업 방지를 위해 PID 리셋 권장
        // (이 스케치는 내부 PID 상태가 static이므로 루프에서 각도/타임스텝으로 자연완화됨)
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


void SteeringPulseInt() {
  long now = micros();
  Steering_us = now - Steering_Edge_before_us;
  Steering_Edge_before_us = now;
}

void AccelPulseInt() {
  long now = micros();
  Accel_us = now - Accel_Edge_before_us;
  Accel_Edge_before_us = now;
}

void ManualPulseInt() {
  long now = micros();
  Manual_us = now - Manual_Edge_before_us;
  Manual_Edge_before_us = now;
}

void AutoPulseInt() {
  long now = micros();
  Auto_us = now - Auto_Edge_before_us;
  Auto_Edge_before_us = now;
}

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
// 구동 함수

void StopMotor() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
  digitalWrite(DIR3, HIGH);
  analogWrite(PWM3, 0);
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
  Serial.begin(57600);

  // RC 입력
  pinMode(STEERING_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);

  pinMode(ACCEL_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);

  pinMode(MANUAL_MODE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MANUAL_MODE_PIN), ManualPulseInt, CHANGE);

  pinMode(AUTO_MODE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(AUTO_MODE_PIN), AutoPulseInt, CHANGE);

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
  // === 시간 계산(µs) ===
  static unsigned long prev_t_us = 0;
  unsigned long t_us = micros();
  unsigned long dt_us = (prev_t_us == 0) ? 1000UL : (t_us - prev_t_us);
  prev_t_us = t_us;

  // === 공유 변수 스냅샷(원자적 복사) ===
  long Steering_us_local, Accel_us_local, Manual_us_local, Auto_us_local;
  long encoderCount_local;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    Steering_us_local = Steering_us;
    Accel_us_local    = Accel_us;
    Manual_us_local   = Manual_us;
    Auto_us_local     = Auto_us;
    encoderCount_local= encoderCount;
  }

  // === 기본값(안전) ===
  int Steering_val = 1500;
  int Accel_val    = 1500;
  int Mode_val     = BREAK_MODE;

  // === 유효 범위 내면 적용 ===
  if ((Steering_us_local > PULSE_MIN) && (Steering_us_local < PULSE_MAX)) {
    Steering_val = Steering_us_local;
  }
  if ((Accel_us_local > PULSE_MIN) && (Accel_us_local < PULSE_MAX)) {
    Accel_val = Accel_us_local;
  }

  // === 모드 판정 ===
  if ((Manual_us_local > 1900 && Auto_us_local <= 1100)) {
    Mode_val = MANUAL_MODE;
  } 
  else if ((Auto_us_local >= 1900 && Manual_us_local <= 1100)) {
    Mode_val = AUTO_MODE;
  } 
  else {
    Mode_val = BREAK_MODE;
  }

  // === 맵핑 ===
  float Throttle_input = Mapping(Accel_val, 984.0f, 1972.0f, -1.0f, 1.0f);
  if (Accel_val >= 1470 && Accel_val <= 1480) Throttle_input = 0.0f;

  float Steer_input = Mapping(Steering_val, 992.2f, 1964.0f, -1.0f, 1.0f);
  double ref_steer_deg = Mapping(Steer_input, -1.0f, 1.0f, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  POTval = analogRead(POTPin);
  double deg = Mapping(POTval, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  // === PID ===
  double u = PID(ref_steer_deg, deg, dt_us);
  u = (u > 1.0) ? 1.0 : ((u < -1.0) ? -1.0 : u);

  // === 동작 모드 ===
  if (Mode_val == BREAK_MODE) {
    StopMotor();
  }
  else if (Mode_val == MANUAL_MODE) {
    if (Throttle_input > 0.0) {
      MoveForward(Throttle_input);
    } else if (Throttle_input < 0.0) {
      MoveBackward(-Throttle_input);
    } else {
      // 중립 스로틀 시 구동 모터 정지
      digitalWrite(DIR1, HIGH); analogWrite(PWM1, 0);
      digitalWrite(DIR2, HIGH); analogWrite(PWM2, 0);
    }
    Steer(-u);
  }
  else if (Mode_val == AUTO_MODE) {
    // 요청대로 AUTO 모드는 정지
    StopMotor();
  }

  // === 디버그(엔코더 + POT) ===
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Encoder Count: "); Serial.print(encoderCount_local);
    Serial.print(" | POT: ");        Serial.print(POTval);
    Serial.print(" | deg: ");        Serial.print(deg, 2);
    Serial.print(" | ref: ");        Serial.print(ref_steer_deg, 2);
    Serial.print(" | u: ");          Serial.println(u, 3);
  }

}
