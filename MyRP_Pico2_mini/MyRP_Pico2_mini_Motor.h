#ifndef MYRP_PICO2_MINI_MOTOR_H
#define MYRP_PICO2_MINI_MOTOR_H

#include "BatteryMonitor.h"

#define PWMA 19
#define AIN1 20
#define AIN2 21
#define PWMB 6
#define BIN1 8
#define BIN2 7

const float VMAX = 12.6f;
const float VMIN = 7.4f;
const float VNOM = 11.55f;
bool DC_Motors = true;

BatteryMonitor bat;

inline void updateBattery() { bat.update(); }
inline float getBatteryVoltage() { return bat.getVoltage(); }

float scale = 1.0f;

void set_Freq(String type) {
  if (type == "Coreless_Motors" || type == "coreless") {
    DC_Motors = false;
    analogWriteFreq(20000);
    Serial.println("Motor: Coreless_Motors (20kHz)");
  } else {
    DC_Motors = true;
    analogWriteFreq(1000);
    Serial.println("Motor: DC_Motors (1kHz)");
  }
}

void bat_control() {
  float voltage = getBatteryVoltage();

  if (voltage > 0.5f && voltage >= VMIN && voltage <= VMAX) {
    scale = pow(VNOM / voltage, 0.95f);
  } else {
    scale = 1.0f;
  }

  // ป้องกัน scale เกินสำหรับ TB6612FNG (พิกัดกระแสต่ำกว่า VNH7070ASTR มาก จึงจำกัดช่วงให้แคบลง)
  scale = constrain(scale, 0.95f, 1.05f);
}

void Motor(int left, int right) {
  updateBattery();
  static unsigned long lastBatUpdate = 0;
  unsigned long now = millis();

  if (now - lastBatUpdate > 70) {
    bat_control();
    lastBatUpdate = now;
  }
  int pwmL = map(abs(left), 0, 100, 0, 4095);
  int pwmR = map(abs(right), 0, 100, 0, 4095);

  pwmL = constrain((int)(pwmL * scale), 0, 3850);   // Safety limit
  pwmR = constrain((int)(pwmR * scale), 0, 3850);

  // LEFT MOTOR
  if (left > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (left < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmL = 0;
  }

  // RIGHT MOTOR
  if (right > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (right < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmR = 0;
  }

  analogWrite(PWMA, pwmL);
  analogWrite(PWMB, pwmR);
}

void Move(int l, int r, int t) {
  Motor(l, r);
  delay(t);
}

void MotorStop() {
  Motor(0, 0);
}

void MotorStop(int t) {
  Motor(0, 0);
  Beep(t);
}

// Active short-brake: ทั้ง IN1/IN2 เป็น HIGH -> มอเตอร์ลัดวงจรผ่าน back-EMF หยุดกะทันหัน
// power = แรงเบรก 0-100%, t = ระยะเวลาที่คงการเบรก (ms) แล้วปล่อยกลับสู่ MotorStop() ให้อัตโนมัติ
// จำกัดเวลา + จำกัด PWM + ใส่ cooldown กันเรียกรัวถี่ ๆ เพื่อไม่ให้กระแสค้างจนไดร์ฟมอเตอร์ร้อนพัง
void MotorShot(int t = 3, int power = 90) {
  static unsigned long lastShot = 0;
  unsigned long now = millis();
  if (now - lastShot < 50) return;  // cooldown กันเบรกซ้อนถี่เกินไป
  lastShot = now;

  updateBattery();
  bat_control();
  int pwm = map(constrain(power, 0, 100), 0, 100, 0, 4095);
  pwm = constrain((int)(pwm * scale), 0, 3850);  // Safety limit เดียวกับ Motor()

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);

  delay(constrain(t, 1, 15));  // จำกัดเวลาเบรกสูงสุด กันค้างที่กระแสสูง
  MotorStop();
}

int BaseSpeed, LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
int L[10], R[10];
int BL[10], BR[10];
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];

// ดัชนีตารางความเร็ว (ใช้กับ setBalanceSpeed/Set_KP_KD ฯลฯ)
#define SPD_10 0
#define SPD_20 1
#define SPD_30 2
#define SPD_40 3
#define SPD_50 4
#define SPD_60 5
#define SPD_70 6
#define SPD_80 7
#define SPD_90 8
#define SPD_100 9

void setBalanceSpeed(int ch, int spdL, int spdR) {
  L[ch] = spdL;
  R[ch] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  BL[ch] = spdL;
  BR[ch] = spdR;
}

void Set_KP_KD(int ch, float kp, float kd) {
  KP[ch] = kp;
  KD[ch] = kd;
}

void Set_KP_KD_Back(int ch, float kp, float kd) {
  KP_Back[ch] = kp;
  KD_Back[ch] = kd;
}

// เลือกชุดค่าความเร็ว/PID ตามช่วงของ BaseSpeed (ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40)
// เดิมเป็น if/else 10 ชุดที่โครงสร้างเหมือนกันทุกอัน ต่างแค่ index ตาราง จึงรวมเป็นสูตรเดียว:
// ดัชนี 0-9 คำนวณจาก (BaseSpeed - 1) / 10 แล้วจำกัดไม่ให้เกิน SPD_100 (ผลลัพธ์เหมือนเดิมทุกกรณี)
void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);

  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];
  PID_KP_Front = KP[idx];       // forward PID
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];   // backward PID
  PID_KD_Back = KD_Back[idx];
}

void fd(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(LeftBaseSpeed, RightBaseSpeed, time_ms);
}

void bk(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(-BackLeftBaseSpeed, -BackRightBaseSpeed, time_ms);
}

void sl(int Speed, int time_ms) {
  MotorStop(0);
  Move(-Speed, Speed, time_ms);
  MotorStop(0);
}

void sr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, -Speed, time_ms);
  MotorStop(0);
}

void tl(int Speed, int time_ms) {
  MotorStop(0);
  Move(0, Speed, time_ms);
  MotorStop(0);
}

void tr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, 0, time_ms);
  MotorStop(0);
}

#endif // MYRP_PICO2_MINI_MOTOR_H
