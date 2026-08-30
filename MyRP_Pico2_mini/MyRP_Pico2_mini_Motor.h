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
bool batteryUsed = false;

void set_Freq(String type) {
  if (type == "Coreless_Motors" || type == "coreless") {
    DC_Motors = false;
    analogWriteFreq(20000);
    Serial.println("Motor: Coreless_Motors (25kHz)");
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
    batteryUsed = true;
  } else {
    scale = 1.0f;
    batteryUsed = false;
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
  // delay(t);
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
// int LastError_F, LastError_B;
int L[10], R[10];
int BL[10], BR[10];
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];

// ��˹� index ���ӧ���
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

void InitialSpeed() {
  //  LastError_F   = 3500, LastError_B = 3500;
  if (BaseSpeed <= 10) {
    LeftBaseSpeed = BaseSpeed - L[SPD_10];
    RightBaseSpeed = BaseSpeed - R[SPD_10];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_10];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_10];
    PID_KP_Front = KP[SPD_10];  //forward PID
    PID_KD_Front = KD[SPD_10];

    PID_KP_Back = KP_Back[SPD_10];  //backward PID
    PID_KD_Back = KD_Back[SPD_10];

  } else if (BaseSpeed <= 20) {
    LeftBaseSpeed = BaseSpeed - L[SPD_20];
    RightBaseSpeed = BaseSpeed - R[SPD_20];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_20];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_20];
    PID_KP_Front = KP[SPD_20];  //forward PID
    PID_KD_Front = KD[SPD_20];

    PID_KP_Back = KP_Back[SPD_20];  //backward PID
    PID_KD_Back = KD_Back[SPD_20];

  } else if (BaseSpeed <= 30) {
    LeftBaseSpeed = BaseSpeed - L[SPD_30];
    RightBaseSpeed = BaseSpeed - R[SPD_30];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_30];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_30];
    PID_KP_Front = KP[SPD_30];  //forward PID
    PID_KD_Front = KD[SPD_30];

    PID_KP_Back = KP_Back[SPD_30];  //backward PID
    PID_KD_Back = KD_Back[SPD_30];

  } else if (BaseSpeed <= 40) {
    LeftBaseSpeed = BaseSpeed - L[SPD_40];
    RightBaseSpeed = BaseSpeed - R[SPD_40];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_40];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_40];
    PID_KP_Front = KP[SPD_40];  //forward PID
    PID_KD_Front = KD[SPD_40];  //11

    PID_KP_Back = KP_Back[SPD_40];  //backward PID
    PID_KD_Back = KD_Back[SPD_40];

  } else if (BaseSpeed <= 50) {
    LeftBaseSpeed = BaseSpeed - L[SPD_50];
    RightBaseSpeed = BaseSpeed - R[SPD_50];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_50];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_50];
    PID_KP_Front = KP[SPD_50];  //forward PID
    PID_KD_Front = KD[SPD_50]; //14

    PID_KP_Back = KP_Back[SPD_50];  //backward PID
    PID_KD_Back = KD_Back[SPD_50];

  } else if (BaseSpeed <= 60) {
    LeftBaseSpeed = BaseSpeed - L[SPD_60];
    RightBaseSpeed = BaseSpeed - R[SPD_60];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_60];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_60];
    PID_KP_Front = KP[SPD_60];  //forward PID
    PID_KD_Front = KD[SPD_60];//17

    PID_KP_Back = KP_Back[SPD_60];  //backward PID
    PID_KD_Back = KD_Back[SPD_60];

  } else if (BaseSpeed <= 70) {
    LeftBaseSpeed = BaseSpeed - L[SPD_70];
    RightBaseSpeed = BaseSpeed - R[SPD_70];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_70];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_70];
    PID_KP_Front = KP[SPD_70];  //forward PID
    PID_KD_Front = KD[SPD_70];//20

    PID_KP_Back = KP_Back[SPD_70];  //backward PID
    PID_KD_Back = KD_Back[SPD_70];

  } else if (BaseSpeed <= 80) {
    LeftBaseSpeed = BaseSpeed - L[SPD_80];
    RightBaseSpeed = BaseSpeed - R[SPD_80];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_80];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_80];
    PID_KP_Front = KP[SPD_80];  //forward PID
    PID_KD_Front = KD[SPD_80]; //20

    PID_KP_Back = KP_Back[SPD_80];  //backward PID
    PID_KD_Back = KD_Back[SPD_80];

  } else if (BaseSpeed <= 90) {
    LeftBaseSpeed = BaseSpeed - L[SPD_90];
    RightBaseSpeed = BaseSpeed - R[SPD_90];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_90];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_90];
    PID_KP_Front = KP[SPD_90];  //forward PID
    PID_KD_Front = KD[SPD_90];//22

    PID_KP_Back = KP_Back[SPD_90];  //backward PID
    PID_KD_Back = KD_Back[SPD_90];

  } else {
    LeftBaseSpeed = BaseSpeed - L[SPD_100];
    RightBaseSpeed = BaseSpeed - R[SPD_100];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_100];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_100];
    PID_KP_Front = KP[SPD_100];  //forward PID
    PID_KD_Front = KD[SPD_100];//25

    PID_KP_Back = KP_Back[SPD_100];  //backward PID
    PID_KD_Back = KD_Back[SPD_100];
  }
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
