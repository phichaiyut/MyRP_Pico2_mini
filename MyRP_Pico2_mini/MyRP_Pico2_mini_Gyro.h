#ifndef MYRP_PICO2_MINI_GYRO_H
#define MYRP_PICO2_MINI_GYRO_H

#include "my_BMI160.h"
my_BMI160 my;

//  void resetAngles();
//  float gyroZ();

// void MyRP_Pico2::resetAngles() {
//     my.resetAngles();
// }

// float MyRP_Pico2::gyroZ() {
//     return my.gyro('z');   // หรือ my.getGyroZ()
//}

float current_degree = 0;
float power_factor = 1.0;
char lastTurnDirGC = 0;  // 'L' หรือ 'R' : ทิศของ tlgc/trgc ครั้งล่าสุด ใช้กำหนดว่าครั้งถัดไปต้องหยุด+min_speed หรือไม่
char lastTurnDirGBC = 0;  // 'L' หรือ 'R' : ทิศของ tlbgc/trbgc ครั้งล่าสุด (เวอร์ชันถอยหลัง)
float previous_errorG = 0;
float previous_errorGB = 0;

/* ---------- sensor setup ---------- */

void resetAngles() {
  my.resetAngles();
}

// void setAngleOffset() {
//   robot.resetAngles();
// }

float gyroZ() {
  return my.gyro('z');
}

void SetRobotAngle() {
  current_degree = gyroZ();
}

float kpHold = 2.5;
float kdHold = 1.5;
float kpFHold = 2.5;
float kdFHold = 1.5;
float kpBHold = 2.5;
float kdBHold = 1.2;
float holdAngle = 0;
float prevErrHold = 0;
float prevErrHoldB = 0;
float prevErrHoldF = 0;

void HoldAngle() {
  float error = current_degree - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(power, -power);   // หมุนอยู่กับที่
  prevErrHold = error;
}

void HoldAngleB() {
  float error = current_degree - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(-power, power);   // หมุนอยู่กับที่
  prevErrHoldB = error;
}

void SetHoldAngle() {
  holdAngle = gyroZ();   // มุมที่ต้องการให้หุ่น "จำ"
  MotorStop();
  prevErrHoldF = 0;
}

void HoldAngleF() {
  float error = holdAngle - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldF;
  int power = (error * kpFHold) + (d * kdFHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(power, -power);   // หมุนอยู่กับที่
  prevErrHoldF = error;
}

void SetFG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleF();
  }
  BZoff();
}

void setfg(int totalTime) {
  SetFG(totalTime);
}

void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngle();
  }
  BZoff();
}

void SetGB(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleB();
  }
  BZoff();
}

/* ---------- spin / turn ---------- */

void spindegree(int Speed, int relative_degree) {
  int min_speed = 10;
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.6;  // เพิ่มจาก 0.35: ยังหมุนเกิน 90° อยู่ จึงเพิ่มแรงหน่วงตามอัตราหมุนให้มากขึ้นอีก
  float small_angle_threshold = 10;  // ลดจาก 25: ช่วงคลานที่ min_speed แคบลง วิ่งเร็วได้นานขึ้นก่อนเข้าเบรก
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;
  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -min_speed);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      Motor(pd_value, -pd_value);
    }

    previous_error = error;
  }

  //SetG(100);
}

void turndegree(int Speed, int relative_degree) {
  int min_speed = 15;
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.6;  // เพิ่มจาก 0.35: ยังหมุนเกิน 90° อยู่ จึงเพิ่มแรงหน่วงตามอัตราหมุนให้มากขึ้นอีก
  float small_angle_threshold = 5;  // ลดจาก 25: ช่วงคลานที่ min_speed แคบลง วิ่งเร็วได้นานขึ้นก่อนเข้าเบรก
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -1);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-1, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) Motor(-1, -pd_value);
      else Motor(pd_value, 1);
    }

    previous_error = error;
  }

  // SetG(10);
}

void turndegreeb(int Speed, int relative_degree) {
  int min_speed = 20;
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.6;  // เพิ่มจาก 0.35: ยังหมุนเกิน 90° อยู่ จึงเพิ่มแรงหน่วงตามอัตราหมุนให้มากขึ้นอีก
  float small_angle_threshold = 15;  // ลดจาก 25: ช่วงคลานที่ min_speed แคบลง วิ่งเร็วได้นานขึ้นก่อนเข้าเบรก
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(1, -min_speed);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 1);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) Motor(pd_value, 1);
      else Motor(-1, -pd_value);
    }

    previous_error = error;
  }

  // SetG(100);
}

// ต่อเนื่อง (chainable): ไม่ใช้ min_speed ตอนใกล้ถึงเป้า และไม่ MotorStop() เมื่อถึงมุม
// เพื่อให้เรียกต่อกันได้ลื่น เช่น tlgc(90); trgc(90);
void turndegreec(int Speed, int relative_degree) {
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.6;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error >= -stop_threshold && error <= stop_threshold) {
      break;
    } else {
      if (error <= 0) Motor(-1, -pd_value);
      else Motor(pd_value, 1);
    }

    previous_error = error;
  }
}

void turndegreec(int relative_degree) {
  turndegreec(30, relative_degree);
}

// ต่อเนื่อง (chainable) เวอร์ชันถอยหลัง: ไม่ใช้ min_speed ตอนใกล้ถึงเป้า และไม่ MotorStop() เมื่อถึงมุม
void turndegreebc(int Speed, int relative_degree) {
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.6;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error >= -stop_threshold && error <= stop_threshold) {
      break;
    } else {
      if (error <= 0) Motor(pd_value, 1);
      else Motor(-1, -pd_value);
    }

    previous_error = error;
  }
}

void turndegreebc(int relative_degree) {
  turndegreebc(30, relative_degree);
}

void spindegree(int relative_degree) {
  spindegree(30, relative_degree);
}

void turndegree(int relative_degree) {
  turndegree(30, relative_degree);
}

void turndegreeb(int relative_degree) {
  turndegreeb(30, relative_degree);
}

/* ---------- gyro-guided straight move ---------- */

float kpG = 2.5;
float kdG = 1.5;
float kpGB = 2.5;
float kdGB = 1.5;

void RunG(int SpeedL, int SpeedR) {
  float error = current_degree - gyroZ();

  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;

  float derivative = error - previous_errorG;
  int pd_value = (int)((error * kpG) + (derivative * kdG));
  int leftPow = SpeedL + pd_value;
  int rightPow = SpeedR - pd_value;

  if (leftPow > SpeedL) leftPow = SpeedL;
  if (leftPow < 0) leftPow = 0;
  if (rightPow > SpeedR) rightPow = SpeedR;
  if (rightPow < 0) rightPow = 0;

  Motor(leftPow, rightPow);
  previous_errorG = error;
}

void RunGB(int SpeedL, int SpeedR) {
  float error = current_degree - gyroZ();
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  float derivative = error - previous_errorGB;
  int pd_value = (int)((error * kpGB) + (derivative * kdGB));
  int leftPow = SpeedL - pd_value;
  int rightPow = SpeedR + pd_value;

  if (leftPow > SpeedL) leftPow = SpeedL;
  if (leftPow < 0) leftPow = 0;
  if (rightPow > SpeedR) rightPow = SpeedR;
  if (rightPow < 0) rightPow = 0;

  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
}

void fftimerg(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
  }
}
void bbtimerg(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
  }
}

/* ---------- distance motion (gyro straight) ---------- */

void ffcmgs(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.65;

  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunG(LeftBaseSpeed, RightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
  //Motor(0, 0);
}

void ffcmg(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  float speed_scale = 0.99;
  bool enableRamp = (distance_cm >= 30.0);

  if (!enableRamp) {
    speed_scale = 1.5;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.7f) break;

    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    RunG(target_speed, target_speed);
  }
}

void bbcmgs(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.5;

  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
  Motor(0, 0);
}

void bbcmg(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  float speed_scale = 0.99;
  bool enableRamp = (distance_cm >= 30.0);

  if (!enableRamp) {
    speed_scale = 1.5;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.8f) break;

    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    RunGB(target_speed, target_speed);
  }
}

/* ---------- spin / turn helpers ---------- */

void spinlg(int Angle) {spindegree(-abs(Angle));}

void spinrg(int Angle) {spindegree(abs(Angle));}

void turnlg(int Angle) {turndegree(-abs(Angle));}

void turnrg(int Angle) {turndegree(abs(Angle));}

void turnlbg(int Angle) {turndegreeb(abs(Angle));}

void turnrbg(int Angle) {turndegreeb(-abs(Angle));}

void spinlg(int spd, int Angle) {spindegree(spd, -abs(Angle));}

void spinrg(int spd, int Angle) {spindegree(spd, abs(Angle));}

void turnlg(int spd, int Angle) {turndegree(spd, -abs(Angle));}

void turnrg(int spd, int Angle) {turndegree(spd, abs(Angle));}

void turnlbg(int spd, int Angle) {turndegreeb(spd, abs(Angle));}

void turnrbg(int spd, int Angle) {turndegreeb(spd, -abs(Angle));}

void slg(int Angle) {spindegree(-abs(Angle));}

void srg(int Angle) {spindegree(abs(Angle));}

void tlg(int Angle) {turndegree(-abs(Angle));}

void trg(int Angle) {turndegree(abs(Angle));}

void tlgc(int Angle) {
  if (lastTurnDirGC == 'R') turndegree(-abs(Angle));   // สลับทิศ: หยุด + min_speed
  else turndegreec(-abs(Angle));                       // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGC = 'L';
}

void trgc(int Angle) {
  if (lastTurnDirGC == 'L') turndegree(abs(Angle));    // สลับทิศ: หยุด + min_speed
  else turndegreec(abs(Angle));                        // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGC = 'R';
}

void tlbg(int Angle) {turndegreeb(abs(Angle));}

void trbg(int Angle) {turndegreeb(-abs(Angle));}

void tlbgc(int Angle) {
  if (lastTurnDirGBC == 'R') turndegreeb(abs(Angle));    // สลับทิศ: หยุด + min_speed
  else turndegreebc(abs(Angle));                         // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGBC = 'L';
}

void trbgc(int Angle) {
  if (lastTurnDirGBC == 'L') turndegreeb(-abs(Angle));   // สลับทิศ: หยุด + min_speed
  else turndegreebc(-abs(Angle));                        // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGBC = 'R';
}

void slg(int spd, int Angle) {spindegree(spd, -abs(Angle));}

void srg(int spd, int Angle) {spindegree(spd, abs(Angle));}

void tlg(int spd, int Angle) {turndegree(spd, -abs(Angle));}

void trg(int spd, int Angle) {turndegree(spd, abs(Angle));}

void tlgc(int spd, int Angle) {
  if (lastTurnDirGC == 'R') turndegree(spd, -abs(Angle));   // สลับทิศ: หยุด + min_speed
  else turndegreec(spd, -abs(Angle));                       // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGC = 'L';
}

void trgc(int spd, int Angle) {
  if (lastTurnDirGC == 'L') turndegree(spd, abs(Angle));    // สลับทิศ: หยุด + min_speed
  else turndegreec(spd, abs(Angle));                        // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGC = 'R';
}

// เลี้ยวซ้ายต่อเนื่องแล้วต่อขวา ในคำสั่งเดียว: ขาซ้ายไหลลื่น ขาขวาหยุด+min_speed
void tlrgc(int Angle1, int Angle2) {
  tlgc(Angle1);
  trgc(Angle2);
}

void tlrgc(int spd, int Angle1, int Angle2) {
  tlgc(spd, Angle1);
  trgc(spd, Angle2);
}

// เลี้ยวขวาต่อเนื่องแล้วต่อซ้าย ในคำสั่งเดียว: ขาขวาไหลลื่น ขาซ้ายหยุด+min_speed
void trlgc(int Angle1, int Angle2) {
  trgc(Angle1);
  tlgc(Angle2);
}

void trlgc(int spd, int Angle1, int Angle2) {
  trgc(spd, Angle1);
  tlgc(spd, Angle2);
}

void tlbg(int spd, int Angle) {turndegreeb(spd, abs(Angle));}

void trbg(int spd, int Angle) {turndegreeb(spd, -abs(Angle));}

void tlbgc(int spd, int Angle) {
  if (lastTurnDirGBC == 'R') turndegreeb(spd, abs(Angle));    // สลับทิศ: หยุด + min_speed
  else turndegreebc(spd, abs(Angle));                         // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGBC = 'L';
}

void trbgc(int spd, int Angle) {
  if (lastTurnDirGBC == 'L') turndegreeb(spd, -abs(Angle));   // สลับทิศ: หยุด + min_speed
  else turndegreebc(spd, -abs(Angle));                        // ทิศเดิม/ครั้งแรก: ต่อเนื่อง
  lastTurnDirGBC = 'R';
}

// เลี้ยวซ้ายถอยหลังต่อเนื่องแล้วต่อขวา ในคำสั่งเดียว: ขาซ้ายไหลลื่น ขาขวาหยุด+min_speed
void tlrbgc(int Angle1, int Angle2) {
  tlbgc(Angle1);
  trbgc(Angle2);
}

void tlrbgc(int spd, int Angle1, int Angle2) {
  tlbgc(spd, Angle1);
  trbgc(spd, Angle2);
}

// เลี้ยวขวาถอยหลังต่อเนื่องแล้วต่อซ้าย ในคำสั่งเดียว: ขาขวาไหลลื่น ขาซ้ายหยุด+min_speed
void trlbgc(int Angle1, int Angle2) {
  trbgc(Angle1);
  tlbgc(Angle2);
}

void trlbgc(int spd, int Angle1, int Angle2) {
  trbgc(spd, Angle1);
  tlbgc(spd, Angle2);
}

void ToCenterLG() {
  BZon();
  for (int i = 0; i < 20; i++) {
    RunG(tctL, tctR);
  }
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC) {
      Motor(-tct, -tct);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterRG() {
  BZon();
  for (int i = 0; i < 20; i++) {
    RunG(tctL, tctR);
  }
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCR] >= RefC) {
      Motor(-tct, -tct);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterLRG() {
  BZon();
  for (int i = 0; i < 20; i++) {
    RunG(tctL, tctR);
  }
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tct, -tct);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void BackCenterG() {
  BZon();
  for (int i = 0; i < 20; i++) {
    RunGB(bctL, bctR);
  }
  while (1) {
    RunGB(bctL, bctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

/* ---------- track select (gyro) ---------- */

void TrackSelectG(int spd, char select) {
  if (select == 'L') {
    spindegree(-90);
  } else if (select == 'l') {
    ToCenterLG();
    spindegree(-90);
  } else if (select == 'R') {
    spindegree(90);
  } else if (select == 'r') {
    ToCenterRG();
    spindegree(90);
  } else if (select == 'q' || select == 'Q') {
    turndegree(-90);
  } else if (select == 'e' || select == 'E') {
    turndegree(90);
  } else if (select == 'p' || select == 'P') {
    BZon();
    ReadCalibrateF();
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) break;
    }
    fftimerg(spd, 5);
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) break;
    }
    BZoff();
  } else if (select == 'c' || select == 'C') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
    }
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(-spd, -spd);
        delay(5);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'b' || select == 'B') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunG(tctL, tctR);
    }
    while (1) {
      RunG(tctL, tctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    while (1) {
      RunG(tctL, tctR);
      ReadCalibrateB();
      if ((B[0] > Ref || B[7] > Ref)) {
        Motor(-10, -10);
        delay(10);
        Motor(-1, -1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 's' || select == 'S') {
    Motor(-10, -10);
    delay(10);
    Motor(-1, -1);
    delay(1);
    MotorStop();
  } else {
    SetG(100);
  }
}

void TrackSelectGB(int spd, char select) {
  if (select == 'L') {
    spindegree(-90);
  } else if (select == 'l') {
    BackCenterG();
    spindegree(-90);
  } else if (select == 'R') {
    spindegree(90);
  } else if (select == 'r') {
    BackCenterG();
    spindegree(90);
  } else if (select == 'q' || select == 'Q') {
    turndegreeb(90);
  } else if (select == 'e' || select == 'E') {
    turndegreeb(-90);
  } else if (select == 'p' || select == 'P') {
    BZon();
    ReadCalibrateB();
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) break;
    }
    bbtimerg(spd, 5);
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) break;
    }
    BZoff();
  } else if (select == 'c' || select == 'C') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    }
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(spd, spd);
        delay(5);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'b' || select == 'B') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunGB(bctL, bctR);
    }
    while (1) {
      RunGB(bctL, bctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    while (1) {
      RunGB(bctL, bctR);
      ReadCalibrateB();
      if ((B[0] > Ref || B[7] > Ref)) {
        Motor(10, 10);
        delay(10);
        Motor(1, 1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 's' || select == 'S') {
    Motor(10, 10);
    delay(10);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else {
    SetG(100);
  }
}

void ffbg(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[1] > Ref || F[2] > Ref || F[3] > Ref || F[4] > Ref || F[5] > Ref || F[6] > Ref) break;
  }
  TrackSelectG(Speed, select);
}

void bbbg(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[1] > Ref || B[2] > Ref || B[3] > Ref || B[4] > Ref || B[5] > Ref || B[6] > Ref) break;
  }
  TrackSelectGB(Speed, select);
}

void ffdg(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (analogRead(DIST) >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void bbdg(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (analogRead(DIST) <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void ffdgs(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (analogRead(DIST) >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void bbdgs(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (analogRead(DIST) <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void fftimerg(int Speed, int totalTime, char select) {fftimerg(Speed, totalTime); TrackSelectG(Speed, select);}
void bbtimerg(int Speed, int totalTime, char select) {bbtimerg(Speed, totalTime); TrackSelectGB(Speed, select);}

void fftg(int Speed, int totalTime, char select) {fftimerg(Speed, totalTime); TrackSelectG(Speed, select);}
void bbtg(int Speed, int totalTime, char select) {bbtimerg(Speed, totalTime); TrackSelectGB(Speed, select);}

void ffcmgs(int Speed, float distance_cm, char select) {ffcmgs(Speed, distance_cm); TrackSelectG(Speed, select);}
void bbcmgs(int Speed, float distance_cm, char select) {bbcmgs(Speed, distance_cm); TrackSelectGB(Speed, select);}

void ffcmg(int Speed, float distance_cm, char select) {ffcmg(Speed, distance_cm); TrackSelectG(Speed, select);}
void bbcmg(int Speed, float distance_cm, char select) {bbcmg(Speed, distance_cm); TrackSelectGB(Speed, select);}

void setg(int time) { SetG(time); }
void setgb(int time) { SetGB(time); }

#endif // MYRP_PICO2_MINI_GYRO_H