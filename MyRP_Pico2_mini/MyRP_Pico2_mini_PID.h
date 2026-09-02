#ifndef MYRP_PICO2_MINI_PID_H
#define MYRP_PICO2_MINI_PID_H

int LastError_F, LastError_B;
int LastError_F_none, LastError_B_none;
int tct, bct, tspd;
int tctL, tctR, bctL, bctR;
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int LTurnBackSpdL, LTurnBackSpdR, TurnBackDelayL;
int RTurnBackSpdL, RTurnBackSpdR, TurnBackDelayR;
// int set_position = 2500;
int set_position = 3500;
//int set_positionL = 500;
int set_positionL = 1500;
// int set_positionR = 4500;
int set_positionR = 5500;
float slow_kp_f = 0.005, slow_kd_f = 0.05;
float slow_kp_b = 0.005, slow_kd_b = 0.05;
float slow_kpf = 0.005, slow_kdf = 0.05;
float slow_kpb = 0.005, slow_kdb = 0.05;
int line_centor = 0;

int MaxSpeed = 100;
int MinSpeed = -5;
int ModePidStatus = 0;
int dottedline = 0;

// ค่า Error ที่เล็กกว่านี้ (หน่วยเดียวกับ set_position, 0-5000) จะถูกมองเป็น 0
// เพื่อกันไม่ให้ PID สั่นตามสัญญาณรบกวนตอนวิ่งตรงกลางเส้น
int PID_DeadBand = 20;

void SetPIDDeadBand(int db) {
  PID_DeadBand = db;
}

// จำนวนครั้งที่ต้องอ่านค่าซ้ำ ๆ ให้ผลตรงกันติดต่อกัน ก่อนจะยอมรับว่า
// "พ้นเส้นเดิม" หรือ "เจอเส้นใหม่" แล้วจริง ๆ (กันสัญญาณรบกวนที่ขอบเส้นทำให้
// spinl()/spinr() หยุดเร็วเกินไปโดยยังไม่พ้นเส้นเดิม)
int SpinDebounceCount = 5;

void SetSpinDebounceCount(int n) {
  SpinDebounceCount = n;
}

// ---------- Config ----------
void SetFG(int time);

void SetToCenterSpeed(int tctv) {
  tct = tctv;
  bct = tctv;
  BaseSpeed = tctv;
  InitialSpeed();
  // Front / back base speed
  tctL = LeftBaseSpeed;
  tctR = RightBaseSpeed;
  bctL = BackLeftBaseSpeed;
  bctR = BackRightBaseSpeed;

  // Slow PID
  slow_kpf = PID_KP_Front;
  slow_kdf = PID_KD_Front;
  slow_kpb = PID_KP_Back;
  slow_kdb = PID_KD_Back;
}

void set_slow_kp_kd(float kp_f, float kd_f, float kp_b, float kd_b) {
  slow_kp_f = kp_f;
  slow_kd_f = kd_f;
  slow_kp_b = kp_b;
  slow_kd_b = kd_b;
}

void Dottedline(int x) {
  dottedline = x;
}

void SetTurnSpeed(int tspdv) {
  tspd = tspdv;
}

void TurnSpeedLeft(int l, int r, int de) {
  LTurnSpdL  = l;
  LTurnSpdR  = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int l, int r, int de) {
  RTurnSpdL  = l;
  RTurnSpdR  = r;
  TurnDelayR = de;
}

void TurnBackSpeedLeft(int l, int r, int de) {
  LTurnBackSpdL  = l;
  LTurnBackSpdR  = r;
  TurnBackDelayL = de;
}

void TurnBackSpeedRight(int l, int r, int de) {
  RTurnBackSpdL  = l;
  RTurnBackSpdR  = r;
  TurnBackDelayR = de;
}

void ModeSpdPID(int moD, int maX, int miN) {
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

void set_position_line(int _pos) {
  if (_pos < 0) {
    set_position = 0;
  } else if (_pos > 7000) {
    set_position = 7000;
  } else {
    set_position = _pos;
  }
}

void set_position_line_l(int _pos) {
  if (_pos < 0) {
    set_positionL = 0;
  } else if (_pos > 7000) {
    set_positionL = 7000;
  } else {
    set_positionL = _pos;
  }
}

void set_position_line_r(int _pos) {
  if (_pos < 0) {
    set_positionR = 0;
  } else if (_pos > 7000) {
    set_positionR = 7000;
  } else {
    set_positionR = _pos;
  }
}

// หมายเหตุ: เดิมเก็บค่า L/R ไว้ในตัวแปร setsensortracklineL/R แต่ไม่มีจุดใดใน
// ไลบรารีอ่านค่ากลับไปใช้เลย (เขียนอย่างเดียว ไม่มีผลต่อพฤติกรรมใด ๆ) จึงตัด
// ตัวแปรที่ไม่ได้ใช้งานทิ้ง และคงฟังก์ชันนี้ไว้เฉยๆ เพื่อไม่ให้สเก็ตช์เดิมที่เรียกใช้พัง
void set_sensor_track_line(int L, int R) {
}

void set_line_center(int x) {
  line_centor = x;
}

// ---------- Position Reading ----------

int readPositionF(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((8 - 1) * 1000) / 2;
  ReadCalibrateF();
  // int S[6] = {F[1], F[2], F[3], F[4], F[5], F[6]};
  for (i = 0; i < 8; i++) {
    // int values = S[i];
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < set_position) return 0 * 1000;
    else return 7 * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((8 - 1) * 1000) / 2;
  ReadCalibrateB();
  // int S[6] = {B[1], B[2], B[3], B[4], B[5], B[6]};
  for (i = 0; i < 8; i++) {
    // int values = S[i];
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < set_position) return 0 * 1000;
    else return 7 * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionF_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((8 - 1) * 1000) / 2;
  ReadCalibrateF();
  // int S[6] = {F[1], F[2], F[3], F[4], F[5], F[6]};
  for (i = 0; i < 8; i++) {
    // int values = S[i];
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < (8 - 1) * 1000 / 2) return set_position;
    else return set_position;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionB_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((8 - 1) * 1000) / 2;
  ReadCalibrateB();
  // int S[6] = {B[1], B[2], B[3], B[4], B[5], B[6]};
  for (i = 0; i < 8; i++) {
    // int values = S[i];
    int values = B[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (8 - 1) * 1000 / 2) return set_position;
    else return set_position;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

// ---------- PID ----------

// จำกัดค่า LeftPower/RightPower ตาม ModePidStatus (ใช้ร่วมกันทั้ง PIDF และ PIDB)
void ClampPIDPower(float &LeftPower, float &RightPower, int SpeedL, int SpeedR) {
  switch (ModePidStatus) {
  case 0:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = MinSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = MinSpeed;
    break;
  case 1:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < MinSpeed) LeftPower = MinSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < MinSpeed) RightPower = MinSpeed;
    break;
  case 2:
    if (LeftPower > SpeedL) LeftPower = SpeedL;
    if (LeftPower < -SpeedL) LeftPower = -SpeedL;
    if (RightPower > SpeedR) RightPower = SpeedR;
    if (RightPower < -SpeedR) RightPower = -SpeedR;
    break;
  case 3:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = -BaseSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = -BaseSpeed;
    break;
  default:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = 0;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = 0;
  }
}

void PIDF(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos = readPositionF(200, 50);
  float Error = Pos - set_position;
  if (fabs(Error) < PID_DeadBand) Error = 0;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;
  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  ClampPIDPower(LeftPower, RightPower, SpeedL, SpeedR);
  Motor(LeftPower, RightPower);
}

void PIDB(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos      = readPositionB(200, 50);
  float Error    = Pos - set_position;
  if (fabs(Error) < PID_DeadBand) Error = 0;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B  = Error;
  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  ClampPIDPower(LeftPower, RightPower, SpeedL, SpeedR);
  Motor(-LeftPower, -RightPower);
}

static bool frontCenterLine() {
  return (F[2] > Ref && F[3] > Ref)
      || (F[3] > Ref && F[4] > Ref)
      || (F[4] > Ref && F[5] > Ref);
}

void PIDF_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  ReadCalibrateF();
  float Pos;
  if (frontCenterLine()) {
    Pos = set_position;
  } else {
    Pos = readPositionF_none(200, 50);
  }

  float Error = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_F_none));
  LastError_F_none = Error;

  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;

  LeftPower  = constrain(LeftPower, -100, 100);
  RightPower = constrain(RightPower, -100, 100);

  Motor(LeftPower, RightPower);
}

static bool backCenterLine() {
  return (B[2] > Ref && B[3] > Ref)
      || (B[3] > Ref && B[4] > Ref)
      || (B[4] > Ref && B[5] > Ref);
}

void PIDB_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos;
  ReadCalibrateB();
  if (backCenterLine()) {
    Pos = set_position;
  } else {
    Pos = readPositionB_none(200, 50);
  }
  float Error    = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_B_none));
  LastError_B_none = Error;
  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  LeftPower  = constrain(LeftPower, -100, 100);
  RightPower = constrain(RightPower, -100, 100);
  Motor(-LeftPower, -RightPower);
}

// ---------- Timed Motion ----------

void fftimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
}

void bbtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
}

void lf(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(0, 0, 0.010, 0.30);
}

void lb(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(0, 0, 0.010, 0.30);
}

void ffcm(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง
  while (1) {
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
  }
}

void bbcm(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง
  while (1) {
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
  }
}

// ---------- ToCenter / BackCenter ----------
void ModeToCenter() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
  }
}

void ModeToCenterBack() {
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    }
  }
}

void ModeToCenterLine() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
  } else {
    PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
  }
}

void ModeToCenterBackLine() {
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
  } else {
    PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
  }
}

void ToCenter() {
  BZon();
  ModeToCenter();
  while (1) {
    ModeToCenterLine();
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterL() {
  BZon();
  ModeToCenter();
  while (1) {
    ModeToCenterLine();
    ReadCalibrateC();
    if (C[CCL] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterR() {
  BZon();
  ModeToCenter();
  while (1) {
    ModeToCenterLine();
    ReadCalibrateC();
    if (C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void BackCenter() {
  BZon();
  ModeToCenterBack();
  while (1) {
    ModeToCenterBackLine();
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

// ---------- Turns / Spins ----------

void TurnLeft() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
 
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrateF();
    if (F[2] >= Ref) break;

  }

}

void TurnRight() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
 
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateF();
    if (F[5] >= Ref) break;
  }

}

void spinl(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);

  // เลือกเซนเซอร์ตามความเร็ว tspd: ยิ่งหมุนช้า ยิ่งใช้เซนเซอร์ที่ห่างออกไป
  // (เผื่อระยะเหวี่ยงที่แคบลง) ตรวจสอบจากช่วงแคบไปกว้างเพื่อให้ทุกเงื่อนไข
  // มีโอกาสถูกใช้จริง (เช็ค <=50 ก่อน <=70 ไม่งั้น <=50 จะไม่มีทางถูกเลือก)
  int sensorIdx;
  if (speed >= 80) sensorIdx = 1;
  else if (speed <= 50) sensorIdx = 3;
  else if (speed <= 70) sensorIdx = 2;
  else sensorIdx = 1;

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[sensorIdx] <= Ref) break;
  }
  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[sensorIdx] <= Ref) break;
  }
  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[sensorIdx] >= Ref) break;
  }
  lf(tspd);
  MotorStop();
}

void spinl() {
  spinl(tspd);
}

void spinl2(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[3] >= Ref) break;
  }

  Motor(-speed, speed);
  delay(30);

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[4] >= Ref) {
      // Motor(speed, -speed);
      // delay(5);
      lf(tspd);
      MotorStop();
      break;
    }
  }
}

void spinl2() {
  spinl2(tspd);
}

void spinr(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);

  // เลือกเซนเซอร์ตามความเร็ว tspd แบบเดียวกับ spinl() แต่ mirror ไปฝั่งขวา
  // (F[2]<->F[5], F[1]<->F[6], F[3]<->F[4], F[0]<->F[7])
  int sensorIdx;
  if (speed >= 80) sensorIdx = 6;
  else if (speed <= 50) sensorIdx = 4;
  else if (speed <= 70) sensorIdx = 5;
  else sensorIdx = 6;


  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[sensorIdx] <= Ref) break;
  }
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[sensorIdx] <= Ref) break;
  }
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[sensorIdx] >= Ref) break;
  }
  lf(tspd);
  MotorStop();
}

void spinr() {
  spinr(tspd);
}

void spinr2(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[5] >= Ref) break;
  }
  Motor(speed, -speed);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[5] >= Ref) {
      // Motor(-speed, speed);
      // delay(5);
      lf(tspd);
      MotorStop();
      break;
    }
  }
}

void spinr2() {
  spinr2(tspd);
}

// ==================== Back Sensor ====================

void TurnLeft_B() {
  Motor(-LTurnBackSpdL, LTurnBackSpdR);
  delay(TurnBackDelayL);

  while (1) {
    Motor(-LTurnBackSpdL, LTurnBackSpdR);
    ReadCalibrateB();
    if (B[5] >= Ref) break;
  }
}

void TurnRight_B() {
  Motor(RTurnBackSpdL, -RTurnBackSpdR);
  delay(TurnBackDelayR);
  while (1) {
    Motor(RTurnBackSpdL, -RTurnBackSpdR);
    ReadCalibrateB();
    if (B[2] >= Ref) break;
  }
}

// ==================== Spin Left Back ====================

void spinl_B(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);

  // เลือกเซนเซอร์ตาม tspd แบบเดียวกับ spinl() แต่ index บนอาเรย์ B[] ซึ่งเรียง
  // กลับด้าน (B_PIN เรียงย้อนจาก F_PIN) ตำแหน่งเซนเซอร์จริงจึงตรงกับที่ spinl()
  // ใช้ (F[1],F[2],F[3],F[0]) แค่แปลงเป็น index ของ B[] คือ B[6],B[5],B[4],B[7]
  int sensorIdx;
  if (tspd >= 80) sensorIdx = 6;
  else if (tspd <= 50) sensorIdx = 4;
  else if (tspd <= 70) sensorIdx = 5;
  else sensorIdx = 6;

  int offCount = 0;
  while (offCount < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[sensorIdx] <= Ref) offCount++;
    else offCount = 0;
  }
  int onCount = 0;
  while (onCount < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[sensorIdx] >= Ref) onCount++;
    else onCount = 0;
  }
  lb(tspd);
  MotorStop();
}

void spinl_B() {
  spinl_B(tspd);
}

// ==================== Spin Left 2 Back ====================

void spinl2_B(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);
  int hitCount1 = 0;
  while (hitCount1 < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[5] >= Ref) hitCount1++;
    else hitCount1 = 0;
  }
  Motor(-speed, speed);
  delay(30);
  int hitCount2 = 0;
  while (hitCount2 < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[5] >= Ref) hitCount2++;
    else hitCount2 = 0;
  }
  lb(tspd);
  MotorStop();
}

void spinl2_B() {
  spinl2_B(tspd);
}

// ==================== Spin Right Back ====================

void spinr_B(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);

  // เลือกเซนเซอร์ตาม tspd แบบเดียวกับ spinr() แต่ index บนอาเรย์ B[] ซึ่งเรียง
  // กลับด้าน ตำแหน่งเซนเซอร์จริงตรงกับที่ spinr() ใช้ (F[6],F[5],F[4],F[7])
  // แปลงเป็น index ของ B[] คือ B[1],B[2],B[3],B[0]
  int sensorIdx;
  if (tspd >= 80) sensorIdx = 1;
  else if (tspd <= 50) sensorIdx = 3;
  else if (tspd <= 70) sensorIdx = 2;
  else sensorIdx = 1;

  int offCount = 0;
  while (offCount < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[sensorIdx] <= Ref) offCount++;
    else offCount = 0;
  }
  int onCount = 0;
  while (onCount < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[sensorIdx] >= Ref) onCount++;
    else onCount = 0;
  }
  lb(tspd);
  MotorStop();
}

void spinr_B() {
  spinr_B(tspd);
}

// ==================== Spin Right 2 Back ====================

void spinr2_B(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);
  int hitCount1 = 0;
  while (hitCount1 < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[2] >= Ref) hitCount1++;
    else hitCount1 = 0;
  }
  Motor(speed, -speed);
  delay(30);
  int hitCount2 = 0;
  while (hitCount2 < SpinDebounceCount) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[2] >= Ref) hitCount2++;
    else hitCount2 = 0;
  }
  lb(tspd);
  MotorStop();
}

void spinr2_B() {
  spinr2_B(tspd);
}

// ---------- Track Select ----------

void TrackSelectF(int spd, char x) {
  if (x == 's') {
    Motor(-spd, -spd);
    delay(5);
    Move(-15, -15, 5);
    Move(-10, -10, 1);
    Move(-1, -1, 1);
    MotorStop();
    // MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
  } else if (x == 'S') {
    while(1){
PIDF(tctL, tctR, slow_kpf, slow_kdf);
    ReadCalibrateF();
    if (F[0] > Ref || F[7] > Ref) {
      Motor(-spd, -spd);
      delay(5);
      Move(-15, -15, 5);
      Move(-10, -10, 1);
      Move(-1, -1, 1);
      MotorStop();
      break;
      // MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
    }

    }
    
  } else if (x == 'p' || x == 'P') {
    BZon();
    ReadCalibrateF();
    int offCount_p = 0;
    while (offCount_p < SpinDebounceCount) {
      PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) offCount_p++;
      else offCount_p = 0;
    }
    BZoff();
  } else if (x == 'l' || x == 'L') {
    ToCenter();
    spinl();
  } else if (x == 'r' || x == 'R') {
    ToCenter();
    spinr();
  } else if (x == 'q' || x == 'Q') {
    int onCount_q = 0;
    while (onCount_q < SpinDebounceCount) {
      PIDF(tctL, tctR, slow_kpf, slow_kdf);
      ReadCalibrateF();
      if (F[0] > Ref) onCount_q++;
      else onCount_q = 0;
    }
    BZon();
    int offCount_q = 0;
    while (1) {
      Motor(tctL / 2, tctR / 2);
      ReadCalibrateF();
      if (F[0] < Ref) offCount_q++;
      else offCount_q = 0;
      if (offCount_q >= SpinDebounceCount) {
        delay(20);
        BZoff();
        break;
      }
    }
    TurnLeft();
  } else if (x == 'e' || x == 'E') {
    int onCount_e = 0;
    while (onCount_e < SpinDebounceCount) {
      PIDF(tctL, tctR, slow_kpf, slow_kdf);
      ReadCalibrateF();
      if (F[7] > Ref) onCount_e++;
      else onCount_e = 0;
    }
    BZon();
    int offCount_e = 0;
    while (1) {
      Motor(tctL / 2, tctR / 2);
      ReadCalibrateF();
      if (F[7] < Ref) offCount_e++;
      else offCount_e = 0;
      if (offCount_e >= SpinDebounceCount) {
        delay(20);
        BZoff();
        break;
      }
    }
    TurnRight();
  } else if (x == 'c' || x == 'C') {
    ToCenter();
  } else if (x == 'd' || x == 'D') {
    ToCenter();
    spinr_B();
  } else if (x == 'a' || x == 'A') {
    ToCenter();
    spinl_B();
  } else if (x == 'b' || x == 'B') {
    BZon();
    ModeToCenter();
    while (1) {
      ModeToCenterLine();
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
  } else if (x == 'g' || x == 'G') {
    SetFG(100);
  } else {
    MotorStop(20);
  }
}

void TrackSelectB(int spd, char x) {
  if (x == 's') {
    Motor(spd, spd);
    delay(5);
    Move(15, 15, 5);
    Move(10, 10, 1);
    Move(1, 1, 1);
    MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
  } else if (x == 'S') {
    while(1){
PIDB(bctL, bctR, slow_kpb, slow_kdb);
    ReadCalibrateB();
    if (B[0] > Ref || B[7] > Ref) {
      Motor(spd, spd);
      delay(5);
      Move(15, 15, 5);
      Move(10, 10, 1);
      Move(1, 1, 1);
      MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
    }

    }
    
  } else if (x == 'p' || x == 'P') {
    BZon();
    ReadCalibrateB();
    while (1) {
      Motor(-spd, -spd);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) break;
    }
    delay(5);
    while (1) {
      Motor(-spd, -spd);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) {
        BZoff();
        break;
      }
    }
  } else if (x == 'l' || x == 'L') {
    BackCenter();
    spinl();
  } else if (x == 'r' || x == 'R') {
    BackCenter();
    spinr();
  } else if (x == 'c' || x == 'C') {
    BackCenter();
  } else if (x == 'd' || x == 'D') {
    BackCenter();
    spinr_B();
  } else if (x == 'a' || x == 'A') {
    BackCenter();
    spinl_B();
  } else if (x == 'e' || x == 'E') {
    int onCount_eB = 0;
    while (onCount_eB < SpinDebounceCount) {
      PIDB(bctL, bctR, slow_kpb, slow_kdb);
      ReadCalibrateB();
      if (B[0] > Ref) onCount_eB++;
      else onCount_eB = 0;
    }
    BZon();
    int offCount_eB = 0;
    while (1) {
      Motor(-bctL / 2, -bctR / 2);
      ReadCalibrateB();
      if (B[0] < Ref) offCount_eB++;
      else offCount_eB = 0;
      if (offCount_eB >= SpinDebounceCount) {
        BZoff();
        break;
      }
    }
    TurnLeft_B();
  } else if (x == 'q' || x == 'Q') {
    int onCount_qB = 0;
    while (onCount_qB < SpinDebounceCount) {
      PIDB(bctL, bctR, slow_kpb, slow_kdb);
      ReadCalibrateB();
      if (B[7] > Ref) onCount_qB++;
      else onCount_qB = 0;
    }
    BZon();
    int offCount_qB = 0;
    while (1) {
      Motor(-bctL / 2, -bctR / 2);
      ReadCalibrateB();
      if (B[7] < Ref) offCount_qB++;
      else offCount_qB = 0;
      if (offCount_qB >= SpinDebounceCount) {
        BZoff();
        break;
      }
    }
    TurnRight_B();
  } else if (x == 'b' || x == 'B') {
    BZon();
    ModeToCenterBack();
    while (1) {
      ModeToCenterBackLine();
      ReadCalibrateF();
      if ((F[0] > Ref || F[7] > Ref)) {
        Motor(10, 10);
        delay(10);
        Motor(1, 1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (x == 'g' || x == 'G') {
    SetFG(100);
  } else {
    MotorStop(20);
  }
}

void fftimer(int Speed, int totalTime, char select) {
  fftimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void bbtimer(int Speed, int totalTime, char select) {
  bbtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}
void fft(int Speed, int totalTime, char select) {
  fftimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void bbt(int Speed, int totalTime, char select) {
  bbtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void ffcm(int Speed, float distance, char select) {
  ffcm(Speed, distance);
  TrackSelectF(Speed, select);
}

void bbcm(int Speed, float distance, char select) {
  bbcm(Speed, distance);
  TrackSelectB(Speed, select);
}

// ---------- ff / bb Patterns ----------
void ff(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref || F[7] > Ref  || (F[2] > Ref && F[5] > Ref)) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}
void bb(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref || B[7] > Ref || (B[2] > Ref && B[5] > Ref)) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void ffc2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[0] > Ref && F[7] > Ref) || (F[1] > Ref && F[6] > Ref) || (F[2] > Ref && F[3] > Ref && F[4] > Ref && F[5] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void ffc(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] >= Ref && F[7] >= Ref) break;
  }
  TrackSelectF(Speed, select);
}

void bbc2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[0] > Ref && B[7] > Ref) || (B[1] > Ref && B[6] > Ref) || (B[2] > Ref && B[3] > Ref && B[4] > Ref && B[5] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void bbc(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] >= Ref && B[7] >= Ref) break;
  }
  TrackSelectB(Speed, select);
}

void ffl(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref ) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}
void ffl0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void ffl2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref && F[1] > Ref && F[2] > Ref && F[3] > Ref && F[4] > Ref) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void bbl(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref ) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}
void bbl0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bbl2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref && B[1] > Ref && B[2] > Ref && B[3] > Ref && B[4] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void ffr(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ( F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void ffr7(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void ffr2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[3] > Ref && F[4] > Ref && F[5] > Ref && F[6] > Ref && F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void bbr(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ( B[7] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bbr7(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[7] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}
void bbr2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[3] > Ref && B[4] > Ref && B[5] > Ref && B[6] > Ref && B[7] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void ffblack(int SpeedL, int SpeedR, char select) {
  Move(SpeedL, SpeedR, 50);
  while (1) {
    Motor(SpeedL, SpeedR);
    ReadCalibrateF();
    if (F[1] > Ref || F[2] > Ref || F[3] > Ref || F[4] > Ref || F[5] > Ref || F[6] > Ref) break;
  }
  TrackSelectF(SpeedL, select);
}

void ffb(int SpeedL, int SpeedR, char select) {
  ffblack(SpeedL, SpeedR, select);
}

void ffblack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  ffblack(LeftBaseSpeed, RightBaseSpeed, select);
}

void ffb(int Speed, char select) {
  ffblack(Speed, select);
}

void bbblack(int SpeedL, int SpeedR, char select) {
  Move(-SpeedL, -SpeedR, 50);
  while (1) {
    Motor(-SpeedL, -SpeedR);
    ReadCalibrateB();
    if (B[1] > Ref || B[2] > Ref || B[3] > Ref || B[4] > Ref || B[5] > Ref || B[6] > Ref) break;
  }
  TrackSelectB(SpeedL, select);
}

void bbb(int SpeedL, int SpeedR, char select) {
  bbblack(SpeedL, SpeedR, select);
}

void bbblack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  bbblack(BackLeftBaseSpeed, BackRightBaseSpeed, select);
}

void bbb(int Speed, char select) {
  bbblack(Speed, select);
}

void ffwhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[0] < Ref && F[1] < Ref && F[2] < Ref && F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void ffw(int Speed, char select) { ffwhite(Speed, select); }

void bbwhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[0] < Ref && B[1] < Ref && B[2] < Ref && B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void bbw(int Speed, char select) { bbwhite(Speed, select); }

void ffnum(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void bbnum(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void ffn(int Speed, char select, int numm) {
  ffnum(Speed, select, numm);
}
void bbn(int Speed, char select, int numm) {
  bbnum(Speed, select, numm);
}

void ff_distance(int Speed, char select, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    if (analogRead(DIST) >= distance) break;
  }
  TrackSelectF(Speed, select);
}

void bb_distance(int Speed, char select, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    if (analogRead(DIST) <= distance) break;
  }
  TrackSelectB(Speed, select);
}

void ffd(int Speed, char select, float distance) {
  ff_distance(Speed, select, distance);
}
void bbd(int Speed, char select, float distance) {
  bb_distance(Speed, select, distance);
}

// ===== Stop PID =====
float PID_KP_STOP = 0.35;
float PID_KD_STOP = 0.8;

int PID_Stop(int ir, int ir_target, int maxSpeed) {
  static int lastError = 0;

  int error = ir_target - ir;
  int derivative = error - lastError;
  lastError = error;

  int out = (PID_KP_STOP * error) +
            (PID_KD_STOP * derivative);

  return constrain(out, 0, maxSpeed);
}

void ff_distances(int Speed, char select, int ir_target) {
  BaseSpeed = Speed;
  InitialSpeed();  // จำเป็นเสมอ: เปลี่ยน BaseSpeed แล้วต้องเรียก InitialSpeed() ใหม่

  while (1) {
    ReadCalibrateF();
    int ir = analogRead(DIST);
    // ===== เข้าโหมดหยุด =====
    if (ir >= ir_target) {
      int newBase = PID_Stop(ir, ir_target, Speed);
      if (newBase != BaseSpeed) {
        BaseSpeed = newBase;
        InitialSpeed();  // BaseSpeed เปลี่ยน ต้อง InitialSpeed() ใหม่
      }
    }
    // ===== PID ตามเส้น =====
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    // ===== หยุดจริง =====
    if (BaseSpeed <= 1) break;
  }
  MotorStop();                   // เบรกนิ่ง
  TrackSelectF(Speed, select);   // คำสั่งถัดไป
}

void ffds(int Speed, char select, int ir_target) {
  ff_distances(Speed, select, ir_target);
}

// ---------- GoStart / GoEnd ----------

void gostart(int LeftSpeed, int RightSpeed) {
  Move(LeftSpeed, RightSpeed, 100);
  while (1) {
    ReadCalibrateF();
    Motor(LeftSpeed, RightSpeed);
    if (F[0] < Ref && F[7] < Ref) {
      Move(LeftSpeed, RightSpeed, 1);
      break;
    }
  }
}

void gostart(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  gostart(LeftBaseSpeed, RightBaseSpeed);
}

void goend(int LeftSpeed, int RightSpeed) {
  while (1) {
    ReadCalibrateC();
    Motor(LeftSpeed, RightSpeed);
    if (C[0] > RefC && C[1] > RefC) {
      Move(LeftSpeed, RightSpeed, 50);
      break;
    }
  }
  Move(-15, -15, 15);
  Move(-10, -10, 10);
  Move(-1, -1, 1);
  MotorStop();
}

void goend(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  goend(LeftBaseSpeed, RightBaseSpeed);
  MotorStop();
}

// ---------- Balance ----------

void balancef(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 50);
    while (1) {
      Motor(10, 10);
      ReadCalibrateF();
      if (F[0] > Ref) {
        while (1) {
          Motor(0, 10);
          ReadCalibrateF();
          if (F[7] > Ref) { MotorStop(); break; }
        }
      }
      if (F[7] > Ref) {
        while (1) {
          Motor(10, 0);
          ReadCalibrateF();
          if (F[0] > Ref) { MotorStop(); break; }
        }
      }
      if (F[0] > Ref && F[7] > Ref) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
}
void setf(int Counter) {
  balancef(Counter);
}

void balanceb(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 50);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateB();
      if (B[0] > Ref) {
        while (1) {
          Motor(0, -10);
          ReadCalibrateB();
          if (B[7] > Ref) { MotorStop(); break; }
        }
      }
      if (B[7] > Ref) {
        while (1) {
          Motor(-10, 0);
          ReadCalibrateB();
          if (B[0] > Ref) { MotorStop(); break; }
        }
      }
      if (B[0] > Ref && B[7] > Ref) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
}

void setb(int Counter) {
  balanceb(Counter);
}

void set_f(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateF();
      delay(5);
      if (F[0] > Ref && F[7] < Ref) {
        Motor(-5, 15);
      } else if (F[0] < Ref && F[7] > Ref) {
        Motor(15, -5);
      } else if (F[0] < Ref && F[7] < Ref) {
        Motor(15, 15);
      } else {
        Motor(-1, -1);
        break;
      }
    }
    if (num > 1) {
      Motor(-15, -15);
      delay(50);
      Motor(-1, -1);
    }
  }
}

void set_b(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateB();
      delay(5);
      if (B[0] > Ref && B[7] < Ref) {
        Motor(5, -15);
      } else if (B[0] < Ref && B[7] > Ref) {
        Motor(-15, 5);
      } else if (B[0] < Ref && B[7] < Ref) {
        Motor(-15, -15);
      } else {
        Motor(1, 1);
        break;
      }
    }
    if (num > 1) {
      Motor(15, 15);
      delay(50);
      Motor(1, 1);
    }
  }
}

void set_fc(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateC();
      delay(5);
      if (C[1] > RefC && C[0] < RefC) {
        Motor(-5, 15);
      } else if (C[1] < RefC && C[0] > RefC) {
        Motor(15, -5);
      } else if (C[1] < RefC && C[0] < RefC) {
        Motor(15, 15);
      } else {
        Motor(-1, -1);
        break;
      }
    }
    if (num > 1) {
      Motor(-15, -15);
      delay(50);
      Motor(-1, -1);
    }
  }
}

void set_bc(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateC();
      delay(5);
      if (C[1] > RefC && C[0] < RefC) {
        Motor(5, -15);
      } else if (C[1] < RefC && C[0] > RefC) {
        Motor(-15, 5);
      } else if (C[1] < RefC && C[0] < RefC) {
        Motor(-15, -15);
      } else {
        Motor(1, 1);
        break;
      }
    }
    if (num > 1) {
      Motor(15, 15);
      delay(50);
      Motor(1, 1);
    }
  }
}

void SerialPositionF() {
  while (1) {
    int pos = readPositionF(200, 50);
    Serial.print("Position F : ");
    Serial.println(pos);
    delay(100);
  }
}

void SerialPositionB() {
  while (1) {
    int pos = readPositionB(200, 50);
    Serial.print("Position  B : ");
    Serial.println(pos);
    delay(100);
  }
}

void SerialPositionFB() {
  while (1) {
    int posF = readPositionF(200, 50);
    int posB = readPositionB(200, 50);
    Serial.print("Position F : ");
    Serial.print(posF);
    Serial.print("  |  Position B : ");
    Serial.println(posB);
    delay(100);
  }
}

// ---------- Circle Motion (CL/CR: เบี่ยงซ้าย/ขวา ด้วย set_positionL/R ชั่วคราว) ----------

void ffcl(int Speed, char select) {
  int temp = set_position;
  set_position = set_positionL;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}

void ffcr(int Speed, char select) {
  int temp = set_position;
  set_position = set_positionR;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}

void fftimercl(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, totalTime);
  set_position = temp;
}

void fftimercr(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, totalTime);
  set_position = temp;
}

void ffcmcl(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, distance);
  set_position = temp;
}

void ffcmcr(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, distance);
  set_position = temp;
}

void fftimercl(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, totalTime, select);
  set_position = temp;
}

void fftimercr(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, totalTime, select);
  set_position = temp;
}

void ffcmcl(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, distance, select);
  set_position = temp;
}

void ffcmcr(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, distance, select);
  set_position = temp;
}

void bbcl(int Speed, char select) {
  int temp = set_position;
  set_position = set_positionL;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbcr(int Speed, char select) {
  int temp = set_position;
  set_position = set_positionR;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[7] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbtimercl(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, totalTime);
  set_position = temp;
}

void bbtimercr(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, totalTime);
  set_position = temp;
}

void bbcmcl(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, distance);
  set_position = temp;
}

void bbcmcr(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, distance);
  set_position = temp;
}

void bbtimercl(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, totalTime, select);
  set_position = temp;
}

void bbtimercr(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, totalTime, select);
  set_position = temp;
}

void bbcmcl(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, distance, select);
  set_position = temp;
}

void bbcmcr(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, distance, select);
  set_position = temp;
}

#endif // MYRP_PICO2_MINI_PID_H
