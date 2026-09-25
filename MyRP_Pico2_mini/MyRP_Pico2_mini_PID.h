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
int LTurnBackFSpdL, LTurnBackFSpdR, TurnBackFDelayL;
int RTurnBackFSpdL, RTurnBackFSpdR, TurnBackFDelayR;
int LTurnBackBSpdL, LTurnBackBSpdR, TurnBackBDelayL;
int RTurnBackBSpdL, RTurnBackBSpdR, TurnBackBDelayR;

int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20; // การหน่วง
int set_position = 2500;
// int set_position = 3500;
int set_positionL = 500;
// int set_positionL = 1500;
int set_positionR = 4500;
// int set_positionR = 5500;
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

// ---------- Config ----------
void SetFG(int time);
void SetRobotAngle();

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
  slow_kp_f = PID_KP_Front;
  slow_kd_f = PID_KD_Front;
  slow_kp_b = PID_KP_Back;
  slow_kd_b = PID_KD_Back;
}

void set_slow_kp_kd(float kp_f, float kd_f, float kp_b, float kd_b) {
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

// ความเร็ว/ดีเลย์ของ TurnLeftBackF()/TurnRightBackF() แยกต่างหากจาก
// TurnSpeedLeft()/TurnSpeedRight() (ที่ใช้กับ TurnLeft()/TurnRight() ฝั่งเดินหน้า)
// เพื่อให้ปรับความเร็วตอนเลี้ยวล้อเดียวถอยหลังได้โดยไม่กระทบการเลี้ยวเดินหน้า
void TurnSpeedLeftBackF(int l, int r, int de) {
  LTurnBackFSpdL  = l;
  LTurnBackFSpdR  = r;
  TurnBackFDelayL = de;
}

void TurnSpeedRightBackF(int l, int r, int de) {
  RTurnBackFSpdL  = l;
  RTurnBackFSpdR  = r;
  TurnBackFDelayR = de;
}

// ความเร็ว/ดีเลย์ของ TurnLeftBackB()/TurnRightBackB() (เลี้ยวล้อเดียวถอยหลัง
// เช็คเส้นด้วยเซนเซอร์หลัง B[]) แยกต่างหากจากทุกชุดข้างต้น
void TurnSpeedLeftBackB(int l, int r, int de) {
  LTurnBackBSpdL  = l;
  LTurnBackBSpdR  = r;
  TurnBackBDelayL = de;
}

void TurnSpeedRightBackB(int l, int r, int de) {
  RTurnBackBSpdL  = l;
  RTurnBackBSpdR  = r;
  TurnBackBDelayR = de;
}

void set_brake_fc(int ff, int fc)
     {       
        break_ff = ff;  
        break_fc = fc;       
     }
void set_brake_bc(int ff, int fc)
     {       
        break_bf = ff;  
        break_bc = fc;       
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
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateF();
  int S[6] = {F[1], F[2], F[3], F[4], F[5], F[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
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
    else return 5 * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateB();
  int S[6] = {B[1], B[2], B[3], B[4], B[5], B[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
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
    else return 5 * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionF_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateF();
  int S[6] = {F[1], F[2], F[3], F[4], F[5], F[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
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
    if (last_value < (6 - 1) * 1000 / 2) return set_position;
    else return set_position;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateB();
  int S[6] = {B[1], B[2], B[3], B[4], B[5], B[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (6 - 1) * 1000 / 2) return set_position;
    else return set_position;
  }
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
  while (millis() <= endTime) PIDF(0, 0, 0.015, 0.10);
  MotorStop();
}

void lb(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(0, 0, 0.015, 0.10);
  MotorStop();
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
      delay(break_fc);
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
      delay(break_fc);
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
      delay(break_fc);
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
      delay(break_bc);
      MotorStop();
      BZoff();
      break;
    }
  }
}


void ToFront() {
  while (1) {
    PIDF(tctL, tctR, slow_kp_f, slow_kd_f);
    ReadCalibrateF();
    if (F[0] > Ref || F[7] > Ref) break;
  }
}

void ToBack() {
  while (1) {
    PIDB(tctL, tctR, slow_kp_f, slow_kd_f);
    ReadCalibrateB();
    if (B[0] > Ref || B[7] > Ref) break;
  }
}

// ---------- Turns / Spins ----------

void TurnLeft() {
  Motor(LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(LTurnSpdL, LTurnSpdR);
    ReadCalibrateF();
    if (F[2] >= Ref) break;
  }
}

void TurnRight() {
  Motor(RTurnSpdL, RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, RTurnSpdR);
    ReadCalibrateF();
    if (F[5] >= Ref) break;
  }
}

// เลี้ยวล้อเดียวแบบถอยหลัง เช็คเส้นด้วยเซนเซอร์หน้า F[] (ต่างจาก TurnLeft_B()/
// TurnRight_B() ที่เปลี่ยนไปใช้เซนเซอร์หลัง B[]) ตั้งความเร็ว/ดีเลย์แยกต่างหากจาก
// TurnLeft()/TurnRight() ผ่าน TurnSpeedLeftBackF()/TurnSpeedRightBackF()
// (ให้ล้อฝั่งที่ไม่ต้องการหมุน = 0 เพื่อให้เป็นการเลี้ยวล้อเดียว)
void TurnLeftBackF() {
  // เลือกเซนเซอร์ตามความเร็วล้อที่หมุน เหมือน spinl(): ยิ่งหมุนช้า ยิ่งใช้
  // เซนเซอร์ที่ห่างออกไป (เผื่อระยะเหวี่ยงที่แคบลง)
  int spd = max(abs(LTurnBackFSpdL), abs(LTurnBackFSpdR));
  int sensorIdx;
  if (spd >= 80) sensorIdx = 6;
  else if (spd <= 50) sensorIdx = 4;
  else if (spd <= 70) sensorIdx = 5;
  else sensorIdx = 1;

  Motor(LTurnBackFSpdL, -LTurnBackFSpdR);
  delay(TurnBackFDelayL);
  while (1) {
    Motor(LTurnBackFSpdL, -LTurnBackFSpdR);
    ReadCalibrateF();
    if (F[sensorIdx] >= Ref) MotorStop(); lf(spd); break;
  }
}

void TurnRightBackF() {
  // mirror ของ TurnLeftBackF() ไปฝั่งขวา (F[2]<->F[5], F[1]<->F[6], F[3]<->F[4])
  int spd = max(abs(RTurnBackFSpdL), abs(RTurnBackFSpdR));
  int sensorIdx;
  if (spd >= 80) sensorIdx = 1;
  else if (spd <= 50) sensorIdx = 3;
  else if (spd <= 70) sensorIdx = 2;
  else sensorIdx = 6;

  Motor(RTurnBackFSpdL, RTurnBackFSpdR);
  delay(TurnBackFDelayR);
  while (1) {
    Motor(RTurnBackFSpdL, RTurnBackFSpdR);
    ReadCalibrateF();
    if (F[sensorIdx] >= Ref) MotorStop(); lf(spd); break;
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
  Motor(speed, -speed);
  delay(5);
  lf(speed);
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
  int sensorIdx;
  if (speed >= 80) sensorIdx = 1;
  else if (speed <= 50) sensorIdx = 3;
  else if (speed <= 70) sensorIdx = 2;
  else sensorIdx = 1;
  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[sensorIdx] >= Ref) break;
  }

  Motor(-speed, speed);
  delay(30);

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[sensorIdx] >= Ref) {
      Motor(speed, -speed);
      delay(5);
      lf(speed);
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
  Motor(-speed, speed);
  delay(5);
  lf(speed);
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
  int sensorIdx;
  if (speed >= 80) sensorIdx = 6;
  else if (speed <= 50) sensorIdx = 4;
  else if (speed <= 70) sensorIdx = 5;
  else sensorIdx = 6;

  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[sensorIdx] >= Ref) break;
  }
  Motor(speed, -speed);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[sensorIdx] >= Ref) {
      Motor(-speed, speed);
      delay(5);
      lf(speed);
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
  Motor(LTurnBackSpdL, LTurnBackSpdR);
  delay(TurnBackDelayL);

  while (1) {
    Motor(LTurnBackSpdL, LTurnBackSpdR);
    ReadCalibrateB();
    if (B[5] >= Ref) break;
  }
}

void TurnRight_B() {
  Motor(RTurnBackSpdL, RTurnBackSpdR);
  delay(TurnBackDelayR);
  while (1) {
    Motor(RTurnBackSpdL, RTurnBackSpdR);
    ReadCalibrateB();
    if (B[2] >= Ref) break;
  }
}

// เลี้ยวล้อเดียวแบบถอยหลัง เช็คเส้นด้วยเซนเซอร์หลัง B[] (คู่กับ TurnLeftBackF()/
// TurnRightBackF() ที่เช็คเซนเซอร์หน้า F[]) กลับทิศมอเตอร์จาก TurnLeft_B()/
// TurnRight_B() ตั้งความเร็ว/ดีเลย์แยกต่างหากผ่าน TurnSpeedLeftBackB()/
// TurnSpeedRightBackB() (ให้ล้อฝั่งที่ไม่ต้องการหมุน = 0 เพื่อให้เป็นการเลี้ยวล้อเดียว)
void TurnLeftBackB() {
  // เลือกเซนเซอร์ตามความเร็วล้อที่หมุน เหมือน spinl_B() (ดัชนีบน B[] เรียงกลับด้าน
  // จาก F[] จึงใช้ชุด 6/4/5 แทน 1/3/2 เพื่อให้ตรงตำแหน่งจริงเดียวกัน)
  int spd = max(abs(LTurnBackBSpdL), abs(LTurnBackBSpdR));
  int sensorIdx;
  if (spd >= 80) sensorIdx = 1;
  else if (spd <= 50) sensorIdx = 3;
  else if (spd <= 70) sensorIdx = 2;
  else sensorIdx = 6;

  Motor(LTurnBackBSpdL, LTurnBackBSpdR);
  delay(TurnBackBDelayL);
  while (1) {
    Motor(LTurnBackBSpdL, LTurnBackBSpdR);
    ReadCalibrateB();
    if (B[sensorIdx] >= Ref) break;
  }
}

void TurnRightBackB() {
  // mirror ของ TurnLeftBackB() ไปฝั่งขวา เหมือน spinr_B()
  int spd = max(abs(RTurnBackBSpdL), abs(RTurnBackBSpdR));
  int sensorIdx;
  if (spd >= 80) sensorIdx = 6;
  else if (spd <= 50) sensorIdx = 4;
  else if (spd <= 70) sensorIdx = 5;
  else sensorIdx = 1;

  Motor(RTurnBackBSpdL, RTurnBackBSpdR);
  delay(TurnBackBDelayR);
  while (1) {
    Motor(RTurnBackBSpdL, RTurnBackBSpdR);
    ReadCalibrateB();
    if (B[sensorIdx] >= Ref) break;
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
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[sensorIdx] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[sensorIdx] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);

    if (B[sensorIdx] >= Ref) {
      Motor(speed, -speed);
      delay(5);
      lb(speed);
      MotorStop();
      break;
    }
  }
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
  int sensorIdx;
  if (tspd >= 80) sensorIdx = 6;
  else if (tspd <= 50) sensorIdx = 4;
  else if (tspd <= 70) sensorIdx = 5;
  else sensorIdx = 6;
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[sensorIdx] >= Ref) break;
  }
  Motor(-speed, speed);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[sensorIdx] >= Ref) {
      Motor(speed, -speed);
      delay(5);
      lb(speed);
      MotorStop();
      break;
    }
  }
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

  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[sensorIdx] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[sensorIdx] >= Ref) {
      Motor(-speed, speed);
      delay(5);
      lb(speed);
      MotorStop();
      break;
    }
  }
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
  int sensorIdx;
  if (tspd >= 80) sensorIdx = 1;
  else if (tspd <= 50) sensorIdx = 3;
  else if (tspd <= 70) sensorIdx = 2;
  else sensorIdx = 1;
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[sensorIdx] >= Ref) break;
  }
  Motor(speed, -speed);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[sensorIdx] >= Ref) {
      Motor(-speed, speed);
      delay(5);
      lb(tspd);
      MotorStop();
      break;
    }
  }
}

void spinr2_B() {
  spinr2_B(tspd);
}

// ---------- Track Select ----------

void TrackSelectF(int spd, char x) {
  switch (x) {
    case 's':
      Motor(-spd, -spd);
      delay(5);
      Move(-15, -15, 5);
      Move(-10, -10, 1);
      Move(-1, -1, 1);
      MotorStop();
      // MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
      break;

    case 'S':
      while (1) {
        PIDF(tctL, tctR, slow_kp_f, slow_kd_f);
        ReadCalibrateF();
        if (F[0] > Ref || F[7] > Ref) {
          Motor(-spd, -spd);
          delay(5);
          Move(-15, -15, 5);
          Move(-10, -10, 1);
          Move(-1, -1, 1);
          MotorStop();
          // MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
          break;
        }
      }
      break;

    case 'p':
      BZon();
      ReadCalibrateF();
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) {
          BZoff();
          break;
        }
      }
      break;

    case 'P':
      ToFront();
      BZon();
      ReadCalibrateF();
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) {
          BZoff();
          break;
        }
      }
      break;

    case 'l':
    case 'L':
      ToCenter();
      spinl();
      break;

    case 'r':
    case 'R':
      ToCenter();
      spinr();
      break;

    case 'q':
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) {
          Motor(-tctL, -tctR);
          delay(break_ff);
          MotorStop();
          BZoff();
          break;
        }
      }
      TurnLeft();
      break;

    case 'Q':
      ToFront();
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) {
          Motor(-tctL, -tctR);
          delay(break_ff);
          MotorStop();
          BZoff();
          break;
        }
      }
      TurnLeft();
      break;

    case 'e':
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) {
          Motor(-tctL, -tctR);
          delay(break_ff);
          MotorStop();
          BZoff();
          break;
        }
      }
      TurnRight();
      break;

    case 'E':
      ToFront();
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[0] < Ref && F[7] < Ref) {
          Motor(-tctL, -tctR);
          delay(break_ff);
          MotorStop();
          BZoff();
          break;
        }
      }
      TurnRight();
      break;

    case 'c':
      ToCenter();
      break;

    case 'C':
      ToFront();
      ToCenter();
      break;

    case 'd':
      ToCenter();
      spinr_B();
      break;

    case 'D':
      ToFront();
      ToCenter();
      spinr_B();
      break;

    case 'a':
      ToCenter();
      spinl_B();
      break;

    case 'A':
      ToFront();
      ToCenter();
      spinl_B();
      break;

    case 'b':
      BZon();
      ModeToCenter();
      while (1) {
        ModeToCenterLine();
        ReadCalibrateB();
        if (B[0] > Ref || B[7] > Ref) {
          Motor(-10, -10);
          delay(10);
          Motor(-1, -1);
          delay(1);
          MotorStop();
          BZoff();
          break;
        }
      }
      break;

    case 'B':
      ToFront();
      BZon();
      ModeToCenter();
      while (1) {
        ModeToCenterLine();
        ReadCalibrateB();
        if (B[0] > Ref || B[7] > Ref) {
          Motor(-10, -10);
          delay(10);
          Motor(-1, -1);
          delay(1);
          MotorStop();
          BZoff();
          break;
        }
      }
      break;

    case 'g':
      SetFG(100);
      break;

    case 'G':
      ToFront();
      SetFG(100);
      break;

    default:
      MotorStop(20);
      break;
  }
}

void TrackSelectB(int spd, char x) {
  switch (x) {
    case 's':
      Motor(spd, spd);
      delay(5);
      Move(15, 15, 5);
      Move(10, 10, 1);
      Move(1, 1, 1);
      MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
      break;

    case 'S':
      while (1) {
        PIDB(bctL, bctR, slow_kp_b, slow_kd_b);
        ReadCalibrateB();
        if (B[0] > Ref || B[7] > Ref) {
          Motor(spd, spd);
          delay(5);
          Move(15, 15, 5);
          Move(10, 10, 1);
          Move(1, 1, 1);
          MotorShot();  // active short-brake (back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
          break;
        }
      }
      break;

    case 'p':
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
      break;

    case 'P':
      ToBack();
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
      break;

    case 'l':
    case 'L':
      BackCenter();
      spinl();
      break;

    case 'r':
    case 'R':
      BackCenter();
      spinr();
      break;

    case 'c':
      BackCenter();
      break;

    case 'C':
      ToBack();
      BackCenter();
      break;

    case 'd':
      BackCenter();
      spinr_B();
      break;

    case 'D':
      ToBack();
      BackCenter();
      spinr_B();
      break;

    case 'a':
      BackCenter();
      spinl_B();
      break;

    case 'A':
      ToBack();
      BackCenter();
      spinl_B();
      break;

    case 'e':
      BZon();
      while (1) {
        Motor(-bctL, -bctR );
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(-bctL, -bctR);
        ReadCalibrateB();
          if (B[0] < Ref && B[7] < Ref) {
          Motor(bctL, bctR);
          delay(break_ff);
          MotorStop();
          break;
        }
      }
      TurnLeft_B();
      break;

    case 'E':
      ToBack();
      BZon();
      while (1) {
        Motor(-bctL, -bctR);
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(-bctL, -bctR);
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) {
          Motor(bctL, bctR);
          delay(break_ff);
          MotorStop();
          BZoff();
          break;
        }
      }
      TurnLeft_B();
      break;

    case 'q':
      BZon();
      while (1) {
        Motor(-bctL, -bctR);
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(-bctL / 2, -bctR / 2);
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) {
          BZoff();
          break;
        }
      }
      TurnRight_B();
      break;

    case 'Q':
      ToBack();
      BZon();
      while (1) {
        Motor(-bctL / 2, -bctR / 2);
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) break;
      }
      delay(5);
      while (1) {
        Motor(-bctL / 2, -bctR / 2);
        ReadCalibrateB();
        if (B[0] < Ref && B[7] < Ref) {
          Motor(bctL, bctR);
          delay(break_ff);
          MotorStop();
          BZoff();
          break;
        }
      }
      TurnRight_B();
      break;

    case 'b':
    case 'B':
      BZon();
      ModeToCenterBack();
      while (1) {
        ModeToCenterBackLine();
        ReadCalibrateF();
        if (F[0] > Ref || F[7] > Ref) {
          Motor(10, 10);
          delay(10);
          Motor(1, 1);
          delay(1);
          MotorStop();
          BZoff();
          break;
        }
      }
      break;

    case 'g':
      SetFG(100);
      break;

    case 'G':
      ToBack();
      SetFG(100);
      break;

    default:
      MotorStop(20);
      break;
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
    if (F[0] > Ref || F[7] > Ref || (F[2] > Ref && F[5] > Ref)) {
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
    if (F[0] > Ref) {
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
    if (B[0] > Ref) {
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
    if (F[7] > Ref || (F[1] < Ref && F[2] < Ref && F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void ffr7(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[7] > Ref || (F[1] < Ref && F[2] < Ref && F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref)) break;
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
    if (B[7] > Ref) {
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
    if (F[numm] > Ref || (F[1] < Ref && F[2] < Ref && F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref)) break;
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
  SetRobotAngle();
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
  SetRobotAngle();
}

void setb(int Counter) {
  balanceb(Counter);
}

void balancefc(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 50);
    while (1) {
      Motor(10, 10);
      ReadCalibrateC();
      if (C[1] > RefC) {
        while (1) {
          Motor(0, 10);
          ReadCalibrateC();
          if (C[0] > RefC) { MotorStop(); break; }
        }
      }
      if (C[0] > RefC) {
        while (1) {
          Motor(10, 0);
          ReadCalibrateC();
          if (C[1] > RefC) { MotorStop(); break; }
        }
      }
      if (C[1] > RefC && C[0] > RefC) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
  SetRobotAngle();
}

void balancebc(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 50);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateC();
      if (C[1] > RefC) {
        while (1) {
          Motor(0, -10);
          ReadCalibrateC();
          if (C[0] > RefC) { MotorStop(); break; }
        }
      }
      if (C[0] > Ref) {
        while (1) {
          Motor(-10, 0);
          ReadCalibrateC();
          if (C[1] > RefC) { MotorStop(); break; }
        }
      }
      if (C[1] > Ref && C[0] > Ref) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
  SetRobotAngle();
}

void setfc(int Counter) {
  balancefc(Counter);
}
void setbc(int Counter) {
  balancebc(Counter);
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
  SetRobotAngle();
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
  SetRobotAngle();
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
  SetRobotAngle();
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
  SetRobotAngle();
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

// Ramp เร่ง/ผ่อนความเร็วช่วงต้น-ท้ายระยะทาง (แบบเดียวกับ ffcmg/bbcmg ใน Gyro.h) ใช้เฉพาะกับ
// ffcmcl/ffcmcr/bbcmcl/bbcmcr เพราะเป็นกลุ่มเดียวในชุด CL/CR ที่รู้ระยะทางเป้าหมายล่วงหน้า
// (ffcl/ffcr/bbcl/bbcr วิ่งจนกว่าเซนเซอร์จะเจอเงื่อนไข และ fftimercl/cr/bbtimercl/cr วิ่งตามเวลา
// ทั้งสองแบบไม่รู้ระยะทางล่วงหน้า จึงคำนวณช่วงผ่อนความเร็วท้ายทางไม่ได้) สเกล LeftBaseSpeed/
// RightBaseSpeed (หรือฝั่ง Back) ด้วยอัตราส่วนเดียวกัน เพื่อคง balance ซ้าย-ขวาเดิมไว้ระหว่าง ramp
static void ffcm_ramped(int Speed, int distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง (เหมือน ffcm)

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const int MIN_SPEED = 10;
  bool enableRamp = (distance >= 30 && target_speed > MIN_SPEED);

  while (1) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (target_speed * speed_scale) * delta_time;
    last_time = current_time;

    if (distance > 0 && traveled_distance >= distance) break;

    float rampSpeed = target_speed;
    if (enableRamp) {
      float remaining = distance - traveled_distance;
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining < DECEL_DISTANCE_CM) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * (remaining / DECEL_DISTANCE_CM);
      }
    }
    float scale = rampSpeed / target_speed;

    PIDF(LeftBaseSpeed * scale, RightBaseSpeed * scale, PID_KP_Front, PID_KD_Front);
  }
}

static void bbcm_ramped(int Speed, int distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง (เหมือน bbcm)

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const int MIN_SPEED = 10;
  bool enableRamp = (distance >= 30 && target_speed > MIN_SPEED);

  while (1) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (target_speed * speed_scale) * delta_time;
    last_time = current_time;

    if (distance > 0 && traveled_distance >= distance) break;

    float rampSpeed = target_speed;
    if (enableRamp) {
      float remaining = distance - traveled_distance;
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining < DECEL_DISTANCE_CM) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * (remaining / DECEL_DISTANCE_CM);
      }
    }
    float scale = rampSpeed / target_speed;

    PIDB(BackLeftBaseSpeed * scale, BackRightBaseSpeed * scale, PID_KP_Back, PID_KD_Back);
  }
}

// Ramp แบบเดียวกับ ffcm_ramped/bbcm_ramped ข้างบน แต่ใช้เวลาที่เหลือ (totalTime) แทนระยะทาง
// เพราะ fftimercl/fftimercr/bbtimercl/bbtimercr วิ่งตามเวลาคงที่ ไม่ใช่ระยะทาง จึงรู้ "เวลารวม"
// ล่วงหน้าแทน ใช้คำนวณช่วงเร่ง/ผ่อนความเร็วต้น-ท้ายได้เหมือนกัน ปิด ramp อัตโนมัติถ้า totalTime
// สั้นเกินไป (< 400ms) กันไม่ให้ช่วงเร่ง/ผ่อนกินเวลาที่สั่งวิ่งทั้งหมด
static void fftimer_ramped(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);

  const unsigned long ACCEL_TIME_MS = 150;
  const unsigned long DECEL_TIME_MS = 150;
  const int MIN_SPEED = 10;
  bool enableRamp = (totalTime >= 400 && target_speed > MIN_SPEED);

  unsigned long startTime = millis();
  unsigned long endTime = startTime + totalTime;

  while (millis() <= endTime) {
    float scale = 1.0;
    if (enableRamp) {
      unsigned long now = millis();
      unsigned long elapsed = now - startTime;
      unsigned long remaining = (endTime > now) ? (endTime - now) : 0;
      float rampSpeed = target_speed;
      if (elapsed < ACCEL_TIME_MS) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * ((float)elapsed / ACCEL_TIME_MS);
      } else if (remaining < DECEL_TIME_MS) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * ((float)remaining / DECEL_TIME_MS);
      }
      scale = rampSpeed / target_speed;
    }
    PIDF(LeftBaseSpeed * scale, RightBaseSpeed * scale, PID_KP_Front, PID_KD_Front);
  }
}

static void bbtimer_ramped(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);

  const unsigned long ACCEL_TIME_MS = 150;
  const unsigned long DECEL_TIME_MS = 150;
  const int MIN_SPEED = 10;
  bool enableRamp = (totalTime >= 400 && target_speed > MIN_SPEED);

  unsigned long startTime = millis();
  unsigned long endTime = startTime + totalTime;

  while (millis() <= endTime) {
    float scale = 1.0;
    if (enableRamp) {
      unsigned long now = millis();
      unsigned long elapsed = now - startTime;
      unsigned long remaining = (endTime > now) ? (endTime - now) : 0;
      float rampSpeed = target_speed;
      if (elapsed < ACCEL_TIME_MS) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * ((float)elapsed / ACCEL_TIME_MS);
      } else if (remaining < DECEL_TIME_MS) {
        rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * ((float)remaining / DECEL_TIME_MS);
      }
      scale = rampSpeed / target_speed;
    }
    PIDB(BackLeftBaseSpeed * scale, BackRightBaseSpeed * scale, PID_KP_Back, PID_KD_Back);
  }
}

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
    if (F[7] > Ref || F[1] < Ref && F[2] < Ref && F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}

void fftimercl(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer_ramped(Speed, totalTime);
  set_position = temp;
}

void fftimercr(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer_ramped(Speed, totalTime);
  set_position = temp;
}

void ffcmcl(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm_ramped(Speed, distance);
  set_position = temp;
}

void ffcmcr(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm_ramped(Speed, distance);
  set_position = temp;
}

void fftimercl(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer_ramped(Speed, totalTime);
  TrackSelectF(Speed, select);
  set_position = temp;
}

void fftimercr(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer_ramped(Speed, totalTime);
  TrackSelectF(Speed, select);
  set_position = temp;
}

void ffcmcl(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm_ramped(Speed, distance);
  TrackSelectF(Speed, select);
  set_position = temp;
}

void ffcmcr(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm_ramped(Speed, distance);
  TrackSelectF(Speed, select);
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
  bbtimer_ramped(Speed, totalTime);
  set_position = temp;
}

void bbtimercr(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer_ramped(Speed, totalTime);
  set_position = temp;
}

void bbcmcl(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm_ramped(Speed, distance);
  set_position = temp;
}

void bbcmcr(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm_ramped(Speed, distance);
  set_position = temp;
}

void bbtimercl(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer_ramped(Speed, totalTime);
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbtimercr(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer_ramped(Speed, totalTime);
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbcmcl(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm_ramped(Speed, distance);
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbcmcr(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm_ramped(Speed, distance);
  TrackSelectB(Speed, select);
  set_position = temp;
}

#endif // MYRP_PICO2_MINI_PID_H
