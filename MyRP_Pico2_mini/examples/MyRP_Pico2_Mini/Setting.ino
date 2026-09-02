void Setting() {
  set_Freq("Coreless_Motors");  // หรือ "DC_Motors"
  RobotSetupSpeed();
  /******************** CALIBRATION ********************/


  clampSensorValueF(100, 800);  //สำหรับกรองค่า  calibrate
  clampSensorValueB(100, 800);  //สำหรับกรองค่า  calibrate
  clampSensorValueC(0, 1000);   //สำหรับกรองค่า  calibrate

  /******************** LINE SENSOR CONFIG ********************/
  RefLineValue(500);        // ค่า threshold เซนเซอร์หน้า-หลัง
  RefCenterLineValue(400);  // ค่า threshold เซนเซอร์คู่กลาง
  TrackLineColor(0);        // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
  Dottedline(0);            // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ

  /******************** LINE CENTERING ********************/
  // set_line_center(0);        // เดินธรรมดา เข้ากลางหุ่น
  set_line_center(1);                        // เดินตามเส้น เข้ากลางหุ่น
  SetToCenterSpeed(20);                      // ความเร็วเข้ากลางหุ่น
  /******************** TURN & SPEED CONFIG ********************/
  SetTurnSpeed(50);  // ความเร็วเลี้ยวหุ่น (l L ) (r R)


  TurnSpeedLeft(15, 80, 60);   // เลี้ยวซ้าย (q Q)
  TurnSpeedRight(80, 15, 60);  // เลี้ยวขวา (e E)

  TurnBackSpeedLeft(15, 80, 60);   // เลี้ยวซ้าย (q Q)
  TurnBackSpeedRight(80, 15, 60);  // เลี้ยวขวา (e E)

  ModeSpdPID(0, 100, -5);  // โหมดควบคุมความเร็ว
  // SetRobotPID(0.014, 0.04);  // PID หลัก (ยังไม่ใช้)

  /******************** LINE POSITION ********************/
  set_position_line(3500);    // 0–5000 | 1000=ซ้าย 2500=กลาง 4000=ขวา
  set_position_line_l(1500);   // 0–5000 | วิ่งโค้งซ้าย
  set_position_line_r(5500);  // 0–5000 | วิ่งโค้งขวา
  /******************** DISTANCE SENSOR ********************/
  SetAnalogDistance(28);  // A0–A3 เซนเซอร์ตรวจจับวัตถุ

  /******************** DEBUG / SERIAL MONITOR ********************/
  // SerialDistance();                  // เซนเซอร์วัดระยะ
  // Serial_FrontSensor();              // เซนเซอร์หน้า
  // Serial_BackSensor();               // เซนเซอร์หลัง
  // Serial_CenterSensor();             // เซนเซอร์กลาง
  // Serial_AllSensor();                // เซนเซอร์รวม
  // SerialCalibrate_FrontSensor();     // ค่า Calibrate หน้า
  // SerialCalibrate_BackSensor();      // ค่า Calibrate หลัง
  // SerialCalibrate_CenterSensor();    // ค่า Calibrate กลาง
  // SerialCalibrate_AllSensor();       //ค่า Calibrate รวม
  // SerialPositionF();                 // ตำแหน่งเส้น (หน้า)
  // SerialPositionB();                 // ตำแหน่งเส้น (หลัง)
  // SerialPositionFB();                 // ตำแหน่งเส้น (หน้า-หลัง)
}


void RobotSetupSpeed() {
  SetBalanceSpeedForward();   // ตั้งค่าความสมดุลมอเตอร์เดินหน้า
  SetBalanceSpeedBackward();  // ตั้งค่าความสมดุลมอเตอร์ถอยหลัง
  SetKpKd();                  // ตั้งค่า KP/KD เดินหน้า
  SetKpKdBack();              // ตั้งค่า KP/KD ถอยหลัง
}

// ==================== PID เดินหน้า ====================

void SetKpKd() {
  Set_KP_KD(SPD_10, 0.003, 0.03);   // ความเร็ว 10
  Set_KP_KD(SPD_20, 0.008, 0.10);   // ความเร็ว 20
  Set_KP_KD(SPD_30, 0.008, 0.10);   // ความเร็ว 30
  Set_KP_KD(SPD_40, 0.012, 0.16);   // ความเร็ว 40
  Set_KP_KD(SPD_50, 0.010, 0.12);   // ความเร็ว 50
  Set_KP_KD(SPD_60, 0.018, 0.20);   // ความเร็ว 60
  Set_KP_KD(SPD_70, 0.018, 0.20);   // ความเร็ว 70
  Set_KP_KD(SPD_80, 0.024, 0.24);   // ความเร็ว 80
  Set_KP_KD(SPD_90, 0.027, 0.27);   // ความเร็ว 90
  Set_KP_KD(SPD_100, 0.030, 0.30);  // ความเร็ว 100
}

// ==================== PID ถอยหลัง ====================

void SetKpKdBack() {
  Set_KP_KD_Back(SPD_10, 0.005, 0.05);   // ความเร็ว 10
  Set_KP_KD_Back(SPD_20, 0.009, 0.09);   // ความเร็ว 20
  Set_KP_KD_Back(SPD_30, 0.011, 0.22);   // ความเร็ว 30
  Set_KP_KD_Back(SPD_40, 0.014, 0.28);   // ความเร็ว 40
  Set_KP_KD_Back(SPD_50, 0.018, 0.20);   // ความเร็ว 50
  Set_KP_KD_Back(SPD_60, 0.014, 0.16);   // ความเร็ว 60
  Set_KP_KD_Back(SPD_70, 0.023, 0.46);   // ความเร็ว 70
  Set_KP_KD_Back(SPD_80, 0.028, 0.56);   // ความเร็ว 80
  Set_KP_KD_Back(SPD_90, 0.031, 0.62);   // ความเร็ว 90
  Set_KP_KD_Back(SPD_100, 0.033, 0.66);  // ความเร็ว 100
}

// ==================== Balance เดินหน้า ====================

void SetBalanceSpeedForward() {
  // ข้างไหนแรงกว่า ให้เพิ่มค่าข้างนั้น
  // setBalanceSpeed(SPEED, LEFT, RIGHT);

  setBalanceSpeed(SPD_10, 0, 0);   // ความเร็ว 10
  setBalanceSpeed(SPD_20, 0, 0);   // ความเร็ว 20
  setBalanceSpeed(SPD_30, 0, 0);   // ความเร็ว 30
  setBalanceSpeed(SPD_40, 0, 0);   // ความเร็ว 40
  setBalanceSpeed(SPD_50, 0, 0);   // ความเร็ว 50
  setBalanceSpeed(SPD_60, 0, 0);   // ความเร็ว 60
  setBalanceSpeed(SPD_70, 0, 0);   // ความเร็ว 70
  setBalanceSpeed(SPD_80, 0, 0);   // ความเร็ว 80
  setBalanceSpeed(SPD_90, 0, 0);   // ความเร็ว 90
  setBalanceSpeed(SPD_100, 0, 0);  // ความเร็ว 100
}

// ==================== Balance ถอยหลัง ====================

void SetBalanceSpeedBackward() {
  // ข้างไหนแรงกว่า ให้เพิ่มค่าข้างนั้น
  // setBalanceBackSpeed(SPEED, LEFT, RIGHT);

  setBalanceBackSpeed(SPD_10, 0, 0);   // ความเร็ว 10
  setBalanceBackSpeed(SPD_20, 0, 0);   // ความเร็ว 20
  setBalanceBackSpeed(SPD_30, 0, 0);   // ความเร็ว 30
  setBalanceBackSpeed(SPD_40, 0, 0);   // ความเร็ว 40
  setBalanceBackSpeed(SPD_50, 0, 0);   // ความเร็ว 50
  setBalanceBackSpeed(SPD_60, 0, 0);   // ความเร็ว 60
  setBalanceBackSpeed(SPD_70, 0, 0);   // ความเร็ว 70
  setBalanceBackSpeed(SPD_80, 0, 0);   // ความเร็ว 80
  setBalanceBackSpeed(SPD_90, 0, 0);   // ความเร็ว 90
  setBalanceBackSpeed(SPD_100, 0, 0);  // ความเร็ว 100
}
