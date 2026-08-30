#include <MyRP_Pico2_mini.h>

//----------------------------------------------------------------------------->> ตั้งค่ามือจับ

void setup() {
  RobotSetup() ;
  
  Setting();
  //_______________
  S0_trim(0); //ตั้งค่าองศา แขนด้านขวา
  S1_trim(0); //ตั้งค่าองศา ขึ้น - ลง
  S10_trim(0); //ตั้งค่าองศา แขนด้านซ้าย
  S28_trim(0); 
  //__________________
  // arm_down_open();
  arm_up_open();
  // arm_down_close();
  // arm_up_close();
  // arm_down();
  sw();
  delay(200);

  SetRobotAngle() ;
  // SerialServoControl(); // 0 10 1 ตั้งค่าเซอร์โวผ่าน serial
  Mission();

  MotorStop();
}

void loop() {
  // HoldAngle();
  Serial.println(gyroZ());
  sw();
  delay(200);
  // SetRobotAngle() ;
  // SerialServoControl(); // 0 10 1 ตั้งค่าเซอร์โวผ่าน serial
  Mission();
}
