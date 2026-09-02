#ifndef MYRP_PICO2_MINI_SERVO_H
#define MYRP_PICO2_MINI_SERVO_H

#include <Servo.h>

// กำหนดขาเซอร์โว
#define Servo0 0
#define Servo1 1
#define Servo10 10
#define Servo28 28

// สร้างออบเจ็กต์เซอร์โว
Servo Servo_0;
Servo Servo_1;
Servo Servo_10;
Servo Servo_28;

// ตัวแปรสำหรับเก็บค่า trim และมุมก่อนหน้า
int Servo_tim10 = 0;
int Servo_tim0 = 0;
int Servo_tim1 = 0;
int Servo_tim28 = 0;

// ฟังก์ชันตั้งค่า trim
void S0_trim(int _s0) {
  Servo_tim0 = _s0;
}

void S1_trim(int _s1) {
  Servo_tim1 = _s1;
}

void S10_trim(int _s10) {
  Servo_tim10 = _s10;
}

void S28_trim(int _s28) {
  Servo_tim28 = _s28;
}

// ฟังก์ชันควบคุมเซอร์โว
void Servo(int servo, int angle) {
  if (servo == 10) {
    if (!Servo_10.attached()) Servo_10.attach(Servo10, 500, 2500);
    Servo_10.write(constrain(angle + Servo_tim10, 0, 180));
  } else if (servo == 0) {
    if (!Servo_0.attached()) Servo_0.attach(Servo0, 500, 2500);
    Servo_0.write(constrain(180 - angle - Servo_tim0, 0, 180));
  } else if (servo == 1) {
    if (!Servo_1.attached()) Servo_1.attach(Servo1, 500, 2500);
    Servo_1.write(constrain(180 - angle - Servo_tim1, 0, 180));
  } else if (servo == 28) {
    if (!Servo_28.attached()) Servo_28.attach(Servo28, 500, 2500);
    Servo_28.write(constrain(angle + Servo_tim28, 0, 180));
  }
}

int currentServo = -1;
int currentAngle = 90;

int pos[3] ={90,90,90};

void SerialServoControl() {
  Serial.println("Serial Servo Control Mode");
  Serial.println("Type: servo angle  (servo id: 0, 1, 10, 28) (ex: 10 90)");

  while (1) {
    // ออกจากโหมดถ้าพิมพ์ 'exit'
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("exit")) {
        Serial.println("Exiting Serial Servo Control Mode");
        break;
      }
    }

    // รับคำสั่งจาก Serial
    if (Serial.available()) {
      int s = Serial.parseInt();
      int a = Serial.parseInt();

      if (a >= 0 && a <= 180) {
        currentServo = s;
        currentAngle = a;

        Serial.print("Set Servo ");
        Serial.print(currentServo);
        Serial.print(" -> ");
        Serial.print(currentAngle);
        Serial.println(" deg");
      } else {
        Serial.println("Angle must be 0-180");
      }

      while (Serial.available()) Serial.read();
    }

    // เขียนค่าเดิมซ้ำเรื่อย ๆ
    if (currentServo != -1) {
      Servo(currentServo, currentAngle);
    }
    delay(50); // ~50Hz เหมาะกับ servo
  }
}

void Servo(int x, int y, int z) {
  MotorStop();
  int a[] = {x, y, z}, s[] = {Servo1, Servo10, Servo0};
  for (int i = 0; i < 3; i++) Servo(s[i], pos[i] = a[i]);
  delay(20);
}

void Servo(int target1, int target2, int target3, int spd) {
  MotorStop();
  int target[3] = {target1, target2, target3};
  int sv[3] = {Servo1, Servo10, Servo0};

  while (pos[0] != target[0] || pos[1] != target[1] || pos[2] != target[2]) {
    for (int i = 0; i < 3; i++) {
      pos[i] += (pos[i] < target[i]) - (pos[i] > target[i]);
      Servo(sv[i], pos[i]);
    }
    delay(spd);
  }

  delay(20);
}

void armupdown(int x, int spd) {
  Servo(x, pos[1], pos[2], spd);
}

void arm_left_right(int l, int r, int spd) {
  Servo(pos[0], l, r, spd);
}

void armupdown(int x) {
  Servo(1, pos[0] = x);
}

void arm_left_right(int l, int r) {
  Servo(10, pos[1] = l);
  Servo(0,  pos[2] = r);
}

#endif // MYRP_PICO2_MINI_SERVO_H
