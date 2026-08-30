#ifndef MYRP_PICO2_MINI_H
#define MYRP_PICO2_MINI_H

#include "MyRP_Pico2_mini_Buzzer.h"
#include "MyRP_Pico2_mini_Sensor.h"
#include "MyRP_Pico2_mini_Motor.h"
#include "MyRP_Pico2_mini_PID.h"
#include "MyRP_Pico2_mini_Servo.h"
#include "MyRP_Pico2_mini_Gyro.h"

void RobotSetup() {
  Serial.begin(115200);
  delay(100);
  Wire.setSDA(4);
  Wire.setSCL(5);

  // ตั้งเป็น GPIO ชั่วคราวเพื่อ clock pulses
  pinMode(4, OUTPUT);  // SDA
  pinMode(5, OUTPUT);  // SCL
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  delay(10);

  // ส่ง 9-10 clock pulses เพื่อเคลียร์ slave ที่อาจค้าง
  for (int i = 0; i < 10; i++) {
    digitalWrite(5, LOW);
    delayMicroseconds(5);
    digitalWrite(5, HIGH);
    delayMicroseconds(5);
  }

  // ปล่อย bus เป็น input (pull-up จะดึง HIGH)
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  delay(10);

  // เริ่ม I2C ใหม่
  Wire.begin();

  bat.begin();
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(DC_Motors ? 1000 : 20000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(9, OUTPUT);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  adc.begin(14, 15, 16, 13);   // Initialize ADC ครั้งเดียว

  loadCalibration();
  loadCalibration_LOCAL();
  if (my.begin()) {
    my.resetAngles();
  } else {
    Serial.println("BMI160 init failed!");
  }

  // ตั้งความเร็ว I2C หลังสุด เพราะ bat.begin()/my.begin() เรียก Wire.begin() ซ้ำข้างใน
  // ซึ่งจะรีเซ็ตความเร็วบัสกลับเป็นค่าเริ่มต้น ถ้าตั้งไว้ก่อนหน้านี้จะโดนทับ
  Wire.setClock(400000);        // แนะนำ 800 kHz (เสถียรกว่า 1 MHz ในทางปฏิบัติ)

  blink(3);
  Serial.println("MyRP_Pico2 Class Ready for Competition!");
}

int ADC_i2c() {
  long ADC01 = 0;
  int adc_01;

  // อ่านจาก Wire (I2C0)
  Wire.requestFrom(MCP3421_ADDR, 4);
  if (Wire.available() == 4) {
    byte b1 = Wire.read();
    byte b2 = Wire.read();
    byte b3 = Wire.read();
    byte cfg = Wire.read();

    ADC01 = ((long)b1 << 16) | ((long)b2 << 8) | b3;
    if (b1 & 0x80) ADC01 |= 0xFF000000; // sign-extend
  }
  adc_01 = map(ADC01, 524048, 282, 4000, 0);
  //Serial.print("ADC01: "); Serial.print(adc_01);
  //delay(10);
  return adc_01;
}

// ==================== sw() - เมนู Calibration + แสดงเซนเซอร์ ====================
void sw()
{MotorStop();
    tone(9, 2000, 100);   // โด
    delay(100);
    tone(9, 2400, 100);   // เร
    delay(100);
    tone(9, 3000, 160);   // มี
    delay(500);
    tone(9, 3000, 60);
    delay(80);
    tone(9, 3000, 60);
    delay(80);

    unsigned long pressStartTime = 0;
    bool isPressed = false;
    unsigned long lastBuzz = 0;
    // ส่วนตรวจสอบปุ่ม + แสดงค่าเซนเซอร์
    while (true)
    {
           bat.update_led();
            updateBattery();
            float voltage = getBatteryVoltage(); // เตือนเมื่อแบตต่ำกว่า 11.0V แต่ยังไม่ถึงขั้นวิกฤติ
            if (voltage <= 11.0 && voltage > 7.0) {
                unsigned long now = millis();
                // ส่งเสียงทุก 3 วินาที (3000 ms)
                if (now - lastBuzz >= 3000) {
                    Serial.println(voltage);              // แสดงแรงดันไฟ
                    // เสียงเตือนแบบ Sci-Fi / Cyber Tech (เท่และชัดเจน)
                    tone(9, 2800, 50);
                    delay(30);
                    tone(9, 3200, 50);
                    delay(30);
                    tone(9, 2400, 120);
                    delay(100);
                    tone(9, 1800, 80);
                    lastBuzz = now;        // อัพเดทเวลาล่าสุดที่ส่งเสียง
                }
              }

        // ปุ่มบน (ปุ่ม 3) → Calibrate A
        if (digitalRead(3) == LOW)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            tone(9, 3000, 300);
            calibrateA();        // แทน get_maxMinA()
            saveCalibA_LOCAL();
            blink(5);
            delay(300);
            digitalWrite(LED_BUILTIN, LOW);
        }

        if(ADC_i2c() < 2500)
                {
                  digitalWrite(LED_BUILTIN, HIGH);
                        delay(200);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(200);
                        digitalWrite(LED_BUILTIN, HIGH);
                        tone(9, 3000, 100);
                  calibrateB();
                  saveCalibB_LOCAL();
                  digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        while (digitalRead(2) == LOW);   // รอปล่อยปุ่ม
                        delay(200);
                  delay(200); // รอ 1 วินาที
                }

        // แสดงค่าซีเรียลทุก 100ms
        Serial.print("From A: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorA(i));
            Serial.print(" ");
        }
        Serial.print("   From B: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorB(i));
            Serial.print(" ");
        }
         Serial.print("   From C: ");
    Serial.print(analogRead(26));
    Serial.print(" ");
    Serial.print(analogRead(27));
    Serial.print(" ");
    Serial.println();
    delay(100);  // ให้ตรงกับที่คอมเมนต์บอกไว้ว่า "ทุก 100ms" กันสแปม Serial/สแกน ADC รัวๆ

        // ตรวจจับกดค้างปุ่ม 2 นาน 3 วินาที → Calibrate C

        if (digitalRead(2) == LOW) 
              {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
                
                if (!isPressed) \
                  {
                    pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
                    isPressed = true;
                  } 
                else 
                  {
                    unsigned long pressDuration = millis() - pressStartTime;   
                     
                    if (pressDuration >= 3000) 
                      { 
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(200);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(200);
                        digitalWrite(LED_BUILTIN, HIGH);
                        tone(9, 3000, 100);
                        tone(9, 3000, 200);
                        calibrateC();
                        saveCalibC_LOCAL();
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        while (digitalRead(2) == LOW);   // รอปล่อยปุ่ม
                        delay(200);
                                while (digitalRead(2) == LOW);  // รอให้ปล่อยปุ่ม
                                delay(200);  // ป้องกันการเด้งของปุ่ม
                              }
                  }
              } 
            else 
              {
                if (isPressed) 
                  {
                    unsigned long pressDuration = millis() - pressStartTime;
                    
                    if (pressDuration >= 50 && pressDuration < 3000)
                      {
                        Serial.println("Entering Mode B");
                        break;
                      }
                    isPressed = false;
                  }
              }
          }




    // โหลดค่า calibration จาก EEPROM หลังออกจากเมนู
    loadCalibration();
loadCalibration_LOCAL();
    tone(9, 3000, 400);
    delay(500);
}




// ==================== sw(timeoutSec) - เหมือน sw() แต่ออกจากเมนูอัตโนมัติถ้าไม่กดปุ่มภายในเวลาที่กำหนด ====================


#endif // MYRP_PICO2_MINI_H
