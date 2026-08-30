#ifndef MYRP_PICO2_MINI_SENSOR_H
#define MYRP_PICO2_MINI_SENSOR_H

#define NUM_SENSORS 8
#include <Wire.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "my_MCP3008s.h"
#define EEPROM_ADDRESS 0x50
#define MCP3421_ADDR 0x68  // I2C Address when A0 = GND

// EEPROM (flash) ภายใน Pico 2 ตัวที่รัน sketch นี้เอง
// Layout: A max[0..15] A min[16..31] B max[32..47] B min[48..63] C max[64..67] C min[68..71]
#define LOCAL_EEPROM_SIZE 128

my_MCP3008s adc;
int F[NUM_SENSORS], B[NUM_SENSORS], C[2];

int sensorMaxA[8], sensorMinA[8];
int sensorMaxB[8], sensorMinB[8];
int sensorMaxC[2], sensorMinC[2];

uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7 };
uint8_t B_PIN[NUM_SENSORS] = { 7, 6, 5, 4, 3, 2, 1, 0 };
uint8_t C_PIN[2] = {26, 27};

// ใช้เฉพาะฟังก์ชันที่มี _LOCAL (เก็บ/โหลด EEPROM flash ของ Pico 2 ตัวนี้เอง)
int minValueF[NUM_SENSORS], maxValueF[NUM_SENSORS];
int minValueB[NUM_SENSORS], maxValueB[NUM_SENSORS];
int minValueC[2], maxValueC[2];

#define CCL 1
#define CCR 0
int Ref = 500;
int RefC = 500;
int LineColor = 0;
int DIST = A3;
// FRONT
int FRONT_MIN = 80;
int FRONT_MAX = 900;

// CENTER
int CENTER_MIN = 200;
int CENTER_MAX = 900;

// BACK
int BACK_MIN = 0;
int BACK_MAX = 1000;

// ==================== Sensor Reading ====================
uint16_t read_sensorA(int sensor) {
  if (sensor < 0 || sensor > 7) return 0;
  adc.begin(14, 15, 16, 13);
  adc.begin(14, 15, 16, 17);  //adc.begin(5, 4, 12, 20);    // 5=clk, 4=IN, 12=out
  return adc.readADC(sensor);
}

uint16_t read_sensorB(int sensor) {
  if (sensor < 0 || sensor > 7) return 0;
  adc.begin(14, 15, 16, 17);
  adc.begin(14, 15, 16, 13);  //adc.begin(5, 4, 12, 20);    // 5=clk, 4=IN, 12=out
  return adc.readADC(sensor);
}

// ==================== EEPROM Helper ====================
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  for (int i = 0; i < dataLength; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
  delay(5);
}

void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, dataLength);
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read();
    }
  }
}

// ==================== EEPROM Helper (local flash, arduino-pico core) ====================
void ensureEEPROMBegin_LOCAL() {
  static bool started = false;
  if (!started) {
    EEPROM.begin(LOCAL_EEPROM_SIZE);
    started = true;
  }
}

void writeEEPROM_LOCAL(unsigned int eeAddress, byte *data, int dataLength) {
  ensureEEPROMBegin_LOCAL();
  for (int i = 0; i < dataLength; i++) {
    EEPROM.write(eeAddress + i, data[i]);
  }
  EEPROM.commit();
}

void readEEPROM_LOCAL(unsigned int eeAddress, byte *buffer, int dataLength) {
  ensureEEPROMBegin_LOCAL();
  for (int i = 0; i < dataLength; i++) {
    buffer[i] = EEPROM.read(eeAddress + i);
  }
}

// ==================== Calibration Functions (เวอร์ชันสมบูรณ์) ====================

void calibrateA() {
  const int samples = 1000;
  int values[8][1000];

  Serial.println("กำลัง calibrate Sensor A (Line Front)...");

  for (int s = 0; s < samples; s++) {
    for (int i = 0; i < 8; i++) {
      values[i][s] = read_sensorA(i);
      delay(1);
    }
  }

  for (int i = 0; i < 8; i++) {
    sensorMaxA[i] = values[i][0];
    sensorMinA[i] = values[i][0];
    for (int s = 1; s < samples; s++) {
      int v = values[i][s];
      if (v > sensorMaxA[i]) sensorMaxA[i] = v;
      if (v < sensorMinA[i]) sensorMinA[i] = v;
    }
  }

  // บันทึกลง EEPROM
  byte buf[16];
  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(sensorMaxA[i]);
    buf[i*2+1] = lowByte(sensorMaxA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 0, buf, 16);

  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(sensorMinA[i]);
    buf[i*2+1] = lowByte(sensorMinA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 16, buf, 16);

  beep(3000, 100); delay(150);
  beep(3000, 200);
  Serial.println("✓ Calibrate Sensor A เสร็จสิ้น");
}

// บันทึกค่า Calibrate Sensor A ลง EEPROM (flash ของ Pico 2 ตัวนี้เอง)
// ใช้ค่าที่ calibrateA() เก็บไว้แล้ว (sensorMaxA/sensorMinA) ไม่สุ่มอ่านซ้ำ
// เพื่อให้ค่าภายนอก (EEPROM 0x50) กับ flash local เป็นค่าเดียวกันเสมอ
void saveCalibA_LOCAL() {
  for (int i = 0; i < 8; i++) {
    maxValueF[i] = sensorMaxA[i];
    minValueF[i] = sensorMinA[i];
  }

  byte buf[16];
  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(maxValueF[i]);
    buf[i*2+1] = lowByte(maxValueF[i]);
  }
  writeEEPROM_LOCAL(0, buf, 16);

  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(minValueF[i]);
    buf[i*2+1] = lowByte(minValueF[i]);
  }
  writeEEPROM_LOCAL(16, buf, 16);

  Serial.println("✓ บันทึก Calibrate Sensor A ลง EEPROM (LOCAL) เสร็จสิ้น");
}

void calibrateB() {
  const int samples = 1000;
  int values[8][1000];

  Serial.println("กำลัง calibrate Sensor B (Line Back)...");

  for (int s = 0; s < samples; s++) {
    for (int i = 0; i < 8; i++) {
      values[i][s] = read_sensorB(i);
      delay(1);
    }
  }

  for (int i = 0; i < 8; i++) {
    sensorMaxB[i] = values[i][0];
    sensorMinB[i] = values[i][0];
    for (int s = 1; s < samples; s++) {
      int v = values[i][s];
      if (v > sensorMaxB[i]) sensorMaxB[i] = v;
      if (v < sensorMinB[i]) sensorMinB[i] = v;
    }
  }

  byte buf[16];
  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(sensorMaxB[i]);
    buf[i*2+1] = lowByte(sensorMaxB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 32, buf, 16);

  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(sensorMinB[i]);
    buf[i*2+1] = lowByte(sensorMinB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 48, buf, 16);

  beep(3000, 100); delay(150);
  beep(3000, 200);
  Serial.println("✓ Calibrate Sensor B เสร็จสิ้น");
}

// บันทึกค่า Calibrate Sensor B ลง EEPROM (flash ของ Pico 2 ตัวนี้เอง)
// ใช้ค่าที่ calibrateB() เก็บไว้แล้ว (sensorMaxB/sensorMinB) ไม่สุ่มอ่านซ้ำ
// เพื่อให้ค่าภายนอก (EEPROM 0x50) กับ flash local เป็นค่าเดียวกันเสมอ
void saveCalibB_LOCAL() {
  for (int i = 0; i < 8; i++) {
    maxValueB[i] = sensorMaxB[i];
    minValueB[i] = sensorMinB[i];
  }

  byte buf[16];
  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(maxValueB[i]);
    buf[i*2+1] = lowByte(maxValueB[i]);
  }
  writeEEPROM_LOCAL(32, buf, 16);

  for (int i = 0; i < 8; i++) {
    buf[i*2]   = highByte(minValueB[i]);
    buf[i*2+1] = lowByte(minValueB[i]);
  }
  writeEEPROM_LOCAL(48, buf, 16);

  Serial.println("✓ บันทึก Calibrate Sensor B ลง EEPROM (LOCAL) เสร็จสิ้น");
}

void calibrateC() {
  const int samples = 1000;
  const int PIN_C0 = 26;
  const int PIN_C1 = 27;
  int values[2][1000];

  Serial.println("กำลัง calibrate Sensor C (Side Sensors)...");

  for (int s = 0; s < samples; s++) {
    values[0][s] = analogRead(PIN_C0);
    values[1][s] = analogRead(PIN_C1);
    delay(5);
  }

  for (int i = 0; i < 2; i++) {
    sensorMaxC[i] = 0;
    sensorMinC[i] = 4095;
    for (int s = 0; s < samples; s++) {
      uint16_t v = values[i][s];
      if (v > sensorMaxC[i]) sensorMaxC[i] = v;
      if (v < sensorMinC[i]) sensorMinC[i] = v;
    }
    sensorMaxC[i] = constrain(sensorMaxC[i], 0, 4000);
    sensorMinC[i] = constrain(sensorMinC[i], 0, 4000);
  }

  uint8_t buf[4];
  for (int i = 0; i < 2; i++) {
    buf[i*2]   = highByte(sensorMaxC[i]);
    buf[i*2+1] = lowByte(sensorMaxC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 64, buf, 4);

  for (int i = 0; i < 2; i++) {
    buf[i*2]   = highByte(sensorMinC[i]);
    buf[i*2+1] = lowByte(sensorMinC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 68, buf, 4);

  beep(3000, 150); delay(200);
  beep(3000, 200);
  Serial.println("✓ Calibrate Sensor C เสร็จสิ้น");
}

// บันทึกค่า Calibrate Sensor C ลง EEPROM (flash ของ Pico 2 ตัวนี้เอง)
// ใช้ค่าที่ calibrateC() เก็บไว้แล้ว (sensorMaxC/sensorMinC) ไม่สุ่มอ่านซ้ำ
// เพื่อให้ค่าภายนอก (EEPROM 0x50) กับ flash local เป็นค่าเดียวกันเสมอ
void saveCalibC_LOCAL() {
  for (int i = 0; i < 2; i++) {
    maxValueC[i] = sensorMaxC[i];
    minValueC[i] = sensorMinC[i];
  }

  uint8_t buf[4];
  for (int i = 0; i < 2; i++) {
    buf[i*2]   = highByte(maxValueC[i]);
    buf[i*2+1] = lowByte(maxValueC[i]);
  }
  writeEEPROM_LOCAL(64, buf, 4);

  for (int i = 0; i < 2; i++) {
    buf[i*2]   = highByte(minValueC[i]);
    buf[i*2+1] = lowByte(minValueC[i]);
  }
  writeEEPROM_LOCAL(68, buf, 4);

  Serial.println("✓ บันทึก Calibrate Sensor C ลง EEPROM (LOCAL) เสร็จสิ้น");
}

// ==================== Load & Print Calibration ====================

void read_eepA() {
  byte buf[16];
  readEEPROM(EEPROM_ADDRESS, 0, buf, 16);
  for (int i = 0; i < 8; i++) {
    sensorMaxA[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 16, buf, 16);
  for (int i = 0; i < 8; i++) {
    sensorMinA[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
}

void read_eepB() {
  byte buf[16];
  readEEPROM(EEPROM_ADDRESS, 32, buf, 16);
  for (int i = 0; i < 8; i++) {
    sensorMaxB[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 48, buf, 16);
  for (int i = 0; i < 8; i++) {
    sensorMinB[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
}

void read_eepC() {
  byte buf[4];
  readEEPROM(EEPROM_ADDRESS, 64, buf, 4);
  for (int i = 0; i < 2; i++) {
    sensorMaxC[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 68, buf, 4);
  for (int i = 0; i < 2; i++) {
    sensorMinC[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
}

void read_eepA_LOCAL() {
  byte buf[16];
  readEEPROM_LOCAL(0, buf, 16);
  for (int i = 0; i < 8; i++) {
    maxValueF[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
  readEEPROM_LOCAL(16, buf, 16);
  for (int i = 0; i < 8; i++) {
    minValueF[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
}

void read_eepB_LOCAL() {
  byte buf[16];
  readEEPROM_LOCAL(32, buf, 16);
  for (int i = 0; i < 8; i++) {
    maxValueB[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
  readEEPROM_LOCAL(48, buf, 16);
  for (int i = 0; i < 8; i++) {
    minValueB[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
}

void read_eepC_LOCAL() {
  byte buf[4];
  readEEPROM_LOCAL(64, buf, 4);
  for (int i = 0; i < 2; i++) {
    maxValueC[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
  readEEPROM_LOCAL(68, buf, 4);
  for (int i = 0; i < 2; i++) {
    minValueC[i] = (buf[i*2] << 8) | buf[i*2 + 1];
  }
}

void printCalibration() {
  Serial.println("\n=== Calibration Values Loaded ===");
  Serial.println("--- Sensor A (Front Line) ---");
  for (int i = 0; i < 8; i++) {
    Serial.printf("A%d → Max: %4d  Min: %4d\n", i, sensorMaxA[i], sensorMinA[i]);
  }

  Serial.println("--- Sensor B (Back Line) ---");
  for (int i = 0; i < 8; i++) {
    Serial.printf("B%d → Max: %4d  Min: %4d\n", i, sensorMaxB[i], sensorMinB[i]);
  }

  Serial.println("--- Sensor C (Side) ---");
  Serial.printf("C0 → Max: %4d  Min: %4d\n", sensorMaxC[0], sensorMinC[0]);
  Serial.printf("C1 → Max: %4d  Min: %4d\n", sensorMaxC[1], sensorMinC[1]);
  Serial.println("=================================\n");
}

// ใช้เฉพาะกับค่า _LOCAL (minValue*/maxValue*)
void printCalibration_LOCAL() {
  Serial.println("\n=== Calibration Values Loaded (LOCAL) ===");
  Serial.println("--- Sensor A (Front Line) ---");
  for (int i = 0; i < 8; i++) {
    Serial.printf("A%d → Max: %4d  Min: %4d\n", i, maxValueF[i], minValueF[i]);
  }

  Serial.println("--- Sensor B (Back Line) ---");
  for (int i = 0; i < 8; i++) {
    Serial.printf("B%d → Max: %4d  Min: %4d\n", i, maxValueB[i], minValueB[i]);
  }

  Serial.println("--- Sensor C (Side) ---");
  Serial.printf("C0 → Max: %4d  Min: %4d\n", maxValueC[0], minValueC[0]);
  Serial.printf("C1 → Max: %4d  Min: %4d\n", maxValueC[1], minValueC[1]);
  Serial.println("=================================\n");
}

void loadCalibration() {
  read_eepA();
  read_eepB();
  read_eepC();
  printCalibration();
}

void loadCalibration_LOCAL() {
  read_eepA_LOCAL();
  read_eepB_LOCAL();
  read_eepC_LOCAL();
  printCalibration_LOCAL();
}

void ReadF() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = read_sensorA(F_PIN[i]);
  }
}

void ReadB() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = read_sensorB(B_PIN[i]);
  }
}

void ReadC() {
  for (int i = 0; i < 2; i++) {
    C[i] = analogRead(C_PIN[i]);
  }
}

void SetAnalogDistance(int x) {
  DIST = x;
}

void TrackLineColor(int Col) {
  LineColor = Col;
}

void clampSensorValueF(int x, int y) {
  FRONT_MIN = x;
  FRONT_MAX = y;
}

void clampSensorValueC(int x, int y) {
  CENTER_MIN = x;
  CENTER_MAX = y;
}

void clampSensorValueB(int x, int y) {
  BACK_MIN = x;
  BACK_MAX = y;
}

void ReadCalibrateF() {
  ReadF();
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = constrain(F[i], minValueF[F_PIN[i]], maxValueF[F_PIN[i]]);
    int16_t x;
    if (LineColor == 0)
      x = map(F[i], minValueF[F_PIN[i]], maxValueF[F_PIN[i]], 1000, 0);
    else
      x = map(F[i], minValueF[F_PIN[i]], maxValueF[F_PIN[i]], 0, 1000);
    if (x < FRONT_MIN) x = 0;
    if (x > FRONT_MAX) x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    F[i] = x;
  }
}

void ReadCalibrateC() {
  ReadC();

  for (int i = 0; i < 2; i++) {
    C[i] = constrain(C[i], minValueC[i], maxValueC[i]);
    int16_t x;
    if (LineColor == 0)
      x = map(C[i], minValueC[i], maxValueC[i], 1000, 0);
    else
      x = map(C[i], minValueC[i], maxValueC[i], 0, 1000);
    if (x < CENTER_MIN) x = 0;
    if (x > CENTER_MAX) x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    C[i] = x;
  }
}

void ReadCalibrateB() {
  ReadB();

  for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = constrain(B[i], minValueB[B_PIN[i]], maxValueB[B_PIN[i]]);
    int16_t x;
    if (LineColor == 0)
      x = map(B[i], minValueB[B_PIN[i]], maxValueB[B_PIN[i]], 1000, 0);
    else
      x = map(B[i], minValueB[B_PIN[i]], maxValueB[B_PIN[i]], 0, 1000);
    if (x < BACK_MIN) x = 0;
    if (x > BACK_MAX) x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    B[i] = x;
  }
}

void ReadSensor() {
  ReadCalibrateF();
  ReadCalibrateB();
  ReadCalibrateC();
}

void ReadSensorRaw() {
  ReadF();
  ReadB();
  ReadC();
}

void RefLineValue(int x) {
  Ref = x;
}

void RefCenterLineValue(int x) {
  RefC = x;
}

///////////////////////////////////////////////////////////////////////////////////////

void Serial_FrontSensor() {
  while (1) {
    ReadSensorRaw();
    for (int _serialF = 0; _serialF < NUM_SENSORS; _serialF++) {
      Serial.print(F[_serialF]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}

void Serial_BackSensor() {
  while (1) {
    ReadSensorRaw();
    for (int _serialB = 0; _serialB < NUM_SENSORS; _serialB++) {
      Serial.print(B[_serialB]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}

void Serial_CenterSensor() {
  while (1) {
    ReadSensorRaw();
    for (int _serialA = 0; _serialA < 2; _serialA++) {
      Serial.print(C[_serialA]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}

void SerialCalibrate_FrontSensor() {
  while (1) {
    ReadSensor();
    for (int _serialF = 0; _serialF < NUM_SENSORS; _serialF++) {
      Serial.print(F[_serialF]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}

void SerialCalibrate_BackSensor() {
  while (1) {
    ReadSensor();
    for (int _serialB = 0; _serialB < NUM_SENSORS; _serialB++) {
      Serial.print(B[_serialB]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}

void SerialCalibrate_CenterSensor() {
  while (1) {
    ReadSensor();
    for (int _serialC = 0; _serialC < 2; _serialC++) {
      Serial.print(C[_serialC]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}

void Serial_AllSensor() {
  while (1) {
    ReadSensorRaw();

    Serial.print("F: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }

    Serial.print("B: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }

    Serial.print("C: ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }

    Serial.println();
    delay(50);
  }
}

void SerialCalibrate_AllSensor() {
  while (1) {
    ReadSensor();
    Serial.print("F : ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }

    Serial.print("B : ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }

    Serial.print("C : ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }

    Serial.println();
    delay(100);
  }
}

void SerialDistance() {
  while (1) {
    Serial.print("ADC : ");
    Serial.println(analogRead(DIST));
    delay(100);
  }
}

#endif // MYRP_PICO2_MINI_SENSOR_H
