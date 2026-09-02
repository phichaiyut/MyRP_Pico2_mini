#ifndef MYRP_PICO2_MINI_BUZZER_H
#define MYRP_PICO2_MINI_BUZZER_H

#define ToNe 3000
#define buzzer_pin 9

void Beep(int delayb) {
  tone(buzzer_pin, ToNe);
  delay(delayb);
  noTone(buzzer_pin);
}

void Beep2(int freq, int delayb) {
  tone(buzzer_pin, freq);
  delay(delayb);
  noTone(buzzer_pin);
}

void BZon() {
  tone(buzzer_pin, ToNe);
}

void BZoff() {
  noTone(buzzer_pin);
}

void beep(int freq, int dur) {
  tone(9, freq, dur);
  delay(dur + 50);
}

// เสียง "ปี๊บ" สั้น แหลม คล้ายเครื่องยิงบาร์โค้ดใน supermarket
void BeepScanner() {
  beep(ToNe, 80);
}

void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(150);
  }
}

#endif // MYRP_PICO2_MINI_BUZZER_H
