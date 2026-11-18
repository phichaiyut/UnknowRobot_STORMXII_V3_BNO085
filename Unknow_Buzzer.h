/* =====================================================================
 *  Unknow_Buzzer.h — เสียงบี๊บ
 *
 *  ระบบเสียงยกมาจาก MyMini_16Pro_BNO086_TEST (MyMini_16Pro_BNO086_TEST_Buzzer.h)
 *  ความถี่ / จังหวะเหมือนต้นแบบ — ขายังเป็นของบอร์ด STORMXII (GP33)
 *
 *  ใช้ tone() / noTone() เท่านั้น (ห้ามกลับไปใช้ analogWrite กับ buzzer)
 *   - tone() ของ arduino-pico ใช้ PIO state machine ไม่ใช้ PWM slice
 *     → ไม่ชนกับ PWM มอเตอร์ EN1 (GP21, slice 2) / EN2 (GP24, slice 4)
 *   - analogWriteFreq() / analogWriteRange() เป็นค่ารวมทั้งชิป ถ้า buzzer เรียก
 *     จะไปเปลี่ยนความถี่ / ความละเอียด PWM มอเตอร์ (20 kHz / 12 บิต)
 *   - เซอร์โว (Servo.h) ก็ใช้ PIO: 6 ตัว + buzzer 1 = 7 จาก 12 SM ของ RP2350 → พอ
 *   - noTone() ปิด SM แล้วดึงขาลง LOW → เงียบสนิท
 *
 *  สารบัญ
 *     1) ขา Buzzer / ความถี่
 *     2) เสียงพื้นฐาน
 *     3) เสียงสำเร็จรูป (คาลิเบรต / ไจโร)
 * ===================================================================== */

#ifndef UNKNOW_BUZZER_H
#define UNKNOW_BUZZER_H

#include <Arduino.h>


// =====================================================================
//  1) ขา Buzzer / ความถี่
// =====================================================================

#define BUZZER_PIN 33        // STORMXII (ต้นแบบ MyMINI ใช้ GP9)
#define ToNe 3500            // เสียงหลักของ Beep() (ต้นแบบ)
#define TONE_CAL_TICK 3000   // เสียงติ๊ดระหว่างกวาดค่าแสง (CALIBRATION_TONE_HZ)
#define TONE_COMPLETE 3500   // เสียงสำเร็จ (COMPLETE_TONE_HZ)
#define CAL_TONE_MS 50           // ติ๊ดยาว 50 ms
#define CAL_TONE_INTERVAL_MS 300 // ติ๊ดทุก 300 ms


// =====================================================================
//  2) เสียงพื้นฐาน
// =====================================================================

void BuzzerBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void Beep(int delayb) {
  tone(BUZZER_PIN, ToNe);
  delay(delayb);
  noTone(BUZZER_PIN);
}

// freq = ความถี่ Hz (แบบต้นแบบ) — เดิมเป็นค่า duty ของ analogWrite
void Beep2(int freq, int delayb) {
  tone(BUZZER_PIN, freq);
  delay(delayb);
  noTone(BUZZER_PIN);
}

void BZon() {
  tone(BUZZER_PIN, ToNe);
}

void BZon(int freq) {
  tone(BUZZER_PIN, freq);
}

void BZoff() {
  noTone(BUZZER_PIN);
}

// ติ๊ดคาลิเบรตแบบไม่บล็อก: เรียกทุกรอบของลูปกวาดค่า
// ถึงเวลา → tone 3000 Hz ยาว 50 ms (ตัวจับเวลาฮาร์ดแวร์ปิดเอง ไม่ต้องรอ) ทุก 300 ms
inline void CalTick(unsigned long &nextTone) {
  unsigned long now = millis();
  if ((long)(now - nextTone) >= 0) {
    tone(BUZZER_PIN, TONE_CAL_TICK, CAL_TONE_MS);
    nextTone = now + CAL_TONE_INTERVAL_MS;
  }
}


// =====================================================================
//  3) เสียงสำเร็จรูป (คาลิเบรต / ไจโร)
// =====================================================================

// คาลิเบรตสำเร็จ + บันทึกแล้ว: 3500 Hz 250 ms × 2 เว้น 150 ms
void SoundSuccess() {
  Beep2(TONE_COMPLETE, 250);
  delay(150);
  Beep2(TONE_COMPLETE, 250);
}

// ไจโรพร้อม (ตั้งศูนย์สำเร็จ): 2093 / 2637 / 3136 Hz ยาว 65 ms ทุก 80 ms
void SoundGyroReady() {
  const int notes[] = { 2093, 2637, 3136 };
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, notes[i], 65);
    delay(80);
  }
  noTone(BUZZER_PIN);
}

#endif
