/* =====================================================================
 *  Unknow_IMU.h — Gyro (BNO085): อ่านมุม, หมุน/เลี้ยวตามองศา, วิ่งตรงด้วย Gyro
 *
 *  สารบัญ
 *     1) ตัวแปร (Global Variables)
 *     2) อ่านมุมจาก BNO085
 *     3) ตั้ง / รีเซ็ตมุมอ้างอิง
 *     4) ค้างมุมอยู่กับที่ (Hold Angle)
 *     5) ตั้งค่า Gyro (เรียกใน Setting.ino)
 *     6) หมุน / เลี้ยว ตามองศา
 *     7) คำสั่งย่อ หมุน / เลี้ยว
 *     8) หันไปทิศสัมบูรณ์ (0, 90, 180, 270 นับจากตอน reset มุม)
 *     9) วิ่งตรงด้วย Gyro
 *    10) คำสั่งเมื่อเจอแยก (Gyro)
 *    11) วิ่งตรงด้วย Gyro + คำสั่งเจอแยก
 *    12) วิ่งตามระยะทางด้วย Gyro (cm)
 *    13) วิ่งตรงตามทิศสัมบูรณ์
 *    14) วิ่งตามเส้นจนหันถึงทิศ (FFdir / BBdir)
 *    15) วิ่งตามเส้นจนหันถึงทิศ / องศา (ffdeg / ffdirg / ffdircl ...)
 * ===================================================================== */

#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "Unknow_Buzzer.h"
#include "Unknow_Motor.h"
#include "Unknow_Sensor.h"
#include "Unknow_PID.h"


// =====================================================================
//  1) ตัวแปร (Global Variables)
// =====================================================================

// ---------- BNO085 / มุมปัจจุบัน ----------
//  1) ตัวแปร (Global Variables)

//  1) ตัวแปร (Global Variables)
BNO08x myIMU;
float yawOffset = 0;
float lastYaw = 0;
constexpr int8_t YAW_SIGN = -1;  // ค่าคงที่ตอนคอมไพล์ ไม่กิน RAM
float current_degree = 0;
int16_t previous_errorG;  // error มุม ±180
int16_t previous_errorGB;
bool useDirectionG = false;  // true = คำสั่ง cm ใช้ทิศที่ตั้งจาก SetDirectionG แทนการจำมุมปัจจุบัน

// ---------- ค้างมุม (Hold) ----------
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
float prevErrHoldBF = 0;

// ---------- หมุน / เลี้ยว — ปรับได้ด้วย SetGyroTurn / SetGyroTurnB / SetGyroSpin ใน Setting.ino ----------
float gyro_Kp_Spin = 0.75f;
float gyro_Kd_Spin = 0.9f;
int8_t gyro_MaxSpd_Spin = 40;  // ความเร็ว 0..100
int8_t gyro_MinSpd_Spin = 8;
float gyro_SmallAngle_Spin = 8.0f;
float gyro_StopThr_Spin = 1.0f;
float gyro_Kp_Turn = 0.75f;
float gyro_Kd_Turn = 0.9f;
int8_t gyro_MaxSpd_Turn = 50;
int8_t gyro_MinSpd_Turn = 10;
float gyro_SmallAngle_Turn = 10.0f;
float gyro_StopThr_Turn = 1.0f;
float gyro_Kp_TurnB = 0.75f;  // เลี้ยวถอยหลัง (turnDegreeB) — ตั้งด้วย SetGyroTurnB
float gyro_Kd_TurnB = 0.9f;
int8_t gyro_MaxSpd_TurnB = 50;
int8_t gyro_MinSpd_TurnB = 10;
float gyro_SmallAngle_TurnB = 10.0f;
float gyro_StopThr_TurnB = 1.0f;

// ---------- วิ่งโค้งกำหนดล้อเอง (rotateDegree / rotateDir) — ปรับได้ด้วย SetGyroRotate ----------
// ช่วงชะลอ: scale = kp × (มุมที่เหลือ + kd × อัตราเข้าหาเป้า[องศา/10ms]) / smallAngle
//   kp > 1 = ชะลอช้าลง (เข้าเร็ว) | kd = มองล่วงหน้ากี่ × 10ms (หมุนเลยมุม → เพิ่ม kd)
float gyro_Kp_Rotate = 1.0f;
float gyro_Kd_Rotate = 2.0f;
int8_t gyro_MinSpd_Rotate = 15;        // ล้อที่เร็วกว่าไม่ต่ำกว่านี้ตอนชะลอ
float gyro_SmallAngle_Rotate = 25.0f;  // เหลือน้อยกว่านี้ (องศา) เริ่มชะลอ
float gyro_StopThr_Rotate = 1.0f;

// ---------- วิ่งตรง — ปรับได้ด้วย SetGyroRun / SetGyroRunB / ModeSpdGyro ใน Setting.ino ----------
float kpG = 1.2;
float kdG = 1.5;

// float kdG = 10;
float kpGB = 1.2;
float kdGB = 1.5;

// float kdGB = 10;

// โหมดจำกัดกำลังมอเตอร์ของ RunG (เดินหน้า) / RunGB (ถอยหลัง)
// ค่าเริ่มต้น = โหมด 4, min -5 (เหมือนพฤติกรรมเดิม) ปรับได้ด้วย ModeSpdGyro() ใน Setting.ino
int8_t MaxSpeedG = 100;
int8_t MinSpeedG = -5;
uint8_t ModeGyroStatus = 4;
uint8_t ModeGyroBStatus = 4;
int8_t gyro_limit_up = 20, gyro_limit_down = 20;  // โหมด 2: -(Speed+down)..(Speed+up)
int8_t gyro_limit4_up = 20;                       // โหมด 4: ..(Speed+up)

// ---------- ตัวคูณแปลงความเร็ว → ระยะทาง ของ FFcmG / BBcmG / FFcmGS / BBcmGS ----------
// ปรับได้ด้วย set_speed_scale_ff_g / set_speed_scale_bb_g / set_speed_scale_ff_bb_gs ใน Setting.ino
float speed_scale_ff_g = 1.5;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bb_g = 1.5;
float speed_scale_ff_g_normal = 0.99;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bb_g_normal = 0.99;
float speed_scale_ff_gs = 1.65;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bb_gs = 1.65;

// ---------- เบรกหลังหมุน / เลี้ยวด้วย Gyro ----------
// _g = เวลาล็อกมุม (SetG) หลังถึงมุม | BounceSpd / BounceMs = เบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม
// ความเร็ว = int8_t | เวลา ms = uint16_t
uint16_t brake_spin_g = 5;
uint16_t brake_turn_g = 5;
uint16_t brake_turn_b_g = 5;
int8_t gyro_BounceSpd_spin = 12;
uint16_t gyro_BounceMs_spin = 5;
int8_t gyro_BounceSpd_turn = 12;
uint16_t gyro_BounceMs_turn = 5;
int8_t gyro_BounceSpd_turn_b = 12;
uint16_t gyro_BounceMs_turn_b = 5;
uint16_t brake_rotate_g = 20;     // rotateDegree / rotateDir: เวลาล็อกมุม (SetG) หลังเบรก
int8_t gyro_BounceSpd_rotate = 15;  // เบรกสวนทิศล้อ (ล้อที่เร็วกว่า = ค่านี้ ล้อช้ากว่าลดตามสัดส่วน)
uint16_t gyro_BounceMs_rotate = 10;

// ---------- วิ่งตามเส้นจนถึงทิศ (FFdir / BBdir) ----------
float line_dir_threshold = 5.0f;  // ถือว่าถึงทิศเมื่อ |error| <= ค่านี้ (องศา)

// ---------- ค่าคงที่ทิศ (อ้างอิงตอน resetAngles() / resetAnglesDir()) ----------
constexpr int DIR_N = 0;
constexpr int DIR_NE = 45;
constexpr int DIR_E = 90;
constexpr int DIR_SE = 135;
constexpr int DIR_S = 180;
constexpr int DIR_SW = -135;
constexpr int DIR_W = -90;
constexpr int DIR_NW = -45;

// ---------- ตั้งลำอัตโนมัติก่อนวิ่งตามทิศ (set_auto_align) ----------
// คำสั่งวิ่งที่ใส่ทิศ: หัวหุ่นคลาดจากทิศเกินค่านี้ → หมุนอยู่กับที่ตั้งลำก่อนวิ่ง | 0 = ปิด
float gyro_AutoAlign_deg = 20.0f;

// ---------- ปัดมุมเข้าแกน 90° ก่อนเลี้ยวที่ทางแยก (TrackSelectG / GB) ----------
bool track_snap90 = true;

// ---------- เบรกไจโรของ select 'f' (ffdir / ffdeg / ffdircl ...) — 0 = คิดจากความเร็ว ----------
int line_dir_brake_spd = 0;
int line_dir_brake_ms = 0;
int line_dir_hold_ms = 60;


// =====================================================================
//  2) อ่านมุมจาก BNO085
// =====================================================================

//  2) อ่านมุมจาก BNO085

//  2) อ่านมุมจาก BNO085
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool waitForRotVec(uint32_t timeout_ms = 200) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      return true;
    }
    delay(1);
  }
  return false;
}

float quatToYawDeg() {
  float q0 = myIMU.getQuatReal();
  float q1 = myIMU.getQuatI();
  float q2 = myIMU.getQuatJ();
  float q3 = myIMU.getQuatK();

  float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3));
  yaw = yaw * 180.0f / PI;
  yaw = wrap180(yaw);

  yaw *= (float)YAW_SIGN;
  yaw = wrap180(yaw);

  return yaw;
}

int updateYawFromEvent() {
  float yaw = quatToYawDeg();
  float yawZeroed = wrap180(yaw - yawOffset);
  return lastYaw = yawZeroed;
}

int angleRead() {
  if (!myIMU.getSensorEvent()) return lastYaw;
  if (myIMU.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) return lastYaw;
  return updateYawFromEvent();
}

// รอค่ามุมใหม่จาก IMU ก่อนคืนค่า (ใช้ตอนตั้งมุมเป้าหมาย)
// angleRead() คืน lastYaw เก่าถ้ายังไม่มี packet ใหม่ ถ้าคำสั่งก่อนหน้าไม่ได้อ่านไจโร (เช่น วิ่งตามเส้น)
// lastYaw จะค้างเป็นมุมเดิม ทำให้เป้าหมายผิด หุ่นหมุนกลับไปมุมเก่าก่อนแล้วค่อยหมุนต่อ
int angleReadFresh() {
  if (!waitForRotVec(50)) return lastYaw;
  return updateYawFromEvent();
}

// คืน true ถ้า reset สำเร็จ, false = gyro ไม่ส่งข้อมูล (บี๊บเตือน และ offset เป็นค่าเดิม)
bool resetYaw() {
  if (!waitForRotVec(200)) {
    Serial.println("ResetYaw timeout: no rotation vector");
    Beep(500);
    return false;
  }
  float yawNow = quatToYawDeg();
  yawOffset = yawNow;
  lastYaw = 0.0f;
  return true;
}


// =====================================================================
//  3) ตั้ง / รีเซ็ตมุมอ้างอิง
// =====================================================================

//  3) ตั้ง / รีเซ็ตมุมอ้างอิง

//  3) ตั้ง / รีเซ็ตมุมอ้างอิง

// รีเซ็ตมุมปัจจุบันของหุ่นยนต์ให้เป็น 0 องศา
void setAngleOffset() {
  //  MotorStop();delay(20);
  resetYaw();
  current_degree = angleRead();
  previous_errorG = 0;
  previous_errorGB = 0;
}

// รีเซ็ต D-term ของ RunG / RunGB ให้เท่ากับ error ปัจจุบัน (ไม่ใช่ 0) กันกระชากตอนออกตัว
// เรียกจาก InitialSpeed() ด้วย | ใช้ lastYaw (ไม่อ่านเซนเซอร์ใหม่) จึงปลอดภัยแม้ยังไม่เริ่ม IMU
void resetGyroRunError() {
  float error = current_degree - (int)lastYaw;  // (int) ให้ตรงกับค่าที่ angleRead() คืน
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  previous_errorG = error;
  previous_errorGB = -error;  // RunGB ใช้ error กลับด้าน (angleRead() - current_degree)
}

void SetRobotAngle() {
  current_degree = angleRead();
  resetGyroRunError();  // คำสั่ง cm เรียกหลัง InitialSpeed() จึงต้องรีเซ็ต D-term ใหม่ตามมุมที่จำ
}

void resetAngle() {
  resetYaw();
  current_degree = angleRead();
  previous_errorG = 0;
  previous_errorGB = 0;
}

void resetAngles() {
  setAngleOffset();
  current_degree = angleRead();
  previous_errorG = 0;
  previous_errorGB = 0;
}

// วางหุ่นหันทิศ start_dir (ไม่ใช่ 0) แล้วรีเซ็ต เช่น resetAnglesDir(DIR_E);
// ควรเรียกตอนหุ่นหยุดนิ่ง | คืน false = gyro ไม่ส่งข้อมูล (บี๊บเตือน ค่าเดิมไม่เปลี่ยน)
bool resetAnglesDir(int start_dir) {
  if (!waitForRotVec(200)) {
    Serial.println("resetAnglesDir timeout: no rotation vector");
    Beep(500);
    return false;
  }
  float dir = wrap180(fmodf((float)start_dir, 360.0f));
  yawOffset = wrap180(quatToYawDeg() - dir);
  lastYaw = dir;
  current_degree = dir;
  previous_errorG = 0;
  previous_errorGB = 0;
  return true;
}


// =====================================================================
//  4) ค้างมุมอยู่กับที่ (Hold Angle)
// =====================================================================

// ---------- PD ค้างมุม (เรียกใน loop) ----------
//  4) ค้างมุมอยู่กับที่ (Hold Angle)

//  4) ค้างมุมอยู่กับที่ (Hold Angle)
void HoldAngle() {
  float error = current_degree - angleRead();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHold = error;
}

void HoldAngleB() {
  float error = angleRead() - current_degree;
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(-power, power);               // หมุนอยู่กับที่ (error กลับด้าน จึงสลับล้อ ให้หมุนแก้ทางเดียวกับ HoldAngle)
  prevErrHoldB = error;
}

void SetHoldAngle() {
  holdAngle = angleRead();  // มุมที่ต้องการให้หุ่น "จำ"
  MotorStop();
  prevErrHoldF = 0;
}

void HoldAngleF() {
  float error = holdAngle - angleRead();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldF;
  int power = (error * kpFHold) + (d * kdFHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHoldF = error;
}

void HoldAngleBF() {
  float error = angleRead() - holdAngle;
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldBF;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(-power, power);               // หมุนอยู่กับที่
  prevErrHoldBF = error;
}

// ---------- ค้างมุมที่ current_degree ----------
void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngle();
  }
  BZoff();
}

// ล็อกมุมแบบถอยหลัง ที่มุมอ้างอิง current_degree (เหมือน SetG)
void SetGB(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleB();
  }
  BZoff();
}

// ---------- ค้างมุม ณ ตอนเรียก ----------
void SetFG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleF();
  }
  BZoff();
}

// ล็อกมุมแบบถอยหลัง: จำมุม ณ ตอนเรียก (เหมือน SetFG)
void SetBG(int totalTime) {
  BZon();
  SetHoldAngle();
  prevErrHoldBF = 0;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleBF();
  }
  BZoff();
}

// ---------- ชื่อย่อ ----------
void setg(int time) {
  SetG(time);
}

void setgb(int time) {
  SetGB(time);
}

void setfg(int totalTime) {
  SetFG(totalTime);
}

void setbg(int totalTime) {
  SetBG(totalTime);
}

// ---------- เบรกไจโร: เบรกสวนทิศวิ่งพร้อมคุมหัวหุ่น แล้วล็อกมุมนิ่ง ----------
// ใช้กับ select 'f' ของ rotateDir / ffdir / ffdeg / ffdircl ...
// backward = true: กำลังถอยหลัง (เบรกโดยเดินหน้า + ล็อกแบบ HoldAngleBF)
static void gyroBrakeStop(int brakeSpd, int brakeMs, int holdMs, bool backward) {
  BZon();
  int base = backward ? abs(brakeSpd) : -abs(brakeSpd);
  Motor(base, base);           // เริ่มเบรกทันที
  holdAngle = angleReadFresh();  // มุมที่หัวหุ่นหันอยู่จริงตอนสั่ง
  float kp = backward ? kpBHold : kpFHold;
  float kd = backward ? kdBHold : kdFHold;
  float prevErr = 0;
  unsigned long endTime = millis() + brakeMs;
  while (millis() <= endTime) {
    float error = wrap180(holdAngle - angleRead());
    int pd = (error * kp) + ((error - prevErr) * kd);
    prevErr = error;
    pd = constrain(pd, -50, 50);
    Motor(constrain(base + pd, -100, 100), constrain(base - pd, -100, 100));
  }
  MotorStop();
  prevErrHoldF = 0;
  prevErrHoldBF = 0;
  endTime = millis() + holdMs;
  while (millis() <= endTime) {
    if (backward) HoldAngleBF();
    else HoldAngleF();
  }
  MotorStop();
  BZoff();
}


// =====================================================================
//  5) ตั้งค่า Gyro (เรียกใน Setting.ino)
// =====================================================================

// ---------- PID หมุน / เลี้ยว / วิ่งตรง ----------
//  5) ตั้งค่า Gyro (เรียกใน Setting.ino)

//  5) ตั้งค่า Gyro (เรียกใน Setting.ino)
void SetGyroTurn(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Turn = kp;
  gyro_Kd_Turn = kd;
  gyro_MaxSpd_Turn = maxSpd;
  gyro_MinSpd_Turn = minSpd;
  gyro_SmallAngle_Turn = smallAngle;
  gyro_StopThr_Turn = stopThr;
}

// เลี้ยวถอยหลังล้อเดียว (turnDegreeB / turnDirectionB) แยกจาก SetGyroTurn
void SetGyroTurnB(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_TurnB = kp;
  gyro_Kd_TurnB = kd;
  gyro_MaxSpd_TurnB = maxSpd;
  gyro_MinSpd_TurnB = minSpd;
  gyro_SmallAngle_TurnB = smallAngle;
  gyro_StopThr_TurnB = stopThr;
}

void SetGyroSpin(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Spin = kp;
  gyro_Kd_Spin = kd;
  gyro_MaxSpd_Spin = maxSpd;
  gyro_MinSpd_Spin = minSpd;
  gyro_SmallAngle_Spin = smallAngle;
  gyro_StopThr_Spin = stopThr;
}

// rotateDegree / rotateDir (วิ่งโค้งกำหนดล้อเอง) — ไม่มี maxSpd เพราะใช้ SpeedL / SpeedR ที่สั่ง
void SetGyroRotate(float kp, float kd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Rotate = kp;
  gyro_Kd_Rotate = kd;
  gyro_MinSpd_Rotate = minSpd;
  gyro_SmallAngle_Rotate = smallAngle;
  gyro_StopThr_Rotate = stopThr;
}

void SetGyroRun(float kp, float kd) {
  kpG = kp;
  kdG = kd;
}

void SetGyroRunB(float kp, float kd) {
  kpGB = kp;
  kdGB = kd;
}

// ตั้งโหมดจำกัดกำลังของ gyro (ใช้ทั้ง RunG และ RunGB)
void ModeSpdGyro(int moD, int maX, int miN) {
  ModeGyroStatus = moD;
  ModeGyroBStatus = moD;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

// ตั้งโหมดแยกเดินหน้า (RunG) / ถอยหลัง (RunGB)
void ModeSpdGyro(int moDF, int moDB, int maX, int miN) {
  ModeGyroStatus = moDF;
  ModeGyroBStatus = moDB;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

// ช่วงกำลังล้อของโหมด 2 (ใช้ทั้ง RunG และ RunGB)
void set_gyro_mode2_limit(int up, int down) {
  gyro_limit_up = up;
  gyro_limit_down = down;
}

// ช่วงกำลังล้อของโหมด 4: ..(Speed+up) (ใช้ทั้ง RunG และ RunGB)
void set_gyro_mode4_limit(int up) {
  gyro_limit4_up = up;
}

// ---------- ตัวคูณระยะทาง (หุ่นเดินเกินระยะ ให้เพิ่มค่า | เดินไม่ถึงระยะ ให้ลดค่า) ----------
// FFcmG: fw_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), fw = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_ff_g(float fw_normal, float fw) {
  speed_scale_ff_g_normal = fw_normal;
  speed_scale_ff_g = fw;
}

// BBcmG: bb_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), bb = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_bb_g(float bb_normal, float bb) {
  speed_scale_bb_g_normal = bb_normal;
  speed_scale_bb_g = bb;
}

// FFcmGS / BBcmGS
void set_speed_scale_ff_bb_gs(float ff, float bb) {
  speed_scale_ff_gs = ff;
  speed_scale_bb_gs = bb;
}

// ---------- เบรกหลังหมุน / เลี้ยว ----------
// _setg(ms)  = เวลาล็อกมุม (SetG) หลังหมุน/เลี้ยวเสร็จ
// (spd, ms)  = เบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม | หมุนเกินมุม ให้เพิ่ม | เด้งกลับ ให้ลด (ใส่ 0 = ไม่เบรก)
void set_brake_spin_gyro_setg(int brake) {
  brake_spin_g = brake;
}

void set_brake_spin_gyro(int spd, int t) {
  gyro_BounceSpd_spin = spd;
  gyro_BounceMs_spin = t;
}

void set_brake_turn_gyro_setg(int brake) {
  brake_turn_g = brake;
}

void set_brake_turn_gyro(int spd, int t) {
  gyro_BounceSpd_turn = spd;
  gyro_BounceMs_turn = t;
}

void set_brake_turn_b_gyro_setg(int brake) {
  brake_turn_b_g = brake;
}

void set_brake_turn_b_gyro(int spd, int t) {
  gyro_BounceSpd_turn_b = spd;
  gyro_BounceMs_turn_b = t;
}

void set_brake_rotate_gyro_setg(int brake) {
  brake_rotate_g = brake;
}

void set_brake_rotate_gyro(int spd, int t) {
  gyro_BounceSpd_rotate = spd;
  gyro_BounceMs_rotate = t;
}

// ---------- วิ่งตามเส้นจนถึงทิศ (FFdir / BBdir) ----------
void set_line_dir_threshold(float thr) {
  line_dir_threshold = thr;
}

// select 'f': แรง / เวลาเบรก (0, 0 = คิดจากความเร็ว: แรง = speed/2 (15..60), เวลา = 10 + speed/3 (20..50))
void set_line_dir_brake(int brake_spd, int brake_ms, int hold_ms = 60) {
  line_dir_brake_spd = brake_spd;
  line_dir_brake_ms = brake_ms;
  line_dir_hold_ms = hold_ms;
}

// ---------- ทิศ ----------
// คำสั่งวิ่งที่ใส่ทิศ: หัวหุ่นคลาดเกิน deg องศา → หมุนตั้งลำก่อนวิ่ง (0 = ปิด) ค่าเริ่มต้น 20
void set_auto_align(float deg) {
  gyro_AutoAlign_deg = deg;
}

// true (ค่าเริ่มต้น) = เลี้ยวที่ทางแยกด้วยไจโร ปัดมุมเข้าแกน 90° ก่อน | false = เลี้ยวสัมพัทธ์แบบเดิม
void set_track_snap90(bool enable) {
  track_snap90 = enable;
}


// =====================================================================
//  6) หมุน / เลี้ยว ตามองศา
// =====================================================================

// ---------- หมุนอยู่กับที่ (spin) ----------
//  6) หมุน / เลี้ยว ตามองศา

//  6) หมุน / เลี้ยว ตามองศา
void spinDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Spin;
  int max_speed = Speed;
  float kp = gyro_Kp_Spin;
  float kd = gyro_Kd_Spin;
  float small_angle_threshold = gyro_SmallAngle_Spin;
  float stop_threshold = gyro_StopThr_Spin;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  int spin_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = หมุนขวา, -1 = หมุนซ้าย, 0 = ไม่ได้หมุน
  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    int pd_value = (kp * error) + (kd * (error - previous_error));
    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -min_speed);
      spin_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
      spin_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      break;
    } else {
      Motor(pd_value, -pd_value);
      if (pd_value != 0) spin_dir = (pd_value > 0) ? 1 : -1;
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
    previous_error = error;
  }
  // เบรกสวนทิศที่หมุนจริง (ไม่ใช้ relative_degree เพราะหลัง wrap ±180 ทิศอาจกลับด้าน)
  if (spin_dir != 0) {
    Motor(-spin_dir * gyro_BounceSpd_spin, spin_dir * gyro_BounceSpd_spin);
    delay(gyro_BounceMs_spin);
  }
  MotorStop();
  SetG(brake_spin_g);
}

void spinDegree(int relative_degree) {
  spinDegree(gyro_MaxSpd_Spin, relative_degree);
}

// ---------- เลี้ยวล้อเดียว (turn) ----------
void turnDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  int turn_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = เลี้ยวขวา (ล้อซ้ายเดิน), -1 = เลี้ยวซ้าย (ล้อขวาเดิน), 0 = ไม่ได้เลี้ยว
  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    int pd_value = (kp * error) + (kd * (error - previous_error));
    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, 0);
      turn_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(0, min_speed);
      turn_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) {
        Motor(-1, -pd_value);
        turn_dir = -1;
      } else {
        Motor(pd_value, 1);
        turn_dir = 1;
      }
    }
    previous_error = error;
  }
  // เบรกล้อที่เดินอยู่ สวนทิศที่เลี้ยวจริง
  if (turn_dir > 0) Motor(-gyro_BounceSpd_turn, 0);
  else if (turn_dir < 0) Motor(0, -gyro_BounceSpd_turn);
  if (turn_dir != 0) delay(gyro_BounceMs_turn);
  MotorStop();
  SetG(brake_turn_g);
}

void turnDegree(int relative_degree) {
  turnDegree(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeB(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_TurnB;  // ค่าชุด TurnB แยกจาก turnDegree (ตั้งด้วย SetGyroTurnB)
  int max_speed = Speed;
  float kp = gyro_Kp_TurnB;
  float kd = gyro_Kd_TurnB;
  float small_angle_threshold = gyro_SmallAngle_TurnB;
  float stop_threshold = gyro_StopThr_TurnB;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  int turn_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = ล้อขวาถอย, -1 = ล้อซ้ายถอย, 0 = ไม่ได้เลี้ยว
  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    int pd_value = (kp * error) + (kd * (error - previous_error));
    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(0, -min_speed);
      turn_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 0);
      turn_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) {
        Motor(pd_value, 1);
        turn_dir = -1;
      } else {
        Motor(-1, -pd_value);
        turn_dir = 1;
      }
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
    previous_error = error;
  }
  // เบรกล้อที่ถอยอยู่ ด้วยการเดินหน้าสวนสั้น ๆ
  if (turn_dir > 0) Motor(0, gyro_BounceSpd_turn_b);
  else if (turn_dir < 0) Motor(gyro_BounceSpd_turn_b, 0);
  if (turn_dir != 0) delay(gyro_BounceMs_turn_b);
  MotorStop();
  SetG(brake_turn_b_g);
}

void turnDegreeB(int relative_degree) {
  turnDegreeB(gyro_MaxSpd_TurnB, relative_degree);
}

// ---------- เลี้ยวล้อเดียวแบบไม่เบรก (_none) ----------
void turnDegree_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // ถ้าหมุนเร็ว มุมจะกระโดดข้ามช่วง ±stop_threshold ได้
    // จึงถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมาย (หมุนเลยเป้าแล้ว) ด้วย ไม่ให้หมุนกลับไปมา
    // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
    bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      //MotorStop();
      break;
    } else if (error > 0) {
      Motor(Speed, -2);
    } else {
      Motor(-2, Speed);
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
  }
}

void turnDegree_none(int relative_degree) {
  turnDegree_none(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeb_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_TurnB;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // ถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมายด้วย (ดู turnDegree_none)
    bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      //MotorStop();
      break;
    } else if (error > 0) {
      Motor(1, -Speed);
    } else {
      Motor(-Speed, 1);
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
  }
}

void turnDegreeb_none(int relative_degree) {
  turnDegreeb_none(gyro_MaxSpd_TurnB, relative_degree);
}

// ---------- โค้งด้วยความเร็วล้อซ้าย / ขวา (rotateDegree) — Ratio-Preserving Arc ----------
// ทิศหมุนกำหนดจากความเร็วล้อเสมอ (ใช้ได้ทั้งโค้งเดินหน้า / ถอยหลัง / หมุนเยื้องศูนย์):
//   SpeedL > SpeedR → มุมเพิ่ม (ขวา) | SpeedL < SpeedR → มุมลด (ซ้าย) | เท่ากัน = ไม่เลี้ยว → ข้าม
// ช่วงไกล: สั่ง (SpeedL, SpeedR) ตรงตามผู้ใช้ 100% → รัศมีโค้งตรง
// ช่วงชะลอ (มุมที่เหลือ < gyro_SmallAngle_Rotate): สั่ง (SpeedL × scale, SpeedR × scale) สัดส่วนล้อเท่าเดิม
//   scale = kp × (มุมที่เหลือ + kd × อัตราเข้าหาเป้า[องศา/10ms]) / gyro_SmallAngle_Rotate
//   ล้อที่เร็วกว่าไม่ต่ำกว่า gyro_MinSpd_Rotate | คำนวณเฉพาะตอนมีมุมใหม่จาก IMU
// หยุดเมื่อ |error| <= gyro_StopThr_Rotate หรือข้ามเป้า (error เปลี่ยนเครื่องหมายตอน |error| < 90)
// brake = true: เบรกสวนทิศล้อตามสัดส่วน → MotorStop() → ล็อกมุม SetG(brake_rotate_g)
// brake = false: ไม่ชะลอ ไม่เบรก มอเตอร์ยังวิ่ง (SpeedL, SpeedR) ต่อ → ต่อคำสั่งถัดไปลื่น
// ต้องเรียก angleReadFresh() มาก่อน (lastYaw สด)
// คืน false = ไม่ได้วิ่ง (ล้อเท่ากัน) หรือ IMU ไม่ส่งมุมเกิน 500ms (หยุดมอเตอร์แล้ว)
static bool rotateArc(int SpeedL, int SpeedR, float target_degree, bool brake, float kp, float kd) {
  SpeedL = constrain(SpeedL, -100, 100);
  SpeedR = constrain(SpeedR, -100, 100);
  if (SpeedL == SpeedR) {
    Serial.println("rotate: SpeedL == SpeedR -> no turning, skipped");
    Beep(100);
    return false;
  }
  const int rot_sign = (SpeedL > SpeedR) ? 1 : -1;  // ทิศที่ล้อพาหุ่นหมุนจริง
  const int maxAbs = max(abs(SpeedL), abs(SpeedR));
  const float minScale = (maxAbs > gyro_MinSpd_Rotate) ? (float)gyro_MinSpd_Rotate / (float)maxAbs : 1.0f;
  const float small_angle = (gyro_SmallAngle_Rotate > 1.0f) ? gyro_SmallAngle_Rotate : 1.0f;
  const float stop_threshold = gyro_StopThr_Rotate;

  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  float prev_error = wrap180(target_degree - lastYaw);  // เริ่มจาก error จริง (ไม่ใช่ 0) กัน crossed / D กระชาก
  bool first = true;
  uint32_t lastFreshUs = micros();
  int cmdL = SpeedL, cmdR = SpeedR;
  Motor(cmdL, cmdR);

  while (1) {
    // ทำงานเฉพาะตอนมีมุมใหม่จาก IMU (ระหว่างรอ คงคำสั่งมอเตอร์เดิม)
    if (!(myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)) {
      if (micros() - lastFreshUs > 500000UL) {  // IMU หลุด → หยุด กันหมุนไม่รู้จบ
        Serial.println("rotate: IMU timeout");
        MotorStop();
        return false;
      }
      continue;
    }
    updateYawFromEvent();
    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastFreshUs;
    lastFreshUs = nowUs;

    float error = wrap180(target_degree - lastYaw);
    // D = การเปลี่ยน error ต่อ 10ms (รอบแรก = 0)
    float d = (first || dtUs == 0) ? 0.0f : (error - prev_error) * 10000.0f / (float)dtUs;
    first = false;

    bool crossed = fabsf(error) < 90.0f && ((error > 0.0f && prev_error < 0.0f) || (error < 0.0f && prev_error > 0.0f));
    prev_error = error;
    if (crossed || fabsf(error) <= stop_threshold) break;

    // rem = มุมที่เหลือตามทิศที่ล้อพาหมุน (+ = เป้ายังอยู่ข้างหน้า, - = อีกฝั่ง ต้องหมุนอีกไกล)
    float rem = error * rot_sign;
    float scale = 1.0f;
    if (brake && rem > 0.0f && rem < small_angle) {
      float rem_pred = rem + kd * d * rot_sign;  // d × rot_sign < 0 = กำลังเข้าหาเป้า
      scale = constrain(kp * rem_pred / small_angle, minScale, 1.0f);
    }
    cmdL = (int)lroundf(SpeedL * scale);
    cmdR = (int)lroundf(SpeedR * scale);
    Motor(cmdL, cmdR);
  }

  if (!brake) {
    resetGyroRunError();  // RunG / FFcmG ถัดไปล็อกทิศ target ต่อโดย D ไม่กระชาก
    return true;
  }
  // เบรกสวนทิศที่ล้อแต่ละข้างกำลังหมุนจริง ตามสัดส่วนเดิม (ล้อเร็วกว่า = gyro_BounceSpd_rotate)
  int cmdMax = max(abs(cmdL), abs(cmdR));
  if (cmdMax > 0 && gyro_BounceSpd_rotate > 0 && gyro_BounceMs_rotate > 0) {
    int bL = -(int)lroundf((float)gyro_BounceSpd_rotate * cmdL / cmdMax);
    int bR = -(int)lroundf((float)gyro_BounceSpd_rotate * cmdR / cmdMax);
    Motor(bL, bR);
    delay(gyro_BounceMs_rotate);
  }
  MotorStop();
  SetG(brake_rotate_g);
  resetGyroRunError();
  return true;
}

// เครื่องหมายมุมตั้งอัตโนมัติจากล้อ: SpeedL > SpeedR → +|deg|, SpeedL < SpeedR → -|deg|
// จำกัดไม่เกิน 360 - 3 × stopThr (≈ 357) ไม่งั้นเป้าห่างจากมุมเริ่มไม่ถึง stopThr → หยุดทันทีไม่ได้หมุน
static float rotateRelTarget(int SpeedL, int SpeedR, int relative_degree) {
  float maxDeg = 360.0f - 3.0f * gyro_StopThr_Rotate;
  float deg = (float)abs(relative_degree);
  if (deg > maxDeg) deg = maxDeg;
  if (SpeedL < SpeedR) deg = -deg;
  angleReadFresh();
  return wrap180(lastYaw + deg);
}

// kp / kd = ค่าชะลอเฉพาะครั้งนี้ (ความหมายดู gyro_Kp_Rotate / gyro_Kd_Rotate)
void rotateDegree(int SpeedL, int SpeedR, int relative_degree, float kp, float kd) {
  if (SpeedL == SpeedR) {
    rotateArc(SpeedL, SpeedR, 0, true, kp, kd);  // แจ้งเตือนแล้วออก
    return;
  }
  rotateArc(SpeedL, SpeedR, rotateRelTarget(SpeedL, SpeedR, relative_degree), true, kp, kd);
}

void rotateDegree(int SpeedL, int SpeedR, int relative_degree) {
  rotateDegree(SpeedL, SpeedR, relative_degree, gyro_Kp_Rotate, gyro_Kd_Rotate);
}

// ไม่ชะลอ ไม่เบรก: ถึงมุมแล้วมอเตอร์ยังวิ่ง (SpeedL, SpeedR) ต่อ → ต่อโค้งตัว S / วิ่งตรงต่อลื่น
void rotateDegree_none(int SpeedL, int SpeedR, int relative_degree) {
  if (SpeedL == SpeedR) {
    rotateArc(SpeedL, SpeedR, 0, false, gyro_Kp_Rotate, gyro_Kd_Rotate);
    return;
  }
  rotateArc(SpeedL, SpeedR, rotateRelTarget(SpeedL, SpeedR, relative_degree), false, gyro_Kp_Rotate, gyro_Kd_Rotate);
}


// =====================================================================
//  7) คำสั่งย่อ หมุน / เลี้ยว
// =====================================================================

// ---------- ทางเดียว ----------
//  7) คำสั่งย่อ หมุน / เลี้ยว

//  7) คำสั่งย่อ หมุน / เลี้ยว
void SpinLG(int Angle) {
  spinDegree(-abs(Angle));
}

void SpinLG(int spd, int Angle) {
  spinDegree(spd, -abs(Angle));
}

void SpinRG(int Angle) {
  spinDegree(abs(Angle));
}

void SpinRG(int spd, int Angle) {
  spinDegree(spd, abs(Angle));
}

void TurnLG(int Angle) {
  turnDegree(-abs(Angle));
}

void TurnLG(int spd, int Angle) {
  turnDegree(spd, -abs(Angle));
}

void TurnRG(int Angle) {
  turnDegree(abs(Angle));
}

void TurnRG(int spd, int Angle) {
  turnDegree(spd, abs(Angle));
}

void TurnLBG(int Angle) {
  turnDegreeB(abs(Angle));
}

void TurnLBG(int spd, int Angle) {
  turnDegreeB(spd, abs(Angle));
}

void TurnRBG(int Angle) {
  turnDegreeB(-abs(Angle));
}

void TurnRBG(int spd, int Angle) {
  turnDegreeB(spd, -abs(Angle));
}

// ---------- เลี้ยวไป-กลับ (tlrg / trlg) ----------
void tlrg(int Angle) {
  turnDegree_none(-abs(Angle));
  turnDegree(abs(Angle));
}

void trlg(int Angle) {
  turnDegree_none(abs(Angle));
  turnDegree(-abs(Angle));
}

void tlrg(int spd, int Angle) {
  turnDegree_none(spd, -abs(Angle));
  turnDegree(spd, abs(Angle));
}

void trlg(int spd, int Angle) {
  turnDegree_none(spd, abs(Angle));
  turnDegree(spd, -abs(Angle));
}

void tlrg(int spd, int Angle, int Angle2) {
  turnDegree_none(spd, -abs(Angle));
  turnDegree(spd, abs(Angle2)); /*SetG(spd);*/
}

void trlg(int spd, int Angle, int Angle2) {
  turnDegree_none(spd, abs(Angle));
  turnDegree(spd, -abs(Angle2)); /*SetG(spd);*/
}

void tlrbg(int Angle) {
  turnDegreeb_none(abs(Angle));
  turnDegreeB(-abs(Angle)); /*SetGB(50);*/
}

void trlbg(int Angle) {
  turnDegreeb_none(-abs(Angle));
  turnDegreeB(abs(Angle)); /*SetGB(50);*/
}

void tlrbg(int spd, int Angle) {
  turnDegreeb_none(spd, abs(Angle));
  turnDegreeB(spd, -abs(Angle)); /*SetGB(spd);*/
}

void trlbg(int spd, int Angle) {
  turnDegreeb_none(spd, -abs(Angle));
  turnDegreeB(spd, abs(Angle)); /*SetGB(spd);*/
}

void tlrbg(int spd, int Angle, int Angle2) {
  turnDegreeb_none(spd, abs(Angle));
  turnDegreeB(spd, -abs(Angle2)); /*SetG(spd);*/
}

void trlbg(int spd, int Angle, int Angle2) {
  turnDegreeb_none(spd, -abs(Angle));
  turnDegreeB(spd, abs(Angle2)); /*SetG(spd);*/
}

// ---------- ชื่อเต็มของ tlrg / trlg ----------
void TurnLRG(int Angle) {
  tlrg(Angle);
}

void TurnRLG(int Angle) {
  trlg(Angle);
}

void TurnLRG(int spd, int Angle) {
  tlrg(spd, Angle);
}

void TurnRLG(int spd, int Angle) {
  trlg(spd, Angle);
}

void TurnLRG(int spd, int Angle, int Angle2) {
  tlrg(spd, Angle, Angle2);
}

void TurnRLG(int spd, int Angle, int Angle2) {
  trlg(spd, Angle, Angle2);
}

void TurnLRBG(int Angle) {
  tlrbg(Angle);
}

void TurnRLBG(int Angle) {
  trlbg(Angle);
}

void TurnLRBG(int spd, int Angle) {
  tlrbg(spd, Angle);
}

void TurnRLBG(int spd, int Angle) {
  trlbg(spd, Angle);
}

void TurnLRBG(int spd, int Angle, int Angle2) {
  tlrbg(spd, Angle, Angle2);
}

void TurnRLBG(int spd, int Angle, int Angle2) {
  trlbg(spd, Angle, Angle2);
}


// =====================================================================
//  8) หันไปทิศสัมบูรณ์ (0, 90, 180, 270 นับจากตอน reset มุม)
// =====================================================================

//  8) หันไปทิศสัมบูรณ์ (0, 90, 180, 270 นับจากตอน reset มุม)

//  8) หันไปทิศสัมบูรณ์ (0, 90, 180, 270 นับจากตอน reset มุม)

// คำนวณมุมที่ต้องหมุนจากมุมปัจจุบันไปยังทิศทางสัมบูรณ์ (ทางที่สั้นที่สุด)
int relativeToDirection(int direction) {
  float relative = fmod((float)direction, 360.0f) - angleReadFresh();
  while (relative > 180.0f) relative -= 360.0f;
  while (relative < -180.0f) relative += 360.0f;
  return (int)roundf(relative);
}

void spinDirection(int Speed, int direction) {
  spinDegree(Speed, relativeToDirection(direction));
}

void spinDirection(int direction) {
  spinDegree(relativeToDirection(direction));
}

void turnDirection(int Speed, int direction) {
  turnDegree(Speed, relativeToDirection(direction));
}

void turnDirection(int direction) {
  turnDegree(relativeToDirection(direction));
}

void turnDirectionB(int Speed, int direction) {
  turnDegreeB(Speed, relativeToDirection(direction));
}

void turnDirectionB(int direction) {
  turnDirectionB(gyro_MaxSpd_TurnB, direction);
}

void turnDirection_none(int Speed, int direction) {
  int rel = relativeToDirection(direction);
  // turnDegree_none เล็งจาก current_degree + rel → ซิงก์กับมุมจริงก่อน (rel คิดจาก (int)มุมสด) ให้ถึงทิศ dir พอดี
  current_degree = (int)lastYaw;
  turnDegree_none(Speed, rel);
}

void turnDirectionb_none(int Speed, int direction) {
  int rel = relativeToDirection(direction);
  // turnDegreeb_none เล็งจาก current_degree + rel → ซิงก์กับมุมจริงก่อน (rel คิดจาก (int)มุมสด) ให้ถึงทิศ dir พอดี
  current_degree = (int)lastYaw;
  turnDegreeb_none(Speed, rel);
}

void turnDirection_none(int direction) {
  turnDegree_none(relativeToDirection(direction));
}

void turnDirectionb_none(int direction) {
  turnDirectionb_none(gyro_MaxSpd_TurnB, direction);
}

// ตั้งทิศทางสัมบูรณ์ให้ RunG / RunGB วิ่งตรงตาม
void SetDirectionG(int direction) {
  float target = fmod((float)direction, 360.0f);
  if (target > 180) target -= 360;
  else if (target < -180) target += 360;
  current_degree = target;
  resetGyroRunError();
}

// ตั้งทิศ "หัวหุ่น" ที่ RunG / RunGB จะล็อก (ไม่หมุนตัว ใช้ก่อนวิ่ง) — อ่านมุมสดก่อนกัน D-term กระชาก
void setHeading(int direction) {
  angleReadFresh();
  SetDirectionG(direction);
}

// ตั้งลำก่อนวิ่งตามทิศ: หัวหุ่นคลาดเกิน gyro_AutoAlign_deg → หมุนอยู่กับที่เข้าทิศก่อน แล้วล็อกทิศ
void alignHeading(int direction) {
  float target = wrap180(fmodf((float)direction, 360.0f));
  angleReadFresh();
  if (gyro_AutoAlign_deg > 0.0f && fabsf(wrap180(target - lastYaw)) > gyro_AutoAlign_deg) {
    spinDirection(direction);
  }
  SetDirectionG(direction);
}

// ---------- ชื่อย่อ: หมุน / เลี้ยวเข้าหาทิศ ----------
void spindir(int Speed, int direction) {
  spinDirection(Speed, direction);
}
void spindir(int direction) {
  spinDirection(direction);
}
void sdg(int Speed, int direction) {
  spinDirection(Speed, direction);
}
void sdg(int direction) {
  spinDirection(direction);
}

void turndir(int Speed, int direction) {
  turnDirection(Speed, direction);
}
void turndir(int direction) {
  turnDirection(direction);
}
void tdg(int Speed, int direction) {
  turnDirection(Speed, direction);
}
void tdg(int direction) {
  turnDirection(direction);
}

void turndirb(int Speed, int direction) {
  turnDirectionB(Speed, direction);
}
void turndirb(int direction) {
  turnDirectionB(direction);
}
void tdbg(int Speed, int direction) {
  turnDirectionB(Speed, direction);
}
void tdbg(int direction) {
  turnDirectionB(direction);
}

void turndir_none(int Speed, int direction) {
  turnDirection_none(Speed, direction);
}
void turndir_none(int direction) {
  turnDirection_none(direction);
}
void turndirb_none(int Speed, int direction) {
  turnDirectionb_none(Speed, direction);
}
void turndirb_none(int direction) {
  turnDirectionb_none(direction);
}

// ---------- วิ่งโค้งกำหนดล้อเอง เข้าหาทิศ (ลอจิกเดียวกับ rotateDegree) ----------
// rotateDir(SpeedL, SpeedR, dir): ตีโค้งตามสัดส่วนล้อจนหัวหุ่นหันทิศ dir แล้วเบรกหยุด
// ทางสั้นที่สุด: ถ้าทิศเป้าอยู่คนละฝั่งกับที่ล้อพาเลี้ยว → สลับล้อ L ↔ R ให้ (โค้งกระจกเงา
//   รัศมีเท่าเดิม เดินหน้า / ถอยหลังเหมือนเดิม) | เป้าอยู่ข้างหลังพอดี (|error| > 175°) ใช้ทิศเดิม
void TrackSelectG(int spd, char select);  // ตัวจริงอยู่ในหัวข้อ 10
void TrackSelectGB(int spd, char select);

static bool rotateDirArc(int SpeedL, int SpeedR, int direction, bool brake) {
  if (SpeedL == SpeedR) return rotateArc(SpeedL, SpeedR, 0, brake, gyro_Kp_Rotate, gyro_Kd_Rotate);
  float target_degree = wrap180(fmod((float)direction, 360.0f));
  angleReadFresh();
  float error = wrap180(target_degree - lastYaw);
  int rot_sign = (SpeedL > SpeedR) ? 1 : -1;
  if (fabsf(error) > gyro_StopThr_Rotate && fabsf(error) <= 175.0f && error * rot_sign < 0.0f) {
    int t = SpeedL;  // สลับล้อ → หมุนกลับฝั่ง เข้าเป้าทางสั้น
    SpeedL = SpeedR;
    SpeedR = t;
  }
  return rotateArc(SpeedL, SpeedR, target_degree, brake, gyro_Kp_Rotate, gyro_Kd_Rotate);
}

void rotateDir(int SpeedL, int SpeedR, int direction) {
  rotateDirArc(SpeedL, SpeedR, direction, true);
}

// ไม่ชะลอ ไม่เบรก ถึงทิศแล้วมอเตอร์ยังวิ่งต่อ
void rotateDir_none(int SpeedL, int SpeedR, int direction) {
  rotateDirArc(SpeedL, SpeedR, direction, false);
}

// ถึงทิศแล้วทำ select ต่อ:
//   'n' / 0 = ไหลต่อ ไม่เบรก | 'f' = เบรกสวนสั้น ๆ แล้ว SetFG (โค้งถอยหลัง = SetBG) แรง / เวลาคิดจากความเร็ว
//   อื่น ๆ  = TrackSelectG (โค้งถอยหลัง = TrackSelectGB) ด้วยความเร็ว = ล้อที่เร็วกว่า
// "ถอยหลัง" = SpeedL + SpeedR < 0 | หมุนอยู่กับที่ล้วน (SpeedL + SpeedR = 0) + 'f' = เบรกแบบ rotateDir
void rotateDir(int SpeedL, int SpeedR, char select, int direction) {
  int sum = SpeedL + SpeedR;
  if (select == 'f' && sum == 0) {
    rotateDir(SpeedL, SpeedR, direction);
    return;
  }
  if (!rotateDirArc(SpeedL, SpeedR, direction, false)) return;  // ล้อเท่ากัน / IMU หลุด
  if (select == 'n' || select == 0) return;
  bool backward = sum < 0;
  int spd = max(abs(SpeedL), abs(SpeedR));
  if (select == 'f') {
    int bs = constrain(spd / 2, 15, 60);
    int bm = constrain(10 + spd / 3, 20, 50);
    gyroBrakeStop(bs, bm, 60, backward);
    // เบรกจำมุมตอนเบรก (อาจเลยเป้ามานิด) → ให้ RunG / FFcmG ถัดไปล็อกทิศ direction จริง
    current_degree = wrap180(fmod((float)direction, 360.0f));
    resetGyroRunError();
    return;
  }
  if (backward) TrackSelectGB(spd, select);
  else TrackSelectG(spd, select);
}


// =====================================================================
//  9) วิ่งตรงด้วย Gyro
// =====================================================================

// ---------- PD คุมทิศ ----------
//  9) วิ่งตรงด้วย Gyro

//  9) วิ่งตรงด้วย Gyro

// จำกัดค่า LeftPower/RightPower ของ gyro ตามโหมด
void ClampGyroPower(float &LeftPower, float &RightPower, int SpeedL, int SpeedR, int mode) {
  switch (mode) {
    case 0:  // ..max, ล้อติดลบ → min
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = MinSpeedG;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = MinSpeedG;
      break;
    case 1:  // min..max
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < MinSpeedG) LeftPower = MinSpeedG;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < MinSpeedG) RightPower = MinSpeedG;
      break;
    case 2:  // -(Speed+down)..Speed+up
      if (LeftPower > SpeedL + gyro_limit_up) LeftPower = SpeedL + gyro_limit_up;
      if (LeftPower < -SpeedL - gyro_limit_down) LeftPower = -SpeedL - gyro_limit_down;
      if (RightPower > SpeedR + gyro_limit_up) RightPower = SpeedR + gyro_limit_up;
      if (RightPower < -SpeedR - gyro_limit_down) RightPower = -SpeedR - gyro_limit_down;
      break;
    case 3:  // ..max, ล้อติดลบ → -BaseSpeed
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = -BaseSpeed;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = -BaseSpeed;
      break;
    case 4:  // ..Speed+up, ล้อติดลบ → min (min = 0 คือไม่ถอยล้อ)
      if (LeftPower > SpeedL + gyro_limit4_up) LeftPower = SpeedL + gyro_limit4_up;
      if (LeftPower < 0) LeftPower = MinSpeedG;
      if (RightPower > SpeedR + gyro_limit4_up) RightPower = SpeedR + gyro_limit4_up;
      if (RightPower < 0) RightPower = MinSpeedG;
      break;
    default:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = 0;
  }
}

void RunG(int SpeedL, int SpeedR) {
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorG;
  int pd_value = (error * kpG) + (derivative * kdG);
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroStatus);
  Motor(leftPow, rightPow);
  previous_errorG = error;
  delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นวิ่งช้าลงและลดการสั่น
}

void RunGB(int SpeedL, int SpeedR) {
  float error = angleRead() - current_degree;  // ถอยหลัง: กลับด้าน error เทียบกับ RunG
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorGB;
  int pd_value = (error * kpGB) + (derivative * kdGB);
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroBStatus);
  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
  delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นวิ่งช้าลงและลดการสั่น
}

// 1 สเต็ป ล็อกทิศ dir (ใช้ใน while loop เอง) — dir = ทิศที่หัวหุ่นหัน ทั้งเดินหน้าและถอยหลัง
// เปลี่ยน current_degree + รีเซ็ต D-term เฉพาะตอนทิศเปลี่ยน (ไม่ล้าง D ทุกสเต็ป)
static void runDirUpdate(int direction) {
  float dir = wrap180(fmodf((float)direction, 360.0f));
  if (fabsf(wrap180(dir - current_degree)) > 0.01f) {
    current_degree = dir;
    resetGyroRunError();
  }
}

void RunG(int SpeedL, int SpeedR, int direction) {
  runDirUpdate(direction);
  RunG(SpeedL, SpeedR);
}

void RunGB(int SpeedL, int SpeedR, int direction) {
  runDirUpdate(direction);
  RunGB(SpeedL, SpeedR);
}

// ---------- จับเวลา ----------
void FFtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
  }
}

void BBtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
  }
}

// ---------- เข้ากลางหุ่น / เข้าหาเส้น ----------
void ToCenterG() {
  RunG(tctL, tctR);
  delay(20);
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(tct_delay_break);
      MotorStop();
      break;
    }
  }
}

void ToFrontG() {
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref) break;
  }
}

void BackToCenterG() {
  RunGB(bctL, bctR);
  delay(20);
  while (1) {
    RunGB(bctL, bctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(bct_delay_break);
      MotorStop();
      break;
    }
  }
}

void BackToFrontG() {
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[3] > Ref || B[12] > Ref) break;
  }
}


// =====================================================================
//  10) คำสั่งเมื่อเจอแยก (Gyro)
// =====================================================================

//  10) คำสั่งเมื่อเจอแยก (Gyro)

//  10) คำสั่งเมื่อเจอแยก (Gyro)
// ---------- ปัดมุมเข้าแกน (snap90) ก่อนเลี้ยวที่ทางแยก ----------
// track_snap90 = true: ปัดมุมจริงของหุ่นเข้าแกนที่ใกล้สุด (0, 90, -90, 180) แล้วหมุน / เลี้ยวไป
// "แกน ± 90" แบบทิศจริง → error มุมไม่สะสมทีละทางแยก | ปิดด้วย set_track_snap90(false)
int snap90(float angle) {
  int s = (int)lroundf(wrap180(angle) / 90.0f) * 90;
  if (s <= -180) s = 180;
  return s;
}

static int trackTarget(int rel) {
  angleReadFresh();
  return snap90(lastYaw) + rel;
}

void trackSpin(int rel) {
  if (track_snap90) spinDirection(trackTarget(rel));
  else spinDegree(rel);
}

void trackTurn(int rel) {
  if (track_snap90) turnDirection(trackTarget(rel));
  else turnDegree(rel);
}

void trackTurnB(int rel) {
  if (track_snap90) turnDirectionB(trackTarget(rel));
  else turnDegreeB(rel);
}

void TrackSelectG(int spd, char select) {
  if (select == 'L') {
    trackSpin(-90);
  } else if (select == 'l') {
    ToCenterG();
    trackSpin(-90);
  } else if (select == 'R') {
    trackSpin(90);
  } else if (select == 'r') {
    ToCenterG();
    trackSpin(90);
  } else if (select == 'Q') {
    trackTurn(-90);
  } else if (select == 'q') {
    ToFrontG();
    trackTurn(-90);
  } else if (select == 'E') {
    trackTurn(90);
  } else if (select == 'e') {
    ToFrontG();
    trackTurn(90);
  } else if (select == 'p') {
    BZon();
    ReadCalibrateF();
    while (1) {
      RunG(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    FFtimerG(spd, 5);
    while (1) {
      RunG(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    BZoff();
  } else if (select == 'P') {
    ToFrontG();
    BZon();
    ReadCalibrateF();
    while (1) {
      RunG(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    FFtimerG(spd, 5);
    while (1) {
      RunG(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    BZoff();
  } else if (select == 's') {
    Motor(-spd, -spd);
    delay(delay_break_f);
    MotorStop();
  } else if (select == 'S') {
    ToFrontG();
    Motor(-tctL, -tctR);
    delay(tct_delay_break);
    MotorStop();
  } else if (select == 'c' || select == 'C') {
    ToCenterG();
  } else if (select == 'b' || select == 'B') {
    ReadCalibrateB();
    while (1) {
      RunG(spd, spd);
      ReadCalibrateB();
      if (B[3] > Ref && B[12] > Ref) break;
    }
  } else if (select == 'g' || select == 'G') {
    SetG(spd);
  } else {
    Stop(100);
  }
}

void TrackSelectGB(int spd, char select) {
  if (select == 'L') {
    trackSpin(-90);
  } else if (select == 'l') {
    BackToCenterG();
    trackSpin(-90);
  } else if (select == 'R') {
    trackSpin(90);
  } else if (select == 'r') {
    BackToCenterG();
    trackSpin(90);
  } else if (select == 'q' || select == 'Q') {
    trackTurnB(90);
  } else if (select == 'e' || select == 'E') {
    trackTurnB(-90);
  } else if (select == 'p') {
    BZon();
    ReadCalibrateB();
    while (1) {
      RunGB(spd, spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BBtimerG(spd, 5);
    while (1) {
      RunGB(spd, spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BZoff();
  } else if (select == 'P') {
    BZon();
    ReadCalibrateB();
    while (1) {
      RunGB(spd, spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BBtimerG(spd, 5);
    while (1) {
      RunGB(spd, spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BZoff();
  } else if (select == 's') {
    Motor(spd, spd);
    delay(delay_break_b);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else if (select == 'S') {
    BackToFrontG();
    Motor(bctL, bctR);
    delay(bct_delay_break);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else if (select == 'c') {
    BackToCenterG();
  } else if (select == 'C') {
    ToFrontG();
    BackToCenterG();
  } else if (select == 'b' || select == 'B') {
    ReadCalibrateB();
    while (1) {
      RunGB(spd, spd);
      ReadCalibrateB();
      if (B[3] > Ref && B[12] > Ref) break;
    }
  } else if (select == 'g' || select == 'G') {
    SetG(spd);
  } else {
    Stop(100);
  }
}


// =====================================================================
//  11) วิ่งตรงด้วย Gyro + คำสั่งเจอแยก
// =====================================================================

//  11) วิ่งตรงด้วย Gyro + คำสั่งเจอแยก

//  11) วิ่งตรงด้วย Gyro + คำสั่งเจอแยก
void FFBG(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[5] > Ref || F[6] > Ref || F[7] > Ref || F[8] > Ref || F[9] > Ref || F[10] > Ref) break;
  }
  TrackSelectG(Speed, select);
}

void BBBG(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[5] > Ref || B[6] > Ref || B[7] > Ref || B[8] > Ref || B[9] > Ref || B[10] > Ref) break;
  }
  TrackSelectGB(Speed, select);
}

void FFtimerG(int Speed, int totalTime, char select) {
  FFtimerG(Speed, totalTime);
  TrackSelectG(Speed, select);
}

void BBtimerG(int Speed, int totalTime, char select) {
  BBtimerG(Speed, totalTime);
  TrackSelectGB(Speed, select);
}


// =====================================================================
//  12) วิ่งตามระยะทางด้วย Gyro (cm)
// =====================================================================

//  12) วิ่งตามระยะทางด้วย Gyro (cm)

//  12) วิ่งตามระยะทางด้วย Gyro (cm)
void FFcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();


  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน
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
      traveled_distance += (target_speed * speed_scale_ff_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
  }
}

void BBcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();


  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน
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
      traveled_distance += (target_speed * speed_scale_bb_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
  }
}

void FFcmG(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));

  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  // ====================== ค่าที่สามารถปรับได้ ======================
  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  // ค่า speed_scale ที่คุณต้องการปรับได้ (ค่าดีฟอลต์ = 0.99)
  float speed_scale = speed_scale_ff_g_normal;  // ปรับได้ด้วย set_speed_scale_ff_g()

  // ตัดสินใจว่าใช้ Ramp หรือไม่
  bool enableRamp = (distance_cm >= 30.0);

  // ถ้าระยะสั้นมาก (< 30) ให้ปรับ speed_scale ได้ง่ายขึ้น
  if (!enableRamp) {
    speed_scale = speed_scale_ff_g;
  }
  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน

  while (true) {
    // คำนวณระยะทาง
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;

    if (remaining_cm <= 0.7f) break;

    // ====================== คำนวณ target_speed ======================
    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        // เร่งช่วงแรก
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        // ชะลอช่วงสุดท้าย
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    // ถ้า enableRamp = false → ใช้ความเร็วคงที่ตลอดทาง
    RunG(target_speed, target_speed);
  }
}

void BBcmG(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));

  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  float speed_scale = speed_scale_bb_g_normal;  // ปรับได้ด้วย set_speed_scale_bb_g()

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  // ตัดสินใจว่าใช้ Ramp หรือไม่
  bool enableRamp = (distance_cm >= 30.0);

  // ถ้าระยะสั้นมาก (< 30) → ไม่ใช้ Ramp + ปรับ speed_scale
  if (!enableRamp) {
    speed_scale = speed_scale_bb_g;
  }
  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน

  while (true) {
    // คำนวณระยะทาง
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;

    if (remaining_cm <= 0.8f) break;

    // ====================== คำนวณความเร็ว ======================
    float target_speed = base_speed;

    if (enableRamp) {  // ใช้ Ramp เฉพาะระยะยาว (>=30 cm)
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    // ถ้า !enableRamp → ใช้ความเร็วคงที่ตลอดทาง (base_speed)

    RunGB(target_speed, target_speed);
  }
}

void FFcmGS(int Speed, float distance_cm, char select) {
  FFcmGS(Speed, distance_cm);
  TrackSelectG(Speed, select);
}

void BBcmGS(int Speed, float distance_cm, char select) {
  BBcmGS(Speed, distance_cm);
  TrackSelectGB(Speed, select);
}

void FFcmG(int Speed, float distance_cm, char select) {
  FFcmG(Speed, distance_cm);
  TrackSelectG(Speed, select);
}

void BBcmG(int Speed, float distance_cm, char select) {
  BBcmG(Speed, distance_cm);
  TrackSelectGB(Speed, select);
}


// =====================================================================
//  13) วิ่งตรงตามทิศสัมบูรณ์
// =====================================================================

// ---------- ไม่มีคำสั่งเจอแยก ----------
//  13) วิ่งตรงตามทิศสัมบูรณ์

//  13) วิ่งตรงตามทิศสัมบูรณ์
void FFtimerG(int Speed, int totalTime, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  FFtimerG(Speed, totalTime);
}

void BBtimerG(int Speed, int totalTime, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  BBtimerG(Speed, totalTime);
}

void FFcmGS(int Speed, float distance_cm, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  useDirectionG = true;
  FFcmGS(Speed, distance_cm);
  useDirectionG = false;
}

void BBcmGS(int Speed, float distance_cm, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  useDirectionG = true;
  BBcmGS(Speed, distance_cm);
  useDirectionG = false;
}

void FFcmG(int Speed, float distance_cm, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  useDirectionG = true;
  FFcmG(Speed, distance_cm);
  useDirectionG = false;
}

void BBcmG(int Speed, float distance_cm, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  useDirectionG = true;
  BBcmG(Speed, distance_cm);
  useDirectionG = false;
}

// ---------- มีคำสั่งเจอแยก ----------
void FFBG(int Speed, char select, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  FFBG(Speed, select);
}

void BBBG(int Speed, char select, int direction) {
  alignHeading(direction);  // คลาดเกิน set_auto_align → หมุนตั้งลำก่อน แล้วล็อกทิศ
  BBBG(Speed, select);
}

void FFtimerG(int Speed, int totalTime, char select, int direction) {
  FFtimerG(Speed, totalTime, direction);
  TrackSelectG(Speed, select);
}

void BBtimerG(int Speed, int totalTime, char select, int direction) {
  BBtimerG(Speed, totalTime, direction);
  TrackSelectGB(Speed, select);
}

void FFcmGS(int Speed, float distance_cm, char select, int direction) {
  FFcmGS(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}

void BBcmGS(int Speed, float distance_cm, char select, int direction) {
  BBcmGS(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

void FFcmG(int Speed, float distance_cm, char select, int direction) {
  FFcmG(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}

void BBcmG(int Speed, float distance_cm, char select, int direction) {
  BBcmG(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

// ---------- คอมโบ: หมุน / เลี้ยวเข้าหาทิศ แล้ววิ่งตามระยะต่อทันที ----------
// dir = ทิศที่หัวหุ่นหัน (ทั้ง ff และ bb) | spin = หมุนอยู่กับที่, turn = เลี้ยวหน้าล้อเดียว, turnb = เลี้ยวถอยล้อเดียว
void spin_ffcmg(int Speed, float distance_cm, int direction) {
  spinDirection(direction);
  FFcmG(Speed, distance_cm, direction);
}
void spin_ffcmg(int Speed, float distance_cm, char select, int direction) {
  spinDirection(direction);
  FFcmG(Speed, distance_cm, select, direction);
}
void spin_bbcmg(int Speed, float distance_cm, int direction) {
  spinDirection(direction);
  BBcmG(Speed, distance_cm, direction);
}
void spin_bbcmg(int Speed, float distance_cm, char select, int direction) {
  spinDirection(direction);
  BBcmG(Speed, distance_cm, select, direction);
}

void turn_ffcmg(int Speed, float distance_cm, int direction) {
  turnDirection(direction);
  FFcmG(Speed, distance_cm, direction);
}
void turn_ffcmg(int Speed, float distance_cm, char select, int direction) {
  turnDirection(direction);
  FFcmG(Speed, distance_cm, select, direction);
}
void turn_bbcmg(int Speed, float distance_cm, int direction) {
  turnDirection(direction);
  BBcmG(Speed, distance_cm, direction);
}
void turn_bbcmg(int Speed, float distance_cm, char select, int direction) {
  turnDirection(direction);
  BBcmG(Speed, distance_cm, select, direction);
}

void turnb_ffcmg(int Speed, float distance_cm, int direction) {
  turnDirectionB(direction);
  FFcmG(Speed, distance_cm, direction);
}
void turnb_ffcmg(int Speed, float distance_cm, char select, int direction) {
  turnDirectionB(direction);
  FFcmG(Speed, distance_cm, select, direction);
}
void turnb_bbcmg(int Speed, float distance_cm, int direction) {
  turnDirectionB(direction);
  BBcmG(Speed, distance_cm, direction);
}
void turnb_bbcmg(int Speed, float distance_cm, char select, int direction) {
  turnDirectionB(direction);
  BBcmG(Speed, distance_cm, select, direction);
}


// =====================================================================
//  14) วิ่งตามเส้นจนหันถึงทิศ (FFdir / BBdir)
// =====================================================================

// วิ่งตามเส้น (PIDF / PIDB) ไปเรื่อย ๆ จนมุมไจโรถึงทิศสัมบูรณ์ (0 / 90 / 180 / 270 อ้างอิงตอน resetAngles)
// ใช้กับเส้นโค้ง เช่น FFdir(50, 270); = วิ่งตามเส้นโค้งไปจนหุ่นหันไปทิศ 270
// ไม่มี select: จบแล้วมอเตอร์ยังหมุนอยู่ ต่อคำสั่งถัดไปได้ทันที

// ---------- ตัวช่วย ----------
// true = มุมปัจจุบันถึงทิศ direction แล้ว (อยู่ในช่วง threshold หรือหมุนเลยเป้าไปแล้ว)
static bool reachedDirection(int direction, float &previous_error) {
  float error = wrap180(fmodf((float)direction, 360.0f) - angleRead());
  // มุมอาจกระโดดข้ามช่วง threshold ตอนเลี้ยวเร็ว จึงถือว่าถึงเมื่อ error เปลี่ยนเครื่องหมายด้วย
  // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
  bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
  previous_error = error;
  return fabsf(error) <= line_dir_threshold || crossed;
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front / PID_KD_Front)
static void FFdir_pid(int Speed, int direction, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  float previous_error = wrap180(fmodf((float)direction, 360.0f) - angleReadFresh());
  while (!reachedDirection(direction, previous_error)) PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Back / PID_KD_Back)
static void BBdir_pid(int Speed, int direction, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  float previous_error = wrap180(fmodf((float)direction, 360.0f) - angleReadFresh());
  while (!reachedDirection(direction, previous_error)) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
}

// ---------- FFdir / BBdir: (speed, [kp, [kd,]] ทิศ [, select])  ใส่ค่าติดลบ = ใช้ค่าจากตาราง ----------
void FFdir(int Speed, int direction) {
  FFdir_pid(Speed, direction, -1, -1);
}

void BBdir(int Speed, int direction) {
  BBdir_pid(Speed, direction, -1, -1);
}

void FFdir(int Speed, int direction, char select) {
  FFdir(Speed, direction);
  TrackSelectF(Speed, select);
}

void BBdir(int Speed, int direction, char select) {
  BBdir(Speed, direction);
  TrackSelectB(Speed, select);
}

void FFdir(int Speed, float Kp, int direction) {
  FFdir_pid(Speed, direction, Kp, -1);
}

void BBdir(int Speed, float Kp, int direction) {
  BBdir_pid(Speed, direction, Kp, -1);
}

void FFdir(int Speed, float Kp, int direction, char select) {
  FFdir_pid(Speed, direction, Kp, -1);
  TrackSelectF(Speed, select);
}

void BBdir(int Speed, float Kp, int direction, char select) {
  BBdir_pid(Speed, direction, Kp, -1);
  TrackSelectB(Speed, select);
}

void FFdir(int Speed, float Kp, float Kd, int direction) {
  FFdir_pid(Speed, direction, Kp, Kd);
}

void BBdir(int Speed, float Kp, float Kd, int direction) {
  BBdir_pid(Speed, direction, Kp, Kd);
}

void FFdir(int Speed, float Kp, float Kd, int direction, char select) {
  FFdir_pid(Speed, direction, Kp, Kd);
  TrackSelectF(Speed, select);
}

void BBdir(int Speed, float Kp, float Kd, int direction, char select) {
  BBdir_pid(Speed, direction, Kp, Kd);
  TrackSelectB(Speed, select);
}

// ---------- วิ่งโค้ง: CirCleL = เบี่ยงซ้าย (set_position_l), CirCleR = เบี่ยงขวา (set_position_r) ----------
void FFdirCirCleL(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, direction);
  set_position = temp;
}

void FFdirCirCleL(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, direction, select);
  set_position = temp;
}

void FFdirCirCleL(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void FFdirCirCleL(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void FFdirCirCleL(int Speed, float Kp, int direction) {
  FFdirCirCleL(Speed, Kp, -1.0f, direction);
}

void FFdirCirCleL(int Speed, float Kp, int direction, char select) {
  FFdirCirCleL(Speed, Kp, -1.0f, direction, select);
}

void FFdirCirCleR(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, direction);
  set_position = temp;
}

void FFdirCirCleR(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, direction, select);
  set_position = temp;
}

void FFdirCirCleR(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void FFdirCirCleR(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void FFdirCirCleR(int Speed, float Kp, int direction) {
  FFdirCirCleR(Speed, Kp, -1.0f, direction);
}

void FFdirCirCleR(int Speed, float Kp, int direction, char select) {
  FFdirCirCleR(Speed, Kp, -1.0f, direction, select);
}

void BBdirCirCleL(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, direction);
  set_position = temp;
}

void BBdirCirCleL(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, direction, select);
  set_position = temp;
}

void BBdirCirCleL(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void BBdirCirCleL(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void BBdirCirCleL(int Speed, float Kp, int direction) {
  BBdirCirCleL(Speed, Kp, -1.0f, direction);
}

void BBdirCirCleL(int Speed, float Kp, int direction, char select) {
  BBdirCirCleL(Speed, Kp, -1.0f, direction, select);
}

void BBdirCirCleR(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, direction);
  set_position = temp;
}

void BBdirCirCleR(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, direction, select);
  set_position = temp;
}

void BBdirCirCleR(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void BBdirCirCleR(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void BBdirCirCleR(int Speed, float Kp, int direction) {
  BBdirCirCleR(Speed, Kp, -1.0f, direction);
}

void BBdirCirCleR(int Speed, float Kp, int direction, char select) {
  BBdirCirCleR(Speed, Kp, -1.0f, direction, select);
}


// =====================================================================
//  15) วิ่งตามเส้นจนหันถึงทิศ / องศา (ffdeg / ffdirg / ffdircl ...)
// =====================================================================

// วิ่งตามเส้น (PIDF / PIDB) แล้วใช้ไจโรเช็กว่าเลี้ยวพ้นโค้งหรือยัง เหมาะกับโค้ง 90°, ตัวยู, ตัว S
// dir = ทิศที่หัวหุ่นหัน (ทั้ง ff และ bb เหมือน FFdir / BBdir) | deg = องศาสัมพัทธ์จากมุมตอนเริ่มคำสั่ง (+ = ขวา)
// ถึงเมื่อ |error| <= set_line_dir_threshold ติดกัน 2 ค่าใหม่จาก IMU หรือข้ามเป้า (ตอน |error| < 45)
// select: 'n' / 0 = ไหลต่อ ไม่เบรก | 'f' = เบรกไจโร (ดู set_line_dir_brake) | อื่น ๆ = TrackSelect
// จบแล้วล็อกทิศเป้าหมาย (current_degree) → ต่อด้วย FFcmG / RunG วิ่งตรงทิศนั้นได้ทันที

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว
static void lineToHeading(int Speed, float target_degree, bool backward, float Kp = -1.0f, float Kd = -1.0f) {
  float tol = (line_dir_threshold > 0.0f) ? line_dir_threshold : 4.0f;
  target_degree = wrap180(target_degree);
  BaseSpeed = Speed;
  InitialSpeed();
  float kp = (Kp >= 0.0f) ? Kp : (backward ? PID_KP_Back : PID_KP_Front);
  float kd = (Kd >= 0.0f) ? Kd : (backward ? PID_KD_Back : PID_KD_Front);

  angleReadFresh();  // ล้าง lastYaw เก่าที่ค้างตอนวิ่งตามเส้นโดยไม่ได้อ่านไจโร
  float previous_error = wrap180(target_degree - lastYaw);
  int in_tol_count = (fabsf(previous_error) <= tol) ? 1 : 0;
  uint32_t lastFresh = millis();

  while (1) {
    if (backward) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    else PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);

    if (!(myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)) {
      if (millis() - lastFresh > 500) {  // IMU หลุด → หยุด กันวิ่งไม่รู้จบ
        Serial.println("line dir: IMU timeout");
        MotorStop();
        Beep(300);
        break;
      }
      continue;
    }
    lastFresh = millis();
    updateYawFromEvent();

    float error = wrap180(target_degree - lastYaw);
    bool crossed = fabsf(error) < 45.0f && ((error > 0.0f && previous_error < 0.0f) || (error < 0.0f && previous_error > 0.0f));
    previous_error = error;
    if (fabsf(error) <= tol) in_tol_count++;
    else in_tol_count = 0;
    if (in_tol_count >= 2 || crossed) break;
  }

  current_degree = target_degree;
  resetGyroRunError();
}

static void lineDirGyroBrake(int Speed, bool backward) {
  int spd = abs(Speed);
  int bs = (line_dir_brake_spd > 0) ? line_dir_brake_spd : constrain(spd / 2, 15, 60);
  int bm = (line_dir_brake_ms > 0) ? line_dir_brake_ms : constrain(10 + spd / 3, 20, 50);
  gyroBrakeStop(bs, bm, line_dir_hold_ms, backward);
}

// ทำ select หลังถึงทิศ: gyroMode = true → TrackSelectG / GB, false → TrackSelectF / B
static void lineDirAct(int Speed, char select, bool backward, bool gyroMode) {
  if (select == 'n' || select == 0) return;
  if (select == 'f') {
    lineDirGyroBrake(Speed, backward);
    return;
  }
  if (gyroMode) {
    if (backward) TrackSelectGB(Speed, select);
    else TrackSelectG(Speed, select);
  } else {
    if (backward) TrackSelectB(Speed, select);
    else TrackSelectF(Speed, select);
  }
}

static float lineDirTarget(int direction) {
  return wrap180(fmodf((float)direction, 360.0f));
}

static float lineRelTarget(int relative_deg) {
  angleReadFresh();
  return wrap180(lastYaw + relative_deg);
}

// ---------- ทิศจริง จบด้วยโหมดไจโร (TrackSelectG / GB) ----------
// ห้ามสลับลำดับ select / ทิศ (ตัวเลขกับตัวอักษรแปลงกันได้เงียบ ๆ → คอมไพล์ไม่ผ่านแทน)
void ffdirg(int Speed, int, char) = delete;
void bbdirg(int Speed, int, char) = delete;
void ffdirf(int Speed, char, int) = delete;
void bbdirb(int Speed, char, int) = delete;
void ffdeg(int Speed, int, char) = delete;
void bbdeg(int Speed, int, char) = delete;
void ffdegg(int Speed, int, char) = delete;
void bbdegg(int Speed, int, char) = delete;

void ffdirg(int Speed, char select, int direction) {
  lineToHeading(Speed, lineDirTarget(direction), false);
  lineDirAct(Speed, select, false, true);
}
void bbdirg(int Speed, char select, int direction) {
  lineToHeading(Speed, lineDirTarget(direction), true);
  lineDirAct(Speed, select, true, true);
}

// ---------- ทิศจริง จบด้วยโหมดเส้น (TrackSelectF / B) — เหมือน FFdir(speed, dir, select) แต่รับ 'n' / 'f' ----------
void ffdirf(int Speed, int direction, char select) {
  lineToHeading(Speed, lineDirTarget(direction), false);
  lineDirAct(Speed, select, false, false);
}
void bbdirb(int Speed, int direction, char select) {
  lineToHeading(Speed, lineDirTarget(direction), true);
  lineDirAct(Speed, select, true, false);
}

// ---------- องศาสัมพัทธ์ (นับจากมุมตอนเริ่มคำสั่ง) ----------
void ffdeg(int Speed, int relative_deg) {
  lineToHeading(Speed, lineRelTarget(relative_deg), false);
}
void bbdeg(int Speed, int relative_deg) {
  lineToHeading(Speed, lineRelTarget(relative_deg), true);
}
void ffdeg(int Speed, char select, int relative_deg) {
  ffdeg(Speed, relative_deg);
  lineDirAct(Speed, select, false, false);
}
void bbdeg(int Speed, char select, int relative_deg) {
  bbdeg(Speed, relative_deg);
  lineDirAct(Speed, select, true, false);
}
void ffdegg(int Speed, char select, int relative_deg) {
  ffdeg(Speed, relative_deg);
  lineDirAct(Speed, select, false, true);
}
void bbdegg(int Speed, char select, int relative_deg) {
  bbdeg(Speed, relative_deg);
  lineDirAct(Speed, select, true, true);
}


// ---------- เกาะเส้นโค้ง cl (set_position_line_l) / cr (set_position_line_r) จนหันถึงทิศ / องศา ----------
// select ทำตอน set_position ยังเบี่ยงอยู่ แล้วคืนค่า set_position
static void lineCurveToHeading(int Speed, float target, bool backward, bool right, char select, bool gyroMode,
                               float Kp = -1.0f, float Kd = -1.0f) {
  int temp = set_position;
  set_position = right ? set_position_r : set_position_l;
  lineToHeading(Speed, target, backward, Kp, Kd);
  lineDirAct(Speed, select, backward, gyroMode);
  set_position = temp;
}

// (Speed, dir) ไหลต่อ | (Speed, select, dir) → TrackSelectF / B | ...g(Speed, select, dir) → TrackSelectG / GB
// กำหนด Kp / Kd เอง: X(Speed, Kp, [Kd,] select, dir) — ไม่ต้องการ select ใส่ 'n'
#define LINE_CURVE_SET(NAME, NAMEG, BACK, RIGHT, TARGET) \
  void NAME(int Speed, int, char) = delete; /* ห้ามสลับลำดับ: select ไว้หน้าทิศ */   void NAMEG(int Speed, int, char) = delete;   void NAME(int Speed, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, 0, false); \
  } \
  void NAME(int Speed, char select, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, select, false); \
  } \
  void NAME(int Speed, float Kp, char select, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, select, false, Kp, -1.0f); \
  } \
  void NAME(int Speed, float Kp, float Kd, char select, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, select, false, Kp, Kd); \
  } \
  void NAMEG(int Speed, char select, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, select, true); \
  } \
  void NAMEG(int Speed, float Kp, char select, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, select, true, Kp, -1.0f); \
  } \
  void NAMEG(int Speed, float Kp, float Kd, char select, int v) { \
    lineCurveToHeading(Speed, TARGET(v), BACK, RIGHT, select, true, Kp, Kd); \
  }

// ทิศจริง
LINE_CURVE_SET(ffdircl, ffdirclg, false, false, lineDirTarget)
LINE_CURVE_SET(ffdircr, ffdircrg, false, true, lineDirTarget)
LINE_CURVE_SET(bbdircl, bbdirclg, true, false, lineDirTarget)
LINE_CURVE_SET(bbdircr, bbdircrg, true, true, lineDirTarget)
// องศาสัมพัทธ์
LINE_CURVE_SET(ffdegcl, ffdegclg, false, false, lineRelTarget)
LINE_CURVE_SET(ffdegcr, ffdegcrg, false, true, lineRelTarget)
LINE_CURVE_SET(bbdegcl, bbdegclg, true, false, lineRelTarget)
LINE_CURVE_SET(bbdegcr, bbdegcrg, true, true, lineRelTarget)

#undef LINE_CURVE_SET
