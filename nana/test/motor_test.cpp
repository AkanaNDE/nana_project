#include <Arduino.h>
#include <ESP32Servo.h>  // ต้องใช้ไลบรารี ESP32Servo

// Pin map
const int IN1 = 25, IN2 = 26, ENA = 32;
const int IN3 = 14, IN4 = 33, ENB = 22;

// PWM config
const int CH_A = 0;
const int CH_B = 1;
const int FREQ = 20000;     // 20 kHz
const int RES  = 10;        // 0-1023

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // ตั้ง PWM
  ledcSetup(CH_A, FREQ, RES);
  ledcSetup(CH_B, FREQ, RES);
  ledcAttachPin(ENA, CH_A);
  ledcAttachPin(ENB, CH_B);
}

// ฟังก์ชันควบคุม
void motorA(int speed, bool dir) {
  digitalWrite(IN1, dir);
  digitalWrite(IN2, !dir);
  speed = constrain(speed, 0, 1023);
  ledcWrite(CH_A, speed);
}

void motorB(int speed, bool dir) {
  digitalWrite(IN3, dir);
  digitalWrite(IN4, !dir);
  speed = constrain(speed, 0, 1023);
  ledcWrite(CH_B, speed);
}

void loop() {
  // วิ่งตรงไป
  motorA(800, true);
  motorB(800, true);
  delay(2000);

  // ถอยหลัง
  motorA(800, false);
  motorB(800, false);
  delay(2000);

  // เลี้ยวซ้าย
  motorA(800, true);
  motorB(400, true);
  delay(2000);

  // หยุด
  motorA(0, true);
  motorB(0, true);
  delay(2000);
}