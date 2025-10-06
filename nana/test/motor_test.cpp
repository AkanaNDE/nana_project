#include <Arduino.h>

// กำหนดขาควบคุม
#define A11 25
#define A12 26
#define B11 14
#define B12 33

#define ENCODER_A 32
#define ENCODER_B 12

volatile long encoderCount = 0;

void IRAM_ATTR handleEncoderA() {
  int b = digitalRead(ENCODER_B);
  if (b == HIGH) encoderCount++;
  else encoderCount--;
}

void setup() {
  Serial.begin(115200);

  pinMode(A11, OUTPUT);
  pinMode(A12, OUTPUT);

  pinMode(B11, OUTPUT);
  pinMode(B12, OUTPUT);


  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, CHANGE);
}

void loop() {
  // หมุนไปข้างหน้า
  ledcAttachPin(A11, 0);
  ledcSetup(0, 1000, 8);
  ledcWrite(0, 200); // ความเร็ว 0-255
  digitalWrite(A12, LOW);
  delay(2000);

  ledcAttachPin(B12, 1);
  ledcSetup(1, 1000, 8);
  ledcWrite(1, 200); // ความเร็ว 0-255
  digitalWrite(B11, LOW);
  delay(2000);

  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);

}