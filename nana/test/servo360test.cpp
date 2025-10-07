#include <Arduino.h>
#include <ESP32Servo.h>  // ต้องใช้ไลบรารี ESP32Servo

Servo myservo;

// กำหนดขา PWM ของ ESP32
int servoPin = 18;   // เลือกขาที่รองรับ PWM เช่น 18, 19, 21, 22, 23 ฯลฯ

void setup() {
  // Attach servo
  myservo.attach(servoPin, 500, 2500); 
  // ค่า 500-2500 คือช่วง pulse width (us) ที่ใช้กับ ESP32
}

void loop() {
  // หยุดนิ่ง (90° pulse)
  myservo.write(90);
  delay(2000);

  // หมุนตามเข็ม (ค่ามากกว่า 90)
  myservo.write(180);   // 100–180 หมุนตามเข็ม ยิ่งสูงยิ่งเร็ว
  delay(2000);

  // หมุนทวนเข็ม (ค่าน้อยกว่า 90)
  myservo.write(0);     // 0–80 หมุนทวนเข็ม ยิ่งต่ำยิ่งเร็ว
  delay(2000);
}