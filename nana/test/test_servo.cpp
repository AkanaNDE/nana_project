#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;
Servo myServo2;
Servo myServo3;

const int servoPin = 27;
const int servo2pin = 26;
const int servo3pin = 25;

void setup() {
  Serial.begin(115200);

  myServo.attach(servoPin);
  myServo2.attach(servo2pin);
  myServo3.attach(servo3pin);

  Serial.println("Servo test start");
}

void loop() {

  // ไปตำแหน่ง 0
  //myServo.write(180);
  myServo2.write(90);
  myServo3.write(0);
  Serial.println("Position 100");
  delay(1000);

  // ไปตำแหน่ง 60
  //myServo.write(120);
  myServo2.write(120);
  myServo3.write(90);
  Serial.println("Position 120");
  delay(1000);

}