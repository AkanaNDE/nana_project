#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

const int servoPin = 18;  // Connect your servo signal wire here

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
  Serial.println("Servo test start");
}

void loop() {
  // Sweep from 0 to 180
  for (int pos = 0; pos <= 180; pos += 1) {
    myServo.write(pos);
    delay(10);
  }

  // Sweep back from 180 to 0
  for (int pos = 180; pos >= 0; pos -= 1) {
    myServo.write(pos);
    delay(10);
  }
}
