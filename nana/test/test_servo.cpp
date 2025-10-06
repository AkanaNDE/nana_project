#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

const int servoPin = 27;  // Connect your servo signal wire here

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);  // You can also use: myServo.attach(servoPin, 500, 2400);
  Serial.println("Servo test start");
}

void loop() {
  // Sweep from 180 down to 0
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(10);
  }

  // Optional: Pause before sweeping back up
  delay(500);

  // Sweep from 0 up to 180
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(10);
  }

  delay(500);
}
