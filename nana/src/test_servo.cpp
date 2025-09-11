#include <Arduino.h>
#include <Servo.h>

Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(18);  // Attach to GPIO 18 (can be any PWM-capable pin)
  Serial.println("Servo ready");
}

void loop() {
  // Sweep 0 to 180
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(15);
  }

  // Sweep back to 0
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(15);
  }
}
