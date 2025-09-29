#include <Arduino.h>

// === Pin Definitions ===
const int STEP_PIN = 22;     // Step signal
const int DIR_PIN = 13;      // Direction signal

const int ENABLE_PIN = 21; // Optional: Uncomment if using an enable pin

// === Stepper Configuration ===
const int STEPS_PER_REV = 200;       // Steps per revolution (adjust as needed)
const int STEP_DELAY_US = 1000;      // Microseconds between steps (controls speed)

// === Direction Constants ===


void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // or 9600 if you prefer
  delay(500);            // Give time for Serial Monitor to connect

  // Set pin modes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);             
  digitalWrite(ENABLE_PIN, LOW);           // Enable driver (LOW for A4988/DRV8825)

  Serial.println("Stepper motor test starting...");
}

void loop() {
  // Rotate one revolution
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  // Rotate back
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}
