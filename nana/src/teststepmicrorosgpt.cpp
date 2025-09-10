#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist.h>

// Define pins
#define STEP_PIN 22
#define DIR_PIN 13
#define ENABLE_PIN 21


// Constants
const int stepsPerRev = 200;        // depends on your motor (e.g., 1.8° = 200 steps/rev)
const int microstepping = 16;       // depends on your driver
const float maxRPM = 1000.0;

// Variables
float target_rpm = 0.0;
unsigned long last_step_time = 0;
unsigned long step_interval_us = 0;
bool step_state = false;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist rpm_msg;

rcl_node_t node;
rclc_executor_t executor;

// Convert RPM to delay between steps (microseconds)
unsigned long rpmToStepInterval(float rpm) {
    float steps_per_min = rpm * stepsPerRev * microstepping;
    float steps_per_sec = steps_per_min / 60.0;
    if (steps_per_sec == 0) return 0;
    return 1000000.0 / steps_per_sec;
}

// Callback for RPM message
void rpm_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    target_rpm = msg->linear.x;

    Serial.print("Received RPM: ");
    Serial.println(target_rpm );

    digitalWrite(DIR_PIN, target_rpm >= 0 ? HIGH : LOW);

    step_interval_us = rpmToStepInterval(abs(target_rpm));
}


void setup() {
    // Setup serial for micro-ROS
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    
    digitalWrite(ENABLE_PIN, LOW); 
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    // micro-ROS setup
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "stepper_node", "", &support);

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nana/cmd_arm/rpm"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &rpm_msg, &rpm_callback, ON_NEW_DATA);
}

void loop() {
    // Keep spin time short (non-blocking)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));  // max 1 ms

    // Stepper logic
    if (step_interval_us > 0) {
        unsigned long now = micros();
        if (now - last_step_time >= step_interval_us) {
            last_step_time = now;
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(2);
            digitalWrite(STEP_PIN, LOW);
        }
    }
}
