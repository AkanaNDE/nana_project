#include <Arduino.h>
#include <ESP32Servo.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>


#include <std_msgs/msg/bool.h>

// ================= Servo =================
Servo servo1;
Servo servo2;

const int servo1_pin = 27;
const int servo2_pin = 26;

// ================= Stepper =================
const int STEP_PIN = 22;
const int DIR_PIN = 23;
const int ENABLE_PIN = 21;

const int STEP_DELAY_US = 800;

// ================= microROS =================
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

std_msgs__msg__Bool plant_msg;
std_msgs__msg__Bool finish_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool start_sequence = false;

// ================= Stepper Function =================
void stepper_run(bool dir, int duration_ms)
{
  digitalWrite(DIR_PIN, dir);

  unsigned long start = millis();

  while (millis() - start < duration_ms)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// ================= Callback =================
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data == true)
  {
    Serial.println("Plant command received");
    start_sequence = true;
  }
}

// ================= Setup =================
void setup()
{
  Serial.begin(115200);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "plant_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/plant");

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/Finishplant");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &plant_msg,
    &subscription_callback,
    ON_NEW_DATA);

  // Servo
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);

  // Stepper
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  Serial.println("Plant system ready");
}

// ================= Loop =================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (start_sequence)
  {
    start_sequence = false;

    Serial.println("Start planting sequence");

    // 1 stepper ทวน 2 วิ
    stepper_run(HIGH, 800);

    // 2 stepper ตาม 1 วิ
    stepper_run(LOW, 300);

    // 3 servo1 120
    servo1.write(120);
    delay(1000);

    // 4 servo1 100
    servo1.write(100);
    delay(1000);

    // 5 stepper ทวน 1 วิ
    stepper_run(HIGH, 300);

    // 6 servo1 160
    servo1.write(175);
    delay(1000);

    // 7 servo1 110
    servo1.write(110);
    delay(1000);

    // 8 stepper ทวน 2 วิ
    stepper_run(LOW, 800);

    servo1.write(180);
    delay(1000);

    // 9 servo2 120
    servo2.write(120);
    delay(1000);

    // 10 servo2 70
    servo2.write(70);
    delay(1000);

    // 11 servo2 120
    servo2.write(120);
    delay(1000);

    // 12 servo2 70
    servo2.write(70);
    delay(1000);

    // publish finish
    finish_msg.data = true;
    rcl_publish(&publisher, &finish_msg, NULL);

    Serial.println("Plant sequence finished");
  }
}