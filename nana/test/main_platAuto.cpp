#include <Arduino.h>
#include <ESP32Servo.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

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

const int STEP_DELAY_US = 1500;

// ================= microROS =================
rcl_subscription_t sub_plant;
rcl_subscription_t sub_plant2;
rcl_publisher_t publisher;

std_msgs__msg__Bool plant_msg;
std_msgs__msg__Bool plant2_msg;
std_msgs__msg__Bool finish_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool start_sequence = false;
bool sequence_running = false;

// ================= Stepper =================
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
void plant_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data && !sequence_running)
  {
    Serial.println("Plant command received");
    start_sequence = true;
  }
}

void plant2_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data && !sequence_running)
  {
    Serial.println("Plant2 command received");
    start_sequence = true;
  }
}

// ================= Setup =================
void setup()
{
  Serial.begin(115200);
  delay(2000);

  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "plant_node", "", &support);

  // subscribe /plant
  rclc_subscription_init_default(
    &sub_plant,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/plant");

  // subscribe /plant2
  rclc_subscription_init_default(
    &sub_plant2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/plant2");

  // publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/Finishplant");

  // executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &sub_plant,
    &plant_msg,
    &plant_callback,
    ON_NEW_DATA);

  rclc_executor_add_subscription(
    &executor,
    &sub_plant2,
    &plant2_msg,
    &plant2_callback,
    ON_NEW_DATA);

  finish_msg.data = false;

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
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  if (start_sequence)
  {
    start_sequence = false;
    sequence_running = true;

    Serial.println("Start planting sequence");

    stepper_run(HIGH, 800);
    stepper_run(LOW, 300);

    servo1.write(120);
    delay(1000);

    servo1.write(100);
    delay(1000);

    stepper_run(HIGH, 300);

    servo1.write(175);
    delay(1000);

    servo1.write(110);
    delay(1000);

    stepper_run(LOW, 800);

    servo1.write(180);
    delay(1000);

    servo2.write(120);
    delay(1000);

    servo2.write(70);
    delay(1000);

    servo2.write(120);
    delay(1000);

    servo2.write(70);
    delay(1000);

    finish_msg.data = true;
    rcl_publish(&publisher, &finish_msg, NULL);

    Serial.println("Plant sequence finished");

    sequence_running = false;
  }
}