#include <Arduino.h>

// ===== micro-ROS =====
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

//////////////////////
// MDD3A Pins
//////////////////////
const int M1_A = 25;
const int M1_B = 26;

const int M2_A = 27;
const int M2_B = 14;

const int M3_A = 12;
const int M3_B = 13;

const int M4_A = 22;
const int M4_B = 23;

//////////////////////
// PWM (ESP32 LEDC)
//////////////////////
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;   // 0..1023

const int CH_M1A = 0, CH_M1B = 1;
const int CH_M2A = 2, CH_M2B = 3;
const int CH_M3A = 4, CH_M3B = 5;
const int CH_M4A = 6, CH_M4B = 7;

//////////////////////
// Control Params
//////////////////////
const int MIN_PWM = 250;
const int MAX_PWM = 400;
const uint32_t CMD_TIMEOUT_MS = 2000;

const float LIN_DB = 0.02f;
const float ANG_DB = 0.02f;

const uint32_t DEBUG_PERIOD_MS = 200;

//////////////////////
// micro-ROS objects
//////////////////////
rcl_subscription_t cmd_subscriber;
rcl_publisher_t ack_publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

geometry_msgs__msg__Twist cmd_msg_in;
std_msgs__msg__String ack_msg;
static char ack_buf[200];

//////////////////////
// State
//////////////////////
static float cmd_x = 0.0f, cmd_y = 0.0f, cmd_z = 0.0f;
static uint32_t last_cmd_ms = 0;
static bool got_first_cmd = false;
static bool micro_ros_connected = false;
static uint32_t last_debug_ms = 0;

void stopAll();

//////////////////////////////////////////////////
// Error handling
//////////////////////////////////////////////////
void error_loop()
{
  stopAll();
  while (1) {
    delay(100);
  }
}

#define RCCHECK(fn)                          \
  {                                          \
    rcl_ret_t temp_rc = fn;                  \
    if ((temp_rc != RCL_RET_OK)) {           \
      error_loop();                          \
    }                                        \
  }

#define RCSOFTCHECK(fn)                      \
  {                                          \
    rcl_ret_t temp_rc = fn;                  \
    (void)temp_rc;                           \
  }

//////////////////////////////////////////////////
// micro-ROS helpers
//////////////////////////////////////////////////
void wait_for_agent() {
  while (!micro_ros_connected) {
    micro_ros_connected = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
    delay(300);
  }
}

void publish_ack(const char* text) {
  if (text == nullptr) return;

  snprintf(ack_buf, sizeof(ack_buf), "%s", text);
  ack_buf[sizeof(ack_buf) - 1] = '\0';
  ack_msg.data.size = strlen(ack_buf);

  RCSOFTCHECK(rcl_publish(&ack_publisher, &ack_msg, NULL));
}

//////////////////////////////////////////////////
// PWM mapping
//////////////////////////////////////////////////
int map_cmd_to_pwm(float u) {
  u = fabs(u);
  if (u < 0.0001f) return 0;
  if (u > 1.0f) u = 1.0f;

  int pwm = (int)(MIN_PWM + u * (MAX_PWM - MIN_PWM));
  return constrain(pwm, 0, 1023);
}

//////////////////////////////////////////////////
// Motor control
//////////////////////////////////////////////////
void mddMotorSet(int pinA, int pinB, int chA, int chB, int speed, bool forward) {
  (void)pinA;
  (void)pinB;

  speed = constrain(speed, 0, 1023);

  if (speed == 0) {
    ledcWrite(chA, 0);
    ledcWrite(chB, 0);
    return;
  }

  if (forward) {
    ledcWrite(chA, speed);
    ledcWrite(chB, 0);
  } else {
    ledcWrite(chA, 0);
    ledcWrite(chB, speed);
  }
}

void stopAll() {
  ledcWrite(CH_M1A, 0); ledcWrite(CH_M1B, 0);
  ledcWrite(CH_M2A, 0); ledcWrite(CH_M2B, 0);
  ledcWrite(CH_M3A, 0); ledcWrite(CH_M3B, 0);
  ledcWrite(CH_M4A, 0); ledcWrite(CH_M4B, 0);
}

void driveLeftSide(int pwm, bool forward) {
  mddMotorSet(M1_A, M1_B, CH_M1A, CH_M1B, pwm, forward);
  mddMotorSet(M3_A, M3_B, CH_M3A, CH_M3B, pwm, forward);
}

void driveRightSide(int pwm, bool forward) {
  mddMotorSet(M2_A, M2_B, CH_M2A, CH_M2B, pwm, forward);
  mddMotorSet(M4_A, M4_B, CH_M4A, CH_M4B, pwm, forward);
}

void driveAll(int pwm, bool forward) {
  driveLeftSide(pwm, forward);
  driveRightSide(pwm, forward);
}

void turnLeftSoft(int pwm) {
  driveLeftSide(pwm, false);
  driveRightSide(pwm, true);
}

void turnRightSoft(int pwm) {
  driveRightSide(pwm, false);
  driveLeftSide(pwm, true);
}

void driveWithTurn(int base_pwm, float turn_sign, int turn_pwm, bool forward_dir) {
  int delta = (int)(0.6f * turn_pwm);
  int left_pwm  = base_pwm;
  int right_pwm = base_pwm;

  if (turn_sign > 0) {
    left_pwm  = max(0, base_pwm - delta);
    right_pwm = min(1023, base_pwm + delta);
  } else {
    right_pwm = max(0, base_pwm - delta);
    left_pwm  = min(1023, base_pwm + delta);
  }

  driveLeftSide(left_pwm, forward_dir);
  driveRightSide(right_pwm, forward_dir);
}

//////////////////////////////////////////////////
// Convert Twist -> Motor
//////////////////////////////////////////////////
void driveFromTwist(float x, float y, float z) {
  float forward = (fabs(y) > 0.0001f) ? y : x;

  if (fabs(forward) < LIN_DB) forward = 0.0f;
  if (fabs(z) < ANG_DB) z = 0.0f;

  int base_pwm = map_cmd_to_pwm(forward);
  int turn_pwm = map_cmd_to_pwm(z);

  uint32_t now = millis();
  if (now - last_debug_ms > DEBUG_PERIOD_MS) {
    last_debug_ms = now;
    char tmp[200];
    snprintf(tmp, sizeof(tmp),
             "cmd x=%.2f y=%.2f z=%.2f | f=%.2f base=%d turn=%d",
             x, y, z, forward, base_pwm, turn_pwm);
    publish_ack(tmp);
  }

  if (forward == 0.0f && z == 0.0f) {
    stopAll();
    return;
  }

  if (forward == 0.0f && z != 0.0f) {
    if (z > 0) turnLeftSoft(turn_pwm);
    else       turnRightSoft(turn_pwm);
    return;
  }

  if (forward > 0.0f && z == 0.0f) {
    driveAll(base_pwm, true);
    return;
  }

  if (forward < 0.0f && z == 0.0f) {
    driveAll(base_pwm, false);
    return;
  }

  if (forward > 0.0f && z != 0.0f) {
    driveWithTurn(base_pwm, (z > 0) ? +1.0f : -1.0f, turn_pwm, true);
    return;
  }

  if (forward < 0.0f && z != 0.0f) {
    driveWithTurn(base_pwm, (z > 0) ? +1.0f : -1.0f, turn_pwm, false);
    return;
  }

  stopAll();
}

//////////////////////////////////////////////////
// Callback
//////////////////////////////////////////////////
void cmd_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * m =
      (const geometry_msgs__msg__Twist *)msgin;

  cmd_x = (float)m->linear.x;
  cmd_y = (float)m->linear.y;
  cmd_z = (float)m->angular.z;

  last_cmd_ms = millis();
  got_first_cmd = true;

  char tmp[200];
  snprintf(tmp, sizeof(tmp), "RX cmd x=%.2f y=%.2f z=%.2f", cmd_x, cmd_y, cmd_z);
  publish_ack(tmp);
}

//////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(M1_A, OUTPUT); pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT); pinMode(M2_B, OUTPUT);
  pinMode(M3_A, OUTPUT); pinMode(M3_B, OUTPUT);
  pinMode(M4_A, OUTPUT); pinMode(M4_B, OUTPUT);

  ledcSetup(CH_M1A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M1B, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M2A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M2B, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M3A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M3B, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M4A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_M4B, PWM_FREQ, PWM_RES);

  ledcAttachPin(M1_A, CH_M1A); ledcAttachPin(M1_B, CH_M1B);
  ledcAttachPin(M2_A, CH_M2A); ledcAttachPin(M2_B, CH_M2B);
  ledcAttachPin(M3_A, CH_M3A); ledcAttachPin(M3_B, CH_M3B);
  ledcAttachPin(M4_A, CH_M4A); ledcAttachPin(M4_B, CH_M4B);

  stopAll();

  allocator = rcl_get_default_allocator();
  node = rcl_get_zero_initialized_node();
  cmd_subscriber = rcl_get_zero_initialized_subscription();
  ack_publisher = rcl_get_zero_initialized_publisher();
  executor = rclc_executor_get_zero_initialized_executor();

  set_microros_serial_transports(Serial);
  delay(200);

  wait_for_agent();
  delay(200);

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));

  cmd_msg_in.linear.x = 0.0;
  cmd_msg_in.linear.y = 0.0;
  cmd_msg_in.linear.z = 0.0;
  cmd_msg_in.angular.x = 0.0;
  cmd_msg_in.angular.y = 0.0;
  cmd_msg_in.angular.z = 0.0;

  RCCHECK(
    rclc_subscription_init_default(
      &cmd_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/nana/cmd_arm"
    )
  );

  ack_msg.data.data = ack_buf;
  ack_msg.data.capacity = sizeof(ack_buf);
  ack_msg.data.size = 0;
  ack_buf[0] = '\0';

  RCCHECK(
    rclc_publisher_init_default(
      &ack_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/esp_rx"
    )
  );

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(
    rclc_executor_add_subscription(
      &executor,
      &cmd_subscriber,
      &cmd_msg_in,
      &cmd_callback,
      ON_NEW_DATA
    )
  );

  last_cmd_ms = millis();
  publish_ack("ESP READY (MDD3A mode, backward enabled)");
}

//////////////////////////////////////////////////
// Loop
//////////////////////////////////////////////////
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  uint32_t now = millis();

  if (!got_first_cmd) {
    stopAll();
    return;
  }

  if ((now - last_cmd_ms) > CMD_TIMEOUT_MS) {
    stopAll();
    publish_ack("CMD TIMEOUT -> STOP");
    return;
  }

  driveFromTwist(cmd_x, cmd_y, cmd_z);
}