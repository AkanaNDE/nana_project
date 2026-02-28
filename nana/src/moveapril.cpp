#include <Arduino.h>

// ===== micro-ROS =====
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <rosidl_runtime_c/string_functions.h>

//////////////////////
// L298N FRONT
//////////////////////
const int M1_IN1 = 25, M1_IN2 = 26, M1_EN = 32;
const int M2_IN1 = 27, M2_IN2 = 14, M2_EN = 33;

//////////////////////
// L298N REAR
//////////////////////
const int M3_IN1 = 12, M3_IN2 = 13, M3_EN = 21;
const int M4_IN1 = 22, M4_IN2 = 23, M4_EN = 5;

//////////////////////
// PWM
//////////////////////
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;        // 0..1023
const int CH[4]    = {0, 1, 2, 3};

//////////////////////
// Behavior params
//////////////////////
const float    FINISH_DISTANCE_CM = 40.0f;

// ✅ ทำให้ไม่ stale ง่าย
const uint32_t DIST_TIMEOUT_MS = 1000;
const uint32_t POS_TIMEOUT_MS  = 1000;

// ✅ คุมความถี่ loop / debug
const uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz
const uint32_t DEBUG_PERIOD_MS   = 200;  // 5 Hz

//////////////////////
// micro-ROS objects
//////////////////////
rcl_subscription_t pos_subscriber;
rcl_subscription_t dist_subscriber;

rcl_publisher_t debug_publisher;
rcl_publisher_t stage_publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

std_msgs__msg__String  pos_msg_in;
std_msgs__msg__Float32 dist_msg_in;

std_msgs__msg__String debug_msg;
std_msgs__msg__String stage_msg;

//////////////////////
// State + Buffers
//////////////////////
static char  current_position[20] = "NOT_FOUND";
static float current_distance_cm  = 9999.0f;

static uint32_t last_pos_ms  = 0;
static uint32_t last_dist_ms = 0;

static bool finish_reached = false;
static bool finish_sent    = false;

static char pos_sub_buffer[64];
static char debug_buffer[96];
static char stage_buffer[32];

static bool micro_ros_connected = false;

static uint32_t last_control_ms = 0;
static uint32_t last_debug_ms   = 0;

//////////////////////////////////////////////////
// Motor functions
//////////////////////////////////////////////////
void motorSet(int in1, int in2, int ch, int speed, bool dir)
{
  digitalWrite(in1, dir);
  digitalWrite(in2, !dir);
  speed = constrain(speed, 0, 1023);
  ledcWrite(ch, speed);
}

void stopAll()
{
  for (int i = 0; i < 4; i++) ledcWrite(CH[i], 0);
}

void moveForward()
{
  motorSet(M1_IN1, M1_IN2, CH[0], 700, true);
  motorSet(M2_IN1, M2_IN2, CH[1], 700, true);
  motorSet(M3_IN1, M3_IN2, CH[2], 700, true);
  motorSet(M4_IN1, M4_IN2, CH[3], 700, true);
}

void turnLeft()
{
  motorSet(M1_IN1, M1_IN2, CH[0], 600, false);
  motorSet(M2_IN1, M2_IN2, CH[1], 600, true);
  motorSet(M3_IN1, M3_IN2, CH[2], 600, false);
  motorSet(M4_IN1, M4_IN2, CH[3], 600, true);
}

void turnRight()
{
  motorSet(M1_IN1, M1_IN2, CH[0], 600, true);
  motorSet(M2_IN1, M2_IN2, CH[1], 600, false);
  motorSet(M3_IN1, M3_IN2, CH[2], 600, true);
  motorSet(M4_IN1, M4_IN2, CH[3], 600, false);
}

//////////////////////////////////////////////////
// ROS Callbacks
//////////////////////////////////////////////////
void position_callback(const void * msgin)
{
  const std_msgs__msg__String * incoming =
      (const std_msgs__msg__String *)msgin;

  last_pos_ms = millis();

  size_t n = incoming->data.size;
  if (n >= sizeof(current_position)) n = sizeof(current_position) - 1;

  memcpy(current_position, incoming->data.data, n);
  current_position[n] = '\0';
}

void distance_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * incoming =
      (const std_msgs__msg__Float32 *)msgin;

  current_distance_cm = incoming->data;
  last_dist_ms = millis();

  if (!finish_reached &&
      current_distance_cm > 0.0f &&
      current_distance_cm < FINISH_DISTANCE_CM) {
    finish_reached = true;
  }
}

//////////////////////////////////////////////////
// micro-ROS Agent wait
//////////////////////////////////////////////////
void wait_for_agent()
{
  while (!micro_ros_connected) {
    micro_ros_connected = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
    delay(300);
  }
}

//////////////////////////////////////////////////
// Publish helpers
//////////////////////////////////////////////////
void publish_stage_finish_once()
{
  if (finish_sent) return;

  snprintf(stage_buffer, sizeof(stage_buffer), "FINISH");
  stage_msg.data.size = strlen(stage_buffer);
  rcl_publish(&stage_publisher, &stage_msg, NULL);

  finish_sent = true;
}

void publish_debug_throttled(const char* state)
{
  uint32_t now = millis();
  if (now - last_debug_ms < DEBUG_PERIOD_MS) return;
  last_debug_ms = now;

  bool pos_stale  = (now - last_pos_ms)  > POS_TIMEOUT_MS;
  bool dist_stale = (now - last_dist_ms) > DIST_TIMEOUT_MS;
  bool dist_invalid = (current_distance_cm <= 0.0f);

  snprintf(debug_buffer, sizeof(debug_buffer),
           "state=%s pos=%s dist=%.1f pos_stale=%d dist_stale=%d dist_invalid=%d",
           state, current_position, current_distance_cm,
           (int)pos_stale, (int)dist_stale, (int)dist_invalid);

  debug_msg.data.size = strlen(debug_buffer);
  rcl_publish(&debug_publisher, &debug_msg, NULL);
}

//////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  delay(1200);

  set_microros_serial_transports(Serial);
  wait_for_agent();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // init incoming position string with fixed buffer
  rosidl_runtime_c__String__init(&pos_msg_in.data);
  pos_msg_in.data.data = pos_sub_buffer;
  pos_msg_in.data.capacity = sizeof(pos_sub_buffer);
  pos_msg_in.data.size = 0;

  // float incoming
  dist_msg_in.data = 0.0f;

  // init debug msg fixed buffer
  rosidl_runtime_c__String__init(&debug_msg.data);
  debug_msg.data.data = debug_buffer;
  debug_msg.data.capacity = sizeof(debug_buffer);
  debug_msg.data.size = 0;

  // init stage msg fixed buffer
  rosidl_runtime_c__String__init(&stage_msg.data);
  stage_msg.data.data = stage_buffer;
  stage_msg.data.capacity = sizeof(stage_buffer);
  stage_msg.data.size = 0;

  // subs
  rclc_subscription_init_default(
    &pos_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/apriltag_position"
  );

  rclc_subscription_init_default(
    &dist_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/apriltag_distance"
  );

  // pubs
  rclc_publisher_init_default(
    &debug_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/esp_debug"
  );

  rclc_publisher_init_default(
    &stage_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/apriltag_stage"
  );

  // executor (2 subs)
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  rclc_executor_add_subscription(
    &executor, &pos_subscriber, &pos_msg_in,
    &position_callback, ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor, &dist_subscriber, &dist_msg_in,
    &distance_callback, ON_NEW_DATA
  );

  // motor pins
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);

  for (int i = 0; i < 4; i++) ledcSetup(CH[i], PWM_FREQ, PWM_RES);

  ledcAttachPin(M1_EN, CH[0]);
  ledcAttachPin(M2_EN, CH[1]);
  ledcAttachPin(M3_EN, CH[2]);
  ledcAttachPin(M4_EN, CH[3]);

  stopAll();

  uint32_t now = millis();
  last_pos_ms = now;
  last_dist_ms = now;
  last_control_ms = now;
  last_debug_ms = now;
}

//////////////////////////////////////////////////
// Loop
//////////////////////////////////////////////////
void loop()
{
  // spin บ่อย ๆ เพื่อไม่ให้ stale
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  uint32_t now = millis();
  if (now - last_control_ms < CONTROL_PERIOD_MS) {
    // อย่าหน่วงนาน ให้ loop ไหล
    publish_debug_throttled(finish_reached ? "FINISH" : "IDLE");
    return;
  }
  last_control_ms = now;

  bool pos_stale   = (now - last_pos_ms)  > POS_TIMEOUT_MS;
  bool dist_stale  = (now - last_dist_ms) > DIST_TIMEOUT_MS;
  bool dist_invalid = (current_distance_cm <= 0.0f);

  // FINISH
  if (finish_reached) {
    stopAll();
    publish_stage_finish_once();
    publish_debug_throttled("FINISH");
    return;
  }

  // Safety: ถ้า distance หาย/invalid หรือ position หาย -> หยุด
  if (dist_stale || dist_invalid || pos_stale) {
    stopAll();
    publish_debug_throttled(dist_stale ? "STOP_STALE_DIST" : (dist_invalid ? "STOP_INVALID_DIST" : "STOP_STALE_POS"));
    return;
  }

  // Normal move
  if (strcmp(current_position, "CENTER") == 0) moveForward();
  else if (strcmp(current_position, "LEFT") == 0) turnLeft();
  else if (strcmp(current_position, "RIGHT") == 0) turnRight();
  else stopAll();

  publish_debug_throttled("RUN");
}