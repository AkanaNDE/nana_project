#include <Arduino.h>

// ===== micro-ROS =====
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

//////////////////////
// L298N FRONT
//////////////////////
const int M1_IN1=25, M1_IN2=26, M1_EN=32;
const int M2_IN1=27, M2_IN2=14, M2_EN=33;

//////////////////////
// L298N REAR
//////////////////////
const int M3_IN1=12, M3_IN2=13, M3_EN=21;
const int M4_IN1=22, M4_IN2=23, M4_EN=5;

//////////////////////
// PWM
//////////////////////
const int PWM_FREQ=20000;
const int PWM_RES=10;
const int CH[4]={0,1,2,3};

//////////////////////
// Control Params (สำคัญ)
//////////////////////
const int MIN_PWM = 650;          // ✅ ต่ำกว่านี้มักไม่เริ่มหมุน (ปรับได้ 550-750)
const int MAX_PWM = 900;          // ✅ อย่าให้สูงเกินไปถ้ารถแรงเกิน
const uint32_t CMD_TIMEOUT_MS = 2000;

// deadband กันสั่น
const float LIN_DB = 0.02f;
const float ANG_DB = 0.02f;

// debug ไป ROS
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
static char ack_buf[160];

//////////////////////
// State
//////////////////////
static float cmd_x=0.0f, cmd_y=0.0f, cmd_z=0.0f;
static uint32_t last_cmd_ms = 0;
static bool got_first_cmd = false;
static bool micro_ros_connected = false;

static uint32_t last_debug_ms = 0;

//////////////////////////////////////////////////
// Motor functions (เหมือนโค้ดที่วิ่งได้)
//////////////////////////////////////////////////
void motorSet(int in1,int in2,int ch,int speed,bool dir){
  digitalWrite(in1,dir);
  digitalWrite(in2,!dir);
  speed=constrain(speed,0,1023);
  ledcWrite(ch,speed);
}

void stopAll(){
  for(int i=0;i<4;i++) ledcWrite(CH[i],0);
}

void forwardAll(int pwm){
  motorSet(M1_IN1,M1_IN2,CH[0],pwm,true);
  motorSet(M2_IN1,M2_IN2,CH[1],pwm,true);
  motorSet(M3_IN1,M3_IN2,CH[2],pwm,true);
  motorSet(M4_IN1,M4_IN2,CH[3],pwm,true);
}

// ✅ SOFT TURN
void turnLeftSoft(int pwm){
  // ซ้ายหยุด
  motorSet(M1_IN1,M1_IN2,CH[0],0,true);
  motorSet(M3_IN1,M3_IN2,CH[2],0,true);
  // ขวาเดินหน้า
  motorSet(M2_IN1,M2_IN2,CH[1],pwm,true);
  motorSet(M4_IN1,M4_IN2,CH[3],pwm,true);
}

void turnRightSoft(int pwm){
  // ขวาหยุด
  motorSet(M2_IN1,M2_IN2,CH[1],0,true);
  motorSet(M4_IN1,M4_IN2,CH[3],0,true);
  // ซ้ายเดินหน้า
  motorSet(M1_IN1,M1_IN2,CH[0],pwm,true);
  motorSet(M3_IN1,M3_IN2,CH[2],pwm,true);
}

// เดิน+เลี้ยวพร้อมกัน (ปรับสปีดซ้ายขวา)
void forwardWithTurn(int base_pwm, float turn_sign, int turn_pwm){
  int delta = (int)(0.6f * turn_pwm);
  int left_pwm  = base_pwm;
  int right_pwm = base_pwm;

  if (turn_sign > 0) { // left
    left_pwm  = max(0, base_pwm - delta);
    right_pwm = min(1023, base_pwm + delta);
  } else {             // right
    right_pwm = max(0, base_pwm - delta);
    left_pwm  = min(1023, base_pwm + delta);
  }

  motorSet(M1_IN1,M1_IN2,CH[0],left_pwm,true);
  motorSet(M3_IN1,M3_IN2,CH[2],left_pwm,true);

  motorSet(M2_IN1,M2_IN2,CH[1],right_pwm,true);
  motorSet(M4_IN1,M4_IN2,CH[3],right_pwm,true);
}

//////////////////////////////////////////////////
// micro-ROS helpers
//////////////////////////////////////////////////
void wait_for_agent(){
  while(!micro_ros_connected){
    micro_ros_connected = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
    delay(300);
  }
}

void publish_ack(const char* text){
  snprintf(ack_buf, sizeof(ack_buf), "%s", text);
  ack_msg.data.size = strlen(ack_buf);
  rcl_publish(&ack_publisher, &ack_msg, NULL);
}

//////////////////////////////////////////////////
// PWM mapping (แก้หลัก!)
//////////////////////////////////////////////////
int map_cmd_to_pwm(float u){
  // u = abs(cmd) (0..1)
  u = fabs(u);
  if (u < 0.0001f) return 0;

  if (u > 1.0f) u = 1.0f;

  // ✅ ให้มีแรงเริ่มหมุนเสมอ: MIN_PWM .. MAX_PWM
  int pwm = (int)(MIN_PWM + u * (MAX_PWM - MIN_PWM));
  pwm = constrain(pwm, 0, 1023);
  return pwm;
}

//////////////////////////////////////////////////
// Convert Twist -> Motor
//////////////////////////////////////////////////
void driveFromTwist(float x, float y, float z){
  // ใช้ y เป็นหลัก (ตามระบบเดิมของคุณ) ถ้า y=0 ค่อยใช้ x
  float forward = (fabs(y) > 0.0001f) ? y : x;

  // deadband กันสั่น
  if (fabs(forward) < LIN_DB) forward = 0.0f;
  if (fabs(z) < ANG_DB) z = 0.0f;

  int base_pwm = map_cmd_to_pwm(forward);
  int turn_pwm = map_cmd_to_pwm(z);

  // debug (throttle)
  uint32_t now = millis();
  if (now - last_debug_ms > DEBUG_PERIOD_MS) {
    last_debug_ms = now;
    char tmp[160];
    snprintf(tmp, sizeof(tmp),
             "cmd x=%.2f y=%.2f z=%.2f | f=%.2f base_pwm=%d turn_pwm=%d",
             x,y,z,forward,base_pwm,turn_pwm);
    publish_ack(tmp);
  }

  // stop
  if (forward == 0.0f && z == 0.0f) {
    stopAll();
    return;
  }

  // turn only
  if (forward == 0.0f && z != 0.0f) {
    if (z > 0) turnLeftSoft(turn_pwm);
    else turnRightSoft(turn_pwm);
    return;
  }

  // forward only (รองรับเดินหน้าเท่านั้น)
  if (forward > 0.0f && z == 0.0f) {
    forwardAll(base_pwm);
    return;
  }

  // forward + turn
  if (forward > 0.0f && z != 0.0f) {
    forwardWithTurn(base_pwm, (z > 0) ? +1.0f : -1.0f, turn_pwm);
    return;
  }

  // ไม่รองรับถอย
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

  char tmp[160];
  snprintf(tmp, sizeof(tmp), "RX cmd x=%.2f y=%.2f z=%.2f", cmd_x, cmd_y, cmd_z);
  publish_ack(tmp);
}

//////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////
void setup(){
  Serial.begin(115200);
  delay(800);

  // motor pins
  pinMode(M1_IN1,OUTPUT); pinMode(M1_IN2,OUTPUT);
  pinMode(M2_IN1,OUTPUT); pinMode(M2_IN2,OUTPUT);
  pinMode(M3_IN1,OUTPUT); pinMode(M3_IN2,OUTPUT);
  pinMode(M4_IN1,OUTPUT); pinMode(M4_IN2,OUTPUT);

  for(int i=0;i<4;i++){
    ledcSetup(CH[i],PWM_FREQ,PWM_RES);
  }
  ledcAttachPin(M1_EN,CH[0]);
  ledcAttachPin(M2_EN,CH[1]);
  ledcAttachPin(M3_EN,CH[2]);
  ledcAttachPin(M4_EN,CH[3]);

  stopAll();

  // micro-ROS
  set_microros_serial_transports(Serial);
  wait_for_agent();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  // sub /nana/cmd_arm
  cmd_msg_in.linear.x = cmd_msg_in.linear.y = cmd_msg_in.linear.z = 0.0;
  cmd_msg_in.angular.x = cmd_msg_in.angular.y = cmd_msg_in.angular.z = 0.0;

  rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/nana/cmd_arm"
  );

  // pub /esp_rx
  rosidl_runtime_c__String__init(&ack_msg.data);
  ack_msg.data.data = ack_buf;
  ack_msg.data.capacity = sizeof(ack_buf);
  ack_msg.data.size = 0;

  rclc_publisher_init_default(
    &ack_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/esp_rx"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &cmd_subscriber,
    &cmd_msg_in,
    &cmd_callback,
    ON_NEW_DATA
  );

  last_cmd_ms = millis();
  publish_ack("ESP READY (MIN_PWM enabled)");
}

//////////////////////////////////////////////////
// Loop
//////////////////////////////////////////////////
void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

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