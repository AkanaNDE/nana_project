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

#include <geometry_msgs/msg/twist.h>
#include "driver/gpio.h"

// ========================== โค้ดที่ 1: Stepper + Servo ==========================

// ===== Macros =====
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) rclErrorLoop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t t_ms = -1; if (t_ms == -1) t_ms = uxr_millis(); if ((int32_t)(uxr_millis() - t_ms) > (MS)) { X; t_ms = uxr_millis(); } } while (0)

// ===== Pins (ปรับให้ไม่ชนกับ DC) =====
#define STEP_PIN1 22
#define DIR_PIN1  13
#define STEP_PIN2 21   // เดิม 25 -> ย้ายเพื่อไม่ชน DC AIN1
#define DIR_PIN2  16

// ===== Stepper settings =====
static const int   STEPS_PER_REV = 200;
static const int   MICROSTEP     = 16;
static const float MAX_ABS_RPM   = 80.0f;

static const uint32_t MIN_STEP_INTERVAL_US = 1800;
static const uint32_t STEP_PULSE_US        = 80;
static const uint32_t DIR_SETUP_US         = 120;

// Fixed-speed hold-to-run for Stepper #1
static const float    FIXED_RPM_1          = 6.0f;

// Stepper2 defaults
static const float    S2_DEFAULT_DEGREES   = 2.0f;
static const uint32_t S2_DEFAULT_INTERVAL  = 3000;
// --- One-shot press mode for Stepper #2 ---
static volatile bool s2_prev_pressed = false;
static uint32_t s2_last_trigger_ms = 0;
static const uint32_t S2_DEBOUNCE_MS = 120;

// Timing / Agent
static const uint32_t CONTROL_PERIOD_MS    = 20;
static const uint32_t COMMAND_KEEPALIVE_MS = 3000;
static const uint32_t AGENT_DISCONNECT_GRACE_MS = 30000;

// ===== ROS entities (จากโค้ดที่ 1) =====
rcl_publisher_t debug_motor_publisher, maho_debug_publisher, grip_debug_publisher;
geometry_msgs__msg__Twist debug_motor_msg, maho_debug_msg, grip_debug_msg;

rcl_subscription_t motor_subscriber, maho_subscriber, grip_subscriber;
geometry_msgs__msg__Twist motor_msg, maho_msg, grip_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;
rcl_init_options_t init_options;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;
static uint32_t last_agent_ok_ms = 0;

// ===== Utils =====
static inline float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }

// ===== HW timer (Stepper #1) =====
hw_timer_t *stepper_timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool     is_running=false, step_high_phase=false, dir_is_cw=true, need_dir_setup_gap=false;
volatile uint32_t current_interval_us=1000, target_interval_us=1000, pulse_us=STEP_PULSE_US;

// โหมด hold-to-run
static volatile bool  s1_held = false;       // กด/ถืออยู่ไหม
static volatile int   s1_dir  = +1;          // +1/-1
static volatile uint32_t last_cmd_ms = 0;

// ===== Timer ISR (Stepper #1) =====
void IRAM_ATTR onStepTimer(){
  if(!is_running){ gpio_set_level((gpio_num_t)STEP_PIN1,0); timerAlarmWrite(stepper_timer,1000,true); return; }

  if(!step_high_phase){
    gpio_set_level((gpio_num_t)DIR_PIN1, dir_is_cw?1:0);
    if(need_dir_setup_gap){ need_dir_setup_gap=false; timerAlarmWrite(stepper_timer, DIR_SETUP_US, true); return; }
    gpio_set_level((gpio_num_t)STEP_PIN1, 1);
    step_high_phase=true;
    timerAlarmWrite(stepper_timer, pulse_us, true);
  }else{
    gpio_set_level((gpio_num_t)STEP_PIN1, 0);
    step_high_phase=false;
    if(current_interval_us != target_interval_us){
      current_interval_us += (current_interval_us < target_interval_us) ? 1 : -1;
      if(current_interval_us < MIN_STEP_INTERVAL_US) current_interval_us = MIN_STEP_INTERVAL_US;
    }
    uint32_t low_us = (current_interval_us > pulse_us) ? (current_interval_us - pulse_us) : MIN_STEP_INTERVAL_US;
    timerAlarmWrite(stepper_timer, low_us, true);
  }
}

// ===== Speed/Direction logic (Stepper #1) =====
void rclErrorLoop(){ while(1){ delay(150);} }

static inline void applyRPM(float rpm){
  if(rpm==0.0f){
    portENTER_CRITICAL(&timerMux);
    is_running=false; step_high_phase=false;
    target_interval_us = current_interval_us = 1000;
    portEXIT_CRITICAL(&timerMux);
    timerAlarmWrite(stepper_timer, 1000, true);
    debug_motor_msg.linear.x=0.0; debug_motor_msg.linear.y=0.0;
    return;
  }
  rpm = clampf(rpm, -MAX_ABS_RPM, +MAX_ABS_RPM);
  bool cw = (rpm>0.0f);
  portENTER_CRITICAL(&timerMux);
  if(dir_is_cw != cw){ dir_is_cw=cw; need_dir_setup_gap=true; }
  portEXIT_CRITICAL(&timerMux);

  float steps_per_rev = (float)STEPS_PER_REV * (float)MICROSTEP;
  float sps = fabsf(rpm) * steps_per_rev / 60.0f;
  uint32_t interval_us = (uint32_t)(1000000.0f / (sps>1.0f ? sps : 1.0f));
  if(interval_us < MIN_STEP_INTERVAL_US) interval_us = MIN_STEP_INTERVAL_US;

  portENTER_CRITICAL(&timerMux);
  target_interval_us = interval_us;
  is_running = true;
  portEXIT_CRITICAL(&timerMux);

  debug_motor_msg.linear.x = rpm;
  debug_motor_msg.linear.y = sps;
}

// ===== Stepper #2 =====
rcl_publisher_t maho_debug_dummy_pub; // not used; keep interface

hw_timer_t *stepper2_timer = nullptr;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
volatile bool s2_running=false, s2_step_high_phase=false, s2_dir_is_cw=true, s2_need_dir_gap=false, s2_just_finished=false;
volatile uint32_t s2_interval_us = S2_DEFAULT_INTERVAL, s2_pulse_us = STEP_PULSE_US;
volatile int32_t  s2_steps_remaining=0, s2_total_steps_cmd=0;
volatile float    s2_degrees_cmd=S2_DEFAULT_DEGREES;

static inline void publishMahoDebug(int dir, int steps, float degrees, bool done){
  maho_debug_msg.linear.x=dir; maho_debug_msg.linear.y=steps; maho_debug_msg.linear.z=degrees;
  maho_debug_msg.angular.z = done ? 1.0f : 0.0f;
  RCSOFTCHECK(rcl_publish(&maho_debug_publisher,&maho_debug_msg,NULL));
}

void IRAM_ATTR onStep2Timer(){
  if(!s2_running){ gpio_set_level((gpio_num_t)STEP_PIN2,0); timerAlarmWrite(stepper2_timer,1000,true); return; }
  if(s2_steps_remaining<=0){ s2_running=false; s2_step_high_phase=false; gpio_set_level((gpio_num_t)STEP_PIN2,0); s2_just_finished=true; timerAlarmWrite(stepper2_timer,1000,true); return; }
  if(!s2_step_high_phase){
    gpio_set_level((gpio_num_t)DIR_PIN2, s2_dir_is_cw?1:0);
    if(s2_need_dir_gap){ s2_need_dir_gap=false; timerAlarmWrite(stepper2_timer, DIR_SETUP_US, true); return; }
    gpio_set_level((gpio_num_t)STEP_PIN2,1); s2_step_high_phase=true; timerAlarmWrite(stepper2_timer, s2_pulse_us, true);
  }else{
    gpio_set_level((gpio_num_t)STEP_PIN2,0); s2_step_high_phase=false; s2_steps_remaining--;
    uint32_t low_us = (s2_interval_us > s2_pulse_us) ? (s2_interval_us - s2_pulse_us) : s2_pulse_us;
    timerAlarmWrite(stepper2_timer, low_us, true);
  }
}

static inline void startStepper2Degrees(float deg, int direction, uint32_t interval_us_for_s2){
  if(direction==0 || deg<=0.0f) return;
  float steps_per_degree = (float)(STEPS_PER_REV * MICROSTEP) / 360.0f;
  int total_steps = (int)(steps_per_degree * deg + 0.5f);
  publishMahoDebug(direction,total_steps,deg,false);
  portENTER_CRITICAL(&timerMux2);
  s2_dir_is_cw = (direction>0); s2_need_dir_gap = true;
  s2_interval_us = (interval_us_for_s2<MIN_STEP_INTERVAL_US)?MIN_STEP_INTERVAL_US:interval_us_for_s2;
  s2_steps_remaining=total_steps; s2_total_steps_cmd=total_steps; s2_degrees_cmd=deg;
  s2_running=true; s2_step_high_phase=false;
  portEXIT_CRITICAL(&timerMux2);
}

// ===== SERVO =====
#define SERVO_PIN 27
#define SERVO_LEDC_CH 4
#define SERVO_FREQ_HZ 50
#define SERVO_RES_BITS 16
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_SLEW_DEG_PER_TICK 4
static volatile float servo_current_deg=180, servo_target_deg=180; static volatile bool servo_reached=true;
static inline uint32_t usToDuty(uint32_t us){ const uint32_t period_us=1000000UL/SERVO_FREQ_HZ, max_duty=(1UL<<SERVO_RES_BITS)-1; if(us<SERVO_MIN_US)us=SERVO_MIN_US; if(us>SERVO_MAX_US)us=SERVO_MAX_US; return (uint32_t)((uint64_t)us*max_duty/period_us); }
static inline uint32_t angleToUs(float d){ d=clampf(d,0,180); return (uint32_t)(SERVO_MIN_US + (SERVO_MAX_US-SERVO_MIN_US)*(d/180.0f) + 0.5f); }
static inline void servoWriteDeg(float d){ ledcWrite(SERVO_LEDC_CH, usToDuty(angleToUs(d))); }
static inline void servoSetTargetFromCmd(int cmd){ if(cmd>0) servo_target_deg=180; else if(cmd<0) servo_target_deg=130; else return; servo_reached=false; }
static inline void servoUpdateAndPublish(){ float c=servo_current_deg,t=servo_target_deg; if(fabsf(t-c)<=SERVO_SLEW_DEG_PER_TICK){ c=t; servo_reached=true; } else { c += (t>c?SERVO_SLEW_DEG_PER_TICK:-SERVO_SLEW_DEG_PER_TICK); } servo_current_deg=clampf(c,0,180); servoWriteDeg(servo_current_deg); grip_debug_msg.linear.x=servo_current_deg; grip_debug_msg.linear.y=servo_target_deg; grip_debug_msg.angular.z=servo_reached?1.0f:0.0f; RCSOFTCHECK(rcl_publish(&grip_debug_publisher,&grip_debug_msg,NULL)); }

// ===== Control Timer (ฝั่ง Stepper/Servo) =====
void Move(){ // โหมด hold-to-run
  if(s1_held) applyRPM((float)s1_dir * FIXED_RPM_1);
  else        applyRPM(0.0f);
}
void publishData(){ RCSOFTCHECK(rcl_publish(&debug_motor_publisher,&debug_motor_msg,NULL)); }

// ===== Callbacks (ฝั่ง Stepper/Servo) =====
void twistCallback(const void* msgin){ // /nana/cmd_arm/rpm
  const auto* msg = (const geometry_msgs__msg__Twist*)msgin;
  float v = msg->linear.x;
  last_cmd_ms = millis();
  if(v > 0.1f){ s1_held=true; s1_dir=+1; }
  else if(v < -0.1f){ s1_held=true; s1_dir=-1; }
  else { s1_held=false; }
}

void mahoCallback(const void* msgin){
  const auto* msg = (const geometry_msgs__msg__Twist*)msgin;

  // ความหมาย:
  // linear.x = ทิศทางกด  (-1, 0, +1)   => 0 = ไม่กด, ±1 = กด
  // linear.y = องศาที่จะหมุน (ถ้า 0 จะใช้ S2_DEFAULT_DEGREES)
  // angular.x = interval_us (ถ้า <=0 จะใช้ S2_DEFAULT_INTERVAL)

  int dir = (int)msg->linear.x;
  float deg = (msg->linear.y != 0.0f) ? (float)msg->linear.y : S2_DEFAULT_DEGREES;
  uint32_t use_us = (msg->angular.x > 0.0f) ? (uint32_t)msg->angular.x : S2_DEFAULT_INTERVAL;

  // กดอยู่ไหม (ถือว่า nonzero = กด)
  bool pressed = (dir == 1 || dir == -1);
  uint32_t now = millis();

  // ดีบาวน์ + edge-trigger: "ไม่เคยกด → กด" เท่านั้นที่ยิงคำสั่ง
  if (pressed && !s2_prev_pressed && (now - s2_last_trigger_ms >= S2_DEBOUNCE_MS)) {
    // ถ้าอยากให้ไม่รับกดซ้ำระหว่างกำลังวิ่ง ก็ปล่อยไว้แบบนี้
    // (สั่งรอบเดียว ไม่สนใจว่าจะยังค้างกดอยู่)
    startStepper2Degrees(deg, dir, use_us);
    s2_last_trigger_ms = now;
  }

  // อัปเดตสถานะเพื่อรอรอบต่อไป (ต้อง "ปล่อย" ก่อน จึงจะกดครั้งใหม่ติด)
  s2_prev_pressed = pressed;
}

void gripCallback(const void* msgin){ const auto* msg=(const geometry_msgs__msg__Twist*)msgin; int cmd=(int)msg->linear.x; if(cmd==1||cmd==-1) servoSetTargetFromCmd(cmd); }

// ========================== โค้ดที่ 2: DC Motor 4 ช่อง (ผสาน) ==========================
// ขา PWM (ไม่ทับกับสเต็ปเปอร์/เซอร์โว)
#define AIN1 25
#define AIN2 26
#define BIN1 14
#define BIN2 33

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_AIN1 0
#define PWM_CHANNEL_AIN2 1
#define PWM_CHANNEL_BIN1 2
#define PWM_CHANNEL_BIN2 3

// Entities เพิ่มเติม (ของโค้ดที่ 2)
rcl_publisher_t debug_motor_publisher_move;
geometry_msgs__msg__Twist debug_motor_msg_move;

rcl_subscription_t motor_subscriber_move;
geometry_msgs__msg__Twist motor_msg_move;

// ฟังก์ชันฝั่ง DC (คงตรรกะเดิม)
void MoveDC(){
  float motor1Speed = motor_msg_move.linear.x;
  float motor2Speed = motor_msg_move.linear.y;

  float max_rpm = 150.0;

  uint8_t duty1 = (uint8_t)((fabs(motor1Speed) / max_rpm) * 255.0);
  uint8_t duty2 = (uint8_t)((fabs(motor2Speed) / max_rpm) * 255.0);

  // Motor A control
  if (motor1Speed > 0) {
      ledcWrite(PWM_CHANNEL_AIN1, duty1);
      ledcWrite(PWM_CHANNEL_AIN2, 0);
  } else if (motor1Speed < 0) {
      ledcWrite(PWM_CHANNEL_AIN1, 0);
      ledcWrite(PWM_CHANNEL_AIN2, duty1);
  } else {
      ledcWrite(PWM_CHANNEL_AIN1, 0);
      ledcWrite(PWM_CHANNEL_AIN2, 0);
  }

  // Motor B control
  if (motor2Speed > 0) {
      ledcWrite(PWM_CHANNEL_BIN1, duty2);
      ledcWrite(PWM_CHANNEL_BIN2, 0);
  } else if (motor2Speed < 0) {
      ledcWrite(PWM_CHANNEL_BIN1, 0);
      ledcWrite(PWM_CHANNEL_BIN2, duty2);
  } else {
      ledcWrite(PWM_CHANNEL_BIN1, 0);
      ledcWrite(PWM_CHANNEL_BIN2, 0);
  }

  debug_motor_msg_move.linear.x = duty1;
  debug_motor_msg_move.linear.y = duty2;
}

void publishDataDC(){
  // พิมพ์ดีบักแบบเดิม
  Serial.print("Publishing debug_motor_msg_move: ");
  Serial.print(debug_motor_msg_move.linear.x);
  Serial.print(", ");
  Serial.println(debug_motor_msg_move.linear.y);

  rcl_ret_t ret = rcl_publish(&debug_motor_publisher_move, &debug_motor_msg_move, NULL);
  if (ret != RCL_RET_OK) {
      Serial.print("Publish failed (move) with error: ");
      Serial.println(rcl_get_error_string().str);
      rcl_reset_error();
  }
}

void twistCallbackMove(const void* msgin){
  const auto* msg = (const geometry_msgs__msg__Twist*)msgin;
  motor_msg_move = *msg;  // copy
}

// ========================== สร้าง/ทำลายเอนทิตีรวม ==========================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  geometry_msgs__msg__Twist__init(&debug_motor_msg);
  geometry_msgs__msg__Twist__init(&maho_debug_msg);
  geometry_msgs__msg__Twist__init(&grip_debug_msg);
  geometry_msgs__msg__Twist__init(&motor_msg);
  geometry_msgs__msg__Twist__init(&maho_msg);
  geometry_msgs__msg__Twist__init(&grip_msg);

  // สำหรับ DC (โค้ดที่ 2)
  geometry_msgs__msg__Twist__init(&motor_msg_move);
  geometry_msgs__msg__Twist__init(&debug_motor_msg_move);

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 96));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_dual_stepper_dc_node", "", &support));

  // Publishers (เดิม)
  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/debug/cmd_arm/rpm"));
  RCCHECK(rclc_publisher_init_best_effort(&maho_debug_publisher,&node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/debug/cmd_maho/rpm"));
  RCCHECK(rclc_publisher_init_best_effort(&grip_debug_publisher,&node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/debug/cmd_grip/rpm"));

  // Subscribers (เดิม)
  RCCHECK(rclc_subscription_init_default(&motor_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/cmd_arm/rpm"));
  RCCHECK(rclc_subscription_init_default(&maho_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/cmd_maho/rpm"));
  RCCHECK(rclc_subscription_init_default(&grip_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/cmd_grip/rpm"));

  // เพิ่มของโค้ดที่ 2 (DC move)
  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_publisher_move,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/debug/cmd_move/rpm"));
  RCCHECK(rclc_subscription_init_default(&motor_subscriber_move,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/nana/cmd_move/rpm"));

  // Timer รวม
  RCCHECK(rclc_timer_init_default(&control_timer,&support,RCL_MS_TO_NS(CONTROL_PERIOD_MS),
    [](rcl_timer_t*, int64_t){
      // ฝั่งสเต็ปเปอร์/เซอร์โว (โค้ดที่ 1)
      if((millis()-last_cmd_ms) > COMMAND_KEEPALIVE_MS) s1_held=false;
      Move();
      publishData();
      servoUpdateAndPublish();
      if(s2_just_finished){
        bool cw; int steps; float deg;
        portENTER_CRITICAL(&timerMux2);
        s2_just_finished=false; cw=s2_dir_is_cw; steps=s2_total_steps_cmd; deg=s2_degrees_cmd;
        portEXIT_CRITICAL(&timerMux2);
        publishMahoDebug(cw?+1:-1,0,deg,true);
      }
      // ฝั่ง DC (โค้ดที่ 2)
      MoveDC();
      publishDataDC();
    }
  ));

  // Executor รวม
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor,&support.context,8,&allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&motor_subscriber,&motor_msg,&twistCallback,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&maho_subscriber,&maho_msg,&mahoCallback,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&grip_subscriber,&grip_msg,&gripCallback,ON_NEW_DATA));

  // เพิ่ม subscription ของโค้ดที่ 2
  RCCHECK(rclc_executor_add_subscription(&executor,&motor_subscriber_move,&motor_msg_move,&twistCallbackMove,ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor,&control_timer));

  s1_held=false; s1_dir=+1; last_cmd_ms=0;
  last_agent_ok_ms = millis();
  return true;
}

bool destroyEntities(){
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context,0);

  // pubs/subs (เดิม)
  RCSOFTCHECK(rcl_publisher_fini(&debug_motor_publisher,&node));
  RCSOFTCHECK(rcl_publisher_fini(&maho_debug_publisher,&node));
  RCSOFTCHECK(rcl_publisher_fini(&grip_debug_publisher,&node));
  RCSOFTCHECK(rcl_subscription_fini(&motor_subscriber,&node));
  RCSOFTCHECK(rcl_subscription_fini(&maho_subscriber,&node));
  RCSOFTCHECK(rcl_subscription_fini(&grip_subscriber,&node));

  // pubs/subs (DC)
  RCSOFTCHECK(rcl_publisher_fini(&debug_motor_publisher_move,&node));
  RCSOFTCHECK(rcl_subscription_fini(&motor_subscriber_move,&node));

  RCSOFTCHECK(rcl_timer_fini(&control_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  geometry_msgs__msg__Twist__fini(&debug_motor_msg);
  geometry_msgs__msg__Twist__fini(&maho_debug_msg);
  geometry_msgs__msg__Twist__fini(&grip_debug_msg);
  geometry_msgs__msg__Twist__fini(&motor_msg);
  geometry_msgs__msg__Twist__fini(&maho_msg);
  geometry_msgs__msg__Twist__fini(&grip_msg);
  geometry_msgs__msg__Twist__fini(&motor_msg_move);
  geometry_msgs__msg__Twist__fini(&debug_motor_msg_move);
  return true;
}

// ========================== Setup/Loop ==========================
void setup(){
  // Serial/micro-ROS
  Serial.begin(460800);                 // ให้ตั้ง agent ด้วยความเร็วที่รองรับ เช่น -b 460800
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  delay(50);
  set_microros_serial_transports(Serial);

  // Stepper pins
  pinMode(STEP_PIN1,OUTPUT); pinMode(DIR_PIN1,OUTPUT);
  pinMode(STEP_PIN2,OUTPUT); pinMode(DIR_PIN2,OUTPUT);
  gpio_set_level((gpio_num_t)STEP_PIN1,0); gpio_set_level((gpio_num_t)DIR_PIN1,0);
  gpio_set_level((gpio_num_t)STEP_PIN2,0); gpio_set_level((gpio_num_t)DIR_PIN2,0);

  // Servo
  ledcSetup(SERVO_LEDC_CH, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CH);
  // ตั้งไว้ที่ 0 หรือ 180 ตามต้องการเริ่มต้น
  servoWriteDeg(180);

  // DC motor pins + PWM
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  ledcSetup(PWM_CHANNEL_AIN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_AIN2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BIN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BIN2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(AIN1, PWM_CHANNEL_AIN1);
  ledcAttachPin(AIN2, PWM_CHANNEL_AIN2);
  ledcAttachPin(BIN1, PWM_CHANNEL_BIN1);
  ledcAttachPin(BIN2, PWM_CHANNEL_BIN2);

  // Timers (Stepper)
  stepper_timer = timerBegin(0,80,true); timerAttachInterrupt(stepper_timer,&onStepTimer,true); timerAlarmWrite(stepper_timer,1000,true); timerAlarmEnable(stepper_timer);
  stepper2_timer= timerBegin(1,80,true); timerAttachInterrupt(stepper2_timer,&onStep2Timer,true); timerAlarmWrite(stepper2_timer,1000,true); timerAlarmEnable(stepper2_timer);

  state = WAITING_AGENT; last_agent_ok_ms = millis();
}

void loop(){
  uint32_t now = millis();
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {
          state = AGENT_AVAILABLE; last_agent_ok_ms = now;
        }
      });
      break;

    case AGENT_AVAILABLE:
      state = createEntities()?AGENT_CONNECTED:WAITING_AGENT;
      break;

    case AGENT_CONNECTED:{
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      EXECUTE_EVERY_N_MS(3000, {
        if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {
          last_agent_ok_ms = millis();
        }
      });
      if (millis() - last_agent_ok_ms > AGENT_DISCONNECT_GRACE_MS) {
        state = AGENT_DISCONNECTED;
      }
    } break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      s1_held=false; applyRPM(0);
      portENTER_CRITICAL(&timerMux2); s2_running=false; s2_steps_remaining=0; portEXIT_CRITICAL(&timerMux2);
      state = WAITING_AGENT;
      break;
  }
}