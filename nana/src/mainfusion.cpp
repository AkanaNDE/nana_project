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

// ========================= Utils / Macros =========================
void rclErrorLoop();

#define RCCHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) rclErrorLoop(); } while(0)
#define RCSOFTCHECK(fn) do { rcl_ret_t rc = (fn); (void)rc; } while(0)
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t t_ms = -1; if (t_ms == -1) t_ms = uxr_millis(); if ((int32_t)(uxr_millis() - t_ms) > (MS)) { X; t_ms = uxr_millis(); } } while (0)

static inline float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }

// ========================= Stepper #1 / #2 + Servo =========================
// Pins (Steppers)
#define STEP_PIN1 22
#define DIR_PIN1  13
#define STEP_PIN2 21
#define DIR_PIN2  16
// Servo
#define SERVO_PIN 27
#define SERVO_LEDC_CH 4
#define SERVO_FREQ_HZ 50
#define SERVO_RES_BITS 16
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_SLEW_DEG_PER_TICK 4

// Stepper timing & params
static const int   STEPS_PER_REV = 200;
static const int   MICROSTEP     = 16;
static const float MAX_ABS_RPM   = 200.0f;
static const uint32_t MIN_STEP_INTERVAL_US = 900;  // slower => more torque
static const uint32_t STEP_PULSE_US        = 70;
static const uint32_t DIR_SETUP_US         = 120;
static const float    FIXED_RPM_1          = 12.0f;
static const float    S2_DEFAULT_DEGREES   = 36.0f;
static const uint32_t S2_DEFAULT_INTERVAL  = 1200;

static const uint32_t CONTROL_PERIOD_MS    = 20;
static const uint32_t COMMAND_KEEPALIVE_MS = 3000;
static const uint32_t AGENT_DISCONNECT_GRACE_MS = 30000;

// HW timer Stepper #1
hw_timer_t *stepper_timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool     is_running=false, step_high_phase=false, dir_is_cw=true, need_dir_setup_gap=false;
volatile uint32_t current_interval_us=1000, target_interval_us=1000, pulse_us=STEP_PULSE_US;

// hold-to-run state
static volatile bool  s1_held = false;
static volatile int   s1_dir  = +1;
static volatile uint32_t last_cmd_ms = 0;

// ISR Stepper #1
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

static inline void applyRPM(float rpm){
  if(rpm==0.0f){
    portENTER_CRITICAL(&timerMux);
    is_running=false; step_high_phase=false;
    target_interval_us = current_interval_us = 1000;
    portEXIT_CRITICAL(&timerMux);
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
}

// Stepper #2
hw_timer_t *stepper2_timer = nullptr;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
volatile bool s2_running=false, s2_step_high_phase=false, s2_dir_is_cw=true, s2_need_dir_gap=false, s2_just_finished=false;
volatile uint32_t s2_interval_us = S2_DEFAULT_INTERVAL, s2_pulse_us = STEP_PULSE_US;
volatile int32_t  s2_steps_remaining=0, s2_total_steps_cmd=0;
volatile float    s2_degrees_cmd=S2_DEFAULT_DEGREES;

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
  portENTER_CRITICAL(&timerMux2);
  s2_dir_is_cw = (direction>0); s2_need_dir_gap = true;
  s2_interval_us = (interval_us_for_s2<MIN_STEP_INTERVAL_US)?MIN_STEP_INTERVAL_US:interval_us_for_s2;
  s2_steps_remaining=total_steps; s2_total_steps_cmd=total_steps; s2_degrees_cmd=deg;
  s2_running=true; s2_step_high_phase=false;
  portEXIT_CRITICAL(&timerMux2);
}

// Servo helpers
static inline uint32_t usToDuty(uint32_t us){
  const uint32_t period_us=1000000UL/SERVO_FREQ_HZ, max_duty=(1UL<<SERVO_RES_BITS)-1;
  if(us<SERVO_MIN_US)us=SERVO_MIN_US; if(us>SERVO_MAX_US)us=SERVO_MAX_US;
  return (uint32_t)((uint64_t)us*max_duty/period_us);
}
static inline uint32_t angleToUs(float d){ d=clampf(d,0,180); return (uint32_t)(SERVO_MIN_US + (SERVO_MAX_US-SERVO_MIN_US)*(d/180.0f) + 0.5f); }
static inline void servoWriteDeg(float d){ ledcWrite(SERVO_LEDC_CH, usToDuty(angleToUs(d))); }

static volatile float servo_current_deg=180, servo_target_deg=180; static volatile bool servo_reached=true;
static inline void servoSetTargetFromCmd(int cmd){ if(cmd>0) servo_target_deg=180; else if(cmd<0) servo_target_deg=130; else return; servo_reached=false; }
static inline void servoUpdate(){
  float c=servo_current_deg,t=servo_target_deg;
  if(fabsf(t-c)<=SERVO_SLEW_DEG_PER_TICK){ c=t; servo_reached=true; }
  else { c += (t>c?SERVO_SLEW_DEG_PER_TICK:-SERVO_SLEW_DEG_PER_TICK); }
  servo_current_deg=clampf(c,0,180); servoWriteDeg(servo_current_deg);
}

// ========================= DC Motors (PWM) =========================
// ===== DC Motor pins (no conflict) =====
#define AIN1 25  // changed from 25 -> 32 to avoid conflict with STEP_PIN2
#define AIN2 26   // changed from 26 -> 33
#define BIN1 12
#define BIN2 14

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_AIN1 0
#define PWM_CHANNEL_AIN2 1
#define PWM_CHANNEL_BIN1 2
#define PWM_CHANNEL_BIN2 3

static inline void dcMotorsSetup(){
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
  // stop
  ledcWrite(PWM_CHANNEL_AIN1, 0); ledcWrite(PWM_CHANNEL_AIN2, 0);
  ledcWrite(PWM_CHANNEL_BIN1, 0); ledcWrite(PWM_CHANNEL_BIN2, 0);
}

static inline void driveDC(float rpm_left, float rpm_right, float max_rpm=150.0f){
  uint8_t dutyL = (uint8_t)(clampf(fabsf(rpm_left)/max_rpm,0,1)*255.0f);
  uint8_t dutyR = (uint8_t)(clampf(fabsf(rpm_right)/max_rpm,0,1)*255.0f);

  // Left (A)
  if (rpm_left > 0){
    ledcWrite(PWM_CHANNEL_AIN1, dutyL); ledcWrite(PWM_CHANNEL_AIN2, 0);
  } else if (rpm_left < 0){
    ledcWrite(PWM_CHANNEL_AIN1, 0);     ledcWrite(PWM_CHANNEL_AIN2, dutyL);
  } else {
    ledcWrite(PWM_CHANNEL_AIN1, 0);     ledcWrite(PWM_CHANNEL_AIN2, 0);
  }

  // Right (B)
  if (rpm_right > 0){
    ledcWrite(PWM_CHANNEL_BIN1, dutyR); ledcWrite(PWM_CHANNEL_BIN2, 0);
  } else if (rpm_right < 0){
    ledcWrite(PWM_CHANNEL_BIN1, 0);     ledcWrite(PWM_CHANNEL_BIN2, dutyR);
  } else {
    ledcWrite(PWM_CHANNEL_BIN1, 0);     ledcWrite(PWM_CHANNEL_BIN2, 0);
  }
}

// ========================= ROS Entities =========================
rcl_publisher_t debug_arm_pub, debug_maho_pub, debug_grip_pub;     // stepper/servo debug
rcl_publisher_t debug_move_pub;                                    // DC motors debug

geometry_msgs__msg__Twist arm_dbg_msg, maho_dbg_msg, grip_dbg_msg;
geometry_msgs__msg__Twist move_dbg_msg;

rcl_subscription_t arm_sub, maho_sub, grip_sub;   // from first sketch
rcl_subscription_t move_sub;                      // from second sketch

geometry_msgs__msg__Twist arm_cmd_msg, maho_cmd_msg, grip_cmd_msg; // reuse structure
geometry_msgs__msg__Twist move_cmd_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;
rcl_init_options_t init_options;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;
static uint32_t last_agent_ok_ms = 0;

// ========================= Callbacks =========================
void armTwistCb(const void* msgin){ // /nana/cmd_arm/rpm (hold-to-run switch)
  const auto* msg = (const geometry_msgs__msg__Twist*)msgin;
  float v = msg->linear.x;
  last_cmd_ms = millis();
  if(v > 0.1f){ s1_held=true; s1_dir=+1; }
  else if(v < -0.1f){ s1_held=true; s1_dir=-1; }
  else { s1_held=false; }
}

void mahoCb(const void* msgin){
  const auto* msg=(const geometry_msgs__msg__Twist*)msgin;
  int dir=(int)msg->linear.x;
  float deg=(msg->linear.y!=0.0f)?(float)msg->linear.y:S2_DEFAULT_DEGREES;
  uint32_t use_us=(msg->angular.x>0.0f)?(uint32_t)msg->angular.x:S2_DEFAULT_INTERVAL;
  if(dir==1||dir==-1) startStepper2Degrees(deg,dir,use_us);
}

void gripCb(const void* msgin){
  const auto* msg=(const geometry_msgs__msg__Twist*)msgin;
  int cmd=(int)msg->linear.x;
  if(cmd==1||cmd==-1) servoSetTargetFromCmd(cmd);
}

void moveCb(const void* msgin){ // /nana/cmd_move/rpm  (DC motors)
  const auto* msg = (const geometry_msgs__msg__Twist*)msgin;
  move_cmd_msg = *msg; // copy
}

// ========================= Periodic control =========================
static inline void stepperMoveTick(){
  if((millis()-last_cmd_ms) > COMMAND_KEEPALIVE_MS) s1_held=false;
  if(s1_held) applyRPM((float)s1_dir * FIXED_RPM_1);
  else        applyRPM(0.0f);
}

static inline void publishStepperDebug(){
  // optional: publish s1 rpm/sps or positions if you track them
  // Here we leave blank or you can add values into arm_dbg_msg if needed.
  RCSOFTCHECK(rcl_publish(&debug_arm_pub,&arm_dbg_msg,NULL));
  RCSOFTCHECK(rcl_publish(&debug_grip_pub,&grip_dbg_msg,NULL));
}

void controlCallback(rcl_timer_t*, int64_t){
  // Stepper #1 hold-to-run + debug
  stepperMoveTick();

  // DC motors command
  driveDC(move_cmd_msg.linear.x, move_cmd_msg.linear.y, 150.0f);
  move_dbg_msg.linear.x = move_cmd_msg.linear.x;
  move_dbg_msg.linear.y = move_cmd_msg.linear.y;
  RCSOFTCHECK(rcl_publish(&debug_move_pub,&move_dbg_msg,NULL));

  // Servo slew + debug
  servoUpdate();
  grip_dbg_msg.linear.x = servo_current_deg;
  grip_dbg_msg.linear.y = servo_target_deg;
  grip_dbg_msg.angular.z = servo_reached?1.0f:0.0f;
  RCSOFTCHECK(rcl_publish(&debug_grip_pub,&grip_dbg_msg,NULL));

  // Stepper #2 “done” event
  if(s2_just_finished){
    bool cw; int steps; float deg;
    portENTER_CRITICAL(&timerMux2);
      s2_just_finished=false; cw=s2_dir_is_cw; steps=s2_total_steps_cmd; deg=s2_degrees_cmd;
    portEXIT_CRITICAL(&timerMux2);
    maho_dbg_msg.linear.x = cw?+1:-1;
    maho_dbg_msg.linear.y = steps;
    maho_dbg_msg.linear.z = deg;
    maho_dbg_msg.angular.z = 1.0f;
    RCSOFTCHECK(rcl_publish(&debug_maho_pub,&maho_dbg_msg,NULL));
  }
}

// ========================= Create / Destroy =========================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  geometry_msgs__msg__Twist__init(&arm_dbg_msg);
  geometry_msgs__msg__Twist__init(&maho_dbg_msg);
  geometry_msgs__msg__Twist__init(&grip_dbg_msg);
  geometry_msgs__msg__Twist__init(&move_dbg_msg);

  geometry_msgs__msg__Twist__init(&arm_cmd_msg);
  geometry_msgs__msg__Twist__init(&maho_cmd_msg);
  geometry_msgs__msg__Twist__init(&grip_cmd_msg);
  geometry_msgs__msg__Twist__init(&move_cmd_msg);

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 96));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_combined_node", "", &support));

  // Publishers (debug)
  RCCHECK(rclc_publisher_init_best_effort(&debug_arm_pub , &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/debug/cmd_arm/rpm"));
  RCCHECK(rclc_publisher_init_best_effort(&debug_maho_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/debug/cmd_maho/rpm"));
  RCCHECK(rclc_publisher_init_best_effort(&debug_grip_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/debug/cmd_grip/rpm"));
  RCCHECK(rclc_publisher_init_best_effort(&debug_move_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/debug/cmd_move/rpm"));

  // Subscriptions (commands)
  RCCHECK(rclc_subscription_init_default(&arm_sub , &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/cmd_arm/rpm"));
  RCCHECK(rclc_subscription_init_default(&maho_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/cmd_maho/rpm"));
  RCCHECK(rclc_subscription_init_default(&grip_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/cmd_grip/rpm"));
  RCCHECK(rclc_subscription_init_default(&move_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "/nana/cmd_move/rpm"));

  // Timer
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS), controlCallback));

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&arm_sub ,&arm_cmd_msg ,&armTwistCb ,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&maho_sub,&maho_cmd_msg,&mahoCb    ,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&grip_sub,&grip_cmd_msg,&gripCb    ,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&move_sub,&move_cmd_msg,&moveCb    ,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor,&control_timer));

  s1_held=false; s1_dir=+1; last_cmd_ms=0;
  last_agent_ok_ms = millis();
  return true;
}

bool destroyEntities(){
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context,0);

  RCSOFTCHECK(rcl_publisher_fini(&debug_arm_pub ,&node));
  RCSOFTCHECK(rcl_publisher_fini(&debug_maho_pub,&node));
  RCSOFTCHECK(rcl_publisher_fini(&debug_grip_pub,&node));
  RCSOFTCHECK(rcl_publisher_fini(&debug_move_pub,&node));

  RCSOFTCHECK(rcl_subscription_fini(&arm_sub ,&node));
  RCSOFTCHECK(rcl_subscription_fini(&maho_sub,&node));
  RCSOFTCHECK(rcl_subscription_fini(&grip_sub,&node));
  RCSOFTCHECK(rcl_subscription_fini(&move_sub,&node));

  RCSOFTCHECK(rcl_timer_fini(&control_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  geometry_msgs__msg__Twist__fini(&arm_dbg_msg);
  geometry_msgs__msg__Twist__fini(&maho_dbg_msg);
  geometry_msgs__msg__Twist__fini(&grip_dbg_msg);
  geometry_msgs__msg__Twist__fini(&move_dbg_msg);
  geometry_msgs__msg__Twist__fini(&arm_cmd_msg);
  geometry_msgs__msg__Twist__fini(&maho_cmd_msg);
  geometry_msgs__msg__Twist__fini(&grip_cmd_msg);
  geometry_msgs__msg__Twist__fini(&move_cmd_msg);
  return true;
}

// ========================= Setup / Loop / Error =========================
void rclErrorLoop(){ while(1){ delay(150);} }

void setup(){
  // Serial for micro-ROS
  Serial.begin(460800);       // <-- Use agent baudrate 460800
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  delay(50);
  set_microros_serial_transports(Serial);

  // GPIO for steppers
  pinMode(STEP_PIN1,OUTPUT); pinMode(DIR_PIN1,OUTPUT);
  pinMode(STEP_PIN2,OUTPUT); pinMode(DIR_PIN2,OUTPUT);
  gpio_set_level((gpio_num_t)STEP_PIN1,0); gpio_set_level((gpio_num_t)DIR_PIN1,0);
  gpio_set_level((gpio_num_t)STEP_PIN2,0); gpio_set_level((gpio_num_t)DIR_PIN2,0);

  // Servo PWM
  ledcSetup(SERVO_LEDC_CH, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CH);
  servoWriteDeg(180);

  // Timers for steppers
  stepper_timer  = timerBegin(0,80,true); timerAttachInterrupt(stepper_timer ,&onStepTimer ,true); timerAlarmWrite(stepper_timer ,1000,true); timerAlarmEnable(stepper_timer );
  stepper2_timer = timerBegin(1,80,true); timerAttachInterrupt(stepper2_timer,&onStep2Timer,true); timerAlarmWrite(stepper2_timer,1000,true); timerAlarmEnable(stepper2_timer);

  // DC motors PWM init
  dcMotorsSetup();

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
      // stop steppers and dc motors safely
      s1_held=false; applyRPM(0);
      portENTER_CRITICAL(&timerMux2); s2_running=false; s2_steps_remaining=0; portEXIT_CRITICAL(&timerMux2);
      driveDC(0,0,150.0f);
      state = WAITING_AGENT;
      break;
  }
}