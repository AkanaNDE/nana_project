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


#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)


#define STEP_PIN 22
#define DIR_PIN 13
#define ENABLE_PIN 21


#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution
#define PWM_CHANNEL_AIN1 0
#define PWM_CHANNEL_AIN2 1
#define PWM_CHANNEL_BIN1 2
const int STEPS_PER_REV = 200;       // Steps per revolution (adjust as needed)
const int STEP_DELAY_US = 1000;      // Microseconds between steps (controls speed)
//------------------------------ < Define > -------------------------------------//

rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rcl_subscription_t grip_subscriber;
geometry_msgs__msg__Twist grip_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;



//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void fullStop();

void Move();

void Grip();

//------------------------------ < Main > -------------------------------------//



void setup()
{   

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

pinMode(STEP_PIN, OUTPUT);
pinMode(DIR_PIN, OUTPUT);
pinMode(ENABLE_PIN, OUTPUT);             
digitalWrite(ENABLE_PIN, LOW); 

ledcSetup(PWM_CHANNEL_AIN1, PWM_FREQ, PWM_RESOLUTION);
ledcSetup(PWM_CHANNEL_AIN2, PWM_FREQ, PWM_RESOLUTION);
ledcSetup(PWM_CHANNEL_BIN1, PWM_FREQ, PWM_RESOLUTION);
ledcSetup(PWM_CHANNEL_BIN2, PWM_FREQ, PWM_RESOLUTION);

ledcAttachPin(AIN1, PWM_CHANNEL_AIN1);
ledcAttachPin(AIN2, PWM_CHANNEL_AIN2);
ledcAttachPin(BIN1, PWM_CHANNEL_BIN1);
ledcAttachPin(BIN2, PWM_CHANNEL_BIN2);

}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//


void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Move();
        publishData();
        
    }
}

void twistCallback(const void *msgin)
{
    prev_cmd_time = millis();
}




bool createEntities()
{
    // Get default allocator
    allocator = rcl_get_default_allocator();

    // Initialize the debug_motor_msg
    geometry_msgs__msg__Twist__init(&debug_motor_msg);

    // Initialize RCL options and set domain ID to 96
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));

    // Initialize support object with custom init options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Initialize the node
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    // Initialize the publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nana/debug/cmd_arm/rpm"));

    // Initialize the subscriber
    RCCHECK(rclc_subscription_init_default(
        &motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nana/cmd_arm/rpm"));

    // Initialize the timer
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    RCCHECK(rclc_subscription_init_default(
        &grip_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nana/cmd_grip/rpm"));

    // Initialize the executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &motor_subscriber,
        &motor_msg,
        &grip_subscriber,
        &grip_msg,
        &twistCallback,
        ON_NEW_DATA));

    // Add timer to executor
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // Sync time (optional)
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_subscription_fini(&grip_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}




void Move(){
    float motor1Speed = motor_msg.linear.x;

    // debug_motor_msg.linear.x = motor1Speed;
    // debug_motor_msg.linear.y = motor2Speed;

    float max_rpm = 150.0;

uint8_t duty1 = (uint8_t)((fabs(motor1Speed) / max_rpm) * 255.0);


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

    debug_motor_msg.linear.x = duty1;

}

void Grip()
{
    float servo = grip_msg.linear.x;

    if (servo == 1)
    {
        for (int pos = 0; pos <= 180; pos += 1) {
        myServo.write(pos);
        delay(10);
        }
    }

    if (servo == -1)
    {
        for (int pos = 180; pos >= 0; pos -= 1) {
        myServo.write(pos);
        delay(10);
        }
    }
}


void publishData()
{
    Serial.print("Publishing debug_motor_msg: ");
    Serial.print(debug_motor_msg.linear.x);
    Serial.print(", ");
    Serial.println(debug_motor_msg.linear.y);
    
    rcl_ret_t ret = rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);
    if (ret != RCL_RET_OK) {
        Serial.print("Publish failed with error: ");
        Serial.println(rcl_get_error_string().str);
        rcl_reset_error();
    }
}

void syncTime()
{

    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    ESP.restart
}