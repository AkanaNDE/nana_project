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

// === Macros ===
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) rclErrorLoop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) init = uxr_millis(); if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)

// === Motor Pins ===
#define STEP_PIN 22
#define DIR_PIN 13
#define ENABLE_PIN 21

// === Stepper Settings ===
const int STEPS_PER_REV = 200;
float target_rpm = 0;
unsigned long last_step_time = 0;

// === ROS Entities ===
rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

hw_timer_t *stepper_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool doStep = false;


// === Function Declarations ===
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
void Move();
struct timespec getTime();
void fullStop();

// === Setup ===
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Stepper motor pin setup
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Enable motor driver

    // Initial ROS state
    state = WAITING_AGENT;

    // Set up hardware timer for step pulses
    stepper_timer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz => 1 tick = 1 µs
    timerAttachInterrupt(stepper_timer, []() {
        portENTER_CRITICAL_ISR(&timerMux);
        doStep = true;
        portEXIT_CRITICAL_ISR(&timerMux);
    }, true);


    Serial.println("Setup complete. Waiting for Micro-ROS Agent...");
}

// === Main Loop ===
void loop()
{

    if (doStep) {
    portENTER_CRITICAL(&timerMux);
    doStep = false;
    portEXIT_CRITICAL(&timerMux);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(STEP_PIN, LOW);
}


    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500,
            state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        );
        break;

    case AGENT_AVAILABLE:
        state = (createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) destroyEntities();
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200,
            state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
        );
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        }
        break;

    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    }
}

void Move()
{
    target_rpm = motor_msg.linear.x;

    // Clamp RPM
    if (fabs(target_rpm) > 1000) {
        target_rpm = (target_rpm > 0) ? 1000 : -1000;
    }

    // Set direction
    digitalWrite(DIR_PIN, (target_rpm >= 0) ? HIGH : LOW);

    // Convert RPM to steps per second
    float steps_per_second = fabs(target_rpm) * (STEPS_PER_REV / 60.0);

    if (steps_per_second == 0) {
        timerAlarmDisable(stepper_timer); // Stop timer
        return;
    }

    // Step interval in µs
    unsigned long step_interval_us = 1000000UL / steps_per_second;
    if (step_interval_us < 100) step_interval_us = 100;  // Safety clamp

    // Set hardware timer
    timerAlarmWrite(stepper_timer, step_interval_us, true); // auto-reload
    timerAlarmEnable(stepper_timer);

    // Publish debug
    debug_motor_msg.linear.x = target_rpm;
    debug_motor_msg.linear.y = steps_per_second;
}



// === Publish Debug Data ===
void publishData()
{
    rcl_ret_t ret = rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);
    if (ret != RCL_RET_OK) {
        rcl_reset_error();
    }
}

// === Timer Callback ===
void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Move();
        publishData();
    }
}

// === Subscriber Callback ===
void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor_msg = *msg;

    Serial.print("Received RPM: ");
    Serial.println(motor_msg.linear.x);
}

// === Create ROS Entities ===
bool createEntities()
{
    allocator = rcl_get_default_allocator();
    geometry_msgs__msg__Twist__init(&debug_motor_msg);

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nana/debug/cmd_arm/rpm"));

    RCCHECK(rclc_subscription_init_default(
        &motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nana/cmd_arm/rpm"));

    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(1),  // 1ms timer
        controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    return true;
}

// === Destroy ROS Entities ===
bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_subscription_fini(&motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

// === Time Sync ===
void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

// === Get Time (ROS) ===
struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// === Error Handling Loop ===
void rclErrorLoop()
{
    const int LED_PIN = 2;
    pinMode(LED_PIN, OUTPUT);
    while (1)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}
