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

Servo servo3;
const int servo3_pin = 25;

// micro-ROS objects
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Servo state
int currentAngle = 0;  // เริ่มต้นที่ 0 องศา

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// Callback เมื่อรับ message จาก topic
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data) {
    // หมุนไปที่ 30 องศา
    currentAngle = -60;
  } else {
    // หมุนกลับไปที่ 0 องศา
    currentAngle = 0;
  }

  servo3.write(currentAngle);
}

void setup()
{
  Serial.begin(115200);

  // ตั้งค่า micro-ROS transport (Serial)
  set_microros_serial_transports(Serial);
  delay(2000);

  // ตั้งค่า Servo
  servo3.attach(servo3_pin);
  servo3.write(0);  // เริ่มต้นที่ 0 องศา

  allocator = rcl_get_default_allocator();

  // สร้าง support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // สร้าง node
  RCCHECK(rclc_node_init_default(&node, "maincam_node", "", &support));

  // สร้าง subscriber รับ Bool จาก topic "servo_cmd"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "servo_cmd"
  ));

  // สร้าง executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA
  ));
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}