#!/usr/bin/env python3


import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos

import time 
import math


class Arm_node(Node):


    moveSpeed: float = 0.0
    slideSpeed: float = 0.0 
    turnSpeed: float = 0.0

    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0


    def __init__(self):
        super().__init__("Arm_node")

        
        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : int = 1023.0 # pwm
        self.maxRPM : int = 150
        self.motor1Speed : float = 0

        self.send_robot_speed = self.create_publisher(
            Twist, "/nana/cmd_arm/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/nana/cmd_arm', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)

    
    

    def cmd_vel(self, msg):

        linear_vel = msg.linear.y   # forward/backward
        angular_vel = msg.angular.z # turning rate

        # Compute left and right wheel speeds (in m/s)
        v_left = linear_vel - angular_vel 

        # Convert to motor speeds in RPM (optional)
        rpm_left = float(v_left * self.maxRPM)

        # Store or send these speeds to motor controller
        self.motor1Speed = rpm_left


        print(f"Left Motor: {self.motor1Speed:.2f} RPM, Right Motor: {self.motor2Speed:.2f} RPM")


        

            
    def sendData(self):
        motorspeed_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        
        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()

    sub = Arm_node()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()