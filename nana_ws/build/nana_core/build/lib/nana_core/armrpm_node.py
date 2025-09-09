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




    def __init__(self):
        super().__init__("Arm_node")

        
        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.wheel_base = 0.2  # distance between wheels (meters)
        self.wheel_radius = 0.060  # radius of wheels (meters)

        self.maxSpeed : int = 1023.0 # pwm
        self.maxRPM : int = 150
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0


        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        self.send_robot_speed = self.create_publisher(
            Twist, "/nana/cmd_arm/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/nana/cmd_arm', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)