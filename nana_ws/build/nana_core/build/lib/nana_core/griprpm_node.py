#!/usr/bin/env python3


import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos

import time 
import math


class Grip_node(Node):
    motor1Speed : float = 0


    def __init__(self):
        super().__init__("Grip_node")
        self.motor1Speed = 0.0


        self.send_robot_speed = self.create_publisher(
            Twist, "/nana/cmd_grip/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/nana/cmd_grip', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.sent_data_timer = self.create_timer(0.03, self.sendData)

    
    

    def cmd_vel(self, msg):

        linear_vel = msg.linear.y   # forward/backward
        angular_vel = msg.angular.z # turning rate

        # Compute left and right wheel speeds (in m/s)
        v = linear_vel - angular_vel 

        self.motor1Speed = v

            
    def sendData(self):
        motorspeed_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        
        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()

    sub = Grip_node()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()