#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import math
import re

class EncoderToDistance(Node):
    def __init__(self):
        super().__init__('encoder_to_distance')
        self.declare_parameter('ticks_per_rev', 1000.0)
        self.declare_parameter('wheel_diameter_cm', 11.0)

        # *** เพิ่ม calibration_factor: ปรับจาก (encoder_cm / actual_cm)
        #     ในกรณีนี้ encoder อ่าน 10 cm จริง 1 cm → factor = 10.0 ***
        self.declare_parameter('calibration_factor', 10.0)

        self.sub = self.create_subscription(String, '/encoder_ticks', self.encoder_cb, 10)
        self.pub = self.create_publisher(Float32, '/current_distance_cm', 10)
        self.get_logger().info("Encoder to Distance Node Started.")

    def encoder_cb(self, msg):
        try:
            ticks_list = re.findall(r'-?\d+', msg.data)
            if not ticks_list: return

            ticks_values = [int(t) for t in ticks_list]
            avg_ticks = sum(abs(t) for t in ticks_values) / len(ticks_values)

            tpr        = self.get_parameter('ticks_per_rev').value
            diameter   = self.get_parameter('wheel_diameter_cm').value
            cal_factor = self.get_parameter('calibration_factor').value

            # *** หาร calibration_factor เพื่อชดเชยค่าที่คลาดเคลื่อน ***
            distance_cm = ((avg_ticks / tpr) * (math.pi * diameter)) / cal_factor

            out_msg = Float32()
            out_msg.data = float(distance_cm)
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()