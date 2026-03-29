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
        self.declare_parameter('wheel_diameter_cm',35.0)
        self.declare_parameter('calibration_factor', 10.0)

        self.sub = self.create_subscription(
            String, '/encoder_ticks', self.encoder_cb, 10)
        self.pub = self.create_publisher(
            Float32, '/current_distance_cm', 10)

        # --- เพิ่มตัวแปรสำหรับจำค่าล่าสุดและสะสมระยะ ---
        self.last_ticks_values = None
        self.total_accumulated_ticks = 0.0 
        # ------------------------------------------

        self.get_logger().info("Encoder to Distance Node Started (with Overflow Handling).")

    def encoder_cb(self, msg):
        try:
            ticks_list = re.findall(r'-?\d+', msg.data)
            if not ticks_list:
                return

            current_ticks_values = [int(t) for t in ticks_list]

            # ถ้าเป็นการรับค่าครั้งแรก ให้จำค่าไว้ก่อนแล้วยังไม่คำนวณ
            if self.last_ticks_values is None:
                self.last_ticks_values = current_ticks_values
                return

            # 1. คำนวณส่วนต่าง (Delta) ของแต่ละล้อ แล้วหาค่าเฉลี่ยของส่วนต่าง
            deltas = []
            for i in range(len(current_ticks_values)):
                diff = current_ticks_values[i] - self.last_ticks_values[i]
                
                # ดักจับการ Wrap-around (กรณี 16-bit: 32767 -> -32768)
                # ถ้าส่วนต่างมันโดดเกินครึ่งหนึ่งของ Range (32768) แสดงว่าเกิดการล้น
                if diff > 30000:
                    diff -= 65536
                elif diff < -30000:
                    diff += 65536
                
                deltas.append(diff)

            # 2. หาค่าเฉลี่ยของส่วนต่างที่เกิดขึ้น (ใช้ abs เพื่อให้เดินหน้าอย่างเดียวตามความตั้งใจเดิมของคุณ)
            avg_delta = sum(abs(d) for d in deltas) / len(deltas)
            
            # 3. สะสมค่าเฉลี่ยส่วนต่างลงในตัวแปรหลัก
            self.total_accumulated_ticks += avg_delta

            # เก็บค่าปัจจุบันไว้เป็นค่าเก่าสำหรับรอบถัดไป
            self.last_ticks_values = current_ticks_values

            # 4. คำนวณระยะทางจากค่าสะสม (Total Accumulated Ticks)
            tpr        = self.get_parameter('ticks_per_rev').value
            diameter   = self.get_parameter('wheel_diameter_cm').value
            cal_factor = self.get_parameter('calibration_factor').value

            distance_cm = ((self.total_accumulated_ticks / tpr) * (math.pi * diameter)) / cal_factor

            out_msg = Float32()
            out_msg.data = float(distance_cm)
            self.pub.publish(out_msg)

            self.get_logger().info(
                f"ticks={current_ticks_values} | total_ticks={self.total_accumulated_ticks:.1f} | "
                f"distance={distance_cm:.3f} cm",
                throttle_duration_sec=1.0)

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