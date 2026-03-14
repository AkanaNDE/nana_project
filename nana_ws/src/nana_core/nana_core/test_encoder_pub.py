#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # เปลี่ยนจาก Int32 เป็น String

class EncoderMockStringPublisher(Node):
    def __init__(self):
        super().__init__('encoder_mock_publisher')
        # ต้องส่งเข้า Topic /encoder_ticks และใช้ Type เป็น String ตาม ESP32
        self.publisher_ = self.create_publisher(String, '/encoder_ticks', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.ticks = 0
        self.get_logger().info('Mock Encoder (String Mode) started...')
        self.get_logger().info('Sending format: "ENC:\\tT1\\tT2\\tT3\\tT4"')

    def timer_callback(self):
        self.ticks += 500  # จำลองว่าล้อหมุนไปเรื่อยๆ
        
        # สร้าง String Format ให้เหมือนกับในโค้ด ESP32 ของคุณ 
        # คือ "ENC:" ตามด้วย \t และค่า encoder 4 ตัว
        mock_string = f"ENC:\t{self.ticks}\t{self.ticks}\t{self.ticks}\t{self.ticks}"
        
        msg = String()
        msg.data = mock_string
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderMockStringPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()