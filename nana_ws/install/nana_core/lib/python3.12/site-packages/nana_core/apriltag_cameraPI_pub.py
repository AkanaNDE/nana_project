#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cap = cv2.VideoCapture(2) # ปรับ Index ตามกล้อง
        self.bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error("ไม่สามารถเปิดกล้องบน Pi ได้!")
            
        self.timer = self.create_timer(0.05, self.timer_callback) # ~20 FPS

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # บีบอัดขนาดภาพก่อนส่ง (Optional: ช่วยลด Bandwidth)
            # frame = cv2.resize(frame, (320, 240)) 
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()