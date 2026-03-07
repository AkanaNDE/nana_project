#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage

class CameraCompressedPublisher(Node):
    def __init__(self):
        super().__init__("camera_compressed_publisher")
        
        # Quality parameters
        self.declare_parameter("camera_index", 4)
        self.declare_parameter("jpeg_quality", 80)
        
        self.camera_index = self.get_parameter("camera_index").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value

        self.pub = self.create_publisher(CompressedImage, "/camera/image/compressed", 10)

        # Initialize Camera with V4L2 for Pi performance
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        
        # Optimize Resolution for AprilTag Detection (640x480 is ideal)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open Pi Camera!")
            return

        self.timer = self.create_timer(0.033, self.timer_callback) # ~30 FPS
        self.get_logger().info("Pi 5 Camera Publisher Started (640x480)")

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        # Encode to JPEG
        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = enc.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCompressedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()