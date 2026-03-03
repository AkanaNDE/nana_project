#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


class TagCameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_tag_node')   # ✅ เปลี่ยนชื่อ node

        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open tag camera")
        else:
            self.get_logger().info("Tag camera opened")

        # ✅ เปลี่ยน topic
        self.pub = self.create_publisher(
            CompressedImage,
            '/camera/tag/compressed',
            10
        )

        self.timer = self.create_timer(0.03, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        ok, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = jpg.tobytes()

        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = TagCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()