#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import CompressedImage


TOPIC_IMAGE_COMPRESSED = "/camera/image/compressed"


class CameraCompressedPublisher(Node):
    def __init__(self):
        super().__init__("camera_compressed_publisher")

        self.pub = self.create_publisher(CompressedImage, TOPIC_IMAGE_COMPRESSED, 10)

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("timer_period", 0.03)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("preview", False)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.preview = bool(self.get_parameter("preview").value)

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera. Check camera index / permissions.")
        else:
            self.get_logger().info(f"Camera opened. index={self.camera_index}")

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Camera Compressed Publisher Started")

    def timer_callback(self):
        if self.cap is None or (not self.cap.isOpened()):
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = enc.tobytes()
        self.pub.publish(msg)

        if self.preview:
            cv2.imshow("Camera Compressed Preview", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCompressedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()