#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('camera_index', 2)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('rotate_90_cw', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('preview', True)

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.rotate_90_cw = bool(self.get_parameter('rotate_90_cw').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.preview = bool(self.get_parameter('preview').value)

        # ✅ ถ้า headless ไม่มีจอ ให้ปิด preview อัตโนมัติ
        if not os.environ.get("DISPLAY"):
            if self.preview:
                self.get_logger().warn("No DISPLAY detected (headless). preview will be disabled.")
            self.preview = False

        self.pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)

        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"CameraPublisher started: index={self.camera_index}, "
            f"{self.width}x{self.height}@{self.fps} -> /camera/image/compressed"
        )

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Camera read failed")
            return

        if self.rotate_90_cw:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
        ok, jpg = cv2.imencode('.jpg', frame, encode_param)
        if not ok:
            self.get_logger().warn("JPEG encode failed")
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = jpg.tobytes()
        self.pub.publish(msg)

        # ✅ Preview (เฉพาะตอนมีจอจริง)
        if self.preview:
            try:
                cv2.imshow("Pi Camera Preview (publishing)", frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"Preview failed, disabling preview. err={e}")
                self.preview = False

    def destroy_node(self):
        try:
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
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()