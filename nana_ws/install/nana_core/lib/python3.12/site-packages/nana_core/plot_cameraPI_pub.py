#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage

TOPIC_YOLO_IMAGE_COMPRESSED = "/yolo_cam/image/compressed"

class YoloCameraCompressedPublisher(Node):
    def __init__(self):
        super().__init__("yolo_camera_compressed_publisher")

        self.pub = self.create_publisher(CompressedImage, TOPIC_YOLO_IMAGE_COMPRESSED, 10)

        self.declare_parameter("camera_index", 2)
        self.declare_parameter("timer_period", 0.03)
        self.declare_parameter("width",        640)
        self.declare_parameter("height",       480)
        self.declare_parameter("jpeg_quality", 40)
        self.declare_parameter("preview",      False)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.width        = int(self.get_parameter("width").value)
        self.height       = int(self.get_parameter("height").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.preview      = bool(self.get_parameter("preview").value)

        # ใช้ CAP_V4L2 เพื่อให้เปิดกล้องได้
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {self.camera_index}")
            return

        self.get_logger().info(
            f"Camera opened (index={self.camera_index}, "
            f"{self.width}x{self.height}, quality={self.jpeg_quality})"
        )

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        frame_small = cv2.resize(frame, (self.width, self.height),
                                 interpolation=cv2.INTER_AREA)

        ok, enc = cv2.imencode(".jpg", frame_small,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return

        msg          = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format   = "jpeg"
        msg.data     = enc.tobytes()
        self.pub.publish(msg)

        if self.preview:
            cv2.imshow("YOLO Camera Publisher Preview", frame_small)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraCompressedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()