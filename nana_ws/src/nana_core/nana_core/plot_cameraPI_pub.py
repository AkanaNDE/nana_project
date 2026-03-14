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

        # ตั้งค่า Parameter
        self.declare_parameter("camera_index", 5)
        self.declare_parameter("timer_period", 0.03)
        self.declare_parameter("width", 640) #ขนาดความกว้าง
        self.declare_parameter("height", 480) #นาดความสูง
        self.declare_parameter("jpeg_quality", 40)#คุณภาพไฟล์ลงเล็กน้อยเพื่อความลื่น
        self.declare_parameter("preview", False)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.preview = bool(self.get_parameter("preview").value)

        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.cap is None or (not self.cap.isOpened()):
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        # ทำการ Resize ภาพให้เล็กลงตามที่ตั้งค่าไว้
        frame_small = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        # Encode ภาพที่ถูก Resize แล้ว
        ok, enc = cv2.imencode(".jpg", frame_small, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = enc.tobytes()
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