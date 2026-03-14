#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

# กำหนด Topic ชื่อเดียวกับที่คุณใช้
TOPIC_YOLO_IMAGE_COMPRESSED = "/yolo_cam/image/compressed"

class YoloCameraCompressedPublisher(Node):
    def __init__(self):
        super().__init__("yolo_camera_compressed_publisher")

        # --- การตั้งค่า Parameters ---
        self.declare_parameter("camera_index", 2)
        self.declare_parameter("timer_period", 0.033) # ~30 FPS
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("jpeg_quality", 40)
        self.declare_parameter("preview", False)

        # แก้ไขบรรทัดที่ 23-28 ให้เป็นแบบนี้ครับ:
        self.camera_index = self.get_parameter("camera_index").value
        self.timer_period = self.get_parameter("timer_period").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        self.preview = self.get_parameter("preview").value

        # --- เริ่มต้น Publisher ---
        self.pub = self.create_publisher(CompressedImage, TOPIC_YOLO_IMAGE_COMPRESSED, 10)

        # --- ตั้งค่ากล้อง MJPG ---
        # ใช้ CAP_V4L2 สำหรับ Linux เพื่อประสิทธิภาพที่ดีที่สุด
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        
        # บังคับใช้ Format MJPG
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"ไม่สามารถเปิดกล้อง Index {self.camera_index} ได้!")
        else:
            self.get_logger().info(f"เปิดกล้อง MJPG สำเร็จที่ความละเอียด {self.width}x{self.height}")

        # สร้าง Timer สำหรับอ่านภาพ
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        success, frame = self.cap.read()
        if not success or frame is None:
            self.get_logger().warn("อ่าน Frame จากกล้องไม่ได้")
            return

        # บีบอัดภาพเป็น JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        
        if result:
            # สร้างและส่ง Message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(encimg).tobytes()
            self.pub.publish(msg)

        # แสดงภาพ Preview (ถ้าเปิดไว้)
        if self.preview:
            cv2.imshow("YOLO Publisher Preview", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.preview = False
                cv2.destroyAllWindows()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraCompressedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("กำลังปิด Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()