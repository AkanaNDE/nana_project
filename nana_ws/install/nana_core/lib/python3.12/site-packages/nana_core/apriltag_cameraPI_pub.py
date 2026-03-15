#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import re
from sensor_msgs.msg import CompressedImage

TOPIC_IMAGE_COMPRESSED = "/camera/image/compressed"
CAMERA_NAME = "UGREEN camera 2K"

def find_camera_index_by_name(target_name: str) -> int:
    try:
        for device in sorted(os.listdir('/sys/class/video4linux')):
            name_path = f'/sys/class/video4linux/{device}/name'
            if os.path.exists(name_path):
                with open(name_path, 'r') as f:
                    name = f.read().strip()
                if target_name.lower() in name.lower():
                    index = int(re.search(r'\d+', device).group())
                    return index
    except Exception as e:
        print(f"[find_camera] Error: {e}")
    return -1

class CameraCompressedPublisher(Node):
    def __init__(self):
        super().__init__("camera_compressed_publisher")
        self.pub = self.create_publisher(CompressedImage, TOPIC_IMAGE_COMPRESSED, 10)

        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("preview", False)

        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.preview      = bool(self.get_parameter("preview").value)

        self.camera_index = find_camera_index_by_name(CAMERA_NAME)
        if self.camera_index < 0:
            self.get_logger().error(f"Camera '{CAMERA_NAME}' not found")
            return

        self.get_logger().info(f"Found '{CAMERA_NAME}' at /dev/video{self.camera_index}")

        # *** แก้ไข 1: ส่ง MJPG ตรงๆ โดยไม่ decode/encode ซ้ำ
        #     multiqueue + leaky=downstream กันเฟรมค้าง
        #     sync=false ใน appsink ลด latency ***
        gst_pipeline = (
            f"v4l2src device=/dev/video{self.camera_index} io-mode=mmap "
            f"! image/jpeg,width=640,height=480,framerate=30/1 "
            f"! multiqueue max-size-buffers=1 leaky=downstream "
            f"! appsink drop=true max-buffers=1 sync=false"
        )

        self.get_logger().info(f"GStreamer pipeline: {gst_pipeline}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("GStreamer failed. Falling back to V4L2...")
            # *** แก้ไข 2: fallback เป็น V4L2 ถ้า GStreamer ไม่ได้ ***
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            # *** แก้ไข 3: ลด internal buffer ของ V4L2 เหลือ 1 เฟรม ***
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            return

        self.get_logger().info("Camera opened successfully.")

        # *** แก้ไข 4: ใช้ thread แยกอ่านเฟรม ไม่ให้ ROS timer block ***
        self.latest_frame = None
        import threading
        self._lock = threading.Lock()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30fps
        self.get_logger().info("Camera Compressed Publisher Started")

    def _capture_loop(self):
        """อ่านเฟรมใน thread แยก เพื่อไม่ให้กระตุกจาก ROS spin"""
        while rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                break
            ok, frame = self.cap.read()
            if ok and frame is not None:
                with self._lock:
                    self.latest_frame = frame

    def timer_callback(self):
        with self._lock:
            frame = self.latest_frame

        if frame is None:
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
            cv2.imshow("Camera Preview", frame)
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