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

        self.declare_parameter("timer_period", 0.03)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("preview", False)

        self.timer_period = float(self.get_parameter("timer_period").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.preview      = bool(self.get_parameter("preview").value)

        self.camera_index = find_camera_index_by_name(CAMERA_NAME)
        if self.camera_index < 0:
            self.get_logger().error(f"Camera '{CAMERA_NAME}' not found in /sys/class/video4linux")
            return

        self.get_logger().info(f"Found '{CAMERA_NAME}' at /dev/video{self.camera_index}")

        # *** เปิดกล้องด้วย GStreamer pipeline (MJPG → jpegdec → videoconvert) ***
        gst_pipeline = (
            f"v4l2src device=/dev/video{self.camera_index} "
            f"! image/jpeg,width=640,height=480,framerate=30/1 "
            f"! jpegdec "
            f"! videoconvert "
            f"! appsink drop=true max-buffers=1"
        )
        self.get_logger().info(f"GStreamer pipeline: {gst_pipeline}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera via GStreamer. Check gstreamer plugins.")
            return

        self.get_logger().info(f"Camera opened via GStreamer at /dev/video{self.camera_index}")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Camera Compressed Publisher Started")

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
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