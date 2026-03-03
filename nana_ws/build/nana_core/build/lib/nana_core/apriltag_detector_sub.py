#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

# Config เหมือนเดิม
FOCAL_LENGTH = 400.0
KNOWN_TAG_SIZE_CM = 7.0

class AprilTagProcessor(Node):
    def __init__(self):
        super().__init__("apriltag_processor_pc")
        
        # Subscriber รับภาพจาก Pi
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers ผลลัพธ์
        self.distance_pub = self.create_publisher(Float32, "/apriltag_distance", 10)
        self.position_pub = self.create_publisher(String, "/apriltag_position", 10)
        
        self.bridge = CvBridge()
        self.detector = Detector(families="tagStandard52h13")
        self.deadzone = 40
        self.get_logger().info("PC Processor Node Started - Waiting for images from Pi...")

    def image_callback(self, msg):
        # แปลง ROS Image เป็น OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        frame_height, frame_width = frame.shape[:2]
        center_x = frame_width // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tag_results = self.detector.detect(gray)

        position_text = "NOT_FOUND"
        distance_cm = -1.0

        if tag_results:
            r = tag_results[0]
            # คำนวณ Distance
            corners = r.corners
            w1 = np.linalg.norm(corners[0] - corners[1])
            w2 = np.linalg.norm(corners[2] - corners[3])
            pixel_width = (w1 + w2) / 2.0
            if pixel_width > 1e-6:
                distance_cm = (KNOWN_TAG_SIZE_CM * FOCAL_LENGTH) / pixel_width

            # คำนวณ Position
            tag_center_x = int(r.center[0])
            error = tag_center_x - center_x
            if abs(error) <= self.deadzone:
                position_text = "CENTER"
            elif error < 0:
                position_text = "LEFT"
            else:
                position_text = "RIGHT"

            # Draw (แสดงผลบนหน้าจอ PC)
            cv2.circle(frame, (int(r.center[0]), int(r.center[1])), 6, (255, 0, 0), -1)
            cv2.putText(frame, f"{position_text} d={distance_cm:.1f}cm", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Publish ผลลัพธ์
        self.publish_outputs(position_text, distance_cm)

        # Visualization
        cv2.line(frame, (center_x, 0), (center_x, frame_height), (0, 255, 0), 2)
        cv2.imshow("PC - AprilTag Detection", frame)
        cv2.waitKey(1)

    def publish_outputs(self, pos, dist):
        p_msg, d_msg = String(), Float32()
        p_msg.data, d_msg.data = pos, float(dist)
        self.position_pub.publish(p_msg)
        self.distance_pub.publish(d_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagProcessor()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()