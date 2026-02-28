import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import math
import numpy as np
from ultralytics import YOLO


class SegmentationSteeringNode(Node):

    def __init__(self):
        super().__init__('segmentation_steering_node')

        # Publisher
        self.publisher_ = self.create_publisher(String, '/steering_cmd', 10)

        # Load YOLO model
        self.model = YOLO("/home/nadeem/nana_project/chanon/rack_segm.pt")

        # Camera
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.deadzone_angle = 3.0

        # Timer (ประมาณ 30 FPS)
        self.timer = self.create_timer(0.03, self.process_frame)

        self.get_logger().info("Segmentation Steering Node Started")


    def process_frame(self):

        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        h, w, _ = frame.shape
        screen_cx = w // 2

        results = self.model.track(frame, persist=True, conf=0.5)

        command = "SEARCHING"
        angle_deg = 0.0

        if results[0].masks is not None and len(results[0].masks.xy) > 0:

            best_area = 0
            best_cx = None
            best_cy = None

            for polygon in results[0].masks.xy:

                pts = np.array(polygon, dtype=np.float32)
                area = cv2.contourArea(pts.astype(np.int32))

                if area > best_area:
                    M = cv2.moments(pts)

                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        best_area = area
                        best_cx = cx
                        best_cy = cy

            if best_cx is not None:

                dist_x = best_cx - screen_cx
                dist_y = h - best_cy

                if dist_y == 0:
                    dist_y = 1

                angle_rad = math.atan2(dist_x, dist_y)
                angle_deg = math.degrees(angle_rad)

                if angle_deg > self.deadzone_angle:
                    command = "RIGHT"
                elif angle_deg < -self.deadzone_angle:
                    command = "LEFT"
                else:
                    command = "FORWARD"

        # Publish command
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)

        # Debug log (optional)
        self.get_logger().info(f"Command: {command}")



    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()