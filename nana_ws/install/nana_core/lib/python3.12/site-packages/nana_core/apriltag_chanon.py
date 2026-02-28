import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import cv2
import numpy as np
from pupil_apriltags import Detector

FOCAL_LENGTH = 400
KNOWN_TAG_SIZE_CM = 7.0


class AprilTagDistancePublisher(Node):

    def __init__(self):
        super().__init__('apriltag_distance_publisher')

        self.distance_pub = self.create_publisher(Float32, 'apriltag_distance', 10)
        self.position_pub = self.create_publisher(String, 'apriltag_position', 10)

        self.timer = self.create_timer(0.03, self.timer_callback)

        self.cap = cv2.VideoCapture(2)
        self.detector = Detector(families='tagStandard52h13')

        self.get_logger().info("AprilTag Distance + Position Publisher Started")

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            return

        frame_height, frame_width = frame.shape[:2]
        center_x = frame_width // 2
        deadzone = 40

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tag_results = self.detector.detect(gray)

        position_text = "NOT_FOUND"

        if tag_results:
            r = tag_results[0]

            # ===== Distance =====
            corners = r.corners
            pixel_width = np.linalg.norm(corners[0] - corners[1])

            if pixel_width > 0:
                distance_cm = (KNOWN_TAG_SIZE_CM * FOCAL_LENGTH) / pixel_width
                dist_msg = Float32()
                dist_msg.data = float(distance_cm)
                self.distance_pub.publish(dist_msg)

            # ===== Position =====
            tag_center_x = int(r.center[0])
            error = tag_center_x - center_x

            if abs(error) <= deadzone:
                position_text = "CENTER"
            elif error < 0:
                position_text = "LEFT"
            else:
                position_text = "RIGHT"

            pos_msg = String()
            pos_msg.data = position_text
            self.position_pub.publish(pos_msg)

            # Visualization
            cv2.circle(frame, (int(r.center[0]), int(r.center[1])), 6, (255, 0, 0), -1)
            cv2.putText(frame, position_text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            pos_msg = String()
            pos_msg.data = position_text
            self.position_pub.publish(pos_msg)

        # Draw center & deadzone
        cv2.line(frame, (center_x, 0), (center_x, frame_height), (0, 255, 0), 2)
        cv2.line(frame, (center_x - deadzone, 0), (center_x - deadzone, frame_height), (0, 0, 255), 1)
        cv2.line(frame, (center_x + deadzone, 0), (center_x + deadzone, frame_height), (0, 0, 255), 1)

        cv2.imshow("AprilTag Distance & Position", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()