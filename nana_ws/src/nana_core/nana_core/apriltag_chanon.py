import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import cv2
import numpy as np
from pupil_apriltags import Detector


FOCAL_LENGTH = 400
KNOWN_TAG_SIZE_CM = 7.0


class AprilTagDistancePublisher(Node):

    def __init__(self):
        super().__init__('apriltag_distance_publisher')
        
        self.publisher_ = self.create_publisher(Float32, 'apriltag_distance', 10)
        
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
        
        self.cap = cv2.VideoCapture(0)
        self.detector = Detector(families='tagStandard52h13')

        self.get_logger().info("AprilTag Distance Publisher Started")

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tag_results = self.detector.detect(gray)

        distance_cm = None

        if tag_results:
            r = tag_results[0]
            corners = r.corners
            pixel_width = np.linalg.norm(corners[0] - corners[1])

            if pixel_width > 0:
                distance_cm = (KNOWN_TAG_SIZE_CM * FOCAL_LENGTH) / pixel_width

                msg = Float32()
                msg.data = float(distance_cm)
                self.publisher_.publish(msg)

                self.get_logger().info(f"Distance: {distance_cm:.2f} cm")

                # Draw visualization
                tag_center_x = int(r.center[0])
                tag_center_y = int(r.center[1])
                cv2.circle(frame, (tag_center_x, tag_center_y), 6, (255, 0, 0), -1)

        cv2.imshow("AprilTag Distance", frame)
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