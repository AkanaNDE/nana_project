#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String
from pupil_apriltags import Detector


FOCAL_LENGTH = 400.0
KNOWN_TAG_SIZE_CM = 7.0


class AprilTagProcessorPC(Node):

    def __init__(self):
        super().__init__('apriltag_processor_pc')

        self.distance_pub = self.create_publisher(Float32, '/apriltag_distance', 10)
        self.position_pub = self.create_publisher(String, '/apriltag_position', 10)

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10
        )

        self.detector = Detector(families="tagStandard52h13")

        self.deadzone = 40

        self.get_logger().info("PC Processor Node Started - Waiting for images from Pi...")

    def image_callback(self, msg: CompressedImage):

        frame = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        if frame is None:
            return

        h, w = frame.shape[:2]
        center_x = w // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)

        position_text = "NOT_FOUND"
        distance_cm = -1.0

        if tags:
            r = tags[0]

            corners = r.corners
            w1 = np.linalg.norm(corners[0] - corners[1])
            w2 = np.linalg.norm(corners[2] - corners[3])
            pixel_width = (w1 + w2) / 2.0

            if pixel_width > 1e-6:
                distance_cm = (KNOWN_TAG_SIZE_CM * FOCAL_LENGTH) / pixel_width

            tag_center_x = int(r.center[0])
            error = tag_center_x - center_x

            if abs(error) <= self.deadzone:
                position_text = "CENTER"
            elif error < 0:
                position_text = "LEFT"
            else:
                position_text = "RIGHT"

            # preview
            cv2.circle(frame, (int(r.center[0]), int(r.center[1])), 6, (255, 0, 0), -1)
            cv2.putText(frame,
                        f"{position_text}  d={distance_cm:.1f}cm",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2)

        # publish
        self.publish_outputs(position_text, distance_cm)

        cv2.imshow("AprilTag (PC)", frame)
        cv2.waitKey(1)

    def publish_outputs(self, position, distance):
        pos_msg = String()
        pos_msg.data = position
        self.position_pub.publish(pos_msg)

        dist_msg = Float32()
        dist_msg.data = float(distance)
        self.distance_pub.publish(dist_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagProcessorPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()