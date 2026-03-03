#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from pupil_apriltags import Detector


FOCAL_LENGTH = 400.0
KNOWN_TAG_SIZE_CM = 7.0

TOPIC_IMAGE_COMPRESSED = "/camera/image/compressed"
TOPIC_DISTANCE = "/apriltag_distance"
TOPIC_POSITION = "/apriltag_position"
STOP_SERVICE   = "/apriltag/stop"


class AprilTagDistancePublisher(Node):
    def __init__(self):
        super().__init__("apriltag_distance_publisher")

        self.distance_pub = self.create_publisher(Float32, TOPIC_DISTANCE, 10)
        self.position_pub = self.create_publisher(String,  TOPIC_POSITION, 10)

        self._running = True
        self.stop_srv = self.create_service(Trigger, STOP_SERVICE, self.handle_stop)

        self.detector = Detector(families="tagStandard52h13")

        self.deadzone = 40
        self.timer_period = 0.03
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.latest_frame = None
        self.sub = self.create_subscription(
            CompressedImage, TOPIC_IMAGE_COMPRESSED, self.image_callback, 10
        )

        self.get_logger().info("AprilTag Distance + Position Publisher Started (CompressedImage)")

    def handle_stop(self, request, response):
        self._running = False
        try:
            if self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        response.success = True
        response.message = "AprilTag node stopped (timer canceled)."
        self.get_logger().warn(response.message)
        return response

    def image_callback(self, msg: CompressedImage):
        # Decode JPEG -> BGR frame
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception:
            return
        self.latest_frame = frame

    def timer_callback(self):
        if not self._running:
            return

        if self.latest_frame is None:
            self.publish_outputs(position="NOT_FOUND", distance=-1.0)
            return

        frame = self.latest_frame
        frame_height, frame_width = frame.shape[:2]
        center_x = frame_width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tag_results = self.detector.detect(gray)

        position_text = "NOT_FOUND"
        distance_cm = -1.0

        if tag_results:
            r = tag_results[0]

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

            cv2.circle(frame, (int(r.center[0]), int(r.center[1])), 6, (255, 0, 0), -1)
            cv2.putText(frame, f"{position_text}  d={distance_cm:.1f}cm", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        self.publish_outputs(position=position_text, distance=distance_cm)

        cv2.line(frame, (center_x, 0), (center_x, frame_height), (0, 255, 0), 2)
        cv2.line(frame, (center_x - self.deadzone, 0), (center_x - self.deadzone, frame_height), (0, 0, 255), 1)
        cv2.line(frame, (center_x + self.deadzone, 0), (center_x + self.deadzone, frame_height), (0, 0, 255), 1)

        cv2.imshow("AprilTag Distance & Position", frame)
        cv2.waitKey(1)

    def publish_outputs(self, position: str, distance: float):
        pos_msg = String()
        pos_msg.data = position
        self.position_pub.publish(pos_msg)

        dist_msg = Float32()
        dist_msg.data = float(distance)
        self.distance_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()