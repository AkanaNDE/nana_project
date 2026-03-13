#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from pupil_apriltags import Detector


OFFSET = 0
FX = 400.0
FY = 400.0

KNOWN_TAG_SIZE_CM = 7.0

TOPIC_IMAGE_COMPRESSED = "/yolo_cam/image/compressed"
TOPIC_DISTANCE = "/apriltag_distance"
TOPIC_POSITION = "/apriltag_position"
TOPIC_ANGLE = "/apriltag_angle"
STOP_SERVICE = "/apriltag/stop"


class AprilTagDistancePublisher2(Node):

    def __init__(self):
        super().__init__("apriltag_distance_publisher2")

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.distance_pub = self.create_publisher(Float32, TOPIC_DISTANCE, 10)
        self.position_pub = self.create_publisher(String, TOPIC_POSITION, 10)
        self.angle_pub = self.create_publisher(Float32, TOPIC_ANGLE, 10)

        self._running = True
        self.stop_srv = self.create_service(Trigger, STOP_SERVICE, self.handle_stop)

        self.detector = Detector(families="tagStandard52h13")

        self.deadzone = 40
        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.latest_frame = None

        self.sub = self.create_subscription(
            CompressedImage,
            TOPIC_IMAGE_COMPRESSED,
            self.image_callback,
            image_qos
        )

        self.get_logger().info("AprilTag Distance + Position + True Angle Publisher Started")

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
        response.message = "AprilTag node stopped."
        self.get_logger().warn(response.message)

        return response

    def image_callback(self, msg: CompressedImage):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            self.latest_frame = frame
        except Exception:
            return

    def compute_true_tag_angle_deg(self, pose_R: np.ndarray) -> float:

        tag_normal = pose_R[:, 2]
        cam_axis = np.array([0.0, 0.0, 1.0])

        cos_theta = np.clip(
            np.dot(tag_normal, cam_axis) / np.linalg.norm(tag_normal),
            -1.0,
            1.0
        )

        angle = np.degrees(np.arccos(cos_theta))

        if tag_normal[0] < 0:
            angle = -angle

        return float(angle)

    def timer_callback(self):

        if not self._running:
            return

        if self.latest_frame is None:
            self.publish_outputs("NOT_FOUND", -1.0, 0.0)
            return

        frame = self.latest_frame.copy()

        frame_height, frame_width = frame.shape[:2]

        center_x = frame_width // 2 + OFFSET
        center_y = frame_height // 2

        cx = frame_width / 2.0 + OFFSET
        cy = frame_height / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tag_results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(FX, FY, cx, cy),
            tag_size=KNOWN_TAG_SIZE_CM / 100.0
        )

        position_text = "NOT_FOUND"
        distance_cm = -1.0
        angle_deg = 0.0

        if tag_results:

            best_tag = None
            best_dist = 1e9

            for r in tag_results:

                if hasattr(r, "pose_t") and r.pose_t is not None:
                    dist_cm = float(np.linalg.norm(r.pose_t) * 100.0)
                else:
                    dist_cm = 1e9

                if dist_cm < best_dist:
                    best_dist = dist_cm
                    best_tag = r

            if best_tag is not None:

                r = best_tag

                corners = r.corners
                tag_center_x = int(r.center[0])
                tag_center_y = int(r.center[1])

                if hasattr(r, "pose_t") and r.pose_t is not None:
                    distance_cm = float(np.linalg.norm(r.pose_t) * 100.0)

                if hasattr(r, "pose_R") and r.pose_R is not None:
                    angle_deg = self.compute_true_tag_angle_deg(r.pose_R)

                error = tag_center_x - center_x

                if abs(error) <= self.deadzone:
                    position_text = "CENTER"
                elif error < 0:
                    position_text = "LEFT"
                else:
                    position_text = "RIGHT"

                pts = corners.astype(int).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 255), 2)

                cv2.circle(frame, (tag_center_x, tag_center_y), 6, (255, 0, 0), -1)

                cv2.putText(
                    frame,
                    f"{position_text} d={distance_cm:.1f}cm true_ang={angle_deg:.1f}deg",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"id={r.tag_id}",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2
                )

        self.publish_outputs(position_text, distance_cm, angle_deg)

        cv2.line(frame, (center_x, 0), (center_x, frame_height), (0, 255, 0), 2)
        cv2.line(frame, (0, center_y), (frame_width, center_y), (255, 255, 0), 1)

        cv2.line(frame, (center_x - self.deadzone, 0), (center_x - self.deadzone, frame_height), (0, 0, 255), 1)
        cv2.line(frame, (center_x + self.deadzone, 0), (center_x + self.deadzone, frame_height), (0, 0, 255), 1)

        cv2.imshow("AprilTag Distance Position True Angle", frame)
        cv2.waitKey(1)

    def publish_outputs(self, position, distance, angle):

        pos_msg = String()
        pos_msg.data = position
        self.position_pub.publish(pos_msg)

        dist_msg = Float32()
        dist_msg.data = float(distance)
        self.distance_pub.publish(dist_msg)

        angle_msg = Float32()
        angle_msg.data = float(angle)
        self.angle_pub.publish(angle_msg)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagDistancePublisher2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()