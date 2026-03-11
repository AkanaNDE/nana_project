#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from pupil_apriltags import Detector


# =========================
# IMPORTANT:
# ต้องใส่ค่า intrinsics ของกล้องจริง
# fx, fy, cx, cy
# =========================
FX = 400.0
FY = 400.0

KNOWN_TAG_SIZE_CM = 7.0

TOPIC_IMAGE_COMPRESSED = "/camera/image/compressed"
TOPIC_DISTANCE = "/apriltag_distance"
TOPIC_POSITION = "/apriltag_position"
TOPIC_ANGLE = "/apriltag_angle"
STOP_SERVICE = "/apriltag/stop"


class AprilTagDistancePublisher(Node):

    def __init__(self):
        super().__init__("apriltag_distance_publisher")

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
            10
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
        except Exception:
            return

        self.latest_frame = frame

    def compute_true_tag_angle_deg(self, pose_R: np.ndarray) -> float:
        """
        มุม signed ระหว่างกล้องกับ AprilTag
        ซ้าย = ลบ
        ขวา = บวก
        """

        # normal ของ tag ใน camera frame
        tag_normal = pose_R[:, 2]

        # camera optical axis
        cam_axis = np.array([0.0, 0.0, 1.0])

        # magnitude ของมุม
        cos_theta = np.clip(np.dot(tag_normal, cam_axis) /
                            (np.linalg.norm(tag_normal)), -1.0, 1.0)

        angle = np.degrees(np.arccos(cos_theta))

        # ใช้แกน x ของ normal เพื่อตัดสินเครื่องหมาย
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
        center_x = frame_width // 2
        center_y = frame_height // 2

        cx = frame_width / 2.0
        cy = frame_height / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ใช้ pose estimation
        tag_results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(FX, FY, cx, cy),
            tag_size=KNOWN_TAG_SIZE_CM / 100.0  # ต้องเป็นเมตร
        )

        position_text = "NOT_FOUND"
        distance_cm = -1.0
        angle_deg = 0.0

        if tag_results:
            # ถ้ามีหลาย tag เลือกตัวที่ใกล้ที่สุด หรือจะเลือกตัวแรกก็ได้
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

            r = best_tag

            corners = r.corners
            tag_center_x = int(r.center[0])
            tag_center_y = int(r.center[1])

            # ===== distance จริงจาก pose_t =====
            if hasattr(r, "pose_t") and r.pose_t is not None:
                distance_cm = float(np.linalg.norm(r.pose_t) * 100.0)
            else:
                distance_cm = -1.0

            # ===== angle จริงจาก pose_R =====
            if hasattr(r, "pose_R") and r.pose_R is not None:
                angle_deg = self.compute_true_tag_angle_deg(r.pose_R)
            else:
                angle_deg = 0.0

            # ===== position เอาไว้บอกซ้าย/ขวาเฉย ๆ =====
            error = tag_center_x - center_x
            if abs(error) <= self.deadzone:
                position_text = "CENTER"
            elif error < 0:
                position_text = "LEFT"
            else:
                position_text = "RIGHT"

            # วาดกรอบ tag
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


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagDistancePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()