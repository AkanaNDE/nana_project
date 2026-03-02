#!/usr/bin/env python3
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

        # Publisher (ให้ orchestrator ใช้งานได้ทันที)
        self.publisher_ = self.create_publisher(String, '/plot_direction', 10)

        # Load YOLO model
        self.model = YOLO("/home/nadeem/nana_project/chanon/rack_segm.pt")

        # Camera
        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.deadzone_angle = 3.0

        # Timer (ประมาณ 30 FPS)
        self.timer = self.create_timer(0.03, self.process_frame)

        self.get_logger().info("Segmentation Steering Node Started (with preview)")


    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        h, w, _ = frame.shape
        screen_cx = w // 2

        results = self.model.track(frame, persist=True, conf=0.5)

        # ===== logic เดิมของคุณ (ไม่เปลี่ยน) =====
        command = "SEARCHING"
        angle_deg = 0.0

        best_cx = None
        best_cy = None
        best_poly = None

        if results[0].masks is not None and len(results[0].masks.xy) > 0:

            best_area = 0

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
                        best_poly = pts.astype(np.int32)

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

        # ===== ทำให้ compatible กับ orchestrator (ไม่เปลี่ยน logic แค่แปลคำ) =====
        if command == "FORWARD":
            out_cmd = "CENTER"
        elif command == "SEARCHING":
            out_cmd = "NOT_FOUND"
        else:
            out_cmd = command  # LEFT / RIGHT

        # Publish command
        msg = String()
        msg.data = out_cmd
        self.publisher_.publish(msg)

        # ====== Visualization (Preview Window) ======
        vis = frame.copy()

        # เส้นกลางจอ
        cv2.line(vis, (screen_cx, 0), (screen_cx, h), (0, 255, 0), 2)

        # deadzone angle text
        cv2.putText(vis, f"deadzone_angle={self.deadzone_angle:.1f} deg",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # วาด polygon ที่ใหญ่สุด + centroid
        if best_poly is not None:
            cv2.polylines(vis, [best_poly], isClosed=True, color=(255, 0, 0), thickness=2)

        if best_cx is not None and best_cy is not None:
            cv2.circle(vis, (best_cx, best_cy), 6, (0, 0, 255), -1)
            cv2.line(vis, (screen_cx, h), (best_cx, best_cy), (0, 255, 255), 2)

        # แสดงคำสั่ง + มุม
        cv2.putText(vis, f"Command: {out_cmd}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(vis, f"Angle: {angle_deg:.1f} deg", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow("Plot Segmentation Steering Preview", vis)
        cv2.waitKey(1)

        # Debug log (optional)
        # (ลด spam: ใช้ debug-level ก็ได้ แต่คงไว้ตามเดิม)
        self.get_logger().info(f"Command: {out_cmd}")


    def destroy_node(self):
        try:
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
    node = SegmentationSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()