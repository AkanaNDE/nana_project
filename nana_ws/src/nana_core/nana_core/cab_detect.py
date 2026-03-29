#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import csv
import os
from datetime import datetime

TOPIC_YOLO_IMAGE_COMPRESSED = "/yolo_cam/image/compressed"

class CabDetect(Node):
    def __init__(self):
        super().__init__('cab_detect')

        # Publishers
        self.size_pub = self.create_publisher(Float32, '/cab_size', 10)
        self.img_pub  = self.create_publisher(CompressedImage, "/yolo_image/compressed", 10)

        # โหลดโมเดล
        self.model = YOLO("/home/nadeem/nana_project/chanon/bestcab4.pt")

        # CONFIG
        self.CAMERA_HEIGHT  = 60.0
        self.PIXEL_CONSTANT = 970.0
        self.prev_size      = 0.0
        self.alpha          = 0.7

        # Frame buffer
        self.latest_frame = None
        self.frame_lock   = threading.Lock()

        # ── CSV Log setup ──
        log_dir  = os.path.expanduser("~/nana_project/logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path  = os.path.join(log_dir, f"cab_detect_{timestamp}.csv")

        self.log_file   = open(log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        # Header: เวลา, frame, box index, track id, cx, cy, w_px, h_px, diameter_px, diameter_cm
        self.csv_writer.writerow([
            'timestamp_s',
            'frame_index',
            'box_index',
            'track_id',
            'cx_px',
            'cy_px',
            'w_px',
            'h_px',
            'diameter_px',
            'diameter_cm'
        ])
        self.log_file.flush()

        self.frame_index = 0
        self.node_start_time = self.get_clock().now()

        self.get_logger().info(f"CSV log → {log_path}")

        # Subscribe ภาพจาก topic
        self.create_subscription(
            CompressedImage,
            TOPIC_YOLO_IMAGE_COMPRESSED,
            self.image_callback,
            10
        )

        # Inference thread
        self.inference_thread = threading.Thread(target=self.detect_loop, daemon=True)
        self.inference_thread.start()

        self.get_logger().info("Cab Detect Node Started (subscribing images)")

    # ===============================
    # Image Callback
    # ===============================
    def image_callback(self, msg: CompressedImage):
        try:
            buf   = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception:
            return
        with self.frame_lock:
            self.latest_frame = frame

    # ===============================
    # Detection Loop
    # ===============================
    def detect_loop(self):
        while rclpy.ok():
            with self.frame_lock:
                frame = self.latest_frame

            if frame is None:
                threading.Event().wait(0.01)
                continue

            results   = self.model.track(frame, persist=True, verbose=False)
            annotated = results[0].plot()

            # ── เวลาปัจจุบัน (วินาทีนับจากเริ่ม node) ──
            now_s = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9

            detected_size = 0.0
            first_box     = True   # ใช้ box แรกสำหรับ publish size

            if results[0].boxes is not None and len(results[0].boxes) > 0:
                for box_index, box in enumerate(results[0].boxes):

                    x, y, w, h = box.xywh[0].cpu().numpy()
                    cx = int(x)
                    cy = int(y)

                    diameter_px = (w + h) / 2.0
                    if diameter_px <= 0:
                        continue

                    diameter_cm = (diameter_px * self.CAMERA_HEIGHT) / self.PIXEL_CONSTANT

                    # ── Track ID (ถ้ามี) ──
                    track_id = int(box.id[0].cpu().numpy()) if box.id is not None else -1

                    # ── บันทึก 1 แถวต่อ 1 bounding box ──
                    self.csv_writer.writerow([
                        f"{now_s:.4f}",
                        self.frame_index,
                        box_index,
                        track_id,
                        cx,
                        cy,
                        f"{float(w):.2f}",
                        f"{float(h):.2f}",
                        f"{diameter_px:.2f}",
                        f"{diameter_cm:.4f}"
                    ])

                    # วาด annotation
                    color = (0, 255, 0) if box_index == 0 else (255, 165, 0)
                    cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(annotated,
                                f"[{box_index}] ID:{track_id} D={diameter_cm:.2f}cm",
                                (cx - 70, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    # ใช้ box แรกสำหรับ publish และ smoothing
                    if first_box:
                        detected_size = diameter_cm
                        first_box     = False

            # flush ทุก frame
            self.log_file.flush()
            self.frame_index += 1

            # Smoothing (ใช้ box แรก)
            size_filtered  = self.alpha * self.prev_size + (1 - self.alpha) * detected_size
            self.prev_size = size_filtered

            # Publish size
            size_msg      = Float32()
            size_msg.data = float(size_filtered)
            self.size_pub.publish(size_msg)

            # Publish annotated image
            ok, buffer = cv2.imencode('.jpg', annotated)
            if ok:
                img_msg              = CompressedImage()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.format       = "jpeg"
                img_msg.data         = buffer.tobytes()
                self.img_pub.publish(img_msg)

            cv2.imshow("Cab Detect", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    # ===============================
    # Destroy
    # ===============================
    def destroy_node(self):
        try:
            self.log_file.flush()
            self.log_file.close()
            self.get_logger().info("CSV log closed.")
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CabDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()