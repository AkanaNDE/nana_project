#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
import cv2
import numpy as np
from pupil_apriltags import Detector

class AprilTagDistancePublisher(Node):
    def __init__(self):
        super().__init__("apriltag_distance_publisher")

        # Configs
        self.focal_length = 400.0  # Adjust based on your camera calibration
        self.tag_size_cm = 7.0     # The actual physical size of your tag
        self.deadzone = 40         # Pixels for CENTER position

        # Publishers for Orchestrator
        self.dist_pub = self.create_publisher(Float32, "/apriltag_distance", 10)
        self.pos_pub  = self.create_publisher(String,  "/apriltag_position", 10)
        self.id_pub   = self.create_publisher(String,  "/apriltag_id", 10)

        # Subscriber to Pi 5
        self.sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, 10
        )

        self.detector = Detector(families="tagStandard52h13")
        self.get_logger().info("PC-Side AprilTag Subscriber Started")

    def image_callback(self, msg):
        # Decode JPEG back to OpenCV frame
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        if results:
            r = results[0]
            
            # 1. Calculate Distance
            # Using width of corners to estimate distance
            p_width = np.linalg.norm(r.corners[0] - r.corners[1])
            dist = (self.tag_size_cm * self.focal_length) / p_width
            
            # 2. Calculate Position
            cx = frame.shape[1] // 2
            error = r.center[0] - cx
            if abs(error) <= self.deadzone:
                pos = "CENTER"
            else:
                pos = "LEFT" if error < 0 else "RIGHT"

            # 3. Publish Data
            self.dist_pub.publish(Float32(data=float(dist)))
            self.pos_pub.publish(String(data=pos))
            self.id_pub.publish(String(data=str(r.tag_id))) # Sends the AB C DE raw ID

            # Visual Feedback (Optional - comment out for better performance)
            cv2.circle(frame, (int(r.center[0]), int(r.center[1])), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"ID:{r.tag_id} Dist:{dist:.1f}cm", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("PC Detection View", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AprilTagDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()