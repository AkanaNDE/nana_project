#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

class MissionState:
    APRILTAG_TRACK          = 'APRILTAG_TRACK'
    STOP_AND_DECODE         = 'STOP_AND_DECODE'
    DRIVE_DISTANCE_AB       = 'DRIVE_DISTANCE_AB'  # Using Encoder for AB distance
    TURN_LEFT_1S            = 'TURN_LEFT_1S'
    FORWARD_GAP_C           = 'FORWARD_GAP_C'       # Using Encoder for C gap
    TURN_RIGHT_1S           = 'TURN_RIGHT_1S'  
    BACKWARD_INTERVAL_DE    = 'BACKWARD_INTERVAL_DE' # Using Encoder for DE interval
    PLOT_TRACK              = 'PLOT_TRACK'

class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')

        # --- Publishers ---
        # Note: Sending to /nana/cmd_arm as per your setup
        self.cmd_pub = self.create_publisher(Twist, '/nana/cmd_arm', qos_profile_system_default)

        # --- Subscribers ---
        self.create_subscription(String,  '/apriltag_position',  self.apr_pos_cb,  10)
        self.create_subscription(Float32, '/apriltag_distance', self.apr_dist_cb, 10)
        self.create_subscription(String,  '/apriltag_id',       self.apr_id_cb,   10)
        self.create_subscription(Odometry, '/odom',             self.odom_cb,     qos_profile_system_default)
        self.create_subscription(String,  '/plot_direction',    self.plot_cb,     10)

        # --- Service Client ---
        self.stop_client = self.create_client(Trigger, '/apriltag/stop')

        # --- State Logic Variables ---
        self.state = MissionState.APRILTAG_TRACK
        self.state_enter_time = time.monotonic()
        
        # AprilTag Data
        self.apr_pos = 'NOT_FOUND'
        self.apr_dist = -1.0
        
        # Plot Data
        self.plot_dir = 'NOT_FOUND'
        
        # Encoder/Odom Data
        self.current_odom_x = 0.0
        self.start_odom_x = 0.0
        
        # Decoded Values from AB C DE
        self.dist_ab = 0.0
        self.gap_c = 0.0
        self.interval_de = 0.0

        # --- Tuning Params ---
        self.stop_distance_cm = 65.0
        self.forward_speed = 0.35
        self.turn_speed = 0.35

        # --- Main Timer (50Hz) ---
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Mission Orchestrator (PC-Side) Started.")

    # ---------------- Callbacks ----------------
    def odom_cb(self, msg):
        # Extract X position from Odometry (Encoder)
        self.current_odom_x = msg.pose.pose.position.x

    def apr_id_cb(self, msg):
        """ Decodes AB C DE from tag ID string (e.g., '20315') """
        code = msg.data
        if len(code) >= 5:
            try:
                # AB: First two digits (cm)
                self.dist_ab = float(code[0:2])
                # C: 1=5cm, 2=10cm, 3=15cm, 4=20cm, 5=25cm
                gap_map = {'1': 5.0, '2': 10.0, '3': 15.0, '4': 20.0, '5': 25.0}
                self.gap_c = gap_map.get(code[2], 0.0)
                # DE: Last two digits (cm)
                self.interval_de = float(code[3:5])
                
                self.get_logger().info(f"TAG DECODED -> AB: {self.dist_ab}cm, C: {self.gap_c}cm, DE: {self.interval_de}cm")
            except Exception as e:
                self.get_logger().error(f"Failed to decode Tag ID: {e}")

    def apr_pos_cb(self, msg): self.apr_pos = msg.data
    def apr_dist_cb(self, msg): self.apr_dist = float(msg.data)
    def plot_cb(self, msg): self.plot_dir = msg.data

    # ---------------- Helpers ----------------
    def publish_cmd(self, linear_y, angular_z):
        t = Twist()
        t.linear.y = float(linear_y)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    def get_distance_traveled_cm(self):
        # Converts Odom meters to CM
        return abs(self.current_odom_x - self.start_odom_x) * 100.0

    def enter_state(self, new_state):
        self.get_logger().info(f"Transitioning: {self.state} -> {new_state}")
        self.state = new_state
        self.state_enter_time = time.monotonic()
        self.start_odom_x = self.current_odom_x # Reset encoder baseline

    # ---------------- State Machine Behaviors ----------------
    def do_apriltag_track(self):
        # Trigger transition if close to Tag
        if 0 < self.apr_dist < self.stop_distance_cm:
            self.publish_cmd(0.0, 0.0)
            self.enter_state(MissionState.STOP_AND_DECODE)
            return

        # Simple Vision Tracking
        if self.apr_pos == 'CENTER':
            self.publish_cmd(self.forward_speed, 0.0)
        elif self.apr_pos == 'LEFT':
            self.publish_cmd(0.0, self.turn_speed)
        elif self.apr_pos == 'RIGHT':
            self.publish_cmd(0.0, -self.turn_speed)
        else:
            self.publish_cmd(0.0, 0.2) # Slow search turn

    def do_stop_and_decode(self):
        self.publish_cmd(0.0, 0.0)
        # Give 1 second to ensure ID is received and robot is still
        if (time.monotonic() - self.state_enter_time) > 1.0:
            # Optionally stop the tag node to save CPU
            if self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
            self.enter_state(MissionState.DRIVE_DISTANCE_AB)

    def do_drive_distance_ab(self):
        # Precision driving using Encoder
        if self.get_distance_traveled_cm() < self.dist_ab:
            self.publish_cmd(self.forward_speed, 0.0)
        else:
            self.publish_cmd(0.0, 0.0)
            self.enter_state(MissionState.TURN_LEFT_1S)

    def do_turn_left_1s(self):
        if (time.monotonic() - self.state_enter_time) < 1.5: # Adjust time for ~90 deg
            self.publish_cmd(0.0, self.turn_speed)
        else:
            self.enter_state(MissionState.FORWARD_GAP_C)

    def do_forward_gap_c(self):
        # Move forward based on decoded 'C' gap
        if self.get_distance_traveled_cm() < self.gap_c:
            self.publish_cmd(self.forward_speed, 0.0)
        else:
            self.enter_state(MissionState.TURN_RIGHT_1S)

    def do_turn_right_1s(self):
        if (time.monotonic() - self.state_enter_time) < 1.5:
            self.publish_cmd(0.0, -self.turn_speed)
        else:
            self.enter_state(MissionState.BACKWARD_INTERVAL_DE)

    def do_backward_interval_de(self):
        # Move backward based on decoded 'DE' interval
        if self.get_distance_traveled_cm() < self.interval_de:
            self.publish_cmd(-self.forward_speed, 0.0) # Negative Y = backward
        else:
            self.enter_state(MissionState.PLOT_TRACK)

    def do_plot_track(self):
        d = self.plot_dir
        if d == 'CENTER':
            self.publish_cmd(self.forward_speed, 0.0)
        elif d == 'LEFT':
            self.publish_cmd(0.0, self.turn_speed)
        elif d == 'RIGHT':
            self.publish_cmd(0.0, -self.turn_speed)
        else:
            self.publish_cmd(0.0, 0.0) # Stop if plot lost

    # ---------------- Main Loop ----------------
    def control_loop(self):
        if self.state == MissionState.APRILTAG_TRACK:
            self.do_apriltag_track()
        elif self.state == MissionState.STOP_AND_DECODE:
            self.do_stop_and_decode()
        elif self.state == MissionState.DRIVE_DISTANCE_AB:
            self.do_drive_distance_ab()
        elif self.state == MissionState.TURN_LEFT_1S:
            self.do_turn_left_1s()
        elif self.state == MissionState.FORWARD_GAP_C:
            self.do_forward_gap_c()
        elif self.state == MissionState.TURN_RIGHT_1S:
            self.do_turn_right_1s()
        elif self.state == MissionState.BACKWARD_INTERVAL_DE:
            self.do_backward_interval_de()
        elif self.state == MissionState.PLOT_TRACK:
            self.do_plot_track()

def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()