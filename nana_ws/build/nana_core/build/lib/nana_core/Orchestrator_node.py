#!/usr/bin/env python3
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class MissionState:
    WAIT_FOR_ANGLE               = 'WAIT_FOR_ANGLE'
    STAGE1_TURN_LEFT_2S          = 'STAGE1_TURN_LEFT_2S'
    STAGE1_FORWARD_4S            = 'STAGE1_FORWARD_4S'
    STAGE1_TURN_RIGHT_TO_CENTER  = 'STAGE1_TURN_RIGHT_TO_CENTER'
    STAGE2_TRACK_TO_30CM         = 'STAGE2_TRACK_TO_30CM'
    STAGE3_TURN_LEFT_3S          = 'STAGE3_TURN_LEFT_3S'
    STAGE3_FORWARD_1S            = 'STAGE3_FORWARD_1S'
    STAGE4_CLEANUP_CHANON        = 'STAGE4_CLEANUP_CHANON'
    STAGE4_STOP_BEFORE_CHANON    = 'STAGE4_STOP_BEFORE_CHANON'
    STAGE4_RUN_PLOT_CHANON       = 'STAGE4_RUN_PLOT_CHANON'
    STAGE4_WAIT_CENTER_1S        = 'STAGE4_WAIT_CENTER_1S'
    STAGE4_FORWARD_2S            = 'STAGE4_FORWARD_2S'
    STAGE4_SWITCH_TO_DETECTOR    = 'STAGE4_SWITCH_TO_DETECTOR'
    STAGE4_WAIT_FOR_DETECTOR_TOPIC = 'STAGE4_WAIT_FOR_DETECTOR_TOPIC'
    STAGE4_BACKWARD_UNTIL_CENTER = 'STAGE4_BACKWARD_UNTIL_CENTER'
    STAGE4_CHECK_DETECTOR_CENTER = 'STAGE4_CHECK_DETECTOR_CENTER'
    STAGE4_WAIT_ENCODER_READY    = 'STAGE4_WAIT_ENCODER_READY'   # *** NEW: รอ encoder_distance พร้อม ***
    STAGE5_FORWARD_BY_AB         = 'STAGE5_FORWARD_BY_AB'
    STAGE4_FINAL_SWITCH_CHANON   = 'STAGE4_FINAL_SWITCH_CHANON'
    DONE                         = 'DONE'

class MissionOrchestrator(Node):

    def __init__(self):
        super().__init__('mission_orchestrator')

        self.cmd_pub = self.create_publisher(Twist, '/nana/cmd_arm', qos_profile_system_default)

        # --- Subscriptions ---
        self.create_subscription(Float32, '/apriltag_angle', self.apr_angle_cb, 10)
        self.create_subscription(String, '/apriltag_position', self.apr_pos_cb, 10)
        self.create_subscription(String, '/apriltag_position_front', self.apr_front_cb, 10)
        self.create_subscription(Float32, '/apriltag_distance', self.apr_distance_cb, 10)
        self.create_subscription(String, '/plot_direction', self.plot_dir_cb, 10)
        self.create_subscription(String, '/apriltag_id', self.apr_id_cb, 10)
        self.create_subscription(Float32, '/current_distance_cm', self.current_dist_cb, 10)

        # --- Data Variables ---
        self.state = MissionState.WAIT_FOR_ANGLE
        self.state_enter_time = time.monotonic()

        self.apr_angle = 999.0
        self.apr_angle_received = False
        self.apr_pos = 'NOT_FOUND'
        self.apr_pos_received_in_stage = False
        self.apr_front = 'NOT_FOUND'
        self.apr_distance = -1.0
        self.plot_direction = 'NONE'
        
        self.target_ab_dist = 0.0   
        self.gap_c_val = 0.0        
        self.interval_de = 0.0      
        
        self.start_odom_dist = 0.0
        self.current_odom_dist = 0.0

        # *** NEW: flag สำหรับรอค่าแรกจาก /current_distance_cm หลังรัน encoder_distance ***
        self.encoder_first_value_received = False

        self.turn_speed = 0.35
        self.forward_speed = 0.25
        self.backward_speed = -0.15 
        self.stop_distance_cm = 60.0

        self.proc_apriltag = None
        self.proc_plot_chanon = None
        self.proc_plot_detector = None
        self.proc_encoder_distance = None   # *** NEW: process handle สำหรับ encoder_distance ***

        self.start_apriltag_detector_sub()
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Mission Orchestrator Started (Jazzy Version).")

# ---------------- Callbacks ----------------

    def apr_id_cb(self, msg):
        tag = msg.data
        if len(tag) == 5 and tag.isdigit():
            self.target_ab_dist = float(tag[0:2])
            gap_map = {'1': 5, '2': 10, '3': 15, '4': 20, '5': 25}
            self.gap_c_val = gap_map.get(tag[2], 0.0)
            self.interval_de = float(tag[3:5])
            self.get_logger().info(f"Tag Parsed: AB={self.target_ab_dist}cm, Gap={self.gap_c_val}cm, DE={self.interval_de}cm")

    def current_dist_cb(self, msg):
        self.current_odom_dist = msg.data

        # *** NEW: บันทึกค่าแรกที่รับได้จาก topic หลังเปลี่ยนเข้า STAGE4_WAIT_ENCODER_READY ***
        if self.state == MissionState.STAGE4_WAIT_ENCODER_READY and not self.encoder_first_value_received:
            self.start_odom_dist = msg.data
            self.encoder_first_value_received = True
            self.get_logger().info(f"Encoder first value captured: start_odom_dist = {self.start_odom_dist:.2f} cm")

    def apr_angle_cb(self, msg): self.apr_angle = float(msg.data); self.apr_angle_received = True
    
    def apr_pos_cb(self, msg): 
        self.apr_pos = msg.data
        self.apr_pos_received_in_stage = True

    def apr_front_cb(self, msg): self.apr_front = msg.data
    def apr_distance_cb(self, msg): self.apr_distance = float(msg.data)
    def plot_dir_cb(self, msg): self.plot_direction = msg.data

# ---------------- Process Helpers ----------------

    def run_ros2_node(self, package_name, executable_name):
        cmd = f"source /opt/ros/jazzy/setup.bash && source ~/nana_project/nana_ws/install/setup.bash && ros2 run {package_name} {executable_name}"
        return subprocess.Popen(cmd, shell=True, executable="/bin/bash")

    def stop_process(self, proc, name="process"):
        if proc is None: return None
        subprocess.run(["pkill", "-9", "-f", name], stderr=subprocess.DEVNULL)
        if proc.poll() is None:
            proc.terminate()
        time.sleep(0.5)
        return None

    def start_apriltag_detector_sub(self):
        if self.proc_apriltag is None or self.proc_apriltag.poll() is not None:
            self.proc_apriltag = self.run_ros2_node("nana_core", "apriltag_detector_sub")

    def publish_cmd(self, linear_y, angular_z):
        t = Twist()
        t.linear.y = float(linear_y)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    def enter_state(self, new_state):
        self.get_logger().info(f"Transition: {self.state} -> {new_state}")
        self.state = new_state
        self.state_enter_time = time.monotonic()
        
        if new_state == MissionState.STAGE4_WAIT_FOR_DETECTOR_TOPIC:
            self.apr_pos_received_in_stage = False

        # *** NEW: รีเซ็ต flag เมื่อเข้าสู่สถานะรอ encoder ***
        if new_state == MissionState.STAGE4_WAIT_ENCODER_READY:
            self.encoder_first_value_received = False

        # *** MODIFIED: ลบการบันทึก start_odom_dist ออกจากที่นี่
        #     (ย้ายไปบันทึกใน current_dist_cb เมื่อรับค่าแรกได้แทน) ***

    def elapsed_in_state(self):
        return time.monotonic() - self.state_enter_time

# ---------------- Control Loop ----------------

    def control_loop(self):
        logic_map = {
            MissionState.WAIT_FOR_ANGLE: self.do_wait_for_angle,
            MissionState.STAGE1_TURN_LEFT_2S: self.do_stage1_turn_left_2s,
            MissionState.STAGE1_FORWARD_4S: self.do_stage1_forward_4s,
            MissionState.STAGE1_TURN_RIGHT_TO_CENTER: self.do_stage1_turn_right_to_center,
            MissionState.STAGE2_TRACK_TO_30CM: self.do_stage2_track_to_30cm,
            MissionState.STAGE3_TURN_LEFT_3S: self.do_stage3_turn_left_3s,
            MissionState.STAGE3_FORWARD_1S: self.do_stage3_forward_1s,
            MissionState.STAGE4_CLEANUP_CHANON: self.do_stage4_cleanup_chanon,
            MissionState.STAGE4_STOP_BEFORE_CHANON: self.do_stage4_stop_before_chanon,
            MissionState.STAGE4_RUN_PLOT_CHANON: self.do_stage4_run_plot_chanon,
            MissionState.STAGE4_WAIT_CENTER_1S: self.do_stage4_wait_center_1s,
            MissionState.STAGE4_FORWARD_2S: self.do_stage4_forward_2s,
            MissionState.STAGE4_SWITCH_TO_DETECTOR: self.do_stage4_switch_to_detector,
            MissionState.STAGE4_WAIT_FOR_DETECTOR_TOPIC: self.do_wait_for_detector_topic,
            MissionState.STAGE4_BACKWARD_UNTIL_CENTER: self.do_stage4_backward_until_center,
            MissionState.STAGE4_CHECK_DETECTOR_CENTER: self.do_stage4_check_detector_center,
            MissionState.STAGE4_WAIT_ENCODER_READY: self.do_stage4_wait_encoder_ready,  # *** NEW ***
            MissionState.STAGE5_FORWARD_BY_AB: self.do_stage5_forward_by_ab,
            MissionState.STAGE4_FINAL_SWITCH_CHANON: self.do_stage4_final_switch_chanon,
            MissionState.DONE: self.do_done
        }
        handler = logic_map.get(self.state)
        if handler: handler()

# ---------------- Logic Handlers ----------------

    def do_wait_for_angle(self):
        if not self.apr_angle_received: return self.publish_cmd(0, 0)
        self.enter_state(MissionState.STAGE2_TRACK_TO_30CM) if self.apr_angle < 0 else self.enter_state(MissionState.STAGE1_TURN_LEFT_2S)

    def do_stage1_turn_left_2s(self):
        if self.elapsed_in_state() >= 5: self.enter_state(MissionState.STAGE1_FORWARD_4S)
        else: self.publish_cmd(0, self.turn_speed)

    def do_stage1_forward_4s(self):
        if self.elapsed_in_state() >= 9: self.enter_state(MissionState.STAGE1_TURN_RIGHT_TO_CENTER)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_stage1_turn_right_to_center(self):
        if self.apr_front == 'CENTER': self.enter_state(MissionState.STAGE2_TRACK_TO_30CM)
        else: self.publish_cmd(0, -self.turn_speed)

    def do_stage2_track_to_30cm(self):
        if 0 < self.apr_distance <= self.stop_distance_cm:
            self.enter_state(MissionState.STAGE3_TURN_LEFT_3S)
        elif self.apr_front == 'CENTER': self.publish_cmd(self.forward_speed, 0)
        elif self.apr_front == 'LEFT': self.publish_cmd(0, self.turn_speed)
        elif self.apr_front == 'RIGHT': self.publish_cmd(0, -self.turn_speed)
        else: self.publish_cmd(0, 0)

    def do_stage3_turn_left_3s(self):
        if self.elapsed_in_state() >= 2: self.enter_state(MissionState.STAGE3_FORWARD_1S)
        else: self.publish_cmd(0, self.turn_speed)

    def do_stage3_forward_1s(self):
        if self.elapsed_in_state() >= 3: self.enter_state(MissionState.STAGE4_CLEANUP_CHANON)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_stage4_cleanup_chanon(self):
        if self.proc_apriltag: self.proc_apriltag = self.stop_process(self.proc_apriltag, "apriltag_detector_sub")
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 1: self.enter_state(MissionState.STAGE4_STOP_BEFORE_CHANON)

    def do_stage4_stop_before_chanon(self):
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 1: self.enter_state(MissionState.STAGE4_RUN_PLOT_CHANON)

    def do_stage4_run_plot_chanon(self):
        if self.proc_plot_chanon is None:
            self.proc_plot_chanon = self.run_ros2_node("nana_core", "plot_chanon")
            self.enter_state(MissionState.STAGE4_WAIT_CENTER_1S)

    def do_stage4_wait_center_1s(self):
        if self.plot_direction == 'CENTER':
            self.publish_cmd(0, 0)
            if self.elapsed_in_state() >= 1: self.enter_state(MissionState.STAGE4_FORWARD_2S)
        elif self.plot_direction in ['LEFT', 'RIGHT']:
            speed = self.turn_speed if self.plot_direction == 'LEFT' else -self.turn_speed
            self.publish_cmd(0, speed)
            self.state_enter_time = time.monotonic()
        else:
            self.publish_cmd(0, 0)
            self.state_enter_time = time.monotonic()

    def do_stage4_forward_2s(self):
        if self.elapsed_in_state() >= 4: self.enter_state(MissionState.STAGE4_SWITCH_TO_DETECTOR)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_stage4_switch_to_detector(self):
        if self.proc_plot_chanon: self.proc_plot_chanon = self.stop_process(self.proc_plot_chanon, "plot_chanon")
        if self.proc_plot_detector is None:
            self.proc_plot_detector = self.run_ros2_node("nana_core", "plot_detector_sub")
        self.enter_state(MissionState.STAGE4_WAIT_FOR_DETECTOR_TOPIC)

    def do_wait_for_detector_topic(self):
        self.publish_cmd(0, 0)
        if self.apr_pos_received_in_stage:
            self.get_logger().info("Detector Topic Active. Starting backward movement.")
            self.enter_state(MissionState.STAGE4_BACKWARD_UNTIL_CENTER)
        else:
            self.get_logger().info("Waiting for /apriltag_position data...", throttle_duration_sec=2.0)
            if self.elapsed_in_state() > 10.0:
                self.get_logger().error("Detector Node Startup Timeout.")
                self.enter_state(MissionState.DONE)

    def do_stage4_backward_until_center(self):
        if self.apr_pos == 'CENTER':
            self.publish_cmd(0, 0)
            self.get_logger().info("Target centered. Preparation for Stage 5.")
            self.enter_state(MissionState.STAGE4_CHECK_DETECTOR_CENTER)
        else:
            self.publish_cmd(self.backward_speed, 0)
            if self.elapsed_in_state() > 15.0:
                self.publish_cmd(0, 0)
                self.get_logger().error("Safety Timeout: Center not found.")
                self.enter_state(MissionState.DONE)

    def do_stage4_check_detector_center(self):
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 1.0:
            if self.target_ab_dist <= 0: self.target_ab_dist = 10.0

            # *** MODIFIED: หยุด plot_detector_sub → รัน encoder_distance → รอค่าแรก ***
            if self.proc_plot_detector:
                self.proc_plot_detector = self.stop_process(self.proc_plot_detector, "plot_detector_sub")
                self.get_logger().info("plot_detector_sub stopped.")

            if self.proc_encoder_distance is None:
                self.proc_encoder_distance = self.run_ros2_node("nana_core", "encoder_distance")
                self.get_logger().info("encoder_distance node started.")

            self.enter_state(MissionState.STAGE4_WAIT_ENCODER_READY)

    # *** NEW: รอรับค่าแรกจาก /current_distance_cm ก่อนเริ่ม Stage 5 ***
    def do_stage4_wait_encoder_ready(self):
        self.publish_cmd(0, 0)
        if self.encoder_first_value_received:
            self.get_logger().info(
                f"Encoder ready. start={self.start_odom_dist:.2f} cm, "
                f"target AB={self.target_ab_dist:.2f} cm"
            )
            self.enter_state(MissionState.STAGE5_FORWARD_BY_AB)
        else:
            self.get_logger().info("Waiting for first /current_distance_cm value...", throttle_duration_sec=2.0)
            if self.elapsed_in_state() > 10.0:
                self.get_logger().error("Encoder Distance Node Startup Timeout.")
                self.enter_state(MissionState.DONE)

    # *** MODIFIED: ใช้ (current_odom_dist - start_odom_dist) แทน abs(current - start) ***
    def do_stage5_forward_by_ab(self):
        traveled = self.current_odom_dist - self.start_odom_dist
        if traveled >= self.target_ab_dist:
            self.publish_cmd(0, 0)
            self.get_logger().info(f"TARGET REACHED: {traveled:.2f} cm")
            self.enter_state(MissionState.STAGE4_FINAL_SWITCH_CHANON)
        else:
            self.publish_cmd(self.forward_speed, 0)
            self.get_logger().info(f"Stage 5: {traveled:.1f}/{self.target_ab_dist} cm", throttle_duration_sec=1.0)

    def do_stage4_final_switch_chanon(self):
        if self.proc_plot_detector: self.proc_plot_detector = self.stop_process(self.proc_plot_detector, "plot_detector_sub")
        if self.proc_plot_chanon is None:
            self.proc_plot_chanon = self.run_ros2_node("nana_core", "plot_chanon")
        self.enter_state(MissionState.DONE)

    def do_done(self):
        self.publish_cmd(0, 0)

# ---------------- Main ----------------

def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for p, n in [(node.proc_apriltag, "apriltag_detector_sub"), 
                     (node.proc_plot_chanon, "plot_chanon"), 
                     (node.proc_plot_detector, "plot_detector_sub"),
                     (node.proc_encoder_distance, "encoder_distance")]:   # *** NEW ***
            if p: node.stop_process(p, n)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()