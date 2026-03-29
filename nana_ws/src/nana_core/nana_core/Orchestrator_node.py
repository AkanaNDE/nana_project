#!/usr/bin/env python3
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist

class MissionState:
    WAIT_FOR_ANGLE                = 'WAIT_FOR_ANGLE'
    STAGE1_TURN_LEFT_2S           = 'STAGE1_TURN_LEFT_2S'
    STAGE1_FORWARD_4S             = 'STAGE1_FORWARD_4S'
    STAGE1_TURN_RIGHT_TO_CENTER   = 'STAGE1_TURN_RIGHT_TO_CENTER'
    STAGE2_TRACK_TO_30CM          = 'STAGE2_TRACK_TO_30CM'
    STAGE3_TURN_LEFT_3S           = 'STAGE3_TURN_LEFT_3S'
    STAGE3_FORWARD_1S             = 'STAGE3_FORWARD_1S'
    STAGE4_CLEANUP_CHANON         = 'STAGE4_CLEANUP_CHANON'
    STAGE4_STOP_BEFORE_CHANON     = 'STAGE4_STOP_BEFORE_CHANON'
    STAGE4_WAIT_CENTER_1S         = 'STAGE4_WAIT_CENTER_1S'
    STAGE4_WAIT_CENTER_2S         = 'STAGE4_WAIT_CENTER_2S'
    STAGE4_FORWARD_2S             = 'STAGE4_FORWARD_2S'
    STAGE4_SWITCH_TO_DETECTOR     = 'STAGE4_SWITCH_TO_DETECTOR'
    STAGE4_WAIT_DETECTOR_READY    = 'STAGE4_WAIT_DETECTOR_READY'
    STAGE4_WAIT_FOR_DETECTOR_TOPIC= 'STAGE4_WAIT_FOR_DETECTOR_TOPIC'
    STAGE4_BACKWARD_UNTIL_CENTER  = 'STAGE4_BACKWARD_UNTIL_CENTER'
    STAGE4_CHECK_DETECTOR_CENTER  = 'STAGE4_CHECK_DETECTOR_CENTER'
    STAGE4_WAIT_ENCODER_READY     = 'STAGE4_WAIT_ENCODER_READY'
    STAGE5_FORWARD_BY_AB          = 'STAGE5_FORWARD_BY_AB'
    STAGE5_PUBLISH_PLANT          = 'STAGE5_PUBLISH_PLANT'
    STAGE5_WAIT_7S                = 'STAGE5_WAIT_7S'
    STAGE5_WAIT_GAP_ENCODER       = 'STAGE5_WAIT_GAP_ENCODER'
    STAGE5_FORWARD_BY_GAP         = 'STAGE5_FORWARD_BY_GAP'
    STAGE5_PUBLISH_PLANT2         = 'STAGE5_PUBLISH_PLANT2'
    STAGE5_WAIT_7S_PLANT2         = 'STAGE5_WAIT_7S_PLANT2'
    STAGE6_FORWARD_BY_C           = 'STAGE6_FORWARD_BY_C'
    STAGE6_TURN_LEFT_5S           = 'STAGE6_TURN_LEFT_5S'
    STAGE6_FORWARD_3S             = 'STAGE6_FORWARD_3S'
    STAGE6_TURN_RIGHT_5S          = 'STAGE6_TURN_RIGHT_5S'
    STAGE7_SERVO_ROTATE           = 'STAGE7_SERVO_ROTATE'
    STAGE7_RUN_CAB_DETECT         = 'STAGE7_RUN_CAB_DETECT'
    STAGE7_WAIT_CAB_DETECT_READY  = 'STAGE7_WAIT_CAB_DETECT_READY'
    # ── เพิ่มเป็น 5 รอบ ──
    STAGE7_FORWARD_DE_1           = 'STAGE7_FORWARD_DE_1'
    STAGE7_STOP_3S_1              = 'STAGE7_STOP_3S_1'
    STAGE7_FORWARD_DE_2           = 'STAGE7_FORWARD_DE_2'
    STAGE7_STOP_3S_2              = 'STAGE7_STOP_3S_2'
    STAGE7_FORWARD_DE_3           = 'STAGE7_FORWARD_DE_3'
    STAGE7_STOP_3S_3              = 'STAGE7_STOP_3S_3'
    STAGE7_FORWARD_DE_4           = 'STAGE7_FORWARD_DE_4'
    STAGE7_STOP_3S_4              = 'STAGE7_STOP_3S_4'
    STAGE7_FORWARD_DE_5           = 'STAGE7_FORWARD_DE_5'
    STAGE7_STOP_3S_5              = 'STAGE7_STOP_3S_5'
    DONE                          = 'DONE'


class MissionOrchestrator(Node):

    def __init__(self):
        super().__init__('mission_orchestrator')

        # Publishers
        self.cmd_pub          = self.create_publisher(Twist, '/nana/cmd_arm', qos_profile_system_default)
        self.plant_pub        = self.create_publisher(Bool, '/plant', 10)
        self.plant2_pub       = self.create_publisher(Bool, '/plant2', 10)
        self.servo_pub        = self.create_publisher(Bool, '/servo_cmd', 10)
        self.finish_track_pub = self.create_publisher(Bool, '/finish_track', 10)
        self.finish_pid_pub   = self.create_publisher(Bool, '/finish_pid', 10)

        # Subscriptions
        self.create_subscription(Float32, '/apriltag_angle',          self.apr_angle_cb,    10)
        self.create_subscription(String,  '/apriltag_position',       self.apr_pos_cb,      10)
        self.create_subscription(String,  '/apriltag_position_front', self.apr_front_cb,    10)
        self.create_subscription(Float32, '/apriltag_distance',       self.apr_distance_cb, 10)
        self.create_subscription(String,  '/plot_direction',          self.plot_dir_cb,     10)
        self.create_subscription(String,  '/apriltag_id',             self.apr_id_cb,       10)
        self.create_subscription(Float32, '/current_distance_cm',     self.current_dist_cb, 10)
        self.create_subscription(Float32, '/cab_size',                self.cab_size_cb,     10)
        self.create_subscription(Float32, '/pid_correction',          self.pid_cb,          10)

        # State
        self.state            = MissionState.WAIT_FOR_ANGLE
        self.state_enter_time = time.monotonic()

        # Sensor data
        self.apr_angle                 = 999.0
        self.apr_angle_received        = False
        self.apr_pos                   = 'NOT_FOUND'
        self.apr_pos_received_in_stage = False
        self.apr_front                 = 'NOT_FOUND'
        self.apr_distance              = -1.0
        self.plot_direction            = 'NONE'
        self.current_cab_size          = 0.0
        self.pid_correction            = 0.0

        # AprilTag params
        self.target_ab_dist = 0.0
        self.gap_c_val      = 0.0
        self.interval_de    = 0.0

        # Odometry
        self.current_odom_dist                = 0.0
        self.encoder_first_value_received     = False
        self.start_odom_dist                  = 0.0
        self.gap_encoder_first_value_received = False
        self.gap_start_odom_dist              = 0.0
        self.de_start_odom                    = 0.0

        # Node readiness flags
        self.plot_chanon_ready    = False
        self.detector_ready       = False
        self.encoder_ready        = False
        self.cab_detect_ready     = False
        self.finish_pid_published = False

        # Config
        self.turn_speed       = 0.3
        self.forward_speed    = 0.3
        self.backward_speed   = -0.15
        self.stop_distance_cm = 60.0

        # Processes
        self.proc_apriltag         = None
        self.proc_plot_chanon      = None
        self.proc_plot_detector    = None
        self.proc_encoder_distance = None
        self.proc_ControlPID       = None
        self.proc_cab_detect       = None

        self.start_apriltag_detector_sub()

        self.proc_ControlPID = self.run_ros2_node("nana_core", "ControlPID")
        self.get_logger().info("[ControlPID] Node started at launch.")

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Mission Orchestrator System Ready.")

    # ──────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────

    def apr_id_cb(self, msg):
        tag = msg.data
        if len(tag) == 5 and tag.isdigit():
            self.target_ab_dist = float(tag[0:2])
            self.gap_c_val      = {'1':5,'2':10,'3':15,'4':20,'5':25}.get(tag[2], 0.0)
            self.interval_de    = float(tag[3:5])

    def current_dist_cb(self, msg):
        self.current_odom_dist = msg.data
        if self.state == MissionState.STAGE4_WAIT_ENCODER_READY and not self.encoder_first_value_received:
            self.start_odom_dist = msg.data
            self.encoder_first_value_received = True
            self.get_logger().info(f"[Encoder] Ready. start={self.start_odom_dist:.2f} cm")
        if self.state == MissionState.STAGE5_WAIT_GAP_ENCODER and not self.gap_encoder_first_value_received:
            self.gap_start_odom_dist = msg.data
            self.gap_encoder_first_value_received = True
            self.get_logger().info(f"[GAP Encoder] Ready. start={self.gap_start_odom_dist:.2f} cm")

    def plot_dir_cb(self, msg):
        self.plot_direction = msg.data
        if not self.plot_chanon_ready:
            self.plot_chanon_ready = True
            self.get_logger().info("[plot_chanon] topic /plot_direction received → READY")

    def apr_pos_cb(self, msg):
        self.apr_pos = msg.data
        self.apr_pos_received_in_stage = True
        if not self.detector_ready:
            self.detector_ready = True
            self.get_logger().info("[plot_detector_sub] topic /apriltag_position received → READY")

    def cab_size_cb(self, msg):
        self.current_cab_size = msg.data
        if not self.cab_detect_ready:
            self.cab_detect_ready = True
            self.get_logger().info("[cab_detect] topic /cab_size received → READY")

    def pid_cb(self, msg):
        self.pid_correction = msg.data

    def apr_angle_cb(self, msg):    self.apr_angle = float(msg.data); self.apr_angle_received = True
    def apr_front_cb(self, msg):    self.apr_front = msg.data
    def apr_distance_cb(self, msg): self.apr_distance = float(msg.data)

    # ──────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────

    def run_ros2_node(self, package_name, executable_name):
        cmd = (f"source /opt/ros/jazzy/setup.bash && "
               f"source ~/nana_project/nana_ws/install/setup.bash && "
               f"ros2 run {package_name} {executable_name}")
        return subprocess.Popen(cmd, shell=True, executable="/bin/bash")

    def stop_process(self, proc, name="process"):
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        return None

    def start_apriltag_detector_sub(self):
        if self.proc_apriltag is None or self.proc_apriltag.poll() is not None:
            self.proc_apriltag = self.run_ros2_node("nana_core", "apriltag_detector_sub")

    def publish_cmd(self, y, z):
        t = Twist()
        t.linear.y = float(y)
        if y != 0 and z == 0:
            corrected_y = float(y) - (self.pid_correction * 0.5)
            if y > 0:
                corrected_y = max(0.05, min(float(y), corrected_y))
            else:
                corrected_y = min(-0.05, max(float(y), corrected_y))
            t.linear.y  = corrected_y
            t.angular.z = 0.0
        else:
            t.linear.y  = float(y)
            t.angular.z = float(z)
        self.cmd_pub.publish(t)

    def publish_servo(self, value):
        msg = Bool(); msg.data = value; self.servo_pub.publish(msg)

    def enter_state(self, new_state):
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self.state            = new_state
        self.state_enter_time = time.monotonic()

    def elapsed_in_state(self):
        return time.monotonic() - self.state_enter_time

    # ──────────────────────────────────────────
    # Control Loop
    # ──────────────────────────────────────────

    def control_loop(self):
        m = MissionState
        logic = {
            m.WAIT_FOR_ANGLE:                self.do_wait_for_angle,
            m.STAGE1_TURN_LEFT_2S:           self.do_s1_left,
            m.STAGE1_FORWARD_4S:             self.do_s1_fwd,
            m.STAGE1_TURN_RIGHT_TO_CENTER:   self.do_s1_center,
            m.STAGE2_TRACK_TO_30CM:          self.do_s2_track,
            m.STAGE3_TURN_LEFT_3S:           self.do_s3_left,
            m.STAGE3_FORWARD_1S:             self.do_s3_fwd,
            m.STAGE4_CLEANUP_CHANON:         self.do_s4_cleanup,
            m.STAGE4_STOP_BEFORE_CHANON:     self.do_s4_stop,
            m.STAGE4_WAIT_CENTER_2S:         self.do_s4_wait_left,
            m.STAGE4_WAIT_CENTER_1S:         self.do_s4_wait_center,
            m.STAGE4_SWITCH_TO_DETECTOR:     self.do_s4_switch,
            m.STAGE4_WAIT_DETECTOR_READY:    self.do_s4_wait_detector_ready,
            m.STAGE4_WAIT_FOR_DETECTOR_TOPIC:self.do_s4_wait_topic,
            m.STAGE4_BACKWARD_UNTIL_CENTER:  self.do_s4_back,
            m.STAGE4_CHECK_DETECTOR_CENTER:  self.do_s4_check,
            m.STAGE4_WAIT_ENCODER_READY:     self.do_s4_enc_ready,
            m.STAGE5_FORWARD_BY_AB:          self.do_s5_ab,
            m.STAGE5_PUBLISH_PLANT:          self.do_s5_plant,
            m.STAGE5_WAIT_7S:                self.do_s5_wait7,
            m.STAGE5_WAIT_GAP_ENCODER:       self.do_s5_wait_gap,
            m.STAGE5_FORWARD_BY_GAP:         self.do_s5_gap,
            m.STAGE5_PUBLISH_PLANT2:         self.do_s5_plant2,
            m.STAGE5_WAIT_7S_PLANT2:         self.do_s5_wait7_2,
            m.STAGE6_FORWARD_BY_C:           self.do_s6_c,
            m.STAGE6_TURN_LEFT_5S:           self.do_s6_left,
            m.STAGE6_FORWARD_3S:             self.do_s6_fwd,
            m.STAGE6_TURN_RIGHT_5S:          self.do_s6_right,
            m.STAGE7_SERVO_ROTATE:           self.do_s7_servo,
            m.STAGE7_RUN_CAB_DETECT:         self.do_s7_run_detect,
            m.STAGE7_WAIT_CAB_DETECT_READY:  self.do_s7_wait_cab_ready,
            # ── 5 รอบ ──
            m.STAGE7_FORWARD_DE_1: lambda: self.do_s7_fwd_de(m.STAGE7_STOP_3S_1, 1),
            m.STAGE7_STOP_3S_1:    lambda: self.do_s7_stop(m.STAGE7_FORWARD_DE_2, 1),
            m.STAGE7_FORWARD_DE_2: lambda: self.do_s7_fwd_de(m.STAGE7_STOP_3S_2, 2),
            m.STAGE7_STOP_3S_2:    lambda: self.do_s7_stop(m.STAGE7_FORWARD_DE_3, 2),
            m.STAGE7_FORWARD_DE_3: lambda: self.do_s7_fwd_de(m.STAGE7_STOP_3S_3, 3),
            m.STAGE7_STOP_3S_3:    lambda: self.do_s7_stop(m.STAGE7_FORWARD_DE_4, 3),
            m.STAGE7_FORWARD_DE_4: lambda: self.do_s7_fwd_de(m.STAGE7_STOP_3S_4, 4),
            m.STAGE7_STOP_3S_4:    lambda: self.do_s7_stop(m.STAGE7_FORWARD_DE_5, 4),
            m.STAGE7_FORWARD_DE_5: lambda: self.do_s7_fwd_de(m.STAGE7_STOP_3S_5, 5),
            m.STAGE7_STOP_3S_5:    lambda: self.do_s7_stop(m.DONE, 5),
            m.DONE:                self.do_done,
        }
        handler = logic.get(self.state)
        if handler: handler()

    # ──────────────────────────────────────────
    # Stage Logic
    # ──────────────────────────────────────────

    def do_wait_for_angle(self):
        self.publish_cmd(0, 0)
        if self.apr_angle_received:
            self.enter_state(MissionState.STAGE2_TRACK_TO_30CM if self.apr_angle < 0
                             else MissionState.STAGE1_TURN_LEFT_2S)

    def do_s1_left(self):
        if self.elapsed_in_state() >= 4: self.enter_state(MissionState.STAGE1_FORWARD_4S)
        else: self.publish_cmd(0, self.turn_speed)

    def do_s1_fwd(self):
        if self.elapsed_in_state() >= 6: self.enter_state(MissionState.STAGE1_TURN_RIGHT_TO_CENTER)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_s1_center(self):
        if self.apr_front == 'CENTER': self.enter_state(MissionState.STAGE2_TRACK_TO_30CM)
        else: self.publish_cmd(0, -self.turn_speed)

    def do_s2_track(self):
        if 0 < self.apr_distance <= self.stop_distance_cm:
            self.enter_state(MissionState.STAGE3_TURN_LEFT_3S)
        elif self.apr_front == 'CENTER': self.publish_cmd(self.forward_speed, 0)
        elif self.apr_front == 'LEFT':   self.publish_cmd(0, self.turn_speed)
        elif self.apr_front == 'RIGHT':  self.publish_cmd(0, -self.turn_speed)
        else: self.publish_cmd(0, 0)

    def do_s3_left(self):
        if self.elapsed_in_state() >= 2: self.enter_state(MissionState.STAGE3_FORWARD_1S)
        else: self.publish_cmd(0, self.turn_speed)

    def do_s3_fwd(self):
        if self.elapsed_in_state() >= 3: self.enter_state(MissionState.STAGE4_CLEANUP_CHANON)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_s4_cleanup(self):
        self.proc_apriltag = self.stop_process(self.proc_apriltag, "apriltag_detector_sub")
        self.publish_cmd(0, 0)
        self.enter_state(MissionState.STAGE4_STOP_BEFORE_CHANON)

    def do_s4_stop(self):
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 1:
            self.enter_state(MissionState.STAGE4_WAIT_CENTER_2S)

    def do_s4_wait_left(self):
        if self.elapsed_in_state() >= 2: self.enter_state(MissionState.STAGE4_WAIT_CENTER_1S)
        else: self.publish_cmd(0, -self.turn_speed)

    def do_s4_wait_center(self):
        if self.elapsed_in_state() >= 4: self.enter_state(MissionState.STAGE4_SWITCH_TO_DETECTOR)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_s4_switch(self):
        self.publish_cmd(0, 0)
        self.proc_plot_chanon = self.stop_process(self.proc_plot_chanon, "plot_chanon")
        if self.proc_plot_detector is None:
            self.detector_ready            = False
            self.apr_pos_received_in_stage = False
            self.proc_plot_detector        = self.run_ros2_node("nana_core", "plot_detector_sub")
            self.get_logger().info("[plot_detector_sub] Node started. Waiting for /apriltag_position...")
        self.enter_state(MissionState.STAGE4_WAIT_DETECTOR_READY)

    def do_s4_wait_detector_ready(self):
        self.publish_cmd(0, 0)
        if self.detector_ready:
            self.get_logger().info("[plot_detector_sub] READY → proceed")
            self.enter_state(MissionState.STAGE4_WAIT_FOR_DETECTOR_TOPIC)
        else:
            self.get_logger().info("Waiting for plot_detector_sub...", throttle_duration_sec=2.0)

    def do_s4_wait_topic(self):
        self.publish_cmd(0, 0)
        if self.apr_pos_received_in_stage:
            self.enter_state(MissionState.STAGE4_BACKWARD_UNTIL_CENTER)

    def do_s4_back(self):
        if self.apr_pos == 'CENTER':
            self.publish_cmd(0, 0)
            self.enter_state(MissionState.STAGE4_CHECK_DETECTOR_CENTER)
        else:
            self.publish_cmd(self.backward_speed, 0)

    def do_s4_check(self):
        self.publish_cmd(0, 0)
        self.proc_plot_detector = self.stop_process(self.proc_plot_detector, "plot_detector_sub")
        if self.proc_encoder_distance is None:
            self.encoder_first_value_received = False
            self.proc_encoder_distance = self.run_ros2_node("nana_core", "encoder_distance")
            self.get_logger().info("[encoder_distance] Node started. Waiting for /current_distance_cm...")
        self.enter_state(MissionState.STAGE4_WAIT_ENCODER_READY)

    def do_s4_enc_ready(self):
        self.publish_cmd(0, 0)
        if self.encoder_first_value_received:
            self.get_logger().info("[encoder_distance] READY → proceed")
            self.enter_state(MissionState.STAGE5_FORWARD_BY_AB)
        else:
            self.get_logger().info("Waiting for encoder_distance...", throttle_duration_sec=2.0)

    def do_s5_ab(self):
        traveled = self.current_odom_dist - self.start_odom_dist
        if traveled >= 30:
            self.publish_cmd(0, 0)
            self.enter_state(MissionState.STAGE5_PUBLISH_PLANT)
        else:
            self.publish_cmd(self.forward_speed, 0)
            self.get_logger().info(f"AB: {traveled:.1f}/{self.target_ab_dist} cm", throttle_duration_sec=1.0)

    def do_s5_plant(self):
        self.publish_cmd(0, 0)
        self.plant_pub.publish(Bool(data=True))
        self.get_logger().info("Published /plant = True")
        self.enter_state(MissionState.STAGE5_WAIT_7S)

    def do_s5_wait7(self):
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 11:
            self.enter_state(MissionState.STAGE5_WAIT_GAP_ENCODER)

    def do_s5_wait_gap(self):
        self.publish_cmd(0, 0)
        if self.gap_encoder_first_value_received:
            self.enter_state(MissionState.STAGE5_FORWARD_BY_GAP)
        else:
            self.get_logger().info("Waiting gap encoder snapshot...", throttle_duration_sec=2.0)

    def do_s5_gap(self):
        traveled = self.current_odom_dist - self.gap_start_odom_dist
        if traveled >= 35:
            self.publish_cmd(0, 0)
            self.enter_state(MissionState.STAGE5_PUBLISH_PLANT2)
        else:
            self.publish_cmd(self.forward_speed, 0)
            self.get_logger().info(f"GAP: {traveled:.1f}/{self.target_ab_dist} cm", throttle_duration_sec=1.0)

    def do_s5_plant2(self):
        self.publish_cmd(0, 0)
        self.plant2_pub.publish(Bool(data=True))
        self.finish_track_pub.publish(Bool(data=True))
        self.get_logger().info("Published /plant2 = True & /finish_track = True")
        self.enter_state(MissionState.STAGE5_WAIT_7S_PLANT2)

    def do_s5_wait7_2(self):
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 11:
            self.gap_start_odom_dist = self.current_odom_dist
            self.enter_state(MissionState.STAGE6_TURN_LEFT_5S)

    def do_s6_left(self):
        if self.elapsed_in_state() >= 2: self.enter_state(MissionState.STAGE6_FORWARD_3S)
        else: self.publish_cmd(0, self.turn_speed)

    def do_s6_fwd(self):
        if self.elapsed_in_state() >= 2: self.enter_state(MissionState.STAGE6_TURN_RIGHT_5S)
        else: self.publish_cmd(self.forward_speed, 0)

    def do_s6_right(self):
        if self.elapsed_in_state() >= 3: self.enter_state(MissionState.STAGE6_FORWARD_BY_C)
        else: self.publish_cmd(0, -self.turn_speed)

    def do_s6_c(self):
        traveled = self.current_odom_dist - self.gap_start_odom_dist
        if traveled >= self.gap_c_val:
            self.publish_cmd(0, 0)
            self.enter_state(MissionState.STAGE7_SERVO_ROTATE)
        else:
            self.publish_cmd(self.forward_speed, 0)
            self.get_logger().info(f"C: {traveled:.1f}/{self.gap_c_val} cm", throttle_duration_sec=1.0)

    # ── Stage 7 ──

    def do_s7_servo(self):
        self.publish_cmd(0, 0)
        self.publish_servo(True)
        self.get_logger().info("[Servo3] → 30°")
        self.enter_state(MissionState.STAGE7_RUN_CAB_DETECT)

    def do_s7_run_detect(self):
        self.publish_cmd(0, 0)
        if self.proc_cab_detect is None:
            self.cab_detect_ready = False
            self.proc_cab_detect  = self.run_ros2_node("nana_core", "cab_detect")
            self.get_logger().info("[cab_detect] Node started. Waiting for /cab_size...")
        self.enter_state(MissionState.STAGE7_WAIT_CAB_DETECT_READY)

    def do_s7_wait_cab_ready(self):
        self.publish_cmd(0, 0)
        if self.cab_detect_ready:
            self.get_logger().info("[cab_detect] READY → proceed")
            self.de_start_odom = self.current_odom_dist
            self.enter_state(MissionState.STAGE7_FORWARD_DE_1)
        else:
            self.get_logger().info("Waiting for cab_detect...", throttle_duration_sec=2.0)

    def do_s7_fwd_de(self, next_state, round_num):
        traveled = self.current_odom_dist - self.de_start_odom
        if traveled >= self.interval_de:
            self.publish_cmd(0, 0)
            self.enter_state(next_state)
        else:
            self.publish_cmd(self.forward_speed, 0)
            self.get_logger().info(f"DE Loop {round_num}/5: {traveled:.1f}/{self.interval_de} cm",
                                   throttle_duration_sec=1.0)

    def do_s7_stop(self, next_state, round_num):
        self.publish_cmd(0, 0)
        if self.elapsed_in_state() >= 3.0:
            if next_state == MissionState.DONE:
                self.publish_servo(False)
                self.get_logger().info("[Servo3] → 0° | Mission DONE")
                self.enter_state(MissionState.DONE)
            else:
                self.de_start_odom = self.current_odom_dist
                self.enter_state(next_state)

    def do_done(self):
        self.publish_cmd(0, 0)
        if not self.finish_pid_published:
            self.finish_pid_pub.publish(Bool(data=True))
            self.finish_pid_published = True
            self.get_logger().info("Published /finish_pid = True | All stages complete.")


# ──────────────────────────────────────────
# Main
# ──────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for p, n in [
            (node.proc_apriltag,         "apriltag_detector_sub"),
            (node.proc_plot_chanon,      "plot_chanon"),
            (node.proc_plot_detector,    "plot_detector_sub"),
            (node.proc_encoder_distance, "encoder_distance"),
            (node.proc_ControlPID,       "ControlPID"),
            (node.proc_cab_detect,       "cab_detect"),
        ]:
            node.stop_process(p, n)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()