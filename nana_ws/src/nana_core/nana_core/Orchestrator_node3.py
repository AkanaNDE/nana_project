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
    # --- เพิ่ม Stage 4 ---
    STAGE4_BACKWARD_1S           = 'STAGE4_BACKWARD_1S' 
    STAGE4_FORWARD_1S            = 'STAGE4_FORWARD_1S' 
    DONE                         = 'DONE' 


class MissionOrchestrator(Node): 

    def __init__(self): 
        super().__init__('mission_orchestrator') 

        self.cmd_pub = self.create_publisher( 
            Twist, 
            '/nana/cmd_arm', 
            qos_profile_system_default 
        ) 

        self.create_subscription(Float32, '/apriltag_angle', self.apr_angle_cb, 10) 
        self.create_subscription(String, '/apriltag_position_front', self.apr_front_cb, 10) 
        self.create_subscription(Float32, '/apriltag_distance', self.apr_distance_cb, 10) 

        self.state = MissionState.WAIT_FOR_ANGLE 
        self.state_enter_time = time.monotonic() 

        self.apr_angle = 999.0 
        self.apr_angle_received = False 

        self.apr_front = 'NOT_FOUND' 
        self.apr_distance = -1.0 

        self.turn_speed = 0.35 
        self.forward_speed = 0.25 
        self.stop_distance_cm = 60.0 

        self.proc_apriltag = None 

        self.start_apriltag_detector_sub() 

        self.timer = self.create_timer(0.02, self.control_loop) 

        self.get_logger().info("Mission Orchestrator Started.") 

    # ---------------- Process helpers ---------------- 

    def run_ros2_node(self, package_name, executable_name): 
        cmd = ( 
            "source /opt/ros/jazzy/setup.bash && " 
            "source ~/nana_project/nana_ws/install/setup.bash && " 
            f"ros2 run {package_name} {executable_name}" 
        ) 
        return subprocess.Popen( 
            cmd, 
            shell=True, 
            executable="/bin/bash" 
        ) 

    def stop_process(self, proc, name="process"): 
        if proc is None: return None 
        if proc.poll() is not None: return None 

        self.get_logger().info(f"Stopping {name} ...") 
        proc.terminate() 
        try: 
            proc.wait(timeout=2.0) 
        except subprocess.TimeoutExpired: 
            proc.kill() 
            proc.wait() 
        return None 

    def start_apriltag_detector_sub(self): 
        if self.proc_apriltag is None or self.proc_apriltag.poll() is not None: 
            self.get_logger().info("Starting apriltag_detector_sub ...") 
            self.proc_apriltag = self.run_ros2_node("nana_core", "apriltag_detector_sub") 

    # ---------------- Callbacks ---------------- 

    def apr_angle_cb(self, msg): 
        self.apr_angle = float(msg.data) 
        self.apr_angle_received = True 

    def apr_front_cb(self, msg): 
        self.apr_front = msg.data 

    def apr_distance_cb(self, msg): 
        self.apr_distance = float(msg.data) 

    # ---------------- Helpers ---------------- 

    def publish_cmd(self, linear_y, angular_z): 
        t = Twist() 
        t.linear.y = float(linear_y) 
        t.angular.z = float(angular_z) 
        self.cmd_pub.publish(t) 

    def stop_robot(self): 
        self.publish_cmd(0.0, 0.0) 

    def enter_state(self, new_state): 
        self.get_logger().info(f"Transitioning: {self.state} -> {new_state}") 
        self.state = new_state 
        self.state_enter_time = time.monotonic() 

    def elapsed_in_state(self): 
        return time.monotonic() - self.state_enter_time 

    # ---------------- State Behaviors ---------------- 

    def do_wait_for_angle(self): 
        if not self.apr_angle_received: 
            self.stop_robot() 
            return 
        if self.apr_angle < 0.0: 
            self.enter_state(MissionState.STAGE2_TRACK_TO_30CM) 
        elif self.apr_angle > 0.0: 
            self.enter_state(MissionState.STAGE1_TURN_LEFT_2S) 
        else: 
            self.stop_robot() 

    # ---------- Stage1 ---------- 

    def do_stage1_turn_left_2s(self): 
        if self.elapsed_in_state() >= 5.0: 
            self.stop_robot() 
            self.enter_state(MissionState.STAGE1_FORWARD_4S) 
            return 
        self.publish_cmd(0.0, self.turn_speed) 

    def do_stage1_forward_4s(self): 
        if self.elapsed_in_state() >= 9.0: 
            self.stop_robot() 
            self.enter_state(MissionState.STAGE1_TURN_RIGHT_TO_CENTER) 
            return 
        self.publish_cmd(self.forward_speed, 0.0) 

    def do_stage1_turn_right_to_center(self): 
        if self.apr_front == 'CENTER': 
            self.stop_robot() 
            self.enter_state(MissionState.STAGE2_TRACK_TO_30CM) 
            return 
        self.publish_cmd(0.0, -self.turn_speed) 

    # ---------- Stage2 ---------- 

    def do_stage2_track_to_30cm(self): 
        if 0.0 < self.apr_distance <= self.stop_distance_cm: 
            self.stop_robot() 
            self.proc_apriltag = self.stop_process(self.proc_apriltag, "apriltag_detector_sub") 
            self.enter_state(MissionState.STAGE3_TURN_LEFT_3S) 
            return 

        if self.apr_front == 'CENTER': self.publish_cmd(self.forward_speed, 0.0) 
        elif self.apr_front == 'LEFT': self.publish_cmd(0.0, self.turn_speed) 
        elif self.apr_front == 'RIGHT': self.publish_cmd(0.0, -self.turn_speed) 
        else: self.stop_robot() 

    # ---------- Stage3 ---------- 

    def do_stage3_turn_left_3s(self): 
        if self.elapsed_in_state() >= 10.0: 
            self.stop_robot() 
            self.enter_state(MissionState.STAGE3_FORWARD_1S) 
            return 
        self.publish_cmd(0.0, self.turn_speed) 

    def do_stage3_forward_1s(self): 
        if self.elapsed_in_state() >= 3.0: 
            self.stop_robot() 
            self.enter_state(MissionState.STAGE4_BACKWARD_1S) # เปลี่ยนจาก DONE เป็น Stage 4
            return 
        self.publish_cmd(self.forward_speed, 0.0) 

    # ---------- Stage4 (ใหม่) ---------- 

    def do_stage4_backward_1s(self): 
        if self.elapsed_in_state() >= 1.0: 
            self.stop_robot() 
            self.enter_state(MissionState.STAGE4_FORWARD_1S) 
            return 
        # ถอยหลังใช้ความเร็วติดลบ
        self.publish_cmd(-self.forward_speed, 0.0) 

    def do_stage4_forward_1s(self): 
        if self.elapsed_in_state() >= 1.0: 
            self.stop_robot() 
            self.enter_state(MissionState.DONE) 
            return 
        self.publish_cmd(self.forward_speed, 0.0) 

    # ---------- Done ---------- 

    def do_done(self): 
        self.stop_robot() 

    # ---------------- Main Loop ---------------- 

    def control_loop(self): 
        if self.state == MissionState.WAIT_FOR_ANGLE: self.do_wait_for_angle() 
        elif self.state == MissionState.STAGE1_TURN_LEFT_2S: self.do_stage1_turn_left_2s() 
        elif self.state == MissionState.STAGE1_FORWARD_4S: self.do_stage1_forward_4s() 
        elif self.state == MissionState.STAGE1_TURN_RIGHT_TO_CENTER: self.do_stage1_turn_right_to_center() 
        elif self.state == MissionState.STAGE2_TRACK_TO_30CM: self.do_stage2_track_to_30cm() 
        elif self.state == MissionState.STAGE3_TURN_LEFT_3S: self.do_stage3_turn_left_3s() 
        elif self.state == MissionState.STAGE3_FORWARD_1S: self.do_stage3_forward_1s() 
        # เพิ่มเงื่อนไขใน loop
        elif self.state == MissionState.STAGE4_BACKWARD_1S: self.do_stage4_backward_1s() 
        elif self.state == MissionState.STAGE4_FORWARD_1S: self.do_stage4_forward_1s() 
        elif self.state == MissionState.DONE: self.do_done() 

    def destroy_node(self): 
        self.stop_robot() 
        self.proc_apriltag = self.stop_process(self.proc_apriltag, "apriltag_detector_sub") 
        super().destroy_node() 


def main(args=None): 
    rclpy.init(args=args) 
    node = MissionOrchestrator() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    finally: 
        node.destroy_node() 
        rclpy.shutdown() 

if __name__ == '__main__': 
    main()