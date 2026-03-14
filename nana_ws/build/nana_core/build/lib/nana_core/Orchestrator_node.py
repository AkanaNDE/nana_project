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
    STAGE4_RUN_PLOT_CHANON       = 'STAGE4_RUN_PLOT_CHANON' 
    STAGE4_MOVE_SEQUENCE         = 'STAGE4_MOVE_SEQUENCE'
    STAGE5_CLEANUP_DETECTOR      = 'STAGE5_CLEANUP_DETECTOR'
    STAGE5_RUN_PLOT_DETECTOR     = 'STAGE5_RUN_PLOT_DETECTOR'
    DONE                         = 'DONE' 


class MissionOrchestrator(Node): 

    def __init__(self): 
        super().__init__('mission_orchestrator') 

        self.cmd_pub = self.create_publisher(Twist, '/nana/cmd_arm', qos_profile_system_default) 

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
        self.proc_plot_chanon = None 
        self.proc_plot_detector = None

        self.start_apriltag_detector_sub() 
        self.timer = self.create_timer(0.02, self.control_loop) 
        self.get_logger().info("Mission Orchestrator Started.") 

    # ---------------- Process helpers ---------------- 
    def run_ros2_node(self, package_name, executable_name): 
        cmd = f"source /opt/ros/jazzy/setup.bash && source ~/nana_project/nana_ws/install/setup.bash && ros2 run {package_name} {executable_name}" 
        return subprocess.Popen(cmd, shell=True, executable="/bin/bash") 

    def stop_process(self, proc, name="process"): 
        if proc is None: return None 
        self.get_logger().info(f"Stopping {name}...") 
        subprocess.run(["pkill", "-9", "-f", name], stderr=subprocess.DEVNULL)
        if proc.poll() is None:
            proc.terminate()
            try: proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired: proc.kill()
        time.sleep(1.0) 
        return None 

    def start_apriltag_detector_sub(self): 
        if self.proc_apriltag is None or self.proc_apriltag.poll() is not None: 
            self.proc_apriltag = self.run_ros2_node("nana_core", "apriltag_detector_sub")

    # ---------------- Callbacks & Helpers ---------------- 
    def apr_angle_cb(self, msg): self.apr_angle = float(msg.data); self.apr_angle_received = True 
    def apr_front_cb(self, msg): self.apr_front = msg.data 
    def apr_distance_cb(self, msg): self.apr_distance = float(msg.data) 

    def publish_cmd(self, linear_y, angular_z): 
        t = Twist(); t.linear.y = float(linear_y); t.angular.z = float(angular_z)
        self.cmd_pub.publish(t) 

    def enter_state(self, new_state): 
        self.get_logger().info(f"Transitioning: {self.state} -> {new_state}") 
        self.state = new_state 
        self.state_enter_time = time.monotonic() 

    def elapsed_in_state(self): return time.monotonic() - self.state_enter_time 

    # ---------------- State Logic ---------------- 
    def do_wait_for_angle(self): 
        if not self.apr_angle_received: return self.publish_cmd(0,0)
        new_st = MissionState.STAGE2_TRACK_TO_30CM if self.apr_angle < 0.0 else MissionState.STAGE1_TURN_LEFT_2S
        self.enter_state(new_st)

    def do_stage1_turn_left_2s(self): 
        if self.elapsed_in_state() >= 5.0: self.enter_state(MissionState.STAGE1_FORWARD_4S)
        else: self.publish_cmd(0.0, self.turn_speed) 

    def do_stage1_forward_4s(self): 
        if self.elapsed_in_state() >= 9.0: self.enter_state(MissionState.STAGE1_TURN_RIGHT_TO_CENTER)
        else: self.publish_cmd(self.forward_speed, 0.0) 

    def do_stage1_turn_right_to_center(self): 
        if self.apr_front == 'CENTER': self.enter_state(MissionState.STAGE2_TRACK_TO_30CM)
        else: self.publish_cmd(0.0, -self.turn_speed) 

    def do_stage2_track_to_30cm(self): 
        if 0.0 < self.apr_distance <= self.stop_distance_cm: self.enter_state(MissionState.STAGE3_TURN_LEFT_3S)
        elif self.apr_front == 'CENTER': self.publish_cmd(self.forward_speed, 0.0) 
        elif self.apr_front == 'LEFT': self.publish_cmd(0.0, self.turn_speed) 
        elif self.apr_front == 'RIGHT': self.publish_cmd(0.0, -self.turn_speed) 
        else: self.publish_cmd(0,0)

    def do_stage3_turn_left_3s(self): 
        if self.elapsed_in_state() >= 2.0: self.enter_state(MissionState.STAGE3_FORWARD_1S)
        else: self.publish_cmd(0.0, self.turn_speed) 

    def do_stage3_forward_1s(self): 
        if self.elapsed_in_state() >= 3.0: self.enter_state(MissionState.STAGE4_CLEANUP_CHANON)
        else: self.publish_cmd(self.forward_speed, 0.0) 

    # ---------- Stage 4: Run Plot Chanon & Movement ----------
    def do_stage4_cleanup_chanon(self):
        if self.proc_apriltag is not None:
            self.proc_apriltag = self.stop_process(self.proc_apriltag, "apriltag_detector_sub")
        if self.elapsed_in_state() >= 2.0: self.enter_state(MissionState.STAGE4_RUN_PLOT_CHANON)

    def do_stage4_run_plot_chanon(self):
        if self.proc_plot_chanon is None:
            self.proc_plot_chanon = self.run_ros2_node("nana_core", "plot_chanon")
            self.enter_state(MissionState.STAGE4_MOVE_SEQUENCE)

    def do_stage4_move_sequence(self):
        elapsed = self.elapsed_in_state()
        if elapsed <= 1.0: # เลี้ยวขวา 1 วิ
            self.publish_cmd(0.0, -self.turn_speed)
        elif elapsed <= 3.0: # เดินตรง 2 วิ (1+2=3)
            self.publish_cmd(self.forward_speed, 0.0)
        elif elapsed <= 8.0: # ถอยหลัง 5 วิ (3+5=8)
            self.publish_cmd(-self.forward_speed, 0.0)
        else:
            self.publish_cmd(0.0, 0.0)
            self.enter_state(MissionState.STAGE5_CLEANUP_DETECTOR)

    # ---------- Stage 5: Switch to Plot Detector ----------
    def do_stage5_run_plot_detector(self):
        if self.proc_plot_detector is None:
            self.get_logger().info("Launching plot_detector_sub...")
            self.proc_plot_detector = self.run_ros2_node("nana_core", "plot_detector_sub")
            self.enter_state(MissionState.DONE)

    def do_done(self): self.publish_cmd(0,0) 

    # ---------------- Main Loop ---------------- 
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
            MissionState.STAGE4_RUN_PLOT_CHANON: self.do_stage4_run_plot_chanon,
            MissionState.STAGE4_MOVE_SEQUENCE: self.do_stage4_move_sequence,
            MissionState.STAGE5_RUN_PLOT_DETECTOR: self.do_stage5_run_plot_detector,
            MissionState.DONE: self.do_done,
        }
        handler = logic_map.get(self.state)
        if handler: handler()

    def destroy_node(self): 
        self.publish_cmd(0,0)
        self.stop_process(self.proc_apriltag, "apriltag_detector_sub") 
        self.stop_process(self.proc_plot_chanon, "plot_chanon")
        self.stop_process(self.proc_plot_detector, "plot_detector_sub")
        super().destroy_node() 

def main(args=None): 
    rclpy.init(args=args) 
    node = MissionOrchestrator() 
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass 
    finally: node.destroy_node(); rclpy.shutdown() 

if __name__ == '__main__': main()