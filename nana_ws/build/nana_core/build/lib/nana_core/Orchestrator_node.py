#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


APRIL_POS_TOPIC   = '/apriltag_position'
APRIL_DIST_TOPIC  = '/apriltag_distance'
APRIL_STOP_SRV    = '/apriltag/stop'

# 🔁 เปลี่ยนชื่อนี้ให้ตรงกับ detect plot node ของคุณ
PLOT_TOPIC        = '/plot_direction'

CMD_TOPIC         = '/nana/cmd_arm'


class MissionState:
    APRILTAG_TRACK = 'APRILTAG_TRACK'
    STOP_AND_KILL_APRILTAG = 'STOP_AND_KILL_APRILTAG'
    TURN_LEFT_1S   = 'TURN_LEFT_1S'
    FORWARD_1S     = 'FORWARD_1S'
    PLOT_TRACK     = 'PLOT_TRACK'


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')

        # --- pubs ---
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, qos_profile_system_default)

        # --- subs (Apriltag) ---
        self.apr_pos = 'NOT_FOUND'
        self.apr_dist = 9999.0
        self.create_subscription(String,  APRIL_POS_TOPIC,  self.apr_pos_cb,  qos_profile_system_default)
        self.create_subscription(Float32, APRIL_DIST_TOPIC, self.apr_dist_cb, qos_profile_system_default)

        # --- subs (Plot) ---
        self.plot_dir = 'NOT_FOUND'
        self.create_subscription(String, PLOT_TOPIC, self.plot_cb, qos_profile_system_default)

        # --- service client for stopping apriltag node ---
        self.stop_client = self.create_client(Trigger, APRIL_STOP_SRV)

        # --- state machine ---
        self.state = MissionState.APRILTAG_TRACK
        self.state_enter_time = time.monotonic()

        # --- params / tuning ---
        self.stop_distance_cm = 50.0  # ตามที่คุณต้องการ (ต่ำกว่า 50 หยุด)
        self.forward_speed = 0.35     # linear.y ที่ส่งไป /nana/cmd_arm
        self.turn_speed = 0.35        # angular.z
        self.search_turn_speed = 0.20 # ตอน NOT_FOUND

        # ถ้าข้อมูล apriltag มาไม่ต่อเนื่อง ให้คุมความปลอดภัย
        self.last_apr_update = time.monotonic()
        self.apr_timeout_s = 1.0

        # control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info("Mission Orchestrator started.")

    # ---------------- callbacks ----------------
    def apr_pos_cb(self, msg: String):
        self.apr_pos = msg.data
        self.last_apr_update = time.monotonic()

    def apr_dist_cb(self, msg: Float32):
        self.apr_dist = float(msg.data)
        self.last_apr_update = time.monotonic()

    def plot_cb(self, msg: String):
        self.plot_dir = msg.data

    # ---------------- helpers ----------------
    def publish_cmd(self, linear_y: float, angular_z: float):
        t = Twist()
        t.linear.y = float(linear_y)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def enter_state(self, new_state: str):
        self.state = new_state
        self.state_enter_time = time.monotonic()
        self.get_logger().info(f"-> STATE: {new_state}")

    def elapsed_in_state(self) -> float:
        return time.monotonic() - self.state_enter_time

    def apr_data_stale(self) -> bool:
        return (time.monotonic() - self.last_apr_update) > self.apr_timeout_s

    def request_stop_apriltag_node(self):
        # call /apriltag/stop once
        if not self.stop_client.service_is_ready():
            # ไม่บังคับรอ ถ้า service ยังไม่พร้อมก็ข้ามไป (node ยังรันต่อได้)
            self.get_logger().warn("Apriltag stop service not ready yet.")
            return

        req = Trigger.Request()
        future = self.stop_client.call_async(req)

        def done_cb(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"Apriltag stop resp: success={resp.success}, msg='{resp.message}'")
            except Exception as e:
                self.get_logger().error(f"Apriltag stop service failed: {e}")

        future.add_done_callback(done_cb)

    # ---------------- state behaviors ----------------
    def do_apriltag_track(self):
        # 1) ใช้ apriltag_position + distance คุมรถ
        # 2) ถ้า distance < 50 -> หยุด + สั่ง stop node apriltag
        if (self.apr_dist > 0.0) and (self.apr_dist < self.stop_distance_cm):
            self.stop_robot()
            self.enter_state(MissionState.STOP_AND_KILL_APRILTAG)
            return

        # ถ้าข้อมูล stale (แท็กหลุด) ให้หยุด/หมุนช้า ๆ หาแท็ก
        if self.apr_data_stale():
            # เลือกเอาแบบที่คุณอยากได้:
            # A) หยุดนิ่ง
            # self.stop_robot()
            # B) หมุนหาแท็กช้า ๆ
            self.publish_cmd(0.0, self.search_turn_speed)
            return

        # คุมตาม position
        if self.apr_pos == 'CENTER':
            self.publish_cmd(self.forward_speed, 0.0)
        elif self.apr_pos == 'LEFT':
            # หมุนซ้าย
            self.publish_cmd(0.0, +self.turn_speed)
        elif self.apr_pos == 'RIGHT':
            # หมุนขวา
            self.publish_cmd(0.0, -self.turn_speed)
        else:
            # NOT_FOUND
            self.publish_cmd(0.0, self.search_turn_speed)

    def do_stop_and_kill_apriltag(self):
        # หยุด 0.3s เพื่อให้รถนิ่ง แล้วค่อย stop apriltag node
        self.stop_robot()

        if self.elapsed_in_state() > 0.30:
            self.request_stop_apriltag_node()
            self.enter_state(MissionState.TURN_LEFT_1S)

    def do_turn_left_1s(self):
        # 3) เลี้ยวซ้าย 1 วินาที
        if self.elapsed_in_state() < 1.0:
            self.publish_cmd(0.0, +self.turn_speed)
        else:
            self.stop_robot()
            self.enter_state(MissionState.FORWARD_1S)

    def do_forward_1s(self):
        # 3) เดินหน้า 1 วินาที
        if self.elapsed_in_state() < 1.0:
            self.publish_cmd(self.forward_speed, 0.0)
        else:
            self.stop_robot()
            self.enter_state(MissionState.PLOT_TRACK)

    def do_plot_track(self):
        # 4) รับทิศทางจาก detect plot แล้วคุมรถต่อ
        d = self.plot_dir

        if d == 'CENTER':
            self.publish_cmd(self.forward_speed, 0.0)
        elif d == 'LEFT':
            self.publish_cmd(0.0, +self.turn_speed)
        elif d == 'RIGHT':
            self.publish_cmd(0.0, -self.turn_speed)
        else:
            # ถ้า plot ไม่เจอ -> หยุดหรือหมุนหา
            self.publish_cmd(0.0, self.search_turn_speed)

    # ---------------- main loop ----------------
    def control_loop(self):
        if self.state == MissionState.APRILTAG_TRACK:
            self.do_apriltag_track()
        elif self.state == MissionState.STOP_AND_KILL_APRILTAG:
            self.do_stop_and_kill_apriltag()
        elif self.state == MissionState.TURN_LEFT_1S:
            self.do_turn_left_1s()
        elif self.state == MissionState.FORWARD_1S:
            self.do_forward_1s()
        elif self.state == MissionState.PLOT_TRACK:
            self.do_plot_track()
        else:
            self.stop_robot()


def main():
    rclpy.init()
    node = MissionOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()