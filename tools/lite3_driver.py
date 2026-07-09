#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Standalone Lite3 driver (no motion_test package needed).

Subscribes to /cmd_vel, /cmd_vel_auto, /cmd_gait, /emergency_stop and talks to
the robot directly over UDP.  Publishes /leg_odom2, /driver_status, and
optionally /ultrasonic/front + /ultrasonic/back parsed from the 0x0901 packet.
"""

import math
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def build_range(frame_id, stamp, dist, max_range=5.0):
    msg = Range()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.radiation_type = Range.ULTRASOUND
    msg.field_of_view = 0.5
    msg.min_range = 0.05
    msg.max_range = max_range
    if dist is None or dist < msg.min_range or dist > max_range:
        msg.range = max_range
    else:
        msg.range = dist
    return msg


class Lite3DriverNode(Node):
    GAIT_CMDS = {
        "slow": 0x21010300,
        "medium": 0x21010307,
        "fast": 0x21010303,
        "stair": 0x21010407,
        "crawl": 0x21010406,
    }

    def __init__(self):
        super().__init__("lite3_driver_node")

        # 网络/控制参数
        self.declare_parameter("robot_ip", "192.168.1.120")
        self.declare_parameter("robot_port", 43893)
        self.declare_parameter("heartbeat_period", 0.5)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("manual_override_timeout", 1.5)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        # 超声波参数
        self.declare_parameter("ultrasound_front_index", -1)   # 0x0901 double 数组索引，-1 禁用
        self.declare_parameter("ultrasound_back_index", -1)
        self.declare_parameter("ultrasound_unit_scale", 1.0)   # raw -> 米
        self.declare_parameter("ultrasound_max_range", 5.0)
        self.declare_parameter("ultrasound_timeout", 0.5)
        self.declare_parameter("obstacle_stop_distance", 0.2)
        self.declare_parameter("ultrasound_front_topic", "/ultrasonic/front")
        self.declare_parameter("ultrasound_back_topic", "/ultrasonic/back")
        self.declare_parameter("ultrasound_debug", False)

        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_port = self.get_parameter("robot_port").value
        self.heartbeat_period = self.get_parameter("heartbeat_period").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.manual_override_timeout = self.get_parameter(
            "manual_override_timeout"
        ).value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        self.front_index = self.get_parameter("ultrasound_front_index").value
        self.back_index = self.get_parameter("ultrasound_back_index").value
        self.us_scale = self.get_parameter("ultrasound_unit_scale").value
        self.us_max_range = self.get_parameter("ultrasound_max_range").value
        self.us_timeout = self.get_parameter("ultrasound_timeout").value
        self.obs_dist = self.get_parameter("obstacle_stop_distance").value
        self.us_debug = self.get_parameter("ultrasound_debug").value

        # 速度源
        self.manual_vel = Twist()
        self.auto_vel = Twist()
        self.last_manual_time = self.get_clock().now()
        self.last_auto_time = self.get_clock().now()
        self.emergency_stop = False
        self.out_vel = Twist()

        # 超声波状态
        self.front_dist = None
        self.back_dist = None
        self.last_us_time = None
        self._warned_us_size = False
        self._last_us_debug_log = self.get_clock().now()

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", self.robot_port))
            self.get_logger().info(f"UDP port {self.robot_port} bound")
        except OSError as e:
            self.get_logger().error(f"Failed to bind UDP port {self.robot_port}: {e}")
        self.sock.setblocking(False)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/leg_odom2", 10)
        self.status_pub = self.create_publisher(String, "/driver_status", 10)
        front_topic = self.get_parameter("ultrasound_front_topic").value
        back_topic = self.get_parameter("ultrasound_back_topic").value
        self.front_pub = self.create_publisher(Range, front_topic, 10)
        self.back_pub = self.create_publisher(Range, back_topic, 10)

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_auto", self.cmd_vel_auto_callback, 10)
        self.create_subscription(String, "/cmd_gait", self.cmd_gait_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self.emergency_callback, 10)

        # Timers
        self.create_timer(self.heartbeat_period, self.send_heartbeat)
        self.create_timer(0.04, self.send_velocity_loop)   # 25 Hz
        self.create_timer(0.02, self.receive_data_loop)    # 50 Hz
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info("Lite3 driver node initialized")
        self.wakeup_robot()

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        self.manual_vel = self._clamp_twist(msg)
        self.last_manual_time = self.get_clock().now()

    def cmd_vel_auto_callback(self, msg):
        self.auto_vel = self._clamp_twist(msg)
        self.last_auto_time = self.get_clock().now()

    def cmd_gait_callback(self, msg):
        gait_name = msg.data.lower()
        if gait_name in self.GAIT_CMDS:
            self.get_logger().info(f"Switch gait to {gait_name}")
            self.send_simple(self.GAIT_CMDS[gait_name])
        else:
            self.get_logger().warning(f"Unknown gait: {gait_name}")

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().error("Emergency stop triggered")

    # ------------------------------------------------------------------
    # Velocity mux
    # ------------------------------------------------------------------
    def _clamp_twist(self, msg):
        out = Twist()
        out.linear.x = max(-1.0, min(1.0, msg.linear.x))
        out.linear.y = max(-0.5, min(0.5, msg.linear.y))
        out.angular.z = max(-1.5, min(1.5, msg.angular.z))
        return out

    def _select_velocity(self):
        now = self.get_clock().now()

        if self.emergency_stop:
            return Twist(), "emergency"

        manual_dt = (now - self.last_manual_time).nanoseconds / 1e9
        auto_dt = (now - self.last_auto_time).nanoseconds / 1e9

        # Manual /cmd_vel has priority for manual_override_timeout seconds.
        if manual_dt < self.manual_override_timeout:
            return self.manual_vel, "manual"

        if auto_dt < self.cmd_timeout:
            return self.auto_vel, "auto"

        return Twist(), "none"

    def _apply_obstacle_clamp(self, vx):
        if self.last_us_time is None:
            return vx
        dt = (self.get_clock().now() - self.last_us_time).nanoseconds / 1e9
        if dt > self.us_timeout:
            return vx

        if self.front_dist is not None and self.front_dist < self.obs_dist and vx > 0:
            return 0.0
        if self.back_dist is not None and self.back_dist < self.obs_dist and vx < 0:
            return 0.0
        return vx

    # ------------------------------------------------------------------
    # UDP protocol helpers
    # ------------------------------------------------------------------
    def send_simple(self, code, value=0):
        self.sock.sendto(
            struct.pack("<IiI", code, value, 0),
            (self.robot_ip, self.robot_port),
        )

    def send_complex(self, code, double_val):
        self.sock.sendto(
            struct.pack("<IIId", code, 8, 1, double_val),
            (self.robot_ip, self.robot_port),
        )

    def send_heartbeat(self):
        self.send_simple(0x21040001, 0)

    def send_velocity_loop(self):
        self.out_vel, self.source = self._select_velocity()
        self.out_vel.linear.x = self._apply_obstacle_clamp(self.out_vel.linear.x)
        self.send_complex(0x0140, self.out_vel.linear.x)
        self.send_complex(0x0145, self.out_vel.linear.y)
        self.send_complex(0x0141, self.out_vel.angular.z)

    # ------------------------------------------------------------------
    # State reception / odometry + ultrasound
    # ------------------------------------------------------------------
    def _read_double(self, data, idx):
        """读取 0x0901 包中第 idx 个 double（从偏移 20 开始）。"""
        offset = 20 + idx * 8
        if len(data) < offset + 8:
            return None
        return struct.unpack_from("<d", data, offset)[0]

    def receive_data_loop(self):
        try:
            while True:
                data, _ = self.sock.recvfrom(2048)
                if len(data) < 140:
                    continue

                code, _, cmd_type = struct.unpack_from("<IiI", data, 0)
                if code != 0x0901 or cmd_type != 1:
                    continue

                body = struct.unpack_from("<15d", data, 20)
                pos_x, pos_y, pos_yaw = body[9], body[10], body[11]
                vel_x, vel_y, vel_yaw = body[12], body[13], body[14]

                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = self.odom_frame_id
                odom.child_frame_id = self.base_frame_id
                odom.pose.pose.position.x = pos_x
                odom.pose.pose.position.y = pos_y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = quaternion_from_yaw(pos_yaw)
                odom.twist.twist.linear.x = vel_x
                odom.twist.twist.linear.y = vel_y
                odom.twist.twist.angular.z = vel_yaw

                self.odom_pub.publish(odom)

                # 超声波
                if self.front_index >= 0 or self.back_index >= 0 or self.us_debug:
                    n_doubles = (len(data) - 20) // 8
                    if self.us_debug:
                        if not self._warned_us_size:
                            self.get_logger().info(
                                f"0x0901 packet length={len(data)}, doubles={n_doubles}"
                            )
                            self._warned_us_size = True
                        if self.front_index < 0 and self.back_index < 0:
                            now = self.get_clock().now()
                            if (now - self._last_us_debug_log).nanoseconds / 1e9 >= 2.0:
                                self._last_us_debug_log = now
                                vals = [self._read_double(data, i) for i in range(n_doubles)]
                                self.get_logger().info(f"ultrasound debug values: {vals}")

                    now = self.get_clock().now()
                    if self.front_index >= 0 and self.front_index < n_doubles:
                        raw = self._read_double(data, self.front_index)
                        if raw is not None:
                            self.front_dist = raw * self.us_scale
                            self.front_pub.publish(
                                build_range("ultrasonic_front", now.to_msg(), self.front_dist, self.us_max_range)
                            )
                    if self.back_index >= 0 and self.back_index < n_doubles:
                        raw = self._read_double(data, self.back_index)
                        if raw is not None:
                            self.back_dist = raw * self.us_scale
                            self.back_pub.publish(
                                build_range("ultrasonic_back", now.to_msg(), self.back_dist, self.us_max_range)
                            )
                    self.last_us_time = now
        except (BlockingIOError, socket.error):
            pass

    # ------------------------------------------------------------------
    # Status diagnostics
    # ------------------------------------------------------------------
    def publish_status(self):
        now = self.get_clock().now()
        manual_dt = (now - self.last_manual_time).nanoseconds / 1e9
        auto_dt = (now - self.last_auto_time).nanoseconds / 1e9

        source = "none"
        if self.emergency_stop:
            source = "emergency"
        elif manual_dt < self.manual_override_timeout:
            source = "manual"
        elif auto_dt < self.cmd_timeout:
            source = "auto"

        front = self.front_dist if self.front_dist is not None else -1.0
        back = self.back_dist if self.back_dist is not None else -1.0

        msg = String()
        msg.data = (
            f"source={source} "
            f"vx={self.out_vel.linear.x:.3f} "
            f"vy={self.out_vel.linear.y:.3f} "
            f"w={self.out_vel.angular.z:.3f} "
            f"front={front:.2f} back={back:.2f}"
        )
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Initialization / shutdown
    # ------------------------------------------------------------------
    def _read_basic_state(self, per_recv_timeout=0.2):
        self.sock.settimeout(per_recv_timeout)
        try:
            data, _ = self.sock.recvfrom(2048)
            if len(data) >= 16:
                code = struct.unpack_from("<I", data, 0)[0]
                cmd_type = struct.unpack_from("<I", data, 8)[0]
                if code == 0x0901 and cmd_type == 1:
                    return struct.unpack_from("<i", data, 12)[0]
        except (socket.timeout, BlockingIOError, socket.error):
            pass
        finally:
            self.sock.setblocking(False)
        return None

    def _wait_for_state(self, target_state, timeout=8.0):
        self.get_logger().info(f"Waiting for robot_basic_state {target_state}...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self._read_basic_state(0.2)
            if state == target_state:
                self.get_logger().info(f"Robot state {target_state} reached")
                return True
        self.get_logger().warning(f"Timeout waiting for state {target_state}")
        return False

    def wakeup_robot(self):
        self.get_logger().info("Wakeup sequence started")
        self.send_simple(0x21010C05)          # return to zero
        self._wait_for_state(1, timeout=10.0)  # lying down

        self.send_simple(0x21010202)          # stand up
        self._wait_for_state(6, timeout=8.0)  # force-control standing

        self.send_simple(0x21010D06)          # switch to locomotion mode
        time.sleep(0.5)
        self.send_simple(0x21010C03)          # autonomous mode
        time.sleep(0.5)
        self.get_logger().info("Wakeup complete")

    def shutdown_robot(self):
        self.get_logger().info("Shutdown sequence started")
        self.manual_vel = Twist()
        self.auto_vel = Twist()
        time.sleep(0.1)
        self.send_velocity_loop()
        time.sleep(0.1)

        self.send_simple(0x21010C02)          # manual mode
        time.sleep(0.5)

        current_state = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            current_state = self._read_basic_state(0.3)
            if current_state is not None:
                break

        self.get_logger().info(f"State before shutdown: {current_state}")
        if current_state != 1:
            self.send_simple(0x21010202)      # lie down
            self._wait_for_state(1, timeout=5.0)

        self.sock.close()
        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = Lite3DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
