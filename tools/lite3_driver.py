#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Standalone Lite3 driver (no motion_test package needed).

Subscribes to /cmd_vel, /cmd_vel_auto, /cmd_gait, /emergency_stop and talks to
the robot directly over UDP.  Publishes /leg_odom2 and /driver_status.
"""

import math
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


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

        self.declare_parameter("robot_ip", "192.168.1.120")
        self.declare_parameter("robot_port", 43893)
        self.declare_parameter("heartbeat_period", 0.5)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("manual_override_timeout", 1.5)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_port = self.get_parameter("robot_port").value
        self.heartbeat_period = self.get_parameter("heartbeat_period").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.manual_override_timeout = self.get_parameter(
            "manual_override_timeout"
        ).value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        # Velocity sources
        self.manual_vel = Twist()
        self.auto_vel = Twist()
        self.last_manual_time = self.get_clock().now()
        self.last_auto_time = self.get_clock().now()
        self.emergency_stop = False
        self.out_vel = Twist()

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
        self.send_complex(0x0140, self.out_vel.linear.x)
        self.send_complex(0x0145, self.out_vel.linear.y)
        self.send_complex(0x0141, self.out_vel.angular.z)

    # ------------------------------------------------------------------
    # State reception / odometry
    # ------------------------------------------------------------------
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

        msg = String()
        msg.data = (
            f"source={source} "
            f"vx={self.out_vel.linear.x:.3f} "
            f"vy={self.out_vel.linear.y:.3f} "
            f"w={self.out_vel.angular.z:.3f}"
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
