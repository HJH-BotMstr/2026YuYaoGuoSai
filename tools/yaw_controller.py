#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Standalone interactive PI yaw controller + Lite3 driver.

This script does NOT depend on the motion_test package.  It talks to the robot
directly over UDP, wakes it up, sends gaits/velocities, receives leg odometry,
and runs a PI yaw controller.  It still publishes /leg_odom2 for debugging.

Assumptions:
- Roll = pitch = 0 (quaternion x = y = 0).
- /cmd_vel (e.g. from a remote controller) has priority over the PI controller.
"""

import math
import os
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


HELP_TEXT = """
交互式偏航控制器命令：
  <角度>        设置相对启动原点的目标航向，单位度（例：90, -45, 180）
  c / current   冻结：目标设为当前航向，速度归零
  r / reset     重新捕获原点：将当前航向设为新原点和目标
  h / help      显示本帮助
  q / quit      安全退出
""".strip()


# ------------------------------------------------------------------------------
# Math helpers
# ------------------------------------------------------------------------------
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    """Extract yaw assuming roll = pitch = 0."""
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_conjugate(q):
    q_out = Quaternion()
    q_out.x = -q.x
    q_out.y = -q.y
    q_out.z = -q.z
    q_out.w = q.w
    return q_out


def quaternion_multiply(q1, q2):
    x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        q_out = Quaternion()
        q_out.w = 1.0
        return q_out

    q_out = Quaternion()
    q_out.x = x / norm
    q_out.y = y / norm
    q_out.z = z / norm
    q_out.w = w / norm
    return q_out


def _clamp(value, limit):
    return max(-abs(limit), min(abs(limit), value))


def _apply_min(value, min_val, max_val):
    if abs(value) < 1e-6:
        return 0.0
    clamped = _clamp(value, max_val)
    if abs(clamped) < min_val:
        return math.copysign(min_val, clamped)
    return clamped


# ------------------------------------------------------------------------------
# Main node
# ------------------------------------------------------------------------------
class YawController(Node):
    GAIT_CMDS = {
        "slow": 0x21010300,
        "medium": 0x21010307,
        "fast": 0x21010303,
        "stair": 0x21010407,
        "crawl": 0x21010406,
    }

    def __init__(self):
        super().__init__("yaw_controller")

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter("robot_ip", "192.168.1.120")
        self.declare_parameter("robot_port", 43893)
        self.declare_parameter("heartbeat_period", 0.5)
        self.declare_parameter("manual_override_timeout", 1.5)
        self.declare_parameter("stale_odom_timeout", 0.3)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("max_vel_yaw", 0.5)
        self.declare_parameter("min_vel_yaw", 0.05)
        self.declare_parameter("yaw_threshold", 0.08)
        self.declare_parameter("control_rate", 25.0)
        self.declare_parameter("gait", "medium")

        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_port = self.get_parameter("robot_port").value
        self.heartbeat_period = self.get_parameter("heartbeat_period").value
        self.manual_override_timeout = self.get_parameter(
            "manual_override_timeout"
        ).value
        self.stale_odom_timeout = self.get_parameter("stale_odom_timeout").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.max_vel_yaw = self.get_parameter("max_vel_yaw").value
        self.min_vel_yaw = self.get_parameter("min_vel_yaw").value
        self.yaw_threshold = self.get_parameter("yaw_threshold").value
        self.control_rate = self.get_parameter("control_rate").value
        self.gait = self.get_parameter("gait").value

        # ----------------------------------------------------------------------
        # Shared state
        # ----------------------------------------------------------------------
        self._lock = threading.Lock()
        self._q_origin = None
        self._q_target = None
        self._q_current = None
        self._integral = 0.0
        self._saturated = False
        self._last_error = 0.0
        self._last_omega = 0.0
        self._shutdown = False
        self._last_udp_time = None
        self._source = "none"

        self._manual_vel = Twist()
        self._last_manual_time = self.get_clock().now()
        self._emergency = False

        # ----------------------------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------------------------
        self.create_subscription(Odometry, "/leg_odom2", self._odom_fallback_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self._emergency_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, "/leg_odom2", 10)
        self.status_pub = self.create_publisher(String, "/driver_status", 10)

        # ----------------------------------------------------------------------
        # UDP setup + wakeup
        # ----------------------------------------------------------------------
        self._init_udp()
        self._wakeup_robot()
        self._set_gait(self.gait)

        # ----------------------------------------------------------------------
        # Timers
        # ----------------------------------------------------------------------
        self.create_timer(self.heartbeat_period, self._send_heartbeat)
        self.create_timer(1.0 / self.control_rate, self._velocity_loop)
        self.create_timer(0.02, self._receive_data_loop)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.1, self._display)

        # ----------------------------------------------------------------------
        # Interactive input
        # ----------------------------------------------------------------------
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info(
            "独立偏航控制器已启动，机器人已唤醒。\n"
            "遥控器通过 /cmd_vel 拥有最高优先级。"
        )

    # --------------------------------------------------------------------------
    # UDP helpers
    # --------------------------------------------------------------------------
    def _init_udp(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", self.robot_port))
            self.get_logger().info(f"UDP 端口 {self.robot_port} 绑定成功")
        except OSError as e:
            self.get_logger().error(f"绑定 UDP 端口 {self.robot_port} 失败: {e}")
            self._shutdown = True
        self.sock.setblocking(False)

    def _send_simple(self, code, value=0):
        self.sock.sendto(
            struct.pack("<IiI", code, value, 0),
            (self.robot_ip, self.robot_port),
        )

    def _send_complex(self, code, double_val):
        self.sock.sendto(
            struct.pack("<IIId", code, 8, 1, double_val),
            (self.robot_ip, self.robot_port),
        )

    def _send_heartbeat(self):
        self._send_simple(0x21040001, 0)

    def _set_gait(self, gait_name):
        gait_name = gait_name.lower()
        if gait_name in self.GAIT_CMDS:
            self._send_simple(self.GAIT_CMDS[gait_name])
            self.get_logger().info(f"切换步态至: {gait_name}")
        else:
            self.get_logger().warn(f"未知步态: {gait_name}")

    def _send_velocity(self, vel: Twist):
        self._send_complex(0x0140, vel.linear.x)
        self._send_complex(0x0145, vel.linear.y)
        self._send_complex(0x0141, vel.angular.z)

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
        self.get_logger().info(f"等待机器人状态 {target_state} ...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self._read_basic_state(0.2)
            if state == target_state:
                self.get_logger().info(f"已到达状态 {target_state}")
                return True
            if state is not None:
                self.get_logger().info(f"当前状态: {state}")
        self.get_logger().warn(f"等待状态 {target_state} 超时 ({timeout}s)")
        return False

    def _wakeup_robot(self):
        self.get_logger().info("开始唤醒机器人 ...")
        self._send_simple(0x21010C05)  # return to zero
        self._wait_for_state(1, timeout=10.0)  # lying down

        self._send_simple(0x21010202)  # stand up
        self._wait_for_state(6, timeout=8.0)  # force-control standing

        self._send_simple(0x21010D06)  # locomotion mode
        time.sleep(0.5)
        self._send_simple(0x21010C03)  # autonomous mode
        time.sleep(0.5)
        self.get_logger().info("唤醒完成")

    def _shutdown_robot(self):
        self.get_logger().info("开始安全关机 ...")
        self._send_velocity(Twist())
        time.sleep(0.1)

        self._send_simple(0x21010C02)  # manual mode
        time.sleep(0.5)

        current_state = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            current_state = self._read_basic_state(0.3)
            if current_state is not None:
                break

        self.get_logger().info(f"关机前状态: {current_state}")
        if current_state != 1:
            self._send_simple(0x21010202)  # lie down
            self._wait_for_state(1, timeout=5.0)

        self.sock.close()
        self.get_logger().info("关机完成")

    # --------------------------------------------------------------------------
    # ROS callbacks
    # --------------------------------------------------------------------------
    def _odom_fallback_cb(self, msg: Odometry):
        """Optional: accept /leg_odom2 from another driver if present."""
        with self._lock:
            if self._q_current is None:
                self._q_current = msg.pose.pose.orientation
                self._q_origin = msg.pose.pose.orientation
                self._q_target = msg.pose.pose.orientation
                self._last_udp_time = self.get_clock().now()

    def _cmd_vel_callback(self, msg: Twist):
        manual = Twist()
        manual.linear.x = msg.linear.x
        manual.linear.y = msg.linear.y
        manual.linear.z = msg.linear.z
        manual.angular.x = msg.angular.x
        manual.angular.y = msg.angular.y
        manual.angular.z = msg.angular.z
        with self._lock:
            self._manual_vel = manual
            self._last_manual_time = self.get_clock().now()

    def _emergency_callback(self, msg: Bool):
        self._emergency = msg.data
        if self._emergency:
            self.get_logger().error("接收到紧急停止信号")

    # --------------------------------------------------------------------------
    # Data reception + odometry publishing
    # --------------------------------------------------------------------------
    def _receive_data_loop(self):
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

                now = self.get_clock().now()

                # Build and publish /leg_odom2
                odom = Odometry()
                odom.header.stamp = now.to_msg()
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

                # Update controller state
                q = odom.pose.pose.orientation
                with self._lock:
                    self._q_current = q
                    self._last_udp_time = now
                    if self._q_origin is None:
                        self._q_origin = q
                        self._q_target = q
                        self.get_logger().info(
                            f"原点已设定: yaw={math.degrees(pos_yaw):.2f}°"
                        )
        except (BlockingIOError, socket.error):
            pass

    # --------------------------------------------------------------------------
    # Velocity control loop
    # --------------------------------------------------------------------------
    def _velocity_loop(self):
        if self._shutdown:
            return

        if self._emergency:
            self._send_velocity(Twist())
            with self._lock:
                self._integral = 0.0
                self._saturated = False
                self._last_omega = 0.0
                self._source = "emergency"
            return

        now = self.get_clock().now()

        # Remote /cmd_vel has highest priority.
        manual_dt = (now - self._last_manual_time).nanoseconds / 1e9
        if manual_dt < self.manual_override_timeout:
            with self._lock:
                vel = Twist()
                vel.linear.x = self._manual_vel.linear.x
                vel.linear.y = self._manual_vel.linear.y
                vel.linear.z = self._manual_vel.linear.z
                vel.angular.x = self._manual_vel.angular.x
                vel.angular.y = self._manual_vel.angular.y
                vel.angular.z = self._manual_vel.angular.z
                self._integral = 0.0
                self._saturated = False
                self._source = "manual"
            self._send_velocity(vel)
            return

        # Auto PI yaw control
        with self._lock:
            q_current = self._q_current
            q_target = self._q_target
            last_udp_time = self._last_udp_time

        if q_current is None or q_target is None or last_udp_time is None:
            self._send_velocity(Twist())
            with self._lock:
                self._last_omega = 0.0
                self._source = "no_odom"
            return

        udp_dt = (now - last_udp_time).nanoseconds / 1e9
        if udp_dt > self.stale_odom_timeout:
            self._send_velocity(Twist())
            with self._lock:
                self._integral = 0.0
                self._saturated = False
                self._last_omega = 0.0
                self._source = "stale_odom"
            return

        omega = self._compute_omega(q_current, q_target)
        vel = Twist()
        vel.angular.z = omega
        self._send_velocity(vel)
        self._source = "auto"

    def _compute_omega(self, q_current, q_target):
        q_error = quaternion_multiply(q_target, quaternion_conjugate(q_current))
        e = yaw_from_quaternion(q_error)

        dt = 1.0 / self.control_rate
        omega = 0.0

        if abs(e) <= self.yaw_threshold:
            with self._lock:
                self._integral = 0.0
                self._saturated = False
                self._last_error = e
                self._last_omega = 0.0
        else:
            with self._lock:
                if self.ki > 0.0 and not self._saturated:
                    self._integral += self.ki * e * dt
                    int_max = self.max_vel_yaw / self.ki
                    self._integral = max(-int_max, min(int_max, self._integral))

                omega_raw = self.kp * e + self.ki * self._integral
                omega_clamped = _clamp(omega_raw, self.max_vel_yaw)
                self._saturated = abs(omega_raw) >= self.max_vel_yaw
                omega = _apply_min(omega_clamped, self.min_vel_yaw, self.max_vel_yaw)

            with self._lock:
                self._last_error = e
                self._last_omega = omega

        return omega

    # --------------------------------------------------------------------------
    # Status publishing
    # --------------------------------------------------------------------------
    def _publish_status(self):
        with self._lock:
            vel = Twist()
            vel.angular.z = self._last_omega
        msg = String()
        msg.data = (
            f"source={self._source} "
            f"vx={vel.linear.x:.3f} vy={vel.linear.y:.3f} w={vel.angular.z:.3f}"
        )
        self.status_pub.publish(msg)

    # --------------------------------------------------------------------------
    # Interactive input
    # --------------------------------------------------------------------------
    def _input_loop(self):
        while rclpy.ok() and not self._shutdown:
            try:
                line = input()
            except EOFError:
                break
            except Exception:
                continue
            self._handle_command(line.strip())

    def _handle_command(self, cmd: str):
        if not cmd:
            self._freeze()
            return

        lower = cmd.lower()
        if lower in ("q", "quit"):
            with self._lock:
                self._shutdown = True
        elif lower in ("h", "help"):
            self.get_logger().info("\n" + HELP_TEXT)
        elif lower in ("c", "current"):
            self._freeze()
        elif lower in ("r", "reset"):
            self._reset_origin()
        else:
            try:
                deg = float(cmd)
            except ValueError:
                self.get_logger().warn(f"未知命令: {cmd}")
                return
            self._set_target_degrees(deg)

    def _freeze(self):
        with self._lock:
            if self._q_current is None:
                return
            self._q_target = self._q_current
            self._integral = 0.0
            self._saturated = False
            yaw = math.degrees(yaw_from_quaternion(self._q_current))
        self.get_logger().info(f"冻结在当前航向: {yaw:.2f}°")

    def _reset_origin(self):
        with self._lock:
            if self._q_current is None:
                return
            self._q_origin = self._q_current
            self._q_target = self._q_current
            self._integral = 0.0
            self._saturated = False
            yaw = math.degrees(yaw_from_quaternion(self._q_current))
        self.get_logger().info(f"重新设定原点: {yaw:.2f}°")

    def _set_target_degrees(self, deg: float):
        with self._lock:
            if self._q_origin is None:
                self.get_logger().warn("尚未收到里程计，无法设定目标")
                return
            q_delta = quaternion_from_yaw(math.radians(deg))
            self._q_target = quaternion_multiply(q_delta, self._q_origin)
            self._integral = 0.0
            self._saturated = False
            target_yaw = math.degrees(yaw_from_quaternion(self._q_target))
        self.get_logger().info(f"新目标航向: {target_yaw:.2f}°")

    # --------------------------------------------------------------------------
    # Display
    # --------------------------------------------------------------------------
    def _display(self):
        with self._lock:
            q_current = self._q_current
            q_target = self._q_target
            q_origin = self._q_origin
            integral = self._integral
            e = self._last_error
            omega = self._last_omega

        if q_current is None or q_origin is None:
            return

        origin_yaw = math.degrees(yaw_from_quaternion(q_origin))
        current_yaw = math.degrees(yaw_from_quaternion(q_current))
        target_yaw = math.degrees(yaw_from_quaternion(q_target))

        self._clear_screen()
        print("=" * 55)
        print("  交互式 PI 偏航控制器（独立 UDP 驱动）")
        print("=" * 55)
        print(f"  原点 yaw : {origin_yaw:>10.2f}°")
        print(f"  当前 yaw : {current_yaw:>10.2f}°")
        print(f"  目标 yaw : {target_yaw:>10.2f}°")
        print(f"  误差     : {math.degrees(e):>10.2f}°")
        print(f"  积分项   : {integral:>10.4f}")
        print(f"  角速度指令: {omega:>10.4f} rad/s")
        print(f"  速度来源 : {self._source}")
        print("=" * 55)
        print(HELP_TEXT)
        print("\nCommand: ", end="", flush=True)

    @staticmethod
    def _clear_screen():
        os.system("clear" if os.name == "posix" else "cls")

    # --------------------------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------------------------
    def shutdown(self):
        with self._lock:
            self._shutdown = True
        self._send_velocity(Twist())
        self._shutdown_robot()
        self._input_thread.join(timeout=0.5)


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = YawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
