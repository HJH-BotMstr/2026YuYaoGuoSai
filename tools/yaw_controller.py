#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Interactive PI yaw controller based on /leg_odom2.

Assumes roll = pitch = 0 (quaternion x = y = 0).  Uses quaternion rotation
composition to avoid yaw wrap-around and gimbal-lock issues.  Publishes to
/cmd_vel_auto so that manual /cmd_vel (e.g. a remote controller) keeps priority.
"""

import sys
from pathlib import Path

# Allow this standalone tool to import motion_test utilities from source.
_MOTION_TEST_DIR = (
    Path(__file__).resolve().parent.parent
    / "assets"
    / "old_code"
    / "lite3"
    / "src"
    / "motion_test"
)
if str(_MOTION_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(_MOTION_TEST_DIR))

import math
import os
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from motion_test.safety_monitor import SafetyMonitor
from motion_test.transforms import quaternion_from_yaw, yaw_from_quaternion


HELP_TEXT = """
交互式偏航控制器命令：
  <角度>        设置相对启动原点的目标航向，单位度（例：90, -45, 180）
  c / current   冻结：目标设为当前航向，速度归零
  r / reset     重新捕获原点：将当前航向设为新原点和目标
  h / help      显示本帮助
  q / quit      安全退出
""".strip()


def quaternion_conjugate(q):
    """Return the conjugate of a geometry_msgs/Quaternion."""
    q_out = type(q)()
    q_out.x = -q.x
    q_out.y = -q.y
    q_out.z = -q.z
    q_out.w = q.w
    return q_out


def quaternion_multiply(q1, q2):
    """Hamilton product of two geometry_msgs/Quaternions."""
    x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        q_out = type(q1)()
        q_out.w = 1.0
        return q_out

    q_out = type(q1)()
    q_out.x = x / norm
    q_out.y = y / norm
    q_out.z = z / norm
    q_out.w = w / norm
    return q_out


class YawController(Node):
    def __init__(self):
        super().__init__("yaw_controller")

        # Parameters (defaults from motion_params.yaml)
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("max_vel_yaw", 0.5)
        self.declare_parameter("min_vel_yaw", 0.05)
        self.declare_parameter("yaw_threshold", 0.08)
        self.declare_parameter("control_rate", 25.0)
        self.declare_parameter("stale_odom_timeout", 0.3)
        self.declare_parameter("gait", "medium")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_auto")
        self.declare_parameter("cmd_gait_topic", "/cmd_gait")

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.max_vel_yaw = self.get_parameter("max_vel_yaw").value
        self.min_vel_yaw = self.get_parameter("min_vel_yaw").value
        self.yaw_threshold = self.get_parameter("yaw_threshold").value
        self.control_rate = self.get_parameter("control_rate").value
        self.stale_odom_timeout = self.get_parameter("stale_odom_timeout").value
        self.gait = self.get_parameter("gait").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_gait_topic = self.get_parameter("cmd_gait_topic").value

        # Shared state protected by self._lock
        self._lock = threading.Lock()
        self._q_origin = None
        self._q_target = None
        self._q_current = None
        self._integral = 0.0
        self._saturated = False
        self._last_error = 0.0
        self._last_omega = 0.0
        self._shutdown = False
        self._gait_sent = False

        # ROS interfaces
        self.create_subscription(
            Odometry, "/leg_odom2", self._odom_callback, 10
        )
        self.safety = SafetyMonitor(self, self.stale_odom_timeout)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_gait_pub = self.create_publisher(String, self.cmd_gait_topic, 10)

        self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.create_timer(0.1, self._display)

        # Interactive input thread
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info(
            "交互式偏航控制器已启动，等待 /leg_odom2 ...\n"
            "遥控器通过 /cmd_vel 仍拥有最高优先级。"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        with self._lock:
            self._q_current = q
            if self._q_origin is None:
                self._q_origin = q
                self._q_target = q
                origin_yaw = yaw_from_quaternion(q)
                self.get_logger().info(
                    f"原点已设定: yaw={math.degrees(origin_yaw):.2f}°"
                )
                if not self._gait_sent:
                    self.cmd_gait_pub.publish(String(data=self.gait))
                    self._gait_sent = True
                    self.get_logger().info(f"已发送步态: {self.gait}")

    # ------------------------------------------------------------------
    # Interactive input
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # PI control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        if self._shutdown:
            return

        if not self.safety.is_safe():
            self._zero_velocity()
            with self._lock:
                self._integral = 0.0
                self._saturated = False
            return

        with self._lock:
            q_current = self._q_current
            q_target = self._q_target

        if q_current is None or q_target is None:
            return

        # Error as the shortest rotation from current to target.
        q_error = quaternion_multiply(q_target, quaternion_conjugate(q_current))
        e = yaw_from_quaternion(q_error)

        dt = 1.0 / self.control_rate
        omega = 0.0

        if abs(e) <= self.yaw_threshold:
            with self._lock:
                self._integral = 0.0
                self._saturated = False
        else:
            with self._lock:
                if self.ki > 0.0 and not self._saturated:
                    self._integral += self.ki * e * dt
                    int_max = self.max_vel_yaw / self.ki
                    self._integral = max(-int_max, min(int_max, self._integral))

                omega_raw = self.kp * e + self.ki * self._integral
                omega_clamped = self._clamp(omega_raw, self.max_vel_yaw)
                self._saturated = abs(omega_raw) >= self.max_vel_yaw
                omega = self._apply_min(
                    omega_clamped, self.min_vel_yaw, self.max_vel_yaw
                )

        twist = Twist()
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

        with self._lock:
            self._last_error = e
            self._last_omega = omega

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------
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
        safe = self.safety.is_safe()

        self._clear_screen()
        print("=" * 55)
        print("  交互式 PI 偏航控制器")
        print("=" * 55)
        print(f"  原点 yaw : {origin_yaw:>10.2f}°")
        print(f"  当前 yaw : {current_yaw:>10.2f}°")
        print(f"  目标 yaw : {target_yaw:>10.2f}°")
        print(f"  误差     : {math.degrees(e):>10.2f}°")
        print(f"  积分项   : {integral:>10.4f}")
        print(f"  角速度指令: {omega:>10.4f} rad/s")
        print(f"  安全状态 : {'SAFE' if safe else 'UNSAFE'}")
        print("=" * 55)
        print(HELP_TEXT)
        print("\nCommand: ", end="", flush=True)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    @staticmethod
    def _clear_screen():
        os.system("clear" if os.name == "posix" else "cls")

    @staticmethod
    def _clamp(value, limit):
        """Clamp |value| to |limit|, preserving sign."""
        return max(-abs(limit), min(abs(limit), value))

    @staticmethod
    def _apply_min(value, min_val, max_val):
        """Apply minimum command magnitude when outside the dead zone."""
        if abs(value) < 1e-6:
            return 0.0
        clamped = YawController._clamp(value, max_val)
        if abs(clamped) < min_val:
            return math.copysign(min_val, clamped)
        return clamped

    def _zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def shutdown(self):
        self._shutdown = True
        self._zero_velocity()
        self._input_thread.join(timeout=0.5)


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
