#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Yaw controller: unwrapped current + shortest-arc target.

Core flow: parse odom -> compute error -> PID -> publish /cmd_vel_auto.
Positive angular.z is clockwise on this robot.
"""

import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def nearest_angle(target, current):
    """Return target + 2*pi*k closest to current (minor arc)."""
    while target - current > math.pi:
        target -= 2.0 * math.pi
    while target - current < -math.pi:
        target += 2.0 * math.pi
    return target


def yaw(q):
    """Extract yaw from a quaternion with x=y=0."""
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def clamp(v, lim):
    return max(-lim, min(lim, v))


class YawController(Node):
    def __init__(self):
        super().__init__("yaw_controller")

        # 可调参数
        self.declare_parameter("kp", 2.0)                 # 比例增益
        self.declare_parameter("ki", 0.0)                 # 积分增益
        self.declare_parameter("kd", 0.3)                 # 微分增益（角速度反馈阻尼）
        self.declare_parameter("max_vel_yaw", 1.6)        # 最大偏航角速度
        self.declare_parameter("angle_threshold", 0.01)   # 死区进入阈值 (rad)
        self.declare_parameter("deadband_hysteresis", 1.5) # 死区退出滞环倍数
        self.declare_parameter("control_rate", 25.0)      # 控制频率 (Hz)
        self.declare_parameter("stale_odom_timeout", 0.3) # 里程计超时时间 (s)
        self.declare_parameter("gait", "slow")            # 启动默认步态

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value
        self.max_vel_yaw = self.get_parameter("max_vel_yaw").value
        self.thresh = self.get_parameter("angle_threshold").value
        self.hyst = self.get_parameter("deadband_hysteresis").value
        self.dt = 1.0 / self.get_parameter("control_rate").value
        self.stale = self.get_parameter("stale_odom_timeout").value
        self.gait = self.get_parameter("gait").value

        # 状态
        self._lock = threading.Lock()
        self._origin_yaw = None
        self._last_raw_yaw = None
        self._current_yaw = 0.0
        self._target_base = 0.0
        self._omega = 0.0
        self._integral = 0.0
        self._last_e = 0.0
        self._last_cmd = 0.0
        self._in_db = False
        self._estop = False
        self._source = "none"
        self._last_odom_time = None
        self._shutdown = False

        # ROS
        self.create_subscription(Odometry, "/leg_odom2", self._odom_cb, 10)
        self.create_subscription(Bool, "/emergency_stop", self._estop_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_auto", 10)
        self.gait_pub = self.create_publisher(String, "/cmd_gait", 10)

        self.create_timer(self.dt, self._control_cb)
        self.create_timer(0.1, self._display)

        threading.Thread(target=self._input_loop, daemon=True).start()
        self.get_logger().info("Yaw controller started; waiting for /leg_odom2 ...")

    # ---------- 感知 ----------
    def _odom_cb(self, msg: Odometry):
        raw = yaw(msg.pose.pose.orientation)
        with self._lock:
            self._omega = msg.twist.twist.angular.z
            self._last_odom_time = self.get_clock().now()
            if self._origin_yaw is None:
                self._origin_yaw = raw
                self._last_raw_yaw = raw
                self._current_yaw = 0.0
                self._target_base = 0.0
                self.gait_pub.publish(String(data=self.gait))
            else:
                self._current_yaw += normalize_angle(raw - self._last_raw_yaw)
                self._last_raw_yaw = raw

    def _estop_cb(self, msg: Bool):
        self._estop = msg.data

    def _odom_fresh(self):
        return self._last_odom_time is not None and \
               (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9 <= self.stale

    # ---------- 决策 ----------
    def _error(self):
        return nearest_angle(self._target_base, self._current_yaw) - self._current_yaw

    def _pid(self, e):
        exit_thr = self.thresh * self.hyst
        self._in_db = abs(e) <= self.thresh or (self._in_db and abs(e) <= exit_thr)

        if self._in_db:
            self._integral = self._last_cmd = 0.0
            self._last_e = e
            return 0.0

        if self.ki:
            self._integral = clamp(self._integral + self.ki * e * self.dt,
                                   self.max_vel_yaw / self.ki)

        # negative sign: e<0 -> clockwise -> positive angular.z
        u = -(self.kp * e + self.ki * self._integral) - self.kd * self._omega
        u = clamp(u, self.max_vel_yaw)
        self._last_e = e
        self._last_cmd = u
        return u

    # ---------- 执行 ----------
    def _publish_cmd(self, u):
        twist = Twist()
        twist.angular.z = u
        self.cmd_pub.publish(twist)

    def _control_cb(self):
        if self._estop or self._origin_yaw is None or not self._odom_fresh():
            self._publish_cmd(0.0)
            self._source = "estop" if self._estop else ("no_odom" if self._origin_yaw is None else "stale")
            return

        with self._lock:
            u = self._pid(self._error())
        self._publish_cmd(u)
        self._source = "auto"

    # ---------- 交互 ----------
    def _input_loop(self):
        while rclpy.ok() and not self._shutdown:
            try:
                line = input().strip()
            except EOFError:
                break
            self._handle(line)

    def _handle(self, cmd):
        c = cmd.lower()
        if c in ("q", "quit"):
            self._shutdown = True
        elif c in ("h", "help"):
            print("90/-45/180 target deg | c freeze | r reset origin | q quit")
        elif c == "c":
            with self._lock:
                self._target_base = normalize_angle(self._current_yaw)
                self._integral = 0.0
        elif c == "r":
            with self._lock:
                if self._last_raw_yaw is not None:
                    self._origin_yaw = self._last_raw_yaw
                    self._current_yaw = self._target_base = self._integral = 0.0
        else:
            try:
                deg = float(cmd)
            except ValueError:
                return
            with self._lock:
                self._target_base = normalize_angle(math.radians(deg))
                self._integral = 0.0

    def _display(self):
        with self._lock:
            origin = self._origin_yaw
            cur = self._current_yaw
            tgt = self._target_base
            e, u = self._last_e, self._last_cmd
        if origin is None:
            return
        origin_d = math.degrees(origin)
        cur_d = math.degrees(normalize_angle(origin + cur))
        tgt_d = math.degrees(normalize_angle(origin + tgt))
        print(f"origin={origin_d:7.2f}  current={cur_d:7.2f}  target={tgt_d:7.2f}")
        print(f"angle_err={math.degrees(e):6.2f}deg  cmd={u:+.4f}rad/s [{self._source}]")

    def shutdown(self):
        self._shutdown = True
        self.cmd_pub.publish(Twist())


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
