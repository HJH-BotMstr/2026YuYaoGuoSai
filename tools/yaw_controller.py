#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lite3 interactive motion controller.

Supports yaw hold/rotate and body-relative x/y translations using /leg_odom2.
Macro commands: x+0.5, x-0.5, y+0.1, y-0.1, yaw90, 90, c, r, q, h.
Positive angular.z is clockwise on this robot.
"""

import math
import re
import threading

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def nearest_angle(target, current):
    while target - current > math.pi:
        target -= 2.0 * math.pi
    while target - current < -math.pi:
        target += 2.0 * math.pi
    return target


def yaw(q):
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def clamp(v, lim):
    return max(-lim, min(lim, v))


def apply_min(v, min_v, max_v):
    if abs(v) < 1e-6:
        return 0.0
    c = clamp(v, max_v)
    if abs(c) < min_v:
        return math.copysign(min_v, c)
    return c


class MotionController(Node):
    RE_XY = re.compile(r"^([xy])([+-]\d+(?:\.\d+)?)$")
    RE_YAW = re.compile(r"^(?:yaw\s*)?([+-]?\d+(?:\.\d+)?)$")

    def __init__(self):
        super().__init__("lite3_motion_controller")

        # 可调参数
        # yaw PID
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.3)
        self.declare_parameter("max_vel_yaw", 1.6)
        self.declare_parameter("angle_threshold", 0.01)
        self.declare_parameter("deadband_hysteresis", 1.5)
        # 位置控制
        self.declare_parameter("max_vel_x", 0.3)
        self.declare_parameter("max_vel_y", 0.2)
        self.declare_parameter("min_vel_x", 0.05)
        self.declare_parameter("min_vel_y", 0.05)
        self.declare_parameter("kp_dist", 1.0)
        self.declare_parameter("kp_lateral", 1.0)
        self.declare_parameter("dist_threshold", 0.05)
        self.declare_parameter("lateral_threshold", 0.03)
        self.declare_parameter("yaw_threshold", 0.08)
        # 运行参数
        self.declare_parameter("control_rate", 25.0)
        self.declare_parameter("stale_odom_timeout", 0.3)
        self.declare_parameter("gait", "slow")
        self.declare_parameter("motion_timeout_buffer", 5.0)
        self.declare_parameter("progress_timeout", 3.0)
        self.declare_parameter("progress_threshold", 0.01)
        # 超声波
        self.declare_parameter("ultrasound_front_topic", "/ultrasonic/front")
        self.declare_parameter("ultrasound_back_topic", "/ultrasonic/back")
        self.declare_parameter("obstacle_stop_distance", 0.2)
        self.declare_parameter("obstacle_safety_margin", 0.05)

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value
        self.max_vel_yaw = self.get_parameter("max_vel_yaw").value
        self.thresh = self.get_parameter("angle_threshold").value
        self.hyst = self.get_parameter("deadband_hysteresis").value

        self.max_vel_x = self.get_parameter("max_vel_x").value
        self.max_vel_y = self.get_parameter("max_vel_y").value
        self.min_vel_x = self.get_parameter("min_vel_x").value
        self.min_vel_y = self.get_parameter("min_vel_y").value
        self.kp_dist = self.get_parameter("kp_dist").value
        self.kp_lateral = self.get_parameter("kp_lateral").value
        self.dist_threshold = self.get_parameter("dist_threshold").value
        self.lateral_threshold = self.get_parameter("lateral_threshold").value
        self.yaw_threshold = self.get_parameter("yaw_threshold").value

        self.dt = 1.0 / self.get_parameter("control_rate").value
        self.stale = self.get_parameter("stale_odom_timeout").value
        self.gait = self.get_parameter("gait").value
        self.motion_buffer = self.get_parameter("motion_timeout_buffer").value
        self.progress_timeout = self.get_parameter("progress_timeout").value
        self.progress_threshold = self.get_parameter("progress_threshold").value

        self.obs_dist = self.get_parameter("obstacle_stop_distance").value
        self.obs_margin = self.get_parameter("obstacle_safety_margin").value

        # 状态
        self._lock = threading.Lock()
        self._origin_x = None
        self._origin_y = None
        self._origin_yaw = None
        self._last_raw_yaw = None
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._omega = 0.0

        self._mode = "idle"
        self._start_x = 0.0
        self._start_y = 0.0
        self._start_yaw = 0.0
        self._dx = 0.0
        self._dy = 0.0
        self._target_base = 0.0
        self._target_x = 0.0
        self._target_y = 0.0

        self._integral = 0.0
        self._last_e = 0.0
        self._last_cmd = 0.0
        self._in_db = False
        self._estop = False
        self._source = "none"
        self._last_odom_time = None
        self._shutdown = False

        self._deadline = None
        self._progress_val = 0.0
        self._progress_time = None

        self._front_dist = None
        self._back_dist = None

        # ROS
        self.create_subscription(Odometry, "/leg_odom2", self._odom_cb, 10)
        self.create_subscription(Bool, "/emergency_stop", self._estop_cb, 10)
        front_topic = self.get_parameter("ultrasound_front_topic").value
        back_topic = self.get_parameter("ultrasound_back_topic").value
        self.create_subscription(Range, front_topic, self._front_cb, 10)
        self.create_subscription(Range, back_topic, self._back_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_auto", 10)
        self.gait_pub = self.create_publisher(String, "/cmd_gait", 10)

        self.create_timer(self.dt, self._control_cb)
        self.create_timer(0.1, self._display)

        threading.Thread(target=self._input_loop, daemon=True).start()
        self.get_logger().info("Motion controller started; waiting for /leg_odom2 ...")

    # ---------- 感知 ----------
    def _odom_cb(self, msg: Odometry):
        raw = yaw(msg.pose.pose.orientation)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._lock:
            self._omega = msg.twist.twist.angular.z
            self._last_odom_time = self.get_clock().now()
            if self._origin_yaw is None:
                self._origin_x = x
                self._origin_y = y
                self._origin_yaw = raw
                self._last_raw_yaw = raw
                self._current_x = 0.0
                self._current_y = 0.0
                self._current_yaw = 0.0
                self._target_base = 0.0
                self._target_x = 0.0
                self._target_y = 0.0
                self.gait_pub.publish(String(data=self.gait))
            else:
                self._current_yaw += normalize_angle(raw - self._last_raw_yaw)
                self._last_raw_yaw = raw
                self._current_x = x - self._origin_x
                self._current_y = y - self._origin_y

    def _estop_cb(self, msg: Bool):
        self._estop = msg.data

    def _front_cb(self, msg: Range):
        self._front_dist = msg.range

    def _back_cb(self, msg: Range):
        self._back_dist = msg.range

    def _odom_fresh(self):
        return self._last_odom_time is not None and \
               (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9 <= self.stale

    # ---------- 决策 ----------
    def _error_yaw(self, target):
        return nearest_angle(target, self._current_yaw) - self._current_yaw

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

        u = -(self.kp * e + self.ki * self._integral) - self.kd * self._omega
        u = clamp(u, self.max_vel_yaw)
        self._last_e = e
        self._last_cmd = u
        return u

    def _project_start(self, x, y):
        """Return (traveled, lateral) relative to start pose."""
        dx = x - self._start_x
        dy = y - self._start_y
        c = math.cos(self._start_yaw)
        s = math.sin(self._start_yaw)
        traveled = dx * c + dy * s
        lateral = -dx * s + dy * c
        return traveled, lateral

    def _set_mode(self, mode):
        self._mode = mode
        self._integral = 0.0
        self._deadline = None
        self._progress_val = 0.0
        self._progress_time = self.get_clock().now()

    def _start_motion_deadline(self, expected_duration):
        self._deadline = self.get_clock().now() + Duration(seconds=expected_duration + self.motion_buffer)

    def _check_progress(self, current_val):
        now = self.get_clock().now()
        if (now - self._progress_time).nanoseconds / 1e9 > self.progress_timeout:
            if (self._progress_val - current_val) < self.progress_threshold:
                return False
            self._progress_val = current_val
            self._progress_time = now
        return True

    def _abort(self, reason):
        self.get_logger().warning(f"Motion aborted: {reason}")
        self._set_mode("idle")
        self._publish_cmd(0.0, 0.0, 0.0)
        self._source = "abort"

    # ---------- 执行 ----------
    def _publish_cmd(self, vx, vy, omega):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.cmd_pub.publish(twist)

    def _control_cb(self):
        if self._estop or self._origin_yaw is None or not self._odom_fresh():
            self._publish_cmd(0.0, 0.0, 0.0)
            self._source = "estop" if self._estop else ("no_odom" if self._origin_yaw is None else "stale")
            return

        with self._lock:
            if self._mode == "idle":
                self._publish_cmd(0.0, 0.0, 0.0)
                self._source = "idle"
                return

            vx = vy = omega = 0.0
            done = False

            if self._mode == "rotate":
                e_yaw = self._error_yaw(self._target_base)
                omega = self._pid(e_yaw)
                done = abs(e_yaw) < self.yaw_threshold
                if not self._check_progress(abs(e_yaw)):
                    self._abort("rotate progress timeout")
                    return

            elif self._mode == "move_x":
                traveled, lateral = self._project_start(self._current_x, self._current_y)
                e_dist = self._dx - traveled
                e_yaw = self._error_yaw(self._start_yaw)
                vx = apply_min(self.kp_dist * e_dist, self.min_vel_x, self.max_vel_x)
                vy = -clamp(self.kp_lateral * lateral, self.max_vel_y)
                omega = self._pid(e_yaw)
                done = (abs(e_dist) < self.dist_threshold and
                        abs(lateral) < self.lateral_threshold and
                        abs(e_yaw) < self.yaw_threshold)
                if not self._check_progress(abs(e_dist)):
                    self._abort("move_x progress timeout")
                    return

            elif self._mode == "move_y":
                traveled, lateral = self._project_start(self._current_x, self._current_y)
                e_left = self._dy - lateral
                e_forward = traveled
                e_yaw = self._error_yaw(self._start_yaw)
                vy = apply_min(self.kp_dist * e_left, self.min_vel_y, self.max_vel_y)
                vx = -clamp(self.kp_lateral * e_forward, self.max_vel_x)
                omega = self._pid(e_yaw)
                done = (abs(e_left) < self.dist_threshold and
                        abs(e_forward) < self.lateral_threshold and
                        abs(e_yaw) < self.yaw_threshold)
                if not self._check_progress(abs(e_left)):
                    self._abort("move_y progress timeout")
                    return

            else:
                done = True

            if self._deadline is not None and (self.get_clock().now() - self._deadline).nanoseconds > 0:
                self._abort("motion timeout")
                return

            if done:
                self._set_mode("idle")
                self._publish_cmd(0.0, 0.0, 0.0)
                self._source = "done"
                return

            self._publish_cmd(vx, vy, omega)
            self._source = self._mode

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
            print("x+0.5/x-0.5  y+0.1/y-0.1  yaw90/90  | c freeze | r reset origin | q quit")
        elif c == "c":
            with self._lock:
                self._target_base = normalize_angle(self._current_yaw)
                self._target_x = self._current_x
                self._target_y = self._current_y
                self._set_mode("idle")
        elif c == "r":
            with self._lock:
                if self._last_raw_yaw is not None:
                    self._origin_x += self._current_x
                    self._origin_y += self._current_y
                    self._origin_yaw = self._last_raw_yaw
                    self._current_x = 0.0
                    self._current_y = 0.0
                    self._current_yaw = 0.0
                    self._target_base = 0.0
                    self._target_x = 0.0
                    self._target_y = 0.0
                    self._set_mode("idle")
        else:
            m = self.RE_XY.match(cmd)
            if m:
                self._start_translation(m.group(1), float(m.group(2)))
                return
            m = self.RE_YAW.match(cmd)
            if m:
                self._start_rotation(float(m.group(1)))

    def _start_translation(self, axis, delta):
        with self._lock:
            if self._origin_yaw is None:
                print("No odometry yet")
                return
            # 起步前检查前后障碍
            if axis == "x":
                if delta > 0 and self._front_dist is not None and self._front_dist < self.obs_dist + self.obs_margin:
                    print(f"Front obstacle too close: {self._front_dist:.2f} m")
                    return
                if delta < 0 and self._back_dist is not None and self._back_dist < self.obs_dist + self.obs_margin:
                    print(f"Back obstacle too close: {self._back_dist:.2f} m")
                    return
            self._start_x = self._current_x
            self._start_y = self._current_y
            self._start_yaw = self._current_yaw
            c = math.cos(self._start_yaw)
            s = math.sin(self._start_yaw)
            if axis == "x":
                self._dx = delta
                self._dy = 0.0
                self._target_x = self._start_x + delta * c
                self._target_y = self._start_y + delta * s
                duration = abs(delta) / self.max_vel_x
                self._set_mode("move_x")
            else:
                self._dx = 0.0
                self._dy = delta
                self._target_x = self._start_x - delta * s
                self._target_y = self._start_y + delta * c
                duration = abs(delta) / self.max_vel_y
                self._set_mode("move_y")
            self._target_base = self._start_yaw
            self._start_motion_deadline(duration)
            self._progress_val = abs(delta)

    def _start_rotation(self, deg):
        with self._lock:
            if self._origin_yaw is None:
                print("No odometry yet")
                return
            self._target_base = normalize_angle(math.radians(deg))
            self._target_x = self._current_x
            self._target_y = self._current_y
            self._set_mode("rotate")
            e = abs(self._error_yaw(self._target_base))
            self._start_motion_deadline(e / self.max_vel_yaw)
            self._progress_val = e

    def _display(self):
        with self._lock:
            origin = self._origin_yaw
            cur = self._current_yaw
            tgt_yaw = self._target_base
            e, u = self._last_e, self._last_cmd
            x, y = self._current_x, self._current_y
            tx, ty = self._target_x, self._target_y
            mode = self._mode
            front = self._front_dist if self._front_dist is not None else -1.0
            back = self._back_dist if self._back_dist is not None else -1.0
        if origin is None:
            return
        origin_d = math.degrees(origin)
        cur_d = math.degrees(normalize_angle(origin + cur))
        tgt_d = math.degrees(normalize_angle(origin + tgt_yaw))
        print(f"origin=({self._origin_x:.2f},{self._origin_y:.2f},{origin_d:.2f})  "
              f"current=({x:.3f},{y:.3f},{cur_d:.2f})  target=({tx:.3f},{ty:.3f},{tgt_d:.2f})")
        print(f"mode={mode}  err=({math.degrees(e):.2f}deg)  cmd={u:+.4f}rad/s [{self._source}]  "
              f"us=({front:.2f},{back:.2f})")

    def shutdown(self):
        self._shutdown = True
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
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
