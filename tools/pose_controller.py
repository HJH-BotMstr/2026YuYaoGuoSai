#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pose controller: position + yaw closed-loop with ultrasonic obstacle avoidance.

Uses the robot's official ROS2 stack:
  - /leg_odom2           -> odometry
  - /us_publisher/ultrasound_distance -> rear ultrasonic (Float64)
  - /cmd_vel             -> velocity commands

Front obstacle avoidance is left for the depth camera in the future.

Supports terminal meta-commands:
  x+0.5   move forward 0.5 m
  x-0.5   move backward 0.5 m
  y+0.1   move left 0.1 m
  y-0.1   move right 0.1 m
  yaw90   rotate to absolute 90 deg (clockwise positive, like yaw_controller)
  90      alias for yaw90
  c       cancel current motion / freeze
  r       reset origin to current pose
  q       quit
"""

import math
import re
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


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


def yaw_from_quaternion(q):
    """Extract yaw from a quaternion with x=y=0."""
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def clamp(v, lim):
    return max(-lim, min(lim, v))


def transform_body_to_world(vx_body, vy_body, yaw):
    """Rotate a body-frame 2D vector into the world frame.

    +x_body = forward, +y_body = left.
    """
    vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
    vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
    return vx_world, vy_world


class PoseController(Node):
    # Command regex: x+0.5, y-0.1, yaw90, or just 90
    MOVE_RE = re.compile(r"^([xy])([+-]?\d+(?:\.\d+)?)$", re.IGNORECASE)
    YAW_RE = re.compile(r"^(?:yaw)?([+-]?\d+(?:\.\d+)?)$", re.IGNORECASE)

    def __init__(self):
        super().__init__("pose_controller")

        # Tunable parameters
        self.declare_parameter("kp_dist", 1.0)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("kd_yaw", 0.3)
        self.declare_parameter("kp_lateral", 1.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("max_vel_x", 0.3)
        self.declare_parameter("max_vel_y", 0.2)
        self.declare_parameter("max_vel_yaw", 1.6)
        self.declare_parameter("dist_threshold", 0.05)
        self.declare_parameter("yaw_threshold", 0.05)
        self.declare_parameter("angle_threshold", 0.01)
        self.declare_parameter("deadband_hysteresis", 1.5)
        self.declare_parameter("control_rate", 25.0)
        self.declare_parameter("stale_odom_timeout", 0.3)
        self.declare_parameter("gait", "slow")
        self.declare_parameter("obstacle_stop_dist", 0.35)
        self.declare_parameter("obstacle_resume_hyst", 0.05)
        self.declare_parameter("sonar_topic", "/us_publisher/ultrasound_distance")
        self.declare_parameter("sonar_is_rear", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.kp_dist = self.get_parameter("kp_dist").value
        self.kp_yaw = self.get_parameter("kp_yaw").value
        self.kd_yaw = self.get_parameter("kd_yaw").value
        self.kp_lateral = self.get_parameter("kp_lateral").value
        self.ki_yaw = self.get_parameter("ki_yaw").value
        self.max_vel_x = self.get_parameter("max_vel_x").value
        self.max_vel_y = self.get_parameter("max_vel_y").value
        self.max_vel_yaw = self.get_parameter("max_vel_yaw").value
        self.dist_threshold = self.get_parameter("dist_threshold").value
        self.yaw_threshold = self.get_parameter("yaw_threshold").value
        self.angle_threshold = self.get_parameter("angle_threshold").value
        self.deadband_hysteresis = self.get_parameter("deadband_hysteresis").value
        self.dt = 1.0 / self.get_parameter("control_rate").value
        self.stale_odom_timeout = self.get_parameter("stale_odom_timeout").value
        self.gait = self.get_parameter("gait").value
        self.obstacle_stop_dist = self.get_parameter("obstacle_stop_dist").value
        self.obstacle_resume_hyst = self.get_parameter("obstacle_resume_hyst").value
        self.sonar_topic = self.get_parameter("sonar_topic").value
        self.sonar_is_rear = self.get_parameter("sonar_is_rear").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        # State
        self._lock = threading.Lock()
        self._origin = None
        self._current = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._last_raw_yaw = None
        self._omega = 0.0
        self._target = None
        self._start_pose = None
        self._state = "idle"
        self._integral = 0.0
        self._last_e_yaw = 0.0
        self._in_db = False
        self._front_blocked = False
        self._rear_blocked = False
        self._rear_dist = float("inf")
        self._estop = False
        self._last_odom_time = None
        self._shutdown = False
        self._last_cmd = (0.0, 0.0, 0.0)
        self._source = "none"

        # ROS
        self.create_subscription(Odometry, "/leg_odom2", self._odom_cb, 10)
        self.create_subscription(Bool, "/emergency_stop", self._estop_cb, 10)
        self.create_subscription(Float64, self.sonar_topic, self._sonar_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.gait_pub = self.create_publisher(String, "/cmd_gait", 10)

        self.create_timer(self.dt, self._control_cb)
        self.create_timer(0.1, self._display)

        threading.Thread(target=self._input_loop, daemon=True).start()
        self.get_logger().info("Pose controller started; waiting for /leg_odom2 ...")

    # ---------- Perception ----------
    def _odom_cb(self, msg: Odometry):
        raw_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._omega = msg.twist.twist.angular.z
            self._last_odom_time = self.get_clock().now()
            self._current["x"] = msg.pose.pose.position.x
            self._current["y"] = msg.pose.pose.position.y

            if self._origin is None:
                self._origin = {
                    "x": self._current["x"],
                    "y": self._current["y"],
                    "yaw": raw_yaw,
                }
                self._last_raw_yaw = raw_yaw
                self._current["yaw"] = 0.0
                self.gait_pub.publish(String(data=self.gait))
            else:
                self._current["yaw"] += normalize_angle(raw_yaw - self._last_raw_yaw)
                self._last_raw_yaw = raw_yaw

    def _estop_cb(self, msg: Bool):
        self._estop = msg.data

    def _sonar_cb(self, msg: Float64):
        # The official topic currently only carries rear distance.
        direction = "rear" if self.sonar_is_rear else "front"
        self._update_blocked(msg.data, direction)

    def _update_blocked(self, distance, direction):
        stop = self.obstacle_stop_dist
        hyst = self.obstacle_resume_hyst
        with self._lock:
            if direction == "front":
                if distance <= stop:
                    self._front_blocked = True
                elif distance > stop + hyst:
                    self._front_blocked = False
            else:
                self._rear_dist = distance
                if distance <= stop:
                    self._rear_blocked = True
                elif distance > stop + hyst:
                    self._rear_blocked = False

    def _odom_fresh(self):
        return (
            self._last_odom_time is not None
            and (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
            <= self.stale_odom_timeout
        )

    # ---------- Command handling ----------
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
            return
        if c in ("h", "help"):
            print(
                "x+0.5/x-0.5  y+0.1/y-0.1  yaw90/90  c cancel  r reset origin  q quit"
            )
            return
        if c == "c":
            with self._lock:
                self._state = "idle"
                self._target = None
                self._integral = 0.0
                self._publish_cmd(0.0, 0.0, 0.0)
            return
        if c == "r":
            with self._lock:
                if self._last_raw_yaw is not None:
                    self._origin = {
                        "x": self._current["x"],
                        "y": self._current["y"],
                        "yaw": self._last_raw_yaw,
                    }
                    self._current["yaw"] = 0.0
                    self._integral = 0.0
                    self._target = None
                    self._state = "idle"
            return

        move_match = self.MOVE_RE.match(c)
        if move_match:
            axis = move_match.group(1).lower()
            value = float(move_match.group(2))
            self._set_move_target(axis, value)
            return

        yaw_match = self.YAW_RE.match(c)
        if yaw_match:
            deg = float(yaw_match.group(1))
            self._set_yaw_target(deg)
            return

    def _set_move_target(self, axis, value):
        with self._lock:
            start = dict(self._current)
            if axis == "x":
                dx_body, dy_body = value, 0.0
                self._state = "moving_x"
            else:
                dx_body, dy_body = 0.0, value
                self._state = "moving_y"

            dx_world, dy_world = transform_body_to_world(
                dx_body, dy_body, start["yaw"]
            )
            self._target = {
                "x": start["x"] + dx_world,
                "y": start["y"] + dy_world,
                "yaw": start["yaw"],
            }
            self._start_pose = start
            self._integral = 0.0
            self._in_db = False

    def _set_yaw_target(self, deg):
        with self._lock:
            target_yaw = normalize_angle(self._origin["yaw"] + math.radians(deg))
            self._target = {
                "x": self._current["x"],
                "y": self._current["y"],
                "yaw": target_yaw,
            }
            self._state = "rotating"
            self._integral = 0.0
            self._in_db = False

    # ---------- Control ----------
    def _error_yaw(self, target_yaw, current_yaw):
        return nearest_angle(target_yaw, current_yaw) - current_yaw

    def _pid_yaw(self, e):
        exit_thr = self.angle_threshold * self.deadband_hysteresis
        self._in_db = abs(e) <= self.angle_threshold or (
            self._in_db and abs(e) <= exit_thr
        )

        if self._in_db:
            self._integral = 0.0
            self._last_e_yaw = e
            return 0.0

        if self.ki_yaw:
            self._integral = clamp(
                self._integral + self.ki_yaw * e * self.dt,
                self.max_vel_yaw / max(self.ki_yaw, 1e-6),
            )

        # Official ROS stack: angular.z > 0 increases yaw (counter-clockwise).
        u = (self.kp_yaw * e + self.ki_yaw * self._integral) - self.kd_yaw * self._omega
        u = clamp(u, self.max_vel_yaw)
        self._last_e_yaw = e
        return u

    def _compute_cmd(self):
        cur = self._current
        tgt = self._target
        state = self._state

        if state == "moving_x":
            travel_dir = self._start_pose["yaw"]
            e_forward = (tgt["x"] - cur["x"]) * math.cos(travel_dir) + (
                tgt["y"] - cur["y"]
            ) * math.sin(travel_dir)
            e_lateral = -(tgt["x"] - cur["x"]) * math.sin(travel_dir) + (
                tgt["y"] - cur["y"]
            ) * math.cos(travel_dir)
            e_yaw = normalize_angle(travel_dir - cur["yaw"])

            vx_body = clamp(self.kp_dist * e_forward, self.max_vel_x)
            vy_body = clamp(self.kp_lateral * e_lateral, self.max_vel_y)
            omega = clamp(self.kp_yaw * e_yaw, self.max_vel_yaw)

        elif state == "moving_y":
            travel_dir = normalize_angle(self._start_pose["yaw"] + math.pi / 2.0)
            e_forward = (tgt["x"] - cur["x"]) * math.cos(travel_dir) + (
                tgt["y"] - cur["y"]
            ) * math.sin(travel_dir)
            e_lateral = -(tgt["x"] - cur["x"]) * math.sin(travel_dir) + (
                tgt["y"] - cur["y"]
            ) * math.cos(travel_dir)
            e_yaw = normalize_angle(self._start_pose["yaw"] - cur["yaw"])

            # Small correction along x to stay on the lateral line.
            vx_body = clamp(-self.kp_lateral * e_lateral, self.max_vel_x)
            vy_body = clamp(self.kp_dist * e_forward, self.max_vel_y)
            omega = clamp(self.kp_yaw * e_yaw, self.max_vel_yaw)

        elif state == "rotating":
            e_yaw = self._error_yaw(tgt["yaw"], cur["yaw"])
            omega = self._pid_yaw(e_yaw)
            vx_body = 0.0
            vy_body = 0.0

        else:
            return 0.0, 0.0, 0.0

        return vx_body, vy_body, omega

    def _apply_obstacle_clamp(self, vx_body, vy_body, omega):
        if self._state == "moving_x":
            if vx_body > 0 and self._front_blocked:
                vx_body = 0.0
            if vx_body < 0 and self._rear_blocked:
                vx_body = 0.0
        return vx_body, vy_body, omega

    def _check_arrived(self, vx_body, vy_body, omega):
        if self._state in ("moving_x", "moving_y"):
            cur = self._current
            tgt = self._target
            if self._state == "moving_x":
                travel_dir = self._start_pose["yaw"]
            else:
                travel_dir = normalize_angle(self._start_pose["yaw"] + math.pi / 2.0)
            e_forward = (tgt["x"] - cur["x"]) * math.cos(travel_dir) + (
                tgt["y"] - cur["y"]
            ) * math.sin(travel_dir)
            e_yaw = normalize_angle(self._start_pose["yaw"] - cur["yaw"])
            return abs(e_forward) < self.dist_threshold and abs(e_yaw) < self.yaw_threshold
        elif self._state == "rotating":
            e_yaw = self._error_yaw(self._target["yaw"], self._current["yaw"])
            return abs(e_yaw) < self.yaw_threshold
        return False

    def _publish_cmd(self, vx, vy, omega):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.cmd_pub.publish(twist)
        self._last_cmd = (vx, vy, omega)

    def _control_cb(self):
        if self._estop or self._origin is None or not self._odom_fresh():
            self._publish_cmd(0.0, 0.0, 0.0)
            self._source = (
                "estop"
                if self._estop
                else ("no_odom" if self._origin is None else "stale")
            )
            return

        with self._lock:
            if self._state == "idle" or self._target is None:
                self._publish_cmd(0.0, 0.0, 0.0)
                self._source = "idle"
                return

            vx_body, vy_body, omega = self._compute_cmd()
            vx_body, vy_body, omega = self._apply_obstacle_clamp(
                vx_body, vy_body, omega
            )

            if self._check_arrived(vx_body, vy_body, omega):
                self._publish_cmd(0.0, 0.0, 0.0)
                self._state = "idle"
                self._target = None
                self._integral = 0.0
                self._source = "arrived"
                return

            self._publish_cmd(vx_body, vy_body, omega)
            self._source = "auto"

    # ---------- Display ----------
    def _display(self):
        with self._lock:
            origin = self._origin
            cur = dict(self._current)
            tgt = self._target
            state = self._state
            rear = self._rear_blocked
            rear_dist = self._rear_dist
            vx, vy, omega = self._last_cmd
            source = self._source

        if origin is None:
            return

        cur_yaw_deg = math.degrees(normalize_angle(origin["yaw"] + cur["yaw"]))
        origin_deg = math.degrees(origin["yaw"])
        if tgt is not None:
            tgt_yaw_deg = math.degrees(normalize_angle(tgt["yaw"]))
            print(
                f"state={state:8s}  "
                f"origin=({origin_deg:7.2f}°)  "
                f"cur=({cur['x']:.3f},{cur['y']:.3f},{cur_yaw_deg:7.2f}°)  "
                f"tgt=({tgt['x']:.3f},{tgt['y']:.3f},{tgt_yaw_deg:7.2f}°)"
            )
        else:
            print(
                f"state={state:8s}  "
                f"origin=({origin_deg:7.2f}°)  "
                f"cur=({cur['x']:.3f},{cur['y']:.3f},{cur_yaw_deg:7.2f}°)"
            )
        print(
            f"cmd=({vx:+.3f},{vy:+.3f},{omega:+.4f})  "
            f"rear_blocked={rear} rear_dist={rear_dist:.2f}m  [{source}]"
        )

    def shutdown(self):
        self._shutdown = True
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PoseController()
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
