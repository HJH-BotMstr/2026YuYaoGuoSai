#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Waypoint recorder / broadcaster for pose_control.

Linux only. Run without arguments to select mode interactively, or pass
`record` / `broadcast` as the first argument.

Recording mode:
  S  set current pose as waypoint 0 (clears previous path)
  A  add current pose as next waypoint
  E  end recording and save waypoints + computed /move commands

Broadcast mode:
  Loads recorded /move triplets, sends reset_origin, then publishes the moves
  sequentially. Progresses to the next move once /cmd_vel has returned to zero
  (i.e. pose_control reports it has finished the current segment).

Usage:
  python tools/waypoint_recorder.py
  python tools/waypoint_recorder.py record [path.json]
  python tools/waypoint_recorder.py broadcast [path.json]
"""

import json
import math
import os
import queue
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_PATH = "waypoints.json"
RESET_WAIT = 1.0         # s
CMD_VEL_ZERO_TIMEOUT = 1.0  # s: consider segment done after zero cmd_vel this long
MOVE_TIMEOUT = 60.0      # s


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    """Extract yaw from a quaternion with x=y=0."""
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def world_to_body(dx_world, dy_world, yaw):
    """Rotate a world-frame 2D vector into the body frame.

    Body frame: +x forward, +y left.
    """
    dx_body = dx_world * math.cos(yaw) + dy_world * math.sin(yaw)
    dy_body = -dx_world * math.sin(yaw) + dy_world * math.cos(yaw)
    return dx_body, dy_body


def compute_moves(waypoints):
    """Compute sequential /move triplets from absolute waypoints.

    Each triplet is (x, y, theta) where x/y are body-frame displacement
    from waypoint i to i+1, and theta is the relative yaw rotation.
    """
    moves = []
    for i in range(len(waypoints) - 1):
        w0 = waypoints[i]
        w1 = waypoints[i + 1]
        dx_world = w1["x"] - w0["x"]
        dy_world = w1["y"] - w0["y"]
        dx_body, dy_body = world_to_body(dx_world, dy_world, w0["yaw"])
        dtheta = math.degrees(normalize_angle(w1["yaw"] - w0["yaw"]))
        moves.append({"x": dx_body, "y": dy_body, "theta": dtheta})
    return moves


def _getch():
    """Read a single character from stdin without waiting for Enter (Linux)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old)


def _input_thread(q):
    """Read single keystrokes and put them into the queue."""
    while True:
        try:
            q.put(_getch().lower())
        except EOFError:
            break


def _select_mode():
    """Interactive mode selection before ROS startup."""
    while True:
        choice = input("Select mode: [r]ecord / [b]roadcast: ").strip().lower()
        if choice in ("r", "record"):
            return "record"
        if choice in ("b", "broadcast"):
            return "broadcast"
        print("Invalid choice. Please enter 'r' or 'b'.")


def _select_filepath(default):
    """Interactive filepath selection before ROS startup."""
    path = input(f"Path file [{default}]: ").strip()
    return _normalize_filepath(path if path else default)


def _normalize_filepath(path):
    """Ensure the given path points to a JSON file.

    If a directory is given, use 'waypoints.json' inside it.
    If the path has no extension, append '.json'.
    """
    if os.path.isdir(path):
        return os.path.join(path, "waypoints.json")
    if os.path.splitext(path)[1] == "":
        return path + ".json"
    return path


def _ensure_dir(path):
    """Create parent directory for the given file path if it does not exist."""
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)


class WaypointTool(Node):
    def __init__(self, mode, filepath):
        super().__init__("waypoint_tool")
        self.mode = mode
        self.filepath = filepath
        self.done = False

        self._current = None
        self._lock = threading.Lock()

        self._waypoints = []
        self._moves = []

        # Broadcast state.
        self._broadcast_state = "init"      # init -> resetting -> moving -> waiting -> done
        self._broadcast_index = 0
        self._reset_time = None
        self._cmd_vel_zero_start = None
        self._cmd_vel_seen_nonzero = False
        self._last_cmd_vel = (0.0, 0.0, 0.0)

        self.create_subscription(Odometry, "/leg_odom2", self._odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self._move_pub = self.create_publisher(Pose2D, "/move", 10)
        self._cmd_pub = self.create_publisher(String, "/pose_control/command", 10)

        if mode == "record":
            self._cmd_queue = queue.Queue()
            threading.Thread(target=_input_thread, args=(self._cmd_queue,), daemon=True).start()
            self.get_logger().info(
                "Recording mode. Keys: S=start, A=add waypoint, E=end"
            )
        elif mode == "broadcast":
            self._load()
            self.get_logger().info(
                f"Broadcast mode. Loaded {len(self._moves)} moves"
            )
        else:
            raise ValueError(f"Unknown mode: {mode}")

    # ---------- ROS callbacks ----------
    def _odom_cb(self, msg: Odometry):
        raw_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._current = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": raw_yaw,
            }

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._last_cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    # ---------- Recording ----------
    def _current_waypoint(self):
        with self._lock:
            if self._current is None:
                return None
            return dict(self._current)

    def _set_start(self):
        wp = self._current_waypoint()
        if wp is None:
            self.get_logger().warn("No odometry received yet")
            return
        self._waypoints = [wp]
        self.get_logger().info(f"Start (waypoint 0): {wp}")

    def _add_waypoint(self):
        if not self._waypoints:
            self.get_logger().warn("Press S first to set start waypoint")
            return
        wp = self._current_waypoint()
        if wp is None:
            self.get_logger().warn("No odometry received yet")
            return
        self._waypoints.append(wp)
        idx = len(self._waypoints) - 1
        self.get_logger().info(f"Added waypoint {idx}: {wp}")

    def _end_record(self):
        if len(self._waypoints) < 2:
            self.get_logger().warn("Need at least 2 waypoints to save a path")
            self.done = True
            return
        self._moves = compute_moves(self._waypoints)
        data = {"waypoints": self._waypoints, "moves": self._moves}
        _ensure_dir(self.filepath)
        with open(self.filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        self.get_logger().info(
            f"Saved {len(self._waypoints)} waypoints to {self.filepath}"
        )
        for i, m in enumerate(self._moves):
            self.get_logger().info(
                f"  move {i}: x={m['x']:.3f}, y={m['y']:.3f}, theta={m['theta']:.2f}"
            )
        self.done = True

    def _handle_record_input(self):
        try:
            cmd = self._cmd_queue.get_nowait()
        except queue.Empty:
            return
        if cmd == "s":
            self._set_start()
        elif cmd == "a":
            self._add_waypoint()
        elif cmd == "e":
            self._end_record()
        else:
            self.get_logger().info("Unknown command. Use S/A/E")

    # ---------- Broadcast ----------
    def _load(self):
        if not os.path.isfile(self.filepath):
            raise FileNotFoundError(
                f"Path file not found: {self.filepath}. "
                "Record a path first or provide a valid JSON file."
            )
        with open(self.filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        self._moves = data.get("moves") or []
        if not self._moves:
            self._waypoints = data.get("waypoints", [])
            if len(self._waypoints) >= 2:
                self._moves = compute_moves(self._waypoints)
            else:
                raise ValueError(
                    f"No valid moves or waypoints in {self.filepath}"
                )

    def _send_reset_origin(self):
        self._cmd_pub.publish(String(data="reset_origin"))
        self.get_logger().info("Sent reset_origin")

    def _publish_move(self, move):
        msg = Pose2D()
        msg.x = move["x"]
        msg.y = move["y"]
        msg.theta = move["theta"]
        self._move_pub.publish(msg)
        self.get_logger().info(
            f"Published /move: x={msg.x:.3f}, y={msg.y:.3f}, theta={msg.theta:.2f}"
        )

    def _tick_broadcast(self):
        now = self.get_clock().now()

        if self._broadcast_state == "init":
            self._send_reset_origin()
            self._reset_time = now
            self._cmd_vel_seen_nonzero = False
            self._cmd_vel_zero_start = None
            self._broadcast_state = "resetting"

        elif self._broadcast_state == "resetting":
            if (now - self._reset_time).nanoseconds / 1e9 >= RESET_WAIT:
                self._broadcast_state = "moving"
                self._broadcast_index = 0

        elif self._broadcast_state == "moving":
            if self._broadcast_index >= len(self._moves):
                self.get_logger().info("Path broadcast complete")
                self.done = True
                return
            self._publish_move(self._moves[self._broadcast_index])
            self._wait_start = now
            self._cmd_vel_seen_nonzero = False
            self._cmd_vel_zero_start = None
            self._broadcast_state = "waiting"

        elif self._broadcast_state == "waiting":
            with self._lock:
                vx, vy, omega = self._last_cmd_vel
            is_zero = abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(omega) < 1e-3

            if not is_zero:
                self._cmd_vel_seen_nonzero = True
                self._cmd_vel_zero_start = None
            elif self._cmd_vel_seen_nonzero:
                if self._cmd_vel_zero_start is None:
                    self._cmd_vel_zero_start = now
                elif (now - self._cmd_vel_zero_start).nanoseconds / 1e9 >= CMD_VEL_ZERO_TIMEOUT:
                    self.get_logger().info(f"Move {self._broadcast_index} complete")
                    self._broadcast_index += 1
                    self._broadcast_state = "moving"
                    self._cmd_vel_seen_nonzero = False
                    self._cmd_vel_zero_start = None

            if (now - self._wait_start).nanoseconds / 1e9 >= MOVE_TIMEOUT:
                self.get_logger().warn(f"Timeout on move {self._broadcast_index}, skipping")
                self._broadcast_index += 1
                self._broadcast_state = "moving"
                self._cmd_vel_seen_nonzero = False
                self._cmd_vel_zero_start = None

    # ---------- Main loop ----------
    def tick(self):
        if self.mode == "record":
            self._handle_record_input()
        elif self.mode == "broadcast":
            self._tick_broadcast()


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) >= 1:
        mode = argv[0].lower()
        filepath = _normalize_filepath(argv[1]) if len(argv) > 1 else DEFAULT_PATH
    else:
        mode = _select_mode()
        filepath = _select_filepath(DEFAULT_PATH)

    if mode not in ("record", "broadcast"):
        print(f"Unknown mode: {mode}")
        return 1

    rclpy.init(args=None)
    node = WaypointTool(mode, filepath)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
