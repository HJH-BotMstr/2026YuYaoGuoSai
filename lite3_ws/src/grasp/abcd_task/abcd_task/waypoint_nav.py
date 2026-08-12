#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_nav — 单段绝对里程计导航封装

封装 pose_controller 的 /move 接口，让 abcd_task_node 用 (x, y, yaw) 世界坐标
表达目标点，内部完成：
  1. 检查里程计新鲜（防止用旧位姿算 world_to_body）
  2. world 坐标增量 → body 坐标增量
  3. 发布 Pose2D 到 /move（theta 单位是度，pose_controller 会做相对旋转）
  4. 等 /cmd_vel 归零（先非零后持续为零 duration_s）判定到位
  5. 支持超时和取消

纯函数 world_to_body / normalize_angle 与 tools/way_point.py 保持一致；
tools/way_point.py 目前不是可 import 的 Python 包，此处内联复制约 30 行。
"""

import math
import threading
import time
from collections import deque
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


# ─── 纯函数：与 tools/way_point.py 保持一致 ─────────────────────────────────── #

def normalize_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q) -> float:
    """从四元数（假设 x=y=0）提取 yaw（弧度）。"""
    return math.atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)


def world_to_body(dx_world: float, dy_world: float, yaw: float) -> Tuple[float, float]:
    """
    世界坐标增量 → 机体坐标增量。
    机体坐标约定：+x 前，+y 左（Lite3 body frame）。
    """
    c = math.cos(yaw)
    s = math.sin(yaw)
    dx_body =  c * dx_world + s * dy_world
    dy_body = -s * dx_world + c * dy_world
    return dx_body, dy_body


# ─── 常量 ─────────────────────────────────────────────────────────────────── #

# 与 way_point.py 一致的 cmd_vel 归零阈值
_CMDVEL_ZERO_THRESHOLD = 1e-3
# 归零持续时间：cmd_vel 三个分量都小于阈值持续这么久才算到位
_CMDVEL_ZERO_DURATION_S_DEFAULT = 1.0
# reset_origin 与后续 /move 之间的短暂等待，防话题竞争（参考 apriltag_place1）
_RESET_ORIGIN_SETTLE_S = 0.15


class WaypointNavigator:
    """
    单段绝对里程计导航封装。

    使用方式：
        nav = WaypointNavigator(node)
        nav.reset_origin()
        ok = nav.navigate_to({"x": 1.0, "y": 0.5, "yaw": 0.0}, timeout_s=30.0)

    线程安全性：该类由 rclpy 单线程 executor 回调驱动。navigate_to() 会自旋
    等待 cmd_vel 归零，期间必须让 executor 继续跑，因此调用方要么在独立
    线程调用 navigate_to()，要么用 rclpy.spin_once 在自旋循环里驱动。
    这里采用后者：navigate_to 内部循环 rclpy.spin_once(node, timeout_sec=0.05)。
    """

    def __init__(
        self,
        node: Node,
        topic_move: str = "/move",
        topic_pose_cmd: str = "/pose_control/command",
        topic_odom: str = "/leg_odom2",
        topic_cmd_vel: str = "/cmd_vel",
        odom_fresh_timeout_s: float = 1.0,
        cmd_vel_zero_duration_s: float = _CMDVEL_ZERO_DURATION_S_DEFAULT,
    ):
        self._node = node
        self._logger = node.get_logger()

        self._odom_fresh_timeout_s = float(odom_fresh_timeout_s)
        self._cmdvel_zero_duration_s = float(cmd_vel_zero_duration_s)

        # 位姿状态
        self._pose_lock = threading.Lock()
        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._current_yaw: float = 0.0
        self._last_odom_time: float = 0.0

        # cmd_vel 历史（用于归零判定）
        self._cmdvel_history: deque = deque(maxlen=64)
        self._last_cmdvel_time: float = 0.0
        self._motion_started: bool = False

        # 发布器
        self._pub_move = node.create_publisher(Pose2D, topic_move, 10)
        self._pub_cmd = node.create_publisher(String, topic_pose_cmd, 10)

        # 订阅器
        self._sub_odom = node.create_subscription(
            Odometry, topic_odom, self._odom_cb, 10)
        self._sub_cmdvel = node.create_subscription(
            Twist, topic_cmd_vel, self._cmdvel_cb, 10)

    # ─── 回调 ─────────────────────────────────────────────────────────── #

    def _odom_cb(self, msg: Odometry) -> None:
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._pose_lock:
            self._current_x = float(msg.pose.pose.position.x)
            self._current_y = float(msg.pose.pose.position.y)
            self._current_yaw = float(yaw)
            self._last_odom_time = time.monotonic()

    def _cmdvel_cb(self, msg: Twist) -> None:
        now = time.monotonic()
        speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        self._cmdvel_history.append((now, speed))
        self._last_cmdvel_time = now
        if speed >= 0.01 and not self._motion_started:
            self._motion_started = True

    # ─── 状态查询 ─────────────────────────────────────────────────────── #

    def odom_fresh(self) -> bool:
        """里程计是否新鲜。"""
        with self._pose_lock:
            if self._last_odom_time <= 0.0:
                return False
            return (time.monotonic() - self._last_odom_time) < self._odom_fresh_timeout_s

    def current_pose(self) -> Dict[str, float]:
        """快照当前世界坐标位姿。"""
        with self._pose_lock:
            return {
                "x": self._current_x,
                "y": self._current_y,
                "yaw": self._current_yaw,
            }

    # ─── 运动指令 ─────────────────────────────────────────────────────── #

    def reset_origin(self) -> None:
        """
        向 pose_controller 发 reset_origin，把当前位置作为 /move 的相对起点。
        """
        self._pub_cmd.publish(String(data="reset_origin"))
        self._logger.info("发布 reset_origin")
        # 保留 apriltag_place1 用过的经验：reset_origin 和后续 /move 走两条话题
        # 不保序，先睡一小会儿让 pose_controller 消化完 reset_origin 再发 /move
        time.sleep(_RESET_ORIGIN_SETTLE_S)

    def send_move(self, x_body: float, y_body: float, theta_deg: float) -> None:
        """低层发送 Pose2D 到 /move；不等待到位。x/y 单位米，theta 单位度。"""
        msg = Pose2D()
        msg.x = float(x_body)
        msg.y = float(y_body)
        msg.theta = float(theta_deg)
        self._pub_move.publish(msg)
        # 重置运动完成检测
        self._motion_started = False
        self._cmdvel_history.clear()
        self._logger.info(
            f"发布 /move  x={x_body:.3f}m  y={y_body:.3f}m  theta={theta_deg:.1f}°"
        )

    def send_zero_move(self) -> None:
        """紧急停车：发一次全零 /move。"""
        self.send_move(0.0, 0.0, 0.0)

    # ─── 到位判定 ─────────────────────────────────────────────────────── #

    def is_cmd_vel_zero(self) -> bool:
        """判断最近 cmd_vel_zero_duration_s 内速度是否持续为零。

        必须先出现过非零速度（_motion_started），才认为运动真正完成——
        否则刚发 /move 但 pose_controller 尚未启动时会立刻误判到位。
        """
        if not self._motion_started:
            return False
        now = time.monotonic()
        cutoff = now - self._cmdvel_zero_duration_s
        recent = [(t, v) for (t, v) in self._cmdvel_history if t >= cutoff]
        if len(recent) < 2:
            return False
        return all(v < _CMDVEL_ZERO_THRESHOLD for (_, v) in recent)

    # ─── 一体化导航 ───────────────────────────────────────────────────── #

    def navigate_to(
        self,
        target: Dict[str, float],
        timeout_s: float = 30.0,
        should_abort=None,
    ) -> bool:
        """
        导航到 world 绝对坐标 target={x, y, yaw}（yaw 弧度）。

        调用方必须保证在**后台 executor**（如 SingleThreadedExecutor.spin 于
        独立线程）中驱动 self._node，否则本方法内部的 busy-wait 无法收到
        /leg_odom2、/cmd_vel 更新，会一直判定为"未到位"直到超时。

        返回：
            True  = cmd_vel 归零判定到位
            False = 超时 / 里程计丢失 / 被 should_abort 打断
        """
        for key in ("x", "y", "yaw"):
            if key not in target:
                raise ValueError(f"target 缺少字段: {key}")

        # 里程计新鲜性
        if not self.odom_fresh():
            self._logger.error(
                f"里程计不新鲜（>{self._odom_fresh_timeout_s:.1f}s 未收到 /leg_odom2）"
            )
            return False

        # reset_origin 把当前位置作为相对起点
        self.reset_origin()

        # 现在快照位姿，作为 world_to_body 的参考
        pose = self.current_pose()
        dx_world = float(target["x"]) - pose["x"]
        dy_world = float(target["y"]) - pose["y"]
        dyaw = normalize_angle(float(target["yaw"]) - pose["yaw"])

        dx_body, dy_body = world_to_body(dx_world, dy_world, pose["yaw"])
        dtheta_deg = math.degrees(dyaw)

        # 零距离特判：如果位移和角度都很小，直接认为到位（避免 pose_controller 不响应导致超时）
        distance = math.sqrt(dx_world * dx_world + dy_world * dy_world)
        if distance < 0.05 and abs(dyaw) < math.radians(5.0):
            self._logger.info(
                f"navigate_to 起点即终点（距离={distance:.3f}m, 角度={math.degrees(abs(dyaw)):.1f}°），无需移动"
            )
            return True

        self._logger.info(
            f"navigate_to target=(x={target['x']:.3f}, y={target['y']:.3f}, "
            f"yaw={target['yaw']:.3f}) from (x={pose['x']:.3f}, y={pose['y']:.3f}, "
            f"yaw={pose['yaw']:.3f}) → body(x={dx_body:.3f}, y={dy_body:.3f}, "
            f"θ={dtheta_deg:.1f}°)"
        )

        # 发送运动指令
        self.send_move(dx_body, dy_body, dtheta_deg)

        return self._wait_arrive(timeout_s, should_abort, "navigate_to")

    def move_relative_body(
        self,
        dx_body: float,
        dy_body: float,
        dtheta_deg: float = 0.0,
        timeout_s: float = 15.0,
        should_abort=None,
    ) -> bool:
        """
        直接在 body 坐标系走一段增量（不依赖里程计），用于 retreat 等场景。

        流程：reset_origin → send_move(dx_body, dy_body, dtheta_deg) → 等 cmd_vel 归零。
        对 executor 的假设与 navigate_to 相同。
        """
        self.reset_origin()
        self.send_move(dx_body, dy_body, dtheta_deg)
        return self._wait_arrive(timeout_s, should_abort, "move_relative_body")

    # ─── 到位等待（executor 驱动版）────────────────────────────────────── #

    def _wait_arrive(self, timeout_s: float, should_abort, tag: str) -> bool:
        """busy-wait 到位：假设 executor 在后台线程 spin，不主动 spin_once。"""
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok():
            if should_abort is not None and should_abort():
                self._logger.warning(f"{tag} 被取消")
                self.send_zero_move()
                return False

            if not self.odom_fresh():
                self._logger.error(f"{tag} 期间里程计丢失，紧急停车")
                self.send_zero_move()
                return False

            if self.is_cmd_vel_zero():
                self._logger.info(f"{tag} 到位（/cmd_vel 归零）")
                return True

            if time.monotonic() > deadline:
                self._logger.error(f"{tag} 超时（>{timeout_s:.1f}s）")
                self.send_zero_move()
                return False

            time.sleep(0.05)

        return False
