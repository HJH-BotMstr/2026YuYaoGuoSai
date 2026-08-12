#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动 ROS 冒烟：只启动 abcd_task_node（无 pose_control / grasp_task / apriltag），
然后另一进程 fake 出 /leg_odom2、/cmd_vel、/grasp/state、/apriltag_place1/done，
观察 abcd_task_node 能不能推进单轮流程。

不由 colcon test 自动跑（需要多进程 + 时间），仅供本地调试：

  T1: source install/setup.bash && ros2 run abcd_task abcd_task_node \\
      --ros-args --params-file src/grasp/abcd_task/config/abcd_task.yaml \\
      -p dry_run_nav:=true

  T2: python3 src/grasp/abcd_task/test/manual_smoke_ros.py

  T2 会：
    - 以 100Hz 发 /leg_odom2 (Odometry)
    - 定期发 /cmd_vel（模拟运动结束后归零）
    - 发 /grasp/state=STANDBY（让 abcd_task 通过 _wait_grasp_standby）
    - 每次收到 /move 后 sleep 1s + 发 cmd_vel=0（模拟到位）
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FakePeers(Node):
    def __init__(self):
        super().__init__("abcd_smoke_peers")
        self._pub_odom = self.create_publisher(Odometry, "/leg_odom2", 10)
        self._pub_cmdvel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_grasp_state = self.create_publisher(String, "/grasp/state", 10)
        self.create_subscription(Pose2D, "/move", self._on_move, 10)
        self.create_subscription(String, "/pose_control/command", self._on_cmd, 10)
        self.create_subscription(Bool, "/apriltag_place1/start", self._on_apriltag, 10)
        self.create_subscription(Bool, "/block_align/start", self._on_block, 10)

        self._motion_state = "idle"  # idle / moving
        self._motion_end_at = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # 100Hz 发 odom + cmd_vel（cmd_vel 只在 moving 时非零）
        self.create_timer(0.01, self._pub_periodic)
        # 1Hz 发 STANDBY
        self.create_timer(1.0, lambda: self._pub_grasp_state.publish(String(data="STANDBY")))
        self.get_logger().info("fake peers ready")

    def _on_move(self, msg: Pose2D):
        self.get_logger().info(
            f"[fake] 收到 /move x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.1f}, 模拟 1.5s 后到位"
        )
        self._motion_state = "moving"
        self._motion_end_at = time.monotonic() + 1.5

    def _on_cmd(self, msg: String):
        self.get_logger().info(f"[fake] 收到 /pose_control/command: {msg.data}")

    def _on_apriltag(self, msg: Bool):
        self.get_logger().info(f"[fake] 收到 /apriltag_place1/start = {msg.data}")

    def _on_block(self, msg: Bool):
        self.get_logger().info(f"[fake] 收到 /block_align/start = {msg.data}")

    def _pub_periodic(self):
        now = time.monotonic()
        odom = Odometry()
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        # 姿态四元数：只 z 非零（yaw 平面）
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        self._pub_odom.publish(odom)

        twist = Twist()
        if self._motion_state == "moving":
            if now < self._motion_end_at:
                twist.linear.x = 0.1
            else:
                self._motion_state = "idle"
        self._pub_cmdvel.publish(twist)


def main():
    rclpy.init()
    node = FakePeers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
