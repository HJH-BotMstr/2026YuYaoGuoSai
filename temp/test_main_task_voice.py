#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main_task.py 免识别语音播报模式冒烟测试（无狗、无摄像头、无声卡）。

- VISION_ENABLED=False，播报表 ["AH", "BL", "CM"] 对应识别点 [2, 5, 15]
- play_mp3_blocking 替换为假实现（只记录文件名，不真的播放）
- FakePoseControl 模拟 pose_control：收到 /move 后 cmd_vel 先非零再归零
- 验证：播报顺序与识别点顺序一致；结果表正确；全程未加载 gauge_vision 模块
"""

import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

import rclpy
from geometry_msgs.msg import Pose2D, Twist

import way_point
import main_task

# 缩短等待时间
way_point.RESET_WAIT = 0.1
way_point.CMD_VEL_ZERO_TIMEOUT = 0.2
way_point.MOVE_TIMEOUT = 10.0
main_task.DETECT_SETTLE_SEC = 0.01

# 免识别模式 + 临时播报表
main_task.VISION_ENABLED = False
main_task.VOICE_BROADCAST_TABLE = ["AH", "BL", "CM"]

played = []
main_task.play_mp3_blocking = lambda p: played.append(Path(str(p)).name)

DETECT_POINTS = [2, 5, 15]


class FakePoseControl(rclpy.node.Node):
    """模拟 pose_control：收到 /move 后 cmd_vel 先非零再归零。"""

    def __init__(self):
        super().__init__("fake_pose_control")
        self.create_subscription(Pose2D, "/move", self._move_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _move_cb(self, msg):
        t = Twist()
        t.linear.x = 0.3
        self._cmd_pub.publish(t)
        threading.Timer(0.1, lambda: self._cmd_pub.publish(Twist())).start()


def main():
    rclpy.init(args=None)
    node = main_task.MainTaskNode(main_task.WAYPOINTS_PATH, DETECT_POINTS)
    fake = FakePoseControl()

    t0 = time.time()
    while rclpy.ok() and not node.done and time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.02)
        rclpy.spin_once(fake, timeout_sec=0.02)
        node.tick()

    print("--- 免识别模式结果 ---")
    print("done:", node.done)
    print("播报顺序:", played, "（预期 ['AH.mp3', 'BL.mp3', 'CM.mp3']）")
    print("最终结果表:", node._final_results,
          "（预期 {'A': 'abnormal', 'B': 'abnormal', 'C': 'normal'}）")
    print("gauge_vision 是否被加载:", "gauge_vision" in sys.modules, "（预期 False）")

    ok = (
        node.done
        and played == ["AH.mp3", "BL.mp3", "CM.mp3"]
        and node._final_results == {"A": "abnormal", "B": "abnormal", "C": "normal"}
        and "gauge_vision" not in sys.modules
    )
    node.destroy_node()
    fake.destroy_node()
    rclpy.shutdown()
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
