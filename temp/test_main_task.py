#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main_task.py 状态机冒烟测试（无狗、无摄像头）。

- 用假 gauge_vision 模块替换视觉（记录 init/recognize/shutdown 调用）
- 用本进程内另一个节点模拟 pose_control：收到 /move 后先发非零 /cmd_vel 再归零
- 验证：识别点在发对应 move 之前触发；识别重试逻辑生效；
  最后一个识别点完成后摄像头关闭；全部 move 走完后 done
"""

import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))


# ---------- 假视觉模块 ----------
class FakeGV:
    def __init__(self):
        self.inited = False
        self.shutdown_called = 0
        self.recognize_calls = 0
        self.results = {}

    def init(self, **kw):
        self.inited = True

    def recognize(self, retries=3, retry_interval=0.4):
        self.recognize_calls += 1
        # 前两次识别不到字母，测试重试；之后识别出 'A'
        if self.recognize_calls <= 2:
            return {"letter": None, "zone": None, "state": None}
        self.results["A"] = "normal"
        return {"letter": "A", "zone": "GREEN", "state": "normal"}

    def get_all(self):
        return dict(self.results)

    def get_state(self, letter):
        return self.results.get(letter)

    def shutdown(self):
        self.shutdown_called += 1


fake_gv = FakeGV()
sys.modules["gauge_vision"] = fake_gv

import rclpy
from geometry_msgs.msg import Pose2D, Twist

import way_point
import main_task

# 缩短等待时间
way_point.RESET_WAIT = 0.1
way_point.CMD_VEL_ZERO_TIMEOUT = 0.2
way_point.MOVE_TIMEOUT = 10.0
main_task.DETECT_SETTLE_SEC = 0.01
main_task.DETECT_ROUND_INTERVAL = 0.01

DETECT_POINTS = [2, 5, 15]  # 15 = 识别点之一（需 < 当前 waypoints.json 点数）

# 预期 move 段数从 waypoints.json 动态读取（重录点位后无需改测试）
import json as _json
with open(PROJECT_ROOT / "tools" / "waypoints.json", "r", encoding="utf-8") as _f:
    EXPECTED_MOVES = len(_json.load(_f).get("moves", []))


class FakePoseControl(rclpy.node.Node):
    """模拟 pose_control：收到 /move 后 cmd_vel 先非零再归零。"""

    def __init__(self):
        super().__init__("fake_pose_control")
        self.moves_received = []
        self.create_subscription(Pose2D, "/move", self._move_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _move_cb(self, msg):
        self.moves_received.append((msg.x, msg.y, msg.theta))
        t = Twist()
        t.linear.x = 0.3
        self._cmd_pub.publish(t)
        # 0.1s 后归零
        def stop():
            self._cmd_pub.publish(Twist())
        threading.Timer(0.1, stop).start()


def main():
    rclpy.init(args=None)
    node = main_task.MainTaskNode(main_task.WAYPOINTS_PATH, DETECT_POINTS)
    fake = FakePoseControl()

    # 识别发生时的 move 下标记录：包一层 _maybe_detect
    detect_at = []
    orig = node._maybe_detect
    def spy(wp_idx):
        if wp_idx in node._detect_points and wp_idx not in node._detected:
            detect_at.append((wp_idx, node._broadcast_index, list(node._moves and [node._broadcast_index])))
        orig(wp_idx)
    node._maybe_detect = spy

    t0 = time.time()
    while rclpy.ok() and not node.done and time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.02)
        rclpy.spin_once(fake, timeout_sec=0.02)
        node.tick()

    print("--- 结果 ---")
    print("done:", node.done)
    print("moves 发布数:", len(fake.moves_received), f"（预期 {EXPECTED_MOVES}）")
    print("识别触发点:", [d[0] for d in detect_at], "（预期 [2, 5, 15]）")
    print("recognize 调用次数:", fake_gv.recognize_calls, "（预期 2+1+1+1=5：前两次失败）")
    print("shutdown 调用次数:", fake_gv.shutdown_called, "（预期 1，全部识别点后关摄像头）")
    print("最终结果表:", node._final_results)

    ok = (
        node.done
        and len(fake.moves_received) == EXPECTED_MOVES
        and [d[0] for d in detect_at] == [2, 5, 15]
        and fake_gv.recognize_calls == 5
        and fake_gv.shutdown_called == 1
        and node._final_results == {"A": "normal"}
    )
    node.destroy_node()
    fake.destroy_node()
    rclpy.shutdown()
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
