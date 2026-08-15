#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""apriltag_place1_node 核心策略离线自测（无摄像头、无真机）。

验证点：
  1. 检测层：目标 ID 过滤、短暂丢失沿用缓存（fresh=False）、超期判真丢失；
  2. 分相闭环全流程（wait_detect→yaw→lateral→approach→final_check→blind→done），
     校验 yaw 单步限幅、lateral 方向、approach 闭环、盲进距离公式、抓取信号发布；
  3. 对齐阶段目标真丢失 → 统一回 wait_detect；
  4. final_check 某项不达标 → 回 yaw_align 重新修正；
  5. 单相超过 max_rounds → error；
  6. 盲进距离异常（<=0）→ error。

运动用理想世界模型模拟（指令即执行），cmd_vel 零速判定与链路预检打桩通过。
"""

import math
import sys
import time
from collections import deque
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/apriltag_place1')

import rclpy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
import apriltag_place1.apriltag_place1_node as M  # noqa: E402

failures = []


def check(name, cond, detail=''):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}  {detail}")
    if not cond:
        failures.append(name)


def make_node():
    """离线节点：摄像头打不开只报错，不影响逻辑；链路预检/零速判定打桩。"""
    node = M.AprilTagPlace1Node()
    node._show_debug = False
    node._check_motion_pipeline = lambda: (True, [])
    node._is_cmd_vel_zero = lambda: True
    return node


def trigger(node):
    node._trigger_cb(Bool(data=True))


def run_fsm(node, max_steps=400):
    """驱动状态机直到终态或步数上限。"""
    for _ in range(max_steps):
        if step_fsm(node):
            return node._state
    return "step_limit"


def step_fsm(node):
    """复刻 _main_loop 的状态分发，单步执行；返回 True 表示已到终态。"""
    frame = np.zeros((480, 640, 3), np.uint8)
    st = node._state
    if st in (M.STATE_WAIT_TRIGGER, M.STATE_DONE, M.STATE_ERROR):
        return True
    if st == M.STATE_WAIT_DETECT:
        node._do_wait_detect(frame)
    elif st == M.STATE_YAW_ALIGN:
        node._do_yaw_align(frame)
    elif st == M.STATE_LATERAL_ALIGN:
        node._do_lateral_align(frame)
    elif st == M.STATE_APPROACH:
        node._do_approach(frame)
    elif st == M.STATE_FINAL_CHECK:
        node._do_final_check(frame)
    elif st == M.STATE_BLIND_FORWARD:
        node._do_blind_forward()
    return False


# ──────────────────────────── 1. 检测层缓存 ──────────────────────────────────── #

def test_detect_cache():
    print("── 场景 1：检测层 ID 过滤与丢失缓存 ──")
    node = make_node()
    frame = np.zeros((480, 640, 3), np.uint8)

    def fake_tag(tag_id, tx, tz):
        return SimpleNamespace(
            tag_id=tag_id,
            pose_t=np.array([[tx], [0.0], [tz]]),
            pose_R=np.eye(3),
            corners=np.array([[0, 0], [10, 0], [10, 10], [0, 10]]),
        )

    detected = [fake_tag(0, 0.05, 0.80)]
    node._detector.detect = lambda grey, **kw: list(detected)

    pose = node._detect_tag(frame)
    check('目标 id=0 命中且 fresh=True',
          pose is not None and pose["raw"]["fresh"]
          and abs(pose["tx"] - 0.05) < 1e-9 and abs(pose["tz"] - 0.80) < 1e-9,
          f"pose={pose}")

    detected.clear()                      # Tag 消失
    pose2 = node._detect_tag(frame)
    check('丢失后沿用缓存位姿（fresh=False）',
          pose2 is not None and not pose2["raw"]["fresh"]
          and abs(pose2["tz"] - 0.80) < 1e-9)

    node._last_valid_time = time.monotonic() - (node._lost_tolerance + 1.0)
    pose3 = node._detect_tag(frame)
    check('超过 lost_tolerance 判真丢失（返回 None）', pose3 is None)

    detected.append(fake_tag(7, 0.10, 0.60))   # 只有别的 ID
    pose4 = node._detect_tag(frame)
    check('非目标 ID 不误锁（返回 None）', pose4 is None)
    check('诊断记录最近 IDs', node._last_detected_tag_ids == [7],
          f"ids={node._last_detected_tag_ids}")

    node.destroy_node()


# ──────────────────────── 2. 分相闭环全流程（happy path）──────────────────────── #

def test_full_pipeline():
    print("── 场景 2：分相闭环全流程 ──")
    node = make_node()
    node._final_fwd_offset = 0.47       # 与 config/apriltag_place1.yaml 一致
    world = {"tx": 0.10, "tz": 0.80}      # 初始：偏右 0.1m、距离 0.8m
    moves = []
    emitted = []

    orig_send = node._send_move
    def rec_send(x, y, theta):
        moves.append((x, y, theta))
        # 理想世界模型：指令即执行（本策略每条指令只有一个自由度非零）
        if abs(theta) > 1e-9:
            alpha = math.degrees(math.atan2(world["tx"], world["tz"])) + theta
            world["tx"] = world["tz"] * math.tan(math.radians(alpha))
        world["tx"] += y
        world["tz"] -= x
        orig_send(x, y, theta)
    node._send_move = rec_send

    orig_emit = node._emit_place1
    node._emit_place1 = lambda: (emitted.append(True), orig_emit())

    # 检测打桩：直接报世界位姿（fresh=True）
    node._detect_tag = lambda frame: {
        "tx": world["tx"], "ty": 0.0, "tz": world["tz"], "R": None,
        "raw": {"fresh": True},
    }

    trigger(node)
    final = run_fsm(node)

    check('全流程收敛到 done', final == M.STATE_DONE, f"final={final}")
    check('/grasp/start 已发布', len(emitted) == 1)

    thetas = [t for (_, _, t) in moves if abs(t) > 1e-9]
    check('yaw 单步限幅 |theta| <= max_yaw_step_deg',
          len(thetas) > 0 and all(abs(t) <= node._max_yaw_step_deg + 1e-9 for t in thetas),
          f"thetas={[round(t, 2) for t in thetas]}")
    check('yaw 指令方向正确（目标偏右 alpha>0 → theta<0）',
          all(t < 0 for t in thetas))

    xs = [x for (x, _, _) in moves if abs(x) > 1e-9]
    check('approach 闭环前进了 0.80-0.35=0.45m',
          any(abs(x - 0.45) < 1e-6 for x in xs), f"xs={[round(x, 3) for x in xs]}")
    check('盲进距离 = tz_measured-0.35+0.47 = 0.47m',
          abs(moves[-1][0] - 0.47) < 1e-6 and moves[-1][1] == 0.0 and moves[-1][2] == 0.0,
          f"最后一次 move={moves[-1] if moves else None}")
    check('末端站位 tz ≈ 0.35-0.47 = -0.12m',
          abs(world["tz"] - (-0.12)) < 1e-6, f"tz={world['tz']:.3f}m")
    check('yaw 两轮内收敛（7.1° 限幅 3° → 2 轮）',
          node._yaw_rounds == 2, f"rounds={node._yaw_rounds}")

    node.destroy_node()


# ──────────────────────── 3. 对齐阶段真丢失 → wait_detect ─────────────────────── #

def test_loss_fallback():
    print("── 场景 3：yaw_align 阶段目标真丢失回退 ──")
    node = make_node()
    world = {"tx": 0.10, "tz": 0.80}

    node._send_move = lambda x, y, theta: None   # 世界不动，只走状态机

    def detect(frame):
        if node._state == M.STATE_WAIT_DETECT:
            return {"tx": world["tx"], "tz": world["tz"], "raw": {"fresh": True}}
        return None                              # 进入对齐阶段即真丢失
    node._detect_tag = detect

    trigger(node)
    # 单步驱动：wait_detect 重新锁定（15 帧）→ yaw_align 发指令 → 丢失回退
    fell_back = False
    for _ in range(300):
        if step_fsm(node):
            break
        if node._state == M.STATE_WAIT_DETECT and node._yaw_rounds > 0:
            fell_back = True
            break
    check('yaw_align 发出过旋转指令后丢失 → 回 wait_detect',
          fell_back,
          f"state={node._state} yaw_rounds={node._yaw_rounds}")
    check('回退后检测截止时间已刷新',
          node._detect_deadline > time.monotonic())

    node.destroy_node()


# ──────────────────────── 4. final_check 不达标 → yaw_align ──────────────────── #

def test_final_check_reject():
    print("── 场景 4：final_check 横向不达标回 yaw_align ──")
    node = make_node()
    node._send_move = lambda x, y, theta: None
    node._detect_tag = lambda frame: {
        "tx": 0.10, "tz": node._closed_loop_end, "raw": {"fresh": True},
    }
    node._state = M.STATE_FINAL_CHECK
    node._yaw_rounds = 5                # 故意弄脏，验证回退时被清零

    frame = np.zeros((480, 640, 3), np.uint8)
    node._do_final_check(frame)

    check('final_check 横向超标 → 回 yaw_align', node._state == M.STATE_YAW_ALIGN)
    check('回退时各相轮次清零', node._yaw_rounds == 0 and node._lat_rounds == 0
          and node._app_rounds == 0)
    check('回退携带最新位姿', node._last_pose is not None
          and abs(node._last_pose["tx"] - 0.10) < 1e-9)
    check('final_check 稳定缓冲已清空', len(node._stable_buf) == 0)

    node.destroy_node()


# ──────────────────────── 5. 超轮次 → error ─────────────────────────────────── #

def test_max_rounds_error():
    print("── 场景 5：yaw_align 超过 max_rounds ──")
    node = make_node()
    node._max_rounds = 3
    node._send_move = lambda x, y, theta: None   # 世界不动：残差永不收敛
    node._detect_tag = lambda frame: {
        "tx": 0.10, "tz": 0.80, "raw": {"fresh": True},
    }
    node._state = M.STATE_YAW_ALIGN
    node._last_pose = {"tx": 0.10, "tz": 0.80}
    node._yaw_rounds = 0

    final = run_fsm(node)
    check('残差不收敛 3 轮后 → error', final == M.STATE_ERROR,
          f"final={final} rounds={node._yaw_rounds}")
    check('轮次计数正确', node._yaw_rounds == node._max_rounds)

    node.destroy_node()


# ──────────────────────── 6. 盲进距离异常 → error ───────────────────────────── #

def test_blind_sanity():
    print("── 场景 6：盲进距离 <= 0 拒绝执行 ──")
    node = make_node()
    node._final_fwd_offset = 0.01
    node._stable_frames = 3
    node._stable_buf = deque(maxlen=15)
    node._send_move = lambda x, y, theta: None
    emitted = []
    node._emit_place1 = lambda: emitted.append(True)
    # tz=0.325 → dist_ok（|0.325-0.35|=0.025<=0.03），但 blind=0.325-0.35+0.01<0
    node._detect_tag = lambda frame: {
        "tx": 0.0, "tz": 0.325, "raw": {"fresh": True},
    }
    node._state = M.STATE_FINAL_CHECK

    final = run_fsm(node)
    check('盲进距离为负 → error', final == M.STATE_ERROR, f"final={final}")
    check('异常路径不发布抓取信号', len(emitted) == 0)

    node.destroy_node()


def main():
    rclpy.init()
    test_detect_cache()
    test_full_pipeline()
    test_loss_fallback()
    test_final_check_reject()
    test_max_rounds_error()
    test_blind_sanity()
    rclpy.shutdown()

    print()
    if failures:
        print(f"共 {len(failures)} 项失败: {failures}")
        sys.exit(1)
    print("全部通过")


if __name__ == '__main__':
    main()
