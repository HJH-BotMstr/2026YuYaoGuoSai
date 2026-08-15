#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""letter_place_align_node 检测层离线自测（无摄像头、无真机）。

验证点：
  1. 合成白纸字母图经 _detect_letter 得到同构位姿 dict，tz 反推误差 < 5%；
  2. OCR 异步线程能认出字母并锁定目标；
  3. 目标消失后在 lost_tolerance_s 内沿用缓存位姿（fresh=False），超期判真丢失；
  4. 视野内只有别的字母时不匹配（不误锁）。
"""

import sys
import time

import numpy as np
import cv2

sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/letter_place_align')

import rclpy  # noqa: E402
from letter_place_align.letter_place_align_node import LetterPlaceAlignNode  # noqa: E402

FY = 387.7497
H_PAPER = 0.297
PAPER_H_PX = 127          # 合成图纸高 → 期望 tz ≈ 0.907m
EXPECTED_TZ = FY * H_PAPER / PAPER_H_PX

failures = []


def check(name, cond, detail=''):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}  {detail}")
    if not cond:
        failures.append(name)


def make_frame(letter='B', cx=320, cy=240, paper_w=90, paper_h=PAPER_H_PX):
    frame = np.full((480, 640, 3), (30, 70, 140), dtype=np.uint8)
    if letter is not None:
        x1, y1 = cx - paper_w // 2, cy - paper_h // 2
        cv2.rectangle(frame, (x1, y1), (x1 + paper_w, y1 + paper_h), (255, 255, 255), -1)
        cv2.putText(frame, letter, (cx - 28, cy + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 0, 0), 8)
    return frame


def main():
    rclpy.init()
    node = LetterPlaceAlignNode()   # 摄像头打不开只报错，不影响离线检测
    node._show_debug = False        # 无显示环境
    node._target_letter = 'B'

    # 1. OCR 异步锁定 + 位姿反推（给 OCR 线程最多 5s）
    frame = make_frame('B')
    pose = None
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        pose = node._detect_letter(frame)
        if pose is not None and pose["raw"].get("fresh"):
            break
        time.sleep(0.05)
    check('目标 B 被锁定（OCR 异步成功）',
          pose is not None and pose["raw"].get("fresh"),
          f"char={pose['raw']['char'] if pose else None}")
    if pose is not None:
        err = abs(pose["tz"] - EXPECTED_TZ) / EXPECTED_TZ
        check('tz 反推误差 < 5%', err < 0.05,
              f"tz={pose['tz']:.3f}m 期望≈{EXPECTED_TZ:.3f}m")
        check('tx ≈ 0（纸居中）', abs(pose["tx"]) < 0.05, f"tx={pose['tx']:.3f}m")

    # 2. 目标消失 → lost_tolerance 内沿用缓存
    empty = make_frame(None)
    pose2 = node._detect_letter(empty)
    check('丢失后沿用缓存位姿（fresh=False）',
          pose2 is not None and not pose2["raw"].get("fresh", True))
    # 伪造超期：把 last_valid_time 拨回 3s 前
    node._last_valid_time = time.monotonic() - 3.0
    pose3 = node._detect_letter(empty)
    check('超过 lost_tolerance 判真丢失（返回 None）', pose3 is None)

    # 3. 只有别的字母（D）时不误锁
    node._last_valid_pose = None
    node._h_px_buf.clear()
    node._target_letter = 'B'
    with node._ocr_lock:
        node._ocr_entries.clear()
    frame_d = make_frame('D')
    locked = None
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        p = node._detect_letter(frame_d)
        if p is not None and p["raw"].get("fresh"):
            locked = p
            break
        time.sleep(0.05)
    check('字母 D 不会被误认为目标 B', locked is None)
    check('防呆计数生效（wrong_letter_count > 0）', node._wrong_letter_count > 0,
          f"count={node._wrong_letter_count}")

    node.destroy_node()
    rclpy.shutdown()

    print()
    if failures:
        print(f"共 {len(failures)} 项失败: {failures}")
        sys.exit(1)
    print("全部通过")


if __name__ == '__main__':
    main()
