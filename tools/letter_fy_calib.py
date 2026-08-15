#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""letter_place_align 摄像头焦距实测标定工具（一次性使用，不入包）。

背景：config/letter_place_align.yaml 的 camera_matrix 借用机械臂内参，
与实际使用的 RMONCAM USB 摄像头（/dev/video6）不符，导致
tz = fy * H / h_px 距离整体按比例偏差。本工具用"已知距离反解 fy"：

    fy_real = distance_true * h_px_median / paper_height_m

用法（在运动主机上执行，无需图形界面）：
    # 1) 把 A4 纸竖直贴在纸箱上，正对摄像头，卷尺量纸面到镜头的距离（如 0.5m）
    python3 tools/letter_fy_calib.py --distance 0.5
    # 2) 换个距离（如 0.8m）再测一次，两次 fy 取平均
    python3 tools/letter_fy_calib.py --distance 0.8
    # 3) 把打印出的 camera_matrix 两行写进 config/letter_place_align.yaml

建议测 2~3 个距离（0.5/0.8/1.0m）取平均；各次 fy 相差 >5% 说明
检测不稳定或距离量错，需重测。
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gauge_yolo_new import detect_letter_papers  # noqa: E402

_PAPER_H_M = {"portrait": 0.297, "landscape": 0.210}


def main():
    parser = argparse.ArgumentParser(description="已知距离反解摄像头 fy 的标定工具")
    parser.add_argument("--camera", type=str, default="/dev/video6",
                        help="摄像头设备（默认 /dev/video6 RMONCAM）")
    parser.add_argument("--distance", type=float, required=True,
                        help="纸面到镜头的实测距离（米），卷尺量取")
    parser.add_argument("--orientation", choices=["portrait", "landscape"],
                        default="portrait", help="A4 纸张贴方向（默认竖贴）")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="采样时长（秒，默认 5）")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    paper_h = _PAPER_H_M[args.orientation]

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"错误: 摄像头打开失败 {args.camera}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print(f"标定采样 {args.seconds:.0f}s：A4 {args.orientation}（高边 {paper_h}m），"
          f"实测距离 {args.distance}m。请保持纸与相机静止…")

    h_list = []
    frames = 0
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        frames += 1
        cands = detect_letter_papers(
            frame,
            orientation=args.orientation,
            center_v=None,          # 标定时纸不一定在画面中心，不做齐平过滤
            ocr=False,
        )
        if not cands:
            continue
        best = max(cands, key=lambda c: c["h_px"])
        h_list.append(best["h_px"])
    cap.release()

    if not h_list:
        print(f"错误: {frames} 帧中未检测到任何 A4 轮廓。"
              "请检查光照/贴纸方向/--orientation 参数后重试")
        sys.exit(2)

    h_med = float(np.median(h_list))
    fy = args.distance * h_med / paper_h
    print(f"\n采样 {frames} 帧，检出 {len(h_list)} 帧"
          f"（检出率 {len(h_list) / max(frames, 1) * 100:.0f}%）")
    print(f"h_px 中位数 = {h_med:.1f} px")
    print(f"反解 fy = {fy:.1f}")
    print("\n建议写入 config/letter_place_align.yaml 的内参"
          "（fx 取同值，cx/cy 取画面中心；多距离测量取 fy 平均）：")
    print(f"  camera_matrix: [{fy:.1f}, 0.0, {args.width / 2:.1f}, "
          f"0.0, {fy:.1f}, {args.height / 2:.1f}, 0.0, 0.0, 1.0]")


if __name__ == "__main__":
    main()
