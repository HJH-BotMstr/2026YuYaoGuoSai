#!/usr/bin/env python3
"""
生成测试集上 GT vs 预测对比图。

每张图保存到 debug_output/ 目录：
  *_compare.png  — GT(绿) vs 预测(红) 叠加在原图上
"""

import json, math
from pathlib import Path

import cv2
import numpy as np

BASE_DIR = Path(__file__).parent
IMG_DIR = BASE_DIR / ".." / "test_set" / "images"
GT_PATH = BASE_DIR / ".." / "test_set" / "calc_params" / "all.json"
DEBUG_DIR = BASE_DIR / ".." / "debug_output"

from evaluate import evaluate_circle, load_ground_truth
from step1_hough_circle import detect_circle


def draw_circle(img, cx, cy, r, color, thickness=2, label=""):
    cv2.circle(img, (int(cx), int(cy)), int(r), color, thickness)
    cv2.circle(img, (int(cx), int(cy)), 4, color, -1)
    if label:
        cv2.putText(img, label, (int(cx) + 5, int(cy) - int(r) + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def main():
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    gt = load_ground_truth(BASE_DIR / ".." / "test_set" / "calc_params" / "all.json")

    total = 0
    passed = 0
    for fname in sorted(gt.keys()):
        stem = Path(fname).stem
        gt_item = gt[fname]
        gt_cx, gt_cy = gt_item["center"]
        gt_r = gt_item["radius_outer"]

        img = cv2.imread(str(IMG_DIR / fname))
        if img is None:
            print(f"  SKIP {fname}: 无法读取")
            continue

        total += 1

        # 检测
        try:
            pred = detect_circle(str(IMG_DIR / fname))
            pred_cx, pred_cy = pred["center"]
            pred_r = pred["radius"]
        except RuntimeError:
            pred_cx = pred_cy = pred_r = 0

        # 评估
        ev = evaluate_circle(gt_item["center"], gt_item["radius_outer"],
                             pred["center"] if pred_r > 0 else [0, 0],
                             pred_r if pred_r > 0 else 0)

        vis = img.copy()

        # GT: 绿色实线
        draw_circle(vis, gt_cx, gt_cy, gt_r, (0, 255, 0), 2, "GT")

        # 预测: 红色虚线
        if pred_r > 0:
            color = (0, 0, 255) if ev["passed"] else (0, 165, 255)
            draw_circle(vis, pred_cx, pred_cy, pred_r, color, 2, "PRED")

        # 误差信息
        info_lines = [
            f"GT   : ({gt_cx:.1f}, {gt_cy:.1f}) r={gt_r:.1f}",
            f"PRED : ({pred_cx:.1f}, {pred_cy:.1f}) r={pred_r:.1f}" if pred_r > 0 else "PRED : FAILED",
            f"center_err={ev['center_rel_err']:.1f}%  radius_err={ev['radius_rel_err']:.1f}%",
            f"{'PASS' if ev['passed'] else 'FAIL'}",
        ]
        for i, line in enumerate(info_lines):
            cv2.putText(vis, line, (10, 15 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                        cv2.LINE_AA)

        out_path = DEBUG_DIR / f"{stem}_compare.png"
        cv2.imwrite(str(out_path), vis)

        if ev["passed"]:
            passed += 1
        print(f"  {'PASS' if ev['passed'] else 'FAIL'} {fname} -> {out_path.name}")

    print(f"\n通过: {passed}/{total} ({passed/total*100:.1f}%)")


if __name__ == "__main__":
    main()
