#!/usr/bin/env python3
"""
生成极坐标展开图 + 列均值柱状图的拼接可视化。

输出到 debug_output/ 目录：
  *_polar_viz.png  — 上方极坐标展开图，下方列亮度柱状图
"""

import sys, math
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from step1_hough_circle import detect_circle
from step2_pointer_detect import extract_roi, polar_unwrap
from evaluate import load_ground_truth

BASE_DIR = Path(__file__).parent
IMG_DIR = BASE_DIR / ".." / "test_set" / "images"
DEBUG_DIR = BASE_DIR / ".." / "debug_output"
DEBUG_DIR.mkdir(parents=True, exist_ok=True)


def main():
    gt = load_ground_truth(BASE_DIR / ".." / "test_set" / "calc_params" / "all.json")

    for fname in sorted(gt.keys()):
        stem = Path(fname).stem
        img = cv2.imread(str(IMG_DIR / fname))
        if img is None:
            continue

        try:
            circle = detect_circle(str(IMG_DIR / fname))
        except RuntimeError:
            continue

        gt_angle = gt[fname]["pointer_angle"]

        # ROI + 极坐标展开
        roi = extract_roi(img, circle["center"][0], circle["center"][1], circle["radius"])
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_roi = clahe.apply(gray_roi)
        polar = polar_unwrap(gray_roi)

        # 列均值（跳过内圈 20%，不平滑）
        clip_start = int(100 * 0.20)
        polar_clip = polar[clip_start:, :]
        col_means = np.mean(polar_clip, axis=0).astype(np.float32)

        # 预测角度
        min_col = int(np.argmin(col_means))
        pred_angle = min_col * 360.0 / 720.0

        # ── 绘图 ────────────────────────────────────────────────
        fig, (ax_top, ax_bot) = plt.subplots(
            2, 1, figsize=(14, 5),
            gridspec_kw={"height_ratios": [2, 1]},
        )

        # 上图：极坐标展开
        ax_top.imshow(polar, cmap="gray", aspect="auto",
                       extent=[0, 360, 100, 0])
        ax_top.axvline(x=gt_angle, color="lime", linewidth=1.5, linestyle="--",
                        label=f"GT={gt_angle:.1f}°")
        ax_top.axvline(x=pred_angle, color="red", linewidth=1.5,
                        label=f"pred={pred_angle:.1f}°")
        ax_top.axhline(y=clip_start, color="cyan", linewidth=0.8, linestyle=":")
        ax_top.set_ylabel("radius (row)")
        ax_top.set_title(f"{stem}  —  Polar Unwrap")
        ax_top.legend(fontsize=7, loc="upper right")

        # 下图：列均值柱状图
        x_deg = np.linspace(0, 360, 720)
        colors = ["#e74c3c" if i == min_col else "#3498db" for i in range(720)]
        ax_bot.bar(x_deg, col_means, width=0.5, color=colors, edgecolor="none")
        ax_bot.axvline(x=gt_angle, color="lime", linewidth=1.5, linestyle="--")
        ax_bot.axvline(x=pred_angle, color="red", linewidth=1.5)
        ax_bot.set_xlabel("angle (°)")
        ax_bot.set_ylabel("mean brightness")
        ax_bot.set_title(f"Column Mean Brightness  (min at {pred_angle:.1f}°, GT={gt_angle:.1f}°)")
        ax_bot.set_xlim(0, 360)

        plt.tight_layout()
        out_path = DEBUG_DIR / f"{stem}_polar_viz.png"
        plt.savefig(out_path, dpi=120, bbox_inches="tight")
        plt.close()

        err = min(abs(pred_angle - gt_angle) % 360, 360 - abs(pred_angle - gt_angle) % 360)
        print(f"  {fname}: pred={pred_angle:.1f}° gt={gt_angle:.1f}° err={err:.1f}° -> {out_path.name}")

    print(f"\n完成，输出到 {DEBUG_DIR}/")


if __name__ == "__main__":
    main()
