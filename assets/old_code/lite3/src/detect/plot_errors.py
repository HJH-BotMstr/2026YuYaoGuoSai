#!/usr/bin/env python3
"""
绘制指针误差角 与 上方向误差角 分布图。
"""

import math, json, sys
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE_DIR = Path(__file__).parent
IMG_DIR = BASE_DIR / ".." / "test_set" / "images"
GT_PATH = BASE_DIR / ".." / "test_set" / "calc_params" / "all.json"
PARAMS_DIR = BASE_DIR / ".." / "test_set" / "params"

ANGLE_RES = 720
RADIUS_RES = 100

sys.path.insert(0, str(BASE_DIR))
from step1_hough_circle import detect_circle
from color_up_detect import extract_roi, enhance_roi, kmeans_segment, get_red_yellow_centers, compute_up_direction


def polar_unwrap(gray_roi):
    h, w = gray_roi.shape
    cx, cy = w / 2.0, h / 2.0
    max_r = min(cx, cy)
    cols = np.arange(ANGLE_RES, dtype=np.float32)
    rad = np.radians(cols * 360.0 / ANGLE_RES)
    rows = np.arange(RADIUS_RES, dtype=np.float32)
    radii = rows * max_r / RADIUS_RES
    map_x = (cx + np.outer(radii, np.cos(rad))).astype(np.float32)
    map_y = (cy + np.outer(radii, np.sin(rad))).astype(np.float32)
    return cv2.remap(gray_roi, map_x, map_y, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def detect_ptr(polar):
    clip = int(RADIUS_RES * 0.20)
    col_means = np.mean(polar[clip:, :], axis=0).astype(np.float32)
    min_col = int(np.argmin(col_means))
    if 1 <= min_col < len(col_means) - 1:
        y0, y1, y2 = col_means[min_col - 1], col_means[min_col], col_means[min_col + 1]
        d = y0 - 2.0 * y1 + y2
        offset = (y0 - y2) / (2.0 * d) if abs(d) > 1e-10 else 0.0
    else:
        offset = 0.0
    return ((min_col + offset) * 360.0 / ANGLE_RES) % 360.0


def main():
    with open(GT_PATH) as f:
        gt_list = json.load(f)
    gt = {item["image"]: item for item in gt_list}
    with open(BASE_DIR / ".." / "test_set" / "test_set_meta.json") as f:
        meta = json.load(f)

    ptr_errors = []
    up_errors = []
    labels = []

    for fname in sorted(gt.keys()):
        stem = Path(fname).stem
        img = cv2.imread(str(IMG_DIR / fname))
        try:
            circle = detect_circle(str(IMG_DIR / fname))
        except RuntimeError:
            continue

        cx, cy, r = circle["center"][0], circle["center"][1], circle["radius"]
        roi = extract_roi(img, cx, cy, r)
        roi_enh = enhance_roi(roi)
        labels_km, centers = kmeans_segment(roi_enh)
        cc = get_red_yellow_centers(labels_km, centers)

        with open(PARAMS_DIR / f"{stem}.json") as pf:
            rotation = json.load(pf)["rotation"]
        gt_up = (270.0 - rotation) % 360.0
        pred_up = compute_up_direction(cc) if ("red" in cc and "yellow" in cc) else 0.0
        up_errors.append(min(abs(pred_up - gt_up) % 360, 360 - abs(pred_up - gt_up) % 360))

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_roi = clahe.apply(gray_roi)
        polar = polar_unwrap(gray_roi)
        pred_ptr = detect_ptr(polar)
        gt_ptr = gt[fname]["pointer_angle"]
        ptr_errors.append(min(abs(pred_ptr - gt_ptr) % 360, 360 - abs(pred_ptr - gt_ptr) % 360))
        labels.append(fname.replace(".png", ""))

    n = len(ptr_errors)
    ptr_pass = sum(1 for e in ptr_errors if e < 10.0)
    up_pass = sum(1 for e in up_errors if e < 10.0)

    plt.rcParams.update({"font.size": 11, "axes.titlesize": 13, "figure.dpi": 150})
    plt.rcParams["axes.unicode_minus"] = False
    from matplotlib.font_manager import fontManager
    fontManager.addfont("/usr/local/share/fonts/truetype/微软雅黑.ttf")
    plt.rcParams["font.family"] = "Microsoft YaHei"

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f"指针与上方向误差分布  (样本数={n}, 指针通过={ptr_pass}/{n}, 上方向通过={up_pass}/{n})",
        fontweight="bold",
    )

    # (0,0) 指针误差条形图
    ax = axes[0, 0]
    colors = ["#2ecc71" if e < 10 else "#e74c3c" for e in ptr_errors]
    ax.bar(range(n), ptr_errors, color=colors, edgecolor="white", linewidth=0.3)
    ax.axhline(y=10, color="gray", linestyle="--", linewidth=1, label="10° 阈值")
    ax.set_xticks(range(n))
    ax.set_xticklabels(labels, rotation=90, fontsize=6)
    ax.set_ylabel("误差 (°)")
    ax.set_title(f"指针角度误差 (均值={np.mean(ptr_errors):.1f}°)")
    ax.legend(fontsize=8)

    # (0,1) 上方向误差条形图
    ax = axes[0, 1]
    colors = ["#9b59b6" if e < 10 else "#e74c3c" for e in up_errors]
    ax.bar(range(n), up_errors, color=colors, edgecolor="white", linewidth=0.3)
    ax.axhline(y=10, color="gray", linestyle="--", linewidth=1, label="10° 阈值")
    ax.set_xticks(range(n))
    ax.set_xticklabels(labels, rotation=90, fontsize=6)
    ax.set_ylabel("误差 (°)")
    ax.set_title(f"上方向误差 (均值={np.mean(up_errors):.1f}°)")
    ax.legend(fontsize=8)

    # (1,0) 误差散点
    ax = axes[1, 0]
    ax.scatter(ptr_errors, up_errors, c="#3498db", s=50, alpha=0.7)
    for i, lab in enumerate(labels):
        ax.annotate(lab.replace("img_", "").replace("_test_", "_"),
                     (ptr_errors[i], up_errors[i]), fontsize=5, alpha=0.7)
    ax.axhline(y=10, color="gray", linestyle="--", linewidth=0.8)
    ax.axvline(x=10, color="gray", linestyle="--", linewidth=0.8)
    ax.set_xlabel("指针误差 (°)")
    ax.set_ylabel("上方向误差 (°)")
    ax.set_title("误差散点图")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)

    # (1,1) 误差直方图
    ax = axes[1, 1]
    ax.hist(ptr_errors, bins=12, color="#2ecc71", alpha=0.6, label=f"指针 (通过={ptr_pass})")
    ax.hist(up_errors, bins=12, color="#9b59b6", alpha=0.6, label=f"上方向 (通过={up_pass})")
    ax.axvline(x=10, color="gray", linestyle="--", linewidth=0.8)
    ax.set_xlabel("误差 (°)")
    ax.set_ylabel("张数")
    ax.set_title("误差直方图 (绿=指针, 紫=上方向)")
    ax.legend(fontsize=8)

    plt.tight_layout()
    out_path = BASE_DIR / ".." / "error_distribution.png"
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"图表: {out_path}")
    print(f"指针通过(10°): {ptr_pass}/{n}  均值={np.mean(ptr_errors):.1f}°")
    print(f"上方向通过(10°): {up_pass}/{n}  均值={np.mean(up_errors):.1f}°")


if __name__ == "__main__":
    main()
