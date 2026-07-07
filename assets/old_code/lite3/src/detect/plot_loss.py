#!/usr/bin/env python3
"""
绘制测试集上霍夫圆检测的损失分布。
"""

import json, math
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE_DIR = Path(__file__).parent
IMG_DIR = BASE_DIR / ".." / "test_set" / "images"
GT_PATH = BASE_DIR / ".." / "test_set" / "calc_params" / "all.json"


def load_ground_truth():
    with open(GT_PATH, "r") as f:
        gt_list = json.load(f)
    return {item["image"]: item for item in gt_list}


def preprocess(gray):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    return cv2.bilateralFilter(enhanced, 9, 75, 75)


def compute_loss(gt_center, gt_radius, pred_center, pred_radius):
    x_gt, y_gt = gt_center
    x_pred, y_pred = pred_center
    r1 = float(gt_radius)
    r2 = float(pred_radius)
    d = math.hypot(x_pred - x_gt, y_pred - y_gt)
    denom = (r1 + r2) / 2.0
    loss = (0.5 * d * d + 0.5 * (r1 - r2) * (r1 - r2)) / denom
    center_err = (d / denom) * 100.0
    radius_err = (abs(r1 - r2) / denom) * 100.0
    return loss, center_err, radius_err


def detect_hough(smooth, w, h):
    edges = cv2.Canny(smooth, 40, 120)
    strategies = [
        (80, 30, 60, True),
        (60, 25, 55, False),
        (100, 35, 60, True),
        (50, 20, 50, False),
        (70, 28, 55, False),
    ]
    all_circles = []
    for p1, p2, min_r, use_edges in strategies:
        src = edges if use_edges else smooth
        circles = cv2.HoughCircles(src, cv2.HOUGH_GRADIENT, dp=1,
                                    minDist=max(w, h), param1=p1, param2=p2,
                                    minRadius=min_r)
        if circles is not None:
            for c in circles[0]:
                all_circles.append((float(c[0]), float(c[1]), float(c[2])))
    radii = [c[2] for c in all_circles]
    best = min(all_circles, key=lambda c: abs(c[2] - float(np.median(radii))))
    return best


def main():
    gt = load_ground_truth()
    losses = []
    center_errs = []
    radius_errs = []
    labels = []

    for fname in sorted(gt.keys()):
        img = cv2.imread(str(IMG_DIR / fname))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        smooth = preprocess(gray)
        circle = detect_hough(smooth, w, h)
        cx, cy, r = circle
        gt_item = gt[fname]
        loss, ce, re = compute_loss(
            gt_item["center"], gt_item["radius_outer"], [cx, cy], r)
        losses.append(loss)
        center_errs.append(ce)
        radius_errs.append(re)
        labels.append(fname.replace(".png", ""))

    n = len(losses)
    mse = sum(l * l for l in losses) / n
    passed = sum(1 for ce, re in zip(center_errs, radius_errs) if ce < 15 and re < 15)
    threshold = 15.0

    plt.rcParams.update({"font.size": 11, "axes.titlesize": 13, "figure.dpi": 150})
    plt.rcParams["axes.unicode_minus"] = False
    from matplotlib.font_manager import fontManager
    fontManager.addfont("/usr/local/share/fonts/truetype/微软雅黑.ttf")
    plt.rcParams["font.family"] = "Microsoft YaHei"

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f"霍夫圆检测损失分布  (样本数={n}, MSE={mse:.4f}, 通过={passed}/{n}={passed/n*100:.1f}%)",
        fontweight="bold",
    )

    # (0,0) 损失条形图
    ax = axes[0, 0]
    colors = ["#2ecc71" if ce < threshold and re < threshold else "#e74c3c"
              for ce, re in zip(center_errs, radius_errs)]
    ax.bar(range(n), losses, color=colors, edgecolor="white", linewidth=0.3)
    ax.axhline(y=np.mean(losses), color="gray", linestyle="--", linewidth=1,
               label=f"均值={np.mean(losses):.4f}")
    ax.set_xticks(range(n))
    ax.set_xticklabels(labels, rotation=90, fontsize=6)
    ax.set_ylabel("损失 L")
    ax.set_title("逐张损失 (绿=通过, 红=失败)")
    ax.legend(fontsize=8)

    # (0,1) 损失直方图
    ax = axes[0, 1]
    ax.hist(losses, bins=15, color="#3498db", edgecolor="white", alpha=0.8)
    ax.axvline(x=np.mean(losses), color="red", linestyle="--",
               label=f"均值={np.mean(losses):.4f}")
    ax.set_xlabel("损失 L")
    ax.set_ylabel("张数")
    ax.set_title("损失直方图")
    ax.legend(fontsize=8)

    # (1,0) 误差散点图
    ax = axes[1, 0]
    passed_mask = np.array([ce < threshold and re < threshold
                            for ce, re in zip(center_errs, radius_errs)])
    ax.scatter(np.array(center_errs)[~passed_mask], np.array(radius_errs)[~passed_mask],
               c="#e74c3c", label="失败", s=40, alpha=0.7)
    ax.scatter(np.array(center_errs)[passed_mask], np.array(radius_errs)[passed_mask],
               c="#2ecc71", label="通过", s=40, alpha=0.7)
    ax.axhline(y=threshold, color="gray", linestyle="--", linewidth=0.8)
    ax.axvline(x=threshold, color="gray", linestyle="--", linewidth=0.8)
    ax.set_xlabel("圆心距相对误差 (%)")
    ax.set_ylabel("半径相对误差 (%)")
    ax.set_title(f"误差散点图 (阈值={threshold:.0f}%)")
    ax.legend(fontsize=8)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)

    # (1,1) 误差排序
    ax = axes[1, 1]
    idx_sorted = np.argsort(losses)
    x_pos = np.arange(n)
    ax.bar(x_pos - 0.2, np.array(center_errs)[idx_sorted], 0.35,
           color="#e67e22", label="圆心误差%", alpha=0.8)
    ax.bar(x_pos + 0.2, np.array(radius_errs)[idx_sorted], 0.35,
           color="#9b59b6", label="半径误差%", alpha=0.8)
    ax.axhline(y=threshold, color="gray", linestyle="--", linewidth=0.8,
               label=f"{threshold:.0f}% 阈值")
    ax.set_xticks(x_pos)
    ax.set_xticklabels([labels[i] for i in idx_sorted], rotation=90, fontsize=6)
    ax.set_ylabel("相对误差 (%)")
    ax.set_title("按损失排序的误差 (橙=圆心, 紫=半径)")
    ax.legend(fontsize=8)

    plt.tight_layout()
    out_path = BASE_DIR / ".." / "loss_distribution.png"
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"图表已保存: {out_path}")
    print(f"样本数: {n}, MSE: {mse:.4f}, 通过: {passed}/{n} ({passed/n*100:.1f}%)")


if __name__ == "__main__":
    main()
