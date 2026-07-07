#!/usr/bin/env python3
"""
彩色 ROI 的 K-means 颜色聚类 — 红绿黄白黑五色区域划分。

对 LAB 增强后的彩色 ROI，以红/绿/黄/白/黑为初始中心做 K-means(k=5)。

用法：
  python color_kmeans.py <image_path>

输出 debug_output/<stem>_km_labels.png  区域标签图
      debug_output/<stem>_km_result.png  聚类重建图
      debug_output/<stem>_roi_color.png  原始彩色ROI
      debug_output/<stem>_roi_enhanced.png LAB增强ROI
"""

import sys
from pathlib import Path

import cv2
import numpy as np

BASE_DIR = Path(__file__).parent
DEBUG_DIR = BASE_DIR / ".." / "debug_output"
DEBUG_DIR.mkdir(parents=True, exist_ok=True)

# 初始聚类中心 (BGR): 红 绿 黄 白 黑
INIT_CENTERS = np.array([
    [0,   0,   255],   # 红
    [0,   255, 0],     # 绿
    [0,   255, 255],   # 黄
    [255, 255, 255],   # 白
    [0,   0,   0],     # 黑
], dtype=np.float32)

K = 5
MAX_ITER = 3
EPS = 1.0


def extract_roi(image, cx, cy, r):
    """裁剪 500×500 ROI。"""
    h, w = image.shape[:2]
    side = int(r * 2.2)
    half = side // 2
    x1, y1 = int(cx) - half, int(cy) - half

    pad_left = max(0, -x1)
    pad_top = max(0, -y1)
    pad_right = max(0, x1 + side - w)
    pad_bottom = max(0, y1 + side - h)

    x1c, y1c = max(0, x1), max(0, y1)
    x2c, y2c = min(w, x1 + side), min(h, y1 + side)
    roi = image[y1c:y2c, x1c:x2c]

    if pad_left or pad_top or pad_right or pad_bottom:
        roi = cv2.copyMakeBorder(roi, pad_top, pad_bottom, pad_left, pad_right,
                                  cv2.BORDER_REPLICATE)
    if roi.shape[0] != side or roi.shape[1] != side:
        roi = cv2.resize(roi, (side, side), interpolation=cv2.INTER_LINEAR)
    roi = cv2.resize(roi, (500, 500), interpolation=cv2.INTER_LINEAR)
    return roi


def kmeans_with_init(roi_bgr):
    """K-means 聚类，以红绿黄白黑为初始中心。"""
    h, w = roi_bgr.shape[:2]
    pixels = roi_bgr.reshape(-1, 3).astype(np.float32)

    dists = np.zeros((pixels.shape[0], K), dtype=np.float32)
    for i in range(K):
        diff = pixels - INIT_CENTERS[i]
        dists[:, i] = np.sum(diff * diff, axis=1)
    init_labels = np.argmin(dists, axis=1).astype(np.int32)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, MAX_ITER, EPS)
    compactness, labels, centers = cv2.kmeans(
        pixels, K, init_labels, criteria, 3, cv2.KMEANS_USE_INITIAL_LABELS,
    )
    labels_2d = labels.reshape(h, w)
    centers = centers.astype(np.uint8)
    return labels_2d, centers


def main():
    if len(sys.argv) < 2:
        print("用法: python color_kmeans.py <image_path>")
        sys.exit(1)

    img_path = Path(sys.argv[1])
    stem = img_path.stem

    img = cv2.imread(str(img_path))
    if img is None:
        print(f"无法读取: {img_path}")
        sys.exit(1)

    from step1_hough_circle import detect_circle
    try:
        circle = detect_circle(str(img_path))
    except RuntimeError as e:
        print(f"圆检测失败: {e}")
        sys.exit(1)

    cx, cy = circle["center"]
    r = circle["radius"]
    print(f"圆: center=({cx:.1f}, {cy:.1f}) r={r:.1f}")

    # ROI
    roi = extract_roi(img, cx, cy, r)
    cv2.imwrite(str(DEBUG_DIR / f"{stem}_roi_color.png"), roi)

    # LAB 增强
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    l_channel = lab[:, :, 0].astype(np.uint8)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l_channel = clahe.apply(l_channel)
    lab[:, :, 0] = l_channel.astype(np.float32)
    lab[:, :, 1] = (lab[:, :, 1] - 128.0) * 2.0 + 128.0
    lab[:, :, 2] = (lab[:, :, 2] - 128.0) * 2.0 + 128.0
    lab = np.clip(lab, 0, 255)
    roi_enhanced = cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_LAB2BGR)
    cv2.imwrite(str(DEBUG_DIR / f"{stem}_roi_enhanced.png"), roi_enhanced)

    # K-means
    labels, centers = kmeans_with_init(roi_enhanced)
    color_names = ["红", "绿", "黄", "白", "黑"]
    print("聚类中心 (BGR):")
    for i, c in enumerate(centers):
        print(f"  标签{i} ({color_names[i]}初始): BGR=({c[0]}, {c[1]}, {c[2]})")

    # 标签图
    palette = [
        [0, 0, 200],     # 红
        [0, 200, 0],     # 绿
        [0, 200, 200],   # 黄
        [200, 200, 200], # 白
        [50, 50, 50],    # 黑
    ]
    label_viz = np.zeros((500, 500, 3), dtype=np.uint8)
    for i in range(K):
        label_viz[labels == i] = palette[i]
    cv2.imwrite(str(DEBUG_DIR / f"{stem}_km_labels.png"), label_viz)

    # 重建图
    reconstructed = np.zeros_like(roi)
    for i in range(K):
        reconstructed[labels == i] = centers[i]
    cv2.imwrite(str(DEBUG_DIR / f"{stem}_km_result.png"), reconstructed)

    print(f"\n输出: {DEBUG_DIR}/")
    print(f"  {stem}_roi_color.png")
    print(f"  {stem}_roi_enhanced.png")
    print(f"  {stem}_km_labels.png")
    print(f"  {stem}_km_result.png")


if __name__ == "__main__":
    main()
