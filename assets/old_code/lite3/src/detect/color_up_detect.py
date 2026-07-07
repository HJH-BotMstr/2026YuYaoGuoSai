#!/usr/bin/env python3
"""
颜色区域 → 上方向检测 + 调试图。

K-means 红/黄分割 → 质心 → 推算上方向，在彩色 ROI 上绘制并保存。
"""

import sys, math, json
from pathlib import Path

import cv2
import numpy as np

BASE_DIR = Path(__file__).parent
DEBUG_DIR = BASE_DIR / ".." / "debug_output"
DEBUG_DIR.mkdir(parents=True, exist_ok=True)


def extract_roi(image, cx, cy, r):
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
    return cv2.resize(roi, (500, 500), interpolation=cv2.INTER_LINEAR)


def enhance_roi(roi):
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    l_ch = lab[:, :, 0].astype(np.uint8)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l_ch = clahe.apply(l_ch)
    lab[:, :, 0] = l_ch.astype(np.float32)
    lab[:, :, 1] = (lab[:, :, 1] - 128.0) * 2.0 + 128.0
    lab[:, :, 2] = (lab[:, :, 2] - 128.0) * 2.0 + 128.0
    lab = np.clip(lab, 0, 255)
    return cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_LAB2BGR)


def kmeans_segment(roi_bgr):
    K = 5
    INIT = np.array([
        [0, 0, 255], [0, 255, 0], [0, 255, 255], [255, 255, 255], [0, 0, 0],
    ], dtype=np.float32)
    pixels = roi_bgr.reshape(-1, 3).astype(np.float32)
    dists = np.zeros((pixels.shape[0], K), dtype=np.float32)
    for i in range(K):
        diff = pixels - INIT[i]
        dists[:, i] = np.sum(diff * diff, axis=1)
    init_labels = np.argmin(dists, axis=1).astype(np.int32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 1.0)
    _, labels, centers = cv2.kmeans(pixels, K, init_labels, criteria, 3,
                                     cv2.KMEANS_USE_INITIAL_LABELS)
    return labels.reshape(500, 500), centers.astype(np.uint8)


def get_red_yellow_centers(labels, centers):
    brightness = [float(c[0]) + float(c[1]) + float(c[2]) for c in centers]
    dark_label = int(np.argmin(brightness))

    target_red = np.array([0, 0, 255], dtype=np.float32)
    target_yellow = np.array([0, 255, 255], dtype=np.float32)

    red_label, yellow_label = None, None
    red_dist, yellow_dist = float("inf"), float("inf")

    for i in range(5):
        if i == dark_label:
            continue
        c = centers[i].astype(np.float32)
        dr = np.sum((c - target_red) ** 2)
        dy = np.sum((c - target_yellow) ** 2)
        if dr < red_dist:
            red_dist, red_label = dr, i
        if dy < yellow_dist:
            yellow_dist, yellow_label = dy, i

    result = {}
    for name, label in [("red", red_label), ("yellow", yellow_label)]:
        mask = (labels == label).astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.erode(mask, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                result[name] = (M["m10"] / M["m00"], M["m01"] / M["m00"])
    return result


def compute_up_direction(color_centers):
    rx, ry = color_centers["red"]
    yx, yy = color_centers["yellow"]
    cx, cy = 250.0, 250.0
    red_angle = math.degrees(math.atan2(ry - cy, rx - cx)) % 360
    up_angle = (red_angle + 270.0) % 360
    return up_angle


def draw_debug(roi, color_centers, up_angle, gt_up=None):
    vis = roi.copy()
    cx, cy = 250, 250
    length = 200

    # 红质心
    if "red" in color_centers:
        rx, ry = int(color_centers["red"][0]), int(color_centers["red"][1])
        cv2.circle(vis, (rx, ry), 8, (0, 0, 255), -1)
        cv2.putText(vis, "RED", (rx + 10, ry), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # 黄质心
    if "yellow" in color_centers:
        yx, yy = int(color_centers["yellow"][0]), int(color_centers["yellow"][1])
        cv2.circle(vis, (yx, yy), 8, (0, 255, 255), -1)
        cv2.putText(vis, "YEL", (yx + 10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # 预测上方向（蓝线）
    rad = math.radians(up_angle)
    ux = int(cx + length * math.cos(rad))
    uy = int(cy + length * math.sin(rad))
    cv2.arrowedLine(vis, (cx, cy), (ux, uy), (255, 0, 0), 2, tipLength=0.08)
    cv2.putText(vis, f"UP={up_angle:.0f}", (ux + 5, uy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

    # GT 上方向（绿线）
    if gt_up is not None:
        rad_gt = math.radians(gt_up)
        gx = int(cx + length * 0.6 * math.cos(rad_gt))
        gy = int(cy + length * 0.6 * math.sin(rad_gt))
        cv2.arrowedLine(vis, (cx, cy), (gx, gy), (0, 255, 0), 2, tipLength=0.08)
        cv2.putText(vis, f"GT={gt_up:.0f}", (gx + 5, gy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    return vis


def main():
    from step1_hough_circle import detect_circle

    meta_path = BASE_DIR / ".." / "test_set" / "test_set_meta.json"
    with open(meta_path) as f:
        meta = json.load(f)

    img_dir = BASE_DIR / ".." / "test_set" / "images"
    params_dir = BASE_DIR / ".." / "test_set" / "params"

    for sample in meta["samples"]:
        fname = sample["image"]
        stem = Path(fname).stem
        img = cv2.imread(str(img_dir / fname))

        try:
            circle = detect_circle(str(img_dir / fname))
        except RuntimeError:
            continue

        with open(params_dir / f"{stem}.json") as f:
            params = json.load(f)
        gt_up = (270.0 - params["rotation"]) % 360.0

        cx, cy = circle["center"]
        r = circle["radius"]
        roi = extract_roi(img, cx, cy, r)
        roi_enh = enhance_roi(roi)
        labels, centers = kmeans_segment(roi_enh)
        cc = get_red_yellow_centers(labels, centers)

        up = compute_up_direction(cc) if "red" in cc and "yellow" in cc else 0.0
        err = min(abs(up - gt_up) % 360, 360 - abs(up - gt_up) % 360)

        vis = draw_debug(roi, cc, up, gt_up)
        cv2.imwrite(str(DEBUG_DIR / f"{stem}_up.png"), vis)
        print(f"  {fname}: up={up:.1f}° gt={gt_up:.1f}° err={err:.1f}°")


if __name__ == "__main__":
    main()
