#!/usr/bin/env python3
"""
仪表盘识别 — 端到端流程。

输入一张图片，输出：
  1. 终端中文识别结果
  2. 标注图（霍夫圆/圆心/黄红中心/上方向/指针方向）

运行: python gauge_recognition.py
"""

import math
from pathlib import Path

import cv2
import numpy as np

BASE_DIR = Path(__file__).parent
IMG_DIR = BASE_DIR / ".." / "test_gen" / "images"
OUT_DIR = BASE_DIR / ".." / "output"
OUT_DIR.mkdir(parents=True, exist_ok=True)

ANGLE_RES = 720
RADIUS_RES = 100

# ──────────────── 颜色分区（本地坐标系，0°=右）────────────────
# 指针在哪个区间就输出对应状态
ZONE_STATES = [
    # (角度范围 deg, 状态文字, 标签)
    # 红/黄不与绿交接的边界向外扩张15°
    ((315, 360), "仪表盘偏高，状态异常", "RED"),
    ((0, 60),    "仪表盘偏高，状态异常", "RED"),
    ((225, 315), "仪表盘正常，状态良好", "GREEN"),
    ((120, 225), "仪表盘偏低，状态异常", "YELLOW"),
]


def classify_pointer(pointer_angle, up_angle):
    """
    根据指针角度和上方向，判断指针在哪个色区。

    pointer_angle: 画布坐标中的指针角度 (atan2 约定)
    up_angle:      画布坐标中的正上方向

    表盘本地坐标系：0°=右，上=270°(绿), 左=180°(黄), 右=0°(红)
    指针相对于表盘的角度 = (pointer_angle - up_angle + 270) % 360
    """
    rel_angle = (pointer_angle - up_angle + 270.0) % 360.0

    for (lo, hi), text, tag in ZONE_STATES:
        if lo <= rel_angle < hi:
            return text, tag
    return "仪表盘状态未知", "UNKNOWN"


# ──────────────── Step 1: 霍夫圆检测 ────────────────────────

def detect_circle(image):
    """检测仪表盘外轮廓圆（简化版，稳定参数）。"""
    h, w = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    smooth = cv2.bilateralFilter(enhanced, 9, 75, 75)
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


# ──────────────── 预处理 ────────────────────────────────────

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


# ──────────────── Step 2: 颜色区域 → 上方向 ────────────────

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


def get_color_centroids(labels, centers):
    """返回 {red: (cx,cy), yellow: (cx,cy)}。"""
    brightness = [float(c[0]) + float(c[1]) + float(c[2]) for c in centers]
    dark_label = int(np.argmin(brightness))

    t_red = np.array([0, 0, 255], dtype=np.float32)
    t_yellow = np.array([0, 255, 255], dtype=np.float32)
    red_label, yellow_label = None, None
    red_dist, yellow_dist = float("inf"), float("inf")

    for i in range(5):
        if i == dark_label:
            continue
        c = centers[i].astype(np.float32)
        dr = np.sum((c - t_red) ** 2)
        dy = np.sum((c - t_yellow) ** 2)
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


def compute_up(red_pt, yellow_pt):
    """红黄质心 → 上方向 (ROI 坐标，atan2 约定)。"""
    rx, ry = red_pt
    cx, cy = 250.0, 250.0
    red_angle = math.degrees(math.atan2(ry - cy, rx - cx)) % 360
    return (red_angle + 270.0) % 360.0


# ──────────────── Step 3: 指针角度 ──────────────────────────

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


def detect_pointer_angle(polar):
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


# ──────────────── 标注绘制 ──────────────────────────────────

def draw_annotations(image, cx, cy, r, red_pt, yellow_pt, up_angle, ptr_angle, status_text):
    """在原始图上绘制所有标注。"""
    vis = image.copy()
    h, w = vis.shape[:2]

    # 霍夫圆 + 圆心
    cv2.circle(vis, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
    cv2.circle(vis, (int(cx), int(cy)), 6, (0, 255, 255), -1)

    # 黄/红中心（映射回原图坐标）
    roi_side = int(r * 2.2)
    scale = 500.0 / roi_side
    offset_x = int(cx) - roi_side // 2
    offset_y = int(cy) - roi_side // 2

    def roi_to_img(px, py):
        return (int(offset_x + px / scale), int(offset_y + py / scale))

    if red_pt:
        rx, ry = roi_to_img(red_pt[0], red_pt[1])
        cv2.circle(vis, (rx, ry), 10, (0, 0, 255), -1)
        cv2.putText(vis, "RED", (rx + 12, ry + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    if yellow_pt:
        yx, yy = roi_to_img(yellow_pt[0], yellow_pt[1])
        cv2.circle(vis, (yx, yy), 10, (0, 255, 255), -1)
        cv2.putText(vis, "YEL", (yx + 12, yy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # 上方向（蓝箭头）
    up_rad = math.radians(up_angle)
    up_len = r * 0.7
    ux = int(cx + up_len * math.cos(up_rad))
    uy = int(cy + up_len * math.sin(up_rad))
    cv2.arrowedLine(vis, (int(cx), int(cy)), (ux, uy), (255, 0, 0), 2, tipLength=0.1)
    cv2.putText(vis, "UP", (ux + 5, uy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # 指针方向（红箭头）
    ptr_rad = math.radians(ptr_angle)
    ptr_len = r * 0.85
    px = int(cx + ptr_len * math.cos(ptr_rad))
    py = int(cy + ptr_len * math.sin(ptr_rad))
    cv2.arrowedLine(vis, (int(cx), int(cy)), (px, py), (0, 0, 255), 3, tipLength=0.1)
    cv2.putText(vis, f"PTR={ptr_angle:.1f}", (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # 状态文字
    cv2.putText(vis, status_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    return vis


# ──────────────── 主流程 ────────────────────────────────────

def process_image(img_path):
    """处理单张图片，返回 (标注图, 状态文字, 标签)。"""
    img = cv2.imread(str(img_path))
    name = img_path.stem

    # Step 1: 霍夫圆
    cx, cy, r = detect_circle(img)
    print(f"  [{name}] 圆: ({cx:.1f}, {cy:.1f}) r={r:.1f}")

    # Step 2: 颜色区域 → 上方向
    roi = extract_roi(img, cx, cy, r)
    roi_enh = enhance_roi(roi)
    labels, centers = kmeans_segment(roi_enh)
    cc = get_color_centroids(labels, centers)

    red_pt = cc.get("red")
    yellow_pt = cc.get("yellow")
    
    if red_pt and yellow_pt:
        up_angle = compute_up(red_pt, yellow_pt)
    else:
        up_angle = 0.0

    # Step 3: 指针角度
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_roi = clahe.apply(gray_roi)
    polar = polar_unwrap(gray_roi)
    ptr_angle = detect_pointer_angle(polar)

    # 分类
    status_text, tag = classify_pointer(ptr_angle, up_angle)
    print(f"  [{name}] 指针: {ptr_angle:.1f}°  上方向: {up_angle:.1f}°  →  {status_text}")

    # 标注
    vis = draw_annotations(img, cx, cy, r, red_pt, yellow_pt, up_angle, ptr_angle, status_text)

    return vis, status_text, tag


def main():
    img_files = sorted(IMG_DIR.glob("img_*.jpeg"))
    if not img_files:
        print(f"未找到图片: {IMG_DIR}")
        return

    print("仪表盘识别结果:\n")
    for img_path in img_files:
        vis, status, tag = process_image(img_path)
        out_path = OUT_DIR / f"result_{img_path.name}"
        cv2.imwrite(str(out_path), vis)
        print(f"  输出图: {out_path}\n")

    print("完成。")


if __name__ == "__main__":
    main()
