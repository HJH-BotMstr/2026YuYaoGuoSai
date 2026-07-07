#!/usr/bin/env python3
"""
Step 2: 指针角度检测

流程：
  1. 根据霍夫圆结果裁剪 ROI → resize 500×500
  2. 极坐标展开：角度→x(720px)，半径→y(100px)
  3. 列均值 → 最小值 → 指针角度
"""

import math
import cv2
import numpy as np
from pathlib import Path

BASE_DIR = Path(__file__).parent

ANGLE_RES = 720
RADIUS_RES = 100


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


def polar_unwrap(gray_roi):
    h, w = gray_roi.shape
    cx, cy = w / 2.0, h / 2.0
    max_r = min(cx, cy)
    cols = np.arange(ANGLE_RES, dtype=np.float32)
    angles_rad = np.radians(cols * 360.0 / ANGLE_RES)
    cos_a, sin_a = np.cos(angles_rad), np.sin(angles_rad)
    rows = np.arange(RADIUS_RES, dtype=np.float32)
    radii = rows * max_r / RADIUS_RES
    map_x = (cx + np.outer(radii, cos_a)).astype(np.float32)
    map_y = (cy + np.outer(radii, sin_a)).astype(np.float32)
    return cv2.remap(gray_roi, map_x, map_y, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def detect_pointer_angle(polar):
    clip_start = int(RADIUS_RES * 0.20)
    polar_clip = polar[clip_start:, :]
    col_means = np.mean(polar_clip, axis=0).astype(np.float32)

    min_col = int(np.argmin(col_means))

    if 1 <= min_col < len(col_means) - 1:
        y0, y1, y2 = col_means[min_col - 1], col_means[min_col], col_means[min_col + 1]
        d = y0 - 2.0 * y1 + y2
        offset = (y0 - y2) / (2.0 * d) if abs(d) > 1e-10 else 0.0
    else:
        offset = 0.0

    angle = ((min_col + offset) * 360.0 / ANGLE_RES) % 360.0
    return round(angle, 2)


def detect_pointer(image, circle):
    cx, cy = circle["center"]
    r = circle["radius"]
    roi = extract_roi(image, cx, cy, r)
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_roi = clahe.apply(gray_roi)
    polar = polar_unwrap(gray_roi)
    angle = detect_pointer_angle(polar)
    return {"pointer_angle": angle}
