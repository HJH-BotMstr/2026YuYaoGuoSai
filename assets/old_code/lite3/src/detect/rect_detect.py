#!/usr/bin/env python3
"""
灰度 ROI 矩形标识检测 v3 — 旋转矩形 (RotatedRect)。

用法：
  python rect_detect.py <image_path>

输出 debug_output/rect/<stem>_rect.png
"""

import sys
from pathlib import Path

import cv2
import numpy as np

BASE_DIR = Path(__file__).parent
DEBUG_DIR = BASE_DIR / ".." / "debug_output" / "rect"
DEBUG_DIR.mkdir(parents=True, exist_ok=True)

# 参考值（来自 img_001_test_000，以表盘半径 r_roi 为基准）
REF_W_RATIO     = 0.163   # 矩形宽 / r_roi
REF_H_RATIO     = 0.344   # 矩形高 / r_roi
REF_DIST_RATIO  = 0.722   # 矩形中心距圆心 / r_roi
REF_ASPECT      = 0.47    # 宽高比

# 浮动容差
TOLERANCE = 0.50


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


def detect_rect(gray):
    """
    在圆形表盘内搜索旋转矩形标识。
    Returns:
        (rrect, binary_image) 或 (None, binary_image)
    """
    h, w = gray.shape
    cx_roi, cy_roi = w / 2.0, h / 2.0
    r_roi = 500.0 / 2.2

    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, (int(cx_roi), int(cy_roi)), int(r_roi * 0.88), 255, -1)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    # 以亮度 Q1（25%分位数）作为二值化阈值
    q1_val = np.percentile(gray[mask == 255], 25)
    _, adapt = cv2.threshold(gray, q1_val, 255, cv2.THRESH_BINARY_INV)
    adapt = cv2.bitwise_and(adapt, adapt, mask=mask)

    # 形态学：闭运算连接碎片（暂注释）
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    # closed = cv2.morphologyEx(adapt, cv2.MORPH_CLOSE, kernel, iterations=1)
    closed = adapt

    contours, hierarchy = cv2.findContours(closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < 100 or area > 8000:
            continue

        rrect = cv2.minAreaRect(cnt)
        (rcx, rcy), (rw, rh), angle = rrect

        # 尺寸约束（参考值 ±TOLERANCE）
        w_lo = r_roi * REF_W_RATIO * (1.0 - TOLERANCE)
        w_hi = r_roi * REF_W_RATIO * (1.0 + TOLERANCE)
        h_lo = r_roi * REF_H_RATIO * (1.0 - TOLERANCE)
        h_hi = r_roi * REF_H_RATIO * (1.0 + TOLERANCE)
        if rw < w_lo or rw > w_hi:
            continue
        if rh < h_lo or rh > h_hi:
            continue

        # 长宽比（参考值 ±TOLERANCE）
        asp_lo = REF_ASPECT * (1.0 - TOLERANCE)
        asp_hi = REF_ASPECT * (1.0 + TOLERANCE)
        aspect = rw / rh if rh > 0 else 0
        if not (asp_lo < aspect < asp_hi):
            continue

        # 矩形度
        rect_area = rw * rh
        solidity = area / rect_area if rect_area > 0 else 0
        if solidity < 0.3:
            continue

        # 距圆心距离（参考值 ±TOLERANCE）
        dst_lo = r_roi * REF_DIST_RATIO * (1.0 - TOLERANCE)
        dst_hi = r_roi * REF_DIST_RATIO * (1.0 + TOLERANCE)
        dist = np.hypot(rcx - cx_roi, rcy - cy_roi)
        if dist < dst_lo or dist > dst_hi:
            continue

        score = area * solidity
        candidates.append((rrect, score, area))

    if not candidates:
        return None, closed

    candidates.sort(key=lambda t: -t[1])
    return candidates[0][0], closed


def draw_rotated_rect(img, rrect, color, thickness=2):
    """绘制旋转矩形。"""
    box = cv2.boxPoints(rrect)
    box = np.int32(np.round(box))
    cv2.drawContours(img, [box], 0, color, thickness)


def main():
    if len(sys.argv) < 2:
        print("用法: python rect_detect.py <image_path>")
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
    roi = extract_roi(img, cx, cy, r)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    rect, bin_img = detect_rect(gray)
    cv2.imwrite(str(DEBUG_DIR / f"{stem}_roi_bin.png"), bin_img)

    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.circle(vis, (250, 250), int(500 / 2.2 * 0.88), (255, 255, 0), 1)

    if rect:
        (rcx, rcy), (rw, rh), angle = rect
        draw_rotated_rect(vis, rect, (0, 255, 0), 2)
        cv2.circle(vis, (int(rcx), int(rcy)), 3, (0, 255, 0), -1)
        print(f"矩形: center=({rcx:.0f},{rcy:.0f}) {rw:.0f}x{rh:.0f} angle={angle:.1f}°")
    else:
        print("矩形: 未检测到")

    cv2.imwrite(str(DEBUG_DIR / f"{stem}_rect.png"), vis)
    print(f"输出: {DEBUG_DIR / f'{stem}_rect.png'}")


if __name__ == "__main__":
    main()
