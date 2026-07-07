#!/usr/bin/env python3
"""
摄像头实时仪表盘识别。

按 q 退出。
"""

import math
import cv2
import numpy as np

ANGLE_RES = 720
RADIUS_RES = 100

ZONE_STATES = [
    ((315, 360), "仪表盘偏高，状态异常", "RED"),
    ((0, 60),    "仪表盘偏高，状态异常", "RED"),
    ((225, 315), "仪表盘正常，状态良好", "GREEN"),
    ((120, 225), "仪表盘偏低，状态异常", "YELLOW"),
]


def detect_circle(gray):
    h, w = gray.shape
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    smooth = cv2.bilateralFilter(clahe.apply(gray), 9, 75, 75)
    edges = cv2.Canny(smooth, 40, 120)

    strategies = [
        (80, 30, 60, True), (60, 25, 55, False), (100, 35, 60, True),
        (50, 20, 50, False), (70, 28, 55, False),
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
    if not all_circles:
        return None
    radii = [c[2] for c in all_circles]
    return min(all_circles, key=lambda c: abs(c[2] - float(np.median(radii))))


def extract_roi(image, cx, cy, r):
    h, w = image.shape[:2]
    side = int(r * 2.2)
    half = side // 2
    x1, y1 = int(cx) - half, int(cy) - half
    pad_left = max(0, -x1); pad_top = max(0, -y1)
    pad_right = max(0, x1 + side - w); pad_bottom = max(0, y1 + side - h)
    x1c, y1c = max(0, x1), max(0, y1)
    x2c, y2c = min(w, x1 + side), min(h, y1 + side)
    roi = image[y1c:y2c, x1c:x2c]
    if pad_left or pad_top or pad_right or pad_bottom:
        roi = cv2.copyMakeBorder(roi, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_REPLICATE)
    if roi.shape[0] != side or roi.shape[1] != side:
        roi = cv2.resize(roi, (side, side), interpolation=cv2.INTER_LINEAR)
    return cv2.resize(roi, (500, 500), interpolation=cv2.INTER_LINEAR)


def enhance_roi(roi):
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    l_ch = lab[:, :, 0].astype(np.uint8)
    l_ch = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(l_ch)
    lab[:, :, 0] = l_ch.astype(np.float32)
    lab[:, :, 1] = (lab[:, :, 1] - 128.0) * 2.0 + 128.0
    lab[:, :, 2] = (lab[:, :, 2] - 128.0) * 2.0 + 128.0
    lab = np.clip(lab, 0, 255)
    return cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_LAB2BGR)


# ── K-means 分割（已废弃，改用 LAB 阈值）───────────────
# def kmeans_segment(roi_bgr): ...
# def get_red_yellow_centers(labels, centers): ...


def lab_threshold_centers(roi):
    """LAB 色彩空间阈值分割，返回红/黄区域质心"""
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

    # LAB 阈值 (OpenCV: L[0-255], a[0-255], b[0-255])
    # 红: L[0,100] a[7,83] b[-55,71] → L[0,255] a[135,211] b[73,199]
    # 黄: L[0,100] a[-57,47] b[17,97] → L[0,255] a[71,175] b[145,225]
    red_mask = cv2.inRange(lab, np.array([0, 135, 73]), np.array([255, 211, 199]))
    yellow_mask = cv2.inRange(lab, np.array([0, 71, 145]), np.array([255, 175, 225]))

    result = {}
    for name, mask in [("red", red_mask), ("yellow", yellow_mask)]:
        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=1)
        # 有效区域仅限圆环带（r=100~220）
        cv2.circle(mask, (250, 250), 100, 0, -1)
        outer = np.zeros((500, 500), dtype=np.uint8)
        cv2.circle(outer, (250, 250), 220, 255, -1)
        mask = cv2.bitwise_and(mask, outer)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                result[name] = (M["m10"] / M["m00"], M["m01"] / M["m00"])
    return result


def compute_up(red_pt):
    rx, ry = red_pt
    return (math.degrees(math.atan2(ry - 250.0, rx - 250.0)) + 270.0) % 360.0


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
    return cv2.remap(gray_roi, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


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


def classify(ptr_angle, up_angle):
    rel = (ptr_angle - up_angle + 270.0) % 360.0
    for (lo, hi), text, tag in ZONE_STATES:
        if lo <= rel < hi:
            return text, tag
    return "状态未知", "UNKNOWN"


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("摄像头实时识别中... 按 q 退出\n")

    history = []  # 最近N帧
    STABLE_N = 3
    ANGLE_TOL = 60.0
    last_state = None
    unstable = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circle = detect_circle(gray)

        if circle:
            cx, cy, r = circle
            roi = extract_roi(frame, cx, cy, r)
            roi_enh = enhance_roi(roi)
            cc = lab_threshold_centers(roi_enh)

            up_angle = compute_up(cc["red"])
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray_roi = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray_roi)
            ptr_angle = detect_ptr(polar_unwrap(gray_roi))

            history.append((ptr_angle, up_angle, cx, cy, r, cc))
            if len(history) > STABLE_N:
                history.pop(0)

            stable = len(history) >= STABLE_N
            if stable:
                ptrs = [h[0] for h in history]
                ups = [h[1] for h in history]
                cxs = [h[2] for h in history]
                cys = [h[3] for h in history]
                def angle_spread(angles):
                    shifted = [(a - angles[0]) % 360 for a in angles]
                    return max(shifted) - min(shifted)
                if (angle_spread(ptrs) > ANGLE_TOL or
                    angle_spread(ups) > ANGLE_TOL or
                    math.hypot(max(cxs) - min(cxs), max(cys) - min(cys)) > r * 0.3):
                    stable = False

            if stable:
                last_state = (cx, cy, r, cc, up_angle, ptr_angle)
                unstable = 0
            else:
                unstable += 1
        else:
            unstable += 1

        if unstable >= 5:
            last_state = None

        if last_state:
            cx, cy, r, cc, up_angle, ptr_angle = last_state
            cv2.circle(frame, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 255), -1)

            rx, ry = cc["red"]
            rx_f = int(cx + (rx - 250) / 500.0 * r * 2.2)
            ry_f = int(cy + (ry - 250) / 500.0 * r * 2.2)
            cv2.circle(frame, (rx_f, ry_f), 8, (0, 0, 255), -1)
            yx, yy = cc["yellow"]
            yx_f = int(cx + (yx - 250) / 500.0 * r * 2.2)
            yy_f = int(cy + (yy - 250) / 500.0 * r * 2.2)
            cv2.circle(frame, (yx_f, yy_f), 8, (0, 255, 255), -1)

            up_rad = math.radians(up_angle)
            up_len = r * 0.7
            cv2.arrowedLine(frame, (int(cx), int(cy)),
                             (int(cx + up_len * math.cos(up_rad)), int(cy + up_len * math.sin(up_rad))),
                             (255, 0, 0), 2, tipLength=0.1)

            ptr_rad = math.radians(ptr_angle)
            ptr_len = r * 0.85
            cv2.arrowedLine(frame, (int(cx), int(cy)),
                             (int(cx + ptr_len * math.cos(ptr_rad)), int(cy + ptr_len * math.sin(ptr_rad))),
                             (0, 0, 255), 3, tipLength=0.1)

            status, tag = classify(ptr_angle, up_angle)
            color = {"GREEN": (0, 255, 0), "RED": (0, 0, 255), "YELLOW": (0, 255, 255)}.get(tag, (255,255,255))
            # 文字输出到终端，不在画面上显示（OpenCV 无中文字体支持）
            if stable:
                print(f"\r  {status}  ptr={ptr_angle:.1f}° up={up_angle:.1f}°", end="")

        cv2.imshow("Gauge Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print()


if __name__ == "__main__":
    main()
