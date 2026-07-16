#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多线程版本仪表盘识别。

不修改 realtime_gauge.py，通过 import 复用其识别函数。
解决 Jetson 上 Python 单线程处理慢导致的画面延迟和卡顿问题。

按 q 退出。
"""

import sys
sys.path.insert(0, '/home/ysc/detect')

import cv2
import math
import threading
import time
import subprocess
import numpy as np

try:
    import pytesseract
    PYTESSERACT_AVAILABLE = True
except ImportError:
    PYTESSERACT_AVAILABLE = False
    print("警告：未安装 pytesseract，字母识别功能不可用")
    print("请执行：sudo apt install tesseract-ocr tesseract-ocr-eng && pip3 install pytesseract")

# 复用 realtime_gauge.py 中的所有识别函数（原代码不做任何修改）
from realtime_gauge import (
    detect_circle,
    extract_roi,
    enhance_roi,
    lab_threshold_centers,
    compute_up,
    polar_unwrap,
    detect_ptr,
    classify,
)


# ============================================================================
# 以下为新增函数：摄像头初始化、预热、缓冲控制
# ============================================================================

def init_camera(camera_id=6, width=640, height=480):
    """
    初始化摄像头。
    - 启用自动曝光和自动白平衡（像 guvcview 一样）
    - 降低分辨率以减轻 Jetson 处理压力
    - 把 OpenCV 内部缓冲队列大小设为 1，防止旧帧堆积造成延迟
    """
    # 启用自动模式（不固定曝光/色温）
    subprocess.run(
        ["v4l2-ctl", f"-d/dev/video{camera_id}", "--set-ctrl=exposure_auto=3"],
        check=False, capture_output=True
    )
    subprocess.run(
        ["v4l2-ctl", f"-d/dev/video{camera_id}", "--set-ctrl=white_balance_temperature_auto=1"],
        check=False, capture_output=True
    )

    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    return cap


def preheat_camera(cap, frames=80):
    """预热摄像头：丢弃前 N 帧，让自动曝光/白平衡稳定。"""
    print(f"  预热中，丢弃前 {frames} 帧 ...")
    for i in range(frames):
        ok = cap.grab()
        if not ok:
            print(f"  警告：第 {i} 帧 grab 失败")
            break
    print("  预热完成")


def clear_buffer(cap, max_discard=10):
    """清空 OpenCV 视频缓冲，返回最新一帧。"""
    for _ in range(max_discard):
        ok = cap.grab()
        if not ok:
            break
    return cap.retrieve()


def recognize_letter(frame, cx, cy, r):
    """
    识别仪表盘上方的 ABCD 字母。
    基于圆心 (cx, cy) 和半径 r 截取圆上方区域，用 Tesseract OCR 识别。
    返回识别到的大写字母（A/B/C/D），未识别到则返回 None。
    """
    if not PYTESSERACT_AVAILABLE:
        return None

    h, w = frame.shape[:2]

    # 截取圆上方区域：圆心上方 1r ~ 3r 处（字母实际位置），左右各 1.5r 宽度
    x1 = max(0, int(cx - 1.5 * r))
    y1 = max(0, int(cy - 3.0 * r))
    x2 = min(w, int(cx + 1.5 * r))
    y2 = max(0, int(cy - 1.0 * r))

    if y2 <= y1 or x2 <= x1:
        return None

    letter_roi = frame[y1:y2, x1:x2]
    if letter_roi.size == 0:
        return None

    try:
        # 灰度 + OTSU 二值化
        gray = cv2.cvtColor(letter_roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 放大 2 倍，Tesseract 对高分辨率效果更好
        binary = cv2.resize(binary, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)

        # OCR：只识别单行大写字母 ABCD
        config = '--psm 7 -c tessedit_char_whitelist=ABCD'
        text = pytesseract.image_to_string(binary, config=config)
        text = text.strip()

        # 从结果中提取第一个合法字母
        for c in text:
            if c in 'ABCD':
                return c
    except Exception as e:
        print(f"\n字母识别失败: {e}")

    return None


# ============================================================================
# 以下为新增函数：圆检测颜色验证
# ============================================================================

def _validate_circle_by_color(frame_bgr, cx, cy, r, min_ratio=0.03):
    """
    粗略验证：圆的外接正方形区域内，红/黄色像素占比是否足够。
    用于排除误检到背景圆形物体的假圆。
    """
    h, w = frame_bgr.shape[:2]
    x1 = max(0, int(cx - r))
    y1 = max(0, int(cy - r))
    x2 = min(w, int(cx + r))
    y2 = min(h, int(cy + r))
    if x2 <= x1 or y2 <= y1:
        return False

    roi = frame_bgr[y1:y2, x1:x2]
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    red_mask = cv2.inRange(lab, np.array([0, 135, 73]), np.array([255, 211, 199]))
    yellow_mask = cv2.inRange(lab, np.array([0, 71, 145]), np.array([255, 175, 225]))

    total = roi.shape[0] * roi.shape[1]
    if total == 0:
        return False
    colored = cv2.countNonZero(red_mask) + cv2.countNonZero(yellow_mask)
    return (colored / total) >= min_ratio


# ============================================================================
# 以下为新增函数：快速霍夫圆检测（在原图缩小后检测，再放大坐标）
# ============================================================================

def detect_circle_fast(gray, frame_bgr=None, scale=0.5):
    """
    对灰度图先缩放再调用 HoughCircles，显著提升 Jetson 上的速度。
    找到圆后把坐标和半径按 scale 反推回原始图像。
    如果 frame_bgr 不为空，会对候选圆做颜色验证，优先返回包含红/黄色像素的圆。
    如果检测失败返回 None。
    """
    h, w = gray.shape
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    smooth = cv2.bilateralFilter(clahe.apply(small), 9, 75, 75)
    edges = cv2.Canny(smooth, 40, 120)

    # 在缩放图上合理的半径范围（原图半径约 50~240，缩放后除以 scale）
    min_r_scaled = max(15, int(40 * scale))
    max_r_scaled = int(250 * scale)

    # 只保留几组参数策略，减少重复调用
    strategies = [
        (80, 30, True),   # edges
        (60, 25, False),  # smooth
        (100, 35, True),  # edges
    ]

    all_circles = []
    for p1, p2, use_edges in strategies:
        src = edges if use_edges else smooth
        circles = cv2.HoughCircles(
            src, cv2.HOUGH_GRADIENT, dp=1,
            minDist=max(small.shape),
            param1=p1, param2=p2,
            minRadius=min_r_scaled,
            maxRadius=max_r_scaled
        )
        if circles is not None:
            for c in circles[0]:
                all_circles.append((float(c[0]), float(c[1]), float(c[2])))

    if not all_circles:
        return None

    radii = [c[2] for c in all_circles]
    median_r = float(np.median(radii))

    # 按半径接近中位数排序，优先选“最常见大小”的圆
    sorted_circles = sorted(all_circles, key=lambda c: abs(c[2] - median_r))

    # 颜色验证：优先返回包含红/黄色像素的圆
    if frame_bgr is not None:
        for c in sorted_circles:
            cx = c[0] / scale
            cy = c[1] / scale
            r = c[2] / scale
            if _validate_circle_by_color(frame_bgr, cx, cy, r):
                return (cx, cy, r)
        # 都验证失败时回退到最接近中位数的候选
        c = sorted_circles[0]
        return (c[0] / scale, c[1] / scale, c[2] / scale)

    c = min(all_circles, key=lambda c: abs(c[2] - median_r))
    return (c[0] / scale, c[1] / scale, c[2] / scale)


# ============================================================================
# 以下为新增函数：平滑版指针检测
# ============================================================================

def _smooth_circular(data, kernel_size):
    """对角度序列做环形移动平均，正确处理 0°/360° 交界。"""
    n = len(data)
    half = kernel_size // 2
    extended = np.concatenate([data[-half:], data, data[:half]])
    kernel = np.ones(kernel_size, dtype=np.float32) / kernel_size
    smoothed = np.convolve(extended, kernel, mode='valid')
    return smoothed[:n]


def detect_ptr_smooth(polar, smooth_kernel=5):
    """
    在 detect_ptr 基础上增加高斯模糊和环形列均值平滑，减少噪声导致的跳变。
    """
    # 对极坐标图做轻微高斯模糊
    polar_blur = cv2.GaussianBlur(polar, (3, 3), 0)

    rows, cols = polar_blur.shape
    clip = int(rows * 0.20)
    col_means = np.mean(polar_blur[clip:, :], axis=0).astype(np.float32)

    # 对列均值做环形移动平均平滑，避免 0°/360° 边界被压到最小值
    if smooth_kernel > 1:
        col_means = _smooth_circular(col_means, smooth_kernel)

    min_col = int(np.argmin(col_means))
    if 1 <= min_col < len(col_means) - 1:
        y0, y1, y2 = col_means[min_col - 1], col_means[min_col], col_means[min_col + 1]
        d = y0 - 2.0 * y1 + y2
        offset = (y0 - y2) / (2.0 * d) if abs(d) > 1e-10 else 0.0
    else:
        offset = 0.0

    return ((min_col + offset) * 360.0 / cols) % 360.0


# ============================================================================
# 以下为新增类：多线程识别器
# ============================================================================

class AsyncGaugeProcessor:
    """
    多线程仪表盘识别器。

    - capture_thread：高帧率读取摄像头，保证画面低延迟、流畅
    - process_thread：定时处理一帧做识别，输出状态
    - 主线程：实时显示画面，并叠加最近一次识别结果
    """

    def __init__(self, camera_id=6, width=640, height=480, process_interval=0.3):
        self.cap = init_camera(camera_id, width, height)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头")

        preheat_camera(self.cap, frames=120)

        self.latest_frame = None      # 最新的摄像头帧
        self.last_state = None        # 最近一次识别结果
        self.running = True
        self.process_interval = process_interval

        # 新增：摄像头就绪标志与处理中标志
        self.camera_ready = threading.Event()
        self.frames_captured = 0
        self.min_ready_frames = 30
        self.is_processing = False

        # 新增：字母识别跳帧计数
        self.frame_count = 0
        self.letter_skip = 5
        self.last_letter = None

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)

        self.capture_thread.start()
        self.process_thread.start()

    # ------------------------------------------------------------------
    # 线程1：持续读取最新帧（高频率，保证画面低延迟）
    # ------------------------------------------------------------------
    def _capture_loop(self):
        while self.running:
            ok, frame = clear_buffer(self.cap)
            if ok:
                self.latest_frame = frame
                self.frames_captured += 1
                if self.frames_captured >= self.min_ready_frames:
                    self.camera_ready.set()
            else:
                time.sleep(0.01)
            time.sleep(0.001)

    # ------------------------------------------------------------------
    # 线程2：定时处理一帧做识别（低频率，避免卡顿）
    # ------------------------------------------------------------------
    def _process_loop(self):
        # 等待摄像头捕获足够多帧后再开始识别
        self.camera_ready.wait(timeout=10.0)
        if not self.camera_ready.is_set():
            print("\n警告：摄像头长时间未就绪")

        while self.running:
            if self.latest_frame is None or self.is_processing:
                time.sleep(0.05)
                continue

            frame = self.latest_frame.copy()
            self.is_processing = True
            try:
                state = self._process_frame(frame)
                if state is not None:
                    self.last_state = state
            except Exception as e:
                print(f"\n识别失败: {e}")
            finally:
                self.is_processing = False

            time.sleep(self.process_interval)

    # ------------------------------------------------------------------
    # 识别逻辑：复用 realtime_gauge.py 的函数，圆检测用快速版
    # ------------------------------------------------------------------
    def _process_frame(self, frame):
        t0 = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        t1 = time.time()

        # 先用快速圆检测；失败再回退到原来的 detect_circle
        circle = detect_circle_fast(gray, frame_bgr=frame, scale=0.5)
        detector_name = "fast"
        t2 = time.time()
        if not circle:
            circle = detect_circle(gray)
            detector_name = "slow"
            t2 = time.time()
        if not circle:
            print(f"\n  未检测到圆（fast+slow），总耗时 {(t2 - t0) * 1000:.1f}ms")
            return None

        cx, cy, r = circle
        roi = extract_roi(frame, cx, cy, r)
        t3 = time.time()

        roi_enh = enhance_roi(roi)
        t4 = time.time()

        cc = lab_threshold_centers(roi_enh)
        t5 = time.time()

        if "red" not in cc:
            print(f"\n  无红色区域，总耗时 {(t5 - t0) * 1000:.1f}ms")
            return None

        up_angle = compute_up(cc["red"])
        t6 = time.time()

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray_roi)
        t7 = time.time()

        ptr_angle = detect_ptr_smooth(polar_unwrap(gray_roi))
        t8 = time.time()

        status, tag = classify(ptr_angle, up_angle)
        t9 = time.time()

        # 字母识别跳帧：每 letter_skip 帧识别一次，其余帧复用上次结果
        self.frame_count += 1
        if self.frame_count % self.letter_skip == 0:
            letter = recognize_letter(frame, cx, cy, r)
            if letter is not None:
                self.last_letter = letter
        else:
            letter = self.last_letter
        t10 = time.time()

        print(
            f"\n  圆: r={r:.1f} | "
            f"耗时(ms): gray={(t1 - t0) * 1000:.1f}  "
            f"detect_circle[{detector_name}]={(t2 - t1) * 1000:.1f}  "
            f"extract_roi={(t3 - t2) * 1000:.1f}  "
            f"enhance={(t4 - t3) * 1000:.1f}  "
            f"lab_threshold={(t5 - t4) * 1000:.1f}  "
            f"compute_up={(t6 - t5) * 1000:.1f}  "
            f"clahe={(t7 - t6) * 1000:.1f}  "
            f"ptr={(t8 - t7) * 1000:.1f}  "
            f"classify={(t9 - t8) * 1000:.1f}  "
            f"letter(skip={self.frame_count % self.letter_skip != 0})={(t10 - t9) * 1000:.1f}  "
            f"TOTAL={(t10 - t0) * 1000:.1f}"
        )

        return {
            'cx': cx, 'cy': cy, 'r': r,
            'cc': cc, 'up_angle': up_angle,
            'ptr_angle': ptr_angle, 'status': status, 'tag': tag,
            'letter': letter
        }

    # ------------------------------------------------------------------
    # 绘制识别结果到画面（从原 main() 的绘制代码复制）
    # ------------------------------------------------------------------
    def _draw_state(self, frame):
        if self.last_state is None:
            return frame

        s = self.last_state
        cx, cy, r = s['cx'], s['cy'], s['r']
        cc = s['cc']

        # 仪表盘外圈
        cv2.circle(frame, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 255), -1)

        # 红色区域质心
        if "red" in cc:
            rx, ry = cc["red"]
            rx_f = int(cx + (rx - 250) / 500.0 * r * 2.2)
            ry_f = int(cy + (ry - 250) / 500.0 * r * 2.2)
            cv2.circle(frame, (rx_f, ry_f), 8, (0, 0, 255), -1)

        # 黄色区域质心
        if "yellow" in cc:
            yx, yy = cc["yellow"]
            yx_f = int(cx + (yx - 250) / 500.0 * r * 2.2)
            yy_f = int(cy + (yy - 250) / 500.0 * r * 2.2)
            cv2.circle(frame, (yx_f, yy_f), 8, (0, 255, 255), -1)

        # 上方向箭头（蓝色）
        up_rad = math.radians(s['up_angle'])
        up_len = r * 0.7
        cv2.arrowedLine(frame, (int(cx), int(cy)),
                        (int(cx + up_len * math.cos(up_rad)), int(cy + up_len * math.sin(up_rad))),
                        (255, 0, 0), 2, tipLength=0.1)

        # 指针箭头（红色）
        ptr_rad = math.radians(s['ptr_angle'])
        ptr_len = r * 0.85
        cv2.arrowedLine(frame, (int(cx), int(cy)),
                        (int(cx + ptr_len * math.cos(ptr_rad)), int(cy + ptr_len * math.sin(ptr_rad))),
                        (0, 0, 255), 3, tipLength=0.1)

        # 注：不绘制任何文字到画面，保持画面为学长的可视化元素（圆、箭头、圆点）
        return frame

    # ------------------------------------------------------------------
    # 主循环：实时显示画面
    # ------------------------------------------------------------------
    def run(self):
        print("按 q 退出\n")
        while True:
            if self.latest_frame is not None:
                display = self._draw_state(self.latest_frame.copy())
                cv2.imshow("Gauge Recognition Async", display)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.running = False
                break

        self.cap.release()
        cv2.destroyAllWindows()
        print()


# ============================================================================
# 主入口
# ============================================================================

def main():
    print("启动多线程仪表盘识别...")
    processor = AsyncGaugeProcessor(
        camera_id=4,
        width=640,
        height=480,
        process_interval=0.3   # 每 300ms 识别一次，画面保持流畅
    )
    processor.run()


if __name__ == "__main__":
    main()
