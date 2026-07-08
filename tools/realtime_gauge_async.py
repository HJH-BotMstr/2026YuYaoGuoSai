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

        preheat_camera(self.cap, frames=80)

        self.latest_frame = None      # 最新的摄像头帧
        self.last_state = None        # 最近一次识别结果
        self.running = True
        self.process_interval = process_interval

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
            time.sleep(0.001)

    # ------------------------------------------------------------------
    # 线程2：定时处理一帧做识别（低频率，避免卡顿）
    # ------------------------------------------------------------------
    def _process_loop(self):
        while self.running:
            if self.latest_frame is None:
                time.sleep(0.05)
                continue

            frame = self.latest_frame.copy()
            try:
                state = self._process_frame(frame)
                if state is not None:
                    self.last_state = state
            except Exception as e:
                print(f"\n识别失败: {e}")

            time.sleep(self.process_interval)

    # ------------------------------------------------------------------
    # 识别逻辑：完全复用 realtime_gauge.py 的函数，不做任何修改
    # ------------------------------------------------------------------
    def _process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circle = detect_circle(gray)
        if not circle:
            return None

        cx, cy, r = circle
        roi = extract_roi(frame, cx, cy, r)
        roi_enh = enhance_roi(roi)
        cc = lab_threshold_centers(roi_enh)

        if "red" not in cc:
            return None

        up_angle = compute_up(cc["red"])
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray_roi)
        ptr_angle = detect_ptr(polar_unwrap(gray_roi))

        status, tag = classify(ptr_angle, up_angle)

        # 新增：识别仪表盘上方字母
        letter = recognize_letter(frame, cx, cy, r)

        print(f"\r  {status}  ptr={ptr_angle:.1f} deg up={up_angle:.1f} deg letter={letter}", end="")

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
        camera_id=6,
        width=640,
        height=480,
        process_interval=0.3   # 每 300ms 识别一次，画面保持流畅
    )
    processor.run()


if __name__ == "__main__":
    main()
