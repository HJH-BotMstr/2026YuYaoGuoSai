#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 服务节点：仪表盘识别。

节点启动时完成摄像头初始化与预热，之后通过 /detect_gauge 服务对外提供识别能力。
复用 realtime_gauge.py 与 realtime_gauge_async.py 中验证过的识别逻辑。
"""

import sys
sys.path.insert(0, '/home/ysc/detect')

import math
import threading
import time
import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from gauge_detector_interfaces.srv import GaugeDetect

try:
    import pytesseract
    PYTESSERACT_AVAILABLE = True
except ImportError:
    PYTESSERACT_AVAILABLE = False

# 复用 realtime_gauge.py 中的识别函数
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
# 摄像头与图像工具函数（从 realtime_gauge_async.py 复用）
# ============================================================================

def init_camera(camera_id=6, width=640, height=480):
    """初始化摄像头，启用自动曝光/白平衡并降低缓冲延迟。"""
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


def preheat_camera(cap, frames=120):
    """预热摄像头：丢弃前 N 帧，让自动曝光/白平衡稳定。"""
    for i in range(frames):
        if not cap.grab():
            break


def clear_buffer(cap, max_discard=10):
    """清空 OpenCV 视频缓冲，返回最新一帧。"""
    for _ in range(max_discard):
        if not cap.grab():
            break
    return cap.retrieve()


def recognize_letter(frame, cx, cy, r):
    """识别仪表盘上方的 ABCD 字母。"""
    if not PYTESSERACT_AVAILABLE:
        return None

    h, w = frame.shape[:2]
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
        gray = cv2.cvtColor(letter_roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary = cv2.resize(binary, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        config = '--psm 7 -c tessedit_char_whitelist=ABCD'
        text = pytesseract.image_to_string(binary, config=config).strip()
        for c in text:
            if c in 'ABCD':
                return c
    except Exception as e:
        pass

    return None


def _validate_circle_by_color(frame_bgr, cx, cy, r, min_ratio=0.03):
    """颜色验证：圆内红/黄色像素占比是否足够。"""
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


def detect_circle_fast(gray, frame_bgr=None, scale=0.5):
    """快速霍夫圆检测：先缩放再检测，可选颜色验证。"""
    h, w = gray.shape
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    smooth = cv2.bilateralFilter(clahe.apply(small), 9, 75, 75)
    edges = cv2.Canny(smooth, 40, 120)

    min_r_scaled = max(15, int(40 * scale))
    max_r_scaled = int(250 * scale)

    strategies = [
        (80, 30, True),
        (60, 25, False),
        (100, 35, True),
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
    sorted_circles = sorted(all_circles, key=lambda c: abs(c[2] - median_r))

    if frame_bgr is not None:
        for c in sorted_circles:
            cx = c[0] / scale
            cy = c[1] / scale
            r = c[2] / scale
            if _validate_circle_by_color(frame_bgr, cx, cy, r):
                return (cx, cy, r)
        c = sorted_circles[0]
        return (c[0] / scale, c[1] / scale, c[2] / scale)

    c = min(all_circles, key=lambda c: abs(c[2] - median_r))
    return (c[0] / scale, c[1] / scale, c[2] / scale)


def _smooth_circular(data, kernel_size):
    """对角度序列做环形移动平均。"""
    n = len(data)
    half = kernel_size // 2
    extended = np.concatenate([data[-half:], data, data[:half]])
    kernel = np.ones(kernel_size, dtype=np.float32) / kernel_size
    smoothed = np.convolve(extended, kernel, mode='valid')
    return smoothed[:n]


def detect_ptr_smooth(polar, smooth_kernel=5):
    """平滑版指针检测。"""
    polar_blur = cv2.GaussianBlur(polar, (3, 3), 0)
    rows, cols = polar_blur.shape
    clip = int(rows * 0.20)
    col_means = np.mean(polar_blur[clip:, :], axis=0).astype(np.float32)

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
# ROS2 节点
# ============================================================================

class GaugeServerNode(Node):
    def __init__(self):
        super().__init__('gauge_server')

        # 参数声明
        self.declare_parameter('camera_id', 6)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('preheat_frames', 120)

        camera_id = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        preheat_frames = self.get_parameter('preheat_frames').value

        self.get_logger().info(f'初始化摄像头 /dev/video{camera_id} ...')
        self.cap = init_camera(camera_id, width, height)
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('无法打开摄像头，节点启动失败')
            raise RuntimeError('无法打开摄像头')

        self.get_logger().info(f'预热摄像头，丢弃前 {preheat_frames} 帧 ...')
        preheat_camera(self.cap, frames=preheat_frames)
        self.get_logger().info('摄像头预热完成')

        self.latest_frame = None
        self.running = True
        self.is_processing = False
        self.frame_count = 0
        self.letter_skip = 5
        self.last_letter = None

        # 后台取图线程
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # 创建服务
        self.srv = self.create_service(
            GaugeDetect,
            'detect_gauge',
            self.detect_callback
        )
        self.get_logger().info('服务 /detect_gauge 已创建')

    def _capture_loop(self):
        """持续读取最新帧。"""
        while self.running:
            ok, frame = clear_buffer(self.cap)
            if ok:
                self.latest_frame = frame
            else:
                time.sleep(0.01)
            time.sleep(0.001)

    def _process_frame(self, frame):
        """单帧识别流程。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        circle = detect_circle_fast(gray, frame_bgr=frame, scale=0.5)
        if not circle:
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
        ptr_angle = detect_ptr_smooth(polar_unwrap(gray_roi))
        status, tag = classify(ptr_angle, up_angle)

        # 字母识别跳帧
        self.frame_count += 1
        if self.frame_count % self.letter_skip == 0:
            letter = recognize_letter(frame, cx, cy, r)
            if letter is not None:
                self.last_letter = letter
        else:
            letter = self.last_letter

        return {
            'status': status,
            'tag': tag,
            'letter': letter,
        }

    def detect_callback(self, request, response):
        """服务回调：取最新一帧识别并返回结果。"""
        if self.is_processing:
            response.success = False
            response.message = '已有识别请求正在处理，请稍后再试'
            return response

        if self.latest_frame is None:
            response.success = False
            response.message = '没有可用图像'
            return response

        self.is_processing = True
        try:
            frame = self.latest_frame.copy()
            state = self._process_frame(frame)

            if state is None:
                response.success = False
                response.message = '识别失败，未检测到有效仪表盘'
                return response

            response.success = True
            response.letter = state['letter'] if state['letter'] is not None else ''
            response.zone = state['tag']
            response.state = 'normal' if state['tag'] == 'GREEN' else 'abnormal'
            response.message = '识别成功'
        except Exception as e:
            self.get_logger().error(f'识别异常: {e}')
            response.success = False
            response.message = f'识别异常: {str(e)}'
        finally:
            self.is_processing = False

        return response

    def destroy_node(self):
        """释放资源。"""
        self.running = False
        self.capture_thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
        self.get_logger().info('摄像头已释放')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GaugeServerNode()
        rclpy.spin(node)
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'节点异常: {e}')
        else:
            print(f'节点异常: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
