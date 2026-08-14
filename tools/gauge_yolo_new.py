#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8 双模型仪表盘识别（Jetson 版）+ MP3 语音播报。

特性：
  - 优先使用 TensorRT .engine 加速推理；若不存在则自动从 .pt 导出
  - 摄像头初始化、预热、缓冲清理（默认 /dev/video4）
  - 多线程：高帧率取图 + 定时推理，避免画面卡顿
  - 识别到字母 A/B/C/D 时，按区域播放对应 MP3 语音

用法：
    source ~/yolov8_env/bin/activate
    python /home/ysc/2026YuYaoGuoSai/tools/gauge_yolo_new.py

按 q 退出。
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.nn.modules.head import Detect, Pose

try:
    import pytesseract
    PYTESSERACT_AVAILABLE = True
except ImportError:
    PYTESSERACT_AVAILABLE = False


# ===================== 路径与语音配置 =====================

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODELS_DIR = (SCRIPT_DIR / '../assets/models').resolve()
DEFAULT_MP3_DIR = (SCRIPT_DIR / '../assets/mp3').resolve()
DEFAULT_DEBUG_LETTER_DIR = (SCRIPT_DIR / '../assets/letter_debug').resolve()

ZONE_SUFFIX = {
    'YELLOW': 'L',   # 偏低
    'RED': 'H',      # 偏高
    'GREEN': 'M',    # 居中
}

STATUS_TO_TAG = {
    '偏高': 'RED',
    '居中': 'GREEN',
    '偏低': 'YELLOW',
}

TAG_TO_CN = {
    'RED': '红',
    'GREEN': '绿',
    'YELLOW': '黄',
}

STATE_TO_CN = {
    'normal': '正常',
    'abnormal': '异常',
}


# ===================== 语音工具 =====================

def get_voice_filename(letter, tag):
    """根据字母和区域 tag 生成 MP3 文件名。支持 A/B/C/D。"""
    if letter not in 'ABCD':
        return None
    suffix = ZONE_SUFFIX.get(tag)
    if suffix is None:
        return None
    return f"{letter}{suffix}.mp3"


def play_mp3(filepath):
    """用 ffmpeg 解码成 WAV，再用 aplay -D pulse 播放。"""
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"音频文件不存在: {filepath}")
    if shutil.which('ffmpeg') is None:
        raise RuntimeError("缺少 ffmpeg")
    if shutil.which('aplay') is None:
        raise RuntimeError("缺少 aplay")

    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run(
            ['ffmpeg', '-y', '-i', filepath,
             '-vn', '-acodec', 'pcm_s16le', '-ar', '44100', '-ac', '2',
             tmp_path],
            check=True, capture_output=True
        )
        subprocess.run(
            ['aplay', '-D', 'pulse', tmp_path],
            check=True, capture_output=True
        )
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


# ===================== 摄像头工具 =====================

def init_camera(camera_id=4, width=640, height=480):
    """初始化摄像头并启用自动曝光/白平衡。"""
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
    """预热摄像头。"""
    print(f"  预热中，丢弃前 {frames} 帧 ...")
    for i in range(frames):
        if not cap.grab():
            print(f"  警告：第 {i} 帧 grab 失败")
            break
    print("  预热完成")


def clear_buffer(cap, max_discard=10):
    """清空缓冲并返回最新帧。"""
    for _ in range(max_discard):
        if not cap.grab():
            break
    return cap.retrieve()


# ===================== 字母识别 =====================

def recognize_letter(frame, cx, cy, r):
    """基于表盘圆心/半径截取上方区域，用 Tesseract 识别 A/B/C/D。"""
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
        print(f"字母识别失败: {e}")

    return None


def recognize_letter_box(frame, gauge_box, debug=False, debug_dir=None, debug_idx=0):
    """
    基于 YOLO 检测到的表盘矩形框识别 A/B/C/D 字母。
    会尝试框上方区域，以及框内上半部分，取第一个识别到的合法字母。
    """
    if not PYTESSERACT_AVAILABLE:
        if debug:
            print(f"[字母调试 {debug_idx}] pytesseract 未安装")
        return None

    h, w = frame.shape[:2]
    x1, y1, x2, y2 = map(int, gauge_box)
    box_w = x2 - x1
    box_h = y2 - y1

    candidates = [
        # 表盘框上方区域
        (max(0, int(x1 - box_w * 0.1)),
         max(0, int(y1 - box_h * 0.8)),
         min(w, int(x2 + box_w * 0.1)),
         y1,
         'above'),
        # 表盘框内上半部分（有些训练数据把字母也框进了 gauge）
        (x1,
         y1,
         x2,
         int(y1 + box_h * 0.45),
         'top_inside'),
    ]

    for roi_x1, roi_y1, roi_x2, roi_y2, name in candidates:
        if roi_y2 <= roi_y1 or roi_x2 <= roi_x1:
            if debug:
                print(f"[字母调试 {debug_idx}] 区域 '{name}' 为空，跳过")
            continue

        letter_roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        if letter_roi.size == 0:
            continue

        try:
            gray = cv2.cvtColor(letter_roi, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            binary = cv2.resize(binary, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            config = '--psm 7 -c tessedit_char_whitelist=ABCD'
            text = pytesseract.image_to_string(binary, config=config).strip()
        except Exception as e:
            if debug:
                print(f"[字母调试 {debug_idx}] 区域 '{name}' OCR 失败: {e}")
            continue

        raw_text = text.replace('\n', ' ').replace('/', '_') or 'empty'
        if debug:
            print(f"[字母调试 {debug_idx}] 区域 '{name}' ROI=({roi_x1},{roi_y1},{roi_x2},{roi_y2}), "
                  f"OCR原始结果: '{raw_text}'")
            if debug_dir is not None:
                os.makedirs(debug_dir, exist_ok=True)
                save_path = os.path.join(
                    debug_dir, f"letter_{debug_idx:04d}_{name}_{raw_text}.png"
                )
                cv2.imwrite(save_path, letter_roi)

        for c in text:
            if c in 'ABCD':
                return c

    return None


# ===================== YOLO 双模型识别器 =====================

def patch_pose_head(model):
    """
    兼容用新版 ultralytics 训练的 pose 模型。
    新版 Pose head 不再保存 self.detect 属性，而 ultralytics 8.1.0 需要它。
    """
    try:
        head = model.model.model[-1]
    except Exception:
        return
    if isinstance(head, Pose) and not hasattr(head, 'detect'):
        head.detect = Detect.forward
        print('已自动修补 Pose head')


class GaugeYOLORecognizer:
    def __init__(self, models_dir, use_engine=True, device=0, imgsz=640):
        self.models_dir = Path(models_dir)
        self.device = device
        self.imgsz = imgsz

        bg_pt = self.models_dir / 'best_bg.pt'
        ptr_pt = self.models_dir / 'best_ptr.pt'
        bg_engine = self.models_dir / 'best_bg.engine'
        ptr_engine = self.models_dir / 'best_ptr.engine'

        if use_engine:
            if not bg_engine.is_file():
                print(f"未找到 {bg_engine}，正在从 {bg_pt} 导出 TensorRT engine ...")
                YOLO(str(bg_pt)).export(format='engine', device=device, half=True, imgsz=imgsz, workspace=2)
            if not ptr_engine.is_file():
                print(f"未找到 {ptr_engine}，正在从 {ptr_pt} 导出 TensorRT engine ...")
                YOLO(str(ptr_pt)).export(format='engine', device=device, half=True, imgsz=imgsz, workspace=2)
            bg_path = str(bg_engine)
            ptr_path = str(ptr_engine)
        else:
            bg_path = str(bg_pt)
            ptr_path = str(ptr_pt)

        print("加载 regions 模型...")
        self.model_reg = YOLO(bg_path, task='detect')
        patch_pose_head(self.model_reg)
        print("加载 pointer 模型...")
        self.model_ptr = YOLO(ptr_path, task='pose')
        patch_pose_head(self.model_ptr)
        print("✓ 全部加载完成\n")

    @staticmethod
    def angle(x1, y1, x2, y2):
        """计算两点连线角度，0°指向右，逆时针增加（OpenCV 坐标系）。"""
        deg = math.degrees(math.atan2(-(y2 - y1), x2 - x1))
        return deg if deg >= 0 else deg + 360

    @staticmethod
    def classify(rel_angle):
        """根据相对角度划分区域：偏高/居中/偏低。"""
        rel = rel_angle % 360
        if rel > 180:
            rel -= 360

        if 0 <= rel <= 45:
            return "偏高", (0, 0, 255)
        elif 45 < rel <= 135:
            return "居中", (0, 255, 0)
        else:
            return "偏低", (0, 255, 255)

    @staticmethod
    def resize_display(img, max_w=1200, max_h=800):
        h, w = img.shape[:2]
        scale = min(max_w / w, max_h / h, 1.0)
        if scale < 1.0:
            return cv2.resize(img, (int(w * scale), int(h * scale)))
        return img

    def process(self, frame):
        """
        单帧推理。
        返回：(status, tag, vis, gauge_info, gauge_box)
        其中任一失败时 status 为 None；gauge_info/gauge_box 用于字母识别。
        """
        h, w = frame.shape[:2]

        # 1. regions 模型：表盘 + 红色区域
        res_reg = self.model_reg(frame, verbose=False, device=self.device)[0]
        boxes = res_reg.boxes

        gauge_box = red_box = None
        if boxes is not None:
            for box in boxes:
                cls = int(box.cls[0])
                xyxy = box.xyxy[0].cpu().numpy()
                if cls == 0:
                    gauge_box = xyxy
                elif cls == 1:
                    red_box = xyxy

        if gauge_box is None:
            return None, None, frame.copy(), None, None

        cx = (gauge_box[0] + gauge_box[2]) / 2
        cy = (gauge_box[1] + gauge_box[3]) / 2
        r = max(gauge_box[2] - gauge_box[0], gauge_box[3] - gauge_box[1]) / 2

        if red_box is not None:
            rx = (red_box[0] + red_box[2]) / 2
            ry = (red_box[1] + red_box[3]) / 2
            up_angle = self.angle(cx, cy, rx, ry)
        else:
            up_angle = 218.83

        # 2. pointer 模型：指针关键点和针尖
        res_ptr = self.model_ptr(frame, verbose=False, device=self.device)[0]
        kpts = res_ptr.keypoints

        if kpts is None or len(kpts.xy) == 0:
            return None, None, frame.copy(), None, None

        pts = kpts.xy[0].cpu().numpy()
        if len(pts) < 2:
            return None, None, frame.copy(), None, None

        rivet = (int(pts[0][0]), int(pts[0][1]))
        tip = (int(pts[1][0]), int(pts[1][1]))
        ptr_angle = self.angle(rivet[0], rivet[1], tip[0], tip[1])

        # 3. 计算区域
        rel_raw = (ptr_angle - up_angle) % 360
        status, color = self.classify(rel_raw)
        tag = STATUS_TO_TAG.get(status)

        # 4. 可视化：只保留框、圆点、线、箭头，不添加任何文字
        vis = frame.copy()
        g = list(map(int, gauge_box))
        cv2.rectangle(vis, (g[0], g[1]), (g[2], g[3]), (255, 0, 0), 2)

        if red_box is not None:
            rbox = list(map(int, red_box))
            cv2.rectangle(vis, (rbox[0], rbox[1]), (rbox[2], rbox[3]), (0, 0, 255), 2)

        cv2.circle(vis, rivet, 10, (0, 255, 0), -1)
        cv2.circle(vis, tip, 10, (255, 0, 0), -1)
        cv2.line(vis, rivet, tip, (0, 0, 255), 3)

        up_rad = math.radians(up_angle)
        up_len = math.hypot(g[2] - g[0], g[3] - g[1]) * 0.35
        cv2.arrowedLine(vis, (int(cx), int(cy)),
                        (int(cx + up_len * math.cos(up_rad)), int(cy + up_len * math.sin(up_rad))),
                        (255, 0, 0), 2, tipLength=0.1)

        ptr_rad = math.radians(ptr_angle)
        ptr_len = math.hypot(g[2] - g[0], g[3] - g[1]) * 0.4
        cv2.arrowedLine(vis, (int(cx), int(cy)),
                        (int(cx + ptr_len * math.cos(ptr_rad)), int(cy + ptr_len * math.sin(ptr_rad))),
                        (0, 0, 255), 3, tipLength=0.1)

        gauge_info = {'cx': cx, 'cy': cy, 'r': r}
        return status, tag, vis, gauge_info, gauge_box


# ===================== 多线程实时处理 =====================

class AsyncYOLOGaugeProcessor:
    def __init__(self, camera_id=4, width=640, height=480,
                 process_interval=0.3, models_dir=DEFAULT_MODELS_DIR,
                 use_engine=True, device=0, imgsz=640,
                 voice_enabled=True, mp3_dir=DEFAULT_MP3_DIR,
                 display=True, debug_letter=False,
                 debug_letter_dir=DEFAULT_DEBUG_LETTER_DIR):
        self.recognizer = GaugeYOLORecognizer(
            models_dir=models_dir, use_engine=use_engine, device=device, imgsz=imgsz
        )

        self.cap = init_camera(camera_id, width, height)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 /dev/video{camera_id}")
        preheat_camera(self.cap, frames=80)

        self.latest_frame = None
        self.last_state = None       # status, tag, rel, letter, vis
        self.running = True
        self.is_processing = False
        self.process_interval = process_interval
        self.frames_captured = 0
        self.camera_ready = threading.Event()
        self.frame_count = 0
        self.letter_skip = 5
        self.last_letter = None
        self.display = display
        self.display_ok = True
        self.last_print_state = None
        self.debug_letter = debug_letter
        self.debug_letter_dir = debug_letter_dir

        # 语音配置
        self.voice_enabled = voice_enabled
        self.mp3_dir = mp3_dir
        self.last_voice_state = None
        self.voice_thread = None

        if self.voice_enabled:
            if shutil.which('ffmpeg') is None or shutil.which('aplay') is None:
                print("警告：未找到 ffmpeg 或 aplay，语音播报已禁用")
                self.voice_enabled = False
            else:
                print(f"语音播报已启用（MP3 目录: {mp3_dir}）")

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.capture_thread.start()
        self.process_thread.start()

    def _capture_loop(self):
        while self.running:
            ok, frame = clear_buffer(self.cap)
            if ok:
                self.latest_frame = frame
                self.frames_captured += 1
                if self.frames_captured >= 30:
                    self.camera_ready.set()
            else:
                time.sleep(0.01)
            time.sleep(0.001)

    def _process_loop(self):
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
                status, tag, vis, gauge_info, gauge_box = self.recognizer.process(frame)
                if status is not None and gauge_box is not None:
                    # 字母跳帧：每 letter_skip 帧识别一次，减少 OCR 抖动
                    self.frame_count += 1
                    if self.frame_count % self.letter_skip == 0:
                        letter = recognize_letter_box(
                            frame, gauge_box,
                            debug=self.debug_letter,
                            debug_dir=self.debug_letter_dir,
                            debug_idx=self.frame_count,
                        )
                        if letter is not None:
                            self.last_letter = letter
                    else:
                        letter = self.last_letter

                    state = {
                        'status': status,
                        'tag': tag,
                        'letter': self.last_letter,
                        'vis': vis,
                    }
                    self.last_state = state
                    self._speak_state(state)
                    self._print_state(state)
                else:
                    # 未识别到时，仍保留一帧可视化
                    self.last_state = {'vis': vis}
            except Exception as e:
                print(f"\n识别失败: {e}")
            finally:
                self.is_processing = False

            time.sleep(self.process_interval)

    def _speak_state(self, state):
        if not self.voice_enabled:
            return

        letter = state.get('letter')
        tag = state.get('tag')
        if not letter or not tag:
            return

        current = (letter, tag)
        if self.last_voice_state == current:
            return
        self.last_voice_state = current

        filename = get_voice_filename(letter, tag)
        if filename is None:
            return

        filepath = os.path.join(self.mp3_dir, filename)
        if not os.path.isfile(filepath):
            print(f"音频文件不存在：{filepath}")
            return

        print(f"语音播报：{filename}")

        def _play():
            try:
                play_mp3(filepath)
            except Exception as e:
                print(f"语音播报失败：{e}")

        self.voice_thread = threading.Thread(target=_play, daemon=True)
        self.voice_thread.start()

    def _print_state(self, state):
        """命令行简洁输出：字母,区域,状态。状态变化时才打印。"""
        letter = state.get('letter') or '未识别'
        tag = state.get('tag')
        if tag is None:
            return

        region = TAG_TO_CN.get(tag, tag)
        state_cn = '正常' if tag == 'GREEN' else '异常'
        current = (letter, tag)

        if self.last_print_state == current:
            return
        self.last_print_state = current

        print(f"{letter},{region},{state_cn}")

    def run(self):
        print("按 q 退出\n")
        while True:
            if self.last_state is not None and 'vis' in self.last_state:
                display = self.recognizer.resize_display(self.last_state['vis'])
            elif self.latest_frame is not None:
                display = self.latest_frame.copy()
            else:
                time.sleep(0.01)
                continue

            if self.display and self.display_ok:
                try:
                    cv2.imshow("YOLO Gauge Jetson", display)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.running = False
                        break
                except cv2.error as e:
                    print(f"无法显示画面（可能是无图形界面或 headless OpenCV）：{e}")
                    self.display_ok = False
                    continue
            else:
                # 无显示时通过打印状态保持反馈
                if self.last_state and 'tag' in self.last_state:
                    self._print_state(self.last_state)
                time.sleep(0.1)

        self.cap.release()
        if self.display_ok:
            cv2.destroyAllWindows()
        print()


# ===================== 主入口 =====================

def main():
    parser = argparse.ArgumentParser(description='YOLOv8 仪表盘识别 Jetson 版')
    parser.add_argument('--camera', type=int, default=6, help='摄像头编号，默认 6')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--models-dir', type=str, default=str(DEFAULT_MODELS_DIR),
                        help='模型目录')
    parser.add_argument('--no-engine', action='store_true',
                        help='不使用 TensorRT engine，直接用 .pt')
    parser.add_argument('--device', type=int, default=0, help='GPU 编号')
    parser.add_argument('--imgsz', type=int, default=640, help='推理尺寸')
    parser.add_argument('--interval', type=float, default=0.3,
                        help='处理间隔（秒）')
    parser.add_argument('--no-voice', action='store_true', help='关闭语音播报')
    parser.add_argument('--mp3-dir', type=str, default=str(DEFAULT_MP3_DIR),
                        help='MP3 目录')
    parser.add_argument('--no-display', action='store_true',
                        help='不显示画面，仅打印状态（适用于无图形界面）')
    parser.add_argument('--debug-letter', action='store_true',
                        help='开启字母识别调试：打印 OCR 结果并保存 ROI 图片')
    parser.add_argument('--debug-letter-dir', type=str, default=str(DEFAULT_DEBUG_LETTER_DIR),
                        help='字母 ROI 调试图片保存目录')
    args = parser.parse_args()

    processor = AsyncYOLOGaugeProcessor(
        camera_id=args.camera,
        width=args.width,
        height=args.height,
        process_interval=args.interval,
        models_dir=args.models_dir,
        use_engine=not args.no_engine,
        device=args.device,
        imgsz=args.imgsz,
        voice_enabled=not args.no_voice,
        mp3_dir=args.mp3_dir,
        display=not args.no_display,
        debug_letter=args.debug_letter,
    )
    processor.run()


if __name__ == '__main__':
    main()
