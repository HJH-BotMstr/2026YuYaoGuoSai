#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仪表盘视觉模块：给主程序调用的简单函数接口（纯 Python，不依赖 ROS）。

功能：
  - init()            初始化：加载 YOLO 双模型 + 打开摄像头 + 预热（只调一次，较慢）
  - recognize()       识别一次：返回当前表盘的字母/区域/状态，存入结果表并语音播报
  - get_state(letter) 查询某个字母的状态：'normal' / 'abnormal' / None（未识别过）
  - get_all()         查询全部结果：{'A': 'normal', 'D': 'abnormal', ...}
  - shutdown()        释放摄像头（程序结束时调用）

语音播报：与 gauge_yolo_new_v2.py 逻辑一致，识别到字母时按 (字母, 区域)
播放对应 MP3（如 A 表绿区播 AM.mp3），状态不变不重复播报。
init(voice_enabled=False) 可关闭。

用法示例：
    import sys
    sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
    import gauge_vision as gv

    gv.init()                       # 初始化 + 摄像头预热（engine 加载约 30~60 秒）
    r = gv.recognize()              # {'letter': 'A', 'zone': 'GREEN', 'state': 'normal'}
    print(gv.get_all())             # {'A': 'normal'}
    print(gv.get_state('D'))        # None（还没识别过）
    gv.shutdown()

    或
    import sys
    sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
    import gauge_vision as gv

    gv.init()            # 程序启动时调一次：加载模型 + 打开 video0 + 预热 80 帧
                         #（engine 加载要 30~60 秒，主程序要预留这个时间）

    r = gv.recognize()   # 需要识别时调一次，返回：
                         # {'letter': 'A', 'zone': 'GREEN', 'state': 'normal'}
                         # letter 可能为 None（这次没识别出字母）
                         # zone/state 为 None 表示画面里没检测到表盘

    gv.get_all()         # 查全部：{'A': 'normal', 'D': 'abnormal'}
    gv.get_state('D')    # 查单个：'normal' / 'abnormal' / None（没识别过）

    gv.shutdown()        # 程序结束时调，释放摄像头

识别逻辑与 gauge_yolo_new_v2.py 完全一致（同一份代码），
区域阈值默认 thr_high=41 / thr_low=145（按 0.7MPa / 0.3MPa 校准）。
"""

import os
import shutil
import sys
import threading
import time
from pathlib import Path

# 保证能 import 同目录下的 gauge_yolo_new_v2
sys.path.insert(0, str(Path(__file__).resolve().parent))

from gauge_yolo_new_v2 import (
    GaugeYOLORecognizer,
    clear_buffer,
    get_voice_filename,
    init_camera,
    play_mp3,
    preheat_camera,
    recognize_letter_box,
)

_MODULE_DIR = Path(__file__).resolve().parent
DEFAULT_MODELS_DIR = (_MODULE_DIR / '../assets/models').resolve()
DEFAULT_MP3_DIR = (_MODULE_DIR / '../assets/mp3').resolve()
DEFAULT_DEBUG_LETTER_DIR = (_MODULE_DIR / '../assets/letter_debug').resolve()


class GaugeVision:
    """仪表盘识别器：后台线程持续取图，recognize() 时拿最新帧做推理。"""

    def __init__(self, camera_id=0, width=640, height=480, preheat_frames=80,
                 models_dir=DEFAULT_MODELS_DIR, use_engine=True, device=0, imgsz=640,
                 thr_high=41.0, thr_low=145.0,
                 voice_enabled=True, mp3_dir=DEFAULT_MP3_DIR,
                 debug_letter=False, debug_letter_dir=DEFAULT_DEBUG_LETTER_DIR):
        # 1. 加载模型（最耗时）
        self.recognizer = GaugeYOLORecognizer(
            models_dir=models_dir, use_engine=use_engine, device=device, imgsz=imgsz,
            thr_high=thr_high, thr_low=thr_low,
        )

        # 2. 初始化并预热摄像头
        self.cap = init_camera(camera_id, width, height)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 /dev/video{camera_id}")
        preheat_camera(self.cap, frames=preheat_frames)

        # 3. 结果存储：{'A': 'normal', 'B': 'abnormal', ...}
        self.results = {}

        # 4. 语音播报：与 gauge_yolo_new_v2 逻辑一致，(字母, 区域) 变化时才播
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

        self.debug_letter = debug_letter
        self.debug_letter_dir = debug_letter_dir
        self.debug_idx = 0

        # 5. 后台取图线程，保证 recognize() 拿到的是最新画面
        self.latest_frame = None
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

    def _capture_loop(self):
        while self.running:
            ok, frame = clear_buffer(self.cap)
            if ok:
                self.latest_frame = frame
            else:
                time.sleep(0.01)
            time.sleep(0.001)

    def _get_frame(self):
        if self.latest_frame is None:
            return None
        return self.latest_frame.copy()

    def recognize(self, retries=3, retry_interval=0.4):
        """
        识别一次当前画面中的仪表盘。

        返回 dict：
            {'letter': 'A' 或 None,   # None 表示这次没识别出字母
             'zone': 'RED'/'GREEN'/'YELLOW' 或 None,  # None 表示没检测到表盘
             'state': 'normal'/'abnormal' 或 None}
        识别到字母时会自动存入结果表（get_all / get_state 可查）。

        retries: 没检测到表盘或没识别出字母时的重试次数（每次取最新一帧）。
        """
        result = {'letter': None, 'zone': None, 'state': None}

        for _ in range(retries):
            frame = self._get_frame()
            if frame is None:
                time.sleep(retry_interval)
                continue

            status, tag, vis, gauge_info, gauge_box = self.recognizer.process(frame)
            if status is None or gauge_box is None:
                time.sleep(retry_interval)
                continue

            state = 'normal' if tag == 'GREEN' else 'abnormal'
            result['zone'] = tag
            result['state'] = state

            self.debug_idx += 1
            letter = recognize_letter_box(
                frame, gauge_box,
                debug=self.debug_letter,
                debug_dir=self.debug_letter_dir,
                debug_idx=self.debug_idx,
            )
            if letter is not None:
                result['letter'] = letter
                self.results[letter] = state
                self._speak(letter, tag)
                break

            time.sleep(retry_interval)

        return result

    def _speak(self, letter, tag):
        """语音播报：与 gauge_yolo_new_v2 一致，(字母, 区域) 变化时才播对应 MP3。"""
        if not self.voice_enabled or not letter or not tag:
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

    def get_state(self, letter):
        """查询某个字母的状态：'normal' / 'abnormal' / None（未识别过）。"""
        return self.results.get(letter)

    def get_all(self):
        """查询全部已识别结果，返回 dict 副本。"""
        return dict(self.results)

    def shutdown(self):
        """释放摄像头和取图线程。"""
        self.running = False
        self.capture_thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()


# ===================== 模块级函数接口（推荐主程序用这层） =====================

_vision = None


def _require_init():
    if _vision is None:
        raise RuntimeError("视觉模块未初始化，请先调用 gauge_vision.init()")


def init(**kwargs):
    """初始化视觉模块（加载模型 + 打开摄像头 + 预热）。只调用一次。

    可选参数（一般不用动）：
        camera_id=0, width=640, height=480, preheat_frames=80,
        models_dir=../assets/models, use_engine=True, device=0, imgsz=640,
        thr_high=41.0, thr_low=145.0,
        voice_enabled=True, mp3_dir=../assets/mp3,
        debug_letter=False, debug_letter_dir=../assets/letter_debug
    """
    global _vision
    if _vision is not None:
        print("视觉模块已初始化，跳过重复 init")
        return
    _vision = GaugeVision(**kwargs)


def recognize(retries=3, retry_interval=0.4):
    """识别一次当前仪表盘，返回 {'letter', 'zone', 'state'}，识别到字母会自动存储。"""
    _require_init()
    return _vision.recognize(retries=retries, retry_interval=retry_interval)


def get_state(letter):
    """查询某个字母（'A'/'B'/'C'/'D'）的状态：'normal' / 'abnormal' / None。"""
    _require_init()
    return _vision.get_state(letter)


def get_all():
    """查询全部已识别结果：{'A': 'normal', ...}"""
    _require_init()
    return _vision.get_all()


def shutdown():
    """释放摄像头。程序结束时调用。"""
    global _vision
    if _vision is not None:
        _vision.shutdown()
        _vision = None


# ===================== 自测试 =====================

if __name__ == '__main__':
    print("视觉模块自测试：初始化中（加载 engine 较慢）...")
    init()
    print("初始化完成，每 2 秒识别一次，Ctrl+C 退出\n")
    try:
        while True:
            r = recognize()
            print(f"识别: {r}   已存: {get_all()}")
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
        print("\n已释放摄像头")
