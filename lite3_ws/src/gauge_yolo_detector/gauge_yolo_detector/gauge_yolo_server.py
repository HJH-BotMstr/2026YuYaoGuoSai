#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 服务节点：YOLOv8 仪表盘识别。

与纯 CV 版节点 gauge_detector 区分开，本节点使用 YOLOv8 双模型：
  - gauge_regions_3d.engine / gauge_pointer_3d_v3.engine（TensorRT 加速）

节点启动时完成：
  1. 加载 regions 模型
  2. 加载 pointer 模型
  3. 初始化摄像头并预热
因此首次启动较慢；之后通过 /detect_gauge_yolo 服务调用可快速返回识别结果。
"""

import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import cv2
import socket
import struct
import rclpy
from rclpy.node import Node

from gauge_detector_interfaces.srv import GaugeDetect

# 复用 tools/gauge_yolo_new_v2.py 中验证过的函数和类
sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
from gauge_yolo_new_v2 import (
    PYTESSERACT_AVAILABLE,
    GaugeYOLORecognizer,
    clear_buffer,
    get_voice_filename,
    init_camera,
    play_mp3,
    preheat_camera,
    recognize_letter_box,
    STATUS_TO_TAG,
)


def send_head_up_command():
    """
    发送语音指令让狗抬头（指令码 0x21010C0A, 指令值 9）
    运动主机 IP: 192.168.1.120, UDP 端口: 43893
    """
    robot_ip = "192.168.1.120"
    robot_port = 43893
    cmd_code = 0x21010C0A  # 语音指令码
    cmd_value = 9          # 抬头

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # UDP 数据包格式：<IiI (code, value, 0)
        data = struct.pack("<IiI", cmd_code, cmd_value, 0)
        sock.sendto(data, (robot_ip, robot_port))
        return True
    except Exception as e:
        return False
    finally:
        sock.close()


class GaugeYoloServerNode(Node):
    def __init__(self):
        super().__init__('gauge_yolo_server')

        # ========== 参数声明 ==========
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('preheat_frames', 80)
        self.declare_parameter('models_dir', '/home/ysc/2026YuYaoGuoSai/assets/models')
        self.declare_parameter('use_engine', True)
        self.declare_parameter('device', 0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('thr_high', 41.0)
        self.declare_parameter('thr_low', 145.0)
        self.declare_parameter('voice_enabled', True)
        self.declare_parameter('mp3_dir', '/home/ysc/2026YuYaoGuoSai/assets/mp3')
        self.declare_parameter('letter_skip', 1)
        self.declare_parameter('debug_letter', False)
        self.declare_parameter('debug_letter_dir', '/home/ysc/2026YuYaoGuoSai/assets/letter_debug')
        self.declare_parameter('voice_on_change_only', True)
        self.declare_parameter('service_name', 'detect_gauge_yolo')

        camera_id = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        preheat_frames = self.get_parameter('preheat_frames').value
        models_dir = self.get_parameter('models_dir').value
        use_engine = self.get_parameter('use_engine').value
        device = self.get_parameter('device').value
        imgsz = self.get_parameter('imgsz').value
        thr_high = self.get_parameter('thr_high').value
        thr_low = self.get_parameter('thr_low').value
        voice_enabled = self.get_parameter('voice_enabled').value
        mp3_dir = self.get_parameter('mp3_dir').value
        self.letter_skip = self.get_parameter('letter_skip').value
        self.debug_letter = self.get_parameter('debug_letter').value
        self.debug_letter_dir = self.get_parameter('debug_letter_dir').value
        self.voice_on_change_only = self.get_parameter('voice_on_change_only').value
        service_name = self.get_parameter('service_name').value

        # ========== 1. 加载 YOLO 模型（启动时最耗时的部分） ==========
        self.get_logger().info('加载 regions 模型...')
        self.get_logger().info('加载 pointer 模型...')
        try:
            self.recognizer = GaugeYOLORecognizer(
                models_dir=models_dir,
                use_engine=use_engine,
                device=device,
                imgsz=imgsz,
                thr_high=thr_high,
                thr_low=thr_low,
            )
            self.get_logger().info('✓ 全部加载完成')
        except Exception as e:
            self.get_logger().error(f'模型加载失败: {e}')
            raise

        # ========== 2. 发送抬头指令，避免仪表盘变形导致识别失败 ==========
        self.get_logger().info('发送抬头指令到运动主机...')
        if send_head_up_command():
            self.get_logger().info('✓ 抬头指令已发送，等待 3 秒让狗完成动作')
            time.sleep(3.0)
        else:
            self.get_logger().warn('⚠ 抬头指令发送失败，继续初始化（可能影响识别效果）')

        # ========== 3. 初始化并预热摄像头 ==========
        self.get_logger().info(f'初始化摄像头 /dev/video{camera_id} ...')
        self.cap = init_camera(camera_id, width, height)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(f'无法打开摄像头 /dev/video{camera_id}')

        self.get_logger().info(f'预热中，丢弃前 {preheat_frames} 帧 ...')
        preheat_camera(self.cap, frames=preheat_frames)
        self.get_logger().info('预热完成')

        # ========== 状态变量 ==========
        self.latest_frame = None
        self.running = True
        self.is_processing = False
        self.frame_count = 0
        self.last_letter = None

        # 语音播报
        self.voice_enabled = voice_enabled
        self.mp3_dir = mp3_dir
        self.last_voice_state = None
        self.voice_thread = None
        if self.voice_enabled:
            if shutil.which('ffmpeg') is None or shutil.which('aplay') is None:
                self.get_logger().warn('未找到 ffmpeg 或 aplay，语音播报已禁用')
                self.voice_enabled = False
            else:
                self.get_logger().info(f'语音播报已启用（MP3 目录: {mp3_dir}）')

        if not PYTESSERACT_AVAILABLE:
            self.get_logger().warn('pytesseract 未安装，字母识别不可用')
        else:
            self.get_logger().info('pytesseract 已就绪，字母识别可用')

        # 后台取图线程
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # ========== 创建 ROS 服务 ==========
        self.srv = self.create_service(
            GaugeDetect,
            service_name,
            self.detect_callback,
        )
        self.get_logger().info(f'服务 /{service_name} 已创建')

    def _capture_loop(self):
        """持续读取最新帧，保证服务调用时拿到的是当前画面。"""
        while self.running:
            ok, frame = clear_buffer(self.cap)
            if ok:
                self.latest_frame = frame
            else:
                time.sleep(0.01)
            time.sleep(0.001)

    def _process_frame(self, frame):
        """单帧识别，返回 state 字典或 None。"""
        status, tag, vis, gauge_info, gauge_box = self.recognizer.process(frame)
        if status is None or gauge_box is None:
            return None

        # 字母识别跳帧，减少 OCR 抖动
        self.frame_count += 1
        if self.frame_count % self.letter_skip == 0:
            letter = recognize_letter_box(
                frame,
                gauge_box,
                debug=self.debug_letter,
                debug_dir=self.debug_letter_dir,
                debug_idx=self.frame_count,
            )
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
        """服务回调：取最新帧进行 YOLO 推理并返回结果。"""
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
                # 失败时把当前帧存下来，便于排查（画面黑、方向错、找错摄像头等）
                debug_path = f'/tmp/gauge_fail_{int(time.time())}.jpg'
                try:
                    cv2.imwrite(debug_path, frame)
                    self.get_logger().warn(f'未检测到仪表盘，已保存当前帧: {debug_path}')
                except Exception as e:
                    self.get_logger().warn(f'保存失败帧异常: {e}')
                response.success = False
                response.message = f'识别失败，未检测到有效仪表盘（已保存 {debug_path}）'
                return response

            response.success = True
            response.letter = state['letter'] if state['letter'] is not None else ''
            response.zone = state['tag']
            response.state = 'normal' if state['tag'] == 'GREEN' else 'abnormal'
            response.message = '识别成功'

            self.get_logger().info(
                f"识别结果：letter={response.letter}, zone={response.zone}, state={response.state}"
            )
            self._speak_state(state)

        except Exception as e:
            self.get_logger().error(f'识别异常: {e}')
            response.success = False
            response.message = f'识别异常: {str(e)}'
        finally:
            self.is_processing = False

        return response

    def _speak_state(self, state):
        """根据识别结果播放对应 MP3，相同状态不重复播报。"""
        if not self.voice_enabled:
            return

        letter = state.get('letter')
        tag = state.get('tag')
        if not letter or not tag:
            return

        current = (letter, tag)
        if self.voice_on_change_only and self.last_voice_state == current:
            return
        self.last_voice_state = current

        filename = get_voice_filename(letter, tag)
        if filename is None:
            self.get_logger().warn(f'无对应音频：letter={letter}, tag={tag}')
            return

        filepath = os.path.join(self.mp3_dir, filename)
        if not os.path.isfile(filepath):
            self.get_logger().warn(f'音频文件不存在：{filepath}')
            return

        self.get_logger().info(f'语音播报：{filename}')

        def _play():
            try:
                play_mp3(filepath)
            except Exception as e:
                self.get_logger().error(f'语音播报失败：{e}')

        self.voice_thread = threading.Thread(target=_play, daemon=True)
        self.voice_thread.start()

    def destroy_node(self):
        """释放摄像头和线程资源。"""
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
        node = GaugeYoloServerNode()
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
