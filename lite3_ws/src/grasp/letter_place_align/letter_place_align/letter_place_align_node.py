#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
letter place align 对齐节点（放置区 A4 纸字母对齐）

流程：
  phase_0_wait_trigger  等待外部触发 /letter_place/start (String, "A/B/C/D")
  phase_1_wait_detect   等待摄像头就绪，连续多帧检测到目标字母纸
  phase_2_yaw_align     旋转机身，消除水平角偏差
  phase_3_lateral_align 横向平移，使字母纸正对摄像头
  phase_4_approach      前进到 vision_min_distance_m（视觉闭环距离下限）
  phase_5_final_check   最终校验（角度 + 横向 + 距离同时达标）
  phase_6_blind_forward 开环前进 blind = tz_measured - target + final_forward + extra_forward
  phase_7_emit_signal   发布 /grasp/place (String)，通知 grasp 模块放置

与 apriltag_place1_node.py 同源（骨架克隆，设计文档
tools/grasp/2026-08-01-letter-place-align-design.md §6）：
状态机、运动接口、零速判定、链路预检等机制完全一致，差异只在：
  · 检测层：A4 轮廓 + 框内 OCR（tools/gauge_yolo_new.py::detect_letter_papers）
  · 位姿估计：纸中心像素 → 角度/横向，纸高像素 → 距离
  · approach 止步 vision_min_distance_m，盲进段更长（§4.4）
  · 触发/完成信号均为 String 且携带目标字母

依赖：
  - opencv-python, numpy, pytesseract（+ 系统 tesseract）
  - rclpy, geometry_msgs, nav_msgs, std_msgs
"""

import math
import os
import queue
import sys
import threading
import time
from collections import Counter, deque
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# 复用 tools/gauge_yolo_new.py 中验证过的检测/OCR 函数（同 gauge_yolo_detector 的做法）
sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
from gauge_yolo_new import (  # noqa: E402
    PYTESSERACT_AVAILABLE,
    detect_letter_papers,
    extract_paper_roi,
    _ocr_letter_roi,
)

# ─── 状态常量 ───────────────────────────────────────────────────────────────── #
STATE_WAIT_TRIGGER   = "wait_trigger"
STATE_WAIT_DETECT    = "wait_detect"
STATE_YAW_ALIGN      = "yaw_align"
STATE_LATERAL_ALIGN  = "lateral_align"
STATE_APPROACH       = "approach"
STATE_FINAL_CHECK    = "final_check"
STATE_BLIND_FORWARD  = "blind_forward"
STATE_DONE           = "done"
STATE_ERROR          = "error"

VALID_LETTERS = ("A", "B", "C", "D")

_PIPELINE_TOPIC_TIMEOUT_S = 0.5  # 运动链路话题超时阈值

# A4 标准尺寸（m）：portrait 距离反推用 0.297 边，landscape 用 0.210 边
_PAPER_SIZE_M = {"portrait": 0.297, "landscape": 0.210}


class LetterPlaceAlignNode(Node):

    def __init__(self):
        super().__init__("letter_place_align_node")

        # ── 参数声明 ────────────────────────────────────────────────────────── #
        self.declare_parameter("trigger_topic",           "/letter_place/start")
        self.declare_parameter("place_topic",             "/grasp/place")
        self.declare_parameter("camera_device",           "/dev/video4")
        self.declare_parameter("image_width",             640)
        self.declare_parameter("image_height",            480)
        self.declare_parameter("fps",                     30)
        self.declare_parameter("camera_matrix",
            [388.1454, 0.0, 329.4121, 0.0, 387.7497, 223.481, 0.0, 0.0, 1.0])
        self.declare_parameter("dist_coeffs",
            [-0.1571, -0.218, -0.0024, -0.0011, 0.2089])
        self.declare_parameter("paper_orientation",       "portrait")
        self.declare_parameter("paper_aspect_ratio",      1.414)
        self.declare_parameter("aspect_tolerance",        0.25)
        self.declare_parameter("center_v_tolerance_px",   120)
        self.declare_parameter("letter_offset_x_m",       0.0)
        self.declare_parameter("min_paper_area_px",       2000)
        self.declare_parameter("max_paper_area_ratio",    0.6)
        self.declare_parameter("detect_stale_s",          1.0)
        self.declare_parameter("lost_tolerance_s",        2.0)
        self.declare_parameter("h_px_median_window",      5)
        self.declare_parameter("wrong_letter_frames",     15)
        self.declare_parameter("ocr_move_tol_px",         15.0)
        self.declare_parameter("ocr_scale_tol",           0.10)
        self.declare_parameter("target_distance_m",       0.08)
        self.declare_parameter("final_forward_offset_m",  0.20)
        self.declare_parameter("extra_forward_m",         0.20)
        self.declare_parameter("vision_min_distance_m",   0.35)
        self.declare_parameter("yaw_align_threshold_deg", 3.0)
        self.declare_parameter("max_yaw_step_deg",        3.0)
        self.declare_parameter("lateral_threshold_m",     0.03)
        self.declare_parameter("distance_threshold_m",    0.03)
        self.declare_parameter("max_rounds",              10)
        self.declare_parameter("stable_frames",           10)
        self.declare_parameter("detect_timeout_s",        15.0)
        # phase_1 超时补救：目标字母曾确认过、且当前仍有候选纸在正前方时，
        # 按候选纸几何估计距离执行开环前进，而不是放弃回 wait_trigger
        # （近距离 OCR 读不出字母导致稳定缓冲永远填不满的场景）
        self.declare_parameter("timeout_blind_forward",   True)
        self.declare_parameter("timeout_blind_min_target_frames", 5)
        self.declare_parameter("timeout_blind_max_m",     1.5)
        self.declare_parameter("cmd_vel_zero_timeout_s",  1.5)
        self.declare_parameter("move_timeout_s",          10.0)
        self.declare_parameter("show_debug_window",       True)

        # ── 读取参数 ────────────────────────────────────────────────────────── #
        self._trigger_topic      = self.get_parameter("trigger_topic").value
        self._place_topic        = self.get_parameter("place_topic").value
        self._cam_device         = self.get_parameter("camera_device").value
        self._img_w              = self.get_parameter("image_width").value
        self._img_h              = self.get_parameter("image_height").value
        self._fps                = self.get_parameter("fps").value
        raw_cm                   = self.get_parameter("camera_matrix").value
        raw_dc                   = self.get_parameter("dist_coeffs").value
        self._cam_mtx            = np.array(raw_cm, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs        = np.array(raw_dc, dtype=np.float64)
        self._paper_orient       = self.get_parameter("paper_orientation").value
        self._paper_aspect       = self.get_parameter("paper_aspect_ratio").value
        self._aspect_tol         = self.get_parameter("aspect_tolerance").value
        self._center_v_tol       = self.get_parameter("center_v_tolerance_px").value
        self._letter_offset_x    = self.get_parameter("letter_offset_x_m").value
        self._min_paper_area     = self.get_parameter("min_paper_area_px").value
        self._max_paper_area_rt  = self.get_parameter("max_paper_area_ratio").value
        self._detect_stale       = self.get_parameter("detect_stale_s").value
        self._lost_tolerance     = self.get_parameter("lost_tolerance_s").value
        self._h_px_median_win    = self.get_parameter("h_px_median_window").value
        self._wrong_letter_max   = self.get_parameter("wrong_letter_frames").value
        self._ocr_move_tol       = self.get_parameter("ocr_move_tol_px").value
        self._ocr_scale_tol      = self.get_parameter("ocr_scale_tol").value
        self._target_dist        = self.get_parameter("target_distance_m").value
        self._final_fwd_offset   = self.get_parameter("final_forward_offset_m").value
        self._extra_forward      = self.get_parameter("extra_forward_m").value
        self._vision_min_dist    = self.get_parameter("vision_min_distance_m").value
        self._yaw_thr_deg        = self.get_parameter("yaw_align_threshold_deg").value
        self._max_yaw_step_deg   = self.get_parameter("max_yaw_step_deg").value
        self._lat_thr            = self.get_parameter("lateral_threshold_m").value
        self._dist_thr           = self.get_parameter("distance_threshold_m").value
        self._max_rounds         = self.get_parameter("max_rounds").value
        self._stable_frames      = self.get_parameter("stable_frames").value
        self._detect_timeout     = self.get_parameter("detect_timeout_s").value
        self._timeout_blind      = self.get_parameter("timeout_blind_forward").value
        self._timeout_blind_min  = self.get_parameter("timeout_blind_min_target_frames").value
        self._timeout_blind_max  = self.get_parameter("timeout_blind_max_m").value
        self._cmdvel_zero_t      = self.get_parameter("cmd_vel_zero_timeout_s").value
        self._move_timeout       = self.get_parameter("move_timeout_s").value
        self._show_debug         = self.get_parameter("show_debug_window").value

        # 无图形界面（SSH 无 X 转发等）时 cv2.imshow 会触发 Qt 致命错误直接中止进程，
        # 必须提前关闭调试窗口，而不是等崩溃
        if self._show_debug and not os.environ.get("DISPLAY"):
            self.get_logger().warning(
                "未检测到 DISPLAY（无图形界面），调试窗口自动关闭。"
                "如需查看识别画面：用 ssh -X 登录、在运动主机桌面运行，"
                "或保持关闭只看日志（不影响对齐功能）")
            self._show_debug = False

        # ── 内参便捷提取 ────────────────────────────────────────────────────── #
        self._fx = float(self._cam_mtx[0, 0])
        self._fy = float(self._cam_mtx[1, 1])
        self._cx = float(self._cam_mtx[0, 2])
        self._cy = float(self._cam_mtx[1, 2])

        if self._paper_orient not in _PAPER_SIZE_M:
            self.get_logger().fatal(
                f"paper_orientation 非法: {self._paper_orient}（可选 portrait/landscape）")
            raise ValueError(f"invalid paper_orientation: {self._paper_orient}")
        self._paper_h_m = _PAPER_SIZE_M[self._paper_orient]

        if not PYTESSERACT_AVAILABLE:
            self.get_logger().warning(
                "pytesseract 不可用，OCR 将始终返回 None，节点无法锁定目标字母")

        # ── 摄像头 ──────────────────────────────────────────────────────────── #
        self._cap: Optional[cv2.VideoCapture] = None
        self._cap_dead = False
        # 采集线程状态：独立线程满速取流、只保留最新帧，检测快慢不再拖低帧率
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_seq = 0
        self._last_read_seq = 0
        self._cap_thread: Optional[threading.Thread] = None
        self._cap_stop = threading.Event()
        self._open_camera()

        # ── ROS 通信 ─────────────────────────────────────────────────────────── #
        # 触发话题双订阅：volatile 订阅兼容普通发布者；transient_local 订阅让
        #   ros2 topic pub --once --qos-durability transient_local --keep-alive 3
        # 的闩锁消息在 DDS 发现完成后仍能送达（Foxy 的 --once 不等待订阅匹配，
        # 直接发大概率丢消息）。注意：不能单独把订阅改成 transient_local——
        # 请求方 QoS 高于发布方（volatile）时 DDS 判定不兼容，反而全收不到。
        _qos_trigger_volatile = QoSProfile(depth=10)
        _qos_trigger_latched  = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub_trigger = self.create_subscription(
            String, self._trigger_topic, self._trigger_cb, _qos_trigger_volatile)
        self._sub_trigger_latched = self.create_subscription(
            String, self._trigger_topic, self._trigger_cb, _qos_trigger_latched)
        self._sub_cmdvel  = self.create_subscription(
            Twist, "/cmd_vel", self._cmdvel_cb, 10)
        self._sub_odom    = self.create_subscription(
            Odometry, "/leg_odom2", self._odom_cb, 10)

        self._pub_move    = self.create_publisher(Pose2D, "/move",                 10)
        self._pub_cmd     = self.create_publisher(String, "/pose_control/command", 10)
        self._pub_place   = self.create_publisher(String, self._place_topic,       10)

        # ── 状态 ─────────────────────────────────────────────────────────────── #
        self._state         = STATE_WAIT_TRIGGER
        self._target_letter: Optional[str] = None
        self._stable_buf    = deque(maxlen=self._stable_frames)
        self._lock          = threading.Lock()

        # 位姿缓存（短暂丢失容忍）
        self._last_valid_pose: Optional[dict] = None
        self._last_valid_time = 0.0
        self._h_px_buf      = deque(maxlen=self._h_px_median_win)

        # OCR 异步缓存：轮廓不动不重认，几何逐帧以最新轮廓为准（设计 §3.3）
        self._ocr_lock      = threading.Lock()
        self._ocr_entries: List[dict] = []
        self._ocr_queue: "queue.Queue" = queue.Queue(maxsize=1)
        self._ocr_stop      = threading.Event()
        self._ocr_thread    = threading.Thread(target=self._ocr_worker, daemon=True)
        self._ocr_thread.start()

        # 对错纸箱防呆统计
        self._wrong_letter_count  = 0
        self._wrong_letter_warned = False

        # cmd_vel 近期记录（用于判断运动是否停止）
        self._cmdvel_history: deque = deque(maxlen=30)

        # 运动链路诊断
        self._last_cmdvel_time = 0.0
        self._cmdvel_received_count = 0
        self._cmdvel_motion_started = False
        self._last_move_time = 0.0
        self._move_ignored_warned = False

        # 里程计诊断
        self._last_legodom_time = 0.0
        self._legodom_received_count = 0

        # 节流心跳日志（key → 上次输出时间）
        self._hb_last = {}
        # 最近一次 detect_letter_papers 的过滤统计
        self._last_detect_stats = {}

        # phase_1 诊断统计
        self._reset_detect_stats()

        # 主循环定时器：10 Hz
        self._timer = self.create_timer(0.1, self._main_loop)
        self.get_logger().info(
            f"letter_place_align_node 已启动，等待触发信号: {self._trigger_topic}"
            " (String, A/B/C/D)")

    # ──────────────────────────── 摄像头 ────────────────────────────────────── #

    def _open_camera(self):
        self._close_camera()
        self.get_logger().info(f"打开摄像头: {self._cam_device}")
        cap = cv2.VideoCapture(self._cam_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"摄像头打开失败: {self._cam_device}")
            self._cap = None
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._img_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._img_h)
        cap.set(cv2.CAP_PROP_FPS,          self._fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 不积压旧帧（驱动不支持时自动忽略）
        ret, frame = cap.read()
        if not ret or frame is None:
            self.get_logger().error(f"摄像头打开成功但无法读帧: {self._cam_device}")
            cap.release()
            self._cap = None
            return
        self._cap = cap
        self._cap_dead = False
        with self._frame_lock:
            self._latest_frame = None
            self._frame_seq = 0
        self._last_read_seq = 0
        self._start_capture_thread()
        self.get_logger().info(f"摄像头已就绪: {self._cam_device}  帧大小={frame.shape}")

    # ── 采集线程：满速取流只留最新帧，与检测/状态机解耦 ─────────────────────── #

    def _start_capture_thread(self):
        self._stop_capture_thread()
        self._cap_stop.clear()
        self._cap_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="camera-capture")
        self._cap_thread.start()

    def _stop_capture_thread(self):
        if self._cap_thread is not None:
            self._cap_stop.set()
            self._cap_thread.join(timeout=2.0)
            self._cap_thread = None

    def _capture_loop(self):
        fail_count = 0
        while not self._cap_stop.is_set():
            cap = self._cap
            if cap is None:
                break
            try:
                ret, frame = cap.read()
            except Exception:
                ret, frame = False, None
            if ret and frame is not None:
                with self._frame_lock:
                    self._latest_frame = frame
                    self._frame_seq += 1
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > 100:    # 连续失败约 5s，判定相机掉线
                    self.get_logger().error("采集线程连续读帧失败，相机可能已掉线")
                    self._cap_dead = True
                    with self._frame_lock:
                        self._latest_frame = None
                    break
                time.sleep(0.05)

    def _close_camera(self):
        self._stop_capture_thread()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        with self._frame_lock:
            self._latest_frame = None
        self._cap_dead = False

    def _reset_detect_stats(self):
        """重置 phase_1 检测诊断统计。"""
        self._detect_frames = 0
        self._detect_any_paper_frames = 0
        self._detect_target_frames = 0
        self._last_candidate_count = 0
        self._last_candidate_chars: List[Optional[str]] = []
        self._last_paper_geo: Optional[dict] = None

    def _heartbeat(self, key: str, interval: float = 2.0) -> bool:
        """节流心跳：距上次同 key 输出超过 interval 秒才返回 True 并刷新时间。"""
        now = time.monotonic()
        last = self._hb_last.get(key, 0.0)
        if now - last >= interval:
            self._hb_last[key] = now
            return True
        return False

    @staticmethod
    def _decide_char(votes) -> Optional[str]:
        """OCR 多数决：最近若干次结果中某字母 >=2 票才采信，否则返回 None 继续重认。

        防止单次错认（如 B→C）被永久锁死；None 票（未识别）不计入。
        """
        valid = [v for v in votes if v is not None]
        if len(valid) < 2:
            return None
        top, cnt = Counter(valid).most_common(1)[0]
        return top if cnt >= 2 else None

    # ──────────────────────────── 回调 ──────────────────────────────────────── #

    def _trigger_cb(self, msg: String):
        letter = msg.data.strip().upper()
        with self._lock:
            if letter in VALID_LETTERS:
                if self._state in (STATE_WAIT_TRIGGER, STATE_ERROR):
                    self.get_logger().info(f"收到触发信号，目标字母={letter}，进入 wait_detect")
                    self._target_letter = letter
                    self._state = STATE_WAIT_DETECT
                    self._stable_buf.clear()
                    self._h_px_buf.clear()
                    self._last_valid_pose = None
                    self._last_valid_time = 0.0
                    self._wrong_letter_count = 0
                    self._wrong_letter_warned = False
                    with self._ocr_lock:
                        self._ocr_entries.clear()
                    self._reset_detect_stats()
                    self._detect_deadline = time.monotonic() + self._detect_timeout
                    self._cmdvel_motion_started = False
                    self._move_ignored_warned = False
                elif letter != self._target_letter:
                    self.get_logger().warning(
                        f"对齐进行中（目标={self._target_letter}），"
                        f"忽略新目标字母 {letter}")
            else:
                if self._state not in (STATE_WAIT_TRIGGER, STATE_DONE):
                    self.get_logger().info(
                        f"收到无效字母 '{msg.data}'，视为取消：停止运动，回到 wait_trigger")
                    self._send_move(0.0, 0.0, 0.0)
                    self._state = STATE_WAIT_TRIGGER
                    self._target_letter = None

    def _cmdvel_cb(self, msg: Twist):
        speed = (abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z))
        now = time.monotonic()
        self._cmdvel_history.append((now, speed))
        self._last_cmdvel_time = now
        self._cmdvel_received_count += 1
        if speed >= 0.01 and not self._cmdvel_motion_started:
            self._cmdvel_motion_started = True

    def _odom_cb(self, msg: Odometry):
        self._last_legodom_time = time.monotonic()
        self._legodom_received_count += 1

    # ──────────────────────────── OCR 异步（设计 §3.3） ──────────────────────── #

    def _ocr_worker(self):
        """OCR 工作线程：慢认身份，不阻塞主循环。"""
        while not self._ocr_stop.is_set():
            try:
                roi, entry = self._ocr_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            char = _ocr_letter_roi(roi)
            with self._ocr_lock:
                entry["votes"].append(char)
                entry["pending"] = False
                entry["char"] = self._decide_char(entry["votes"])

    def _fill_ocr_chars(self, frame: np.ndarray, candidates: List[dict]):
        """给每个候选轮廓匹配 OCR 缓存；新轮廓/未识别轮廓投递 OCR 任务。"""
        now = time.monotonic()
        with self._ocr_lock:
            # 清理超期未见的缓存条目
            self._ocr_entries = [
                e for e in self._ocr_entries
                if now - e["last_seen"] <= self._detect_stale
            ]
            for cand in candidates:
                u, v, h = cand["u"], cand["v"], cand["h_px"]
                entry = None
                for e in self._ocr_entries:
                    if (abs(u - e["u"]) <= self._ocr_move_tol
                            and abs(v - e["v"]) <= self._ocr_move_tol
                            and e["h_px"] > 0
                            and abs(h - e["h_px"]) / e["h_px"] <= self._ocr_scale_tol):
                        entry = e
                        break
                if entry is None:
                    entry = {"u": u, "v": v, "h_px": h,
                             "char": None, "pending": False, "last_seen": now,
                             "votes": deque(maxlen=5)}
                    self._ocr_entries.append(entry)
                else:
                    # 跟踪缓慢移动：位置/尺寸以最新帧为准
                    entry["u"], entry["v"], entry["h_px"] = u, v, h
                    entry["last_seen"] = now
                cand["char"] = entry["char"]

                if entry["char"] is None and not entry["pending"]:
                    roi = extract_paper_roi(frame, cand["corners"])
                    if roi is not None:
                        try:
                            self._ocr_queue.put_nowait((roi.copy(), entry))
                            entry["pending"] = True
                        except queue.Full:
                            pass

    # ──────────────────────────── 检测 ──────────────────────────────────────── #

    def _read_frame(self) -> Optional[np.ndarray]:
        """取采集线程的最新帧；等待语义与原 cap.read() 一致（最多等 1s）。
        只返回未消费过的新帧，避免稳定判定被重复旧帧污染。"""
        if self._cap is None or self._cap_dead:
            return None
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with self._frame_lock:
                if (self._latest_frame is not None
                        and self._frame_seq != self._last_read_seq):
                    self._last_read_seq = self._frame_seq
                    return self._latest_frame.copy()
            if self._cap_dead or self._cap is None:
                return None
            time.sleep(0.01)
        return None

    def _detect_letter(self, frame: np.ndarray) -> Optional[dict]:
        """检测目标字母纸并推算位姿，返回与 apriltag 版 _detect_tag 同构的 dict。

        几何（u/v/h_px）逐帧以最新轮廓为准；OCR 身份由异步缓存填充。
        短暂丢失（< lost_tolerance_s）沿用最近一次有效位姿。
        返回 dict 的 raw['fresh'] 标记本帧是否真实检测到目标。
        """
        candidates, self._last_detect_stats = detect_letter_papers(
            frame,
            orientation=self._paper_orient,
            aspect_ratio=self._paper_aspect,
            aspect_tol=self._aspect_tol,
            center_v=self._cy,
            center_v_tol_px=self._center_v_tol,
            min_area_px=self._min_paper_area,
            max_area_ratio=self._max_paper_area_rt,
            ocr=False,
            return_stats=True,
        )
        self._fill_ocr_chars(frame, candidates)
        self._last_candidate_count = len(candidates)
        self._last_candidate_chars = [c["char"] for c in candidates]
        if candidates:
            # 候选纸几何（不依赖 OCR），供 phase_1 超时补救估计距离
            best = max(candidates, key=lambda c: c["h_px"])
            self._last_paper_geo = {"u": best["u"], "h_px": best["h_px"],
                                    "t": time.monotonic()}

        # 对错纸箱防呆：其他纸稳定识别为别的字母 → 明确告警（设计 §3.4）
        if self._target_letter is not None:
            others = [c["char"] for c in candidates
                      if c["char"] is not None and c["char"] != self._target_letter]
            if others:
                self._wrong_letter_count += 1
                if (self._wrong_letter_count >= self._wrong_letter_max
                        and not self._wrong_letter_warned):
                    self.get_logger().error(
                        f"连续 {self._wrong_letter_count} 帧识别到字母 {others}，"
                        f"与目标 {self._target_letter} 不符——可能对错纸箱，"
                        "请人工确认放置区！")
                    self._wrong_letter_warned = True
            else:
                self._wrong_letter_count = 0

        # 目标选取：滤出目标字母，多个取 h_px 最大者（最近最可信）
        matches = [c for c in candidates
                   if c["char"] is not None and c["char"] == self._target_letter]

        pose = None
        fresh = False
        now = time.monotonic()
        if matches:
            target = max(matches, key=lambda c: c["h_px"])
            self._h_px_buf.append(target["h_px"])
            h_px_med = float(np.median(self._h_px_buf))
            if h_px_med > 0:
                tz = self._fy * self._paper_h_m / h_px_med
                tx = tz * (target["u"] - self._cx) / self._fx + self._letter_offset_x
                pose = {"tx": float(tx), "ty": 0.0, "tz": float(tz), "R": None,
                        "raw": {"u": target["u"], "v": target["v"],
                                "h_px": target["h_px"], "h_px_med": h_px_med,
                                "aspect": target["aspect"],
                                "char": target["char"]}}
                self._last_valid_pose = pose
                self._last_valid_time = now
                fresh = True
        elif (self._last_valid_pose is not None
                and now - self._last_valid_time <= self._lost_tolerance):
            # 短暂丢失（OCR 抖动/轮廓瞬时丢失）：沿用最近一次有效位姿
            pose = self._last_valid_pose
        else:
            if self._last_valid_pose is not None:
                self.get_logger().warning(
                    f"目标丢失超过 {self._lost_tolerance:.1f}s，判定真丢失")
                self._last_valid_pose = None
                self._h_px_buf.clear()

        # 调试窗口
        if self._show_debug:
            vis = frame.copy()
            for c in candidates:
                pts = c["corners"].astype(int)
                if c["char"] == self._target_letter and c["char"] is not None:
                    color = (0, 255, 0)
                elif c["char"] is not None:
                    color = (0, 165, 255)
                else:
                    color = (160, 160, 160)
                cv2.polylines(vis, [pts], True, color, 2)
                label = c["char"] if c["char"] else "?"
                cv2.putText(vis, label, (int(c["u"]) - 10, int(c["v"])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            if pose is not None:
                tag = "" if fresh else " (缓存)"
                cv2.putText(vis,
                            f"tx={pose['tx']:.3f}m  tz={pose['tz']:.3f}m{tag}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(vis, f"state={self._state} target={self._target_letter}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("letter_place_align", vis)
            cv2.waitKey(1)

        if pose is not None:
            pose["raw"]["fresh"] = fresh
        return pose

    # ──────────────────────────── 运动指令 ──────────────────────────────────── #

    def _send_move(self, x: float, y: float, theta_deg: float):
        msg = Pose2D()
        msg.x     = float(x)
        msg.y     = float(y)
        msg.theta = float(theta_deg)
        self._pub_move.publish(msg)
        self._cmdvel_motion_started = False
        self._last_move_time = time.monotonic()
        self._move_ignored_warned = False
        if self._pub_move.get_subscription_count() == 0:
            self.get_logger().warning("/move 当前无订阅者，运动指令可能丢失")
        self.get_logger().info(f"发布 /move  x={x:.3f}  y={y:.3f}  theta={theta_deg:.1f}°")

    def _reset_origin(self):
        msg = String()
        msg.data = "reset_origin"
        self._pub_cmd.publish(msg)
        self.get_logger().info("发布 reset_origin")

    def _emit_place(self):
        """phase_7：发布 /grasp/place = String(target_letter)，通知 grasp 模块放置。"""
        msg = String()
        msg.data = self._target_letter
        self._pub_place.publish(msg)
        self.get_logger().info(f"发布 {self._place_topic} = '{self._target_letter}'")

    # ──────────────────────────── 运动完成判断 ──────────────────────────────── #

    def _is_cmd_vel_zero(self) -> bool:
        """判断最近 cmd_vel_zero_timeout_s 内速度是否持续接近零。
        必须已经先出现过非零速度，才认为运动真正完成。
        """
        now = time.monotonic()
        cutoff = now - self._cmdvel_zero_t
        recent = [(t, v) for (t, v) in self._cmdvel_history if t >= cutoff]
        if not recent:
            return False
        all_zero = all(v < 0.01 for (_, v) in recent)
        started = self._cmdvel_motion_started
        if not started and self._last_move_time > 0.0 and not self._move_ignored_warned:
            elapsed = now - self._last_move_time
            if elapsed > self._cmdvel_zero_t:
                self.get_logger().warning(
                    "运动指令发出后未检测到 /cmd_vel 非零，"
                    "可能被控制器忽略（检查 /leg_odom2 是否已发布）"
                )
                self._move_ignored_warned = True
        return started and all_zero

    def _wait_motion_done(self, timeout: Optional[float] = None) -> bool:
        """阻塞等待机器人运动完成（/cmd_vel 速度降为零）。"""
        if timeout is None:
            timeout = self._move_timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._is_cmd_vel_zero():
                return True
            time.sleep(0.05)
        if not self._cmdvel_motion_started:
            self.get_logger().warning(
                f"等待运动完成超时 ({timeout:.1f}s)：未检测到 /cmd_vel 非零，"
                "运动指令可能被控制器忽略（检查 /leg_odom2 和 pose_controller）"
            )
        else:
            self.get_logger().warning(f"等待运动完成超时 ({timeout:.1f}s)")
        return False

    def _check_motion_pipeline(self) -> Tuple[bool, List[str]]:
        """检查运动链路是否就绪：/move 有订阅者，/leg_odom2 和 /cmd_vel 近期有发布。"""
        issues = []
        if self._pub_move.get_subscription_count() == 0:
            issues.append("/move 无订阅者")
        now = time.monotonic()
        if now - self._last_legodom_time > _PIPELINE_TOPIC_TIMEOUT_S:
            issues.append(f"/leg_odom2 未收到或已超时（{now - self._last_legodom_time:.1f}s）")
        if now - self._last_cmdvel_time > _PIPELINE_TOPIC_TIMEOUT_S:
            issues.append(f"/cmd_vel 未收到或已超时（{now - self._last_cmdvel_time:.1f}s）")
        return (not issues), issues

    # ──────────────────────────── 稳定帧检测 ────────────────────────────────── #

    def _is_stable(self, pose: dict) -> bool:
        """把 pose 加入缓冲，若缓冲满且方差足够小则认为稳定。"""
        self._stable_buf.append(pose)
        if len(self._stable_buf) < self._stable_frames:
            return False
        tzs = [p["tz"] for p in self._stable_buf]
        txs = [p["tx"] for p in self._stable_buf]
        # tz 标准差 < 0.05 m，tx 标准差 < 0.05 m 认为稳定
        return (np.std(tzs) < 0.05) and (np.std(txs) < 0.05)

    # ──────────────────────────── 丢失回退 ──────────────────────────────────── #

    def _fallback_to_wait_detect(self, phase: str):
        """各对齐阶段目标真丢失时统一回 wait_detect。"""
        self.get_logger().warning(f"{phase}: 目标丢失，回到 wait_detect")
        with self._lock:
            self._stable_buf.clear()
            self._detect_deadline = time.monotonic() + self._detect_timeout
            self._state = STATE_WAIT_DETECT

    # ──────────────────────────── 主循环 ────────────────────────────────────── #

    def _main_loop(self):
        with self._lock:
            state = self._state

        if state == STATE_WAIT_TRIGGER:
            if self._heartbeat("wait_trigger", 3.0):
                self.get_logger().info(
                    f"等待触发信号: {self._trigger_topic} (String, A/B/C/D) — "
                    "触发命令: ros2 topic pub " + self._trigger_topic +
                    " std_msgs/String \"data: 'B'\" --once"
                    " --qos-durability transient_local --keep-alive 3")
            return

        if state == STATE_DONE:
            return

        if state == STATE_ERROR:
            return

        if state == STATE_BLIND_FORWARD:
            self._do_blind_forward()
            return

        # 以下阶段需要摄像头，先确保摄像头就绪（含采集线程判定的掉线）
        if self._cap is None or self._cap_dead:
            self.get_logger().warning("摄像头未就绪，尝试重新打开")
            self._open_camera()
            if self._cap is None:
                with self._lock:
                    self._state = STATE_ERROR
                return

        frame = self._read_frame()
        if frame is None:
            self.get_logger().warning("读帧失败，跳过本帧")
            return

        if state == STATE_WAIT_DETECT:
            self._do_wait_detect(frame)
        elif state == STATE_YAW_ALIGN:
            self._do_yaw_align(frame)
        elif state == STATE_LATERAL_ALIGN:
            self._do_lateral_align(frame)
        elif state == STATE_APPROACH:
            self._do_approach(frame)
        elif state == STATE_FINAL_CHECK:
            self._do_final_check(frame)

    # ──────────────────────────── phase_1 ───────────────────────────────────── #

    def _do_wait_detect(self, frame: np.ndarray):
        self._detect_frames += 1
        if time.monotonic() > self._detect_deadline:
            if self._try_timeout_blind_forward():
                return
            chars_str = (f" 最近一次候选字母={self._last_candidate_chars}"
                         if self._last_candidate_chars else "")
            self.get_logger().error(
                f"phase_1 超时，未检测到目标字母 {self._target_letter}。诊断："
                f"已处理 {self._detect_frames} 帧，"
                f"检测到候选纸的帧 {self._detect_any_paper_frames} 次{chars_str}，"
                f"识别为目标字母的帧 {self._detect_target_frames} 次，"
                f"稳定缓冲 {len(self._stable_buf)}/{self._stable_frames}，"
                f"回到 wait_trigger"
            )
            with self._lock:
                self._state = STATE_WAIT_TRIGGER
            return

        pose = self._detect_letter(frame)
        if self._last_candidate_count > 0:
            self._detect_any_paper_frames += 1

        if self._heartbeat("wait_detect", 2.0):
            s = self._last_detect_stats or {}
            self.get_logger().info(
                f"wait_detect: 候选纸 {self._last_candidate_count} 张 "
                f"OCR={self._last_candidate_chars} 目标={self._target_letter} "
                f"稳定缓冲 {len(self._stable_buf)}/{self._stable_frames} | "
                f"过滤统计: 轮廓{s.get('contours_total', 0)} "
                f"面积✗{s.get('rej_area', 0)} 四边形✗{s.get('rej_quad', 0)} "
                f"长宽比✗{s.get('rej_aspect', 0)} 齐平✗{s.get('rej_center_v', 0)}")

        if pose is None:
            return
        self._detect_target_frames += 1

        # 只用本帧真实检测到的位姿做稳定判定，避免缓存位姿造成假稳定
        if not pose["raw"].get("fresh", False):
            return

        if self._is_stable(pose):
            self.get_logger().info(
                f"字母 {self._target_letter} 稳定锁定: "
                f"tx={pose['tx']:.3f}  tz={pose['tz']:.3f}")
            with self._lock:
                self._last_pose  = pose
                self._yaw_rounds = 0
                self._lat_rounds = 0
                self._app_rounds = 0
                self._stable_buf.clear()
                self._state = STATE_YAW_ALIGN
                self._phase_busy = False

    def _try_timeout_blind_forward(self) -> bool:
        """phase_1 超时补救：近距离 OCR 读不出字母、稳定缓冲填不满，但本轮
        已确认过目标字母且候选纸此刻仍在画面中时，按候选纸几何估计距离，
        直接进 blind_forward 开环前进（与 final_check 路径同一公式），
        返回 True 表示已接管。不满足条件返回 False，走原回 wait_trigger 逻辑。
        """
        if not self._timeout_blind:
            return False
        if self._detect_target_frames < self._timeout_blind_min:
            return False            # 本轮从未确认过目标字母：可能对错纸箱/无纸，放弃
        if self._wrong_letter_warned:
            return False            # 曾稳定识别为其他字母：明确是对错纸箱，放弃
        geo = self._last_paper_geo
        if geo is None or time.monotonic() - geo["t"] > self._detect_stale:
            return False            # 此刻画面里没有候选纸，无距离依据，放弃
        if geo["h_px"] <= 0:
            return False

        tz = self._fy * self._paper_h_m / geo["h_px"]
        blind = tz - self._target_dist + self._final_fwd_offset + self._extra_forward
        if blind <= 0.0 or blind > self._timeout_blind_max:
            self.get_logger().warning(
                f"phase_1 超时补救放弃：估计 tz={tz:.3f}m → blind={blind:.3f}m "
                f"超出安全范围 (0, {self._timeout_blind_max:.2f}m]")
            return False

        self.get_logger().warning(
            f"phase_1 超时但目标 {self._target_letter} 曾确认 "
            f"{self._detect_target_frames} 帧、候选纸在画面中"
            f"（近距离 OCR 困难场景）：按 tz≈{tz:.3f}m 开环前进 {blind:.3f}m")
        self._blind_dist = blind
        self._blind_started = False
        with self._lock:
            self._stable_buf.clear()
            self._state = STATE_BLIND_FORWARD
        return True

    # ──────────────────────────── phase_2 ───────────────────────────────────── #

    def _do_yaw_align(self, frame: np.ndarray):
        if getattr(self, "_phase_busy", False):
            if not self._is_cmd_vel_zero():
                if self._heartbeat("motion_wait", 2.0):
                    self.get_logger().info("yaw_align: 等待旋转运动停止…")
                return
            # 运动停止，重新检测
            pose = self._detect_letter(frame)
            if pose is None:
                self._fallback_to_wait_detect("yaw_align")
                return
            self._last_pose = pose
            self._phase_busy = False

        pose  = self._last_pose
        tx, tz = pose["tx"], pose["tz"]
        if tz <= 0.0:
            self.get_logger().warning(f"yaw_align: tz={tz:.3f} 异常，跳过")
            return

        alpha_rad = math.atan2(tx, tz)
        alpha_deg = math.degrees(alpha_rad)

        if abs(alpha_deg) <= self._yaw_thr_deg:
            self.get_logger().info(f"yaw_align 完成: alpha={alpha_deg:.2f}°")
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_LATERAL_ALIGN
                self._phase_busy = False
            return

        if self._yaw_rounds >= self._max_rounds:
            self.get_logger().error(
                f"yaw_align 超过最大轮次，放弃。最终 alpha={alpha_deg:.2f}°，"
                f"tx={tx:.3f}m tz={tz:.3f}m。请检查机器人是否实际响应 /cmd_vel。"
            )
            with self._lock:
                self._state = STATE_ERROR
            return

        # 首次发送运动指令前检查运动链路
        if self._yaw_rounds == 0:
            ready, issues = self._check_motion_pipeline()
            if not ready:
                self.get_logger().error(
                    "运动链路未就绪，无法执行 yaw_align：" + "；".join(issues) +
                    "。请确认已启动 pose_controller 且 /leg_odom2 有数据。"
                )
                with self._lock:
                    self._state = STATE_ERROR
                return

        self._yaw_rounds += 1
        # 单次旋转不超过 max_yaw_step_deg，避免惯性冲过头
        theta_cmd = -max(min(alpha_deg, self._max_yaw_step_deg), -self._max_yaw_step_deg)
        self.get_logger().info(
            f"yaw_align 轮次 {self._yaw_rounds}: alpha={alpha_deg:.2f}°，"
            f"发送旋转指令 theta={theta_cmd:.2f}°")
        self._reset_origin()
        time.sleep(0.15)  # 等待 reset_origin 在 pose_controller 中先处理，避免 /move 被清空
        # ROS 约定：theta 逆时针为正，目标偏右(alpha>0)需要狗向右转(负)
        self._send_move(0.0, 0.0, theta_cmd)
        self._phase_busy = True

    # ──────────────────────────── phase_3 ───────────────────────────────────── #

    def _do_lateral_align(self, frame: np.ndarray):
        if getattr(self, "_phase_busy", False):
            if not self._is_cmd_vel_zero():
                if self._heartbeat("motion_wait", 2.0):
                    self.get_logger().info("lateral_align: 等待横移运动停止…")
                return
            pose = self._detect_letter(frame)
            if pose is None:
                self._fallback_to_wait_detect("lateral_align")
                return
            self._last_pose = pose
            self._phase_busy = False

        pose = self._last_pose
        tx   = pose["tx"]

        if abs(tx) <= self._lat_thr:
            self.get_logger().info(f"lateral_align 完成: tx={tx:.3f}m")
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_APPROACH
                self._phase_busy = False
            return

        if self._lat_rounds >= self._max_rounds:
            self.get_logger().error("lateral_align 超过最大轮次，放弃")
            with self._lock:
                self._state = STATE_ERROR
            return

        self._lat_rounds += 1
        self.get_logger().info(
            f"lateral_align 轮次 {self._lat_rounds}: tx={tx:.3f}m，发送横移指令")
        # /move y 正方向为左移；tx 相机坐标右正，故取负
        self._send_move(0.0, -tx, 0.0)
        self._phase_busy = True

    # ──────────────────────────── phase_4 ───────────────────────────────────── #

    def _do_approach(self, frame: np.ndarray):
        """视觉闭环逼近，直到 tz <= vision_min_distance_m（设计 §4.4）。

        A4 纸在 ~0.3m 以内会超出画面，视觉闭环有距离下限；
        剩余距离由 phase_6 开环盲进完成。
        """
        if getattr(self, "_phase_busy", False):
            if not self._is_cmd_vel_zero():
                if self._heartbeat("motion_wait", 2.0):
                    self.get_logger().info("approach: 等待前进运动停止…")
                return
            pose = self._detect_letter(frame)
            if pose is None:
                self._fallback_to_wait_detect("approach")
                return
            self._last_pose = pose
            self._phase_busy = False

        pose = self._last_pose
        tz   = pose["tz"]

        # 精确前进到 vision_min_distance_m
        delta = tz - self._vision_min_dist
        if abs(delta) <= self._dist_thr:
            self.get_logger().info(
                f"approach 完成: tz={tz:.3f}m（视觉闭环下限 {self._vision_min_dist:.2f}m）")
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_FINAL_CHECK
                self._phase_busy = False
            return

        if self._app_rounds >= self._max_rounds:
            self.get_logger().error("approach 超过最大轮次，放弃")
            with self._lock:
                self._state = STATE_ERROR
            return

        self._app_rounds += 1
        self.get_logger().info(
            f"approach 轮次 {self._app_rounds}: tz={tz:.3f}m  delta={delta:.3f}m")
        self._send_move(delta, 0.0, 0.0)
        self._phase_busy = True

    # ──────────────────────────── phase_5 ───────────────────────────────────── #

    def _do_final_check(self, frame: np.ndarray):
        pose = self._detect_letter(frame)
        if pose is None:
            self._fallback_to_wait_detect("final_check")
            return

        tx, tz = pose["tx"], pose["tz"]
        alpha_deg = math.degrees(math.atan2(tx, tz)) if tz > 0 else 999.0

        yaw_ok  = abs(alpha_deg) <= self._yaw_thr_deg
        lat_ok  = abs(tx)        <= self._lat_thr
        dist_ok = abs(tz - self._vision_min_dist) <= self._dist_thr

        if yaw_ok and lat_ok and dist_ok:
            self._stable_buf.append({"tx": tx, "tz": tz})
        else:
            self._stable_buf.clear()
            if self._heartbeat("final_check", 2.0):
                self.get_logger().info(
                    f"final_check 未全达标: yaw={yaw_ok}({alpha_deg:.1f}°) "
                    f"lat={lat_ok}({tx:.3f}m) "
                    f"dist={dist_ok}({tz - self._vision_min_dist:+.3f}m)，回 yaw_align 修正")
            # 任何一项不达标都回到 yaw_align 重新修正
            with self._lock:
                self._yaw_rounds = 0
                self._lat_rounds = 0
                self._app_rounds = 0
                self._last_pose  = pose
                self._state      = STATE_YAW_ALIGN
                self._phase_busy = False
            return

        if len(self._stable_buf) >= self._stable_frames:
            pose = self._stable_buf[-1]
            tx, tz = pose["tx"], pose["tz"]
            alpha_deg = math.degrees(math.atan2(tx, tz)) if tz > 0 else 999.0
            # 盲进距离：由构造保证最终站位与抓取前 AprilTag 站位一致（设计 §4.4），
            # 再叠加仿 apriltag_place1 的 extra_forward_m 额外开环前进
            blind = tz - self._target_dist + self._final_fwd_offset + self._extra_forward
            if blind <= 0.0:
                self.get_logger().error(
                    f"final_check 通过但盲进距离异常: tz={tz:.3f}m → blind={blind:.3f}m，"
                    "请检查站位参数配置")
                self._stable_buf.clear()
                with self._lock:
                    self._state = STATE_ERROR
                return
            self.get_logger().info(
                f"final_check 通过！tx={tx:.3f}m  tz={tz:.3f}m  alpha={alpha_deg:.2f}°，"
                f"准备开环盲进 {blind:.3f}m")
            self._stable_buf.clear()
            self._blind_dist = blind
            self._blind_started = False
            with self._lock:
                self._state = STATE_BLIND_FORWARD
            return

    # ──────────────────────────── phase_6 ───────────────────────────────────── #

    def _do_blind_forward(self):
        """final_check 通过后，开环直线前进 blind_dist，再发放置信号（设计 §4.4）。

        blind = tz_measured - target_distance_m + final_forward_offset_m + extra_forward_m
        其中 tz_measured 为 final_check 通过时的实测距离；
        extra_forward_m 仿 apriltag_place1 的 final_forward，最后额外加长一段。
        """
        if not getattr(self, "_blind_started", False):
            ready, issues = self._check_motion_pipeline()
            if not ready:
                self.get_logger().error(
                    "blind_forward: 运动链路未就绪：" + "；".join(issues))
                with self._lock:
                    self._state = STATE_ERROR
                return

            self.get_logger().info(
                f"blind_forward: 重置原点并开环前进 {self._blind_dist:.3f}m")
            self._reset_origin()
            time.sleep(0.15)
            self._send_move(self._blind_dist, 0.0, 0.0)
            self._blind_started = True
            return

        if not self._is_cmd_vel_zero():
            if self._heartbeat("motion_wait", 2.0):
                self.get_logger().info("blind_forward: 等待盲进运动停止…")
            return

        self.get_logger().info(
            f"blind_forward 完成，前进 {self._blind_dist:.3f}m，发布放置信号")
        self._emit_place()
        with self._lock:
            self._state = STATE_DONE
        self._blind_started = False

    # ──────────────────────────── 析构 ──────────────────────────────────────── #

    def destroy_node(self):
        self._ocr_stop.set()
        if self._ocr_thread.is_alive():
            self._ocr_thread.join(timeout=1.0)
        self._close_camera()
        if self._show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = LetterPlaceAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
