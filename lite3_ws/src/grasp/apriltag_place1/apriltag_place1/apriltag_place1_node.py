#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag place1 对齐节点（2D 主策略：横向 → 前后 → yaw 收尾微调）

流程（2026-08-11 重构）：
  phase_0_wait_trigger  等待外部触发 /apriltag_place1/start (Bool)
  phase_1_wait_detect   多帧稳定检测目标 Tag，取中位数位姿作为对齐起点（狗不动）
  phase_2_lateral_align 横向平移，使 Tag 正对摄像头（|tx| ≤ lateral_threshold_m）
  phase_3_approach      前进到 closed_loop_end_dist（默认 1.4m，视觉闭环）
  phase_4_yaw_finetune  最多 max_yaw_finetune_rounds 次（默认 3）yaw 微调；
                        达标或用尽次数都直接完成——yaw 残余偏差不视为失败
  phase_5_emit_signal   发布 /grasp/start (Bool) 与 /apriltag_place1/done (Bool)

设计要点：
  - 策略参考 block_align_node：2D 主对齐（横向 + 前后），最后再做 yaw 收尾
  - yaw_finetune 用尽次数即使有残余偏差也 emit 完成，不打断上层 abcd_task 流程
  - 去掉了旧的 final_check（三轴同时达标）与 blind_forward（开环 0.47m）阶段，
    合并到 approach 一步走到 1.4m
  - 检测层仍用 pupil_apriltags 解算 6DoF 位姿（tx/ty/tz 来自 pose_t）
  - 短暂丢失（<lost_tolerance_s）沿用最近位姿，超时后视为真丢失

依赖：
  - pupil-apriltags  (pip3 install pupil-apriltags)
  - opencv-python
  - rclpy, geometry_msgs, nav_msgs, std_msgs
"""

import math
import os
import time
import threading
from collections import deque
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


# TRANSIENT_LOCAL：晚订阅的节点也能收到最后一条 done 信号，避免 abcd_task
# 状态机切到 WAIT_APRILTAG_DONE 时错过瞬时消息。
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


# ─── 状态常量 ───────────────────────────────────────────────────────────────── #
# 2026-08-11 重构：改用 block_align 风格的 2D 策略——
#   先 lateral_align（横向），再 approach（前后到 1.4m），最后 yaw_finetune（3 次内）。
# 去掉了原有的 final_check（三轴同时达标）与 blind_forward（开环盲进 0.47m）阶段。
# yaw 微调即使未达标也不视为失败，直接 emit 抓取信号（对上层是 tag_align 完成）。
STATE_WAIT_TRIGGER   = "wait_trigger"
STATE_WAIT_DETECT    = "wait_detect"
STATE_SEARCH         = "search"           # 横向搜索兜底
STATE_LATERAL_ALIGN  = "lateral_align"
STATE_APPROACH       = "approach"
STATE_YAW_FINETUNE   = "yaw_finetune"    # 原 yaw_align 语义变化：改为 approach 之后的收尾修正
STATE_DONE           = "done"
STATE_ERROR          = "error"

_PIPELINE_TOPIC_TIMEOUT_S = 0.5  # 运动链路话题超时阈值


class AprilTagPlace1Node(Node):

    def __init__(self):
        super().__init__("apriltag_place1_node")

        # ── 参数声明 ────────────────────────────────────────────────────────── #
        self.declare_parameter("trigger_topic",           "/apriltag_place1/start")
        self.declare_parameter("camera_device",           "/dev/video6")
        self.declare_parameter("show_debug_window",       True)
        self.declare_parameter("image_width",             640)
        self.declare_parameter("image_height",            480)
        self.declare_parameter("fps",                     30)
        self.declare_parameter("camera_matrix",
            [388.1454, 0.0, 329.4121, 0.0, 387.7497, 223.481, 0.0, 0.0, 1.0])
        self.declare_parameter("dist_coeffs",
            [-0.1571, -0.218, -0.0024, -0.0011, 0.2089])
        self.declare_parameter("tag_family",              "tag25h9")
        self.declare_parameter("target_tag_id",           0)
        self.declare_parameter("tag_size_m",              0.083)
        # closed_loop_end_dist：approach 完成后距离 Tag 的目标 tz（米）。
        # 2026-08-11 重构：从旧的 0.35m + blind_forward 0.47m 合并为一步走到 1.4m。
        # 1.4m 处 tag_size=0.083 仍能稳定识别，且对上层 abcd_task 后续 block_align
        # 的机械臂视野接近距离更合适。
        self.declare_parameter("closed_loop_end_dist",    1.4)
        # final_forward_offset_m 已废弃（不再有 blind_forward），保留声明以便向后
        # 兼容旧的 yaml 文件——载入后不再使用。
        self.declare_parameter("final_forward_offset_m",  0.0)
        self.declare_parameter("yaw_align_threshold_deg", 3.0)
        self.declare_parameter("max_yaw_step_deg",        3.0)
        self.declare_parameter("lateral_threshold_m",     0.03)
        self.declare_parameter("distance_threshold_m",    0.03)
        self.declare_parameter("max_rounds",              10)
        # yaw 微调最大次数：达到即使仍有角度偏差也直接 emit + done（不视为失败）。
        # 独立于 max_rounds（后者管 lateral/approach）。
        self.declare_parameter("max_yaw_finetune_rounds", 3)
        self.declare_parameter("stable_frames",           15)
        self.declare_parameter("lost_tolerance_s",        2.0)
        self.declare_parameter("detect_timeout_s",        10.0)
        self.declare_parameter("cmd_vel_zero_timeout_s",  1.5)
        self.declare_parameter("move_timeout_s",          10.0)
        # ── pose_controller 健康检查（2026-08-14）──
        self.declare_parameter("motion_watchdog_timeout_s", 3.0)
        self.declare_parameter("motion_reset_recovery_s",   1.0)
        # 搜索参数（检测超时后横向搜索兜底）
        self.declare_parameter("search_step_m",           0.10)   # 每步 10cm
        self.declare_parameter("search_max_steps",        5)      # 最多 5 步（±50cm）
        self.declare_parameter("search_timeout_s",        30.0)   # 搜索总超时

        # ── 读取参数 ────────────────────────────────────────────────────────── #
        self._trigger_topic      = self.get_parameter("trigger_topic").value
        self._cam_device         = self.get_parameter("camera_device").value
        self._img_w              = self.get_parameter("image_width").value
        self._img_h              = self.get_parameter("image_height").value
        self._fps                = self.get_parameter("fps").value
        self._show_debug         = self.get_parameter("show_debug_window").value
        if self._show_debug and not os.environ.get("DISPLAY"):
            self.get_logger().warning("无显示环境（DISPLAY 未设置），调试窗口已禁用")
            self._show_debug = False
        raw_cm                   = self.get_parameter("camera_matrix").value
        raw_dc                   = self.get_parameter("dist_coeffs").value
        self._cam_mtx            = np.array(raw_cm, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs        = np.array(raw_dc, dtype=np.float64)
        self._tag_family         = self.get_parameter("tag_family").value
        self._target_tag_id      = self.get_parameter("target_tag_id").value
        self._tag_size_m         = self.get_parameter("tag_size_m").value
        self._closed_loop_end    = self.get_parameter("closed_loop_end_dist").value
        self._final_fwd_offset   = self.get_parameter("final_forward_offset_m").value  # 已废弃，保留占位
        self._yaw_thr_deg        = self.get_parameter("yaw_align_threshold_deg").value
        self._max_yaw_step_deg   = self.get_parameter("max_yaw_step_deg").value
        self._lat_thr            = self.get_parameter("lateral_threshold_m").value
        self._dist_thr           = self.get_parameter("distance_threshold_m").value
        self._max_rounds         = self.get_parameter("max_rounds").value
        self._max_yaw_finetune   = int(self.get_parameter("max_yaw_finetune_rounds").value)
        self._stable_frames      = self.get_parameter("stable_frames").value
        self._lost_tolerance     = self.get_parameter("lost_tolerance_s").value
        self._detect_timeout     = self.get_parameter("detect_timeout_s").value
        self._cmdvel_zero_t      = self.get_parameter("cmd_vel_zero_timeout_s").value
        self._move_timeout       = self.get_parameter("move_timeout_s").value
        self._watchdog_timeout   = float(self.get_parameter("motion_watchdog_timeout_s").value)
        self._reset_recovery     = float(self.get_parameter("motion_reset_recovery_s").value)
        self._search_step_m      = self.get_parameter("search_step_m").value
        self._search_max_steps   = int(self.get_parameter("search_max_steps").value)
        self._search_timeout_s   = self.get_parameter("search_timeout_s").value

        # ── 内参便捷提取 ────────────────────────────────────────────────────── #
        self._fx = float(self._cam_mtx[0, 0])
        self._fy = float(self._cam_mtx[1, 1])
        self._cx = float(self._cam_mtx[0, 2])
        self._cy = float(self._cam_mtx[1, 2])

        # ── AprilTag 检测器 ──────────────────────────────────────────────────── #
        try:
            from pupil_apriltags import Detector
            # 优化检测速度：quad_decimate 从 1.0 提高到 2.0
            # 在 640x480 下可以将检测速度提升约 4 倍（200ms → 50ms）
            # 对于 0.083m Tag 在 1-2m 距离仍能稳定检测
            self._detector = Detector(
                families=self._tag_family,
                nthreads=4,
                quad_decimate=2.0,  # 提高到 2.0 加速检测
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
            self.get_logger().info(f"pupil_apriltags Detector 初始化成功，family={self._tag_family}, quad_decimate=2.0（优化速度）")
        except ImportError:
            self.get_logger().fatal("未找到 pupil_apriltags，请 pip3 install pupil-apriltags")
            raise

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
        self._sub_trigger = self.create_subscription(
            Bool, self._trigger_topic, self._trigger_cb, 10)
        self._sub_cmdvel  = self.create_subscription(
            Twist, "/cmd_vel", self._cmdvel_cb, 10)
        self._sub_odom    = self.create_subscription(
            Odometry, "/leg_odom2", self._odom_cb, 10)

        self._pub_move    = self.create_publisher(Pose2D,  "/move",                 10)
        self._pub_cmd     = self.create_publisher(String,  "/pose_control/command", 10)
        self._pub_grasp   = self.create_publisher(Bool,    "/grasp/start",          10)
        # /apriltag_place1/done：Tag 对齐完成信号（专用），供上层编排（abcd_task）
        # 与 /grasp/start（供 grasp_task 触发抓取，与 block_align 共享）区分。
        # TRANSIENT_LOCAL 让晚订阅也能拿到最后一条 True。
        self._pub_done    = self.create_publisher(Bool,    "/apriltag_place1/done", _LATCHED_QOS)

        # ── 状态 ─────────────────────────────────────────────────────────────── #
        self._state         = STATE_WAIT_TRIGGER
        self._stable_buf    = deque(maxlen=self._stable_frames)
        self._lock          = threading.Lock()

        # 分相闭环状态
        self._last_pose       = None     # 各相位姿（wait_detect 中位数 / 停稳后重检测）
        self._yaw_rounds      = 0        # yaw_finetune 已用次数
        self._lat_rounds      = 0
        self._app_rounds      = 0
        self._phase_busy      = False
        self._phase_start_time = 0.0     # 运动开始时间戳（用于超时检测）

        # 位姿缓存（短暂丢失容忍）
        self._last_valid_pose = None
        self._last_valid_time = 0.0

        # cmd_vel 近期记录（用于判断运动是否停止）
        self._cmdvel_history: deque = deque(maxlen=30)

        # 运动链路诊断
        self._last_cmdvel_time = time.monotonic()  # 初始化为当前时间，避免启动时误报超时
        self._cmdvel_received_count = 0
        self._cmdvel_motion_started = False
        self._last_move_time = 0.0
        self._move_ignored_warned = False

        # 里程计诊断
        self._last_legodom_time = time.monotonic()  # 初始化为当前时间，避免启动时误报超时
        self._legodom_received_count = 0

        # 看门狗：记录上次成功收到运动链路数据的时间
        self._last_pipeline_healthy_time = time.monotonic()
        self._pipeline_reset_count       = 0

        # 节流心跳日志（key → 上次输出时间）
        self._hb_last = {}

        # phase_1 诊断统计
        self._reset_detect_stats()

        # 搜索状态
        self._search_steps_done = 0
        self._search_deadline = float("inf")
        self._settle_tracker_ready = False  # 搜索后停下重建 tracker 的标志

        # 主循环定时器：10 Hz
        self._timer = self.create_timer(0.1, self._main_loop)
        self.get_logger().info(f"apriltag_place1_node 已启动，等待触发信号: {self._trigger_topic}")

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
        self._detect_any_tags_frames = 0
        self._detect_target_frames = 0
        self._last_detected_tag_count = 0
        self._last_detected_tag_ids: List[int] = []

    def _heartbeat(self, key: str, interval: float = 2.0) -> bool:
        """节流心跳：距上次同 key 输出超过 interval 秒才返回 True 并刷新时间。"""
        now = time.monotonic()
        last = self._hb_last.get(key, 0.0)
        if now - last >= interval:
            self._hb_last[key] = now
            return True
        return False

    # ──────────────────────────── 回调 ──────────────────────────────────────── #

    def _trigger_cb(self, msg: Bool):
        with self._lock:
            if msg.data:
                if self._state in (STATE_WAIT_TRIGGER, STATE_ERROR):
                    # 每次触发前重读 target_tag_id，支持运行时热更新
                    # （abcd_task 通过 rcl_interfaces/SetParameters 修改此参数）
                    try:
                        new_tag_id = int(self.get_parameter("target_tag_id").value)
                    except Exception as e:
                        self.get_logger().warning(
                            f"读取 target_tag_id 失败，沿用旧值 {self._target_tag_id}: {e}"
                        )
                        new_tag_id = self._target_tag_id
                    if new_tag_id != self._target_tag_id:
                        self.get_logger().info(
                            f"target_tag_id 切换: {self._target_tag_id} -> {new_tag_id}"
                        )
                        self._target_tag_id = new_tag_id

                    self.get_logger().info(
                        f"收到触发信号，进入 wait_detect，target_tag_id={self._target_tag_id}"
                    )
                    now = time.monotonic()
                    self._state = STATE_WAIT_DETECT
                    self._stable_buf.clear()
                    self._reset_detect_stats()
                    self._detect_deadline = now + self._detect_timeout
                    self._cmdvel_motion_started = False
                    self._move_ignored_warned = False
                    # 重置运动链路时间戳，避免刚启动时误报超时
                    self._last_cmdvel_time = now
                    self._last_legodom_time = now
                    self._last_pose = None
                    self._yaw_rounds = 0
                    self._lat_rounds = 0
                    self._app_rounds = 0
                    self._phase_busy = False
                    self._last_valid_pose = None
                    self._last_valid_time = 0.0
            else:
                if self._state not in (STATE_WAIT_TRIGGER, STATE_DONE):
                    self.get_logger().info("收到取消信号，停止运动，回到 wait_trigger")
                    self._send_move(0.0, 0.0, 0.0)
                    self._state = STATE_WAIT_TRIGGER

    def _cmdvel_cb(self, msg: Twist):
        speed = (abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z))
        now = time.monotonic()
        self._cmdvel_history.append((now, speed))
        self._last_cmdvel_time = now
        self._cmdvel_received_count += 1
        if speed >= 0.01 and not self._cmdvel_motion_started:
            self._cmdvel_motion_started = True
        # 更新运动链路健康时间戳
        self._last_pipeline_healthy_time = now

    def _odom_cb(self, msg: Odometry):
        self._last_legodom_time = time.monotonic()
        self._legodom_received_count += 1
        # 更新运动链路健康时间戳
        self._last_pipeline_healthy_time = time.monotonic()

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

    def _detect_tag(self, frame: np.ndarray) -> Optional[dict]:
        """检测目标 Tag 并返回位姿 dict {tx, ty, tz, R, raw}，未锁定返回 None。

        短暂丢失（< lost_tolerance_s）沿用最近一次有效位姿，raw['fresh']=False；
        超期判定真丢失返回 None，由各阶段统一回 wait_detect。
        """
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self._detector.detect(
            grey,
            estimate_tag_pose=True,
            camera_params=[self._fx, self._fy, self._cx, self._cy],
            tag_size=self._tag_size_m,
        )
        self._last_detected_tag_count = len(tags)
        self._last_detected_tag_ids = [t.tag_id for t in tags]
        if tags:
            self.get_logger().info(f"检测到 {len(tags)} 个 Tag，IDs={self._last_detected_tag_ids}")

        pose = None
        fresh = False
        now = time.monotonic()
        for tag in tags:
            if tag.tag_id == self._target_tag_id:
                t = tag.pose_t.flatten()
                pose = {"tx": float(t[0]), "ty": float(t[1]), "tz": float(t[2]),
                        "R": tag.pose_R, "raw": {}}
                self._last_valid_pose = pose
                self._last_valid_time = now
                fresh = True
                break
        if pose is None:
            if (self._last_valid_pose is not None
                    and now - self._last_valid_time <= self._lost_tolerance):
                # 短暂丢失：沿用最近一次有效位姿
                pose = self._last_valid_pose
            else:
                if self._last_valid_pose is not None:
                    self.get_logger().warning(
                        f"目标丢失超过 {self._lost_tolerance:.1f}s，判定真丢失")
                    self._last_valid_pose = None

        # 调试窗口：显示检测结果
        if self._show_debug:
            vis = frame.copy()
            for tag in tags:
                pts = tag.corners.astype(int)
                cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
                cx = int(pts[:, 0].mean())
                cy = int(pts[:, 1].mean())
                color = (0, 255, 0) if tag.tag_id == self._target_tag_id else (0, 165, 255)
                cv2.putText(vis, f"id={tag.tag_id}", (cx - 20, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if pose is not None:
                tag = "" if fresh else " (缓存)"
                cv2.putText(vis,
                            f"tx={pose['tx']:.3f}m  tz={pose['tz']:.3f}m{tag}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(vis, f"state={self._state}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("apriltag_place1", vis)
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

    def _reset_pose_controller(self):
        """发送 cancel 命令重置 pose_controller，清空其内部运动队列和状态。"""
        msg = String()
        msg.data = "cancel"
        self._pub_cmd.publish(msg)
        self.get_logger().warning("发送 cancel 命令重置 pose_controller")
        self._pipeline_reset_count += 1
        # 等待恢复
        time.sleep(self._reset_recovery)
        # 重置运动状态标志
        self._cmdvel_motion_started = False
        self._move_ignored_warned = False
        self._last_pipeline_healthy_time = time.monotonic()

    def _check_motion_watchdog(self) -> bool:
        """运动链路看门狗：如果正在等待运动完成，且链路数据超时未更新，返回 True。

        Returns:
            True: 链路异常，需要重置
            False: 链路正常
        """
        # 只在 phase_busy（正在等待运动完成）时才检查
        if not self._phase_busy:
            return False

        now = time.monotonic()
        elapsed = now - self._last_pipeline_healthy_time

        if elapsed > self._watchdog_timeout:
            self.get_logger().error(
                f"运动链路看门狗触发：{elapsed:.1f}s 未收到 /cmd_vel 或 /leg_odom2 更新，"
                f"pose_controller 可能异常（已重置 {self._pipeline_reset_count} 次）")
            return True

        return False

    def _emit_place1(self):
        """phase_7：发布对齐完成信号。

        修复 2026-08-12：只发 /apriltag_place1/done，不再发 /grasp/start。
        新流程：apriltag → abcd_task 拉起 block_align → block_align 完成后发 /grasp/start。

        旧问题：apriltag_place1 和 block_align 同时发 /grasp/start，导致：
          1. grasp_node 抢先被触发，打开摄像头开始检测
          2. block_align_node 无法获取摄像头（被占用）
          3. block_align 的横向搜索功能无法执行

        新方案：apriltag_place1 只负责 AprilTag 对齐，完成后通知 abcd_task；
        abcd_task 拉起 block_align 进行色块对齐+搜索，block_align 完成后触发 grasp_task。
        """
        msg = Bool()
        msg.data = True
        # 不再发布 /grasp/start，由 block_align 接管触发 grasp_task
        # self._pub_grasp.publish(msg)
        self._pub_done.publish(msg)
        self.get_logger().info("发布 /apriltag_place1/done = True (不发 /grasp/start，由 block_align 接管)")

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
        """把 pose 加入瞄准缓冲，缓冲满(stable_frames)且方差足够小则可瞄准。"""
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
            return

        if state == STATE_DONE:
            return

        if state == STATE_ERROR:
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
        elif state == STATE_SEARCH:
            self._do_search(frame)
        elif state == STATE_LATERAL_ALIGN:
            self._do_lateral_align(frame)
        elif state == STATE_APPROACH:
            self._do_approach(frame)
        elif state == STATE_YAW_FINETUNE:
            self._do_yaw_finetune(frame)

    # ──────────────────────────── phase_1 ───────────────────────────────────── #

    def _do_wait_detect(self, frame: np.ndarray):
        self._detect_frames += 1
        if time.monotonic() > self._detect_deadline:
            ids_str = f" 最近一次 IDs={self._last_detected_tag_ids}" if self._last_detected_tag_ids else ""
            self.get_logger().warning(
                f"wait_detect 超时（{self._detect_timeout:.1f}s），进入横向搜索。诊断："
                f"已处理 {self._detect_frames} 帧，"
                f"检测到任意 Tag 的帧 {self._detect_any_tags_frames} 次{ids_str}，"
                f"检测到目标 ID 的帧 {self._detect_target_frames} 次，"
                f"稳定缓冲 {len(self._stable_buf)}/{self._stable_frames}"
            )
            # 进入横向搜索
            ready, issues = self._check_motion_pipeline()
            if not ready:
                self.get_logger().error(
                    f"运动链路未就绪，无法搜索：{'; '.join(issues)}")
                with self._lock:
                    self._state = STATE_ERROR
                return
            with self._lock:
                self._state = STATE_SEARCH
                self._search_steps_done = 0
                self._phase_busy = False
                self._settle_tracker_ready = False
                per_step_budget = self._move_timeout + 1.0
                self._search_deadline = time.monotonic() + min(
                    self._search_timeout_s,
                    self._search_max_steps * per_step_budget + 5.0,
                )
                self._stable_buf.clear()
            return

        pose = self._detect_tag(frame)
        if self._last_detected_tag_count > 0:
            self._detect_any_tags_frames += 1
        if pose is None:
            return
        self._detect_target_frames += 1

        # 只用本帧真实检测到的位姿做稳定判定，避免缓存位姿造成假稳定
        if not pose["raw"].get("fresh", False):
            return

        if self._is_stable(pose):
            # 滤波收敛 → 取缓冲中位数作为对齐起点，进入 2D 对齐（横向 → 前后 → yaw 微调）
            tx = float(np.median([p["tx"] for p in self._stable_buf]))
            tz = float(np.median([p["tz"] for p in self._stable_buf]))
            alpha = math.degrees(math.atan2(tx, tz))
            self.get_logger().info(
                f"瞄准锁定: tx={tx:.3f}m tz={tz:.3f}m alpha={alpha:.2f}°，进入 lateral_align"
            )
            with self._lock:
                self._last_pose  = {"tx": tx, "tz": tz}
                self._yaw_rounds = 0
                self._lat_rounds = 0
                self._app_rounds = 0
                self._stable_buf.clear()
                self._state = STATE_LATERAL_ALIGN
                self._phase_busy = False

    # ──────────────────────────── 横向搜索兜底 ─────────────────────────────── #

    def _do_search(self, frame: np.ndarray):
        """横向搜索：每步 search_step_m，最多 search_max_steps 步。
        每步停下后重新检测，检测到目标则进入 lateral_align。
        """
        now = time.monotonic()

        # 全局搜索超时
        if now > self._search_deadline:
            self.get_logger().error(
                f"search 超时（>{int(self._search_timeout_s)}s），未找到 Tag id={self._target_tag_id}")
            with self._lock:
                self._state = STATE_ERROR
            return

        # 步数用尽
        if self._search_steps_done >= self._search_max_steps:
            self.get_logger().error(
                f"search 步数用尽（{self._search_max_steps} 步 × {self._search_step_m:.2f}m），未找到 Tag")
            with self._lock:
                self._state = STATE_ERROR
            return

        # 正在运动中：等 cmd_vel 归零后再检测
        if self._phase_busy:
            # 检查运动超时
            if time.monotonic() - self._phase_start_time > self._move_timeout:
                self.get_logger().error(
                    f"search: 第 {self._search_steps_done} 步运动超时（{self._move_timeout}s），放弃")
                with self._lock:
                    self._state = STATE_ERROR
                return

            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                self._settle_tracker_ready = False
                # 重新尝试当前步
                return

            if not self._is_cmd_vel_zero():
                return
            # 运动停止：重建稳定缓冲
            if not self._settle_tracker_ready:
                self._stable_buf.clear()
                self._settle_tracker_ready = True

            pose = self._detect_tag(frame)
            if pose is None:
                return  # 本帧未检测到，继续等下一帧

            # 检测到目标：累积到稳定缓冲
            if not pose["raw"].get("fresh", False):
                return

            if self._is_stable(pose):
                tx = float(np.median([p["tx"] for p in self._stable_buf]))
                tz = float(np.median([p["tz"] for p in self._stable_buf]))
                self.get_logger().info(
                    f"search 第 {self._search_steps_done} 步命中: tx={tx:.3f}m tz={tz:.3f}m")
                with self._lock:
                    self._last_pose  = {"tx": tx, "tz": tz}
                    self._lat_rounds = 0
                    self._app_rounds = 0
                    self._yaw_rounds = 0
                    self._phase_busy = False
                    self._settle_tracker_ready = False
                    self._stable_buf.clear()
                    self._state = STATE_LATERAL_ALIGN
                return
            # 缓冲未满，继续累积
            return

        # 空闲：发下一步搜索指令（向右搜索，y 负方向）
        y_move = -self._search_step_m
        self._search_steps_done += 1
        self.get_logger().info(
            f"search 步 {self._search_steps_done}/{self._search_max_steps}: 发 /move y={y_move:.3f}m")
        self._send_move(0.0, y_move, 0.0)
        self._phase_busy = True
        self._phase_start_time = time.monotonic()
        self._settle_tracker_ready = False

    # ──────────────────────────── phase_2：横向对齐 ───────────────────────── #

    def _do_lateral_align(self, frame: np.ndarray):
        if getattr(self, "_phase_busy", False):
            # 检查运动超时
            if time.monotonic() - self._phase_start_time > self._move_timeout:
                self.get_logger().error(
                    f"lateral_align: 运动超时（{self._move_timeout}s），放弃")
                with self._lock:
                    self._state = STATE_ERROR
                return

            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                # 重新尝试当前轮次
                return

            if not self._is_cmd_vel_zero():
                if self._heartbeat("motion_wait", 2.0):
                    self.get_logger().info("lateral_align: 等待横移运动停止…")
                return
            # 运动停止：重建 tracker（清空缓冲，下一帧重新检测）
            self._stable_buf.clear()
            pose = self._detect_tag(frame)
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
        # 限幅：单次横移最大 10cm，防止晃出视野（参考 block_align 小步策略）
        y_move = -tx  # tx 右正 → y 左正，取负
        y_move = max(-0.10, min(0.10, y_move))
        self.get_logger().info(
            f"lateral_align 轮次 {self._lat_rounds}: tx={tx:.3f}m → y_move={y_move:.3f}m（限幅±10cm）")
        self._send_move(0.0, y_move, 0.0)
        self._phase_busy = True
        self._phase_start_time = time.monotonic()

    # ──────────────────────────── phase_3：前后逼近 ───────────────────────── #

    def _do_approach(self, frame: np.ndarray):
        """视觉闭环逼近，直到 tz ≈ closed_loop_end_dist（默认 1.0m）。

        2026-08-11 重构：不再有后续的 blind_forward，approach 完成后直接进入
        yaw_finetune（最多 5 次），yaw 修正即使未达标也 emit 抓取信号。
        """
        if getattr(self, "_phase_busy", False):
            # 检查运动超时
            if time.monotonic() - self._phase_start_time > self._move_timeout:
                self.get_logger().error(
                    f"approach: 运动超时（{self._move_timeout}s），放弃")
                with self._lock:
                    self._state = STATE_ERROR
                return

            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                # 重新尝试当前轮次
                return

            if not self._is_cmd_vel_zero():
                if self._heartbeat("motion_wait", 2.0):
                    self.get_logger().info("approach: 等待前进运动停止…")
                return
            # 运动停止：重建 tracker（清空缓冲，下一帧重新检测）
            self._stable_buf.clear()
            pose = self._detect_tag(frame)
            if pose is None:
                self._fallback_to_wait_detect("approach")
                return
            self._last_pose = pose
            self._phase_busy = False

        pose = self._last_pose
        tz   = pose["tz"]

        # 精确前进到 closed_loop_end_dist
        delta = tz - self._closed_loop_end
        if abs(delta) <= self._dist_thr:
            self.get_logger().info(
                f"approach 完成: tz={tz:.3f}m（闭环终点 {self._closed_loop_end:.2f}m），"
                f"进入 yaw_finetune（最多 {self._max_yaw_finetune} 次）")
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_YAW_FINETUNE
                self._yaw_rounds = 0
                self._phase_busy = False
            return

        if self._app_rounds >= self._max_rounds:
            self.get_logger().error("approach 超过最大轮次，放弃")
            with self._lock:
                self._state = STATE_ERROR
            return

        self._app_rounds += 1

        # 阻尼系数：只走误差的 60%，防止 pose_controller 超调
        damped_delta = delta * 0.6
        # 限幅：单次前进最大 20cm，防止晃出视野（从 30cm 改为 20cm）
        damped_delta = max(-0.20, min(0.20, damped_delta))
        # 最小步长：小于 5cm 不走了，直接认为到位
        if abs(damped_delta) < 0.05:
            self.get_logger().info(
                f"approach 完成（剩余误差 {delta:.3f}m < 5cm 阈值），"
                f"进入 yaw_finetune（最多 {self._max_yaw_finetune} 次）")
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_YAW_FINETUNE
                self._yaw_rounds = 0
                self._phase_busy = False
            return

        self.get_logger().info(
            f"approach 轮次 {self._app_rounds}: tz={tz:.3f}m  delta={delta:.3f}m → 阻尼+限幅后={damped_delta:.3f}m")
        self._send_move(damped_delta, 0.0, 0.0)
        self._phase_busy = True
        self._phase_start_time = time.monotonic()

    # ──────────────────────────── phase_4：yaw 收尾修正 ────────────────────── #

    def _do_yaw_finetune(self, frame: np.ndarray):
        """
        approach 完成（tz≈closed_loop_end_dist）后的 yaw 收尾修正。

        与 block_align 类似的 2D 主策略：先横向 + 前后到位，最后再做 yaw 微调。
        约束：
          - 最多 self._max_yaw_finetune 次（默认 3），达到即使仍有偏差也直接
            emit + done（不视为失败）。设计文档里 "yaw 有偏差不影响流程"。
          - 每次旋转单步限幅 max_yaw_step_deg（默认 3°），防惯性冲过。
          - 单次运动完毕重新检测 tag pose；若 tag 完全丢失（超出 lost_tolerance）
            则同样直接 emit + done——距离已到位，abcd_task 上层可以继续，
            不会因为 tag 视野问题卡住。
        """
        # busy：等运动停下，重新检测一次
        if getattr(self, "_phase_busy", False):
            # 检查运动超时
            elapsed = time.monotonic() - self._phase_start_time
            if elapsed > self._move_timeout:
                self.get_logger().error(
                    f"yaw_finetune: 运动超时（{self._move_timeout}s），跳过修正，直接完成")
                self._emit_and_done()
                return

            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                # 重新尝试当前轮次
                return

            if not self._is_cmd_vel_zero():
                if self._heartbeat("motion_wait", 2.0):
                    self.get_logger().info(f"yaw_finetune: 等待旋转运动停止…（已等待 {elapsed:.1f}s）")
                return
            pose = self._detect_tag(frame)
            if pose is None:
                # Tag 真丢失：距离已经到位，yaw 偏差不影响上层——直接结束
                self.get_logger().warning(
                    "yaw_finetune: tag 丢失（含 lost_tolerance 后），"
                    "距离已到位，跳过后续修正，直接结束")
                self._emit_and_done()
                return
            self._last_pose = pose
            self._phase_busy = False

        pose = self._last_pose
        tx, tz = pose["tx"], pose["tz"]
        if tz <= 0.0:
            self.get_logger().warning(
                f"yaw_finetune: tz={tz:.3f} 异常，跳过 yaw 修正，直接结束")
            self._emit_and_done()
            return

        alpha_rad = math.atan2(tx, tz)
        alpha_deg = math.degrees(alpha_rad)

        # 阈值内视为达标 → 直接完成
        if abs(alpha_deg) <= self._yaw_thr_deg:
            self.get_logger().info(
                f"yaw_finetune 完成: alpha={alpha_deg:.2f}° "
                f"(轮次={self._yaw_rounds}/{self._max_yaw_finetune})")
            self._emit_and_done()
            return

        # 用尽最大次数 → warning + 直接完成（关键行为：有偏差不影响流程）
        if self._yaw_rounds >= self._max_yaw_finetune:
            self.get_logger().warning(
                f"yaw_finetune: 达到最大次数 {self._max_yaw_finetune}，"
                f"仍有残余偏差 alpha={alpha_deg:.2f}°，按设计直接 emit 完成"
            )
            self._emit_and_done()
            return

        # 发一次修正
        self._yaw_rounds += 1
        theta_cmd = -max(min(alpha_deg, self._max_yaw_step_deg),
                         -self._max_yaw_step_deg)
        self.get_logger().info(
            f"yaw_finetune 轮次 {self._yaw_rounds}/{self._max_yaw_finetune}: "
            f"alpha={alpha_deg:.2f}°，发送旋转 theta={theta_cmd:.2f}°"
        )
        # 去掉 reset_origin 调用，避免与 pose_controller 时序冲突
        # （参考 grasp_node.py commit 5791914 的修复经验）
        # self._reset_origin()
        # time.sleep(0.15)

        # theta 逆时针为正，目标偏右(alpha>0) → 狗需向右转（theta 负）
        self._send_move(0.0, 0.0, theta_cmd)
        self._phase_busy = True
        self._phase_start_time = time.monotonic()

    def _emit_and_done(self):
        """统一收尾：发抓取信号 + 转 STATE_DONE。yaw_finetune 的多个出口共享。"""
        self._emit_place1()
        with self._lock:
            self._stable_buf.clear()
            self._phase_busy = False
            self._state = STATE_DONE

    # ──────────────────────────── 析构 ──────────────────────────────────────── #

    def destroy_node(self):
        self._close_camera()
        if self._show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPlace1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
