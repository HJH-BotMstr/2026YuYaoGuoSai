#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
block_align_node — 色块视觉对齐节点（无 AprilTag，无旋转步骤）

流程：
  wait_trigger   等待 /block_align/start
  wait_detect    BlockDetection + TargetTracker 稳定检测到色块
  lateral_align  横向平移，消除 X_cam 偏差
  approach       前进到 target_distance_m
  done           发布 /grasp/start（TRANSIENT_LOCAL，避免 race condition）

依赖：
  tools/grasp/utils/BlockDetection.py
  tools/grasp/utils/TargetTracker.py
  tools/grasp/config.yaml（检测参数）
"""

import sys
import os
import math
import time
import threading
from collections import deque
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

# ── tools/grasp 路径 ───────────────────────────────────────────────────────── #
_TOOLS_GRASP_DEFAULT = os.path.expanduser("~/2026YuYaoGuoSai/tools/grasp")

# ─── 状态常量 ──────────────────────────────────────────────────────────────── #
STATE_WAIT_TRIGGER  = "wait_trigger"
STATE_WAIT_DETECT   = "wait_detect"
STATE_SEARCH        = "search"          # 新增：wait_detect 超时后横向扫描
STATE_LATERAL_ALIGN = "lateral_align"
STATE_APPROACH      = "approach"
STATE_DONE          = "done"
STATE_ERROR         = "error"

_PIPELINE_TOPIC_TIMEOUT_S = 0.5

# TRANSIENT_LOCAL QoS：晚订阅的节点也能收到最后一条消息，解决 /grasp/start race
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class BlockAlignNode(Node):

    def __init__(self):
        super().__init__("block_align_node")

        # ── 参数声明 ─────────────────────────────────────────────────────── #
        self.declare_parameter("trigger_topic",          "/block_align/start")
        self.declare_parameter("tools_config_path",      _TOOLS_GRASP_DEFAULT + "/config.yaml")
        self.declare_parameter("camera_device",          "/dev/video0")
        self.declare_parameter("target_distance_m",      0.25)
        self.declare_parameter("lateral_threshold_mm",   10.0)
        self.declare_parameter("distance_threshold_m",   0.02)
        self.declare_parameter("max_rounds",             5)
        self.declare_parameter("stable_frames",          10)
        self.declare_parameter("detect_timeout_s",       15.0)
        self.declare_parameter("cmd_vel_zero_timeout_s", 0.5)
        self.declare_parameter("move_timeout_s",         10.0)
        # ── pose_controller 健康检查（2026-08-14）──
        self.declare_parameter("motion_watchdog_timeout_s", 3.0)
        self.declare_parameter("motion_reset_recovery_s",   1.0)
        # +1: 目标偏右时向右移（Pose2D.y 负值）
        # -1: 目标偏右时向左移（Pose2D.y 正值）
        # 现场实测后调整；默认 -1（X_cam>0 偏右 → y=−X_cam/1000 → 狗向右）
        self.declare_parameter("lateral_polarity",       -1)
        self.declare_parameter("show_debug_window",      False)
        # ── 横向搜索（wait_detect 超时后触发）──
        # 搜索方向：与 lateral_polarity 相反的 y 方向（正常抓取时 polarity=-1，
        # 目标偏右→往右移；搜索时 polarity=-1 → 从右往左扫，符合设计文档 §7.2）
        self.declare_parameter("search_step_m",          0.10)
        self.declare_parameter("search_max_steps",       10)
        self.declare_parameter("search_timeout_s",       60.0)
        # ── 目标颜色过滤（2026-08-12 新增）──
        # "red" | "green" | "" (空字符串 = 检测所有颜色，向后兼容)
        # abcd_task_node 会根据字母配置通过 set_parameters 动态设置
        self.declare_parameter("target_color",           "")

        # ── 读取参数 ─────────────────────────────────────────────────────── #
        self._trigger_topic  = self.get_parameter("trigger_topic").value
        self._tools_cfg_path = self.get_parameter("tools_config_path").value
        self._cam_device     = self.get_parameter("camera_device").value
        self._target_dist    = self.get_parameter("target_distance_m").value
        self._lat_thr_mm     = self.get_parameter("lateral_threshold_mm").value
        self._dist_thr       = self.get_parameter("distance_threshold_m").value
        self._max_rounds     = self.get_parameter("max_rounds").value
        self._stable_frames  = self.get_parameter("stable_frames").value
        self._detect_timeout = self.get_parameter("detect_timeout_s").value
        self._cmdvel_zero_t  = self.get_parameter("cmd_vel_zero_timeout_s").value
        self._move_timeout   = self.get_parameter("move_timeout_s").value
        self._watchdog_timeout = float(self.get_parameter("motion_watchdog_timeout_s").value)
        self._reset_recovery   = float(self.get_parameter("motion_reset_recovery_s").value)
        self._lat_polarity   = int(self.get_parameter("lateral_polarity").value)
        self._show_debug     = bool(self.get_parameter("show_debug_window").value)
        self._search_step_m      = float(self.get_parameter("search_step_m").value)
        self._search_max_steps   = int(self.get_parameter("search_max_steps").value)
        self._search_timeout_s   = float(self.get_parameter("search_timeout_s").value)
        self._target_color_param = str(self.get_parameter("target_color").value)
        # 空字符串转为 None（向后兼容：不指定颜色时检测所有）
        self._target_color = self._target_color_param if self._target_color_param else None

        # ── 加载 tools/grasp/config.yaml ─────────────────────────────────── #
        self._tools_cfg = self._load_tools_config()

        # ── 将 tools/grasp 加入 Python 路径 ──────────────────────────────── #
        tools_path = os.path.dirname(self._tools_cfg_path)
        if tools_path not in sys.path:
            sys.path.insert(0, tools_path)

        # ── BlockDetection + TargetTracker ────────────────────────────────── #
        try:
            from utils.BlockDetection import BlockDetection
            from utils.TargetTracker  import TargetTracker
            det_cfg = self._tools_cfg["detection"]
            g_cfg   = self._tools_cfg["grasp"]
            # 传入 target_color 参数进行颜色过滤
            self._detector = BlockDetection(det_cfg, target_color=self._target_color)
            self._tracker  = TargetTracker(
                avg_window=int(g_cfg["distance_avg_window"]),
                lost_frames_max=int(g_cfg["lost_frames_max"]),
            )
            self._TargetTracker = TargetTracker
            self._tracker_avg   = int(g_cfg["distance_avg_window"])
            self._tracker_lost  = int(g_cfg["lost_frames_max"])
            if self._target_color:
                self.get_logger().info(f"BlockDetection 目标颜色过滤: {self._target_color}")
        except ImportError as e:
            self.get_logger().fatal("BlockDetection/TargetTracker 导入失败: %s" % e)
            raise

        # ── 摄像头 ───────────────────────────────────────────────────────── #
        self._cap: Optional[cv2.VideoCapture] = None
        self._open_camera()

        # ── ROS 通信 ─────────────────────────────────────────────────────── #
        self._sub_trigger = self.create_subscription(
            Bool, self._trigger_topic, self._trigger_cb, 10)
        self._sub_cmdvel  = self.create_subscription(
            Twist, "/cmd_vel", self._cmdvel_cb, 10)
        self._sub_odom    = self.create_subscription(
            Odometry, "/leg_odom2", self._odom_cb, 10)

        self._pub_move  = self.create_publisher(Pose2D,  "/move",                 10)
        self._pub_cmd   = self.create_publisher(String,  "/pose_control/command", 10)
        self._pub_grasp = self.create_publisher(Bool,    "/grasp/start",          _LATCHED_QOS)

        # ── 运行时状态 ───────────────────────────────────────────────────── #
        self._state     = STATE_WAIT_TRIGGER
        self._lock      = threading.Lock()
        self._last_pose: Optional[dict] = None   # 最近稳定检测结果

        # cmd_vel 监测
        self._cmdvel_history: deque = deque(maxlen=30)
        self._last_cmdvel_time      = 0.0
        self._cmdvel_received_count = 0
        self._cmdvel_motion_started = False
        self._last_move_time        = 0.0
        self._move_ignored_warned   = False

        # odom 监测
        self._last_legodom_time     = 0.0

        # 看门狗：记录上次成功收到运动链路数据的时间
        self._last_pipeline_healthy_time = time.monotonic()
        self._pipeline_reset_count       = 0

        # 轮次计数
        self._lat_rounds = 0
        self._app_rounds = 0
        self._phase_busy = False
        self._settle_tracker_ready = False   # 见 _do_lateral_align
        self._detect_deadline = float("inf")
        # 横向搜索状态
        self._search_steps_done  = 0
        self._search_deadline    = float("inf")

        # 主循环 10 Hz
        self._timer = self.create_timer(0.1, self._main_loop)
        self.get_logger().info(
            "block_align_node 已启动，等待触发: %s" % self._trigger_topic)

    # ──────────────────────────── 配置加载 ──────────────────────────────────── #

    def _load_tools_config(self) -> dict:
        path = self._tools_cfg_path
        if not os.path.exists(path):
            self.get_logger().fatal("tools config 不存在: %s" % path)
            raise FileNotFoundError(path)
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    # ──────────────────────────── 摄像头 ────────────────────────────────────── #

    def _open_camera(self):
        self.get_logger().info("打开摄像头: %s" % self._cam_device)
        cap = cv2.VideoCapture(self._cam_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error("摄像头打开失败: %s" % self._cam_device)
            self._cap = None
            return
        ret, frame = cap.read()
        if not ret or frame is None:
            self.get_logger().error("摄像头无法读帧: %s" % self._cam_device)
            cap.release()
            self._cap = None
            return
        self._cap = cap
        self.get_logger().info("摄像头已就绪: %s  帧大小=%s" % (self._cam_device, frame.shape))

    def _read_frame(self) -> Optional[np.ndarray]:
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        return frame if (ret and frame is not None) else None

    # ──────────────────────────── 回调 ──────────────────────────────────────── #

    def _trigger_cb(self, msg: Bool):
        with self._lock:
            if msg.data:
                if self._state in (STATE_WAIT_TRIGGER, STATE_ERROR):
                    self.get_logger().info("收到触发信号，进入 wait_detect")
                    self._state = STATE_WAIT_DETECT
                    self._tracker = self._TargetTracker(
                        avg_window=self._tracker_avg,
                        lost_frames_max=self._tracker_lost,
                    )
                    self._lat_rounds = 0
                    self._app_rounds = 0
                    self._phase_busy = False
                    self._settle_tracker_ready = False
                    self._cmdvel_motion_started = False
                    self._move_ignored_warned   = False
                    self._detect_deadline = time.monotonic() + self._detect_timeout
                    self._search_steps_done = 0
                    self._search_deadline    = float("inf")
            else:
                if self._state not in (STATE_WAIT_TRIGGER, STATE_DONE):
                    self.get_logger().info("收到取消信号，回到 wait_trigger")
                    self._send_move(0.0, 0.0, 0.0)
                    self._state = STATE_WAIT_TRIGGER

    def _cmdvel_cb(self, msg: Twist):
        speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
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
        # 更新运动链路健康时间戳
        self._last_pipeline_healthy_time = time.monotonic()

    # ──────────────────────────── 检测 ──────────────────────────────────────── #

    def _detect_frame(self, frame: np.ndarray) -> Optional[dict]:
        """单帧检测，更新 TargetTracker，返回稳定结果或 None。"""
        candidates = self._detector.detect_all(frame)
        self._tracker.update(candidates)

        # 调试窗口（无 X display 时会拉起 Qt 崩溃，默认关闭）
        if self._show_debug:
            # 画 tracker 当前锁定的目标（而非最近的 candidates[0]），
            # 未锁定或未匹配到时退回画 candidates[0]，都没有就不画
            locked = self._tracker.get_current_target()
            draw = locked if locked is not None else (candidates[0] if candidates else None)
            vis = self._detector.visualize(frame.copy(), draw)
            cv2.putText(vis, "state=%s" % self._state, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("block_align", vis)
            cv2.waitKey(1)

        return self._tracker.get_stable_target()

    # ──────────────────────────── 运动指令 ──────────────────────────────────── #

    def _send_move(self, x: float, y: float, theta_deg: float):
        msg = Pose2D()
        msg.x     = float(x)
        msg.y     = float(y)
        msg.theta = float(theta_deg)
        self._pub_move.publish(msg)
        self._cmdvel_motion_started = False
        self._last_move_time  = time.monotonic()
        self._move_ignored_warned = False
        if self._pub_move.get_subscription_count() == 0:
            self.get_logger().warning("/move 当前无订阅者，运动指令可能丢失")
        self.get_logger().info(
            "发布 /move  x=%.3f  y=%.3f  theta=%.1f°" % (x, y, theta_deg))

    def _reset_origin(self):
        self._pub_cmd.publish(String(data="reset_origin"))
        self.get_logger().info("发布 reset_origin")

    def _reset_pose_controller(self):
        """发送 cancel 命令重置 pose_controller，清空其内部运动队列和状态。"""
        self._pub_cmd.publish(String(data="cancel"))
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

    def _emit_grasp_start(self):
        """
        发布 /grasp/start = True 触发 grasp_task，并释放摄像头。

        修复 2026-08-12：block_align 完成后发布此信号，取代原 apriltag_place1 的触发。
        新流程：apriltag → block_align（色块对齐+横向搜索）→ grasp_task（抓取）。
        避免了 apriltag 和 block_align 同时触发 grasp_task 导致的摄像头资源竞争。

        修复 2026-08-14：立即退出节点，彻底释放摄像头资源。
        block_align 完成任务后不再需要存在，立即退出避免与 grasp_task 竞争 /dev/video0。
        """
        self._pub_grasp.publish(Bool(data=True))
        self.get_logger().info("发布 /grasp/start = True (触发 grasp_task 抓取)")
        # 立刻释放摄像头，把 /dev/video0 让给 grasp_task 的 DETECTING 阶段；
        # grasp_flow 会随后杀掉本节点，spin 期间无需再持有设备
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception as e:
                self.get_logger().warning("释放摄像头异常: %s" % (e,))
            self._cap = None

        # 2026-08-14: 立即退出节点，彻底释放摄像头资源
        self.get_logger().info("摄像头已释放，节点即将退出")
        import sys
        sys.exit(0)

    # ──────────────────────────── 运动完成判断 ──────────────────────────────── #

    def _is_cmd_vel_zero(self) -> bool:
        now = time.monotonic()
        started  = self._cmdvel_motion_started

        # 兜底检查必须在 recent 检查之前：防止 recent=[] 时提前返回 False
        # 极小步（如 y=0.012m）cmd_vel 峰值不到 0.01，_cmdvel_motion_started
        # 永远不翻 True，会把 phase_busy 分支卡死。发出指令后超过 move_timeout_s
        # 仍未检测到运动，视为已完成，放行下一相位。
        if (not started
                and self._last_move_time > 0.0
                and now - self._last_move_time > self._move_timeout):
            if not self._move_ignored_warned:
                self.get_logger().warning(
                    "运动指令 %.1fs 内未见非零 cmd_vel，按已完成放行" % self._move_timeout)
                self._move_ignored_warned = True
            return True

        cutoff = now - self._cmdvel_zero_t
        recent = [(t, v) for (t, v) in self._cmdvel_history if t >= cutoff]
        if not recent:
            return False
        all_zero = all(v < 0.01 for (_, v) in recent)

        if not started and self._last_move_time > 0.0 and not self._move_ignored_warned:
            if now - self._last_move_time > self._cmdvel_zero_t:
                self.get_logger().warning(
                    "运动指令发出后未检测到 /cmd_vel 非零，"
                    "可能被控制器忽略（检查 /leg_odom2 是否已发布）")
                self._move_ignored_warned = True

        return started and all_zero

    def _check_motion_pipeline(self) -> Tuple[bool, List[str]]:
        issues = []
        if self._pub_move.get_subscription_count() == 0:
            issues.append("/move 无订阅者")
        now = time.monotonic()
        if now - self._last_legodom_time > _PIPELINE_TOPIC_TIMEOUT_S:
            issues.append("/leg_odom2 未收到或已超时 (%.1fs)" % (now - self._last_legodom_time))
        if now - self._last_cmdvel_time > _PIPELINE_TOPIC_TIMEOUT_S:
            issues.append("/cmd_vel 未收到或已超时 (%.1fs)" % (now - self._last_cmdvel_time))
        return (not issues), issues

    # ──────────────────────────── 主循环 ────────────────────────────────────── #

    def _main_loop(self):
        with self._lock:
            state = self._state

        if state in (STATE_WAIT_TRIGGER, STATE_DONE, STATE_ERROR):
            return

        if self._cap is None:
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

    # ──────────────────────────── wait_detect ───────────────────────────────── #

    def _do_wait_detect(self, frame: np.ndarray):
        if time.monotonic() > getattr(self, "_detect_deadline", float("inf")):
            # 检测超时：进入横向搜索（每步 search_step_m，最多 search_max_steps 步）
            self.get_logger().warning(
                "wait_detect 超时（%.1fs），进入 search 横向搜索" % self._detect_timeout)
            # 先检查运动链路——搜索要靠 /move
            ready, issues = self._check_motion_pipeline()
            if not ready:
                self.get_logger().error(
                    "运动链路未就绪，无法搜索：%s" % "；".join(issues))
                with self._lock:
                    self._state = STATE_ERROR
                return
            with self._lock:
                self._state = STATE_SEARCH
                self._search_steps_done  = 0
                self._phase_busy         = False
                self._settle_tracker_ready = False
                # 每步预算 = move_timeout + 1s 缓冲，兜住整个搜索最长耗时
                per_step_budget = self._move_timeout + 1.0
                self._search_deadline = time.monotonic() + min(
                    self._search_timeout_s,
                    self._search_max_steps * per_step_budget + 5.0,
                )
                # 搜索开始前重建 tracker，避免带着历史噪声
                self._tracker = self._TargetTracker(
                    avg_window=self._tracker_avg,
                    lost_frames_max=self._tracker_lost,
                )
            return

        stable = self._detect_frame(frame)
        if stable is None:
            return

        # 修复 2026-08-14：确保锁定最右边的目标
        # 场景：两个同色方块同框时，tracker 可能锁定了靠中心的（不是最右的）
        # 解决：检测到稳定目标后，如果当前帧有多个候选且 tracker 没锁定最右的，重新锁定
        all_candidates = self._detector.detect_all(frame)
        if len(all_candidates) >= 2:
            rightmost = max(all_candidates, key=lambda r: r["pos_3d"][0])
            # 判断 tracker 当前锁定的是否是最右的
            if stable["pos_3d"][0] < rightmost["pos_3d"][0] - 5.0:  # 5mm 容差
                self.get_logger().info(
                    "wait_detect 检测到更右的目标 (%.1fmm > %.1fmm)，重新锁定" % (
                        rightmost["pos_3d"][0], stable["pos_3d"][0]))
                # 重建 tracker 并锁定最右的
                self._tracker = self._TargetTracker(
                    avg_window=self._tracker_avg,
                    lost_frames_max=self._tracker_lost,
                )
                self._tracker.update([rightmost])
                return  # 等待下一帧稳定

        X_cam, Y_cam, _ = stable["pos_3d"]
        self.get_logger().info(
            "色块稳定锁定: X_cam=%.1fmm  Y_cam=%.1fmm  dist=%.1fmm" % (
                X_cam, Y_cam, stable["distance_mm"]))

        # 首次检测到稳定目标前检查运动链路
        ready, issues = self._check_motion_pipeline()
        if not ready:
            self.get_logger().error(
                "运动链路未就绪：%s。请确认 pose_controller 和 /leg_odom2 已启动。" % "；".join(issues))
            with self._lock:
                self._state = STATE_ERROR
            return

        with self._lock:
            self._last_pose  = stable
            self._lat_rounds = 0
            self._app_rounds = 0
            self._phase_busy = False
            self._state      = STATE_LATERAL_ALIGN

    # ──────────────────────────── search ────────────────────────────────────── #

    def _do_search(self, frame: np.ndarray):
        """
        横向搜索：wait_detect 超时后触发。每步沿 body y 方向平移 search_step_m，
        运动停止后重新检测一帧；发现色块立即转入 lateral_align，否则继续下一步。

        搜索方向由 lateral_polarity 反号决定：
          - lateral_polarity=-1（默认，目标偏右→向右移）→ 搜索沿 +y（body 左）
            即从右往左扫，与设计文档 §7.2 "从右往左"一致。
          - lateral_polarity=+1 → 搜索沿 -y（body 右）。
        """
        now = time.monotonic()

        # 全局搜索超时
        if now > self._search_deadline:
            self.get_logger().error(
                "search 超时（>%ds），未找到色块" % int(self._search_timeout_s))
            with self._lock:
                self._state = STATE_ERROR
            return

        # 步数用尽
        if self._search_steps_done >= self._search_max_steps:
            self.get_logger().error(
                "search 步数用尽（%d 步 × %.2fm），未找到色块" % (
                    self._search_max_steps, self._search_step_m))
            with self._lock:
                self._state = STATE_ERROR
            return

        # 正在运动中：等 cmd_vel 归零后再检测
        if self._phase_busy:
            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                self._settle_tracker_ready = False
                # 重新尝试当前步
                return

            if not self._is_cmd_vel_zero():
                return
            # 运动停止：重建 tracker 一次，然后连续多帧才会 stable
            if not self._settle_tracker_ready:
                self._tracker = self._TargetTracker(
                    avg_window=self._tracker_avg,
                    lost_frames_max=self._tracker_lost,
                )
                self._settle_tracker_ready = True

            stable = self._detect_frame(frame)
            if stable is None:
                return  # tracker 未稳定，继续等下一帧

            # 修复 2026-08-14：确保锁定最右边的目标
            # 场景：两个同色方块同框时，tracker 可能锁定了靠中心的（不是最右的）
            # 解决：检测到稳定目标后，如果当前帧有多个候选且 tracker 没锁定最右的，重新锁定
            all_candidates = self._detector.detect_all(frame)
            if len(all_candidates) >= 2:
                rightmost = max(all_candidates, key=lambda r: r["pos_3d"][0])
                # 判断 tracker 当前锁定的是否是最右的
                if stable["pos_3d"][0] < rightmost["pos_3d"][0] - 5.0:  # 5mm 容差
                    self.get_logger().info(
                        "search 检测到更右的目标 (%.1fmm > %.1fmm)，重新锁定" % (
                            rightmost["pos_3d"][0], stable["pos_3d"][0]))
                    # 重建 tracker 并锁定最右的
                    self._tracker = self._TargetTracker(
                        avg_window=self._tracker_avg,
                        lost_frames_max=self._tracker_lost,
                    )
                    self._tracker.update([rightmost])
                    self._settle_tracker_ready = False
                    return  # 等待下一帧稳定

            X_cam, Y_cam, _ = stable["pos_3d"]
            self.get_logger().info(
                "search 第 %d 步命中: X_cam=%.1fmm  Y_cam=%.1fmm  dist=%.1fmm" % (
                    self._search_steps_done, X_cam, Y_cam, stable["distance_mm"]))
            with self._lock:
                self._last_pose  = stable
                self._lat_rounds = 0
                self._app_rounds = 0
                self._phase_busy = False
                self._settle_tracker_ready = False
                self._state     = STATE_LATERAL_ALIGN
            return

        # 空闲：发下一步搜索指令
        # 修复 2026-08-14：搜索方向改为"从左往右"（body -y 方向），
        # 优先覆盖右侧区域，配合 TargetTracker 的 max(X_cam) 确保锁定最右边的目标
        # lateral_polarity=-1 时向 body -y 方向搜（从左往右）→ 取 +polarity
        y_dir = self._lat_polarity
        y_move = y_dir * self._search_step_m
        self._search_steps_done += 1
        self.get_logger().info(
            "search 步 %d/%d: 发 /move y=%.3f m (polarity=%d, dir=%+d)" % (
                self._search_steps_done, self._search_max_steps,
                y_move, self._lat_polarity, y_dir))
        self._send_move(0.0, y_move, 0.0)
        self._phase_busy = True
        self._settle_tracker_ready = False

    # ──────────────────────────── lateral_align ─────────────────────────────── #

    def _do_lateral_align(self, frame: np.ndarray):
        if self._phase_busy:
            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                self._settle_tracker_ready = False
                # 重新尝试当前轮次
                return

            if not self._is_cmd_vel_zero():
                return
            # 运动停止后重新检测：tracker 只在"刚停下"那一帧重建一次，
            # 之后连续多帧 update 才能凑够 stable_frames；每次都重建的话
            # 计数永远从 0 开始，永远不会稳定，node 会一直卡在这里
            if not self._settle_tracker_ready:
                self._tracker = self._TargetTracker(
                    avg_window=self._tracker_avg,
                    lost_frames_max=self._tracker_lost,
                )
                self._settle_tracker_ready = True
            stable = self._detect_frame(frame)
            if stable is None:
                # 增强 debug：避免静默等待，打印当前状态
                if self._lat_rounds > 0:
                    self.get_logger().warn(
                        "lateral_align 等待稳定检测（已执行 %d 轮，tracker 未稳定）" %
                        self._lat_rounds)
                return   # tracker 未稳定，等下一帧
            self._last_pose  = stable
            self._phase_busy = False
            self._settle_tracker_ready = False

        pose  = self._last_pose
        X_cam = pose["pos_3d"][0]

        if abs(X_cam) <= self._lat_thr_mm:
            self.get_logger().info("lateral_align 完成: X_cam=%.1fmm" % X_cam)
            with self._lock:
                self._tracker = self._TargetTracker(
                    avg_window=self._tracker_avg,
                    lost_frames_max=self._tracker_lost,
                )
                self._phase_busy = False
                self._state      = STATE_APPROACH
            return

        if self._lat_rounds >= self._max_rounds:
            self.get_logger().error(
                "lateral_align 超过最大轮次，放弃。X_cam=%.1fmm" % X_cam)
            with self._lock:
                self._state = STATE_ERROR
            return

        self._lat_rounds += 1
        # lateral_polarity: -1 → 偏右时 y 为负（向右移动）; +1 → 偏右时 y 为正（向左移动）
        y_move = self._lat_polarity * X_cam / 1000.0
        self.get_logger().info(
            "lateral_align 轮次 %d: X_cam=%.1fmm → Pose2D.y=%.3fm (polarity=%d)" % (
                self._lat_rounds, X_cam, y_move, self._lat_polarity))
        # 不发 reset_origin：/move 本身会把 _integral_* 清零、start_pose 用当前位姿，
        # 但 reset_origin 跟 /move 走两个话题不保序，先到会把 target 清空、state=idle，
        # 于是 approach 阶段 /move 一发就被立刻取消，_is_cmd_vel_zero() 误判"到位"
        self._send_move(0.0, y_move, 0.0)
        self._phase_busy = True

    # ──────────────────────────── approach ──────────────────────────────────── #

    def _do_approach(self, frame: np.ndarray):
        if self._phase_busy:
            # 运动链路看门狗检查
            if self._check_motion_watchdog():
                self._reset_pose_controller()
                self._phase_busy = False
                self._settle_tracker_ready = False
                # 重新尝试当前轮次
                return

            if not self._is_cmd_vel_zero():
                return
            # 同 _do_lateral_align：tracker 只在刚停下时重建一次
            if not self._settle_tracker_ready:
                self._tracker = self._TargetTracker(
                    avg_window=self._tracker_avg,
                    lost_frames_max=self._tracker_lost,
                )
                self._settle_tracker_ready = True
            stable = self._detect_frame(frame)
            if stable is None:
                # 增强 debug：避免静默等待，打印当前状态
                if self._app_rounds > 0:
                    self.get_logger().warn(
                        "approach 等待稳定检测（已执行 %d 轮，tracker 未稳定）" %
                        self._app_rounds)
                return
            self._last_pose  = stable
            self._phase_busy = False
            self._settle_tracker_ready = False

        pose  = self._last_pose
        Y_cam = pose["pos_3d"][1]          # mm，光轴向前
        dist_m = Y_cam / 1000.0            # 转为米
        delta  = dist_m - self._target_dist

        if abs(delta) <= self._dist_thr:
            self.get_logger().info(
                "approach 完成: Y_cam=%.1fmm  dist=%.3fm" % (Y_cam, dist_m))
            self._emit_grasp_start()
            with self._lock:
                self._state = STATE_DONE
            return

        if self._app_rounds >= self._max_rounds:
            self.get_logger().error(
                "approach 超过最大轮次，放弃。dist=%.3fm" % dist_m)
            with self._lock:
                self._state = STATE_ERROR
            return

        self._app_rounds += 1
        self.get_logger().info(
            "approach 轮次 %d: Y_cam=%.1fmm  delta=%.3fm" % (
                self._app_rounds, Y_cam, delta))
        # 见 _do_lateral 注释：reset_origin 与 /move 竞态会使 /move 被作废
        self._send_move(delta, 0.0, 0.0)
        self._phase_busy = True

    # ──────────────────────────── 析构 ──────────────────────────────────────── #

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = BlockAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
