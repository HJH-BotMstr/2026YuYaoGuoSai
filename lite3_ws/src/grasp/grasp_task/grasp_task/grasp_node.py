#!/usr/bin/env python3
"""
grasp_task 主节点：把 tools/grasp 的 8-phase 抓取流程封装为 ROS2 节点。

2026-08-15 合并 block_align 逻辑：
  将 block_align_node 的色块对齐功能合并到 grasp_task，消除摄像头重复开关延迟。
  新增 BLOCK_ALIGNING 阶段，在 DETECTING 之前执行视觉对齐和接近。

对外接口：
  sub /grasp/start  (std_msgs/Bool)   : 启动抓取流程（包含 block_align）
  sub /grasp/place  (std_msgs/String): 触发放置，携带 A/B/C/D
  sub /grasp/set_zone (std_msgs/String): 仅设置目标放置区
  sub /cmd_vel      (geometry_msgs/Twist): 判断 pose_control 是否到位
  pub /grasp/state  (std_msgs/String): 当前状态
  pub /grasp/result (std_msgs/Bool) : 最终成功/失败
  pub /move         (geometry_msgs/Pose2D): 横向对齐指令
  pub /pose_control/command (std_msgs/String): 如 reset_origin
"""
import sys
import os
import signal
import time
import threading
import logging

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool as BoolMsg, String
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

# 将 tools/grasp 加入 Python 搜索路径，以便复用 ArmController / BlockDetection 等
TOOLS_GRASP = "/home/ysc/2026YuYaoGuoSai/tools/grasp"
if TOOLS_GRASP not in sys.path:
    sys.path.insert(0, TOOLS_GRASP)

from utils.ArmController import ArmController
from utils.BlockDetection import BlockDetection
from utils.TargetTracker import TargetTracker
from utils.InspectionMemory import InspectionMemory

from .config_loader import load_config
from .motion_waiter import MotionWaiter


VALID_ZONES = {"A", "B", "C", "D"}


def _open_camera(device: str, retries: int = 3, delay: float = 1.0, logger=None):
    """尝试打开摄像头，失败时重试；返回 VideoCapture 或 None。"""
    for attempt in range(1, retries + 1):
        if logger:
            logger.info("尝试打开摄像头: %s (第 %d/%d 次)" % (device, attempt, retries))
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                if logger:
                    logger.info("摄像头打开成功: %s, 帧大小=%s" % (device, frame.shape))
                return cap
            else:
                if logger:
                    logger.warning("摄像头能打开但无法读帧，尝试重新打开")
                cap.release()
        else:
            if logger:
                logger.warning("摄像头打开失败: %s" % (device))
        if attempt < retries:
            time.sleep(delay)
    return None


class GraspTaskNode(Node):
    """ROS2 抓取任务节点。"""

    def __init__(self):
        super().__init__("grasp_task")

        # 声明参数
        self.declare_parameter("tools_config_path",
                               "/home/ysc/2026YuYaoGuoSai/tools/grasp/config.yaml")
        self.declare_parameter("start_topic", "/grasp/start")
        self.declare_parameter("place_topic", "/grasp/place")
        self.declare_parameter("set_zone_topic", "/grasp/set_zone")
        self.declare_parameter("state_topic", "/grasp/state")
        self.declare_parameter("result_topic", "/grasp/result")
        self.declare_parameter("move_topic", "/move")
        self.declare_parameter("command_topic", "/pose_control/command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/leg_odom2")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("cv_show", False)
        self.declare_parameter("motion_stop_timeout_s", 15.0)
        self.declare_parameter("motion_stop_zero_duration_s", 0.5)
        self.declare_parameter("odom_fresh_timeout_s", 0.5)
        # 多轮循环：max_rounds=1 时保持原有单轮行为；abcd_task 会覆写为 4。
        # inter_round_wait_s 是两轮之间的短暂间隔，让 latched 消息/上层编排刷新。
        self.declare_parameter("max_rounds", 1)
        self.declare_parameter("inter_round_wait_s", 0.5)
        # ROI 检测区域限制（仅在 grasp_task 检测阶段使用，避免左右两侧干扰色块）
        self.declare_parameter("use_roi", True)
        self.declare_parameter("roi_center_ratio", 0.6)
        # 横向对齐开关（已废弃，合并 block_align 后始终执行对齐）
        self.declare_parameter("enable_lateral_align", True)

        # ── 2026-08-15: block_align 合并参数 ──────────────────────────────
        self.declare_parameter("enable_block_align", True)          # 是否启用 block_align 阶段
        self.declare_parameter("block_align_target_distance_m", 0.25)  # block_align 目标停止距离
        self.declare_parameter("block_align_lateral_threshold_mm", 15.0)  # 横向对齐阈值
        self.declare_parameter("block_align_distance_threshold_m", 0.02)  # 前进距离阈值
        self.declare_parameter("block_align_max_rounds", 5)         # 对齐最大轮次
        self.declare_parameter("block_align_detect_timeout_s", 15.0)  # 检测超时
        self.declare_parameter("block_align_lateral_polarity", -1)  # 横向极性
        self.declare_parameter("block_align_motion_watchdog_s", 3.0)  # 运动看门狗超时
        self.declare_parameter("target_color", "")                  # 目标颜色过滤 "red"/"green"/""

        # 加载配置并强制 robot 模式
        self.cfg = load_config(self)
        self.dry_run_param = self.get_parameter("dry_run").value
        # launch 参数可能以字符串传入，统一转成 bool
        if isinstance(self.dry_run_param, str):
            self.dry_run = self.dry_run_param.strip().lower() in ("true", "1", "yes")
        else:
            self.dry_run = bool(self.dry_run_param)
        self.cv_show = self.get_parameter("cv_show").value

        # ROI 参数
        self.use_roi = self.get_parameter("use_roi").value
        self.roi_center_ratio = self.get_parameter("roi_center_ratio").value

        # 横向对齐开关
        self.enable_lateral_align = self.get_parameter("enable_lateral_align").value

        # ── 2026-08-15: block_align 合并参数读取 ─────────────────────────
        self.enable_block_align = self.get_parameter("enable_block_align").value
        self.block_align_target_dist = self.get_parameter("block_align_target_distance_m").value
        self.block_align_lat_thr_mm = self.get_parameter("block_align_lateral_threshold_mm").value
        self.block_align_dist_thr = self.get_parameter("block_align_distance_threshold_m").value
        self.block_align_max_rounds = self.get_parameter("block_align_max_rounds").value
        self.block_align_detect_timeout = self.get_parameter("block_align_detect_timeout_s").value
        self.block_align_lat_polarity = int(self.get_parameter("block_align_lateral_polarity").value)
        self.block_align_watchdog_timeout = self.get_parameter("block_align_motion_watchdog_s").value

        target_color_param = str(self.get_parameter("target_color").value)
        self.target_color = target_color_param if target_color_param else None

        # block_align 运行时状态
        self._ba_lat_rounds = 0
        self._ba_app_rounds = 0
        self._ba_phase_busy = False
        self._ba_settle_tracker_ready = False
        self._ba_last_pipeline_healthy_time = 0.0
        self._ba_cmdvel_motion_started = False
        self._ba_last_move_time = 0.0
        self._ba_move_ignored_warned = False

        self._lock = threading.Lock()
        self._start_event = threading.Event()
        self._place_event = threading.Event()
        self._target_zone: str = None
        self._estop = False
        self._odom_fresh = False
        self._last_odom_time = 0.0

        # topic 名称
        self._start_topic = self.get_parameter("start_topic").value
        self._place_topic = self.get_parameter("place_topic").value
        self._set_zone_topic = self.get_parameter("set_zone_topic").value
        self._state_topic = self.get_parameter("state_topic").value
        self._result_topic = self.get_parameter("result_topic").value
        self._move_topic = self.get_parameter("move_topic").value
        self._command_topic = self.get_parameter("command_topic").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value

        motion_timeout = self.get_parameter("motion_stop_timeout_s").value
        motion_zero_dur = self.get_parameter("motion_stop_zero_duration_s").value
        self._motion_waiter = MotionWaiter(
            zero_duration_s=motion_zero_dur,
            timeout_s=motion_timeout,
        )
        self._motion_stop_timeout_s = motion_timeout  # block_align 阶段兜底超时复用

        # 订阅
        self.create_subscription(BoolMsg, self._start_topic, self._on_start, 10)
        self.create_subscription(String, self._place_topic, self._on_place, 10)
        self.create_subscription(String, self._set_zone_topic, self._on_zone, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(BoolMsg, "/emergency_stop", self._on_estop, 10)

        # 发布
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self._result_pub = self.create_publisher(BoolMsg, self._result_topic, 10)
        self._move_pub = self.create_publisher(Pose2D, self._move_topic, 10)
        self._cmd_pub = self.create_publisher(String, self._command_topic, 10)

        # 状态心跳：state 话题为 volatile，晚订阅的节点（如 grasp_flow 编排器）
        # 会错过一次性状态跳变，1Hz 重发当前状态兜底
        self._current_state = "BOOT"
        self.create_timer(1.0, self._republish_state)

        # 初始化硬件
        self.arm = None
        self.detector = None
        self.tracker = None
        self.memory = None
        self.arm_cam = None
        try:
            self._init_hardware()
        except Exception as e:
            self.get_logger().error("硬件初始化失败: %s" % (e,))
            self._publish_state("ERROR:HW_INIT_FAILED")
            self._result_pub.publish(BoolMsg(data=False))
            raise

        self.get_logger().info(
            "grasp_task 节点已初始化，等待 %s 信号 (dry_run=%s)" % (self._start_topic, self.dry_run)
        )

    # ------------------------------------------------------------------ #
    # ROS 回调
    # ------------------------------------------------------------------ #
    def _on_start(self, msg: BoolMsg):
        if msg.data:
            self.get_logger().info("收到 /grasp/start 信号")
            self._start_event.set()

    def _on_place(self, msg: String):
        zone = msg.data.upper()
        if zone in VALID_ZONES:
            self.memory.set_zone(zone)
            self.get_logger().info("收到 /grasp/place 信号，zone=%s" % (zone))
            self._place_event.set()
        else:
            self.get_logger().warn("收到无效放置区: %s" % (msg.data))

    def _on_zone(self, msg: String):
        zone = msg.data.upper()
        if zone in VALID_ZONES:
            self.memory.set_zone(zone)
            self.get_logger().info("通过 /grasp/set_zone 设置 zone=%s" % (zone))
        else:
            self.get_logger().warn("收到无效放置区: %s" % (msg.data))

    def _on_cmd_vel(self, msg: Twist):
        self._motion_waiter.on_cmd_vel(msg)
        # 2026-08-15: 更新 block_align 运动状态
        speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        if speed >= 0.01 and not self._ba_cmdvel_motion_started:
            self._ba_cmdvel_motion_started = True
        self._ba_last_pipeline_healthy_time = time.monotonic()

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._last_odom_time = time.monotonic()
            self._odom_fresh = True

    def _on_estop(self, msg: BoolMsg):
        if msg.data:
            self.get_logger().error("收到急停信号")
            with self._lock:
                self._estop = True

    # ------------------------------------------------------------------ #
    # 硬件初始化
    # ------------------------------------------------------------------ #
    def _init_hardware(self):
        """初始化机械臂、摄像头、检测器、跟踪器和记忆模块。"""
        cfg = self.cfg
        if self.dry_run:
            self.get_logger().info("dry_run=True，跳过机械臂和摄像头初始化")
            # 即使 dry_run 也创建检测器/跟踪器/记忆，以便后续流程可复用
            self.detector = BlockDetection({**cfg["detection"]})
            cfg_g = cfg["grasp"]
            self.tracker = TargetTracker(
                avg_window=int(cfg_g["distance_avg_window"]),
                lost_frames_max=int(cfg_g["lost_frames_max"]),
            )
            self.memory = InspectionMemory(default_zone=cfg["inspection"]["default_zone"])
            return

        try:
            self.arm = ArmController(
                device=cfg["hardware"]["arm_serial_port"],
                cfg={**cfg["arm"], "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]},
            )
        except Exception as e:
            self.get_logger().error("机械臂初始化失败: %s" % (e))
            raise

        # 2026-08-15: 为 block_align 创建独立的 detector（支持颜色过滤）
        if self.target_color:
            self.get_logger().info(f"BlockDetection 目标颜色过滤: {self.target_color}")
            # block_align 专用检测器，启用颜色过滤
            from utils.BlockDetection import BlockDetection as BlockDetectionFiltered
            self.ba_detector = BlockDetectionFiltered({**cfg["detection"]}, target_color=self.target_color)
        else:
            # 无颜色过滤时复用原检测器
            self.ba_detector = BlockDetection({**cfg["detection"]})

        self.detector = BlockDetection({**cfg["detection"]})
        cfg_g = cfg["grasp"]
        self.tracker = TargetTracker(
            avg_window=int(cfg_g["distance_avg_window"]),
            lost_frames_max=int(cfg_g["lost_frames_max"]),
        )
        self.memory = InspectionMemory(default_zone=cfg["inspection"]["default_zone"])

        # 摄像头懒打开：STANDBY 时不占用 /dev/video0，把设备让给 block_align 对齐节点，
        # 进入 DETECTING 阶段再 open，抓完 release
        self._cam_device = cfg["hardware"]["arm_cam_device"]

    def _ensure_arm_cam_open(self):
        if self.dry_run:
            return
        if self.arm_cam is not None:
            return
        # 2026-08-15: 合并 block_align 后，摄像头在 BLOCK_ALIGNING 阶段已打开，
        # DETECTING 阶段无需等待，直接使用。如果摄像头未打开（enable_block_align=False），
        # 则按原逻辑打开。
        if not self.enable_block_align:
            # 原逻辑：等待 block_align 子进程退出（已废弃）
            self.get_logger().info("等待 2s，确保 block_align 释放摄像头...")
            time.sleep(2.0)
        self.arm_cam = _open_camera(self._cam_device, logger=self.get_logger())
        if self.arm_cam is None:
            raise RuntimeError(f"机械臂摄像头打开失败: {self._cam_device}")

    def _release_arm_cam(self):
        if self.arm_cam is None:
            return
        try:
            self.arm_cam.release()
        except Exception as e:
            self.get_logger().warning("释放摄像头时异常: %s" % (e,))
        self.arm_cam = None

    # ------------------------------------------------------------------ #
    # 工具方法
    # ------------------------------------------------------------------ #
    def _publish_state(self, state: str):
        self._current_state = state
        self._state_pub.publish(String(data=state))
        self.get_logger().info("state -> %s" % (state))

    def _republish_state(self):
        self._state_pub.publish(String(data=self._current_state))

    def _check_estop(self) -> bool:
        with self._lock:
            return self._estop

    def _fail(self, reason: str) -> bool:
        self.get_logger().error("任务失败: %s" % (reason))
        self._publish_state(f"ERROR:{reason}")
        self._result_pub.publish(BoolMsg(data=False))
        return False

    def _reset_command(self):
        """重置 pose_control 原点，避免多次 /move 漂移。"""
        self._cmd_pub.publish(String(data="reset_origin"))
        self.get_logger().info("发布 %s: reset_origin" % (self._command_topic))
        time.sleep(0.2)

    # ------------------------------------------------------------------ #
    # 视觉检测
    # ------------------------------------------------------------------ #
    def _apply_roi(self, frame):
        """
        对画面应用 ROI 裁剪，只限制横向（左右两侧），纵向高度保持完整。
        返回：裁剪后的 frame, (x_offset, y_offset)
        """
        if not self.use_roi:
            return frame, (0, 0)

        h, w = frame.shape[:2]
        roi_w = int(w * self.roi_center_ratio)
        roi_h = h  # 纵向不做限制，保持全高度
        x_offset = (w - roi_w) // 2
        y_offset = 0  # 从顶部开始

        roi_frame = frame[y_offset:y_offset + roi_h, x_offset:x_offset + roi_w]
        return roi_frame, (x_offset, y_offset)

    def _restore_roi_coords(self, candidates: list, offset: tuple) -> list:
        """
        将 ROI 内检测到的坐标还原到原始画面坐标系。
        candidates: 在 ROI 内检测到的结果列表
        offset: (x_offset, y_offset)
        返回：坐标还原后的结果列表
        """
        if not self.use_roi or not candidates:
            return candidates

        x_off, y_off = offset
        restored = []
        for c in candidates:
            c_copy = c.copy()
            # 还原 bbox
            (x1, y1), (x2, y2) = c_copy["bbox"]
            c_copy["bbox"] = ((x1 + x_off, y1 + y_off), (x2 + x_off, y2 + y_off))
            # pos_3d 不需要调整（3D坐标系与图像坐标系独立）
            restored.append(c_copy)
        return restored

    def _detect_stable(self) -> dict:
        """
        多帧检测，TargetTracker 滑动均值稳定后返回稳定目标读数。
        返回 dict 或 None（超时）。
        """
        if self.dry_run:
            self.get_logger().info("dry_run: 模拟检测成功")
            return {
                "color": "red",
                "bbox": ((0, 0), (1, 1)),
                "center_offset_x": 0,
                "distance_mm": 300.0,
                "pos_3d": (0.0, 300.0, 0.0),
            }

        timeout = float(self.cfg["grasp"]["detect_timeout"])
        deadline = time.monotonic() + timeout

        # 首次启动时输出 ROI 状态
        if self.use_roi:
            self.get_logger().info(
                f"开始视觉识别，超时 {timeout:.1f}s（ROI 已启用：中心 {self.roi_center_ratio*100:.0f}%）"
            )
        else:
            self.get_logger().info(f"开始视觉识别，超时 {timeout:.1f}s")

        while time.monotonic() < deadline:
            if self._check_estop():
                return None

            ret, frame = self.arm_cam.read()
            if not ret:
                self.get_logger().warning("摄像头读帧失败，跳过")
                continue

            # 应用 ROI 裁剪（仅在 grasp_task 检测阶段，避免左右两侧干扰色块）
            roi_frame, roi_offset = self._apply_roi(frame)

            # 在 ROI 区域内检测
            candidates = self.detector.detect_all(roi_frame)

            # 还原坐标到原始画面（用于可视化和后续处理）
            candidates = self._restore_roi_coords(candidates, roi_offset)

            # 当检测到多个同色候选时，记录选择逻辑（帮助调试）
            if len(candidates) > 1:
                x_values = [c["pos_3d"][0] for c in candidates]
                self.get_logger().info(
                    f"检测到 {len(candidates)} 个候选色块，X_cam 值={x_values}，"
                    f"将锁定最右边的 (X_cam={max(x_values):.1f}mm)"
                )

            self.tracker.update(candidates)

            if self.cv_show:
                # 可视化显示 tracker 当前锁定的目标（最右边的），而非 candidates[0]
                vis_result = self.tracker.get_current_target()
                vis = self.detector.visualize(frame.copy(), vis_result)

                # 在可视化上绘制 ROI 边界（绿色矩形）
                if self.use_roi:
                    h, w = frame.shape[:2]
                    x_off, y_off = roi_offset
                    roi_w = int(w * self.roi_center_ratio)
                    roi_h = int(h * self.roi_center_ratio)
                    cv2.rectangle(vis, (x_off, y_off),
                                (x_off + roi_w, y_off + roi_h),
                                (0, 255, 0), 2)

                cv2.imshow("arm_cam", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return None

            stable = self.tracker.get_stable_target()
            if stable is not None:
                self.get_logger().info(
                    "目标锁定稳定: dist=%.1fmm offset_x=%d" % (stable["distance_mm"], stable["center_offset_x"])
                )
                return stable

        self.get_logger().error("视觉识别超时 (%.1fs)，未检测到稳定色块" % (timeout))
        return None

    # ------------------------------------------------------------------ #
    # 横向对齐
    # ------------------------------------------------------------------ #
    def _align_laterally(self, stable: dict) -> bool:
        """
        根据 X_cam 偏移发布 /move，让 pose_control 驱动机器狗横向对齐。

        修复 2026-08-14：
        1. tracker 只在运动停止后重建一次，之后连续多帧积累保证滑动平均稳定性
        2. 强制锁定最右边的目标，避免多个同色方块时跟踪错误
        参考 block_align_node._do_lateral_align 实现
        """
        cfg_g = self.cfg["grasp"]
        thr_mm = float(cfg_g["align_offset_threshold_mm"])
        max_rounds = 5
        X_cam = stable["pos_3d"][0]

        # 运动后重新检测的控制标志（与 block_align_node 对齐）
        settle_tracker_ready = False

        for round_i in range(max_rounds):
            if self._check_estop():
                return False

            if abs(X_cam) <= thr_mm:
                self.get_logger().info("横向已对齐: X_cam=%.1fmm (阈值=%.1fmm)" % (X_cam, thr_mm))
                return True

            # 不发 reset_origin：/move 本身会在 pose_controller 里以当前位姿为
            # start_pose 并清零 _integral_*（见 pose_controller_node._move_cb）；
            # 而 reset_origin 走 /pose_control/command 与 /move 走两个话题，
            # 顺序不保证。若 reset_origin 后到，会把 _target 清空、state=idle，
            # 刚发的 /move 立即被取消 → cmd_vel 一直近零 → motion_waiter
            # 15s 超时 → ALIGN_FAILED。与 block_align_node 保持一致（见其
            # _do_lateral_align 内注释）。
            self._motion_waiter.reset()

            # 发布横向移动：ROS 约定 Pose2D.y 正=左移；相机侧 X_cam>0 表示物块在图像/视野右侧，
            # 要把它拉到画面中心，狗需要向右移动 → y 取正号（修正：极性与 block_align 相反）。
            msg = Pose2D(x=0.0, y=X_cam / 1000.0, theta=0.0)
            self._move_pub.publish(msg)
            self.get_logger().info("第 %d 轮横向对齐: y=%.3fm (X_cam=%.1fmm)" % (round_i + 1, msg.y, X_cam))

            if not self._motion_waiter.wait_for_stop():
                self.get_logger().error("等待横向到位超时")
                return False

            # 修复：运动停止后重新检测，tracker 只在"刚停下"时重建一次，
            # 之后连续多帧 update 才能凑够 stable_frames；每次都重建的话
            # 计数永远从 0 开始，永远不会稳定
            new_stable = self._detect_stable_after_move(settle_tracker_ready, cfg_g)
            if new_stable is None:
                self.get_logger().error("对齐后重新识别失败")
                return False

            # 重建后首次稳定，下一轮不再重建
            settle_tracker_ready = True

            X_cam = new_stable["pos_3d"][0]
            stable.update(new_stable)

        self.get_logger().error("横向对齐超过最大轮次 (%d)，仍未对齐 X_cam=%.1fmm" % (max_rounds, X_cam))
        return False

    def _detect_stable_after_move(self, tracker_ready: bool, cfg_g: dict) -> dict:
        """
        运动停止后重新检测稳定目标，确保 tracker 连续性和锁定最右边目标。

        Args:
            tracker_ready: True 表示 tracker 已重建过，继续使用；False 表示需要重建
            cfg_g: grasp 配置字典

        Returns:
            稳定检测结果 dict，或 None（超时/失败）
        """
        # 只在 tracker 未就绪时重建一次（与 block_align_node 逻辑一致）
        if not tracker_ready:
            self.tracker = TargetTracker(
                avg_window=int(cfg_g["distance_avg_window"]),
                lost_frames_max=int(cfg_g["lost_frames_max"]),
            )

        # 调用原有检测逻辑，获取稳定目标
        stable = self._detect_stable()
        if stable is None:
            return None

        # 修复：确保锁定最右边的目标（参考 block_align_node._do_wait_detect）
        # 场景：两个同色方块同框时，tracker 可能锁定了靠中心的（不是最右的）
        # 解决：检测到稳定目标后，如果当前帧有多个候选且 tracker 没锁定最右的，重新锁定
        ret, frame = self.arm_cam.read()
        if ret and frame is not None:
            # 应用 ROI（与 _detect_stable 内部逻辑保持一致）
            roi_frame, roi_offset = self._apply_roi(frame)
            all_candidates = self.detector.detect_all(roi_frame)
            all_candidates = self._restore_roi_coords(all_candidates, roi_offset)

            if len(all_candidates) >= 2:
                rightmost = max(all_candidates, key=lambda r: r["pos_3d"][0])
                # 判断 tracker 当前锁定的是否是最右的（5mm 容差）
                if stable["pos_3d"][0] < rightmost["pos_3d"][0] - 5.0:
                    self.get_logger().info(
                        "横向对齐检测到更右的目标 (%.1fmm > %.1fmm)，重新锁定" % (
                            rightmost["pos_3d"][0], stable["pos_3d"][0]))
                    # 重建 tracker 并锁定最右的
                    self.tracker = TargetTracker(
                        avg_window=int(cfg_g["distance_avg_window"]),
                        lost_frames_max=int(cfg_g["lost_frames_max"]),
                    )
                    self.tracker.update([rightmost])
                    # 递归调用，等待新 tracker 稳定（标记为已重建）
                    return self._detect_stable_after_move(True, cfg_g)

        return stable

    # ------------------------------------------------------------------ #
    # block_align 合并逻辑 (2026-08-15)
    # ------------------------------------------------------------------ #
    def _ba_detect_frame(self, frame) -> dict:
        """单帧检测，使用 ba_detector（支持颜色过滤），返回稳定结果或 None。"""
        candidates = self.ba_detector.detect_all(frame)

        # 当检测到多个同色候选时，记录选择逻辑（帮助调试）
        if len(candidates) > 1:
            x_values = [c["pos_3d"][0] for c in candidates]
            self.get_logger().info(
                f"[block_align] 检测到 {len(candidates)} 个候选色块，X_cam 值={x_values}，"
                f"将锁定最右边的 (X_cam={max(x_values):.1f}mm)"
            )

        self.tracker.update(candidates)

        if self.cv_show:
            locked = self.tracker.get_current_target()
            draw = locked if locked is not None else (candidates[0] if candidates else None)
            vis = self.ba_detector.visualize(frame.copy(), draw)
            cv2.putText(vis, "BLOCK_ALIGNING", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("arm_cam", vis)
            cv2.waitKey(1)

        return self.tracker.get_stable_target()

    def _ba_is_cmd_vel_zero(self) -> bool:
        """判断 pose_controller 是否已停止运动（复用 motion_waiter 逻辑）。"""
        now = time.monotonic()
        started = self._ba_cmdvel_motion_started

        # 兜底：发出指令后超过 motion_stop_timeout_s 仍未检测到运动，视为已完成
        if (not started
                and self._ba_last_move_time > 0.0
                and now - self._ba_last_move_time > self._motion_stop_timeout_s):
            if not self._ba_move_ignored_warned:
                self.get_logger().warning(
                    "[block_align] 运动指令 %.1fs 内未见非零 cmd_vel，按已完成放行" %
                    self._motion_stop_timeout_s)
                self._ba_move_ignored_warned = True
            return True

        # 检查 motion_waiter 是否已停止
        return self._motion_waiter.is_stopped()

    def _ba_check_motion_watchdog(self) -> bool:
        """运动链路看门狗：检查 /cmd_vel 或 /leg_odom2 是否超时未更新。"""
        if not self._ba_phase_busy:
            return False

        now = time.monotonic()
        elapsed = now - self._ba_last_pipeline_healthy_time

        if elapsed > self.block_align_watchdog_timeout:
            self.get_logger().error(
                f"[block_align] 运动链路看门狗触发：{elapsed:.1f}s 未收到更新，"
                f"pose_controller 可能异常")
            return True

        return False

    def _ba_send_move(self, x: float, y: float, theta_deg: float):
        """发布 /move 指令（block_align 阶段）。"""
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta_deg)
        self._move_pub.publish(msg)
        self._ba_cmdvel_motion_started = False
        self._ba_last_move_time = time.monotonic()
        self._ba_move_ignored_warned = False
        self._ba_last_pipeline_healthy_time = time.monotonic()
        self.get_logger().info(
            "[block_align] 发布 /move  x=%.3f  y=%.3f  theta=%.1f°" % (x, y, theta_deg))

    def _ba_reset_pose_controller(self):
        """重置 pose_controller（发送 cancel 命令）。"""
        self._cmd_pub.publish(String(data="cancel"))
        self.get_logger().warning("[block_align] 发送 cancel 命令重置 pose_controller")
        time.sleep(1.0)  # 恢复等待
        self._ba_cmdvel_motion_started = False
        self._ba_move_ignored_warned = False
        self._ba_last_pipeline_healthy_time = time.monotonic()

    def _do_block_align_wait_detect(self) -> dict:
        """block_align 等待检测阶段：多帧检测直到稳定或超时。"""
        deadline = time.monotonic() + self.block_align_detect_timeout

        self.get_logger().info(
            f"[block_align] 开始色块检测，超时 {self.block_align_detect_timeout:.1f}s"
            + (f"（目标颜色={self.target_color}）" if self.target_color else ""))

        while time.monotonic() < deadline:
            if self._check_estop():
                return None

            ret, frame = self.arm_cam.read()
            if not ret:
                self.get_logger().warning("[block_align] 摄像头读帧失败，跳过")
                continue

            stable = self._ba_detect_frame(frame)
            if stable is None:
                continue

            # 确保锁定最右边的目标（防止多个同色方块时锁定错误）
            all_candidates = self.ba_detector.detect_all(frame)
            if len(all_candidates) >= 2:
                rightmost = max(all_candidates, key=lambda r: r["pos_3d"][0])
                if stable["pos_3d"][0] < rightmost["pos_3d"][0] - 5.0:  # 5mm 容差
                    self.get_logger().info(
                        f"[block_align] 检测到更右的目标 ({rightmost['pos_3d'][0]:.1f}mm > "
                        f"{stable['pos_3d'][0]:.1f}mm)，重新锁定")
                    # 重建 tracker 并锁定最右的
                    cfg_g = self.cfg["grasp"]
                    self.tracker = TargetTracker(
                        avg_window=int(cfg_g["distance_avg_window"]),
                        lost_frames_max=int(cfg_g["lost_frames_max"]),
                    )
                    self.tracker.update([rightmost])
                    continue

            X_cam, Y_cam, _ = stable["pos_3d"]
            self.get_logger().info(
                f"[block_align] 色块稳定锁定: X_cam={X_cam:.1f}mm  Y_cam={Y_cam:.1f}mm  "
                f"dist={stable['distance_mm']:.1f}mm")
            return stable

        self.get_logger().warning(
            f"[block_align] 检测超时（{self.block_align_detect_timeout:.1f}s），未检测到色块")
        return None

    def _do_block_align_lateral(self, stable: dict) -> bool:
        """block_align 横向对齐阶段。"""
        self._ba_lat_rounds = 0
        self._ba_phase_busy = False
        self._ba_settle_tracker_ready = False

        max_rounds = self.block_align_max_rounds
        thr_mm = self.block_align_lat_thr_mm

        for round_i in range(max_rounds):
            if self._check_estop():
                return False

            X_cam = stable["pos_3d"][0]

            if abs(X_cam) <= thr_mm:
                self.get_logger().info(
                    f"[block_align] 横向对齐完成: X_cam={X_cam:.1f}mm (阈值={thr_mm}mm)")
                return True

            # 发布横向移动
            y_move = self.block_align_lat_polarity * X_cam / 1000.0
            self.get_logger().info(
                f"[block_align] 横向对齐轮次 {round_i + 1}: X_cam={X_cam:.1f}mm → y={y_move:.3f}m")

            self._motion_waiter.reset()
            self._ba_send_move(0.0, y_move, 0.0)
            self._ba_phase_busy = True

            # 等待运动完成
            while not self._ba_is_cmd_vel_zero():
                if self._ba_check_motion_watchdog():
                    self._ba_reset_pose_controller()
                    self._ba_phase_busy = False
                    break
                time.sleep(0.1)

            if not self._motion_waiter.wait_for_stop():
                self.get_logger().error("[block_align] 等待横向到位超时")
                return False

            # 运动停止后重新检测
            if not self._ba_settle_tracker_ready:
                cfg_g = self.cfg["grasp"]
                self.tracker = TargetTracker(
                    avg_window=int(cfg_g["distance_avg_window"]),
                    lost_frames_max=int(cfg_g["lost_frames_max"]),
                )
                self._ba_settle_tracker_ready = True

            new_stable = self._detect_stable()
            if new_stable is None:
                self.get_logger().error("[block_align] 对齐后重新识别失败")
                return False

            stable.update(new_stable)
            self._ba_phase_busy = False

        self.get_logger().error(
            f"[block_align] 横向对齐超过最大轮次 ({max_rounds})，X_cam={stable['pos_3d'][0]:.1f}mm")
        return False

    def _do_block_align_approach(self, stable: dict) -> bool:
        """block_align 前进接近阶段。"""
        self._ba_app_rounds = 0
        self._ba_phase_busy = False
        self._ba_settle_tracker_ready = False

        max_rounds = self.block_align_max_rounds
        target_dist = self.block_align_target_dist
        dist_thr = self.block_align_dist_thr

        for round_i in range(max_rounds):
            if self._check_estop():
                return False

            Y_cam = stable["pos_3d"][1]  # mm
            dist_m = Y_cam / 1000.0
            delta = dist_m - target_dist

            if abs(delta) <= dist_thr:
                self.get_logger().info(
                    f"[block_align] 前进接近完成: Y_cam={Y_cam:.1f}mm  dist={dist_m:.3f}m")
                return True

            # 阻尼 + 限幅
            damped_delta = delta * 0.7
            damped_delta = max(-0.15, min(0.15, damped_delta))

            if abs(damped_delta) < 0.02:
                self.get_logger().info(
                    f"[block_align] 前进接近完成（剩余误差 {delta:.3f}m < 2cm）")
                return True

            self.get_logger().info(
                f"[block_align] 前进接近轮次 {round_i + 1}: Y_cam={Y_cam:.1f}mm  "
                f"delta={delta:.3f}m → 阻尼后={damped_delta:.3f}m")

            self._motion_waiter.reset()
            self._ba_send_move(damped_delta, 0.0, 0.0)
            self._ba_phase_busy = True

            # 等待运动完成
            while not self._ba_is_cmd_vel_zero():
                if self._ba_check_motion_watchdog():
                    self._ba_reset_pose_controller()
                    self._ba_phase_busy = False
                    break
                time.sleep(0.1)

            if not self._motion_waiter.wait_for_stop():
                self.get_logger().error("[block_align] 等待前进到位超时")
                return False

            # 运动停止后重新检测
            if not self._ba_settle_tracker_ready:
                cfg_g = self.cfg["grasp"]
                self.tracker = TargetTracker(
                    avg_window=int(cfg_g["distance_avg_window"]),
                    lost_frames_max=int(cfg_g["lost_frames_max"]),
                )
                self._ba_settle_tracker_ready = True

            new_stable = self._detect_stable()
            if new_stable is None:
                self.get_logger().error("[block_align] 接近后重新识别失败")
                return False

            stable.update(new_stable)
            self._ba_phase_busy = False

        self.get_logger().error(
            f"[block_align] 前进接近超过最大轮次 ({max_rounds})，dist={dist_m:.3f}m")
        return False

    # ------------------------------------------------------------------ #
    # 接近与抓取
    # ------------------------------------------------------------------ #
    def _approach_and_grasp(self, stable: dict) -> bool:
        if self.dry_run:
            self.get_logger().info("dry_run: 模拟抓取成功")
            return True

        cfg_g = self.cfg["grasp"]
        arm = self.arm

        cv2.destroyAllWindows()

        clearance = float(cfg_g["approach_clearance_mm"])
        h_object = float(cfg_g["h_object"])
        dist_offset = float(cfg_g.get("distance_offset_mm", 0.0))

        X_cam, Y_cam, Z_cam = stable["pos_3d"]
        dis_target = Y_cam + dist_offset
        dis_safe = max(dis_target - clearance, 30.0)

        self.get_logger().info(
            "物块坐标（相机系）: X=%.1fmm Y=%.1fmm Z=%.1fmm" % (X_cam, Y_cam, Z_cam)
        )
        self.get_logger().info(
            "IK 输入: dis_safe=%.1fmm -> dis=%.1fmm, h=%.1fmm" % (dis_safe, dis_target, h_object)
        )

        from utils.RobotArm.three_Inverse_kinematics import Arm as IKArm

        # 步骤 1：移到安全距离，下降到目标高度
        self.get_logger().info("步骤1: dis=%.1fmm h=%.1fmm" % (dis_safe, h_object))
        ok = arm.grap(dis_safe, h_object)
        if not ok:
            self.get_logger().error("步骤1 IK 解超出范围 (dis=%.1f h=%.1f)" % (dis_safe, h_object))
            return False
        a3, a4, a5 = IKArm(dis_safe, h_object)
        arm.wait_for_position({3: a3, 4: a4, 5: a5})

        # 步骤 2：前进并抓取
        self.get_logger().info("步骤2: dis=%.1fmm h=%.1fmm" % (dis_target, h_object))
        success = arm.grasp_with_verify(dis=dis_target, height=h_object)
        if success:
            self.get_logger().info("抓取成功")
        else:
            self.get_logger().error("抓取失败（已重试 %s 次）" % (cfg_g.get("grasp_retry_max", 3)))
        return success

    # ------------------------------------------------------------------ #
    # 放置
    # ------------------------------------------------------------------ #
    def _place(self) -> bool:
        if self.dry_run:
            self.get_logger().info("dry_run: 模拟放置成功")
            return True

        arm = self.arm
        memory = self.memory
        cfg_p = self.cfg["placement"]

        zone = memory.get_zone()
        zone_cfg = cfg_p["zones"].get(zone)
        if zone_cfg is None:
            self.get_logger().error("未知放置区: %s" % (zone))
            return False

        dis = float(zone_cfg["dis"])
        height = float(zone_cfg["height"])
        self.get_logger().info("放置到 %s 区 (dis=%.1fmm, height=%.1fmm)" % (zone, dis, height))

        try:
            from utils.RobotArm.three_Inverse_kinematics import Arm as IKArm
            ok = arm.grap(dis, height, keep_gripper=True)
            if not ok:
                self.get_logger().error("放置 IK 解超出范围")
                return False
            a3, a4, a5 = IKArm(dis, height)
            arm.wait_for_position({3: a3, 4: a4, 5: a5})
            time.sleep(float(cfg_p.get("lower_timeout", 2.0)))
            arm.open_gripper()
            self.get_logger().info("已放置，夹爪已张开")
            return True
        except Exception as e:
            self.get_logger().error("放置失败: %s" % (e))
            return False

    # ------------------------------------------------------------------ #
    # 8-phase 状态机（2026-08-15: 增加 BLOCK_ALIGNING 阶段）
    # ------------------------------------------------------------------ #
    def run_state_machine(self):
        """主线程运行的抓取流程，合并 block_align 逻辑。"""
        self._publish_state("INIT")

        if not self.dry_run and self.arm is not None:
            self.arm.set_pose(0)
            self.arm.set_pose(2)

        self._publish_state("STANDBY")
        self.get_logger().info("进入待命，等待 %s 信号..." % (self._start_topic))
        self._start_event.wait()
        if self._check_estop():
            return self._fail("ESTOP")

        # ── 新增: BLOCK_ALIGNING 阶段 ────────────────────────────────────
        if self.enable_block_align:
            self._publish_state("BLOCK_ALIGNING")
            try:
                self._ensure_arm_cam_open()
            except Exception as e:
                return self._fail(f"CAM_OPEN_FAILED:{e}")

            # 重置 tracker（用于 block_align 检测）
            cfg_g = self.cfg["grasp"]
            self.tracker = TargetTracker(
                avg_window=int(cfg_g["distance_avg_window"]),
                lost_frames_max=int(cfg_g["lost_frames_max"]),
            )
            self._ba_last_pipeline_healthy_time = time.monotonic()

            # 1) 等待检测
            stable = self._do_block_align_wait_detect()
            if stable is None:
                self.get_logger().warning(
                    "[block_align] 检测超时，跳过对齐，直接进入抓取阶段")
                # 不报错，允许盲抓
            else:
                # 2) 横向对齐
                if not self._do_block_align_lateral(stable):
                    return self._fail("BLOCK_ALIGN_LATERAL_FAILED")

                # 3) 前进接近
                if not self._do_block_align_approach(stable):
                    return self._fail("BLOCK_ALIGN_APPROACH_FAILED")

            self.get_logger().info("[block_align] 对齐完成，进入检测抓取阶段")

        # ── phase_2: detect ──────────────────────────────────────────────
        self._publish_state("DETECTING")
        try:
            self._ensure_arm_cam_open()
        except Exception as e:
            return self._fail(f"CAM_OPEN_FAILED:{e}")

        # 重置 tracker（用于精确抓取检测，不使用颜色过滤）
        cfg_g = self.cfg["grasp"]
        self.tracker = TargetTracker(
            avg_window=int(cfg_g["distance_avg_window"]),
            lost_frames_max=int(cfg_g["lost_frames_max"]),
        )

        stable = self._detect_stable()
        if stable is None:
            return self._fail("DETECT_TIMEOUT")

        # phase_3: align
        self._publish_state("ALIGNING")
        if self.enable_lateral_align:
            if not self._align_laterally(stable):
                return self._fail("ALIGN_FAILED")
        else:
            self.get_logger().info("横向对齐已禁用，跳过对齐阶段")

        # phase_4: grasp
        self._publish_state("GRASPING")
        if not self._approach_and_grasp(stable):
            return self._fail("GRASP_FAILED")

        # 抓取完成后释放摄像头，让 letter_place_align 或后续流程可以自由使用
        self._release_arm_cam()

        # phase_5: transport
        self._publish_state("TRANSPORT")
        if not self.dry_run and self.arm is not None:
            self.arm.set_pose(3, keep_gripper=True)

        # phase_6: place
        self._publish_state("PLACING")
        self.get_logger().info("等待 %s 信号..." % (self._place_topic))
        self._place_event.wait()
        if self._check_estop():
            return self._fail("ESTOP")
        if not self._place():
            return self._fail("PLACE_FAILED")

        # phase_7: home
        self._publish_state("DONE")
        if not self.dry_run and self.arm is not None:
            self.arm.set_pose(0)
        self._result_pub.publish(BoolMsg(data=True))
        self.get_logger().info("任务完成")
        return True

    # ------------------------------------------------------------------ #
    # 资源释放
    # ------------------------------------------------------------------ #
    def finalize(self):
        """释放摄像头和机械臂资源。退出前先让机械臂回初始姿态并张开夹爪。

        为防止用户连按 Ctrl+C 打断复位流程（KeyboardInterrupt 继承自
        BaseException，普通 try/except Exception 抓不到），进入本函数后
        暂时忽略 SIGINT/SIGTERM，跑完再恢复。每一步都单独 try，一步炸掉
        不影响后一步——最关键的是最后 arm.finalize() 里的 emergency_stop
        必须发出，才能断力矩防舵机过热。
        """
        self.get_logger().info("释放资源...")

        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        prev_sigterm = signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if not self.dry_run:
                try:
                    self._release_arm_cam()
                except BaseException as e:
                    self.get_logger().warning("释放摄像头异常: %s" % (e,))
                try:
                    cv2.destroyAllWindows()
                except BaseException:
                    pass

                if self.arm is not None:
                    self.get_logger().info("安全复位：张开夹爪 + 回初始姿态 + 断力矩")
                    # 每一步都独立 try——open_gripper 挂掉不影响 set_pose，
                    # set_pose 挂掉也一定要走到 arm.finalize() 里的 emergency_stop
                    try:
                        self.arm.open_gripper()
                    except BaseException as e:
                        self.get_logger().warning("open_gripper 异常: %s" % (e,))
                    try:
                        time.sleep(0.3)
                    except BaseException:
                        pass
                    try:
                        self.arm.set_pose(0)
                    except BaseException as e:
                        self.get_logger().warning("set_pose(0) 异常: %s" % (e,))
                    try:
                        time.sleep(2.5)   # 等舵机走完初始姿态
                    except BaseException:
                        pass
                    try:
                        # arm.finalize 已改造成"先 emergency_stop 再关串口"，
                        # 即使前面 set_pose 没到位，这里也会把六路舵机断力矩防过热
                        self.arm.finalize()
                    except BaseException as e:
                        self.get_logger().warning("arm.finalize 异常: %s" % (e,))
        finally:
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    # 多轮循环支持：默认 max_rounds=1 保持原单轮行为；abcd_task 里覆写为 4
    try:
        max_rounds = int(node.get_parameter("max_rounds").value)
    except Exception:
        max_rounds = 1
    max_rounds = max(1, max_rounds)
    try:
        inter_wait = float(node.get_parameter("inter_round_wait_s").value)
    except Exception:
        inter_wait = 0.5
    inter_wait = max(0.0, inter_wait)

    try:
        for round_idx in range(max_rounds):
            if not rclpy.ok():
                break
            if max_rounds > 1:
                node.get_logger().info(
                    "=== round %d / %d ===" % (round_idx + 1, max_rounds))
            # 每轮开始前清空 Event，防止上一轮 /grasp/start、/grasp/place 的
            # 缓存立刻触发新一轮；同时 tools 侧 InspectionMemory 保留 zone。
            node._start_event.clear()
            node._place_event.clear()
            node.run_state_machine()
            if round_idx + 1 < max_rounds and rclpy.ok():
                time.sleep(inter_wait)
    except KeyboardInterrupt:
        node.get_logger().warning("用户中断，执行安全归位")
    except Exception as e:
        node.get_logger().error("状态机异常: %s" % (e))
    finally:
        node.finalize()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        exec_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
