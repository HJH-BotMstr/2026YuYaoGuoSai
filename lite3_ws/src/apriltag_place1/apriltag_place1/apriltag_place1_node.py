#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag place1 对齐节点

流程：
  phase_0_wait_trigger  等待外部触发 /apriltag_place1/start
  phase_1_wait_detect   等待摄像头就绪，连续多帧检测到目标 Tag
  phase_2_yaw_align     旋转机身，消除水平角偏差
  phase_3_lateral_align 横向平移，使 Tag 正对摄像头
  phase_4_approach      前进到 target_distance_m
  phase_5_final_check   最终校验（角度 + 横向 + 距离同时达标）
  phase_6_emit_signal   发布 /grasp/start，通知 grasp 模块

依赖：
  - pupil-apriltags  (pip3 install pupil-apriltags)
  - opencv-python
  - rclpy, geometry_msgs, std_msgs
"""

import math
import time
import threading
from collections import deque
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, String

# ─── 状态常量 ───────────────────────────────────────────────────────────────── #
STATE_WAIT_TRIGGER   = "wait_trigger"
STATE_WAIT_DETECT    = "wait_detect"
STATE_YAW_ALIGN      = "yaw_align"
STATE_LATERAL_ALIGN  = "lateral_align"
STATE_APPROACH       = "approach"
STATE_FINAL_CHECK    = "final_check"
STATE_DONE           = "done"
STATE_ERROR          = "error"


class AprilTagPlace1Node(Node):

    def __init__(self):
        super().__init__("apriltag_place1_node")

        # ── 参数声明 ────────────────────────────────────────────────────────── #
        self.declare_parameter("trigger_topic",           "/apriltag_place1/start")
        self.declare_parameter("camera_device",           "/dev/video4")
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
        self.declare_parameter("target_distance_m",       0.20)
        self.declare_parameter("yaw_align_threshold_deg", 3.0)
        self.declare_parameter("lateral_threshold_m",     0.03)
        self.declare_parameter("distance_threshold_m",    0.02)
        self.declare_parameter("max_rounds",              5)
        self.declare_parameter("stable_frames",           10)
        self.declare_parameter("detect_timeout_s",        10.0)
        self.declare_parameter("cmd_vel_zero_timeout_s",  0.5)
        self.declare_parameter("move_timeout_s",          10.0)

        # ── 读取参数 ────────────────────────────────────────────────────────── #
        self._trigger_topic      = self.get_parameter("trigger_topic").value
        self._cam_device         = self.get_parameter("camera_device").value
        self._img_w              = self.get_parameter("image_width").value
        self._img_h              = self.get_parameter("image_height").value
        self._fps                = self.get_parameter("fps").value
        raw_cm                   = self.get_parameter("camera_matrix").value
        raw_dc                   = self.get_parameter("dist_coeffs").value
        self._cam_mtx            = np.array(raw_cm, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs        = np.array(raw_dc, dtype=np.float64)
        self._tag_family         = self.get_parameter("tag_family").value
        self._target_tag_id      = self.get_parameter("target_tag_id").value
        self._tag_size_m         = self.get_parameter("tag_size_m").value
        self._target_dist        = self.get_parameter("target_distance_m").value
        self._yaw_thr_deg        = self.get_parameter("yaw_align_threshold_deg").value
        self._lat_thr            = self.get_parameter("lateral_threshold_m").value
        self._dist_thr           = self.get_parameter("distance_threshold_m").value
        self._max_rounds         = self.get_parameter("max_rounds").value
        self._stable_frames      = self.get_parameter("stable_frames").value
        self._detect_timeout     = self.get_parameter("detect_timeout_s").value
        self._cmdvel_zero_t      = self.get_parameter("cmd_vel_zero_timeout_s").value
        self._move_timeout       = self.get_parameter("move_timeout_s").value

        # ── 内参便捷提取 ────────────────────────────────────────────────────── #
        self._fx = float(self._cam_mtx[0, 0])
        self._fy = float(self._cam_mtx[1, 1])
        self._cx = float(self._cam_mtx[0, 2])
        self._cy = float(self._cam_mtx[1, 2])

        # ── AprilTag 检测器 ──────────────────────────────────────────────────── #
        try:
            from pupil_apriltags import Detector
            self._detector = Detector(
                families=self._tag_family,
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
            self.get_logger().info("pupil_apriltags Detector 初始化成功，family=%s", self._tag_family)
        except ImportError:
            self.get_logger().fatal("未找到 pupil_apriltags，请 pip3 install pupil-apriltags")
            raise

        # ── 摄像头 ──────────────────────────────────────────────────────────── #
        self._cap: Optional[cv2.VideoCapture] = None
        self._open_camera()

        # ── ROS 通信 ─────────────────────────────────────────────────────────── #
        self._sub_trigger = self.create_subscription(
            Bool, self._trigger_topic, self._trigger_cb, 10)
        self._sub_cmdvel  = self.create_subscription(
            Twist, "/cmd_vel", self._cmdvel_cb, 10)

        self._pub_move    = self.create_publisher(Pose2D,  "/move",                 10)
        self._pub_cmd     = self.create_publisher(String,  "/pose_control/command", 10)
        self._pub_grasp   = self.create_publisher(Bool,    "/grasp/start",          10)

        # ── 状态 ─────────────────────────────────────────────────────────────── #
        self._state         = STATE_WAIT_TRIGGER
        self._stable_buf    = deque(maxlen=self._stable_frames)
        self._lock          = threading.Lock()

        # cmd_vel 近期记录（用于判断运动是否停止）
        self._cmdvel_history: deque = deque(maxlen=30)

        # 主循环定时器：10 Hz
        self._timer = self.create_timer(0.1, self._main_loop)
        self.get_logger().info("apriltag_place1_node 已启动，等待触发信号: %s", self._trigger_topic)

    # ──────────────────────────── 摄像头 ────────────────────────────────────── #

    def _open_camera(self):
        self.get_logger().info("打开摄像头: %s", self._cam_device)
        cap = cv2.VideoCapture(self._cam_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error("摄像头打开失败: %s", self._cam_device)
            self._cap = None
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._img_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._img_h)
        cap.set(cv2.CAP_PROP_FPS,          self._fps)
        ret, frame = cap.read()
        if not ret or frame is None:
            self.get_logger().error("摄像头打开成功但无法读帧: %s", self._cam_device)
            cap.release()
            self._cap = None
            return
        self._cap = cap
        self.get_logger().info("摄像头已就绪: %s  帧大小=%s", self._cam_device, frame.shape)

    # ──────────────────────────── 回调 ──────────────────────────────────────── #

    def _trigger_cb(self, msg: Bool):
        with self._lock:
            if msg.data:
                if self._state in (STATE_WAIT_TRIGGER, STATE_ERROR):
                    self.get_logger().info("收到触发信号，进入 wait_detect")
                    self._state = STATE_WAIT_DETECT
                    self._stable_buf.clear()
                    self._detect_deadline = time.monotonic() + self._detect_timeout
            else:
                if self._state not in (STATE_WAIT_TRIGGER, STATE_DONE):
                    self.get_logger().info("收到取消信号，停止运动，回到 wait_trigger")
                    self._send_move(0.0, 0.0, 0.0)
                    self._state = STATE_WAIT_TRIGGER

    def _cmdvel_cb(self, msg: Twist):
        speed = (abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z))
        self._cmdvel_history.append((time.monotonic(), speed))

    # ──────────────────────────── 检测 ──────────────────────────────────────── #

    def _read_frame(self) -> Optional[np.ndarray]:
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        if not ret or frame is None:
            return None
        return frame

    def _detect_tag(self, frame: np.ndarray) -> Optional[dict]:
        """返回目标 Tag 的位姿，或 None。"""
        undist = cv2.undistort(frame, self._cam_mtx, self._dist_coeffs)
        grey   = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        tags   = self._detector.detect(
            grey,
            estimate_tag_pose=True,
            camera_params=[self._fx, self._fy, self._cx, self._cy],
            tag_size=self._tag_size_m,
        )
        for tag in tags:
            if tag.tag_id == self._target_tag_id:
                t = tag.pose_t.flatten()   # [tx, ty, tz] in camera frame
                R = tag.pose_R             # 3x3 rotation matrix
                return {"tx": float(t[0]), "ty": float(t[1]), "tz": float(t[2]), "R": R}
        return None

    # ──────────────────────────── 运动指令 ──────────────────────────────────── #

    def _send_move(self, x: float, y: float, theta_deg: float):
        msg = Pose2D()
        msg.x     = float(x)
        msg.y     = float(y)
        msg.theta = float(theta_deg)
        self._pub_move.publish(msg)
        self.get_logger().info("发布 /move  x=%.3f  y=%.3f  theta=%.1f°", x, y, theta_deg)

    def _reset_origin(self):
        msg = String()
        msg.data = "reset_origin"
        self._pub_cmd.publish(msg)
        self.get_logger().info("发布 reset_origin")

    def _emit_place1(self):
        msg = Bool()
        msg.data = True
        self._pub_grasp.publish(msg)
        self.get_logger().info("发布 /grasp/start = True")

    # ──────────────────────────── 运动完成判断 ──────────────────────────────── #

    def _is_cmd_vel_zero(self) -> bool:
        """判断最近 cmd_vel_zero_timeout_s 内速度是否持续接近零。"""
        now = time.monotonic()
        cutoff = now - self._cmdvel_zero_t
        recent = [(t, v) for (t, v) in self._cmdvel_history if t >= cutoff]
        if not recent:
            return False
        return all(v < 0.01 for (_, v) in recent)

    def _wait_motion_done(self, timeout: Optional[float] = None) -> bool:
        """阻塞等待机器人运动完成（/cmd_vel 速度降为零）。"""
        if timeout is None:
            timeout = self._move_timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._is_cmd_vel_zero():
                return True
            time.sleep(0.05)
        self.get_logger().warning("等待运动完成超时 (%.1fs)", timeout)
        return False

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

        # 以下阶段需要摄像头，先确保摄像头就绪
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
        if time.monotonic() > self._detect_deadline:
            self.get_logger().error("phase_1 超时，未检测到 Tag id=%d，回到 wait_trigger",
                                    self._target_tag_id)
            with self._lock:
                self._state = STATE_WAIT_TRIGGER
            return

        pose = self._detect_tag(frame)
        if pose is None:
            return

        if self._is_stable(pose):
            self.get_logger().info(
                "Tag 稳定锁定: tx=%.3f  tz=%.3f", pose["tx"], pose["tz"])
            with self._lock:
                self._last_pose  = pose
                self._yaw_rounds = 0
                self._lat_rounds = 0
                self._app_rounds = 0
                self._stable_buf.clear()
                self._state = STATE_YAW_ALIGN
                self._phase_busy = False

    # ──────────────────────────── phase_2 ───────────────────────────────────── #

    def _do_yaw_align(self, frame: np.ndarray):
        if getattr(self, "_phase_busy", False):
            if not self._is_cmd_vel_zero():
                return
            # 运动停止，重新检测
            pose = self._detect_tag(frame)
            if pose is None:
                self.get_logger().warning("yaw_align: Tag 丢失，回到 wait_detect")
                with self._lock:
                    self._stable_buf.clear()
                    self._detect_deadline = time.monotonic() + self._detect_timeout
                    self._state = STATE_WAIT_DETECT
                return
            self._last_pose = pose
            self._phase_busy = False

        pose  = self._last_pose
        tx, tz = pose["tx"], pose["tz"]
        if tz <= 0.0:
            self.get_logger().warning("yaw_align: tz=%.3f 异常，跳过", tz)
            return

        alpha_rad = math.atan2(tx, tz)
        alpha_deg = math.degrees(alpha_rad)

        if abs(alpha_deg) <= self._yaw_thr_deg:
            self.get_logger().info("yaw_align 完成: alpha=%.2f°", alpha_deg)
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_LATERAL_ALIGN
                self._phase_busy = False
            return

        if self._yaw_rounds >= self._max_rounds:
            self.get_logger().error("yaw_align 超过最大轮次，放弃")
            with self._lock:
                self._state = STATE_ERROR
            return

        self._yaw_rounds += 1
        self.get_logger().info(
            "yaw_align 轮次 %d: alpha=%.2f°，发送旋转指令", self._yaw_rounds, alpha_deg)
        self._reset_origin()
        # ROS 约定：theta 逆时针为正，Tag 偏右(alpha>0)需要狗向右转(负)
        self._send_move(0.0, 0.0, -alpha_deg)
        self._phase_busy = True

    # ──────────────────────────── phase_3 ───────────────────────────────────── #

    def _do_lateral_align(self, frame: np.ndarray):
        if getattr(self, "_phase_busy", False):
            if not self._is_cmd_vel_zero():
                return
            pose = self._detect_tag(frame)
            if pose is None:
                self.get_logger().warning("lateral_align: Tag 丢失，回到 wait_detect")
                with self._lock:
                    self._stable_buf.clear()
                    self._detect_deadline = time.monotonic() + self._detect_timeout
                    self._state = STATE_WAIT_DETECT
                return
            self._last_pose = pose
            self._phase_busy = False

        pose = self._last_pose
        tx   = pose["tx"]

        if abs(tx) <= self._lat_thr:
            self.get_logger().info("lateral_align 完成: tx=%.3fm", tx)
            with self._lock:
                self._stable_buf.clear()
                self._state = STATE_APPROACH
                self._phase_busy = False
                self._app_step   = 1
            return

        if self._lat_rounds >= self._max_rounds:
            self.get_logger().error("lateral_align 超过最大轮次，放弃")
            with self._lock:
                self._state = STATE_ERROR
            return

        self._lat_rounds += 1
        self.get_logger().info(
            "lateral_align 轮次 %d: tx=%.3fm，发送横移指令", self._lat_rounds, tx)
        # /move y 正方向为左移；tx 相机坐标右正，故取负
        self._send_move(0.0, -tx, 0.0)
        self._phase_busy = True

    # ──────────────────────────── phase_4 ───────────────────────────────────── #

    def _do_approach(self, frame: np.ndarray):
        if getattr(self, "_phase_busy", False):
            if not self._is_cmd_vel_zero():
                return
            pose = self._detect_tag(frame)
            if pose is None:
                self.get_logger().warning("approach: Tag 丢失，回到 wait_detect")
                with self._lock:
                    self._stable_buf.clear()
                    self._detect_deadline = time.monotonic() + self._detect_timeout
                    self._state = STATE_WAIT_DETECT
                return
            self._last_pose = pose
            self._phase_busy = False

        pose = self._last_pose
        tz   = pose["tz"]

        if getattr(self, "_app_step", 1) == 1:
            # 步骤1：前进到 target + 0.05 m
            interim_target = self._target_dist + 0.05
            delta = tz - interim_target
            if delta > 0.01:
                self.get_logger().info("approach step1: tz=%.3fm  delta=%.3fm", tz, delta)
                self._send_move(delta, 0.0, 0.0)
                self._phase_busy = True
                self._app_step   = 2
                return
            else:
                self._app_step = 2

        # 步骤2：精确前进到 target_distance_m
        delta = tz - self._target_dist
        if abs(delta) <= self._dist_thr:
            self.get_logger().info("approach 完成: tz=%.3fm", tz)
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
            "approach 轮次 %d: tz=%.3fm  delta=%.3fm", self._app_rounds, tz, delta)
        self._send_move(delta, 0.0, 0.0)
        self._phase_busy = True

    # ──────────────────────────── phase_5 ───────────────────────────────────── #

    def _do_final_check(self, frame: np.ndarray):
        pose = self._detect_tag(frame)
        if pose is None:
            self.get_logger().warning("final_check: Tag 丢失，回到 wait_detect")
            with self._lock:
                self._stable_buf.clear()
                self._detect_deadline = time.monotonic() + self._detect_timeout
                self._state = STATE_WAIT_DETECT
            return

        tx, tz = pose["tx"], pose["tz"]
        alpha_deg = math.degrees(math.atan2(tx, tz)) if tz > 0 else 999.0

        yaw_ok  = abs(alpha_deg) <= self._yaw_thr_deg
        lat_ok  = abs(tx)        <= self._lat_thr
        dist_ok = abs(tz - self._target_dist) <= self._dist_thr

        if yaw_ok and lat_ok and dist_ok:
            self._stable_buf.append({"tx": tx, "tz": tz})
        else:
            self._stable_buf.clear()
            self.get_logger().debug(
                "final_check 未全达标: yaw=%s(%.1f°) lat=%s(%.3fm) dist=%s(%.3fm)",
                yaw_ok, alpha_deg, lat_ok, tx, dist_ok, tz - self._target_dist)
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
            self.get_logger().info(
                "final_check 通过！tx=%.3fm  tz=%.3fm  alpha=%.2f°", tx, tz, alpha_deg)
            self._emit_place1()
            with self._lock:
                self._state = STATE_DONE

    # ──────────────────────────── 析构 ──────────────────────────────────────── #

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
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
