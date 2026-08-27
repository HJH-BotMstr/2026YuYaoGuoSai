#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
abcd_task_node — ABCD 四块物块抓取放置全流程编排。

2026-08-15 修改：移除 block_align 子进程启动逻辑，该功能已合并到 grasp_task。

顶层状态机，逐字母循环调用现有下层模块：
  1. WaypointNavigator（本包）—— 单段绝对里程计导航
  2. apriltag_place1_node —— AprilTag 视觉对齐（每字母不同 tag_id）
  3. grasp_task grasp_node —— 机械臂抓取（内含 block_align 逻辑，本包 launch 里以 max_rounds=4 拉起）

单轮子状态（对每个字母执行一遍）：
  NAV_TO_TASK           从中转点导航到该字母的 task_point
  SET_TAG_ID            通过 /apriltag_place1_node/set_parameters 切换 target_tag_id
  TAG_ALIGN             发 /apriltag_place1/start=True，等 /apriltag_place1/done=True
  WAIT_GRASP_TRANSPORT  发 /grasp/start=True（1Hz），等 /grasp/state == TRANSPORT
  RETREAT               后退 retreat_dist_m
  NAV_TO_TRANSIT        导航回中转点
  NAV_TO_TASK_2         再次导航到 task_point（放置准备）
  SIGNAL_PLACE          发 /grasp/place=<letter>（2Hz × 5s）
  WAIT_PLACE_RESULT     等 /grasp/result=True
  NAV_BACK_TO_TRANSIT   返回中转点

全局：INIT → 对每个字母执行单轮 → ALL_DONE / ERROR

对外接口：
  sub  /grasp/state         String   —— 观察 grasp_task 是否 TRANSPORT
  sub  /grasp/result        Bool     —— 观察 grasp_task 放置是否成功
  sub  /apriltag_place1/done Bool    —— 观察 apriltag_place1 是否对齐完成
  sub  /leg_odom2 /cmd_vel（由 WaypointNavigator 内部订阅）
  pub  /apriltag_place1/start Bool
  pub  /grasp/start         Bool     —— 触发 grasp_task（包含 block_align）
  pub  /grasp/place         String
  pub  /move  /pose_control/command（由 WaypointNavigator 内部）
  cli  /apriltag_place1_node/set_parameters （rcl_interfaces/SetParameters）
"""

import os
# 2026-08-15: 移除不再使用的 signal/subprocess/ament_index（block_align 已合并到 grasp_task）
# import signal
# import subprocess
import threading
import time
from typing import Dict, List, Optional

import yaml

import rclpy
# 2026-08-15: get_package_share_directory 仅在 block_align 子进程代码中用到，已一并移除
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .waypoint_nav import WaypointNavigator


VALID_LETTERS = ("A", "B", "C", "D")


class AbcdTaskNode(Node):
    """ABCD 四轮抓取放置顶层编排。"""

    # ── 全局状态 ─────────────────────────────────────────────────────────── #
    G_INIT      = "INIT"
    G_RUN       = "RUN_LETTER"
    G_ALL_DONE  = "ALL_DONE"
    G_ERROR     = "ERROR"

    # ── 单轮子状态（仅用于日志/心跳） ────────────────────────────────────── #
    # 注：SET_TAG_ID 已合并到全局 INIT 阶段（tag_id 全流程唯一，不再每轮切换）
    # 2026-08-15: 移除 START_BLOCK_ALIGN 和 KILL_BLOCK_ALIGN 状态
    R_NAV_TO_TASK          = "NAV_TO_TASK"
    R_TAG_ALIGN            = "TAG_ALIGN"
    R_WAIT_GRASP_TRANSPORT = "WAIT_GRASP_TRANSPORT"
    R_RETREAT              = "RETREAT"
    R_NAV_TO_TRANSIT       = "NAV_TO_TRANSIT"
    R_NAV_TO_TASK_2        = "NAV_TO_TASK_2"
    R_SIGNAL_PLACE         = "SIGNAL_PLACE"
    R_WAIT_PLACE_RESULT    = "WAIT_PLACE_RESULT"
    R_NAV_BACK_TO_TRANSIT  = "NAV_BACK_TO_TRANSIT"

    def __init__(self):
        super().__init__("abcd_task_node")

        # ── 参数声明 ─────────────────────────────────────────────────── #
        self.declare_parameter("abcd_config_path",
                               "/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/abcd_task/config/abcd_config.yaml")
        self.declare_parameter("task_order", ["A", "B", "C", "D"])
        self.declare_parameter("start_from", "A")
        self.declare_parameter("skip_on_error", False)
        self.declare_parameter("dry_run_nav", False)

        self.declare_parameter("retreat_dist_m",        0.5)
        self.declare_parameter("nav_timeout_s",         30.0)
        self.declare_parameter("tag_align_timeout_s",   30.0)
        self.declare_parameter("grasp_flow_timeout_s",  300.0)
        self.declare_parameter("place_timeout_s",       120.0)
        self.declare_parameter("odom_fresh_timeout_s",  1.0)
        self.declare_parameter("cmd_vel_zero_duration_s", 1.0)
        self.declare_parameter("inter_state_pause_s",   0.3)
        self.declare_parameter("standby_wait_timeout_s", 60.0)

        # 话题名（可覆盖便于测试）
        self.declare_parameter("topic_apriltag_start", "/apriltag_place1/start")
        self.declare_parameter("topic_apriltag_done",  "/apriltag_place1/done")
        self.declare_parameter("topic_grasp_start",    "/grasp/start")  # 2026-08-15: 新增
        self.declare_parameter("topic_grasp_state",    "/grasp/state")
        self.declare_parameter("topic_grasp_result",   "/grasp/result")
        self.declare_parameter("topic_grasp_place",    "/grasp/place")
        self.declare_parameter("topic_odom",           "/leg_odom2")
        self.declare_parameter("topic_cmd_vel",        "/cmd_vel")
        self.declare_parameter("topic_move",           "/move")
        self.declare_parameter("topic_pose_cmd",       "/pose_control/command")

        self.declare_parameter("apriltag_set_param_service",
                               "/apriltag_place1_node/set_parameters")
        self.declare_parameter("apriltag_set_param_timeout_s", 5.0)

        # 2026-08-15: 移除 block_align 子进程相关参数
        # self.declare_parameter("block_align_package",  "block_align")
        # self.declare_parameter("block_align_executable", "block_align_node")
        # self.declare_parameter("block_align_params_file", "")

        # 放置命令连发窗口（复用 grasp_flow_b 的模式）
        self.declare_parameter("place_signal_rate_hz",     2.0)
        self.declare_parameter("place_signal_duration_s",  5.0)

        # ── 读取参数 ─────────────────────────────────────────────────── #
        self._abcd_config_path  = self.get_parameter("abcd_config_path").value
        self._task_order        = list(self.get_parameter("task_order").value)
        self._start_from        = str(self.get_parameter("start_from").value).upper()
        self._skip_on_error     = bool(self.get_parameter("skip_on_error").value)
        self._dry_run_nav       = bool(self.get_parameter("dry_run_nav").value)

        self._retreat_dist_m    = float(self.get_parameter("retreat_dist_m").value)
        self._nav_timeout_s     = float(self.get_parameter("nav_timeout_s").value)
        self._tag_timeout_s     = float(self.get_parameter("tag_align_timeout_s").value)
        self._grasp_timeout_s   = float(self.get_parameter("grasp_flow_timeout_s").value)
        self._place_timeout_s   = float(self.get_parameter("place_timeout_s").value)
        self._odom_fresh_t      = float(self.get_parameter("odom_fresh_timeout_s").value)
        self._cmdvel_zero_d     = float(self.get_parameter("cmd_vel_zero_duration_s").value)
        self._inter_pause_s     = float(self.get_parameter("inter_state_pause_s").value)
        self._standby_wait_s    = float(self.get_parameter("standby_wait_timeout_s").value)

        self._topic_apriltag_start = self.get_parameter("topic_apriltag_start").value
        self._topic_apriltag_done  = self.get_parameter("topic_apriltag_done").value
        self._topic_grasp_start    = self.get_parameter("topic_grasp_start").value  # 2026-08-15
        self._topic_grasp_state    = self.get_parameter("topic_grasp_state").value
        self._topic_grasp_result   = self.get_parameter("topic_grasp_result").value
        self._topic_grasp_place    = self.get_parameter("topic_grasp_place").value
        self._topic_odom           = self.get_parameter("topic_odom").value
        self._topic_cmd_vel        = self.get_parameter("topic_cmd_vel").value
        self._topic_move           = self.get_parameter("topic_move").value
        self._topic_pose_cmd       = self.get_parameter("topic_pose_cmd").value

        self._apriltag_srv        = self.get_parameter("apriltag_set_param_service").value
        self._apriltag_srv_timeout = float(self.get_parameter("apriltag_set_param_timeout_s").value)

        # 2026-08-15: 移除 block_align 子进程参数读取
        # self._ba_pkg  = self.get_parameter("block_align_package").value
        # self._ba_exec = self.get_parameter("block_align_executable").value
        # self._ba_params_file = self.get_parameter("block_align_params_file").value

        self._place_rate_hz    = float(self.get_parameter("place_signal_rate_hz").value)
        self._place_duration_s = float(self.get_parameter("place_signal_duration_s").value)

        # ── ABCD 配置加载 ─────────────────────────────────────────────── #
        try:
            self._abcd_config = self._load_abcd_config(self._abcd_config_path)
        except Exception as e:
            self.get_logger().fatal(f"加载 abcd_config 失败: {e}")
            raise

        # 校验 task_order + start_from
        self._task_order = [c.upper() for c in self._task_order]
        for c in self._task_order:
            if c not in VALID_LETTERS:
                raise ValueError(f"task_order 含未知字母: {c}")
            if c not in self._abcd_config["letters"]:
                raise ValueError(f"abcd_config 缺少字母配置: {c}")

        if self._start_from not in self._task_order:
            self.get_logger().warning(
                f"start_from={self._start_from} 不在 task_order 中，按第一个字母开始")
            self._start_idx = 0
        else:
            self._start_idx = self._task_order.index(self._start_from)

        # 2026-08-15: 移除 block_align 参数文件推导逻辑
        # if not self._ba_params_file:
        #     try:
        #         share = get_package_share_directory("block_align")
        #         self._ba_params_file = os.path.join(share, "config", "block_align.yaml")
        #     except Exception as e:
        #         self.get_logger().warning(
        #             f"未能定位 block_align share 目录: {e}；子进程将使用节点默认参数")
        #         self._ba_params_file = ""

        # ── 内部状态 ─────────────────────────────────────────────────── #
        self._state_lock = threading.Lock()
        self._state = self.G_INIT
        self._current_letter: Optional[str] = None
        self._current_round_state: str = ""
        self._should_abort = False

        # 抓取/放置事件
        self._grasp_state_lock = threading.Lock()
        self._last_grasp_state: str = ""
        self._grasp_state_history: List[str] = []
        self._grasp_result_lock = threading.Lock()
        self._grasp_result_event = threading.Event()
        self._grasp_result_value: Optional[bool] = None
        # apriltag done：TRANSIENT_LOCAL 从对面发过来，我们本地打时间戳后判定
        self._apriltag_done_lock = threading.Lock()
        self._apriltag_done_seen_at_mono: float = 0.0
        # arm_gate：进入某轮 TAG_ALIGN 之前记录时刻，只承认在此之后的 done
        self._arm_gate_time_mono: float = 0.0

        # ── ROS 通信 ─────────────────────────────────────────────────── #
        self._pub_apriltag_start = self.create_publisher(
            Bool, self._topic_apriltag_start, 10)
        self._pub_grasp_start = self.create_publisher(
            Bool, self._topic_grasp_start, 10)  # 2026-08-15: 新增
        self._pub_grasp_place = self.create_publisher(
            String, self._topic_grasp_place, 10)

        # 订阅
        self.create_subscription(
            String, self._topic_grasp_state, self._on_grasp_state, 10)
        self.create_subscription(
            Bool, self._topic_grasp_result, self._on_grasp_result, 10)
        # apriltag/done 用 latched QoS（TRANSIENT_LOCAL）以匹配发布端
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Bool, self._topic_apriltag_done, self._on_apriltag_done, latched)

        # WaypointNavigator（内部订阅 odom/cmd_vel、发布 /move、/pose_control/command）
        self._nav = WaypointNavigator(
            self,
            topic_move=self._topic_move,
            topic_pose_cmd=self._topic_pose_cmd,
            topic_odom=self._topic_odom,
            topic_cmd_vel=self._topic_cmd_vel,
            odom_fresh_timeout_s=self._odom_fresh_t,
            cmd_vel_zero_duration_s=self._cmdvel_zero_d,
        )

        # SetParameters 客户端（apriltag_place1）
        self._set_param_client = self.create_client(
            SetParameters, self._apriltag_srv)

        # 2026-08-15: 移除子进程管理（不再需要启动 block_align）
        # self._procs: Dict[str, subprocess.Popen] = {}

        # 心跳定时器
        self._heartbeat_last = 0.0
        self.create_timer(1.0, self._heartbeat_cb)

        self.get_logger().info(
            f"abcd_task_node 已初始化，task_order={self._task_order}, "
            f"start_from={self._start_from} (idx={self._start_idx}), "
            f"skip_on_error={self._skip_on_error}, dry_run_nav={self._dry_run_nav}"
        )

    # ─── 配置加载 ────────────────────────────────────────────────────── #

    @staticmethod
    def _load_abcd_config(path: str) -> dict:
        if not os.path.exists(path):
            raise FileNotFoundError(f"abcd_config 不存在: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise ValueError("abcd_config 顶层必须是 dict")
        for key in ("letters", "transit_point", "apriltag_tag_id"):
            if key not in data:
                raise ValueError(f"abcd_config 必须含 '{key}'")

        # 校验 transit_point
        tp = data["transit_point"]
        for k in ("x", "y", "yaw"):
            if k not in tp:
                raise ValueError(f"transit_point 缺字段: {k}")

        # 校验每个字母（tag_id 是全局的，不在此校验）
        for letter, cfg in data["letters"].items():
            for k in ("color", "task_x", "task_y", "task_yaw"):
                if k not in cfg:
                    raise ValueError(f"letters[{letter}] 缺字段: {k}")

        # 校验 tag_id 类型
        try:
            int(data["apriltag_tag_id"])
        except (TypeError, ValueError):
            raise ValueError(
                f"apriltag_tag_id 必须是整数，当前={data['apriltag_tag_id']!r}")

        return data

    # ─── ROS 回调 ────────────────────────────────────────────────────── #

    def _on_grasp_state(self, msg: String) -> None:
        s = msg.data
        with self._grasp_state_lock:
            if s != self._last_grasp_state:
                self.get_logger().info(f"/grasp/state → {s}")
                self._grasp_state_history.append(s)
                if len(self._grasp_state_history) > 32:
                    self._grasp_state_history.pop(0)
            self._last_grasp_state = s

    def _on_grasp_result(self, msg: Bool) -> None:
        with self._grasp_result_lock:
            self._grasp_result_value = bool(msg.data)
            self._grasp_result_event.set()
        self.get_logger().info(f"/grasp/result = {msg.data}")

    def _on_apriltag_done(self, msg: Bool) -> None:
        if not msg.data:
            return
        now = time.monotonic()
        with self._apriltag_done_lock:
            self._apriltag_done_seen_at_mono = now
        self.get_logger().info(f"/apriltag_place1/done = True @ mono={now:.3f}")

    def _heartbeat_cb(self) -> None:
        now = time.monotonic()
        if now - self._heartbeat_last < 5.0:
            return
        self._heartbeat_last = now
        with self._state_lock:
            g = self._state
            letter = self._current_letter
            rst = self._current_round_state
        with self._grasp_state_lock:
            gs = self._last_grasp_state
        self.get_logger().info(
            f"[HB] global={g} letter={letter} round={rst} grasp_state={gs}"
        )

    # ─── 状态与终止 ───────────────────────────────────────────────────── #

    def _set_state(self, g: str) -> None:
        with self._state_lock:
            self._state = g

    def _set_round_state(self, r: str) -> None:
        with self._state_lock:
            self._current_round_state = r
        self.get_logger().info(f"[{self._current_letter}] state → {r}")

    def _abort_requested(self) -> bool:
        with self._state_lock:
            return self._should_abort

    def request_abort(self) -> None:
        with self._state_lock:
            self._should_abort = True

    # 2026-08-15: 移除子进程管理方法（不再需要）
    # def _spawn_block_align(self, target_color: str = "") -> bool:
    # def _kill_proc(self, key: str, grace_s: float = 5.0) -> None:
    # def _kill_all_procs(self) -> None:

    # ─── 状态机执行 ──────────────────────────────────────────────────── #

    def run(self) -> bool:
        """
        主流程：INIT → 对每个字母执行单轮 → ALL_DONE。
        阻塞式，由 main() 在独立线程外调用（executor 在 daemon 线程 spin）。

        返回：True=全部成功；False=中途 ERROR 且未跳过。
        """
        # ── INIT ─────────────────────────────────────────────────────── #
        self._set_state(self.G_INIT)
        if not self._wait_odom_fresh():
            return self._fail_global("里程计未就绪")

        if not self._wait_grasp_standby():
            return self._fail_global("grasp_task 未进入 STANDBY")

        if not self._wait_apriltag_service():
            return self._fail_global("apriltag_place1 SetParameters 服务不可用")

        # 全流程只有一个 tag_id，INIT 阶段一次性写入 apriltag_place1_node
        # （防止 apriltag_place1.yaml 里的默认值被漏改）
        if not self._dry_run_nav:
            tag_id = int(self._abcd_config["apriltag_tag_id"])
            if not self._set_apriltag_target_id(tag_id):
                return self._fail_global(
                    f"启动阶段 SetParameters(target_tag_id={tag_id}) 失败")

        # ── RUN 每个字母 ─────────────────────────────────────────────── #
        self._set_state(self.G_RUN)
        letters = self._task_order[self._start_idx:]
        for letter in letters:
            if self._abort_requested() or not rclpy.ok():
                return self._fail_global("用户中止")

            with self._state_lock:
                self._current_letter = letter

            self.get_logger().info(f"════════ letter {letter} 开始 ════════")
            ok = self._run_letter(letter)
            self.get_logger().info(
                f"════════ letter {letter} {'完成' if ok else '失败'} ════════")

            if not ok:
                if self._skip_on_error:
                    self.get_logger().warning(f"letter {letter} 失败，skip_on_error=True，继续下一个")
                    continue
                return self._fail_global(f"letter {letter} 失败")

            time.sleep(self._inter_pause_s)

        self._set_state(self.G_ALL_DONE)
        self.get_logger().info("═══════════ ABCD 全部完成 ═══════════")
        return True

    def _fail_global(self, reason: str) -> bool:
        self.get_logger().error(f"[GLOBAL ERROR] {reason}")
        self._set_state(self.G_ERROR)
        # 2026-08-15: 不再需要杀掉子进程
        # self._kill_all_procs()
        return False

    # ─── 前置检查 ─────────────────────────────────────────────────────── #

    def _wait_odom_fresh(self, timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not self._abort_requested():
            if self._nav.odom_fresh():
                self.get_logger().info("里程计已就绪")
                return True
            if time.monotonic() > deadline:
                self.get_logger().error("等待里程计超时")
                return False
            time.sleep(0.1)
        return False

    def _wait_grasp_standby(self) -> bool:
        deadline = time.monotonic() + self._standby_wait_s
        while rclpy.ok() and not self._abort_requested():
            with self._grasp_state_lock:
                s = self._last_grasp_state
            if s == "STANDBY":
                self.get_logger().info("grasp_task 已进入 STANDBY")
                return True
            if s.startswith("ERROR"):
                self.get_logger().error(f"grasp_task 报错: {s}")
                return False
            if time.monotonic() > deadline:
                self.get_logger().error(f"等待 grasp_task STANDBY 超时（当前 state={s}）")
                return False
            time.sleep(0.2)
        return False

    def _wait_apriltag_service(self, timeout_s: float = 10.0) -> bool:
        # 若 dry_run_nav=True，跳过 apriltag 相关准备
        if self._dry_run_nav:
            self.get_logger().info("dry_run_nav=True，跳过 apriltag 服务就绪检查")
            return True
        ok = self._set_param_client.wait_for_service(timeout_sec=timeout_s)
        if ok:
            self.get_logger().info(f"apriltag SetParameters 服务已就绪: {self._apriltag_srv}")
        else:
            self.get_logger().error(f"apriltag SetParameters 服务不可用: {self._apriltag_srv}")
        return ok

    # ─── 单轮流程 ─────────────────────────────────────────────────────── #

    def _run_letter(self, letter: str) -> bool:
        cfg = self._abcd_config["letters"][letter]
        transit = self._abcd_config["transit_point"]
        task_point = {
            "x":   float(cfg["task_x"]),
            "y":   float(cfg["task_y"]),
            "yaw": float(cfg["task_yaw"]),
        }

        # 1) NAV_TO_TASK
        self._set_round_state(self.R_NAV_TO_TASK)
        if not self._nav.navigate_to(task_point, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "NAV_TO_TASK 失败")

        # dry_run_nav 模式：跳过 tag/grasp/place，直接跑完导航序列
        if self._dry_run_nav:
            return self._dry_run_letter_tail(letter, transit, task_point)

        # 2) TAG_ALIGN
        # tag_id 已在 INIT 阶段一次性配置，全流程唯一，无需每轮切换
        self._set_round_state(self.R_TAG_ALIGN)
        # 记录 arm_gate：只承认此后的 apriltag done
        self._arm_gate_time_mono = time.monotonic()
        self._pub_apriltag_start.publish(Bool(data=True))
        self.get_logger().info(f"发布 {self._topic_apriltag_start} = True")

        if not self._wait_apriltag_done(self._tag_timeout_s):
            self._pub_apriltag_start.publish(Bool(data=False))
            return self._fail_round(letter, "TAG_ALIGN 超时/失败")

        time.sleep(self._inter_pause_s)

        # 3) WAIT_GRASP_TRANSPORT（直接触发 grasp_task，包含 block_align）
        # 2026-08-15: 移除 START_BLOCK_ALIGN 和 KILL_BLOCK_ALIGN，直接发 /grasp/start
        self._set_round_state(self.R_WAIT_GRASP_TRANSPORT)
        # 从 abcd_config 读取当前字母对应的目标颜色，通过参数传递给 grasp_task
        # (需要在 launch 文件中配置 target_color 参数映射)
        if not self._wait_grasp_transport(self._grasp_timeout_s):
            self._pub_grasp_start.publish(Bool(data=False))
            return self._fail_round(letter, "WAIT_GRASP_TRANSPORT 超时/失败")

        # 4) RETREAT
        self._set_round_state(self.R_RETREAT)
        if not self._nav.move_relative_body(
                dx_body=-self._retreat_dist_m, dy_body=0.0, dtheta_deg=0.0,
                timeout_s=self._nav_timeout_s,
                should_abort=self._abort_requested):
            return self._fail_round(letter, "RETREAT 失败")

        # 8) NAV_TO_TRANSIT
        self._set_round_state(self.R_NAV_TO_TRANSIT)
        if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "NAV_TO_TRANSIT 失败")

        # 9) NAV_TO_TASK_2（放置准备）
        self._set_round_state(self.R_NAV_TO_TASK_2)
        if not self._nav.navigate_to(task_point, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "NAV_TO_TASK_2 失败")

        # 10) SIGNAL_PLACE + 11) WAIT_PLACE_RESULT
        self._set_round_state(self.R_SIGNAL_PLACE)
        self._grasp_result_event.clear()
        self._grasp_result_value = None
        # 2Hz × 5s 连发 /grasp/place（防订阅竞争，参考 grasp_flow_b）
        self._burst_publish_place(letter)

        self._set_round_state(self.R_WAIT_PLACE_RESULT)
        if not self._wait_place_result(self._place_timeout_s):
            return self._fail_round(letter, "WAIT_PLACE_RESULT 失败或超时")

        # 12) NAV_BACK_TO_TRANSIT
        self._set_round_state(self.R_NAV_BACK_TO_TRANSIT)
        if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "NAV_BACK_TO_TRANSIT 失败")

        return True

    def _dry_run_letter_tail(self, letter, transit, task_point) -> bool:
        """dry_run_nav 模式：只跑导航，跳过 tag_align/grasp/place。"""
        self._set_round_state(self.R_RETREAT)
        if not self._nav.move_relative_body(
                -self._retreat_dist_m, 0.0, 0.0,
                timeout_s=self._nav_timeout_s,
                should_abort=self._abort_requested):
            return self._fail_round(letter, "[dry_run] RETREAT 失败")

        self._set_round_state(self.R_NAV_TO_TRANSIT)
        if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "[dry_run] NAV_TO_TRANSIT 失败")

        self._set_round_state(self.R_NAV_TO_TASK_2)
        if not self._nav.navigate_to(task_point, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "[dry_run] NAV_TO_TASK_2 失败")

        self._set_round_state(self.R_NAV_BACK_TO_TRANSIT)
        if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                     should_abort=self._abort_requested):
            return self._fail_round(letter, "[dry_run] NAV_BACK_TO_TRANSIT 失败")

        return True

    def _fail_round(self, letter: str, reason: str) -> bool:
        self.get_logger().error(f"[{letter}] round ERROR: {reason}")
        # 2026-08-15: 不再需要杀掉 block_align 子进程
        self._nav.send_zero_move()
        return False

    # ─── 具体等待/触发实现 ───────────────────────────────────────────── #

    def _set_apriltag_target_id(self, tag_id: int) -> bool:
        """通过 rcl_interfaces/SetParameters 服务修改 apriltag_place1 的 target_tag_id。"""
        req = SetParameters.Request()
        param = Parameter()
        param.name = "target_tag_id"
        param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=int(tag_id))
        req.parameters = [param]

        future = self._set_param_client.call_async(req)
        deadline = time.monotonic() + self._apriltag_srv_timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"SetParameters(target_tag_id={tag_id}) 超时（>{self._apriltag_srv_timeout}s）")
                return False
            if self._abort_requested():
                return False
            time.sleep(0.02)
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"SetParameters 异常: {e}")
            return False
        if resp is None or not resp.results:
            self.get_logger().error("SetParameters 无返回")
            return False
        for r in resp.results:
            if not r.successful:
                self.get_logger().error(f"SetParameters 失败: {r.reason}")
                return False
        self.get_logger().info(f"apriltag target_tag_id 已设为 {tag_id}")
        return True

    def _wait_apriltag_done(self, timeout_s: float) -> bool:
        """
        等待 apriltag_place1 完成对齐。判定标准：/apriltag_place1/done 收到 True
        且时间戳 >= _arm_gate_time_mono（防止拿到上一轮 latched 的旧 True）。
        """
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and not self._abort_requested():
            with self._apriltag_done_lock:
                seen = self._apriltag_done_seen_at_mono
            if seen >= self._arm_gate_time_mono and seen > 0.0:
                self.get_logger().info("TAG_ALIGN 完成")
                return True
            if time.monotonic() > deadline:
                self.get_logger().error(f"TAG_ALIGN 等待超时（>{timeout_s:.1f}s）")
                return False
            time.sleep(0.05)
        return False

    def _wait_grasp_transport(self, timeout_s: float) -> bool:
        """
        等 grasp_task 进入 TRANSPORT 状态（或 PLACING/DONE 视为已冲过）。
        2026-08-15: 改为以 1Hz 重复发 /grasp/start=True（原为 /block_align/start）。
        """
        active = {"TRANSPORT", "PLACING", "DONE"}
        deadline = time.monotonic() + float(timeout_s)
        next_pub = 0.0
        while rclpy.ok() and not self._abort_requested():
            now = time.monotonic()
            if now >= next_pub:
                self._pub_grasp_start.publish(Bool(data=True))
                next_pub = now + 1.0

            with self._grasp_state_lock:
                s = self._last_grasp_state
            if s in active:
                self.get_logger().info(f"grasp_task 已推进到 {s}")
                return True
            if s.startswith("ERROR"):
                self.get_logger().error(f"grasp_task 报错: {s}")
                return False
            if now > deadline:
                self.get_logger().error(f"等待 TRANSPORT 超时（>{timeout_s:.1f}s，last state={s}）")
                return False
            time.sleep(0.1)
        return False

    def _burst_publish_place(self, letter: str) -> None:
        """
        高频重发 /grasp/place = letter，覆盖到 grasp_task 的 _place_event。
        参考 grasp_flow_b 的 2Hz × 5s 兜底节奏。
        """
        end = time.monotonic() + self._place_duration_s
        period = 1.0 / max(0.1, self._place_rate_hz)
        while rclpy.ok() and not self._abort_requested() and time.monotonic() < end:
            self._pub_grasp_place.publish(String(data=letter))
            time.sleep(period)
        self.get_logger().info(
            f"/grasp/place 发送窗口结束（letter={letter}, {self._place_duration_s}s）")

    def _wait_place_result(self, timeout_s: float) -> bool:
        """等 /grasp/result（Bool）。True=放置成功，False=失败。"""
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and not self._abort_requested():
            if self._grasp_result_event.is_set():
                with self._grasp_result_lock:
                    ok = self._grasp_result_value
                if ok is True:
                    self.get_logger().info("放置成功 /grasp/result=True")
                    return True
                self.get_logger().error(f"放置失败 /grasp/result={ok}")
                return False
            # grasp_task 中若出错，会把 state 打成 ERROR:...
            with self._grasp_state_lock:
                s = self._last_grasp_state
            if s.startswith("ERROR"):
                self.get_logger().error(f"grasp_task 报错: {s}")
                return False
            if time.monotonic() > deadline:
                self.get_logger().error(f"WAIT_PLACE_RESULT 超时（>{timeout_s:.1f}s）")
                return False
            time.sleep(0.1)
        return False

    # ─── 析构 ─────────────────────────────────────────────────────────── #

    def destroy_node(self):
        # 2026-08-15: 不再需要杀掉子进程
        super().destroy_node()


# ─── main ─────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = AbcdTaskNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    exit_code = 0
    try:
        ok = node.run()
        if not ok:
            exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warning("用户中断")
        node.request_abort()
    except Exception as e:
        node.get_logger().error(f"顶层异常: {e}")
        exit_code = 2
    finally:
        # 停车（不再需要关闭子进程）
        try:
            node._nav.send_zero_move()
        except Exception:
            pass
        # 2026-08-15: 移除 _kill_all_procs()

        # 先停 executor，再 destroy_node，防止 handle 竞态
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        exec_thread.join(timeout=2.0)

    return exit_code


if __name__ == "__main__":
    main()

