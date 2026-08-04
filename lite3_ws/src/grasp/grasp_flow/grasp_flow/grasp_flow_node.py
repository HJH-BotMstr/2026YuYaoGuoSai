#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取全流程编排节点。

任务流程：
  1. WAIT_DOG_READY     等待机械狗进入自动模式（/leg_odom2 有新鲜数据，
                        即 lite3_driver 已启动并完成唤醒序列）
  2. WAIT_ARM_STANDBY   等待 grasp_task 机械臂进入准备姿态（/grasp/state == STANDBY）
  3. BLOCK_ALIGN        拉起 block_align 节点并触发色块对齐；
                        对齐完成它会自动发 /grasp/start
  4. GRASPING           监视 grasp_task 抓取，直到 /grasp/state == TRANSPORT（运输姿态）
  5. WAIT_MANUAL_LETTER 提示人工搬运到放置点，命令行输入放置字母(A/B/C/D)后
                        拉起 letter_place_align 节点并触发放置对齐
  6. LETTER_PLACING     letter_place_align 对齐完成自动发 /grasp/place，
                        监视 grasp_task 放置直到 /grasp/result
  7. DONE / ERROR       终态

两个对齐节点（block_align / letter_place_align）由本节点按需拉起与关闭，
保证摄像头设备与 /move 指令总线在任意时刻只有一个对齐节点占用。
"""

import os
import queue
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

VALID_LETTERS = ("A", "B", "C", "D")


class GraspFlowNode(Node):

    # 状态名
    ST_WAIT_DOG = "WAIT_DOG_READY"
    ST_WAIT_ARM = "WAIT_ARM_STANDBY"
    ST_BLOCK_ALIGN = "BLOCK_ALIGN"
    ST_GRASPING = "GRASPING"
    ST_WAIT_LETTER = "WAIT_MANUAL_LETTER"
    ST_PLACING = "LETTER_PLACING"
    ST_DONE = "DONE"
    ST_ERROR = "ERROR"

    # grasp_task 收到 /grasp/start 后进入的状态（说明色块对齐触发已生效）。
    # dry_run 下 DETECTING/ALIGNING/GRASPING 可能瞬间冲过，主循环读到时已是
    # TRANSPORT/PLACING，故后续状态也算"已接管"
    GRASP_ACTIVE_STATES = ("DETECTING", "ALIGNING", "GRASPING",
                           "TRANSPORT", "PLACING", "DONE")

    def __init__(self):
        super().__init__("grasp_flow_node")

        # ── 参数 ──────────────────────────────────────────────────────────── #
        self.declare_parameter("odom_topic", "/leg_odom2")
        self.declare_parameter("grasp_state_topic", "/grasp/state")
        self.declare_parameter("grasp_result_topic", "/grasp/result")
        self.declare_parameter("block_align_trigger_topic", "/block_align/start")
        self.declare_parameter("letter_trigger_topic", "/letter_place/start")
        self.declare_parameter("odom_fresh_timeout_s", 1.0)
        self.declare_parameter("block_align_timeout_s", 240.0)
        self.declare_parameter("grasp_timeout_s", 300.0)
        self.declare_parameter("letter_place_timeout_s", 600.0)
        self.declare_parameter("enable_prompt", True)
        self.declare_parameter("manage_align_nodes", True)

        gp = self.get_parameter
        self._odom_fresh_s = gp("odom_fresh_timeout_s").value
        self._block_align_timeout_s = gp("block_align_timeout_s").value
        self._grasp_timeout_s = gp("grasp_timeout_s").value
        self._letter_timeout_s = gp("letter_place_timeout_s").value
        self._enable_prompt = gp("enable_prompt").value
        self._manage = gp("manage_align_nodes").value

        # ── 订阅 ──────────────────────────────────────────────────────────── #
        self.create_subscription(
            Odometry, gp("odom_topic").value, self._odom_cb, 10)
        self.create_subscription(
            String, gp("grasp_state_topic").value, self._grasp_state_cb, 10)
        self.create_subscription(
            Bool, gp("grasp_result_topic").value, self._grasp_result_cb, 10)

        # ── 发布 ──────────────────────────────────────────────────────────── #
        self._pub_block_align = self.create_publisher(
            Bool, gp("block_align_trigger_topic").value, 10)
        self._pub_letter = self.create_publisher(
            String, gp("letter_trigger_topic").value, 10)

        # ── 运行时状态 ────────────────────────────────────────────────────── #
        self._state = self.ST_WAIT_DOG
        self._state_since = self._now()
        self._last_odom_time = None
        self._grasp_state = ""
        self._grasp_result = None          # None / True / False（LETTER_PLACING 内有效）
        self._error_reason = ""
        self._error_retriable = False
        self._last_heartbeat = 0.0
        self._last_trigger_pub = 0.0       # 触发话题重发节拍
        self._letter_pub_until = 0.0       # 字母触发持续重发截止时刻

        self._input_queue = queue.Queue()
        self._procs = {}                   # key -> subprocess.Popen

        if self._enable_prompt:
            threading.Thread(target=self._input_loop, daemon=True).start()

        self.create_timer(0.1, self._main_loop)
        self.get_logger().info("grasp_flow 编排节点已启动，等待机械狗进入自动模式 …")

    # ═══════════════════════════ 回调 ═══════════════════════════════════════ #

    def _odom_cb(self, _msg):
        self._last_odom_time = self._now()

    def _grasp_state_cb(self, msg):
        if msg.data != self._grasp_state:
            self.get_logger().info(f"grasp_task 状态: {msg.data}")
        self._grasp_state = msg.data

    def _grasp_result_cb(self, msg):
        # 只在放置监控阶段采信，避免历史消息干扰
        if self._state == self.ST_PLACING:
            self._grasp_result = msg.data

    def _input_loop(self):
        while rclpy.ok():
            try:
                line = input()
            except EOFError:
                return
            except Exception:
                return
            self._input_queue.put(line.strip().upper())

    # ═══════════════════════════ 工具 ═══════════════════════════════════════ #

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set_state(self, state):
        self.get_logger().info(f"══ 流程状态: {self._state} → {state}")
        self._state = state
        self._state_since = self._now()
        self._last_heartbeat = 0.0

    def _elapsed(self):
        return self._now() - self._state_since

    def _heartbeat(self, text, period=3.0):
        if self._now() - self._last_heartbeat >= period:
            self._last_heartbeat = self._now()
            self.get_logger().info(text)

    def _odom_fresh(self):
        return (self._last_odom_time is not None
                and self._now() - self._last_odom_time <= self._odom_fresh_s)

    def _fail(self, reason, retriable=False):
        self._error_reason = reason
        self._error_retriable = retriable
        self.get_logger().error(f"流程失败: {reason}")
        self._kill_all()
        self._set_state(self.ST_ERROR)

    def _drain_input(self):
        """清空启动阶段等历史输入。"""
        while True:
            try:
                self._input_queue.get_nowait()
            except queue.Empty:
                return

    def _poll_input(self):
        try:
            return self._input_queue.get_nowait()
        except queue.Empty:
            return None

    # ── 对齐节点进程管理 ───────────────────────────────────────────────────── #

    def _spawn(self, key, package, executable, config_name):
        if not self._manage:
            return
        share = get_package_share_directory(package)
        params = os.path.join(share, "config", config_name)
        cmd = ["ros2", "run", package, executable,
               "--ros-args", "--params-file", params]
        self.get_logger().info(f"拉起对齐节点: {' '.join(cmd)}")
        self._procs[key] = subprocess.Popen(cmd, preexec_fn=os.setsid)

    def _kill(self, key):
        proc = self._procs.pop(key, None)
        if proc is None:
            return
        if proc.poll() is None:
            self.get_logger().info(f"关闭对齐节点进程组: {key} (pid={proc.pid})")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f"{key} 未响应 SIGINT，强制 SIGKILL")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5.0)
            except ProcessLookupError:
                pass

    def _kill_all(self):
        for key in list(self._procs):
            self._kill(key)

    # ═══════════════════════════ 主状态机 ═══════════════════════════════════ #

    def _main_loop(self):
        handler = {
            self.ST_WAIT_DOG: self._st_wait_dog,
            self.ST_WAIT_ARM: self._st_wait_arm,
            self.ST_BLOCK_ALIGN: self._st_block_align,
            self.ST_GRASPING: self._st_grasping,
            self.ST_WAIT_LETTER: self._st_wait_letter,
            self.ST_PLACING: self._st_placing,
            self.ST_DONE: self._st_done,
            self.ST_ERROR: self._st_error,
        }[self._state]
        handler()

    def _st_wait_dog(self):
        if self._odom_fresh():
            self.get_logger().info("里程计数据正常，机械狗已就绪（自动模式）")
            self._set_state(self.ST_WAIT_ARM)
            return
        self._heartbeat("等待机械狗进入自动模式（lite3_driver 启动后自动唤醒）…")

    def _st_wait_arm(self):
        if self._grasp_state == "STANDBY":
            self.get_logger().info("机械臂已进入准备姿态，启动色块对齐")
            self._spawn("block_align", "block_align",
                        "block_align_node", "block_align.yaml")
            self._set_state(self.ST_BLOCK_ALIGN)
            return
        if self._grasp_state.startswith("ERROR"):
            self._fail(f"grasp_task 初始化失败: {self._grasp_state}")
            return
        self._heartbeat("等待 grasp_task 机械臂进入准备姿态(STANDBY) …")

    def _st_block_align(self):
        # 周期重发触发，覆盖 block_align 节点刚拉起订阅未就绪的窗口；
        # 节点处于活动态时会忽略重复触发，无副作用
        if self._now() - self._last_trigger_pub >= 1.0:
            self._last_trigger_pub = self._now()
            self._pub_block_align.publish(Bool(data=True))

        if self._grasp_state in self.GRASP_ACTIVE_STATES:
            self.get_logger().info(
                "色块对齐完成，grasp_task 已接管，关闭 block_align 节点释放摄像头")
            self._kill("block_align")
            self._set_state(self.ST_GRASPING)
            return
        if self._grasp_state.startswith("ERROR"):
            self._kill("block_align")
            self._fail(f"grasp_task 异常: {self._grasp_state}")
            return
        if self._elapsed() > self._block_align_timeout_s:
            self._pub_block_align.publish(Bool(data=False))  # 取消 block_align 侧状态机
            self._kill("block_align")
            self._fail("色块对齐超时（详见 block_align 节点日志）")

    def _st_grasping(self):
        # PLACING 也算抓取完成：grasp_task 已越过 TRANSPORT（运输姿态）
        # 进入等待 /grasp/place 阶段；dry_run 下 TRANSPORT 可能一闪而过
        if self._grasp_state in ("TRANSPORT", "PLACING"):
            self.get_logger().info(
                "抓取完成，机械臂已切换运输姿态。请人工搬运机械狗到放置点。")
            self._drain_input()
            self._set_state(self.ST_WAIT_LETTER)
            return
        if self._grasp_state.startswith("ERROR"):
            self._fail(f"抓取失败: {self._grasp_state}")
            return
        if self._elapsed() > self._grasp_timeout_s:
            self._fail("抓取阶段超时")
            return
        self._heartbeat(f"grasp_task 抓取中（{self._grasp_state}）…")

    def _st_wait_letter(self):
        if not self._enable_prompt:
            self._heartbeat("请人工发布 /letter_place/start 触发放置对齐")
        else:
            self._heartbeat(
                "搬运到位后，在此终端输入放置字母 A/B/C/D 并回车开始放置对齐"
                "（输入 q 中止任务）")
        cmd = self._poll_input()
        if cmd is None:
            return
        if cmd == "Q":
            self._fail("人工中止")
            return
        if cmd in VALID_LETTERS:
            self._letter = cmd
            self.get_logger().info(f"收到放置字母 {cmd}，启动放置对齐")
            self._spawn("letter", "letter_place_align",
                        "letter_place_align_node", "letter_place_align.yaml")
            # letter 节点拉起需十秒级（加载依赖+打开摄像头），触发以 2Hz
            # 持续重发 30 秒保证到达；节点活动态会忽略重复触发，无副作用
            self._letter_pub_until = self._now() + 30.0
            self._grasp_result = None
            self._set_state(self.ST_PLACING)
        else:
            self.get_logger().warning(f"无效输入 {cmd!r}，请输入 A/B/C/D（q 中止）")

    def _st_placing(self):
        if (self._now() < self._letter_pub_until
                and self._now() - self._last_trigger_pub >= 0.5):
            self._last_trigger_pub = self._now()
            self._pub_letter.publish(String(data=self._letter))

        if self._grasp_result is True:
            self.get_logger().info("放置完成，任务全流程结束 ✔")
            self._kill("letter")
            self._set_state(self.ST_DONE)
            return
        if self._grasp_result is False or self._grasp_state.startswith("ERROR"):
            reason = self._grasp_state if self._grasp_state.startswith("ERROR") \
                else "grasp_task 放置失败（/grasp/result=False）"
            self._fail(f"{reason}；如需重试需重启 grasp_task 节点")
            return
        if self._elapsed() > self._letter_timeout_s:
            self._fail("放置对齐/放置超时（letter_place_align 可能未锁定目标，"
                       "输入 r 可重新触发放置对齐）", retriable=True)
            return

        cmd = self._poll_input()
        if cmd == "Q":
            # 任意非 ABCD 值会让 letter_place_align 取消回 wait_trigger
            self._pub_letter.publish(String(data="X"))
            self._kill("letter")
            self._drain_input()
            self._set_state(self.ST_WAIT_LETTER)
            return
        self._heartbeat(f"放置对齐/放置进行中（grasp_task: {self._grasp_state}）…")

    def _st_done(self):
        self._heartbeat("任务已完成。可按 Ctrl+C 退出。", period=30.0)

    def _st_error(self):
        if self._error_retriable:
            self._heartbeat(
                f"流程处于 ERROR（{self._error_reason}）。输入 r 重新触发放置对齐，"
                "或 Ctrl+C 退出。")
            if self._poll_input() == "R":
                self._drain_input()
                self._set_state(self.ST_WAIT_LETTER)
        else:
            self._heartbeat(
                f"流程处于 ERROR（{self._error_reason}）。请排查后重启。", period=30.0)

    # ═══════════════════════════ 析构 ═══════════════════════════════════════ #

    def destroy_node(self):
        self._kill_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
