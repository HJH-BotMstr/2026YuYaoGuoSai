#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取全流程编排节点（放置对齐已解耦为外部接口版本）。

与 grasp_flow_node.py 的差异：
  - 完全去掉 letter_place_align：不再拉起字母对齐节点，也不再需要
    命令行选择字母。
  - 放置阶段改由外部接口触发：其它模块判定机械狗到位后，向
    /grasp_flow/place_ready 发一次 std_msgs/Bool(data=true) 即可
    激活机械臂放置操作。
  - 临时占位：外部接口接入前，在本节点所在终端按回车（任意一行输入）
    等同于收到一次外部触发消息，便于手工联调。
  - 放置目标区硬编码为 HARDCODED_LETTER（默认 "B"）。要换字母改此处即可。

任务流程：
  1. WAIT_DOG_READY     等待机械狗进入自动模式
  2. WAIT_ARM_STANDBY   等待 grasp_task 机械臂 STANDBY
  3. BLOCK_ALIGN        拉起 block_align 触发色块对齐
  4. GRASPING           监视 grasp_task 抓取直到 TRANSPORT
  5. WAIT_MANUAL_LETTER 等待 /grasp_flow/place_ready 或终端回车触发放置
  6. LETTER_PLACING     发 /grasp/place（zone=HARDCODED_LETTER），
                        监视 grasp_task 放置直到 /grasp/result
  7. DONE / ERROR       终态
"""

import os
import queue
import signal
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

# 将 tools/grasp 加入路径以便 Ctrl+C 时能调用 ArmController 复位
TOOLS_GRASP = "/home/ysc/2026YuYaoGuoSai/tools/grasp"
if TOOLS_GRASP not in sys.path:
    sys.path.insert(0, TOOLS_GRASP)

VALID_LETTERS = ("A", "B", "C", "D")

# 硬编码放置字母：改这里就换目标区
HARDCODED_LETTER = "B"


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
        super().__init__("grasp_flow_node_b")

        # ── 参数 ──────────────────────────────────────────────────────────── #
        self.declare_parameter("odom_topic", "/leg_odom2")
        self.declare_parameter("grasp_state_topic", "/grasp/state")
        self.declare_parameter("grasp_result_topic", "/grasp/result")
        self.declare_parameter("block_align_trigger_topic", "/block_align/start")
        self.declare_parameter("grasp_place_topic", "/grasp/place")
        self.declare_parameter("place_trigger_topic", "/grasp_flow/place_ready")
        self.declare_parameter("odom_fresh_timeout_s", 1.0)
        self.declare_parameter("block_align_timeout_s", 240.0)
        self.declare_parameter("grasp_timeout_s", 300.0)
        self.declare_parameter("place_timeout_s", 600.0)
        self.declare_parameter("enable_prompt", True)
        self.declare_parameter("manage_align_nodes", True)
        self.declare_parameter("arm_serial_port", "/dev/ttyUSB0")
        self.declare_parameter("tools_config_path",
                               "/home/ysc/2026YuYaoGuoSai/tools/grasp/config.yaml")

        gp = self.get_parameter
        self._odom_fresh_s = gp("odom_fresh_timeout_s").value
        self._block_align_timeout_s = gp("block_align_timeout_s").value
        self._grasp_timeout_s = gp("grasp_timeout_s").value
        self._place_timeout_s = gp("place_timeout_s").value
        self._enable_prompt = gp("enable_prompt").value
        self._manage = gp("manage_align_nodes").value
        self._arm_port = gp("arm_serial_port").value
        self._config_path = gp("tools_config_path").value
        self._place_trigger_topic = gp("place_trigger_topic").value

        # ── 订阅 ──────────────────────────────────────────────────────────── #
        self.create_subscription(
            Odometry, gp("odom_topic").value, self._odom_cb, 10)
        self.create_subscription(
            String, gp("grasp_state_topic").value, self._grasp_state_cb, 10)
        self.create_subscription(
            Bool, gp("grasp_result_topic").value, self._grasp_result_cb, 10)
        # 放置就绪外部接口：未来由到位判定模块发布 True 后触发放置
        self.create_subscription(
            Bool, self._place_trigger_topic, self._place_trigger_cb, 10)

        # ── 发布 ──────────────────────────────────────────────────────────── #
        self._pub_block_align = self.create_publisher(
            Bool, gp("block_align_trigger_topic").value, 10)
        # 放置由本节点直接向 grasp_task 发触发，不再走 letter_place_align
        self._pub_grasp_place = self.create_publisher(
            String, gp("grasp_place_topic").value, 10)

        # ── 运行时状态 ────────────────────────────────────────────────────── #
        self._state = self.ST_WAIT_DOG
        self._state_since = self._now()
        self._last_odom_time = None
        self._grasp_state = ""
        self._grasp_result = None          # None / True / False（LETTER_PLACING 内有效）
        self._error_reason = ""
        self._error_retriable = False
        self._last_heartbeat = 0.0
        self._last_trigger_pub = 0.0       # /grasp/place 重发节拍
        self._place_pub_until = 0.0        # /grasp/place 持续重发截止时刻
        self._place_triggered = False      # 收到 /grasp_flow/place_ready 或终端回车

        self._input_queue = queue.Queue()
        self._procs = {}                   # key -> subprocess.Popen
        self._arm_ctrl = None              # 延迟初始化，用于 Ctrl+C 复位

        # 终端回车作为临时触发；外部话题接入后仍可保留，同时生效
        if self._enable_prompt:
            threading.Thread(target=self._input_loop, daemon=True).start()

        self.create_timer(0.1, self._main_loop)
        self.get_logger().info(
            "grasp_flow_b 编排节点已启动（放置字母=%s，触发话题=%s），"
            "等待机械狗进入自动模式 …"
            % (HARDCODED_LETTER, self._place_trigger_topic))

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

    def _place_trigger_cb(self, msg):
        # 只接受 True；只在等待放置阶段生效，避免抓取中途误触发
        if not msg.data:
            return
        if self._state != self.ST_WAIT_LETTER:
            self.get_logger().warning(
                f"忽略 {self._place_trigger_topic} 触发：当前状态 {self._state}"
                " 非 WAIT_MANUAL_LETTER"
            )
            return
        if not self._place_triggered:
            self.get_logger().info(
                f"收到 {self._place_trigger_topic}，放置就绪触发生效"
            )
        self._place_triggered = True

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

    def _reset_arm_safe(self):
        """Ctrl+C 时尝试复位机械臂到初始姿态并张开夹爪，失败不阻断退出。"""
        if self._arm_ctrl is not None:
            return  # 已初始化过，复用
        try:
            import yaml
            from utils.ArmController import ArmController
            cfg = yaml.safe_load(open(self._config_path))
            arm_cfg = {**cfg['arm'],
                       'arm_serial_baud': cfg['hardware']['arm_serial_baud']}
            self._arm_ctrl = ArmController(device=self._arm_port, cfg=arm_cfg)
            self.get_logger().info("Ctrl+C 复位：机械臂回到初始姿态并张开夹爪")
            self._arm_ctrl.open_gripper()   # 先张开，避免归位途中把物块挂到别处
            import time
            time.sleep(0.3)
            self._arm_ctrl.set_pose(0)      # mode=0 初始姿态
            time.sleep(2.5)                 # 等舵机走完初始姿态
            self._arm_ctrl.finalize()
        except Exception as e:
            self.get_logger().warning(f"Ctrl+C 复位机械臂失败（非致命）: {e}")

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
            self._place_triggered = False
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
        # 触发条件（任一满足）：
        #   1) 外部模块向 /grasp_flow/place_ready 发 True（未来主路径）
        #   2) 本节点终端敲任意一行输入（回车即可，临时占位）
        if not self._place_triggered:
            cmd = self._poll_input()
            if cmd is not None:
                self.get_logger().info(
                    "收到终端输入（临时触发），视为放置就绪信号")
                self._place_triggered = True

        if not self._place_triggered:
            self._heartbeat(
                f"等待放置就绪信号：向 {self._place_trigger_topic} 发 "
                f"std_msgs/Bool data=true，或在此终端回车触发放置"
                f"（zone={HARDCODED_LETTER}）"
            )
            return

        self._letter = HARDCODED_LETTER
        self.get_logger().info(
            f"放置触发生效，向 grasp_task 发 /grasp/place zone={HARDCODED_LETTER}"
        )
        # grasp_task 的 /grasp/place 由 Event 一次锁存；但发布/订阅有短暂建链
        # 窗口，重启后订阅可能未就绪，故 2Hz 重发 5 秒保证首条一定被吃到，
        # 之后再收到的重复消息 grasp_task 会安全忽略
        self._pub_grasp_place.publish(String(data=self._letter))
        self._place_pub_until = self._now() + 5.0
        self._last_trigger_pub = self._now()
        self._grasp_result = None
        self._set_state(self.ST_PLACING)

    def _st_placing(self):
        if (self._now() < self._place_pub_until
                and self._now() - self._last_trigger_pub >= 0.5):
            self._last_trigger_pub = self._now()
            self._pub_grasp_place.publish(String(data=self._letter))

        if self._grasp_result is True:
            self.get_logger().info("放置完成，任务全流程结束 ✔")
            self._set_state(self.ST_DONE)
            return
        if self._grasp_result is False or self._grasp_state.startswith("ERROR"):
            reason = self._grasp_state if self._grasp_state.startswith("ERROR") \
                else "grasp_task 放置失败（/grasp/result=False）"
            self._fail(f"{reason}；如需重试需重启 grasp_task 节点")
            return
        if self._elapsed() > self._place_timeout_s:
            self._fail("放置超时（grasp_task 未在预期时间内报告完成）")
            return

        self._heartbeat(f"放置进行中（grasp_task: {self._grasp_state}）…")

    def _st_done(self):
        self._heartbeat("任务已完成。可按 Ctrl+C 退出。", period=30.0)

    def _st_error(self):
        self._heartbeat(
            f"流程处于 ERROR（{self._error_reason}）。请排查后重启。", period=30.0)

    # ═══════════════════════════ 析构 ═══════════════════════════════════════ #

    def destroy_node(self):
        # 只负责 kill 掉自己拉起的对齐子进程，机械臂复位由 grasp_task 自己的
        # finalize() 负责——这边如果再开 ArmController 会跟 grasp_task 抢
        # /dev/ttyUSB0 打架，反而导致夹爪没张开、舵机堵转过热。
        self._kill_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，正在安全退出...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
