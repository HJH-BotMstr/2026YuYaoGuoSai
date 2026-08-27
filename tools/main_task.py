#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""主程序：路径回放 + 定点仪表盘识别（或免识别语音播报兜底）。

流程：
  1. VISION_ENABLED=True 时启动脚本即初始化视觉模块（加载 YOLO 模型 + 打开 /dev/video0 + 预热）；
     VISION_ENABLED=False 时跳过初始化，不加载模型、不开摄像头
  2. 复用 way_point.py 回放模式：reset_origin 后按 tools/waypoints.json
     记录的 move 序列逐段导航
  3. 到达 DETECT_WAYPOINTS 指定的 waypoint（0 起编号，与 way_point.py 日志一致）时暂停运动：
     - 视觉模式：执行仪表盘识别（循环识别直到读出字母，上限 DETECT_MAX_ROUNDS 轮）
     - 免识别模式：按 VOICE_BROADCAST_TABLE 直接播放对应 MP3 语音
     完成后继续向下一个点导航
  4. 全部识别点完成后关闭摄像头（仅视觉模式），继续走到最后一个点结束

前置条件（本脚本不负责启动）：
  - 官方 ROS2 栈已运行（提供 /leg_odom2、/cmd_vel）
  - pose_controller 已运行：ros2 run pose_control start_pose_control
    （或直接 python3 lite3_ws/src/pose_control/pose_controller_node.py）

运行：
  source /opt/ros/foxy/setup.bash
  source ~/yolov8_env/bin/activate   # 仅视觉模式需要
  python3 tools/main_task.py [waypoints.json]
  （或一键脚本：tools/script/waypoint_playback.sh）
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from way_point import WaypointTool

# ===================== 可配置项 =====================

WAYPOINTS_PATH = PROJECT_ROOT / "tools" / "waypoints.json"

# 需要执行仪表盘识别/播报的 waypoint 序号（0 起编号，waypoint 0 = 录制时按 S 的起点）。
# 占位值，比赛前按实际路线手动修改。
DETECT_WAYPOINTS = [1, 2, 3, 4]

# 视觉识别总开关：
#   True  = 正常视觉识别（启动时加载模型 + 预热摄像头，识别到字母自动播报）
#   False = 免识别兜底：不加载模型、不开摄像头，到识别点直接按下方播报表播语音
VISION_ENABLED = True

# 免识别模式（VISION_ENABLED=False）的语音播报表：
# 按 DETECT_WAYPOINTS 升序一一对应，第 N 个识别点播第 N 条。
# 条目格式 "字母+区域"，对应 assets/mp3/<条目>.mp3：
#   字母 A/B/C/D；区域 H=偏高(RED)  L=偏低(YELLOW)  M=居中(GREEN，正常)
# 示例：["AH", "BL", "CM", "DM"] = A偏高、B偏低、C正常、D正常
VOICE_BROADCAST_TABLE = ["AM", "BM", "CM", "DM"]

MP3_DIR = PROJECT_ROOT / "assets" / "mp3"

DETECT_MAX_ROUNDS = 5     # 每个识别点最多识别轮数（每轮 recognize() 内部还会重试 3 次）
DETECT_SETTLE_SEC = 1.0   # 到达识别点后先静止稳定的时间（秒）
DETECT_ROUND_INTERVAL = 0.5  # 每轮识别之间的间隔（秒）

# ====================================================

_gv = None  # 视觉模块懒加载：仅视觉模式才 import（免识别模式不依赖 YOLO 环境）


def _ensure_gv():
    global _gv
    if _gv is None:
        import gauge_vision
        _gv = gauge_vision
    return _gv


def play_mp3_blocking(filepath):
    """ffmpeg 解码成 WAV 再用 aplay 播放（与 gauge_yolo_new_v2.play_mp3 同逻辑，阻塞式）。"""
    if shutil.which('ffmpeg') is None or shutil.which('aplay') is None:
        raise RuntimeError("缺少 ffmpeg 或 aplay，无法语音播报")
    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run(
            ['ffmpeg', '-y', '-i', str(filepath),
             '-vn', '-acodec', 'pcm_s16le', '-ar', '44100', '-ac', '2',
             tmp_path],
            check=True, capture_output=True
        )
        subprocess.run(
            ['aplay', '-D', 'pulse', tmp_path],
            check=True, capture_output=True
        )
    finally:
        os.unlink(tmp_path)


# 播报表条目格式：字母 A-D + 区域 H/L/M
VOICE_ENTRY_RE = re.compile(r'^[A-D][HLM]$')
# 区域后缀 -> 状态（M=居中=正常，H/L=偏高/偏低=异常）
VOICE_SUFFIX_STATE = {'M': 'normal', 'H': 'abnormal', 'L': 'abnormal'}


class MainTaskNode(WaypointTool):
    """在 way_point 回放状态机上插入定点识别/播报钩子。"""

    def __init__(self, filepath, detect_points):
        super().__init__("broadcast", str(filepath))
        self._detect_points = set(detect_points)
        self._detected = set()
        self._camera_closed = False
        self._final_results = {}

        # 免识别模式：校验播报表并建立 waypoint 序号 -> 播报条目 的映射
        self._voice_plan = {}
        if not VISION_ENABLED:
            ordered = sorted(self._detect_points)
            if len(VOICE_BROADCAST_TABLE) != len(ordered):
                raise ValueError(
                    f"播报表条数({len(VOICE_BROADCAST_TABLE)})与识别点数({len(ordered)})不一致，"
                    "请修改 VOICE_BROADCAST_TABLE 或 DETECT_WAYPOINTS"
                )
            for wp_idx, entry in zip(ordered, VOICE_BROADCAST_TABLE):
                entry = entry.strip().upper()
                if not VOICE_ENTRY_RE.match(entry):
                    raise ValueError(
                        f"播报表条目 '{entry}' 格式错误，应为 '字母+区域'（如 AH/BL/CM）"
                    )
                self._voice_plan[wp_idx] = entry

        # 加载 waypoint 列表用于打印路径表和校验序号
        with open(filepath, "r", encoding="utf-8") as f:
            self._waypoint_list = json.load(f).get("waypoints", [])

        n = len(self._waypoint_list)
        for idx in self._detect_points:
            if not 0 <= idx < n:
                raise ValueError(
                    f"识别点序号 {idx} 越界：waypoints.json 共 {n} 个点（0 ~ {n - 1}）"
                )
        self._print_path_table()

    def _print_path_table(self):
        self.get_logger().info(
            f"路径共 {len(self._waypoint_list)} 个 waypoint，"
            f"识别点：{sorted(self._detect_points)}"
            f"（{'视觉识别' if VISION_ENABLED else '免识别语音播报'}模式）"
        )
        for i, wp in enumerate(self._waypoint_list):
            mark = ""
            if i in self._detect_points:
                mark = " *识别点"
                if i in self._voice_plan:
                    mark += f"(播报 {self._voice_plan[i]})"
            self.get_logger().info(
                f"  [{i}] x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}{mark}"
            )

    def _voice_broadcast(self, wp_idx):
        """免识别模式：直接播放该识别点对应的 MP3（阻塞，狗保持静止）。"""
        entry = self._voice_plan[wp_idx]
        filepath = MP3_DIR / f"{entry}.mp3"
        if not filepath.is_file():
            self.get_logger().error(f"音频文件不存在：{filepath}，跳过播报")
            return

        letter, suffix = entry[0], entry[1]
        state = VOICE_SUFFIX_STATE[suffix]
        self._final_results[letter] = state
        self.get_logger().info(
            f"识别点 {wp_idx} 语音播报：{entry}.mp3（{letter} 表 {state}）"
        )
        try:
            play_mp3_blocking(filepath)
        except Exception as e:
            self.get_logger().error(f"语音播报失败：{e}")

    def _maybe_detect(self, wp_idx):
        """若 wp_idx 是未完成识别的识别点，则执行识别/播报任务（阻塞，狗保持静止）。"""
        if wp_idx not in self._detect_points or wp_idx in self._detected:
            return

        self.get_logger().info(f"到达识别点 waypoint {wp_idx}，静止稳定 {DETECT_SETTLE_SEC}s")
        time.sleep(DETECT_SETTLE_SEC)

        if not VISION_ENABLED:
            self._voice_broadcast(wp_idx)
            self._detected.add(wp_idx)
            self.get_logger().info(f"当前结果表：{self._final_results}")
            return

        gv = _ensure_gv()
        for round_i in range(1, DETECT_MAX_ROUNDS + 1):
            r = gv.recognize()
            self.get_logger().info(f"识别点 {wp_idx} 第 {round_i} 轮：{r}")
            if r.get("letter") is not None:
                break
            if round_i < DETECT_MAX_ROUNDS:
                time.sleep(DETECT_ROUND_INTERVAL)
        else:
            self.get_logger().warn(
                f"识别点 {wp_idx} 达到最大轮数 {DETECT_MAX_ROUNDS} 仍未读出字母，继续导航"
            )

        self._detected.add(wp_idx)
        self._final_results = gv.get_all()
        self.get_logger().info(f"当前结果表：{self._final_results}")

        # 全部识别点完成后关闭摄像头，后续导航不再占用
        if not self._camera_closed and self._detected >= self._detect_points:
            gv.shutdown()
            self._camera_closed = True
            self.get_logger().info("全部识别点完成，摄像头已关闭")

    def _tick_broadcast(self):
        if self._broadcast_state == "moving":
            # 即将发布 move i（从 waypoint i 走向 i+1），当前位于 waypoint i
            self._maybe_detect(self._broadcast_index)
            if self._broadcast_index >= len(self._moves):
                # 路径终点是最后一个 waypoint，也可能是识别点
                self._maybe_detect(len(self._moves))
        super()._tick_broadcast()


def main():
    # 可选命令行参数：waypoints 文件路径（默认 tools/waypoints.json）
    waypoints_path = Path(sys.argv[1]) if len(sys.argv) > 1 else WAYPOINTS_PATH

    print("=" * 50)
    if VISION_ENABLED:
        print("主程序启动（视觉识别模式）：正在初始化视觉模块（加载模型 + 预热 video0，约 30~60 秒）...")
        _ensure_gv().init()
        print("视觉模块初始化完成")
    else:
        print("主程序启动（免识别语音播报模式）：跳过视觉初始化")
        print(f"播报表：{VOICE_BROADCAST_TABLE}")

    rclpy.init(args=None)
    node = MainTaskNode(waypoints_path, DETECT_WAYPOINTS)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n最终识别结果：{node._final_results}")
        node.destroy_node()
        rclpy.shutdown()
        if _gv is not None:
            _gv.shutdown()  # 兜底释放（已关闭时为空操作）

    print("主程序结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
