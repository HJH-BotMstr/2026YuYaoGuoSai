#!/bin/bash
# 一键回放主任务：后台启动 pose_control 节点 + 前台运行 main_task.py
# 流程：reset_origin -> 按 tools/waypoints.json 逐段导航 -> 到识别点做仪表盘识别/语音播报
# 前置：
#   - 官方 ROS2 栈已运行（提供 /leg_odom2、/cmd_vel）
#   - 狗已用遥控器站起来，处于可行走状态
# 用法：
#   tools/script/waypoint_playback.sh                   # 默认加载 tools/waypoints.json
#   tools/script/waypoint_playback.sh path/to/file.json # 加载指定 JSON 文件

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}" || exit 1

# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

WAYPOINT_FILE="${1:-tools/waypoints.json}"
POSE_CONTROLLER="lite3_ws/src/pose_control/pose_controller_node.py"

# 后台启动 pose_control 节点，退出时自动清理
python3 "${POSE_CONTROLLER}" &
POSE_PID=$!
trap 'kill "${POSE_PID}" 2>/dev/null' EXIT
echo "pose_control 节点已在后台启动，PID: ${POSE_PID}"
sleep 1  # 等节点起来订阅 /move

# main_task.py 需要 YOLO 环境（视觉识别模式）；免识别语音播报模式不依赖也可
# shellcheck disable=SC1091
source ~/yolov8_env/bin/activate

exec python3 tools/main_task.py
