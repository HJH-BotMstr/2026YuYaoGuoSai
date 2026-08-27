#!/bin/bash
# 一键启动 way_point.py 录制模式（遥控器走点位，键盘 S/A/E 标点）
# 前置：官方 ROS2 栈已运行（提供 /leg_odom2）
# 用法：
#   tools/script/waypoint_record.sh                   # 默认保存到 tools/waypoints.json
#   tools/script/waypoint_record.sh path/to/file.json # 保存到指定 JSON 文件

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}" || exit 1

# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

WAYPOINT_FILE="${1:-tools/waypoints.json}"

echo "录制模式：S=设起点(清空旧路径)  A=添加点位  E=结束并保存到 ${WAYPOINT_FILE}"
exec python3 tools/way_point.py record "${WAYPOINT_FILE}"
