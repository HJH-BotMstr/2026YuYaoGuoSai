#!/usr/bin/env python3
"""加载 tools/grasp/config.yaml 并合并 ROS2 参数覆盖。"""
import os
import yaml


def load_config(node):
    """
    从 ROS2 参数 'tools_config_path' 读取 YAML 配置文件，
    强制将 runtime.mode 设为 'robot'，并返回配置字典。
    """
    tools_path = node.get_parameter('tools_config_path').value
    if not os.path.isfile(tools_path):
        raise FileNotFoundError(f"找不到 tools/grasp 配置文件: {tools_path}")

    with open(tools_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    if cfg is None:
        cfg = {}

    # 强制 robot 模式：ROS2 节点不再区分 pc/robot
    if 'runtime' not in cfg or cfg['runtime'] is None:
        cfg['runtime'] = {}
    cfg['runtime']['mode'] = 'robot'

    node.get_logger().info("加载 tools/grasp 配置: %s" % (tools_path))
    return cfg
