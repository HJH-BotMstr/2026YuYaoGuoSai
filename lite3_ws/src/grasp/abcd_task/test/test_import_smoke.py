#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冒烟测试：确认 abcd_task 主模块能 import。

不实例化节点（避免依赖 rclpy.init 环境），只检查：
  - abcd_task.abcd_task_node 可 import
  - abcd_task.waypoint_nav 可 import
  - 关键类/函数存在
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def test_import_waypoint_nav():
    # 依赖 rclpy，若环境未 source ROS 2 会跳过
    try:
        from abcd_task import waypoint_nav
    except ImportError as e:
        if "rclpy" in str(e):
            pytest.skip(f"ROS 2 环境未 source: {e}")
        raise
    assert hasattr(waypoint_nav, "world_to_body")
    assert hasattr(waypoint_nav, "normalize_angle")
    assert hasattr(waypoint_nav, "WaypointNavigator")


def test_import_abcd_task_node():
    try:
        from abcd_task import abcd_task_node
    except ImportError as e:
        if "rclpy" in str(e):
            pytest.skip(f"ROS 2 环境未 source: {e}")
        raise
    assert hasattr(abcd_task_node, "AbcdTaskNode")
    assert hasattr(abcd_task_node, "main")
    assert hasattr(abcd_task_node, "VALID_LETTERS")
    assert set(abcd_task_node.VALID_LETTERS) == {"A", "B", "C", "D"}


def test_static_config_loader():
    """AbcdTaskNode._load_abcd_config 是 staticmethod，可无 rclpy 调用。"""
    try:
        from abcd_task.abcd_task_node import AbcdTaskNode
    except ImportError as e:
        if "rclpy" in str(e):
            pytest.skip(f"ROS 2 环境未 source: {e}")
        raise
    real_path = os.path.join(_PKG_ROOT, "config", "abcd_config.yaml")
    cfg = AbcdTaskNode._load_abcd_config(real_path)
    assert "letters" in cfg
    assert "transit_point" in cfg
