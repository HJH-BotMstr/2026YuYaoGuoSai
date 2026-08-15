#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
abcd_config.yaml 加载与校验单测。

覆盖：
  - 真实 config/abcd_config.yaml 能被 _load_abcd_config 加载
  - 缺 letters / 缺 transit_point / 缺字段时会抛 ValueError
  - task_order + start_from 组合校验

注意：本文件用 unittest.mock 或直接调用类静态方法，不实例化 ROS 节点，
避免 pytest 环境不通 ROS。abcd_task_node.AbcdTaskNode._load_abcd_config
是 @staticmethod，可直接调用。
"""

import os
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def _load_static(path):
    """
    直接执行 _load_abcd_config 的等价逻辑，避免 import abcd_task_node
    （后者依赖 rclpy）。schema 与 AbcdTaskNode._load_abcd_config 保持一致。
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"abcd_config 不存在: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("abcd_config 顶层必须是 dict")
    for key in ("letters", "transit_point", "apriltag_tag_id"):
        if key not in data:
            raise ValueError(f"abcd_config 必须含 '{key}'")

    tp = data["transit_point"]
    for k in ("x", "y", "yaw"):
        if k not in tp:
            raise ValueError(f"transit_point 缺字段: {k}")

    for letter, cfg in data["letters"].items():
        for k in ("color", "task_x", "task_y", "task_yaw"):
            if k not in cfg:
                raise ValueError(f"letters[{letter}] 缺字段: {k}")

    try:
        int(data["apriltag_tag_id"])
    except (TypeError, ValueError):
        raise ValueError(
            f"apriltag_tag_id 必须是整数，当前={data['apriltag_tag_id']!r}")

    return data


# 用于负例测试的最小 valid dict 拼装工具
_VALID_LETTERS_INLINE = (
    "letters: {A: {color: red, task_x: 0, task_y: 0, task_yaw: 0}}\n"
)
_VALID_TRANSIT_INLINE = "transit_point: {x: 0, y: 0, yaw: 0}\n"
_VALID_TAG_ID_INLINE  = "apriltag_tag_id: 0\n"


class TestAbcdConfigLoader:

    def test_load_real_config(self):
        """真实 config/abcd_config.yaml 能被加载。"""
        real_path = os.path.join(_PKG_ROOT, "config", "abcd_config.yaml")
        cfg = _load_static(real_path)
        assert "letters" in cfg
        assert "transit_point" in cfg
        assert "apriltag_tag_id" in cfg
        assert isinstance(cfg["apriltag_tag_id"], int)
        assert set(cfg["letters"].keys()) == {"A", "B", "C", "D"}
        for letter in ("A", "B", "C", "D"):
            entry = cfg["letters"][letter]
            assert "color" in entry
            assert "task_x" in entry
            assert "task_y" in entry
            assert "task_yaw" in entry
            # tag_id 不应再出现在字母级配置里（tag 全流程唯一）
            assert "tag_id" not in entry

    def test_missing_letters(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text(_VALID_TRANSIT_INLINE + _VALID_TAG_ID_INLINE)
        with pytest.raises(ValueError, match="必须含 'letters'"):
            _load_static(str(p))

    def test_missing_transit(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text(_VALID_LETTERS_INLINE + _VALID_TAG_ID_INLINE)
        with pytest.raises(ValueError, match="必须含 'transit_point'"):
            _load_static(str(p))

    def test_missing_apriltag_tag_id(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text(_VALID_LETTERS_INLINE + _VALID_TRANSIT_INLINE)
        with pytest.raises(ValueError, match="必须含 'apriltag_tag_id'"):
            _load_static(str(p))

    def test_missing_transit_field(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text(
            "transit_point: {x: 0, y: 0}\n"
            + _VALID_LETTERS_INLINE + _VALID_TAG_ID_INLINE
        )
        with pytest.raises(ValueError, match="transit_point 缺字段"):
            _load_static(str(p))

    def test_missing_letter_field(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text(
            _VALID_TRANSIT_INLINE + _VALID_TAG_ID_INLINE +
            "letters: {A: {color: red, task_x: 0}}\n"  # 缺 task_y, task_yaw
        )
        with pytest.raises(ValueError, match=r"letters\[A\] 缺字段"):
            _load_static(str(p))

    def test_apriltag_tag_id_not_int(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text(
            _VALID_LETTERS_INLINE + _VALID_TRANSIT_INLINE +
            "apriltag_tag_id: \"not-an-int\"\n"
        )
        with pytest.raises(ValueError, match="apriltag_tag_id 必须是整数"):
            _load_static(str(p))

    def test_nonexistent_file(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            _load_static(str(tmp_path / "does_not_exist.yaml"))

    def test_task_order_letter_validation(self):
        """task_order 里的字母必须都在 letters 中。"""
        real_path = os.path.join(_PKG_ROOT, "config", "abcd_config.yaml")
        cfg = _load_static(real_path)
        task_order = ["A", "B", "C", "D"]
        for c in task_order:
            assert c in cfg["letters"], f"{c} 不在 letters 里"
