#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯函数单元测试：world_to_body / normalize_angle。

不依赖 ROS 或硬件，`colcon test --packages-select abcd_task` 或
`pytest test/` 直接跑。
"""

import math
import os
import sys

# 让 pytest 在 colcon test 之外也能直接跑（不通过 ament 安装路径）
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest

from abcd_task.waypoint_nav import normalize_angle, world_to_body


class TestNormalizeAngle:

    def test_zero(self):
        assert normalize_angle(0.0) == pytest.approx(0.0, abs=1e-12)

    def test_within_range(self):
        assert normalize_angle(1.5) == pytest.approx(1.5, abs=1e-12)
        assert normalize_angle(-1.5) == pytest.approx(-1.5, abs=1e-12)

    def test_positive_overflow(self):
        # 3π → π（归到 (-π, π] 的正边界）
        assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)

    def test_negative_overflow(self):
        # -3π → π（-3π = π - 4π，落在 (-π, π]）
        assert normalize_angle(-3.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)

    def test_large_positive(self):
        # 7π = π mod 2π
        assert normalize_angle(7.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)

    def test_large_negative(self):
        # -5π = π mod 2π（-5π + 6π = π）
        assert normalize_angle(-5.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)


class TestWorldToBody:
    """
    Lite3 body frame 约定：+x 前，+y 左。
    yaw 是机体相对世界系的旋转（弧度，绕 z 轴），yaw>0 表示逆时针。
    """

    def test_zero_yaw_identity(self):
        # yaw=0 时 body == world
        dx, dy = world_to_body(1.0, 0.5, 0.0)
        assert dx == pytest.approx(1.0, abs=1e-12)
        assert dy == pytest.approx(0.5, abs=1e-12)

    def test_yaw_pi_over_2(self):
        # yaw=+π/2（机体朝世界 +y 方向）：世界 +x 变成机体 -y（机体右方）
        dx, dy = world_to_body(1.0, 0.0, math.pi / 2.0)
        assert dx == pytest.approx(0.0, abs=1e-9)
        assert dy == pytest.approx(-1.0, abs=1e-9)

    def test_yaw_pi_over_2_lateral(self):
        # yaw=+π/2：世界 +y 变成机体 +x（机体正前方）
        dx, dy = world_to_body(0.0, 1.0, math.pi / 2.0)
        assert dx == pytest.approx(1.0, abs=1e-9)
        assert dy == pytest.approx(0.0, abs=1e-9)

    def test_yaw_pi(self):
        # yaw=π（机体调头 180°）：世界 +x 变成机体 -x
        dx, dy = world_to_body(1.0, 0.0, math.pi)
        assert dx == pytest.approx(-1.0, abs=1e-9)
        assert dy == pytest.approx(0.0, abs=1e-9)

    def test_yaw_negative_pi_over_2(self):
        # yaw=-π/2（机体朝世界 -y 方向）：世界 +x 变成机体 +y
        dx, dy = world_to_body(1.0, 0.0, -math.pi / 2.0)
        assert dx == pytest.approx(0.0, abs=1e-9)
        assert dy == pytest.approx(1.0, abs=1e-9)

    def test_rotation_inverse(self):
        # world → body → world 恒等
        yaw = 0.7
        dx_w, dy_w = 2.3, -1.1
        dx_b, dy_b = world_to_body(dx_w, dy_w, yaw)
        # 逆变换：body → world
        c = math.cos(yaw)
        s = math.sin(yaw)
        back_x = c * dx_b - s * dy_b
        back_y = s * dx_b + c * dy_b
        assert back_x == pytest.approx(dx_w, abs=1e-9)
        assert back_y == pytest.approx(dy_w, abs=1e-9)

    def test_preserves_length(self):
        dx_w, dy_w = 3.0, 4.0
        for yaw in (0.0, 0.5, math.pi / 3.0, math.pi, -math.pi / 6.0):
            dx_b, dy_b = world_to_body(dx_w, dy_w, yaw)
            len_w = math.hypot(dx_w, dy_w)
            len_b = math.hypot(dx_b, dy_b)
            assert len_b == pytest.approx(len_w, abs=1e-9)
