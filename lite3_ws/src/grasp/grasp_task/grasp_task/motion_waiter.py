#!/usr/bin/env python3
"""基于 /cmd_vel 判断 pose_control 是否到位的辅助类。"""
import threading
import time


class MotionWaiter:
    """
    监听 geometry_msgs/Twist，先等待速度非零（运动开始），
    再等待速度持续接近零（运动停止）。
    """

    def __init__(self, zero_duration_s=0.5, timeout_s=15.0,
                 linear_thr=0.01, angular_thr=0.01):
        self._zero_duration_s = zero_duration_s
        self._timeout_s = timeout_s
        self._linear_thr = linear_thr
        self._angular_thr = angular_thr
        self._lock = threading.Lock()
        self._started = False
        self._zero_since = None
        self._last_update_time = 0.0

    def on_cmd_vel(self, msg):
        """由节点回调调用。"""
        with self._lock:
            if (abs(msg.linear.x) < self._linear_thr and
                    abs(msg.linear.y) < self._linear_thr and
                    abs(msg.angular.z) < self._angular_thr):
                if self._started and self._zero_since is None:
                    self._zero_since = time.monotonic()
            else:
                self._started = True
                self._zero_since = None
            self._last_update_time = time.monotonic()

    def wait_for_stop(self):
        """
        阻塞等待直到运动开始后再停止，或超时。
        返回 True 表示已到位，False 表示超时。
        """
        deadline = time.monotonic() + self._timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._started and self._zero_since is not None:
                    if time.monotonic() - self._zero_since >= self._zero_duration_s:
                        return True
            time.sleep(0.05)
        return False

    def reset(self):
        """重置到位计时器。"""
        with self._lock:
            self._started = False
            self._zero_since = None
