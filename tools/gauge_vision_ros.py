#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仪表盘视觉模块（ROS 服务调用版）：给主程序调用的函数接口。

与纯 Python 版 gauge_vision.py 接口完全一致，区别是识别通过调用
ROS 服务 /detect_gauge_yolo 完成——模型加载、摄像头、语音播报都在
服务端节点 gauge_yolo_server 里，需要先启动：

    cd /home/ysc/2026YuYaoGuoSai/lite3_ws
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    ros2 run gauge_yolo_detector gauge_yolo_server

使用前主程序所在终端也要 source ROS 环境（同上两行 source）。

功能：
  - init()            初始化：连接服务端节点（等服务上线）
  - recognize()       识别一次：调用服务，返回字母/区域/状态，并存入结果表
  - get_state(letter) 查询某个字母的状态：'normal' / 'abnormal' / None（未识别过）
  - get_all()         查询全部结果：{'A': 'normal', 'D': 'abnormal', ...}
  - shutdown()        关闭节点（程序结束时调用）

语音播报由服务端节点完成（字母+区域变化时播对应 MP3），本模块不管。

用法示例：
    import sys
    sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
    import gauge_vision_ros as gv

    gv.init()                       # 连接服务（会等服务上线）
    r = gv.recognize()              # {'letter': 'A', 'zone': 'GREEN', 'state': 'normal'}
    print(gv.get_all())             # {'A': 'normal'}
    print(gv.get_state('D'))        # None（还没识别过）
    gv.shutdown()
"""

import time

import rclpy
from rclpy.node import Node

from gauge_detector_interfaces.srv import GaugeDetect


class GaugeVisionRos:
    """仪表盘识别客户端：recognize() 时调用 /detect_gauge_yolo 服务。"""

    def __init__(self, service_name='detect_gauge_yolo',
                 node_name='gauge_vision_client', wait_timeout=None):
        """
        service_name: 服务端服务名，默认 detect_gauge_yolo
        wait_timeout: 等待服务上线的秒数，None 表示一直等
        """
        # 主程序如果自己已经 rclpy.init() 过就不重复初始化
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self.node = Node(node_name)
        self.cli = self.node.create_client(GaugeDetect, service_name)

        print(f"等待服务 /{service_name} 上线 ...")
        t0 = time.time()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if wait_timeout is not None and time.time() - t0 > wait_timeout:
                raise RuntimeError(f"等待服务 /{service_name} 超时（{wait_timeout} 秒），"
                                   f"请确认 gauge_yolo_server 节点已启动")
            print(f"  继续等待 /{service_name} ...")
        print(f"服务 /{service_name} 已连接")

        # 结果存储：{'A': 'normal', 'B': 'abnormal', ...}
        self.results = {}

    def recognize(self, retries=3, retry_interval=0.4, timeout=5.0):
        """
        识别一次当前画面中的仪表盘（调用一次或多次服务）。

        返回 dict：
            {'letter': 'A' 或 None,   # None 表示这次没识别出字母
             'zone': 'RED'/'GREEN'/'YELLOW' 或 None,  # None 表示没检测到表盘
             'state': 'normal'/'abnormal' 或 None}
        识别到字母时会自动存入结果表（get_all / get_state 可查）。

        retries: 服务调用成功但没识别出字母时的重试次数。
        timeout: 单次服务调用的超时秒数。
        """
        result = {'letter': None, 'zone': None, 'state': None}

        for _ in range(retries):
            future = self.cli.call_async(GaugeDetect.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

            if not future.done():
                print(f"警告：服务调用超时（{timeout} 秒）")
                time.sleep(retry_interval)
                continue

            resp = future.result()
            if resp is None or not resp.success:
                # success=False：未检测到表盘或服务忙，重试
                time.sleep(retry_interval)
                continue

            result['zone'] = resp.zone or None
            result['state'] = resp.state or None

            letter = resp.letter or None
            if letter is not None:
                result['letter'] = letter
                self.results[letter] = resp.state
                break

            time.sleep(retry_interval)

        return result

    def get_state(self, letter):
        """查询某个字母的状态：'normal' / 'abnormal' / None（未识别过）。"""
        return self.results.get(letter)

    def get_all(self):
        """查询全部已识别结果，返回 dict 副本。"""
        return dict(self.results)

    def shutdown(self):
        """销毁节点；如果 rclpy 是本模块初始化的，一并 shutdown。"""
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


# ===================== 模块级函数接口（推荐主程序用这层） =====================

_vision = None


def _require_init():
    if _vision is None:
        raise RuntimeError("视觉模块未初始化，请先调用 gauge_vision_ros.init()")


def init(**kwargs):
    """初始化视觉模块（连接 ROS 服务端）。只调用一次。

    可选参数（一般不用动）：
        service_name='detect_gauge_yolo', node_name='gauge_vision_client',
        wait_timeout=None（等待服务上线的秒数，None 表示一直等）
    """
    global _vision
    if _vision is not None:
        print("视觉模块已初始化，跳过重复 init")
        return
    _vision = GaugeVisionRos(**kwargs)


def recognize(retries=3, retry_interval=0.4, timeout=5.0):
    """识别一次当前仪表盘，返回 {'letter', 'zone', 'state'}，识别到字母会自动存储。"""
    _require_init()
    return _vision.recognize(retries=retries, retry_interval=retry_interval,
                             timeout=timeout)


def get_state(letter):
    """查询某个字母（'A'/'B'/'C'/'D'）的状态：'normal' / 'abnormal' / None。"""
    _require_init()
    return _vision.get_state(letter)


def get_all():
    """查询全部已识别结果：{'A': 'normal', ...}"""
    _require_init()
    return _vision.get_all()


def shutdown():
    """销毁节点。程序结束时调用。"""
    global _vision
    if _vision is not None:
        _vision.shutdown()
        _vision = None


# ===================== 自测试 =====================

if __name__ == '__main__':
    print("视觉模块（ROS 版）自测试：连接服务中 ...")
    init()
    print("已连接，每 2 秒识别一次，Ctrl+C 退出\n")
    try:
        while True:
            r = recognize()
            print(f"识别: {r}   已存: {get_all()}")
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
        print("\n已关闭")
