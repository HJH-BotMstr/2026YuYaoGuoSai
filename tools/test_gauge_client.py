#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的 ROS2 服务客户端，用于测试 /detect_gauge 服务。
不需要 ros2cli，直接运行即可。
"""

import sys
import time

import rclpy
from rclpy.node import Node
from gauge_detector_interfaces.srv import GaugeDetect


class GaugeClient(Node):
    def __init__(self):
        super().__init__('gauge_client')
        self.cli = self.create_client(GaugeDetect, '/detect_gauge')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /detect_gauge 服务上线 ...')
        self.req = GaugeDetect.Request()

    def call_once(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            resp = future.result()
            print('\n----- 服务返回 -----')
            print(f"success: {resp.success}")
            print(f"letter : {resp.letter}")
            print(f"zone   : {resp.zone}")
            print(f"state  : {resp.state}")
            print(f"message: {resp.message}")
            print('--------------------\n')
            return resp
        else:
            print('服务调用超时，没有收到响应')
            return None


def main():
    rclpy.init(args=sys.argv)
    client = GaugeClient()
    try:
        while True:
            client.call_once()
            time.sleep(2.0)
    except KeyboardInterrupt:
        print('退出')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
