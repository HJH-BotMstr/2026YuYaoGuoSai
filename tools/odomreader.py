#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        self.odom_msg = None
        self.origin = None  # 程序启动时的初始位置

        self.sub_leg_odom2 = self.create_subscription(
            Odometry,
            '/leg_odom2',
            self.leg_odom2_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.display)
        self.get_logger().info('已订阅 /leg_odom2，等待第一帧数据设定原点...')

    def _clear_screen(self):
        os.system('clear' if os.name == 'posix' else 'cls')

    def leg_odom2_callback(self, msg: Odometry):
        # 第一帧数据设为原点
        if self.origin is None:
            self.origin = msg.pose.pose.position
            self.get_logger().info(
                f'原点已设定: x={self.origin.x:.4f}, y={self.origin.y:.4f}, z={self.origin.z:.4f}'
            )
        self.odom_msg = msg

    def display(self):
        if self.odom_msg is None or self.origin is None:
            return

        self._clear_screen()

        p = self.odom_msg.pose.pose.position
        q = self.odom_msg.pose.pose.orientation
        v = self.odom_msg.twist.twist.linear
        w = self.odom_msg.twist.twist.angular

        # 修正后的相对位置（以启动位置为原点）
        rel_x = p.x - self.origin.x
        rel_y = p.y - self.origin.y
        rel_z = p.z - self.origin.z

        print("=" * 50)
        print("  /leg_odom2  (nav_msgs/Odometry)")
        print("=" * 50)
        print(f"  原始位置:  x={p.x:>12.6f}  y={p.y:>12.6f}  z={p.z:>12.6f}")
        print(f"  修正位置:  x={rel_x:>12.6f}  y={rel_y:>12.6f}  z={rel_z:>12.6f}  <-- 相对原点")
        print()
        print(f"  姿态 (Quaternion):")
        print(f"    x = {q.x:>12.6f}")
        print(f"    y = {q.y:>12.6f}")
        print(f"    z = {q.z:>12.6f}")
        print(f"    w = {q.w:>12.6f}")
        print()
        print(f"  线速度:")
        print(f"    x = {v.x:>12.6f}")
        print(f"    y = {v.y:>12.6f}")
        print(f"    z = {v.z:>12.6f}")
        print()
        print(f"  角速度:")
        print(f"    x = {w.x:>12.6f}")
        print(f"    y = {w.y:>12.6f}")
        print(f"    z = {w.z:>12.6f}")
        print("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()