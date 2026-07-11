#!/usr/bin/env python3
"""
机械臂三连杆实时正运动学查看器

基于 tools/grasp/utils/RobotArm/three_Inverse_kinematics.py 的连杆模型，
实时读取 3/4/5 号舵机当前角度，计算并输出各关节在基座平面坐标系下的 (x, y) 坐标。

坐标系：
  - 原点：三连杆基座（5 号舵机旋转中心）
  - +x：机械臂前方（水平伸出方向）
  - +y：竖直向上
  - 连杆长度：L1=105 mm, L2=110 mm, L3=110 mm

用法：
  python3 tests/test_arm_kinematics_viewer.py
  python3 tests/test_arm_kinematics_viewer.py --config config.yaml --interval 0.5
  python3 tests/test_arm_kinematics_viewer.py --pulse "2047,2047,2047"   # 离线验证

按键：
  Ctrl+C 退出
"""
import sys
import os
import argparse
import time
import math
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.ArmController import ArmController
from utils.RobotArm.three_Inverse_kinematics import CAM_OFFSET_X, CAM_OFFSET_Y


L1, L2, L3 = 105.0, 110.0, 110.0
PULSE_ZERO = 2047.0
PULSE_TO_RAD = 2.0 * math.pi / 4096.0  # 11.375 deg/pulse ~= 4096 pulse/360°


def pulse_to_q(angle_3: int, angle_4: int, angle_5: int):
    """
    把舵机脉冲转换为三连杆相对转角 q1/q2/q3（弧度）。
    符号与 three_Inverse_kinematics.py 中的逆运动学对应。
    """
    q1 = (angle_5 - PULSE_ZERO) * PULSE_TO_RAD
    q2 = (angle_4 - PULSE_ZERO) * PULSE_TO_RAD
    q3 = -(angle_3 - PULSE_ZERO) * PULSE_TO_RAD
    return q1, q2, q3


def forward_kinematics(angle_3: int, angle_4: int, angle_5: int):
    """
    三连杆正运动学。
    返回基座坐标系下：
      - J0: 基座原点 (0, 0)
      - J1: L1 末端 / L2 起点
      - J2: L2 末端 / L3 起点
      - end: L3 末端（夹爪附近）
    """
    q1, q2, q3 = pulse_to_q(angle_3, angle_4, angle_5)

    # 连杆绝对方向：zero 位（所有脉冲 2047）时，L1 竖直向上（+pi/2）
    phi1 = q1 + math.pi / 2.0
    phi2 = q1 + q2 + math.pi / 2.0
    phi3 = q1 + q2 + q3 + math.pi / 2.0

    J0 = (0.0, 0.0)
    J1 = (L1 * math.cos(phi1), L1 * math.sin(phi1))
    J2 = (J1[0] + L2 * math.cos(phi2),
          J1[1] + L2 * math.sin(phi2))
    end = (J2[0] + L3 * math.cos(phi3),
           J2[1] + L3 * math.sin(phi3))

    # 相机安装在末端连杆上，随 L3 一起旋转
    cam_x = end[0] + (CAM_OFFSET_X * math.cos(phi3)
                      - CAM_OFFSET_Y * math.sin(phi3))
    cam_y = end[1] + (CAM_OFFSET_X * math.sin(phi3)
                      + CAM_OFFSET_Y * math.cos(phi3))

    return {
        "J0": J0,
        "J1": J1,
        "J2": J2,
        "end": end,
        "cam": (cam_x, cam_y),
        "angles_deg": (math.degrees(q1), math.degrees(q2), math.degrees(q3)),
    }


def print_coordinates(pos, pulses):
    """清屏并在命令行打印当前坐标。"""
    # ANSI 清屏 + 光标移到左上角
    print("\033[2J\033[H", end="")

    print("=" * 56)
    print("机械臂三连杆实时坐标查看器")
    print("=" * 56)
    print(f"舵机脉冲:  3={pulses[0]:>5d}  4={pulses[1]:>5d}  5={pulses[2]:>5d}")
    q1, q2, q3 = pos["angles_deg"]
    print(f"相对角度: q1={q1:>7.1f}° q2={q2:>7.1f}° q3={q3:>7.1f}°")
    print("-" * 56)
    print(f"{'关节':>6s}  {'x (mm)':>12s}  {'y (mm)':>12s}")
    print("-" * 56)
    for name in ("J0", "J1", "J2", "end", "cam"):
        x, y = pos[name]
        print(f"{name:>6s}  {x:>12.1f}  {y:>12.1f}")
    print("=" * 56)
    print("按 Ctrl+C 退出")


def main():
    parser = argparse.ArgumentParser(description="机械臂三连杆坐标查看器")
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
    parser.add_argument("--interval", type=float, default=0.5,
                        help="刷新周期（秒），默认 0.5")
    parser.add_argument("--pulse", type=str, default=None,
                        help="离线模式：直接输入 3,4,5 号舵机脉冲，如 '2047,2047,2047'")
    args = parser.parse_args()

    # 离线模式：不连接硬件，仅做一次计算
    if args.pulse:
        try:
            a3, a4, a5 = map(int, args.pulse.split(","))
        except ValueError:
            print("--pulse 格式错误，应为 'a3,a4,a5'，例如 '2047,2047,2047'")
            sys.exit(1)
        pos = forward_kinematics(a3, a4, a5)
        print_coordinates(pos, (a3, a4, a5))
        return

    # 加载配置并连接机械臂
    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    arm = ArmController(
        device=cfg["hardware"]["arm_serial_port"],
        cfg={**cfg["arm"], "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]},
    )

    try:
        while True:
            positions = arm.read_positions((3, 4, 5))
            a3 = positions.get(3, -1)
            a4 = positions.get(4, -1)
            a5 = positions.get(5, -1)

            if -1 in (a3, a4, a5):
                print("\033[2J\033[H", end="")
                print("读取舵机位置失败，请检查串口连接。")
                time.sleep(args.interval)
                continue

            pos = forward_kinematics(a3, a4, a5)
            print_coordinates(pos, (a3, a4, a5))
            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n用户中断，关闭串口...")
    finally:
        arm.finalize()


if __name__ == "__main__":
    main()
