#!/usr/bin/env python3
"""
单关节圆周运动测试脚本

用途：让 3/4/5 号舵机中的一个单独运动，其余舵机保持固定，
      通过正运动学输出各关节 / 末端 / 相机的理论坐标，
      方便你用尺子/视觉对比实际运动轨迹，排查零点、连杆长度、相机偏移问题。

运行：
  python3 tests/test_single_joint_circle.py
  python3 tests/test_single_joint_circle.py --config config.yaml

流程：
  1. 选择要测试的关节（3/4/5）
  2. 输入中心脉冲、摆动幅度、步长、每步停留时间
  3. 脚本自动摆动并输出理论坐标
  4. 可选：每步输入手测的 end/cam 坐标，脚本会计算实际圆心/半径

安全：
  - 第一次运动前会要求输入 y 确认
  - 运动过程中按 Ctrl+C 会回到中心位置并关闭串口
"""
import sys
import os
import argparse
import time
import math
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.ArmController import ArmController
from utils.RobotArm.three_Inverse_kinematics import L1, L2, L3, CAM_J2_X, CAM_J2_Y

PULSE_ZERO = 2047.0
PULSE_TO_RAD = 2.0 * math.pi / 4096.0


def pulse_to_q(angle_3, angle_4, angle_5):
    q1 = (angle_5 - PULSE_ZERO) * PULSE_TO_RAD
    q2 = (angle_4 - PULSE_ZERO) * PULSE_TO_RAD
    q3 = -(angle_3 - PULSE_ZERO) * PULSE_TO_RAD
    return q1, q2, q3


def forward_kinematics(angle_3, angle_4, angle_5):
    q1, q2, q3 = pulse_to_q(angle_3, angle_4, angle_5)
    phi1 = q1 + math.pi / 2.0
    phi2 = q1 + q2 + math.pi / 2.0
    phi3 = q1 + q2 + q3 + math.pi / 2.0

    J0 = (0.0, 0.0)
    J1 = (L1 * math.cos(phi1), L1 * math.sin(phi1))
    J2 = (J1[0] + L2 * math.cos(phi2), J1[1] + L2 * math.sin(phi2))
    end = (J2[0] + L3 * math.cos(phi3), J2[1] + L3 * math.sin(phi3))

    cam_x = J2[0] + CAM_J2_X * math.cos(phi3) - CAM_J2_Y * math.sin(phi3)
    cam_y = J2[1] + CAM_J2_X * math.sin(phi3) + CAM_J2_Y * math.cos(phi3)

    return {
        "J0": J0, "J1": J1, "J2": J2, "end": end, "cam": (cam_x, cam_y),
        "angles_deg": (math.degrees(q1), math.degrees(q2), math.degrees(q3)),
    }


def print_table_row(step, pulses, pos):
    a3, a4, a5 = pulses
    print(f"{step:>3d} | "
          f"3={a3:>5d} 4={a4:>5d} 5={a5:>5d} | "
          f"J1=({pos['J1'][0]:>6.1f},{pos['J1'][1]:>6.1f}) | "
          f"J2=({pos['J2'][0]:>6.1f},{pos['J2'][1]:>6.1f}) | "
          f"end=({pos['end'][0]:>6.1f},{pos['end'][1]:>6.1f}) | "
          f"cam=({pos['cam'][0]:>6.1f},{pos['cam'][1]:>6.1f})")


def move_and_read(arm, servo_id, target, fixed_pulses, speed, acc, settle=0.5):
    """移动指定舵机到 target，等待到位，读取所有位置。"""
    ph = arm.packetHandler
    ph.WritePosEx(servo_id, target, speed, acc)
    # 等待到位
    arm.wait_for_position({servo_id: target}, timeout=3.0)
    time.sleep(settle)
    positions = arm.read_positions((3, 4, 5))
    pulses = (positions.get(3, -1), positions.get(4, -1), positions.get(5, -1))
    return pulses


def run_test(arm, test_joint, center, amplitude, step, delay, fixed_pulses,
             speed, acc, manual_mode=False):
    """
    test_joint: 3, 4 or 5
    center: 中心脉冲
    amplitude: 相对中心的摆动幅度（脉冲）
    step: 每步脉冲增量
    delay: 每步停留时间（秒）
    fixed_pulses: 其余关节保持的脉冲值 dict {id: pulse}
    """
    if test_joint not in (3, 4, 5):
        print("错误：只能选择关节 3/4/5")
        return

    # 先回到中心位姿
    print("\n>>> 先回到中心位置...")
    for sid, val in fixed_pulses.items():
        arm.packetHandler.WritePosEx(sid, val, speed, acc)
    arm.packetHandler.WritePosEx(test_joint, center, speed, acc)
    arm.wait_for_position({**fixed_pulses, test_joint: center}, timeout=5.0)
    time.sleep(0.5)

    print("\n运动计划：")
    print(f"  测试关节: {test_joint}")
    print(f"  中心脉冲: {center}")
    print(f"  摆动范围: {center - amplitude} ~ {center + amplitude}")
    print(f"  步长: {step}, 每步停留: {delay}s")
    if test_joint == 5:
        print(f"  预期: J1 绕 J0 画圆，半径 ≈ {L1} mm")
    elif test_joint == 4:
        print(f"  预期: J2 绕 J1 画圆，半径 ≈ {L2} mm")
    elif test_joint == 3:
        print(f"  预期: end 绕 J2 画圆，半径 ≈ {L3} mm；cam 绕 J2 画小圆")

    confirm = input("\n确认开始运动？输入 y 继续，其他退出: ").strip().lower()
    if confirm != 'y':
        print("已取消")
        return

    # 生成摆动点：中心 -> 正向最大 -> 负向最大 -> 中心
    targets = []
    t = center
    while t <= center + amplitude:
        targets.append(t)
        t += step
    t = center + amplitude - step
    while t >= center - amplitude:
        targets.append(t)
        t -= step
    t = center - amplitude + step
    while t <= center:
        targets.append(t)
        t += step

    print("\n" + "-" * 110)
    print(f"{'step':>3} | pulses            | J1 (x,y)      | J2 (x,y)      | end (x,y)     | cam (x,y)")
    print("-" * 110)

    measured = []
    try:
        for i, target in enumerate(targets):
            pulses = move_and_read(arm, test_joint, target, fixed_pulses, speed, acc)
            if -1 in pulses:
                print(f"第 {i} 步读取失败，跳过")
                continue
            pos = forward_kinematics(*pulses)
            print_table_row(i, pulses, pos)

            if manual_mode:
                try:
                    end_x = float(input("  手测 end x (mm)，无则回车跳过: ") or "nan")
                    end_y = float(input("  手测 end y (mm)，无则回车跳过: ") or "nan")
                    cam_x = float(input("  手测 cam x (mm)，无则回车跳过: ") or "nan")
                    cam_y = float(input("  手测 cam y (mm)，无则回车跳过: ") or "nan")
                    measured.append({
                        "pulses": pulses,
                        "end": (end_x, end_y),
                        "cam": (cam_x, cam_y),
                    })
                except ValueError:
                    print("  输入格式错误，跳过本步测量")

            time.sleep(delay)

    except KeyboardInterrupt:
        print("\n用户中断，回到中心位置...")

    # 回到中心
    for sid, val in fixed_pulses.items():
        arm.packetHandler.WritePosEx(sid, val, speed, acc)
    arm.packetHandler.WritePosEx(test_joint, center, speed, acc)
    arm.wait_for_position({**fixed_pulses, test_joint: center}, timeout=5.0)
    print("\n已回到中心位置")

    if manual_mode and len(measured) >= 3:
        print("\n>>> 手测数据分析：")
        for key in ("end", "cam"):
            pts = [(m[key][0], m[key][1]) for m in measured
                   if not (math.isnan(m[key][0]) or math.isnan(m[key][1]))]
            if len(pts) < 3:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            radius = sum(math.hypot(p[0]-cx, p[1]-cy) for p in pts) / len(pts)
            print(f"  {key}: 实测圆心 ≈ ({cx:.1f}, {cy:.1f}), 实测平均半径 ≈ {radius:.1f} mm")


def main():
    parser = argparse.ArgumentParser(description="单关节圆周运动测试")
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
    parser.add_argument("--manual", action="store_true",
                        help="每步提示输入手测坐标，用于计算实际圆心/半径")
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    arm = ArmController(
        device=cfg["hardware"]["arm_serial_port"],
        cfg={**cfg["arm"], "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]},
    )

    speed = int(cfg["arm"]["moving_speed"])
    acc = int(cfg["arm"]["moving_acc"])

    # 启动后先回到竖直零位（3/4/5 均为 2047）
    print("\n>>> 正在回到竖直零位（3=2047, 4=2047, 5=2047）...")
    home = {3: 2047, 4: 2047, 5: 2047}
    for sid, val in home.items():
        arm.packetHandler.WritePosEx(sid, val, speed, acc)
    if not arm.wait_for_position(home, timeout=5.0):
        print("警告：回到竖直零位超时")
    else:
        print("已回到竖直零位\n")

    try:
        print("=" * 60)
        print("单关节圆周运动测试")
        print("=" * 60)
        print("请选择要测试的关节：")
        print("  5: 底座旋转关节 -> 观察 J1 是否绕基座画圆（半径≈105mm）")
        print("  4: 第二连杆关节 -> 观察 J2 是否绕 J1 画圆（半径≈110mm）")
        print("  3: 末端连杆关节 -> 观察 end 是否绕 J2 画圆（半径≈110mm）")

        joint = int(input("输入关节号 (3/4/5): ").strip())

        default_center = 2047
        default_amp = {5: 200, 4: 400, 3: 600}.get(joint, 300)
        default_step = {5: 50, 4: 100, 3: 150}.get(joint, 100)

        center = int(input(f"中心脉冲 (默认 {default_center}): ").strip() or default_center)
        amplitude = int(input(f"摆动幅度 (默认 {default_amp}): ").strip() or default_amp)
        step = int(input(f"步长 (默认 {default_step}): ").strip() or default_step)
        delay = float(input("每步停留时间，秒 (默认 1.0): ").strip() or "1.0")

        # 其余关节默认保持 2047
        fixed_pulses = {sid: 2047 for sid in (3, 4, 5) if sid != joint}

        speed = int(cfg["arm"]["moving_speed"])
        acc = int(cfg["arm"]["moving_acc"])

        run_test(arm, joint, center, amplitude, step, delay,
                 fixed_pulses, speed, acc, manual_mode=args.manual)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.finalize()


if __name__ == "__main__":
    main()
