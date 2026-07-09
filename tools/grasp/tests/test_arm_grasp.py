#!/usr/bin/env python3
"""
机械臂硬件单步抓取测试脚本
用途机械臂连接 Lite3 感知主机时，验证基础动作和单次完整抓取流程
中断方式：
  Ctrl+C  — 安全归位
  E + 回车 — 急停（所有舵机立即断力矩，机械臂可能因重力下垂，注意安全）
"""
import sys
import os
import argparse
import time
import threading
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.ArmController import ArmController

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')

# 全局急停标志
_estop_requested = False


def _estop_listener(arm: ArmController):
    """后台线程：等待用户输入 e/E 触发急停。"""
    global _estop_requested
    while not _estop_requested:
        try:
            key = input()
        except EOFError:
            break
        if key.strip().lower() == 'e':
            _estop_requested = True
            print("\n!!! 急停触发 — 所有舵机断力矩 !!!")
            print("注意：机械臂可能因重力下垂，确认周围安全后再操作。")
            arm.emergency_stop()
            break


def load_cfg(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_basic_poses(arm: ArmController):
    print("\n[1/4] 初始姿态 (mode=0)，观察 3 秒...")
    arm.set_pose(0)
    time.sleep(3)

    print("[2/4] 识别/运动姿态 (mode=2)，观察 3 秒...")
    arm.set_pose(2)
    time.sleep(3)

    print("[3/4] 运输姿态水平 (mode=3)，观察 3 秒...")
    arm.set_pose(3)
    time.sleep(3)

    print("[4/4] 回到初始姿态 (mode=0)...")
    arm.set_pose(0)
    time.sleep(2)
    print("基础姿态测试完成。")


def test_gripper(arm: ArmController):
    """测试夹爪开合。"""
    print("\n[夹爪] 张开...")
    arm.open_gripper()
    time.sleep(2)
    print("[夹爪] 闭合...")
    arm.close_gripper()
    time.sleep(2)
    print("[夹爪] 张开（复位）...")
    arm.open_gripper()
    time.sleep(1)
    print("夹爪测试完成。")


def test_single_grasp(arm: ArmController, dis: float, height: float):
    """单次完整抓取流程（不含视觉，手动给定位置）。"""
    print(f"\n[抓取] dis={dis}mm, height={height}mm")
    print("机械臂进入运动姿态...")
    arm.set_pose(2)
    time.sleep(3)

    print("5 秒后开始抓取，期间可按 Ctrl+C 中止...")
    for i in range(5, 0, -1):
        print(f"  {i}...", flush=True)
        time.sleep(1)
    ok = arm.grasp_with_verify(dis=dis, height=height)
    if ok:
        print("抓取成功！进入运输姿态...")
        arm.set_pose(3)
        time.sleep(3)
        print("3 秒后松开夹爪...")
        time.sleep(3)
        arm.open_gripper()
        time.sleep(1)
    else:
        print("抓取失败（超过最大重试次数）。")

    print("归位...")
    arm.set_pose(0)
    time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description="机械臂硬件测试")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode",
                        choices=["pose", "gripper", "grasp", "all"],
                        default="all",
                        help="测试模式: pose/gripper/grasp/all")
    parser.add_argument("--dis",    type=float, default=200.0,
                        help="抓取水平距离 mm（grasp 模式）")
    parser.add_argument("--height", type=float, default=30.0,
                        help="抓取末端高度 mm（grasp 模式）")
    args = parser.parse_args()

    cfg = load_cfg(args.config)
    port = cfg["hardware"]["arm_serial_port"]
    arm_cfg = {**cfg["arm"],
               "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]}

    print(f"连接机械臂串口: {port}")
    print("提示：运行中按 E + 回车 急停（断力矩）；Ctrl+C 安全归位后退出")
    arm = ArmController(device=port, cfg=arm_cfg)

    # 启动急停监听线程
    estop_thread = threading.Thread(target=_estop_listener, args=(arm,), daemon=True)
    estop_thread.start()

    try:
        if args.mode in ("pose", "all"):
            test_basic_poses(arm)
        if args.mode in ("gripper", "all"):
            test_gripper(arm)
        if args.mode in ("grasp", "all"):
            test_single_grasp(arm, args.dis, args.height)
        if not _estop_requested:
            print("\n所有测试完成。")
    except KeyboardInterrupt:
        if _estop_requested:
            print("\n急停已触发，跳过归位。")
        else:
            print("\nCtrl+C：执行安全归位...")
            arm.set_pose(0)
            time.sleep(2)
    finally:
        arm.finalize()
        print("串口已关闭。")


if __name__ == "__main__":
    main()
