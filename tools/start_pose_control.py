#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""一键启动位姿控制器。

默认情况下只启动 pose_controller.py，依赖机器人官方 ROS2 栈提供：
  - /leg_odom2                          里程计
  - /us_publisher/ultrasound_distance   后超声波
  - /cmd_vel                            速度指令输入

如需使用 standalone 版 lite3_driver.py（无官方栈环境），加 --use-driver。
"""

import argparse
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DRIVER_CMD = [sys.executable, str(PROJECT_ROOT / "tools" / "lite3_driver.py")]
CONTROLLER_CMD = [sys.executable, str(PROJECT_ROOT / "tools" / "pose_controller.py")]


def main():
    parser = argparse.ArgumentParser(
        description="Start Lite3 pose controller"
    )
    parser.add_argument(
        "--use-driver",
        action="store_true",
        help="also start the standalone lite3_driver.py (for environments without the official stack)",
    )
    parser.add_argument(
        "--show-driver",
        action="store_true",
        help="show driver output in the same terminal (only with --use-driver)",
    )
    parser.add_argument(
        "--driver-log",
        type=Path,
        default=PROJECT_ROOT / "driver.log",
        help="path to driver log file (only with --use-driver)",
    )
    args = parser.parse_args()

    driver_proc = None
    log_file = None

    if args.use_driver:
        driver_cmd = list(DRIVER_CMD)
        if args.show_driver:
            driver_stdout = None
            driver_stderr = None
        else:
            log_file = args.driver_log.open("w", encoding="utf-8")
            driver_stdout = log_file
            driver_stderr = subprocess.STDOUT
            print(f"驱动日志将写入: {args.driver_log}")

        print("正在启动 lite3_driver.py ...")
        driver_proc = subprocess.Popen(
            driver_cmd,
            stdout=driver_stdout,
            stderr=driver_stderr,
            cwd=PROJECT_ROOT,
        )

    print("正在启动 pose_controller.py，请在下方输入命令 ...\n")
    exit_code = 0
    try:
        result = subprocess.run(CONTROLLER_CMD, cwd=PROJECT_ROOT)
        exit_code = result.returncode
    except KeyboardInterrupt:
        pass
    finally:
        if driver_proc is not None:
            print("\n正在停止 lite3_driver.py ...")
            driver_proc.terminate()
            try:
                driver_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                print("驱动未能在 5s 内退出，强制结束 ...")
                driver_proc.kill()
                driver_proc.wait()
            if log_file is not None:
                log_file.close()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
