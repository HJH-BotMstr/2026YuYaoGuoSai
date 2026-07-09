#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""一键启动独立版 Lite3 驱动 + 位姿控制器。

驱动在后台运行，控制器在前台运行，可以直接在终端输入元指令。
退出控制器时会自动终止驱动进程。
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
        description="Start standalone Lite3 driver and pose controller"
    )
    parser.add_argument(
        "--show-driver",
        action="store_true",
        help="show driver output in the same terminal (will interleave with controller)",
    )
    parser.add_argument(
        "--driver-log",
        type=Path,
        default=PROJECT_ROOT / "driver.log",
        help="path to driver log file (ignored if --show-driver)",
    )
    parser.add_argument(
        "--sonar",
        action="store_true",
        help="enable ultrasonic parsing in driver",
    )
    parser.add_argument(
        "--sonar-debug",
        action="store_true",
        help="enable ultrasonic debug output in driver",
    )
    args = parser.parse_args()

    driver_cmd = list(DRIVER_CMD)
    if args.sonar or args.sonar_debug:
        ros_args = ["--ros-args"]
        if args.sonar:
            ros_args.extend(["-p", "sonar_enable:=true"])
        if args.sonar_debug:
            ros_args.extend(["-p", "sonar_enable:=true", "-p", "sonar_debug:=true"])
        driver_cmd.extend(ros_args)

    if args.show_driver:
        driver_stdout = None
        driver_stderr = None
        log_file = None
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

    exit_code = 0
    try:
        print("正在启动 pose_controller.py，请在下方输入命令 ...\n")
        result = subprocess.run(CONTROLLER_CMD, cwd=PROJECT_ROOT)
        exit_code = result.returncode
    except KeyboardInterrupt:
        pass
    finally:
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
