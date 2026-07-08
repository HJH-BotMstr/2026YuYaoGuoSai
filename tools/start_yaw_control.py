#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""一键启动 Lite3 驱动 + 交互式偏航控制器。

驱动在后台运行，控制器在前台运行，可以直接在终端输入命令。
退出控制器时会自动终止驱动进程。

用法：
    python tools/start_yaw_control.py

可选参数：
    --show-driver     在当前终端同时显示驱动输出（会和控制器刷屏冲突）
    --driver-log PATH 指定驱动日志路径（默认 project_root/driver.log）
"""

import argparse
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DRIVER_CMD = ["ros2", "run", "motion_test", "lite3_driver_node"]
CONTROLLER_CMD = [
    sys.executable,
    str(PROJECT_ROOT / "tools" / "yaw_controller.py"),
]


def main():
    parser = argparse.ArgumentParser(
        description="Start Lite3 driver and interactive yaw controller"
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
    args = parser.parse_args()

    if args.show_driver:
        driver_stdout = None
        driver_stderr = None
        log_file = None
    else:
        log_file = args.driver_log.open("w", encoding="utf-8")
        driver_stdout = log_file
        driver_stderr = subprocess.STDOUT
        print(f"驱动日志将写入: {args.driver_log}")

    print("正在启动 lite3_driver_node ...")
    driver_proc = subprocess.Popen(
        DRIVER_CMD,
        stdout=driver_stdout,
        stderr=driver_stderr,
        cwd=PROJECT_ROOT,
    )

    exit_code = 0
    try:
        print("正在启动 yaw_controller，请在下方输入命令 ...\n")
        result = subprocess.run(CONTROLLER_CMD, cwd=PROJECT_ROOT)
        exit_code = result.returncode
    except KeyboardInterrupt:
        pass
    finally:
        print("\n正在停止 lite3_driver_node ...")
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
