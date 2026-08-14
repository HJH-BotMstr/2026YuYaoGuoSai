#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试狗抬头角度的工具脚本
用途：打开摄像头实时显示画面，通过按键调整俯仰角，找到最佳识别角度
"""

import cv2
import socket
import struct
import sys
import time


def send_voice_command(command_value):
    """
    发送语音指令到运动主机
    command_value: 9=抬头, 8=低头
    返回: (成功, 错误信息)
    """
    robot_ip = "192.168.1.120"
    robot_port = 43893
    cmd_code = 0x21010C0A  # 语音指令码

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        data = struct.pack("<IiI", cmd_code, command_value, 0)
        sock.sendto(data, (robot_ip, robot_port))
        return True, None
    except Exception as e:
        return False, str(e)
    finally:
        sock.close()


def init_camera(camera_id=6, width=640, height=480):
    """初始化摄像头"""
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main():
    print("=" * 60)
    print("狗抬头角度测试工具")
    print("=" * 60)
    print("用途：调整俯仰角，找到能清晰识别仪表盘+字母的最佳角度")
    print()
    print("控制说明：")
    print("  W 键：发送抬头指令")
    print("  S 键：发送低头指令")
    print("  空格：确认当前角度（记录并继续显示）")
    print("  Q 键：退出程序")
    print()
    print("注意：")
    print("  1. 确保狗已站立并处于原地模式")
    print("  2. 语音指令会让狗执行预设动作，不是连续调整")
    print("  3. 如果需要微调，使用遥控器手动调整俯仰角")
    print("  4. 找到合适角度后按空格记录，退出后告诉我")
    print("=" * 60)
    print()

    camera_id = 6
    input_camera = input(f"请输入摄像头 ID（默认 {camera_id}，直接回车使用默认值）: ").strip()
    if input_camera:
        camera_id = int(input_camera)

    print(f"\n初始化摄像头 /dev/video{camera_id} ...")
    cap = init_camera(camera_id)
    if cap is None:
        print(f"❌ 无法打开摄像头 /dev/video{camera_id}")
        return 1

    print("✓ 摄像头已打开")
    print("\n预热中，丢弃前 30 帧...")
    for _ in range(30):
        cap.grab()
    print("✓ 预热完成")

    print("\n窗口即将打开，按照提示操作...")
    print("=" * 60)

    confirmed_angles = []
    window_name = "Head Pitch Test - Press W/S to adjust, SPACE to confirm, Q to quit"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠ 读取帧失败")
            time.sleep(0.1)
            continue

        # 在画面上叠加提示信息
        h, w = frame.shape[:2]

        # 半透明背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (w - 10, 150), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

        # 文字提示
        y = 35
        cv2.putText(frame, "W: Head UP", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        y += 30
        cv2.putText(frame, "S: Head DOWN", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y += 30
        cv2.putText(frame, "SPACE: Confirm angle", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        y += 30
        cv2.putText(frame, "Q: Quit", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if confirmed_angles:
            y += 40
            cv2.putText(frame, f"Confirmed: {len(confirmed_angles)} angle(s)",
                       (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == ord('Q'):
            print("\n退出程序...")
            break

        elif key == ord('w') or key == ord('W'):
            print("\n>>> 发送抬头指令...")
            success, err = send_voice_command(9)
            if success:
                print("✓ 抬头指令已发送")
            else:
                print(f"❌ 发送失败: {err}")

        elif key == ord('s') or key == ord('S'):
            print("\n>>> 发送低头指令...")
            success, err = send_voice_command(8)
            if success:
                print("✓ 低头指令已发送")
            else:
                print(f"❌ 发送失败: {err}")

        elif key == ord(' '):
            timestamp = time.strftime("%H:%M:%S")
            confirmed_angles.append(timestamp)
            print(f"\n✓ 已记录当前角度 [{timestamp}]（总计 {len(confirmed_angles)} 个）")
            print("  如果这个角度合适，退出后告诉我你按了几次 W/S")

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    print("测试完成！")
    if confirmed_angles:
        print(f"你确认了 {len(confirmed_angles)} 个角度，记录时间：")
        for i, t in enumerate(confirmed_angles, 1):
            print(f"  {i}. {t}")
    else:
        print("未记录任何角度")
    print("\n请告诉我：")
    print("  1. 最终确认的角度是按了几次 W（抬头）或 S（低头）")
    print("  2. 或者是用遥控器手动调整的（告诉我大概角度描述）")
    print("=" * 60)

    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n用户中断")
        sys.exit(1)
