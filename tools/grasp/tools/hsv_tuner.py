#!/usr/bin/env python3
"""
HSV 实时标定工具
用滑动条调整 H/S/V 范围，实时预览掩码效果，按 p 打印当前参数。

用法：
  python3 tools/hsv_tuner.py                        # 使用 config.yaml 的摄像头
  python3 tools/hsv_tuner.py --device /dev/video4   # 指定摄像头
  python3 tools/hsv_tuner.py --image frame.jpg       # 用静态图片（不需要摄像头）

操作：
  p   — 打印当前 HSV 范围（复制到 config.yaml）
  s   — 保存当前帧到 captured.jpg
  q   — 退出
"""
import sys
import os
import argparse
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"
os.environ.pop("QT_QPA_PLATFORM", None)
os.environ["QT_QPA_PLATFORM"] = "xcb"
import cv2
import numpy as np
import yaml

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')

WIN_ORIG = "原图 (p=打印参数  s=保存帧  q=退出)"
WIN_MASK = "掩码 (白色=检测到)"


def nothing(_): pass


def create_trackbars():
    cv2.namedWindow(WIN_MASK, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_MASK, 640, 400)   # 先确保窗口创建完成
    cv2.waitKey(1)
    # 默认值：浅粉/浅红色大致范围
    cv2.createTrackbar("H min", WIN_MASK, 0,   179, nothing)
    cv2.createTrackbar("H max", WIN_MASK, 20,  179, nothing)
    cv2.createTrackbar("S min", WIN_MASK, 30,  255, nothing)
    cv2.createTrackbar("S max", WIN_MASK, 255, 255, nothing)
    cv2.createTrackbar("V min", WIN_MASK, 100, 255, nothing)
    cv2.createTrackbar("V max", WIN_MASK, 255, 255, nothing)


def get_range():
    h_min = cv2.getTrackbarPos("H min", WIN_MASK)
    h_max = cv2.getTrackbarPos("H max", WIN_MASK)
    s_min = cv2.getTrackbarPos("S min", WIN_MASK)
    s_max = cv2.getTrackbarPos("S max", WIN_MASK)
    v_min = cv2.getTrackbarPos("V min", WIN_MASK)
    v_max = cv2.getTrackbarPos("V max", WIN_MASK)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    return lower, upper


def process(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower, upper = get_range()
    mask = cv2.inRange(hsv, lower, upper)
    # 形态学去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # 在原图上叠加轮廓
    vis = frame.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, contours, -1, (0, 255, 0), 2)
    # 打印像素 HSV 值（点击位置）
    return vis, mask, lower, upper


def print_params(lower, upper):
    h_min, s_min, v_min = lower.tolist()
    h_max, s_max, v_max = upper.tolist()
    print("\n===== 当前 HSV 参数（复制到 config.yaml）=====")
    print(f"  hsv_red_lower1: [{h_min}, {s_min}, {v_min}]")
    print(f"  hsv_red_upper1: [{h_max}, {s_max}, {v_max}]")
    print("提示：浅粉色通常 H≈0-15, S≈30-120, V≈150-255")
    print("      若颜色偏粉（饱和度低），降低 S min 到 30~60")
    print("===============================================\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--device", default=None)
    parser.add_argument("--image",  default=None, help="用静态图片代替摄像头")
    args = parser.parse_args()

    create_trackbars()
    cv2.namedWindow(WIN_ORIG, cv2.WINDOW_NORMAL)

    # 静态图片模式
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"无法读取图片: {args.image}")
            sys.exit(1)
        print(f"静态图片模式: {args.image}  (按 p 打印参数, q 退出)")
        while True:
            vis, mask, lower, upper = process(frame)
            cv2.imshow(WIN_ORIG, vis)
            cv2.imshow(WIN_MASK, mask)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                print_params(lower, upper)
        cv2.destroyAllWindows()
        return

    # 摄像头模式
    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    device = args.device or cfg["hardware"]["arm_cam_device"]
    print(f"打开摄像头: {device}  (按 p 打印参数, s 保存帧, q 退出)")

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"无法打开摄像头: {device}")
        sys.exit(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        vis, mask, lower, upper = process(frame)

        # 叠加当前参数文字
        h_min, s_min, v_min = lower.tolist()
        h_max, s_max, v_max = upper.tolist()
        info = f"H:{h_min}-{h_max}  S:{s_min}-{s_max}  V:{v_min}-{v_max}"
        cv2.putText(vis, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow(WIN_ORIG, vis)
        cv2.imshow(WIN_MASK, mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            print_params(lower, upper)
        elif key == ord('s'):
            cv2.imwrite("captured.jpg", frame)
            print("已保存 captured.jpg")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
