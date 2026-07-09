#!/usr/bin/env python3
"""
色块识别实时调试脚本
用途：不需要机械臂，只用摄像头，实时显示检测结果，用于现场 HSV 标定验证。

运行方法：
  python3 tests/test_block_detection.py
  python3 tests/test_block_detection.py --config config.yaml --device /dev/video2

操作：
  q 退出
  s 保存当前帧到 debug_frame.jpg
"""
import sys
import os
import argparse
import yaml
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.BlockDetection import BlockDetection

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')


def main():
    parser = argparse.ArgumentParser(description="色块识别实时调试")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--device", default=None,
                        help="摄像头设备路径（覆盖 config.yaml 的值）")
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    device = args.device or cfg["hardware"]["arm_cam_device"]
    detector = BlockDetection(cfg["detection"])

    print(f"打开摄像头: {device}")
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"错误：无法打开摄像头 {device}")
        sys.exit(1)

    print("按 q 退出，按 s 保存当前帧")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("读帧失败，跳过")
            continue

        result = detector.detect(frame)
        vis = detector.visualize(frame.copy(), result)

        # 状态栏
        if result:
            status = (f"color={result['color']}  "
                      f"offset_x={result['center_offset_x']}px  "
                      f"dist={result['distance_mm']:.0f}mm")
            color_bgr = (0, 0, 255) if result["color"] == "red" else (0, 200, 0)
        else:
            status = "no block detected"
            color_bgr = (128, 128, 128)

        cv2.putText(vis, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        cv2.imshow("BlockDetection debug", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite("debug_frame.jpg", frame)
            print("已保存 debug_frame.jpg")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
