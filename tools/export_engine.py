#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 Jetson Xavier NX 上将 YOLOv8 .pt 导出为 TensorRT .engine。

用法：
    source ~/yolov8_env/bin/activate
    python export_engine.py /path/to/best.pt

导出后的文件和原 .pt 在同一目录，例如 best.engine。
"""

import argparse
import os
import sys

from ultralytics import YOLO
from ultralytics.nn.modules.head import Detect, Pose


def patch_pose_head(model):
    """
    兼容用新版 ultralytics 训练的 pose 模型。
    新版 Pose head 不再保存 self.detect 属性，而 ultralytics 8.1.0 需要它。
    """
    try:
        head = model.model.model[-1]
    except Exception:
        return
    if isinstance(head, Pose) and not hasattr(head, 'detect'):
        head.detect = Detect.forward
        print('已自动修补 Pose head')


def main():
    parser = argparse.ArgumentParser(description='YOLOv8 pt -> TensorRT engine')
    parser.add_argument('weights', help='输入 .pt 文件路径')
    parser.add_argument('--imgsz', type=int, nargs='+', default=[640],
                        help='推理尺寸，默认 640')
    parser.add_argument('--half', action='store_true', default=True,
                        help='使用 FP16，默认开启')
    parser.add_argument('--device', type=int, default=0,
                        help='GPU 设备号，默认 0')
    parser.add_argument('--workspace', type=int, default=2,
                        help='TensorRT 工作区大小（GB），Xavier NX 建议 1~2，默认 2')
    args = parser.parse_args()

    if not os.path.isfile(args.weights):
        print(f'错误：找不到模型文件 {args.weights}', file=sys.stderr)
        sys.exit(1)

    model = YOLO(args.weights)
    patch_pose_head(model)

    # ultralytics 的 export 会自动走 ONNX -> TensorRT 流程
    # half=True 在 Xavier NX 上通常有显著加速
    model.export(
        format='engine',
        imgsz=args.imgsz,
        half=args.half,
        device=args.device,
        workspace=args.workspace,
    )

    engine_path = os.path.splitext(args.weights)[0] + '.engine'
    if os.path.isfile(engine_path):
        print(f'导出成功：{engine_path}')
    else:
        print('导出完成，但未找到预期的 .engine 文件，请检查上方日志')


if __name__ == '__main__':
    main()
