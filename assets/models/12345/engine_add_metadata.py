#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
给 trtexec 生成的纯 TensorRT .engine 文件补写 ultralytics 元数据头。

背景：
  ultralytics 8.1.0 加载 .engine 时，按如下格式读取文件开头：
      [4 字节小端长度][JSON 元数据][TensorRT 序列化数据]
  trtexec 直接转出的 .engine 没有这个头，YOLO() 加载时会报：
      'utf-8' codec can't decode byte 0xXX ... invalid continuation byte
  本脚本把元数据头补到文件开头，补完后即可被 YOLO(engine_path) 正常加载。

用法（在 Jetson 上，对本项目的两个新模型）：
    cd /home/ysc/2026YuYaoGuoSai/assets/models
    python /home/ysc/2026YuYaoGuoSai/tools/engine_add_metadata.py \
        gauge_regions_3d.engine --task detect --names gauge red
    python /home/ysc/2026YuYaoGuoSai/tools/engine_add_metadata.py \
        gauge_pointer_3d_v3.engine --task pose --names pointer --kpt-shape 2 3

说明：
  - 只依赖 Python 标准库，无需激活 yolov8_env
  - 覆盖前会把原文件备份为 <文件名>.bak
  - 如果文件已有合法元数据头则默认跳过，--force 可强制重写
"""

import argparse
import json
import os
import sys
from datetime import datetime


def read_existing_metadata(path):
    """若文件开头已有合法 ultralytics 元数据头则返回 dict，否则返回 None。"""
    with open(path, 'rb') as f:
        head = f.read(4)
        if len(head) < 4:
            return None
        meta_len = int.from_bytes(head, byteorder='little')
        if not 0 < meta_len < 65536:
            return None
        raw = f.read(meta_len)
        if len(raw) < meta_len:
            return None
    try:
        meta = json.loads(raw.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    return meta if isinstance(meta, dict) and 'task' in meta else None


def build_metadata(task, names, imgsz, stride, batch, version, kpt_shape=None):
    """按 ultralytics 8.1.0 export 的格式构造元数据 dict（键顺序保持一致）。"""
    meta = {
        'description': f'Ultralytics {task} model (engine built by trtexec, metadata added manually)',
        'author': 'Ultralytics',
        'license': 'AGPL-3.0 https://ultralytics.com/license',
        'date': datetime.now().isoformat(),
        'version': version,
        'stride': stride,
        'task': task,
        'batch': batch,
        'imgsz': list(imgsz),
        'names': {str(i): n for i, n in enumerate(names)},
    }
    if task == 'pose':
        if kpt_shape is None:
            raise ValueError('pose 模型必须提供 --kpt-shape，例如 --kpt-shape 2 3')
        meta['kpt_shape'] = list(kpt_shape)
    return meta


def main():
    parser = argparse.ArgumentParser(description='给 trtexec 的 .engine 补写 ultralytics 元数据头')
    parser.add_argument('engine', help='trtexec 生成的 .engine 文件路径')
    parser.add_argument('--task', required=True, choices=['detect', 'pose', 'segment', 'classify'],
                        help='模型任务类型')
    parser.add_argument('--names', required=True, nargs='+',
                        help='类别名，按类别索引顺序给出，例如 --names gauge red')
    parser.add_argument('--kpt-shape', type=int, nargs=2, default=None, metavar=('N_KPT', 'DIM'),
                        help='pose 模型关键点形状，例如 --kpt-shape 2 3')
    parser.add_argument('--imgsz', type=int, nargs=2, default=[640, 640], metavar=('H', 'W'),
                        help='推理尺寸，默认 640 640')
    parser.add_argument('--stride', type=int, default=32, help='模型 stride，默认 32')
    parser.add_argument('--batch', type=int, default=1, help='batch size，默认 1')
    parser.add_argument('--version', default='8.1.0',
                        help='目标环境的 ultralytics 版本，默认 8.1.0')
    parser.add_argument('--force', action='store_true', help='即使已有元数据头也强制重写')
    args = parser.parse_args()

    if not os.path.isfile(args.engine):
        print(f'错误：找不到文件 {args.engine}', file=sys.stderr)
        sys.exit(1)

    existing = read_existing_metadata(args.engine)
    if existing is not None and not args.force:
        print(f'{args.engine} 已有元数据头，无需处理：')
        print(json.dumps(existing, ensure_ascii=False, indent=2))
        print('如需重写请加 --force')
        return

    with open(args.engine, 'rb') as f:
        engine_bytes = f.read()

    try:
        meta = build_metadata(
            task=args.task,
            names=args.names,
            imgsz=args.imgsz,
            stride=args.stride,
            batch=args.batch,
            version=args.version,
            kpt_shape=args.kpt_shape,
        )
    except ValueError as e:
        print(f'错误：{e}', file=sys.stderr)
        sys.exit(1)

    meta_bytes = json.dumps(meta).encode('utf-8')

    backup = args.engine + '.bak'
    if not os.path.exists(backup):
        with open(backup, 'wb') as f:
            f.write(engine_bytes)
        print(f'原文件已备份到 {backup}')

    with open(args.engine, 'wb') as f:
        f.write(len(meta_bytes).to_bytes(4, byteorder='little'))
        f.write(meta_bytes)
        f.write(engine_bytes)

    # 回读验证
    check = read_existing_metadata(args.engine)
    if check is None:
        print('错误：写入后回读验证失败，请检查文件', file=sys.stderr)
        sys.exit(1)

    print(f'✓ 元数据头已写入 {args.engine}')
    print(json.dumps(check, ensure_ascii=False, indent=2))
    print('现在可以用 YOLO() 直接加载该 engine 了')


if __name__ == '__main__':
    main()
