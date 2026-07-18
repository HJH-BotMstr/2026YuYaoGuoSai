---
title: YOLOv8 仪表盘识别实现说明
date: 2026-07-19
tags: [yolo, jetson, tensorrt, gauge, vision]
---

# YOLOv8 仪表盘识别实现说明

本文档说明机器狗上基于 **YOLOv8 + TensorRT** 的仪表盘识别方案：如何训练、导出、部署，以及最终如何运行。

## 1. 方案概述

相比传统霍夫圆 + 颜色阈值的方案，YOLO 版把识别拆成两个专门的检测模型：

| 模型 | 任务 | 输出 | 文件名 |
|---|---|---|---|
| `best_bg` | 目标检测 | 表盘外框（class 0）、红色警戒区域（class 1） | `best_bg.pt` / `best_bg.engine` |
| `best_ptr` | 关键点检测 | 指针铆钉（keypoint 0）、指针针尖（keypoint 1） | `best_ptr.pt` / `best_ptr.engine` |

推理流程：

1. `best_bg` 找出表盘框和红色区域框，计算表盘中心与“上方向”。
2. `best_ptr` 找出指针两个端点，计算指针角度。
3. 根据相对角度判断指针所在区域：
   - 0°~45° → 偏高（红区，异常）
   - 45°~135° → 居中（绿区，正常）
   - 其余 → 偏低（黄区，异常）
4. 用 Tesseract OCR 识别表盘上方的 A/B/C/D 字母。
5. 根据 `(字母, 区域)` 播放对应的预录 MP3 语音。

## 2. 关键文件

```text
/home/ysc/2026YuYaoGuoSai/
├── assets/
│   ├── models/
│   │   ├── best_bg.pt          # 自己电脑训练的 regions 模型
│   │   ├── best_bg.engine      # Jetson 上导出的 TensorRT engine
│   │   ├── best_ptr.pt         # 自己电脑训练的 pointer 模型
│   │   └── best_ptr.engine     # Jetson 上导出的 TensorRT engine
│   ├── mp3/
│   │   ├── AL.mp3 ~ DM.mp3     # 预录语音（A/B/C/D × 低/高/中）
│   └── letter_debug/           # --debug-letter 保存的字母 ROI（运行时生成，可删）
├── doc/
│   └── gauge_yolo_usage.md     # 本文档
├── tools/
│   ├── gauge_yolo_new.py       # 最终可运行的实时识别脚本
│   ├── gauge_yolo.py           # 原始单线程脚本（未使用）
│   └── yolo/
│       ├── export_engine.py    # 用于把 .pt 导出为 .engine
│       └── infer_engine.py     # 单张图片/摄像头推理测试（可选）
└── yolov8n-pose.pt             # 临时下载的 COCO 预训练模型，可删除
```

### 可以删除的文件

- `yolov8n-pose.pt`：调试时意外下载的 COCO 预训练权重，与项目无关，**可以删除**。
- `tools/yolo/infer_engine.py`：最终没有用到，**可以删除**。
- `tools/yolo/__pycache__/`：Python 缓存，提交前删除或加入 `.gitignore`。

## 3. Jetson 环境

在 `~/yolov8_env` 虚拟环境中安装：

| 依赖 | 版本/来源 | 说明 |
|---|---|---|
| Python | 3.8 | JetPack 5.1.2 默认 |
| PyTorch | 2.1.0 (NVIDIA wheel) | `torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl` |
| torchvision | 0.16.2 | 源码编译 |
| ultralytics | 8.1.0 | 新版依赖太多/需要编译，固定此版本 |
| onnx | 1.17.0 | engine 导出需要 |
| numpy | 1.23.5 | 必须降到此版本，`1.24+` 会报 `np.bool` 错误 |
| OpenCV | 4.2.0 (系统 apt) | 使用系统 `python3-opencv`，支持 `cv2.imshow` |
| pytesseract | pip | Python 调用 Tesseract |
| TensorRT | 8.5.2.2 (系统) | 通过软链接把系统包接到 venv |

系统 apt 需要安装：

```bash
sudo apt install -y tesseract-ocr tesseract-ocr-eng \
                    v4l-utils ffmpeg alsa-utils pulseaudio \
                    python3-opencv
```

TensorRT Python 绑定接入 venv：

```bash
site_pkg=$(python -c "import site; print(site.getsitepackages()[0])")
ln -s /usr/lib/python3.8/dist-packages/tensorrt "$site_pkg/tensorrt"
```

## 4. 模型训练与导出

### 4.1 训练

- `best_bg.pt`：在自己电脑上训练 YOLOv8 检测模型，类别为 `gauge`、`red`。
- `best_ptr.pt`：在自己电脑上训练 YOLOv8 关键点模型，两个关键点 `rivet`、`tip`。
- 训练完成后把两个 `.pt` 复制到 `assets/models/`。

### 4.2 导出 TensorRT engine

在 Jetson 上激活虚拟环境后运行：

```bash
source ~/yolov8_env/bin/activate

python /home/ysc/2026YuYaoGuoSai/tools/yolo/export_engine.py \
  /home/ysc/2026YuYaoGuoSai/assets/models/best_bg.pt --workspace 2

python /home/ysc/2026YuYaoGuoSai/tools/yolo/export_engine.py \
  /home/ysc/2026YuYaoGuoSai/assets/models/best_ptr.pt --workspace 2
```

> `workspace` 必须是整数（GB）。Xavier NX 显存紧张，建议 `1` 或 `2`。

导出成功后会在 `assets/models/` 生成：

```text
best_bg.engine
best_ptr.engine
```

### 4.3 Pose 模型兼容性补丁

`best_ptr.pt` 用新版 ultralytics 训练，Pose head 缺少 `self.detect`。脚本里已自动修补：

```python
from ultralytics.nn.modules.head import Detect, Pose

head = model.model.model[-1]
if isinstance(head, Pose) and not hasattr(head, 'detect'):
    head.detect = Detect.forward
```

## 5. 运行识别

### 5.1 带画面

```bash
cd /home/ysc/2026YuYaoGuoSai/tools
python gauge_yolo_new.py
```

需要在有显示输出的终端运行（本地桌面或 `ssh -X`）。

### 5.2 无画面

```bash
python gauge_yolo_new.py --no-display
```

终端只输出简洁状态，例如：

```text
A,绿,正常
B,红,异常
C,黄,异常
```

### 5.3 关闭语音

```bash
python gauge_yolo_new.py --no-voice
```

### 5.4 调试字母识别

```bash
python gauge_yolo_new.py --debug-letter --no-display
```

会在 `assets/letter_debug/` 保存每次 OCR 的 ROI 图片，并打印原始 OCR 结果。

## 6. 实现要点

### 6.1 多线程

- `capture_thread`：高频率取最新帧，保证画面低延迟。
- `process_thread`：每 `--interval` 秒（默认 0.3s）做一次推理。
- 主线程：显示画面并等待按键退出。

### 6.2 摄像头

- 默认 `/dev/video4`，通过 `v4l2-ctl` 开启自动曝光和白平衡。
- 启动时预热 80 帧，丢弃旧帧避免延迟。

### 6.3 角度计算

- 上方向：表盘中心 → 红色区域中心。
- 指针方向：铆钉 → 针尖。
- 相对角度 `(ptr - up) % 360` 后分类。

### 6.4 字母识别

- 基于 `best_bg` 检测到的表盘矩形框。
- 优先识别框上方区域；若为空， fallback 到框内上半部分。
- 每 5 帧跑一次 OCR，其余帧复用上次结果。

### 6.5 语音播报

- 仅当 `(字母, 区域)` 变化时播放，避免重复。
- 文件名规则：`{字母}{后缀}.mp3`，后缀 `L=偏低`、`H=偏高`、`M=居中`。

## 7. 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `module 'numpy' has no attribute 'bool'` | numpy 1.24+ 删除了 `np.bool` | `pip install numpy==1.23.5` |
| `AttributeError: 'Pose' object has no attribute 'detect'` | pose 模型版本与 ultralytics 8.1.0 不兼容 | 脚本已自动修补 |
| `workspace=2.0` 类型错误 | ultralytics 8.1.0 要求 `workspace` 为 int | 使用 `--workspace 2` |
| `cv2.imshow` 报 `function is not implemented` | 用了 `opencv-python-headless` | 换系统 `python3-opencv` |
| `cannot open display` | SSH 没有 X11 转发 | 本地运行或 `ssh -X` |
| 字母一直识别不到 | ROI 位置不对或 Tesseract 没装 | 用 `--debug-letter` 看 ROI；确认 `tesseract-ocr` 和 `pytesseract` 已装 |

## 8. 后续可优化

- 当前字母 OCR 依赖 Tesseract，若字母位置变化大，可在 `recognize_letter_box` 里调整 ROI。
- 若需要更高帧率，可进一步降低推理尺寸 `--imgsz 480` 或缩短 `--interval`。
- 工程文件较大，提交云端时建议：
  - 不提交 `.pt`、`.engine`、`.onnx` 模型文件。
  - 不提交 `__pycache__/`、`letter_debug/`。
  - 保留 `export_engine.py` 和 `gauge_yolo_new.py`，方便复现。
