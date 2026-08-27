---
title: YOLOv8 仪表盘识别 ROS2 服务节点使用说明
date: 2026-07-19
tags: [ros2, yolo, gauge, service, jetson]
---

# YOLOv8 仪表盘识别 ROS2 服务节点使用说明

本文档说明如何在机器狗上启动和使用 YOLO 版仪表盘识别 ROS2 服务节点。

该节点与纯 CV 版节点 `gauge_detector` 完全独立：

| 项目 | 纯 CV 版 | YOLO 版 |
|---|---|---|
| 包名 | `gauge_detector` | `gauge_yolo_detector` |
| 节点名 | `gauge_server` | `gauge_yolo_server` |
| 服务名 | `/detect_gauge` | `/detect_gauge_yolo` |
| 实现方式 | 霍夫圆 + 颜色阈值 | `gauge_regions_3d.engine` + `gauge_pointer_3d_v3.engine` |

## 1. 前置条件

- Jetson Xavier NX 已刷 JetPack 5.1.2，ROS 2 Foxy 已安装
- 已创建 YOLO 虚拟环境并安装依赖（PyTorch、ultralytics、TensorRT 等）
- 模型文件已到位：
  - `/home/ysc/2026YuYaoGuoSai/assets/models/gauge_regions_3d.engine`
  - `/home/ysc/2026YuYaoGuoSai/assets/models/gauge_pointer_3d_v3.engine`
- 语音文件已到位：
  - `/home/ysc/2026YuYaoGuoSai/assets/mp3/AL.mp3 ~ DM.mp3`

如果还没有 `.engine` 文件，在 Jetson 上生成（engine 不跨设备，必须本机生成）：

```bash
source ~/yolov8_env/bin/activate

# regions（detect 模型）：直接从 .pt 导出
python /home/ysc/2026YuYaoGuoSai/tools/export_engine.py \
  /home/ysc/2026YuYaoGuoSai/assets/models/gauge_regions_3d.pt --workspace 2

# pointer（pose 模型）：8.1.0 导不出 pose，需在训练电脑上导出 .onnx，
# 拷到狗上后用 trtexec 转 engine，再补 ultralytics 元数据头
/usr/src/tensorrt/bin/trtexec \
  --onnx=gauge_pointer_3d_v3.onnx \
  --saveEngine=gauge_pointer_3d_v3.engine \
  --fp16 --workspace=4096
python /home/ysc/2026YuYaoGuoSai/tools/engine_add_metadata.py \
  gauge_pointer_3d_v3.engine --task pose --names pointer --kpt-shape 2 3
```

## 2. 编译

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select gauge_detector_interfaces gauge_yolo_detector --symlink-install
```

## 3. 启动节点

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run gauge_yolo_detector gauge_yolo_server
```

启动后节点会依次完成：

1. 加载 `gauge_regions_3d.engine` 和 `gauge_pointer_3d_v3.engine`（最耗时，约 30~60 秒）
2. 初始化 `/dev/video0`
3. 预热 80 帧
4. 创建 `/detect_gauge_yolo` 服务

之后每次服务调用只做单帧推理，响应很快。

## 4. 服务调用

### 4.1 命令行测试

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash
ros2 service call /detect_gauge_yolo gauge_detector_interfaces/srv/GaugeDetect "{}"
```

### 4.2 返回字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否识别成功 |
| `letter` | `string` | 识别到的字母 `A/B/C/D`，未识别为空 |
| `zone` | `string` | 指针所在区域：`RED` / `GREEN` / `YELLOW` |
| `state` | `string` | 整体状态：`normal`（绿区）/ `abnormal`（红区或黄区） |
| `message` | `string` | 提示信息 |

### 4.3 示例返回

```text
gauge_detector_interfaces.srv.GaugeDetect_Response(
    success=True,
    letter='A',
    zone='GREEN',
    state='normal',
    message='识别成功'
)
```

## 5. 节点参数

启动时可通过 `--ros-args -p 参数名:=值` 修改。

| 参数名 | 默认值 | 说明 |
|---|---|---|
| `camera_id` | `0` | 摄像头编号，对应 `/dev/video0` |
| `width` | `640` | 采集宽度 |
| `height` | `480` | 采集高度 |
| `preheat_frames` | `80` | 启动时丢弃的预热帧数 |
| `models_dir` | `/home/ysc/2026YuYaoGuoSai/assets/models` | 模型目录 |
| `use_engine` | `true` | 是否优先使用 TensorRT `.engine` |
| `device` | `0` | GPU 编号 |
| `imgsz` | `640` | YOLO 推理尺寸 |
| `thr_high` | `41.0` | 偏高/居中边界（相对角度，双侧对称），按 0.7MPa 校准 |
| `thr_low` | `145.0` | 居中/偏低边界（相对角度），按 0.3MPa 校准 |
| `voice_enabled` | `true` | 是否启用语音播报 |
| `mp3_dir` | `/home/ysc/2026YuYaoGuoSai/assets/mp3` | MP3 文件目录 |
| `voice_on_change_only` | `true` | 是否只在状态变化时播报 |
| `letter_skip` | `1` | 每几次识别跑一次 OCR，YOLO 版默认每次调用都识别 |
| `debug_letter` | `false` | 是否保存字母 ROI 调试图 |
| `debug_letter_dir` | `/home/ysc/2026YuYaoGuoSai/assets/letter_debug` | 调试图保存目录 |

### 5.1 常用启动示例

**关闭语音：**

```bash
ros2 run gauge_yolo_detector gauge_yolo_server --ros-args -p voice_enabled:=false
```

**每次调用都播报（不抑制重复）：**

```bash
ros2 run gauge_yolo_detector gauge_yolo_server --ros-args -p voice_on_change_only:=false
```

**调试字母识别：**

```bash
ros2 run gauge_yolo_detector gauge_yolo_server --ros-args -p debug_letter:=true
```

**换用 USB 摄像头 `/dev/video6`：**

```bash
ros2 run gauge_yolo_detector gauge_yolo_server --ros-args -p camera_id:=6
```

## 6. 语音播报规则

- 播报文件名规则：`{字母}{后缀}.mp3`
  - `L`：偏低（黄区）
  - `H`：偏高（红区）
  - `M`：居中（绿区）
- 示例：`A` 字母 + 绿区 → 播放 `AM.mp3`
- 默认只在 `(字母, 区域)` 状态变化时播报，避免重复

## 7. 常见问题

### 7.1 第一次服务调用返回“识别失败，未检测到有效仪表盘”

通常是模型还没完全加载。等终端出现 `服务 /detect_gauge_yolo 已创建` 并且 TensorRT engine 加载完成后再调用。

### 7.2 字母一直为空

先确认 `pytesseract` 可用：启动时应该看到 `pytesseract 已就绪，字母识别可用`。如果显示未安装：

```bash
sudo apt install tesseract-ocr tesseract-ocr-eng python3-pytesseract
```

再用 `debug_letter:=true` 启动，查看 `/home/ysc/2026YuYaoGuoSai/assets/letter_debug/` 里的 ROI 图片。

### 7.3 语音只报一次

这是 `voice_on_change_only:=true` 的默认行为。如果希望每次调用都报，启动时加上 `-p voice_on_change_only:=false`。

### 7.4 同时运行纯 CV 版和 YOLO 版会冲突吗？

不会。两个节点服务名不同：

- 纯 CV：`/detect_gauge`
- YOLO：`/detect_gauge_yolo`

调用方按需选择即可。

## 8. 从其他 ROS2 节点调用

Python 示例：

```python
import rclpy
from rclpy.node import Node
from gauge_detector_interfaces.srv import GaugeDetect

class Caller(Node):
    def __init__(self):
        super().__init__('gauge_caller')
        self.cli = self.create_client(GaugeDetect, 'detect_gauge_yolo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')
        self.req = GaugeDetect.Request()

    def call(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

rclpy.init()
caller = Caller()
resp = caller.call()
print(f"letter={resp.letter}, zone={resp.zone}, state={resp.state}")
```

## 9. 相关文档

- YOLO 实现细节：[[gauge_yolo_usage]]
- 纯 CV 版 ROS 节点：见 `gauge_detector/README.md`
