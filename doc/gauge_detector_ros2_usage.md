# 仪表盘识别 ROS2 服务节点使用说明

## 1. 功能概述

本 ROS2 包将纯 Python 仪表盘识别程序封装为服务节点，供机器狗其他节点调用。

- **服务名**：`/detect_gauge`
- **服务类型**：`gauge_detector_interfaces/srv/GaugeDetect`
- **启动时行为**：初始化摄像头并完成一次性预热（约 120 帧）。
- **调用时行为**：直接取最新帧进行识别，不重复预热。
- **语音播报**：识别成功后，根据识别到的字母和区域自动播放对应 MP3 音频。

## 2. 返回字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 识别是否成功 |
| `letter` | `string` | 识别到的仪表盘字母：A / B / C / D |
| `zone` | `string` | 指针所在区域：RED / GREEN / YELLOW |
| `state` | `string` | 整体状态：GREEN 区为 `normal`，其他为 `abnormal` |
| `message` | `string` | 提示信息 |

## 3. 工作空间位置

```
/home/ysc/2026YuYaoGuoSai/lite3_ws/src/
├── gauge_detector_interfaces/   # ament_cmake 接口包
│   └── srv/GaugeDetect.srv
└── gauge_detector/              # ament_python 节点包
    └── gauge_detector/gauge_server.py
```

## 4. 编译

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select gauge_detector_interfaces gauge_detector --symlink-install
```

## 5. 启动服务节点

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash
ros2 run gauge_detector gauge_server
```

等待终端输出：

```
摄像头预热完成
服务 /detect_gauge 已创建
```

### 可配置参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `camera_id` | 4 | 摄像头设备号 |
| `width` | 640 | 图像宽度 |
| `height` | 480 | 图像高度 |
| `preheat_frames` | 120 | 预热丢弃帧数 |
| `voice_enabled` | true | 是否启用语音播报 |
| `mp3_dir` | `/home/ysc/2026YuYaoGuoSai/assets/mp3` | 音频文件目录 |

示例：关闭语音

```bash
ros2 run gauge_detector gauge_server --ros-args -p voice_enabled:=false
```

## 6. 调用服务

### 6.1 使用提供的 Python 客户端（推荐）

因为当前环境下 `ros2 service call` 有 `ros2cli==0.9.13` 问题（见第 8 节），建议使用仓库内的测试客户端：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash
python3 /home/ysc/2026YuYaoGuoSai/tools/test_gauge_client.py
```

输出示例：

```
----- 服务返回 -----
success: True
letter : B
zone   : GREEN
state  : normal
message: 识别成功
--------------------
```

### 6.2 使用 ros2 service call（当前环境不可用）

正常环境下可以使用：

```bash
ros2 service call /detect_gauge gauge_detector_interfaces/srv/GaugeDetect "{}"
```

当前环境会报错：

```
DistributionNotFound: The 'ros2cli==0.9.13' distribution was not found
```

## 7. 语音播报

### 7.1 音频文件规则

根据识别到的 **字母** 和 **区域**，自动播放对应音频文件：

| 字母 | 偏低（YELLOW） | 偏高（RED） | 居中（GREEN） |
|------|---------------|------------|--------------|
| A | `AL.mp3` | `AH.mp3` | `AM.mp3` |
| B | `BL.mp3` | `BH.mp3` | `BM.mp3` |
| C | `CL.mp3` | `CH.mp3` | `CM.mp3` |
| D | `DL.mp3` | `DH.mp3` | `DM.mp3` |

文件实际格式支持 MP3、MOV、M4A 等常见音频格式（底层通过 `ffmpeg` 解码）。

### 7.2 音频文件位置

默认目录：

```
/home/ysc/2026YuYaoGuoSai/assets/mp3/
```

### 7.3 依赖

- `ffmpeg`：用于音频解码
- `aplay`（alsa-utils）：用于播放，走 pulseaudio 默认设备

安装命令：

```bash
sudo apt install ffmpeg alsa-utils
```

### 7.4 行为

- 相同状态不会重复播报。
- 识别到 D 但没有 `DL/DH/DM.mp3` 时，终端会提示文件不存在，不播放声音。
- 缺少 `ffmpeg` 或 `aplay` 时，语音播报自动禁用，不影响识别服务。

## 8. 关键设计

- **预热与识别分离**：节点启动时一次性完成摄像头预热，服务调用时不重复预热，避免每次调用等待和识别错误。
- **后台持续取图**：服务节点内部有一个 capture 线程，持续刷新 `latest_frame`，保证每次服务调用拿到的都是较新的图像。
- **快速圆检测**：使用 0.5 倍缩放 + 颜色验证的 `detect_circle_fast`，将单帧识别耗时从 10~25 秒降到约 0.2~0.5 秒。
- **指针平滑**：使用环形卷积对指针角度做移动平均，避免 0°/360° 跳变，且指针方向会随仪表盘旋转正确变化。
- **字母跳帧识别**：每 5 帧跑一次 Tesseract OCR，避免连续服务调用时重复耗时。

## 9. 需要向其他同学/负责人提交的问题

### 9.1 ros2cli 环境异常

**现象**：

```bash
ros2 service call /detect_gauge gauge_detector_interfaces/srv/GaugeDetect "{}"
```

报错：

```
DistributionNotFound: The 'ros2cli==0.9.13' distribution was not found
```

**影响**：无法使用命令行工具调用服务、查看 topic、查看节点列表等。

**建议**：请负责 ROS2 环境/系统配置的同学检查 `/opt/ros/foxy/lib/python3.8/site-packages/ros2cli` 及其 `dist-info` 是否完整，或重新安装 `ros-foxy-ros2cli` 包。这个问题不是本节点代码导致的，本节点用 Python 客户端可正常调用。

### 9.2 摄像头编号参数化

当前代码默认使用 `/dev/video4`：

```python
self.declare_parameter('camera_id', 4)
```

如果机器狗上摄像头编号不同，请在启动节点时通过参数修改：

```bash
ros2 run gauge_detector gauge_server --ros-args -p camera_id:=0
```

如果最终部署时摄像头编号不确定，建议负责硬件集成的同学确认并固定摄像头设备号（例如通过 udev 规则绑定）。

### 9.3 Tesseract OCR 依赖

字母识别依赖 `pytesseract` 和系统 `tesseract-ocr`。

- Python 包：`pytesseract`
- 系统包：`tesseract-ocr`（以及英文语言包）

如果部署到新机器上字母识别失败，请先确认：

```bash
tesseract --version
python3 -c "import pytesseract; print(pytesseract.get_tesseract_version())"
```

## 10. 纯 Python 调试版本

 standalone 调试程序有两个版本：

### 10.1 基础可视化版本

```
/home/ysc/2026YuYaoGuoSai/tools/realtime_gauge_async.py
```

运行方式：

```bash
python3 /home/ysc/2026YuYaoGuoSai/tools/realtime_gauge_async.py
```

按 `q` 退出。带可视化窗口，可用于调试字母 ROI、二值化结果、指针方向等。

### 10.2 MP3 语音播报版本

```
/home/ysc/2026YuYaoGuoSai/tools/realtime_gauge_async_new.py
```

运行方式：

```bash
python3 /home/ysc/2026YuYaoGuoSai/tools/realtime_gauge_async_new.py
```

识别到状态变化时自动播放 `/home/ysc/2026YuYaoGuoSai/assets/mp3/` 下对应音频文件。

## 11. 后续可扩展项

- 在 `GaugeDetect.srv` 的请求部分增加 `force_letter` 或 `camera_id` 字段，实现单节点多摄像头支持。
- 增加图像发布话题，把识别画面通过 `sensor_msgs/Image` 发出来，方便远程调试图像。
- 将节点配置改为参数文件或 launch 文件启动，便于比赛现场快速调整参数。
