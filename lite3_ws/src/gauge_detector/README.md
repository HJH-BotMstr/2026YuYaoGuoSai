# gauge_detector

ROS2 仪表盘识别服务节点。

## 功能

- 启动时初始化摄像头并完成一次性预热（默认 120 帧）。
- 创建服务 `/detect_gauge`，供其他节点调用。
- 调用时直接取最新帧进行识别，不重复预热。
- 返回仪表盘字母、指针区域、整体状态。

## 订阅与发布

- **订阅**：无（服务节点，不订阅 ROS topic）。
- **发布**：无（仅提供 ROS 服务）。
- **服务**：`/detect_gauge`（类型：`gauge_detector_interfaces/srv/GaugeDetect`）。

## 依赖

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `gauge_detector_interfaces`
- OpenCV (`cv2`)
- NumPy
- `pytesseract` + 系统 `tesseract-ocr`

## 编译

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select gauge_detector_interfaces gauge_detector --symlink-install
```

## 启动

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash
ros2 run gauge_detector gauge_server
```

等待终端输出：

```text
摄像头预热完成
服务 /detect_gauge 已创建
```

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `camera_id` | `int` | `6` | 摄像头设备号，对应 `/dev/video{camera_id}` |
| `width` | `int` | `640` | 图像宽度 |
| `height` | `int` | `480` | 图像高度 |
| `preheat_frames` | `int` | `120` | 预热丢弃帧数 |

修改摄像头编号：

```bash
ros2 run gauge_detector gauge_server --ros-args -p camera_id:=0
```

## 调用服务

### 推荐：仓库内测试客户端

当前环境 `ros2cli==0.9.13` 异常，建议使用仓库客户端：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash
python3 /home/ysc/2026YuYaoGuoSai/tools/test_gauge_client.py
```

输出示例：

```text
----- 服务返回 -----
success: True
letter : B
zone   : GREEN
state  : normal
message: 识别成功
--------------------
```

### 命令行方式（当前环境不可用）

正常环境下：

```bash
ros2 service call /detect_gauge gauge_detector_interfaces/srv/GaugeDetect "{}"
```

当前环境会报错 `DistributionNotFound: The 'ros2cli==0.9.13' distribution was not found`，需由负责 ROS2 环境的同学修复 `ros2cli`。

## 关键设计

- **预热与识别分离**：节点启动时一次性完成摄像头预热，服务调用时不重复预热。
- **后台持续取图**：内部 capture 线程持续刷新 `latest_frame`，保证每次调用拿到较新图像。
- **快速圆检测**：0.5 倍缩放 + 颜色验证，单帧识别耗时约 0.2~0.5 秒。
- **指针平滑**：环形卷积移动平均，避免 0°/360° 跳变。
- **字母跳帧识别**：每 5 帧跑一次 Tesseract OCR，避免连续调用重复耗时。

## 已知问题

1. **ros2cli 环境异常**：无法使用 `ros2 service call` 等命令行工具，需检查 `/opt/ros/foxy/lib/python3.8/site-packages/ros2cli`。
2. **摄像头编号依赖硬件**：默认 `/dev/video6`，部署前请确认设备号或绑定 udev 规则。
3. **Tesseract 依赖**：需安装系统包 `tesseract-ocr` 和 Python 包 `pytesseract`。

## 调试工具

纯 Python 调试程序（带可视化窗口）：

```bash
python3 /home/ysc/2026YuYaoGuoSai/tools/realtime_gauge_async.py
```

按 `q` 退出，可用于调试字母 ROI、二值化结果、指针方向等。

## 后续可扩展项

- 在 `GaugeDetect.srv` 请求中增加 `force_letter` 或 `camera_id` 字段，支持单节点多摄像头。
- 增加图像发布话题，把识别画面通过 `sensor_msgs/Image` 发出来，方便远程调试。
- 改用参数文件或 launch 文件启动，便于比赛现场快速调整参数。
