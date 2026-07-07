# Lite3 机械狗指针式仪表识别部署说明

基于对本项目全部源码的分析，梳理核心运行逻辑并给出在 **Lite3 机械狗** 上的部署策略。

---

## 一、核心运行逻辑

本项目采用 **传统计算机视觉方案** 实现指针式仪表读数识别，不依赖深度学习。整体处理流程如下：

```
摄像头图像
   ↓
圆盘检测（Hough 圆 + RANSAC / 最小二乘精修）
   ↓
ROI 裁剪并归一化到 500×500
   ↓
颜色 / 方向检测（K-Means 或 LAB 阈值 → 找红色区域 → 计算 12 点钟方向）
   ↓
极坐标展开（720 角度 × 100 半径）
   ↓
指针角度提取（列均值最小值 + 抛物线插值）
   ↓
区域分类（红 / 绿 / 黄三区：偏高 / 正常 / 偏低）
```

### 关键文件分工

| 文件 | 作用 |
|------|------|
| [gauge_recognition.py](gauge_recognition.py) | 完整离线 pipeline，使用 K-Means，精度高但速度慢 |
| [realtime_gauge.cpp](realtime_gauge.cpp) | C++ 实时版本，用 LAB 阈值替换 K-Means，性能最优 |
| [realtime_gauge.py](realtime_gauge.py) | Python 实时版本，带 3 帧时序稳定滤波 |
| [step1_hough_circle.py](step1_hough_circle.py) | 生产级圆盘检测：Hough + 径向扫描 + RANSAC |
| [step2_pointer_detect.py](step2_pointer_detect.py) | 指针检测模块：极坐标展开 + 列均值最小值 |
| [evaluate.py](evaluate.py) | 评估指标：圆心误差、半径误差、指针角度误差 |
| `color_*.py` | 颜色分割与方向检测调试工具 |
| `plot_*.py` / `viz_*.py` | 误差分析、损失可视化、极坐标可视化工具 |

### 性能估计

| 版本 | 帧率 | 适用场景 |
|------|------|----------|
| Python 离线版（K-Means） | 5–10 FPS | 离线测试、高精度标定 |
| Python 实时版（LAB 阈值） | 10–15 FPS | 原型验证、需要稳定滤波 |
| **C++ 实时版** | **20–30 FPS** | **Lite3 部署首选** |

---

## 二、Lite3 机械狗部署策略

### 1. 硬件层适配

Lite3 机载算力通常有限（多为 ARM 嵌入式平台或低功耗 x86），部署时应遵循以下原则：

- **优先使用 C++ 版本**：避开 Python GIL 与 numpy 开销，直接编译为二进制运行。
- **摄像头接入**：Lite3 通常搭载 RealSense D435i 或类似 RGB-D 相机。若仅识别仪表，使用 **RGB 流** 即可，推荐分辨率 640×480，足以支撑圆盘检测。
- **避免使用 K-Means**：C++ 版已用 LAB 阈值替代 K-Means，部署时保持该选择。
- **算力预留**：圆盘检测（`HoughCircles`）是最耗时环节，建议在 Lite3 上实测 CPU 占用，必要时减少 Hough 策略数量，或在相机位姿固定后简化检测流程。

### 2. 软件环境构建

```bash
# 1. 安装 OpenCV 4.x（C++）
sudo apt update
sudo apt install libopencv-dev libopencv-contrib-dev

# 2. 编译实时程序
g++ -O2 -std=c++17 realtime_gauge.cpp -o realtime_gauge \
    $(pkg-config --cflags --libs opencv4)

# 3. Python 依赖（仅调试脚本需要）
pip install opencv-python numpy matplotlib
```

> **注意**：当前目录下的 `realtime_gauge` 二进制文件是在 OpenCV 4.6 环境下编译的。若 Lite3 上 OpenCV 版本不同，**必须重新编译**。

### 3. 与 Lite3 控制系统集成

#### 方案 A：ROS / ROS2 节点封装（推荐）

将 C++ 实时程序封装为 ROS2 节点：

- **输入**：订阅相机话题 `/camera/color/image_raw`
- **输出**：
  - `std_msgs/Float32`：指针角度
  - `std_msgs/String`：状态（正常 / 偏高 / 偏低）
  - `geometry_msgs/Point`：圆盘中心坐标
- **可视化**：发布标注后的图像到 `/gauge_detection/annotated_image`

这样 Lite3 的上位机或其他节点可直接订阅识别结果，决策层无需关心图像处理细节。

#### 方案 B：独立进程 + Socket / 共享内存

若 Lite3 不使用 ROS，可将 C++ 程序作为独立进程运行，通过 TCP/UDP 或 ZeroMQ 输出 JSON 结果：

```json
{
  "pointer_angle": 287.5,
  "up_angle": 270.0,
  "status": "正常",
  "center": [320, 240],
  "radius": 120
}
```

### 4. 部署参数调优

Lite3 上部署时，以下硬编码参数需要针对实际场景重新标定：

| 参数 | 当前值 | 部署建议 |
|------|--------|----------|
| `minRadius / maxRadius` | 50–250 | 根据 Lite3 摄像头到仪表的距离重新设定 |
| ROI 大小 `r * 2.2` | 2.2 × 半径 | 若摄像头固定，可适当缩小 ROI 减少计算量 |
| LAB 红 / 黄阈值 | C++ 中硬编码 | 在 Lite3 真实光照下用 [color_kmeans.py](color_kmeans.py) 重新采样标定 |
| `ANGLE_RES = 720` | 0.5° / 列 | 可降为 360（1° / 列）提升性能，精度损失很小 |
| `STABLE_N = 3` | Python 版 | 机械狗移动时建议加大到 5–10，避免抖动 |
| `ANGLE_TOL = 60°` | 稳定滤波 | 移动场景下可放宽，但会引入延迟 |

### 5. 实际部署流程

建议按以下步骤落地：

1. **静态标定**：将 Lite3 固定在仪表正前方，用 [gauge_recognition.py](gauge_recognition.py) 跑一批图像，确认圆盘检测和颜色分类准确。
2. **阈值固化**：用标定结果修正 [realtime_gauge.cpp](realtime_gauge.cpp) 中的 LAB 阈值。
3. **C++ 编译**：在 Lite3 本机或交叉编译环境下生成 `realtime_gauge`。
4. **ROS2 封装**：编写节点，将 C++ 逻辑接入相机话题。
5. **运动测试**：让 Lite3 小幅移动，观察指针角度是否稳定，必要时开启 Python 版的时序稳定滤波或提高 C++ 版的滤波窗口。
6. **异常处理**：加入检测丢失保护，连续 N 帧未检测到圆盘时发布 `status: unknown`。

### 6. 关键风险与对策

| 风险 | 原因 | 对策 |
|------|------|------|
| 检测丢失 | 机械狗移动导致画面抖动 | 增加时序稳定滤波；降低识别置信度阈值 |
| 光照变化 | 不同环境光下颜色阈值失效 | 提前多场景标定 LAB 阈值；或改用自适应阈值 |
| 圆盘检测慢 | HoughCircles 多策略计算量大 | 固定相机位姿后减少策略数；或降低输入分辨率 |
| 二进制无法运行 | OpenCV 版本不匹配 | 在 Lite3 上重新编译 |
| 多仪表干扰 | 代码假设单圆盘 | 确保每次只对一个仪表；或增加 ROI 预选 |

---

## 三、推荐部署架构

```
Lite3 机械狗
├── RGB-D 相机
│   └── 发布 /camera/color/image_raw
├── ROS2 节点：gauge_detector（C++ 编译）
│   ├── 订阅图像流
│   ├── 运行 realtime_gauge.cpp 核心逻辑
│   └── 发布 /gauge/pointer_angle、/gauge/status
├── 决策 / 运动控制节点
│   └── 订阅仪表读数，执行巡检任务
└── 上位机 / 地面站（可选）
    └── 订阅标注图像用于监控
```

**首选命令行启动方式**：

```bash
ros2 run lite3_gauge gauge_detector --ros-args \
  -p input_topic:=/camera/color/image_raw \
  -p output_topic:=/gauge/status \
  -p show_viz:=false
```

---

## 四、后续可扩展工作

- 将 [realtime_gauge.cpp](realtime_gauge.cpp) 改造为 ROS2 节点；
- 编写 Lite3 上的交叉编译脚本；
- 针对具体仪表设计 LAB 阈值自动标定脚本。
