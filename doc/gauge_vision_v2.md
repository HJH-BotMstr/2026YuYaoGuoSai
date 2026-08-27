---
title: 仪表盘识别 v2：新模型替换与视觉模块使用说明
date: 2026-08-16
tags: [yolo, gauge, vision, tensorrt, ros2, jetson, 主程序接口]
---

# 仪表盘识别 v2：新模型替换与视觉模块使用说明

本文档记录 2026-08-15 ~ 08-16 的模型替换工作和新增的视觉接口模块，包括：做了什么修改、为什么这么做、以及如何使用。

## 1. 改动总览

| 项目 | 旧版 | 新版 |
|---|---|---|
| regions 模型（表盘+红区检测） | `best_bg.pt/.engine`（3 类：gauge/red/yellow） | `gauge_regions_3d.pt/.engine`（2 类：gauge/red） |
| pointer 模型（指针关键点） | `best_ptr.pt/.engine` | `gauge_pointer_3d_v3.pt/.engine` |
| 摄像头 | `/dev/video4` | `/dev/video0` |
| 单文档验证程序 | `tools/gauge_yolo_new.py` | `tools/gauge_yolo_new_v2.py` |
| 区域判定阈值 | 写死 45°/135°（单侧偏高） | 参数化，默认 41°/145°，偏高双侧对称 |

新增文件：

- `tools/engine_add_metadata.py` — 给 trtexec 生成的 engine 补 ultralytics 元数据头
- `tools/gauge_vision.py` — 主程序视觉模块（纯 Python 版，含语音播报）
- `tools/gauge_vision_ros.py` — 主程序视觉模块（ROS 服务调用版）

修改文件：

- `tools/gauge_yolo_new_v2.py` — 单文档验证程序（大量修复，见 [[#3. gauge_yolo_new_v2.py 改动细节]]）
- `lite3_ws/src/gauge_yolo_detector/gauge_yolo_detector/gauge_yolo_server.py` — ROS 节点
- `doc/gauge_yolo_detector_usage.md` — ROS 节点使用文档

## 2. 新模型的 engine 生成流程（重要）

> 机器狗环境：Jetson Xavier NX，JetPack 5.1.2，ultralytics **8.1.0**（不方便升级）。

### 2.1 背景：trtexec 的 engine 为什么不能直接用

ultralytics 8.1.0 加载 `.engine` 时期望的文件格式是：

```text
[4 字节小端长度][JSON 元数据（task/names/stride/kpt_shape 等）][TensorRT 序列化数据]
```

`trtexec` 转出的 engine 是纯 TensorRT 序列化数据，**没有元数据头**，直接加载会报：

```text
'utf-8' codec can't decode byte 0xe8 in position 4: invalid continuation byte
```

（加载器把文件开头的二进制数据当成 JSON 元数据长度和内容来读了。）

### 2.2 两个模型各自的生成方式

**regions（detect 模型）** —— 直接在狗上用 ultralytics 导出（自带元数据头）：

```bash
source ~/yolov8_env/bin/activate
cd /home/ysc/2026YuYaoGuoSai/assets/models
python /home/ysc/2026YuYaoGuoSai/tools/export_engine.py gauge_regions_3d.pt
```

**pointer（pose 模型）** —— ultralytics 8.1.0 在狗上无法导出 pose 模型（已知 bug，报 `Module [ModuleList] is missing the required "forward" function`），流程是：

1. 在训练电脑（Windows）上把 `.pt` 导出为 `.onnx`（ONNX 跨平台，无影响）
2. 把 `.onnx` 拷到狗上，用 trtexec 转 engine（engine 不跨设备，必须在狗上本机转）：

```bash
cd /home/ysc/2026YuYaoGuoSai/assets/models
/usr/src/tensorrt/bin/trtexec \
  --onnx=gauge_pointer_3d_v3.onnx \
  --saveEngine=gauge_pointer_3d_v3.engine \
  --fp16 --workspace=4096
```

3. 补 ultralytics 元数据头：

```bash
python /home/ysc/2026YuYaoGuoSai/tools/engine_add_metadata.py \
    gauge_pointer_3d_v3.engine --task pose --names pointer --kpt-shape 2 3
```

### 2.3 engine_add_metadata.py 工具说明

`tools/engine_add_metadata.py`：给 trtexec 生成的 engine 文件头部插入 ultralytics 8.1.0 格式的元数据（格式从旧 `best_*.engine` 中逐字段复刻）。纯标准库，不用激活环境。

```bash
# detect 模型示例（regions 如果也想走 trtexec 的话）
python engine_add_metadata.py gauge_regions_3d.engine --task detect --names gauge red

# pose 模型必须带 --kpt-shape（本项目 pointer 是 2 个关键点 × 3 维）
python engine_add_metadata.py gauge_pointer_3d_v3.engine --task pose --names pointer --kpt-shape 2 3
```

特性：覆盖前自动备份为 `.bak`；写入后回读验证；已有元数据的文件自动跳过（`--force` 可强制重写）。

模型结构核对结果（从 ONNX 输出维度确认）：

| 模型 | 输出维度 | 含义 |
|---|---|---|
| `gauge_regions_3d` | `[1, 6, 8400]` | 4 + 2 类（gauge, red） |
| `gauge_pointer_3d_v3` | `[1, 11, 8400]` | 4 + 1 类 + 2 关键点×3 维，与旧 pointer 一致 |

## 3. gauge_yolo_new_v2.py 改动细节

相对 `gauge_yolo_new.py` 的全部差异：

### 3.1 模型与摄像头

- 模型换成 `gauge_regions_3d` / `gauge_pointer_3d_v3`，**优先 `.engine`，不存在时降级 `.pt`**（不再自动导出 engine）
- 摄像头默认 `/dev/video4` → `/dev/video0`

### 3.2 指针实例筛选（修：绿点跑到画面左上角）

原代码取 `kpts.xy[0]`（第一个检出实例）。pose 模型会把画面里其他细长物（笔、纸角等）也当成"指针"，取错实例后铆钉定在画面角落，角度全错。

现在遍历所有实例，**取铆钉离表盘圆心最近的一个**；所有实例铆钉都离圆心超过 1.5 倍表盘半径时，判定本帧未检出（宁可不报也不报错）。

### 3.3 相对角度平滑（修：居中和偏低来回跳）

指针读数在区域边界附近时，关键点每帧几个像素的抖动会导致区域来回跳。现在对相对角做**最近 5 帧圆均值平滑**（用 sin/cos 平均，0°/360° 跨界不会算错）。代价是约 1.5 秒延迟（5 帧 × 0.3 秒间隔）。

### 3.4 区域阈值重新校准（修：绿区判错）

旧阈值 45°/135° 是按旧模型的红框位置调的，新模型红框位置不同导致基准角 `up_angle` 平移，绿区被挤压。

**校准计算**（基于第一版数据集标注：0MPa→218.5°，0.5MPa→86.7°，1MPa→314.8°）：

- 刻度线性，量程 263.7°/MPa，θ(v) = 218.5 − 263.7·v
- 红区（0.7~1.0MPa）弧中心 ≈ 355°，即 up_angle 基准
- 红绿边界 0.7MPa → rel ≈ **+39°**；黄绿边界 0.3MPa → rel ≈ **+144°**

另外发现原逻辑的隐藏 bug：指针转过红区中心后 rel 变负（满量程 1.0MPa 时 rel ≈ −40°），单侧 `[0, 45°]` 的偏高判定会把满量程误判为偏低。已改为**双侧对称** `|rel| ≤ thr_high`。

最终默认值（全量程 0~1.0MPa 扫描验证通过）：

```python
thr_high = 41   # |rel| <= 41° → 偏高（覆盖整个红区含满量程）
thr_low = 145   # 41° < rel <= 145° → 居中；其余 → 偏低
```

边界值 0.3/0.7 恰好压线时判到异常一侧（比赛场景从严，更安全）。

命令行可调，输出带 rel 角度便于复校：

```bash
python gauge_yolo_new_v2.py --thr-high 41 --thr-low 145
# 输出示例：A,绿,正常,rel=92°
```

参考校验点：指针在 0.5MPa 时 rel 应 ≈ 92°。

### 3.5 箭头镜像修复（修：红蓝箭头与实际不符）

`angle()` 用数学坐标系（y 向上）算角度，画箭头时直接 `cy + len*sin(θ)` 用了图像坐标系（y 向下），导致画出的箭头是真实方向的**上下镜像**。已改为 `cy - len*sin(θ)`。修复后：蓝箭头正好指向红框中心，红箭头与指针连线同方向。

### 3.6 字母 OCR 区域放宽（修：字母识别不出/A、D 串）

"表盘上方"ROI 从 0.8 倍框高、左右各 0.1 倍框宽，放宽到 **1.5 倍框高、左右各 0.3 倍框宽**。原来字母稍高就被切成半个导致误识别。

注意：如果字母被**相机画面本身的顶边**切掉，放宽 ROI 没用，要让狗离远一点或头低一点。

调试命令（终端逐帧打印 OCR 原文，ROI 图存 `assets/letter_debug/`）：

```bash
python gauge_yolo_new_v2.py --debug-letter --no-voice
```

### 3.7 字母投票的尝试与还原（记录）

曾尝试过两种字母防抖方案，**均已还原**（实测效果更差，记录备查）：

1. 连续 2 次 OCR 一致才采纳 → A/D 抖动时条件永远不满足，字母卡死不出
2. 最近 5 次窗口多数投票 + 换表盘清空缓存 → 实测仍不理想，按需求还原

当前字母逻辑与最初版本一致：每 5 帧 OCR 一次，识别到就直接更新。

## 4. 主程序视觉模块（两版，接口一致）

主程序对接视觉只需要 5 个函数，两版接口完全一致，改一行 import 即可切换：

```python
import sys
sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
import gauge_vision as gv          # 纯 Python 版
# import gauge_vision_ros as gv    # 或 ROS 版

gv.init()            # 初始化（只调一次）
r = gv.recognize()   # 识别一次，返回 dict
gv.get_all()         # {'A': 'normal', 'D': 'abnormal'}
gv.get_state('D')    # 'normal' / 'abnormal' / None（未识别过）
gv.shutdown()        # 程序结束时释放资源
```

`recognize()` 返回值：

```python
{'letter': 'A',      # 或 None（这次没识别出字母）
 'zone': 'GREEN',    # 'RED'/'GREEN'/'YELLOW'，或 None（没检测到表盘）
 'state': 'normal'}  # 'normal'（绿区）/'abnormal'（红区或黄区），或 None
```

共同行为：

- **存储自动**：识别到字母就自动写入结果表，同字母再识别会覆盖旧值
- **内部重试**：`recognize(retries=3, retry_interval=0.4)`，没检测到表盘或没读出字母会自动取新帧重试

### 4.1 纯 Python 版 `tools/gauge_vision.py`

- `init()` 自己加载模型 + 打开 `/dev/video0` + 预热 80 帧（engine 加载约 30~60 秒，主程序要预留时间）
- 后台取图线程持续清缓冲，`recognize()` 拿到的永远是最新画面
- **语音播报内置**，逻辑与单文档程序一致：字母+区域变化时播对应 MP3（如 `AM.mp3`），状态不变不重复播；`init(voice_enabled=False)` 关闭
- 识别核心与 `gauge_yolo_new_v2.py` 同一份代码（指针筛选、角度平滑、41/145 阈值全带）

可选参数（一般不用动）：`camera_id / width / height / preheat_frames / models_dir / use_engine / device / imgsz / thr_high / thr_low / voice_enabled / mp3_dir / debug_letter / debug_letter_dir`

### 4.2 ROS 版 `tools/gauge_vision_ros.py`

- 识别通过调用 `/detect_gauge_yolo` 服务完成，模型/摄像头/语音都在服务端节点
- **前提**：先启动服务端（模型只加载一次，之后主程序重启很快）：

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run gauge_yolo_detector gauge_yolo_server   # 终端1 常驻
```

- 主程序所在终端也要 source ROS 环境（否则 import 不到 rclpy）：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash
```

- `init()` 会等服务上线（`wait_timeout=秒数` 可设上限）；`recognize(timeout=5.0)` 单次调用超时 5 秒
- 可选参数：`service_name / node_name / wait_timeout`

### 4.3 两版自测试

两个文件都可以直接运行自测（初始化后每 2 秒识别一次并打印结果表，Ctrl+C 退出）：

```bash
python /home/ysc/2026YuYaoGuoSai/tools/gauge_vision.py       # 纯 Python 版
python /home/ysc/2026YuYaoGuoSai/tools/gauge_vision_ros.py   # ROS 版（先起服务端）
```

### 4.4 选型参考

| 对比项 | 纯 Python 版 | ROS 版 |
|---|---|---|
| 依赖 | 只需 yolov8_env 环境 | 需要 ROS 环境 + 服务端节点常驻 |
| 主程序启动速度 | 慢（每次都要加载 engine） | 快（模型由服务端加载一次） |
| 耦合度 | 零耦合，单进程 | 多一个常驻进程，但模块间解耦 |
| 语音 | 模块内置 | 服务端负责 |

## 5. ROS 节点改动（gauge_yolo_server.py）

详见 [[gauge_yolo_detector_usage]]。本次改动：

- 复用代码从 `gauge_yolo_new` 改为 `gauge_yolo_new_v2`（自动获得全部识别修复）
- `camera_id` 默认值 4 → 0
- 新增参数 `thr_high`（默认 41.0）、`thr_low`（默认 145.0），启动时可 `-p thr_high:=41.0` 覆盖

改了节点代码后重新编译（`--symlink-install` 模式下改 .py 其实直接生效，重跑一次保险）：

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select gauge_yolo_detector --symlink-install
source install/setup.bash
ros2 run gauge_yolo_detector gauge_yolo_server
```

服务测试：

```bash
ros2 service call /detect_gauge_yolo gauge_detector_interfaces/srv/GaugeDetect "{}"
```

## 6. 可视化元素对照表

单文档程序画面上的标记含义（调试时对照）：

| 元素 | 含义 |
|---|---|
| 蓝色方框 | regions 模型检出的表盘 |
| 红色方框 | regions 模型检出的红色区域 |
| 绿点 | pointer 模型检出的铆钉（指针轴心） |
| 蓝点 | pointer 模型检出的针尖 |
| 红色短线（绿点→蓝点） | 模型认为的指针本体 |
| 蓝色箭头（从表盘中心） | 基准方向 up_angle：指向红框中心 |
| 红色箭头（从表盘中心） | 指针方向 ptr_angle |

区域判定 = 红箭头相对蓝箭头的夹角 rel：|rel| ≤ 41° 偏高，41° < rel ≤ 145° 居中，其余偏低。

## 7. 已知遗留事项

- 字母 OCR 仍有抖动可能（靠 ROI 放宽改善，未根治）；`recognize()` 返回 `letter=None` 时主程序可再调一次
- 如果红箭头与指针连线方向恒差 180°，说明 pointer 模型针尖/铆钉标反，需要改训练数据
- 纯 CV 版节点 `gauge_detector` 未动，与本次替换无关

## 8. 相关文档

- ROS 节点详细使用：[[gauge_yolo_detector_usage]]
- 单文档程序说明：[[gauge_yolo_usage]]
- 项目任务安排：[[TODO]]
