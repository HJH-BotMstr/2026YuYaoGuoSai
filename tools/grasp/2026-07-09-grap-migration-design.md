# grasp 抓取模块迁移设计文档

**日期**：2026-07-09  
**作者**：胡峻豪  
**目标**：适配 Lite3 感知主机，机器狗运动由外部 ROS2 节点负责，本模块只管机械臂。

---

## 一、背景与范围

### 范围
- **包含**：机械臂视觉识别红色长条、闭环抓取、运输、放置到目标区域
- **不包含**：机器狗导航/对齐（由外部 ROS2 节点负责）、仪表盘巡检识别（已有代码，本模块用占位接口）

### 比赛抓取任务逻辑（附件3后50分）
1. 机器狗已由外部节点移动到抓取区并停稳
2. 机械臂摄像头识别高台上的红色长条（100×40×30 mm）
3. 抓取红色长条，最多允许失败重试 3 次
4. 运输到目标放置区（字母由巡检模块注入，占位默认 A 区）
5. 放下长条

---

## 二、文件结构

```
lite3_ws/src/grasp/
│
├── config.yaml                      # 所有可调参数，TODO 标注需现场标定项
├── main.py                          # 抓取任务主入口，按 phase 顺序执行
│
├── utils/
│   ├── __init__.py
│   ├── ArmController.py             # 基于原版增强：夹爪开合、到位校验、抓取判断
│   ├── BlockDetection.py            # 新建：红/绿长条 HSV 检测 + 中心偏移 + 距离估算
│   ├── InspectionMemory.py          # 占位接口：set_zone() / get_zone() 供 ROS2 注入
│   └── RobotArm/                    # 完整复制原 SDK，不改动
│       ├── scservo_sdk/
│       └── three_Inverse_kinematics.py
│
└── tests/
    ├── test_block_detection.py      # 离线/单摄像头测试色块识别
    └── test_arm_grasp.py            # 机械臂单步抓取测试（沿用 pc_test 风格）
```

### 原文件处理策略
- `assets/old_code/DeepRobotDog/utils/RobotArm/` → **原样复制**到 `grasp/utils/RobotArm/`，不改动
- `assets/old_code/DeepRobotDog/utils/ArmController.py` → **复制后增强**，原接口全部保留
- `assets/old_code/DeepRobotDog/utils/ColorDetection.py` → **不复制**，由 `BlockDetection.py` 替代

---

## 三、config.yaml 参数设计

```yaml
hardware:
  arm_serial_port: "/dev/ttyUSB0"   # [TODO: 现场标定] Lite3 感知主机串口
  arm_serial_baud: 500000
  arm_cam_device: "/dev/video2"     # [TODO: 现场标定] 机械臂摄像头设备号

arm:
  moving_speed: 1500
  moving_acc: 50
  gripper_open_val: 2047
  gripper_close_val: 2400           # 最大 2450
  gripper_load_threshold: 200       # [TODO: 现场标定] 夹住物体负载差值阈值
  grasp_retry_max: 3
  wait_position_timeout: 5.0
  wait_position_threshold: 30       # 舵机值容差 ≈ ±3°

detection:
  arm_cam_fx: 388.1454              # [TODO: 现场标定] 用 camera_params.py 重新标定
  arm_cam_fy: 387.7497
  arm_cam_cx: 329.4121
  arm_cam_cy: 223.481
  arm_cam_dist: [-0.1571, -0.218, -0.0024, -0.0011, 0.2089]
  hsv_red_lower1: [0,   120, 100]  # [TODO: 现场标定] 用 hsv_picker.py 提取
  hsv_red_upper1: [10,  255, 255]
  hsv_red_lower2: [160, 120, 100]
  hsv_red_upper2: [180, 255, 255]
  hsv_green_lower: [40, 80, 80]    # [TODO: 现场标定]
  hsv_green_upper: [80, 255, 255]
  block_min_area: 800
  block_real_width_mm: 40.0         # 题目给定，不变

grasp:
  D_hand_mm: 150.0                  # [TODO: 现场标定] 视觉闭环目标距离
  D_hand_thr_mm: 15.0
  grasp_height_mm: 30.0             # [TODO: 现场标定] 抓取时末端高度
  center_offset_threshold: 15       # 像素，超过则微调 6 号舵机横向对齐

placement:
  zones:                            # [TODO: 现场标定] 各区放置姿态
    A: {dis: 220, height: 30}
    B: {dis: 220, height: 30}
    C: {dis: 220, height: 30}
    D: {dis: 220, height: 30}

inspection:
  default_zone: "A"                 # 占位默认区，真实值由 set_zone() 注入
```

---

## 四、模块接口设计

### 4.1 `utils/ArmController.py`

原有接口全部保留，新增：

| 方法 | 说明 |
|------|------|
| `open_gripper()` | 张开夹爪到 `gripper_open_val` |
| `close_gripper()` | 闭合夹爪到 `gripper_close_val` |
| `read_positions(ids)` | 读取指定舵机当前位置，返回 `{id: pos}` |
| `wait_for_position(ids, targets, timeout)` | 阻塞等待舵机到位，返回 `bool` |
| `grasp_with_verify(dis, height)` | 完整抓取+校验流程，返回 `bool` |

**`grasp_with_verify` 内部流程**：
1. `open_gripper()`
2. `grap(dis, height)` 下发逆运动学目标
3. `wait_for_position([3,4,5], targets, timeout)` 等待关节到位
4. `close_gripper()`，等待 0.5s
5. 读取 1 号舵机 Present_Load，与空载基准比较
6. 若负载差 > `gripper_load_threshold` → 抓取成功，返回 True
7. 否则重试，超过 `grasp_retry_max` 次返回 False

### 4.2 `utils/BlockDetection.py`

```python
class BlockDetection:
    def __init__(self, cfg: dict)

    def detect(self, frame) -> dict | None
    # 返回 {"color", "bbox", "center_offset_x", "distance_mm"}
    # 未检测到返回 None

    def visualize(self, frame, result) -> frame
```

**红色检测**：两段 HSV（跨 0°）分别生成掩码后做 `cv2.bitwise_or`，取最大连通域。  
**距离估算**：`distance_mm = fx * block_real_width_mm / bbox_width_pixels`（针孔模型）

### 4.3 `utils/InspectionMemory.py`

```python
class InspectionMemory:
    def __init__(self, default_zone: str = "A")

    def set_zone(self, zone: str)    # ROS2 回调线程调用（内部加锁）
    def get_zone(self) -> str        # main.py 查询
    def is_ready(self) -> bool       # 占位时恒返回 True
```

**ROS2 集成提示**：后期在 ROS2 节点的话题回调里调 `memory.set_zone(msg.data)` 即可，接口不需要改动。

### 4.4 `main.py` 阶段流程

| 阶段 | 名称 | 核心操作 | 失败处理 |
|------|------|----------|----------|
| phase_0 | 初始化 | 读 config、初始化各模块、打开摄像头 | 立即退出 |
| phase_1 | 待命 | `set_pose(1)` 初始姿态，等机器狗就位 | — |
| phase_2 | 识别 | 循环读帧，找到红色长条且距离稳定（滑动均值窗口） | 超时退出 |
| phase_3 | 抓取 | `set_pose(2)` + `grasp_with_verify`，失败重试最多3次 | 3次失败退出 |
| phase_4 | 运输 | `set_pose(3)` 运输姿态（药瓶水平） | — |
| phase_5 | 放置 | 查 `InspectionMemory`，`grap(zone.dis, zone.height)`，`open_gripper` | 记录日志 |
| phase_6 | 归位 | `set_pose(1)` 归位，关闭摄像头和串口 | — |

**错误处理原则**：
- 每个 phase 用 `try/except` 包裹，异常时打印日志并执行安全归位
- `KeyboardInterrupt` 任意阶段可中断，自动 `set_pose(1)` + `finalize()`
- 所有 `phase_X` 函数签名统一为 `def phase_X(ctx: dict) -> bool`，`ctx` 传递共享状态

---

## 五、数据流

```
摄像头帧
  └─▶ BlockDetection.detect()
        ├─▶ center_offset_x ─▶ 调整 6 号舵机横向对齐（phase_2）
        └─▶ distance_mm ──────▶ 判断是否进入抓取（phase_3）

InspectionMemory.get_zone()
  └─▶ config.placement.zones[zone] ─▶ (dis, height) ─▶ grap() 放置（phase_5）
```

---

## 六、测试脚本设计

### `tests/test_block_detection.py`
- 不需要机械臂，只需摄像头
- 打开视频流，实时显示检测结果（色块框、颜色标签、距离、中心偏移）
- 用于现场 HSV 调参验证

### `tests/test_arm_grasp.py`
- 沿用 `pc_test_arm_grasp.py` 风格
- 支持命令行传入 `dis` 和 `height` 参数
- 单步测试：open_gripper → grap → close_gripper → wait → set_pose(3) → open_gripper → set_pose(1)

---

## 七、现场标定顺序建议

1. 串口确认：`ls /dev/ttyUSB*`，更新 `hardware.arm_serial_port`
2. 摄像头确认：`ls /dev/video*`，更新 `hardware.arm_cam_device`
3. 运行 `test_arm_grasp.py` 验证机械臂基础动作
4. 运行 `camera_params.py` 标定机械臂摄像头，更新 `detection.arm_cam_*`
5. 运行 `hsv_picker.py` 提取红/绿 HSV，更新 `detection.hsv_*`
6. 运行 `test_block_detection.py` 验证识别效果
7. 实测 `D_hand_mm` 和 `grasp_height_mm`，更新 `grasp.*`
8. 实测各放置区坐标，更新 `placement.zones.*`

---

## 八、与 ROS2 集成备注

本模块设计为**纯 Python 脚本**，不依赖 ROS2。后期集成时两种方案均可：

- **方案 A（推荐）**：将 `main.py` 改写为 ROS2 节点，`InspectionMemory.set_zone` 接巡检结果话题回调，机器狗就位信号接 `/grasp/start` 服务
- **方案 B**：保持独立脚本，通过文件/管道传递巡检结果

`InspectionMemory` 接口已为方案 A 预留，改动最小。
