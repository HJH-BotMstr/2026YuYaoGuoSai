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

## 九、机械臂连杆模型与 DH 参数说明

### 9.1 现实情况：项目中没有标准 DH 参数表

经过对代码、文档、旧资料的全面检索，**本项目目前不存在标准 DH 参数表**（即没有按 Denavit-Hartenberg 约定整理的 α、a、d、θ 参数表）。

机械臂的运动学求解采用的是**简化的平面三连杆几何模型**，直接根据连杆长度和末端位置用三角法求解关节角，没有建立完整的连杆坐标系变换矩阵。

### 9.2 连杆模型数据

当前代码中硬编码的连杆长度如下：

| 连杆 | 长度 | 说明 |
|------|------|------|
| L1   | 105 mm | 第一根连杆（靠近底座） |
| L2   | 100 mm | 第二根连杆 |
| L3   | 120 mm | 第三根连杆（末端连杆） |

定义位置：

- `tools/grasp/utils/RobotArm/three_Inverse_kinematics.py`
- `assets/old_code/DeepRobotDog/utils/RobotArm/three_Inverse_kinematics.py`

核心代码片段：

```python
# 定义连杆长度，单位为毫米
L1 = 105     # L1
L2 = 100     # L2
L3 = 120     # L3
```

### 9.3 舵机编号与功能

机械臂共 6 个舵机，编号与功能对应关系：

| 舵机 ID | 功能 |
|---------|------|
| 1       | 夹爪开合 |
| 2       | 底座旋转/横摆 |
| 3       | 三连杆第一关节 |
| 4       | 三连杆第二关节 |
| 5       | 三连杆第三关节 |
| 6       | 腕部/末端旋转 |

实际参与逆运动学计算的是 **3、4、5 号舵机**，对应三连杆的三个关节角。

### 9.4 

如果需要将机械臂接入 ROS2、Gazebo 或 MoveIt，必须自行推导或测量一套 DH 参数。推导时需要明确：

1. 各关节旋转轴方向（是绕 Z 轴旋转的旋转关节，还是平移关节）；
2. 相邻关节轴之间的公垂线长度 `a`；
3. 相邻关节轴之间的扭角 `α`；
4. 连杆偏置 `d` 和关节角 `θ` 的零位定义；
5. 末端夹爪相对于最后一个关节的固定变换。

在获得 DH 参数后，建议补充到本项目的 `doc/` 或 `tools/grasp/` 目录下，并替换当前的简化逆运动学实现，以便后续做仿真和更高精度的运动规划。

---

## 十、机械臂手眼标定方案

### 10.1 标定类型判断

本项目机械臂摄像头设备为 `/dev/video4`（见 `config.yaml`），摄像头安装在机械臂末端附近，属于 **Eye-in-Hand（眼在手上）** 标定。若后续改为固定机位拍摄机械臂，则按 **Eye-to-Hand（眼在手外）** 处理，下面以 Eye-in-Hand 为主进行说明，Eye-to-Hand 仅给出变换链差异。

### 10.2 坐标系与符号约定

| 坐标系 | 符号 | 说明 |
|--------|------|------|
| 机械臂基座标系 | {B} | 机械臂运动学参考坐标系，通常取底座或机身固定点 |
| 末端/夹爪坐标系 | {E} | 机械臂末端，摄像头安装点 |
| 摄像头坐标系 | {C} | 摄像头光心坐标系，Z 轴沿光轴向前 |
| 标定板坐标系 | {T} | 打印的 ArUco 板/棋盘格坐标系 |

Eye-in-Hand 待求量：

- `T_E^C`：摄像头相对于末端的位姿（手眼矩阵）。

变换链（Eye-in-Hand）：

```
T_B^T = T_B^E · T_E^C · T_C^T
```

其中：

- `T_B^E`：由机械臂正运动学（当前为三连杆平面模型）计算得到；
- `T_C^T`：由摄像头识别标定板并通过 `cv2.solvePnP` 得到；
- `T_E^C`：待标定的手眼矩阵。

Eye-to-Hand 变换链（仅作参考）：

```
T_B^T = T_B^C · T_C^T
```

此时待求量为摄像头相对于基座的位姿 `T_B^C`。

### 10.3 前置条件

1. **摄像头内参已标定**：`detection.arm_cam_fx/fy/cx/cy/dist` 已填入 `config.yaml`；
2. **机械臂关节角可读取**：能通过 `ArmController.read_positions()` 获取 3、4、5 号舵机当前脉冲值；
3. **标定板已准备**：推荐打印 `ArUco` 标定板（如 `cv2.aruco.DICT_4X4_50`，5×7 或更大），尺寸精确到毫米；
4. **标定板固定**：将标定板平放在地面/桌面上，确保标定过程中不移动；
5. **机械臂运动空间充足**：能够覆盖多个不同的位姿，姿态差异尽量大（平移 + 旋转都要变化）。

### 10.4 数据采集流程

建议采集 **12～20 组** 位姿，步骤如下：

1. 控制机械臂到达一个稳定位姿（可通过 `set_pose()` 或手动调整）；
2. 读取并记录当前 3、4、5 号舵机脉冲值；
3. 用机械臂摄像头拍摄一帧图像，保存图像；
4. 检测图像中的 ArUco 标定板，计算 `T_C^T`（`rvec_cam2target`, `tvec_cam2target`）；
5. 根据舵机脉冲值计算当前关节角，再由正运动学计算 `T_B^E`；
6. 将 `(T_B^E, T_C^T)` 作为一组样本加入列表；
7. 换一个差异较大的位姿，重复步骤 1～6。

> 位姿选择原则：尽量让标定板在图像中分布于不同位置，机械臂末端姿态（俯仰角）变化要明显，避免所有样本过于接近。

### 10.5 关节角与正运动学

当前机械臂没有完整 DH 参数，可先用简化平面模型做手眼标定的初版：

```python
import math

def servo_to_angle(pulse):
    """舵机脉冲 → 弧度，2047 为中位，4096 对应 360°"""
    return (pulse - 2047) * 2 * math.pi / 4096

def forward_kinematics(theta3, theta4, theta5):
    """基于三连杆平面模型的正运动学，返回末端在基座标系下的 (x, y, z, yaw, pitch, roll)
    注意：这是简化模型，实际使用时应替换为完整 DH/URDF 正运动学。
    """
    L1, L2, L3 = 105, 100, 120  # mm
    # 这里仅给出二维平面示例，具体实现需根据实际关节零位和方向调整
    x = L1 * math.cos(theta3) + L2 * math.cos(theta3 + theta4) + L3 * math.cos(theta3 + theta4 + theta5)
    y = 0.0  # 若底座无旋转，可暂设为 0
    z = L1 * math.sin(theta3) + L2 * math.sin(theta3 + theta4) + L3 * math.sin(theta3 + theta4 + theta5)
    return x, y, z
```

> 注意：上述正运动学是平面近似，手眼标定结果会受此影响。获得完整 DH 参数后应替换为正解。

### 10.6 调用 OpenCV 手眼标定

OpenCV 提供 `cv2.calibrateHandEye()`，输入为 N 组末端相对基座的位姿和标定板相对摄像头的位姿：

```python
import cv2
import numpy as np

R_gripper2base = []   # 末端相对基座的旋转矩阵列表
T_gripper2base = []   # 末端相对基座的平移向量列表
R_target2cam = []     # 标定板相对摄像头的旋转矩阵列表
T_target2cam = []     # 标定板相对摄像头的平移向量列表

# 将采集的样本填入上述列表
for sample in samples:
    R_gripper2base.append(sample['R_base2gripper'])
    T_gripper2base.append(sample['t_base2gripper'])
    R_target2cam.append(sample['R_cam2target'])
    T_target2cam.append(sample['t_cam2target'])

# 方法可选：CALIB_HAND_EYE_TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS
R_cam2gripper, T_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, T_gripper2base,
    R_target2cam, T_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

print("摄像头相对末端旋转矩阵:\n", R_cam2gripper)
print("摄像头相对末端平移向量:", T_cam2gripper)
```

### 10.7 标定结果验证

1. **重投影误差**：将标定板角点通过 `T_E^C` 和 `T_B^E` 投影回基座坐标系，与直接测量的标定板位置比较；
2. **验证位姿**：采集 3～5 组未参与标定的位姿，计算标定板在基座坐标系下的位置，检查是否稳定一致；
3. **抓取闭环验证**：让机械臂识别一个已知位置的物体，计算基座坐标系下目标位置，指挥机械臂到达，观察实际抓取偏差。

### 10.8 文件规划

建议在 `tools/grasp/` 下新增以下文件：

```
tools/grasp/
├── utils/
│   ├── HandEyeCalibration.py      # 手眼标定核心：采集、计算、保存
│   └── ForwardKinematics.py       # 机械臂正运动学（先用简化模型，后续替换为 DH）
├── tests/
│   ├── test_handeye_collect.py    # 数据采集脚本
│   └── test_handeye_verify.py     # 标定结果验证脚本
└── config/
    └── handeye_calib.yaml         # 标定结果保存文件
```

### 10.9 与抓取流程的集成

完成手眼标定后，抓取流程中目标位置转换如下：

1. `BlockDetection` 输出目标在图像中的像素坐标 `u, v` 和距离 `Z`；
2. 通过相机内参反投影，得到目标在摄像头坐标系下的三维坐标 `P_C`；
3. 利用标定结果 `T_E^C`，转换到末端坐标系 `P_E = T_E^C · P_C`；
4. 根据当前机械臂姿态，计算基座坐标系下目标位置 `P_B = T_B^E · P_E`；
5. 由 `P_B` 解算所需的 `(dis, height)`，调用 `ArmController.grap(dis, height)` 执行抓取。

### 10.10 现场标定 TODO 清单

- [ ] 打印 ArUco 标定板并精确测量尺寸；
- [ ] 标定 `/dev/video4` 摄像头内参，更新 `config.yaml`；
- [ ] 确认 3、4、5 号舵机零位方向与正运动学定义一致；
- [ ] 采集 12～20 组不同位姿的样本；
- [ ] 运行 `HandEyeCalibration.py` 计算 `T_E^C`；
- [ ] 保存结果到 `config/handeye_calib.yaml`；
- [ ] 用验证位姿和实际抓取闭环验证标定精度。