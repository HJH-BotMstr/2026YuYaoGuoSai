# Grasp 抓取模块重构设计文档

**日期**：2026-07-11  
**状态**：待实现  
**目标**：重构 grasp 模块，实现完整的"机器狗到站→视觉识别→对齐→抓取→运输→放置"流程，支持 robot/pc 双模式切换。

---

## 1. 整体流程

机器狗有两个关键站位：
- **place1**：由 AR 码引导机器狗到达的初始抓取站位，机械臂进入 mode=2 相机初始位姿
- **place2**：横向对齐后的精确站位，机械臂从此位置执行抓取

```
phase_0_init          初始化所有模块（ArmController / BlockDetection / 接口）
phase_1_standby       机械臂进入 mode=2，等待机器狗到达 place1 停稳信号
phase_2_detect        相机初始位姿下多帧滑动均值，锁定最近目标，输出稳定 (X_cam, Z_cam)
phase_3_align         若 |X_cam| > 阈值，通知机器狗横向调整到 place2，循环直到对齐
phase_4_approach      机械臂末端下降到抓取高度，再前进到抓取位置，执行夹取
phase_5_transport     切换运输姿态（keep_gripper=True）
phase_6_place         等放置触发信号，执行放置动作，松开夹爪
phase_7_home          归位 mode=0，准备下次任务
```

---

## 2. 运行模式开关

### 配置方式

```yaml
# config.yaml
runtime:
  mode: "pc"    # "robot" | "pc"
```

命令行参数优先级高于 config.yaml：

```bash
python3 main.py --mode pc
python3 main.py --mode robot
```

### 两种模式行为对比

| 阶段 | robot 模式 | pc 模式 |
|------|-----------|---------|
| phase_1 等待停稳 | 等待 ROS2 `/grasp/start` stub | 终端提示，按回车继续 |
| phase_3 横向对齐 | 发 ROS2 横向调整指令，等对齐回调 stub | 打印偏移值，手动调整后按回车 |
| phase_6 等待放置 | 等待 ROS2 `/grasp/place` stub | 终端提示，按回车触发放置 |

---

## 3. 坐标计算与抓取位置推算（方案 A：相机初始位姿近似法）

### 3.1 前提假设

机器狗到达 place2（横向对齐完成）后，机械臂保持 mode=2 不动，此时相机位姿固定，以此作为坐标计算的参考基准（近似替代机械臂基座坐标系）。由于只有关节 3/4/5 运动，相机相对基座仅有 Y/Z 方向变化，X 方向基本固定，该近似在水平前向距离上误差可通过 d_object 标定补偿。

### 3.2 坐标系定义（相机坐标系，原点在相机光心）

```
X：水平向右为正（画面左右方向）
Y：光轴向前为正（= distance_mm，即色块与相机的前向距离）
Z：垂直向下为正（画面上下方向）
```

### 3.3 色块 3D 位置计算（复用 BlockDetection.detect()）

针孔模型反投影（已在 BlockDetection 中实现，直接复用）：

```
Y_cam = fx * real_width_mm / bbox_width_px     # 前向距离
X_cam = (cx_block - cam_cx) / fx * Y_cam       # 左右偏移
Z_cam = (cy_block - cam_cy) / fy * Y_cam       # 垂直偏移（当前不用于 IK）
```

### 3.4 从相机坐标到 IK 输入的映射

place2 对齐完成后，相机初始位姿下取 N 帧均值得到稳定的 `(X_cam, Y_cam)`，映射关系：

```
grap() 输入:
  dis    = Y_cam_mean          # 水平前向距离，直接送 ArmController.grap()
  height = h_object            # 人工标定，config.yaml 设置，单位 mm
                               # 含义：色块底端以上 10mm，机械臂基座坐标系下的末端高度
```

**为什么可以直接用 Y_cam 作为 dis？**  
mode=2 时相机朝向与机械臂前进方向基本一致，Y_cam（光轴前向）≈ 水平地面距离。固定偏差通过 d_object 标定时人工补偿（不需要额外变换）。

### 3.5 抓取两步动作（phase_4_approach）

为提高抓取成功率，末端分两步到达目标：

```
步骤 1：先下降到抓取高度
  arm.grap(dis=当前dis_safe, height=h_object)
  dis_safe = 较远的安全距离（保持末端不碰到物块，默认 = Y_cam_mean + 50mm）
  等待到位: arm.wait_for_position(...)   ← 复用已有方法

步骤 2：水平前进到最终抓取位置
  arm.grap(dis=d_object, height=h_object)
  等待到位: arm.wait_for_position(...)   ← 复用已有方法

步骤 3：执行夹取
  arm.grasp_with_verify(dis=d_object, height=h_object)  ← 复用已有方法
  含内部重试逻辑（最多 grasp_retry_max 次）
```

`d_object` 为最终抓取距离，人工标定，表示末端夹爪刚好套住物块时的水平距离。`dis_safe` 在代码中计算为 `d_object + approach_clearance_mm`（config 可配，默认 50mm）。

---

## 4. 多目标选择与锁定

### 4.1 目标选择

`BlockDetection` 新增 `detect_all()` 方法，返回画面中所有满足面积阈值的候选列表（每项结构与 `detect()` 返回值相同）。  
phase_2 选 `Y_cam`（前向距离）最小的目标，即距离最近的色块优先。  
原 `detect()` 保留不变，兼容现有测试脚本。

### 4.2 目标锁定

选定目标后记录其 bbox 中心像素坐标 `(cx_locked, cy_locked)`。后续帧匹配规则：
- 计算所有候选中心与锁定中心的欧氏距离
- 选距离最近且 `< bbox短边 * 0.5` 的候选作为当前帧的锁定目标
- 若无匹配（目标消失），保持上一帧读数，累计消失帧超过 `lost_frames_max`（默认 10）则重新选目标

### 4.3 均值滤波

对锁定目标的 `Y_cam` 和 `X_cam` 分别维护滑动窗口（长度 = `distance_avg_window`，默认 20 帧）。窗口满后输出均值，才允许进入 phase_3/4。

---

## 5. 接口定义

### DogAlignInterface（横向对齐接口）

```python
class DogAlignInterface:
    def __init__(self, mode: str): ...  # mode = "robot" | "pc"

    def send_align(self, offset_x_mm: float) -> None:
        """发送横向偏移量（正=右移，负=左移）。
        robot: 发 ROS2 指令（stub，TODO: 接入实际 ROS2 topic）
        pc:    打印偏移值提示，不发指令
        """

    def wait_aligned(self, timeout: float = 10.0) -> bool:
        """等待对齐完成。
        robot: 等 ROS2 回调（stub，直接返回 True）
        pc:    打印偏移值，等用户回车，返回 True
        """
```

### RobotSignalInterface（启动/放置信号接口）

```python
class RobotSignalInterface:
    def __init__(self, mode: str): ...

    def wait_start(self) -> bool:
        """等待机器狗到达 place1 停稳信号（phase_1）。
        robot: 等 ROS2 /grasp/start（stub）
        pc:    打印提示，等回车
        """

    def wait_place(self, zone: str) -> bool:
        """等待机器狗到达放置站位信号，携带目标区信息（phase_6）。
        robot: 等 ROS2 /grasp/place（stub）
        pc:    打印提示，等回车
        """
```

---

## 6. config.yaml 新增/变更字段

```yaml
runtime:
  mode: "pc"                          # "robot" | "pc"

grasp:
  d_object: 220.0                     # [TODO: 现场标定] 最终抓取水平距离 mm
  h_object: 25.0                      # [TODO: 现场标定] 末端抓取高度 mm（基座坐标系）
  approach_clearance_mm: 50.0         # 下降阶段与目标的水平安全余量 mm
  align_offset_threshold_mm: 15.0     # X_cam 偏移超过此值才触发横向对齐
  lost_frames_max: 10                 # 目标连续丢失帧数超过此值则重新选目标
  distance_avg_window: 20             # 滑动均值窗口帧数（已有，保留）
  detect_timeout: 30.0                # 识别超时秒数（已有，保留）
```

原字段 `D_hand_mm` / `grasp_height_mm` 由 `d_object` / `h_object` 替代，保持语义一致。

---

## 7. 文件变更清单

| 文件 | 变更类型 | 说明 |
|------|---------|------|
| `main.py` | 重写 | 8 phase 流程，mode 分支 |
| `utils/BlockDetection.py` | 新增方法 | `detect_all()` 返回候选列表 |
| `utils/DogAlignInterface.py` | 新建 | 横向对齐接口，robot/pc 双实现 |
| `utils/RobotSignalInterface.py` | 新建 | 启动/放置信号接口，robot/pc 双实现 |
| `config.yaml` | 修改 | 新增 runtime / d_object / h_object 等字段 |
| `utils/ArmController.py` | 不变 | `grap()` / `grasp_with_verify()` / `wait_for_position()` / `set_pose(keep_gripper)` 全部复用 |

---

## 8. 可复用的现有代码

| 现有方法 | 用途 | 复用位置 |
|---------|------|---------|
| `arm.grap(dis, height)` | IK 求解 + 下发关节目标 | phase_4 两步动作 |
| `arm.grasp_with_verify(dis, height)` | 抓取 + 夹爪位置校验 + 重试 | phase_4 步骤 3 |
| `arm.wait_for_position(targets)` | 等待关节到位 | phase_4 步骤 1/2 |
| `arm.set_pose(mode, keep_gripper)` | 姿态切换 | phase_1/5/7 |
| `arm.open_gripper()` | 松开夹爪 | phase_6 |
| `BlockDetection.detect()` | 单目标检测 | 保留兼容 |
| `BlockDetection.visualize()` | 调试可视化 | phase_2/3 调试显示 |
| `InspectionMemory.get_zone()` | 获取目标放置区 | phase_6 |

---

## 9. 待标定参数

- `d_object`：机器狗停稳在 place2 后，夹爪刚好套住物块时的水平距离
- `h_object`：物块底端以上 10mm 对应的末端高度（机械臂基座坐标系）
- `approach_clearance_mm`：下降阶段保持的水平安全距离，防止下降时碰到物块
- `align_offset_threshold_mm`：现场测试确定横向对齐容差
