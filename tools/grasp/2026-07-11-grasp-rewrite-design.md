# Grasp 抓取模块重构设计文档

**日期**：2026-07-11  
**状态**：已实现并调试通过  
**目标**：重构 grasp 模块，实现完整的"机器狗到站→视觉识别→对齐→抓取→运输→放置"流程，支持 robot/pc 双模式切换。

---

## 1. 整体流程

机器狗有两个关键站位：
- **place1**：由 AR 码引导机器狗到达的初始抓取站位，机械臂进入 mode=2 相机初始位姿
- **place2**：横向对齐后的精确站位，机械臂从此位置执行抓取

```
phase_0_init          初始化所有模块（ArmController / BlockDetection / TargetTracker / 接口）
phase_1_standby       机械臂进入 mode=2，等待机器狗到达 place1 停稳信号
phase_2_detect        相机初始位姿下多帧滑动均值，TargetTracker 锁定最近目标，输出稳定 (X_cam, Y_cam)
phase_3_align         若 |X_cam| > 阈值，通知机器狗横向调整到 place2，循环直到对齐
phase_4_approach      两步接近：先退到安全距离下降，再前进到物块位置执行夹取
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
python3 main.py --zone A   # 手动指定放置区
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

机器狗到达 place2（横向对齐完成）后，机械臂保持 mode=2 不动，此时相机位姿固定，以此作为坐标计算的参考基准（近似替代机械臂基座坐标系）。Y_cam（光轴前向）≈ 水平地面距离，固定系统误差通过 `distance_offset_mm` 标定补偿。Z_cam 受相机俯仰角影响误差较大，**不用于 IK 高度输入**。

### 3.2 坐标系定义（相机坐标系，原点在相机光心）

```
X：水平向右为正（画面左右方向）
Y：光轴向前为正（= distance_mm，即色块与相机的前向距离）
Z：垂直向下为正（画面上下方向）
```

### 3.3 色块 3D 位置计算（BlockDetection.detect_all）

针孔模型反投影（已在 BlockDetection 中实现）：

```
Y_cam = fx * real_width_mm / bbox_width_px     # 前向距离（原始值）
X_cam = (cx_block - cam_cx) / fx * Y_cam       # 左右偏移（用于横向对齐）
Z_cam = (cy_block - cam_cy) / fy * Y_cam       # 垂直偏移（仅用于参考，不送 IK）
```

### 3.4 从相机坐标到 IK 输入的映射

```
dis    = Y_cam_mean + distance_offset_mm   # 加固定补偿后的水平前向距离
height = h_object                          # 固定高度，人工标定
```

**distance_offset_mm 说明**：针孔模型原始测距与实际基座距离存在固定系统误差（约 +200mm），通过此参数补偿，无需重新标定内参。

### 3.5 抓取两步动作（phase_4）

```
步骤 1：先下降到 h_object，保持安全水平距离
  dis_safe = dis_target - approach_clearance_mm（不小于 30mm）
  arm.grap(dis_safe, h_object)
  arm.wait_for_position(...)

步骤 2：前进到物块位置并夹取
  arm.grasp_with_verify(dis=dis_target, height=h_object)
  含内部重试逻辑（最多 grasp_retry_max 次）
```

**注意**：phase_4 开始前调用 `cv2.destroyAllWindows()` 关闭预览窗口，防止 USB 带宽竞争导致摄像头卡死。

---

## 4. 多目标选择与锁定（TargetTracker）

### 4.1 目标选择

`BlockDetection.detect_all()` 返回所有满足面积阈值的候选列表（按 distance_mm 升序），每项结构与 `detect()` 相同。phase_2 通过 `TargetTracker` 自动选距离最近的目标。原 `detect()` 保留兼容。

### 4.2 目标锁定

`TargetTracker` 维护锁定状态：
- 首次检测：选 `distance_mm` 最小的目标锁定，记录 bbox 中心
- 后续帧：在候选中找欧氏距离最近且 `< bbox短边 * 0.5` 的目标继续跟踪
- 连续丢失帧数 `>= lost_frames_max` 则重置，重新选目标

### 4.3 滑动均值滤波

对锁定目标的 `Y_cam`、`X_cam` 分别维护滑动窗口（长度 = `distance_avg_window`，默认 20 帧）。窗口满后才输出稳定读数，才允许进入 phase_3/4。

---

## 5. 接口定义

### DogAlignInterface（横向对齐接口）

```python
class DogAlignInterface:
    def __init__(self, mode: str): ...  # mode = "robot" | "pc"

    def send_align(self, offset_x_mm: float) -> None:
        # robot: 发 ROS2 指令（stub，TODO: 接入实际 ROS2 topic）
        # pc:    打印偏移值提示

    def wait_aligned(self, timeout: float = 10.0) -> bool:
        # robot: 等 ROS2 回调（stub，直接返回 True）
        # pc:    直接返回 True（由 main.py phase_3 调 _pc_wait() 等回车）
```

### RobotSignalInterface（启动/放置信号接口）

```python
class RobotSignalInterface:
    def __init__(self, mode: str): ...

    def wait_start(self) -> bool:
        # robot: 等 ROS2 /grasp/start（stub）
        # pc:    直接返回 True

    def wait_place(self, zone: str) -> bool:
        # robot: 等 ROS2 /grasp/place（stub）
        # pc:    直接返回 True
```

---

## 6. config.yaml 字段说明

```yaml
runtime:
  mode: "pc"                          # "robot" | "pc"

grasp:
  h_object: 25.0                      # [现场标定] 末端抓取高度（mm，基座坐标系）
  distance_offset_mm: 200.0           # [现场标定] 视觉测距固定补偿（mm）
  approach_clearance_mm: 25.0         # 步骤1安全余量，dis_safe = dis_target - clearance
  align_offset_threshold_mm: 10.0     # X_cam 偏移超过此值才触发横向对齐
  lost_frames_max: 10                 # 目标连续丢失帧数超过此值则重新选目标
  distance_avg_window: 20             # 滑动均值窗口帧数
  detect_timeout: 30.0                # phase_2 识别超时（秒）
  grasp_retry_max: 3                  # 抓取失败最大重试次数
```

---

## 7. 文件变更清单

| 文件 | 变更类型 | 说明 |
|------|---------|------|
| `main.py` | 重写 | 8 phase 流程，robot/pc 双模式 |
| `utils/BlockDetection.py` | 新增方法 | `detect_all()` 返回候选列表，按距离升序 |
| `utils/TargetTracker.py` | 新建 | 多目标选择、锁定、滑动均值滤波 |
| `utils/DogAlignInterface.py` | 新建 | 横向对齐接口，robot/pc 双实现 |
| `utils/RobotSignalInterface.py` | 新建 | 启动/放置信号接口，robot/pc 双实现 |
| `config.yaml` | 修改 | 新增 runtime / h_object / distance_offset_mm 等字段 |
| `utils/ArmController.py` | 不变 | 全部复用 |

---

## 8. 可复用的现有代码

| 现有方法 | 用途 | 复用位置 |
|---------|------|---------|
| `arm.grap(dis, height)` | IK 求解 + 下发关节目标 | phase_4 步骤 1 |
| `arm.grasp_with_verify(dis, height)` | 抓取 + 夹爪位置校验 + 重试 | phase_4 步骤 2 |
| `arm.wait_for_position(targets)` | 等待关节到位 | phase_4 步骤 1 |
| `arm.set_pose(mode, keep_gripper)` | 姿态切换 | phase_1/5/7 |
| `arm.open_gripper()` | 松开夹爪 | phase_6 |
| `BlockDetection.detect()` | 单目标检测 | 保留兼容现有测试脚本 |
| `BlockDetection.visualize()` | 调试可视化 | phase_2 预览窗口 |
| `InspectionMemory.get_zone()` | 获取目标放置区 | phase_6 |

---

## 9. 待标定参数

| 参数 | 说明 | 当前值 |
|------|------|--------|
| `h_object` | 夹爪刚好套住物块时的基座高度（mm） | 25.0 |
| `distance_offset_mm` | 视觉测距与实际基座距离的固定差值 | 200.0 |
| `approach_clearance_mm` | 步骤1停在物块前方的安全余量 | 25.0 |
| `align_offset_threshold_mm` | 横向对齐容差 | 10.0 |

---

## 10. 已知问题与注意事项

- **摄像头设备号**：USB 拔插后 `/dev/videoX` 可能变化，运行前确认 `arm_cam_device` 配置正确
- **串口权限**：每次重启需 `sudo chmod 666 /dev/ttyUSB0`，或永久加组 `sudo usermod -aG dialout $USER`
- **Z_cam 不可用于高度**：相机俯仰角随机械臂运动变化，Z_cam 误差大，只能用固定 `h_object`
- **phase_4 前必须关闭预览窗口**：USB 带宽竞争会导致摄像头卡死，已在代码中处理
