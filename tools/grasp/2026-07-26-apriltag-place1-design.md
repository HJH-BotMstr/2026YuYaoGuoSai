# AprilTag 视觉定位到达 place1 设计文档

**日期**：2026-07-26  
**状态**：待实现  
**目标**：利用 [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) 实现机械狗对墙上 AprilTag 的识别、对齐，并停在 Tag 正前方 20 cm 处，最后向 grasp 模块发出 `/grasp/start`（到达 place1）信号。优先复用 `lite3_ws/src/pose_control`、`tools/lite3_driver.py`、`tools/grasp/utils` 等现有代码。

---

## 1. 应用场景与坐标约定

### 1.1 场景

- 将指定 AprilTag（建议 **tag25h9** 家族，ID 固定，例如 `id=0`）打印后贴在墙上。
- Tag 中心高度与机械狗头部 RGB 摄像头光心齐平，减小俯仰角带来的测距误差。
- 机械狗从远处朝 Tag 方向行走，进入摄像头视野后启动闭环对齐。
- 最终停在 Tag 正前方 `target_distance_m = 0.20 m`，且机身正面大致平行于墙面。

### 1.2 坐标系

以**机械狗头部 RGB 摄像头**为参考：

| 轴 | 方向 | 说明 |
|---|---|---|
| `X` | 右正左负 | Tag 在画面中偏右 → 狗需要向右横移 |
| `Y` | 下正上负 | 仅用于判断 Tag 是否在合理高度范围内 |
| `Z` | 前正后负 | Tag 到摄像头的水平前向距离，≈ 到墙面的距离 |

> 这里 `Z` 对应机械狗前后方向，`X` 对应左右方向。后续控制指令全部转换到机身坐标系（`+x` 前进，`+y` 左移，`+z` 逆时针旋转）。

---

## 2. 整体架构

新增一个 ROS2 节点 `apriltag_place1_node`，与现有节点关系如下：

```
                         外部触发信号
                    /apriltag_place1/start
                               │
                               ▼
┌─────────────────┐     RGB 帧      ┌──────────────────────┐
│ 头部摄像头      │ ───────────────▶│ apriltag_place1_node │
│ (RealSense /dev/video4)           │  (本设计新增)        │
└─────────────────┘                 └──────────┬───────────┘
                                               │
                          ┌────────────────────┼────────────────────┐
                          │                    │                    │
                          ▼                    ▼                    ▼
                   ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
                   │ /move       │    │ /pose_control│    │ /grasp/start    │
                   │ (Pose2D)    │    │ /command     │    │ (Bool / String) │
                   └──────┬──────┘    └──────┬───────┘    └────────┬────────┘
                          │                    │                    │
                          └────────────────────┼────────────────────┘
                                               ▼
                              ┌─────────────────────────────┐
                              │  pose_controller_node       │
                              │  (lite3_ws/src/pose_control)│
                              └──────────────┬──────────────┘
                                             │ /cmd_vel
                                             ▼
                              ┌─────────────────────────────┐
                              │  lite3_driver.py            │
                              │  (已有 UDP 驱动)            │
                              └──────────────┬──────────────┘
                                             │ UDP
                                             ▼
                              ┌─────────────────────────────┐
                              │        绝影 Lite3           │
                              └─────────────────────────────┘
```

### 2.1 为什么这样拆分

- `pose_control` 已经有成熟的里程计闭环、速度指令融合、超声波避障、`/move` 与 `/pose_control/command` 接口。**不要重写运动控制**。
- `apriltag_place1_node` 只负责：
  1. 订阅外部触发信号 `/apriltag_place1/start`；
  2. 读摄像头；
  3. AprilTag 检测与位姿估计；
  4. 把视觉误差转换成 `/move` 和 `/pose_control/command`；
  5. 到位后发布 `/grasp/start`。

---

## 3. AprilTag 库选型与安装

### 3.1 推荐方案

在 Ubuntu + Python 3.8 环境下，**优先使用预编译 wheel**：

```bash
pip3 install pupil-apriltags
```

如 wheel 不可用，则从源码编译 AprilRobotics/apriltag 的 Python 绑定：

```bash
# 依赖
sudo apt-get install cmake libeigen3-dev

# 编译安装
mkdir -p ~/third_party && cd ~/third_party
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# Python 绑定（如果 apriltag 目录下存在 python 子目录）
cd ../apriltag-python  # 或对应路径
python3 setup.py build
pip3 install .
```

> 比赛中建议提前把 `pupil-apriltags` 固定到 `requirements.txt`，避免现场编译。

### 3.2 Python 使用示例

```python
import cv2
from pupil_apriltags import Detector

detector = Detector(
    families="tag25h9",
    nthreads=4,
    quad_decimate=1.0,      # 可降到 2.0 提高速度
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)

grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
tags = detector.detect(
    grey,
    estimate_tag_pose=True,
    camera_params=[fx, fy, cx, cy],
    tag_size=tag_size_m,
)
for tag in tags:
    if tag.tag_id == TARGET_TAG_ID:
        t = tag.pose_t.flatten()  # [tx, ty, tz] in camera frame
        # 这里 t[2] 是前向距离，t[0] 是左右偏移
```

---

## 4. 摄像头与内参

### 4.1 摄像头选择

使用机械狗头部 **Intel RealSense D435i 的 RGB 流**，对应设备号 `/dev/video4`（以现场 `v4l2-ctl --list-devices` 为准）。

### 4.2 分辨率与帧率

为兼顾识别距离与运算量，推荐：

```yaml
apriltag:
  camera_device: "/dev/video4"
  image_width: 640
  image_height: 480
  fps: 30
```

### 4.3 内参标定

- 用 `tools_box/camera_params/camera_params.py`（或 OpenCV `calibrateCamera`）标定 `/dev/video4`。
- 输出 `camera_matrix` 和 `dist_coeffs` 写入配置文件。
- 检测前必须 `cv2.undistort(frame, camera_matrix, dist_coeffs)`，否则边缘畸变会导致位姿抖动。

---

## 5. 对齐流程（核心算法）

流程设计遵循“**外部触发 → 先转后移、小步逼近、多帧稳定**”原则，保证稳定鲁棒。

```
phase_0_wait_trigger     等待外部触发信号（人工 / 导航模块），确认狗已大致朝墙
phase_1_wait_detect      等待摄像头就绪，循环检测目标 Tag
phase_2_yaw_align        旋转机身，使 Tag 位于画面中央（消除水平角偏差）
phase_3_lateral_align    根据 Tag 的 X 偏移，横向平移，使 Tag 正对摄像头
phase_4_approach         前进到 target_distance_m（默认 0.20 m）
phase_5_final_check      最终校验：角度、横向、距离都达标
phase_6_emit_signal      发布 /grasp/start，任务完成
```

### 5.1 关键变量

| 变量 | 含义 | 推荐值 |
|---|---|---|
| `trigger_topic` | 外部触发话题名 | `"/apriltag_place1/start"` |
| `target_tag_id` | 目标 AprilTag ID | `0` |
| `tag_family` | Tag 家族 | `"tag25h9"` |
| `tag_size_m` | 打印 Tag **黑色编码区域**的边长（米，不含白边） | `0.083`（8.3 cm，需量黑色区域） |
| `target_distance_m` | 最终距离墙面/Tag 的距离 | `0.20` |
| `yaw_align_threshold_deg` | 航向对准阈值 | `3.0` |
| `lateral_threshold_m` | 横向对准阈值 | `0.03` |
| `distance_threshold_m` | 距离到位阈值 | `0.02` |
| `max_rounds` | 最大对齐轮次 | `5` |
| `stable_frames` | 稳定帧数 | `10` |

### 5.2 phase_0：等待外部触发

- 节点启动后进入 `phase_0_wait_trigger`，**不主动运动**，只订阅外部触发信号。
- 触发话题：`/apriltag_place1/start`，类型 `std_msgs/Bool`。
- 收到 `data=True` 后，认为机器狗已被放到大致朝墙的方向，进入 `phase_1` 开始搜索 Tag。
- 如果收到 `data=False`，视为取消/复位，回到等待状态。

> 目前由**人工**通过 `ros2 topic pub` 触发；后续由**导航模块**在把狗带到站位附近后自动发布该 topic。

### 5.3 phase_1：搜索目标 Tag

- 摄像头打开后循环检测。
- 连续 `stable_frames` 帧检测到 `target_tag_id` 且位姿稳定，才进入下一步。
- 如果超过 `detect_timeout_s` 仍未检测到，认为触发时机不对，回到 `phase_0` 等待下一次触发。

### 5.4 phase_2：航向对准

- 输入：Tag 在相机系下的位置 `(tx, ty, tz)`。
- 计算水平角：
  ```
  alpha = atan2(tx, tz)
  ```
- 如果 `|alpha| > yaw_align_threshold_deg`，发布 `/pose_control/command reset_origin`，然后发布 `/move`：
  ```
  Pose2D(x=0.0, y=0.0, theta=-degrees(alpha))   # 负号：ROS 约定逆时针为正
  ```
- 等待 `/cmd_vel` 连续 0.5 s 接近零，视为旋转完成。
- 重新检测 Tag，重复最多 3 轮。

> 注意：这里用 Tag 的**水平视角**来对准航向，而不是直接横移，因为远距离时横移效率低，且先对准航向能显著降低后续横向误差。

### 5.5 phase_3：横向对准

- 航向对准后，Tag 应该基本在画面正中央，但机身可能还不在 Tag 正前方。
- 计算横向偏差 `tx`（单位 m）。
- 如果 `|tx| > lateral_threshold_m`，发布 `/move`：
  ```
  Pose2D(x=0.0, y=tx, theta=0.0)   # y 正方向为左移
  ```
- 等待到位后重新检测，重复最多 3 轮。

### 5.6 phase_4：前进到位

- 计算前向距离 `tz`。
- 需要前进的距离：`delta_z = tz - target_distance_m`。
- 发布 `/move`：
  ```
  Pose2D(x=delta_z, y=0.0, theta=0.0)
  ```
- 由于里程计漂移和视觉噪声，建议分两步：
  1. 先前进到 `target_distance_m + 0.05 m`；
  2. 停稳后重新用视觉测量，再前进剩余距离。

### 5.7 phase_5：最终校验

- 同时满足：
  - `|alpha| <= yaw_align_threshold_deg`
  - `|tx| <= lateral_threshold_m`
  - `|tz - target_distance_m| <= distance_threshold_m`
- 连续 `stable_frames` 帧满足条件，才进入下一步。

### 5.8 phase_6：发布 place1 信号

- 发布 `/grasp/start`：
  - 类型建议 `std_msgs/Bool`，`data=True`；或沿用 `std_msgs/String` `data="place1"`。
- `grasp/main.py` 中 `RobotSignalInterface.wait_start()` 订阅该 topic，触发后续抓取流程。

### 5.9 外部触发接口规范

`apriltag_place1_node` 对外暴露一个**开放触发接口**，方便人工调试或后续导航模块接入。

| 字段 | 值 |
|---|---|
| 话题名 | `/apriltag_place1/start` |
| 类型 | `std_msgs/Bool` |
| 含义 | `data=True`：启动对齐流程；`data=False`：取消/复位 |
| 发布方 | 目前：人工 / 调试脚本；后续：导航模块 |
| 订阅方 | `apriltag_place1_node` |

**人工触发示例**：

```bash
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: true" --once
```

**后续导航模块调用约定**：

- 导航模块负责把机器狗带到 Tag 附近（例如距离墙面 1.0 m 以内，机身大致朝墙）。
- 到达站位后，导航模块发布 `/apriltag_place1/start`。
- `apriltag_place1_node` 接管后续精细对齐，到位后再发 `/grasp/start`。
- 如果导航模块发 `data=False`，`apriltag_place1_node` 应停止当前运动，回到 `phase_0` 等待状态。

---

## 6. 与现有代码的复用

### 6.1 运动控制复用

| 现有代码 | 复用方式 |
|---|---|
| `lite3_ws/src/pose_control/pose_controller_node.py` | 作为位姿闭环控制器启动，订阅 `/move` 和 `/pose_control/command` |
| `tools/lite3_driver.py` | 已有 UDP 驱动，订阅 `/cmd_vel`，发布 `/leg_odom2`；启动 pose_control 时可用 `--use-driver` 一键启动 |
| `tools/yaw_controller.py` | 参考其 `rotate`/`move_x`/`move_y` 状态机，但不直接调用；统一走 `/move` 话题 |

### 6.2 视觉与接口复用

| 现有代码 | 复用方式 |
|---|---|
| `tools/grasp/utils/RobotSignalInterface.py` | 反向复用：本节点**发布** `/grasp/start`，该接口**订阅** |
| `tools/grasp/utils/DogAlignInterface.py` | 概念类似，但这里是“狗自己根据视觉对齐”，不依赖外部发横向指令 |
| `tools/grasp/utils/BlockDetection.py` | 参考其 `cv2.undistort`、相机内参组织方式、可视化方法 |
| `tools/grasp/main.py` | 启动参数解析、日志配置、config.yaml 加载方式 |

---

## 7. 节点设计（新增）

### 7.1 文件位置

建议放在 `lite3_ws/src/apriltag_place1/apriltag_place1/apriltag_place1_node.py`，作为一个新的 ROS2 包。

### 7.2 类设计

```python
class AprilTagPlace1Node(Node):
    def __init__(self):
        # 参数加载
        # 初始化 Detector
        # 打开摄像头
        # 订阅：trigger_topic、/leg_odom2（可选）、/cmd_vel（判断运动停止）
        # 发布：/move、/pose_control/command、/grasp/start
        # Timer：主循环 10 Hz

    def _trigger_cb(self, msg: Bool):
        # 处理外部触发信号，data=True 进入搜索阶段

    def _detect_tag(self, frame) -> Optional[dict]:
        # undistort -> grey -> detect -> 选 target_tag_id -> 返回 pose_t, pose_R

    def _is_cmd_vel_zero(self) -> bool:
        # 判断最近 0.5 s 内 /cmd_vel 是否持续接近零

    def _send_move(self, x, y, theta_deg):
        # 发布 Pose2D 到 /move

    def _reset_origin(self):
        # 发布 String("reset_origin") 到 /pose_control/command

    def _emit_place1(self):
        # 发布 Bool(data=True) 到 /grasp/start

    def _main_loop(self):
        # 状态机：wait_trigger / wait_detect / yaw_align / lateral_align /
        #         approach / final_check / done
```

### 7.3 状态机

```python
STATE_WAIT_TRIGGER = "wait_trigger"
STATE_WAIT_DETECT = "wait_detect"
STATE_YAW_ALIGN = "yaw_align"
STATE_LATERAL_ALIGN = "lateral_align"
STATE_APPROACH = "approach"
STATE_FINAL_CHECK = "final_check"
STATE_DONE = "done"
```

---

## 8. 配置文件

新增 `lite3_ws/src/apriltag_place1/config/apriltag_place1.yaml`：

```yaml
apriltag_place1:
  # 外部触发接口（人工 / 导航模块）
  trigger_topic: "/apriltag_place1/start"

  # 摄像头
  camera_device: "/dev/video4"
  image_width: 640
  image_height: 480
  fps: 30

  # 内参 [TODO: 现场标定]
  camera_matrix: [388.1454, 0.0, 329.4121, 0.0, 387.7497, 223.481, 0.0, 0.0, 1.0]
  dist_coeffs: [-0.1571, -0.218, -0.0024, -0.0011, 0.2089]

  # Tag 参数 [TODO: 根据打印尺寸修改]
  # tag_size_m 指黑色编码区域的边长，不包含白色外边框！
  tag_family: "tag25h9"
  target_tag_id: 0
  tag_size_m: 0.083

  # 目标位置（10cm 标签建议 0.20m，避免贴脸出画）
  target_distance_m: 0.20

  # 阈值
  yaw_align_threshold_deg: 3.0
  lateral_threshold_m: 0.03
  distance_threshold_m: 0.02

  # 流程控制
  max_rounds: 5
  stable_frames: 10
  detect_timeout_s: 10.0       # phase_1 搜索 Tag 超时（秒）
  cmd_vel_zero_timeout_s: 0.5
  move_timeout_s: 10.0
```

---

## 9. 文件变更清单

| 文件 | 变更类型 | 说明 |
|---|---|---|
| `lite3_ws/src/apriltag_place1/` | 新建 ROS2 包 | 节点、配置、launch、setup.py、package.xml |
| `tools/grasp/utils/RobotSignalInterface.py` | 修改 | `wait_start()` 改为订阅 `/grasp/start` topic |
| `tools/grasp/config.yaml` | 修改 | 新增 `apriltag_place1` 配置段（可选，也可单独放） |
| `requirements.txt`（若存在） | 修改 | 添加 `pupil-apriltags` |
| `lite3_ws/src/pose_control/launch/pose_control.launch.py` | 可选修改 | 如需默认启动 apriltag_place1，可加入 launch |

---

## 10. 待标定参数

| 参数 | 说明 | 当前值 |
|---|---|---|
| `camera_matrix` / `dist_coeffs` | `/dev/video4` RGB 摄像头内参 | 暂用机械臂摄像头参数，必须重新标定 |
| `tag_size_m` | 打印 Tag **黑色编码区域**的实际边长（不含白边） | `0.083`（需用尺子量黑色方块区域） |
| `target_distance_m` | 最终距墙面距离 | `0.20` |
| `yaw_align_threshold_deg` | 航向对准容差 | `3.0` |
| `lateral_threshold_m` | 横向对准容差 | `0.03` |
| `distance_threshold_m` | 距离到位容差 | `0.02` |

---

## 11. 启动流程

```bash
# 1. source 环境
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 2. 启动位姿控制器（如果需要 standalone 驱动，加 --use-driver）
ros2 run pose_control pose_control

# 3. 启动 AprilTag place1 节点
ros2 run apriltag_place1 apriltag_place1_node --ros-args --params-file src/apriltag_place1/config/apriltag_place1.yaml

# 4. （当前人工触发）把机器狗大致转到朝墙方向后，发送触发信号
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: true" --once

# 5. 启动 grasp 主流程（等待 /grasp/start）
cd /home/ysc/2026YuYaoGuoSai/tools/grasp
python3 main.py --mode robot
```

> 第 4 步目前由人工执行；后续导航模块开发完成后，由导航模块在把狗带到站位附近后自动发布 `/apriltag_place1/start`。

---

## 12. 鲁棒性设计

1. **多帧稳定**：每个阶段都连续多帧满足阈值才认为到位，避免单帧噪声导致误判。
2. **最大轮次限制**：`max_rounds` 限制每阶段重试次数，防止无限震荡。
3. **运动停止判断**：通过订阅 `/cmd_vel` 判断 pose_control 是否执行完毕，不盲目等待固定时间。
4. **丢失重检测**：如果某阶段 Tag 丢失，节点回到 `STATE_WAIT_DETECT` 重新搜索，而不是报错退出。
5. **定时心跳**：持续发布 `/move` 或 `/pose_control/command` 前检查里程计是否新鲜。
6. **Tag 打印建议**：
   - 使用 `tag36h11` 家族，ID 固定；
   - 打印后四周留白边（border），不要裁剪到黑框；
   - 用哑光材料，避免反光；
   - 尺寸不小于 15 cm，保证 1 m 外仍能稳定检测。

---

## 13. 注意事项

- `/dev/video4` 是 Intel RealSense，需要确认 V4L2 设备号是否稳定；不稳定时可用 `v4l2-ctl --list-devices` 配合 udev 规则固定。
- `pupil-apriltags` 与 `apriltag` Python 包 API 略有差异，实现时以实际安装的包为准。
- 如果现场 RealSense RGB 流不可用，可降级到机械臂 USB 摄像头 `/dev/video0`，但此时需要重新标定内参，且摄像头高度、视角不同，需重新调参。
- `/grasp/start` 的信号类型需要与 `RobotSignalInterface` 协商一致，建议用 `std_msgs/Bool`。
