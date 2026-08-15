# AprilTag 视觉定位到达 place1 设计文档

**日期**：2026-07-26（2026-07-28 按实现与真机验证结果更新）  
**状态**：已实现，2026-07-28 在 Lite3 真机上跑通全流程  
**目标**：利用 [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) 实现机械狗对墙上 AprilTag 的识别、对齐，视觉闭环停在 Tag 正前方 `target_distance_m` 处，再开环前进 `final_forward_offset_m`，最后向 grasp 模块发出 `/grasp/start`（到达 place1）信号。实现位于 `lite3_ws/src/grasp/apriltag_place1/`，复用 `lite3_ws/src/pose_control` 做运动闭环。

> 本文档最初为设计文档，现已更新为与实现一致的现状文档。所有参数默认值、状态名、日志行为均以 `lite3_ws/src/grasp/apriltag_place1/` 实际代码为准。

---

## 1. 应用场景与坐标约定

### 1.1 场景

- 将指定 AprilTag（**tag25h9** 家族，ID 固定为 `0`）打印后贴在墙上。
- Tag 中心高度与机械狗头部 RGB 摄像头光心齐平，减小俯仰角带来的测距误差。
- 机械狗从远处朝 Tag 方向行走，进入摄像头视野后由人工（后续为导航模块）发触发信号，启动闭环对齐。
- 视觉闭环停在 Tag 正前方 `target_distance_m`（当前配置 `0.08 m`），随后**开环**再前进 `final_forward_offset_m`（当前配置 `0.20 m`），使机械臂进入抓取范围，最后发布 `/grasp/start`。

### 1.2 坐标系

以**机械狗头部 RGB 摄像头**为参考：

| 轴 | 方向 | 说明 |
|---|---|---|
| `X` | 右正左负 | Tag 在画面中偏右 → 狗需要向右横移 |
| `Y` | 下正上负 | 仅用于判断 Tag 是否在合理高度范围内 |
| `Z` | 前正后负 | Tag 到摄像头的水平前向距离，≈ 到墙面的距离 |

> 这里 `Z` 对应机械狗前后方向，`X` 对应左右方向。控制指令转换到机身坐标系（`/move`：`+x` 前进，`+y` 左移，`+theta` 逆时针，角度制）。

---

## 2. 整体架构

节点 `apriltag_place1_node` 已实现，与现有节点关系如下：

```
                         外部触发信号
                    /apriltag_place1/start
                               │
                               ▼
┌─────────────────┐     RGB 帧      ┌──────────────────────┐
│ 头部摄像头      │ ───────────────▶│ apriltag_place1_node │
│ (RealSense /dev/video6)           │  (lite3_ws/src/      │
└─────────────────┘                 │   apriltag_place1)   │
                                    └──────────┬───────────┘
                                               │
                          ┌────────────────────┼────────────────────┐
                          │                    │                    │
                          ▼                    ▼                    ▼
                   ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
                   │ /move       │    │ /pose_control│    │ /grasp/start    │
                   │ (Pose2D)    │    │ /command     │    │ (std_msgs/Bool) │
                   └──────┬──────┘    └──────┬───────┘    └────────┬────────┘
                          │                  │                     │
                          └──────────────────┤                     │
                                             ▼                     ▼
                              ┌─────────────────────────────┐   ┌────────────────┐
                              │  pose_controller_node       │   │ grasp_node     │
                              │  (lite3_ws/src/pose_control)│   │ (grasp_task 包)│
                              └──────────────┬──────────────┘   └────────────────┘
                                             │ /cmd_vel
                                             ▼
                              ┌─────────────────────────────┐
                              │  官方 ROS2 栈（运动主机）    │
                              │  提供 /leg_odom2，订阅      │
                              │  /cmd_vel                   │
                              └──────────────┬──────────────┘
                                             ▼
                              ┌─────────────────────────────┐
                              │        绝影 Lite3           │
                              └─────────────────────────────┘
```

### 2.1 为什么这样拆分

- `pose_control` 已经有成熟的里程计闭环、速度指令融合、超声波避障、`/move` 与 `/pose_control/command` 接口。**不重写运动控制**。
- `apriltag_place1_node` 只负责：
  1. 订阅外部触发信号 `/apriltag_place1/start`；
  2. 读摄像头；
  3. AprilTag 检测与位姿估计；
  4. 把视觉误差转换成 `/move` 和 `/pose_control/command`；
  5. 到位后发布 `/grasp/start`（`std_msgs/Bool`，`data=True`）。

---

## 3. AprilTag 库选型与安装

### 3.1 实际使用方案

使用预编译 wheel（真机已验证可用）：

```bash
pip3 install pupil-apriltags
```

### 3.2 Python 使用示例

```python
import cv2
from pupil_apriltags import Detector

detector = Detector(
    families="tag25h9",
    nthreads=4,
    quad_decimate=1.0,
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
        # t[2] 是前向距离，t[0] 是左右偏移
```

> 以上参数与节点实际初始化一致（见 `apriltag_place1_node.py`）。

---

## 4. 摄像头与内参

### 4.1 摄像头选择

使用机械狗头部 **Intel RealSense D435i 的 RGB 流**，当前配置设备号为 `/dev/video6`（真机实测；`/dev/video4` 也是 RealSense 流，`/dev/video0` 是机械臂 USB 摄像头）。设备号重插后可能变化，以现场 `v4l2-ctl --list-devices` 为准。

### 4.2 分辨率与帧率

```yaml
camera_device: "/dev/video6"
image_width: 640
image_height: 480
fps: 30
```

### 4.3 内参标定（未完成）

- 当前配置中的 `camera_matrix` / `dist_coeffs` 仍**借用机械臂摄像头内参**，仅供调试，测距存在系统误差，上机前必须用头部 RealSense RGB 重新标定（OpenCV 棋盘格 `calibrateCamera` 即可）。
- **注意**：由于内参尚未重新标定，当前实现的 `_detect_tag()` **跳过了 `cv2.undistort`**（见代码注释"内参未重标定时跳过 undistort"）。待内参标定完成后，应恢复 undistort 以消除边缘畸变导致的位姿抖动。

---

## 5. 对齐流程（核心算法，已实现）

流程遵循"**外部触发 → 先转后移、小步逼近、多帧稳定**"原则。实际实现共 8 个阶段：

```
phase_0_wait_trigger     等待外部触发信号（人工 / 导航模块）
phase_1_wait_detect      循环检测目标 Tag，连续多帧稳定才锁定
phase_2_yaw_align        旋转机身，消除水平角偏差（单次旋转限幅）
phase_3_lateral_align    横向平移，使 Tag 正对摄像头
phase_4_approach         前进到 target_distance_m
phase_5_final_check      最终校验（角度 + 横向 + 距离同时达标）
phase_6_final_forward    开环额外前进 final_forward_offset_m（无视觉反馈）
phase_7_emit_signal      发布 /grasp/start，进入 done
```

状态常量（`apriltag_place1_node.py`）：`wait_trigger / wait_detect / yaw_align / lateral_align / approach / final_check / final_forward / done / error`。任何阶段 Tag 丢失都回到 `wait_detect` 重新搜索；任何阶段超过 `max_rounds` 或运动链路未就绪都进入 `error`（停止运动，可用触发信号重新启动）。

### 5.1 关键参数（当前配置值）

| 参数 | 含义 | 当前值（config yaml） |
|---|---|---|
| `trigger_topic` | 外部触发话题名 | `"/apriltag_place1/start"` |
| `target_tag_id` | 目标 AprilTag ID | `0` |
| `tag_family` | Tag 家族 | `"tag25h9"` |
| `tag_size_m` | 打印 Tag **黑色编码区域**边长（米，不含白边） | `0.083` |
| `target_distance_m` | 视觉闭环目标距离 | `0.08` |
| `approach_interim_offset_m` | approach 两步靠近的中间点偏移，`0.0` = 一步直接到位 | `0.0` |
| `final_forward_offset_m` | final_check 通过后开环额外前进距离 | `0.20` |
| `yaw_align_threshold_deg` | 航向对准阈值 | `3.0` |
| `max_yaw_step_deg` | 单次旋转上限（防惯性过冲） | `3.0` |
| `lateral_threshold_m` | 横向对准阈值 | `0.03` |
| `distance_threshold_m` | 距离到位阈值 | `0.02` |
| `max_rounds` | 每阶段最大调整轮次 | `10` |
| `stable_frames` | 稳定帧数 | `10` |
| `detect_timeout_s` | phase_1 搜索超时 | `10.0` |
| `cmd_vel_zero_timeout_s` | 判定运动停止的零速窗口（狗惯性大，需加长） | `1.5` |
| `move_timeout_s` | 单步运动最大等待时间 | `10.0` |

> 注意：`apriltag_place1_node.py` 中 `declare_parameter` 的代码默认值与 yaml 不同（如 `target_distance_m` 代码默认 `0.20`、yaml 为 `0.08`），**以 yaml 为准**，launch 和 `--params-file` 方式都会加载 yaml。

### 5.2 phase_0：等待外部触发

- 节点启动后进入 `wait_trigger`，**不主动运动**。
- 触发话题 `/apriltag_place1/start`，类型 `std_msgs/Bool`。
- 收到 `data=True` 后进入 `wait_detect`；在 `error` 状态下收到 `data=True` 也可重新启动。
- 收到 `data=False` 视为取消：发送 `/move (0,0,0)` 停止当前运动，回到 `wait_trigger`（`done` 状态下忽略）。

### 5.3 phase_1：搜索目标 Tag

- 循环检测，将每帧位姿放入长度为 `stable_frames` 的缓冲；缓冲满且 `std(tx) < 0.05 m`、`std(tz) < 0.05 m` 才认为"稳定锁定"，进入 `yaw_align`。
- 超过 `detect_timeout_s` 未锁定则输出诊断统计（处理帧数、检测到任意 Tag 的帧数、目标 ID 帧数等），回到 `wait_trigger` 等待下一次触发。

### 5.4 phase_2：航向对准

- 计算水平角 `alpha = atan2(tx, tz)`。
- `|alpha| <= yaw_align_threshold_deg` → 进入 `lateral_align`。
- 否则发出旋转指令（**单次限幅**，防止惯性过冲）：
  ```
  theta_cmd = -clamp(alpha_deg, ±max_yaw_step_deg)   # 负号：ROS 逆时针为正
  ```
  发送顺序：先 `/pose_control/command` 发 `reset_origin`，`sleep(0.15)` 等待其生效，再发 `/move Pose2D(0, 0, theta_cmd)`。
- 首次发运动指令前执行**运动链路检查**（见 §5.10），未就绪直接进入 `error`。
- 运动停止（§5.9）后重新检测，重复最多 `max_rounds` 轮，超轮进 `error`。

### 5.5 phase_3：横向对准

- 横向偏差 `tx`，`|tx| <= lateral_threshold_m` → 进入 `approach`。
- 否则发 `/move Pose2D(x=0, y=-tx, theta=0)`（`/move` y 正方向为左移，`tx` 相机系右正，故取负）。
- 停止后重新检测，重复最多 `max_rounds` 轮。

### 5.6 phase_4：前进到位

- 步骤 1（仅当 `approach_interim_offset_m > 0` 时有效）：先前进到 `target_distance_m + approach_interim_offset_m`；当前配置为 `0.0`，即直接一步到位。
- 步骤 2：精确逼近，`delta = tz - target_distance_m`，`|delta| <= distance_threshold_m` → 进入 `final_check`，否则发 `/move Pose2D(delta, 0, 0)`，最多 `max_rounds` 轮。

### 5.7 phase_5：最终校验

- 同时满足以下三条并累计满 `stable_frames` 帧才通过：
  - `|alpha| <= yaw_align_threshold_deg`
  - `|tx| <= lateral_threshold_m`
  - `|tz - target_distance_m| <= distance_threshold_m`
- **任何一项不达标**：清空稳定缓冲、重置各阶段轮次计数，回到 `yaw_align` 重新修正（真机上近距离段反复修正 yaw 是常态，见 §16 验证记录）。

### 5.8 phase_6：开环额外前进（final_forward）

- final_check 通过后，**不再使用视觉反馈**（距墙太近 Tag 容易出画）：发 `reset_origin`，`sleep(0.15)`，发 `/move Pose2D(final_forward_offset_m, 0, 0)`。
- 运动停止后进入下一阶段。该阶段开始前同样做运动链路检查。

### 5.9 phase_7：发布 place1 信号

- 发布 `/grasp/start`（`std_msgs/Bool`，`data=True`），状态进入 `done`。
- 订阅方：`grasp_task` 包的 `grasp_node`（`start_topic` 参数，默认 `/grasp/start`），以及旧版 `tools/grasp/utils/RobotSignalInterface.py` 的 `wait_start()`（同样订阅 `std_msgs/Bool`）。

### 5.10 运动完成判定与链路诊断

- **运动完成**：必须先在本次指令后观察到非零 `/cmd_vel`（速度绝对值之和 ≥ 0.01），随后速度持续 `cmd_vel_zero_timeout_s`（当前 1.5 s）低于 0.01，才认为完成——避免"指令被忽略却误判完成"。
- **链路检查**（首次发指令前与 final_forward 前执行）：`/move` 有订阅者、`/leg_odom2` 与 `/cmd_vel` 在 0.5 s 内有数据，否则报错进 `error`。
- 指令发出后超过 `cmd_vel_zero_timeout_s` 仍未见非零 `/cmd_vel`，输出"运动指令可能被控制器忽略"警告（通常意味着 `/leg_odom2` 未发布）。

### 5.11 外部触发接口规范

| 字段 | 值 |
|---|---|
| 话题名 | `/apriltag_place1/start` |
| 类型 | `std_msgs/Bool` |
| 含义 | `data=True`：启动/重启对齐流程；`data=False`：取消/复位 |
| 发布方 | 目前：人工 / 调试脚本；后续：导航模块 |
| 订阅方 | `apriltag_place1_node` |

**人工触发示例**：

```bash
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: true" --once
```

**后续导航模块调用约定**：

- 导航模块负责把机器狗带到 Tag 附近（例如距离墙面 1.0 m 以内，机身大致朝墙）。
- 到达站位后发布 `/apriltag_place1/start`，本节点接管精细对齐，到位后发 `/grasp/start`。
- 发 `data=False` 可让本节点停止运动、回到 `wait_trigger`。

---

## 6. 与现有代码的复用

### 6.1 运动控制复用

| 现有代码 | 复用方式 |
|---|---|
| `lite3_ws/src/pose_control/pose_controller_node.py` | 由 `apriltag_place1.launch.py` 一并启动，订阅 `/move` 和 `/pose_control/command`，launch 中传入真机调好的参数（见 §11） |
| 官方 ROS2 栈（`lite_cog_ros2/transfer`） | 提供 `/leg_odom2`、订阅 `/cmd_vel`，运动前需先发 `/simple_cmd` 序列（见 §15.1） |
| `tools/yaw_controller.py` | 参考其状态机思路，未直接调用；统一走 `/move` 话题 |

### 6.2 视觉与接口复用

| 现有代码 | 复用方式 |
|---|---|
| `tools/grasp/utils/RobotSignalInterface.py` | 已改为订阅 `/grasp/start`（`std_msgs/Bool`），与本节点发布端对接（旧版 grasp 流程用） |
| `lite3_ws/src/grasp/grasp_task` | 新版 grasp ROS2 包，`grasp_node` 订阅 `/grasp/start` 启动 8 阶段抓取状态机（推荐） |
| `tools/grasp/utils/BlockDetection.py` | 参考其相机内参组织方式、可视化方法 |
| `tools/grasp/main.py` | 参考其参数解析、日志配置、config.yaml 加载方式 |

---

## 7. 节点实现（已实现）

### 7.1 文件位置

`lite3_ws/src/grasp/apriltag_place1/`（ROS2 包，含 `apriltag_place1_node.py`、`config/apriltag_place1.yaml`、`launch/apriltag_place1.launch.py`、`README.md`）。

### 7.2 主要方法（与代码一致）

```python
class AprilTagPlace1Node(Node):
    # 订阅：trigger_topic(Bool)、/cmd_vel(Twist)、/leg_odom2(Odometry)
    # 发布：/move(Pose2D)、/pose_control/command(String)、/grasp/start(Bool)
    # Timer：主循环 10 Hz

    _open_camera()          # 打开摄像头并读一帧验证；失败则 _cap=None，主循环每帧重试
    _trigger_cb(msg)        # True→wait_detect（wait_trigger/error 状态可启动）；False→停运动回 wait_trigger
    _detect_tag(frame)      # grey→detect→选 target_tag_id→返回 {tx,ty,tz,R}；带 cv2.imshow 调试窗口
    _is_stable(pose)        # 稳定缓冲满且 std(tx),std(tz) < 0.05
    _is_cmd_vel_zero()      # 须先出现过非零速度，再持续零速才算运动完成
    _wait_motion_done()     # 阻塞等待运动完成（当前主要用非阻塞轮询）
    _check_motion_pipeline()# /move 订阅者 + /leg_odom2、/cmd_vel 新鲜度检查
    _send_move(x,y,theta)   # 发布 Pose2D（theta 角度制）
    _reset_origin()         # 发布 String("reset_origin")
    _emit_place1()          # 发布 Bool(True) 到 /grasp/start
    _main_loop()            # 10 Hz 状态机分发
    _do_wait_detect / _do_yaw_align / _do_lateral_align / _do_approach
    _do_final_check / _do_final_forward
```

### 7.3 状态机

```python
STATE_WAIT_TRIGGER   = "wait_trigger"
STATE_WAIT_DETECT    = "wait_detect"
STATE_YAW_ALIGN      = "yaw_align"
STATE_LATERAL_ALIGN  = "lateral_align"
STATE_APPROACH       = "approach"
STATE_FINAL_CHECK    = "final_check"
STATE_FINAL_FORWARD  = "final_forward"
STATE_DONE           = "done"
STATE_ERROR          = "error"
```

---

## 8. 配置文件

`lite3_ws/src/grasp/apriltag_place1/config/apriltag_place1.yaml`（当前实际内容）：

```yaml
apriltag_place1:
  ros__parameters:
    # ── 外部触发接口 ──
    trigger_topic: "/apriltag_place1/start"

    # ── 摄像头（头部 Intel RealSense D435i RGB 流）──
    # /dev/video4 也是 RealSense，/dev/video0 是机械臂 USB 摄像头
    camera_device: "/dev/video6"
    image_width: 640
    image_height: 480
    fps: 30

    # ── 内参 [TODO: 用 RealSense RGB 重新标定，当前借用机械臂内参仅供调试] ──
    camera_matrix: [388.1454, 0.0, 329.4121, 0.0, 387.7497, 223.481, 0.0, 0.0, 1.0]
    dist_coeffs: [-0.1571, -0.218, -0.0024, -0.0011, 0.2089]

    # ── Tag 参数（tag_size_m 须量黑色编码区域边长，不含白边）──
    tag_family: "tag25h9"
    target_tag_id: 0
    tag_size_m: 0.083

    # ── 目标位置 ──
    target_distance_m: 0.08
    approach_interim_offset_m: 0.0    # 两步靠近中间点偏移，0.0=一步直接到位
    final_forward_offset_m: 0.20      # final_check 通过后开环额外前进距离

    # ── 对准阈值 ──
    yaw_align_threshold_deg: 3.0
    max_yaw_step_deg: 3.0             # 单次旋转上限，防惯性过冲
    lateral_threshold_m: 0.03
    distance_threshold_m: 0.02

    # ── 流程控制 ──
    max_rounds: 10                    # 步长小，允许更多轮次
    stable_frames: 10
    detect_timeout_s: 10.0
    cmd_vel_zero_timeout_s: 1.5       # 狗惯性大，零速窗口需加长
    move_timeout_s: 10.0
```

---

## 9. 文件变更清单（实际落地情况）

| 文件 | 状态 | 说明 |
|---|---|---|
| `lite3_ws/src/grasp/apriltag_place1/` | 已建立 | 节点、配置、launch、setup.py、package.xml、README.md |
| `tools/grasp/utils/RobotSignalInterface.py` | 已修改 | `wait_start()` 已改为订阅 `/grasp/start`（`std_msgs/Bool`） |
| `lite3_ws/src/grasp/grasp_task/` | 已建立 | 新版 grasp ROS2 包（推荐启动方式），含 `launch/grasp.launch.py` |
| `tools/grasp/config.yaml` | 未改动 | apriltag 配置单独放在 apriltag_place1 包内 |
| `requirements.txt` | — | `pupil-apriltags` 需手动 `pip3 install` |

---

## 10. 待标定参数

| 参数 | 说明 | 当前值 |
|---|---|---|
| `camera_matrix` / `dist_coeffs` | RealSense RGB 摄像头内参 | **仍借用机械臂摄像头参数，必须重新标定**（标定后恢复 undistort） |
| `tag_size_m` | 打印 Tag **黑色编码区域**实际边长（不含白边） | `0.083` |
| `target_distance_m` | 视觉闭环目标距离 | `0.08` |
| `final_forward_offset_m` | 开环额外前进距离 | `0.20` |
| `yaw_align_threshold_deg` | 航向对准容差 | `3.0` |
| `lateral_threshold_m` | 横向对准容差 | `0.03` |
| `distance_threshold_m` | 距离到位容差 | `0.02` |

---

## 11. 启动流程（当前推荐方式）

```bash
# 0. 编译（grasp_task 是新包，首次必须 build）
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select apriltag_place1 pose_control grasp_task --symlink-install
source install/setup.bash

# 1. 启动对齐（launch 同时拉起 pose_control + apriltag_place1_node，自动加载 yaml）
ros2 launch apriltag_place1 apriltag_place1.launch.py

# 2. 另开终端，启动抓取节点（进入 STANDBY 等待 /grasp/start；调试可加 dry_run:=true）
ros2 launch grasp_task grasp.launch.py

# 3. 另开终端，把狗大致转到朝墙方向后触发
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: true" --once
```

`apriltag_place1.launch.py` 给 `pose_control` 传入的真机调参（不要随手改）：

| 参数 | 值 | 原因 |
|---|---|---|
| `kp_dist` / `kp_lateral` | `2.0` | 狗对低速指令响应差，提高增益让小距离也能产生可见速度 |
| `dist_threshold` / `yaw_threshold` | `0.015` / `0.025` | 控制器到位阈值必须**小于** apriltag_place1 的阈值，否则上层一直发指令狗却不动 |
| `obstacle_stop_dist` | `0.35` | 超声波避障距离 |
| `enable_terminal` | `false` | launch 下关闭终端交互 |

旧版 grasp 流程（`cd tools/grasp && python3 main.py --mode robot`）仍可用，同样等待 `/grasp/start`，但推荐用 `grasp_task`。

---

## 12. 鲁棒性设计（实现情况）

1. **多帧稳定**：`stable_frames` 缓冲 + 标准差判据（`std < 0.05 m`），避免单帧噪声误判。
2. **最大轮次限制**：每阶段 `max_rounds`（当前 10）限制重试，超轮进 `error` 而非无限震荡。
3. **运动停止判断**：订阅 `/cmd_vel`，要求"先出现过非零速度、再持续零速"才算完成，防止指令被忽略时误判。
4. **丢失重检测**：各阶段 Tag 丢失统一回到 `wait_detect` 重新搜索，不报错退出。
5. **链路预检**：首次发运动指令与 final_forward 前检查 `/move` 订阅者与 `/leg_odom2`、`/cmd_vel` 新鲜度。
6. **单次旋转限幅**：`max_yaw_step_deg` 限制单轮旋转量，防惯性过冲。
7. **Tag 打印建议**：
   - 使用 `tag25h9` 家族，ID 固定为 0；
   - 打印后四周留白边，不要裁剪到黑框；
   - 用哑光材料，避免反光；
   - 黑色编码区域边长建议 8 cm 以上，保证 1 m 外稳定检测。

---

## 13. 注意事项

- RealSense 的 V4L2 设备号不稳定（真机上出现过 `/dev/video6` 首次打开失败、重试后成功的情况），节点会在摄像头未就绪时每帧尝试重开；必要时用 udev 规则固定。
- 节点带 `cv2.imshow("apriltag_place1", ...)` 调试窗口，**需要显示环境**（真机本地桌面或 X 转发）；纯无头环境需注释掉。
- `/grasp/start` 类型已统一为 `std_msgs/Bool`，与 `grasp_task` 及 `RobotSignalInterface` 一致。
- 如果 RealSense RGB 流不可用，可降级到机械臂 USB 摄像头 `/dev/video0`，但需重新标定内参并重调参数（摄像头高度、视角不同）。

---

## 14. 上机必做检查清单

### 14.1 依赖安装

```bash
python3 -c "from pupil_apriltags import Detector; print('OK')"
# 若报错：pip3 install pupil-apriltags
```

### 14.2 确认摄像头设备号

```bash
v4l2-ctl --list-devices
```

找到 `Intel RealSense` 对应的 `/dev/videoX`（当前配置 `/dev/video6`），填入 `config/apriltag_place1.yaml` → `camera_device`。验证读帧：

```bash
python3 -c "
import cv2
cap = cv2.VideoCapture('/dev/video6', cv2.CAP_V4L2)
ret, f = cap.read()
print('帧大小:', f.shape if ret else '读帧失败')
cap.release()
"
```

### 14.3 标定头部摄像头内参（当前未完成）

config 中的默认内参来自机械臂摄像头，**必须替换为 RealSense RGB 的内参**（OpenCV 棋盘格标定），否则位姿估计存在系统误差。标定后把 `camera_matrix`（3×3 展开为 9 个数）和 `dist_coeffs`（5 个数）写入配置文件，并在 `_detect_tag()` 中恢复 `cv2.undistort`。

### 14.4 量取 Tag 实际尺寸

`tag_size_m` 必须是打印后**黑色编码区域的实际边长（米）**，不含白色外边框。典型值：A4 纸打印约 8~9 cm。量完后更新配置文件，误差 ±2 mm 以内可接受。

### 14.5 编译并 source

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select apriltag_place1
source install/setup.bash
```

### 14.6 静态检测验证

不启动 pose_control、不发触发信号，先静态验证检测（把 `DEVICE` 和内参改成现场值）：

```bash
python3 - <<'EOF'
import cv2
from pupil_apriltags import Detector

DEVICE   = "/dev/video6"
FAMILY   = "tag25h9"
TAG_ID   = 0
TAG_SIZE = 0.083
FX, FY, CX, CY = 388.1454, 387.7497, 329.4121, 223.481

cap = cv2.VideoCapture(DEVICE)
detector = Detector(families=FAMILY, nthreads=4)
print("按 q 退出，正在检测...")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(grey, estimate_tag_pose=True,
                           camera_params=[FX, FY, CX, CY], tag_size=TAG_SIZE)
    for t in tags:
        if t.tag_id == TAG_ID:
            tx, ty, tz = t.pose_t.flatten()
            cv2.putText(frame, f"id={t.tag_id}  tx={tx:.3f}  tz={tz:.3f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            print(f"tx={tx:.3f}m  ty={ty:.3f}m  tz={tz:.3f}m")
    cv2.imshow("apriltag", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
EOF
```

预期：正对 Tag 时 `tx ≈ 0`，`tz ≈ 实际距离`。误差偏大先查内参和 `tag_size_m`。

### 14.7 调参建议顺序

1. 先确认静态检测正常（14.6）
2. 固定狗，只测 yaw_align（发触发信号，观察旋转方向是否正确）
3. 固定狗，只测 lateral_align（观察横移方向）
4. 完整跑通一次，观察 final_check 是否稳定通过
5. 调整 `stable_frames`、各阈值到现场合适的值

---

## 15. 使用指南

### 15.0 真机前置步骤（运动主机官方栈）

在运动主机上先 source 官方栈并发的 `/simple_cmd` 序列让狗进入可运动状态：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/transfer/install/setup.bash

# 1) 回零
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553716741, size: 0, type: 0}" --once
# 2) 起立
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553714178, size: 0, type: 0}" --once
# 3) 切换到 locomotion（运动）模式
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553716998, size: 0, type: 0}" --once
# 4) 切换到 autonomous（自动）模式
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553716739, size: 0, type: 0}" --once
# 5) 设置步态为 slow
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553714432, size: 0, type: 0}" --once
```

确认 `/leg_odom2` 有数据（`ros2 topic echo /leg_odom2`）后再启动本节点。

### 15.1 日常启动（两终端 + 触发）

**终端 1 — 对齐（launch 一键，含 pose_control）**

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch apriltag_place1 apriltag_place1.launch.py
```

**终端 2 — grasp（ROS2 版，推荐）**

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch grasp_task grasp.launch.py          # 调试可加 dry_run:=true
```

grasp 启动后进入 STANDBY，阻塞等待 `/grasp/start`。

（旧版仍可用：`cd tools/grasp && python3 main.py --mode robot`，同样等待 `/grasp/start`。）

### 15.2 手动触发对齐

```bash
# 启动对齐
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: true" --once

# 取消 / 复位（节点停止运动，回到等待状态）
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: false" --once
```

### 15.3 监控运行状态

```bash
ros2 node list | grep apriltag     # 节点是否在线
ros2 topic echo /move              # 当前 /move 指令
ros2 topic echo /cmd_vel           # 机器人速度
ros2 topic echo /grasp/start       # 是否已发出抓取信号
ros2 topic echo /grasp/state       # grasp 状态机进度（grasp_task）
```

### 15.4 常见问题排查

| 现象 | 可能原因 | 排查方法 |
|---|---|---|
| 节点启动后立即崩溃 | `pupil-apriltags` 未安装 | `pip3 install pupil-apriltags` |
| 摄像头打开失败 | 设备号错误或 RealSense 未就绪 | `v4l2-ctl --list-devices`；节点会每帧重试，可稍等 |
| 发触发信号后无反应 | 节点未订阅到 topic | `ros2 topic info /apriltag_place1/start` 确认订阅方 |
| `tz` 与实际距离偏差大 | 内参未重新标定 / `tag_size_m` 不准 | 重新标定，重新量 Tag 尺寸 |
| 旋转方向反了 | 坐标系约定与实际不符 | 检查 `_do_yaw_align` 符号，或确认相机安装朝向 |
| phase_1 超时退出 | Tag 不在视野内 / 检测不稳定 | 看超时诊断统计；静态脚本（14.6）验证；调大 `detect_timeout_s` |
| 报"运动链路未就绪"或"指令可能被忽略" | pose_control 未启动 / `/leg_odom2` 无数据 | 用 launch 一键启动；`ros2 topic echo /leg_odom2`；确认狗已完成 §15.0 前置步骤 |
| 狗不动但节点在发 /move | pose_control 到位阈值 ≥ 上层阈值 | 确认用 launch 启动（已带调好的 `dist_threshold=0.015` 等参数） |
| 对齐震荡不收敛 | 阈值过紧或步长过大 | 放宽阈值、减小 `max_yaw_step_deg` |
| grasp 未收到信号 | `/grasp/start` topic 类型不一致 | `ros2 topic info /grasp/start` 应为 `std_msgs/msg/Bool` |

### 15.5 修改参数不重编译

只改 yaml 参数时不需要重新 colcon build，重启节点即可生效（launch 与 `--params-file` 方式每次启动都重新读 yaml）。修改 `.py` 源码后需重新编译：

```bash
colcon build --packages-select apriltag_place1 && source install/setup.bash
```

---

## 16. 真机验证记录（2026-07-28，Lite3）

- 全流程跑通：`wait_trigger → wait_detect → yaw_align → lateral_align → approach → final_check → final_forward → 发布 /grasp/start`，grasp 端（旧版 `main.py --mode robot`）成功收到信号。
- 当时配置为 `target_distance_m=0.20`、`final_forward_offset_m=0.15`，后调整为现在的 `0.08 / 0.20`（视觉闭环贴得更近，再开环送进抓取范围）。
- 典型收敛过程：yaw 首轮 alpha=5.09° → 近距离段出现 -17.37° 偏差被重新修正 → final_check 时 alpha=0.54°、tx=0.002 m；approach 经 3~4 轮小步逼近到位。**近距离段 final_check 反复打回 yaw_align 修正是正常行为**。
- 已知现象：`/dev/video6` 首次打开曾失败（`can't be used to capture by name`），重启节点后成功——设备号/就绪时序问题，节点已有每帧重开摄像头的容错。
