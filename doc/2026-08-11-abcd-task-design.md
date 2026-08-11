# ABCD 抓取放置全流程设计文档

> 日期：2026-08-11
> 负责人：胡峻豪
> 状态：待实现

---

## 1. 背景与目标

本模块实现比赛中 ABCD 四块物块的完整抓取–放置任务。机械狗从中转点出发，依次完成 A→B→C→D 的抓取和放置，最终返回中转点。

**目标约束：**
- 最大化复用现有代码（`grasp_flow_b`、`block_align`、`apriltag_place1`、`way_point.py`）
- 每个子流程可单独拿出来测试
- 路径统一使用 `/home/ysc/2026YuYaoGuoSai/...`（机械狗上路径）
- 解决夹爪堵转问题

---

## 2. 系统架构

### 2.1 分层结构

```
abcd_task_node                      ← 新建：ABCD 四轮顶层编排
  ├── waypoint_nav.py               ← 新建：封装 world_to_body + /move 发布
  ├── apriltag_place1_node          ← 复用：检测 Tag，调整朝向
  ├── block_align_node（改造）      ← 改造：新增横向搜索状态
  ├── grasp_task_node               ← 复用
  └── grasp_flow_node_b             ← 复用：单次抓+放子流程编排
```

### 2.2 ROS 话题接口总览

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/leg_odom2` | `nav_msgs/Odometry` | sub | 里程计，用于导航 |
| `/move` | `geometry_msgs/Pose2D` | pub | 发给 pose_controller |
| `/pose_control/command` | `std_msgs/String` | pub | reset_origin 等指令 |
| `/cmd_vel` | `geometry_msgs/Twist` | sub | 判断运动是否停止 |
| `/apriltag_place1/start` | `std_msgs/Bool` | pub | 触发 Tag 对齐 |
| `/apriltag_place1/done` | `std_msgs/Bool` | sub | Tag 对齐完成信号 |
| `/block_align/start` | `std_msgs/Bool` | pub | 触发色块对齐+搜索 |
| `/grasp_flow/place_ready` | `std_msgs/Bool` | pub | 触发放置 |
| `/grasp/result` | `std_msgs/Bool` | sub | 抓取/放置结果 |
| `/grasp/state` | `std_msgs/String` | sub | grasp_task 当前状态 |

---

## 3. 文件结构

```
lite3_ws/src/grasp/
└── abcd_task/
    ├── abcd_task/
    │   ├── __init__.py
    │   ├── abcd_task_node.py        # 顶层编排节点
    │   └── waypoint_nav.py          # 导航工具类
    ├── config/
    │   ├── abcd_task.yaml           # 任务运行参数（超时等）
    │   └── abcd_config.yaml         # ABCD 颜色/tag/坐标配置
    ├── launch/
    │   └── abcd_task.launch.py      # 一键启动全链路
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── resource/
        └── abcd_task
```

**改造现有文件：**
- `block_align/block_align/block_align_node.py`：新增 `STATE_SEARCH` 横向搜索状态
- `grasp_task/grasp_task/grasp_node.py`：修复 `finalize()` 中夹爪释放时序
- `tools/grasp/config.yaml`：新增 `gripper_release_torque: 0`

---

## 4. 配置文件设计

### 4.1 `abcd_config.yaml`

```yaml
# ABCD 物块任务配置
# 绿色 = 正常，红色 = 异常
# tag_id 和 task_point 需现场标定后填入

blocks:
  A: {color: red,   tag_id: 0, task_point: {x: 0.0, y: 0.0, yaw: 0.0}}
  B: {color: red,   tag_id: 1, task_point: {x: 0.0, y: 0.0, yaw: 0.0}}
  C: {color: green, tag_id: 2, task_point: {x: 0.0, y: 0.0, yaw: 0.0}}
  D: {color: green, tag_id: 3, task_point: {x: 0.0, y: 0.0, yaw: 0.0}}

# 中转点（里程计绝对坐标，标定后填入）
transit_point: {x: 0.0, y: 0.0, yaw: 0.0}
```

**说明：**
- `task_point` 抓取准备点 = 放置准备点，同一坐标，到位后动作不同
- 后续根据实际检测结果动态修改 `color` 字段即可，无需改代码

### 4.2 `abcd_task.yaml`

```yaml
abcd_task_node:
  ros__parameters:
    abcd_config_path: "/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/abcd_task/config/abcd_config.yaml"
    task_order: ["A", "B", "C", "D"]     # 执行顺序，可调整
    start_from: "A"                       # 支持从任意字母开始，便于单独测试
    retreat_dist_m: 0.5                   # 抓取后后退距离
    nav_timeout_s: 30.0                   # 单段导航超时
    tag_align_timeout_s: 20.0            # Tag 对齐超时
    grasp_flow_timeout_s: 600.0          # 单次抓取全流程超时
    place_timeout_s: 120.0               # 单次放置超时
    odom_fresh_timeout_s: 1.0
```

---

## 5. `abcd_task_node` 状态机

### 5.1 单轮状态序列

每个字母执行以下序列（以 A 为例）：

```
IDLE
  │  收到启动信号或自动开始
  ▼
NAV_TO_TASK_POINT          从中转点导航到 A 的 task_point
  │  /move 到位（cmd_vel 归零）
  ▼
TAG_ALIGN                  拉起 apriltag_place1，识别 tag_id=0
  │                        调整旋转，垂直面向 Tag
  │  /grasp/state 进入 DETECTING（apriltag_place1 发 /grasp/start 后）
  ▼
BLOCK_SEARCH_AND_GRASP     触发改造后的 block_align（含横向搜索）
  │                        block_align 找到色块后触发 grasp_flow_b
  │  /grasp/state = TRANSPORT
  ▼
RETREAT                    发 /move (x=-0.5) 后退 0.5m
  │  cmd_vel 归零
  ▼
NAV_TO_TRANSIT             导航回中转点
  │  到位
  ▼
NAV_TO_TASK_POINT          再次导航到同一 task_point（放置准备）
  │  到位
  ▼
PLACING                    向 /grasp_flow/place_ready 发 True
  │                        等待 /grasp/result = True
  │  放置完成
  ▼
NAV_BACK_TO_TRANSIT        返回中转点
  │  到位
  ▼
NEXT_LETTER（或 ALL_DONE）
```

### 5.2 全局状态机

```
ALL_IDLE → [对每个字母执行单轮序列] → ALL_DONE
                │
                └─ 任意步骤失败 → ERROR（记录失败字母，可选跳过继续）
```

### 5.3 可测试性

节点参数 `start_from: "C"` 可以从 C 开始跳过 AB，方便单独测试某一轮。  
每个子状态对应独立触发，也可手动发话题触发单步：

```bash
# 单独测试导航
ros2 run abcd_task abcd_task_node --ros-args -p start_from:=A

# 单独测试 block_align 搜索
ros2 launch block_align block_align.launch.py
ros2 topic pub /block_align/start std_msgs/Bool "data: true" --once

# 单独测试 grasp_flow_b
ros2 launch grasp_flow grasp_flow_b.launch.py
```

---

## 6. `waypoint_nav.py` 设计

封装 `way_point.py` 中的导航逻辑为可复用类，供 `abcd_task_node` 调用。

```python
class WaypointNav:
    """封装单段 /move 导航：从当前里程计位置导航到目标绝对坐标。"""

    def navigate_to(self, target: dict) -> bool:
        """
        target: {"x": float, "y": float, "yaw": float}（绝对里程计坐标）
        返回 True=到位，False=超时
        复用 way_point.py 的 world_to_body + compute_moves 逻辑
        """
```

核心逻辑直接 `from tools.way_point import world_to_body, compute_moves` 复用，不重复实现。

---

## 7. `block_align_node` 改造：横向搜索

### 7.1 新增状态

在 `wait_detect` 超时前，新增 `STATE_SEARCH = "search"` 状态：

```
wait_trigger → wait_detect → [检测超时] → search → [发现目标] → lateral_align
                            → [直接检测到] → lateral_align
```

### 7.2 搜索逻辑

- 搜索方向：从右往左（`y` 正方向，每步 `search_step_m`，默认 0.1m）
- 每步发 `/move`，等 `cmd_vel` 归零后再检测一帧
- 检测到目标色块后立即进入 `lateral_align`
- 超过 `search_max_steps`（默认 10 步，即 1m）后报 ERROR

### 7.3 新增配置参数（`block_align.yaml`）

```yaml
search_step_m: 0.1        # 每步横移距离
search_max_steps: 10      # 最大搜索步数
search_timeout_s: 60.0    # 搜索总超时
```

---

## 8. 夹爪堵转修复

### 8.1 根本原因

`ros2 launch` Ctrl+C 时所有节点同时收 SIGINT，`grasp_task` 的 `finalize()` 依赖 `rclpy.spin` 退出后才执行，时序不保证，导致 `open_gripper()` 可能未走到。

### 8.2 修复方案

**修改 `grasp_node.py`：**

1. `finalize()` 最开头立即调用 `open_gripper()`，在 `signal.SIG_IGN` 保护窗口内执行，超时 2s 强制继续，不阻断后续流程
2. 放置完成后（`/grasp/result` 发布前）将夹爪力矩降至 `gripper_release_torque: 0`，避免长时间持续施力
3. 抓取失败重试前必须先 `open_gripper()`，再重新闭合

**修改 `config.yaml`：**
```yaml
arm:
  gripper_release_torque: 0    # 放置/失败后立即归零力矩
  gripper_open_timeout_s: 2.0  # open_gripper 最长等待
```

**修改 `grasp_flow_node_b.py`：**

`destroy_node()` 里在 `_kill_all()` 前先向 `/grasp_flow/gripper_release` 发布一次 Bool(True)，由 `grasp_task` 订阅后立即执行 `open_gripper()`，解耦两个节点的退出时序。

---

## 9. launch 文件设计

`abcd_task.launch.py` 启动以下节点：

| 节点 | 包 | 说明 |
|------|----|------|
| `lite3_driver` | 进程 | 机械狗驱动（可选） |
| `pose_controller` | `pose_control` | 运动闭环控制 |
| `grasp_task` | `grasp_task` | 机械臂控制 |
| `grasp_flow_node_b` | `grasp_flow` | 单次抓+放编排 |
| `apriltag_place1_node` | `apriltag_place1` | Tag 检测对齐 |
| `abcd_task_node` | `abcd_task` | 顶层 ABCD 编排 |

`block_align_node` 由 `grasp_flow_node_b` 按需拉起，不在 launch 里预启动。

---

## 10. 标定清单

现场上机前需标定以下参数并填入配置：

| 参数 | 配置文件 | 说明 |
|------|---------|------|
| `transit_point` | `abcd_config.yaml` | 中转点绝对里程计坐标 |
| `task_point` (A/B/C/D) | `abcd_config.yaml` | 各区任务点坐标 |
| `tag_id` (A/B/C/D) | `abcd_config.yaml` | 各区 AprilTag ID |
| `color` (A/B/C/D) | `abcd_config.yaml` | 实际颜色（绿=正常/红=异常） |
| `placement.zones` | `tools/grasp/config.yaml` | 各区放置机械臂参数 |
| `lateral_polarity` | `block_align.yaml` | 横移方向极性 |

---

## 11. 测试方案

每个子模块可单独测试：

```bash
# 1. 测试单段导航
ros2 run abcd_task abcd_task_node --ros-args -p start_from:=A -p nav_only:=true

# 2. 测试 block_align 横向搜索
ros2 launch block_align block_align.launch.py
ros2 topic pub /block_align/start std_msgs/Bool "data: true" --once

# 3. 测试单次抓取全流程
ros2 launch grasp_flow grasp_flow_b.launch.py

# 4. 测试完整 ABCD 流程（从 C 开始）
ros2 launch abcd_task abcd_task.launch.py start_from:=C

# 5. 测试夹爪复位
# Ctrl+C 后检查夹爪是否自动松开
```

---

## 12. 依赖说明

- 所有路径以 `/home/ysc/2026YuYaoGuoSai/...` 为准（机械狗上路径）
- PC 开发阶段路径为 `/home/fishros/2026YuYaoGuoSai/...`，launch 文件统一用 ysc 路径
- Python 依赖：`pupil-apriltags`、`opencv-python`、`pyyaml`、`rclpy`
