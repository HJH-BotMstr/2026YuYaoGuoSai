# grasp 模块 ROS2 化迁移方案

**日期**：2026-07-26  
**状态**：设计稿，待实现  
**目标**：在不修改、不删除 `tools/grasp` 现有代码的前提下，把当前抓取流程封装成一个 ROS2 节点/包，放到 `lite3_ws/src` 中运行，使其能与已有的运动控制（`pose_control` / `lite3_driver`）、视觉定位（`apriltag_place1`，设计中）以及巡检/表计识别（`gauge_detector` / `gauge_yolo_detector`）模块协同工作。

> 说明：仓库中实际工作空间为 `lite3_ws`，下文均按 `lite3_ws/src` 书写。如果你另有一个 `lite_ws`，只需把路径里的 `lite3_ws` 替换即可。

---

## 1. 现状梳理

### 1.1 `tools/grasp` 现状

| 文件 | 职责 | 是否复用 |
|---|---|---|
| [tools/grasp/main.py](tools/grasp/main.py) | 8-phase 抓取流程：初始化 → 待命 → 识别 → 横向对齐 → 接近抓取 → 运输 → 放置 → 归位 | **流程骨架复用，ROS2 节点重写** |
| [tools/grasp/config.yaml](tools/grasp/config.yaml) | 机械臂串口、摄像头、内参、HSV、抓取/放置参数 | **直接加载复用** |
| [tools/grasp/utils/ArmController.py](tools/grasp/utils/ArmController.py) | 6 路舵机控制、逆运动学、夹爪、到位判断 | **复用** |
| [tools/grasp/utils/BlockDetection.py](tools/grasp/utils/BlockDetection.py) | 机械臂摄像头色块检测（红/绿）、针孔测距、3D 位置 | **复用** |
| [tools/grasp/utils/TargetTracker.py](tools/grasp/utils/TargetTracker.py) | 多帧锁定 + 滑动均值稳定 | **复用** |
| [tools/grasp/utils/InspectionMemory.py](tools/grasp/utils/InspectionMemory.py) | 放置区 A/B/C/D 记忆 | **复用或替代** |
| [tools/grasp/utils/DogAlignInterface.py](tools/grasp/utils/DogAlignInterface.py) | 横向对齐接口（当前为 stub） | **不调用，改由节点直接发 `/move`** |
| [tools/grasp/utils/RobotSignalInterface.py](tools/grasp/utils/RobotSignalInterface.py) | 启动/放置信号接口（当前为 stub） | **不调用，改由节点直接订阅 topic** |

当前 `tools/grasp` **不是 ROS2 包**，没有 `package.xml/setup.py`，也不发布/订阅任何 ROS topic。它通过串口直接驱动机械臂，通过 V4L2 直接读取机械臂摄像头。

### 1.2 现有 ROS2 模块接口

#### 运动控制：`pose_control`

- 节点：`pose_controller`（[lite3_ws/src/pose_control/pose_control/pose_controller_node.py](lite3_ws/src/pose_control/pose_control/pose_controller_node.py)）
- 订阅：
  - `/leg_odom2` (`nav_msgs/Odometry`)
  - `/move` (`geometry_msgs/Pose2D`)：机身坐标系相对位移，`x` 前进，`y` 左移，`theta` 逆时针旋转角度（度）
  - `/pose_control/command` (`std_msgs/String`)：`cancel` / `reset_origin` / `pause` / `resume` / `quit`
  - `/emergency_stop` (`std_msgs/Bool`)
- 发布：`/cmd_vel` (`geometry_msgs/Twist`)，`/cmd_gait` (`std_msgs/String`)

#### 底层驱动：`tools/lite3_driver.py`

- 订阅 `/cmd_vel`、`/cmd_gait` 等
- 发布 `/leg_odom2`、`/driver_status`
- 通过 UDP 与机器狗通信

#### AprilTag 到位导航（设计中）

见 [tools/grasp/2026-07-26-apriltag-place1-design.md](tools/grasp/2026-07-26-apriltag-place1-design.md)。其核心输出是：

- 发布 `/grasp/start`（`std_msgs/Bool`）——机器狗到达 place1 后触发抓取
- 复用 `/move`、`/pose_control/command` 完成航向/横向/距离对准

#### 表计/巡检识别

- `gauge_detector` 提供 `/detect_gauge` 服务
- `gauge_yolo_detector` 提供 `/detect_gauge_yolo` 服务
- 返回 `letter`、`zone`、`state` 等字段，可用于决定放置区 A/B/C/D

---

## 2. 设计原则

1. **`tools/grasp` 零改动、零删除**。所有 ROS2 适配逻辑写在新包里。
2. **最大复用现有代码**：机械臂控制、色块检测、目标跟踪、配置文件、逆运动学直接 import。
3. **接口清晰、模块解耦**：grasp 节点只负责“抓取+放置”子任务；到位导航、放置区决策由外部节点负责。
4. **与现有 `pose_control` 接口保持一致**：横向对齐仍走 `/move` + `/pose_control/command`，不另写运动控制器。
5. **渐进式迁移**：先让 ROS2 节点把 main.py 的 8-phase 跑通，后续再考虑把摄像头拆成独立节点。

---

## 3. 整体架构

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                         上层任务调度 / 人工程序                          │
│  (可手动发布 /grasp/start、/grasp/place，或后续由 mission_planner 统一)   │
└─────────────────────────────────────────────────────────────────────────┘
       │ start (Bool)              │ place (String, "A/B/C/D")
       ▼                           ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         grasp_task_node（新增）                          │
│  - 加载 tools/grasp/config.yaml                                          │
│  - 复用 ArmController / BlockDetection / TargetTracker / InspectionMemory │
│  - 直接打开 /dev/video0 读取机械臂摄像头（方案 A）                        │
│  - 发布 /move、/pose_control/command、/grasp/state                        │
│  - 订阅 /grasp/start、/grasp/place、/cmd_vel、/leg_odom2                  │
└─────────────────────────────────────────────────────────────────────────┘
       │ /move (Pose2D)        │ reset_origin
       ▼                       ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         pose_control 节点                                │
│                         （已有，不改动）                                  │
└─────────────────────────────────────────────────────────────────────────┘
       │ /cmd_vel
       ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      tools/lite3_driver.py                               │
│                      （已有 UDP 驱动，不改动）                            │
└─────────────────────────────────────────────────────────────────────────┘
       │ UDP
       ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            绝影 Lite3                                    │
└─────────────────────────────────────────────────────────────────────────┘

其他视觉模块：
  apriltag_place1_node ──► /grasp/start
  inspection_bridge （可选）──► 调用 /detect_gauge ──► /grasp/set_zone
```

---

## 4. 新增 ROS2 包：`grasp_task`

建议在 `lite3_ws/src` 下新建包 `grasp_task`（不要叫 `grasp`，避免与 `tools/grasp` 在 `PYTHONPATH` 中冲突）。

### 4.1 目录结构

```text
lite3_ws/src/grasp/grasp_task/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/grasp_task
├── grasp_task/
│   ├── __init__.py
│   ├── grasp_node.py          # ROS2 主节点
│   ├── config_loader.py       # 加载并合并 tools/grasp/config.yaml
│   └── motion_waiter.py       # 通过 /cmd_vel 判断 pose_control 是否到位
├── config/
│   └── grasp_task.yaml        # ROS2 参数
└── launch/
    └── grasp.launch.py
```

### 4.2 依赖

`package.xml` 关键依赖：

```xml
<buildtool_depend>ament_python</buildtool_depend>

<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>

<exec_depend>python3-yaml</exec_depend>
<exec_depend>python3-opencv</exec_depend>
<exec_depend>python3-serial</exec_depend>

<export>
  <build_type>ament_python</build_type>
</export>
```

> `cv_bridge` 不是必须的，因为本方案先保留直接 V4L2 取图。

---

## 5. 模块复用方式

### 5.1 把 `tools/grasp` 加入 Python 搜索路径

在 [grasp_node.py](lite3_ws/src/grasp/grasp_task/grasp_task/grasp_node.py) 顶部：

```python
import sys
TOOLS_GRASP = "/home/ysc/2026YuYaoGuoSai/tools/grasp"
if TOOLS_GRASP not in sys.path:
    sys.path.insert(0, TOOLS_GRASP)

from utils.ArmController import ArmController
from utils.BlockDetection import BlockDetection
from utils.TargetTracker import TargetTracker
from utils.InspectionMemory import InspectionMemory
```

- 这样机械臂、视觉、跟踪类完全复用，无需复制代码。
- `tools/grasp/main.py` 中的辅助函数 `_open_camera` 也可以直接拷到 `grasp_node.py` 中使用（它是一个独立函数，不引入新依赖）。

### 5.2 替换两个 stub 接口

当前 [DogAlignInterface.py](tools/grasp/utils/DogAlignInterface.py) 和 [RobotSignalInterface.py](tools/grasp/utils/RobotSignalInterface.py) 是占位实现。ROS2 节点不再调用它们，而是直接在节点内实现等价的 ROS 行为：

| 原接口行为 | ROS2 替代方式 |
|---|---|
| `RobotSignalInterface.wait_start()` | 订阅 `/grasp/start`，收到 `data=True` 后 `threading.Event.set()` |
| `RobotSignalInterface.wait_place(zone)` | 订阅 `/grasp/place`，收到对应 zone 字符串后 `Event.set()` |
| `DogAlignInterface.send_align(offset_x_mm)` | 发布 `/move`：`Pose2D(x=0.0, y=offset_x_mm/1000.0, theta=0.0)` |
| `DogAlignInterface.wait_aligned()` | 订阅 `/cmd_vel`，连续一段时间接近零即认为到位 |

### 5.3 复用 `config.yaml`

`config_loader.py` 负责：

1. 从 ROS 参数 `tools_config_path` 读取 `tools/grasp/config.yaml` 路径。
2. 用 `yaml.safe_load` 加载成 `dict`。
3. 把 `runtime.mode` 强制覆盖为 `"robot"`。
4. 用 ROS 参数覆盖部分配置（如摄像头设备号、放置区参数），方便调参。

```python
def load_config(node: Node) -> dict:
    tools_path = node.get_parameter("tools_config_path").value
    with open(tools_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["runtime"]["mode"] = "robot"
    # 可选：用 ROS 参数覆盖
    return cfg
```

> 这样 `tools/grasp/config.yaml` 继续作为唯一权威配置来源；ROS2 参数只负责“在哪里找到它”以及少量运行时覆盖。

---

## 6. 对外接口定义

所有接口尽量复用标准消息，不新增自定义消息包。

| Topic / Service | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/grasp/start` | `std_msgs/Bool` | **sub** | 启动抓取流程。`data=True` 时从 phase_1 继续。由 `apriltag_place1_node` 或任务调度发布。 |
| `/grasp/place` | `std_msgs/String` | **sub** | 到达 place2 并告知放置区，如 `"A"`。节点进入 phase_6。 |
| `/grasp/set_zone` | `std_msgs/String` | **sub** | （可选）仅注入放置区，不触发 phase_6；可让 inspection_bridge 提前设置。 |
| `/grasp/state` | `std_msgs/String` | **pub** | 当前状态：`INIT`、`STANDBY`、`DETECTING`、`ALIGNING`、`GRASPING`、`TRANSPORT`、`PLACING`、`DONE`、`ERROR`。 |
| `/grasp/result` | `std_msgs/Bool` | **pub** | 流程最终完成时 `data=True`；失败时 `data=False`（并切到 `ERROR`）。 |
| `/move` | `geometry_msgs/Pose2D` | **pub** | 横向对齐：`y = X_cam / 1000.0`（m），`x = 0`，`theta = 0`。 |
| `/pose_control/command` | `std_msgs/String` | **pub** | 在合适时机发 `reset_origin` 重置里程计原点。 |
| `/cmd_vel` | `geometry_msgs/Twist` | **sub** | 判断 pose_control 是否执行完毕。 |
| `/leg_odom2` | `nav_msgs/Odometry` | **sub** | （可选）判断里程计是否新鲜。 |
| `/emergency_stop` | `std_msgs/Bool` | **sub** | （可选）收到 True 时立即停止机械臂并进入 ERROR。 |

---

## 7. 节点内部设计

### 7.1 执行模型

- 使用 `SingleThreadedExecutor` 在后台线程运行 ROS 回调。
- 主线程沿用 [main.py](tools/grasp/main.py) 的 8-phase 顺序流程，但通过 `threading.Event` 等待外部信号。
- 所有共享状态（最新 `/cmd_vel`、目标区、start/place 事件）加锁保护。

```python
executor = SingleThreadedExecutor()
executor.add_node(node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

try:
    node.run_state_machine()   # 主线程顺序跑 8-phase
finally:
    executor.shutdown()
```

### 7.2 状态机

与 [main.py](tools/grasp/main.py) 保持一致：

```text
INIT ──► STANDBY ──► DETECTING ──► ALIGNING ──► GRASPING
  │        ▲            │             │            │
  │        │            ▼             ▼            ▼
  │     等待        超时/失败可重试   发布 /move   调用 ArmController
  │   /grasp/start
  │
  └──► TRANSPORT ──► PLACING ──► DONE
            │           ▲
            │     等待 /grasp/place
            ▼
        调用 set_pose(3)
```

### 7.3 类骨架

```python
class GraspTaskNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("grasp_task")
        self.cfg = cfg
        self._lock = threading.Lock()

        # 事件
        self._start_event = threading.Event()
        self._place_event = threading.Event()

        # 最新外部输入
        self._target_zone: Optional[str] = None
        self._cmd_vel_zero_since: Optional[float] = None
        self._last_cmd_vel_time = 0.0

        # 订阅/发布
        self.create_subscription(Bool,   "start_topic",   self._on_start, 10)
        self.create_subscription(String, "place_topic",   self._on_place, 10)
        self.create_subscription(String, "set_zone_topic", self._on_zone, 10)
        self.create_subscription(Twist,  "cmd_vel_topic", self._on_cmd_vel, 10)

        self._state_pub = self.create_publisher(String, "state_topic", 10)
        self._result_pub = self.create_publisher(Bool, "result_topic", 10)
        self._move_pub = self.create_publisher(Pose2D, "move_topic", 10)
        self._cmd_pub = self.create_publisher(String, "command_topic", 10)

        # 初始化硬件
        self.arm = ArmController(
            device=cfg["hardware"]["arm_serial_port"],
            cfg={**cfg["arm"], "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]},
        )
        self.detector = BlockDetection({**cfg["detection"]})
        self.tracker = TargetTracker(
            avg_window=int(cfg["grasp"]["distance_avg_window"]),
            lost_frames_max=int(cfg["grasp"]["lost_frames_max"]),
        )
        self.memory = InspectionMemory(default_zone=cfg["inspection"]["default_zone"])
        self.arm_cam = self._open_camera(cfg["hardware"]["arm_cam_device"])

    # ---------- ROS 回调 ----------
    def _on_start(self, msg: Bool):
        if msg.data:
            self._start_event.set()

    def _on_place(self, msg: String):
        zone = msg.data.upper()
        if zone in {"A", "B", "C", "D"}:
            self.memory.set_zone(zone)
            self._place_event.set()

    def _on_zone(self, msg: String):
        self.memory.set_zone(msg.data.upper())

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            if abs(msg.linear.x) < 0.01 and abs(msg.linear.y) < 0.01 and abs(msg.angular.z) < 0.01:
                if self._cmd_vel_zero_since is None:
                    self._cmd_vel_zero_since = time.monotonic()
            else:
                self._cmd_vel_zero_since = None
            self._last_cmd_vel_time = time.monotonic()

    def _publish_state(self, state: str):
        self._state_pub.publish(String(data=state))
        self.get_logger().info("state -> %s", state)

    # ---------- 运动等待 ----------
    def _wait_motion_stop(self, timeout: float, zero_duration: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._cmd_vel_zero_since is not None:
                    if time.monotonic() - self._cmd_vel_zero_since >= zero_duration:
                        return True
            time.sleep(0.05)
        return False

    # ---------- 8-phase 主流程 ----------
    def run_state_machine(self):
        self._publish_state("INIT")
        # phase_1: standby
        self.arm.set_pose(0)
        self.arm.set_pose(2)
        self._publish_state("STANDBY")
        self._start_event.wait()

        # phase_2: detect
        self._publish_state("DETECTING")
        stable = self._detect_stable()
        if stable is None:
            return self._fail("DETECT_TIMEOUT")

        # phase_3: align (publish /move and wait)
        self._publish_state("ALIGNING")
        if not self._align_laterally(stable):
            return self._fail("ALIGN_FAILED")

        # phase_4: approach & grasp
        self._publish_state("GRASPING")
        if not self._approach_and_grasp(stable):
            return self._fail("GRASP_FAILED")

        # phase_5: transport
        self._publish_state("TRANSPORT")
        self.arm.set_pose(3, keep_gripper=True)

        # phase_6: place
        self._publish_state("PLACING")
        self._place_event.wait()
        if not self._place():
            return self._fail("PLACE_FAILED")

        # phase_7: home
        self._publish_state("DONE")
        self.arm.set_pose(0)
        self._result_pub.publish(Bool(data=True))

    def _fail(self, reason: str):
        self.get_logger().error("任务失败: %s", reason)
        self._publish_state(f"ERROR:{reason}")
        self._result_pub.publish(Bool(data=False))
        return False

    # ... _detect_stable / _align_laterally / _approach_and_grasp / _place
    # 直接复用 main.py 中对应 phase 的核心算法
```

### 7.4 横向对齐逻辑

```python
def _align_laterally(self, stable: dict) -> bool:
    cfg_g = self.cfg["grasp"]
    thr_m = float(cfg_g["align_offset_threshold_mm"]) / 1000.0
    max_rounds = 5

    X_cam_mm = stable["pos_3d"][0]
    for _ in range(max_rounds):
        if abs(X_cam_mm / 1000.0) <= thr_m:
            return True

        # 重置 tracker，避免旧窗口影响
        self.tracker = TargetTracker(
            avg_window=int(cfg_g["distance_avg_window"]),
            lost_frames_max=int(cfg_g["lost_frames_max"]),
        )

        # 发布横向移动：y 正方向为左移
        msg = Pose2D(x=0.0, y=X_cam_mm / 1000.0, theta=0.0)
        self._move_pub.publish(msg)
        self.get_logger().info("横向对齐: y=%.3fm", msg.y)

        # 等待 pose_control 执行到位
        if not self._wait_motion_stop(timeout=15.0, zero_duration=0.5):
            return False

        # 重新检测
        new_stable = self._detect_stable()
        if new_stable is None:
            return False
        X_cam_mm = new_stable["pos_3d"][0]
    return False
```

> `pose_control` 的 `/move` 语义中 `+y` 是左移。当前 [BlockDetection.py](tools/grasp/utils/BlockDetection.py) 中 `+X` 是右移，因此 `X_cam_mm > 0` 时发布 `+y` 会让狗向左移动，正好补偿。

### 7.5 接近与抓取逻辑

直接复用 [main.py:235-278](tools/grasp/main.py#L235-L278) 的两步策略：

```python
def _approach_and_grasp(self, stable: dict) -> bool:
    cfg_g = self.cfg["grasp"]
    X_cam, Y_cam, Z_cam = stable["pos_3d"]
    dis_target = Y_cam + float(cfg_g.get("distance_offset_mm", 0.0))
    dis_safe = max(dis_target - float(cfg_g["approach_clearance_mm"]), 30.0)
    h_object = float(cfg_g["h_object"])

    # 步骤1：安全距离 + 目标高度
    if not self.arm.grap(dis_safe, h_object):
        return False
    # 等待到位 ...

    # 步骤2：前进并抓取 + 位置校验
    return self.arm.grasp_with_verify(dis=dis_target, height=h_object)
```

---

## 8. 配置与启动文件

### 8.1 `config/grasp_task.yaml`

```yaml
grasp_task:
  # 权威配置来源：保持不变，继续由 tools/grasp/config.yaml 维护
  tools_config_path: "/home/ysc/2026YuYaoGuoSai/tools/grasp/config.yaml"

  # topic 名，保持与现有模块一致
  start_topic: "/grasp/start"
  place_topic: "/grasp/place"
  set_zone_topic: "/grasp/set_zone"
  state_topic: "/grasp/state"
  result_topic: "/grasp/result"
  move_topic: "/move"
  command_topic: "/pose_control/command"
  cmd_vel_topic: "/cmd_vel"
  odom_topic: "/leg_odom2"

  # 运行时行为
  cv_show: false                       # 是否 cv2.imshow 调试图像
  motion_stop_timeout_s: 15.0          # 等 pose_control 执行到位的最大时间
  motion_stop_zero_duration_s: 0.5     # /cmd_vel 持续接近零多少秒算到位
  odom_fresh_timeout_s: 0.5            # 里程计超时判定
```

### 8.2 `launch/grasp.launch.py`

```python
import os
from glob import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("grasp_task")
    params = os.path.join(pkg_share, "config", "grasp_task.yaml")

    return LaunchDescription([
        Node(
            package="grasp_task",
            executable="grasp_node",
            name="grasp_task",
            output="screen",
            parameters=[params],
        ),
    ])
```

### 8.3 `setup.py`

```python
from setuptools import setup
from glob import glob
import os

package_name = "grasp_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email",
    description="ROS2 wrapper for tools/grasp",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_node = grasp_task.grasp_node:main",
        ],
    },
)
```

---

## 9. 与其他模块的协作细节

### 9.1 与 `apriltag_place1` 的衔接

根据 [tools/grasp/2026-07-26-apriltag-place1-design.md](tools/grasp/2026-07-26-apriltag-place1-design.md)：

1. `apriltag_place1_node` 完成“航向 → 横向 → 前进”对准，停在 place1。
2. 对准完成后，它发布 `/grasp/start`。
3. `grasp_task_node` 订阅 `/grasp/start`，进入 phase_2 开始识别。

> **注意**：`apriltag_place1_node` 和 `grasp_task_node` 都会发布 `/move`。为避免冲突，二者**不应同时处于运动控制状态**。推荐由上层任务节点（或 launch 顺序）保证：apriltag 运行期间 grasp 节点处于 `STANDBY`；收到 start 后 apriltag 节点不再发 `/move`。

### 9.2 与 `pose_control` 的衔接

- `grasp_task_node` 发的 `/move` 与 apriltag_place1 使用同一接口，无需修改 [pose_controller_node.py](lite3_ws/src/pose_control/pose_control/pose_controller_node.py)。
- 在 phase_3 每次发 `/move` 前，可以发 `reset_origin` 把当前位置设为零点，避免多次横向调整后里程计漂移叠加。
- 通过 `/cmd_vel` 判断到位，不新增 action/service。

### 9.3 与巡检/表计模块的衔接

方案 A（推荐）：由单独的 `inspection_bridge` 节点负责：

```text
mission_planner/inspection_bridge
    ├── 调用 /detect_gauge 或 /detect_gauge_yolo
    └── 发布 /grasp/set_zone (String, e.g. "B")
```

`grasp_task_node` 只订阅 `/grasp/set_zone` 或 `/grasp/place`：

- `/grasp/set_zone`：仅更新 `InspectionMemory` 中的 zone。
- `/grasp/place`：同时更新 zone 并触发 phase_6 放置。

这样 grasp 节点不需要知道 gauge 服务的细节，保持解耦。

---

## 10. 文件变更清单

| 文件/目录 | 变更类型 | 说明 |
|---|---|---|
| `lite3_ws/src/grasp/grasp_task/` | 新增 | 完整 ROS2 包 |
| `lite3_ws/src/grasp/grasp_task/grasp_task/grasp_node.py` | 新增 | ROS2 抓取任务节点 |
| `lite3_ws/src/grasp/grasp_task/grasp_task/config_loader.py` | 新增 | 加载 tools/grasp/config.yaml |
| `lite3_ws/src/grasp/grasp_task/grasp_task/motion_waiter.py` | 新增 | 基于 /cmd_vel 的到位判断 |
| `lite3_ws/src/grasp/grasp_task/config/grasp_task.yaml` | 新增 | ROS2 参数 |
| `lite3_ws/src/grasp/grasp_task/launch/grasp.launch.py` | 新增 | 启动文件 |
| `lite3_ws/src/grasp/grasp_task/package.xml` | 新增 | 包元数据 |
| `lite3_ws/src/grasp/grasp_task/setup.py` | 新增 | ament_python 入口 |
| `tools/grasp/` | **不改动** | 零修改、零删除，仅被 import |
| `lite3_ws/src/pose_control/` | **不改动** | 复用现有 `/move` 接口 |

---

## 11. 编译与运行

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select grasp_task
source install/setup.bash

# 1. 启动 pose_control（带底层驱动）
ros2 run pose_control pose_control
# 或在另一终端启动 driver
# python3 /home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py

# 2. 启动 grasp_task
ros2 launch grasp_task grasp.launch.py

# 3. 手动触发（若 apriltag_place1 未实现）
ros2 topic pub /grasp/start std_msgs/Bool "{data: true}" --once
# 到达 place2 后
ros2 topic pub /grasp/place std_msgs/String "{data: 'B'}" --once
```

---

## 12. 测试建议

1. **PC 仿真测试（不插机械臂/不插狗）**
   - 保持 `tools/grasp/config.yaml` 的 `mode: pc` 不变。
   - 启动 `grasp_task_node`。
   - 手动 pub `/grasp/start`，观察 `/grasp/state` 是否按预期推进。
   - 机械臂串口未连接时 `ArmController` 会报错，这是正常的；可先用 `try/except` 包装初始化，或在 PC 测试时临时注释硬件初始化。

2. **运动控制分段测试**
   - 不接机械臂，只测 `/move` 发布和 `/cmd_vel` 到位判断。
   - 用 `ros2 topic pub /cmd_vel geometry_msgs/Twist ...` 模拟 pose_control 的输出。

3. **真机集成测试**
   - 先单独跑通 `apriltag_place1_node` 发布 `/grasp/start`。
   - 再启动 `grasp_task_node`，确认 phase_1 → phase_4 能完整执行。
   - 最后加入 `/grasp/place` 触发放置。

---

## 13. 风险与应对

| 风险 | 影响 | 应对 |
|---|---|---|
| `tools/grasp` 与 `grasp_task` 包名冲突 | import 失败 | ROS2 包名使用 `grasp_task`，不命名 `grasp` |
| `sys.path.insert` 导致 `utils` 命名冲突 | 若其他包也叫 `utils` 会冲突 | 可用 `importlib.util.spec_from_file_location` 做别名加载；简单场景先用 sys.path |
| `pose_control` 收到 `/move` 时里程计未就绪 | 指令被忽略 | 节点启动后先等待 `/leg_odom2` 一段时间，必要时发 `reset_origin` |
| 摄像头 `cv2.imshow` 在后台线程报错 | OpenCV 崩溃 | 只在主线程显示；`cv_show` 参数默认 false |
| 多次 `/move` 后横向误差累积 | 对不齐 | 每轮对齐前发 `reset_origin`；在 `/move` 后重新检测并修正 |
| `/grasp/start` 与 `/grasp/place` 同时或乱序到达 | 状态机异常 | 在非预期状态收到信号时记录 warn 并忽略；使用 `Event` 自动清空前一次状态 |

---

## 14. 后续可选优化（不在本次迁移范围内）

1. **拆分摄像头节点**：把机械臂摄像头独立成 `grasp_camera_node`，发布 `sensor_msgs/Image`，`grasp_task_node` 用 `cv_bridge` 订阅。这样多个视觉模块可共享同一路图像，但会引入 `cv_bridge` 依赖和延迟。
2. **自定义 `grasp_task_interfaces`**：当状态信息变复杂时，定义 `GraspStatus.msg`、`GraspCommand.srv` 等。
3. **Action 化**：把抓取封装成 ROS2 Action（`Grasp.action`），支持取消、反馈进度，适合上层 mission planner 调用。
4. **把机械臂 SDK 也做成 ROS2 node**：当前仍是串口直接控制；未来可封装成 `arm_controller_node`，`grasp_task_node` 通过 service 调用。

---

## 15. 总结

本方案通过“**新建 ROS2 包 + 运行时 import tools/grasp**”的方式，在不改动现有工具代码的前提下，把抓取流程接入 `lite3_ws`。核心思路：

- **复用**：`ArmController`、`BlockDetection`、`TargetTracker`、`config.yaml`。
- **替换**：`DogAlignInterface` / `RobotSignalInterface` 两个 stub 改由 ROS topic 实现。
- **适配**：横向对齐走 `/move`，到位信号走 `/grasp/start` 和 `/grasp/place`，状态对外发布 `/grasp/state`。
- **协作**：与 `pose_control`、`apriltag_place1`、`gauge_detector` 等模块通过标准消息交互，不互相侵入。

按此方案实施后，`tools/grasp/main.py` 仍可独立运行做 PC 调试；新的 `grasp_task_node` 负责真机 ROS2 集成，两者并行不悖。