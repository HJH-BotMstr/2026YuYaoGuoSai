# 后半段抓取代码修改指南

> 日期：2026-08-15
> 操作路径：机械狗主机 `/home/ysc/2026YuYaoGuoSai/`
> 设计依据：`doc/2026-08-15-full-pipeline-nav-design.md`
> 状态：待实施

---

## 0. 修改范围与原则

**不动的部分（现在完全正常）：**
- `transit_point → grasp_task_point`（NAV_TO_TASK）
- TAG_ALIGN / START_BLOCK_ALIGN / WAIT_GRASP_TRANSPORT / KILL_BLOCK_ALIGN
- RETREAT（body frame 后退 0.5m）
- NAV_TO_TRANSIT（回中转点）
- `waypoint_nav.py`（WaypointNavigator 整体不动）
- `grasp_task_node.py`、`block_align_node.py`、`apriltag_place1_node.py`

**要改的部分（目标：TRANSPORT 之后走到正确放置点）：**

| 现状 | 目标 |
|------|------|
| 每字母各有 `task_x/task_y/task_yaw` | 全程共用 `grasp_task_point`，各字母有独立 `place_points[X]` |
| 步骤9 导航到 `task_point`（抓取点）| 步骤9 导航到 `place_points[letter]`（放置点） |
| `_fail_round()` 直接返回 False | `_fail_round()` 先导航回 `transit_point` 再返回 False |
| `skip_on_error: false` | `skip_on_error: true` |

**代码改动量：约 30 行，不新建任何文件。**

---

## 1. 文件改动总览

```
lite3_ws/src/grasp/abcd_task/
├── config/
│   ├── abcd_config.yaml     ← 结构重写（待标定）
│   └── abcd_task.yaml       ← skip_on_error 改为 true
└── abcd_task/
    └── abcd_task_node.py    ← 约 30 行改动
```

---

## 2. `abcd_config.yaml` — 完整替换

**文件路径：**
```
/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/abcd_task/config/abcd_config.yaml
```

**替换为以下内容（标定后填入实际坐标）：**

```yaml
# ABCD 物块任务标定配置
#
# 所有坐标均为 /leg_odom2 全局里程计坐标系（原点=上电位置）。
# 标定方法：使用 tools/way_point.py record 模式，一次记录所有点，
#           顺序末尾依次是 transit_point → place_A → place_B → place_C → place_D → grasp_task_point
#           手动将这 6 个点的 x/y/yaw 复制粘贴到下方对应字段。

apriltag_tag_id: 0

# 中转点：yaw 朝向抓取区（与 grasp_task_point 方向相同）
transit_point:
  x: 0.0    # 标定后填入
  y: 0.0
  yaw: 0.0

# 所有字母共用的抓取准备点（抓取前 TAG_ALIGN 起始位）
# yaw 与 transit_point 相同（面向抓取区）
grasp_task_point:
  x: 0.0    # 标定后填入
  y: 0.0
  yaw: 0.0

# ABCD 各放置点（yaw ≈ transit_point.yaw + π，面向放置区）
place_points:
  A: {x: 0.0, y: 0.0, yaw: 0.0}   # 标定后填入
  B: {x: 0.0, y: 0.0, yaw: 0.0}
  C: {x: 0.0, y: 0.0, yaw: 0.0}
  D: {x: 0.0, y: 0.0, yaw: 0.0}

# 各字母对应色块颜色
letters:
  A: {color: "red"}
  B: {color: "red"}
  C: {color: "green"}
  D: {color: "green"}
```

---

## 3. `abcd_task.yaml` — 单行修改

**文件路径：**
```
/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/abcd_task/config/abcd_task.yaml
```

找到：
```yaml
skip_on_error: false
```
改为：
```yaml
skip_on_error: true
```

---

## 4. `abcd_task_node.py` — 代码修改

**文件路径：**
```
/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/abcd_task/abcd_task/abcd_task_node.py
```

按以下顺序逐步修改，每处标注行号范围（基于当前版本）。

---

### 4.1 `_load_abcd_config()` — 校验逻辑更新

**当前代码（约第 282–305 行）：**
```python
for key in ("letters", "transit_point", "apriltag_tag_id"):
    if key not in data:
        raise ValueError(f"abcd_config 必须含 '{key}'")

# 校验 transit_point
tp = data["transit_point"]
for k in ("x", "y", "yaw"):
    if k not in tp:
        raise ValueError(f"transit_point 缺字段: {k}")

# 校验每个字母（tag_id 是全局的，不在此校验）
for letter, cfg in data["letters"].items():
    for k in ("color", "task_x", "task_y", "task_yaw"):
        if k not in cfg:
            raise ValueError(f"letters[{letter}] 缺字段: {k}")
```

**改为：**
```python
for key in ("letters", "transit_point", "grasp_task_point",
            "place_points", "apriltag_tag_id"):
    if key not in data:
        raise ValueError(f"abcd_config 必须含 '{key}'")

# 校验 transit_point
tp = data["transit_point"]
for k in ("x", "y", "yaw"):
    if k not in tp:
        raise ValueError(f"transit_point 缺字段: {k}")

# 校验 grasp_task_point
gtp = data["grasp_task_point"]
for k in ("x", "y", "yaw"):
    if k not in gtp:
        raise ValueError(f"grasp_task_point 缺字段: {k}")

# 校验 place_points[A/B/C/D]
for letter in ("A", "B", "C", "D"):
    if letter not in data["place_points"]:
        raise ValueError(f"place_points 缺字母: {letter}")
    pp = data["place_points"][letter]
    for k in ("x", "y", "yaw"):
        if k not in pp:
            raise ValueError(f"place_points[{letter}] 缺字段: {k}")

# 校验每个字母（只需 color）
for letter, cfg in data["letters"].items():
    if "color" not in cfg:
        raise ValueError(f"letters[{letter}] 缺字段: color")
```

---

### 4.2 状态常量 — 重命名

**当前代码（约第 82 行）：**
```python
R_NAV_TO_TASK_2        = "NAV_TO_TASK_2"
```

**改为：**
```python
R_NAV_TO_PLACE         = "NAV_TO_PLACE"
```

---

### 4.3 `_run_letter()` — 三处修改

**第一处：读取 `task_point`（约第 544–551 行）**

当前：
```python
cfg = self._abcd_config["letters"][letter]
transit = self._abcd_config["transit_point"]
task_point = {
    "x":   float(cfg["task_x"]),
    "y":   float(cfg["task_y"]),
    "yaw": float(cfg["task_yaw"]),
}
```

改为：
```python
cfg = self._abcd_config["letters"][letter]
transit = self._abcd_config["transit_point"]
task_point = {
    "x":   float(self._abcd_config["grasp_task_point"]["x"]),
    "y":   float(self._abcd_config["grasp_task_point"]["y"]),
    "yaw": float(self._abcd_config["grasp_task_point"]["yaw"]),
}
place_pt = {
    "x":   float(self._abcd_config["place_points"][letter]["x"]),
    "y":   float(self._abcd_config["place_points"][letter]["y"]),
    "yaw": float(self._abcd_config["place_points"][letter]["yaw"]),
}
```

**第二处：步骤9 导航目标（约第 610–614 行）**

当前：
```python
# 9) NAV_TO_TASK_2（放置准备）
self._set_round_state(self.R_NAV_TO_TASK_2)
if not self._nav.navigate_to(task_point, timeout_s=self._nav_timeout_s,
                             should_abort=self._abort_requested):
    return self._fail_round(letter, "NAV_TO_TASK_2 失败")
```

改为：
```python
# 9) NAV_TO_PLACE（导航到该字母对应放置点）
self._set_round_state(self.R_NAV_TO_PLACE)
if not self._nav.navigate_to(place_pt, timeout_s=self._nav_timeout_s,
                             should_abort=self._abort_requested):
    return self._fail_round(letter, "NAV_TO_PLACE 失败")
```

**第三处：dry_run 模式入口传参（约第 560–561 行）**

当前：
```python
if self._dry_run_nav:
    return self._dry_run_letter_tail(letter, transit, task_point)
```

改为：
```python
if self._dry_run_nav:
    return self._dry_run_letter_tail(letter, transit, task_point, place_pt)
```

---

### 4.4 `_dry_run_letter_tail()` — 同步更新

**当前代码（约第 635–659 行）：**
```python
def _dry_run_letter_tail(self, letter, transit, task_point) -> bool:
    """dry_run_nav 模式：只跑导航，跳过 tag_align/grasp/place。"""
    self._set_round_state(self.R_RETREAT)
    if not self._nav.move_relative_body(
            -self._retreat_dist_m, 0.0, 0.0,
            timeout_s=self._nav_timeout_s,
            should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] RETREAT 失败")

    self._set_round_state(self.R_NAV_TO_TRANSIT)
    if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                 should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] NAV_TO_TRANSIT 失败")

    self._set_round_state(self.R_NAV_TO_TASK_2)
    if not self._nav.navigate_to(task_point, timeout_s=self._nav_timeout_s,
                                 should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] NAV_TO_TASK_2 失败")

    self._set_round_state(self.R_NAV_BACK_TO_TRANSIT)
    if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                 should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] NAV_BACK_TO_TRANSIT 失败")

    return True
```

**改为：**
```python
def _dry_run_letter_tail(self, letter, transit, task_point, place_pt) -> bool:
    """dry_run_nav 模式：只跑导航，跳过 tag_align/grasp/place。"""
    self._set_round_state(self.R_RETREAT)
    if not self._nav.move_relative_body(
            -self._retreat_dist_m, 0.0, 0.0,
            timeout_s=self._nav_timeout_s,
            should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] RETREAT 失败")

    self._set_round_state(self.R_NAV_TO_TRANSIT)
    if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                 should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] NAV_TO_TRANSIT 失败")

    self._set_round_state(self.R_NAV_TO_PLACE)
    if not self._nav.navigate_to(place_pt, timeout_s=self._nav_timeout_s,
                                 should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] NAV_TO_PLACE 失败")

    self._set_round_state(self.R_NAV_BACK_TO_TRANSIT)
    if not self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                                 should_abort=self._abort_requested):
        return self._fail_round(letter, "[dry_run] NAV_BACK_TO_TRANSIT 失败")

    return True
```

---

### 4.5 `_fail_round()` — 增加归位导航

**当前代码（约第 661–666 行）：**
```python
def _fail_round(self, letter: str, reason: str) -> bool:
    self.get_logger().error(f"[{letter}] round ERROR: {reason}")
    # 保险起手：停车、杀掉 block_align 子进程
    self._nav.send_zero_move()
    self._kill_proc("block_align")
    return False
```

**改为：**
```python
def _fail_round(self, letter: str, reason: str) -> bool:
    self.get_logger().error(f"[{letter}] round ERROR: {reason}")
    self._nav.send_zero_move()
    self._kill_proc("block_align")
    # 尝试回中转点，让 skip_on_error 继续下一字母时有干净起点
    try:
        transit = self._abcd_config["transit_point"]
        self.get_logger().warning(f"[{letter}] 尝试归位至 transit_point")
        self._nav.navigate_to(transit, timeout_s=self._nav_timeout_s,
                              should_abort=self._abort_requested)
    except Exception as e:
        self.get_logger().error(f"[{letter}] 归位失败: {e}")
    return False
```

> 注：`_fail_round()` 归位失败不影响返回值，因为主循环拿到 `False` 后
> 由 `skip_on_error=true` 决定是否继续——归位是尽力而为，不做强保证。

---

## 5. 修改后完整逻辑验证

改完后 `_run_letter()` 的步骤序列（以字母 A 为例）：

```
1  NAV_TO_TASK          → navigate_to(grasp_task_point)        # 共用抓取准备点
2  TAG_ALIGN            → apriltag_place1 对齐 tag_id=0
3  START_BLOCK_ALIGN    → spawn block_align，目标色块=red
4  WAIT_GRASP_TRANSPORT → 等 /grasp/state == TRANSPORT
5  KILL_BLOCK_ALIGN     → SIGTERM block_align
6  RETREAT              → move_relative_body(dx=-0.5m)
7  NAV_TO_TRANSIT       → navigate_to(transit_point)
8  NAV_TO_PLACE         → navigate_to(place_points["A"])       # ← 核心改动
9  SIGNAL_PLACE         → 发 /grasp/place="A" (2Hz×5s)
10 WAIT_PLACE_RESULT    → 等 /grasp/result=True
11 NAV_BACK_TO_TRANSIT  → navigate_to(transit_point)
```

失败时（任意步骤）：
```
_fail_round() → send_zero_move → kill block_align → navigate_to(transit_point)
主循环 skip_on_error=True → continue → 开始字母 B
```

---

## 6. 标定流程（机械狗主机执行）

### 6.1 启动 pose_controller

```bash
# 机械狗主机
ros2 launch pose_control pose_control.launch.py
```

### 6.2 启动 way_point.py record

```bash
cd /home/ysc/2026YuYaoGuoSai/tools
python3 way_point.py record
```

### 6.3 记录顺序

按以下顺序移动机械狗并记录（键盘：`S`=开始，`A`=添加，`E`=结束）：

```
S  → 出发区（前半部分路径起点）
A  → ... 前半部分拐点、巡检位等 ...
A  → 中转点 transit_point（朝向抓取区，AprilTag 可见）
A  → A 放置点 place_A（朝向放置区，yaw ≈ transit_yaw + π）
A  → B 放置点 place_B
A  → C 放置点 place_C
A  → D 放置点 place_D
A  → 抓取准备点 grasp_task_point（朝向抓取区，距 AprilTag 1.0~1.4m）
E  → 保存 waypoints.json
```

### 6.4 复制坐标到 abcd_config.yaml

```bash
# 查看最后 6 个点（文件中倒数 6 条 waypoint）
cat /home/ysc/2026YuYaoGuoSai/tools/waypoints.json | python3 -c "
import json, sys
d = json.load(sys.stdin)
for wp in d['waypoints'][-6:]:
    print(wp)
"
```

输出格式：`{"x": ..., "y": ..., "yaw": ...}`

按顺序对应：`transit_point, place_A, place_B, place_C, place_D, grasp_task_point`

手动填入 `/home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/abcd_task/config/abcd_config.yaml`。

---

## 7. 单独后半部分测试

不依赖前半部分巡检，直接测试抓取放置流程。

### 7.1 准备工作

- 机械狗上电，摆放在**标定时的 transit_point 位置**（原点必须一致）
- 确认 pose_controller、grasp_task 已启动

### 7.2 启动命令

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source install/setup.bash

# 只跑导航链路（dry run，验证坐标是否正确）
ros2 launch abcd_task abcd_task.launch.py \
    dry_run_nav:=true \
    task_order:='["A"]' \
    start_from:=A

# 单轮完整测试（字母 A）
ros2 launch abcd_task abcd_task.launch.py \
    task_order:='["A"]' \
    start_from:=A

# 全流程 ABCD
ros2 launch abcd_task abcd_task.launch.py
```

### 7.3 测试顺序

1. `dry_run_nav=true, task_order=["A"]` — 验证 transit→grasp_task→transit→place_A→transit 路径无碰撞
2. 单轮 A 完整测试 — 验证抓取 + 放置
3. 双轮 AB — 验证第二轮起点（A 结束后的 transit_point 精度）
4. 全流程 ABCD — 验证 skip_on_error：手动拦住 A（不放物块），观察是否自动跳过到 B

---

## 8. 与前半部分巡检全流程接入接口

前半部分巡检结束时，机械狗到达 `transit_point`。后半部分 `abcd_task_node` 在 `INIT` 阶段等待 `/grasp/state == STANDBY`，这是两个部分唯一的握手信号。

### 8.1 接入方式

**方式一（推荐）：顺序 launch 脚本**

巡检 launch 结束后自动起 abcd_task：
```bash
# 前半部分结束后执行：
ros2 launch abcd_task abcd_task.launch.py
```
只需确保 `grasp_task_node` 已在运行，`abcd_task_node` 会等 `/grasp/state == STANDBY`（最多 60s，`standby_wait_timeout_s` 参数）。

**方式二：前半部分发信号给 abcd_task**

前半部分最后一步发布：`ros2 topic pub /abcd_task/trigger std_msgs/Bool "data: true" --once`

当前代码无此订阅；如需要，在 `AbcdTaskNode.__init__` 添加：
```python
self.create_subscription(Bool, "/abcd_task/trigger", self._on_trigger, 10)
```
并在 `_on_trigger` 中设置一个 `threading.Event`，在 `run()` 开头等待该 event。

**目前代码无需改动即可接入——前半部分手动结束后，直接 `ros2 launch abcd_task` 即可。**

### 8.2 接入检查清单

- [ ] 前半部分结束时，机械狗确实在 `transit_point` 坐标附近（误差 < 0.15m）
- [ ] `grasp_task_node` 已在运行，`/grasp/state` 为 `STANDBY`
- [ ] `abcd_config.yaml` 的 `transit_point` 坐标与前半部分标定共用同一原点（同一次上电）

---

## 9. 快速定位参考（行号基于当前版本）

| 修改位置 | 文件 | 约行号 |
|----------|------|--------|
| `_load_abcd_config` 校验 | `abcd_task_node.py` | 283–305 |
| `R_NAV_TO_TASK_2` 常量 | `abcd_task_node.py` | 82 |
| `_run_letter` 读取 task_point | `abcd_task_node.py` | 544–551 |
| `_run_letter` 步骤9 | `abcd_task_node.py` | 610–614 |
| `_run_letter` dry_run 入口 | `abcd_task_node.py` | 560–561 |
| `_dry_run_letter_tail` 函数签名+步骤9 | `abcd_task_node.py` | 635–659 |
| `_fail_round` | `abcd_task_node.py` | 661–666 |
| `skip_on_error` | `abcd_task.yaml` | 第9行 |
| 整个配置文件 | `abcd_config.yaml` | 全文替换 |
