# ABCD 抓取放置全流程时序与逻辑分析

## 目录
1. [整体架构](#整体架构)
2. [单轮流程详解](#单轮流程详解)
3. [关键节点交互](#关键节点交互)
4. [时序图](#时序图)
5. [潜在问题分析](#潜在问题分析)

---

## 整体架构

### 三层架构
```
abcd_task_node (顶层编排)
    ↓ 调用
apriltag_place1_node + block_align_node (视觉对齐层)
    ↓ 触发
grasp_task/grasp_node (抓取执行层)
```

### 节点职责

| 节点 | 职责 | 生命周期 |
|------|------|----------|
| `abcd_task_node` | ABCD 四轮编排、导航、子进程管理 | 全流程常驻 |
| `apriltag_place1_node` | AprilTag 视觉对齐 | 全流程常驻 |
| `block_align_node` | 色块横向对齐+搜索 | **每轮动态拉起/销毁** |
| `grasp_node` | 8-phase 机械臂抓取放置 | 全流程常驻（max_rounds=4）|

---

## 单轮流程详解

### 阶段1：导航到任务点 (NAV_TO_TASK)

```python
# abcd_task_node.py:554-557
self._nav.navigate_to(task_point, timeout_s=30.0)
```

**输入**：`abcd_config.yaml` 中的 `task_x, task_y, task_yaw`  
**输出**：机器人到达任务点附近  
**超时**：30s

---

### 阶段2：AprilTag 对齐 (TAG_ALIGN)

```python
# abcd_task_node.py:565-573
self._arm_gate_time_mono = time.monotonic()  # 防止拿到旧 latched 消息
self._pub_apriltag_start.publish(Bool(data=True))
self._wait_apriltag_done(timeout_s=30.0)
```

**apriltag_place1_node 流程**：
1. `wait_detect`：等待检测到目标 AprilTag（15s 超时）
2. `search`：如果超时，横向搜索（10步 × 0.1m）
3. `lateral_align`：消除横向偏差（X 方向）
4. `approach`：前进到目标距离（1.0m）
5. `yaw_finetune`：微调角度偏差
6. 发布 `/apriltag_place1/done = True`

**关键参数**：
- `target_distance_m: 1.0`（AprilTag 对齐的停止距离）
- `detect_timeout_s: 15.0`
- `search_step_m: 0.10`

**超时**：30s

---

### 阶段3：启动 block_align 子进程 (START_BLOCK_ALIGN)

```python
# abcd_task_node.py:578-582
target_color = cfg.get("color", "")  # 从 abcd_config 读取 "red"/"green"
self._spawn_block_align(target_color=target_color)
```

**子进程启动命令**：
```bash
ros2 run block_align block_align_node \
  --ros-args \
  --params-file /path/to/block_align.yaml \
  -p target_color:=red  # 命令行参数，覆盖 yaml
```

**为什么用子进程？**
- 避免 `/grasp/start` 的 TRANSIENT_LOCAL QoS 被多个发布者竞争
- 每轮独立拉起，确保状态干净
- 抓取完成后销毁，释放摄像头资源

---

### 阶段4：等待抓取完成 (WAIT_GRASP_TRANSPORT)

```python
# abcd_task_node.py:585-589
# 1Hz 重复发 /block_align/start=True，覆盖订阅竞争
while timeout not reached:
    self._pub_block_align.publish(Bool(data=True))
    if grasp_state in {"TRANSPORT", "PLACING", "DONE"}:
        return True  # 成功
```

**block_align_node 流程**：
1. `wait_detect`：等待稳定检测到色块（15s 超时）
   - 如果有多个同色方块，**选最右边的**（TargetTracker: `max(X_cam)`）
   - 如果超时 → 进入 `search`
2. `search`：横向搜索（**修复后：从左往右**）
   - 每步 0.1m，最多 10 步
   - 检测到稳定目标后 → 进入 `lateral_align`
3. `lateral_align`：消除横向偏差
   - 目标：`|X_cam| <= 10mm`
   - 最多 5 轮
4. `approach`：前进到目标距离
   - 目标：`0.20m`（**修复后，原来 0.10m**）
5. 发布 `/grasp/start = True`，触发 grasp_node
6. **立即释放摄像头**（`cap.release()`）

**grasp_node 流程**（8-phase）：
1. `STANDBY` → 等待 `/grasp/start`
2. `DETECTING` → 打开摄像头，检测色块（30s 超时）
3. `ALIGNING` → 横向对齐（最多 5 轮，阈值 15mm）
4. `GRASPING` → 机械臂抓取
5. `TRANSPORT` → **到达此状态，abcd_task 认为抓取成功**
6. `PLACING` → 等待 `/grasp/place`
7. `DONE` → 放置完成
8. 发布 `/grasp/result = True`

**超时**：300s

---

### 阶段5：销毁 block_align (KILL_BLOCK_ALIGN)

```python
# abcd_task_node.py:592-594
os.killpg(os.getpgid(proc.pid), signal.SIGTERM)  # 5s grace
os.killpg(os.getpgid(proc.pid), signal.SIGKILL)  # 兜底
```

**为什么要销毁？**
- 释放 `/grasp/start` 的 TRANSIENT_LOCAL QoS 发布端
- 避免下一轮 grasp_node 收到上一轮的 latched 消息

---

### 阶段6-12：后退、导航、放置

```python
# 6) 后退 0.5m
self._nav.move_relative_body(dx_body=-0.5, dy_body=0.0)

# 7) 导航回中转点
self._nav.navigate_to(transit_point)

# 8) 再次导航到任务点（放置准备）
self._nav.navigate_to(task_point)

# 9-10) 发布 /grasp/place = letter，等待 /grasp/result
self._burst_publish_place(letter)  # 2Hz × 5s
self._wait_place_result(timeout_s=120.0)

# 11) 导航回中转点
self._nav.navigate_to(transit_point)
```

---

## 关键节点交互

### 1. apriltag → block_align 交接

```
apriltag_place1_node:
  approach 完成 → tz=1.0m
  发布 /apriltag_place1/done = True
  ❌ 不再发布 /grasp/start（修复 2026-08-12）

abcd_task_node:
  收到 /apriltag_place1/done
  → spawn block_align 子进程
  → 1Hz 发布 /block_align/start = True

block_align_node:
  收到 /block_align/start
  → wait_detect / search / lateral_align / approach
  → 发布 /grasp/start = True
  → 立即释放摄像头
```

**关键修复点**：
- apriltag 不再触发 grasp（避免摄像头竞争）
- block_align 完成后立即释放摄像头
- block_align 是子进程，抓取后被销毁

---

### 2. block_align → grasp_node 交接

```
block_align_node:
  approach 完成 → dist=0.20m
  发布 /grasp/start = True (TRANSIENT_LOCAL QoS)
  立即 cap.release()

grasp_node:
  收到 /grasp/start
  → 进入 DETECTING
  → _ensure_arm_cam_open()  # 重新打开摄像头
  → _detect_stable()
  → _align_laterally()
  → _approach_and_grasp()
  → _release_arm_cam()  # 抓取完释放
  → 进入 TRANSPORT 状态
```

**摄像头资源管理**：
| 阶段 | 占用者 | 备注 |
|------|--------|------|
| apriltag 对齐 | apriltag_place1_node | 自己的摄像头 |
| block_align 检测 | block_align_node | 打开 /dev/video0 |
| block_align 完成 | **释放** | `cap.release()` |
| grasp DETECTING | grasp_node | 重新打开 /dev/video0 |
| grasp 抓取后 | **释放** | `_release_arm_cam()` |

---

### 3. abcd_task → grasp_node 放置交接

```
abcd_task_node:
  等待 grasp_state == "TRANSPORT"
  → KILL_BLOCK_ALIGN
  → 后退 + 导航
  → 2Hz × 5s 连发 /grasp/place = letter

grasp_node:
  TRANSPORT 状态
  → 等待 /grasp/place
  → 收到信号
  → _place()
  → 发布 /grasp/result = True
  → 进入 DONE 状态
```

**为什么连发？**
- TRANSIENT_LOCAL QoS：晚订阅者能收到最后一条
- 但如果订阅早于发布，可能错过
- 连发 5 秒确保覆盖到订阅窗口

---

## 时序图

### 正常流程时序

```
时间轴 →

abcd_task         apriltag_place1    block_align         grasp_node
    |                   |                  |                  |
    | NAV_TO_TASK       |                  |                  |
    |------------------>|                  |                  |
    |                   |                  |                  |
    | /apriltag/start   |                  |                  |
    |------------------>| wait_detect      |                  |
    |                   | search           |                  |
    |                   | lateral_align    |                  |
    |                   | approach         |                  |
    |                   |                  |                  |
    | /apriltag/done    |                  |                  |
    |<------------------|                  |                  |
    |                   |                  |                  |
    | spawn subprocess  |                  |                  |
    |------------------------------>| (启动)               |
    |                   |                  |                  |
    | /block_align/start(1Hz)             |                  |
    |------------------------------>| wait_detect          |
    |                   |                  | search           |
    |                   |                  | lateral_align    |
    |                   |                  | approach         |
    |                   |                  |                  |
    |                   |                  | /grasp/start     |
    |                   |                  |----------------->| DETECTING
    |                   |                  | cap.release()    | _detect_stable()
    |                   |                  |                  | _align_laterally()
    |                   |                  |                  | _approach_and_grasp()
    |                   |                  |                  | → TRANSPORT
    |                   |                  |                  |
    | grasp_state=TRANSPORT                |                  |
    |<----------------------------------------------------|
    |                   |                  |                  |
    | SIGTERM           |                  |                  |
    |---------------------------->| (销毁)                |
    |                   |                  |                  |
    | RETREAT + NAV     |                  |                  |
    |                   |                  |                  |
    | /grasp/place=A (2Hz×5s)              |                  |
    |----------------------------------------------------->| PLACING
    |                   |                  |                  | _place()
    |                   |                  |                  | → DONE
    |                   |                  |                  |
    | /grasp/result=True                   |                  |
    |<----------------------------------------------------|
    |                   |                  |                  |
    | NAV_BACK          |                  |                  |
```

---

## 潜在问题分析

### ❌ 问题1：_is_cmd_vel_zero() 的兜底逻辑顺序错误（已修复）

**位置**：`block_align_node.py:325-350`

**问题**：
```python
def _is_cmd_vel_zero(self) -> bool:
    recent = [(t, v) for (t, v) in self._cmdvel_history if t >= cutoff]
    if not recent:
        return False  # ← 提前返回，永远执行不到后面的超时兜底
    
    # 超时兜底在这里 ← 永远执行不到
    if (not started and now - self._last_move_time > self._move_timeout):
        return True
```

**后果**：
- 如果 `pose_controller` 不发 cmd_vel（距离太小被忽略）
- `_cmdvel_history` 在 0.5s 后全部过期，`recent=[]`
- 第 330 行返回 False
- `lateral_align` 和 `approach` 无限卡在 `if not self._is_cmd_vel_zero(): return`

**修复**：✅ 已将超时兜底逻辑提前到检查 `recent` 之前

---

### ❌ 问题2：TargetTracker 选择了错误的目标（已修复）

**位置**：`TargetTracker.py:54`

**代码**：
```python
chosen = max(candidates, key=lambda r: r["pos_3d"][0])  # 选 X_cam 最大（最右）
```

**场景**：
- 从右到左：`红1 ← 红2 ← 绿1 ← 绿2`
- 目标：抓红1（最右边的红色）
- `target_color="red"` 过滤后，`candidates = [红1, 红2]`

**问题**：
- 如果 apriltag 对齐后摄像头偏左，只能看到红2
- 或者 search 从右往左扫描，先看到红2
- tracker 锁定红2 后，即使后续看到红1 也不会切换

**修复**：
1. ✅ 修改 search 方向为"从左往右"（优先覆盖右侧）
2. ✅ 在 `wait_detect` 和 `search` 检测到稳定目标时，检查是否有更右的候选，强制重新锁定

---

### ❌ 问题3：search 方向与目标选择逻辑不匹配（已修复）

**原逻辑**：
```python
y_dir = -self._lat_polarity  # lateral_polarity=-1 → y_dir=+1 → 向左搜索
```

**问题**：
- search 从右往左
- 如果初始位置偏左，向左扫描会远离最右边的目标

**修复**：✅ 改为"从左往右"搜索
```python
y_dir = self._lat_polarity  # lateral_polarity=-1 → y_dir=-1 → 向右搜索
```

---

### ⚠️ 潜在问题4：target_distance_m 可能导致视野问题

**当前值**：`0.20m`（修复后，原来 0.10m）

**问题**：
- 如果 0.20m 仍然太近，方块可能超出摄像头视野下边缘
- grasp_node 的 DETECTING 阶段会检测不到

**建议**：
- 现场测试 0.20m 是否合适
- 如果不够，继续增大到 0.25m 或 0.30m

---

### ⚠️ 潜在问题5：grasp_node 的 DETECTING 阶段可能检测不到目标

**场景**：
- block_align 完成后，距离=0.20m
- block_align 释放摄像头
- grasp_node 重新打开摄像头，开始检测

**问题**：
- 如果此时机器人位置有微小漂移
- 或者 tracker 状态未重置
- 可能检测不到目标，导致 30s 超时

**解决方案**：
- grasp_node 的 `_detect_stable()` 会重新创建 TargetTracker
- 但需要确保初始位置合适

---

### ⚠️ 潜在问题6：block_align 和 grasp_node 都做横向对齐，重复劳动

**block_align 对齐**：
- `lateral_align`：`|X_cam| <= 10mm`

**grasp_node 对齐**：
- `_align_laterally()`：`|X_cam| <= 15mm`

**问题**：
- block_align 已经对齐到 10mm 以内
- grasp_node 重新检测后，理论上应该在 15mm 以内，不需要再对齐
- 但实际上 grasp_node 总是会执行对齐

**建议**：
- 如果 block_align 的对齐精度足够，grasp_node 可以跳过 ALIGNING 阶段
- 或者增大 grasp_node 的阈值到 20mm，减少对齐次数

---

### ⚠️ 潜在问题7：摄像头资源竞争的时间窗口

**时序**：
```
block_align: cap.release()
               ↓ (短暂间隔)
grasp_node: _ensure_arm_cam_open()
```

**问题**：
- 如果 `cap.release()` 和 `_ensure_arm_cam_open()` 之间间隔太短
- 摄像头驱动可能还没完全释放设备
- 导致 `_ensure_arm_cam_open()` 失败

**解决方案**：
- `_ensure_arm_cam_open()` 有重试机制（3次，间隔1秒）
- 理论上足够覆盖释放时间

---

### ✅ 优化点：减少不必要的对齐

**当前流程**：
```
apriltag: 对齐到 1.0m，yaw 微调
    ↓
block_align: lateral_align（X 方向）+ approach（前进到 0.20m）
    ↓
grasp_node: _align_laterally()（X 方向，再次对齐）
```

**建议**：
- 如果 block_align 的对齐已经足够精确
- grasp_node 可以检查 `|X_cam| < threshold` 后直接跳过对齐
- 节省时间，减少运动次数

---

## 总结

### 已修复的关键问题
1. ✅ `_is_cmd_vel_zero()` 兜底逻辑顺序错误
2. ✅ TargetTracker 选择目标逻辑（在 wait_detect/search 层强制选最右）
3. ✅ search 方向修改为"从左往右"
4. ✅ 增大 `target_distance_m` 到 0.20m
5. ✅ 增强 debug 日志

### 流程逻辑评估
- ✅ 节点职责划分清晰
- ✅ 摄像头资源管理合理
- ✅ 子进程生命周期管理正确
- ✅ 超时保护机制完善（修复后）
- ⚠️ 重复对齐可优化
- ⚠️ 需要现场测试验证各参数

### 建议后续优化
1. 现场测试 `target_distance_m = 0.20m` 是否合适
2. 考虑减少 grasp_node 的对齐次数（如果 block_align 精度够）
3. 监控 pose_controller 忽略小距离指令的阈值
4. 添加更多实时状态监控日志

