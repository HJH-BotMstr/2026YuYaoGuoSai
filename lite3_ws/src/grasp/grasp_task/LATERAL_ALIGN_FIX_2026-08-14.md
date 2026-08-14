# grasp_task 横向对齐修复文档

**日期**: 2026-08-14  
**问题**: grasp_task 横向对齐精度不如 block_align  
**影响**: 抓取前的横向对齐不准确，可能导致机械臂抓取失败

---

## 问题分析

### 主要原因
**Tracker 频繁重建破坏滑动平均稳定性**

原代码在每轮对齐开始时都完全重建 `TargetTracker`：
```python
# 旧代码 (grasp_node.py:451-455)
self.tracker = TargetTracker(
    avg_window=int(cfg_g["distance_avg_window"]),
    lost_frames_max=int(cfg_g["lost_frames_max"]),
)
```

**问题**:
- 滑动平均窗口被清空，失去历史数据的稳定作用
- 每次检测都从零开始，容易受噪声干扰
- 导致横向对齐抖动和不收敛

**对比 block_align_node**:
```python
# block_align_node.py:651-656
if not self._settle_tracker_ready:
    self._tracker = self._TargetTracker(...)
    self._settle_tracker_ready = True
```
- 运动停止后只重建**一次**
- 之后连续多帧积累，保证滑动平均有效

### 次要原因
**缺少"强制锁定最右边目标"逻辑**

原代码只有日志记录，没有强制重新锁定：
```python
# 旧代码 (grasp_node.py:393-398)
if len(candidates) > 1:
    self.get_logger().info(
        f"检测到 {len(candidates)} 个候选色块，X_cam 值={x_values}，"
        f"将锁定最右边的 (X_cam={max(x_values):.1f}mm)"
    )
```

**问题**:
- 依赖 TargetTracker 内部逻辑自动选择
- 多个同色方块时可能锁定错误目标（靠中心的而非最右的）

**对比 block_align_node**:
```python
# block_align_node.py:494-508
all_candidates = self._detector.detect_all(frame)
if len(all_candidates) >= 2:
    rightmost = max(all_candidates, key=lambda r: r["pos_3d"][0])
    if stable["pos_3d"][0] < rightmost["pos_3d"][0] - 5.0:
        # 重建 tracker 并锁定最右的
        self._tracker = self._TargetTracker(...)
        self._tracker.update([rightmost])
        return  # 等待下一帧稳定
```

---

## 修复方案

### 1. 引入 settle_tracker_ready 机制

**修改位置**: `_align_laterally()` 方法

**修改内容**:
```python
# 新增控制标志
settle_tracker_ready = False

for round_i in range(max_rounds):
    # ... 运动指令发布 ...
    
    # 运动停止后调用新方法
    new_stable = self._detect_stable_after_move(settle_tracker_ready, cfg_g)
    
    # 首次重建后，标记为已就绪
    settle_tracker_ready = True
```

**效果**:
- 第一轮：`settle_tracker_ready=False`，重建 tracker
- 第二轮起：`settle_tracker_ready=True`，复用 tracker
- 保持滑动平均窗口的连续性

### 2. 新增 _detect_stable_after_move() 方法

**职责**:
1. 根据 `tracker_ready` 标志决定是否重建 tracker（只重建一次）
2. 调用原有 `_detect_stable()` 获取稳定目标
3. 检查是否有多个同色候选，强制锁定最右边的
4. 如果重新锁定，递归调用等待新 tracker 稳定

**关键逻辑**:
```python
def _detect_stable_after_move(self, tracker_ready: bool, cfg_g: dict) -> dict:
    # 只在 tracker 未就绪时重建一次
    if not tracker_ready:
        self.tracker = TargetTracker(...)
    
    stable = self._detect_stable()
    if stable is None:
        return None
    
    # 获取当前帧所有候选（应用 ROI）
    ret, frame = self.arm_cam.read()
    if ret and frame is not None:
        roi_frame, roi_offset = self._apply_roi(frame)
        all_candidates = self.detector.detect_all(roi_frame)
        all_candidates = self._restore_roi_coords(all_candidates, roi_offset)
        
        # 强制锁定最右边的目标
        if len(all_candidates) >= 2:
            rightmost = max(all_candidates, key=lambda r: r["pos_3d"][0])
            if stable["pos_3d"][0] < rightmost["pos_3d"][0] - 5.0:
                self.get_logger().info("检测到更右的目标，重新锁定")
                self.tracker = TargetTracker(...)
                self.tracker.update([rightmost])
                return self._detect_stable_after_move(True, cfg_g)  # 递归等待稳定
    
    return stable
```

---

## 代码规范保证

### 接口适配
- ✅ 复用现有 `_detect_stable()` 方法，无需修改其接口
- ✅ 复用现有 `_apply_roi()` 和 `_restore_roi_coords()` 方法
- ✅ 新方法参数清晰：`tracker_ready` 控制重建，`cfg_g` 传递配置

### 时序逻辑
- ✅ 第一轮：重建 tracker → 等待运动停止 → 多帧检测稳定 → 锁定最右边
- ✅ 第二轮起：复用 tracker → 等待运动停止 → 多帧检测稳定 → 锁定最右边
- ✅ 递归调用确保重新锁定后也要等待 stable_frames 帧稳定

### 可维护性
- ✅ 提取独立方法 `_detect_stable_after_move()`，职责单一
- ✅ 详细注释说明修复原因和参考来源（block_align_node）
- ✅ 保留原有日志输出，增加新的锁定日志
- ✅ 错误处理：检测失败返回 None，上层统一处理

---

## 测试建议

### 场景 1: 单个目标色块
**预期**: 第一轮重建 tracker，后续轮次复用，对齐精度提升

### 场景 2: 两个同色色块
**预期**: 
- 自动锁定最右边的目标
- 日志输出 "检测到更右的目标，重新锁定"
- 横向对齐到最右边的色块

### 场景 3: ROI 边缘情况
**预期**:
- 目标在 ROI 边缘时，tracker 连续性保证不会丢失
- 如果移出 ROI，检测超时后返回 None，上层报错

### 验证方法
1. 对比修复前后的横向对齐日志：
   - 旧版：每轮 `X_cam` 值可能跳变（tracker 重建导致）
   - 新版：`X_cam` 值平滑收敛（tracker 连续积累）

2. 观察 cv_show 可视化：
   - 锁定框应始终跟踪最右边的色块
   - 不应在同色方块间跳变

3. 测试指标：
   - 横向对齐轮次减少（更快收敛）
   - 最终 X_cam 偏差更小（更精确）
   - 抓取成功率提升

---

## 与 block_align_node 对比

| 特性 | block_align_node | grasp_task (修复后) |
|------|------------------|---------------------|
| tracker 重建策略 | 运动停止后重建一次 | ✅ 运动停止后重建一次 |
| 锁定最右边目标 | ✅ 主动检查并重新锁定 | ✅ 主动检查并重新锁定 |
| ROI 限制 | ❌ 无 | ✅ 有（grasp_task 特有） |
| settle_tracker_ready | ✅ `_settle_tracker_ready` | ✅ `settle_tracker_ready` |

**差异点**:
- block_align_node 是成员变量 `self._settle_tracker_ready`
- grasp_task 是局部变量 `settle_tracker_ready`（因为是在循环内，不需要跨方法共享）

---

## 回退方案

如果新版本出现问题，可回退到旧版本：
```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws/src/grasp/grasp_task
git diff grasp_task/grasp_node.py  # 查看修改
git checkout grasp_task/grasp_node.py  # 回退
colcon build --packages-select grasp_task
```

---

## 相关文件

- **修改文件**: `lite3_ws/src/grasp/grasp_task/grasp_task/grasp_node.py`
- **参考实现**: `lite3_ws/src/grasp/block_align/block_align/block_align_node.py`
- **配置文件**: `lite3_ws/src/grasp/grasp_task/config/grasp_task.yaml`

## 关键代码位置

- 修复主体: [grasp_node.py:434-551](grasp_node.py#L434-L551)
  - `_align_laterally()`: 434-485行
  - `_detect_stable_after_move()`: 487-551行
- 参考逻辑: [block_align_node.py:636-701](../../../block_align/block_align/block_align_node.py#L636-L701)
