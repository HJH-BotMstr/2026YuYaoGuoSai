# apriltag_place1

AprilTag 视觉对齐节点，用于让 Lite3 机器狗对准 place1 目标位姿。

## 对齐流程（状态机）

采用与 `letter_place_align` 同源的「分相闭环 + 最终校验 + 开环盲进」策略
（2026-08-03 起替换原「瞄准-一步走-复核」策略）：

1. `wait_trigger`：等待 `/apriltag_place1/start` (Bool) 触发。
2. `wait_detect`：连续采集 Tag 位姿填入滤波缓冲，缓冲满且方差足够小后取**中位数**作为对齐起点。
3. `yaw_align`：旋转机身消除水平角偏差。单步旋转限幅 `max_yaw_step_deg`，每步停稳后重新检测再决策。
4. `lateral_align`：横向平移，使 Tag 正对摄像头（|tx| ≤ `lateral_threshold_m`）。
5. `approach`：视觉闭环前进到 `closed_loop_end_dist`（Tag 再近解算误差增大，闭环到此为止）。
6. `final_check`：yaw/横向/距离三项同时达标且连续 `stable_frames` 帧，任何一项不达标回 `yaw_align` 重新修正。
7. `blind_forward`：开环直线前进 `blind = tz_measured - closed_loop_end_dist + final_forward_offset_m`
   （`tz_measured` 为 final_check 通过时的实测距离，约等于 `final_forward_offset_m`），
   完成后发布 `/grasp/start` (Bool)，进入 `done`。

各阶段 Tag 短暂丢失（< `lost_tolerance_s`）沿用上次有效位姿，持续丢失统一回 `wait_detect` 重新瞄准。

## 依赖

- `pupil-apriltags`
- `opencv-python`
- `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`

## 启动方式

### 方式 1：launch 一键启动（推荐）

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch apriltag_place1 apriltag_place1.launch.py
```

该 launch 会同时启动 `pose_control` 位姿控制器和 `apriltag_place1_node`。

### 方式 2：分别启动

终端 1（必须先启动，等待官方栈的 `/leg_odom2`）：

```bash
ros2 run pose_control pose_control
```

终端 2：

```bash
ros2 run apriltag_place1 apriltag_place1_node \
    --ros-args --params-file src/grasp/apriltag_place1/config/apriltag_place1.yaml
```

终端 3（触发对齐）：

```bash
ros2 topic pub /apriltag_place1/start std_msgs/Bool "data: true" --once
```

## 离线自测

无摄像头、无真机的核心策略自测（检测缓存 + 状态机全流程 + 异常路径）：

```bash
source /opt/ros/foxy/setup.bash
python3 /home/ysc/2026YuYaoGuoSai/temp/test_apriltag_place1_node.py
```

## 常见问题

### 1. phase_1 超时：未检测到 Tag

节点现在会输出详细诊断，例如：

```text
phase_1 超时，未检测到 Tag id=0。诊断：已处理 120 帧，检测到任意 Tag 的帧 80 次 最近一次 IDs=[1,2]，检测到目标 ID 的帧 45 次，稳定缓冲 8/15，回到 wait_trigger
```

可能原因与处理：

- **目标 ID 未出现**：确认 `target_tag_id` 与实际贴上的 Tag 一致。
- **看到 Tag 但不稳定**：确保 Tag 在画面中稳定、无抖动、光照充足；必要时增大 `stable_frames` 或放宽稳定标准差阈值。
- **摄像头未就绪**：检查 `camera_device`（可用 `v4l2-ctl --list-devices` 确认）。
- **Tag 尺寸不对**：`tag_size_m` 必须是黑色编码区域边长（不含白边），用尺子测量后填入。

### 2. yaw_align / approach 报运动链路未就绪或运动超时，机器人没动

这是**运动链路未就绪**的典型表现。节点会在首次发送运动指令前检查：

- `/move` 是否有订阅者
- `/leg_odom2` 近期是否有数据
- `/cmd_vel` 近期是否有数据

常见原因与处理：

1. **没启动 `pose_control` 节点**：
   ```bash
   ros2 run pose_control pose_control
   ```
   或使用上方推荐的一键 launch。

2. **官方 ROS2 栈未发布 `/leg_odom2`**：
   `pose_control` 需要 `/leg_odom2` 里程计才能处理 `/move` 指令。确认机器狗运动主机上的官方节点已启动，并且：
   ```bash
   ros2 topic echo /leg_odom2
   ```
   有数据输出。

3. **`/cmd_vel` 没有到达机器狗**：
   检查官方运动节点是否订阅了 `/cmd_vel`：
   ```bash
   ros2 topic info /cmd_vel
   ```

4. **机器人处于急停或锁机状态**：
   检查 `/emergency_stop` 是否为 `true`，以及机器狗是否已上电、已解锁。

### 3. 日志出现“运动指令可能被控制器忽略”

说明 `/move` 已发出，但 `pose_control` 没有输出非零 `/cmd_vel`。最可能的原因是 `/leg_odom2` 尚未发布，`pose_control` 会拒绝所有 `/move` 指令并持续输出零速度。

## 参数说明

见 `config/apriltag_place1.yaml`。

| 参数 | 说明 |
|---|---|
| `camera_device` | V4L2 摄像头设备节点 |
| `target_tag_id` | 目标 AprilTag ID |
| `tag_size_m` | 黑色编码区域边长（米） |
| `closed_loop_end_dist` | 视觉闭环终点与 Tag 的距离，此后转开环盲进 |
| `final_forward_offset_m` | 盲进基准量：blind = tz_measured - closed_loop_end_dist + 本参数 |
| `yaw_align_threshold_deg` | 航向对准阈值 |
| `max_yaw_step_deg` | yaw_align 单步旋转限幅（防惯性过冲） |
| `lateral_threshold_m` | 横向对准阈值 |
| `distance_threshold_m` | 距离到位阈值（相对 closed_loop_end_dist） |
| `stable_frames` | wait_detect 瞄准滤波帧数 / final_check 连续达标帧数 |
| `max_rounds` | yaw/lateral/approach 各相最大修正轮次 |
| `lost_tolerance_s` | Tag 短暂丢失沿用上次位姿的容忍时长 |
| `detect_timeout_s` | phase_1 检测超时 |
| `cmd_vel_zero_timeout_s` | 判断运动停止的 cmd_vel 零速持续窗口 |
| `move_timeout_s` | 单步运动最大等待时间 |
