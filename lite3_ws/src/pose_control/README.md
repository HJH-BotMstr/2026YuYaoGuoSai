# pose_control

Lite3 位姿闭环控制 ROS2 包。基于官方 ROS2 栈的里程计做位置/航向闭环，输出速度指令；同时订阅后向超声波实现后退避障，并可通过 `/move` 话题接收外部位移指令。

## 功能

- **位置闭环移动**：接收 `/move` 话题的 `(x, y, yaw)` 三元组，按机身坐标系相对移动，到达后自动停止。
- **航向闭环旋转**：支持原地旋转到相对程序启动原点的目标角度。
- **超声波避障**：后退方向遇到障碍物（默认 0.35 m）时停止，移开后自动继续。
- **终端调试**：默认开启终端命令输入，支持 `x+0.5`、`y-0.1`、`yaw90` 等元指令（可用 `enable_terminal` 关闭）。

## 订阅

| Topic | 类型 | 说明 |
|---|---|---|
| `/leg_odom2` | `nav_msgs/Odometry` | 官方栈发布的足上里程计，用于位置与航向反馈。 |
| `/us_publisher/ultrasound_distance` | `std_msgs/Float64` | 后向超声波距离（米），参数 `sonar_topic` 可修改。 |
| `/emergency_stop` | `std_msgs/Bool` | 急停信号，`true` 时强制输出零速度。 |
| `/move` | `geometry_msgs/Pose2D` | 外部目标指令：`x` 前进/后退，`y` 左/右平移，`theta` 相对旋转（度）。 |

## 发布

| Topic | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令，`linear.x/y` 为机身前后/左右速度，`angular.z` 为旋转速度。 |
| `/cmd_gait` | `std_msgs/String` | 步态指令，首次收到里程计时发布一次（默认 `slow`）。 |

## 依赖

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `launch_ros`（用于 launch 文件）

## 编译

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select pose_control --symlink-install
```

## 启动

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/2026YuYaoGuoSai/lite3_ws/install/setup.bash

# 方式 1：ros2 run
ros2 run pose_control pose_control

# 方式 2：start_pose_control（可选启动 standalone 驱动）
ros2 run pose_control start_pose_control --use-driver

# 方式 3：launch
ros2 launch pose_control pose_control.launch.py
```

## /move 话题用法

`/move` 使用 `geometry_msgs/Pose2D`：

| 字段 | 单位 | 说明 |
|---|---|---|
| `x` | 米 | 沿机身前进方向移动距离，正为前进，负为后退。 |
| `y` | 米 | 沿机身左侧方向移动距离，正为左移，负为右移。 |
| `theta` | 度 | 相对程序启动原点的旋转角度，正为逆时针。 |

示例：

```bash
# 前进 0.5 m
ros2 topic pub /move geometry_msgs/Pose2D "{x: 0.5, y: 0.0, theta: 0.0}" --once

# 左移 0.1 m 并逆时针旋转 90°
ros2 topic pub /move geometry_msgs/Pose2D "{x: 0.0, y: 0.1, theta: 90.0}" --once
```

> 当前环境 `ros2cli==0.9.13` 异常，若命令行不可用，可写临时 Python 发布节点或修复 ros2cli。

## 终端命令（enable_terminal=true 时）

| 命令 | 功能 |
|---|---|
| `x+0.5` / `x-0.5` | 前进/后退 0.5 m |
| `y+0.1` / `y-0.1` | 左/右平移 0.1 m |
| `yaw90` 或 `90` | 旋转到相对程序启动原点 90° |
| `c` | 取消当前运动并清零速度 |
| `r` | 重置程序启动原点为当前位姿 |
| `q` | 退出节点 |
| `h` | 显示帮助 |

## 关键参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `kp_dist` | `float` | `1.0` | 前后位置 P 增益 |
| `ki_dist` | `float` | `0.0` | 前后位置 I 增益（默认关闭） |
| `kp_lateral` | `float` | `1.0` | 横向修正 P 增益 |
| `ki_lateral` | `float` | `0.0` | 横向修正 I 增益（默认关闭） |
| `kp_yaw` | `float` | `2.0` | 航向 P 增益 |
| `kd_yaw` | `float` | `0.3` | 航向 D 增益 |
| `ki_yaw` | `float` | `0.0` | 航向 I 增益（默认关闭） |
| `max_vel_x` | `float` | `0.3` | 最大前后速度（m/s） |
| `max_vel_y` | `float` | `0.2` | 最大左右速度（m/s） |
| `max_vel_yaw` | `float` | `1.6` | 最大旋转速度（rad/s） |
| `dist_threshold` | `float` | `0.05` | 位置到位阈值（m） |
| `yaw_threshold` | `float` | `0.05` | 航向到位阈值（rad） |
| `obstacle_stop_dist` | `float` | `0.35` | 超声波触发停止距离（m） |
| `enable_terminal` | `bool` | `true` | 是否开启终端输入 |
| `move_topic` | `string` | `/move` | 外部目标话题名 |

修改参数示例：

```bash
ros2 run pose_control pose_control --ros-args -p obstacle_stop_dist:=0.30 -p enable_terminal:=false
```

## 坐标约定

- `+x`：机身前进方向。
- `+y`：机身左侧。
- `+yaw`：逆时针方向（与官方 ROS 约定一致，`angular.z > 0` 增加 yaw）。
- 世界坐标系原点：首次收到 `/leg_odom2` 时的位姿（可用 `r` 重置）。

## 实现说明

- `/move` 指令先执行位置移动，到位后再执行旋转（若 `theta` 非零）。
- 位置阶段复用 `moving_x` 状态机，机身坐标系下的 `(x, y)` 会转换到里程计坐标系下的目标点。
- 后向超声波通过 `/us_publisher/ultrasound_distance` 订阅，参数 `sonar_is_rear` 控制方向。
- 前向避障当前未接入，待后续深度相机实现。

## 调试

若需要关闭终端输入并仅通过 `/move` 控制：

```bash
ros2 run pose_control pose_control --ros-args -p enable_terminal:=false
```
