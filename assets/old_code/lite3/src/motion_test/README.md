# Lite3 ROS2 测试驱动使用说明

## 运行使用方法

### 前提

- 已安装 ROS2（脚本基于 `rclpy`）
- 本机与机器人处于同一局域网，机器人 IP 为 `192.168.1.120`
- 运行前确保没有其他程序占用 UDP 端口 `43893`（legodom 版会 bind 该端口）
- 使用 ROS2 Foxy 时，建议新开一个终端并只 source `/opt/ros/foxy/setup.bash`，避免与 ROS1 Noetic 等环境混用

### 启动驱动节点

```bash
# 进入项目目录
cd /home/fishros/lite3/src/test

# 方案 A：仅运动控制（不发里程计）
python3 lite3_ros2_driver.py

# 方案 B：运动控制 + 里程计反馈
python3 lite3_ros2_driver_legodom.py
```

### 控制机器人运动

```bash
# 前进，linear.x = 0.3


# 旋转，angular.z = 0.5
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once

# 停止
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### 切换步态

```bash
# 切换为平地中速
ros2 topic pub /cmd_gait std_msgs/String "{data: 'medium'}" --once

# 切换为高踏步越障
ros2 topic pub /cmd_gait std_msgs/String "{data: 'stair'}" --once
```

### 查看里程计（仅 legodom 版）

```bash
ros2 topic echo /leg_odom2
```

### 安全退出

在运行驱动的终端按 `Ctrl+C`，脚本会自动执行趴下、切手动、关 socket 的安全流程。
