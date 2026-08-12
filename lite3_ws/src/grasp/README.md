# grasp 抓取全流程

Lite3 机械狗 + 机械臂的抓取-搬运-放置全流程 ROS2 包集合。

```
grasp/
├── apriltag_place1/    抓取对齐：AprilTag 视觉定位，对齐完成发 /grasp/start
├── grasp_task/         机械臂抓取/放置 8 阶段状态机
├── letter_place_align/ 放置对齐：A4 纸字母识别对齐，对齐完成发 /grasp/place
└── grasp_flow/         全流程编排器 + 一键 launch（新增）
```

## 全链路信号流

```
lite3_driver 启动 → 狗自动唤醒进入自主模式（回零→站立→运动模式→0x21010C03）
grasp_task 启动  → 机械臂自动摆准备姿态 → /grasp/state = STANDBY
grasp_flow       → 发 /apriltag_place1/start
apriltag_place1  → AprilTag 对齐完成 → /grasp/start
grasp_task       → 检测→对齐→抓取 → /grasp/state = TRANSPORT（运输姿态）
【人工搬运机械狗到放置点】
grasp_flow       → 命令行输入放置字母(A/B/C/D) → 发 /letter_place/start
letter_place_align → 字母对齐完成 → /grasp/place
grasp_task       → 放置 → /grasp/state = DONE + /grasp/result = True
```

两个对齐节点（apriltag_place1 / letter_place_align）由编排器**按需拉起与关闭**，
保证摄像头（yaml 目前均为 `/dev/video6`）与 `/move` 指令总线任意时刻只有一个占用。

---

## 编译

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select apriltag_place1 letter_place_align grasp_task grasp_flow
source install/setup.bash
```

---

## 一、全链路一键启动

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch grasp_flow grasp_flow_b.launch.py

ros2 launch grasp_flow grasp_flow.launch.py



```

启动后自动执行：狗唤醒进自主模式 → 机械臂准备姿态 → AprilTag 抓取对齐 → 抓取。
抓取完成后终端提示：

```
搬运到位后，在此终端输入放置字母 A/B/C/D 并回车开始放置对齐（输入 q 中止任务）
```

人工把狗搬到放置点，在**同一终端**输入字母（如 `B`）回车 → 自动放置对齐 → 放置 → 全流程结束。

### launch 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `start_dog_driver` | `true` | 自动拉起 lite3_driver（启动即唤醒狗）；狗驱动外部已启动则设 `false` |
| `dog_driver_path` | `/home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py` | 狗驱动脚本路径 |
| `dry_run` | `false` | `true` 时 grasp_task 跳过真实机械臂/摄像头，仅通信链路测试 |

示例：

```bash
ros2 launch grasp_flow grasp_flow.launch.py dry_run:=true                    # 无机械臂通信测试
ros2 launch grasp_flow grasp_flow.launch.py start_dog_driver:=false          # 狗驱动另行启动
```

### 运行中的人工干预

- 放置对齐/放置阶段输入 `q` 回车：取消 letter_place_align，回到等待字母输入状态。
- 放置超时进入 ERROR 后输入 `r` 回车：重新触发放置对齐（grasp_task 本身已报错则只能重启）。

---

## 二、单部分独立启动测试

> 以下每个终端都先执行：
> ```bash
> cd /home/ysc/2026YuYaoGuoSai/lite3_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash
> ```

### 1. 机械狗驱动（自动模式）

```bash
python3 /home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py
```

启动即自动执行唤醒序列：回零 → 站立 → 运动模式 → 自主模式（0x21010C03）。
发布 `/leg_odom2`，订阅 `/cmd_vel`、`/cmd_gait`、`/emergency_stop`。
Ctrl+C 退出时自动执行落地关机序列。

### 2. 位置环控制器 pose_control

```bash
ros2 launch pose_control pose_control.launch.py
```

订阅 `/move`(Pose2D，x 前进/y 左移/theta 度数)、`/pose_control/command`（`reset_origin`），发布 `/cmd_vel`。
默认会以 10Hz 打印状态到终端；加 `-p show_display:=false` 可关闭（grasp_flow 一键 launch 已默认关闭）。

手动测试走 0.2m：

```bash
ros2 topic pub /pose_control/command std_msgs/String "data: 'reset_origin'" --once
ros2 topic pub /move geometry_msgs/Pose2D "{x: 0.2, y: 0.0, theta: 0.0}" --once
ros2 topic echo /cmd_vel
```

### 3. 机械臂 grasp_task

```bash
ros2 launch grasp_task grasp.launch.py                 # 真机
ros2 launch grasp_task grasp.launch.py dry_run:=true   # 无硬件通信测试
```

启动后自动 INIT → STANDBY（准备姿态），之后：

```bash
# 监视状态
ros2 topic echo /grasp/state
ros2 topic echo /grasp/result

# 触发抓取（对齐完成信号，模拟 apriltag_place1 输出）
ros2 topic pub /grasp/start std_msgs/Bool "{data: true}" -r 1 -t 3

# 触发放置（模拟 letter_place_align 输出，zone=A/B/C/D）
ros2 topic pub /grasp/place std_msgs/String "{data: 'B'}" -r 1 -t 3

# 只设区域不触发
ros2 topic pub /grasp/set_zone std_msgs/String "{data: 'B'}" --once

# 紧急停止
ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}" --once
```

状态序列：`INIT → STANDBY → DETECTING → ALIGNING → GRASPING → TRANSPORT → PLACING → DONE`，
失败为 `ERROR:<原因>`。节点单轮运行，重测需重启节点。

### 4. 抓取对齐 apriltag_place1

```bash
ros2 launch apriltag_place1 apriltag_place1.launch.py   # 自带 pose_controller
```

```bash
# 触发对齐
ros2 topic pub /apriltag_place1/start std_msgs/Bool "{data: true}" --once
# 取消回待触发
ros2 topic pub /apriltag_place1/start std_msgs/Bool "{data: false}" --once
```

成功：状态走到 `done` 并自动发 `/grasp/start`（可用 `ros2 topic echo /grasp/start` 验证）。
参数在 `src/grasp/apriltag_place1/config/apriltag_place1.yaml`（摄像头 `/dev/video6`、Tag ID 0、站位距离等）。

> 注意：节点带 cv2 显示窗口，无显示环境（纯 SSH 无 X 转发）会因 imshow 崩溃。


## 三、常用监视命令

```bash
ros2 topic echo /grasp/state          # grasp_task 状态（1Hz 心跳重发）
ros2 topic echo /grasp/result         # 最终结果
ros2 topic echo /leg_odom2            # 里程计
ros2 node list                        # 节点清单
```

# 机械臂 USB 摄像头（block_align / grasp_task DETECTING 用的）
ffplay -f v4l2 -framerate 30 -video_size 640x480 /dev/video0

# RealSense D435i RGB（apriltag_place1 / letter_place_align 用的）
ffplay -f v4l2 -framerate 30 -video_size 640x480 /dev/video6


cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select abcd_task apriltag_place1 block_align grasp_task
source install/setup.bash

# 全流程（ABCD 四轮）
ros2 launch abcd_task abcd_task.launch.py

# 仅测导航链路（不接机械臂/视觉）
ros2 launch abcd_task abcd_task.launch.py dry_run:=true dry_run_nav:=true

# 从 C 开始（跳过 AB）
ros2 launch abcd_task abcd_task.launch.py start_from:=C max_rounds:=2

# 单字母测试
ros2 launch abcd_task abcd_task.launch.py start_from:=A max_rounds:=1
