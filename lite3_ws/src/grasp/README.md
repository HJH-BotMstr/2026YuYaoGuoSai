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

### 5. 放置对齐 letter_place_align

```bash
ros2 launch letter_place_align letter_place_align.launch.py   # 自带 pose_controller
```

```bash
# 触发对齐（字母 A/B/C/D；transient_local 或连发 3 次二选一）
ros2 topic pub /letter_place/start std_msgs/String "data: 'B'" --once --qos-durability transient_local --keep-alive 3
ros2 topic pub /letter_place/start std_msgs/String "data: 'B'" -r 1 -t 3

# 取消（任意非 ABCD 值）
ros2 topic pub /letter_place/start std_msgs/String "data: 'X'" -r 1 -t 3
```

成功：状态走到 `done` 并自动发 `/grasp/place`（`ros2 topic echo /grasp/place` 验证）。
参数在 `src/grasp/letter_place_align/config/letter_place_align.yaml`（摄像头设备、`paper_orientation`、站位参数需与 apriltag_place1 一致）。
完成后节点停 `done` 态不可重触发，重测需重启节点。

### 6. 编排器 grasp_flow（单独）

```bash
ros2 run grasp_flow grasp_flow_node
ros2 run grasp_flow grasp_flow_node --ros-args -p apriltag_timeout_s:=300.0
```

单独跑时前提：狗驱动、pose_control、grasp_task 已在运行（编排器按状态自动推进，
两个对齐节点由它按需拉起）。主要参数：`odom_fresh_timeout_s`、`apriltag_timeout_s`(240)、
`grasp_timeout_s`(300)、`letter_place_timeout_s`(600)、`enable_prompt`(true)、`manage_align_nodes`(true)。

---

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

^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
[pose_control-2] Traceback (most recent call last):
[pose_control-2]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/pose_control/lib/pose_control/pose_control", line 11, in <module>
[pose_control-2]     load_entry_point('pose-control==0.0.0', 'console_scripts', 'pose_control')()
[pose_control-2]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/pose_control/lib/python3.8/site-packages/pose_control/pose_controller_node.py", line 754, in main
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
^C[ERROR] [grasp_node-3]: process has died [pid 11598, exit code -2, cmd '/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_task/lib/grasp_task/grasp_node --ros-args -r __node:=grasp_task --params-file /home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_task/share/grasp_task/config/grasp_task.yaml --params-file /tmp/launch_params_0f6s_ywg'].
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
[ERROR] [pose_control-2]: process has died [pid 11596, exit code -2, cmd '/home/ysc/2026YuYaoGuoSai/lite3_ws/install/pose_control/lib/pose_control/pose_control --ros-args -r __node:=pose_controller --params-file /tmp/launch_params_4hbjiwe_'].
[pose_control-2] state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.82°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.82°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.83°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.84°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.84°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.85°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.86°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=False rear_dist=0.60m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.87°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=idle      origin=(  -0.79°)  cur=(3.016,-0.434,  -2.87°)
cmd=(+0.000,+0.000,+0.0000)  rear_blocked=True rear_dist=0.28m  [idle]
state=moving_x  origin=(  -3.66°)  cur=(3.015,-0.433,  -0.15°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+1.100,-0.050,+0.0050)  rear_blocked=False rear_dist=0.60m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(3.164,-0.451,  -0.26°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+1.100,+0.050,+0.0069)  rear_blocked=True rear_dist=0.28m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(3.434,-0.462,  -0.28°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+1.100,+0.050,+0.0095)  rear_blocked=False rear_dist=0.51m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(3.702,-0.467,  -0.13°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+1.100,-0.050,+0.0089)  rear_blocked=False rear_dist=0.80m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(3.950,-0.494,   0.15°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.978,+0.050,-0.0052)  rear_blocked=False rear_dist=0.94m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.087,-0.462,   1.11°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.728,-0.084,-0.0343)  rear_blocked=False rear_dist=1.22m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.132,-0.484,   1.81°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.643,-0.050,-0.0633)  rear_blocked=False rear_dist=1.04m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.191,-0.519,   4.38°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.533,+0.050,-0.1358)  rear_blocked=False rear_dist=1.01m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.235,-0.538,   6.48°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.446,+0.050,-0.2264)  rear_blocked=False rear_dist=1.08m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.268,-0.543,   7.12°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.384,+0.057,-0.2487)  rear_blocked=False rear_dist=1.09m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.287,-0.541,   7.82°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.348,+0.051,-0.2732)  rear_blocked=False rear_dist=1.09m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.309,-0.519,   7.38°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.311,+0.050,-0.2578)  rear_blocked=False rear_dist=1.10m  [auto]
state=moving_x  origin=(  -3.66°)  cur=(4.315,-0.499,   9.30°)  tgt=(4.345,-0.519,   3.66°)
cmd=(+0.301,-0.050,-0.3248)  rear_blocked=False rear_dist=1.09m  [auto]

[pose_control-2] Traceback (most recent call last):
  File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/pose_control/lib/pose_control/pose_control", line 11, in <module>
    load_entry_point('pose-control==0.0.0', 'console_scripts', 'pose_control')()
  File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/pose_control/lib/python3.8/site-packages/pose_control/pose_controller_node.py", line 754, in main
    node.destroy_node()
  File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/node.py", line 1533, in destroy_node
    self.handle.destroy()
  File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/handle.py", line 95, in destroy
    self.__destroy()
  File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/handle.py", line 138, in __destroy
    _rclpy_capsule.rclpy_pycapsule_destroy(self.__capsule)
KeyboardInterrupt

[ERROR] [python3-1]: process has died [pid 11594, exit code -2, cmd 'python3 /home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py'].
[grasp_flow_node_b-4] Traceback (most recent call last):
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/python3.8/site-packages/grasp_flow/grasp_flow_node_b.py", line 392, in main
[grasp_flow_node_b-4]     rclpy.spin(node)
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/__init__.py", line 191, in spin
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py", line 717, in spin_once
[grasp_flow_node_b-4]     handler()
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/task.py", line 239, in __call__
[grasp_flow_node_b-4]     self._handler.send(None)
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py", line 422, in handler
[grasp_flow_node_b-4]     arg = take_from_wait_list(entity)
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py", line 347, in _take_subscription
[grasp_flow_node_b-4]     msg_info = _rclpy.rclpy_take(capsule, sub.msg_type, sub.raw)
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/geometry_msgs/msg/_pose_with_covariance.py", line 77, in __init__
[grasp_flow_node_b-4]     def __init__(self, **kwargs):
[grasp_flow_node_b-4] KeyboardInterrupt
[grasp_flow_node_b-4] 
[grasp_flow_node_b-4] During handling of the above exception, another exception occurred:
[grasp_flow_node_b-4] 
[grasp_flow_node_b-4] Traceback (most recent call last):
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/python3.8/site-packages/grasp_flow/grasp_flow_node_b.py", line 394, in main
[grasp_flow_node_b-4]     node.get_logger().info("收到 Ctrl+C，正在安全退出...")
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/impl/rcutils_logger.py", line 334, in info
[grasp_flow_node_b-4]     return self.log(message, LoggingSeverity.INFO, **kwargs)
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/impl/rcutils_logger.py", line 290, in log
[grasp_flow_node_b-4]     caller_id = CallerId()
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/impl/rcutils_logger.py", line 58, in __new__
[grasp_flow_node_b-4]     frame = _find_caller(inspect.currentframe())
[grasp_flow_node_b-4]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/impl/rcutils_logger.py", line 49, in _find_caller
[grasp_flow_node_b-4]     file_path = os.path.realpath(inspect.getframeinfo(frame).filename)
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/posixpath.py", line 391, in realpath
[grasp_flow_node_b-4]     path, ok = _joinrealpath(filename[:0], filename, {})
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/posixpath.py", line 425, in _joinrealpath
[grasp_flow_node_b-4]     if not islink(newpath):
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/posixpath.py", line 167, in islink
[grasp_flow_node_b-4]     st = os.lstat(path)
[grasp_flow_node_b-4] KeyboardInterrupt
[grasp_flow_node_b-4] 
[grasp_flow_node_b-4] During handling of the above exception, another exception occurred:
[grasp_flow_node_b-4] 
[grasp_flow_node_b-4] Traceback (most recent call last):
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/subprocess.py", line 1083, in wait
[grasp_flow_node_b-4]     return self._wait(timeout=timeout)
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/subprocess.py", line 1800, in _wait
[grasp_flow_node_b-4]     time.sleep(delay)
[grasp_flow_node_b-4] KeyboardInterrupt
[grasp_flow_node_b-4] 
[grasp_flow_node_b-4] During handling of the above exception, another exception occurred:
[grasp_flow_node_b-4] 
[grasp_flow_node_b-4] Traceback (most recent call last):
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/grasp_flow/grasp_flow_node_b", line 11, in <module>
[grasp_flow_node_b-4]     load_entry_point('grasp-flow==0.1.0', 'console_scripts', 'grasp_flow_node_b')()
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/python3.8/site-packages/grasp_flow/grasp_flow_node_b.py", line 396, in main
[grasp_flow_node_b-4]     node.destroy_node()
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/python3.8/site-packages/grasp_flow/grasp_flow_node_b.py", line 383, in destroy_node
[grasp_flow_node_b-4]     self._kill_all()
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/python3.8/site-packages/grasp_flow/grasp_flow_node_b.py", line 231, in _kill_all
[grasp_flow_node_b-4]     self._kill(key)
[grasp_flow_node_b-4]   File "/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/python3.8/site-packages/grasp_flow/grasp_flow_node_b.py", line 221, in _kill
[grasp_flow_node_b-4]     proc.wait(timeout=5.0)
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/subprocess.py", line 1096, in wait
[grasp_flow_node_b-4]     self._wait(timeout=sigint_timeout)
[grasp_flow_node_b-4]   File "/usr/lib/python3.8/subprocess.py", line 1800, in _wait
[grasp_flow_node_b-4]     time.sleep(delay)
[grasp_flow_node_b-4] KeyboardInterrupt
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
[ERROR] [grasp_flow_node_b-4]: process has died [pid 11600, exit code -2, cmd '/home/ysc/2026YuYaoGuoSai/lite3_ws/install/grasp_flow/lib/grasp_flow/grasp_flow_node_b --ros-args -r __node:=grasp_flow_node_b'].
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
[ERROR] [grasp_node-3]: process[grasp_node-3] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
[ERROR] [grasp_node-3]: process[grasp_node-3] failed to terminate '10.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...



