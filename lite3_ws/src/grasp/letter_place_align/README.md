# letter_place_align

放置区字母对齐节点：机械狗携带方块到达放置区附近后，用头部摄像头识别纸箱 A4 纸上的
目标字母（A/B/C/D），以纸张轮廓为几何参照完成"正对字母"对齐，最终站位与抓取前
AprilTag 对齐站位一致，然后向 grasp_task 发布 `/grasp/place` 下发放置指令。

设计文档：`doc/2026-08-01-letter-place-align-design.md`

> 本包状态机骨架克隆自已真机验证的 `apriltag_place1`（设计 §6 取舍：赛后如需第三处
> 对齐场景再合并抽象）。核心差异只在检测层（A4 轮廓 + 框内 OCR，替代 AprilTag）与
> approach/盲进段（视觉闭环止步 `vision_min_distance_m`，剩余距离开环盲进）。

## 接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| sub | `/letter_place/start` | std_msgs/String | 触发信号，data ∈ {A,B,C,D}（人工发布）；其他值视为取消。节点同时挂 volatile + transient_local 双订阅，闩锁发布也能收到 |
| sub | `/cmd_vel` | geometry_msgs/Twist | 运动完成判定（零速检测） |
| sub | `/leg_odom2` | nav_msgs/Odometry | 运动链路预检 |
| pub | `/move` | geometry_msgs/Pose2D | 运动指令（x 前进 / y 左移 / theta 逆时针，角度制） |
| pub | `/pose_control/command` | std_msgs/String | `reset_origin` |
| pub | `/grasp/place` | std_msgs/String | 对齐完成，携带目标字母，对接 grasp_task PLACING 阶段 |

## 检测与位姿

- 检测函数 `detect_letter_papers()` 在 `tools/gauge_yolo_new.py`（sys.path 方式 import，
  同 gauge_yolo_detector 的做法），修改时请同步检查本节点。
- 两级设计：A4 轮廓管几何（中心像素 → 角度/横向，纸高像素 → 距离），OCR 只回答身份。
- OCR 异步投票：结果取多数决（≥2 票一致才采信，防单次错认锁死），
  未达票数持续异步重认；几何逐帧以最新轮廓为准。
- 距离反推：`tz = fy * H_paper / h_px`，H_paper 由 `paper_orientation` 取 0.297 / 0.210 m。

## 启动

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select letter_place_align --symlink-install
source install/setup.bash

# 终端 1：grasp（已在 PLACING 阶段等待 /grasp/place）
ros2 launch grasp_task grasp.launch.py

# 终端 2：字母对齐（launch 含 pose_control）
ros2 launch letter_place_align letter_place_align.launch.py

# 终端 3：狗到位后人工触发（携带目标字母）
# 推荐：闩锁发布 + 驻留 3s，DDS 发现完成后仍能送达（Foxy 的 --once 不等订阅匹配，
# 裸用大概率丢消息）
ros2 topic pub /letter_place/start std_msgs/String "data: 'B'" --once \
    --qos-durability transient_local --keep-alive 3

# 备选：1Hz 连发 3 次（重复发同字母无副作用，对齐中会被忽略）
ros2 topic pub /letter_place/start std_msgs/String "data: 'B'" -r 1 -t 3

# 需要中止时发任意非 ABCD 的值（视为取消，狗停下回到等待态）
ros2 topic pub /letter_place/start std_msgs/String "data: 'X'" -r 1 -t 3
```

真机前置步骤（官方栈 source + `/simple_cmd` 序列）与 apriltag_place1 完全相同。

## 运行期日志

节点各状态均有节流心跳输出（约 2~3s 一条），终端无输出即为异常：

- `wait_trigger`：周期提示等待触发及触发命令；
- `wait_detect`：候选纸数量、各候选 OCR 投票结果、稳定缓冲进度、
  轮廓过滤统计（面积/四边形/长宽比/齐平各拒绝多少，定位"检不出"卡在哪一关）；
- 对齐/盲进阶段：运动等待心跳；`final_check` 未达标原因（yaw/横向/距离哪项超差）。

## 内参标定（距离精度的前提）

yaml 中 `camera_matrix` 若与实际头摄 RealSense RGB 流（/dev/video4，枚举可能漂移）
不符时，`tz = fy * H / h_px` 会按比例整体偏差。上机后用已知距离反解 fy：

```bash
# A4 纸正对摄像头贴好，卷尺量纸面到镜头距离（建议 0.5/0.8/1.0m 测 2~3 次取平均）
python3 /home/ysc/2026YuYaoGuoSai/tools/letter_fy_calib.py --distance 0.5
# 把打印的 camera_matrix 写入 config/letter_place_align.yaml，重新 launch
```

## 待标定

- `paper_orientation`：A4 竖贴 portrait / 横贴 landscape（决定距离反推用哪条边）。
  注意与实际贴纸方向一致，否则距离差 1.414 倍。
- 头部摄像头内参：用 `tools/letter_fy_calib.py` 实测 fy（见上节）。
- `letter_offset_x_m`：字母不居中印刷时补偿横向偏移。
