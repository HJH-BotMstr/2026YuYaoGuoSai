#!/usr/bin/env python3
"""
grasp_task 主节点：把 tools/grasp 的 8-phase 抓取流程封装为 ROS2 节点。

对外接口：
  sub /grasp/start  (std_msgs/Bool)   : 启动抓取流程
  sub /grasp/place  (std_msgs/String): 触发放置，携带 A/B/C/D
  sub /grasp/set_zone (std_msgs/String): 仅设置目标放置区
  sub /cmd_vel      (geometry_msgs/Twist): 判断 pose_control 是否到位
  pub /grasp/state  (std_msgs/String): 当前状态
  pub /grasp/result (std_msgs/Bool) : 最终成功/失败
  pub /move         (geometry_msgs/Pose2D): 横向对齐指令
  pub /pose_control/command (std_msgs/String): 如 reset_origin
"""
import sys
import os
import signal
import time
import threading
import logging

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool as BoolMsg, String
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

# 将 tools/grasp 加入 Python 搜索路径，以便复用 ArmController / BlockDetection 等
TOOLS_GRASP = "/home/ysc/2026YuYaoGuoSai/tools/grasp"
if TOOLS_GRASP not in sys.path:
    sys.path.insert(0, TOOLS_GRASP)

from utils.ArmController import ArmController
from utils.BlockDetection import BlockDetection
from utils.TargetTracker import TargetTracker
from utils.InspectionMemory import InspectionMemory

from .config_loader import load_config
from .motion_waiter import MotionWaiter


VALID_ZONES = {"A", "B", "C", "D"}


def _open_camera(device: str, retries: int = 3, delay: float = 1.0, logger=None):
    """尝试打开摄像头，失败时重试；返回 VideoCapture 或 None。"""
    for attempt in range(1, retries + 1):
        if logger:
            logger.info("尝试打开摄像头: %s (第 %d/%d 次)" % (device, attempt, retries))
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                if logger:
                    logger.info("摄像头打开成功: %s, 帧大小=%s" % (device, frame.shape))
                return cap
            else:
                if logger:
                    logger.warning("摄像头能打开但无法读帧，尝试重新打开")
                cap.release()
        else:
            if logger:
                logger.warning("摄像头打开失败: %s" % (device))
        if attempt < retries:
            time.sleep(delay)
    return None


class GraspTaskNode(Node):
    """ROS2 抓取任务节点。"""

    def __init__(self):
        super().__init__("grasp_task")

        # 声明参数
        self.declare_parameter("tools_config_path",
                               "/home/ysc/2026YuYaoGuoSai/tools/grasp/config.yaml")
        self.declare_parameter("start_topic", "/grasp/start")
        self.declare_parameter("place_topic", "/grasp/place")
        self.declare_parameter("set_zone_topic", "/grasp/set_zone")
        self.declare_parameter("state_topic", "/grasp/state")
        self.declare_parameter("result_topic", "/grasp/result")
        self.declare_parameter("move_topic", "/move")
        self.declare_parameter("command_topic", "/pose_control/command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/leg_odom2")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("cv_show", False)
        self.declare_parameter("motion_stop_timeout_s", 15.0)
        self.declare_parameter("motion_stop_zero_duration_s", 0.5)
        self.declare_parameter("odom_fresh_timeout_s", 0.5)
        # 多轮循环：max_rounds=1 时保持原有单轮行为；abcd_task 会覆写为 4。
        # inter_round_wait_s 是两轮之间的短暂间隔，让 latched 消息/上层编排刷新。
        self.declare_parameter("max_rounds", 1)
        self.declare_parameter("inter_round_wait_s", 0.5)

        # 加载配置并强制 robot 模式
        self.cfg = load_config(self)
        self.dry_run_param = self.get_parameter("dry_run").value
        # launch 参数可能以字符串传入，统一转成 bool
        if isinstance(self.dry_run_param, str):
            self.dry_run = self.dry_run_param.strip().lower() in ("true", "1", "yes")
        else:
            self.dry_run = bool(self.dry_run_param)
        self.cv_show = self.get_parameter("cv_show").value

        self._lock = threading.Lock()
        self._start_event = threading.Event()
        self._place_event = threading.Event()
        self._target_zone: str = None
        self._estop = False
        self._odom_fresh = False
        self._last_odom_time = 0.0

        # topic 名称
        self._start_topic = self.get_parameter("start_topic").value
        self._place_topic = self.get_parameter("place_topic").value
        self._set_zone_topic = self.get_parameter("set_zone_topic").value
        self._state_topic = self.get_parameter("state_topic").value
        self._result_topic = self.get_parameter("result_topic").value
        self._move_topic = self.get_parameter("move_topic").value
        self._command_topic = self.get_parameter("command_topic").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value

        motion_timeout = self.get_parameter("motion_stop_timeout_s").value
        motion_zero_dur = self.get_parameter("motion_stop_zero_duration_s").value
        self._motion_waiter = MotionWaiter(
            zero_duration_s=motion_zero_dur,
            timeout_s=motion_timeout,
        )

        # 订阅
        self.create_subscription(BoolMsg, self._start_topic, self._on_start, 10)
        self.create_subscription(String, self._place_topic, self._on_place, 10)
        self.create_subscription(String, self._set_zone_topic, self._on_zone, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(BoolMsg, "/emergency_stop", self._on_estop, 10)

        # 发布
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self._result_pub = self.create_publisher(BoolMsg, self._result_topic, 10)
        self._move_pub = self.create_publisher(Pose2D, self._move_topic, 10)
        self._cmd_pub = self.create_publisher(String, self._command_topic, 10)

        # 状态心跳：state 话题为 volatile，晚订阅的节点（如 grasp_flow 编排器）
        # 会错过一次性状态跳变，1Hz 重发当前状态兜底
        self._current_state = "BOOT"
        self.create_timer(1.0, self._republish_state)

        # 初始化硬件
        self.arm = None
        self.detector = None
        self.tracker = None
        self.memory = None
        self.arm_cam = None
        try:
            self._init_hardware()
        except Exception as e:
            self.get_logger().error("硬件初始化失败: %s" % (e,))
            self._publish_state("ERROR:HW_INIT_FAILED")
            self._result_pub.publish(BoolMsg(data=False))
            raise

        self.get_logger().info(
            "grasp_task 节点已初始化，等待 %s 信号 (dry_run=%s)" % (self._start_topic, self.dry_run)
        )

    # ------------------------------------------------------------------ #
    # ROS 回调
    # ------------------------------------------------------------------ #
    def _on_start(self, msg: BoolMsg):
        if msg.data:
            self.get_logger().info("收到 /grasp/start 信号")
            self._start_event.set()

    def _on_place(self, msg: String):
        zone = msg.data.upper()
        if zone in VALID_ZONES:
            self.memory.set_zone(zone)
            self.get_logger().info("收到 /grasp/place 信号，zone=%s" % (zone))
            self._place_event.set()
        else:
            self.get_logger().warn("收到无效放置区: %s" % (msg.data))

    def _on_zone(self, msg: String):
        zone = msg.data.upper()
        if zone in VALID_ZONES:
            self.memory.set_zone(zone)
            self.get_logger().info("通过 /grasp/set_zone 设置 zone=%s" % (zone))
        else:
            self.get_logger().warn("收到无效放置区: %s" % (msg.data))

    def _on_cmd_vel(self, msg: Twist):
        self._motion_waiter.on_cmd_vel(msg)

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._last_odom_time = time.monotonic()
            self._odom_fresh = True

    def _on_estop(self, msg: BoolMsg):
        if msg.data:
            self.get_logger().error("收到急停信号")
            with self._lock:
                self._estop = True

    # ------------------------------------------------------------------ #
    # 硬件初始化
    # ------------------------------------------------------------------ #
    def _init_hardware(self):
        """初始化机械臂、摄像头、检测器、跟踪器和记忆模块。"""
        cfg = self.cfg
        if self.dry_run:
            self.get_logger().info("dry_run=True，跳过机械臂和摄像头初始化")
            # 即使 dry_run 也创建检测器/跟踪器/记忆，以便后续流程可复用
            self.detector = BlockDetection({**cfg["detection"]})
            cfg_g = cfg["grasp"]
            self.tracker = TargetTracker(
                avg_window=int(cfg_g["distance_avg_window"]),
                lost_frames_max=int(cfg_g["lost_frames_max"]),
            )
            self.memory = InspectionMemory(default_zone=cfg["inspection"]["default_zone"])
            return

        try:
            self.arm = ArmController(
                device=cfg["hardware"]["arm_serial_port"],
                cfg={**cfg["arm"], "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]},
            )
        except Exception as e:
            self.get_logger().error("机械臂初始化失败: %s" % (e))
            raise

        self.detector = BlockDetection({**cfg["detection"]})
        cfg_g = cfg["grasp"]
        self.tracker = TargetTracker(
            avg_window=int(cfg_g["distance_avg_window"]),
            lost_frames_max=int(cfg_g["lost_frames_max"]),
        )
        self.memory = InspectionMemory(default_zone=cfg["inspection"]["default_zone"])

        # 摄像头懒打开：STANDBY 时不占用 /dev/video0，把设备让给 block_align 对齐节点，
        # 进入 DETECTING 阶段再 open，抓完 release
        self._cam_device = cfg["hardware"]["arm_cam_device"]

    def _ensure_arm_cam_open(self):
        if self.dry_run:
            return
        if self.arm_cam is not None:
            return
        self.arm_cam = _open_camera(self._cam_device, logger=self.get_logger())
        if self.arm_cam is None:
            raise RuntimeError(f"机械臂摄像头打开失败: {self._cam_device}")

    def _release_arm_cam(self):
        if self.arm_cam is None:
            return
        try:
            self.arm_cam.release()
        except Exception as e:
            self.get_logger().warning("释放摄像头时异常: %s" % (e,))
        self.arm_cam = None

    # ------------------------------------------------------------------ #
    # 工具方法
    # ------------------------------------------------------------------ #
    def _publish_state(self, state: str):
        self._current_state = state
        self._state_pub.publish(String(data=state))
        self.get_logger().info("state -> %s" % (state))

    def _republish_state(self):
        self._state_pub.publish(String(data=self._current_state))

    def _check_estop(self) -> bool:
        with self._lock:
            return self._estop

    def _fail(self, reason: str) -> bool:
        self.get_logger().error("任务失败: %s" % (reason))
        self._publish_state(f"ERROR:{reason}")
        self._result_pub.publish(BoolMsg(data=False))
        return False

    def _reset_command(self):
        """重置 pose_control 原点，避免多次 /move 漂移。"""
        self._cmd_pub.publish(String(data="reset_origin"))
        self.get_logger().info("发布 %s: reset_origin" % (self._command_topic))
        time.sleep(0.2)

    # ------------------------------------------------------------------ #
    # 视觉检测
    # ------------------------------------------------------------------ #
    def _detect_stable(self) -> dict:
        """
        多帧检测，TargetTracker 滑动均值稳定后返回稳定目标读数。
        返回 dict 或 None（超时）。
        """
        if self.dry_run:
            self.get_logger().info("dry_run: 模拟检测成功")
            return {
                "color": "red",
                "bbox": ((0, 0), (1, 1)),
                "center_offset_x": 0,
                "distance_mm": 300.0,
                "pos_3d": (0.0, 300.0, 0.0),
            }

        timeout = float(self.cfg["grasp"]["detect_timeout"])
        deadline = time.monotonic() + timeout
        self.get_logger().info("开始视觉识别，超时 %.1fs" % (timeout))

        while time.monotonic() < deadline:
            if self._check_estop():
                return None

            ret, frame = self.arm_cam.read()
            if not ret:
                self.get_logger().warning("摄像头读帧失败，跳过")
                continue

            candidates = self.detector.detect_all(frame)
            self.tracker.update(candidates)

            if self.cv_show:
                vis_result = candidates[0] if candidates else None
                vis = self.detector.visualize(frame.copy(), vis_result)
                cv2.imshow("arm_cam", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return None

            stable = self.tracker.get_stable_target()
            if stable is not None:
                self.get_logger().info(
                    "目标锁定稳定: dist=%.1fmm offset_x=%d" % (stable["distance_mm"], stable["center_offset_x"])
                )
                return stable

        self.get_logger().error("视觉识别超时 (%.1fs)，未检测到稳定色块" % (timeout))
        return None

    # ------------------------------------------------------------------ #
    # 横向对齐
    # ------------------------------------------------------------------ #
    def _align_laterally(self, stable: dict) -> bool:
        """
        根据 X_cam 偏移发布 /move，让 pose_control 驱动机器狗横向对齐。
        """
        cfg_g = self.cfg["grasp"]
        thr_mm = float(cfg_g["align_offset_threshold_mm"])
        max_rounds = 5
        X_cam = stable["pos_3d"][0]

        for round_i in range(max_rounds):
            if self._check_estop():
                return False

            if abs(X_cam) <= thr_mm:
                self.get_logger().info("横向已对齐: X_cam=%.1fmm (阈值=%.1fmm)" % (X_cam, thr_mm))
                return True

            # 重置 tracker，避免旧窗口影响
            self.tracker = TargetTracker(
                avg_window=int(cfg_g["distance_avg_window"]),
                lost_frames_max=int(cfg_g["lost_frames_max"]),
            )

            self._reset_command()
            self._motion_waiter.reset()

            # 发布横向移动：ROS 约定 Pose2D.y 正=左移；相机侧 X_cam>0 表示物块在图像/视野右侧，
            # 要把它拉到画面中心，狗需要向右移动 → y 取负号。
            # 与 block_align 的 lateral_polarity=-1 保持一致。
            msg = Pose2D(x=0.0, y=-X_cam / 1000.0, theta=0.0)
            self._move_pub.publish(msg)
            self.get_logger().info("第 %d 轮横向对齐: y=%.3fm" % (round_i + 1, msg.y))

            if not self._motion_waiter.wait_for_stop():
                self.get_logger().error("等待横向到位超时")
                return False

            new_stable = self._detect_stable()
            if new_stable is None:
                self.get_logger().error("对齐后重新识别失败")
                return False
            X_cam = new_stable["pos_3d"][0]
            stable.update(new_stable)

        self.get_logger().error("横向对齐超过最大轮次 (%d)，仍未对齐 X_cam=%.1fmm" % (max_rounds, X_cam))
        return False

    # ------------------------------------------------------------------ #
    # 接近与抓取
    # ------------------------------------------------------------------ #
    def _approach_and_grasp(self, stable: dict) -> bool:
        if self.dry_run:
            self.get_logger().info("dry_run: 模拟抓取成功")
            return True

        cfg_g = self.cfg["grasp"]
        arm = self.arm

        cv2.destroyAllWindows()

        clearance = float(cfg_g["approach_clearance_mm"])
        h_object = float(cfg_g["h_object"])
        dist_offset = float(cfg_g.get("distance_offset_mm", 0.0))

        X_cam, Y_cam, Z_cam = stable["pos_3d"]
        dis_target = Y_cam + dist_offset
        dis_safe = max(dis_target - clearance, 30.0)

        self.get_logger().info(
            "物块坐标（相机系）: X=%.1fmm Y=%.1fmm Z=%.1fmm" % (X_cam, Y_cam, Z_cam)
        )
        self.get_logger().info(
            "IK 输入: dis_safe=%.1fmm -> dis=%.1fmm, h=%.1fmm" % (dis_safe, dis_target, h_object)
        )

        from utils.RobotArm.three_Inverse_kinematics import Arm as IKArm

        # 步骤 1：移到安全距离，下降到目标高度
        self.get_logger().info("步骤1: dis=%.1fmm h=%.1fmm" % (dis_safe, h_object))
        ok = arm.grap(dis_safe, h_object)
        if not ok:
            self.get_logger().error("步骤1 IK 解超出范围 (dis=%.1f h=%.1f)" % (dis_safe, h_object))
            return False
        a3, a4, a5 = IKArm(dis_safe, h_object)
        arm.wait_for_position({3: a3, 4: a4, 5: a5})

        # 步骤 2：前进并抓取
        self.get_logger().info("步骤2: dis=%.1fmm h=%.1fmm" % (dis_target, h_object))
        success = arm.grasp_with_verify(dis=dis_target, height=h_object)
        if success:
            self.get_logger().info("抓取成功")
        else:
            self.get_logger().error("抓取失败（已重试 %s 次）" % (cfg_g.get("grasp_retry_max", 3)))
        return success

    # ------------------------------------------------------------------ #
    # 放置
    # ------------------------------------------------------------------ #
    def _place(self) -> bool:
        if self.dry_run:
            self.get_logger().info("dry_run: 模拟放置成功")
            return True

        arm = self.arm
        memory = self.memory
        cfg_p = self.cfg["placement"]

        zone = memory.get_zone()
        zone_cfg = cfg_p["zones"].get(zone)
        if zone_cfg is None:
            self.get_logger().error("未知放置区: %s" % (zone))
            return False

        dis = float(zone_cfg["dis"])
        height = float(zone_cfg["height"])
        self.get_logger().info("放置到 %s 区 (dis=%.1fmm, height=%.1fmm)" % (zone, dis, height))

        try:
            from utils.RobotArm.three_Inverse_kinematics import Arm as IKArm
            ok = arm.grap(dis, height, keep_gripper=True)
            if not ok:
                self.get_logger().error("放置 IK 解超出范围")
                return False
            a3, a4, a5 = IKArm(dis, height)
            arm.wait_for_position({3: a3, 4: a4, 5: a5})
            time.sleep(float(cfg_p.get("lower_timeout", 2.0)))
            arm.open_gripper()
            self.get_logger().info("已放置，夹爪已张开")
            return True
        except Exception as e:
            self.get_logger().error("放置失败: %s" % (e))
            return False

    # ------------------------------------------------------------------ #
    # 8-phase 状态机
    # ------------------------------------------------------------------ #
    def run_state_machine(self):
        """主线程运行的 8-phase 抓取流程。"""
        self._publish_state("INIT")

        if not self.dry_run and self.arm is not None:
            self.arm.set_pose(0)
            self.arm.set_pose(2)

        self._publish_state("STANDBY")
        self.get_logger().info("进入待命，等待 %s 信号..." % (self._start_topic))
        self._start_event.wait()
        if self._check_estop():
            return self._fail("ESTOP")

        # phase_2: detect
        self._publish_state("DETECTING")
        try:
            self._ensure_arm_cam_open()
        except Exception as e:
            return self._fail(f"CAM_OPEN_FAILED:{e}")
        stable = self._detect_stable()
        if stable is None:
            return self._fail("DETECT_TIMEOUT")

        # phase_3: align
        self._publish_state("ALIGNING")
        if not self._align_laterally(stable):
            return self._fail("ALIGN_FAILED")

        # phase_4: grasp
        self._publish_state("GRASPING")
        if not self._approach_and_grasp(stable):
            return self._fail("GRASP_FAILED")

        # 抓取完成后释放摄像头，让 letter_place_align 或后续流程可以自由使用
        self._release_arm_cam()

        # phase_5: transport
        self._publish_state("TRANSPORT")
        if not self.dry_run and self.arm is not None:
            self.arm.set_pose(3, keep_gripper=True)

        # phase_6: place
        self._publish_state("PLACING")
        self.get_logger().info("等待 %s 信号..." % (self._place_topic))
        self._place_event.wait()
        if self._check_estop():
            return self._fail("ESTOP")
        if not self._place():
            return self._fail("PLACE_FAILED")

        # phase_7: home
        self._publish_state("DONE")
        if not self.dry_run and self.arm is not None:
            self.arm.set_pose(0)
        self._result_pub.publish(BoolMsg(data=True))
        self.get_logger().info("任务完成")
        return True

    # ------------------------------------------------------------------ #
    # 资源释放
    # ------------------------------------------------------------------ #
    def finalize(self):
        """释放摄像头和机械臂资源。退出前先让机械臂回初始姿态并张开夹爪。

        为防止用户连按 Ctrl+C 打断复位流程（KeyboardInterrupt 继承自
        BaseException，普通 try/except Exception 抓不到），进入本函数后
        暂时忽略 SIGINT/SIGTERM，跑完再恢复。每一步都单独 try，一步炸掉
        不影响后一步——最关键的是最后 arm.finalize() 里的 emergency_stop
        必须发出，才能断力矩防舵机过热。
        """
        self.get_logger().info("释放资源...")

        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        prev_sigterm = signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if not self.dry_run:
                try:
                    self._release_arm_cam()
                except BaseException as e:
                    self.get_logger().warning("释放摄像头异常: %s" % (e,))
                try:
                    cv2.destroyAllWindows()
                except BaseException:
                    pass

                if self.arm is not None:
                    self.get_logger().info("安全复位：张开夹爪 + 回初始姿态 + 断力矩")
                    # 每一步都独立 try——open_gripper 挂掉不影响 set_pose，
                    # set_pose 挂掉也一定要走到 arm.finalize() 里的 emergency_stop
                    try:
                        self.arm.open_gripper()
                    except BaseException as e:
                        self.get_logger().warning("open_gripper 异常: %s" % (e,))
                    try:
                        time.sleep(0.3)
                    except BaseException:
                        pass
                    try:
                        self.arm.set_pose(0)
                    except BaseException as e:
                        self.get_logger().warning("set_pose(0) 异常: %s" % (e,))
                    try:
                        time.sleep(2.5)   # 等舵机走完初始姿态
                    except BaseException:
                        pass
                    try:
                        # arm.finalize 已改造成"先 emergency_stop 再关串口"，
                        # 即使前面 set_pose 没到位，这里也会把六路舵机断力矩防过热
                        self.arm.finalize()
                    except BaseException as e:
                        self.get_logger().warning("arm.finalize 异常: %s" % (e,))
        finally:
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    # 多轮循环支持：默认 max_rounds=1 保持原单轮行为；abcd_task 里覆写为 4
    try:
        max_rounds = int(node.get_parameter("max_rounds").value)
    except Exception:
        max_rounds = 1
    max_rounds = max(1, max_rounds)
    try:
        inter_wait = float(node.get_parameter("inter_round_wait_s").value)
    except Exception:
        inter_wait = 0.5
    inter_wait = max(0.0, inter_wait)

    try:
        for round_idx in range(max_rounds):
            if not rclpy.ok():
                break
            if max_rounds > 1:
                node.get_logger().info(
                    "=== round %d / %d ===" % (round_idx + 1, max_rounds))
            # 每轮开始前清空 Event，防止上一轮 /grasp/start、/grasp/place 的
            # 缓存立刻触发新一轮；同时 tools 侧 InspectionMemory 保留 zone。
            node._start_event.clear()
            node._place_event.clear()
            node.run_state_machine()
            if round_idx + 1 < max_rounds and rclpy.ok():
                time.sleep(inter_wait)
    except KeyboardInterrupt:
        node.get_logger().warning("用户中断，执行安全归位")
    except Exception as e:
        node.get_logger().error("状态机异常: %s" % (e))
    finally:
        node.finalize()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        exec_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
