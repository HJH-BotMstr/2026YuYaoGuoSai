#!/usr/bin/env python3
"""
grasp 抓取任务主入口
运行环境：Lite3 感知主机（Ubuntu）
用法：python3 main.py [--config config.yaml] [--zone A]
"""
import sys
import os
import argparse
import logging
import time
import yaml
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.ArmController    import ArmController
from utils.BlockDetection   import BlockDetection
from utils.InspectionMemory import InspectionMemory

# 日志配置 
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("grap_run.log", encoding="utf-8"),
    ],
)
logger = logging.getLogger("main")


# 辅助：读取 config 
def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

#  阶段函数，统一签名 phase_X(ctx) -> bool                           
#  ctx: 共享状态字典，包含所有初始化好的对象和配置                      

def phase_0_init(cfg: dict) -> dict | None:
    """初始化所有模块，返回 ctx 字典；失败返回 None。"""
    logger.info("=== phase_0: 初始化 ===")
    try:
        arm = ArmController(
            device=cfg["hardware"]["arm_serial_port"],
            cfg={**cfg["arm"], "arm_serial_baud": cfg["hardware"]["arm_serial_baud"]},
        )
        detector = BlockDetection({**cfg["detection"]})
        memory   = InspectionMemory(
            default_zone=cfg["inspection"]["default_zone"]
        )

        cam_device = cfg["hardware"]["arm_cam_device"]
        arm_cam = cv2.VideoCapture(cam_device, cv2.CAP_V4L2)
        if not arm_cam.isOpened():
            logger.error("机械臂摄像头打开失败: %s", cam_device)
            arm.finalize()
            return None

        logger.info("初始化完成。摄像头: %s  串口: %s",
                    cam_device, cfg["hardware"]["arm_serial_port"])
        return {
            "cfg":      cfg,
            "arm":      arm,
            "detector": detector,
            "memory":   memory,
            "arm_cam":  arm_cam,
        }
    except Exception as e:
        logger.error("初始化失败: %s", e)
        return None


def phase_1_standby(ctx: dict) -> bool:
    """机械臂回初始姿态，等待机器狗就位（外部信号驱动，此处为简单等待）。"""
    logger.info("=== phase_1: 待命 ===")
    try:
        ctx["arm"].set_pose(0)
        time.sleep(2)
        ctx["arm"].set_pose(2)   # 进入运动/摄像头视野姿态
        time.sleep(3)
        logger.info("机械臂就绪，等待机器狗停稳...")
        # [TODO: ROS2集成] 此处替换为订阅 /grap/start 服务或话题信号
        input("确认机器狗已停稳，按回车继续...")
        return True
    except KeyboardInterrupt:
        return False


def phase_2_detect_block(ctx: dict) -> dict | None:
    """
    持续读帧，检测红色长条，距离稳定后返回检测结果。
    返回最终 result dict，或超时返回 None。
    """
    logger.info("=== phase_2: 识别红色长条 ===")
    cfg_g    = ctx["cfg"]["grasp"]
    detector = ctx["detector"]
    arm      = ctx["arm"]
    arm_cam  = ctx["arm_cam"]
    cfg_arm  = ctx["cfg"]["arm"]

    D_target    = float(cfg_g["D_hand_mm"])
    D_thr       = float(cfg_g["D_hand_thr_mm"])
    off_thr     = int(cfg_g["center_offset_threshold"])
    window      = int(cfg_g["distance_avg_window"])
    timeout     = float(cfg_g["detect_timeout"])
    spd, acc    = int(cfg_arm["moving_speed"]), int(cfg_arm["moving_acc"])

    distance_buf = []
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        ret, frame = arm_cam.read()
        if not ret:
            logger.warning("摄像头读帧失败，跳过")
            continue

        result = detector.detect(frame)
        vis = detector.visualize(frame.copy(), result)
        cv2.imshow("arm_cam", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            logger.info("用户中断识别")
            return None

        if result is None:
            distance_buf.clear()
            logger.debug("未检测到色块")
            continue

        # 横向对齐：微调 6 号舵机
        off_x = result["center_offset_x"]
        if abs(off_x) > off_thr:
            # 6号舵机：值大→顺时针(右)，小→逆时针(左)
            pos6_data = arm.read_positions((6,))
            cur6 = pos6_data.get(6, 2047)
            adjust = int(off_x * 0.3)   # 比例系数可现场调
            new6 = max(0, min(4095, cur6 + adjust))
            arm.packetHandler.WritePosEx(6, new6, spd, acc)
            logger.debug("横向调整 6 号舵机: %d -> %d (offset=%d)", cur6, new6, off_x)
            distance_buf.clear()
            continue

        # 距离滑动均值
        distance_buf.append(result["distance_mm"])
        if len(distance_buf) > window:
            distance_buf.pop(0)

        if len(distance_buf) == window:
            avg_dist = sum(distance_buf) / window
            logger.info("距离均值: %.1f mm（目标 %.1f ±%.1f）", avg_dist, D_target, D_thr)
            if abs(avg_dist - D_target) <= D_thr:
                logger.info("距离稳定，进入抓取。distance=%.1f mm", avg_dist)
                result["distance_mm"] = avg_dist
                return result

    logger.error("phase_2 超时 (%.1fs)，未检测到稳定色块", timeout)
    return None


def phase_3_grasp(ctx: dict, detect_result: dict) -> bool:
    """调用 grasp_with_verify 执行抓取。"""
    logger.info("=== phase_3: 抓取 ===")
    cfg_g = ctx["cfg"]["grasp"]
    arm   = ctx["arm"]

    dis    = detect_result["distance_mm"]
    height = float(cfg_g["grasp_height_mm"])

    ok = arm.grasp_with_verify(dis=dis, height=height)
    if ok:
        logger.info("抓取成功")
    else:
        logger.error("抓取失败（已重试 %d 次）", ctx["cfg"]["arm"]["grasp_retry_max"])
    return ok


def phase_4_transport(ctx: dict) -> bool:
    """进入运输姿态。"""
    logger.info("=== phase_4: 运输姿态 ===")
    try:
        ctx["arm"].set_pose(3)
        time.sleep(3)
        return True
    except Exception as e:
        logger.error("运输姿态失败: %s", e)
        return False


def phase_5_place(ctx: dict) -> bool:
    """查询目标放置区，执行放置动作。"""
    logger.info("=== phase_5: 放置 ===")
    arm    = ctx["arm"]
    memory = ctx["memory"]
    cfg_p  = ctx["cfg"]["placement"]

    zone = memory.get_zone()
    zone_cfg = cfg_p["zones"].get(zone)
    if zone_cfg is None:
        logger.error("未知放置区: %s", zone)
        return False

    dis    = float(zone_cfg["dis"])
    height = float(zone_cfg["height"])
    logger.info("放置到 %s 区 (dis=%.1fmm, height=%.1fmm)", zone, dis, height)

    try:
        result = arm.grap(dis, height)
        if not result:
            logger.error("放置位置 IK 解超出范围")
            return False

        # 等待关节到位
        from utils.RobotArm.three_Inverse_kinematics import Arm as IKArm
        a3, a4, a5 = IKArm(dis, height)
        arm.wait_for_position({3: a3, 4: a4, 5: a5})

        time.sleep(float(cfg_p.get("lower_timeout", 2.0)))
        arm.open_gripper()
        time.sleep(0.5)
        logger.info("已放置，夹爪已张开")
        return True
    except Exception as e:
        logger.error("放置失败: %s", e)
        return False


def phase_6_home(ctx: dict) -> None:
    """归位并释放资源。"""
    logger.info("=== phase_6: 归位 ===")
    try:
        ctx["arm"].set_pose(0)
        time.sleep(2)
    except Exception as e:
        logger.warning("归位时异常: %s", e)
    finally:
        ctx["arm_cam"].release()
        cv2.destroyAllWindows()
        ctx["arm"].finalize()
        logger.info("资源已释放")

#  主流程                                                              

def main():
    parser = argparse.ArgumentParser(description="grap 抓取任务")
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "config.yaml"))
    parser.add_argument("--zone",   default=None,
                        help="手动指定放置区（覆盖占位默认值），如 --zone B")
    args = parser.parse_args()

    cfg = load_config(args.config)
    logger.info("配置加载完成: %s", args.config)

    ctx = phase_0_init(cfg)
    if ctx is None:
        sys.exit(1)

    # 手动注入放置区（测试/调试用）
    if args.zone:
        ctx["memory"].set_zone(args.zone.upper())
        logger.info("手动设置放置区: %s", args.zone.upper())

    try:
        # phase_1: 待命
        if not phase_1_standby(ctx):
            logger.error("待命阶段中止")
            phase_6_home(ctx)
            sys.exit(1)

        # phase_2: 识别
        detect_result = phase_2_detect_block(ctx)
        if detect_result is None:
            logger.error("识别失败，任务终止")
            phase_6_home(ctx)
            sys.exit(1)

        # phase_3: 抓取
        if not phase_3_grasp(ctx, detect_result):
            logger.error("抓取失败，任务终止")
            phase_6_home(ctx)
            sys.exit(1)

        # phase_4: 运输
        if not phase_4_transport(ctx):
            logger.error("运输姿态失败，任务终止")
            phase_6_home(ctx)
            sys.exit(1)

        # phase_5: 放置
        if not phase_5_place(ctx):
            logger.error("放置失败")
            phase_6_home(ctx)
            sys.exit(1)

        logger.info("任务完成！")

    except KeyboardInterrupt:
        logger.warning("用户中断，执行安全归位")
    finally:
        phase_6_home(ctx)


if __name__ == "__main__":
    main()
