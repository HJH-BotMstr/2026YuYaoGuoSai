"""
ArmController — 基于原版增强
原有接口（set_pose / grap / finalize）完整保留，新增夹爪控制、到位校验、抓取判断。
"""
import sys
import os
import time
import logging

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from RobotArm.scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
from RobotArm.three_Inverse_kinematics import Arm

logger = logging.getLogger(__name__)

# 舵机 ID 常量 
SCS_ID_1 = 1   
SCS_ID_2 = 2   
SCS_ID_3 = 3   
SCS_ID_4 = 4   
SCS_ID_5 = 5   
SCS_ID_6 = 6  

#  舵机预设值（单位：舵机脉冲，2047 = 中间/水平） 
SCS_1_INIT_VALUE      = 2400   #夹爪闭合（初始/运输状态）
SCS_1_STATUS_VALUE    = 2047   #2047  

SCS_2_INIT_VALUE      = 2047
SCS_2_STATUS_VALUE    = 2047

SCS_3_INIT_VALUE      = 3000   #3080
SCS_3_STATUS_VALUE    = 2800
SCS_3_MOVE_VALUE      = 1500   #1070
SCS_3_TRANSPORT1_VALUE= 2000   #3060
SCS_3_TRANSPORT2_VALUE= 2940

SCS_4_INIT_VALUE      = 800
SCS_4_STATUS_VALUE    = 1100
SCS_4_MOVE_VALUE      = 650    #540
SCS_4_TRANSPORT1_VALUE= 1024
SCS_4_TRANSPORT2_VALUE= 1430

SCS_5_INIT_VALUE      = 2400
SCS_5_STATUS_VALUE    = 3030
SCS_5_MOVE_VALUE      = 2047   #1540
SCS_5_TRANSPORT1_VALUE= 2200
SCS_5_TRANSPORT2_VALUE= 2540

SCS_6_INIT_VALUE      = 2047
SCS_6_STATUS_VALUE    = 2047

DEFAULT_SPEED = 1500
DEFAULT_ACC   = 50

# 安全范围（逆运动学输出超出则拒绝执行）
SAFE_ANGLE_4 = (540,  3400)
SAFE_ANGLE_5 = (1000, 3050)

# 默认 cfg（硬编码后备，正常由 config.yaml 注入
_DEFAULT_CFG = {
    "moving_speed":           DEFAULT_SPEED,
    "moving_acc":             DEFAULT_ACC,
    "gripper_open_val":       SCS_1_STATUS_VALUE,
    "gripper_close_val":      SCS_1_INIT_VALUE,
    "gripper_load_threshold": 200,
    "grasp_retry_max":        3,
    "wait_position_timeout":  5.0,
    "wait_position_threshold":30,
}


class ArmController:

    def __init__(self, device: str = "/dev/ttyUSB0", cfg: dict | None = None):
        self._cfg = {**_DEFAULT_CFG, **(cfg or {})}
        self._speed = int(self._cfg["moving_speed"])
        self._acc   = int(self._cfg["moving_acc"])

        self.portHandler   = PortHandler(device)
        self.packetHandler = sms_sts(self.portHandler)

        if self.portHandler.openPort():
            logger.info("串口已打开: %s", device)
        else:
            logger.error("串口打开失败: %s", device)

        if self.portHandler.setBaudRate(int(self._cfg.get("arm_serial_baud", 500000))):
            logger.info("波特率设置成功")
        else:
            logger.error("波特率设置失败")                                

    def set_pose(self, mode: int = 0) -> None:
        """切换预设姿态。0=初始 1=姿态1 2=运动 3=运输水平 4=运输垂直"""
        ph = self.packetHandler
        spd, acc = self._speed, self._acc

        if mode == 0:
            ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
        elif mode == 1:
            ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_STATUS_VALUE,     spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_STATUS_VALUE,     spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_STATUS_VALUE,     spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
        elif mode == 2:
            ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_MOVE_VALUE,       spd, acc)
            for _ in range(20):   
                time.sleep(0.1)
            ph.WritePosEx(SCS_ID_4, SCS_4_MOVE_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_MOVE_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
        elif mode == 3:
            ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
            for _ in range(10):   
                time.sleep(0.1)
            ph.WritePosEx(SCS_ID_5, SCS_5_TRANSPORT1_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_TRANSPORT1_VALUE, spd, acc)
            for _ in range(10):   
                time.sleep(0.1)
            ph.WritePosEx(SCS_ID_3, SCS_3_TRANSPORT1_VALUE, spd, acc)
        elif mode == 4:
            ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_TRANSPORT2_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_TRANSPORT2_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_TRANSPORT2_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)

    def grap(self, dis: float, height: float = 30) -> bool:
        """
        逆运动学求解并下发关节目标。
        dis: 水平距离(mm)，height: 末端高度(mm)。
        解超出安全范围时返回 False 并不执行。
        """
        angle_3, angle_4, angle_5 = Arm(dis, height)
        if not (SAFE_ANGLE_3[0] <= angle_3 <= SAFE_ANGLE_3[1] and
                SAFE_ANGLE_4[0] <= angle_4 <= SAFE_ANGLE_4[1] and
                SAFE_ANGLE_5[0] <= angle_5 <= SAFE_ANGLE_5[1]):
            logger.warning(
                "IK 解超出安全范围: a3=%d a4=%d a5=%d (dis=%.1f height=%.1f)",
                angle_3, angle_4, angle_5, dis, height
            )
            return False

        ph = self.packetHandler
        spd, acc = self._speed, self._acc
        ph.WritePosEx(SCS_ID_1, SCS_1_STATUS_VALUE, spd, acc)
        ph.WritePosEx(SCS_ID_2, SCS_2_STATUS_VALUE, spd, acc)
        ph.WritePosEx(SCS_ID_4, angle_4,            spd, acc)
        time.sleep(1)
        ph.WritePosEx(SCS_ID_3, angle_3,            spd, acc)
        ph.WritePosEx(SCS_ID_5, angle_5,            spd, acc)
        ph.WritePosEx(SCS_ID_6, SCS_6_STATUS_VALUE, spd, acc)
        return True

    def emergency_stop(self) -> None:
        """
        急停：向所有舵机写 TORQUE_ENABLE=0，立即断力矩原地停住，机械臂会因重力自然下垂
       
        """
        for sid in (SCS_ID_1, SCS_ID_2, SCS_ID_3, SCS_ID_4, SCS_ID_5, SCS_ID_6):
            self.packetHandler.write1ByteTxRx(sid, 40, 0)  # 40 = SMS_STS_TORQUE_ENABLE
        logger.warning("急停：所有舵机已断力矩")

    def finalize(self) -> None:
        """关闭串口。"""
        self.portHandler.closePort()

    def open_gripper(self) -> None:
        """张开夹爪。"""
        val = int(self._cfg["gripper_open_val"])
        self.packetHandler.WritePosEx(SCS_ID_1, val, self._speed, self._acc)
        logger.debug("夹爪张开 -> %d", val)

    def close_gripper(self) -> None:
        """闭合夹爪。"""
        val = int(self._cfg["gripper_close_val"])
        self.packetHandler.WritePosEx(SCS_ID_1, val, self._speed, self._acc)
        logger.debug("夹爪闭合 -> %d", val)

    def read_positions(self, ids=(1, 2, 3, 4, 5, 6)) -> dict:
        """
        读取指定舵机当前位置。
        返回 {servo_id: position}。通信失败时该 id 对应值为 -1。
        """
        result = {}
        for sid in ids:
            pos, comm, err = self.packetHandler.ReadPos(sid)
            result[sid] = pos if comm == COMM_SUCCESS else -1
        return result

    def wait_for_position(self, targets: dict, timeout: float | None = None) -> bool:
        """
        阻塞直到所有 targets 舵机到达目标位置（在阈值内）或超时。
        targets: {servo_id: target_position}
        返回 True 表示全部到位，False 表示超时。
        """
        if timeout is None:
            timeout = float(self._cfg["wait_position_timeout"])
        thr = int(self._cfg["wait_position_threshold"])
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            positions = self.read_positions(list(targets.keys()))
            if all(abs(positions.get(sid, -9999) - tgt) <= thr
                   for sid, tgt in targets.items()):
                return True
            time.sleep(0.05)
        logger.warning("wait_for_position 超时 (%.1fs)，targets=%s", timeout, targets)
        return False

    def grasp_with_verify(self, dis: float, height: float) -> bool:
        """
        完整抓取 + 负载校验流程。
        流程：open → grap（原始接口）→ wait_for_position → close → 读负载 → 判断
        失败自动重试，最多 grasp_retry_max 次。
        返回 True 表示成功抓住，False 表示全部重试失败。
        """
        max_retry   = int(self._cfg["grasp_retry_max"])
        load_thr    = int(self._cfg["gripper_load_threshold"])
        pos_timeout = float(self._cfg["wait_position_timeout"])
        spd, acc    = self._speed, self._acc

        # 读取空载基准负载
        base_load, _, _ = self.packetHandler.ReadLoad(SCS_ID_1)

        for attempt in range(1, max_retry + 1):
            logger.info("抓取尝试 %d/%d", attempt, max_retry)

            self.open_gripper()
            time.sleep(0.3)

            if not self.grap(dis, height):
                logger.error("IK 解超出范围，放弃抓取")
                return False

            # 等待关节 3/4/5 到位
            angle_3, angle_4, angle_5 = Arm(dis, height)
            targets = {SCS_ID_3: angle_3, SCS_ID_4: angle_4, SCS_ID_5: angle_5}
            self.wait_for_position(targets, timeout=pos_timeout)

            self.close_gripper()
            time.sleep(0.5)

            # 读夹爪负载，与基准差值判断是否夹住
            current_load, _, _ = self.packetHandler.ReadLoad(SCS_ID_1)
            load_diff = abs(current_load - base_load)
            logger.debug("负载差值: %d (阈值 %d)", load_diff, load_thr)

            if load_diff >= load_thr:
                logger.info("抓取成功（负载差 %d）", load_diff)
                return True

            logger.warning("抓取失败（负载差 %d < %d），准备重试", load_diff, load_thr)
            self.open_gripper()
            time.sleep(0.3)

        logger.error("抓取失败，已重试 %d 次", max_retry)
        return False
