"""
ArmController — 基于原版增强
原有接口（set_pose / grap / finalize）完整保留，新增夹爪控制、到位校验、抓取判断。
"""
import sys
import os
import time
import logging

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from typing import Optional

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
SCS_1_INIT_VALUE      = 1847   #2400   #夹爪闭合（初始/运输状态）
SCS_1_STATUS_VALUE    = 1847   #2047  

SCS_2_INIT_VALUE      = 2047
SCS_2_STATUS_VALUE    = 2047

SCS_3_INIT_VALUE      = 2750  #3080
SCS_3_STATUS_VALUE    = 2800
SCS_3_MOVE_VALUE      = 2800   #1070
SCS_3_TRANSPORT1_VALUE= 2000   #3060
SCS_3_TRANSPORT2_VALUE= 2940

SCS_4_INIT_VALUE      = 1000
SCS_4_STATUS_VALUE    = 1100
SCS_4_MOVE_VALUE      = 1300    #540
SCS_4_TRANSPORT1_VALUE= 1324
SCS_4_TRANSPORT2_VALUE= 1430

SCS_5_INIT_VALUE      = 2747
SCS_5_STATUS_VALUE    = 3030
SCS_5_MOVE_VALUE      = 2547   #1540
SCS_5_TRANSPORT1_VALUE= 2500
SCS_5_TRANSPORT2_VALUE= 2540

SCS_6_INIT_VALUE      = 2047
SCS_6_STATUS_VALUE    = 2047

DEFAULT_SPEED = 1500
DEFAULT_ACC   = 50

# 夹爪舵机寄存器地址（Feetech STS/SMS 内存表）
REG_MAX_TORQUE_LIMIT = 16   # EEPROM, 2 字节, 0-1000
REG_MODE             = 33   # EEPROM, 1 字节, 0=位置模式
REG_TORQUE_ENABLE    = 40   # SRAM,   1 字节, 1=使能
REG_LOCK             = 55   # SRAM,   1 字节, 0=解锁 EEPROM 可写

# 安全范围（逆运动学输出超出则拒绝执行）
SAFE_ANGLE_3 = (1000, 3200)
SAFE_ANGLE_4 = (540,  3400)
SAFE_ANGLE_5 = (1000, 3050)

# 默认 cfg（硬编码后备，正常由 config.yaml 注入
_DEFAULT_CFG = {
    "moving_speed":           DEFAULT_SPEED,
    "moving_acc":             DEFAULT_ACC,
    "gripper_open_val":       SCS_1_STATUS_VALUE,
    "gripper_close_val":      SCS_1_INIT_VALUE,
    "gripper_load_threshold": 200,
    "gripper_max_torque":     400,   # 夹爪舵机最大力矩上限（0-1000），封顶防堵转
    "grasp_retry_max":        3,
    "wait_position_timeout":  5.0,
    "wait_position_threshold":30,
}
class ArmController:

    def __init__(self, device: str = "/dev/ttyUSB0", cfg: Optional[dict] = None):
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

        # 夹爪舵机安全初始化：位置模式 + 力矩上限 + 力矩使能
        self._init_gripper_safety()

    def _init_gripper_safety(self) -> None:
        """
        夹爪舵机上电初始化：
          1) 确保工作在位置模式（mode=0）
          2) 把 Max_Torque_Limit 写到 cfg 指定值（默认 400/1000 = 40%）
          3) 确保力矩使能

        Max_Torque_Limit 是 EEPROM 寄存器，写入需先解锁 lock=0，写完锁回 lock=1。
        为减少 EEPROM 磨损，只在读到的值与目标不一致时才写。
        """
        ph = self.packetHandler
        sid = SCS_ID_1
        target_torque = int(self._cfg["gripper_max_torque"])
        target_torque = max(0, min(1000, target_torque))

        try:
            mode, comm_m, _  = ph.read1ByteTxRx(sid, REG_MODE)
            torq, comm_t, _  = ph.read2ByteTxRx(sid, REG_MAX_TORQUE_LIMIT)
            if comm_m != COMM_SUCCESS or comm_t != COMM_SUCCESS:
                logger.warning("夹爪舵机寄存器读取失败，跳过安全初始化（检查 ID/接线）")
                return

            need_write_mode   = (mode != 0)
            need_write_torque = (torq != target_torque)

            if need_write_mode or need_write_torque:
                ph.write1ByteTxRx(sid, REG_LOCK, 0)                # 解锁 EEPROM
                if need_write_mode:
                    ph.write1ByteTxRx(sid, REG_MODE, 0)
                    logger.info("夹爪舵机 mode: %d -> 0 (位置模式)", mode)
                if need_write_torque:
                    ph.write2ByteTxRx(sid, REG_MAX_TORQUE_LIMIT, target_torque)
                    logger.info("夹爪舵机 Max_Torque_Limit: %d -> %d",
                                torq, target_torque)
                ph.write1ByteTxRx(sid, REG_LOCK, 1)                # 重新锁定
            else:
                logger.info("夹爪舵机已在位置模式，力矩上限=%d，无需改写 EEPROM",
                            target_torque)

            # 力矩使能（SRAM，每次上电都要写）
            ph.write1ByteTxRx(sid, REG_TORQUE_ENABLE, 1)
        except Exception as e:
            logger.warning("夹爪安全初始化异常：%s（继续运行，可手动检查）", e)

    def set_pose(self, mode: int = 0, keep_gripper: bool = False) -> None:
        """切换预设姿态。0=初始 1=姿态1 2=运动 3=运输水平 4=运输垂直
        keep_gripper=True 时不改变夹爪位置（抓取后运输/归位时使用）。
        """
        ph = self.packetHandler
        spd, acc = self._speed, self._acc

        if mode == 0:
            if not keep_gripper:
                ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
        elif mode == 1:
            if not keep_gripper:
                ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_STATUS_VALUE,     spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_STATUS_VALUE,     spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_STATUS_VALUE,     spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
        elif mode == 2:
            if not keep_gripper:
                ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_MOVE_VALUE,       spd, acc)
            for _ in range(20):
                time.sleep(0.1)
            ph.WritePosEx(SCS_ID_4, SCS_4_MOVE_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_MOVE_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)
        elif mode == 3:
            if not keep_gripper:
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
            if not keep_gripper:
                ph.WritePosEx(SCS_ID_1, SCS_1_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_2, SCS_2_INIT_VALUE,       spd, acc)
            ph.WritePosEx(SCS_ID_5, SCS_5_TRANSPORT2_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_4, SCS_4_TRANSPORT2_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_3, SCS_3_TRANSPORT2_VALUE, spd, acc)
            ph.WritePosEx(SCS_ID_6, SCS_6_INIT_VALUE,       spd, acc)

    def grap(self, dis: float, height: float = 30, keep_gripper: bool = False) -> bool:
        """
        逆运动学求解并下发关节目标。
        dis: 水平距离(mm)，height: 末端高度(mm)。
        keep_gripper=True 时不改变夹爪位置。
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
        if not keep_gripper:
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
        """安全关闭：先给所有舵机断力矩再关串口。

        断力矩前提是位置已到位（否则臂会因重力自然下垂）。调用方应先
        set_pose(0) 并等它稳定，再进 finalize。断了力矩后即便进程崩掉、
        串口关闭，舵机也不会继续通电发热，能避免"忘复位 → 舵机堵转过热"。
        """
        try:
            self.emergency_stop()
        except Exception as e:
            logger.warning("finalize 中断力矩失败（继续关串口）: %s", e)
        try:
            self.portHandler.closePort()
        except Exception as e:
            logger.warning("串口关闭异常: %s", e)

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

    def wait_for_position(self, targets: dict, timeout: Optional[float] = None) -> bool:
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
        完整抓取 + 位置校验流程。
        判断逻辑：夹爪闭合后读实际位置，若实际位置与目标有明显差距（夹到了物体），
        则认为抓取成功；若夹到底（接近目标位置），则认为空夹失败。
        """
        max_retry   = int(self._cfg["grasp_retry_max"])
        pos_timeout = float(self._cfg["wait_position_timeout"])
        close_val   = int(self._cfg["gripper_close_val"])
        # 空夹时夹爪能夹到底，位置差小；夹到物体时夹不到底，位置差大
        pos_gap_thr = int(self._cfg.get("gripper_pos_gap_threshold", 100))

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
            # 等夹爪停稳
            time.sleep(0.5)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                moving, _, _ = self.packetHandler.ReadMoving(SCS_ID_1)
                if moving == 0:
                    break
                time.sleep(0.05)
            time.sleep(0.1)

            # 读夹爪实际位置 
            actual_pos, comm, _ = self.packetHandler.ReadPos(SCS_ID_1)
            pos_gap = abs(actual_pos - close_val)
            logger.info("夹爪位置: actual=%d target=%d gap=%d (阈值 %d) comm=%d",
                        actual_pos, close_val, pos_gap, pos_gap_thr, comm)

            if pos_gap >= pos_gap_thr:
                logger.info("抓取成功（位置差 %d，夹到物体）", pos_gap)
                return True

            logger.warning("抓取失败（位置差 %d < %d，空夹），准备重试", pos_gap, pos_gap_thr)
            self.open_gripper()
            time.sleep(0.3)

        logger.error("抓取失败，已重试 %d 次", max_retry)
        return False
