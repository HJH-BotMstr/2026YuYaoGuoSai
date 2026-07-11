"""
RobotSignalInterface — 启动 / 放置信号接口
封装"等待机器狗到位"和"等待放置触发"两个信号。

robot 模式：等待 ROS2 服务（当前为 stub）
pc 模式：非交互环境直接返回 True；交互等待由 main.py 负责
"""
import logging

logger = logging.getLogger(__name__)

_VALID_MODES = {"robot", "pc"}


class RobotSignalInterface:

    def __init__(self, mode: str):
        if mode not in _VALID_MODES:
            raise ValueError(f"无效模式: {mode!r}，应为 {_VALID_MODES}")
        self._mode = mode

    def wait_start(self) -> bool:
        """等待机器狗到达 place1 停稳信号（phase_1）。"""
        if self._mode == "robot":
            # [TODO: ROS2集成] 等待 /grasp/start 服务
            logger.info("[stub] 等待 /grasp/start 信号（stub 直接返回 True）")
            return True
        else:
            return True

    def wait_place(self, zone: str) -> bool:
        """等待机器狗到达放置站位信号，携带目标区（phase_6）。"""
        if self._mode == "robot":
            # [TODO: ROS2集成] 等待 /grasp/place 服务，获取 zone 参数
            logger.info("[stub] 等待 /grasp/place 信号 zone=%s（stub 直接返回 True）", zone)
            return True
        else:
            return True
