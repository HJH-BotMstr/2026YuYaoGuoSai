"""
RobotSignalInterface — 启动 / 放置信号接口
封装"等待机器狗到位"和"等待放置触发"两个信号。

robot 模式：
  wait_start()  — 订阅 /grasp/start (std_msgs/Bool)，阻塞直到收到 data=True
  wait_place()  — stub（暂未接入 ROS2）
pc 模式：非交互环境直接返回 True；交互等待由 main.py 负责
"""
import logging
import threading

logger = logging.getLogger(__name__)

_VALID_MODES = {"robot", "pc"}

_GRASP_START_TOPIC = "/grasp/start"
_WAIT_TIMEOUT_S    = 300.0   # 最长等待 5 分钟


class RobotSignalInterface:

    def __init__(self, mode: str):
        if mode not in _VALID_MODES:
            raise ValueError(f"无效模式: {mode!r}，应为 {_VALID_MODES}")
        self._mode = mode

    def wait_start(self) -> bool:
        """等待机器狗到达 place1 停稳信号（phase_1）。

        robot 模式：订阅 /grasp/start (std_msgs/Bool)，收到 data=True 才返回。
        pc 模式：直接返回 True，交互确认由 main.py 负责。
        """
        if self._mode != "robot":
            return True

        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Bool as BoolMsg

            if not rclpy.ok():
                rclpy.init()

            received = threading.Event()
            result   = [False]

            class _WaitNode(Node):
                def __init__(self):
                    super().__init__("_grasp_start_waiter")
                    self._sub = self.create_subscription(
                        BoolMsg, _GRASP_START_TOPIC, self._cb, 10)

                def _cb(self, msg: BoolMsg):
                    if msg.data:
                        result[0] = True
                        received.set()

            node = _WaitNode()
            logger.info("等待 /grasp/start 信号 (topic=%s, 超时=%ss)...",
                        _GRASP_START_TOPIC, _WAIT_TIMEOUT_S)

            deadline_check_hz = 20
            import time
            deadline = time.monotonic() + _WAIT_TIMEOUT_S
            while not received.is_set() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=1.0 / deadline_check_hz)

            node.destroy_node()

            if not result[0]:
                logger.error("wait_start 超时 (%.0fs)，未收到 /grasp/start", _WAIT_TIMEOUT_S)
                return False

            logger.info("收到 /grasp/start，机器狗已到位")
            return True

        except Exception as e:
            logger.exception("wait_start ROS2 集成异常: %s", e)
            return False

    def wait_place(self, zone: str) -> bool:
        """等待机器狗到达放置站位信号，携带目标区（phase_6）。"""
        if self._mode == "robot":
            # [TODO: ROS2集成] 等待 /grasp/place 服务，获取 zone 参数
            logger.info("[stub] 等待 /grasp/place 信号 zone=%s（stub 直接返回 True）", zone)
            return True
        else:
            return True
