"""
DogAlignInterface — 横向对齐接口
封装"向机器狗发横向调整指令"和"等待对齐完成"两个动作。

robot 模式：发 ROS2 指令（当前为 stub，TODO: 接入实际 ROS2 topic）
pc 模式：打印提示，测试环境跳过 stdin 等待
"""
import logging

logger = logging.getLogger(__name__)

_VALID_MODES = {"robot", "pc"}


class DogAlignInterface:

    def __init__(self, mode: str):
        if mode not in _VALID_MODES:
            raise ValueError(f"无效模式: {mode!r}，应为 {_VALID_MODES}")
        self._mode = mode

    def send_align(self, offset_x_mm: float) -> None:
        """发送横向偏移量给机器狗（正=右移，负=左移，单位 mm）。"""
        if self._mode == "robot":
            # [TODO: ROS2集成] 发布到 /dog/lateral_adjust topic
            logger.info("[stub] 发送横向调整指令: offset_x=%.1f mm", offset_x_mm)
        else:
            direction = "右" if offset_x_mm > 0 else "左"
            print(f"[pc] 横向偏移 {abs(offset_x_mm):.1f}mm，请手动向{direction}调整机器狗位置")

    def wait_aligned(self, timeout: float = 10.0) -> bool:
        """等待对齐完成信号。返回 True 表示已对齐，False 表示超时。"""
        if self._mode == "robot":
            # [TODO: ROS2集成] 等待 /dog/aligned 回调
            logger.info("[stub] 等待机器狗对齐完成信号（stub 直接返回 True）")
            return True
        else:
            # pc 模式：非交互环境（测试/自动化）直接返回 True
            # 交互环境由 main.py 的 phase_3 负责调用 input() 等待用户操作
            return True
