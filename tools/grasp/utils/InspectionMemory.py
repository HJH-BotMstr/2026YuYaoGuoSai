import threading


VALID_ZONES = {"A", "B", "C", "D"}


class InspectionMemory:
    """
    保存巡检结果（目标放置区字母）。
    当前为占位实现，默认返回 default_zone。
    ROS2 集成时在话题回调里调用 set_zone(zone) 注入真实结果。
    """

    def __init__(self, default_zone: str = "A"):
        if default_zone not in VALID_ZONES:
            raise ValueError(f"default_zone 必须是 {VALID_ZONES}，收到 {default_zone!r}")
        self._default_zone = default_zone
        self._zone = default_zone
        self._lock = threading.Lock()

    def set_zone(self, zone: str) -> None:
        """注入巡检结果。zone 必须是 A/B/C/D，否则抛出 ValueError。"""
        if zone not in VALID_ZONES:
            raise ValueError(f"zone 必须是 {VALID_ZONES}，收到 {zone!r}")
        with self._lock:
            self._zone = zone

    def get_zone(self) -> str:
        """返回当前目标放置区字母。"""
        with self._lock:
            return self._zone

    def is_ready(self) -> bool:
        """是否已准备好（占位实现恒返回 True）。"""
        return True

    def reset(self) -> None:
        """重置为默认区，用于多次运行之间清理状态。"""
        with self._lock:
            self._zone = self._default_zone
