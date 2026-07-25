"""
TargetTracker — 多目标选择与锁定

职责：
  1. 每帧接收 detect_all() 候选列表；
  2. 首次检测：选 distance_mm 最小（最近）的目标并锁定；
  3. 后续帧：按 bbox 中心欧氏距离继续跟踪同一目标；
  4. 对 distance_mm 和 center_offset_x 做滑动窗口均值滤波；
  5. 窗口满后输出稳定读数（get_stable_target）；
  6. 连续丢失超过 lost_frames_max 帧则重置，等待重新选目标。
"""
import math
from collections import deque
from typing import Optional


class TargetTracker:

    def __init__(self, avg_window: int = 20, lost_frames_max: int = 10):
        self._window = avg_window
        self._lost_max = lost_frames_max
        self._reset()

    # ------------------------------------------------------------------ #
    def _reset(self):
        self._locked = False
        self._lock_cx = None          # 锁定目标的画面中心 x
        self._lock_cy = None          # 锁定目标的画面中心 y
        self._lock_bbox_short = None  # bbox 短边长度，用于匹配半径
        self._lost_count = 0
        self._dist_buf = deque(maxlen=self._window)
        self._offset_buf = deque(maxlen=self._window)
        self._last_result = None      # 上一帧成功匹配的原始 result

    # ------------------------------------------------------------------ #
    def _bbox_center(self, result):
        (x1, y1), (x2, y2) = result["bbox"]
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0

    def _bbox_short(self, result):
        (x1, y1), (x2, y2) = result["bbox"]
        return min(abs(x2 - x1), abs(y2 - y1))

    # ------------------------------------------------------------------ #
    def update(self, candidates: list) -> None:
        """
        用本帧检测结果更新 tracker。
        candidates: detect_all() 返回的列表（可为空）。
        """
        if not self._locked:
            if not candidates:
                return
            # 选最近目标（已按 distance_mm 升序）
            chosen = min(candidates, key=lambda r: r["distance_mm"])
            self._locked = True
            cx, cy = self._bbox_center(chosen)
            self._lock_cx = cx
            self._lock_cy = cy
            self._lock_bbox_short = max(self._bbox_short(chosen), 1.0)
            self._dist_buf.append(chosen["distance_mm"])
            self._offset_buf.append(chosen["center_offset_x"])
            self._last_result = chosen
            return

        # 已锁定：在候选中找距锁定中心最近且在半径内的
        best_match = None
        best_dist = float("inf")
        radius = self._lock_bbox_short * 0.5

        for r in candidates:
            cx, cy = self._bbox_center(r)
            d = math.hypot(cx - self._lock_cx, cy - self._lock_cy)
            if d < radius and d < best_dist:
                best_dist = d
                best_match = r

        if best_match is not None:
            cx, cy = self._bbox_center(best_match)
            self._lock_cx = cx
            self._lock_cy = cy
            self._lock_bbox_short = max(self._bbox_short(best_match), 1.0)
            self._dist_buf.append(best_match["distance_mm"])
            self._offset_buf.append(best_match["center_offset_x"])
            self._last_result = best_match
            self._lost_count = 0
        else:
            self._lost_count += 1
            if self._lost_count >= self._lost_max:
                self._reset()

    # ------------------------------------------------------------------ #
    def is_locked(self) -> bool:
        return self._locked

    def get_stable_target(self) -> Optional[dict]:
        """
        返回滑动均值稳定后的目标读数。
        窗口未满时返回 None（表示读数尚不稳定）。
        """
        if not self._locked or len(self._dist_buf) < self._window:
            return None
        avg_dist = sum(self._dist_buf) / len(self._dist_buf)
        avg_offset = sum(self._offset_buf) / len(self._offset_buf)
        result = dict(self._last_result)
        result["distance_mm"] = round(avg_dist, 1)
        result["center_offset_x"] = int(round(avg_offset))
        return result
