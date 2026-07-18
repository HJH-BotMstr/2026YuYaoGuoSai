"""
BlockDetection.detect_all() 和 TargetTracker 单元测试
用合成图像测试，不依赖真实摄像头。
"""
import sys
import os
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.BlockDetection import BlockDetection

CFG = {
    "arm_cam_fx": 388.1454,
    "arm_cam_fy": 387.7497,
    "arm_cam_cx": 320.0,
    "arm_cam_cy": 240.0,
    "arm_cam_dist": [0.0, 0.0, 0.0, 0.0, 0.0],
    "hsv_red_lower1": [0,   120, 100],
    "hsv_red_upper1": [10,  255, 255],
    "hsv_red_lower2": [160, 120, 100],
    "hsv_red_upper2": [180, 255, 255],
    "hsv_green_lower": [40, 80, 80],
    "hsv_green_upper": [80, 255, 255],
    "block_min_area": 200,
    "block_real_width_mm": 40.0,
}


def _make_frame(*rects):
    """生成含多个色块的测试帧。rects: [(color_bgr, x, y, w, h), ...]"""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    for color_bgr, x, y, w, h in rects:
        cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, -1)
    return frame


# ──────────────────────── detect_all ─────────────────────────────────────── #

def test_detect_all_returns_empty_on_blank():
    det = BlockDetection(CFG)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    results = det.detect_all(frame)
    assert results == [], f"空帧应返回空列表，实际 {results}"


def test_detect_all_returns_one_block():
    det = BlockDetection(CFG)
    frame = _make_frame(((0, 0, 200), 200, 150, 100, 60))
    results = det.detect_all(frame)
    assert len(results) == 1
    assert results[0]["color"] == "red"


def test_detect_all_returns_two_blocks_different_colors():
    det = BlockDetection(CFG)
    frame = _make_frame(
        ((0, 0, 200), 50,  150, 80, 50),   # 红色
        ((0, 180, 0), 400, 150, 80, 50),   # 绿色
    )
    results = det.detect_all(frame)
    assert len(results) == 2
    colors = {r["color"] for r in results}
    assert "red" in colors and "green" in colors


def test_detect_all_result_has_required_keys():
    det = BlockDetection(CFG)
    frame = _make_frame(((0, 0, 200), 200, 150, 100, 60))
    results = det.detect_all(frame)
    for key in ("color", "bbox", "center_offset_x", "distance_mm", "pos_3d"):
        assert key in results[0], f"缺少键 {key!r}"


def test_detect_all_selects_nearest_is_smallest_distance():
    """两个红色块，距离近的（更宽的 bbox）distance_mm 应更小。"""
    det = BlockDetection(CFG)
    frame = _make_frame(
        ((0, 0, 200), 100, 100, 200, 60),  # 宽=200px，近
        ((0, 0, 200), 400, 200,  50, 30),  # 宽=50px，远
    )
    results = det.detect_all(frame)
    # 至少检测到一个
    assert len(results) >= 1
    # 最近的（distance_mm 最小）应该是宽=200px 那个
    nearest = min(results, key=lambda r: r["distance_mm"])
    (x1, _), (x2, _) = nearest["bbox"]
    assert (x2 - x1) > 100, "最近目标应是宽 bbox"


# ──────────────────────── TargetTracker ──────────────────────────────────── #

def test_tracker_no_target_initially():
    from utils.TargetTracker import TargetTracker
    tracker = TargetTracker(avg_window=5, lost_frames_max=3)
    assert not tracker.is_locked()
    assert tracker.get_stable_target() is None


def test_tracker_locks_on_first_detection():
    from utils.TargetTracker import TargetTracker
    tracker = TargetTracker(avg_window=3, lost_frames_max=3)
    result = {"color": "red", "bbox": ((100, 100), (200, 160)),
              "center_offset_x": 10, "distance_mm": 300.0,
              "pos_3d": (10.0, 300.0, 5.0)}
    tracker.update([result])
    assert tracker.is_locked()


def test_tracker_stable_after_window_full():
    from utils.TargetTracker import TargetTracker
    tracker = TargetTracker(avg_window=3, lost_frames_max=5)
    results = [{"color": "red", "bbox": ((100, 100), (200, 160)),
                "center_offset_x": 0, "distance_mm": float(300 + i),
                "pos_3d": (0.0, float(300 + i), 0.0)}
               for i in range(3)]
    for r in results:
        tracker.update([r])
    stable = tracker.get_stable_target()
    assert stable is not None
    assert stable["distance_mm"] == pytest_approx(301.0, abs=1.0)


def test_tracker_not_stable_before_window_full():
    from utils.TargetTracker import TargetTracker
    tracker = TargetTracker(avg_window=5, lost_frames_max=5)
    result = {"color": "red", "bbox": ((100, 100), (200, 160)),
              "center_offset_x": 0, "distance_mm": 300.0,
              "pos_3d": (0.0, 300.0, 0.0)}
    tracker.update([result])
    tracker.update([result])
    assert tracker.get_stable_target() is None   # 只更新 2 次，窗口未满


def test_tracker_selects_nearest_candidate():
    from utils.TargetTracker import TargetTracker
    tracker = TargetTracker(avg_window=1, lost_frames_max=3)
    near = {"color": "red", "bbox": ((100, 100), (200, 160)),
            "center_offset_x": 0, "distance_mm": 150.0,
            "pos_3d": (0.0, 150.0, 0.0)}
    far  = {"color": "red", "bbox": ((300, 100), (380, 160)),
            "center_offset_x": 100, "distance_mm": 400.0,
            "pos_3d": (100.0, 400.0, 0.0)}
    tracker.update([far, near])
    stable = tracker.get_stable_target()
    assert stable["distance_mm"] == 150.0


def test_tracker_resets_after_lost_max_frames():
    from utils.TargetTracker import TargetTracker
    tracker = TargetTracker(avg_window=2, lost_frames_max=2)
    result = {"color": "red", "bbox": ((100, 100), (200, 160)),
              "center_offset_x": 0, "distance_mm": 200.0,
              "pos_3d": (0.0, 200.0, 0.0)}
    tracker.update([result])
    tracker.update([])   # 丢失 1
    tracker.update([])   # 丢失 2 → 超过 lost_frames_max，重置
    assert not tracker.is_locked()


# pytest 兼容性辅助
def pytest_approx(val, abs=1e-6):
    _tol = abs

    class _Approx:
        def __eq__(self, other):
            return __builtins__["abs"](other - val) <= _tol if isinstance(__builtins__, dict) \
                else __import__("builtins").abs(other - val) <= _tol

    return _Approx()


if __name__ == "__main__":
    tests = [
        test_detect_all_returns_empty_on_blank,
        test_detect_all_returns_one_block,
        test_detect_all_returns_two_blocks_different_colors,
        test_detect_all_result_has_required_keys,
        test_detect_all_selects_nearest_is_smallest_distance,
        test_tracker_no_target_initially,
        test_tracker_locks_on_first_detection,
        test_tracker_stable_after_window_full,
        test_tracker_not_stable_before_window_full,
        test_tracker_selects_nearest_candidate,
        test_tracker_resets_after_lost_max_frames,
    ]
    passed = failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {t.__name__}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
