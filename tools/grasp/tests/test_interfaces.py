"""
DogAlignInterface / RobotSignalInterface 单元测试
不依赖硬件，pc 模式逻辑可直接验证，robot stub 只验证接口可调用。
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ──────────────────────── DogAlignInterface ──────────────────────────────── #

def test_dog_align_pc_send_does_not_raise():
    from utils.DogAlignInterface import DogAlignInterface
    iface = DogAlignInterface(mode="pc")
    iface.send_align(30.0)   # 仅要求不抛异常


def test_dog_align_pc_wait_returns_true():
    from utils.DogAlignInterface import DogAlignInterface
    iface = DogAlignInterface(mode="pc")
    # pc 模式不等用户输入（测试环境无 stdin），直接返回 True
    result = iface.wait_aligned(timeout=0.1)
    assert result is True


def test_dog_align_robot_send_does_not_raise():
    from utils.DogAlignInterface import DogAlignInterface
    iface = DogAlignInterface(mode="robot")
    iface.send_align(-15.5)  # stub 不发真实 ROS2，不应抛异常


def test_dog_align_robot_wait_returns_true():
    from utils.DogAlignInterface import DogAlignInterface
    iface = DogAlignInterface(mode="robot")
    result = iface.wait_aligned(timeout=0.1)
    assert result is True   # stub 直接返回 True


def test_dog_align_invalid_mode_raises():
    from utils.DogAlignInterface import DogAlignInterface
    try:
        DogAlignInterface(mode="unknown")
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


# ──────────────────────── RobotSignalInterface ───────────────────────────── #

def test_robot_signal_pc_wait_start_returns_true():
    from utils.RobotSignalInterface import RobotSignalInterface
    iface = RobotSignalInterface(mode="pc")
    result = iface.wait_start()
    assert result is True


def test_robot_signal_pc_wait_place_returns_true():
    from utils.RobotSignalInterface import RobotSignalInterface
    iface = RobotSignalInterface(mode="pc")
    result = iface.wait_place(zone="A")
    assert result is True


def test_robot_signal_robot_wait_start_returns_true():
    from utils.RobotSignalInterface import RobotSignalInterface
    iface = RobotSignalInterface(mode="robot")
    result = iface.wait_start()
    assert result is True


def test_robot_signal_robot_wait_place_returns_true():
    from utils.RobotSignalInterface import RobotSignalInterface
    iface = RobotSignalInterface(mode="robot")
    result = iface.wait_place(zone="B")
    assert result is True


def test_robot_signal_invalid_mode_raises():
    from utils.RobotSignalInterface import RobotSignalInterface
    try:
        RobotSignalInterface(mode="bad")
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


if __name__ == "__main__":
    tests = [
        test_dog_align_pc_send_does_not_raise,
        test_dog_align_pc_wait_returns_true,
        test_dog_align_robot_send_does_not_raise,
        test_dog_align_robot_wait_returns_true,
        test_dog_align_invalid_mode_raises,
        test_robot_signal_pc_wait_start_returns_true,
        test_robot_signal_pc_wait_place_returns_true,
        test_robot_signal_robot_wait_start_returns_true,
        test_robot_signal_robot_wait_place_returns_true,
        test_robot_signal_invalid_mode_raises,
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
