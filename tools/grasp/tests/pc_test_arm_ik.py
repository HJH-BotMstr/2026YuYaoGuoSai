import sys

sys.path.insert(0, '/home/fishros/2026YuYaoGuoSai/assets/old_code/DeepRobotDog')

from utils.RobotArm.three_Inverse_kinematics import Arm


def check_safe(angle_3, angle_4, angle_5):
    """检查舵机值是否在常用安全范围内"""
    return (1300 <= angle_3 <= 3000 and
            540 <= angle_4 <= 3400 and
            1000 <= angle_5 <= 3050)


def main():
    print("测试不同 (dis, height) 下的逆运动学解：")
    print("=" * 60)

    test_cases = [
        (200, 25),
        (220, 25),
        (180, 25),
        (200, 50),
        (200, 0),
        (250, 25),
        (150, 25),
    ]

    for dis, height in test_cases:
        angle_3, angle_4, angle_5 = Arm(dis, height)
        safe = check_safe(angle_3, angle_4, angle_5)
        status = "安全" if safe else "超范围"
        print(f"dis={dis:3d}mm height={height:2d}mm -> "
              f"angle_3={angle_3:4d} angle_4={angle_4:4d} angle_5={angle_5:4d} [{status}]")


if __name__ == '__main__':
    main()
