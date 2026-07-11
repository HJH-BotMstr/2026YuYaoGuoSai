'''
Time:2023.3.22
Company:小二极客科技有限公司
Use:Inverse kinematics algorithm for a three link manipulator(三连杆接机械臂逆运动学算法)
      增加：机械臂末端安装相机的坐标逆运动学
'''

import math

# 连杆长度，单位 mm
L1 = 105     # L1底座
L2 = 110     # L2
L3 = 110     # L3末端

# 相机在末端连杆坐标系下的固定偏移。
# 坐标系定义：x 沿 L3 从关节 3 指向末端，y 垂直于 L3 指向左侧。
# 由机械臂竖直（所有舵机 2047）时实测得到：
#   末端坐标 (0, 325)，相机坐标 (5, 216.5)
# 解得相机相对末端的偏移为 (-108.5, -5.0) mm
CAM_OFFSET_X = -108.5
CAM_OFFSET_Y = -5.0


def Arm(x=None, y=None, theta_deg=0):
    """
    三连杆逆运动学。
    输入末端目标坐标 (x, y) 与 L3 姿态角 theta_deg（L3 与 X 轴夹角，单位度）。
    返回 (angle_3, angle_4, angle_5) 舵机脉冲值。
    """
    pi = 3.14

    if x is None:
        x = int(input("x:"))
    if y is None:
        y = int(input("y:"))
    theta = math.radians(theta_deg)

    # 计算中间位置 Bx,By，即第二个关节（L2 末端 / L3 起点）的位置
    Bx = x - L3 * math.cos(theta)
    By = y - L3 * math.sin(theta)

    # 二连杆逆运动学求 q1, q2
    lp = Bx**2 + By**2
    alpha = math.atan2(By, Bx)
    tmp = (L1*L1 + lp - L2*L2) / (2*L1*math.sqrt(lp))
    if tmp < -1:
        tmp = -1
    elif tmp > 1:
        tmp = 1
    beta = math.acos(tmp)
    q1 = -(pi/2.0 - alpha - beta)

    tmp = (L1*L1 + L2*L2 - lp) / (2*L1*L2)
    if tmp < -1:
        tmp = -1
    elif tmp > 1:
        tmp = 1
    q2 = math.acos(tmp) - pi

    # 第三个关节角，使 L3 达到目标姿态 theta
    q3 = theta - q1 - q2 - pi/2

    # 舵机脉冲转换
    # 3号：角度为正 数值减小；4、5号：角度为正 数值增大
    angle_5 = int(2047 + int(math.degrees(q1) * 11.375))
    angle_4 = int(2047 + int(math.degrees(q2) * 11.375))
    angle_3 = int(2047 - int(math.degrees(q3) * 11.375))

    print("-------------------------")
    print("theta = ", theta_deg)
    print("5 = ", int(math.degrees(q1)))
    print("4 = ", int(math.degrees(q2)))
    print("3 = ", int(math.degrees(q3)))
    print("-------------------------")
    print("angle_5 = ", angle_5)
    print("angle_4 = ", angle_4)
    print("angle_3 = ", angle_3)

    return angle_3, angle_4, angle_5


def ArmCamera(x=None, y=None, theta_deg=0):
    """
    相机坐标逆运动学。
    输入期望的相机目标坐标 (x, y) 与 L3 姿态角 theta_deg，
    内部换算成末端坐标后调用 Arm()，返回 (angle_3, angle_4, angle_5)。
    """
    if x is None:
        x = int(input("camera x:"))
    if y is None:
        y = int(input("camera y:"))

    theta = math.radians(theta_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # 末端坐标 = 相机坐标 - 相机在末端坐标系下的偏移
    end_x = x - (CAM_OFFSET_X * cos_t - CAM_OFFSET_Y * sin_t)
    end_y = y - (CAM_OFFSET_X * sin_t + CAM_OFFSET_Y * cos_t)

    print("\n[相机目标] x=%.1f  y=%.1f  theta=%d°  ->  [末端目标] x=%.1f  y=%.1f"
          % (x, y, theta_deg, end_x, end_y))

    return Arm(end_x, end_y, theta_deg=theta_deg)
