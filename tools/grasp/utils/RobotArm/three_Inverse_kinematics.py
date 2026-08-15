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

# 相机在夹爪连杆坐标系下的安装位置，单位 mm。
# 坐标系定义：原点 J2，x 沿 L3 指向夹爪，y 垂直 L3 向外。
# 实测安装位置：(CAM_J2_X, CAM_J2_Y) = (15, 50)
CAM_J2_X = 15.0
CAM_J2_Y = 50.0


def Arm(x=None, y=None, theta_deg=0):
    """
    三连杆逆运动学。
    输入末端目标坐标 (x, y) 与 L3 姿态角 theta_deg（L3 与 X 轴夹角，单位度）。
    返回 (angle_3, angle_4, angle_5) 舵机脉冲值。
    """
    pi = math.pi

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
    输入期望的相机目标坐标 (x, y)（基座世界坐标系）与 L3 姿态角 theta_deg，
    返回 (angle_3, angle_4, angle_5)。

    相机安装位置在夹爪连杆坐标系下为 (CAM_J2_X, CAM_J2_Y)，原点为 J2。
    世界坐标 = J2_world + R(phi3) * [CAM_J2_X, CAM_J2_Y]^T
    反解：J2_world = cam_world - R(phi3) * [CAM_J2_X, CAM_J2_Y]^T
    再由 J2 沿 phi3 走 L3 得 end_world，传入 Arm()。
    """
    if x is None:
        x = int(input("camera x:"))
    if y is None:
        y = int(input("camera y:"))

    theta = math.radians(theta_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # J2 的目标世界坐标
    j2_x = x - (CAM_J2_X * cos_t - CAM_J2_Y * sin_t)
    j2_y = y - (CAM_J2_X * sin_t + CAM_J2_Y * cos_t)

    # end 的目标世界坐标（J2 沿 L3 方向延伸）
    end_x = j2_x + L3 * cos_t
    end_y = j2_y + L3 * sin_t

    print("\n[相机目标] x=%.1f  y=%.1f  theta=%d°  ->  [J2目标] x=%.1f  y=%.1f  ->  [末端目标] x=%.1f  y=%.1f"
          % (x, y, theta_deg, j2_x, j2_y, end_x, end_y))

    return Arm(end_x, end_y, theta_deg=theta_deg)
