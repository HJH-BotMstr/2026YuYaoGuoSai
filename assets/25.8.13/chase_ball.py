from typing import Optional
import cv2.typing
import numpy as np
import cv2
from xgolib import XGO
from pydantic import BaseModel

from cv2.typing import MatLike

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
print("摄像头初始化完毕")

# dog = XGO(port="/dev/ttyAMA0", version="xgolite")


def normalize_brightness(
    bgr_img: MatLike, clip_limit=2.0, grid_size=(8, 8)
) -> np.ndarray:
    """
    对BGR图像的亮度通道进行自适应直方图均衡化

    参数:
        bgr_img (numpy.ndarray): BGR格式的输入图像
        clip_limit (float): CLAHE的对比度限制阈值
        grid_size (tuple): CLAHE的网格尺寸

    返回:
        numpy.ndarray: 处理后的LAB图像
    """
    # 检查输入是否为有效的BGR图像
    if bgr_img is None or bgr_img.size == 0:
        raise ValueError("输入图像为空")

    # 转换到LAB色彩空间
    lab_img = cv2.cvtColor(bgr_img, cv2.COLOR_RGB2LAB)

    # 分离LAB通道
    l_channel, a_channel, b_channel = cv2.split(lab_img)

    # 创建CLAHE对象并应用于L通道（亮度）
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
    normalized_l = clahe.apply(l_channel)

    # 合并处理后的通道
    return cv2.merge([normalized_l, a_channel, b_channel])


class BallInfo(BaseModel):
    """
    识别到的小球信息

    x 在 -1 ~ 1
    y 在 -1 ~ 1
    r 与输入图像大小无关（不改变宽高比）
    """

    x: float
    y: float
    r: float


def binarize_in_lab(
    lab_img, l_range=(117, 245), a_range=(80, 112), b_range=(144, 167)
) -> np.ndarray:
    """
    在LAB色彩空间中通过阈值二值化过滤色彩区域

    在线调整LAB阈值: https://wiki.sipeed.com/threshold

    OepnCV LAB 定义: https://docs.opencv.org/4.x/de/d25/imgproc_color_conversions.html

    L <- L*255/100, a <- a+128, b <- b+128

    参数:
        lab_img (numpy.ndarray): 输入的LAB格式图像
        l_range (tuple): 亮度L通道阈值范围
        a_range (tuple): A通道阈值范围
        b_range (tuple): B通道阈值范围

    返回:
        numpy.ndarray: 二值化图像(0或255值)
    """
    # 分离LAB通道
    l_channel, a_channel, b_channel = cv2.split(lab_img)

    # 创建阈值掩膜
    l_mask = cv2.inRange(l_channel, l_range[0], l_range[1])
    a_mask = cv2.inRange(a_channel, a_range[0], a_range[1])
    b_mask = cv2.inRange(b_channel, b_range[0], b_range[1])

    # 组合多个通道的掩膜
    mask = cv2.bitwise_and(l_mask, a_mask)
    mask = cv2.bitwise_and(mask, b_mask)

    return mask


def find_largest_circle(
    binary_img: np.ndarray, min_radius=0, max_radius=0.7
) -> Optional[BallInfo]:
    """
    对二值化图像进行形态学处理后，找出最大的圆

    参数:
        binary_img (numpy.ndarray): 二值化图像
        min_radius (float): 检测圆的最小半径
        max_radius (float): 检测圆的最大半径

    返回:
        BallInfo | None
        未找到轮廓或半径不在范围内则返回None，不然返回BallInfo
    """
    # 1. 形态学操作预处理
    # 创建椭圆和矩形核
    kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    # 开运算去除噪声和小物体
    cleaned = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel_ellipse, iterations=2)

    # 闭运算填充孔洞
    processed = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel_rect, iterations=3)

    # 2. 寻找最大轮廓
    contours, _ = cv2.findContours(
        processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    # 找到面积最大的轮廓
    largest_contour = max(contours, key=cv2.contourArea)

    # 3. 检测轮廓中的圆
    (x, y), r = cv2.minEnclosingCircle(largest_contour)

    # 归一化数值
    h, w = binary_img.shape

    x = 2 * (x - w / 2) / w
    y = -2 * (y - h / 2) / h
    r = r / ((w + h) / 2)

    # 检查半径是否在有效范围内
    if not (min_radius <= r <= max_radius):
        return None

    return BallInfo(x=x, y=y, r=r)


def find_ball(img: np.ndarray) -> BallInfo:
    img = normalize_brightness(img)
    img = binarize_in_lab(img)
    ball_info = find_largest_circle(img)


def test_bright():
    img = cv2.imread("test.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    img = normalize_brightness(img)
    img = cv2.cvtColor(img, cv2.COLOR_LAB2RGB)
    cv2.imwrite("test_bright.jpg", img)


def test_bin():
    img = cv2.imread("test_bright.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    bin = binarize_in_lab(img)
    cv2.imshow("bin", bin)
    cv2.waitKey(0)


def test_find_largest_circle():
    img = cv2.imread("test_bright.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    img = binarize_in_lab(img)
    res = find_largest_circle(img)
    assert res is not None
    print(res)

def keep_center(ball: BallInfo):
    """控制机械狗保持小球在画面中心"""
    # 这些值需要根据实际机器狗进行调整
    max_yaw = 11    # 最大左右旋转量
    max_pitch = 15  # 最大俯仰量
    gain = 1.0     # 控制增益系数，值越大反应越快
    
    # 将归一化位置转换为控制偏移量
    # x方向：右正左负（对应机器狗的yaw）
    # y方向：上正下负（对应机器狗的pitch）
    yaw_offset = -ball.x * max_yaw  # 取负号因为机器狗转动方向与小球方向相反
    pitch_offset = ball.y * max_pitch
    
    # 应用增益并限幅防止过大的运动
    yaw_offset = np.clip(yaw_offset / gain, -max_yaw, max_yaw)
    pitch_offset = np.clip(pitch_offset / gain, -max_pitch, max_pitch)
    
    try:
        # 如果机器狗对象存在，发送控制命令
        dog
        dog.attitude(['y', 'p'], [yaw_offset, pitch_offset])
        print(f"控制命令: yaw={yaw_offset:.2f}, pitch={pitch_offset:.2f}")
    except NameError:
        # 如果没有连接机器狗，只打印控制值
        print(f"（无机器狗）控制模拟: yaw={yaw_offset:.2f}, pitch={pitch_offset:.2f}")

if __name__ == "__main__":
    while True:
        # 1. 获取摄像头帧
        ret, frame = cap.read()
        if not ret: break
        
        # 2. 处理图像流程
        normalized = normalize_brightness(frame)  # 亮度归一化
        binary = binarize_in_lab(normalized)     # 颜色二值化
        ball = find_largest_circle(binary)       # 查找小球
        
        # 3. 输出结果
        if ball:
            print(f"检测到小球: X={ball.x:.2f}, Y={ball.y:.2f}, 半径={ball.r:.2f}")
            keep_center(ball)
            # 可视化标记（可选）
            h, w = frame.shape[:2]
            center_x = int(w/2 * (1 + ball.x))
            center_y = int(h/2 * (1 - ball.y))  # Y轴反向
            radius = int(ball.r * (w+h)/4)
            cv2.circle(frame, (center_x, center_y), radius, (0,255,0), 2)
        
        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()