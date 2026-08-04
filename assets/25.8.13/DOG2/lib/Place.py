import cv2
import time
import math
import threading
import numpy as np
from xgolib import XGO

# 定义一个名为CylinderDetectionAndPlacement的类，用于实现圆柱体检测和放置相关的功能
class CylinderDetectionAndPlacement:
    def __init__(self, port='/dev/ttyAMA0', version="xgomini"):
        """
        类的构造函数，用于初始化CylinderDetectionAndPlacement类的实例。

        :param port: 设备连接的端口号，默认为'/dev/ttyAMA0'，用于与特定设备（可能是机器人等）进行通信。
        :param version: 设备的版本信息，默认为"xgomini"。
        """
        self.dog = XGO(port=port, version=version)

        # 定义颜色的HSV范围
        self.color_ranges = {
            "red": ([0, 97, 141], [179, 144, 185]),
            "blue": ([79, 169, 65], [108, 255, 150]),
            "green": ([61, 192, 75], [88, 255, 154])
        }

        # 初始化颜色检测计数
        self.detection_counts = {
            "red": 0,
            "blue": 0,
            "green": 0
        }

        # 定义达到的检测次数上限
        self.MAX_COUNT = 2

        # 定义全局变量来存储误差和标志
        self.global_error_x = None
        self.global_error_y = None
        self.flag = False
        self.area = 0

    def adjust_x(self, vx, runtime):
        """
        控制设备在x轴方向上移动指定的速度和时间。

        :param vx: x轴方向的移动速度。
        :param runtime: 移动持续的时间。
        """
        self.dog.move_x(vx)
        time.sleep(runtime)
        self.dog.move_x(0)

    def adjust_y(self, vy, runtime):
        """
        控制设备在y轴方向上移动指定的速度和时间。

        :param vy: y轴方向的移动速度。
        :param runtime: 移动持续的时间。
        """
        self.dog.move_y(vy)
        time.sleep(runtime)
        self.dog.move_y(0)

    def grasp(self):
        """
        执行抓取动作的一系列操作，包括控制爪子、移动、调整电机等，用于抓取圆柱体。
        """
        self.dog.claw(0)
        self.dog.translation("x", 20)
        self.dog.motor(52, -55)
        time.sleep(0.5)
        self.dog.translation("z", 60)
        time.sleep(0.5)
        self.dog.motor(53, 90)
        self.dog.attitude("p", 20)
        time.sleep(2)
        self.dog.claw(255)
        time.sleep(2)
        self.dog.reset()
        time.sleep(2)
        self.dog.motor(52, -55)
        time.sleep(2)

    def ready_for_place(self):
        """
        对设备进行一些准备动作，使其处于适合放置圆柱体的姿态。
        """
        self.dog.attitude("p", 18)
        time.sleep(0.5)

    def detect_cylinders(self, frame):
        """
        在输入的图像帧中检测符合条件的圆柱体，并返回相关信息，包括处理后的图像、检测到的颜色、误差等。

        :param frame: 输入的图像帧数据，通常是通过摄像头获取的。
        :return: 处理后的原始图像、颜色过滤后的图像、检测到的颜色列表、误差信息列表、是否达到检测次数上限的标志以及圆柱体的面积。
        """
        self.flag = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        filtered_frame = None  # 用于保存颜色过滤后的图像
        detected_colors = []  # 用于保存检测到的颜色圆柱体信息
        errors = []  # 用于保存误差信息

        # 获取相机视图的中心点
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        for color_name, (lower, upper) in self.color_ranges.items():
            lower_bound = np.array(lower, dtype="uint8")
            upper_bound = np.array(upper, dtype="uint8")

            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            if filtered_frame is None:
                filtered_frame = mask
            else:
                filtered_frame = cv2.bitwise_or(filtered_frame, mask)

            # 使用findContours找到颜色区域的轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # 忽略小轮廓
                if cv2.contourArea(contour) < 1000:
                    continue

                # 计算轮廓的矩形边界框
                x, y, w, h = cv2.boundingRect(contour)
                self.area = w * h
                print(f"area:{self.area}")

                # 计算长宽比来识别圆柱体（直径大，高度小）
                aspect_ratio = w / float(h)
                print(f"aspect_ratio:{aspect_ratio}")

                if 1 < aspect_ratio < 5:  # 根据你的圆柱体的形状调整这个阈值
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, f"{color_name} cylinder", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255),
                                2)
                    detected_colors.append(color_name)

                    # 更新检测计数
                    self.detection_counts[color_name] += 1

                    # 检查计数是否达到最大值
                    if self.detection_counts[color_name] == self.MAX_COUNT:
                        # 当计数达到5时，清零并执行相应操作
                        self.detection_counts[color_name] = 0
                        self.flag = True

                    # 计算圆柱体的中心点
                    cylinder_center_x = x + w // 2
                    cylinder_center_y = y + h // 2

                    # 计算圆柱体中心点与相机中心点的误差
                    error_x = cylinder_center_x - frame_center_x
                    error_y = cylinder_center_y - frame_center_y

                    # 更新全局变量
                    self.global_error_x = error_x
                    self.global_error_y = error_y

                    errors.append((color_name, error_x, error_y))

        return frame, filtered_frame, detected_colors, errors, self.flag, self.area

    def adjust(self, m_x, m_y, m_s, des_x=350, des_y=120):
        """
        根据圆柱体与目标位置的误差以及圆柱体的面积等信息，对设备的位置进行调整。

        :param m_x: 圆柱体在x方向上的位置信息（可能是相对于相机视图的坐标等）。
        :param m_y: 圆柱体在y方向上的位置信息。
        :param m_s: 圆柱体的面积信息。
        :param des_x: 目标位置在x方向上的坐标，默认为350。
        :param des_y: 目标位置在y方向上的坐标，默认为120。
        :return: 是否调整到合适位置可以进行放置操作的标志。
        """
        err_x = -m_x
        err_y = des_y - m_y
        s = m_s
        print(f"左右误差:{err_x}area:{s}")
        self.dog.attitude("p", 0)
        time.sleep(0.5)
        if abs(err_x) < 40:
            if s > 14000:
                return True
            else:
                self.dog.translation("x", -10)
                self.adjust_x(10, 1)  # 10
                self.dog.translation("x", 0)
        else:
            self.adjust_y(math.copysign(20, err_x), 0.5)  # 20
        time.sleep(0.3)
        return False

    def place(self):
        """
        执行放置动作的一系列操作，包括控制设备的移动、电机等，将抓取的圆柱体放置到指定位置。
        """
        self.dog.translation("x", 20)
        self.dog.motor(52, -55)
        time.sleep(0.5)
        self.dog.translation("z", 100)
        time.sleep(0.5)
        self.dog.motor(53, 90)
        self.dog.attitude("p", 10)
        time.sleep(2)
        self.dog.claw(0)
        time.sleep(2)
        self.dog.reset()
        time.sleep(10)

    def main(self):
        """
        主函数，用于打开摄像头获取图像帧，调用detect_cylinders函数进行圆柱体检测，并显示处理后的图像。
        """
        cap = cv2.VideoCapture(0)  # 打开摄像头
        cap.set(3, 640)
        cap.set(4, 480)

        if not cap.isOpened():
            print("无法打开摄像头")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取帧")
                break

            frame, filtered_frame, detected_colors, errors, self.flag, self.area = self.detect_cylinders(frame)

            cv2.imshow("Original Frame", frame)
            if filtered_frame is not None:
                cv2.imshow("Filtered Frame", filtered_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def move(self):
        """
        根据检测到的圆柱体位置误差等信息，对设备进行移动操作，使其接近目标位置并尝试放置圆柱体。
        """
        self.dog.motor(52,-55)
        time.sleep(2)
        while True:
            if self.flag:
                # 在这里使用 error_x 和 error_y 执行移动操作
                print(f"Moving with errors - X: {self.global_error_x}, Y: {self.global_error_y}")
                # 在这里添加你的移动逻辑代码
                self.ready_for_place()
                res = self.adjust(self.global_error_x, self.global_error_y, self.area)
                self.ready_for_place()
                if res:
                    self.place()
                    break

                self.global_error_x = None  # 重置全局变量
                self.global_error_y = None
                self.flag = False  # 重置 flag 变量

            time.sleep(0.1)  # 添加延迟避免过度占用CPU

    def run(self):
        """
        创建两个线程，分别执行main函数和move函数，实现同时进行图像获取与处理以及设备移动控制的功能。
        """
        t1 = threading.Thread(target=self.main)
        t2 = threading.Thread(target=self.move)
        t1.start()
        t2.start()
        t1.join()
        t2.join()

if __name__ == "__main__":
    detector_and_placement = CylinderDetectionAndPlacement()
    detector_and_placement.run()
