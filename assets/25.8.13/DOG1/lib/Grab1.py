#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2024/10/5
# @Author  : WuJunYi
# @File    : Grab.py
# @Software: PyCharm

import math
import time
import cv2
import numpy as np
import xgolib
from lib.xgoled import XGO
import threading

# 定义XGODogController类，用于控制机器狗相关的操作，如移动、抓取、识别颜色等
class XGODogController:
    def __init__(self):
        """
        类的构造函数，用于初始化机器狗控制器的各种参数和资源。
        """
        # 创建一个xgolib.XGO对象，用于控制机器狗的基本动作，通过指定串口设备路径连接到机器狗。
        self.dog = xgolib.XGO("/dev/ttyAMA0")
        # 创建一个XGO对象，用于控制机器狗身上的LED灯，同样通过指定串口设备路径连接。
        self.led = XGO("/dev/ttyAMA0")
        # 用字典保存三种颜色阈值
        self.color_thresholds = {
            'red': [[[0, 45, 97], [25, 148, 245]]],
            'green': [[[61, 192, 75], [88, 255, 154]]],
            'blue': [[[79, 169, 65], [108, 255, 150]]]
        }
        self.m_angle, self.m_x, self.m_y = 0, 0, 0
        self.count = 0
        self.COUNT_MAX = 2
        self.flag1 = True
        self.flag2 = True
        self.lock = threading.Lock()
        # 最终要过滤的颜色保存为字符串，初始为空
        self.selected_color = None
    
    def output_color(self):
        """
        返回当前选定的要处理的目标颜色。

        :return: 选定的颜色字符串（如'red'、'green'、'blue'）
        """
        return self.selected_color

    def adjust_x(self, vx, runtime):
        """
        控制机器狗在x轴方向上移动指定的速度和时间。

        :param vx: x轴方向的移动速度
        :param runtime: 移动持续的时间
        """
        self.dog.move_x(vx)
        time.sleep(runtime)
        self.dog.move_x(0)

    def adjust_y(self, vy, runtime):
        """
        控制机器狗在y轴方向上移动指定的速度和时间。

        :param vy: y轴方向的移动速度
        :param runtime: 移动持续的时间
        """
        self.dog.move_y(vy)
        time.sleep(runtime)
        self.dog.move_y(0)

    def adjust_yaw(self, vyaw, runtime):
        """
        控制机器狗绕yaw轴（偏航轴）旋转指定的角度和时间。

        :param vyaw: yaw轴方向的旋转角度
        :param runtime: 旋转持续的时间
        """
        self.dog.turn(vyaw)
        time.sleep(runtime)
        self.dog.turn(0)

    # 新函数用于根据传入的颜色列表确定要使用的颜色范围
    def determine_color_ranges(self, image, color_list):
        """
        根据传入的图像和颜色列表，确定在图像中占主导地位的颜色，并设置为要处理的目标颜色。

        :param image: 输入的图像数据（通常是通过摄像头获取的帧）
        :param color_list: 要检测的颜色列表（如['red', 'green']）
        """
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pixel_counts = {}
        for color in color_list:
            for sub_thresholds in self.color_thresholds[color]:
                color_lower = np.array(sub_thresholds[0])
                color_upper = np.array(sub_thresholds[1])
                mask = cv2.inRange(hsv_img, color_lower, color_upper)
                pixel_counts[color] = np.sum(mask == 255)

        dominant_color = sorted(pixel_counts, key=pixel_counts.get, reverse=True)[0]
        print(dominant_color)
        self.selected_color = dominant_color

    def filter_img(self, frame):
        """
        根据选定的目标颜色对输入的图像帧进行过滤，只保留目标颜色所在的区域。

        :param frame: 输入的图像帧数据
        :return: 过滤后的图像，只包含目标颜色区域
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        for sub_thresholds in self.color_thresholds[self.selected_color]:
            color_lower = np.array(sub_thresholds[0])
            color_upper = np.array(sub_thresholds[1])
            if mask is None:
                mask = cv2.inRange(hsv, color_lower, color_upper)
            else:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, color_lower, color_upper))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        img_mask = cv2.bitwise_and(frame, frame, mask=mask)
        return img_mask

    def detect_contours(self, frame):
        """
        对输入的图像帧进行边缘检测和轮廓提取操作。

        :param frame: 输入的图像帧数据
        :return: 提取到的轮廓信息和边缘图像
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours, edges

    def detect_block(self, contours, frame):
        """
        在提取到的轮廓中检测符合特定条件的目标物体（可能是方块等），并返回相关信息。

        :param contours: 提取到的轮廓信息
        :param frame: 输入的图像帧数据
        :return: 是否检测到目标物体的标志，目标物体的长度、宽度、角度、中心x坐标、中心y坐标以及绘制了目标物体轮廓的图像帧
        """
        flag = False
        length, width, angle, s_x, s_y = 0, 0, 0, 0, 0
        for i in range(0, len(contours)):
            if cv2.contourArea(contours[i]) < 1000:
                continue
            rect = cv2.minAreaRect(contours[i])
            if 0.6 < rect[1][0] / rect[1][1] < 1:
                continue
            if not flag:
                if rect[2] > 45:
                    length = rect[1][0]
                    width = rect[1][1]
                    angle = rect[2]
                else:
                    length = rect[1][1]
                    width = rect[1][0]
                    angle = rect[2]
                s_x = rect[0][1]
                s_y = rect[0][0]
                flag = True
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 5)
            else:
                flag = False
                break
        return flag, length, width, angle, s_x, s_y, frame

    def ready_for_grasp(self):
        """
        对机器狗进行一些准备动作，使其处于适合抓取的姿态。
        """
        self.dog.reset()
        self.dog.attitude("p", 25)
        time.sleep(0.5)

    def grasp(self):
        """
        执行抓取动作的一系列操作，包括控制爪子、移动、调整电机等。
        """
        self.dog.claw(0)
        time.sleep(2)
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
        self.dog.motor(52,-55)
        time.sleep(2)

    def adjust(self, m_angle, m_x, m_y, des_x=440, des_y=320): # 350 300
        """
        根据目标位置和当前检测到的物体位置信息，对机器狗的位置进行调整。

        :param m_angle: 当前检测到物体的角度信息
        :param m_x: 当前检测到物体的x坐标信息
        :param m_y: 当前检测到物体的y坐标信息
        :param des_x: 目标位置的x坐标（默认值为440）
        :param des_y: 目标位置的y坐标（默认值为320）
        :return: 是否调整到合适位置可以进行抓取的标志
        """
        err_x = des_x - m_x
        err_y = des_y - m_y
        print(f"左右误差:{err_y}前后误差:{err_x}")
        self.dog.reset()
        if abs(err_y) < 40: # 20
            if abs(err_x) < 90: # 120
                return True
            else:
                self.dog.translation("x", -10)
                self.adjust_x(10, 1.2)
                self.dog.translation("x", 0)
        else:
            self.adjust_y(math.copysign(20, err_y), 0.5)
        time.sleep(0.3) # 0.3
        return False

    def search(self, color_list):
        """
        通过摄像头搜索指定颜色列表中的颜色物体，进行一系列的图像检测和处理操作。

        :param color_list: 要搜索的颜色列表
        """
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
        _, image = cap.read()
        # 在进入循环前确定颜色
        self.determine_color_ranges(image, color_list)
        while self.flag1:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture image")
                break
            filtered_img = self.filter_img(frame)
            contours, edges = self.detect_contours(filtered_img)
            flag, length, width, angle, s_x, s_y, frame_with_block = self.detect_block(contours, filtered_img)
            if flag:
                with self.lock:
                    self.count += 1
                    self.m_angle = (self.count - 1) / self.count * self.m_angle + angle / self.count
                    self.m_x = (self.count - 1) / self.count * self.m_x + s_x / self.count
                    self.m_y = (self.count - 1) / self.count * self.m_y + s_y / self.count
                print(f"search中的count:{self.count}")
                print(f"m_x:{self.m_x}m_y:{self.m_y}")
            if self.count == self.COUNT_MAX:
                with self.lock:
                    self.count = 0

            cv2.imshow("Filtered Image", filtered_img)
            cv2.imshow("Edges", edges)
            cv2.imshow("Contours", frame_with_block)

            if cv2.waitKey(1) == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    def move(self):
        """
        根据之前搜索到的物体信息，对机器狗进行移动操作，使其接近目标物体并尝试抓取。
        """
        while self.flag2:
            with self.lock:
                current_count = self.count
            if current_count == self.COUNT_MAX:
                with self.lock:
                    current_m_x = self.m_x
                    current_m_y = self.m_y
                    current_m_angle = self.m_angle
                    self.count = 0
                self.ready_for_grasp()    
                res = self.adjust(current_m_angle, current_m_x, current_m_y)
                self.ready_for_grasp()
                if res:
                    self.grasp()
                    self.flag1 = False
                    if self.selected_color == "blue":
                        self.led.rider_led(1, [0, 0, 255])
                        time.sleep(2.5)
                        self.led.rider_led(1, [0, 0, 0])
                    if self.selected_color == "red":
                        self.led.rider_led(1, [255, 0, 0])
                        time.sleep(2.5)
                        self.led.rider_led(1, [0, 0, 0])
                    if self.selected_color == "green":
                        self.led.rider_led(1, [0, 255, 0])
                        time.sleep(2.5)
                        self.led.rider_led(1, [0, 0, 0])
                    print(f"flag1:{self.flag1}")
                    self.flag2 = False
                    

    def place(self):
        """
        执行放置动作的一系列操作，包括控制机器狗的移动、电机等，将抓取的物体放置到指定位置。
        """
        self.dog.translation("x", 20)
        self.dog.motor(52, -55)
        time.sleep(0.5)
        self.dog.translation("z", 60)
        time.sleep(0.5)
        self.dog.motor(53, 80)
        self.dog.attitude("p", 20)
        time.sleep(2)
        self.dog.claw(0)
        time.sleep(2)
        self.dog.reset()
        time.sleep(10)
