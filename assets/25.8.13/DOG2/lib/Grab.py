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


# 定义XGODogController类，用于控制机器狗相关的操作，如移动、抓取、识别颜色等
class XGODogController:
    def __init__(self):
        """
        类的构造函数，用于初始化机器狗控制器的各种参数和资源。
        """
        self.dog = xgolib.XGO("/dev/ttyAMA0") # xgolib.XGO对象，用于控制机器狗的基本动作
        self.led = XGO("/dev/ttyAMA0") # XGO对象，用于控制机器狗身上的LED灯
        self.color_thresholds = {
            'red': ([0, 97, 141], [179, 144, 185]),
            'green': ([61, 192, 75], [88, 255, 154]),
            'blue': ([79, 169, 65], [108, 255, 150])
        }
        self.detection_counts = {
            "red": 0,
            "blue": 0,
            "green": 0
        }
        self.MAX_COUNT = 2
        self.global_error_x = None
        self.global_error_y = None
        self.flag = False
        self.flag1 = True
        self.flag2 = True
        self.area = 0
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
        self.dog.motor(52, -55)
        time.sleep(2)

    def adjust(self, m_x, m_s):
        """
        根据误差调整机器人位置

        Args:
            m_x (int): x轴方向的误差
            m_y (int): y轴方向的误差
            m_s (int): 面积信息
            des_x (int): x轴方向的目标位置，默认为350
            des_y (int): y轴方向的目标位置，默认为120

        Returns:
            bool: 是否满足抓取条件
        """
        err_x = -m_x
        s = m_s
        print(f"左右误差:{err_x}area:{s}")
        self.dog.attitude("p", 0)
        time.sleep(0.5)
        if abs(err_x) < 30:
            if s > 12500:
                return True
            else:
                self.dog.translation("x", -10)
                self.adjust_x(10, 1)
                self.dog.translation("x", 0)
        else:
            self.adjust_y(math.copysign(15, err_x), 0.5)
        time.sleep(0.3)
        return False

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
            sub_thresholds = self.color_thresholds[color]
            color_lower = np.array(sub_thresholds[0])
            color_upper = np.array(sub_thresholds[1])
            mask = cv2.inRange(hsv_img, color_lower, color_upper)
            pixel_counts[color] = np.sum(mask == 255)

        dominant_color = sorted(pixel_counts, key=pixel_counts.get, reverse=True)[0]
        print(pixel_counts)
        print(dominant_color)
        self.selected_color = dominant_color

    def detect_cuboids(self, frame):
        """
        在给定的图像帧中检测颜色圆柱体

        Args:
            frame (numpy.ndarray): 输入的图像帧

        Returns:
            tuple: 包含处理后的图像帧、过滤后的图像帧、检测到的颜色列表、误差信息列表、标志位、面积信息
        """
        self.flag = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        filtered_frame = None
        detected_colors = []
        errors = []

        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        if self.selected_color:
            color_name = self.selected_color
            lower, upper = self.color_thresholds[color_name]
            lower_bound = np.array(lower, dtype="uint8")
            upper_bound = np.array(upper, dtype="uint8")

            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            filtered_frame = mask

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) < 1000:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                self.area = w * h
                aspect_ratio = w / float(h)

                if 0 < aspect_ratio < 5:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, f"{color_name} cubiod", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 255),
                                2)
                    detected_colors.append(color_name)

                    self.detection_counts[color_name] += 1

                    if self.detection_counts[color_name] == self.MAX_COUNT:
                        self.detection_counts[color_name] = 0
                        self.flag = True

                    cylinder_center_x = x + w // 2
                    cylinder_center_y = y + h // 2

                    error_x = cylinder_center_x - frame_center_x
                    error_y = cylinder_center_y - frame_center_y

                    self.global_error_x = error_x
                    self.global_error_y = error_y

                    errors.append((color_name, error_x, error_y))

        return frame, filtered_frame, detected_colors, errors, self.flag, self.area

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
        print(self.selected_color)
        while self.flag1:
            ret, frame = cap.read()
            if not ret:
                print("无法读取帧")
                break

            frame, filtered_frame, detected_colors, errors, flag, area = self.detect_cuboids(frame)

            cv2.imshow("Original Frame", frame)
            if filtered_frame is not None:
                cv2.imshow("Filtered Frame", filtered_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def move(self):
        """
        根据之前搜索到的物体信息，对机器狗进行移动操作，使其接近目标物体并尝试抓取。
        """
        self.flag2 = True
        while self.flag2:
            if self.flag:
                print(f"Moving with errors - X: {self.global_error_x}, Y: {self.global_error_y}")
                self.ready_for_grasp()
                res = self.adjust(self.global_error_x, self.area)
                self.ready_for_grasp()
                if res:
                    self.grasp()
                    self.flag1 = False
                    self.flag2 = False

                self.global_error_x = None
                self.global_error_y = None
                self.flag = False

            time.sleep(0.1)
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

