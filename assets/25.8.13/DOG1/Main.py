#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :2024/10/5
# @Author  :WuJunYi
# @File    :Path.py
# @Software:PyCharm

import cv2
import time
import xgolib
import threading
import subprocess
from xgoedu import XGOEDU
from lib.Motion import Motion
from lib.Grab1 import XGODogController
from lib.GetColor import ColorInfoGetter
from lib.ClientSocket import ColorSocket, Enhance

from lib.dog_network.image_remote_process import send_image_with_retry, LABELS_CN

def detect_img():
    """
    函数功能：
    - 连接到远程服务器，采集摄像头图像并保存。
    - 读取保存的图片并发送到远程服务器进行识别。
    - 接收识别结果并打印检测到的物体类别、置信度和坐标。
    """
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

    _, frame = cap.read()
    cv2.imwrite("test1.jpg", frame)

    computer_ip = "192.168.136.73"  # 服务器 IP 地址
    res = send_image_with_retry(
        frame, identify="xgo1", timeout=1.0, computer_ip=computer_ip, max_retries=3
    )

    if res is None:
        print("未检测到物体。")
        return
    xyxy = res["xyxy"]
    conf = res["conf"]
    cls_id = res["cls"]
    print(f"检测到{LABELS_CN[cls_id]}，置信度：{conf:.2f}，坐标：{xyxy}")

def showPicture_wait():
    """
    函数功能：
    - 初始化XGO_edu对象和服务器对象。
    - 在LCD上显示'blue.jpg'图片。
    - 等待服务器连接和接收标志，然后关闭服务器连接，最后清除LCD显示。
    """
    XGO_edu = XGOEDU()
    server = ColorSocket("0.0.0.0", 8888)
    XGO_edu.lcd_picture("blue.jpg")
    server.wait_for_connection()
    received_flag = server.wait_for_flag()
    server.close_connection()
    time.sleep(0.5)
    XGO_edu.lcd_clear()
    
def start_grab():
    """
    函数功能：
    - 准备机械臂进行抓取操作。
    - 创建两个线程，分别用于搜索和移动操作，启动线程并等待它们完成。
    """
    global column_color, controller
    controller.ready_for_grasp()
    t1 = threading.Thread(target=controller.search, args=(column_color,))
    t2 = threading.Thread(target=controller.move)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    
def start_get_color():
    """
    函数功能：
    - 初始化颜色信息获取对象。
    - 调整机器狗姿态，获取颜色信息并添加到列表中，最后恢复机器狗姿态。
    """
    global column_color
    color = ColorInfoGetter()
    dog.attitude("p", 25)
    time.sleep(0.5)
    Color = color.start_get()
    column_color.append(Color)
    dog.attitude("p", 0)
    time.sleep(0.5)

def get_column_color():
    """
    函数功能：
    - 初始化颜色信息获取对象。
    - 调整机器狗姿态，获取当前颜色信息，最后恢复机器狗姿态。
    """
    global now_column_color
    color = ColorInfoGetter()
    dog.attitude("p", 25)
    time.sleep(0.5)
    Color = color.start_get()
    now_column_color = Color
    dog.attitude("p", 0)
    time.sleep(0.5)
    
now_column_color = None
column_color = ['red', 'red']
catched_rectangle_color = None

if __name__ == '__main__':

    dog = xgolib.XGO("/dev/ttyAMA0")
    controller = XGODogController()
    move = Motion()
    
    # dog.move_x(10)
    # time.sleep(2.5)
    # dog.move_x(0)
    # time.sleep(0.5)
    # #
    # detect_img()
    # #斜向前进 好了
    # move.change_yaw(-60)
    # dog.move_x(25)
    # time.sleep(2.3)
    # dog.move_x(0)
    # time.sleep(0.5)
    # #回正
    # move.change_yaw(0)
    # #前进 好了
    # dog.move_x(25)
    # time.sleep(1.3)
    # dog.move_x(0)
    # time.sleep(0.5)
    # #
    # detect_img()
    # #斜向前进 好了
    # move.change_yaw(40)
    # dog.move_x(25)
    # time.sleep(2.7)
    # dog.move_x(0)
    # time.sleep(0.5)
    # #转动回正 好了
    # move.change_yaw(90)
    # dog.move_x(25)
    # time.sleep(2.5)
    # dog.move_x(0)
    # time.sleep(0.5)
    # move.change_yaw(0)
    # time.sleep(0.5)
    # #
    # detect_img()
    # #斜向前进 好啦
    # move.change_yaw(-50)
    # dog.move_x(25)
    # time.sleep(4.3)
    # dog.move_x(0)
    # time.sleep(0.5)
    # #回正
    # move.change_yaw(0)
    #可以不要
    # dog.move_x(10)
    # time.sleep(5)
    # dog.move_x(0)
    # time.sleep(0.5)
    # move.change_yaw(0)
    print("-------------------")
    global flag1, flag2
    flag1 = True
    flag2 = True
    
    start_grab()
    
    #斜向前进
    move.change_yaw(150)
    dog.turn(0)
    time.sleep(0.5)
    
    dog.move_x(10)
    time.sleep(13)
    dog.move_x(0)
    time.sleep(0.5)
    
    dog.turn(-35)
    time.sleep(2)
    dog.turn(0)
    time.sleep(0.5)
    dog.move_x(10)
    time.sleep(2)
    dog.move_x(0)
    time.sleep(0.5)
    subprocess.Popen(['python3', '/home/pi/Desktop/RaspberryPi-CM4-main-1030/Main/photo_adjust.py'])