#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :2024/10/5
# @Author  :WuJunYi
# @File    :Client.py
# @Software:PyCharm

import cv2
from lib.ClientSocket import ClientSocket, Enhance

if __name__ == '__main__':
    client = ClientSocket("192.168.2.220", 8888)

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    _, frame = cap.read()
    cv2.imwrite("test1.jpg", frame)

    # 发送图片（假设从文件读取图片数据）
    image = cv2.imread('test1.jpg')

    # 处理原图片
    image = Enhance.enhance_image(image)

    cv2.imwrite('test2.jpg', image)

    _, img_encoded = cv2.imencode('.jpg', image)
    image_data = img_encoded.tobytes()
    client.send_data(image_data)

    client.close()
