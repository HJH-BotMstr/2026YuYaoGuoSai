#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :2024/10/5
# @Author  :WuJunYi
# @File    :ClientSocket.py
# @Software:PyCharm

import socket
import struct
import cv2
import logging
import time
import numpy as np
from lib.Sound import Sound

# 设置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# 定义客户端套接字类，用于与服务器进行通信并发送数据
class ClientSocket:
        def __init__(self, ip_address, port):
                """
                类的构造函数，用于初始化客户端套接字对象

                :param ip_address: 服务器的IP地址，指定要连接的服务器位置
                :param port: 服务器的端口号，用于确定在服务器上的哪个服务端口进行连接
                """
                
                # 创建一个基于IPv4地址族（AF_INET）和TCP协议（SOCK_STREAM）的套接字对象
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # 使用创建的套接字对象连接到指定的服务器IP地址和端口号
                self.socket.connect((ip_address, port))

        def send_data(self, data):
                """
                根据传入数据的类型，选择合适的发送方法将数据发送到服务器

                :param data: 要发送的数据，可以是字符串类型或字节类型（用于发送图片数据等）
                """
                if isinstance(data, str):
                        self.send_string(data)
                elif isinstance(data, bytes):
                        self.send_image(data)
                else:
                        raise ValueError("Unsupported data type. Only strings and images (bytes) are supported.")

        def send_string(self, string_data):
                """
                发送字符串数据到服务器的具体方法

                :param string_data: 要发送的字符串内容
                """
                # 发送字符串长度
                string_length = len(string_data)
                # 将字符串长度打包成网络字节序（大端序）的无符号4字节整数格式，以便在网络中传输
                self.socket.sendall(struct.pack('!I', string_length))
                # 发送字符串内容
                self.socket.sendall(string_data.encode())

        def send_image(self, image_data):
                """
                发送图片数据到服务器的具体方法，并接收服务器的响应，根据响应让狗子发出相应声音

                :param image_data: 要发送的图片数据，以字节流形式存在
                """
                # 发送图片数据长度
                image_length = len(image_data)
                self.socket.sendall(struct.pack('!I', image_length))
                # 发送图片数据
                self.socket.sendall(image_data)
                logging.info(f"向服务器发送图片数据成功，大小为 {image_length} 字节")
                # 接收服务器的响应
                response = self.socket.recv(1024).decode('utf-8')
                logging.info(f"接收到服务器响应：{response}")
                # 狗子发声
                Sound.play_sound(animal_name = response)

        def close(self):
                """
                关闭客户端套接字连接的方法
                """
                if self.socket:
                        self.socket.close()

# 定义图像增强类，用于对图像进行一些增强处理操作
class Enhance:
        @staticmethod
        def enhance_image(image):
                """
                对输入的图像进行增强处理的静态方法

                :param image: 要进行增强处理的图像数据，通常是一个numpy数组表示的图像
                :return: 经过增强处理后的图像数据
                """
                # 增亮
                brightened_image = np.clip(image + 35, 0, 255).astype(np.uint8)

                # 将增亮后的彩色图转换为 HSV 色彩空间
                hsv_image = cv2.cvtColor(brightened_image, cv2.COLOR_BGR2HSV)
                value_channel = hsv_image[:, :, 2]

                # 对亮度通道进行直方图均衡化
                equalized_value = cv2.equalizeHist(value_channel)
                hsv_image[:, :, 2] = equalized_value
                # 将处理后的 HSV 图像转换回 BGR 色彩空间
                enhanced_color_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

                return enhanced_color_image

# 定义服务器套接字类，用于监听客户端连接并处理相关操作
class ColorSocket:
        def __init__(self, target_ip='192.168.31.181', port=8888):
                """
                类的构造函数，用于初始化服务器套接字对象并开始监听指定端口

                :param target_ip: 服务器要监听的IP地址，默认设置为'192.168.31.181'
                :param port: 服务器要监听的端口号，默认设置为8888
                """
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # 将服务器套接字绑定到指定的IP地址和端口号上，使其能够在该地址和端口监听客户端连接请求
                self.server_socket.bind((target_ip, port))
                # 开始监听客户端连接请求，参数1表示最大允许同时连接的客户端数量为1个
                self.server_socket.listen(1)
                self.client_socket = None
                print(f"等待在 IP {target_ip} 上的客户端连接...")
        
        def wait_for_connection(self):
                """
                等待并接受客户端连接的方法，当有客户端连接成功后，会记录客户端的地址信息并打印连接成功的提示信息
                """
                self.client_socket, self.client_address = self.server_socket.accept()
                print(f"与客户端 {self.client_address} 建立连接。")
                
        def wait_for_flag(self):
                """
                等待客户端发送特定标志（这里是"start"）的方法，一旦接收到该标志就退出循环并返回接收到的标志内容
                """
                while True:
                        data = self.client_socket.recv(1024)
                        if data.decode() == "start":
                                break
                print(f"接收到标志：{data.decode()}")
                return data.decode()
                
        def close_connection(self):
                """
                关闭与客户端的连接以及服务器套接字的方法
                """
                self.client_socket.close()
                self.server_socket.close()

