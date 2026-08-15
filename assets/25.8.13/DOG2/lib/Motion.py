#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :2024/10/5
# @Author  :WuJunYi
# @File    :Motion.py
# @Software:PyCharm
import time
import xgolib

# 定义Motion类，用于控制机器的运动相关操作
class Motion:
	def __init__(self):
		"""
        类的构造函数，用于初始化Motion类的实例。

        在这里创建了一个与机器设备通信的对象，并获取机器初始的偏航角（yaw）值，
        以便后续在运动控制中作为参考基准。
        """
		self.move = xgolib.XGO("/dev/ttyAMA0")
		self.start_yaw = self.move.read_yaw()
	
	def go_a_block(self,vx=25,runtime=2.3):
		"""前进一个区块"""
		dog = self.move
		dog.move_x(vx)
		time.sleep(runtime)
		dog.move_x(0)

	def go_left(self,vy=18,runtime=2.45):
		"""左一个区块"""
		dog = self.move
		dog.move_x(2)
		dog.move_y(vy)
		time.sleep(runtime)
		dog.stop()

	def go_right(self,vy=18,runtime=2.45):
		"""右一个区块"""
		dog = self.move
		dog.move_x(4.5)
		dog.move_y(-vy)
		time.sleep(runtime)
		dog.stop()
 
	def change_yaw(self, target_angle):
		"""
        用于将设备的偏航角调整到指定的目标角度。

        :param target_yaw: 期望设备达到的目标偏航角。
        """
		kp= 1.2
		times = 0
		while True:
			times += 1
			angle_now = self.move.read_yaw() - self.start_yaw
        
			err = target_angle - angle_now
			speed = kp*((min(err, 150)) if err > 0 else (max(-150, err)))
			self.move.turn(speed)
        
			if (abs(angle_now - target_angle) < 3) or (times > 50):
				self.move.turn(0)
				break
			time.sleep(0.1)
		time.sleep(0.5)
		
	def to_first_image(self):
		"""
        控制设备移动到第一个目标位置的操作。

        先使设备在x轴方向上以一定速度移动一段时间，然后停止移动并暂停一段时间。
        """
		self.move.move_x(10)
		time.sleep(2.5)
		self.move.move_x(0)
		time.sleep(0.5)
	
	def to_second_image(self):
		"""
        控制设备移动到第二个目标位置的操作。

        先使设备绕yaw轴旋转一定角度，然后在x轴方向上以一定速度移动一段时间，最后停止移动并暂停一段时间。
        """
		self.move.turn(25)
		time.sleep(2.5)
		self.move.turn(0)
		time.sleep(0.5)
		
		self.move.move_x(19)
		time.sleep(5.0)
		self.move.move_x(0)
		time.sleep(0.5)
	# def to_second_image(self):
	# 	"""
	# 	控制设备移动到第二个目标位置的操作。
	# 	先使设备绕yaw轴旋转一定角度，然后在x轴方向上以一定速度移动一段时间，最后停止移动并暂停一段时间。
	# 	"""
	# 	self.go_left()
	# 	time.sleep(0.5)
	# 	self.go_left()
	# 	time.sleep(0.5)
	# 	self.go_a_block()
	# 	time.sleep(0.5)
		
	def to_connect1(self):
		"""
        控制设备移动到连接点1的操作。

        涉及到设备在x轴方向和yaw轴方向上的多次移动和旋转操作，以逐步到达目标位置并调整到特定的偏航角状态。
        """
		self.change_yaw(-45)
		time.sleep(0.5)

		self.move.move_x(18)
		time.sleep(5.8)
		self.move.move_x(0)
		time.sleep(0.5)
		'''
		self.move.turn(22)
		time.sleep(3)
		self.move.turn(0)
		time.sleep(0.5)'''
		
		self.change_yaw(0)

		self.move.move_x(19)
		time.sleep(4.0)
		self.move.move_x(0)
		time.sleep(0.5)
		'''
		self.move.turn(32)
		time.sleep(3)
		self.move.turn(0)
		time.sleep(0.5)'''
		
		self.change_yaw(90)
		
		self.move.move_x(20)
		time.sleep(8.0)
		self.move.move_x(0)
		time.sleep(0.5)
		


		
		
		
	
