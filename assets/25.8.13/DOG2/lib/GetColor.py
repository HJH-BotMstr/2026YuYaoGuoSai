import cv2
import numpy as np

# 定义一个名为ColorInfoGetter的类，用于获取图像中主要颜色的信息
class ColorInfoGetter:
	def __init__(self):
		"""
        类的构造函数，用于初始化一些必要的资源，这里主要是打开默认摄像头。
        """
		self.cap = cv2.VideoCapture(0)  # 0 代表默认摄像头

	def get_color_info(self):
		"""
		从摄像头读取一帧图像，将其转换到Lab颜色空间，然后通过设定的颜色范围来检测图像中红、绿、蓝三种颜色的像素数量，
		最后确定像素数量最多的颜色并返回该颜色的名称。
		"""
		ret, frame = self.cap.read()
		if not ret:
			return "无法获取图像"

		cv2.imwrite("roundtest1.jpg", frame)
		lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
		
		# 定义Lab颜色空间中的颜色范围
		# 红色范围 (在Lab空间中，红色通常有较高的a分量)
		# LAB Thresholds: L(30-90) A(130-255) B(140-255)
		lower_red = np.array([30, 130, 140])
		upper_red = np.array([90, 255, 255])
		
		# 绿色范围 (在Lab空间中，绿色通常有较低的a分量)
		lower_green = np.array([40, 0, 135])
		upper_green = np.array([90, 100, 255])
		
		# 蓝色范围 (在Lab空间中，蓝色通常有较低的b分量)
		lower_blue = np.array([0, 120, 0])
		upper_blue = np.array([90, 255, 115])
		
		# 创建颜色掩码
		red_mask = cv2.inRange(lab_frame, lower_red, upper_red)
		green_mask = cv2.inRange(lab_frame, lower_green, upper_green)
		blue_mask = cv2.inRange(lab_frame, lower_blue, upper_blue)
		# cv2.imshow("red_mask", red_mask)
		# cv2.imshow("green_mask", green_mask)
		# cv2.imshow("blue_mask", blue_mask)
  	
		# 计算每种颜色的像素数量
		red_pixels = np.sum(red_mask == 255)
		green_pixels = np.sum(green_mask == 255)
		blue_pixels = np.sum(blue_mask == 255)

		# 确定像素数量最多的颜色
		max_pixels = max(red_pixels, green_pixels, blue_pixels)
		if max_pixels == red_pixels:
			print("red")
			return "red"
		elif max_pixels == green_pixels:
			print("green")
			return "green"
		else:
			print("blue")
			return "blue"

	def release_capture(self):
		"""
        释放摄像头资源，当不再需要从摄像头读取图像时，应该调用此方法来关闭摄像头设备，释放相关资源。
        """
		self.cap.release()
		
	def start_get(self):
		"""
        调用get_color_info方法获取图像中像素数量最多的颜色信息，然后释放摄像头资源，最后返回获取到的颜色信息。
        """
		color = self.get_color_info()
		self.release_capture()
		return color
        
if __name__ == '__main__':
	color_getter = ColorInfoGetter()
	while True:
		color_info = color_getter.get_color_info()
		print(f"当前图像中像素数量最多的颜色是：{color_info}")
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	cv2.destroyAllWindows()

