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
        从摄像头读取一帧图像，将其转换到HSV颜色空间，然后通过设定的颜色范围来检测图像中红、绿、蓝三种颜色的像素数量，
        最后确定像素数量最多的颜色并返回该颜色的名称。
        """
		ret, frame = self.cap.read()
		if not ret:
			return "无法获取图像"

		cv2.imwrite("roundtest1.jpg", frame)
		hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
		lower_red = np.array([0, 52, 38])
		upper_red = np.array([43, 210, 190])
        
		lower_green = np.array([61, 192, 75])
		upper_green = np.array([88, 255, 154])

		lower_blue = np.array([79, 169, 65])
		upper_blue = np.array([108, 255, 150])
		
		# 创建颜色掩码
		red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
		green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
		blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        
        # 计算每种颜色的像素数量
		red_pixels = np.sum(red_mask == 255)
		green_pixels = np.sum(green_mask == 255)
		blue_pixels = np.sum(blue_mask == 255)

        # 确定像素数量最多的颜色
		max_pixels = max(red_pixels, green_pixels, blue_pixels)
		if max_pixels == red_pixels:
			return "red"
		elif max_pixels == green_pixels:
			return "green"
		else:
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

