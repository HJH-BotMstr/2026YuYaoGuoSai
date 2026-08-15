import cv2
import numpy as np

# 全局变量用于存储四个点的坐标
points = []
img_copy = None

def mouse_callback(event, x, y, flags, param):
	global points, img_copy
	if event == cv2.EVENT_LBUTTONDOWN:
		points.append((x, y))
		cv2.circle(img_copy, (x, y), 5, (0, 255, 0), -1)
		cv2.imshow('image', img_copy)
		print(f"已标记点：{(x, y)}。")

cap = cv2.VideoCapture(0)
while True:
	ret, img = cap.read()
	if not ret:
		break
	img_copy = img.copy()
	cv2.imshow('image', img_copy)
	cv2.setMouseCallback('image', mouse_callback)
	key = cv2.waitKey(1)
	if key == ord('k') and len(points) == 4:
		print("四个点已确定，准备进行透视变换。")
		break
	elif key == 27:
		print("按下 Esc 键，退出程序。")
		exit(0)

# 目标正方形的四个顶点坐标
dst_points = np.array([[0, 0], [500, 0], [500, 500], [0, 500]], dtype=np.float32)
src_points = np.array(points, dtype=np.float32)

# 计算透视变换矩阵
M = cv2.getPerspectiveTransform(src_points, dst_points)

# 对图像进行透视变换
warped_img = cv2.warpPerspective(img, M, (500, 500))

cv2.imshow('warped image', warped_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
cap.release()
