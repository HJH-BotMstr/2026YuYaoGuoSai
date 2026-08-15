import math
import time
import cv2
import numpy as np
import xgolib
import subprocess
import threading
from lib.ClientSocket import ClientSocket, Enhance

from lib.dog_network.image_remote_process import send_image_with_retry, LABELS_CN


dog = xgolib.XGO("/dev/ttyAMA0")

start_yaw = dog.read_yaw()

def change_yaw(target_angle, start_yaw=start_yaw):
		"""
        用于将设备的偏航角调整到指定的目标角度。

        :param target_yaw: 期望设备达到的目标偏航角。
        """
		kp= 1.2
		times = 0
		while True:
			times += 1
			angle_now = dog.read_yaw() - start_yaw
			print("yaw: ", angle_now)
        
			err = target_angle - angle_now
			speed = kp*((min(err, 150)) if err > 0 else (max(-150, err)))
			print(speed)
			print("speed: ", speed)
			dog.turn(speed)
        
			if (abs(angle_now - target_angle) < 3) or (times > 50):
				dog.turn(0)
				break
			time.sleep(0.1)
		time.sleep(0.5)

#定义颜色的HSV范围
color_ranges = {
    "red": ([110, 83, 0], [179, 255, 255]),
    "blue": ([79, 169, 65], [108, 255, 150]),
    "green": ([61, 192, 75], [88, 255, 154])
}

#初始化颜色检测计数
detection_counts = {
    "red": 0,
    "blue": 0,
    "green": 0
}

#前后移动调整
def adjust_x(vx, runtime):
    dog.move_x(vx)
    time.sleep(runtime)
    dog.move_x(0)

#左右移动调整
def adjust_y(vy, runtime):
    dog.move_y(vy)
    time.sleep(runtime)
    dog.move_y(0)

#机身姿态调整
def ready_for_grasp():
    dog.reset()
    dog.attitude("p", 18)
    time.sleep(0.5)

#调整机身抓取
def grasp():
    dog.claw(0)
    time.sleep(2)
    dog.translation("x", 20)
    dog.motor(52, -55)
    time.sleep(0.5)
    dog.translation("z", 60)
    time.sleep(0.5)
    dog.motor(53, 90)
    dog.attitude("p", 20)
    time.sleep(2)
    dog.claw(255)
    time.sleep(2)
    dog.reset()
    time.sleep(2)
    dog.motor(52, -55)
    time.sleep(2)

#调整机身放置
def place():
    dog.translation("x", 20)
    dog.motor(52, -55)
    time.sleep(0.5)
    dog.translation("z", 60)
    time.sleep(0.5)
    dog.motor(53, 80)
    dog.attitude("p", 20)
    time.sleep(2)
    dog.claw(0)
    time.sleep(2)
    dog.reset()
    time.sleep(10)

# 定义达到的检测次数上限
MAX_COUNT = 2

# 定义全局变量来存储误差和标志
global_error_x = None
global_error_y = None
flag = False  # 在此处初始化 flag 变量


def detect_cuboids(frame):
    global global_error_x, global_error_y, flag, area

    flag = False
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    filtered_frame = None  # 用于保存颜色过滤后的图像
    detected_colors = []  # 用于保存检测到的颜色圆柱体信息
    errors = []  # 用于保存误差信息
    area = 0

    # 获取相机视图的中心点
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2

    for color_name, (lower, upper) in color_ranges.items():
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
            area = w * h
            print(f"area:{area}")

            # 计算长宽比来识别圆柱体（直径大，高度小）
            aspect_ratio = w / float(h)
            print(f"aspect_ratio:{aspect_ratio}")

            if 0 < aspect_ratio < 5:  # 根据你的圆柱体的形状调整这个阈值
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, f"{color_name} cubiod", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255),
                            2)
                detected_colors.append(color_name)

                # 更新检测计数
                detection_counts[color_name] += 1

                # 检查计数是否达到最大值
                if detection_counts[color_name] == MAX_COUNT:
                    # 当计数达到 5 时，清零并执行相应操作
                    detection_counts[color_name] = 0
                    flag = True

                # 计算圆柱体的中心点
                cylinder_center_x = x + w // 2
                cylinder_center_y = y + h // 2

                # 计算圆柱体中心点与相机中心点的误差
                error_x = cylinder_center_x - frame_center_x
                error_y = cylinder_center_y - frame_center_y

                # 更新全局变量
                global_error_x = error_x
                global_error_y = error_y

                errors.append((color_name, error_x, error_y))

    return frame, filtered_frame, detected_colors, errors, flag, area

#opencv检测硬件摄像头
def search():
    global flag1
    cap = cv2.VideoCapture(0)  # 打开摄像头
    cap.set(3, 640)
    cap.set(4, 480)
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    while flag1:
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧")
            break

        frame, filtered_frame, detected_colors, errors, flag, area = detect_cuboids(frame)

        # cv2.imshow("Original Frame", frame)
        # if filtered_frame is not None:
        #     cv2.imshow("Filtered Frame", filtered_frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    cap.release()
    cv2.destroyAllWindows()

#机身位姿态调整
def adjust(m_x, m_y, m_s, des_x=350, des_y=120):
    err_x = -m_x
    err_y = des_y - m_y
    s = m_s
    print(f"左右误差:{err_x}area:{s}")
    dog.attitude("p", 0)
    time.sleep(0.5)
    if abs(err_x) < 30:
        if s > 14000:
            return True
        else:
            dog.translation("x", -10)
            adjust_x(10, 1)  # 10
            dog.translation("x", 0)
    else:
        adjust_y(math.copysign(15, err_x), 0.5)  # 20
    time.sleep(0.3)
    return False

#机器狗移动
def move():
    global global_error_x, global_error_y, flag2, area, flag, flag1

    while flag2:
        if flag:
            print(f"Moving with errors - X: {global_error_x}, Y: {global_error_y}")
            ready_for_grasp()
            res = adjust(global_error_x, global_error_y, area)
            ready_for_grasp()
            if res:
                grasp()
                flag1 = False
                flag2 = False

            global_error_x = None  # 重置全局变量
            global_error_y = None
            flag = False  # 重置 flag 变量

        time.sleep(0.1)  # 添加延迟避免过度占用CPU

#cv图片检测
def detect():
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
        frame, identify="xgo2", timeout=1.0, computer_ip=computer_ip, max_retries=3
    )

    if res is None:
        print("未检测到物体。")
        return
    xyxy = res["xyxy"]
    conf = res["conf"]
    cls_id = res["cls"]
    print(f"检测到{LABELS_CN[cls_id]}，置信度：{conf:.2f}，坐标：{xyxy}")



#if __name__ == '__main__':
#前进 好了
dog.move_x(10)
time.sleep(2.5)
dog.move_x(0)
time.sleep(0.5)
#
detect()
#斜向前进 好了
change_yaw(-60)
dog.move_x(25)
time.sleep(2.3)
dog.move_x(0)
time.sleep(0.5)
#回正
change_yaw(0)
#前进 好了
dog.move_x(25)
time.sleep(1.3)
dog.move_x(0)
time.sleep(0.5)
#
detect()
#斜向前进 好了
change_yaw(40)
dog.move_x(25)
time.sleep(2.7)
dog.move_x(0)
time.sleep(0.5)
#转动回正 好了
change_yaw(90)
dog.move_x(25)
time.sleep(2.5)
dog.move_x(0)
time.sleep(0.5)
change_yaw(0)
time.sleep(0.5)
#
detect()
#斜向前进 好啦
change_yaw(-50)
dog.move_x(25)
time.sleep(4.3)
dog.move_x(0)
time.sleep(0.5)
#回正
change_yaw(0)
#可以不要
# dog.move_x(10)
# time.sleep(5)
# dog.move_x(0)
# time.sleep(0.5)
# change_yaw(0)
print("-------------------")
# global flag1, flag2
# flag1 = True
# flag2 = True
#ready_for_grasp()
# t1 = threading.Thread(target=search)
# t2 = threading.Thread(target=move)
# t1.start()
# t2.start()
# t1.join()
# t2.join()
#
dog.turn(33)
time.sleep(4.5)
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












