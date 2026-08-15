import math
import time
import cv2
import socket
import numpy as np
import xgolib
import threading
from lib.ClientSocket import ClientSocket, Enhance

dog = xgolib.XGO("/dev/ttyAMA0")

# 定义HSV范围

min_red1 = [0, 50, 50]
max_red1 = [10, 255, 255]
min_red2 = [107, 57, 149]
max_red2 = [180, 255, 255]

min_blue = [79, 169, 65]
max_blue = [108, 255, 150]

min_green = [61, 192, 75]
max_green = [88, 255, 154]

# 颜色范围列表
color_ranges = [
    [min_red1, max_red1],
    [min_red2, max_red2],
    [min_blue, max_blue],
    [min_green, max_green]
]


def adjust_x(vx, runtime):
    dog.move_x(vx)
    time.sleep(runtime)
    dog.move_x(0)


def adjust_y(vy, runtime):
    dog.move_y(vy)
    time.sleep(runtime)
    dog.move_y(0)


def adjust_yaw(vyaw, runtime):
    dog.turn(vyaw)
    time.sleep(runtime)
    dog.turn(0)


def filter_img(frame, color_ranges):  # 颜色过滤
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = None
    for color in color_ranges:
        color_lower = np.array(color[0])
        color_upper = np.array(color[1])
        if mask is None:
            mask = cv2.inRange(hsv, color_lower, color_upper)
        else:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, color_lower, color_upper))

    # 形态学操作：腐蚀和膨胀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  #调整核大小
    mask = cv2.erode(mask, kernel, iterations=1)  #腐蚀，去掉噪点
    mask = cv2.dilate(mask, kernel, iterations=1)  #膨胀，恢复目标物体

    img_mask = cv2.bitwise_and(frame, frame, mask=mask)
    return img_mask


def detect_contours(frame):
    # 将图像转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 使用Canny边缘检测
    edges = cv2.Canny(gray, 40, 150)
    # 查找轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, edges


def detect_block(contours, frame):  # 绘制外接矩形
    flag = False
    length, width, angle, s_x, s_y = 0, 0, 0, 0, 0
    for i in range(0, len(contours)):
        if cv2.contourArea(contours[i]) < 1000:  #
            continue
        rect = cv2.minAreaRect(contours[i])
        # if 0.6 < rect[1][0] / rect[1][1] < 1:  # 利用长宽比过滤矩形
        # continue
        if not flag:
            if rect[2] > 45:
                length = rect[1][0]
                width = rect[1][1]
                angle = rect[2]
            else:
                length = rect[1][1]
                width = rect[1][0]
                angle = rect[2]
            s_x = rect[0][1]  # s_代表屏幕坐标系
            s_y = rect[0][0]
            flag = True
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # 绘制最小外接矩形
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 5)
        else:  # 识别出两个及以上的矩形退出
            flag = False
            break
    return flag, length, width, angle, s_x, s_y, frame


def ready_for_grasp():
    dog.reset()
    dog.attitude("p", 18)
    time.sleep(0.5)


def grasp():
    dog.claw(0)
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


def adjust(m_angle, m_x, m_y, des_x=360, des_y=320):
    err_x = des_x - m_x
    err_y = des_y - m_y
    print(f"左右误差:{err_y}前后误差:{err_x}")
    if abs(err_y) < 30:  # 20
        if abs(err_x) < 50:
            return True
        else:
            dog.translation("x", -10)
            adjust_x(10, 1.2)
            dog.translation("x", 0)
    else:
        adjust_y(math.copysign(20, err_y), 0.5)
    time.sleep(0.3)
    return False


# 创建锁
lock = threading.Lock()


def search():
    global count, m_x, m_y, m_angle, flag1
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    while flag1:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break
        # 过滤红色
        filtered_img = filter_img(frame, [min_blue, max_blue])
        # 检测轮廓
        contours, edges = detect_contours(filtered_img)
        flag, length, width, angle, s_x, s_y, frame_with_block = detect_block(contours, filtered_img)
        if flag:
            with lock:
                count += 1
                m_angle = (count - 1) / count * m_angle + angle / count
                m_x = (count - 1) / count * m_x + s_x / count
                m_y = (count - 1) / count * m_y + s_y / count
            print(f"search中的count:{count}")
            # print(f"m_x:{m_x}m_y:{m_y}")
        if count == COUNT_MAX:
            with lock:
                count = 0

        cv2.imshow("Filtered Image", filtered_img)
        cv2.imshow("Edges", edges)
        cv2.imshow("Contours", frame_with_block)

        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


def search_all():
    global count, m_x, m_y, m_angle, flag1
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    while flag1:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break
        # 过滤红色、蓝色和绿色
        filtered_img = filter_img(frame, color_ranges)
        # 检测轮廓
        contours, edges = detect_contours(filtered_img)
        flag, length, width, angle, s_x, s_y, frame_with_block = detect_block(contours, filtered_img)
        if flag:
            with lock:
                count += 1
                m_angle = (count - 1) / count * m_angle + angle / count
                m_x = (count - 1) / count * m_x + s_x / count
                m_y = (count - 1) / count * m_y + s_y / count
            print(f"search中的count:{count}")
            # print(f"m_x:{m_x}m_y:{m_y}")
        if count == COUNT_MAX:
            with lock:
                count = 0

        cv2.imshow("Filtered Image", filtered_img)
        cv2.imshow("Edges", edges)
        cv2.imshow("Contours", frame_with_block)

        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


def move():
    global count, m_x, m_y, m_angle, flag1, flag2
    while flag2:
        with lock:
            current_count = count  # 获取当前的 count
        # print(f"move中的count：{current_count}")
        if current_count == COUNT_MAX:
            with lock:
                current_m_x = m_x
                current_m_y = m_y
                current_m_angle = m_angle
                count = 0  # 重置 count
            res = adjust(current_m_angle, current_m_x, current_m_y)
            if res:
                grasp()
                flag1 = False
                print(f"flag1:{flag1}")
                flag2 = False


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


def detect():
    client = ClientSocket("192.168.31.226", 8888)
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


m_angle, m_x, m_y = 0, 0, 0
count = 0
COUNT_MAX = 3


def calculate_center_of_contour(contour):
    """计算轮廓的中心点"""
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        return (cX, cY)
    return None


def show_error_in_center(frame):
    # 获取图像的宽高
    global p_error_x, p_error_y, flag4, area
    height, width, _ = frame.shape
    center_x = width // 2
    center_y = height // 2
    camera_center = (center_x, center_y)

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 设置blue的HSV范围
    lower_blue = np.array([90, 202, 229])  # 下限
    upper_blue = np.array([180, 255, 255])  # 上限

    # 阈值化提取绿色区域
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 寻找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历轮廓，找出最大的矩形
    for contour in contours:
        # 获取矩形的边界框
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h
        # print(f"area:{w * h}")

        # 过滤掉太小的矩形
        if w * h > 200:  # 设定最小面积过滤
            # 计算矩形的中心点
            rectangle_center = calculate_center_of_contour(contour)
            if rectangle_center != None:
                flag4 = True
            else:
                flag4 = False
            # 计算误差
            if rectangle_center:
                p_error_x = camera_center[0] - rectangle_center[0]
                p_error_y = camera_center[1] - rectangle_center[1]

                # 绘制矩形和中心点
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, rectangle_center, 5, (0, 0, 255), -1)  # 绘制矩形中心点
                cv2.circle(frame, camera_center, 5, (255, 0, 0), -1)  # 绘制相机中心点

    # 显示结果
    cv2.imshow("Frame", frame)


def picture():
    # 打开相机
    cap = cv2.VideoCapture(0)
    global flag3
    while flag3:
        ret, frame = cap.read()
        if not ret:
            break

        # 处理帧，识别绿色矩形并计算误差
        show_error_in_center(frame)

        # 按下 'Esc' 键退出
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def picture_adjust(p_error_x, p_error_y, area):
    px = p_error_x
    py = p_error_y
    s = area
    print(px, py, s)
    # print(f"px:{px}py:{py}")
    if abs(px) < 40:  # 50
        if s > 1000:
            print("match")
            return True
        else:
            adjust_x(10, 1.2)
        time.sleep(0.5)
    else:
        adjust_y(math.copysign(8, p_error_x), 1)
    time.sleep(0.5)
    return False


def picture_move():
    global flag4, p_error_x, p_error_y, flag3, area
    p_error_x = 0
    p_error_y = 0
    area = 0
    while flag4:
        # print(f"px:{p_error_x}py:{p_error_y}")
        res = picture_adjust(p_error_x, p_error_y, area)
        if res:
            place()
            flag3 = False
            flag4 = False


def change_yaw(target_yaw):
    target_err = 2

    while True:
        yaw = dog.read_yaw()
        print(yaw)
        if (target_yaw - target_err) < yaw < (target_yaw + target_err):
            break
        else:
            if yaw < target_yaw:
                dog.turn(10)
                time.sleep(0.8)
                dog.turn(0)
                time.sleep(0.5)
            else:
                dog.turn(-10)
                time.sleep(0.8)
                dog.turn(0)
                time.sleep(0.5)


def send_dog2_msgs(data):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("192.168.2.199", 8888))

    client.send(data.encode())
    print("sent")

    client.close()


if __name__ == '__main__':
    global flag3, flag4

    flag3 = True
    flag4 = True
    # grasp()

    dog.motor(52, -55)
    time.sleep(2)

    # change_yaw(144)
    t3 = threading.Thread(target=picture)
    t4 = threading.Thread(target=picture_move)
    t3.start()
    t4.start()
    t3.join()
    t4.join()
    send_dog2_msgs("start")

    # 前往抓取长条
    dog.turn(-33)
    time.sleep(4.5)
    dog.turn(0)
    time.sleep(0.5)
    dog.move_x(10)
    time.sleep(5)
    dog.move_x(0)
    time.sleep(0.5)










