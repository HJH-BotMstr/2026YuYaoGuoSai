#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8 双模型仪表盘识别系统
"""

import cv2
import math
import numpy as np
from ultralytics import YOLO


# ==================== 用户配置区 ====================
POINTER_MODEL = r"F:\Creatf\2026YuYaoGuoSai\assets\models\best_ptr.pt"
REGIONS_MODEL = r"F:\Creatf\2026YuYaoGuoSai\assets\models\best_bg.pt"

CAMERA_ID = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 分类阈值（根据你的实测数据标定）
# 实测数据：
#   0.70MPa(绿红边界): rel=41.0°
#   0.50MPa(绿色中心): rel=95.0°
#   0.29MPa(黄绿边界): rel=148.9°
#   0MPa(黄色边界):    rel=-133.6°
THRESH_RED = 45      # rel 0°~45° → 偏高（红色区域）
THRESH_GREEN = 135   # rel 45°~135° → 居中（绿色区域）
                     # rel 其他 → 偏低（黄色区域）

FALLBACK_UP_ANGLE = 218.83

# 显示缩放限制
MAX_DISPLAY_W = 1200
MAX_DISPLAY_H = 800
# ===================================================


class GaugeYOLORecognizer:
    def __init__(self):
        print("加载 pointer 模型...")
        self.model_ptr = YOLO(POINTER_MODEL)
        print("加载 regions 模型...")
        self.model_reg = YOLO(REGIONS_MODEL)
        print("✓ 全部加载完成\n")

    def angle(self, x1, y1, x2, y2):
        """计算两点连线角度，0°指向右，逆时针增加"""
        deg = math.degrees(math.atan2(-(y2 - y1), x2 - x1))
        return deg if deg >= 0 else deg + 360

    def classify(self, rel_angle):
        """
        rel_angle: (ptr - up) % 360 的原始值
        根据实测数据划分：
          0°~45°   → 偏高（红色，1.0MPa~0.7MPa）
          45°~135° → 居中（绿色，0.7MPa~0.3MPa）
          其他     → 偏低（黄色，0.3MPa~0MPa）
        """
        rel = rel_angle % 360
        if rel > 180:
            rel -= 360

        if 0 <= rel <= THRESH_RED:
            return "偏高", (0, 0, 255)
        elif THRESH_RED < rel <= THRESH_GREEN:
            return "居中", (0, 255, 0)
        else:
            return "偏低", (0, 255, 255)

    def resize_display(self, img):
        """缩放图片到适合屏幕"""
        h, w = img.shape[:2]
        scale = min(MAX_DISPLAY_W / w, MAX_DISPLAY_H / h, 1.0)
        if scale < 1.0:
            return cv2.resize(img, (int(w * scale), int(h * scale)))
        return img

    def process(self, frame):
        h, w = frame.shape[:2]

        # ---------- 1. regions 模型 ----------
        res_reg = self.model_reg(frame, verbose=False)[0]
        boxes = res_reg.boxes

        gauge_box = red_box = None
        for box in boxes:
            cls = int(box.cls[0])
            xyxy = box.xyxy[0].cpu().numpy()
            if cls == 0:
                gauge_box = xyxy
            elif cls == 1:
                red_box = xyxy

        if gauge_box is None:
            return None, None, frame

        # 圆心
        cx = (gauge_box[0] + gauge_box[2]) / 2
        cy = (gauge_box[1] + gauge_box[3]) / 2

        # 上方向 = red 中心
        if red_box is not None:
            rx = (red_box[0] + red_box[2]) / 2
            ry = (red_box[1] + red_box[3]) / 2
            up_angle = self.angle(cx, cy, rx, ry)
        else:
            up_angle = FALLBACK_UP_ANGLE

        # ---------- 2. pointer 模型 ----------
        res_ptr = self.model_ptr(frame, verbose=False)[0]
        kpts = res_ptr.keypoints

        if kpts is None or len(kpts.xy) == 0:
            return None, None, frame

        pts = kpts.xy[0].cpu().numpy()
        if len(pts) < 2:
            return None, None, frame

        rivet = (int(pts[0][0]), int(pts[0][1]))
        tip = (int(pts[1][0]), int(pts[1][1]))
        ptr_angle = self.angle(rivet[0], rivet[1], tip[0], tip[1])

        # ---------- 3. 计算并分类 ----------
        rel_raw = (ptr_angle - up_angle) % 360
        status, color = self.classify(rel_raw)

        rel_signed = rel_raw if rel_raw <= 180 else rel_raw - 360
        print(f"  DEBUG: ptr={ptr_angle:.1f} up={up_angle:.1f} "
              f"rel_raw={rel_raw:.1f} rel_signed={rel_signed:.1f} → {status}")

        # ---------- 4. 可视化 ----------
        vis = frame.copy()
        g = list(map(int, gauge_box))

        # 1. 蓝色矩形框 = 仪表盘外框 (gauge)
        #    作用：确认模型找到了表盘，框中心就是圆心
        cv2.rectangle(vis, (g[0], g[1]), (g[2], g[3]), (255, 0, 0), 2)

        # 2. 红色矩形框 = 红色警戒区域 (red)
        #    作用：确认模型找到了红色区域，框中心 = "上方向"参考点
        if red_box is not None:
            r = list(map(int, red_box))
            cv2.rectangle(vis, (r[0], r[1]), (r[2], r[3]), (0, 0, 255), 2)

        # 3. 绿色圆点 = 指针铆钉中心 (rivet)
        #    作用：指针旋转的圆心，关键点模型直接输出
        cv2.circle(vis, rivet, 10, (0, 255, 0), -1)

        # 4. 蓝色圆点 = 指针针尖 (tip)
        #    作用：指针指向的终点，和 rivet 连线就是指针方向
        cv2.circle(vis, tip, 10, (255, 0, 0), -1)

        # 5. 红色粗线 = 指针本体（rivet 连到 tip）
        cv2.line(vis, rivet, tip, (0, 0, 255), 3)

        # 6. 蓝色箭头 = 上方向（从圆心指向红色区域中心）
        #    含义：这是"高压参考方向"，1.0MPa 应该指向这里
        up_rad = math.radians(up_angle)
        up_len = math.hypot(g[2]-g[0], g[3]-g[1]) * 0.35
        cv2.arrowedLine(vis, (int(cx), int(cy)),
                        (int(cx + up_len*math.cos(up_rad)), int(cy + up_len*math.sin(up_rad))),
                        (255, 0, 0), 2, tipLength=0.1)

        # 7. 红色箭头 = 指针当前方向（从圆心指向针尖）
        #    含义：指针实际指向哪里
        ptr_rad = math.radians(ptr_angle)
        ptr_len = math.hypot(g[2]-g[0], g[3]-g[1]) * 0.4
        cv2.arrowedLine(vis, (int(cx), int(cy)),
                        (int(cx + ptr_len*math.cos(ptr_rad)), int(cy + ptr_len*math.sin(ptr_rad))),
                        (0, 0, 255), 3, tipLength=0.1)

        # 8. 顶部信息条
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
        vis = cv2.addWeighted(overlay, 0.6, vis, 0.4, 0)

        cv2.putText(vis, f"Ptr:{ptr_angle:.1f} Up:{up_angle:.1f} Rel:{rel_signed:.1f}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, f"Status: {status}", (15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        return status, rel_signed, vis

    def run_image(self, path):
        img = cv2.imread(path)
        if img is None:
            print(f"❌ 无法读取: {path}")
            return

        status, angle, vis = self.process(img)

        if status is None:
            print("⚠️ 未检测到表盘或指针")
            cv2.imshow("Result", self.resize_display(vis))
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            return

        print(f"\n结果: {status}  (相对角度: {angle:.2f}°)")

        cv2.imshow("Result", self.resize_display(vis))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def run_camera(self):
        cap = cv2.VideoCapture(CAMERA_ID)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            print("❌ 摄像头打开失败")
            return

        print("实时识别中... 按 Q 退出")
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            status, angle, vis = self.process(frame)

            if status is None:
                cv2.putText(vis, "NO GAUGE/POINTER", (20, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow("YOLO Gauge", self.resize_display(vis))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    recognizer = GaugeYOLORecognizer()
    recognizer.run_image(r"F:\Creatf\2026YuYaoGuoSai\assets\images\3.png")
    # recognizer.run_camera()