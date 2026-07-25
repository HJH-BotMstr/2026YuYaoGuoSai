import cv2
import numpy as np
from typing import Optional


class BlockDetection:
    """
    机械臂摄像头识别红/绿长条。
    输出色块颜色、包围框、中心偏移（像素）、距离估算（mm）。
    """

    def __init__(self, cfg: dict):
        self._fx = float(cfg["arm_cam_fx"])
        self._real_width_mm = float(cfg["block_real_width_mm"])
        self._min_area = int(cfg.get("block_min_area", 800))

        self._red_lower1 = np.array(cfg["hsv_red_lower1"], dtype=np.uint8)
        self._red_upper1 = np.array(cfg["hsv_red_upper1"], dtype=np.uint8)
        self._red_lower2 = np.array(cfg["hsv_red_lower2"], dtype=np.uint8)
        self._red_upper2 = np.array(cfg["hsv_red_upper2"], dtype=np.uint8)
        self._green_lower = np.array(cfg["hsv_green_lower"], dtype=np.uint8)
        self._green_upper = np.array(cfg["hsv_green_upper"], dtype=np.uint8)

        # 畸变参数，用于 undistort
        dist = cfg.get("arm_cam_dist", [0.0, 0.0, 0.0, 0.0, 0.0])
        self._dist = np.array(dist, dtype=np.float64)
        cx = float(cfg.get("arm_cam_cx", 0.0))
        cy = float(cfg.get("arm_cam_cy", 0.0))
        fy = float(cfg.get("arm_cam_fy", self._fx))
        self._cam_mtx = np.array([[self._fx, 0, cx],
                                   [0, fy, cy],
                                   [0, 0, 1]], dtype=np.float64)

    # ------------------------------------------------------------------ #
    def detect(self, frame) -> Optional[dict]:
        """
        在 frame 中检测最大红色/绿色长条。
        返回 dict 或 None（未检测到）。

        返回字段：
          color            : "red" | "green"
          bbox             : ((x1,y1),(x2,y2))
          center_offset_x  : 像素偏移（正=右）
          distance_mm      : 针孔模型距离（mm）
          pos_3d           : (X, Y, Z) mm，相机坐标系，Z 向前
        """
        undist = cv2.undistort(frame, self._cam_mtx, self._dist)
        hsv = cv2.cvtColor(undist, cv2.COLOR_BGR2HSV)
        h, w = undist.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0

        # 红色：两段 HSV 合并
        mask_r1 = cv2.inRange(hsv, self._red_lower1, self._red_upper1)
        mask_r2 = cv2.inRange(hsv, self._red_lower2, self._red_upper2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        # 绿色
        mask_green = cv2.inRange(hsv, self._green_lower, self._green_upper)

        best = None  # (area, color, x, y, bw, bh)
        for color, mask in (("red", mask_red), ("green", mask_green)):
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._min_area:
                    continue
                x, y, bw, bh = cv2.boundingRect(cnt)
                if best is None or area > best[0]:
                    best = (area, color, x, y, bw, bh)

        if best is None:
            return None

        _, color, x, y, bw, bh = best
        cx_block = x + bw / 2.0
        cy_block = y + bh / 2.0
        offset_x = int(cx_block - cx_img)

        # 针孔模型距离（沿光轴 Z）：Z = fx * real_width / pixel_width
        # 同时用 fy 和高度方向做加权平均，减少单方向误差
        fx = self._cam_mtx[0, 0]
        fy = self._cam_mtx[1, 1]
        Z_from_w = fx * self._real_width_mm / bw if bw > 0 else 0.0
        # 若长条在画面里高度也可信（非遮挡），与宽度估距加权
        real_height_mm = self._real_width_mm * (bh / bw) if bw > 0 else 0.0
        Z_from_h = fy * real_height_mm / bh if (bh > 0 and real_height_mm > 0) else Z_from_w
        # 取宽度方向为主（竖直方向受俯仰角影响更大）
        distance_mm = Z_from_w

        # 反投影：像素坐标 → 相机坐标系 3D 位置
        # X = (u - cx) / fx * Z,  Z = (v - cy) / fy * dist,  Y = distance_mm（光轴向前）
        cam_cx = self._cam_mtx[0, 2]
        cam_cy = self._cam_mtx[1, 2]
        X = (cx_block - cam_cx) / fx * distance_mm
        Z = (cy_block - cam_cy) / fy * distance_mm
        Y = distance_mm

        return {
            "color": color,
            "bbox": ((x, y), (x + bw, y + bh)),
            "center_offset_x": offset_x,
            "distance_mm": round(distance_mm, 1),
            "pos_3d": (round(X, 1), round(Y, 1), round(Z, 1)),
        }

    def detect_all(self, frame) -> list:
        """
        检测所有满足面积阈值的红/绿色块，返回候选列表。
        每项结构与 detect() 返回值相同，列表按 distance_mm 升序排列（近→远）。
        无目标时返回空列表。
        """
        undist = cv2.undistort(frame, self._cam_mtx, self._dist)
        hsv = cv2.cvtColor(undist, cv2.COLOR_BGR2HSV)
        h, w = undist.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0

        mask_r1 = cv2.inRange(hsv, self._red_lower1, self._red_upper1)
        mask_r2 = cv2.inRange(hsv, self._red_lower2, self._red_upper2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_green = cv2.inRange(hsv, self._green_lower, self._green_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        results = []
        fx = self._cam_mtx[0, 0]
        fy = self._cam_mtx[1, 1]
        cam_cx = self._cam_mtx[0, 2]
        cam_cy = self._cam_mtx[1, 2]

        for color, raw_mask in (("red", mask_red), ("green", mask_green)):
            mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask,     cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._min_area:
                    continue
                x, y, bw, bh = cv2.boundingRect(cnt)
                cx_block = x + bw / 2.0
                cy_block = y + bh / 2.0
                offset_x = int(cx_block - cx_img)
                distance_mm = fx * self._real_width_mm / bw if bw > 0 else 0.0
                X = (cx_block - cam_cx) / fx * distance_mm
                Z = (cy_block - cam_cy) / fy * distance_mm
                Y = distance_mm
                results.append({
                    "color": color,
                    "bbox": ((x, y), (x + bw, y + bh)),
                    "center_offset_x": offset_x,
                    "distance_mm": round(distance_mm, 1),
                    "pos_3d": (round(X, 1), round(Y, 1), round(Z, 1)),
                })

        results.sort(key=lambda r: r["distance_mm"])
        return results

    def visualize(self, frame, result: Optional[dict]):
        """在 frame 上绘制检测结果，用于调试。"""
        if result is None:
            return frame
        (x1, y1), (x2, y2) = result["bbox"]
        color_bgr = (0, 0, 255) if result["color"] == "red" else (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color_bgr, 2)
        X, Y, Z = result["pos_3d"]
        line1 = (f"{result['color']}  "
                 f"off={result['center_offset_x']}px  "
                 f"dist={result['distance_mm']:.0f}mm")
        line2 = f"3D X={X:.0f} Y={Y:.0f} Z={Z:.0f} mm"
        cv2.putText(frame, line1, (x1, max(y1 - 20, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1)
        cv2.putText(frame, line2, (x1, max(y1 - 4, 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1)
        return frame
