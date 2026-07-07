#!/usr/bin/env python3
"""
Step 1: 霍夫圆检测 — 检测仪表盘外轮廓（圆心 + 外半径）。

算法（v3）：
  1. 预处理：CLAHE + 双边滤波
  2. 多策略霍夫圆粗筛 → 估计大致半径和位置
  3. 全角度径向扫描 → 精确找外边界点（暗环→亮背景过渡）
  4. RANSAC + 最小二乘圆拟合 → 高精度中心+半径

输入：图像路径 或 BGR numpy array
输出：{"center": [x, y], "radius": r}  或 raise RuntimeError
"""

import math
import cv2
import numpy as np


def detect_circle(image):
    """检测仪表盘外轮廓圆。"""
    if isinstance(image, str):
        img = cv2.imread(image)
        if img is None:
            raise RuntimeError(f"无法读取图片: {image}")
    else:
        img = image

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # ── 1. 预处理 ──────────────────────────────────────────────
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    smooth = cv2.bilateralFilter(enhanced, 9, 75, 75)

    # ── 2. 粗筛：霍夫圆多策略 ──────────────────────────────────
    cx0, cy0, r0 = _coarse_hough(smooth, w, h)
    if cx0 is None:
        raise RuntimeError("霍夫圆未检测到任何候选圆")

    # ── 3. 全角度径向扫描找外边界 ──────────────────────────────
    boundary_pts = _scan_boundary(enhanced, cx0, cy0, r0, w, h)

    # ── 4. RANSAC + LSQ 圆拟合 ─────────────────────────────────
    result = _fit_circle_robust(boundary_pts)
    cx, cy, r = result

    if r <= 0 or cx < 0 or cy < 0 or cx >= w or cy >= h:
        raise RuntimeError(f"拟合结果越界: ({cx:.1f}, {cy:.1f}) r={r:.1f}")

    return {"center": [round(cx, 2), round(cy, 2)], "radius": round(r, 2)}


def _coarse_hough(smooth, w, h):
    """
    多策略霍夫圆 → 投票确定大致圆心和半径。
    返回 (cx, cy, r) 或 (None, None, None)。
    """
    edges = cv2.Canny(smooth, 40, 120, apertureSize=3)

    strategies = [
        (80, 30, 60, 220, True),
        (60, 25, 55, 230, False),
        (100, 35, 60, 200, True),
        (50, 20, 50, 250, False),
        (70, 28, 55, 230, False),
        (90, 32, 60, 210, True),
    ]

    all_circles = []
    for p1, p2, min_r, max_r, use_edges in strategies:
        src = edges if use_edges else smooth
        circles = cv2.HoughCircles(
            src, cv2.HOUGH_GRADIENT, dp=1,
            minDist=max(w, h), param1=p1, param2=p2,
            minRadius=min_r, maxRadius=max_r,
        )
        if circles is not None:
            for c in circles[0]:
                all_circles.append((float(c[0]), float(c[1]), float(c[2])))

    if not all_circles:
        return None, None, None

    # 中心聚类
    clusters = _cluster_points(
        [(c[0], c[1]) for c in all_circles], threshold=30.0
    )
    if not clusters:
        return None, None, None

    best_cluster = max(clusters, key=len)
    cx = float(np.median([p[0] for p in best_cluster]))
    cy = float(np.median([p[1] for p in best_cluster]))

    # 该簇的半径中位数
    cluster_radii = [
        c[2] for c in all_circles
        if math.hypot(c[0] - cx, c[1] - cy) < 30.0
    ]
    if not cluster_radii:
        cluster_radii = [c[2] for c in all_circles]
    r = float(np.median(cluster_radii))

    return cx, cy, r


def _cluster_points(points, threshold=30.0):
    """简单距离聚类。"""
    if not points:
        return []
    clusters = []
    used = [False] * len(points)
    for i, p in enumerate(points):
        if used[i]:
            continue
        cluster = [p]
        used[i] = True
        for j, q in enumerate(points):
            if used[j]:
                continue
            if math.hypot(p[0] - q[0], p[1] - q[1]) < threshold:
                cluster.append(q)
                used[j] = True
        clusters.append(cluster)
    return clusters


def _scan_boundary(gray, cx0, cy0, r0, w, h):
    """
    全角度径向扫描：找出暗环→亮背景过渡的最外侧位置。

    返回边界点列表 [(x, y), ...]。
    """
    n_rays = 180  # 每2°一条射线
    boundary_pts = []

    for i in range(n_rays):
        angle = 2.0 * math.pi * i / n_rays
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        # 采样范围：0.65 * r0 到 1.30 * r0
        profile = []
        for frac in np.linspace(0.65, 1.30, 100):
            rr = r0 * frac
            x = int(cx0 + rr * cos_a)
            y = int(cy0 + rr * sin_a)
            if 0 <= x < w and 0 <= y < h:
                profile.append((rr, float(gray[y, x])))

        if len(profile) < 20:
            continue

        # 找全局最小值位置（暗环最暗处）
        vals = [v for _, v in profile]
        min_idx = int(np.argmin(vals))
        min_val = vals[min_idx]

        # 从最暗处向外找：从 dark→light 的过渡
        # 条件：值恢复到 min_val + 30 以上 且 在 r0*0.85 之外
        outer_pt = None
        for j in range(min_idx + 1, len(profile)):
            rr_j, val_j = profile[j]
            if rr_j < r0 * 0.75:
                continue  # 还在内圈，跳过
            if val_j > min_val + 30:
                outer_pt = (cx0 + rr_j * cos_a, cy0 + rr_j * sin_a)
                break

        if outer_pt is not None:
            x_pt, y_pt = outer_pt
            if 0 <= x_pt < w and 0 <= y_pt < h:
                boundary_pts.append((x_pt, y_pt))

    return boundary_pts


def _fit_circle_robust(points):
    """
    RANSAC + 最小二乘圆拟合。
    返回 (cx, cy, r) 或 None。
    """
    pts = np.array(points, dtype=np.float64)
    n = len(pts)
    if n < 10:
        return None

    # 先用 RANSAC 去除离群点
    best_inliers = 0
    best_params = None
    n_iter = min(500, n * 10)
    inlier_thresh = 3.0

    for _ in range(n_iter):
        idx = np.random.choice(n, 3, replace=False)
        params = _circle_from_3pts(pts[idx])
        if params is None:
            continue

        cx_f, cy_f, r_f = params
        if r_f <= 20 or r_f > 400:
            continue

        dists = np.abs(
            np.sqrt((pts[:, 0] - cx_f) ** 2 + (pts[:, 1] - cy_f) ** 2) - r_f
        )
        inliers = np.sum(dists < inlier_thresh)
        if inliers > best_inliers:
            best_inliers = inliers
            best_params = params

    if best_params is None or best_inliers < 10:
        return None

    # 用 inlier 做最小二乘精炼
    cx_f, cy_f, r_f = best_params
    dists = np.abs(
        np.sqrt((pts[:, 0] - cx_f) ** 2 + (pts[:, 1] - cy_f) ** 2) - r_f
    )
    inlier_mask = dists < inlier_thresh * 1.5  # 放宽一点
    inlier_pts = pts[inlier_mask]

    if len(inlier_pts) < 6:
        return best_params

    refined = _lsq_circle(inlier_pts)
    return refined if refined is not None else best_params


def _circle_from_3pts(pts):
    """三点确定圆。"""
    (x1, y1), (x2, y2), (x3, y3) = pts
    d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-10:
        return None
    ux = ((x1 * x1 + y1 * y1) * (y2 - y3) +
          (x2 * x2 + y2 * y2) * (y3 - y1) +
          (x3 * x3 + y3 * y3) * (y1 - y2)) / d
    uy = ((x1 * x1 + y1 * y1) * (x3 - x2) +
          (x2 * x2 + y2 * y2) * (x1 - x3) +
          (x3 * x3 + y3 * y3) * (x2 - x1)) / d
    r = math.hypot(ux - x1, uy - y1)
    return (ux, uy, r)


def _lsq_circle(pts):
    """最小二乘圆拟合（代数法）。"""
    n = len(pts)
    if n < 3:
        return None
    x = pts[:, 0]
    y = pts[:, 1]
    A = np.column_stack([x, y, np.ones(n)])
    b = -(x * x + y * y)
    try:
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a, b_coef, c_coef = sol
    cx = -a / 2.0
    cy = -b_coef / 2.0
    r_sq = cx * cx + cy * cy - c_coef
    if r_sq <= 0:
        return None
    return (cx, cy, math.sqrt(r_sq))
