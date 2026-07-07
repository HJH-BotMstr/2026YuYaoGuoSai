#!/usr/bin/env python3
"""
验证逻辑：计算霍夫圆检测的损失和通过与否。

公式（严格遵循 AGENTS.md）：

  真实值: (x, y, r)    ← 来自 calc_params（参数变换后的几何真值）
  预测值: (x', y', r')  ← 霍夫圆+筛选算法

  r1 = r      (真值半径)
  r2 = r'     (预测半径)
  d  = sqrt((x-x')² + (y-y')²)

  分母 denom = (r1 + r2) / 2

  损失函数 L = (0.5·d² + 0.5·(r1-r2)²) / denom

  测试集 MSE = sum(L²) / n

  通过标准（以 denom 为分母计算相对误差）：
    圆心距相对误差 = d / denom  < 15%
    半径相对误差   = |r1-r2| / denom  < 15%
"""

import json
import math
from pathlib import Path

BASE_DIR = Path(__file__).parent


def evaluate_circle(gt_center, gt_radius, pred_center, pred_radius):
    """
    评估单个霍夫圆检测结果。

    Args:
        gt_center:   [x, y]  真实圆心
        gt_radius:   float   真实外半径
        pred_center: [x, y]  预测圆心
        pred_radius: float   预测半径

    Returns:
        dict with keys:
            loss           — 单张损失 L
            center_rel_err — 圆心距相对误差（百分比）
            radius_rel_err — 半径相对误差（百分比）
            passed         — 是否通过（两项均 <15%）
    """
    x_gt, y_gt = gt_center
    x_pred, y_pred = pred_center
    r1 = float(gt_radius)
    r2 = float(pred_radius)

    d = math.hypot(x_pred - x_gt, y_pred - y_gt)

    denom = (r1 + r2) / 2.0

    # 损失
    loss = (0.5 * d * d + 0.5 * (r1 - r2) * (r1 - r2)) / denom

    # 相对误差（以 denom 为分母）
    center_rel_err = (d / denom) * 100.0
    radius_rel_err = (abs(r1 - r2) / denom) * 100.0

    passed = center_rel_err < 15.0 and radius_rel_err < 15.0

    return {
        "loss": loss,
        "center_rel_err": round(center_rel_err, 4),
        "radius_rel_err": round(radius_rel_err, 4),
        "passed": passed,
    }


def evaluate_pointer(pred_angle, gt_angle):
    """
    评估指针角度检测结果。

    Args:
        pred_angle: float  预测角度 (0-360)
        gt_angle:   float  真实角度 (0-360)

    Returns:
        dict with keys:
            angle_err — 最小角度差（度）
            passed    — 是否通过（<10°）
    """
    diff = abs(pred_angle - gt_angle) % 360.0
    angle_err = min(diff, 360.0 - diff)
    return {
        "angle_err": round(angle_err, 4),
        "passed": angle_err < 10.0,
    }


def evaluate_all(predictions, ground_truth):
    """
    在整个测试集上评估。

    Args:
        predictions:  dict {image_name: {"center": [x,y], "radius": r, "pointer_angle": a}}
        ground_truth: dict {image_name: {"center": [x,y], "radius_outer": r, "pointer_angle": a}}

    Returns:
        dict with summary and per-image results
    """
    results = []
    circle_passed = 0
    pointer_passed = 0
    both_passed = 0
    total = 0
    circle_losses = []
    pointer_errors = []

    for image_name, gt in sorted(ground_truth.items()):
        pred = predictions.get(image_name)
        if pred is None:
            results.append({"image": image_name, "status": "no_prediction"})
            continue

        total += 1
        entry = {"image": image_name}

        if "center" in pred and "radius" in pred:
            circle = evaluate_circle(
                gt["center"], gt["radius_outer"],
                pred["center"], pred["radius"],
            )
            entry.update({
                "circle_loss": circle["loss"],
                "center_rel_err_pct": circle["center_rel_err"],
                "radius_rel_err_pct": circle["radius_rel_err"],
                "circle_pass": circle["passed"],
            })
            if circle["passed"]:
                circle_passed += 1
            circle_losses.append(circle["loss"])
        else:
            entry["circle_pass"] = False

        if "pointer_angle" in pred and "pointer_angle" in gt:
            ptr = evaluate_pointer(pred["pointer_angle"], gt["pointer_angle"])
            entry.update({
                "pointer_err_deg": ptr["angle_err"],
                "pointer_pass": ptr["passed"],
            })
            if ptr["passed"]:
                pointer_passed += 1
            pointer_errors.append(ptr["angle_err"])

        c_ok = entry.get("circle_pass", False)
        p_ok = entry.get("pointer_pass", False)
        entry["passed"] = c_ok and p_ok
        if entry["passed"]:
            both_passed += 1

        results.append(entry)

    summary = {
        "total": total,
        "circle_passed": circle_passed,
        "circle_pass_rate": round(circle_passed / total * 100, 2) if total else 0,
        "pointer_passed": pointer_passed,
        "pointer_pass_rate": round(pointer_passed / total * 100, 2) if total else 0,
        "both_passed": both_passed,
        "pass_rate": round(both_passed / total * 100, 2) if total else 0,
    }
    if circle_losses:
        summary["circle_mse"] = round(
            sum(l * l for l in circle_losses) / len(circle_losses), 6
        )
    if pointer_errors:
        summary["pointer_mse"] = round(
            sum(e * e for e in pointer_errors) / len(pointer_errors), 6
        )

    return {"summary": summary, "results": results}


def load_ground_truth(gt_path):
    with open(gt_path, "r") as f:
        gt_list = json.load(f)
    return {item["image"]: item for item in gt_list}
