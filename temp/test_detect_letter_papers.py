#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detect_letter_papers 合成图自测（一次性验证脚本，不入包）。

验证点：
  1. 棕色背景上的白色 A4 比例四边形能被检出，几何（u/v/h_px/aspect）正确；
  2. 正方形亮块（模拟纸箱面/标签）被长宽比过滤；
  3. 纸内大字母能被 OCR 认出（合成字体与印刷体有差异，失败仅告警不算失败）；
  4. ocr=False 时几何照常输出、char 为 None（身份/几何解耦）；
  5. tz = fy * H_paper / h_px 反推误差 < 5%。
"""

import sys
import numpy as np
import cv2

sys.path.insert(0, '/home/ysc/2026YuYaoGuoSai/tools')
from gauge_yolo_new import detect_letter_papers

W, H = 640, 480
FY = 387.7497          # 与配置内参一致
H_PAPER = 0.297        # portrait
PASS, FAIL = 'PASS', 'FAIL'
failures = []


def make_frame(letter='B', paper_w=90, paper_h=127, cx=320, cy=240,
               add_square=True):
    """棕色背景 + 白色 A4 比例矩形（长宽比 127/90 ≈ 1.411）+ 黑字母。"""
    frame = np.full((H, W, 3), (30, 70, 140), dtype=np.uint8)  # 牛皮纸色
    x1, y1 = cx - paper_w // 2, cy - paper_h // 2
    cv2.rectangle(frame, (x1, y1), (x1 + paper_w, y1 + paper_h), (255, 255, 255), -1)
    cv2.putText(frame, letter, (cx - 28, cy + 35),
                cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 0, 0), 8)
    if add_square:
        # 正方形亮块（长宽比 1.0，应被过滤）
        cv2.rectangle(frame, (60, 300), (160, 400), (240, 240, 240), -1)
    return frame, paper_h


def check(name, cond, detail=''):
    status = PASS if cond else FAIL
    if not cond:
        failures.append(name)
    print(f"[{status}] {name}  {detail}")


def main():
    frame, paper_h = make_frame()

    # 1. 检出 + 几何
    cands = detect_letter_papers(frame, center_v=240.0, center_v_tol_px=120, debug=True)
    check('候选数量为 1（正方形被过滤）', len(cands) == 1, f'n={len(cands)}')
    if not cands:
        sys.exit(1)
    c = cands[0]
    check('中心 u≈320', abs(c['u'] - 320) < 3, f"u={c['u']:.1f}")
    check('中心 v≈240', abs(c['v'] - 240) < 3, f"v={c['v']:.1f}")
    check('h_px≈paper_h', abs(c['h_px'] - paper_h) < 4, f"h_px={c['h_px']:.1f} 期望{paper_h}")
    check('aspect≈1.414', abs(c['aspect'] - 1.414) < 0.1, f"aspect={c['aspect']:.3f}")

    # 2. OCR（合成字体，认出算加分，认不出只提示）
    print(f"       OCR 结果: char={c['char']}（合成字体，None 不判失败）")

    # 3. ocr=False 解耦
    cands2 = detect_letter_papers(frame, center_v=240.0, ocr=False)
    check('ocr=False 仍检出且 char=None',
          len(cands2) == 1 and cands2[0]['char'] is None)

    # 4. tz 反推：paper_h 像素对应 1.0m 左右（tz = fy*H/h_px）
    tz = FY * H_PAPER / c['h_px']
    # 合成图按 paper_h=127px 画，等效 tz ≈ 387.75*0.297/127 ≈ 0.907m
    expected = FY * H_PAPER / paper_h
    err = abs(tz - expected) / expected
    check('tz 反推误差 < 5%', err < 0.05, f"tz={tz:.3f}m 期望≈{expected:.3f}m err={err:.1%}")

    # 5. 齐平过滤：center_v 给 240，纸画到 y=100 处应被过滤
    frame2, _ = make_frame(cy=100, add_square=False)
    cands3 = detect_letter_papers(frame2, center_v=240.0, center_v_tol_px=60, ocr=False)
    check('center_v 过滤生效', len(cands3) == 0, f'n={len(cands3)}')

    print()
    if failures:
        print(f"共 {len(failures)} 项失败: {failures}")
        sys.exit(1)
    print("全部通过")


if __name__ == '__main__':
    main()
