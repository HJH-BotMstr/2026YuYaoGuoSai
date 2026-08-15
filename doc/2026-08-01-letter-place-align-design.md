# 放置区字母对齐（letter place align）设计方案

**日期**：2026-08-01（v2，按现场确认事实修订）  
**状态**：待实现（方案评审）  
**目标**：机械狗携带方块到达放置区附近后，用头部摄像头识别放置区纸箱上的目标字母（A/B/C/D），**不依赖 AprilTag、不使用超声波**，以字母所在的 A4 纸为视觉参照完成"正对字母"的对齐，最终站位与抓取前 AprilTag 对齐的站位保持一致，然后向 `grasp_task` 发布 `/grasp/place` 信号下发放置指令。

> 本文档格式与思路参照 `2026-07-26-apriltag-place1-design.md`。状态机与运动接口大量复用该已实现模块，**核心差异只在"目标检测与位姿估计"一环**。
>
> v2 修订依据的现场确认事实：
> 1. 放置区为**纸箱 + 一定高度贴 A4 纸打印的字母**，纸面与机械狗摄像头**齐平**；
> 2. **不用超声波**；
> 3. 目标字母**暂时由人工指定**。
>
> 这三条事实让方案大幅简化：A4 纸是标准尺寸（210×297 mm）、白纸贴在纸箱上对比强、边缘锐利——**纸张轮廓本身就是比 OCR 字母框稳定得多的几何参照**；纸面与摄像头齐平则消除了俯仰角问题。

---

## 0. 与 AprilTag 方案的差异分析（先想清楚再动手）

| 维度 | apriltag_place1（已实现） | 本模块（待实现） |
|---|---|---|
| 检测对象 | AprilTag（tag25h9, id=0），视野内唯一 | 纸箱上的 A4 纸 + 字母，**多个放置区可能同时入镜**，需按目标字母挑选 |
| 身份识别 | 检测库自带 tag_id | Tesseract OCR（现成代码），只回答"是不是目标字母" |
| 几何参照 | 检测库输出 6D 位姿 `(tx, ty, tz, R)` | **A4 纸轮廓**：中心像素 → 角度/横向；纸高像素 → 距离（§4） |
| 检测速度 | 30 fps 逐帧检测 | 轮廓检测很快，但 OCR 慢（0.2~0.5 s），需异步降频（§3.3） |
| 近距离表现 | Tag 贴脸仍可检测（已验证到 0.08 m） | **A4 纸在 ~0.3 m 以内会超出画面**，视觉闭环有距离下限（§4.4） |
| 完成后信号 | `/grasp/start`（Bool） | `/grasp/place`（String，"A/B/C/D"，对接 grasp_node 的 PLACING 阶段） |
| 触发信号 | `/apriltag_place1/start`（Bool） | 需**携带目标字母**：`/letter_place/start`（String，"A/B/C/D"，人工发布） |

结论：状态机骨架（触发→搜索→航向→横向→前进→校验→盲进→发信号）基本沿用；需要重新设计的是**两级检测（轮廓 + OCR）**、**由纸张轮廓推算位姿**、以及**视觉闭环距离下限引起的 approach 段改造**。

---

## 1. 应用场景与坐标约定

### 1.1 场景（已现场确认）

- 放置区为四个纸箱，每个纸箱在与**机械狗摄像头齐平的高度**贴一张 A4 纸，纸上依次打印一个大字母（A/B/C/D），字母居中于纸面。
- 目标字母由**人工**通过触发消息指定（巡检自动判定是后续项，接口已预留）。
- 机械狗抓取方块后，由导航/人工带到放置区附近：目标 A4 纸已进入摄像头视野，距纸箱约 0.5~1.2 m，机身大致朝纸箱。
- 对齐完成后狗停在纸箱正前方，**最终站位与抓取前一致**（§4.4 参数化保证），随后发布 `/grasp/place`。

### 1.2 前提假设

1. A4 纸为标准尺寸（210×297 mm），横竖贴法固定（配置项 `paper_orientation`），白纸上黑色印刷字母。
2. 纸箱面近似竖直，狗对齐时近似正对（先航向对准再逼近的流程保证）。
3. 字母在 A4 纸上居中印刷；若不居中，用配置 `letter_offset_x_m` 补偿。
4. 纸箱为**牛皮纸色、近似正方形箱面、尺寸远大于 A4**，A4 白纸贴在箱面**正中间**（已现场确认）。因此即使相邻纸箱紧贴，两张 A4 之间也始终隔着牛皮纸边距，白纸轮廓天然独立，不存在粘连问题；正方形箱面长宽比 ≈ 1.0，过不了 §3.1 的 √2 筛选，不会被误检为 A4。
5. 触发时目标 A4 纸已在视野内，本模块**不做大范围搜索**。

### 1.3 坐标系

与 apriltag_place1 完全一致：以头部 RGB 摄像头为参考，`X` 右正、`Z` 前正；运动指令走 `/move`（`+x` 前进，`+y` 左移，`+theta` 逆时针，角度制）。纸面与摄像头齐平，**无需考虑俯仰**，`v`（纸中心纵坐标）只用作合理性过滤（应 ≈ `cy`）。

---

## 2. 整体架构

新增 ROS2 包 `letter_place_align`（`lite3_ws/src/grasp/letter_place_align/`）：

```
            /letter_place/start (String, "A/B/C/D", 人工发布)
                               │
                               ▼
┌─────────────────┐     RGB 帧      ┌──────────────────────┐
│ 头部摄像头      │ ───────────────▶│ letter_place_align   │
│ (RealSense /dev/video6)           │ _node (新增)         │
└─────────────────┘                 │  · A4 轮廓检测       │
                                    │  · 框内 OCR(复用     │
                                    │    gauge_yolo_new)   │
                                    │  · 轮廓→位姿推算     │
                                    └──────────┬───────────┘
                                               │
                          ┌────────────────────┼────────────────────┐
                          ▼                    ▼                    ▼
                   ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
                   │ /move       │    │ /pose_control│    │ /grasp/place    │
                   │ (Pose2D)    │    │ /command     │    │ (String,"A-D")  │
                   └──────┬──────┘    └──────┬───────┘    └────────┬────────┘
                          └──────────────────┤                     │
                                             ▼                     ▼
                              ┌─────────────────────────┐   ┌────────────────┐
                              │ pose_controller_node    │   │ grasp_node     │
                              │ (pose_control, 复用)     │   │ (grasp_task,   │
                              └──────────────┬──────────┘   │  PLACING 阶段  │
                                             │ /cmd_vel     │  等待此信号)   │
                                             ▼              └────────────────┘
                                   官方 ROS2 栈 → Lite3
```

运动链路（`/move`、`/pose_control/command`、`/cmd_vel` 零速判定、链路预检）与 apriltag_place1 **完全一致，不做任何改动**。

### 2.1 与 grasp_task 的时序衔接

```
grasp_node:  ... → 抓取 → 运输(TRANSPORTING) → PLACING 等待 /grasp/place ─┐
                                                                          │
人工:        把狗带到放置区附近                                           │
             发布 /letter_place/start "B"                                 │
                                                                          ▼
letter_place_align_node:  搜索字母 B → 对齐 → 盲进 → 发布 /grasp/place "B" ─▶ grasp_node 执行放置 → /grasp/result
```

注意：grasp_node 的 `_on_place` 会同时执行 `memory.set_zone(zone)`，因此**不需要**提前发 `/grasp/set_zone`，一个 `/grasp/place` 即完成区域指定 + 触发放置。

---

## 3. 检测：A4 轮廓定位 + 框内 OCR 两级设计

这是"简单但稳定精确"的关键取舍：**不让 OCR 承担几何测量**（OCR 框松散、抖动大），轮廓和字符各司其职。

### 3.1 第一级：A4 纸轮廓检测（快，逐帧可做）

白纸贴纸箱，亮度/饱和度对比强，用经典 OpenCV 即可，不引入任何新依赖：

```
灰度 → 二值化（Otsu 或固定阈值，现场定）→ findContours（外轮廓）
→ 多边形逼近 approxPolyDP → 筛选：
   · 四边形
   · 长宽比 ≈ √2 ≈ 1.414（±20%，透视下会变形，阈值放宽）
   · 面积在合理范围（由距离范围 0.3~1.2 m 反推像素面积上下限）
   · 中心纵坐标 v ≈ cy ± 容差（纸与摄像头齐平，过滤无关亮块）
→ 输出每个候选：四角点、中心 (u,v)、纸高 h_px（竖直方向边长）
```

- 竖贴时 `h_px` 取较长边（0.297 m 对应边），横贴取短边（0.210 m），由 `paper_orientation` 配置决定。
- 长宽比校验同时是**检测质量自证**：比值偏离 1.414 太多说明轮廓残缺（被遮挡/粘连），该帧结果不可信。

### 3.2 第二级：框内 OCR（慢，只回答身份）

对每个候选轮廓取**内接矩形 ROI**（向内收缩约 15% 去掉纸边），套用 `tools/gauge_yolo_new.py` 中已验证的 OCR 预处理流水线：

```
ROI → 灰度 → Otsu 二值化 → 2× 放大 → pytesseract.image_to_string
      (config='--psm 7 -c tessedit_char_whitelist=ABCD') → 取第一个合法字符
```

与现有 `recognize_letter` / `recognize_letter_box` 的处理完全同构，只是 ROI 来源从"表盘框上方"换成"A4 轮廓内部"。几何信息已由轮廓提供，**`image_to_string` 就够用，不需要 `image_to_data`**——这是相对 v1 方案的简化。

复用方式沿用 `gauge_yolo_detector` 的做法：`sys.path.insert(0, '.../tools')` 后直接 import。建议在 `tools/gauge_yolo_new.py` 新增一个组合函数，保持单一来源：

```python
def detect_letter_papers(frame, orientation='portrait', min_area=..., ...) -> List[dict]:
    """A4 轮廓检测 + 框内 OCR，返回:
    [{'char': 'B',        # OCR 结果，识别失败为 None
      'u': 412.0, 'v': 205.0,          # 轮廓中心像素
      'h_px': 115.0,                    # 纸高像素（orientation 对应边）
      'aspect': 1.39,                   # 长宽比（质量自证）
      'corners': [...]}, ...]"""
```

### 3.3 检测节奏：身份慢认、几何快算

- **OCR 只在轮廓集合变化时触发**：连续两帧轮廓中心/尺寸变化小于阈值时，沿用上一轮的 `char` 结果（狗停稳时纸不动，没必要反复 OCR）；轮廓明显变化（运动后）才对该轮廓重新 OCR。OCR 放独立线程，不阻塞主循环。
- **轮廓检测每帧都做**（毫秒级），`u/v/h_px` 始终以最新帧为准，主循环 10 Hz 不受 OCR 拖累。
- 结果带时间戳，超过 `detect_stale_s`（建议 1.0 s）视为丢失。

### 3.4 目标选取

1. 候选中滤掉 `char != target_letter` 的轮廓（`char=None` 即 OCR 未认出的纸，忽略并计诊断）；
2. 多个匹配取 `h_px` 最大者（最近最可信）；
3. 对齐过程中若**另一张纸稳定识别为别的字母且置信连续多次**，说明对错了纸箱，回 `wait_detect` 并明确告警（放错区比放失败更严重）。

---

## 4. 位姿估计：从纸张轮廓推算

### 4.1 水平角与横向偏差

```
alpha = atan((u - cx) / fx)
```

- **yaw_align**：与 apriltag 版同构，`theta_cmd = -clamp(degrees(alpha), ±max_yaw_step_deg)`。
- **lateral_align**：航向对准后 `tx ≈ tz · (u-cx)/fx + letter_offset_x_m`，`tz` 取 §4.2 估计值；对准后 `(u-cx)/fx` 很小，`tz` 误差被压缩，横向精度足够。

### 4.2 前向距离：纸高反推（主方案）

```
tz = fy · H_paper / h_px        # H_paper：竖贴 0.297 m / 横贴 0.210 m
```

为什么比 v1 的"字母像素高反推"好：

| 对比项 | 字母 OCR 框 | **A4 纸轮廓（采用）** |
|---|---|---|
| 尺寸基准 | 字高需现场量，且字体/字号不定 | **A4 标准尺寸，免测量** |
| 边缘稳定性 | OCR 框松散，逐帧抖动 ±10% | 纸边锐利，轮廓误差 1~2 px |
| 可检测距离 | 字太小 OCR 认不出 | 轮廓在 1.2 m 外仍约 90 px，好检 |
| 身份无关 | 框随识别结果有无 | 轮廓永远在，OCR 失败不丢几何 |

配合措施：

1. **中位数平滑**：最近 N 帧 `h_px` 取中位数；
2. **长宽比自证**（§3.1）：轮廓残缺时该帧不参与；
3. **用竖直边**：纸高不受水平 yaw 旋转的透视压缩影响（yaw 未对准时宽度会被压缩、距离被高估，高度不会），这也是先 yaw_align 再 approach 之外的第二重保险。

### 4.3 与状态机的接口

`_detect_letter()` 返回与 apriltag 版 `_detect_tag()` 同构的 dict，状态机其余代码原样复用：

```python
{"tx": float, "ty": 0.0, "tz": float, "R": None, "raw": {...}}
```

### 4.4 视觉闭环距离下限与 approach 改造（重要）

A4 竖贴纸高 0.297 m，在 640×480、fy≈388 的内参下：

| 距离 | 纸高像素 | 状态 |
|---|---|---|
| 1.0 m | ≈115 px | 检测舒适 |
| 0.35 m | ≈330 px | 仍可完整入镜 |
| **< 0.30 m** | > 390 px | **上下边缘开始出画，轮廓不可靠** |

抓取前的站位是"视觉到 0.08 m + 盲进 0.20 m"，而字母方案在 0.08 m 处纸早已出画。因此 approach 段必须改造（这是与 apriltag 流程唯一的结构性差异）：

```
phase_4_approach      视觉闭环逼近，直到 tz <= vision_min_distance_m（默认 0.35）
phase_5_final_check   在 vision_min 处校验（yaw + 横向 + 距离 vs vision_min）
phase_6_blind_forward 一次性开环前进：
                      blind = (tz_measured - target_distance_m) + final_forward_offset_m
                      其中 tz_measured 为 final_check 通过时的实测距离
phase_7_emit_signal   发布 /grasp/place
```

- `target_distance_m` / `final_forward_offset_m` **与 apriltag_place1 保持同值**（0.08 / 0.20），于是 `blind ≡ tz_measured - 0.08 + 0.20`，最终站位与抓取站位**由构造保证一致**——视觉段停在 0.35 m 还是 0.08 m 不影响终点，只是盲进段更长。
- 盲进段加长（约 0.45 m vs apriltag 版 0.20 m）带来的里程计漂移风险：前进方向已对准、路面平整的室内场景下 0.45 m 开环漂移约 1~2 cm，可接受；盲进前 `reset_origin`（沿用 apriltag 版做法）保证里程计基准干净。

---

## 5. 对齐流程

```
phase_0_wait_trigger     等待 /letter_place/start (String)
                         · data ∈ {A,B,C,D} → 记录 target_letter，进入搜索
                         · 其他/空串 → 取消，回 wait_trigger
phase_1_wait_detect      等待目标字母纸出现，稳定缓冲锁定
                         （std(tx),std(tz) < 0.05，同 apriltag 版）
phase_2_yaw_align        用 §4.1 的 alpha，逻辑不变
phase_3_lateral_align    用 §4.1 的 tx，逻辑不变
phase_4_approach         视觉逼近至 vision_min_distance_m（§4.4）
phase_5_final_check      在 vision_min 处三项校验（距离判据改为 vs vision_min）
phase_6_blind_forward    开环前进 blind = tz_measured - target + final_forward（§4.4）
phase_7_emit_signal      发布 /grasp/place = String(target_letter)
```

其余机制全部继承 apriltag 版：任何阶段丢失目标回 `wait_detect`、超 `max_rounds` 进 `error`、首次运动前链路预检、"先见非零速度再持续零速"的运动完成判定、`error` 状态可被触发信号重启。OCR 固有抖动用 `lost_tolerance_s`（建议 2.0 s）容忍：短暂丢失沿用最近一次有效位姿，不回退。

---

## 6. 与现有代码的复用

| 现有代码 | 复用方式 |
|---|---|
| `lite3_ws/src/grasp/apriltag_place1/apriltag_place1_node.py` | **骨架来源**：状态机、`_is_cmd_vel_zero`、`_check_motion_pipeline`、稳定缓冲、盲进、日志风格整体沿用；替换检测层 + approach/emit 段（§4.4、§5） |
| `tools/gauge_yolo_new.py` | OCR 预处理流水线直接 import（sys.path 方式，同 gauge_yolo_detector）；新增 `detect_letter_papers()` 也放这里 |
| `lite3_ws/src/pose_control` | launch 中一并启动，参数直接抄 apriltag_place1.launch.py 已调好的那组（kp=2.0、dist_threshold=0.015 等） |
| `lite3_ws/src/grasp/grasp_task` | 下游，`/grasp/place` 消费方，无需改动 |
| 官方栈 `/simple_cmd` 序列 | 真机前置步骤与 apriltag 版相同 |

**克隆而非抽象的取舍**：apriltag_place1_node 约 733 行，其中约 500 行与检测无关。理想做法是抽公共基类，但 apriltag 模块已真机验证，动它有回归风险，比赛时间紧——本期克隆骨架 + 替换检测层，两包互留同源注释，赛后如出现第三处对齐场景再合并抽象。

---

## 7. 节点设计（新增）

### 7.1 文件位置

```
lite3_ws/src/grasp/letter_place_align/
├── letter_place_align/
│   └── letter_place_align_node.py     # 状态机（克隆 apriltag_place1_node 骨架）
├── config/letter_place_align.yaml
├── launch/letter_place_align.launch.py # 同时启动 pose_control + 本节点
├── setup.py / package.xml / README.md
```

`tools/gauge_yolo_new.py` 新增 `detect_letter_papers()`（§3.2）。

### 7.2 类设计（只列与 apriltag 版不同的部分）

```python
class LetterPlaceAlignNode(Node):
    # 订阅：/letter_place/start(String)、/cmd_vel(Twist)、/leg_odom2(Odometry)
    # 发布：/move(Pose2D)、/pose_control/command(String)、/grasp/place(String)

    def _trigger_cb(self, msg: String):
        # data ∈ ABCD → 记录 target_letter，进 wait_detect；否则取消回 wait_trigger

    def _detect_letter(self, frame) -> Optional[dict]:
        # detect_letter_papers(frame) → 选目标字母纸（§3.4）
        # → 推算 alpha/tx/tz（§4）→ 返回与 _detect_tag 同构的 dict
        # OCR 结果按 §3.3 缓存复用；几何每帧重算
        # 陈旧(> detect_stale_s) / 短暂丢失(< lost_tolerance_s) 的处理

    def _do_approach(self, frame):
        # 视觉逼近至 vision_min_distance_m（§4.4）

    def _do_blind_forward(self):
        # blind = tz_measured - target_distance_m + final_forward_offset_m
        # reset_origin → sleep(0.15) → /move(blind) → 零速判定

    # 以下与 apriltag 版同构，不再赘述：
    # _is_stable / _is_cmd_vel_zero / _check_motion_pipeline / _send_move
    # _reset_origin / _do_wait_detect / _do_yaw_align / _do_lateral_align / _main_loop

    def _emit_place(self):
        # 发布 String(data=self._target_letter) 到 /grasp/place
```

---

## 8. 配置文件（草案）

`lite3_ws/src/grasp/letter_place_align/config/letter_place_align.yaml`：

```yaml
letter_place_align:
  ros__parameters:
    # ── 触发与完成信号 ──
    trigger_topic: "/letter_place/start"   # std_msgs/String，"A/B/C/D"（人工发布）
    place_topic:   "/grasp/place"          # std_msgs/String，对接 grasp_task

    # ── 摄像头（与 apriltag 版同一颗头）──
    camera_device: "/dev/video6"
    image_width: 640
    image_height: 480
    fps: 30

    # ── 内参（同 apriltag 版现状：借用机械臂内参，待标定）──
    camera_matrix: [388.1454, 0.0, 329.4121, 0.0, 387.7497, 223.481, 0.0, 0.0, 1.0]
    dist_coeffs: [-0.1571, -0.218, -0.0024, -0.0011, 0.2089]

    # ── A4 纸参数（标准尺寸，一般不用改）──
    paper_orientation: "portrait"    # portrait: 纸高边 0.297m / landscape: 0.210m
    paper_aspect_ratio: 1.414        # √2，轮廓质量自证
    aspect_tolerance: 0.25           # 长宽比允许偏差比例（透视变形）
    center_v_tolerance_px: 120       # 纸中心纵坐标与 cy 的允许偏差（齐平过滤）
    letter_offset_x_m: 0.0           # 字母相对纸中心的横向偏移（居中印刷为 0）

    # ── 检测节奏 ──
    min_conf: 40.0                   # OCR 置信度下限（若用 image_to_data 则有效）
    detect_stale_s: 1.0              # 结果超过该时长视为丢失
    lost_tolerance_s: 2.0            # 短暂丢失容忍（沿用最近位姿）

    # ── 站位（target/final_forward 与 apriltag_place1 保持一致，勿单独调）──
    target_distance_m: 0.08
    final_forward_offset_m: 0.20
    vision_min_distance_m: 0.35      # 视觉闭环距离下限（A4 出画临界，§4.4）

    # ── 阈值 ──
    yaw_align_threshold_deg: 3.0
    max_yaw_step_deg: 3.0
    lateral_threshold_m: 0.03
    distance_threshold_m: 0.03       # 相对 vision_min 的到位容差

    # ── 流程控制 ──
    max_rounds: 10
    stable_frames: 10
    detect_timeout_s: 15.0           # 比 apriltag 版略长（OCR 慢）
    cmd_vel_zero_timeout_s: 1.5
    move_timeout_s: 10.0
```

---

## 9. 文件变更清单

| 文件 | 变更类型 | 说明 |
|---|---|---|
| `lite3_ws/src/grasp/letter_place_align/` | 新建 ROS2 包 | 节点、配置、launch、README |
| `tools/gauge_yolo_new.py` | 修改 | 新增 `detect_letter_papers()`，不动现有函数 |
| `lite3_ws/src/grasp/grasp_task/` | 不改 | `/grasp/place` 接口已就绪 |
| `lite3_ws/src/grasp/apriltag_place1/` | 不改 | 已验证模块，避免回归 |

---

## 10. 待确认 / 待标定

| 项 | 说明 | 影响 |
|---|---|---|
| `paper_orientation` | A4 竖贴还是横贴 | 决定距离反推用哪条边（0.297 / 0.210） |
| 纸箱现场布局 | 纸箱间距、纸面反光、触发时狗与纸箱的典型距离 | 决定面积过滤范围、`detect_timeout_s` |
| 头部摄像头内参 | 与 apriltag 版共用，仍借用机械臂内参 | 影响 fx/cx 进而影响 alpha 与 tz；两个模块一次标定同时受益 |
| 字母是否居中 | 不居中则填 `letter_offset_x_m` | 横向对准精度 |
| 目标字母自动来源 | 目前人工发 `/letter_place/start` | 后续由巡检模块（表计识别结果）自动发布，接口已兼容 |

---

## 11. 启动流程（实现后预期）

```bash
cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select letter_place_align grasp_task --symlink-install
source install/setup.bash

# 终端 1：grasp（已在 PLACING 阶段等待 /grasp/place）
ros2 launch grasp_task grasp.launch.py

# 终端 2：字母对齐（launch 含 pose_control）
ros2 launch letter_place_align letter_place_align.launch.py

# 终端 3：狗到位后人工触发（携带目标字母）
ros2 topic pub /letter_place/start std_msgs/String "data: 'B'" --once
```

真机前置步骤（官方栈 source + `/simple_cmd` 序列）与 apriltag_place1 完全相同。

---

## 12. 鲁棒性设计

1. **轮廓质量自证**：四边形 + 长宽比 ≈ √2 校验，残缺轮廓、正方形箱面、胶带标签等干扰直接丢弃（§3.1）。
2. **身份与几何解耦**：OCR 失败只丢身份、不丢几何；轮廓变化才重新 OCR（§3.3）。
3. **纸高抗 yaw 畸变**：距离用竖直边反推，不受航向未对准的透视压缩影响（§4.2）。
4. **近距离出画防护**：视觉闭环止步 `vision_min_distance_m`，剩余距离开环盲进（§4.4）。
5. **目标锁定 + 防呆**：只对 `target_letter` 的纸闭环；稳定看到别的字母回退告警（§3.4）。
6. **短暂丢失容忍**：`lost_tolerance_s` 内沿用最近位姿（§5）。
7. **其余继承**：多帧稳定、最大轮次、零速判定、链路预检等全部沿用 apriltag 版已验证机制。

---

## 13. 风险与开放问题

1. **箱面干扰与胶带/标签误检**：牛皮纸箱上可能有白色胶带、快递标签等亮块，可能形成类四边形亮区。缓解：√2 长宽比 + 面积范围 + `v ≈ cy` 齐平过滤三重筛选（§3.1），正方形箱面本身（长宽比 ≈ 1.0）天然被长宽比校验排除。
2. **纸面反光 / 光照不均**导致二值化失败：哑光纸 + 现场调二值化阈值；必要时加自适应阈值（`adaptiveThreshold`），实现时先试 Otsu。
3. **OCR 远距离识别率低**：触发前提要求纸在 1.2 m 内入镜；若现场 OCR 只能认 0.8 m 内，由"人工送达位置"保证，或等待逼近过程中轮廓变大后自然识别（身份在 yaw_align 阶段前锁定即可，逼近中轮廓变大后 OCR 会更稳）。
4. **盲进段加长（约 0.45 m）的开环漂移**：室内平整地面约 1~2 cm，可接受；若实测偏大，可在盲进中途插入一次"低头看地面标记"之类的二次校正——本期不做，先实测。

---

## 14. 验证计划（实现后）

1. **纯视觉静态验证**：手持摄像头对贴了 A4 的纸箱跑 `detect_letter_papers` + tz 推算脚本（参照 apriltag 版 §14.6），确认 0.35~1.2 m 内轮廓检出率、OCR 识别率、tz 误差（目标：tz 误差 < 5%）。
2. **干跑**：`grasp_task` 以 `dry_run:=true` 启动，全流程跑通，确认 `/grasp/place` 的字母与触发字母一致。
3. **真机分阶段**：固定狗先测 yaw_align → lateral_align → approach → blind_forward，同 apriltag 版 §14.7 的顺序；重点验证盲进后的实际站位与抓取站位的一致性（尺量）。
4. **联调**：与 grasp_task 完整跑"抓取→运输→字母对齐→放置"，确认机械臂放置动作可达。

source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/transfer/install/setup.bash

# 1) 回零
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553716741, size: 0, type: 0}" --once
# 2) 起立
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553714178, size: 0, type: 0}" --once
# 3) 切换到 locomotion（运动）模式
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553716998, size: 0, type: 0}" --once
# 4) 切换到 autonomous（自动）模式
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553716739, size: 0, type: 0}" --once
# 5) 设置步态为 slow
ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 553714432, size: 0, type: 0}" --once

# 确认里程计有数据再继续（有数据输出即 Ctrl+C）
ros2 topic echo /leg_odom2


cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash && source install/setup.bash

# 第一次先干跑（不真动机械臂），确认能收到 /grasp/place
ros2 launch grasp_task grasp.launch.py dry_run:=true

# 验证通过后去掉 dry_run 真跑，并走到 PLACING 阶段等待放置信号
ros2 launch grasp_task grasp.launch.py



cd /home/ysc/2026YuYaoGuoSai/lite3_ws
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch letter_place_align letter_place_align.launch.py

source /opt/ros/foxy/setup.bash

# 狗到放置区附近（目标纸入镜、距离 0.5~1.2m、大致朝向纸箱）后触发，B 换成实际目标字母
ros2 topic pub /letter_place/start std_msgs/String "data: 'B'" --once

# 需要中止时发任意非 ABCD 的值（视为取消，狗停下回到等待态）
ros2 topic pub /letter_place/start std_msgs/String "data: 'X'" --once
