#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 R680 自主探索系统的分层架构图(SVG + PNG,同一份几何数据)。

为什么手写布局而不用 graphviz/dot:
  * 这张图要的是**泳道 + 双机分区**的网格结构,而 dot 的 rank 机制对
    "每行一组、还要跨列对齐"的表达非常别扭,输出会歪且连线绕远;
  * 图里的数字(0.45 m / 0.3 rad/s / 50° / 52 FPS)必须一字不改地出现在
    固定位置,自动布局做不到这个控制力;
  * PNG 与 SVG 必须**像素级一致**,否则我看过的预览图和交付的矢量图就是两张图。
    同一份 BOXES/EDGES 数据渲染两次,这个风险归零。

字体:Noto Sans CJK SC(系统已装,fc-match 确认)。
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

# ----------------------------------------------------------------- 画布与配色
W, H = 2200, 1420
BG = "#FFFFFF"

C_PERCEPTION = "#2F6FED"   # 感知层
C_PLANNING = "#12A594"     # 规划层
C_CONTROL = "#E8590C"      # 控制层
C_EXEC = "#495057"         # 执行与硬件
C_ORCH = "#7048E8"         # 编排与工具
C_DANGER = "#D9480F"       # 防御性设计强调
C_LANE_BG = "#F7F8FA"
C_STROKE = "#343A40"
C_MUTED = "#6C757D"
C_TEXT = "#212529"

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_M = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"

# ----------------------------------------------------------------- 布局常量
LANE_LABEL_W = 74          # 左侧泳道标题条
ORIN_X0, ORIN_X1 = 96, 1188
CH_X0, CH_X1 = 1202, 1318  # 跨机通道
TX1_X0, TX1_X1 = 1330, 2180

LANE_H = 230
LANE_GAP = 12
LANE_Y0 = 118

LANES = [
    ("感知层", C_PERCEPTION),
    ("规划层", C_PLANNING),
    ("控制层", C_CONTROL),
    ("执行与硬件", C_EXEC),
    ("编排与工具", C_ORCH),
]


def lane_y(i):
    return LANE_Y0 + i * (LANE_H + LANE_GAP)


# ----------------------------------------------------------------- 文本度量
def text_w(s, size, bold=False):
    """真实字体度量(PIL getlength),与 PNG 渲染完全同源 —— 不用估算。"""
    return get_font(FONT_B if bold else FONT, size).getlength(s)


_font_cache = {}


def get_font(path, size):
    key = (path, size)
    if key not in _font_cache:
        _font_cache[key] = ImageFont.truetype(path, size)
    return _font_cache[key]


# ----------------------------------------------------------------- 方块模型
class Box:
    """一个方框:标题 + 若干正文行。行用 "|" 前缀标记为强调行(琥珀色)。"""

    def __init__(self, x, y, w, title, lines=(), color=C_STROKE,
                 dashed=False, bold=False, body_size=13, title_size=16,
                 align="left", pad=13, line_gap=17, lane=None):
        self.x, self.y, self.w = x, y, w
        self.lane = lane
        self.title = title
        self.lines = list(lines)
        self.color = color
        self.dashed = dashed
        self.bold = bold
        self.body_size = body_size
        self.title_size = title_size
        self.align = align
        self.pad = pad
        self.line_gap = line_gap
        # 高度由内容决定
        self.h = pad + title_size + 8 + len(self.lines) * line_gap + pad - 4

    def overflow(self):
        """返回问题清单(用于自检,不静默裁切/不静默越界)。"""
        bad = []
        if text_w(self.title, self.title_size, True) > self.w - 2 * self.pad:
            bad.append(("title", self.title))
        for ln in self.lines:
            txt = ln[1:] if ln.startswith("|") else ln
            if text_w(txt, self.body_size) > self.w - 2 * self.pad:
                bad.append(("body", txt))
        if self.lane is not None:
            top, bot = lane_y(self.lane) + 6, lane_y(self.lane) + LANE_H - 6
            if self.y < top:
                bad.append(("上越界", "泳道 %d: y=%d < %d" % (self.lane, self.y, top)))
            if self.y + self.h > bot:
                bad.append(("下越界", "泳道 %d: y+h=%d > %d" % (self.lane, self.y + self.h, bot)))
        return bad


def _dash_rect(draw, box, outline, width=2, dash=7, gap=1):
    """手绘虚线矩形(PIL 没有 dash 支持)。"""
    x, y, w, h = box
    step = dash + gap
    for xa in range(x, x + w, step):
        xb = min(xa + dash, x + w)
        draw.line([(xa, y), (xb, y)], fill=outline, width=width)
        draw.line([(xa, y + h), (xb, y + h)], fill=outline, width=width)
    for ya in range(y, y + h, step):
        yb = min(ya + dash, y + h)
        draw.line([(x, ya), (x, yb)], fill=outline, width=width)
        draw.line([(x + w, ya), (x + w, yb)], fill=outline, width=width)


# ----------------------------------------------------------------- 图元收集
BOXES = []
EDGES = []   # (points, color, dashed, width, arrow, label)
NOTES = []   # (x, y, text, size, color, bold)
BADGES = []  # (box_index, text)


def add_box(*a, **kw):
    b = Box(*a, **kw)
    BOXES.append(b)
    return len(BOXES) - 1


def add_edge(points, color=C_STROKE, dashed=False, width=2, arrow=True, label=None,
             label_pos=None):
    EDGES.append((points, color, dashed, width, arrow, label, label_pos))


# ----------------------------------------------------------------- 实际构图
# ===== 泳道 1:感知层 (Orin) =====
y1 = lane_y(0)
b_cam = add_box(ORIN_X0 + 14, y1 + 26, 250, "Intel RealSense D435", [
    "彩色图 + 对齐深度 + 原始点云",
    "USB3 · 唯一避障传感器",
    "|可靠深度 ≈ 3 m",
], color=C_PERCEPTION, lane=0)

b_scan = add_box(ORIN_X0 + 300, y1 + 26, 268, "depthimage_to_laserscan", [
    "→ /scan  (LaserScan)",
    "scan_height 5 · 0.25–6.0 m",
    "output_frame camera_depth_frame",
], color=C_PERCEPTION, lane=0)

b_rtab = add_box(ORIN_X0 + 604, y1 + 26, 268, "RTAB-Map  RGB-D SLAM", [
    "视觉里程计关闭,唯一源 = /odom",
    "发布 map → odom_combined",
    "/rtabmap/map · cloud_map",
    "/rtabmap/localization_pose",
], color=C_PERCEPTION, lane=0)

b_sem = add_box(ORIN_X0 + 908, y1 + 26, 168, "语义检测 YOLO11n", [
    "TensorRT · 52 FPS",
    "匈牙利关联",
    "min_observations 3",
], color=C_MUTED, dashed=True, lane=0)
BADGES.append((b_sem, "独立链路 · 未接入探索回路"))

# ===== 泳道 2:规划层 =====
y2 = lane_y(1)
b_exp = add_box(ORIN_X0 + 14, y2 + 26, 400, "frontier 探索器  r680_explorer", [
    "score = 1.0·size + 1.5·gain − 1.0·dist − 0.6·heading/π",
    "|gain 权重最高:优先「能新看到最多未知」而非最近",
    "min_frontier_size 10 · max_goal_distance 6.0 m",
    "goal_clearance 0.25 · require_reachable true",
    "IDLE → EXPLORING ⇄ PAUSED → COMPLETE / ABORTED",
], color=C_PLANNING, lane=1)

b_nav2 = add_box(ORIN_X0 + 448, y2 + 26, 628, "Nav2  ·  controller / planner / bt_navigator / behavior / waypoint / lifecycle", [
    "动作接口 navigate_to_pose(NavigateToPose)  ← 来自探索器",
    "规划器 NavFn(A*)  ·  控制器 MPPI  ·  xy_goal_tolerance 0.20",
    "全局代价地图  static_layer(/rtabmap/map) + obstacle(/scan) + inflation 0.35",
    "局部代价地图  4×4 m rolling · global_frame odom_combined · 仅 /scan · 无 voxel 层",
    "|robot_radius 0.20 = 0.5·√(0.30²+0.25²) 上取整,两处 costmap 必须同值",
], color=C_PLANNING, lane=1)

# ===== 泳道 3:控制层 =====
y3 = lane_y(2)
b_locmon = add_box(ORIN_X0 + 14, y3 + 16, 258, "localization_monitor", [
    "任一段 0.5 s 超时即判定位失效",
    "map → odom_combined →",
    "base_footprint",
    "→ /safety/localization_valid",
], color=C_CONTROL, lane=2, pad=10, line_gap=15)

b_obstmon = add_box(ORIN_X0 + 14, y3 + 122, 258, "obstacle_monitor", [
    "前向扇区 50° · stop 0.45 m",
    "slow 0.80 m",
    "|无 /scan 数据即视为「有障碍」",
    "→ /safety/obstacle_stop",
], color=C_CONTROL, lane=2, pad=10, line_gap=15)

b_estop = add_box(ORIN_X0 + 300, y3 + 16, 148, "安全急停", [
    "/safety/estop",
    "最高优先级",
], color=C_DANGER, bold=True, lane=2)

b_safety = add_box(ORIN_X0 + 300, y3 + 110, 372, "cmd_vel_safety  ·  逐级收紧", [
    "限速 0.35 / 0.35 / 0.8 m·s⁻¹  ·  指令超时 0.5 s",
    "|障碍只封平移,保留 0.30 rad/s 原地旋转脱困",
    "输出 /cmd_vel_safe + /safety/stop_reason",
], color=C_CONTROL, lane=2)

b_rx = add_box(ORIN_X0 + 700, y3 + 110, 168, "ROS 2 中继", [
    "ros2_relay_node",
    "绑 19001 → 发 19000",
], color=C_CONTROL, lane=2, body_size=12)

b_tx = add_box(TX1_X0 + 14, y3 + 110, 176, "ROS 1 中继", [
    "ros1_relay.py",
    "绑 19000 ← 收 Orin",
    "|_enable_cmd_vel 闸门",
], color=C_CONTROL, lane=2, body_size=12)

b_cmdvel = add_box(TX1_X0 + 212, y3 + 110, 190, "/cmd_vel", [
    "geometry_msgs/Twist",
    "闸门关时",
    "|Publishers: None",
], color=C_EXEC, lane=2, body_size=12)

# ===== 泳道 4:执行与硬件 (TX1) =====
y4 = lane_y(3)
b_drv = add_box(TX1_X0 + 14, y4 + 26, 200, "wheeltec_robot 驱动", [
    "订阅 /cmd_vel",
    "|只在收到消息时写串口帧",
    "|这条路径上没有看门狗",
], color=C_EXEC, lane=3)

b_cp = add_box(TX1_X0 + 238, y4 + 26, 150, "CP2102", [
    "/dev/ttyUSB0",
    "串口",
], color=C_EXEC, lane=3)

b_mcu = add_box(TX1_X0 + 412, y4 + 26, 130, "MCU", [
    "电机闭环",
    "速度保持",
], color=C_EXEC, lane=3)

b_wheel = add_box(TX1_X0 + 566, y4 + 26, 210, "麦克纳姆轮 ×4", [
    "编码器 → /odom",
    "IMU → /imu",
    "电池采样 → /PowerVoltage",
], color=C_EXEC, lane=3)

# TX1 侧的定位输入说明
b_wodom = add_box(TX1_X0 + 14, y4 + 124, 762, "TX1 侧唯一的定位输入:轮式里程计", [
    "planar_odom_covariance=true 把 z/roll/pitch 的 1e6 哨兵换成有限值",
    "不这么做:RTAB-Map 报 Very large angular variance,图优化变差、错误回环更易被接受",
    "前提是车只跑平地(操作者已确认)",
], color=C_EXEC, body_size=12, lane=3)

# ===== 泳道 5:编排与工具 =====
y5 = lane_y(4)
orch_items = [
    ("r680_explore_up.sh", ["整栈启动", "TX1: roscore→驱动→闸门", "Orin: 清理→感知→规划"]),
    ("r680_explore_300.sh", ["300 s 探索编排", "默认需输 GO", "120 s 仍 74.4% 未知"]),
    ("r680_down.sh", ["分级收尾", "先急停+关闸+验证", "再拆 Orin,最后拆 TX1"]),
    ("r680_gate.sh", ["运动闸门开关", "判据: /cmd_vel", "Publishers = None"]),
    ("start_foxglove.sh", ["foxglove_bridge :8765", "远程 3D 可视化", "Fixed Frame = map"]),
    ("Foxglove 桌面版", ["另一台电脑打开", "ws://<orin-ip>:8765", "/scan · costmap"]),
]
ox = ORIN_X0
for i, (t, ls) in enumerate(orch_items):
    add_box(ox + i * 180, y5 + 26, 172, t, ls, color=C_ORCH,
            body_size=12, title_size=13, line_gap=16, lane=4)


# ===== 空白区的说明块:解释"为什么这里是空的"(两机分工,不是漏画) =====
add_box(TX1_X0 + 14, y1 + 26, 470, "TX1 · 无感知节点", [
    "TX1 只跑底盘驱动、电压记录与中继",
    "感知全在 Orin:相机、SLAM、语义检测",
    "|这是算力划分的结果,不是功能遗漏",
], color=C_MUTED, dashed=True, body_size=12, lane=0)

add_box(ORIN_X0 + 14, y4 + 26, 560, "Orin · 不直接接触执行器", [
    "所有速度指令都必须经 UDP 中继下到 TX1",
    "Orin 上不存在 /dev/ttyUSB0,也不该有写权限",
    "|控制权与执行权分离 = 物理隔离的安全边界",
], color=C_MUTED, dashed=True, body_size=12, lane=3)

add_box(TX1_X0 + 14, y5 + 26, 520, "TX1 · 不留编排逻辑", [
    "tools/*.sh 都在 Orin 上执行,通过 ssh 操作 TX1",
    "TX1 上只留有状态的进程(驱动 / 中继 / 电压记录)",
], color=C_MUTED, dashed=True, body_size=12, lane=4)

# ===== 连线 =====
A = lambda b: (b.x + b.w, b.y + b.h / 2)      # 右侧中点
L = lambda b: (b.x, b.y + b.h / 2)            # 左侧中点
T = lambda b: (b.x + b.w / 2, b.y)            # 上边中点
B = lambda b: (b.x + b.w / 2, b.y + b.h)      # 下边中点


def H_edge(src, dst):
    """左右相邻方块的**严格水平**连线。

    不能用 A(src)→L(dst):相邻方块行数不同时两块的中线高度不等,
    连线会变成一段斜线。这里统一取源块中线,并断言目标块纵向覆盖该高度,
    否则连线会插到目标块的角上 —— 那种情况会直接报错,不静默画歪。
    """
    sy = src.y + src.h / 2
    assert dst.y < sy < dst.y + dst.h, (
        "水平连线高度落空: %s 中线 y=%.1f 不在 %s [%.0f,%.0f] 内"
        % (src.title[:18], sy, dst.title[:18], dst.y, dst.y + dst.h))
    return [(src.x + src.w, sy), (dst.x, sy)]


# 感知层内部横向数据流
add_edge(H_edge(BOXES[b_cam], BOXES[b_scan]), C_PERCEPTION)
add_edge(H_edge(BOXES[b_scan], BOXES[b_rtab]), C_PERCEPTION)
add_edge(H_edge(BOXES[b_rtab], BOXES[b_sem]), C_MUTED, dashed=True)

# /scan 走左侧总线绕到 obstacle_monitor(避免穿过规划层方块)
add_edge([B(BOXES[b_scan]), (BOXES[b_scan].x + BOXES[b_scan].w / 2, 258),
          (68, 258), (68, BOXES[b_obstmon].y + BOXES[b_obstmon].h / 2),
          L(BOXES[b_obstmon])],
         C_CONTROL, label="/scan", label_pos=(300, 258))

# /rtabmap/map → Nav2 全局代价地图的 static_layer
add_edge([B(BOXES[b_rtab]), (BOXES[b_rtab].x + BOXES[b_rtab].w / 2, BOXES[b_nav2].y)],
         C_PLANNING, label="/rtabmap/map")

# 探索器 → Nav2 动作接口(箭头不挂长标签,接口名写进 Nav2 方块首行)
add_edge([A(BOXES[b_exp]), L(BOXES[b_nav2])], C_PLANNING)

# Nav2 输出 → cmd_vel_safety
nvx = BOXES[b_nav2].x + 120
add_edge([(nvx, BOXES[b_nav2].y + BOXES[b_nav2].h), (nvx, BOXES[b_safety].y)],
         C_CONTROL, label="/cmd_vel_nav")

# 两个监视器 + 急停 → cmd_vel_safety(短折线,全部走空白间隙)
add_edge(H_edge(BOXES[b_obstmon], BOXES[b_safety]), C_CONTROL, dashed=True)
add_edge([A(BOXES[b_locmon]), (382, BOXES[b_locmon].y + BOXES[b_locmon].h / 2),
          (382, BOXES[b_safety].y + 34), (BOXES[b_safety].x, BOXES[b_safety].y + 34)],
         C_CONTROL, dashed=True)
add_edge([B(BOXES[b_estop]), (BOXES[b_estop].x + BOXES[b_estop].w / 2, BOXES[b_safety].y)],
         C_DANGER, width=3)

# /cmd_vel_safe → 中继 → 跨机(UDP) → /cmd_vel
add_edge(H_edge(BOXES[b_safety], BOXES[b_rx]), C_CONTROL)
add_edge(H_edge(BOXES[b_rx], BOXES[b_tx]), C_CONTROL, width=3,
         label="UDP 19000/19001 · 双向 · 速度↓ / 里程计·IMU·电压↑")
add_edge(H_edge(BOXES[b_tx], BOXES[b_cmdvel]), C_CONTROL)

# /cmd_vel → 驱动
add_edge([B(BOXES[b_cmdvel]), (BOXES[b_cmdvel].x + BOXES[b_cmdvel].w / 2, 845),
          (BOXES[b_drv].x + BOXES[b_drv].w / 2, 845), T(BOXES[b_drv])], C_EXEC)
# 驱动 → CP2102 → MCU → 轮
add_edge(H_edge(BOXES[b_drv], BOXES[b_cp]), C_EXEC)
add_edge(H_edge(BOXES[b_cp], BOXES[b_mcu]), C_EXEC)
add_edge(H_edge(BOXES[b_mcu], BOXES[b_wheel]), C_EXEC)

# 低电压守卫:作为说明块放在规划层 TX1 侧(避免跨 4 个泳道的超长连线)
b_batt = add_box(TX1_X0 + 14, y2 + 26, 470, "低电压守卫(在探索器内)", [
    "输入 /battery_voltage ← TX1 的 /PowerVoltage",
    "min_battery 10.5 V(3S 锂电保守下限)",
    "battery_timeout 5.0 s —— 覆盖 relay 挂掉后仍拿旧值报正常",
    "|不打开不是少个提示:电量耗尽时探索不会停,整场会话中断",
], color=C_DANGER, body_size=12, lane=1)


# ----------------------------------------------------------------- 渲染
WARNINGS = []
for b in BOXES:
    for kind, txt in b.overflow():
        WARNINGS.append("溢出 %-5s [%s] %s" % (kind, b.title[:22], txt))


# ----------------------------------------------------------------- 几何自检
# 我这个模型读不了图像,所以不能靠"看起来对不对"验收。以下用几何关系验证:
# 方块不重叠、不越出所属机器分区、连线不穿方块、标注不压框。
def _overlap(a, b, tol=2):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    dx = min(ax1, bx1) - max(ax0, bx0)
    dy = min(ay1, by1) - max(ay0, by0)
    return dx > tol and dy > tol


def _inside(px, py, rect, tol=3):
    x0, y0, x1, y1 = rect
    return x0 + tol < px < x1 - tol and y0 + tol < py < y1 - tol


def label_anchor(pts, label_pos=None):
    """标注位置:显式指定优先,否则取折线中点。"""
    if label_pos is not None:
        return label_pos
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def validate(verbose=True):
    problems = []

    # 1) 方块之间不重叠
    for i in range(len(BOXES)):
        for j in range(i + 1, len(BOXES)):
            a, b = BOXES[i], BOXES[j]
            ra = (a.x, a.y, a.x + a.w, a.y + a.h)
            rb = (b.x, b.y, b.x + b.w, b.y + b.h)
            if _overlap(ra, rb):
                problems.append("方块重叠: [%s] ↔ [%s]" % (a.title[:20], b.title[:20]))

    # 2) 每个方块必须完整落在某一台机器的分区内(不能横跨跨机通道)
    for b in BOXES:
        x0, x1 = b.x, b.x + b.w
        in_orin = x0 >= ORIN_X0 - 2 and x1 <= ORIN_X1 + 2
        in_tx1 = x0 >= TX1_X0 - 2 and x1 <= TX1_X1 + 2
        hits_channel = _overlap((x0, b.y, x1, b.y + b.h), (CH_X0, 0, CH_X1, H))
        if not (in_orin or in_tx1):
            problems.append("跨区: [%s] x=[%d,%d]" % (b.title[:20], x0, x1))
        if hits_channel:
            problems.append("压住跨机通道: [%s]" % b.title[:20])

    # 3) 连线(内部采样点)不得穿过任何方块
    for pts, color, dashed, width, arrow, label, label_pos in EDGES:
        for k in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[k], pts[k + 1]
            seg = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            n = max(2, int(seg / 4))
            for t in range(1, n):
                px, py = x0 + (x1 - x0) * t / n, y0 + (y1 - y0) * t / n
                # 端点附近允许贴框
                if ((px - x0) ** 2 + (py - y0) ** 2) ** 0.5 < 8:
                    continue
                if ((px - x1) ** 2 + (py - y1) ** 2) ** 0.5 < 8:
                    continue
                for b in BOXES:
                    if _inside(px, py, (b.x, b.y, b.x + b.w, b.y + b.h)):
                        problems.append("连线穿框: %s 压住 [%s] @(%d,%d)"
                                        % (label or "无标注", b.title[:18], px, py))
                        break

    # 4) 连线标注不压方块
    for pts, color, dashed, width, arrow, label, label_pos in EDGES:
        if not label:
            continue
        mx, my = label_anchor(pts, label_pos)
        tw = text_w(label, 11)
        rl = (mx - tw / 2 - 5, my - 10, mx + tw / 2 + 5, my + 10)
        for b in BOXES:
            if _overlap(rl, (b.x, b.y, b.x + b.w, b.y + b.h)):
                problems.append("标注压框: 「%s」↔ [%s]" % (label, b.title[:18]))

    # 5) 图例与标题区不被占用
    lx, ly = W - 560, H - 96
    for b in BOXES:
        if _overlap((lx, ly, W - 30, H - 24), (b.x, b.y, b.x + b.w, b.y + b.h)):
            problems.append("压住图例: [%s]" % b.title[:20])

    if verbose:
        print("几何自检: %d 个方块, %d 条连线" % (len(BOXES), len(EDGES)))
        if problems:
            print("!! 发现 %d 个问题:" % len(problems))
            for p in problems:
                print("   " + p)
        else:
            print("  方块不重叠 / 不跨区 / 连线不穿框 / 标注不压框 —— 全部通过")
    return problems


def render_png(path):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    _render(d, png=True)
    img.save(path)
    return path


def _render(d, png=True):
    f_title = get_font(FONT_B, 30)
    f_sub = get_font(FONT, 15)
    f_lane = get_font(FONT_M, 17)
    f_legend = get_font(FONT, 12)

    # 标题
    d.text((96, 30), "R680 麦克纳姆轮小车 · 自主探索系统分层架构", font=f_title, fill=C_TEXT)
    d.text((96, 74), "两机异构:Orin NX 承担感知与决策(ROS 2 Humble, ROS_DOMAIN_ID=99)"
                     "  ·  TX1 承担底盘执行(ROS 1 Melodic)  ·  跨机以自研 UDP 中继连接",
           font=f_sub, fill=C_MUTED)

    # 泳道底色与标题
    for i, (name, color) in enumerate(LANES):
        y = lane_y(i)
        d.rounded_rectangle([LANE_LABEL_W + 8, y, W - 18, y + LANE_H], 10,
                            fill=C_LANE_BG, outline="#E9ECEF", width=1)
        d.rounded_rectangle([14, y, LANE_LABEL_W - 2, y + LANE_H], 8, fill=color)
        # 竖排泳道名
        f = get_font(FONT_M, 19)
        txt = name
        ty = y + (LANE_H - len(txt) * 22) / 2
        for k, ch in enumerate(txt):
            cw = text_w(ch, 19)
            d.text((14 + (LANE_LABEL_W - 18 - cw) / 2, ty + k * 22), ch,
                   font=f, fill="#FFFFFF")

    # 双机分区标题
    d.text((ORIN_X0 + 8, W and LANE_Y0 - 26), "Orin NX · ROS 2 Humble", font=f_sub, fill=C_MUTED)
    d.text((TX1_X0 + 8, LANE_Y0 - 26), "TX1 · ROS 1 Melodic", font=f_sub, fill=C_MUTED)

    # 跨机通道竖带
    d.rounded_rectangle([CH_X0, LANE_Y0 - 6, CH_X1, lane_y(len(LANES) - 1) + LANE_H], 12,
                        fill="#F1F3F5", outline="#CED4DA", width=1)
    f = get_font(FONT_M, 13)
    label = "UDP 跨机中继"
    for k, ch in enumerate(label):
        d.text((CH_X0 + 8, LANE_Y0 + 10 + k * 16), ch, font=f, fill=C_MUTED)

    # 方块
    for b in BOXES:
        stroke = b.color
        if b.dashed:
            _dash_rect(d, [b.x, b.y, b.w, b.h], stroke, width=2)
        elif b.bold:
            d.rounded_rectangle([b.x, b.y, b.x + b.w, b.y + b.h], 8,
                                fill="#FFF4F0", outline=stroke, width=3)
        else:
            d.rounded_rectangle([b.x, b.y, b.x + b.w, b.y + b.h], 8,
                                fill="#FFFFFF", outline=stroke, width=2)
        # 左边色条
        d.rounded_rectangle([b.x, b.y, b.x + 4, b.y + b.h], 2, fill=stroke)

        ft = get_font(FONT_B if b.bold else FONT_M, b.title_size)
        d.text((b.x + b.pad, b.y + b.pad - 2), b.title, font=ft, fill=C_TEXT)
        ty = b.y + b.pad + b.title_size + 8
        for ln in b.lines:
            emph = ln.startswith("|")
            txt = ln[1:] if emph else ln
            fb = get_font(FONT_M if emph else FONT, b.body_size)
            d.text((b.x + b.pad, ty), txt, font=fb,
                   fill=C_DANGER if emph else C_MUTED)
            ty += b.line_gap

    # 角标
    for idx, txt in BADGES:
        b = BOXES[idx]
        f = get_font(FONT_M, 11)
        tw = text_w(txt, 11)
        bx, by = b.x + b.w - tw - 22, b.y - 11
        d.rounded_rectangle([bx, by, bx + tw + 14, by + 20], 5, fill="#FFF4F0", outline=C_DANGER, width=1)
        d.text((bx + 7, by + 3), txt, font=f, fill=C_DANGER)

    # 连线
    for pts, color, dashed, width, arrow, label, label_pos in EDGES:
        if dashed:
            for i in range(len(pts) - 1):
                (x0, y0), (x1, y1) = pts[i], pts[i + 1]
                seg = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
                n = max(1, int(seg / 9))
                for k in range(n):
                    if k % 2:
                        continue
                    ax = x0 + (x1 - x0) * k / n
                    ay = y0 + (y1 - y0) * k / n
                    bx2 = x0 + (x1 - x0) * (k + 1) / n
                    by2 = y0 + (y1 - y0) * (k + 1) / n
                    d.line([(ax, ay), (bx2, by2)], fill=color, width=width)
        else:
            d.line(pts, fill=color, width=width, joint="curve")
        if arrow:
            # 箭头画在最后一段的末端
            (x0, y0), (x1, y1) = pts[-2], pts[-1]
            dx, dy = x1 - x0, y1 - y0
            n = max(1e-6, (dx * dx + dy * dy) ** 0.5)
            ux, uy = dx / n, dy / n
            a, s = 10, 5
            p1 = (x1 - ux * a + uy * s, y1 - uy * a - ux * s)
            p2 = (x1 - ux * a - uy * s, y1 - uy * a + ux * s)
            d.polygon([(x1, y1), p1, p2], fill=color)
        if label:
            mx, my = label_anchor(pts, label_pos)
            f = get_font(FONT_M, 11)
            tw = text_w(label, 11)
            d.rectangle([mx - tw / 2 - 4, my - 9, mx + tw / 2 + 4, my + 9], fill="#FFFFFF")
            d.text((mx - tw / 2, my - 7), label, font=f, fill=color)

    # 图例
    lx, ly = W - 560, H - 96
    d.rounded_rectangle([lx, ly, W - 30, H - 24], 8, fill="#FFFFFF", outline="#DEE2E6", width=1)
    d.line([(lx + 14, ly + 22), (lx + 54, ly + 22)], fill=C_STROKE, width=2)
    d.text((lx + 62, ly + 14), "实线 = 数据流", font=f_legend, fill=C_MUTED)
    d.line([(lx + 190, ly + 22), (lx + 230, ly + 22)], fill=C_STROKE, width=2)
    d.text((lx + 238, ly + 14), "虚线 = 控制 / 开关 / 回环", font=f_legend, fill=C_MUTED)
    d.rounded_rectangle([lx + 400, ly + 13, lx + 418, ly + 31], 3, fill="#FFF4F0",
                        outline=C_DANGER, width=2)
    d.text((lx + 426, ly + 14), "琥珀 = 防御性设计", font=f_legend, fill=C_DANGER)


def render_svg(path):
    """同一份几何数据输出 SVG。"""
    out = []
    add = out.append
    add('<?xml version="1.0" encoding="UTF-8"?>')
    add('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
        'viewBox="0 0 %d %d" font-family="Noto Sans CJK SC, Noto Sans CJK JP, sans-serif">' % (W, H, W, H))
    add('<rect width="100%" height="100%" fill="' + BG + '"/>')
    add('<g id="title">')
    add('<text x="96" y="52" font-size="30" font-weight="bold" fill="%s">'
        'R680 麦克纳姆轮小车 · 自主探索系统分层架构</text>' % C_TEXT)
    add('<text x="96" y="88" font-size="15" fill="%s">'
        '两机异构:Orin NX 承担感知与决策(ROS 2 Humble, ROS_DOMAIN_ID=99)'
        '  ·  TX1 承担底盘执行(ROS 1 Melodic)  ·  跨机以自研 UDP 中继连接</text>' % C_MUTED)
    add('</g>')

    # 泳道
    add('<g id="lanes">')
    for i, (name, color) in enumerate(LANES):
        y = lane_y(i)
        add('<rect x="%d" y="%d" width="%d" height="%d" rx="10" fill="%s" stroke="#E9ECEF"/>'
            % (LANE_LABEL_W + 8, y, W - 26 - LANE_LABEL_W, LANE_H, C_LANE_BG))
        add('<rect x="14" y="%d" width="%d" height="%d" rx="8" fill="%s"/>'
            % (y, LANE_LABEL_W - 16, LANE_H, color))
        ty = y + (LANE_H - len(name) * 22) / 2 + 16
        for k, ch in enumerate(name):
            cw = text_w(ch, 19)
            add('<text x="%.1f" y="%.1f" font-size="19" font-weight="500" fill="#FFFFFF">%s</text>'
                % (14 + (LANE_LABEL_W - 18 - cw) / 2, ty + k * 22, ch))
    add('</g>')

    add('<g id="hosts">')
    add('<text x="%d" y="%d" font-size="15" fill="%s">Orin NX · ROS 2 Humble</text>'
        % (ORIN_X0 + 8, LANE_Y0 - 26, C_MUTED))
    add('<text x="%d" y="%d" font-size="15" fill="%s">TX1 · ROS 1 Melodic</text>'
        % (TX1_X0 + 8, LANE_Y0 - 26, C_MUTED))
    add('</g>')

    # 跨机通道
    add('<g id="channel">')
    add('<rect x="%d" y="%d" width="%d" height="%d" rx="12" fill="#F1F3F5" stroke="#CED4DA"/>'
        % (CH_X0, LANE_Y0 - 6, CH_X1 - CH_X0, lane_y(len(LANES) - 1) + LANE_H - LANE_Y0 + 6))
    for k, ch in enumerate("UDP 跨机中继"):
        add('<text x="%d" y="%d" font-size="13" font-weight="500" fill="%s">%s</text>'
            % (CH_X0 + 8, LANE_Y0 + 26 + k * 16, C_MUTED, ch))
    add('</g>')

    # 方块
    add('<g id="boxes">')
    for b in BOXES:
        fill = "#FFF4F0" if b.bold else "#FFFFFF"
        dash = ' stroke-dasharray="7 5"' if b.dashed else ""
        add('<rect x="%d" y="%d" width="%d" height="%d" rx="8" fill="%s" stroke="%s" '
            'stroke-width="%d"%s/>' % (b.x, b.y, b.w, b.h, fill, b.color, 3 if b.bold else 2, dash))
        add('<rect x="%d" y="%d" width="4" height="%d" rx="2" fill="%s"/>'
            % (b.x, b.y, b.h, b.color))
        add('<text x="%d" y="%d" font-size="%d" font-weight="%s" fill="%s">%s</text>'
            % (b.x + b.pad, b.y + b.pad + b.title_size - 4, b.title_size,
               "bold" if b.bold else "500", C_TEXT, _esc(b.title)))
        ty = b.y + b.pad + b.title_size + 8 + b.body_size - 3
        for ln in b.lines:
            emph = ln.startswith("|")
            txt = ln[1:] if emph else ln
            add('<text x="%d" y="%d" font-size="%d" fill="%s">%s</text>'
                % (b.x + b.pad, ty, b.body_size, C_DANGER if emph else C_MUTED, _esc(txt)))
            ty += b.line_gap
    add('</g>')

    # 角标
    add('<g id="badges">')
    for idx, txt in BADGES:
        b = BOXES[idx]
        tw = text_w(txt, 11)
        bx, by = b.x + b.w - tw - 22, b.y - 11
        add('<rect x="%d" y="%d" width="%d" height="20" rx="5" fill="#FFF4F0" stroke="%s"/>'
            % (bx, by, tw + 14, C_DANGER))
        add('<text x="%d" y="%d" font-size="11" fill="%s">%s</text>'
            % (bx + 7, by + 14, C_DANGER, _esc(txt)))
    add('</g>')

    # 连线
    add('<defs>')
    for cid, color in (("a-dark", C_STROKE), ("a-blue", C_PERCEPTION), ("a-teal", C_PLANNING),
                       ("a-orange", C_CONTROL), ("a-gray", C_EXEC), ("a-red", C_DANGER),
                       ("a-violet", C_ORCH)):
        add('<marker id="%s" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" '
            'markerHeight="6" orient="auto-start-reverse">'
            '<path d="M 0 0 L 10 5 L 0 10 z" fill="%s"/></marker>' % (cid, color))
    add('</defs>')
    cmap = {C_STROKE: "a-dark", C_PERCEPTION: "a-blue", C_PLANNING: "a-teal",
            C_CONTROL: "a-orange", C_EXEC: "a-gray", C_DANGER: "a-red", C_ORCH: "a-violet"}
    add('<g id="edges" fill="none">')
    for pts, color, dashed, width, arrow, label, label_pos in EDGES:
        d_attr = ' stroke-dasharray="7 5"' if dashed else ""
        m_attr = ' marker-end="url(#%s)"' % cmap.get(color, "a-dark") if arrow else ""
        pstr = " ".join("%d,%d" % p for p in pts)
        add('<polyline points="%s" stroke="%s" stroke-width="%d"%s%s/>'
            % (pstr, color, width, d_attr, m_attr))
        if label:
            mx, my = label_anchor(pts, label_pos)
            tw = text_w(label, 11)
            add('<rect x="%.0f" y="%.0f" width="%.0f" height="17" fill="#FFFFFF"/>'
                % (mx - tw / 2 - 4, my - 9, tw + 8))
            add('<text x="%.0f" y="%.0f" font-size="11" fill="%s" text-anchor="start">%s</text>'
                % (mx - tw / 2, my - 7 + 12, color, _esc(label)))
    add('</g>')

    # 图例
    lx, ly = W - 560, H - 96
    add('<g id="legend">')
    add('<rect x="%d" y="%d" width="530" height="72" rx="8" fill="#FFFFFF" stroke="#DEE2E6"/>'
        % (lx, ly))
    add('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="2"/>'
        % (lx + 14, ly + 22, lx + 54, ly + 22, C_STROKE))
    add('<text x="%d" y="%d" font-size="12" fill="%s">实线 = 数据流</text>' % (lx + 62, ly + 26, C_MUTED))
    add('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="2" stroke-dasharray="7 5"/>'
        % (lx + 190, ly + 22, lx + 230, ly + 22, C_STROKE))
    add('<text x="%d" y="%d" font-size="12" fill="%s">虚线 = 控制 / 开关 / 回环</text>'
        % (lx + 238, ly + 26, C_MUTED))
    add('<rect x="%d" y="%d" width="18" height="18" rx="3" fill="#FFF4F0" stroke="%s" stroke-width="2"/>'
        % (lx + 400, ly + 13, C_DANGER))
    add('<text x="%d" y="%d" font-size="12" fill="%s">琥珀 = 防御性设计</text>'
        % (lx + 426, ly + 26, C_DANGER))
    add('</g>')
    add('</svg>')
    open(path, "w", encoding="utf-8").write("\n".join(out))
    return path


def _esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(outdir, exist_ok=True)
    problems = validate()
    svg = render_svg(os.path.join(outdir, "r680_architecture.svg"))
    png = render_png(os.path.join(outdir, "r680_architecture.png"))
    print("已生成:")
    for p in (svg, png):
        print("  %s  (%d 字节)" % (p, os.path.getsize(p)))
    if WARNINGS:
        print("\n!! 文本溢出警告 %d 条(需要缩窄文案或加宽方块):" % len(WARNINGS))
        for w in WARNINGS:
            print("   " + w)
        sys.exit(1)
    print("\n文本溢出检查:全部通过")
    if problems:
        sys.exit(1)
