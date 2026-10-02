#!/usr/bin/env python3
"""对比 v1（分位数+固定容差）与 v2（模态自适应）的反投影深度估计。

三组测试
--------
T1 真实数据：S01 透明瓶（真值 1.5 m，33 帧）
T2 真实数据：ArUco 标定板（真值 1.576 m）
T3 合成受控：前景/背景深度间隔扫描 —— 定位 v1 固定容差的失效边界

用法
----
    source /opt/ros/humble/setup.bash
    source ~/VIO/semantic_ws/install/setup.bash
    python3 ~/VIO/tools/compare_v1_v2.py
"""
import csv
import math
import os
import sqlite3
import statistics
import sys
from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backprojection_v2 import robust_depth_modesek  # noqa: E402

FX = FY = 598.757
CX, CY = 326.344, 250.245
BAG = "/home/jetson/VIO/data/bags/_s01"
BOARD = "/home/jetson/VIO/data/board_150"


# ---------------- v1：复刻生产代码 ----------------
def v1_depth(depth, bbox, roi_ratio=0.6, percentile=20.0, tol=0.35,
             min_depth=0.2, max_depth=12.0, min_valid=12, min_cluster=10):
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in bbox]
    x0 = max(0., min(w - 1., x0)); y0 = max(0., min(h - 1., y0))
    x1 = max(x0 + 1., min(float(w), x1)); y1 = max(y0 + 1., min(float(h), y1))
    mx, my = (x0 + x1) * .5, (y0 + y1) * .5
    hw, hh = max(1., (x1 - x0) * roi_ratio * .5), max(1., (y1 - y0) * roi_ratio * .5)
    rx0 = int(max(0, math.floor(mx - hw))); rx1 = int(min(w, math.ceil(mx + hw)))
    ry0 = int(max(0, math.floor(my - hh))); ry1 = int(min(h, math.ceil(my + hh)))
    roi = depth[ry0:ry1, rx0:rx1]
    ok = np.isfinite(roi) & (roi > min_depth) & (roi < max_depth)
    if int(ok.sum()) < min_valid:
        return None
    z0 = float(np.percentile(roi[ok], percentile))
    bx0 = int(max(0, math.floor(x0))); bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0))); by1 = int(min(h, math.ceil(y1)))
    patch = depth[by0:by1, bx0:bx1]
    vd = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    cl = vd & (np.abs(patch - z0) <= tol)
    if int(cl.sum()) < min_cluster:
        cl = vd
    zs = patch[cl].astype(np.float64)
    if zs.size < min_valid:
        return None
    return float(zs.mean()), z0, int(cl.sum())


def v2_depth(depth, bbox):
    r = robust_depth_modesek(depth, bbox, min_depth=0.2, max_depth=12.0)
    if r is None:
        return None
    return r.z, r.mode_z, r


# ---------------- 读 S01 bag ----------------
def load_bottle():
    db = BAG
    if os.path.isdir(db):
        c = [f for f in sorted(os.listdir(db)) if f.endswith(".db3")]
        db = os.path.join(db, c[0])
    conn = sqlite3.connect(db)
    tmap = {int(i): (str(n), str(t)) for i, n, t in conn.execute("SELECT id,name,type FROM topics")}
    bridge = CvBridge()
    dh, info, out = deque(maxlen=8), None, []
    for tid, _ts, data in conn.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
        e = tmap.get(int(tid))
        if e is None:
            continue
        tp, ts = e
        m = deserialize_message(bytes(data), get_message(ts))
        if tp.endswith("camera_info"):
            info = m; continue
        if tp == "/camera/camera/aligned_depth_to_color/image_raw":
            raw = bridge.imgmsg_to_cv2(m, "passthrough")
            dh.append((int(m.header.stamp.sec) * 10**9 + int(m.header.stamp.nanosec),
                       raw.astype(np.float32) * 0.001))
            continue
        if tp != "/semantics/objects" or info is None or not dh:
            continue
        objs = [o for o in m.objects if o.class_name == "bottle"]
        if not objs:
            continue
        ot = int(m.header.stamp.sec) * 10**9 + int(m.header.stamp.nanosec)
        _, depth = min(dh, key=lambda x: abs(x[0] - ot))
        for o in objs:
            out.append((depth.copy(), tuple(float(v) for v in o.bbox)))
    return out


# ---------------- T3 合成场景 ----------------
def make_scene(z_obj, gap, fg_frac, seed=0, shape=(80, 60)):
    """模拟检测框：中心椭圆为物体，四角为背景，含随距离增长的噪声。"""
    rng = np.random.default_rng(seed)
    h, w = shape
    yy, xx = np.mgrid[0:h, 0:w]
    cy, cx = (h - 1) / 2, (w - 1) / 2
    # 椭圆面积占比 = fg_frac  → 半轴比例 a*b/(h*w) = fg_frac
    r = math.sqrt(fg_frac / math.pi)
    a, b = r * w, r * h
    obj = (((xx - cx) / a) ** 2 + ((yy - cy) / b) ** 2) <= 1.0
    sig = 0.002 * z_obj ** 2                     # 主动立体：误差 ∝ z²
    d = np.where(obj, z_obj, z_obj + gap).astype(np.float64)
    d += rng.normal(0.0, max(1e-4, sig), d.shape)
    # 物体边缘制造少量无效像素（真实传感器在边缘会失效）
    mask = rng.random(d.shape) < 0.02
    d[mask] = 0.0
    return d.astype(np.float32), (0.0, 0.0, float(w), float(h))


def main():
    print("=" * 78)
    print("T1  真实数据：S01 透明瓶（真值 1.5 m）")
    print("=" * 78)
    data = load_bottle()
    print(f"  加载 {len(data)} 个检测")
    e1, e2, z1, z2, m2 = [], [], [], [], []
    for depth, bbox in data:
        a = v1_depth(depth, bbox)
        b = v2_depth(depth, bbox)
        if a:
            e1.append(abs(a[0] - 1.5) * 1000); z1.append(a[0])
        if b:
            e2.append(abs(b[0] - 1.5) * 1000); z2.append(b[0]); m2.append(b[2])
    print(f"  v1（分位数+固定容差）: MAE-z {statistics.fmean(e1):7.2f} mm   "
          f"σ {statistics.pstdev(z1)*1000:5.2f} mm   n={len(e1)}")
    print(f"  v2（模态自适应）      : MAE-z {statistics.fmean(e2):7.2f} mm   "
          f"σ {statistics.pstdev(z2)*1000:5.2f} mm   n={len(e2)}")
    if m2:
        nm = [r.n_modes for r in m2]
        print(f"  v2 模态数分布: {sorted(set(nm))}  "
              f"(众数 {max(set(nm), key=nm.count)})")
        print(f"  v2 平均等效容差: {statistics.fmean(r.tolerance for r in m2)*1000:.0f} mm "
              f"(v1 固定 350 mm)")
        print(f"  v2 平均置信度: {statistics.fmean(r.confidence for r in m2):.2f}")

    print()
    print("=" * 78)
    print("T2  真实数据：ArUco 标定板（真值 1.576 m）")
    print("=" * 78)
    depth = np.load(f"{BOARD}_depth.npy")
    valid = (depth > 0.2) & (depth < 12) & np.isfinite(depth)
    h, w = depth.shape
    z0 = float(depth[h // 2, w // 2])
    m = (valid & (np.abs(depth - z0) < 0.15)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    cl = int(lab[h // 2, w // 2]) or 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab == cl)
    bbox = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))
    gt = float(np.median(depth[lab == cl]))
    a = v1_depth(depth, bbox)
    b = v2_depth(depth, bbox)
    print(f"  真值（板面中位）  : {gt:.4f} m")
    if a:
        print(f"  v1: z {a[0]:.4f} m  误差 {abs(a[0]-gt)*1000:5.1f} mm")
    if b:
        print(f"  v2: z {b[0]:.4f} m  误差 {abs(b[0]-gt)*1000:5.1f} mm  "
              f"| 模态数 {b[2].n_modes}  容差 {b[2].tolerance*1000:.0f} mm  "
              f"置信度 {b[2].confidence:.2f}  flags={b[2].flags}")

    print()
    print("=" * 78)
    print("T3  合成受控扫描：前景/背景深度间隔 → 两种方法何时失效")
    print("=" * 78)
    print("  物体占框比例固定 60%，噪声 σ=0.002·z²，同深度带无效像素 2%")
    print()
    print(f"{'z_obj':>7} {'gap':>7} | {'v1 误差/mm':>11} {'v2 误差/mm':>11} | "
          f"{'v2 模态':>7} {'v2 容差/mm':>10} {'v2 置信':>7}")
    print("-" * 78)
    rows = []
    for z_obj in (1.0, 1.5, 3.0):
        for gap in (0.05, 0.10, 0.20, 0.35, 0.50, 1.00, 2.00):
            d, bx = make_scene(z_obj, gap, 0.60)
            a = v1_depth(d, bx)
            b = v2_depth(d, bx)
            e1 = abs(a[0] - z_obj) * 1000 if a else float("nan")
            e2 = abs(b[0] - z_obj) * 1000 if b else float("nan")
            nm = b[2].n_modes if b else -1
            to = b[2].tolerance * 1000 if b else float("nan")
            cf = b[2].confidence if b else float("nan")
            rows.append((z_obj, gap, e1, e2))
            print(f"{z_obj:>6.1f}m {gap:>6.2f}m | {e1:>11.1f} {e2:>11.1f} | "
                  f"{nm:>7} {to:>10.0f} {cf:>7.2f}")
    print()
    bad1 = sum(1 for _, _, a, b in rows if a > 50)
    bad2 = sum(1 for _, _, a, b in rows if b > 50)
    print(f"  误差 > 50 mm 的工况数:  v1 {bad1}/{len(rows)}   v2 {bad2}/{len(rows)}")


if __name__ == "__main__":
    main()
