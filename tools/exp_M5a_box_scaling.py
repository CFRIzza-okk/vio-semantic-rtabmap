#!/usr/bin/env python3
"""M5-a：同一物体、不同检测框大小下，判别量 r 的稳定性。

要回答的问题
------------
原稿选 r =（中心 60% 有效深度占比）/（整框有效深度占比）作为材质判据，
理由之一是「自带归一化，不受物体大小、距离、框面积的直接影响」。

但当前不透明类只有 1 个样本（ArUco 标定板），且其框面积是瓶子的 19 倍。
**若 r 的分离其实来自框面积差异，而不是材质，这条理由就不成立。**

本实验在严格配对的条件下检验：同一帧、同一物体、同一深度图，
只把检测框以中心为基准等比例缩放，观察 r 是否随 s 漂移。

设计
----
s ∈ {0.6, 0.8, 1.0, 1.25, 1.5}，1.0 为原始框。
中心 60% ROI 随框同步缩放（它是「框的」60%，不是固定像素区）。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/VIO/semantic_ws/install/setup.bash
    python3 ~/VIO/tools/exp_M5a_box_scaling.py
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

BAG = "/home/jetson/VIO/data/bags/_s01"
BOARD = "/home/jetson/VIO/data/board_150"
SCALES = (0.6, 0.8, 1.0, 1.25, 1.5)
OUT = "/home/jetson/VIO/data/exp_M5a"
MIN_D, MAX_D = 0.2, 12.0


def metrics(depth, bbox, roi_ratio=0.6):
    """整框有效占比、中心 ROI 有效占比、r。"""
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in bbox]
    bx0 = int(max(0, math.floor(x0))); bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0))); by1 = int(min(h, math.ceil(y1)))
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None
    patch = depth[by0:by1, bx0:bx1]
    v = np.isfinite(patch) & (patch > MIN_D) & (patch < MAX_D)
    if int(v.sum()) < 12:
        return None
    v_box = float(v.mean())
    mx, my = (bx0 + bx1) * .5, (by0 + by1) * .5
    hw = max(1.0, (bx1 - bx0) * roi_ratio * .5)
    hh = max(1.0, (by1 - by0) * roi_ratio * .5)
    rx0 = int(max(0, math.floor(mx - hw))); rx1 = int(min(w, math.ceil(mx + hw)))
    ry0 = int(max(0, math.floor(my - hh))); ry1 = int(min(h, math.ceil(my + hh)))
    roi = depth[ry0:ry1, rx0:rx1]
    if roi.size < 4:
        return None
    rv = np.isfinite(roi) & (roi > MIN_D) & (roi < MAX_D)
    v_roi = float(rv.mean())
    if v_box < 1e-9:
        return None
    return {"box_px": int(patch.size), "roi_px": int(roi.size),
            "valid_box": v_box, "valid_roi": v_roi, "r": v_roi / v_box}


def scale_box(bbox, s):
    x0, y0, x1, y1 = bbox
    cx, cy = (x0 + x1) * .5, (y0 + y1) * .5
    hw, hh = (x1 - x0) * .5 * s, (y1 - y0) * .5 * s
    return (cx - hw, cy - hh, cx + hw, cy + hh)


def load_bottle():
    db = BAG
    if os.path.isdir(db):
        c = [f for f in sorted(os.listdir(db)) if f.endswith(".db3")]
        db = os.path.join(db, c[0])
    conn = sqlite3.connect(db)
    tmap = {int(i): (str(n), str(t)) for i, n, t in
            conn.execute("SELECT id,name,type FROM topics")}
    bridge = CvBridge()
    dh, info, out = deque(maxlen=8), None, []
    for tid, _ts, data in conn.execute(
            "SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
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
            out.append((depth, tuple(float(v) for v in o.bbox)))
    return out


def load_board():
    depth = np.load(f"{BOARD}_depth.npy")
    valid = (depth > MIN_D) & (depth < MAX_D) & np.isfinite(depth)
    h, w = depth.shape
    z0 = float(depth[h // 2, w // 2])
    m = (valid & (np.abs(depth - z0) < 0.15)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    cl = int(lab[h // 2, w // 2]) or 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab == cl)
    return depth, (float(xs.min()), float(ys.min()),
                   float(xs.max() + 1), float(ys.max() + 1))


def run_group(name, items):
    """items: [(depth, bbox), ...]"""
    rows = []
    for s in SCALES:
        per = []
        for depth, bbox in items:
            m = metrics(depth, scale_box(bbox, s))
            if m:
                per.append(m)
        if not per:
            continue
        rows.append({
            "group": name, "s": s, "n": len(per),
            "box_px": statistics.fmean(x["box_px"] for x in per),
            "valid_box": statistics.fmean(x["valid_box"] for x in per),
            "valid_roi": statistics.fmean(x["valid_roi"] for x in per),
            "r": statistics.fmean(x["r"] for x in per),
            "r_std": statistics.pstdev([x["r"] for x in per]) if len(per) > 1 else 0.0,
        })
    return rows


def main():
    os.makedirs(OUT, exist_ok=True)
    print("=" * 78)
    print("M5-a  框缩放稳定性：r 是否随框面积漂移")
    print("=" * 78)
    print(f"  缩放档位 s = {SCALES}   （1.0 = 原始框）")
    print(f"  中心 60% ROI 随框同步缩放")
    print()

    bottle = load_bottle()
    print(f"  透明瓶：加载 {len(bottle)} 个检测")
    depth_b, bbox_b = load_board()
    print(f"  标定板：1 帧，框 {[round(v) for v in bbox_b]}")

    all_rows = []
    for name, items in (("透明塑料瓶", bottle), ("不透明标定板", [(depth_b, bbox_b)])):
        rs = run_group(name, items)
        all_rows += rs
        print()
        print(f"  ── {name} ──")
        print(f"  {'s':>6} {'框面积/px':>10} {'整框有效率':>10} "
              f"{'中心有效率':>10} {'r':>8} {'r 标准差':>9}")
        for x in rs:
            print(f"  {x['s']:>6.2f} {x['box_px']:>10.0f} {x['valid_box']*100:>9.1f}% "
                  f"{x['valid_roi']*100:>9.1f}% {x['r']:>8.3f} {x['r_std']:>9.4f}")
        rr = [x["r"] for x in rs]
        print(f"  → r 极差 {max(rr)-min(rr):.3f}   "
              f"相对变化 {(max(rr)-min(rr))/statistics.fmean(rr)*100:.1f}%")

    # 两类是否仍然分离
    print()
    print("=" * 78)
    print("两类分离度随 s 的变化（判据：透明 r < 1.0 < 不透明 r，阈值带 0.7~1.0）")
    print("=" * 78)
    bl = {x["s"]: x["r"] for x in all_rows if x["group"] == "透明塑料瓶"}
    bd = {x["s"]: x["r"] for x in all_rows if x["group"] == "不透明标定板"}
    print(f"  {'s':>6} {'透明 r':>9} {'不透明 r':>10} {'间隔':>9} {'是否分离':>9}")
    for s in SCALES:
        if s in bl and s in bd:
            sep = bd[s] - bl[s]
            ok = "是" if (bl[s] < 1.0 < bd[s]) else "否"
            print(f"  {s:>6.2f} {bl[s]:>9.3f} {bd[s]:>10.3f} {sep:>9.3f} {ok:>9}")

    csv_path = os.path.join(OUT, "M5a_box_scaling.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["group", "s", "n", "box_px",
                                          "valid_box", "valid_roi", "r", "r_std"])
        w.writeheader(); w.writerows(all_rows)
    print(f"\nCSV: {csv_path}")


if __name__ == "__main__":
    main()
