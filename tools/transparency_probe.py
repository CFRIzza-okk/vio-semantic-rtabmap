#!/usr/bin/env python3
"""量化「红外可穿透 / 不可穿透」两类目标的深度特征差异，为分类器定阈值。

思路
----
不需要额外传感器：检测框内的**有效深度占比**本身就是判别量。
- 不透明物体：整个可见表面都返回深度 → 有效占比高
- 透明物体：红外穿透或折射，中心区域大量无效 → 有效占比低，
  且**中心 ROI 的占比比整框更低**（因为中心是最透明的部分）

本脚本对同一份数据分别统计两类目标，给出可用的判别特征与阈值参考。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/VIO/semantic_ws/install/setup.bash
    python3 ~/VIO/tools/transparency_probe.py \
        --bottle-bag ~/VIO/data/bags/_s01 \
        --board-prefix ~/VIO/data/board_150 \
        --out ~/VIO/data/transparency_probe
"""
import argparse
import csv
import json
import math
import os
import sqlite3
import statistics
from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def features(depth, bbox, min_depth=0.2, max_depth=12.0, roi_ratio=0.6):
    """返回单个检测框的深度可判性特征。"""
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in bbox]
    bx0 = int(max(0, math.floor(x0))); bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0))); by1 = int(min(h, math.ceil(y1)))
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None
    patch = depth[by0:by1, bx0:bx1]
    valid = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    n_valid = int(valid.sum())
    if n_valid < 12:
        return None
    vals = patch[valid]

    mx, my = (bx0 + bx1) * 0.5, (by0 + by1) * 0.5
    hw = max(1.0, (bx1 - bx0) * roi_ratio * 0.5)
    hh = max(1.0, (by1 - by0) * roi_ratio * 0.5)
    rx0 = int(max(0, math.floor(mx - hw))); rx1 = int(min(w, math.ceil(mx + hw)))
    ry0 = int(max(0, math.floor(my - hh))); ry1 = int(min(h, math.ceil(my + hh)))
    roi = depth[ry0:ry1, rx0:rx1]
    rvalid = int((np.isfinite(roi) & (roi > min_depth) & (roi < max_depth)).sum())

    med = float(np.median(vals))
    # 前景占比：与中位数同深度带（±0.35 m）的像素占有效像素的比例
    fg = int((valid & (np.abs(patch - med) <= 0.35)).sum())
    return {
        "box_px": int(patch.size),
        "valid_px": n_valid,
        "valid_ratio": n_valid / patch.size,
        "roi_px": int(roi.size),
        "roi_valid_px": rvalid,
        "roi_valid_ratio": rvalid / max(1, roi.size),
        "valid_ratio_ratio": (rvalid / max(1, roi.size)) / max(1e-9, n_valid / patch.size),
        "fg_frac": fg / max(1, n_valid),
        "z_median": med,
        "z_spread_p2p98": float(np.percentile(vals, 98) - np.percentile(vals, 2)),
    }


def load_bag_bottles(bag, class_name="bottle", z_lo=1.0, z_hi=2.0):
    db = bag
    if os.path.isdir(db):
        c = [f for f in sorted(os.listdir(db)) if f.endswith(".db3")]
        if not c:
            return []
        db = os.path.join(db, c[0])
    conn = sqlite3.connect(db)
    tmap = {int(i): (str(n), str(t)) for i, n, t in conn.execute("SELECT id,name,type FROM topics")}
    bridge = CvBridge()
    dh = deque(maxlen=8)
    info = None
    out = []
    for tid, _ts, data in conn.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
        e = tmap.get(int(tid))
        if e is None:
            continue
        tp, ts = e
        m = deserialize_message(bytes(data), get_message(ts))
        if tp.endswith("camera_info"):
            info = m
            continue
        if tp == "/camera/camera/aligned_depth_to_color/image_raw":
            raw = bridge.imgmsg_to_cv2(m, "passthrough")
            dh.append((int(m.header.stamp.sec) * 10**9 + int(m.header.stamp.nanosec),
                       raw.astype(np.float32) * 0.001))
            continue
        if tp != "/semantics/objects" or info is None or not dh:
            continue
        objs = [o for o in m.objects if o.class_name == class_name]
        if not objs:
            continue
        ot = int(m.header.stamp.sec) * 10**9 + int(m.header.stamp.nanosec)
        _, depth = min(dh, key=lambda x: abs(x[0] - ot))
        for o in objs:
            # 只取站在 1~2 m 的那一个瓶子（排除画面里其他瓶子）
            f = features(depth, tuple(float(v) for v in o.bbox))
            if f and z_lo < f["z_median"] < z_hi:
                out.append(f)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bottle-bag", default="/home/jetson/VIO/data/bags/_s01")
    ap.add_argument("--board-prefix", default="/home/jetson/VIO/data/board_150")
    ap.add_argument("--out", default="/home/jetson/VIO/data/transparency_probe")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    bottles = load_bag_bottles(args.bottle_bag)
    board = []
    if os.path.exists(f"{args.board_prefix}_depth.npy"):
        depth = np.load(f"{args.board_prefix}_depth.npy")
        # 用深度连通域得到标定板外接框，当作"不透明目标"的检测框
        valid = (depth > 0.2) & (depth < 12.0) & np.isfinite(depth)
        h, w = depth.shape
        z0 = float(depth[h // 2, w // 2])
        m = (valid & (np.abs(depth - z0) < 0.15)).astype(np.uint8)
        n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
        cl = int(lab[h // 2, w // 2]) or 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        ys, xs = np.nonzero(lab == cl)
        if xs.size:
            f = features(depth, (xs.min(), ys.min(), xs.max() + 1, ys.max() + 1))
            if f:
                board.append(f)

    def summarize(name, rows):
        if not rows:
            print(f"\n{name}: 无样本")
            return None
        def col(k):
            return [float(r[k]) for r in rows]
        s = {
            "group": name, "n": len(rows),
            "valid_ratio_mean": statistics.fmean(col("valid_ratio")),
            "roi_valid_ratio_mean": statistics.fmean(col("roi_valid_ratio")),
            "valid_ratio_ratio_mean": statistics.fmean(col("valid_ratio_ratio")),
            "fg_frac_mean": statistics.fmean(col("fg_frac")),
            "box_px_mean": statistics.fmean(col("box_px")),
        }
        print(f"\n===== {name} (n={len(rows)}) =====")
        print(f"  整框有效深度占比   : {s['valid_ratio_mean']*100:6.1f} %"
              f"   (范围 {min(col('valid_ratio'))*100:.1f}~{max(col('valid_ratio'))*100:.1f})")
        print(f"  中心60%有效占比    : {s['roi_valid_ratio_mean']*100:6.1f} %"
              f"   (范围 {min(col('roi_valid_ratio'))*100:.1f}~{max(col('roi_valid_ratio'))*100:.1f})")
        print(f"  中心/整框 之比     : {s['valid_ratio_ratio_mean']:6.2f}")
        print(f"  前景像素占比       : {s['fg_frac_mean']*100:6.1f} %")
        print(f"  框面积             : {s['box_px_mean']:6.0f} px")
        return s

    print("=" * 70)
    print("红外可穿透 vs 不可穿透 —— 深度特征对照")
    print("=" * 70)
    sb = summarize("透明塑料瓶（红外可穿透）", bottles)
    so = summarize("不透明标定板（不可穿透）", board)

    if sb and so:
        print("\n" + "=" * 70)
        print("判别特征与建议阈值")
        print("=" * 70)
        for k, label, better in (("valid_ratio", "整框有效深度占比", "低=透明"),
                                 ("roi_valid_ratio", "中心60%有效占比", "低=透明"),
                                 ("valid_ratio_ratio", "中心/整框之比", "低=透明")):
            a, b = sb[k + "_mean"], so[k + "_mean"]
            thr = (a + b) / 2
            print(f"  {label:<14}: 透明 {a:.3f}   不透明 {b:.3f}   "
                  f"中值阈值 {thr:.3f}   ({better})")
        print(f"\n  两类之间分离度:")
        print(f"    整框占比   透明 {sb['valid_ratio_mean']*100:.0f}% vs "
              f"不透明 {so['valid_ratio_mean']*100:.0f}%  "
              f"(差 {abs(so['valid_ratio_mean']-sb['valid_ratio_mean'])*100:.0f} 个百分点)")
        print(f"    中心占比   透明 {sb['roi_valid_ratio_mean']*100:.0f}% vs "
              f"不透明 {so['roi_valid_ratio_mean']*100:.0f}%  "
              f"(差 {abs(so['roi_valid_ratio_mean']-sb['roi_valid_ratio_mean'])*100:.0f} 个百分点)")

    rows = board + bottles
    if rows:
        csv_path = os.path.join(args.out, "transparency_features.csv")
        keys = ["box_px", "valid_px", "valid_ratio", "roi_px", "roi_valid_px",
                "roi_valid_ratio", "valid_ratio_ratio", "fg_frac", "z_median",
                "z_spread_p2p98"]
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            wr.writeheader()
            wr.writerows(rows)
        print(f"\n特征明细: {csv_path}")


if __name__ == "__main__":
    main()
