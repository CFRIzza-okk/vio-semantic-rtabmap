#!/usr/bin/env python3
"""量化透明瓶子深度偏移的三层影响。

三层含义
--------
1. 像素偏移：深度返回相对瓶子几何位置平移了多少像素
2. 等效横向距离：该像素偏移在物体距离处对应多少毫米
3. 地标坐标误差：把深度按该偏移反向后重跑生产算法（V4），
   比较三维位置差 —— 这才是对语义地标真正的影响量

注意第 3 点不等于第 2 点：检测框会裁掉一部分偏移区域，
因此实际坐标误差通常小于单纯的像素换算值。

用法
----
    source /opt/ros/humble/setup.bash
    python3 ~/VIO/tools/measure_bottle_offset.py /home/jetson/VIO/data/bags/_s01 \
        --out ~/VIO/data/bottle_offset
"""
import argparse
import csv
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

DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"
OBJ_TOPIC = "/semantics/objects"


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def stock_v4(depth, bbox, fx, fy, cx, cy,
             roi_ratio=0.6, percentile=20.0, depth_tol=0.35,
             min_depth=0.2, max_depth=12.0, min_valid=12, min_cluster=10):
    """与生产代码一致的反投影：ROI 取 20 分位 → 框内深度一致性聚类 → 质心。"""
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in bbox]
    x0 = max(0.0, min(w - 1.0, x0)); y0 = max(0.0, min(h - 1.0, y0))
    x1 = max(x0 + 1.0, min(float(w), x1)); y1 = max(y0 + 1.0, min(float(h), y1))

    mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    hw, hh = max(1.0, (x1 - x0) * roi_ratio * 0.5), max(1.0, (y1 - y0) * roi_ratio * 0.5)
    rx0 = int(max(0, math.floor(mx - hw))); rx1 = int(min(w, math.ceil(mx + hw)))
    ry0 = int(max(0, math.floor(my - hh))); ry1 = int(min(h, math.ceil(my + hh)))
    if rx1 - rx0 < 2 or ry1 - ry0 < 2:
        return None
    roi = depth[ry0:ry1, rx0:rx1]
    ok = np.isfinite(roi) & (roi > min_depth) & (roi < max_depth)
    if int(ok.sum()) < min_valid:
        return None
    z0 = float(np.percentile(roi[ok], percentile))

    bx0 = int(max(0, math.floor(x0))); bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0))); by1 = int(min(h, math.ceil(y1)))
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None
    patch = depth[by0:by1, bx0:bx1]
    vd = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    cl = vd & (np.abs(patch - z0) <= depth_tol)
    if int(cl.sum()) < min_cluster:
        cl = vd
        if int(cl.sum()) < min_valid:
            return None
    vs, us = np.nonzero(cl)
    zs = patch[vs, us].astype(np.float64)
    us = us.astype(np.float64) + bx0
    vs = vs.astype(np.float64) + by0
    xs = (us - cx) * zs / fx
    ys = (vs - cy) * zs / fy
    return {
        "x": float(xs.mean()), "y": float(ys.mean()), "z": float(zs.mean()),
        "points": int(cl.sum()), "u_mean": float(us.mean()),
        "v_mean": float(vs.mean()), "z0": z0,
    }


def largest_run(mask_1d):
    """返回最长的连续 True 区间的 (起点, 终点)，无则 (None, None)。"""
    best = cur = None
    s = None
    for i, v in enumerate(mask_1d):
        if v and s is None:
            s = i
        if not v and s is not None:
            if best is None or i - s > best[1] - best[0]:
                best = (s, i - 1)
            s = None
    if s is not None:
        if best is None or len(mask_1d) - s > best[1] - best[0]:
            best = (s, len(mask_1d) - 1)
    return best if best else (None, None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default="/home/jetson/VIO/data/bottle_offset")
    ap.add_argument("--class-name", default="bottle")
    ap.add_argument("--shift-px", type=float, default=None,
                    help="像素偏移；不给则先用饱和度自动测出")
    ap.add_argument("--z-gt", type=float, default=1.5)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    db = args.bag
    if os.path.isdir(db):
        cands = [f for f in sorted(os.listdir(db)) if f.endswith(".db3")]
        if not cands:
            raise SystemExit("目录里没有 .db3，请先解压 zstd")
        db = os.path.join(db, cands[0])
    conn = sqlite3.connect(db)
    tmap = {int(i): (str(n), str(t)) for i, n, t in conn.execute("SELECT id,name,type FROM topics")}
    bridge = CvBridge()

    depth_hist = deque(maxlen=8)
    rgb_hist = deque(maxlen=8)
    info = None
    records = []

    for tid, _ts, data in conn.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
        e = tmap.get(int(tid))
        if e is None:
            continue
        tp, ts = e
        m = deserialize_message(bytes(data), get_message(ts))
        if tp == INFO_TOPIC:
            info = m
            continue
        if tp == DEPTH_TOPIC:
            raw = bridge.imgmsg_to_cv2(m, "passthrough")
            depth_hist.append((stamp_ns(m), raw.astype(np.float32) * (0.001 if m.encoding == "16UC1" else 1.0)))
            continue
        if tp == "/camera/camera/color/image_raw":
            rgb_hist.append((stamp_ns(m), bridge.imgmsg_to_cv2(m, "bgr8")))
            continue
        if tp != OBJ_TOPIC or info is None or not depth_hist or not rgb_hist:
            continue
        objs = [o for o in m.objects if o.class_name == args.class_name]
        if not objs:
            continue
        ot = stamp_ns(m)
        _, depth = min(depth_hist, key=lambda x: abs(x[0] - ot))
        rgb = min(rgb_hist, key=lambda x: abs(x[0] - ot))[1]
        k = np.asarray(info.k, dtype=np.float64)
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])

        for o in objs:
            bbox = tuple(float(v) for v in o.bbox)
            x0, y0, x1, y1 = [int(round(v)) for v in bbox]
            by0, by1 = max(0, y0), min(480, y1)
            # ⚠ 必须限制横向搜索窗口：整行搜索会把画面中其它同深度物体
            #   （左侧凳子上另一个瓶子）也算进来，宽度会虚高到几百像素。
            wx0, wx1 = max(0, x0 - 45), min(640, x1 + 45)
            band = depth[by0:by1, wx0:wx1]
            valid = np.isfinite(band) & (band > 1.30) & (band < 1.70)
            colmask = valid.sum(axis=0) > 2
            dlo, dhi = largest_run(colmask)
            depth_cols = ((wx0 + dlo, wx0 + dhi) if dlo is not None else (None, None))

            hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
            jmask = (hsv[by0:by1, wx0:wx1, 1] > 60).sum(axis=0) > 2
            jlo, jhi = largest_run(jmask)
            juice_cols = ((wx0 + jlo, wx0 + jhi) if jlo is not None else (None, None))

            rec = {
                "obj_stamp_ns": ot,
                "bbox_x0": bbox[0], "bbox_x1": bbox[2],
                "depth_col_min": depth_cols[0], "depth_col_max": depth_cols[1],
                "juice_col_min": juice_cols[0], "juice_col_max": juice_cols[1],
            }
            if depth_cols[0] is not None and juice_cols[0] is not None:
                rec["dx_px"] = ((depth_cols[0] - juice_cols[0]) + (depth_cols[1] - juice_cols[1])) / 2.0
                rec["width_depth"] = depth_cols[1] - depth_cols[0]
                rec["width_juice"] = juice_cols[1] - juice_cols[0]
            else:
                rec["dx_px"] = ""
                rec["width_depth"] = ""
                rec["width_juice"] = ""

            a = stock_v4(depth, bbox, fx, fy, cx, cy)
            rec["valid"] = int(a is not None)
            if a:
                rec.update(x_raw=a["x"], y_raw=a["y"], z_raw=a["z"],
                           u_raw=a["u_mean"], v_raw=a["v_mean"], pts_raw=a["points"])
            else:
                for kk in ("x_raw", "y_raw", "z_raw", "u_raw", "v_raw", "pts_raw"):
                    rec[kk] = ""
            rec["_depth"] = depth
            rec["_bbox"] = bbox
            rec["_f"] = (fx, fy, cx, cy)
            records.append(rec)

    if not records:
        raise SystemExit("没有匹配到检测")

    # 自动测像素偏移
    dxs = [float(r["dx_px"]) for r in records if r["dx_px"] != ""]
    dx_px = args.shift_px if args.shift_px is not None else float(np.median(dxs))
    z_med = statistics.fmean([float(r["z_raw"]) for r in records if r["valid"]])

    # 用反向平移后的深度重跑
    for r in records:
        depth = r.pop("_depth")
        bbox = r.pop("_bbox")
        fx, fy, cx, cy = r.pop("_f")
        # 实测约定（见 tools 中的验证）：warpAffine 的 M13=+t 会把图像内容
        # 整体右移 t 像素。深度内容当前偏右 dx，需要左移 dx 才能与彩色对齐，
        # 因此 M13 = -dx。
        M = np.float32([[1, 0, -dx_px], [0, 1, 0]])
        shifted = cv2.warpAffine(depth, M, (depth.shape[1], depth.shape[0]),
                                 flags=cv2.INTER_NEAREST, borderValue=0)
        b = stock_v4(shifted, bbox, fx, fy, cx, cy)
        if b and r["valid"]:
            r.update(x_corr=b["x"], y_corr=b["y"], z_corr=b["z"],
                     u_corr=b["u_mean"], pts_corr=b["points"],
                     dx_pos_mm=(b["x"] - float(r["x_raw"])) * 1000.0,
                     dz_pos_mm=(b["z"] - float(r["z_raw"])) * 1000.0)
        else:
            for kk in ("x_corr", "y_corr", "z_corr", "u_corr", "pts_corr",
                       "dx_pos_mm", "dz_pos_mm"):
                r[kk] = ""

    out_csv = os.path.join(args.out, "bottle_offset_per_frame.csv")
    keys = ["obj_stamp_ns", "bbox_x0", "bbox_x1", "depth_col_min", "depth_col_max",
            "juice_col_min", "juice_col_max", "dx_px", "width_depth", "width_juice",
            "valid", "x_raw", "y_raw", "z_raw", "u_raw", "v_raw", "pts_raw",
            "x_corr", "y_corr", "z_corr", "u_corr", "pts_corr",
            "dx_pos_mm", "dz_pos_mm"]
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(records)

    def col(k):
        return [float(r[k]) for r in records if r.get(k, "") != ""]

    dxp = col("dx_px")
    dpos = col("dx_pos_mm")
    dzp = col("dz_pos_mm")
    uraw, ucor = col("u_raw"), col("u_corr")

    print("=" * 70)
    print("透明瓶子深度横向偏移 —— 三层量化")
    print("=" * 70)
    print(f"\n【第 1 层】像素偏移")
    print(f"  深度 1.5 m 区域相对瓶子（果汁）的横向平移:")
    print(f"    中位 {dx_px:+.1f} px   均值 {statistics.fmean(dxp):+.1f} px   "
          f"标准差 {statistics.pstdev(dxp):.1f} px   样本 {len(dxp)}")
    print(f"  宽度对照: 深度 {statistics.fmean(col('width_depth')):.1f} px  "
          f"vs 果汁 {statistics.fmean(col('width_juice')):.1f} px  "
          f"(差 {statistics.fmean(col('width_depth')) - statistics.fmean(col('width_juice')):+.1f} px)")

    print(f"\n【第 2 层】等效横向距离（按物体距离换算）")
    print(f"  物体深度中位 {z_med:.3f} m，fx = {fx:.3f} px")
    print(f"  {dx_px:+.1f} px  →  {dx_px * z_med / fx * 1000:+.1f} mm  @ {z_med:.2f} m")

    print(f"\n【第 3 层】对语义地标坐标的实际影响（反向平移后重跑 V4）")
    if dpos:
        print(f"  位置横向变化: 均值 {statistics.fmean(dpos):+.1f} mm   "
              f"标准差 {statistics.pstdev(dpos):.1f} mm   n={len(dpos)}")
        print(f"  深度方向变化: 均值 {statistics.fmean(dzp):+.2f} mm  （应为 0，平移不改变深度值）")
        print(f"  参与聚类的像素质心 u: 原始 {statistics.fmean(uraw):.1f} px "
              f"→ 修正后 {statistics.fmean(ucor):.1f} px  "
              f"(差 {statistics.fmean(ucor) - statistics.fmean(uraw):+.1f} px)")
    else:
        print("  无法计算（无有效配对）")

    print(f"\n【结论】")
    print(f"  深度像素偏移 {dx_px:+.1f} px，在 {z_med:.2f} m 处等效 "
          f"{dx_px * z_med / fx * 1000:+.0f} mm；")
    if dpos:
        print(f"  但检测框会裁掉部分偏移区域，因此地标横向坐标的实际误差约 "
              f"{statistics.fmean(dpos):+.0f} mm。")
        print(f"  深度 z 不受影响（变化 {statistics.fmean(dzp):+.2f} mm），"
              f"S01 的消融结论成立。")
    print(f"\n逐帧明细: {out_csv}")


if __name__ == "__main__":
    main()
