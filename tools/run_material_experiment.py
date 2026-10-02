#!/usr/bin/env python3
"""材质对照实验统一脚本：反投影消融 + 材质特征，一次输出。

用于「不透明物体（椅子/书/标定板）」与「透明物体（瓶子）」的对照实验。
对任意 rosbag + 任意 COCO 类别都可运行，结果追加到共享汇总表，
便于最终直接生成论文里的跨物体对照表。

包含的方法
----------
V0  完整框 + 中位数
V1  完整框 + 均值
V2  中心 60% + 中位数
V3  中心 60% + 20 分位数
V4  中心 60% + 20 分位数 + 框内深度一致性聚类      ← 当前生产方法
V4p 完整框 + 20 分位数 + 框内深度一致性聚类        ← 去掉 ROI 收缩（对照 V4）

同时输出材质特征
----------------
valid_ratio       整框有效深度占比
roi_valid_ratio   中心 60% 有效深度占比
ratio_r           两者之比（透明 < 1，不透明 > 1）
fg_frac           前景像素占有效像素比例
fg_in_roi_frac    前景像素中落在中心 ROI 内的比例

用法
----
    source /opt/ros/humble/setup.bash
    source ~/VIO/semantic_ws/install/setup.bash

    python3 ~/VIO/tools/run_material_experiment.py \
        ~/VIO/data/bags/_s01 \
        --class-name bottle --z-gt 1.5 --label "瓶子_1.5m_远背景" \
        --out ~/VIO/data/material_exp/bottle_150 \
        --summary ~/VIO/data/material_exp/summary.csv
"""
import argparse
import csv
import math
import os
import sqlite3
import statistics
from collections import deque

import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

METHODS = ("V0_full_median", "V1_full_mean", "V2_roi_median",
           "V3_roi_p20", "V4_roi_p20_cluster", "V4p_full_p20_cluster")

MIN_DEPTH, MAX_DEPTH = 0.2, 12.0
DEPTH_TOL, MIN_VALID, MIN_CLUSTER = 0.35, 12, 10


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def clampi(v, lo, hi):
    return int(max(lo, min(hi, v)))


def box_bounds(bbox, ratio, w, h):
    x0, y0, x1, y1 = [float(v) for v in bbox]
    x0 = max(0.0, min(w - 1.0, x0)); y0 = max(0.0, min(h - 1.0, y0))
    x1 = max(x0 + 1.0, min(float(w), x1)); y1 = max(y0 + 1.0, min(float(h), y1))
    if ratio >= 0.999:
        return clampi(math.floor(x0), 0, w), clampi(math.floor(y0), 0, h), \
               clampi(math.ceil(x1), 0, w), clampi(math.ceil(y1), 0, h)
    mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    hw = max(1.0, (x1 - x0) * ratio * 0.5)
    hh = max(1.0, (y1 - y0) * ratio * 0.5)
    return (clampi(math.floor(mx - hw), 0, w), clampi(math.floor(my - hh), 0, h),
            clampi(math.ceil(mx + hw), 0, w), clampi(math.ceil(my + hh), 0, h))


def valid_mask(a):
    return np.isfinite(a) & (a > MIN_DEPTH) & (a < MAX_DEPTH)


def eval_method(depth, bbox, fx, fy, cx, cy, method):
    """返回 dict(position, points, z0)，失败返回 None。"""
    h, w = depth.shape[:2]
    bx0, by0, bx1, by1 = box_bounds(bbox, 1.0, w, h)
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None

    use_roi = method in ("V2_roi_median", "V3_roi_p20", "V4_roi_p20_cluster")
    rx0, ry0, rx1, ry1 = box_bounds(bbox, 0.6 if use_roi else 1.0, w, h)
    roi = depth[ry0:ry1, rx0:rx1]
    rv = valid_mask(roi)
    vals = roi[rv]
    if vals.size < MIN_VALID:
        return None

    if method in ("V0_full_median", "V2_roi_median"):
        z0 = float(np.median(vals))
    elif method == "V1_full_mean":
        z0 = float(vals.mean())
    else:  # 所有 p20 变体
        z0 = float(np.percentile(vals, 20.0))
    if not (MIN_DEPTH < z0 < MAX_DEPTH):
        return None

    if method in ("V4_roi_p20_cluster", "V4p_full_p20_cluster"):
        patch = depth[by0:by1, bx0:bx1]
        pv = valid_mask(patch)
        cl = pv & (np.abs(patch - z0) <= DEPTH_TOL)
        if int(cl.sum()) < MIN_CLUSTER:
            cl = pv
            if int(cl.sum()) < MIN_VALID:
                return None
        vs, us = np.nonzero(cl)
        zs = patch[vs, us].astype(np.float64)
        us = us.astype(np.float64) + bx0
        vs = vs.astype(np.float64) + by0
        xs = (us - cx) * zs / fx
        ys = (vs - cy) * zs / fy
        return {"pos": np.array([xs.mean(), ys.mean(), zs.mean()]),
                "points": int(cl.sum()), "z0": z0, "u_mean": float(us.mean())}

    # V0~V3：x/y 取框中心投影，z 用该变体的深度估计值
    u_b = (float(bbox[0]) + float(bbox[2])) * 0.5
    v_b = (float(bbox[1]) + float(bbox[3])) * 0.5
    return {"pos": np.array([(u_b - cx) * z0 / fx, (v_b - cy) * z0 / fy, z0]),
            "points": int(vals.size), "z0": z0, "u_mean": float(u_b)}


def material_features(depth, bbox):
    """材质判别特征。"""
    h, w = depth.shape[:2]
    bx0, by0, bx1, by1 = box_bounds(bbox, 1.0, w, h)
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None
    patch = depth[by0:by1, bx0:bx1]
    pv = valid_mask(patch)
    n_valid = int(pv.sum())
    if n_valid < MIN_VALID:
        return None
    vals = patch[pv]

    rx0, ry0, rx1, ry1 = box_bounds(bbox, 0.6, w, h)
    roi = depth[ry0:ry1, rx0:rx1]
    rv = valid_mask(roi)

    med = float(np.median(vals))
    fg = pv & (np.abs(patch - med) <= DEPTH_TOL)
    n_fg = int(fg.sum())

    # 前景像素中落在中心 ROI 内的比例
    in_roi = np.zeros(patch.shape, dtype=bool)
    cx0 = clampi(rx0 - bx0, 0, patch.shape[1]); cx1 = clampi(rx1 - bx0, 0, patch.shape[1])
    cy0 = clampi(ry0 - by0, 0, patch.shape[0]); cy1 = clampi(ry1 - by0, 0, patch.shape[0])
    in_roi[cy0:cy1, cx0:cx1] = True
    n_fg_in_roi = int((fg & in_roi).sum())

    vr = n_valid / patch.size
    rvr = int(rv.sum()) / max(1, roi.size)
    return {
        "box_px": int(patch.size), "valid_px": n_valid, "valid_ratio": vr,
        "roi_px": int(roi.size), "roi_valid_px": int(rv.sum()), "roi_valid_ratio": rvr,
        "ratio_r": rvr / vr if vr > 1e-9 else float("nan"),
        "fg_px": n_fg, "fg_frac": n_fg / max(1, n_valid),
        "fg_in_roi_px": n_fg_in_roi,
        "fg_in_roi_frac": n_fg_in_roi / max(1, n_fg),
        "z_median": med,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--class-name", required=True)
    ap.add_argument("--z-gt", type=float, required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--summary", default="")
    ap.add_argument("--max-det", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    label = args.label or f"{args.class_name}_{args.z_gt}m"

    db = os.path.expanduser(args.bag)
    if os.path.isdir(db):
        c = [f for f in sorted(os.listdir(db)) if f.endswith(".db3")]
        if not c:
            raise SystemExit(f"{db} 里没有 .db3，若是 zstd 压缩请先解压")
        db = os.path.join(db, c[0])
    conn = sqlite3.connect(db)
    tmap = {int(i): (str(n), str(t)) for i, n, t in conn.execute("SELECT id,name,type FROM topics")}
    bridge = CvBridge()
    dh = deque(maxlen=8)
    info = None
    rows = []
    feats = []
    n_det = 0

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
            dh.append((stamp_ns(m), raw.astype(np.float32) * (0.001 if m.encoding == "16UC1" else 1.0)))
            continue
        if tp != "/semantics/objects" or info is None or not dh:
            continue
        objs = [o for o in m.objects if o.class_name == args.class_name]
        if not objs:
            continue
        ot = stamp_ns(m)
        _, depth = min(dh, key=lambda x: abs(x[0] - ot))
        k = np.asarray(info.k, dtype=np.float64)
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])

        for o in objs:
            if args.max_det and n_det >= args.max_det:
                break
            bbox = tuple(float(v) for v in o.bbox)
            f = material_features(depth, bbox)
            if f is None:
                continue
            f = dict(f)
            f.update(label=label, class_name=args.class_name, z_gt=args.z_gt,
                     det_index=n_det, bbox_x0=bbox[0], bbox_y0=bbox[1],
                     bbox_x1=bbox[2], bbox_y1=bbox[3])
            feats.append(f)
            for meth in METHODS:
                r = eval_method(depth, bbox, fx, fy, cx, cy, meth)
                row = dict(label=label, class_name=args.class_name, z_gt=args.z_gt,
                           det_index=n_det, method=meth, valid=int(r is not None))
                if r:
                    z = float(r["pos"][2])
                    row.update(z_est=z, x_est=float(r["pos"][0]), y_est=float(r["pos"][1]),
                               abs_err=abs(z - args.z_gt), points=r["points"], z0=r["z0"])
                else:
                    row.update(z_est="", x_est="", y_est="", abs_err="", points="", z0="")
                rows.append(row)
            n_det += 1
        if args.max_det and n_det >= args.max_det:
            break

    if not rows:
        raise SystemExit(f"没有匹配到 {args.class_name} 的检测（bag 里 /semantics/objects 有该类吗？）")

    # 逐帧明细
    rf = os.path.join(args.out, "per_frame.csv")
    keys = ["label", "class_name", "z_gt", "det_index", "method", "valid",
            "z_est", "x_est", "y_est", "abs_err", "points", "z0"]
    with open(rf, "w", newline="", encoding="utf-8") as fp:
        wr = csv.DictWriter(fp, fieldnames=keys, extrasaction="ignore")
        wr.writeheader(); wr.writerows(rows)

    # 材质特征明细
    ff = os.path.join(args.out, "features.csv")
    fkeys = ["label", "class_name", "z_gt", "det_index", "box_px", "valid_px",
             "valid_ratio", "roi_px", "roi_valid_px", "roi_valid_ratio", "ratio_r",
             "fg_px", "fg_frac", "fg_in_roi_px", "fg_in_roi_frac", "z_median",
             "bbox_x0", "bbox_y0", "bbox_x1", "bbox_y1"]
    with open(ff, "w", newline="", encoding="utf-8") as fp:
        wr = csv.DictWriter(fp, fieldnames=fkeys, extrasaction="ignore")
        wr.writeheader(); wr.writerows(feats)

    def mean(key):
        v = [float(r[key]) for r in feats if r.get(key) == r.get(key)]
        return statistics.fmean(v) if v else float("nan")

    # 汇总行
    summ = {
        "label": label, "class_name": args.class_name, "z_gt": args.z_gt,
        "n_det": n_det, "n_det_valid": len(feats),
        "box_px": mean("box_px"), "valid_ratio": mean("valid_ratio"),
        "roi_valid_ratio": mean("roi_valid_ratio"), "ratio_r": mean("ratio_r"),
        "fg_frac": mean("fg_frac"), "fg_in_roi_frac": mean("fg_in_roi_frac"),
    }
    for meth in METHODS:
        er = [float(r["abs_err"]) for r in rows if r["method"] == meth and r["valid"] == 1]
        zs = [float(r["z_est"]) for r in rows if r["method"] == meth and r["valid"] == 1]
        tag = meth.split("_")[0]
        summ[f"{tag}_mae"] = statistics.fmean(er) if er else float("nan")
        summ[f"{tag}_std"] = statistics.pstdev(zs) if len(zs) > 1 else (0.0 if zs else float("nan"))
        summ[f"{tag}_max"] = max(er) if er else float("nan")
        summ[f"{tag}_n"] = len(er)

    summ_path = os.path.expanduser(args.summary) if args.summary else os.path.join(args.out, "summary.csv")
    if args.summary:
        os.makedirs(os.path.dirname(summ_path), exist_ok=True)
        exists = os.path.exists(summ_path)
        # 同 label 覆盖，避免重复追加
        old = []
        if exists:
            with open(summ_path, newline="", encoding="utf-8") as fp:
                old = [r for r in csv.DictReader(fp) if r.get("label") != label]
        with open(summ_path, "w", newline="", encoding="utf-8") as fp:
            wr = csv.DictWriter(fp, fieldnames=list(summ.keys()))
            wr.writeheader()
            for r in old:
                wr.writerow(r)
            wr.writerow(summ)
    else:
        with open(summ_path, "w", newline="", encoding="utf-8") as fp:
            wr = csv.DictWriter(fp, fieldnames=list(summ.keys()))
            wr.writeheader(); wr.writerow(summ)

    # 打印
    print("=" * 78)
    print(f"{label}   类别={args.class_name}   真值={args.z_gt} m   检测数={n_det}")
    print("=" * 78)
    print(f"材质特征  框面积 {summ['box_px']:.0f} px | 整框有效 {summ['valid_ratio']*100:.1f}%"
          f" | 中心有效 {summ['roi_valid_ratio']*100:.1f}% | r={summ['ratio_r']:.2f}")
    print(f"          前景占比 {summ['fg_frac']*100:.1f}% | 前景落在中心ROI内 "
          f"{summ['fg_in_roi_frac']*100:.1f}%")
    print()
    print(f"{'方法':<24}{'有效':>5}{'MAE-z/m':>11}{'STD-z/m':>11}{'最大误差/m':>12}")
    for meth in METHODS:
        tag = meth.split("_")[0]
        print(f"{meth:<24}{summ[f'{tag}_n']:>5}{summ[f'{tag}_mae']:>11.5f}"
              f"{summ[f'{tag}_std']:>11.5f}{summ[f'{tag}_max']:>12.5f}")
    print()
    v4, v4p = summ["V4_mae"], summ["V4p_mae"]
    if v4 == v4 or v4p == v4p:
        better = "V4（含ROI收缩）" if v4 < v4p else "V4p（无ROI收缩）"
        print(f"V4 vs V4p: {v4*1000:.2f} mm vs {v4p*1000:.2f} mm  → {better} 更好")
    print(f"\n逐帧: {rf}\n特征: {ff}\n汇总: {summ_path}")


if __name__ == "__main__":
    main()
