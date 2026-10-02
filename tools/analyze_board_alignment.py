#!/usr/bin/env python3
"""用标定板判定「彩色图 ↔ 对齐深度图」是否存在横向错位。

背景
----
S01 实验中，透明塑料瓶的深度返回在横向上被整体平移了 +17.7 px
（左右边界平移量相同、宽度不变 → 刚性位移特征）。
必须判定这是彩色-深度对齐误差，还是透明材质自身的伪影。

判据
----
对同一个不透明目标，若是**对齐误差**，则左右边界同向平移、宽度不变；
若是**传感器边缘膨胀**，则左右边界都向外扩、宽度变大、中心基本不动。

因此比较两个量：
    宽度差 = 深度轮廓宽 - 彩色轮廓宽
    中心差 = 深度中心   - 彩色中心

用法
----
    source /opt/ros/humble/setup.bash
    python3 ~/VIO/tools/analyze_board_alignment.py \
        ~/VIO/data/board_150 --out ~/VIO/data/board_align
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np

# 可选：报告 ArUco 解码情况（板子小的时候会解不出来，不影响主判据）
DICT_CANDIDATES = [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
    "DICT_APRILTAG_36h11", "DICT_ARUCO_ORIGINAL",
]


def segment_board(depth, valid, band=0.15):
    """取包含图像中心的同深度连通域作为标定板。

    ⚠ 不能用 morphologyEx(CLOSE)：闭运算会填洞，把远处背景的深度
    混进来（实测极差会从 0.23 m 变成 9.96 m），平面拟合随之完全失真。
    """
    h, w = depth.shape
    if not valid[h // 2, w // 2]:
        return None
    z0 = float(depth[h // 2, w // 2])
    m = (valid & (np.abs(depth - z0) < band)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    cl = int(lab[h // 2, w // 2])
    if cl == 0:
        cl = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return lab == cl


def edge_pos(profile, x, half=12):
    """在 x±half 范围内找灰度梯度最大处，即纸面边缘。"""
    a, b = max(0, x - half), min(len(profile) - 1, x + half + 1)
    if b - a < 3:
        return x
    g = np.abs(np.diff(profile[a:b]))
    return a + int(np.argmax(g))


def try_aruco(gray):
    best = None
    for name in DICT_CANDIDATES:
        if not hasattr(cv2.aruco, name):
            continue
        try:
            d = cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))
            p = cv2.aruco.DetectorParameters_create()
            p.adaptiveThreshWinSizeMin = 3
            p.adaptiveThreshWinSizeMax = 71
            p.adaptiveThreshWinSizeStep = 4
            p.minMarkerPerimeterRate = 0.008
            p.polygonalApproxAccuracyRate = 0.06
            p.errorCorrectionRate = 0.8
            c, i, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
        except Exception:
            continue
        k = 0 if i is None else len(i)
        if k and (best is None or k > best["n"]):
            best = {"dict": name, "n": k, "corners": c, "ids": i}
    return best


def analyze(prefix, out_dir):
    rgb = np.load(f"{prefix}_color.npy")
    depth = np.load(f"{prefix}_depth.npy")
    with open(f"{prefix}_info.json", encoding="utf-8") as f:
        info = json.load(f)
    name = os.path.basename(prefix)
    gray = cv2.blur(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(np.float32), (5, 5))
    valid = (depth > 0.2) & (depth < 12.0) & np.isfinite(depth)
    fx, fy, cx, cy = info["fx"], info["fy"], info["cx"], info["cy"]

    comp = segment_board(depth, valid)
    if comp is None:
        print(f"[{name}] 无法从深度图分割出标定板")
        return None
    ys, xs = np.nonzero(comp)
    by0, by1 = int(ys.min()), int(ys.max())
    bx0, bx1 = int(xs.min()), int(xs.max())

    rows, dLs, dRs, pLs, pRs, wds, cds = [], [], [], [], [], [], []
    y_lo = by0 + int(0.15 * (by1 - by0))
    y_hi = by1 - int(0.15 * (by1 - by0))
    for y in range(y_lo, y_hi + 1, max(1, (y_hi - y_lo) // 15)):
        row = np.nonzero(comp[y])[0]
        if row.size < 20:
            continue
        dL, dR = int(row.min()), int(row.max())
        pL, pR = edge_pos(gray[y], dL), edge_pos(gray[y], dR)
        rows.append(y)
        dLs.append(dL); dRs.append(dR); pLs.append(pL); pRs.append(pR)
        wds.append((dR - dL) - (pR - pL))
        cds.append((dL + dR) / 2.0 - (pL + pR) / 2.0)

    if not rows:
        print(f"[{name}] 有效行不足")
        return None

    z_med = float(np.median(depth[comp]))
    res = {
        "prefix": name,
        "depth_median_m": z_med,
        "board_x": f"{bx0}..{bx1}", "board_y": f"{by0}..{by1}",
        "rows": len(rows),
        "dL_mean": float(np.mean(np.array(dLs) - np.array(pLs))),
        "dR_mean": float(np.mean(np.array(dRs) - np.array(pRs))),
        "width_diff_px": float(np.mean(wds)),
        "center_diff_px": float(np.mean(cds)),
        "center_diff_std_px": float(np.std(cds)),
        "center_diff_mm": float(np.mean(cds) * z_med / fx * 1000.0),
    }

    # 深度平面拟合（评估深度值本身的精度）
    zz = depth[comp].astype(np.float64)
    X = (xs - cx) * zz / fx
    Y = (ys - cy) * zz / fy
    P = np.stack([X, Y, zz], axis=1)
    c0 = P.mean(axis=0)
    q = P - c0
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    nrm = vt[-1] / np.linalg.norm(vt[-1])
    res["plane_rms_mm"] = float((q @ nrm).std() * 1000.0)
    res["plane_tilt_deg"] = float(np.degrees(np.arccos(min(1.0, abs(nrm[2])))))

    det = try_aruco(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY))
    res["aruco_markers"] = 0 if det is None else det["n"]
    res["aruco_dict"] = "" if det is None else det["dict"]

    print(f"\n===== {name} =====")
    print(f"  标定板深度轮廓: x {bx0}..{bx1}  y {by0}..{by1}   深度中位 {z_med:.3f} m")
    print(f"  深度平面拟合: RMS {res['plane_rms_mm']:.2f} mm   倾角 {res['plane_tilt_deg']:.1f}°")
    print(f"  ArUco: {res['aruco_markers']} 个标记"
          + (f" ({res['aruco_dict']})" if res["aruco_dict"] else " (未解码，不影响主判据)"))
    print(f"  逐行比对 {len(rows)} 行：ΔL {res['dL_mean']:+.1f} px   ΔR {res['dR_mean']:+.1f} px")
    print(f"  宽度差 {res['width_diff_px']:+.1f} px   中心差 {res['center_diff_px']:+.1f} px "
          f"(σ {res['center_diff_std_px']:.1f})")
    print(f"  → 中心差等效 {res['center_diff_mm']:+.1f} mm @ {z_med:.2f} m")

    # 可视化
    vis = rgb.copy()
    cnts, _ = cv2.findContours(comp.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, [max(cnts, key=cv2.contourArea)], -1, (0, 255, 0), 1)
    for y, dL, dR, pL, pR in zip(rows, dLs, dRs, pLs, pRs):
        cv2.circle(vis, (dL, y), 2, (0, 255, 255), -1)
        cv2.circle(vis, (dR, y), 2, (0, 255, 255), -1)
        cv2.circle(vis, (pL, y), 2, (0, 0, 255), -1)
        cv2.circle(vis, (pR, y), 2, (0, 0, 255), -1)
    cv2.putText(vis, "green=depth   yellow=depth edge   red=RGB paper edge",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(vis, "green=depth   yellow=depth edge   red=RGB paper edge",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, f"width {res['width_diff_px']:+.1f}px  center {res['center_diff_px']:+.1f}px",
                (6, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(vis, f"width {res['width_diff_px']:+.1f}px  center {res['center_diff_px']:+.1f}px",
                (6, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    out_png = os.path.join(out_dir, f"align_{name}.png")
    cv2.imwrite(out_png, cv2.resize(vis, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    res["figure"] = out_png
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefixes", nargs="+")
    ap.add_argument("--out", default="/home/jetson/VIO/data/board_align")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = []
    for p in args.prefixes:
        try:
            r = analyze(p, args.out)
        except FileNotFoundError as e:
            print(f"[{p}] 缺少文件: {e}")
            continue
        if r:
            rows.append(r)
    if not rows:
        print("\n没有可用结果")
        return

    keys = ["prefix", "depth_median_m", "plane_rms_mm", "plane_tilt_deg",
            "aruco_markers", "aruco_dict", "rows", "dL_mean", "dR_mean",
            "width_diff_px", "center_diff_px", "center_diff_std_px",
            "center_diff_mm", "board_x", "board_y"]
    csv_path = os.path.join(args.out, "board_alignment_summary.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(rows)

    print("\n" + "=" * 76)
    print("汇总")
    print("=" * 76)
    print(f"{'前缀':<20}{'距离/m':>8}{'ΔL/px':>8}{'ΔR/px':>8}"
          f"{'宽度差/px':>11}{'中心差/px':>11}{'RMS/mm':>9}")
    for r in rows:
        print(f"{r['prefix']:<20}{r['depth_median_m']:>8.3f}{r['dL_mean']:>8.1f}"
              f"{r['dR_mean']:>8.1f}{r['width_diff_px']:>11.1f}"
              f"{r['center_diff_px']:>11.1f}{r['plane_rms_mm']:>9.2f}")
    print(f"\nCSV: {csv_path}")
    print("""
判读：
  不透明标定板：宽度差大、中心差小  → 只是传感器边缘膨胀，对齐正常
  透明瓶子    ：宽度差≈0、中心差大  → 等宽平移，属透明材质伪影
""")


if __name__ == "__main__":
    main()
