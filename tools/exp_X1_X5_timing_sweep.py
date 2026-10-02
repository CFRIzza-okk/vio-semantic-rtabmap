#!/usr/bin/env python3
"""X1（v2 耗时）+ X5（参数敏感性），均使用已有 S01 数据，零采集。

X1  对比 v1 / v2 单帧反投影耗时，判断 v2 能否满足 5 Hz（200 ms）预算
X5  扫描 roi_ratio / percentile / depth_tolerance，给出精度-参数曲线

用法
----
    source /opt/ros/humble/setup.bash
    source ~/VIO/semantic_ws/install/setup.bash
    python3 ~/VIO/tools/exp_X1_X5_timing_sweep.py
"""
import csv
import os
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backprojection_v2 import robust_depth_modesek          # noqa: E402
from exp_M5a_box_scaling import load_bottle                  # noqa: E402
from compare_v1_v2 import v1_depth                           # noqa: E402

OUT = "/home/jetson/VIO/data/exp_X1_X5"
Z_GT = 1.5


def err(est, gt=Z_GT):
    return abs(est - gt) * 1000.0


def main():
    os.makedirs(OUT, exist_ok=True)
    data = load_bottle()
    print(f"加载 {len(data)} 个检测（透明瓶，真值 {Z_GT} m）\n")

    # ---------------- X1 耗时 ----------------
    print("=" * 74)
    print("X1  单帧反投影耗时（v1 生产算法 vs v2 模态自适应）")
    print("=" * 74)
    N = 5
    for name, fn in (("v1 分位数+固定容差", lambda d, b: v1_depth(d, b)),
                     ("v2 模态自适应", lambda d, b: robust_depth_modesek(d, b))):
        # 预热
        for d, b in data[:3]:
            fn(d, b)
        t0 = time.perf_counter()
        for _ in range(N):
            for d, b in data:
                fn(d, b)
        dt = (time.perf_counter() - t0) / (N * len(data))
        print(f"  {name:<22} {dt*1000:8.2f} ms/次   预算占比 {dt/0.2*100:5.2f}% (5 Hz = 200 ms)")

    # 单独测一下 v2 内部各阶段
    d0, b0 = data[0]
    t0 = time.perf_counter()
    for _ in range(200):
        robust_depth_modesek(d0, b0)
    print(f"\n  v2 单次明细约 {(time.perf_counter()-t0)/200*1000:.2f} ms"
          f"（含直方图 + 峰值检测；纯 numpy，无 scipy）")

    # ---------------- X5 参数敏感性 ----------------
    print()
    print("=" * 74)
    print("X5  参数敏感性（透明瓶，33 帧，MAE-z / mm）")
    print("=" * 74)

    rows = []

    def sweep(axis, values, base):
        print(f"\n  ── {axis} 扫描（其余固定：{base}）──")
        print(f"  {'取值':>8} {'MAE-z/mm':>10} {'STD-z/mm':>10} {'失败':>6}")
        for v in values:
            kw = dict(base)
            kw[axis] = v
            es, zs, fail = [], [], 0
            for d, b in data:
                r = v1_depth(d, b, **kw)
                if r is None:
                    fail += 1
                    continue
                es.append(err(r[0]))
                zs.append(r[0])
            mae = statistics.fmean(es) if es else float("nan")
            std = statistics.pstdev(zs) * 1000 if len(zs) > 1 else 0.0
            print(f"  {v:>8} {mae:>10.2f} {std:>10.2f} {fail:>6}")
            rows.append({"axis": axis, "value": v, "mae_mm": mae,
                         "std_mm": std, "fail": fail})

    base = dict(roi_ratio=0.6, percentile=20.0, tol=0.35)
    sweep("roi_ratio", [0.4, 0.5, 0.6, 0.7, 0.8, 1.0], base)
    sweep("percentile", [5.0, 10.0, 20.0, 30.0, 50.0], base)
    sweep("tol", [0.10, 0.20, 0.35, 0.50, 1.00], base)

    csv_path = os.path.join(OUT, "X5_parameter_sweep.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["axis", "value", "mae_mm", "std_mm", "fail"])
        w.writeheader()
        w.writerows(rows)
    print(f"\nCSV: {csv_path}")

    print()
    print("=" * 74)
    print("判读")
    print("=" * 74)
    print("""  · roi_ratio 若在 0.6~1.0 之间几乎不变，说明「中心 ROI 收缩」对透明物体冗余
  · percentile 若在 20~50 之间平坦，说明 20 分位不是敏感参数
  · tol 若在 0.2~0.5 之间平坦，说明本场景（远背景）对容差不敏感
    —— 但注意：远背景下容差不敏感，不能证明近背景也不敏感（见 X3）""")


if __name__ == "__main__":
    main()
