#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语义地图可视化工具
==================

把 mapper_node 输出的 semantic_map.json 画成一张四联图，用于论文插图与
现场快速检查地标质量。

    图 A  俯视图 (XY)   —— 地标在地面上的分布，点大小 ∝ log(观测次数)
    图 B  侧视图 (XZ)   —— 暴露 z 轴异常（穿地 / 悬空）的伪地标
    图 C  观测次数 vs 置信度 —— 一眼区分"可信地标"与"闪一下就没了"的伪地标
    图 D  类别分布柱状图

坐标系说明（RTAB-Map 地图坐标系 map）
    x : 前方      y : 左方      z : 上方
    地面大致在 z = 0。z < -0.3 基本可判定为伪地标（穿到地板下面了）。

用法
----
    python3 tools/plot_semantic_map.py                       # 默认读 data/semantic_map.json
    python3 tools/plot_semantic_map.py --json 别的文件.json
    python3 tools/plot_semantic_map.py --out 图.png
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")           # 无显示环境（ssh / 后台）也能出图
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# 每个类别一个固定颜色，保证多次出图颜色一致，方便论文里前后对比
CLASS_COLORS = {
    "chair":        "#d62728",
    "dining table": "#ff7f0e",
    "tv":           "#1f77b4",
    "microwave":    "#2ca02c",
    "refrigerator": "#9467bd",
    "bottle":       "#17becf",
    "cup":          "#8c564b",
    "book":         "#bcbd22",
    "person":       "#e377c2",
    "laptop":       "#7f7f7f",
}
FALLBACK_COLORS = ["#333333", "#666666", "#999999", "#bbbbbb"]


def color_for(name, all_names):
    if name in CLASS_COLORS:
        return CLASS_COLORS[name]
    idx = sorted(all_names).index(name) % len(FALLBACK_COLORS)
    return FALLBACK_COLORS[idx]


def marker_size(obs):
    """观测次数 -> 点面积。用对数映射，否则 2000 次的点会把 5 次的点完全盖住。"""
    return 20.0 + 60.0 * math.log10(max(1, obs))


def load_tracks(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    tracks = data.get("tracks", [])
    # 兼容两种可能的结构：list 或 dict(id -> track)
    if isinstance(tracks, dict):
        tracks = list(tracks.values())
    return data, tracks


def trust_rank(t):
    """可信度打分：观测次数为主，同时惩罚明显不合理的 z。仅用于排序展示。"""
    obs = t.get("observation_count", 0)
    z = t.get("position", [0, 0, 0])[2]
    score = math.log10(max(1, obs))
    if z < -0.35 or z > 2.5:      # 穿地或悬空过高
        score -= 2.0
    return score


def main():
    ap = argparse.ArgumentParser(description="语义地图可视化")
    default_json = os.path.expanduser("~/VIO/data/semantic_map.json")
    default_out = os.path.expanduser("~/VIO/data/semantic_map_preview.png")
    ap.add_argument("--json", default=default_json, help="semantic_map.json 路径")
    ap.add_argument("--out", default=default_out, help="输出 PNG 路径")
    ap.add_argument("--z-floor", type=float, default=-0.35,
                    help="低于该高度视为穿地异常（默认 -0.35 m）")
    args = ap.parse_args()

    if not os.path.isfile(args.json):
        print(f"[错误] 找不到语义地图文件: {args.json}", file=sys.stderr)
        return 1

    data, tracks = load_tracks(args.json)
    if not tracks:
        print(f"[错误] {args.json} 里没有任何地标 (tracks 为空)", file=sys.stderr)
        return 1

    stats = data.get("stats", {})
    classes = sorted({t.get("class_name", "?") for t in tracks})

    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(
        "Semantic Map  |  tracks=%d  frames=%s  detections=%s  matched=%s  created=%s  pruned=%s"
        % (len(tracks), stats.get("frames", "?"), stats.get("detections", "?"),
           stats.get("matched", "?"), stats.get("created", "?"), stats.get("pruned", "?")),
        fontsize=13,
    )

    # ---------------- 图 A：俯视图 XY ----------------
    ax = axes[0][0]
    for t in tracks:
        x, y, z = t["position"][:3]
        obs = t.get("observation_count", 0)
        c = color_for(t.get("class_name", "?"), classes)
        bad = z < args.z_floor
        ax.scatter(x, y, s=marker_size(obs), c=c,
                   marker="x" if bad else "o",
                   alpha=0.45 if bad else 0.85,
                   edgecolors="black", linewidths=0.6, zorder=3)
        ax.annotate(str(t.get("instance_id", "?")), (x, y),
                    fontsize=7, ha="center", va="center", zorder=4)
    ax.axhline(0, color="gray", lw=0.6, ls="--", zorder=1)
    ax.axvline(0, color="gray", lw=0.6, ls="--", zorder=1)
    ax.set_xlabel("x  (forward) [m]")
    ax.set_ylabel("y  (left) [m]")
    ax.set_title("A. Top view (XY) - circle=sane, x=z below floor")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="datalim")

    # ---------------- 图 B：侧视图 XZ ----------------
    ax = axes[0][1]
    for t in tracks:
        x, y, z = t["position"][:3]
        obs = t.get("observation_count", 0)
        c = color_for(t.get("class_name", "?"), classes)
        bad = z < args.z_floor
        ax.scatter(x, z, s=marker_size(obs), c=c,
                   marker="x" if bad else "o",
                   alpha=0.45 if bad else 0.85,
                   edgecolors="black", linewidths=0.6, zorder=3)
        ax.annotate(str(t.get("instance_id", "?")), (x, z),
                    fontsize=7, ha="center", va="center", zorder=4)
    ax.axhline(0, color="green", lw=1.2, ls="-", label="floor z=0")
    ax.axhline(args.z_floor, color="red", lw=1.0, ls="--",
               label=f"z_floor={args.z_floor}")
    ax.set_xlabel("x  (forward) [m]")
    ax.set_ylabel("z  (up) [m]")
    ax.set_title("B. Side view (XZ) - exposes underground / floating landmarks")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)

    # ---------------- 图 C：观测次数 vs 置信度 ----------------
    ax = axes[1][0]
    for t in tracks:
        obs = max(1, t.get("observation_count", 0))
        conf = t.get("confidence", 0.0)
        c = color_for(t.get("class_name", "?"), classes)
        ax.scatter(obs, conf, s=70, c=c, edgecolors="black",
                   linewidths=0.6, zorder=3)
        ax.annotate(str(t.get("instance_id", "?")), (obs, conf),
                    fontsize=7, ha="center", va="center", zorder=4)
    ax.set_xscale("log")
    ax.axvline(3, color="red", lw=1.0, ls="--", label="min_observations=3")
    ax.axhline(0.3, color="orange", lw=1.0, ls=":", label="min_confidence=0.3")
    ax.set_xlabel("observation_count  (log)")
    ax.set_ylabel("confidence")
    ax.set_title("C. Observations vs confidence - bottom-left = untrustworthy")
    ax.grid(True, alpha=0.25, which="both")
    ax.legend(fontsize=8)

    # ---------------- 图 D：类别分布 ----------------
    ax = axes[1][1]
    counts = {}
    for t in tracks:
        counts[t.get("class_name", "?")] = counts.get(t.get("class_name", "?"), 0) + 1
    names = sorted(counts, key=lambda k: -counts[k])
    ax.barh(range(len(names)), [counts[n] for n in names],
            color=[color_for(n, classes) for n in names],
            edgecolor="black", linewidth=0.6)
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names)
    ax.invert_yaxis()
    ax.set_xlabel("number of landmarks")
    ax.set_title("D. Class distribution")
    ax.grid(True, axis="x", alpha=0.25)

    # ---------------- 图例 ----------------
    handles = [Line2D([0], [0], marker="o", color="none", markerfacecolor=color_for(c, classes),
                      markeredgecolor="black", markersize=9, label=c) for c in classes]
    fig.legend(handles=handles, loc="lower center", ncol=min(len(classes), 8),
               fontsize=10, frameon=False, bbox_to_anchor=(0.5, 0.005))

    fig.tight_layout(rect=[0, 0.045, 1, 0.965])
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=140)
    plt.close(fig)
    print(f"[OK] 图已保存: {args.out}")

    # ---------------- 终端里再打一张可信度排序表 ----------------
    ranked = sorted(tracks, key=trust_rank, reverse=True)
    print()
    print("可信度排序（观测次数为主，穿地/悬空的排到最后）")
    print("%-4s %-14s %6s %7s %8s   %-26s %-20s" %
          ("id", "class", "obs", "conf", "best", "position (x,y,z)", "size (sx,sy,sz)"))
    print("-" * 108)
    for t in ranked:
        pos = t.get("position", [0, 0, 0])
        size = t.get("size", [0, 0, 0])
        flag = ""
        z = pos[2]
        if z < args.z_floor:
            flag = "  <-- 穿地, 伪地标"
        elif z > 2.5:
            flag = "  <-- 悬空过高, 伪地标"
        elif t.get("observation_count", 0) < 3:
            flag = "  <-- 观测太少"
        print("%-4s %-14s %6d %7.3f %8.3f   (%7.3f,%7.3f,%7.3f)  (%.2f,%.2f,%.2f)%s" %
              (t.get("instance_id", "?"), t.get("class_name", "?"),
               t.get("observation_count", 0), t.get("confidence", 0.0),
               t.get("best_confidence", 0.0),
               pos[0], pos[1], pos[2], size[0], size[1], size[2], flag))

    # rtabmap_node_id 全为 -1 是个已知缺口，顺手报一下
    no_node = sum(1 for t in tracks if t.get("rtabmap_node_id", -1) in (-1, None))
    print()
    print(f"rtabmap_node_id 未写入的地标: {no_node}/{len(tracks)}"
          f"   (全为 -1 说明还没和 RTAB-Map 节点建立关联)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
