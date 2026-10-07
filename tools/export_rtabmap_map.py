#!/usr/bin/env python3
# ============================================================================
# export_rtabmap_map.py —— 从 RTAB-Map 数据库导出轨迹、回环和语义地标
#
# 即使 Data / Feature 表损坏（没有点云），只要 Node 和 Link 完好，本工具
# 仍然可以把轨迹、回环和地标导出来，并画成图。
#
# 输出的图包含：
#   左上  俯视轨迹（按时间着色），叠加回环连线
#   右上  高度 z 随时间变化
#   左下  语义地标位置（世界坐标）
#   右下  统计信息
#
# 用法：
#   python3 tools/export_rtabmap_map.py 数据库.db --out 轨迹.png
#   python3 tools/export_rtabmap_map.py 数据库.db --csv 轨迹.csv --no-plot
# ============================================================================

import argparse
import csv
import os
import sqlite3
import struct
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

# RTAB-Map Link::Type，取自 corelib/include/rtabmap/core/Link.h
LINK_TYPES = {
    0: "neighbor",
    1: "global_closure",
    2: "local_space_closure",
    3: "local_time_closure",
    4: "user_closure",
    5: "virtual_closure",
    6: "neighbor_merged",
    7: "pose_prior",
    8: "landmark",
    9: "gravity",
}


def decode_transform(blob):
    """把 3x4 float32 (行主序) 解成 (R, t)。"""
    if not blob or len(blob) < 48:
        return None
    v = struct.unpack("<12f", blob[:48])
    rot = ((v[0], v[1], v[2]), (v[4], v[5], v[6]), (v[8], v[9], v[10]))
    return rot, (v[3], v[7], v[11])


def decode_pose(blob):
    """Node.pose 是 3x4 float32，行主序；平移量在索引 3 / 7 / 11。"""
    dec = decode_transform(blob)
    return None if dec is None else dec[1]


def load_nodes(conn):
    rows = conn.execute("select id, stamp, pose from Node order by id").fetchall()
    nodes = []
    for node_id, stamp, pose in rows:
        dec = decode_transform(pose)
        if dec is None:
            continue
        rot, t = dec
        nodes.append(
            {
                "id": node_id,
                "stamp": stamp,
                "x": t[0],
                "y": t[1],
                "z": t[2],
                "R": rot,
            }
        )
    return nodes


def load_links(conn):
    """返回 (位姿链接, 地标链接)。地标链接的 to_id 为负数。"""
    pose_links, landmark_links = [], []
    for from_id, to_id, ltype, transform in conn.execute(
        "select from_id, to_id, type, transform from Link"
    ):
        if to_id < 0:
            landmark_links.append((from_id, to_id, ltype, transform))
        else:
            pose_links.append((from_id, to_id, ltype))
    return pose_links, landmark_links


def landmark_world_positions(nodes_by_id, landmark_links):
    """地标链接的 transform 是 base_link -> landmark。

    世界坐标 = R_node * t_landmark + t_node。必须带上旋转，否则相机转动后
    地标会被算到错误的位置。
    """
    out = {}
    for from_id, to_id, _ltype, transform in landmark_links:
        node = nodes_by_id.get(from_id)
        lm = decode_transform(transform)
        if node is None or lm is None:
            continue
        (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = node["R"]
        lx, ly, lz = lm[1]
        out.setdefault(to_id, []).append(
            (
                r00 * lx + r01 * ly + r02 * lz + node["x"],
                r10 * lx + r11 * ly + r12 * lz + node["y"],
                r20 * lx + r21 * ly + r22 * lz + node["z"],
            )
        )
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description="从 RTAB-Map 数据库导出轨迹、回环和语义地标。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("db", help="RTAB-Map .db 文件")
    parser.add_argument("--out", default="", help="输出 PNG 路径")
    parser.add_argument("--csv", default="", help="输出轨迹 CSV 路径")
    parser.add_argument("--title", default="", help="图标题")
    parser.add_argument("--no-plot", action="store_true", help="只导出数据，不画图")
    parser.add_argument("--landmark-max-spread", type=float, default=0.25,
                        help="地标多帧观测的平均离散度上限（米），超过视为不可信")
    args = parser.parse_args()

    if not os.path.isfile(args.db):
        print(f"错误：找不到数据库 {args.db}", file=sys.stderr)
        return 1

    conn = sqlite3.connect(f"file:{args.db}?mode=ro", uri=True)
    nodes = load_nodes(conn)
    if not nodes:
        print("错误：数据库里没有可用的 Node 位姿。", file=sys.stderr)
        return 1
    pose_links, landmark_links = load_links(conn)
    conn.close()

    nodes_by_id = {n["id"]: n for n in nodes}
    by_type = {}
    for _f, _t, ltype in pose_links:
        by_type[ltype] = by_type.get(ltype, 0) + 1
    closures = [(f, t) for f, t, lt in pose_links if lt in (1, 2)]
    landmarks = landmark_world_positions(nodes_by_id, landmark_links)

    # 同一地标的多帧观测应当收敛到同一点。离散度大说明这次观测不可信，
    # 画图时按阈值过滤，避免离群点把图撑坏。
    landmark_stats = {}
    for lid, obs in landmarks.items():
        cx = sum(o[0] for o in obs) / len(obs)
        cy = sum(o[1] for o in obs) / len(obs)
        cz = sum(o[2] for o in obs) / len(obs)
        spread = sum(
            ((o[0] - cx) ** 2 + (o[1] - cy) ** 2 + (o[2] - cz) ** 2) ** 0.5
            for o in obs
        ) / len(obs)
        landmark_stats[lid] = {"pos": (cx, cy, cz), "n": len(obs), "spread": spread}

    reliable = {k: v for k, v in landmark_stats.items()
                if v["spread"] <= args.landmark_max_spread}
    rejected = {k: v for k, v in landmark_stats.items()
                if v["spread"] > args.landmark_max_spread}

    t0 = nodes[0]["stamp"] or 0.0
    span = (nodes[-1]["stamp"] or 0.0) - t0

    print(f"数据库      : {args.db}")
    print(f"节点        : {len(nodes):,}")
    print(f"位姿链接    : {len(pose_links):,}  " +
          " ".join(f"{LINK_TYPES.get(k, k)}={v}" for k, v in sorted(by_type.items())))
    print(f"回环(全局+局部): {len(closures):,}")
    print(f"地标        : {len(landmarks):,} 个，观测 {len(landmark_links):,} 次"
          f"（可信 {len(reliable)}，离散 {len(rejected)}）")
    for lid, st in sorted(reliable.items(), key=lambda kv: -kv[1]["n"])[:10]:
        print(f"    {lid:>10d}  观测{st['n']:>5d}  位置"
              f"({st['pos'][0]:6.2f},{st['pos'][1]:6.2f},{st['pos'][2]:6.2f})  "
              f"离散度 {st['spread']:.3f}m")
    print(f"时长        : {span:.1f} 秒 ({span/60:.1f} 分钟)")
    xs = [n["x"] for n in nodes]
    ys = [n["y"] for n in nodes]
    zs = [n["z"] for n in nodes]
    print(f"轨迹范围    : x [{min(xs):.2f}, {max(xs):.2f}]  "
          f"y [{min(ys):.2f}, {max(ys):.2f}]  z [{min(zs):.2f}, {max(zs):.2f}] (米)")

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["id", "stamp", "t_rel_s", "x", "y", "z"])
            for n in nodes:
                w.writerow([n["id"], n["stamp"], (n["stamp"] or 0) - t0,
                            n["x"], n["y"], n["z"]])
        print(f"已写出 CSV  : {args.csv}")

    if args.no_plot:
        return 0

    out = args.out or os.path.splitext(os.path.basename(args.db))[0] + "_trajectory.png"
    title = args.title or os.path.basename(args.db)

    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(f"RTAB-Map export — {title}", fontsize=15, fontweight="bold")

    # --- 俯视轨迹 ---------------------------------------------------------
    ax = axes[0][0]
    times = [(n["stamp"] or t0) - t0 for n in nodes]
    pts = [(n["x"], n["y"]) for n in nodes]
    segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
    lc = LineCollection(segs, cmap="viridis", linewidths=1.8)
    lc.set_array(np.array(times[:-1]))
    ax.add_collection(lc)
    for f, t in closures:
        a, b = nodes_by_id.get(f), nodes_by_id.get(t)
        if a and b:
            ax.plot([a["x"], b["x"]], [a["y"], b["y"]],
                    color="crimson", linewidth=0.5, alpha=0.45, zorder=1)
    ax.scatter([pts[0][0]], [pts[0][1]], c="lime", s=110, marker="o",
               edgecolors="black", zorder=5, label="start")
    ax.scatter([pts[-1][0]], [pts[-1][1]], c="red", s=110, marker="X",
               edgecolors="black", zorder=5, label="end")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"Top-down trajectory ({len(nodes)} nodes, "
                 f"{len(closures)} closures)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    cb = fig.colorbar(lc, ax=ax, fraction=0.046)
    cb.set_label("time (s)")

    # --- 高度随时间 -------------------------------------------------------
    ax = axes[0][1]
    ax.plot(times, zs, color="tab:blue", linewidth=1.2)
    ax.set_title("Height z over time")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("z (m)")
    ax.grid(alpha=0.3)

    # --- 地标 -------------------------------------------------------------
    ax = axes[1][0]
    if landmark_stats:
        if reliable:
            for lid, st in reliable.items():
                mx, my = st["pos"][0], st["pos"][1]
                ax.scatter([mx], [my], s=130, marker="*", zorder=5)
                ax.annotate(f"{lid}\n({st['n']})", (mx, my), fontsize=7,
                            textcoords="offset points", xytext=(6, 4))
        if rejected:
            ax.scatter([st["pos"][0] for st in rejected.values()],
                       [st["pos"][1] for st in rejected.values()],
                       s=45, marker="x", color="crimson", zorder=4,
                       label=f"unstable ({len(rejected)})")
            ax.legend(loc="best", fontsize=9)
        ax.plot(xs, ys, color="lightgray", linewidth=1.0, zorder=1)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(f"Semantic landmarks — {len(reliable)} reliable / "
                     f"{len(landmark_stats)} total (spread <= "
                     f"{args.landmark_max_spread:g} m)")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(alpha=0.3)
    else:
        ax.axis("off")
        ax.text(0.5, 0.5, "no landmark links", ha="center", va="center")

    # --- 统计 -------------------------------------------------------------
    ax = axes[1][1]
    ax.axis("off")
    lines = [
        f"database        : {os.path.basename(args.db)}",
        f"size            : {os.path.getsize(args.db)/1048576:.1f} MB",
        f"nodes           : {len(nodes):,}",
        f"duration        : {span/60:.1f} min",
        f"pose links      : {len(pose_links):,}",
    ]
    for k, v in sorted(by_type.items()):
        lines.append(f"    {LINK_TYPES.get(k, k):<20s}: {v:,}")
    lines += [
        f"closures        : {len(closures):,}",
        f"landmarks       : {len(landmarks):,}",
        f"landmark obs    : {len(landmark_links):,}",
        "",
        f"x range         : {min(xs):.2f} .. {max(xs):.2f} m",
        f"y range         : {min(ys):.2f} .. {max(ys):.2f} m",
        f"z range         : {min(zs):.2f} .. {max(zs):.2f} m",
    ]
    ax.text(0.02, 0.98, "\n".join(lines), va="top", ha="left",
            family="monospace", fontsize=10)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, dpi=130)
    print(f"已写出图片  : {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
