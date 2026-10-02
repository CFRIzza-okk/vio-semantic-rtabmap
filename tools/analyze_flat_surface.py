#!/usr/bin/env python3
"""固定 ROI 表面分析：用于 COCO 检不出的镜面 / 玻璃板 / 抛光面。

为什么需要这个脚本
------------------
M2 的主样本（金属保温杯）能被 YOLO 检出，走 analyze_metal_m2.py 的 bag 流程。
但**平面镜、玻璃板在 COCO 80 类里没有对应类别**，没有检测框可用。
这类物体是平面的、位置固定，用「手工指定一块矩形 ROI」分析反而更准：
没有检测框抖动，也没有分类误差。

它回答的问题
------------
对一块平面（镜面 / 玻璃板）而言，红外到底发生了什么：

    穿过      框内大部分有效深度 ≈ **板后面背景**的距离
    反射      框内有效深度 ≈ 某个**非板面**的距离（虚像），或整块无回波
    正常返回  框内有效深度 ≈ 卷尺测的**板面**距离

用法
----
    source /opt/ros/humble/setup.bash

    # 第 1 步：先出一张带坐标网格的图，方便你读出 ROI 的像素坐标
    python3 ~/VIO/tools/analyze_flat_surface.py ~/VIO/data/glass_150 --grid

    # 第 2 步：按读到的坐标分析（两种写法都行）
    python3 ~/VIO/tools/analyze_flat_surface.py ~/VIO/data/glass_150 \
        --box 240,150,420,330 --z-gt 1.5 \
        --label "玻璃板_1.5m" --out ~/VIO/data/material_exp/glass_150

    # 或者用相对整幅图的比例（0~1），不想读坐标时更方便
    python3 ~/VIO/tools/analyze_flat_surface.py ~/VIO/data/glass_150 \
        --box-frac 0.38,0.31,0.66,0.69 --z-gt 1.5 \
        --label "玻璃板_1.5m" --out ~/VIO/data/material_exp/glass_150
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")  # 无显示环境也能出图
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    from analyze_metal_m2 import (  # noqa: E402
        MIN_DEPTH, MAX_DEPTH, GT_TOL, BEYOND_TOL, NO_RETURN_V,
        setup_cjk_font, fmt,
    )
except ImportError as _e:  # 最常见原因就是没 source ROS
    raise SystemExit(
        f"导入 analyze_metal_m2 失败：{_e}\n"
        "通常是因为没有 source ROS，请先执行：\n"
        "    source /opt/ros/humble/setup.bash\n"
        "    source ~/VIO/semantic_ws/install/setup.bash\n"
    )

RING_PX = 45          # 板外参考环的宽度（像素）
BG_TOL = 0.15         # 与背景中位数差 15 cm 内 → 认为打到了背景


def valid_mask(a):
    return np.isfinite(a) & (a > MIN_DEPTH) & (a < MAX_DEPTH)


def load_prefix(prefix):
    d = np.load(f"{prefix}_depth.npy").astype(np.float32)
    c = None
    if os.path.exists(f"{prefix}_color.npy"):
        c = np.load(f"{prefix}_color.npy")
    info = {}
    if os.path.exists(f"{prefix}_info.json"):
        with open(f"{prefix}_info.json", encoding="utf-8") as f:
            info = json.load(f)
    return c, d, info


def ring_mask(shape, box, width=RING_PX):
    """框外一圈（矩形环），作为「板后面背景」的参考。"""
    h, w = shape[:2]
    x0, y0, x1, y1 = box
    ox0, oy0 = max(0, x0 - width), max(0, y0 - width)
    ox1, oy1 = min(w, x1 + width), min(h, y1 + width)
    outer = np.zeros((h, w), dtype=bool)
    outer[oy0:oy1, ox0:ox1] = True
    inner = np.zeros((h, w), dtype=bool)
    inner[y0:y1, x0:x1] = True
    return outer & ~inner


def make_grid(prefix, color, out_png):
    """带像素坐标网格的彩色图，方便读出 ROI 坐标。"""
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.imshow(color[:, :, ::-1])
    h, w = color.shape[:2]
    step = 50
    for x in range(0, w, step):
        ax.axvline(x, color="yellow", lw=0.5, alpha=0.55)
        ax.text(x + 2, 12, str(x), color="yellow", fontsize=7)
    for y in range(0, h, step):
        ax.axhline(y, color="yellow", lw=0.5, alpha=0.55)
        ax.text(2, y + 12, str(y), color="yellow", fontsize=7)
    ax.set_title(f"{os.path.basename(prefix)} —— 每 {step} px 一条网格线，坐标已标注\n"
                 f"读出目标矩形的左上角与右下角像素坐标，用于 --box x0,y0,x1,y1")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return out_png


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix", help="capture_rgbd.py 的前缀，如 ~/VIO/data/glass_150")
    ap.add_argument("--box", default="", help="ROI 像素坐标 x0,y0,x1,y1")
    ap.add_argument("--box-frac", default="", help="ROI 相对坐标 fx0,fy0,fx1,fy1（0~1）")
    ap.add_argument("--z-gt", type=float, default=float("nan"),
                    help="卷尺实测的「相机→板面」距离 [m]")
    ap.add_argument("--z-bg", type=float, default=float("nan"),
                    help="（可选）卷尺实测的「相机→板后面背景」距离 [m]")
    ap.add_argument("--grid", action="store_true", help="只输出带坐标网格的图")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    font = setup_cjk_font()
    prefix = os.path.expanduser(args.prefix)
    out = os.path.expanduser(args.out) if args.out else os.path.dirname(prefix) or "."
    os.makedirs(out, exist_ok=True)
    label = args.label or os.path.basename(prefix)

    if not os.path.exists(f"{prefix}_depth.npy"):
        raise SystemExit(f"找不到 {prefix}_depth.npy —— 先用 capture_rgbd.py 抓一对 RGB-D")

    color, depth, info = load_prefix(prefix)
    h, w = depth.shape[:2]
    vm = valid_mask(depth)

    # ---------------- 只出网格图
    if args.grid:
        if color is None:
            raise SystemExit("没有彩色图，无法画网格")
        p = make_grid(prefix, color, os.path.join(out, f"grid_{os.path.basename(prefix)}.png"))
        print(f"已输出带坐标网格的图：{p}")
        print("读出目标矩形的左上角、右下角像素坐标，再用 --box x0,y0,x1,y1 运行。")
        return

    # ---------------- 解析 ROI
    if args.box:
        x0, y0, x1, y1 = [int(float(v)) for v in args.box.split(",")]
    elif args.box_frac:
        fx0, fy0, fx1, fy1 = [float(v) for v in args.box_frac.split(",")]
        x0, y0, x1, y1 = int(fx0 * w), int(fy0 * h), int(fx1 * w), int(fy1 * h)
    else:
        raise SystemExit("必须给 --box 或 --box-frac（先跑 --grid 读坐标）")

    x0, x1 = max(0, min(x0, w - 1)), max(1, min(x1, w))
    y0, y1 = max(0, min(y0, h - 1)), max(1, min(y1, h))
    if x1 - x0 < 5 or y1 - y0 < 5:
        raise SystemExit(f"ROI 太小：{(x1-x0)}x{(y1-y0)} px")
    box = (x0, y0, x1, y1)

    inbox = np.zeros((h, w), dtype=bool)
    inbox[y0:y1, x0:x1] = True
    ring = ring_mask((h, w), box)   # 紧贴框外的一圈（容易被污染）
    outside = ~inbox                # 整幅图去掉 ROI —— 更稳的背景参考

    din = depth[inbox]
    vin = valid_mask(din)
    dring, vring = depth[ring], valid_mask(depth[ring])
    dout, vout = depth[outside], valid_mask(depth[outside])

    n_in, n_in_valid = int(din.size), int(vin.sum())
    v_box = n_in_valid / max(1, n_in)
    v_ring = int(vring.sum()) / max(1, int(dring.size))

    z_in = din[vin].astype(np.float64)
    z_ring = dring[vring].astype(np.float64)
    z_out = dout[vout].astype(np.float64)
    z_ring_med = float(np.median(z_ring)) if z_ring.size else float("nan")
    z_out_med = float(np.median(z_out)) if z_out.size else float("nan")

    # ROI 内深度的主要峰（最多 3 个），用于看结构
    peaks = []
    if z_in.size > 20:
        hist, edges = np.histogram(z_in, bins=80)
        centers = 0.5 * (edges[:-1] + edges[1:])
        for i in np.argsort(hist)[::-1][:6]:
            if hist[i] < max(5, 0.02 * z_in.size):
                continue
            if any(abs(centers[i] - p[0]) < 0.25 for p in peaks):
                continue
            peaks.append((float(centers[i]), int(hist[i]), float(hist[i]) / z_in.size))
            if len(peaks) == 3:
                break

    # ---------------- 归类
    zgt = args.z_gt
    if args.z_bg == args.z_bg:
        bg_ref, bg_tol, bg_src = args.z_bg, GT_TOL, "你给的背景真值"
    else:
        bg_ref, bg_tol, bg_src = z_out_med, BG_TOL, "ROI 外参考"

    near_plate = np.zeros(0, dtype=bool)
    near_bg = np.zeros(0, dtype=bool)
    other = np.zeros(0, dtype=bool)
    if z_in.size:
        near_plate = (np.abs(z_in - zgt) <= GT_TOL) if zgt == zgt \
            else np.zeros(z_in.size, bool)
        near_bg = (np.abs(z_in - bg_ref) <= bg_tol) if bg_ref == bg_ref \
            else np.zeros(z_in.size, bool)
        near_bg = near_bg & ~near_plate
        other = ~(near_plate | near_bg)

    def frac(m):
        return float(m.sum()) / z_in.size if z_in.size else float("nan")

    f_plate, f_bg, f_other = frac(near_plate), frac(near_bg), frac(other)

    # 污染告警：ROI **外**的深度中位数居然和板面真值很接近，
    # 说明这个 ROI 多半把板外的背景也框进来了，后面的判定不可信。
    contaminated = (zgt == zgt and z_out_med == z_out_med
                    and abs(z_out_med - zgt) <= 0.25)

    # 峰归属：每个主峰要么是板面、要么是背景、要么「无法解释」。
    # 「无法解释」的峰既可能是镜面虚像（M2 要找的东西），
    # 也可能是 ROI 框进了别的东西 —— 所以只提示、不擅自下结论。
    def peak_kind(zc):
        if zgt == zgt and abs(zc - zgt) <= GT_TOL:
            return "plate"
        if bg_ref == bg_ref and abs(zc - bg_ref) <= max(bg_tol, BG_TOL):
            return "bg"
        return "unexplained"

    unexplained = [(z, c, f) for (z, c, f) in peaks
                   if f >= 0.05 and peak_kind(z) == "unexplained"]
    # 真正的镜面虚像会是一个**集中的**峰；如果非板面回波是散的，
    # 那更像是 ROI 里混进了别的东西，不能叫「反射」。
    top_other = unexplained[0] if unexplained else None
    concentrated = top_other is not None and top_other[2] >= 0.25

    # ---------------- 判定
    if v_box < NO_RETURN_V:
        mode, note = "no_return", "框内几乎没有有效深度 —— 强镜面 / 全反射最典型的表现"
    elif contaminated:
        mode, note = "roi_suspect", (
            "ROI **以外**的深度中位数与板面真值接近，说明这个 ROI 很可能把板外的背景"
            "也框进来了，判定不可信。请先用 --grid 读准坐标再重跑。")
    elif z_in.size and f_bg > 0.5:
        mode, note = "through", "大部分回波来自板**后面**的背景 —— 红外穿透了"
    elif z_in.size and f_plate > 0.5:
        mode, note = "surface", "大部分回波来自板面本身 —— 板面正常返回"
    elif z_in.size and f_other > 0.4 and concentrated:
        mode = "reflection"
        note = (f"非板面回波集中在 {top_other[0]:.3f} m 的单一峰上"
                f"（占有效回波 {top_other[2]*100:.1f}%）—— 疑似反射 / 虚像"
                f"（板面 {zgt:.3f} m，背景参考 {bg_ref:.3f} m）")
    elif z_in.size and f_other > 0.4:
        mode = "mixed"
        biggest = f"{top_other[0]:.3f} m / {top_other[2]*100:.1f}%" if top_other else "无显著峰"
        note = (f"非板面回波占总回波 {f_other*100:.0f}%，但**很分散**"
                f"（最大非板面峰仅 {biggest}），不是集中的虚像。"
                f"更像是这块 ROI 混进了背景或前景杂物 —— 请收紧 ROI 重跑。")
    else:
        mode, note = "mixed", "回波来源混杂 —— 需要看直方图逐峰判断"

    # ---------------- 报告
    print("=" * 76)
    print(f"{label}    文件={os.path.basename(prefix)}    图像 {w}x{h}")
    print("=" * 76)
    print(f"ROI 像素坐标 x0,y0,x1,y1 = {x0},{y0},{x1},{y1}   （{x1-x0}×{y1-y0} px）")
    print(f"matplotlib 中文字体：{font or '未找到'}")
    print()
    print("【有效性】")
    print(f"  ROI 内有效深度占比      {v_box*100:6.1f}%   ({n_in_valid}/{n_in} px)")
    print(f"  板外 {RING_PX}px 参考环    {v_ring*100:6.1f}%   （紧贴框外，易受污染）")
    print()
    print("【深度分布】")
    if z_in.size:
        print(f"  ROI 内有效深度  中位数 {np.median(z_in):.3f} m   "
              f"P10 {np.percentile(z_in,10):.3f}   P90 {np.percentile(z_in,90):.3f}")
    else:
        print("  ROI 内有效深度  无")
    print(f"  参考环中位数          {fmt(z_ring_med,3)} m")
    print(f"  ROI 外中位数          {fmt(z_out_med,3)} m   （背景参考，来自「{bg_src}」）")
    if zgt == zgt:
        print(f"  板面真值（卷尺）       {zgt:.3f} m")
    if peaks:
        print("  ROI 内主要深度峰（归属按「板面真值 / 背景参考」判定）：")
        for zc, cnt, fr in peaks:
            k = peak_kind(zc)
            kcn = {"plate": "板面", "bg": "背景", "unexplained": "★无法解释"}[k]
            print(f"      {zc:6.3f} m   {cnt:>7} px  ({fr*100:4.1f}%)   {kcn}")
    print()
    print("【回波归属】占 ROI 内有效回波的比例")
    print(f"  打在板面上 (±{GT_TOL*100:.0f}cm)   {f_plate*100:6.1f}%")
    print(f"  打在板后背景             {f_bg*100:6.1f}%")
    print(f"  其他（可疑反射/虚像）    {f_other*100:6.1f}%")
    print()
    if contaminated:
        print("⚠ ROI 污染告警：ROI 以外的深度中位数（"
              f"{z_out_med:.3f} m）与板面真值（{zgt:.3f} m）相差不到 25 cm，")
        print("  说明这块 ROI 很可能把板外的背景一起框进来了，请用 --grid 重新读坐标。")
        print()
    if unexplained:
        print("★ 存在「无法解释」的深度峰（既不是板面、也不是背景参考）：")
        for zc, cnt, fr in unexplained:
            print(f"      {zc:.3f} m   占 ROI 有效回波 {fr*100:.1f}%")
        print("  它可能是镜面/玻璃的**虚像**（M2 要找的现象），")
        print("  也可能只是这块 ROI 框进了别的东西 —— 请结合现场摆放判断，")
        print("  必要时收紧 ROI 重跑一次对照。")
        print()
    print(f"【判定】{mode}")
    print(f"  {note}")
    print()

    # ---------------- 出图
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2))

    ax = axes[0]
    if color is not None:
        ax.imshow(color[:, :, ::-1])
        ov = np.zeros((h, w, 4), dtype=np.float32)
        ov[inbox & vm] = (0, 1, 0, 0.40)
        ov[inbox & ~vm] = (1, 0, 0, 0.40)
        ax.imshow(ov)
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                   edgecolor="yellow", lw=2))
    ax.set_title("彩色 + 有效性掩码\n绿=有回波 红=无回波")
    ax.axis("off")

    ax = axes[1]
    dsp = np.where(vm, depth, np.nan)
    lo = np.nanpercentile(dsp, 2) if np.isfinite(dsp).any() else 0.0
    hi = np.nanpercentile(dsp, 98) if np.isfinite(dsp).any() else 5.0
    im = ax.imshow(dsp, cmap="turbo", vmin=lo, vmax=hi)
    plt.colorbar(im, ax=ax, fraction=0.046, label="深度 [m]")
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                               edgecolor="lime", lw=2))
    ax.set_title(f"深度图（{lo:.1f}~{hi:.1f} m）")
    ax.axis("off")

    ax = axes[2]
    if z_in.size:
        ax.hist(z_in, bins=70, color="tab:purple", edgecolor="black", lw=0.3,
                label=f"ROI 内（{z_in.size} px）")
    if z_ring.size:
        ax.hist(z_ring, bins=70, color="tab:gray", alpha=0.45, edgecolor="black",
                lw=0.3, label=f"板外参考环（{z_ring.size} px）")
    if zgt == zgt:
        ax.axvline(zgt, color="red", lw=2, label=f"板面真值 {zgt:.2f} m")
    if z_ring_med == z_ring_med:
        ax.axvline(z_ring_med, color="black", lw=1.6, ls="--",
                   label=f"参考环中位 {z_ring_med:.2f} m")
    ax.set_xlabel("深度 [m]")
    ax.set_ylabel("像素数")
    ax.set_title(f"ROI vs 板外 深度直方图\n判定：{mode}")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle(f"固定 ROI 表面分析 — {label}   "
                 f"ROI 有效 {v_box*100:.1f}% ｜ 打在板面 {f_plate*100:.0f}% ｜ "
                 f"穿透到背景 {f_bg*100:.0f}% ｜ 其他 {f_other*100:.0f}%", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    p = os.path.join(out, f"fig_roi_{label}.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    print(f"图: {p}")


if __name__ == "__main__":
    main()
