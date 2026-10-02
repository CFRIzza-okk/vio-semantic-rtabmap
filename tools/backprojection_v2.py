#!/usr/bin/env python3
"""改进版反投影：用深度直方图的模态结构自适应确定前景范围。

与 v1（backprojection.py）的区别
--------------------------------
v1：ROI 取 20 分位数 → 框内 ±0.35 m 固定容差聚类

    问题：
      · 0.35 m 是调出来的魔数，且尺度相关
        （0.79 m 处覆盖 ±25.0°，6.30 m 处仅 ±3.2°）
      · 两阶段无收敛保证，S01 中二者互相矛盾
        （V3 只有分位数时 MAE 12.84 mm，加上聚类才降到 2.03 mm）
      · 聚类太小时的兜底 `cluster = valid` 会静默切换估计器

v2：对框内有效深度做直方图 → 找显著峰 → 用「峰到谷」自动定前景边界

    单峰：框内无背景污染 → 整框使用，不做任何裁剪
    多峰：取最靠近相机、且质量达标的峰，
          前景边界 = 该峰与下一显著峰之间的谷底
    → 容差由数据决定，不引入魔数
    → 同时输出模态数 / 前景质量 / 置信度，可直接用于位姿图约束加权

依赖：仅 numpy（不引入 scipy，便于在 Jetson 节点中部署）
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class DepthModeResult:
    z: float                 # 最终深度估计（所选簇的加权均值）
    mode_z: float            # 所选峰的位置
    z_lo: float              # 所选簇的深度下界
    cutoff: float            # 前景上边界（谷底位置）
    tolerance: float         # 等效容差 = cutoff - mode_z
    n_modes: int             # 显著峰数量
    peak_frac: List[float]   # 各峰质量占有效像素比例
    fg_frac: float           # 所选簇占有效像素比例
    points: int              # 所选簇像素数
    confidence: float        # 0~1
    flags: List[str] = field(default_factory=list)


@dataclass
class BackprojectionV2Result:
    position: np.ndarray
    size: np.ndarray
    yaw: float
    distance: float
    points_used: int
    depth: DepthModeResult


# ----------------------------------------------------------------------
# 基础工具（无 scipy 依赖）
# ----------------------------------------------------------------------
def _gaussian_kernel(sigma: float) -> np.ndarray:
    r = max(1, int(math.ceil(3.0 * sigma)))
    x = np.arange(-r, r + 1, dtype=np.float64)
    k = np.exp(-0.5 * (x / sigma) ** 2)
    return k / k.sum()


def _smooth(y: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0 or y.size < 3:
        return y.astype(np.float64)
    k = _gaussian_kernel(sigma)
    return np.convolve(y.astype(np.float64), k, mode="same")


def _find_peaks(y: np.ndarray, min_prominence: float) -> List[Tuple[int, float]]:
    """返回 (索引, prominence) 列表（索引相对原始 y）。

    prominence 定义为峰高减去「到达更高地形前所经过的最低鞍部」。

    ⚠ 必须在两端补零后再找：真实的深度直方图里，物体的尖峰经常就落在
    第 0 或最后一个 bin（例如瓶子 1.49~1.51 m 全部落在同一格），
    若只扫描 [1, n-2] 会完全漏掉，导致退化成"无峰"分支。
    """
    n = y.size
    if n < 3:
        return []
    pad = 2
    p = np.concatenate([np.zeros(pad), y.astype(np.float64), np.zeros(pad)])
    m = p.size
    cand = [i for i in range(1, m - 1) if p[i] >= p[i - 1] and p[i] > p[i + 1]]
    out = []
    for i in cand:
        lmin, j = p[i], i
        while j > 0:
            j -= 1
            if p[j] > p[i]:
                break
            lmin = min(lmin, p[j])
        rmin, j = p[i], i
        while j < m - 1:
            j += 1
            if p[j] > p[i]:
                break
            rmin = min(rmin, p[j])
        prom = float(p[i] - max(lmin, rmin))
        if prom >= min_prominence:
            out.append((i - pad, prom))
    return out


def _edge(h_s: np.ndarray, idx: int, d: int, thr: float,
          peak_val: float, rise_frac: float = 0.15) -> int:
    """从峰 idx 向 d 方向走，返回前景在该侧的边界 bin。

    终止条件（二者之一）：
      1. 直方图降到 thr 以下 —— 物体与背景之间有明显的空档
      2. 先到谷底、随后上升幅度超过 rise_frac×峰高 —— 物体与背景
         在深度上相邻，必须停在谷底，否则会把背景吞进来
    """
    n = h_s.size
    i = idx
    while 0 <= i + d < n:
        j = i + d
        if h_s[j] <= thr:
            i = j
            break
        if h_s[j] > h_s[i] and (h_s[j] - h_s[i]) >= rise_frac * peak_val:
            break                      # i 就是谷底
        i = j
    return i


# ----------------------------------------------------------------------
# 核心：模态自适应深度估计
# ----------------------------------------------------------------------
def robust_depth_modesek(
    depth_m: np.ndarray,
    bbox: Tuple[float, float, float, float],
    min_depth: float = 0.2,
    max_depth: float = 12.0,
    bin_width: float = 0.02,
    min_bins: int = 16,
    max_bins: int = 200,
    smooth_sigma: float = 1.2,
    peak_prominence_frac: float = 0.12,
    peak_height_frac: float = 0.10,
    min_peak_mass_frac: float = 0.20,
    min_valid_px: int = 12,
    min_cluster_px: int = 10,
) -> Optional[DepthModeResult]:
    """在检测框内用深度直方图的模态结构估计物体深度。

    返回 None 表示框内有效深度不足。
    """
    h, w = depth_m.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in bbox]
    bx0 = int(max(0, math.floor(x0)))
    bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0)))
    by1 = int(min(h, math.ceil(y1)))
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None

    patch = depth_m[by0:by1, bx0:bx1]
    valid = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    n_valid = int(valid.sum())
    if n_valid < min_valid_px:
        return None
    vals = patch[valid].astype(np.float64)

    # --- 1. 直方图 ---
    vmin, vmax = float(vals.min()), float(vals.max())
    if vmax - vmin < 1e-6:                      # 退化：全部同深度
        z = float(vals.mean())
        return DepthModeResult(z=z, mode_z=z, z_lo=vmin, cutoff=vmax,
                               tolerance=0.0, n_modes=1, peak_frac=[1.0],
                               fg_frac=1.0, points=n_valid, confidence=1.0,
                               flags=["flat_depth"])
    n_bins = int(np.clip(round((vmax - vmin) / bin_width), min_bins, max_bins))
    hist, edges = np.histogram(vals, bins=n_bins, range=(vmin, vmax))
    centers = 0.5 * (edges[:-1] + edges[1:])
    h_s = _smooth(hist.astype(np.float64), smooth_sigma)

    # --- 2. 找显著峰 ---
    peaks = _find_peaks(h_s, min_prominence=peak_prominence_frac * h_s.max())
    peaks = [(i, p) for i, p in peaks if h_s[i] >= peak_height_frac * h_s.max()]
    if not peaks:
        # 没有清晰峰：退化为整框稳健中位数（并标注低置信度）
        z = float(np.median(vals))
        return DepthModeResult(z=z, mode_z=z, z_lo=vmin, cutoff=vmax,
                               tolerance=float(vmax - z), n_modes=0,
                               peak_frac=[], fg_frac=1.0, points=n_valid,
                               confidence=0.3, flags=["no_peak"])
    peaks.sort(key=lambda ip: centers[ip[0]])          # 按深度由近到远

    # --- 3. 选峰：最靠近相机、且质量达标 ---
    total = float(hist.sum())
    chosen = None
    peak_fracs = []
    for idx, _prom in peaks:
        # 峰的归属质量：向两侧扩展到更低处（简易分水岭）
        lo = idx
        while lo > 0 and h_s[lo - 1] <= h_s[lo]:
            lo -= 1
        hi = idx
        while hi < len(h_s) - 1 and h_s[hi + 1] <= h_s[hi]:
            hi += 1
        mass = float(hist[lo:hi + 1].sum()) / max(1.0, total)
        peak_fracs.append(mass)
        if chosen is None and mass >= min_peak_mass_frac:
            chosen = (idx, lo, hi, mass)
    if chosen is None:                                  # 质量都不够：取最大峰
        idx = int(np.argmax(h_s))
        chosen = (idx, idx, idx, 1.0)
        peak_fracs = [1.0]

    idx, lo, hi, mass = chosen
    mode_z = float(centers[idx])
    peak_val = float(h_s[idx])

    # --- 4. 前景范围：以所选峰为中心向两侧走到边界 ---
    # 不依赖"必须找到第二个峰"：单峰时走向直方图低于阈值处，
    # 双峰时在谷底停住。这样"单峰 → 整框使用"这个错误逻辑被去掉
    # （标定板就是被它拉偏了 188 mm）。
    thr = max(peak_val * 0.10, 1.0)
    left = _edge(h_s, idx, -1, thr, peak_val)
    right = _edge(h_s, idx, +1, thr, peak_val)
    z_lo = float(edges[left])
    z_hi = float(edges[right + 1]) if right + 1 < len(edges) else float(vmax)

    flags = []
    if len(peaks) >= 2:
        flags.append("multi_mode")
    elif len(peaks) == 1:
        flags.append("unimodal")
    if right - left <= 1:
        flags.append("narrow_mode")

    cluster = valid & (patch >= z_lo) & (patch <= z_hi)
    if int(cluster.sum()) < min_cluster_px:
        cluster = valid & (np.abs(patch - mode_z) <= max(0.05, 0.02 * mode_z))
        flags.append("cluster_fallback")
    if int(cluster.sum()) < min_valid_px:
        return None

    zs = patch[cluster].astype(np.float64)
    z = float(zs.mean())
    fg_frac = float(zs.size) / max(1, n_valid)
    tol = float(z_hi - mode_z)

    # --- 5. 置信度 ---
    conf = fg_frac
    if "multi_mode" in flags:
        conf *= 0.8            # 有背景污染：前景比例本身就是可靠度
    if "narrow_mode" in flags:
        conf *= 0.6
    if "cluster_fallback" in flags:
        conf *= 0.4
    conf = float(np.clip(conf, 0.0, 1.0))

    return DepthModeResult(
        z=z, mode_z=mode_z, z_lo=z_lo, cutoff=z_hi, tolerance=tol,
        n_modes=len(peaks), peak_frac=peak_fracs, fg_frac=fg_frac,
        points=int(zs.size), confidence=conf, flags=flags,
    )


def backproject_v2(
    depth_m: np.ndarray,
    bbox: Tuple[float, float, float, float],
    fx: float, fy: float, cx: float, cy: float,
    min_depth: float = 0.2, max_depth: float = 12.0,
    **kw,
) -> Optional[BackprojectionV2Result]:
    """模态自适应反投影：返回三维位置（相机光学坐标系）。"""
    dr = robust_depth_modesek(depth_m, bbox, min_depth=min_depth,
                              max_depth=max_depth, **kw)
    if dr is None:
        return None

    h, w = depth_m.shape[:2]
    x0, y0, x1, y1 = [float(v) for v in bbox]
    bx0 = int(max(0, math.floor(x0))); bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0))); by1 = int(min(h, math.ceil(y1)))
    patch = depth_m[by0:by1, bx0:bx1]
    valid = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    cluster = valid & (patch >= dr.z_lo) & (patch <= dr.cutoff)
    if int(cluster.sum()) < 10:
        cluster = valid & (np.abs(patch - dr.mode_z) <= max(0.05, 0.02 * dr.mode_z))
    if int(cluster.sum()) < 10:
        cluster = valid
    vs, us = np.nonzero(cluster)
    zs = patch[vs, us].astype(np.float64)
    us = us.astype(np.float64) + bx0
    vs = vs.astype(np.float64) + by0
    xs = (us - cx) * zs / fx
    ys = (vs - cy) * zs / fy

    def rng(a):
        if a.size < 4:
            return float(a.max() - a.min()) if a.size else 0.0
        return float(np.percentile(a, 98) - np.percentile(a, 2))

    return BackprojectionV2Result(
        position=np.array([xs.mean(), ys.mean(), zs.mean()]),
        size=np.array([rng(xs), rng(ys), rng(zs)]),
        yaw=0.0,
        distance=float(np.linalg.norm([xs.mean(), ys.mean(), zs.mean()])),
        points_used=int(cluster.sum()),
        depth=dr,
    )
