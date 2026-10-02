"""2D 检测框 -> 3D 语义物体的几何反投影工具。

这部分是整个语义层精度最关键的地方，答辩时可以按下面的思路讲：

1) 为什么不能直接用"检测框中心像素的深度"？
   RealSense 的深度图在物体边缘会发生 depth bleeding（前景物体的深度会
   "渗出"到背景像素上），同时镜面/黑色/远距离表面会产生无效值。框中心
   恰好落在无效像素或背景上的概率并不低。

2) 本实现的三步策略
   (a) ROI 收缩：只在检测框中心 roi_ratio 比例的区域采样，主动回避边界。
   (b) 鲁棒深度：对 ROI 内有效深度取 depth_percentile 分位数而不是均值。
       取低分位数（默认 20）等于"偏向物体前景"，因为背景总是更远。
   (c) 深度一致性聚类：以 (b) 得到的 z0 为中心，保留 |z - z0| <= tolerance
       的像素，认为它们属于同一个物体表面；再对这些像素求 3D 质心。
       这比单像素反投影稳定得多，也顺带给出物体的可见尺寸。

3) 已知局限（写进论文的"局限性"一节）
   单视角深度只能观测到物体朝向相机的一面，所以 size.z 是"可见表面厚度"
   而不是真实深度。真正的物体尺寸必须靠多视角融合才能恢复。yaw 同理，
   单帧无法可靠估计，这里只根据点云在 x-y 平面上的主方向给出粗值。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class BackprojectionResult:
    """一次成功的反投影结果（全部在相机光学坐标系下，单位米）。"""

    position: np.ndarray      # (3,) 物体中心
    size: np.ndarray          # (3,) 可见尺寸 (x=宽, y=高, z=厚度)
    yaw: float                # 绕 z 轴朝向，弧度
    distance: float           # 到光心距离
    points_used: int          # 参与计算的有效像素数


def robust_depth_from_bbox(
    depth_m: np.ndarray,
    bbox: Tuple[float, float, float, float],
    roi_ratio: float = 0.6,
    percentile: float = 20.0,
    min_depth: float = 0.2,
    max_depth: float = 12.0,
    min_valid_px: int = 12,
) -> Optional[float]:
    """从检测框 ROI 内估计一个鲁棒的物体深度 z0。

    返回 None 表示该框内没有足够的有效深度（物体太远/反光/遮挡）。
    """
    h, w = depth_m.shape[:2]
    x0, y0, x1, y1 = bbox

    # --- (a) ROI 收缩：取框中心的一块，回避 depth bleeding 的边界 ---
    cx, cy = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    half_w = max(1.0, (x1 - x0) * roi_ratio * 0.5)
    half_h = max(1.0, (y1 - y0) * roi_ratio * 0.5)

    rx0 = int(max(0, math.floor(cx - half_w)))
    rx1 = int(min(w, math.ceil(cx + half_w)))
    ry0 = int(max(0, math.floor(cy - half_h)))
    ry1 = int(min(h, math.ceil(cy + half_h)))
    if rx1 - rx0 < 2 or ry1 - ry0 < 2:
        return None

    roi = depth_m[ry0:ry1, rx0:rx1]

    # --- 有效深度掩码：排除 0 / NaN / inf / 越界 ---
    valid = np.isfinite(roi) & (roi > min_depth) & (roi < max_depth)
    if int(valid.sum()) < min_valid_px:
        return None

    # --- (b) 取低分位数，偏向物体前景而非背景 ---
    z0 = float(np.percentile(roi[valid], percentile))
    if not (min_depth < z0 < max_depth):
        return None
    return z0


def backproject_bbox(
    depth_m: np.ndarray,
    bbox: Tuple[float, float, float, float],
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    roi_ratio: float = 0.6,
    percentile: float = 20.0,
    depth_tolerance: float = 0.35,
    min_depth: float = 0.2,
    max_depth: float = 12.0,
    min_valid_px: int = 12,
    min_cluster_px: int = 10,
) -> Optional[BackprojectionResult]:
    """把 2D 检测框反投影成 3D 物体位姿（相机光学坐标系）。

    针孔模型:
        Z = depth(u, v)
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

    像素范围约定: bbox 为 (x_min, y_min, x_max, y_max)，采样区间按
    [x_min, ceil(x_max)) 处理，即右/下边界向内取整。这是处理浮点检测框的
    标准做法，避免把框外一个像素的背景深度混进来。
    """
    h, w = depth_m.shape[:2]
    x0, y0, x1, y1 = bbox

    # 第一步：鲁棒深度
    z0 = robust_depth_from_bbox(
        depth_m, bbox, roi_ratio, percentile, min_depth, max_depth, min_valid_px
    )
    if z0 is None:
        return None

    # 第二步：在框内（含边界）找所有与 z0 深度一致的像素 —— 深度一致性聚类
    bx0 = int(max(0, math.floor(x0)))
    bx1 = int(min(w, math.ceil(x1)))
    by0 = int(max(0, math.floor(y0)))
    by1 = int(min(h, math.ceil(y1)))
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None

    patch = depth_m[by0:by1, bx0:bx1]
    valid = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    cluster = valid & (np.abs(patch - z0) <= depth_tolerance)
    if int(cluster.sum()) < min_cluster_px:
        # 聚类太小就退化为只用 z0 + 框中心，保证不丢检测
        cluster = valid
        if int(cluster.sum()) < min_valid_px:
            return None

    # 第三步：把聚类像素反投影成 3D 点，求质心
    vs, us = np.nonzero(cluster)          # vs 是行(y)，us 是列(x)
    zs = patch[vs, us].astype(np.float64)
    us = (us + bx0).astype(np.float64)    # 换回整图坐标
    vs = (vs + by0).astype(np.float64)

    xs = (us - cx) * zs / fx
    ys = (vs - cy) * zs / fy

    position = np.array([xs.mean(), ys.mean(), zs.mean()], dtype=np.float64)

    # --- 可见尺寸估计 ---
    # x/y 方向用点云的极差（去掉 2% 离群点），z 方向用深度极差
    def robust_range(a: np.ndarray) -> float:
        if a.size < 4:
            return float(a.max() - a.min()) if a.size else 0.0
        lo, hi = np.percentile(a, [2.0, 98.0])
        return float(hi - lo)

    size = np.array([robust_range(xs), robust_range(ys), robust_range(zs)], dtype=np.float64)

    # 尺寸兜底：点云极差偶尔会退化成 0（物体表面几乎平行于像平面时），
    # 此时用检测框的角点投影换算一个几何估计
    if size[0] < 1e-3:
        size[0] = abs(x1 - x0) * z0 / fx
    if size[1] < 1e-3:
        size[1] = abs(y1 - y0) * z0 / fy

    # --- 朝向：点云在 x-y 平面主方向。单帧不可靠，仅作初值 ---
    yaw = 0.0
    if xs.size >= 6:
        pts = np.stack([xs - xs.mean(), ys - ys.mean()], axis=1)
        cov = pts.T @ pts / max(1, pts.shape[0])
        try:
            evals, evecs = np.linalg.eigh(cov)
            # 取主特征向量，且让 x 分量为正，避免 180 度歧义
            principal = evecs[:, int(np.argmax(evals))]
            if principal[0] < 0:
                principal = -principal
            yaw = float(math.atan2(principal[1], principal[0]))
        except np.linalg.LinAlgError:
            yaw = 0.0

    distance = float(np.linalg.norm(position))
    return BackprojectionResult(
        position=position,
        size=size,
        yaw=yaw,
        distance=distance,
        points_used=int(cluster.sum()),
    )


def depth_to_meters(depth_raw: np.ndarray, encoding: str, depth_scale: float = 0.001) -> np.ndarray:
    """把原始深度图统一成 float32 米制。

    RealSense 的 aligned_depth_to_color 默认是 16UC1 毫米，depth_scale=0.001。
    """
    if encoding == "16UC1":
        return depth_raw.astype(np.float32) * depth_scale
    if encoding == "32FC1":
        return depth_raw.astype(np.float32)
    raise ValueError(f"不支持的深度编码: {encoding}")
