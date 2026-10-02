#!/usr/bin/env python3
"""backprojection 模块的单元测试。

不上硬件，用合成深度图验证反投影数学的正确性。
运行: python3 /home/jetson/VIO/tools/test_backprojection.py
"""

import sys

import numpy as np

sys.path.insert(0, "/home/jetson/VIO/semantic_ws/src/semantic_rtabmap")

from semantic_rtabmap.backprojection import (  # noqa: E402
    backproject_bbox,
    depth_to_meters,
    robust_depth_from_bbox,
)

FX = FY = 600.0
CX, CY = 320.0, 240.0
W, H = 640, 480

failures = []


def check(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


def make_scene():
    """构造场景: 一块 2.0m 远的方形物体, 放在 5.0m 的背景前面。

    物体像素范围 u in [200,300], v in [180,260]。
    """
    depth = np.full((H, W), 5.0, dtype=np.float32)
    depth[180:261, 200:301] = 2.0
    return depth


print("=== 1. depth_to_meters 单位换算 ===")
raw = np.full((4, 4), 2000, dtype=np.uint16)
m = depth_to_meters(raw, "16UC1", 0.001)
check("16UC1 毫米->米", np.allclose(m, 2.0), f"得到 {m[0, 0]}")
f = depth_to_meters(np.full((4, 4), 2.5, dtype=np.float32), "32FC1")
check("32FC1 原样透传", np.allclose(f, 2.5), f"得到 {f[0, 0]}")
try:
    depth_to_meters(raw, "mono16")
    check("非法编码应抛异常", False)
except ValueError:
    check("非法编码应抛异常", True)

print("\n=== 2. 鲁棒深度估计 ===")
depth = make_scene()
z = robust_depth_from_bbox(depth, (200, 180, 300, 260), roi_ratio=0.6, percentile=20.0)
check("取到物体深度 2.0m", z is not None and abs(z - 2.0) < 0.05, f"得到 {z}")

# 背景更远时应偏向物体（20 分位数），不能把背景平均进来
check("低分位数偏向前景而非背景", z is not None and z < 2.5, f"得到 {z}")

z_none = robust_depth_from_bbox(
    np.zeros((H, W), dtype=np.float32), (200, 180, 300, 260)
)
check("全零深度返回 None", z_none is None, f"得到 {z_none}")

print("\n=== 3. 3D 反投影位置 ===")
res = backproject_bbox(depth, (200, 180, 300, 260), FX, FY, CX, CY)
check("反投影成功", res is not None)
if res is not None:
    # 期望质心: u 均值 250 -> x=(250-320)*2/600, v 均值 220 -> y=(220-240)*2/600
    exp_x = (250.0 - CX) * 2.0 / FX
    exp_y = (220.0 - CY) * 2.0 / FY
    check(
        "X 坐标正确",
        abs(res.position[0] - exp_x) < 0.01,
        f"期望 {exp_x:.4f} 得到 {res.position[0]:.4f}",
    )
    check(
        "Y 坐标正确",
        abs(res.position[1] - exp_y) < 0.01,
        f"期望 {exp_y:.4f} 得到 {res.position[1]:.4f}",
    )
    check(
        "Z 坐标正确",
        abs(res.position[2] - 2.0) < 0.01,
        f"期望 2.0 得到 {res.position[2]:.4f}",
    )
    check("距离合理", abs(res.distance - np.linalg.norm(res.position)) < 1e-6)
    # 物体宽 100 像素 @2m -> 100*2/600 = 0.3333 m
    check(
        "宽度估计接近真值 0.333m",
        abs(res.size[0] - 100 * 2.0 / FX) < 0.05,
        f"得到 {res.size[0]:.4f}",
    )
    check(
        "高度估计接近真值 0.267m",
        abs(res.size[1] - 80 * 2.0 / FY) < 0.05,
        f"得到 {res.size[1]:.4f}",
    )
    # 像素范围约定为 [x0, x1) 半开区间并向上取整，即 x1=300 覆盖到像素 299。
    # 对 YOLO 输出的浮点框，这个约定等价于"右边界向内取整"，是标准做法。
    check("有效像素数正确", res.points_used == 100 * 80, f"得到 {res.points_used}")

print("\n=== 4. 边界与鲁棒性 ===")
# 深度同为 2m 时，聚类必须能把背景(5m)排除掉
res2 = backproject_bbox(depth, (150, 150, 350, 300), FX, FY, CX, CY, depth_tolerance=0.35)
check("大框中混入背景仍能锁定物体", res2 is not None)
if res2 is not None:
    check(
        "背景未被混入 Z",
        abs(res2.position[2] - 2.0) < 0.05,
        f"得到 {res2.position[2]:.4f}",
    )

# 框完全落在无效区域
res3 = backproject_bbox(
    np.zeros((H, W), dtype=np.float32), (200, 180, 300, 260), FX, FY, CX, CY
)
check("无有效深度返回 None", res3 is None)

# 框超出图像边界不应崩溃
res4 = backproject_bbox(depth, (-50, -50, 700, 520), FX, FY, CX, CY)
check("越界框不崩溃", res4 is not None)

# 极小框
res5 = backproject_bbox(depth, (250, 220, 252, 222), FX, FY, CX, CY)
check("极小框安全返回", True)

# NaN / inf 污染
depth_bad = depth.copy()
depth_bad[190:200, 210:220] = np.nan
depth_bad[230:240, 260:270] = np.inf
res6 = backproject_bbox(depth_bad, (200, 180, 300, 260), FX, FY, CX, CY)
check("NaN/inf 被正确过滤", res6 is not None and np.isfinite(res6.position).all())

print("\n=== 5. 深度学习一致性 ===")
# 相机前移，同一物体距离应变近
depth_near = np.full((H, W), 5.0, dtype=np.float32)
depth_near[180:261, 200:301] = 1.0
res_near = backproject_bbox(depth_near, (200, 180, 300, 260), FX, FY, CX, CY)
check(
    "距离变近时 Z 减小",
    res_near is not None and abs(res_near.position[2] - 1.0) < 0.01,
    f"得到 {res_near.position[2] if res_near else None}",
)

print("\n" + "=" * 46)
if failures:
    print(f"结果: {len(failures)} 项失败 -> {failures}")
    sys.exit(1)
print("结果: 全部通过")
