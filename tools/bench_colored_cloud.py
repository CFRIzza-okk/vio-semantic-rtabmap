#!/usr/bin/env python3
"""验证 /semantics/colored_cloud 向量化实现的正确性与性能。

背景
----
detector_node._publish_colored_cloud 的早期版本用 Python 双层循环遍历整幅
深度图（640x480、stride=2 共 76800 次迭代），实测单帧约 350 ms，把语义节点
从配置的 5 Hz 压到 2.4 Hz。现改为纯 numpy 向量化。

本脚本做两件事
--------------
1. 正确性：把向量化版本的输出与旧版逐点对比（点坐标 + RGB 打包值），
   确保重写没有改变行为（尤其是 PointCloud2 rgb 字段的字节序）。
2. 性能：测量两者单帧耗时，给出提速倍数。

用法
----
    python3 ~/VIO/tools/bench_colored_cloud.py
"""
import math
import struct
import sys
import time
import types

import numpy as np

sys.path.insert(0, "/home/jetson/VIO/semantic_ws/src/semantic_rtabmap")

from sensor_msgs.msg import PointField  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from semantic_rtabmap.detector_node import SemanticDetector, class_color  # noqa: E402

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]


# ----------------------------------------------------------------------
# 旧实现（纯 Python 双层循环），逐行照抄自重写前的代码，用于对照
# ----------------------------------------------------------------------
def old_impl(depth_m, regions, min_depth, max_depth, depth_tol,
             fx, fy, cx, cy, stride):
    h, w = depth_m.shape[:2]
    points = []
    for v in range(0, h, stride):
        for u in range(0, w, stride):
            z = float(depth_m[v, u])
            if not (min_depth < z < max_depth) or not math.isfinite(z):
                continue
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            color = None
            for x0, y0, x1, y1, z0, cid in regions:
                if x0 <= u <= x1 and y0 <= v <= y1 and abs(z - z0) <= depth_tol:
                    color = class_color(cid)
                    break
            if color is None:
                continue
            r = int(max(0, min(255, round(color.r * 255.0))))
            g = int(max(0, min(255, round(color.g * 255.0))))
            b = int(max(0, min(255, round(color.b * 255.0))))
            rgba = (255 << 24) | (b << 16) | (g << 8) | r
            rgb_float = struct.unpack("f", struct.pack("I", rgba))[0]
            points.append((x, y, z, rgb_float))
    return points


# ----------------------------------------------------------------------
# 桩对象：把真实方法绑到一个轻量对象上，避免启动完整 ROS 节点
# ----------------------------------------------------------------------
class Stub:
    pass


class CapturePub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def make_stub(min_depth=0.2, max_depth=12.0, depth_tol=0.35, stride=2):
    s = Stub()
    s.cloud_stride = stride
    s.min_depth = min_depth
    s.max_depth = max_depth
    s.depth_tol = depth_tol
    s.camera_frame = "camera_color_optical_frame"
    s.pub_colored_cloud = CapturePub()
    s._publish_colored_cloud = types.MethodType(
        SemanticDetector._publish_colored_cloud, s
    )
    return s


def read_back(msg):
    """从 PointCloud2 取回 (N,4) float32 数组。"""
    pts = point_cloud2.read_points(
        msg, field_names=("x", "y", "z", "rgb"), skip_nans=False
    )
    arr = np.asarray(list(pts))
    if arr.dtype.names:  # read_points 返回结构化数组
        return np.column_stack(
            [arr[n] for n in ("x", "y", "z", "rgb")]
        ).astype(np.float32)
    return np.asarray(arr, dtype=np.float32).reshape(-1, 4)


def synthetic_scene(h=480, w=640, seed=0):
    """构造一幅带前景物体与背景的合成深度图。"""
    rng = np.random.default_rng(seed)
    depth = np.full((h, w), 3.0, dtype=np.float32)  # 背景墙 3 m
    depth += rng.normal(0.0, 0.01, size=(h, w)).astype(np.float32)

    # 物体 A（近，0.8 m），物体 B（远，1.6 m）
    depth[120:300, 80:260] = 0.8 + rng.normal(0, 0.005, size=(180, 180))
    depth[200:380, 380:560] = 1.6 + rng.normal(0, 0.01, size=(180, 180))

    # 制造无效像素（0）与 NaN，验证掩码逻辑
    depth[0:10, 0:10] = 0.0
    depth[5:8, 5:8] = np.nan
    return depth


def synthetic_regions():
    # (x0, y0, x1, y1, z0, class_id) —— 与 detector_node 里 append 的顺序一致
    return [
        (80.0, 120.0, 260.0, 300.0, 0.8, 39),    # bottle
        (380.0, 200.0, 560.0, 380.0, 1.6, 56),   # chair
        (100.0, 130.0, 200.0, 250.0, 0.8, 39),   # 与前一个重叠，测"取第一个"
        (0.0, 0.0, 30.0, 30.0, 5.0, 0),          # 无有效深度区域
    ]


def main():
    fx, fy, cx, cy = 605.0, 605.0, 320.0, 240.0
    depth = synthetic_scene()
    regions = synthetic_regions()

    print("=" * 66)
    print("  正确性：向量化版本 vs 旧版（逐点对比）")
    print("=" * 66)

    stub = make_stub()
    stub._publish_colored_cloud(None, depth, Header().stamp,
                                fx, fy, cx, cy, regions)
    new_msg = stub.pub_colored_cloud.msgs[-1]
    new_pts = read_back(new_msg)

    old_pts = np.array(
        old_impl(depth, regions, stub.min_depth, stub.max_depth,
                 stub.depth_tol, fx, fy, cx, cy, stub.cloud_stride),
        dtype=np.float32,
    ).reshape(-1, 4)

    print(f"  旧版点数 = {len(old_pts)}")
    print(f"  新版点数 = {len(new_pts)}")

    ok = True
    if len(old_pts) != len(new_pts):
        print("  [失败] 点数不一致")
        ok = False
    else:
        # 坐标：允许 float32/float64 运算顺序带来的极小差异
        for col, name in enumerate(("x", "y", "z")):
            same = np.allclose(old_pts[:, col], new_pts[:, col],
                               rtol=1e-5, atol=1e-6)
            print(f"  {name} 一致: {same}")
            ok &= same
        # rgb：按位重解释，必须完全相等（否则颜色会出错）
        rgb_same = np.array_equal(
            old_pts[:, 3].view(np.uint32), new_pts[:, 3].view(np.uint32)
        )
        print(f"  rgb 打包值完全一致: {rgb_same}")
        ok &= rgb_same

    # 额外校验：rgb 字节序是否符合 0x00RRGGBB
    first = new_pts[0, 3].view(np.uint32) if len(new_pts) else np.uint32(0)
    r = int(first & 0xFF)
    g = int((first >> 8) & 0xFF)
    b = int((first >> 16) & 0xFF)
    a = int((first >> 24) & 0xFF)
    print(f"  首点解包 R={r} G={g} B={b} A={a}  "
          f"(class_color 类别 {int(regions[0][5])})")
    c0 = class_color(int(regions[0][5]))
    exp = (min(255, max(0, int(round(c0.r * 255)))),
           min(255, max(0, int(round(c0.g * 255)))),
           min(255, max(0, int(round(c0.b * 255)))))
    print(f"  期望 R,G,B = {exp}")
    rgb_ok = (r, g, b) == exp
    print(f"  字节序正确: {rgb_ok}")
    ok &= rgb_ok

    # 空 regions / 无有效深度两条早退分支
    stub2 = make_stub()
    stub2._publish_colored_cloud(None, depth, Header().stamp,
                                 fx, fy, cx, cy, [])
    n_empty = read_back(stub2.pub_colored_cloud.msgs[-1]).shape[0]
    print(f"  空 regions -> 点数 {n_empty} (期望 0)")
    ok &= (n_empty == 0)

    stub3 = make_stub()
    stub3._publish_colored_cloud(None, np.zeros((480, 640), dtype=np.float32),
                                 Header().stamp, fx, fy, cx, cy, regions)
    n_zero = read_back(stub3.pub_colored_cloud.msgs[-1]).shape[0]
    print(f"  全零深度 -> 点数 {n_zero} (期望 0)")
    ok &= (n_zero == 0)

    print()
    print("=" * 66)
    print("  性能：单帧耗时")
    print("=" * 66)

    stub = make_stub()
    n_rep = 3

    # 预热
    stub._publish_colored_cloud(None, depth, Header().stamp,
                                fx, fy, cx, cy, regions)

    t0 = time.perf_counter()
    for _ in range(n_rep):
        stub._publish_colored_cloud(None, depth, Header().stamp,
                                    fx, fy, cx, cy, regions)
    t_new = (time.perf_counter() - t0) / n_rep

    t0 = time.perf_counter()
    old_impl(depth, regions, stub.min_depth, stub.max_depth,
             stub.depth_tol, fx, fy, cx, cy, stub.cloud_stride)
    t_old = time.perf_counter() - t0

    print(f"  旧版（Python 双层循环）: {t_old * 1000:8.1f} ms/帧")
    print(f"  新版（numpy 向量化）   : {t_new * 1000:8.1f} ms/帧")
    print(f"  提速倍数              : {t_old / t_new:8.1f} x")
    print()
    print(f"  结论: {'全部通过 ✓' if ok else '存在失败项 ✗'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
