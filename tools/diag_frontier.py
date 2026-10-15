#!/usr/bin/env python3
"""诊断：为什么探索器只找到 4 个 frontier？

不改任何代码，直接拿真实的 /rtabmap/map 喂给真实的 frontier.py，把
detect_frontiers 内部的每一个中间量都数出来，用数据而不是猜测定位瓶颈。

计数项与它可能揭示的问题：
  * frontier 单元总数 / 连通簇数      -> 若簇数≈自由空间块数，说明"环连通"假设成立
  * 通过 min_size 的簇数              -> min_frontier_size 是否砍太狠
  * 通过可达性的簇数                  -> require_reachable 是否误杀
  * 通过 goal_clearance 的簇数        -> 0.25 m 膨胀是否把目标点全否掉
  * 机器人所在自由空间分量            -> robot_label 是否为 0（那会让可达性检查被跳过）
  * 各簇的尺寸与质心                  -> 看质心是否落在自由空间中间（环的形状导致）
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, "/home/jetson/VIO/semantic_ws/src/semantic_rtabmap")
from semantic_rtabmap.frontier import (  # noqa: E402
    GridInfo, detect_frontiers, frontier_mask, inflate, occupancy_arrays,
)

from scipy import ndimage  # noqa: E402


def emit(m):
    print(m)
    sys.stdout.flush()


class Diag(Node):
    def __init__(self):
        super().__init__("frontier_diag")
        self.grid = None
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.RELIABLE
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/rtabmap/map", self._cb, q)

    def _cb(self, m):
        self.grid = m


def main():
    rclpy.init()
    n = Diag()
    t = time.time()
    while time.time() - t < 12 and n.grid is None:
        rclpy.spin_once(n, timeout_sec=0.3)
    if n.grid is None:
        emit("  ✗ 没收到地图")
        return 2

    g = n.grid
    # 字段顺序是 (resolution, origin_x, origin_y, width, height) —— 按 dataclass
    # 声明的顺序传，不是按名字看起来"自然"的顺序。
    info = GridInfo(float(g.info.resolution),
                    float(g.info.origin.position.x), float(g.info.origin.position.y),
                    int(g.info.width), int(g.info.height))
    data = np.asarray(g.data, dtype=np.int16).reshape((info.height, info.width))

    emit("=" * 70)
    emit("  地图: %dx%d @ %.3f m, 原点 (%.3f, %.3f)"
         % (info.width, info.height, info.resolution,
            g.info.origin.position.x, g.info.origin.position.y))
    emit("=" * 70)

    THRESH = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    MINSIZE = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    CLEAR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25
    emit("  参数: occupied_threshold=%d  min_frontier_size=%d  goal_clearance=%.2f"
         % (THRESH, MINSIZE, CLEAR))
    emit("")

    free, occupied, unknown = occupancy_arrays(data, THRESH)
    emit("  ① 栅格分类")
    emit("     空闲 %6d   占据 %6d   未知 %6d  (共 %d)"
         % (free.sum(), occupied.sum(), unknown.sum(), data.size))

    mask = frontier_mask(data, THRESH)
    emit("")
    emit("  ② frontier 掩码")
    emit("     frontier 单元总数: %d" % mask.sum())
    if not mask.any():
        emit("     ✗ 掩码为空 —— 检测在第一步就已失败")
        return 1

    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    sizes = ndimage.sum(mask, labels, range(1, count + 1)).astype(int)
    emit("     连通簇总数(未过滤): %d" % count)
    emit("     簇尺寸: 最大 %d, 中位 %d, 最小 %d"
         % (sizes.max(), int(np.median(sizes)), sizes.min()))
    emit("     尺寸分布:")
    for lo, hi, tag in ((1, 2, "1-2"), (3, 9, "3-9"), (10, 49, "10-49"),
                        (50, 199, "50-199"), (200, 100000, ">=200")):
        c = int(((sizes >= lo) & (sizes <= hi)).sum())
        emit("       %-8s : %3d 个" % (tag, c))

    emit("")
    emit("  ③ 自由空间连通性(决定可达性检查)")
    flabels, fcount = ndimage.label(free, structure=np.ones((3, 3), dtype=int))
    fsizes = ndimage.sum(free, flabels, range(1, fcount + 1)).astype(int)
    emit("     自由空间连通块数: %d" % fcount)
    if fcount:
        big = np.sort(fsizes)[::-1][:8]
        emit("     最大的几块尺寸: %s" % ", ".join(str(int(v)) for v in big))
    emit("     ⚠ 若上一步'连通簇总数'≈此处的块数，即证实：围绕每块自由空间的")
    emit("       frontier 边界是一条连通的环，于是**一块自由空间只产生一个 frontier**。")

    emit("")
    emit("  ④ 机器人在哪一块")
    rx, ry = 0.0, 0.0
    rcx, rcy = info.world_to_cell(rx, ry)
    inside = info.contains_cell(rcx, rcy)
    rl = int(flabels[rcy, rcx]) if inside else -1
    emit("     机器人世界坐标假定 (%.2f, %.2f) -> 栅格 (%d, %d), 在图内=%s"
         % (rx, ry, rcx, rcy, inside))
    emit("     该栅格: 空闲=%s 占据=%s 未知=%s"
         % (bool(free[rcy, rcx]) if inside else "n/a",
            bool(occupied[rcy, rcx]) if inside else "n/a",
            bool(unknown[rcy, rcx]) if inside else "n/a"))
    emit("     free 分量标签 robot_label = %d" % rl)
    if rl == 0:
        emit("     ⚠ robot_label=0 意味着机器人所在栅格**不是空闲**。代码里")
        emit("       `if robot_label > 0:` 不成立 -> 可达性过滤被整个跳过（放宽，不是收紧）。")

    emit("")
    emit("  ⑤ 逐级过滤的实际效果")
    blocked = inflate(occupied, CLEAR, info.resolution)
    emit("     goal_clearance 膨胀: 占据 %d -> 阻塞 %d 个栅格 (%.0f%% 的图)"
         % (occupied.sum(), blocked.sum(), 100.0 * blocked.sum() / data.size))
    free_usable = free & ~blocked
    emit("     可供作为目标点的栅格(空闲且未阻塞): %d (占空闲的 %.0f%%)"
         % (free_usable.sum(), 100.0 * free_usable.sum() / max(1, free.sum())))

    goals = detect_frontiers(data, info, robot_xy=(rx, ry), min_size=MINSIZE,
                             occupied_threshold=THRESH, goal_clearance=CLEAR,
                             require_reachable=True)
    emit("")
    emit("  ⑥ detect_frontiers 的最终产出: %d 个" % len(goals))
    for f in goals:
        emit("     x=%+6.2f y=%+6.2f  size=%5d  score=%+6.2f"
             % (f.centroid[0], f.centroid[1], f.size, f.score))

    emit("")
    emit("  ⑦ 反事实对照(看每个开关砍掉了多少)")
    for tag, kw in (
        ("min_size=1",            dict(min_size=1)),
        ("不要求可达",             dict(min_size=MINSIZE, require_reachable=False)),
        ("clearance=0",           dict(min_size=MINSIZE, goal_clearance=0.0)),
        ("全放开(1/否/0)",         dict(min_size=1, require_reachable=False, goal_clearance=0.0)),
    ):
        got = detect_frontiers(data, info, robot_xy=(rx, ry),
                               occupied_threshold=THRESH, **kw)
        emit("     %-20s -> %d 个" % (tag, len(got)))

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
