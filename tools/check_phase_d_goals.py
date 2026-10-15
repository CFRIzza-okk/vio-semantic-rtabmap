#!/usr/bin/env python3
"""Check the explorer's goals against Phase D's acceptance criteria, numerically.

Phase D is "pick goals, do not move". Its pass conditions are about goal QUALITY,
and every one of them is a question about the occupancy grid, so they can be
answered with numbers instead of by eye:

  1. is the goal cell free (not occupied, not unknown)?
  2. does it respect goal_clearance -- i.e. how far is the nearest mapped wall?
  3. is it actually a frontier (adjacent to unknown space)?  A goal in the middle
     of already-known free space gains no information.
  4. is it inside the explored region rather than floating in unmapped space?
  5. is the straight line from the robot to the goal free of mapped obstacles?
     This is a proxy, not a planner: it catches "goal is behind a wall", which is
     the failure mode that would send the robot into a wall in Phase F.
  6. is it within [min_goal_distance, max_goal_distance] of the robot?

Reads the map and the goals from live topics; exits with os._exit and flushes
stdout first (os._exit skips buffer flushing, which silently ate the first run's
output of the sibling renderer).
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf2_ros

# From config/r680_exploration.yaml
GOAL_CLEARANCE = 0.25
MIN_GOAL_DISTANCE = 0.4
MAX_GOAL_DISTANCE = 6.0
MIN_FRONTIER_SIZE = 10

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/phase_d_goals.png"
# How long to keep collecting goals. /exploration/goal is published VOLATILE, so
# a subscriber that arrives late receives nothing -- the goals exist only in the
# log. Collection therefore has to stay open for the whole exploration burst.
# (An earlier version of this script broke out of the loop as soon as ONE goal
# had arrived, so it could never report more than the first.)
COLLECT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0


def emit(m):
    sys.stdout.write(m + "\n")
    sys.stdout.flush()


class N(Node):
    def __init__(self):
        super().__init__("phase_d_goal_check")
        q = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.goals = []
        self.create_subscription(OccupancyGrid, "/rtabmap/map", self._m, q)
        self.create_subscription(PoseStamped, "/exploration/goal", self._g, 10)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

    def _m(self, m): self.map = m
    def _g(self, g):
        p = (g.pose.position.x, g.pose.position.y)
        if p not in self.goals:
            self.goals.append(p)


def main():
    rclpy.init()
    n = N()
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.2)      # warm the TF buffer
    t0 = time.time()
    while time.time() - t0 < COLLECT_S:
        rclpy.spin_once(n, timeout_sec=0.3)
    # keep collecting goals for the whole window; do not break on the first one

    if n.map is None:
        emit("FAIL: no /rtabmap/map"); os._exit(1)

    m = n.map
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    w, h = m.info.width, m.info.height
    g = np.array(m.data, dtype=np.int8).reshape(h, w)

    def cell_xy(c, r):
        return ox + (c + 0.5) * res, oy + (r + 0.5) * res
    def xy_cell(x, y):
        return int((x - ox) / res), int((y - oy) / res)
    def at(c, r):
        if 0 <= c < w and 0 <= r < h:
            return g[r, c]
        return -1

    occ_r, occ_c = np.where(g == 100)
    occ_xy = np.stack([ox + (occ_c + 0.5) * res, oy + (occ_r + 0.5) * res], axis=1) \
        if len(occ_r) else np.zeros((0, 2))
    unk_r, unk_c = np.where(g == -1)
    unk_xy = np.stack([ox + (unk_c + 0.5) * res, oy + (unk_r + 0.5) * res], axis=1) \
        if len(unk_r) else np.zeros((0, 2))

    robot = None
    try:
        tf = n.tfbuf.lookup_transform("map", "base_footprint", rclpy.time.Time(),
                                      rclpy.duration.Duration(seconds=0.5))
        robot = (tf.transform.translation.x, tf.transform.translation.y)
    except Exception as e:
        emit("robot TF unavailable: %s" % type(e).__name__)

    emit("map  : %dx%d @ %.3f m, origin (%.3f, %.3f)" % (w, h, res, ox, oy))
    emit("robot: %s" % ("(%.3f, %.3f)" % robot if robot else "unknown"))
    emit("goals seen on /exploration/goal: %d" % len(n.goals))
    emit("")
    emit("criteria: clearance >= %.2f m, dist in [%.2f, %.2f], frontier >= %d cells"
         % (GOAL_CLEARANCE, MIN_GOAL_DISTANCE, MAX_GOAL_DISTANCE, MIN_FRONTIER_SIZE))
    emit("")

    for i, (gx, gy) in enumerate(n.goals, 1):
        c, r = xy_cell(gx, gy)
        v = at(c, r)
        label = {0: "free", 100: "OCCUPIED", -1: "unknown"}.get(v, "?")
        emit("goal %d: (%.2f, %.2f)  cell=(%d,%d)  occupancy=%s" % (i, gx, gy, c, r, label))

        if len(occ_xy):
            d = np.hypot(occ_xy[:, 0] - gx, occ_xy[:, 1] - gy).min()
            emit("        clearance to nearest wall: %.3f m  -> %s"
                 % (d, "PASS" if d >= GOAL_CLEARANCE else "FAIL (too close)"))
        if len(unk_xy):
            du = np.hypot(unk_xy[:, 0] - gx, unk_xy[:, 1] - gy).min()
            # a frontier goal sits at the boundary of the known region
            emit("        distance to nearest unknown cell: %.3f m  -> %s"
                 % (du, "frontier" if du <= res * 1.5 else "already-explored interior"))

        # frontier size: connected free cells touching the goal, roughly
        free = (g == 0)
        seen = np.zeros_like(free, dtype=bool)
        stack = [(c, r)]
        cnt = 0
        while stack and cnt < 5000:
            cc, rr = stack.pop()
            if not (0 <= cc < w and 0 <= rr < h) or seen[rr, cc] or not free[rr, cc]:
                continue
            seen[rr, cc] = True; cnt += 1
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                stack.append((cc + dc, rr + dr))
        emit("        connected free region around goal: %d cells (min_frontier_size=%d)"
             % (cnt, MIN_FRONTIER_SIZE))

        if robot:
            dist = math.hypot(gx - robot[0], gy - robot[1])
            ok = MIN_GOAL_DISTANCE <= dist <= MAX_GOAL_DISTANCE
            emit("        distance from robot: %.3f m  -> %s" % (dist, "PASS" if ok else "FAIL"))
            # line of sight: sample the segment, count occupied cells crossed
            steps = max(2, int(dist / (res * 0.5)))
            blocked = 0
            for s in range(steps + 1):
                t = s / float(steps)
                px = robot[0] + t * (gx - robot[0]); py = robot[1] + t * (gy - robot[1])
                cc, rr = xy_cell(px, py)
                if at(cc, rr) == 100:
                    blocked += 1
            emit("        straight-line obstacles between robot and goal: %d cell(s) -> %s"
                 % (blocked, "clear line of sight" if blocked == 0
                    else "BLOCKED (goal may be behind a wall)"))
        emit("")

    os._exit(0)


if __name__ == "__main__":
    main()
