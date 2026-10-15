#!/usr/bin/env python3
"""Render the live RTAB-Map occupancy grid for Phase D visual verification.

Produces a PNG showing, in the map frame:
  * the occupancy grid (white free / black occupied / grey unknown) with its
    metric gridlines, so wall positions can be compared against the real room;
  * the robot's current pose (blue arrow) from map->base_fragment TF;
  * the latest /scan beams (red dots) -- if these trace the mapped walls, the
    geometry is self-consistent;
  * the explorer's /exploration/goal (green arrow), the Phase D subject.

Everything is measured, not assumed: the script prints the transform it used and
the scan-vs-map agreement statistic, so the picture can be checked against numbers.

Exits with os._exit(0) after writing. rclpy.shutdown() does NOT interrupt an
active rclpy.spin() -- an earlier probe of mine hung forever for exactly that
reason, so termination here is deliberate and unconditional.
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros

MAP_TOPIC = "/rtabmap/map"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/phase_d_map.png"
DEADLINE_S = 25.0


def latched():
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Collector(Node):
    def __init__(self):
        super().__init__("phase_d_render")
        self.map = None
        self.scan = None
        self.goal = None
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, latched())
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(PoseStamped, "/exploration/goal", self._on_goal, 10)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

    def _on_map(self, m): self.map = m
    def _on_scan(self, m): self.scan = m
    def _on_goal(self, m): self.goal = m


def main():
    rclpy.init()
    n = Collector()
    import time

    # Warm the TF buffer BEFORE collecting anything.
    #
    # The dynamic chain (map -> odom_combined -> base_footprint) arrives on /tf
    # at 60 Hz and fills the buffer almost immediately, but camera_depth_frame
    # hangs off the STATIC chain (base_link -> camera_link -> camera_depth_frame)
    # delivered latched on /tf_static. That one lands later, so a lookup issued
    # right after the node starts finds the robot but reports
    # ConnectivityException for the scan frame -- "no path between frames" --
    # even though the path exists a moment later. tf2_echo shows the same
    # behaviour: it warns once, then resolves.
    t_warm = time.time()
    while time.time() - t_warm < 8.0:
        rclpy.spin_once(n, timeout_sec=0.2)

    def emit(msg):
        sys.stdout.write(msg + "\n")
        sys.stdout.flush()

    t0 = time.time()
    while time.time() - t0 < DEADLINE_S:
        rclpy.spin_once(n, timeout_sec=0.3)
        if n.map is not None and n.scan is not None:
            break

    # os._exit() skips stdio flushing, so every diagnostic below would be lost if
    # we did not flush explicitly. That is exactly what happened on the first run
    # of this script: the PNG appeared but the numbers did not, which read as
    # "the script printed nothing" instead of "the output was buffered away".
    def emit(msg):
        sys.stdout.write(msg + "\n")
        sys.stdout.flush()

    if n.map is None:
        emit("FAIL: no message on %s within %.0fs" % (MAP_TOPIC, DEADLINE_S))
        os._exit(1)

    m = n.map
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    w, h = m.info.width, m.info.height
    grid = np.array(m.data, dtype=np.int8).reshape(h, w)
    emit("map   : %dx%d @ %.3f m, origin (%.3f, %.3f)" % (w, h, res, ox, oy))
    emit("        extent x[%.2f, %.2f]  y[%.2f, %.2f]"
          % (ox, ox + w * res, oy, oy + h * res))
    nfree = int((grid == 0).sum()); nocc = int((grid == 100).sum())
    nunk = int((grid == -1).sum())
    emit("        free %d (%.0f%%), occupied %d (%.0f%%), unknown %d (%.0f%%)"
          % (nfree, 100.0 * nfree / grid.size, nocc, 100.0 * nocc / grid.size,
             nunk, 100.0 * nunk / grid.size))

    # --- TF lookups need a warm-up -------------------------------------------
    # A freshly constructed Buffer is empty. The loop above breaks as soon as the
    # map and scan messages arrive, which can be well under a second after the
    # node is created -- at that point the listener has not yet received the /tf
    # stream, and the lookup raises LookupException ("frame does not exist").
    # The frame is perfectly valid a moment later (tf2_echo shows exactly this:
    # it warns once, then resolves). So retry against a deadline instead of
    # treating the first failure as a fact about the tree.
    def lookup_with_retry(target, source, timeout_s=15.0):
        import time
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout_s:
            try:
                return n.tfbuf.lookup_transform(
                    target, source, rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5))
            except Exception as e:
                last = e
                rclpy.spin_once(n, timeout_sec=0.2)
        raise last

    # ---- robot pose in map frame
    robot_xy = None
    robot_yaw = 0.0
    try:
        tf = lookup_with_retry("map", "base_footprint")
        t = tf.transform.translation; q = tf.transform.rotation
        robot_xy = (t.x, t.y)
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        emit("robot : map->base_footprint  x=%.3f y=%.3f yaw=%.1f deg"
              % (t.x, t.y, math.degrees(robot_yaw)))
    except Exception as e:
        emit("robot : TF lookup failed (%s: %s)" % (type(e).__name__, e))

    # ---- scan in map frame
    scan_pts = []
    if n.scan is not None and robot_xy is not None:
        try:
            stf = lookup_with_retry("map", n.scan.header.frame_id)
            st = stf.transform.translation; sq = stf.transform.rotation
            syaw = math.atan2(2.0 * (sq.w * sq.z + sq.x * sq.y),
                              1.0 - 2.0 * (sq.y * sq.y + sq.z * sq.z))
            for i, r in enumerate(n.scan.ranges):
                if not math.isfinite(r) or r < n.scan.range_min or r > n.scan.range_max:
                    continue
                a = n.scan.angle_min + i * n.scan.angle_increment
                # point in the scan frame (x forward, y left)
                px, py = r * math.cos(a), r * math.sin(a)
                ca, sa = math.cos(syaw), math.sin(syaw)
                scan_pts.append((st.x + px * ca - py * sa, st.y + px * sa + py * ca))
            emit("scan  : %d/%d valid beams, frame=%s"
                  % (len(scan_pts), len(n.scan.ranges), n.scan.header.frame_id))
        except Exception as e:
            emit("scan  : TF lookup failed (%s)" % type(e).__name__)

    # ---- how well does the scan agree with the mapped obstacles?
    agree = None
    if scan_pts and nocc:
        occ_r, occ_c = np.where(grid == 100)
        occ_xy = np.stack([ox + (occ_c + 0.5) * res, oy + (occ_r + 0.5) * res], axis=1)
        d = []
        for sx, sy in scan_pts:
            dd = np.hypot(occ_xy[:, 0] - sx, occ_xy[:, 1] - sy).min()
            d.append(dd)
        d = np.array(d)
        for tol in (0.05, 0.10, 0.20):
            agree = 100.0 * (d <= tol).mean()
            emit("        scan-to-nearest-mapped-obstacle <= %.2f m: %.1f%%" % (tol, agree))

    goal_xy = None
    if n.goal is not None:
        goal_xy = (n.goal.pose.position.x, n.goal.pose.position.y)
        emit("goal  : (%.3f, %.3f) frame=%s"
              % (goal_xy[0], goal_xy[1], n.goal.header.frame_id))

    # ------------------------------------------------ render
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    fig, ax = plt.subplots(figsize=(13, 10), dpi=110)
    cmap = ListedColormap(["#ffffff", "#d9d9d9", "#000000"])  # free, unknown, occupied
    img = np.zeros_like(grid, dtype=float)
    img[grid == -1] = 1.0
    img[grid == 100] = 2.0
    ax.imshow(img, cmap=cmap, origin="lower", interpolation="nearest",
              extent=[ox, ox + w * res, oy, oy + h * res], vmin=0, vmax=2)

    if scan_pts:
        sx = [p[0] for p in scan_pts]; sy = [p[1] for p in scan_pts]
        ax.scatter(sx, sy, s=4, c="#e8000b", marker=".", label="/scan (depth)")

    if robot_xy:
        ax.plot(robot_xy[0], robot_xy[1], "o", ms=13, mfc="#0064ff",
                mec="black", mew=1.2, label="robot (base_footprint)", zorder=6)
        ax.arrow(robot_xy[0], robot_xy[1],
                 0.30 * math.cos(robot_yaw), 0.30 * math.sin(robot_yaw),
                 head_width=0.10, head_length=0.10, fc="#0064ff", ec="black",
                 length_includes_head=True, zorder=7)

    if goal_xy:
        ax.plot(goal_xy[0], goal_xy[1], "*", ms=26, mfc="#00c800", mec="black",
                mew=1.2, label="/exploration/goal", zorder=8)

    ax.set_title("R680 Phase D -- occupancy grid %dx%d @ %.2f m, robot at (%.2f, %.2f)"
                 % (w, h, res, robot_xy[0] if robot_xy else 0,
                    robot_xy[1] if robot_xy else 0))
    ax.set_xlabel("x [m]  (map frame)")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True, which="major", color="#999999", lw=0.5, alpha=0.6)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT)
    emit("wrote %s" % OUT)
    os._exit(0)


if __name__ == "__main__":
    main()
