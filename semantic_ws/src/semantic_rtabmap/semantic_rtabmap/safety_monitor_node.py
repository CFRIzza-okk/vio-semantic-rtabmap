#!/usr/bin/env python3
"""Safety-input monitors for the R680 exploration stack.

The navigation safety gate (``cmd_vel_safety``) refuses to forward motion
unless ``/safety/obstacle_stop`` and ``/safety/localization_valid`` are both
false and fresh. Nobody published those topics before, so this module supplies
them from the two sensors the rover actually has:

* :class:`ObstacleMonitor` watches the 2D scan produced from the D435 depth
  image and raises ``/safety/obstacle_stop`` when something appears inside a
  narrow forward corridor. This is the last line of defence that does not
  depend on Nav2's costmap being correct.
* :class:`LocalizationMonitor` watches the ``map -> odom`` and
  ``odom -> base_footprint`` transforms. If either goes stale, the robot no
  longer knows where it is and must stop.

Both nodes only ever publish ``Bool`` safety flags. They never publish motion,
so they can be run and observed with the robot stationary.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


class ObstacleMonitor(Node):
    """Raise an obstacle stop when the forward corridor is blocked."""

    def __init__(self) -> None:
        super().__init__("obstacle_monitor")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/safety/obstacle_stop")
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 0.80)
        self.declare_parameter("forward_sector_deg", 60.0)
        self.declare_parameter("min_valid_range", 0.15)
        self.declare_parameter("scan_timeout", 1.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._stopped = False
        self._last_scan = 0.0
        self._min_range: Optional[float] = None
        self._pub = self.create_publisher(
            Bool, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, qos
        )
        self.create_timer(0.1, self._tick)

    def _on_scan(self, msg: LaserScan) -> None:
        sector = math.radians(float(self.get_parameter("forward_sector_deg").value)) / 2.0
        min_valid = float(self.get_parameter("min_valid_range").value)
        best = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < min_valid or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(math.atan2(math.sin(angle), math.cos(angle))) > sector:
                continue
            if best is None or r < best:
                best = float(r)
        self._min_range = best
        self._last_scan = time.monotonic()

    def _tick(self) -> None:
        stop = False
        timeout = float(self.get_parameter("scan_timeout").value)
        if self._last_scan == 0.0 or time.monotonic() - self._last_scan > timeout:
            # No scan means no way to prove the path is clear. Stopping is the
            # safe default: the depth camera is the only obstacle sensor.
            stop = True
        elif self._min_range is not None:
            stop = self._min_range < float(self.get_parameter("stop_distance").value)

        if stop != self._stopped:
            self.get_logger().info(
                "obstacle_stop -> %s (min_range=%s)"
                % (stop, "n/a" if self._min_range is None else "%.2f" % self._min_range)
            )
            self._stopped = stop
        self._pub.publish(Bool(data=stop))


class LocalizationMonitor(Node):
    """Raise a localization-invalid flag when the TF chain goes stale.

    Freshness is measured with the *local monotonic clock at message arrival*,
    never by comparing ``header.stamp`` against ``self.get_clock()``. The TX1
    clock is not synchronised with the Jetson (it has been observed years off),
    so a stamp-based age check would report every relayed transform as stale
    and permanently block all motion.
    """

    def __init__(self) -> None:
        super().__init__("localization_monitor")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom_combined")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("output_topic", "/safety/localization_valid")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("tf_static_topic", "/tf_static")
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("require_map_to_odom", True)

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._arrivals = {}  # (parent, child) -> local monotonic arrival time (dynamic /tf)
        self._static_pairs = set()  # (parent, child) seen on latched /tf_static
        self._valid = False
        self._pub = self.create_publisher(
            Bool, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            TFMessage, str(self.get_parameter("tf_topic").value), self._on_tf, qos
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("tf_static_topic").value),
            self._on_tf_static,
            static_qos,
        )
        self.create_timer(0.1, self._tick)

    @staticmethod
    def _norm(frame: str) -> str:
        return frame[1:] if frame.startswith("/") else frame

    def _on_tf(self, msg: TFMessage) -> None:
        now = time.monotonic()
        for transform in msg.transforms:
            key = (self._norm(transform.header.frame_id), self._norm(transform.child_frame_id))
            self._arrivals[key] = now

    def _on_tf_static(self, msg: TFMessage) -> None:
        """Record latched transforms, which never expire.

        A static transform is published once and stays valid for the lifetime of
        the session, so it must not be judged by arrival time. Doing so makes
        the flag flap to invalid one timeout period after startup, which would
        permanently block motion.
        """
        for transform in msg.transforms:
            self._static_pairs.add(
                (self._norm(transform.header.frame_id), self._norm(transform.child_frame_id))
            )

    def _fresh(self, parent: str, child: str, now: float, timeout: float) -> bool:
        if (parent, child) in self._static_pairs:
            return True
        seen = self._arrivals.get((parent, child))
        return seen is not None and (now - seen) <= timeout

    def _chain_ok(self, parent: str, child: str) -> bool:
        """Confirm the whole chain resolves, not just the two end frames."""
        try:
            self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return True
        except Exception:
            return False

    def _tick(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter("timeout").value)
        odom = str(self.get_parameter("odom_frame").value)
        base = str(self.get_parameter("base_frame").value)
        global_frame = str(self.get_parameter("global_frame").value)

        valid = self._fresh(odom, base, now, timeout)
        if valid and bool(self.get_parameter("require_map_to_odom").value):
            valid = self._fresh(global_frame, odom, now, timeout)
        if valid:
            valid = self._chain_ok(global_frame, base)

        if valid != self._valid:
            self.get_logger().info("localization_valid -> %s" % valid)
            self._valid = valid
        self._pub.publish(Bool(data=valid))


def main(args=None) -> None:
    """Run the obstacle monitor (kept for symmetry with the other nodes)."""
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_localization(args=None) -> None:
    """Run the localization monitor."""
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
