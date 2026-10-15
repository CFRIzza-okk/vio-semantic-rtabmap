"""Tests for the safety monitors that feed the ``cmd_vel_safety`` gate.

The important property here is clock-skew independence: the TX1 clock is not
synchronised with the Jetson, so freshness must be judged by local arrival
time, never by comparing a relayed ``header.stamp`` against the local clock.

Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_safety_monitors.py -v
"""

import math
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from semantic_rtabmap.safety_monitor_node import LocalizationMonitor, ObstacleMonitor

# A stamp from a clock that is years away from the local one.
SKEWED_STAMP = TimeMsg(sec=1693000000, nanosec=0)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def spin_for(node, seconds=0.4):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def make_tf(parent, child, stamp=None):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.header.stamp = stamp if stamp is not None else TimeMsg(sec=0, nanosec=0)
    transform.transform.rotation.w = 1.0
    msg = TFMessage()
    msg.transforms = [transform]
    return msg


# ----------------------------------------------------------------------
# ObstacleMonitor
# ----------------------------------------------------------------------
def make_scan(ranges, angle_increment=math.radians(1.0), range_max=6.0):
    msg = LaserScan()
    msg.angle_min = -math.radians(len(ranges) / 2.0) * angle_increment / angle_increment
    msg.angle_min = -(len(ranges) / 2.0) * angle_increment
    msg.angle_max = (len(ranges) / 2.0) * angle_increment
    msg.angle_increment = angle_increment
    msg.range_min = 0.1
    msg.range_max = range_max
    msg.ranges = [float(r) for r in ranges]
    return msg


def test_obstacle_monitor_stops_without_any_scan(ros):
    node = ObstacleMonitor()
    try:
        published = []
        node.create_subscription(Bool, "/safety/obstacle_stop", published.append, 10)
        spin_for(node, 0.5)
        assert published and published[-1].data is True, (
            "no scan data must be treated as an obstacle"
        )
    finally:
        node.destroy_node()


def test_obstacle_monitor_clears_with_clear_scan(ros):
    node = ObstacleMonitor()
    try:
        published = []
        node.create_subscription(Bool, "/safety/obstacle_stop", published.append, 10)
        node._on_scan(make_scan([2.0] * 181))
        spin_for(node, 0.5)
        assert published and published[-1].data is False
    finally:
        node.destroy_node()


def test_obstacle_monitor_stops_on_close_obstacle(ros):
    node = ObstacleMonitor()
    try:
        published = []
        node.create_subscription(Bool, "/safety/obstacle_stop", published.append, 10)
        ranges = [2.0] * 181
        ranges[90] = 0.20  # directly ahead, inside stop_distance 0.45
        node._on_scan(make_scan(ranges))
        spin_for(node, 0.5)
        assert published and published[-1].data is True
    finally:
        node.destroy_node()


def test_obstacle_monitor_ignores_obstacle_outside_forward_sector(ros):
    node = ObstacleMonitor()
    try:
        published = []
        node.create_subscription(Bool, "/safety/obstacle_stop", published.append, 10)
        ranges = [2.0] * 181
        ranges[0] = 0.10  # ~90 degrees to one side, outside the 50 degree sector
        node._on_scan(make_scan(ranges))
        spin_for(node, 0.5)
        assert published and published[-1].data is False
    finally:
        node.destroy_node()


def test_obstacle_monitor_ignores_infinite_returns(ros):
    node = ObstacleMonitor()
    try:
        published = []
        node.create_subscription(Bool, "/safety/obstacle_stop", published.append, 10)
        node._on_scan(make_scan([float("inf")] * 181))
        spin_for(node, 0.5)
        assert published and published[-1].data is False
    finally:
        node.destroy_node()


def test_obstacle_monitor_goes_stale(ros):
    node = ObstacleMonitor()
    try:
        node.set_parameters([rclpy.parameter.Parameter("scan_timeout", value=0.1)])
        published = []
        node.create_subscription(Bool, "/safety/obstacle_stop", published.append, 10)
        node._on_scan(make_scan([2.0] * 181))
        spin_for(node, 0.5)  # no further scans, so it must fall back to stopping
        assert published and published[-1].data is True
    finally:
        node.destroy_node()


# ----------------------------------------------------------------------
# LocalizationMonitor
# ----------------------------------------------------------------------
def test_localization_invalid_without_tf(ros):
    node = LocalizationMonitor()
    try:
        published = []
        node.create_subscription(Bool, "/safety/localization_valid", published.append, 10)
        spin_for(node, 0.5)
        assert published and published[-1].data is False
    finally:
        node.destroy_node()


def test_localization_accepts_fresh_tf_despite_skewed_stamp(ros):
    """The core regression test for the TX1/Jetson clock skew."""
    node = LocalizationMonitor()
    try:
        # Feed the chain with stamps from a clock years away from the local one.
        node._on_tf(make_tf("map", "odom_combined", SKEWED_STAMP))
        node._on_tf(make_tf("odom_combined", "base_footprint", SKEWED_STAMP))
        now = time.monotonic()
        assert node._fresh("odom_combined", "base_footprint", now, 0.5)
        assert node._fresh("map", "odom_combined", now, 0.5)
    finally:
        node.destroy_node()


def test_localization_rejects_stale_arrival(ros):
    node = LocalizationMonitor()
    try:
        node._on_tf(make_tf("odom_combined", "base_footprint"))
        # Pretend the transform arrived long ago.
        node._arrivals[("odom_combined", "base_footprint")] = time.monotonic() - 10.0
        now = time.monotonic()
        assert not node._fresh("odom_combined", "base_footprint", now, 0.5)
    finally:
        node.destroy_node()


def test_localization_requires_map_to_odom_when_configured(ros):
    node = LocalizationMonitor()
    try:
        assert node.get_parameter("require_map_to_odom").value is True
        node._on_tf(make_tf("odom_combined", "base_footprint"))
        now = time.monotonic()
        assert node._fresh("odom_combined", "base_footprint", now, 0.5)
        assert not node._fresh("map", "odom_combined", now, 0.5)
    finally:
        node.destroy_node()


def test_frame_names_with_leading_slash_are_normalised(ros):
    node = LocalizationMonitor()
    try:
        node._on_tf(make_tf("/odom_combined", "/base_footprint"))
        now = time.monotonic()
        assert node._fresh("odom_combined", "base_footprint", now, 0.5)
    finally:
        node.destroy_node()


def test_static_transform_never_expires(ros):
    """Regression test: a latched static transform must not go stale.

    static_transform_publisher publishes on /tf_static exactly once. Treating it
    like a dynamic transform made localization_valid flap to False one timeout
    period after startup, which would permanently block all motion.
    """
    node = LocalizationMonitor()
    try:
        node._on_tf_static(make_tf("odom_combined", "base_footprint"))
        # Look far into the future: a static transform is still valid.
        future = time.monotonic() + 10_000.0
        assert node._fresh("odom_combined", "base_footprint", future, 0.5)

        node._on_tf_static(make_tf("map", "odom_combined"))
        assert node._fresh("map", "odom_combined", future, 0.5)
    finally:
        node.destroy_node()


def test_dynamic_transform_still_expires(ros):
    """The dynamic path must keep its staleness check."""
    node = LocalizationMonitor()
    try:
        node._on_tf(make_tf("odom_combined", "base_footprint"))
        future = time.monotonic() + 10_000.0
        assert not node._fresh("odom_combined", "base_footprint", future, 0.5)
    finally:
        node.destroy_node()
