"""The relay's /tf_static publisher must stay TRANSIENT_LOCAL (latched).

Why this test exists
--------------------
Static transforms are published once, by definition, and the whole tf2 contract
is that a node starting later still receives them.  That only works if the
publisher uses TRANSIENT_LOCAL durability.

The relay used to do ``create_publisher(TFMessage, "/tf_static", 10)``, whose
default durability is VOLATILE.  The failure mode is nastier than "a warning in
the log": in ROS 2 a VOLATILE publisher and a TRANSIENT_LOCAL subscriber are an
*incompatible* QoS pair, so the connection is refused outright and the subscriber
receives nothing at all.  Measured on the real robot, ``ros2 run tf2_ros
tf2_echo`` reported::

    New publisher discovered on topic '/tf_static', offering incompatible QoS.
    No messages will be sent to it. Last incompatible policy: DURABILITY_QOS_POLICY

and a ``--qos-durability transient_local`` subscriber saw zero messages.  Both
tf2_ros and this package's safety_monitor_node subscribe with TRANSIENT_LOCAL, so
silently flipping this back to VOLATILE would take out static TF for every
consumer at once, while still looking healthy in ``ros2 topic list``.

/tf (dynamic) is deliberately NOT part of this: it must stay VOLATILE.
"""
import os
import sys

import pytest

rclpy = pytest.importorskip("rclpy")


def _relay_module():
    """Import the node module without needing a running relay."""
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    if pkg not in sys.path:
        sys.path.insert(0, pkg)
    from semantic_rtabmap import ros2_relay_node
    return ros2_relay_node


def test_tf_static_qos_is_transient_local():
    from rclpy.qos import DurabilityPolicy
    mod = _relay_module()
    assert mod.TF_STATIC_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL, (
        "the /tf_static publisher must be latched; VOLATILE makes it QoS-"
        "incompatible with every tf2 subscriber, which then receive nothing"
    )


def test_tf_static_qos_is_reliable():
    """tf2 relies on RELIABLE; BEST_EFFORT would silently drop transforms."""
    from rclpy.qos import ReliabilityPolicy
    mod = _relay_module()
    assert mod.TF_STATIC_QOS.reliability == ReliabilityPolicy.RELIABLE


def test_tf_static_qos_keeps_a_history():
    """depth must be >= 1 or the latched sample has nowhere to live."""
    mod = _relay_module()
    assert mod.TF_STATIC_QOS.depth >= 1


def test_live_publisher_really_uses_the_latched_profile():
    """Guard the wiring, not just the constant.

    The constants above could be right while ``create_publisher`` still passes a
    bare integer, which is exactly how this bug was introduced.
    """
    from rclpy.qos import DurabilityPolicy

    mod = _relay_module()
    # Bind a scratch port: the real relay may well be running on 19001 (and this
    # test must not steal packets from it).
    rclpy.init(args=["--ros-args", "-p", "rx_port:=19999", "-p", "tx_port:=19998"])
    node = None
    try:
        node = mod.Relay()
        got = node.tf_static_pub.qos_profile.durability
        assert got == DurabilityPolicy.TRANSIENT_LOCAL, (
            "Relay published /tf_static with durability=%s; a bare integer depth "
            "gives VOLATILE and breaks late subscribers" % (got,)
        )
        # The dynamic topic must NOT have been dragged along with it.
        assert node.tf_pub.qos_profile.durability == DurabilityPolicy.VOLATILE, (
            "/tf is a dynamic topic and must stay VOLATILE"
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
