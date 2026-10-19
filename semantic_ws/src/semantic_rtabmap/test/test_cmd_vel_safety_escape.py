"""Guards for the obstacle deadlock fix in ``cmd_vel_safety``.

Why these exist
---------------
``obstacle_stop`` used to zero the *entire* Twist, ``angular.z`` included. The
rover can therefore never turn out of the 0.45 m forward dead zone, and neither
can Nav2's own spin/backup recovery, because all of it arrives through this
node. Measured on the robot: the second exploration run locked up at 66 s,
produced 5 consecutive goal timeouts and ended in ABORTED, and the documented
remedy was for a human to walk over and move the car.

The fix lets a latched obstacle be answered with **rotation only**. The
property that makes that acceptable is narrow and must never erode:

  * while ``obstacle_stop`` is latched, ``linear.x`` and ``linear.y`` are 0 --
    the rover cannot translate into anything the forward sector cannot see;
  * every hard stop (e-stop, invalid localization, stale command) still zeroes
    **everything**, rotation included, and outranks the obstacle branch.

Each test below fails if that erodes, and the priority tests exist because the
original branch order put the obstacle first -- which, with escape rotation
added, would have let an e-stopped or unlocalized robot spin in place.

Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_cmd_vel_safety_escape.py -v
"""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from semantic_rtabmap.cmd_vel_safety_node import CmdVelSafetyNode, TimedFlag


@pytest.fixture
def make_node():
    created = []

    def factory(**params):
        args = ["--ros-args"]
        for key, value in params.items():
            args += ["-p", "%s:=%s" % (key, value)]
        rclpy.init(args=args)
        node = CmdVelSafetyNode()
        created.append(node)
        return node

    yield factory

    for node in created:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_for(node, seconds=0.3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def run_one_tick(node, *, cmd=None, obstacle=None, estop=None, localized=None, stale=False):
    """Drive the node into a known state and collect what it publishes.

    Returns ``(twist, reason)`` from the last messages seen.
    """
    received = []
    reasons = []
    sub = node.create_subscription(Twist, "/cmd_vel_safe", received.append, 10)
    rsub = node.create_subscription(String, "/safety/stop_reason", reasons.append, 10)
    try:
        if cmd is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd
            node._input = msg
            # A command is only usable while fresh, so stamp it -- unless the
            # test is specifically about staleness.
            node._input_ns = node._now() - (int(2e9) if stale else 0)
        if obstacle is not None:
            node._obstacle = TimedFlag(obstacle, node._now())
        if estop is not None:
            node._estop = TimedFlag(estop, node._now())
        if localized is not None:
            node._localized = TimedFlag(localized, node._now())
        spin_for(node, 0.3)
    finally:
        node.destroy_subscription(sub)
        node.destroy_subscription(rsub)
    assert received, "cmd_vel_safety published nothing"
    return received[-1], (reasons[-1].data if reasons else None)


# ----------------------------------------------------------------------
# the fix itself
# ----------------------------------------------------------------------
def test_obstacle_allows_escape_rotation_but_never_translation(make_node):
    node = make_node(obstacle_escape_rotation=True, obstacle_escape_angular_z=0.30)
    twist, reason = run_one_tick(
        node, cmd=(0.20, 0.08, 0.50), obstacle=True, localized=True
    )
    assert twist.linear.x == 0.0, "obstacle latched must never pass forward motion"
    assert twist.linear.y == 0.0, "obstacle latched must never pass lateral motion"
    assert twist.angular.z == pytest.approx(0.30), "escape yaw must pass, capped"
    assert reason == "obstacle_stop_escape_rotation"


def test_escape_rotation_can_be_turned_off(make_node):
    node = make_node(obstacle_escape_rotation=False)
    twist, reason = run_one_tick(
        node, cmd=(0.20, 0.0, 0.50), obstacle=True, localized=True
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert reason == "obstacle_stop"


def test_escape_rotation_is_capped(make_node):
    node = make_node(obstacle_escape_rotation=True, obstacle_escape_angular_z=0.30)
    twist, _ = run_one_tick(node, cmd=(0.0, 0.0, 5.0), obstacle=True, localized=True)
    assert twist.angular.z == pytest.approx(0.30)


def test_escape_rotation_handles_negative_yaw(make_node):
    node = make_node(obstacle_escape_rotation=True, obstacle_escape_angular_z=0.30)
    twist, _ = run_one_tick(node, cmd=(0.0, 0.0, -0.50), obstacle=True, localized=True)
    assert twist.angular.z == pytest.approx(-0.30)


# ----------------------------------------------------------------------
# the hard stops must still win -- this is the part that must not erode
# ----------------------------------------------------------------------
def test_estop_zeroes_rotation_even_with_escape_enabled(make_node):
    node = make_node(obstacle_escape_rotation=True)
    twist, reason = run_one_tick(
        node, cmd=(0.2, 0.0, 0.5), obstacle=True, estop=True, localized=True
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert reason == "emergency_stop"


def test_invalid_localization_zeroes_rotation_even_with_escape_enabled(make_node):
    node = make_node(obstacle_escape_rotation=True, require_localization=True)
    twist, reason = run_one_tick(
        node, cmd=(0.2, 0.0, 0.5), obstacle=True, localized=False
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert reason == "localization_invalid_or_timeout"


def test_stale_command_is_not_reused_for_escape_rotation(make_node):
    """A rotation the planner has stopped repeating must not keep being applied."""
    node = make_node(obstacle_escape_rotation=True, localized=True)
    twist, reason = run_one_tick(
        node, cmd=(0.2, 0.0, 0.5), obstacle=True, localized=True, stale=True
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert reason == "cmd_vel_timeout"


# ----------------------------------------------------------------------
# unchanged behaviour on a clear path
# ----------------------------------------------------------------------
def test_clear_path_still_passes_motion(make_node):
    node = make_node(obstacle_escape_rotation=True, localized=True)
    twist, reason = run_one_tick(
        node, cmd=(0.20, 0.05, 0.40), obstacle=False, localized=True
    )
    assert twist.linear.x == pytest.approx(0.20)
    assert twist.linear.y == pytest.approx(0.05)
    assert twist.angular.z == pytest.approx(0.40)
    assert reason == "ok"


def test_clear_path_still_clamps_to_limits(make_node):
    node = make_node(max_linear_x=0.10, max_linear_y=0.02, max_angular_z=0.30, localized=True)
    twist, _ = run_one_tick(node, cmd=(0.90, 0.90, 0.90), obstacle=False, localized=True)
    assert twist.linear.x == pytest.approx(0.10)
    assert twist.linear.y == pytest.approx(0.02)
    assert twist.angular.z == pytest.approx(0.30)
