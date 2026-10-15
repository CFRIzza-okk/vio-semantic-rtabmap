"""Node-level tests for the exploration state machine.

These run the real :class:`ExplorationNode` inside ``rclpy`` with a synthetic
occupancy grid and an injected robot pose, so the timers, guards, publishers
and the dry-run safety path are all exercised. Nav2, TF and the physical robot
are never involved.

Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_exploration_node.py -v
"""

import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from std_msgs.msg import String

from semantic_rtabmap.exploration_node import (
    STATE_COMPLETE,
    STATE_EXPLORING,
    STATE_IDLE,
    ExplorationNode,
)

RES = 0.1
UNKNOWN = -1


def make_grid(width=40, height=40, free_rows=None):
    """Build an OccupancyGrid with an unexplored border and a free middle."""
    msg = OccupancyGrid()
    msg.info.resolution = RES
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    data = np.full((height, width), UNKNOWN, dtype=np.int8)
    if free_rows is None:
        free_rows = (10, 30)
    data[free_rows[0]:free_rows[1], 10:30] = 0
    msg.data = data.reshape(-1).tolist()
    return msg


def make_all_free_grid(width=40, height=40):
    msg = make_grid(width, height)
    msg.data = [0] * (width * height)
    return msg


@pytest.fixture
def node_factory():
    """Create ExplorationNode instances with extra parameter overrides."""
    created = []

    def factory(**params):
        args = ["--ros-args"]
        for key, value in params.items():
            args += ["-p", "%s:=%s" % (key, value)]
        rclpy.init(args=args)
        node = ExplorationNode()
        created.append(node)
        return node

    yield factory

    for node in created:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_for(node, seconds=0.6):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def test_starts_idle_and_does_not_move_by_default(node_factory):
    node = node_factory()
    assert node._state == STATE_IDLE
    assert node.get_parameter("dry_run").value is True
    assert node.get_parameter("autostart").value is False


def test_autostart_enters_exploring(node_factory):
    node = node_factory(autostart=True, dry_run=True, replan_period=0.1)
    assert node._state == STATE_EXPLORING


def test_goal_is_selected_from_synthetic_map_in_dry_run(node_factory):
    node = node_factory(autostart=True, dry_run=True, replan_period=0.1)
    goals = []
    node.create_subscription(PoseStamped, "/exploration/goal", goals.append, 10)

    node._on_map(make_grid())
    # Inject a pose instead of requiring TF.
    node._robot_pose = lambda: (1.0, 1.5, 0.0)
    spin_for(node)

    assert goals, "explorer should publish a goal for a frontier"
    goal = goals[-1]
    assert goal.header.frame_id == "map"
    assert 0.0 <= goal.pose.position.x <= 4.0
    assert 0.0 <= goal.pose.position.y <= 4.0
    # Dry run must never consume the Nav2 action server.
    assert node._goal_handle is None
    assert node._completed_goals == 0
    assert node._state == STATE_EXPLORING


def test_dry_run_blacklists_chosen_goal(node_factory):
    node = node_factory(autostart=True, dry_run=True, replan_period=0.1)
    node._on_map(make_grid())
    node._robot_pose = lambda: (1.0, 1.5, 0.0)
    spin_for(node, 0.8)
    assert len(node._blacklist) >= 1
    assert node._failed_goals == 0  # blacklisting in dry run is not a failure


def test_complete_after_repeated_empty_cycles(node_factory):
    node = node_factory(
        autostart=True,
        dry_run=True,
        replan_period=0.05,
        complete_confirmations=2,
        min_frontier_size=10,
    )
    # A fully free map has no frontier at all.
    node._on_map(make_all_free_grid())
    node._robot_pose = lambda: (2.0, 2.0, 0.0)
    spin_for(node, 1.0)
    assert node._state == STATE_COMPLETE
    assert "no reachable frontier" in node._message


def test_time_budget_aborts_exploration(node_factory):
    node = node_factory(
        autostart=True, dry_run=True, replan_period=10.0, max_exploration_time=0.0
    )
    spin_for(node, 0.5)
    assert node._state == "ABORTED"
    assert "time budget" in node._message


def test_consecutive_failure_limit_aborts(node_factory):
    node = node_factory(
        autostart=True,
        dry_run=True,
        replan_period=10.0,
        max_consecutive_failures=1,
    )
    node._consecutive_failures = 1
    spin_for(node, 0.5)
    assert node._state == "ABORTED"
    assert "consecutive goal failures" in node._message


def test_low_battery_aborts_exploration(node_factory):
    node = node_factory(
        autostart=True, dry_run=True, replan_period=10.0, min_battery=11.0
    )
    node._battery = 10.0
    spin_for(node, 0.5)
    assert node._state == "ABORTED"
    assert "battery" in node._message


def test_missing_map_keeps_waiting(node_factory):
    node = node_factory(autostart=True, dry_run=True, replan_period=0.05)
    node._robot_pose = lambda: (1.0, 1.0, 0.0)
    spin_for(node, 0.4)
    assert node._state == STATE_EXPLORING
    assert node._message == "waiting for /map"


def test_missing_localization_keeps_waiting(node_factory):
    node = node_factory(
        autostart=True, dry_run=True, replan_period=0.05, require_localization=True
    )
    node._on_map(make_grid())
    node._robot_pose = lambda: None
    spin_for(node, 0.4)
    assert node._state == STATE_EXPLORING
    assert "localization" in node._message


def test_status_payload_is_published(node_factory):
    node = node_factory(autostart=True, dry_run=True, replan_period=0.1)
    states, statuses = [], []
    node.create_subscription(String, "/exploration/state", states.append, 10)
    node.create_subscription(String, "/exploration/status", statuses.append, 10)
    node._on_map(make_grid())
    node._robot_pose = lambda: (1.0, 1.5, 0.0)
    spin_for(node, 1.5)
    assert states
    assert statuses
    import json

    payload = json.loads(statuses[-1].data)
    for key in ("state", "dry_run", "goals_completed", "blacklisted", "map_age"):
        assert key in payload


def test_stop_service_returns_to_idle(node_factory):
    from std_srvs.srv import Trigger

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0)
    client = node.create_client(Trigger, "/exploration/stop")
    assert client.wait_for_service(timeout_sec=1.0)
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < deadline:
        spin_for(node, 0.05)
    assert future.result() is not None
    assert future.result().success
    assert node._state == STATE_IDLE


def test_rejects_double_start(node_factory):
    from std_srvs.srv import Trigger

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0)
    client = node.create_client(Trigger, "/exploration/start")
    assert client.wait_for_service(timeout_sec=1.0)
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < deadline:
        spin_for(node, 0.05)
    result = future.result()
    assert result is not None and not result.success
    assert "already exploring" in result.message


def test_cancelling_a_goal_invalidates_its_callbacks(node_factory):
    """A cancelled goal must not resurrect EXPLORING or count as a failure.

    Regression test: cancelling a Nav2 goal still yields a CANCELED result, and
    that result used to overwrite the operator's PAUSED state.
    """
    node = node_factory(autostart=True, dry_run=True, replan_period=10.0)
    node._active_goal = None
    node._goal_handle = None
    token_before = node._goal_seq

    failed_before = node._failed_goals
    node._cancel_goal()
    assert node._goal_seq == token_before + 1, "cancel must invalidate the token"

    # Simulate the late arrival of the cancelled goal's result.
    class _FakeResult:
        status = 5  # CANCELED

    class _FakeFuture:
        def result(self):
            return _FakeResult()

    node._set_state("PAUSED", "paused by operator")
    node._on_goal_result(_FakeFuture(), token_before)
    assert node._state == "PAUSED", "stale result must not change the state"
    assert node._failed_goals == failed_before, "stale result must not count a failure"


def test_current_goal_result_is_still_processed(node_factory):
    """A result carrying the live token must still be handled normally."""
    node = node_factory(autostart=True, dry_run=True, replan_period=10.0)

    class _FakeResult:
        status = 4  # SUCCEEDED

    class _FakeFuture:
        def result(self):
            return _FakeResult()

    completed_before = node._completed_goals
    node._on_goal_result(_FakeFuture(), node._goal_seq)
    assert node._completed_goals == completed_before + 1
    assert node._consecutive_failures == 0
