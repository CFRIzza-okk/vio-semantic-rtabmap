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
    STATE_DRY_RUN_DONE,
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
    # The verdict must carry the counts that produced it. On 2026-10-18 the log
    # showed one goal driven and then COMPLETE while the two intermediate empty
    # rounds left no trace, so "the map had no frontier" and "every frontier was
    # filtered out" could not be told apart -- and they need opposite fixes.
    assert "last check:" in node._message
    assert "mask=0" in node._message
    assert "clusters=0" in node._message


def test_frontier_check_stats_reach_the_node(node_factory):
    """The counts must survive the trip from the policy into the node.

    The COMPLETE message is assembled in a later cycle than the check that
    produced the numbers, so the node has to keep them; a pure-function test
    alone would not catch the dict being dropped on the floor.
    """
    node = node_factory(
        autostart=True,
        dry_run=True,
        replan_period=0.05,
        complete_confirmations=2,
        min_frontier_size=10,
    )
    node._on_map(make_all_free_grid())
    node._robot_pose = lambda: (2.0, 2.0, 0.0)
    spin_for(node, 1.0)

    assert node._last_frontier_stats, "the node must keep the last check's counts"
    assert node._last_frontier_stats["mask_cells"] == 0
    assert node._last_frontier_stats["kept"] == 0
    assert "selected" in node._last_frontier_stats


def test_dry_run_sweep_reports_dry_run_done_not_complete(node_factory):
    """A dry-run sweep that runs out of candidates is NOT a completed survey.

    The sweep blacklists each goal it inspects so the next cycle evaluates a
    different candidate. Once every candidate is blacklisted, empty cycles pile
    up and the node used to announce COMPLETE / "no reachable frontier" -- a
    claim about the world that nothing had established, because not a single
    goal was driven. It also read as a failure during the first Phase F
    rehearsal. It must be reported as its own state instead.

    The map here deliberately HAS a frontier, which is what separates this case
    from test_complete_after_repeated_empty_cycles above (no frontier at all,
    where COMPLETE is the truthful answer).
    """
    node = node_factory(
        autostart=True,
        dry_run=True,
        replan_period=0.05,
        complete_confirmations=2,
        min_frontier_size=10,
    )
    node._on_map(make_grid())  # free middle -> at least one frontier
    # Deliberately OFF the centre of the free block. A frontier is the closed
    # ring around a free region, so for a symmetric block the ring's centroid
    # lands on the block centre -- if the robot stands there too, the goal is
    # rejected by min_goal_distance (0.4 m) and the cycle reports "no frontier"
    # even though the whole ring is walkable. Keeping the robot off-centre is
    # what makes this test exercise the dry-run sweep rather than that effect.
    node._robot_pose = lambda: (1.5, 1.5, 0.0)
    spin_for(node, 1.5)

    assert node._dry_run_seen >= 1, "the sweep must have inspected a candidate"
    assert node._state == STATE_DRY_RUN_DONE
    assert node._state != STATE_COMPLETE
    assert "nothing was driven" in node._message
    # Nothing may have been sent to Nav2, and no goal may count as visited.
    assert node._goal_handle is None
    assert node._completed_goals == 0


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


# ----------------------------------------------------------------------
# 反复失败的区域 -> "无法探索"
# ----------------------------------------------------------------------
def test_repeated_failures_mark_the_area_unexplorable(node_factory):
    from semantic_rtabmap.frontier import Frontier

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0,
                        unexplorable_failures=2, unexplorable_radius=1.5)
    goal = Frontier(centroid=(3.0, 3.0), size=10, distance=1.0, score=1.0)

    node._active_goal = goal
    node._reject_active_goal("timeout")
    assert len(node._unexplorable) == 0, "一次失败不该就判死刑"

    node._active_goal = Frontier(centroid=(3.05, 3.02), size=10, distance=1.0, score=1.0)
    node._reject_active_goal("timeout")
    assert len(node._unexplorable) == 1, "同一地区第二次失败必须被标注"
    assert "UNEXPLORABLE" in node._message


def test_failures_far_apart_are_counted_separately(node_factory):
    from semantic_rtabmap.frontier import Frontier

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0,
                        unexplorable_failures=2, unexplorable_radius=1.5)
    for x in (1.0, 9.0):
        for _ in range(2):
            node._active_goal = Frontier(
                centroid=(x, 1.0), size=10, distance=1.0, score=1.0
            )
            node._reject_active_goal("timeout")
    # 两处各自累计到 2，所以两处都被标注 —— 而不是合并成一处。
    assert len(node._unexplorable) == 2


def test_goal_result_failure_also_marks_the_area(node_factory):
    """走 Nav2 结果回调的那条失败路径也必须计数，不能只算超时路径。"""
    from semantic_rtabmap.frontier import Frontier

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0,
                        unexplorable_failures=1, unexplorable_radius=1.5)
    node._active_goal = Frontier(centroid=(2.0, 2.0), size=10, distance=1.0, score=1.0)
    node._goal_seq = 7

    class FakeFuture:
        def result(self):
            class R:
                status = 6  # ABORTED
            return R()

    node._on_goal_result(FakeFuture(), token=7)
    assert len(node._unexplorable) == 1


def test_unexplorable_area_is_not_selected_again(node_factory):
    """端到端：选点 -> 失败两次 -> 再选点，不能再选中同一片地方。"""
    node = node_factory(autostart=True, dry_run=False, replan_period=10.0,
                        unexplorable_failures=2, unexplorable_radius=1.5,
                        min_frontier_size=4, complete_confirmations=1)
    node._on_map(make_grid())
    node._robot_pose = lambda: (1.0, 1.5, 0.0)
    node._client.wait_for_server = lambda timeout_sec=0.5: True
    sent = []
    node._send_goal = lambda goal: sent.append(goal)

    node._plan_once(0.0, (1.0, 1.5, 0.0))
    assert sent, "第一次应该选出目标"
    first = sent[-1]
    blocked = (first.centroid[0], first.centroid[1])

    # 同一片地方失败两次 -> 标注
    for _ in range(2):
        node._active_goal = type(first)(
            centroid=blocked, size=first.size, distance=first.distance, score=first.score
        )
        node._reject_active_goal("timeout")

    sent.clear()
    node._blacklist.clear()          # 排掉会过期的黑名单，只留"无法探索"在起作用
    node._empty_cycles = 0
    node._consecutive_failures = 0
    node._plan_once(100.0, (1.0, 1.5, 0.0))
    if sent:
        chosen = (sent[-1].centroid[0], sent[-1].centroid[1])
        assert not node._unexplorable.blocked(*chosen), (
            "不能再把车派去已被标注无法探索的地方"
        )
    else:
        # 只剩这一个候选时，正确地什么都不选，而不是回头重试。
        assert node._unexplorable.blocked(*blocked)


def test_unexplorable_topic_is_published(node_factory):
    import json

    from semantic_rtabmap.frontier import Frontier

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0,
                        unexplorable_failures=1, unexplorable_radius=1.5)
    seen = []
    node.create_subscription(String, "/exploration/unexplorable", seen.append, 10)
    node._active_goal = Frontier(centroid=(3.0, 3.0), size=10, distance=1.0, score=1.0)
    node._reject_active_goal("timeout")
    spin_for(node, 1.5)

    assert seen, "必须把无法探索的区域发出来，操作员才知道哪里去不了"
    payload = json.loads(seen[-1].data)
    assert payload == [{"x": 3.0, "y": 3.0}]


def test_status_reports_unexplorable_count(node_factory):
    import json

    from semantic_rtabmap.frontier import Frontier

    node = node_factory(autostart=True, dry_run=True, replan_period=10.0,
                        unexplorable_failures=1, unexplorable_radius=1.5)
    statuses = []
    node.create_subscription(String, "/exploration/status", statuses.append, 10)
    node._active_goal = Frontier(centroid=(3.0, 3.0), size=10, distance=1.0, score=1.0)
    node._reject_active_goal("timeout")
    spin_for(node, 1.5)
    payload = json.loads(statuses[-1].data)
    assert payload["unexplorable"] == 1
