#!/usr/bin/env python3
"""Frontier-based autonomous exploration for the R680 mecanum rover.

Why this design
---------------
The R680 carries a forward-facing D435 and no 2D lidar, so the map is built
from a narrow field of view and is always incomplete. Full coverage therefore
comes from repeatedly driving to the boundary between known free space and
unknown space ("frontiers"), looking around, and repeating.

The node deliberately owns *no* velocity output. It only ever asks Nav2 to
drive somewhere, and Nav2's output passes through the existing safety gate
(``/cmd_vel_nav`` -> ``/cmd_vel_safe``) before it can reach the TX1 base. That
keeps one and only one place able to stop the robot.

Safety properties enforced here:

* ``autostart`` defaults to ``false``: the node idles until an operator calls
  ``/exploration/start``.
* ``dry_run`` defaults to ``true``: goals are published as a marker and logged,
  but Nav2 is never asked to move the robot. Validate the chosen goals first.
* A global time budget, a per-goal timeout and a consecutive-failure limit all
  stop exploration instead of letting it grind on.
* Goals are rejected when they sit inside an inflated obstacle, when they are
  cut off from the robot's own free-space region, or when a previous attempt
  there already failed recently.
* Every state change is published on ``/exploration/state`` and
  ``/exploration/status`` so an operator can always tell what it is doing.
"""

from __future__ import annotations

import json
import math
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from semantic_rtabmap.frontier import (
    Frontier,
    GoalBlacklist,
    GridInfo,
    UnexplorableRegions,
    detect_frontiers,
    score_frontiers,
    select_goal,
    wrap_angle,
)

STATE_IDLE = "IDLE"
STATE_EXPLORING = "EXPLORING"
STATE_PAUSED = "PAUSED"
STATE_COMPLETE = "COMPLETE"
STATE_ABORTED = "ABORTED"
# dry_run 专用的终态：候选被遍历完，而不是"世界上没有 frontier 了"。
#
# 为什么必须和 COMPLETE 分开：dry_run 的 _publish_goal 分支会把每个选中的目标
# 拉黑，好让下一轮换一个候选。遍历完所有候选后 _on_no_goal 必然被连续调用，
# 达到 complete_confirmations 就报 COMPLETE —— 于是一次 dry_run **必然**在
# 十几秒内以 COMPLETE 结束。但那时一个目标都没执行、车一步没动，用
# "no reachable frontier" 来描述是**假话**：它描述的是遍历进度，不是世界。
# 这个假话会让人（第一次做阶段 F 演练时）把正常的 dry_run 误判成故障，
# 也可能让人以为"地图上没地方可去"而放弃探索。
#
# 用独立状态表达，COMPLETE 就只在真的没有可达 frontier 时出现。
STATE_DRY_RUN_DONE = "DRY_RUN_DONE"


class ExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("r680_explorer")

        # --- interfaces -------------------------------------------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("navigate_action", "navigate_to_pose")

        # --- safety / gating --------------------------------------------
        self.declare_parameter("autostart", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("max_exploration_time", 1800.0)
        self.declare_parameter("goal_timeout", 120.0)
        self.declare_parameter("max_consecutive_failures", 5)
        self.declare_parameter("require_localization", True)
        self.declare_parameter("localization_timeout", 1.0)
        self.declare_parameter("battery_topic", "")
        self.declare_parameter("min_battery", 10.5)
        # How long a battery sample stays trustworthy. TX1's /PowerVoltage is
        # low-rate (a few Hz), so this is generous -- it is meant to catch a dead
        # relay or a dropped link, not normal jitter.
        self.declare_parameter("battery_timeout", 5.0)

        # --- frontier policy --------------------------------------------
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("max_frontiers", 64)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("robot_radius", 0.15)
        self.declare_parameter("goal_clearance", 0.20)
        self.declare_parameter("require_reachable", True)
        self.declare_parameter("min_goal_distance", 0.35)
        self.declare_parameter("max_goal_distance", 8.0)
        self.declare_parameter("info_gain_radius", 1.5)
        self.declare_parameter("score_weight_size", 1.0)
        self.declare_parameter("score_weight_gain", 1.5)
        self.declare_parameter("score_weight_distance", 1.0)
        self.declare_parameter("score_weight_heading", 0.6)
        self.declare_parameter("blacklist_radius", 1.0)
        self.declare_parameter("blacklist_ttl", 60.0)
        # 反复失败后被永久排除的区域（"无法探索"）。
        #
        # 为什么不能只靠 blacklist_ttl：黑名单 60 s 就过期，于是同一个到不了的
        # frontier 会被反复重试。按 goal_timeout 60 s 算，连续三次重试就是三分
        # 钟预算花在重复证明同一件事，而且最后是以"连续失败"中止整轮探索 ——
        # 真正的原因是**那一片地方去不了**，不是探索策略错了。
        #
        # unexplorable_failures 是按**区域**计的（半径 unexplorable_radius），
        # 不是按坐标精确匹配：frontier 质心每次都会差几厘米，精确匹配等于永不
        # 触发。
        self.declare_parameter("unexplorable_radius", 1.5)
        self.declare_parameter("unexplorable_failures", 2)
        self.declare_parameter("replan_period", 3.0)
        self.declare_parameter("complete_confirmations", 3)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._map: Optional[OccupancyGrid] = None
        self._map_stamp = 0.0
        self._state = STATE_IDLE
        self._message = "waiting for start"
        self._started_at = 0.0
        self._deadline = 0.0
        self._blacklist = GoalBlacklist(
            radius=float(self.get_parameter("blacklist_radius").value),
            ttl=float(self.get_parameter("blacklist_ttl").value),
        )
        self._unexplorable = UnexplorableRegions(
            radius=float(self.get_parameter("unexplorable_radius").value)
        )
        # [[x, y, failures], ...] -- one row per area that has failed at least
        # once. Kept separately from the unexplorable set so the transition is
        # observable (and testable) instead of appearing as a single jump.
        self._region_failures: List[List[float]] = []
        self._goal_handle = None
        self._goal_started = 0.0
        self._active_goal: Optional[Frontier] = None
        self._goal_seq = 0
        self._consecutive_failures = 0
        self._completed_goals = 0
        self._failed_goals = 0
        self._empty_cycles = 0
        # How many candidates the dry-run traversal has inspected. Reported in
        # the DRY_RUN_DONE message so the number means something to the operator.
        self._dry_run_seen = 0
        self._last_plan = 0.0
        # Counts from the most recent frontier check, published in the COMPLETE
        # message and in every "no goal this check" line. Kept as an attribute
        # because the COMPLETE decision is made in a later cycle than the check
        # that produced them.
        self._last_frontier_stats: dict = {}
        self._battery: Optional[float] = None
        self._battery_stamp = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, map_qos
        )
        battery_topic = str(self.get_parameter("battery_topic").value)
        if battery_topic:
            # TX1 publishes /PowerVoltage but the relay does not forward it yet,
            # and its message type has not been confirmed on the robot. This
            # subscription is therefore opt-in: if the type does not match the
            # TX1 publisher, ROS 2 delivers no data and the battery guard simply
            # stays inactive instead of failing.
            self.create_subscription(Float32, battery_topic, self._on_battery, qos)

        self._state_pub = self.create_publisher(String, "/exploration/state", qos)
        self._status_pub = self.create_publisher(String, "/exploration/status", qos)
        self._goal_pub = self.create_publisher(PoseStamped, "/exploration/goal", qos)
        # 被判定"无法探索"的区域，JSON 列表 [{"x":..,"y":..}, ...]。
        # 单独发一个话题而不是只塞进 status：这是操作员要拿去用（在 RViz 里画、
        # 或者人工判断该不该搬开障碍物）的东西，不该逼人去解析状态字符串。
        self._unexplorable_pub = self.create_publisher(
            String, "/exploration/unexplorable", qos
        )

        self.create_service(Trigger, "/exploration/start", self._srv_start)
        self.create_service(Trigger, "/exploration/stop", self._srv_stop)
        self.create_service(Trigger, "/exploration/pause", self._srv_pause)
        self.create_service(Trigger, "/exploration/resume", self._srv_resume)

        self._client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigate_action").value)
        )

        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status)

        if bool(self.get_parameter("autostart").value):
            self._begin()
        self.get_logger().info(
            "explorer ready (state=%s, dry_run=%s)"
            % (self._state, bool(self.get_parameter("dry_run").value))
        )

    # ------------------------------------------------------------------
    # inputs
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._map_stamp = time.monotonic()

    def _on_battery(self, msg: Float32) -> None:
        """Record a battery voltage sample for the low-voltage guard."""
        self._battery = float(msg.data)
        self._battery_stamp = time.monotonic()

    def _robot_pose(self):
        """Return ``(x, y, yaw)`` from TF, or ``None`` if unavailable."""
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter("global_frame").value),
                str(self.get_parameter("robot_base_frame").value),
                rclpy.time.Time(),
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return (float(t.x), float(t.y), yaw)

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def _begin(self) -> None:
        self._started_at = time.monotonic()
        budget = float(self.get_parameter("max_exploration_time").value)
        self._deadline = self._started_at + budget
        self._blacklist.clear()
        self._consecutive_failures = 0
        self._empty_cycles = 0
        self._dry_run_seen = 0
        self._set_state(STATE_EXPLORING, "exploration started")

    def _srv_start(self, _req, res):
        if self._state == STATE_EXPLORING:
            res.success, res.message = False, "already exploring"
            return res
        self._begin()
        res.success, res.message = True, "exploration started"
        return res

    def _srv_stop(self, _req, res):
        self._cancel_goal()
        self._set_state(STATE_IDLE, "stopped by operator")
        res.success, res.message = True, "stopped"
        return res

    def _srv_pause(self, _req, res):
        if self._state != STATE_EXPLORING:
            res.success, res.message = False, "not exploring"
            return res
        self._cancel_goal()
        self._set_state(STATE_PAUSED, "paused by operator")
        res.success, res.message = True, "paused"
        return res

    def _srv_resume(self, _req, res):
        if self._state != STATE_PAUSED:
            res.success, res.message = False, "not paused"
            return res
        self._set_state(STATE_EXPLORING, "resumed")
        res.success, res.message = True, "resumed"
        return res

    # ------------------------------------------------------------------
    # state helpers
    # ------------------------------------------------------------------
    def _set_state(self, state: str, message: str) -> None:
        if state != self._state:
            self.get_logger().info("state %s -> %s (%s)" % (self._state, state, message))
        self._state = state
        self._message = message
        self._state_pub.publish(String(data=state))

    def _invalidate_goal(self) -> None:
        """Bump the goal token so pending callbacks for the old goal are ignored.

        Cancelling a goal still produces a result with status CANCELED. Without
        this, an operator pause would be immediately undone by that result and
        counted as a failure.
        """
        self._goal_seq += 1

    def _cancel_goal(self) -> None:
        self._invalidate_goal()
        handle = self._goal_handle
        self._goal_handle = None
        self._active_goal = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # pragma: no cover - best effort cancel
                pass

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._state != STATE_EXPLORING:
            return

        now = time.monotonic()

        if now > self._deadline:
            self._cancel_goal()
            self._set_state(STATE_ABORTED, "time budget exhausted")
            return

        if self._consecutive_failures >= int(
            self.get_parameter("max_consecutive_failures").value
        ):
            self._cancel_goal()
            # 把"无法探索"的区域数一并报出来：同一句"连续失败"可能是两种完全
            # 不同的处境 —— 策略选不出好点，或者那片地方真的去不了。数字在这
            # 里，操作员才能判断该改参数还是该搬开障碍物。
            self._set_state(
                STATE_ABORTED,
                "%d consecutive goal failures (%d area(s) marked unexplorable)"
                % (self._consecutive_failures, len(self._unexplorable)),
            )
            return

        if self._battery is not None:
            minimum = float(self.get_parameter("min_battery").value)
            battery_timeout = float(self.get_parameter("battery_timeout").value)
            age = now - self._battery_stamp
            # A reading that has stopped arriving must not be treated as a current
            # one. `_battery_stamp` was already being recorded here but never read,
            # so a relay that died mid-run left the guard comparing against the
            # last voltage it ever saw -- possibly minutes old -- and reporting
            # "fine". That is the same silent-failure shape as the other defects
            # in this project: the sensor is dead, the system looks healthy.
            #
            # Stale is NOT the same as low, so this aborts rather than silently
            # continuing: an autonomous run that cannot see its own battery has
            # lost the protection it was relying on.
            if age > battery_timeout:
                self._cancel_goal()
                self._set_state(
                    STATE_ABORTED,
                    "battery reading stale: last sample %.1f s ago (limit %.1f s)"
                    % (age, battery_timeout),
                )
                return
            if self._battery < minimum:
                self._cancel_goal()
                self._set_state(
                    STATE_ABORTED,
                    "battery %.2f below %.2f" % (self._battery, minimum),
                )
                return

        if self._goal_handle is not None:
            timeout = float(self.get_parameter("goal_timeout").value)
            if now - self._goal_started > timeout:
                self.get_logger().warn("goal timed out; rejecting it")
                self._reject_active_goal("timeout")
            return

        if now - self._last_plan < float(self.get_parameter("replan_period").value):
            return
        self._last_plan = now

        if self._map is None:
            self._set_state(STATE_EXPLORING, "waiting for /map")
            return

        pose = self._robot_pose()
        if pose is None:
            if bool(self.get_parameter("require_localization").value):
                self._set_state(STATE_EXPLORING, "waiting for localization TF")
            return
        self._plan_once(now, pose)

    def _plan_once(self, now: float, pose) -> None:
        """Choose a frontier for ``pose`` and dispatch it.

        Kept separate from :meth:`_tick` so the policy can be exercised with an
        injected pose in tests, without TF, Nav2 or a robot.
        """
        grid = self._map
        if grid is None:
            return
        rx, ry, yaw = pose

        info = GridInfo(
            resolution=float(grid.info.resolution),
            origin_x=float(grid.info.origin.position.x),
            origin_y=float(grid.info.origin.position.y),
            width=int(grid.info.width),
            height=int(grid.info.height),
        )
        data = self._grid_array(grid)
        if data is None:
            return

        check_stats: dict = {}
        frontiers = detect_frontiers(
            data,
            info,
            robot_xy=(rx, ry),
            min_size=int(self.get_parameter("min_frontier_size").value),
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
            goal_clearance=float(self.get_parameter("goal_clearance").value),
            require_reachable=bool(self.get_parameter("require_reachable").value),
            max_frontiers=int(self.get_parameter("max_frontiers").value),
            stats=check_stats,
        )
        score_frontiers(
            frontiers,
            robot_xy=(rx, ry),
            robot_yaw=yaw,
            weight_size=float(self.get_parameter("score_weight_size").value),
            weight_gain=float(self.get_parameter("score_weight_gain").value),
            weight_distance=float(self.get_parameter("score_weight_distance").value),
            weight_heading=float(self.get_parameter("score_weight_heading").value),
            data=data,
            info=info,
            gain_radius=float(self.get_parameter("info_gain_radius").value),
        )
        goal = select_goal(
            frontiers,
            blacklist=self._blacklist,
            now=now,
            min_distance=float(self.get_parameter("min_goal_distance").value),
            max_distance=float(self.get_parameter("max_goal_distance").value),
            robot_xy=(rx, ry),
            unexplorable=self._unexplorable,
            stats=check_stats,
        )
        self._last_frontier_stats = check_stats
        if goal is None:
            # Say WHY every cycle, not only when the state changes. The state
            # machine used to be the only narrator here (_set_state logs on
            # transition only), so on 2026-10-18 the log showed one goal driven
            # and then COMPLETE, with the two intermediate "empty cycle" rounds
            # leaving no trace and no way to tell "the map has no frontier" from
            # "every frontier was rejected" -- the two have completely different
            # fixes.
            self.get_logger().info("no goal this check: %s" % self._describe_stats())
            self._on_no_goal(frontiers)
            return

        # INFO, not DEBUG: _tick reaches this line only when no goal is in
        # flight, so it is one line per dispatched goal (~20-50 s apart), not one
        # per cycle. At DEBUG the 2026-10-18 16:15 run left no trace of the four
        # goals it dispatched -- two of them unplannable -- so the only evidence
        # left was Nav2's side of the story.
        self.get_logger().info(
            "goal %s: %s" % (self._describe_goal(goal), self._describe_stats())
        )

        self._empty_cycles = 0
        self._active_goal = goal
        self._goal_started = now
        self._publish_goal(goal, yaw)

        if bool(self.get_parameter("dry_run").value):
            self.get_logger().info(
                "dry_run goal x=%.2f y=%.2f size=%d score=%.2f"
                % (goal.centroid[0], goal.centroid[1], goal.size, goal.score)
            )
            # In dry run the goal is never sent to Nav2; reject it locally so
            # the next cycle evaluates a different part of the map. This makes
            # dry_run a SWEEP of the candidates rather than a single pick --
            # which is the point, but it also means the sweep runs out and
            # _on_no_goal fires; see STATE_DRY_RUN_DONE for why that must not
            # be reported as COMPLETE.
            self._dry_run_seen += 1
            self._blacklist.add(goal.centroid[0], goal.centroid[1], now)
            self._active_goal = None
            return

        self._send_goal(goal)

    def _on_no_goal(self, frontiers) -> None:
        self._empty_cycles += 1
        needed = int(self.get_parameter("complete_confirmations").value)
        if self._empty_cycles >= needed:
            # Two genuinely different situations reach this point in dry_run,
            # and reporting both as COMPLETE would be wrong for one of them:
            #
            #   (a) no candidate was ever seen (_dry_run_seen == 0) -- e.g. a map
            #       with no frontier at all. "no reachable frontier" is then a
            #       TRUE statement about the world, so COMPLETE is right.
            #   (b) candidates were seen and the sweep blacklisted them all.
            #       Nothing was driven, so this says nothing about whether
            #       frontiers remain -- COMPLETE would be a false claim.
            #
            # The discriminator is whether the sweep actually consumed anything,
            # not merely that dry_run is on. An earlier version of this fix keyed
            # only on dry_run and so mislabelled case (a) -- trading one false
            # message for another. Caught by the existing test
            # test_complete_after_repeated_empty_cycles, which drives a fully
            # free map (no frontier at all) in dry_run and rightly expects
            # COMPLETE.
            if bool(self.get_parameter("dry_run").value) and self._dry_run_seen > 0:
                self._set_state(
                    STATE_DRY_RUN_DONE,
                    "dry_run swept %d candidate(s); nothing was driven, so this "
                    "says nothing about whether frontiers remain"
                    % (self._dry_run_seen,),
                )
            else:
                self._set_state(
                    STATE_COMPLETE,
                    "no reachable frontier after %d checks (%d goal(s) visited); "
                    "last check: %s"
                    % (self._empty_cycles, self._completed_goals, self._describe_stats()),
                )
        else:
            self._set_state(
                STATE_EXPLORING,
                "no frontier this cycle (%d/%d)" % (self._empty_cycles, needed),
            )

    def _describe_stats(self) -> str:
        """One-line summary of the last frontier check, for the log and messages.

        The two stages are printed as two groups because they answer different
        questions: the first says whether the map had frontiers at all and which
        filter removed them, the second says whether any surviving candidate was
        pickable (distance window, blacklist, unexplorable regions).
        """
        s = self._last_frontier_stats or {}
        get = lambda key: int(s.get(key, 0))  # noqa: E731 - tiny local helper
        return (
            "mask=%d clusters=%d small=%d unreachable=%d no_cell=%d kept=%d "
            "robot_label=%d | out_of_range=%d blacklisted=%d unexplorable=%d "
            "unscored=%d"
            % (
                get("mask_cells"),
                get("clusters"),
                get("dropped_small"),
                get("dropped_unreachable"),
                get("dropped_no_goal_cell"),
                get("kept"),
                get("robot_label"),
                get("out_of_range"),
                get("blacklisted"),
                get("unexplorable"),
                get("unscored"),
            )
        )

    @staticmethod
    def _describe_goal(goal) -> str:
        return "x=%.2f y=%.2f size=%d score=%.2f" % (
            goal.centroid[0],
            goal.centroid[1],
            goal.size,
            goal.score,
        )

    @staticmethod
    def _grid_array(grid: OccupancyGrid):
        try:
            import numpy as np

            array = np.asarray(grid.data, dtype=np.int16)
            if array.size != grid.info.width * grid.info.height:
                return None
            return array.reshape((grid.info.height, grid.info.width))
        except Exception:  # pragma: no cover - malformed grid
            return None

    # ------------------------------------------------------------------
    # goal dispatch
    # ------------------------------------------------------------------
    def _publish_goal(self, goal: Frontier, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = str(self.get_parameter("goal_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal.centroid[0])
        msg.pose.position.y = float(goal.centroid[1])
        # Face along the robot's current heading; Nav2 only needs a position
        # here and the controller will orient the base itself.
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(msg)

    def _send_goal(self, goal: Frontier) -> None:
        if not self._client.wait_for_server(timeout_sec=0.5):
            self._set_state(STATE_EXPLORING, "waiting for Nav2 action server")
            self._active_goal = None
            return

        msg = PoseStamped()
        msg.header.frame_id = str(self.get_parameter("goal_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal.centroid[0])
        msg.pose.position.y = float(goal.centroid[1])
        msg.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        # Each dispatched goal gets a token; callbacks for a superseded or
        # cancelled goal are dropped instead of mutating the state machine.
        self._goal_seq += 1
        token = self._goal_seq
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_goal_response(f, token))

    def _on_goal_response(self, future, token: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # pragma: no cover - transport failure
            self.get_logger().warn("goal request failed: %s" % exc)
            if token == self._goal_seq:
                self._reject_active_goal("request failed")
            return

        if token != self._goal_seq:
            # Already cancelled or superseded: make sure it does not run.
            if handle is not None and handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception:  # pragma: no cover
                    pass
            return

        if not handle.accepted:
            self._reject_active_goal("goal rejected")
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_goal_result(f, token)
        )

    def _on_goal_result(self, future, token: int) -> None:
        if token != self._goal_seq:
            # Result of a goal we deliberately cancelled (pause/stop/timeout).
            return
        self._goal_handle = None
        goal = self._active_goal
        self._active_goal = None
        try:
            status = future.result().status
        except Exception as exc:  # pragma: no cover - transport failure
            self.get_logger().warn("goal result failed: %s" % exc)
            self._reject_active_goal("result failed")
            return

        # ActionStatus: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self._completed_goals += 1
            self._consecutive_failures = 0
            self._set_state(
                STATE_EXPLORING, "goal reached (%d completed)" % self._completed_goals
            )
        else:
            marked = False
            if goal is not None:
                self._blacklist.add(
                    goal.centroid[0], goal.centroid[1], time.monotonic()
                )
                marked = self._record_failure(goal.centroid[0], goal.centroid[1])
            self._failed_goals += 1
            self._consecutive_failures += 1
            self._set_state(
                STATE_EXPLORING,
                "goal not reached (status=%d, %d consecutive failure(s))%s"
                % (
                    status,
                    self._consecutive_failures,
                    " -- area marked UNEXPLORABLE" if marked else "",
                ),
            )

    def _record_failure(self, x: float, y: float) -> bool:
        """Count one failure for the area around ``(x, y)``.

        Returns ``True`` exactly when this failure is the one that crossed the
        threshold and marked the area unexplorable, so the caller can say so in
        the operator-visible message instead of leaving the transition silent.

        Matching is by radius, not by exact coordinates: a frontier centroid
        moves by centimetres from cycle to cycle, so exact matching would never
        accumulate and the area would never be given up on.
        """
        radius = self._unexplorable.radius
        r2 = radius * radius
        row = None
        for candidate in self._region_failures:
            if (candidate[0] - x) ** 2 + (candidate[1] - y) ** 2 <= r2:
                row = candidate
                break
        if row is None:
            row = [float(x), float(y), 0.0]
            self._region_failures.append(row)
        row[2] += 1

        threshold = int(self.get_parameter("unexplorable_failures").value)
        if row[2] >= threshold and not self._unexplorable.blocked(x, y):
            self._unexplorable.add(x, y)
            self.get_logger().warn(
                "area (%.2f, %.2f) failed %d time(s) -- marked UNEXPLORABLE"
                % (x, y, int(row[2]))
            )
            return True
        return False

    def _reject_active_goal(self, reason: str) -> None:
        goal = self._active_goal
        marked = False
        if goal is not None:
            self._blacklist.add(goal.centroid[0], goal.centroid[1], time.monotonic())
            marked = self._record_failure(goal.centroid[0], goal.centroid[1])
        self._cancel_goal()
        self._failed_goals += 1
        self._consecutive_failures += 1
        self._set_state(
            STATE_EXPLORING,
            "goal rejected: %s (%d consecutive)%s"
            % (
                reason,
                self._consecutive_failures,
                " -- area marked UNEXPLORABLE" if marked else "",
            ),
        )

    # ------------------------------------------------------------------
    def _publish_status(self) -> None:
        self._blacklist.prune(time.monotonic())
        payload = {
            "state": self._state,
            "message": self._message,
            "dry_run": bool(self.get_parameter("dry_run").value),
            "goals_completed": self._completed_goals,
            "goals_failed": self._failed_goals,
            "consecutive_failures": self._consecutive_failures,
            "blacklisted": len(self._blacklist),
            "unexplorable": len(self._unexplorable),
            "elapsed": (
                round(time.monotonic() - self._started_at, 1) if self._started_at else 0.0
            ),
            "map_age": (
                round(time.monotonic() - self._map_stamp, 1)
                if self._map_stamp
                else None
            ),
        }
        self._status_pub.publish(String(data=json.dumps(payload)))
        self._state_pub.publish(String(data=self._state))
        # 每次都发，不是只在变化时发：操作员（或录制器）通常是在车已经跑起来
        # 之后才连上来的，只在边沿发布会让它看到一个空话题。
        self._unexplorable_pub.publish(
            String(data=json.dumps(self._unexplorable.as_list()))
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationNode()
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
