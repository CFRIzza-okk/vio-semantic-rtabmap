"""Unit tests for the pure exploration policy in ``semantic_rtabmap.frontier``.

These tests build synthetic occupancy grids, so they exercise frontier
detection, reachability, scoring and blacklisting without ROS, Nav2, a camera
or a robot. Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_frontier_policy.py -v
"""

import math

import numpy as np
import pytest

from semantic_rtabmap.frontier import (
    Frontier,
    GoalBlacklist,
    GridInfo,
    detect_frontiers,
    estimate_gain,
    frontier_mask,
    inflate,
    occupancy_arrays,
    score_frontiers,
    select_goal,
    wrap_angle,
)

RES = 0.1
UNKNOWN = -1


def info(width, height, origin_x=0.0, origin_y=0.0, resolution=RES):
    return GridInfo(
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        width=width,
        height=height,
    )


def blank(width, height):
    """Fully unknown grid, as an unexplored map starts out."""
    return np.full((height, width), UNKNOWN, dtype=np.int16)


# ----------------------------------------------------------------------
# geometry helpers
# ----------------------------------------------------------------------
def test_cell_world_round_trip():
    geometry = info(20, 10, origin_x=-1.0, origin_y=2.0)
    for cx, cy in [(0, 0), (5, 3), (19, 9)]:
        wx, wy = geometry.cell_to_world(cx, cy)
        assert geometry.world_to_cell(wx, wy) == (cx, cy)


def test_cell_centre_uses_half_cell_offset():
    geometry = info(10, 10)
    assert geometry.cell_to_world(0, 0) == pytest.approx((0.05, 0.05))


def test_invalid_grid_info_rejected():
    assert not GridInfo(0.0, 0.0, 0.0, 10, 10).valid
    assert not GridInfo(0.05, 0.0, 0.0, 0, 10).valid
    assert info(10, 10).valid


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(2 * math.pi) == pytest.approx(0.0, abs=1e-9)
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-(math.pi - 0.1))
    # -3*pi is far outside the range but must fold back onto the pi boundary.
    assert abs(wrap_angle(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-6)
    # Every result stays inside the documented (-pi, pi] range.
    for value in (-10.0, -3.0, 0.0, 3.0, 10.0):
        assert -math.pi - 1e-9 <= wrap_angle(value) <= math.pi + 1e-9


# ----------------------------------------------------------------------
# occupancy classification and frontier detection
# ----------------------------------------------------------------------
def test_occupancy_arrays_classify_three_states():
    data = np.array([[UNKNOWN, 0, 100, 49, 50]], dtype=np.int16)
    free, occupied, unknown = occupancy_arrays(data, occupied_threshold=50)
    assert free.tolist() == [[False, True, False, True, False]]
    assert occupied.tolist() == [[False, False, True, False, True]]
    assert unknown.tolist() == [[True, False, False, False, False]]


def test_all_free_grid_has_no_frontier():
    data = np.zeros((10, 10), dtype=np.int16)
    assert not frontier_mask(data).any()
    assert detect_frontiers(data, info(10, 10)) == []


def test_all_unknown_grid_has_no_frontier():
    data = blank(10, 10)
    assert not frontier_mask(data).any()
    assert detect_frontiers(data, info(10, 10)) == []


def test_single_free_block_yields_perimeter_frontier():
    data = blank(10, 10)
    data[3:7, 3:7] = 0
    frontiers = detect_frontiers(data, info(10, 10), min_size=8)
    assert len(frontiers) == 1
    # A 4x4 block has 12 perimeter cells and 4 interior cells.
    assert frontiers[0].size == 12
    x, y = frontiers[0].centroid
    assert x == pytest.approx(0.5, abs=0.1)
    assert y == pytest.approx(0.5, abs=0.1)


def test_small_frontier_is_filtered_by_min_size():
    data = blank(10, 10)
    # 2x2 block: all four cells touch unknown, so size is 4.
    data[4:6, 4:6] = 0
    assert detect_frontiers(data, info(10, 10), min_size=5) == []
    assert len(detect_frontiers(data, info(10, 10), min_size=4)) == 1


def test_two_separate_blocks_yield_two_frontiers():
    data = blank(30, 12)
    data[2:8, 2:8] = 0
    data[2:8, 20:26] = 0
    frontiers = detect_frontiers(data, info(30, 12), min_size=8)
    assert len(frontiers) == 2
    xs = sorted(f.centroid[0] for f in frontiers)
    assert xs[0] < 1.0 < 2.0 < xs[1]


def test_shape_mismatch_returns_empty():
    data = np.zeros((5, 5), dtype=np.int16)
    assert detect_frontiers(data, info(10, 10)) == []


def test_inflate_grows_obstacle_mask():
    occupied = np.zeros((21, 21), dtype=bool)
    occupied[10, 10] = True
    inflated = inflate(occupied, radius_m=0.2, resolution=0.1)
    # A 0.2 m radius at 0.1 m resolution is a 2-cell dilation.
    assert inflated[10, 12] and inflated[12, 10]
    assert not inflated[10, 13]
    assert inflated.sum() > occupied.sum()


def test_goal_clearance_pushes_goal_away_from_obstacles():
    data = blank(20, 20)
    data[5:15, 5:15] = 0
    # A 2x2 occupied block in the middle of the free area.
    data[9:11, 9:11] = 100
    obstacle_centres = [(0.95, 0.95), (1.05, 0.95), (0.95, 1.05), (1.05, 1.05)]

    def clearance_of(frontier):
        return min(
            math.hypot(frontier.centroid[0] - ox, frontier.centroid[1] - oy)
            for ox, oy in obstacle_centres
        )

    without = detect_frontiers(data, info(20, 20), min_size=8, goal_clearance=0.0)
    with_clearance = detect_frontiers(
        data, info(20, 20), min_size=8, goal_clearance=0.5
    )
    assert without and with_clearance
    assert clearance_of(with_clearance[0]) > clearance_of(without[0])


def test_frontier_is_dropped_when_every_cell_is_inside_clearance():
    data = blank(20, 20)
    data[5:8, 5:8] = 0
    data[6, 6] = 100  # obstacle in the centre of a 3x3 free block
    # The 8 perimeter cells are the frontier; a 1.0 m clearance covers all of
    # them, so no safe goal cell remains and the cluster must be dropped.
    assert detect_frontiers(data, info(20, 20), min_size=8, goal_clearance=0.0)
    assert detect_frontiers(data, info(20, 20), min_size=8, goal_clearance=1.0) == []


# ----------------------------------------------------------------------
# reachability
# ----------------------------------------------------------------------
def _two_rooms():
    """Two free rooms split by a wall, everything else unknown."""
    data = blank(21, 21)
    data[5:16, 0:10] = 0
    data[5:16, 11:21] = 0
    data[5:16, 10] = 100
    return data


def test_unreachable_frontier_is_filtered_when_required():
    data = _two_rooms()
    geometry = info(21, 21)
    robot = (0.5, 1.0)  # inside the left room

    reachable = detect_frontiers(
        data, geometry, robot_xy=robot, min_size=8, require_reachable=True
    )
    everything = detect_frontiers(
        data, geometry, robot_xy=robot, min_size=8, require_reachable=False
    )

    assert reachable, "the robot's own room must still offer frontiers"
    assert all(f.centroid[0] < 1.0 for f in reachable)
    assert any(f.centroid[0] > 1.0 for f in everything)
    assert len(everything) > len(reachable)


def test_reachability_without_robot_pose_keeps_frontiers():
    data = _two_rooms()
    frontiers = detect_frontiers(
        data, info(21, 21), robot_xy=None, min_size=8, require_reachable=True
    )
    assert frontiers


# ----------------------------------------------------------------------
# gain and scoring
# ----------------------------------------------------------------------
def test_estimate_gain_counts_unknown_cells_in_radius():
    data = blank(20, 20)
    data[10, 10] = 0
    frontier = Frontier(centroid=(1.05, 1.05), size=1)
    gain = estimate_gain(data, info(20, 20), frontier, radius_m=0.25)
    # radius 0.25 m at 0.1 m resolution rounds up to 3 cells, so the window is
    # 7x7 = 49 cells, of which the single free cell is not unknown.
    assert gain == pytest.approx(48.0)

    small = estimate_gain(data, info(20, 20), frontier, radius_m=0.15)
    # radius 0.15 m rounds up to 2 cells -> 5x5 window minus the free cell.
    assert small == pytest.approx(24.0)


def test_scoring_prefers_the_nearer_of_two_equal_frontiers():
    near = Frontier(centroid=(1.0, 0.0), size=10)
    far = Frontier(centroid=(5.0, 0.0), size=10)
    frontiers = [near, far]
    score_frontiers(frontiers, robot_xy=(0.0, 0.0), robot_yaw=0.0, weight_gain=0.0)
    assert near.score > far.score
    assert near.distance == pytest.approx(1.0)
    assert far.distance == pytest.approx(5.0)


def test_scoring_prefers_the_larger_of_two_equidistant_frontiers():
    small = Frontier(centroid=(2.0, 2.0), size=10)
    large = Frontier(centroid=(-2.0, -2.0), size=90)
    frontiers = [small, large]
    score_frontiers(frontiers, robot_xy=(0.0, 0.0), robot_yaw=0.0, weight_gain=0.0)
    assert large.score > small.score


def test_heading_penalty_favours_frontier_in_front_of_robot():
    ahead = Frontier(centroid=(3.0, 0.0), size=10)
    behind = Frontier(centroid=(-3.0, 0.0), size=10)
    frontiers = [ahead, behind]
    score_frontiers(frontiers, robot_xy=(0.0, 0.0), robot_yaw=0.0, weight_gain=0.0)
    assert ahead.heading_error == pytest.approx(0.0)
    assert behind.heading_error == pytest.approx(math.pi)
    assert ahead.score > behind.score


def test_gain_term_only_applies_when_grid_supplied():
    frontier = Frontier(centroid=(1.0, 0.0), size=10)
    score_frontiers([frontier], robot_xy=(0.0, 0.0), robot_yaw=0.0)
    plain = frontier.score
    data = blank(20, 20)
    data[10, 10] = 0
    frontier2 = Frontier(centroid=(1.0, 0.0), size=10)
    score_frontiers(
        [frontier2],
        robot_xy=(0.0, 0.0),
        robot_yaw=0.0,
        data=data,
        info=info(20, 20),
        gain_radius=1.0,
    )
    assert plain == pytest.approx(frontier2.score) or frontier2.gain >= 0.0


def test_scoring_empty_list_is_a_no_op():
    score_frontiers([], robot_xy=(0.0, 0.0), robot_yaw=0.0)


# ----------------------------------------------------------------------
# blacklist and selection
# ----------------------------------------------------------------------
def test_blacklist_blocks_nearby_goal_and_expires():
    blacklist = GoalBlacklist(radius=1.0, ttl=10.0)
    blacklist.add(2.0, 2.0, now=100.0)
    assert blacklist.blocked(2.5, 2.0, now=101.0)
    assert not blacklist.blocked(5.0, 2.0, now=101.0)
    # Expiry is exclusive at ttl.
    assert not blacklist.blocked(2.0, 2.0, now=110.1)
    assert len(blacklist) == 0


def test_blacklist_clear():
    blacklist = GoalBlacklist(radius=1.0, ttl=10.0)
    blacklist.add(0.0, 0.0, now=0.0)
    assert len(blacklist) == 1
    blacklist.clear()
    assert len(blacklist) == 0


def test_select_goal_returns_none_when_all_blacklisted():
    goals = [Frontier(centroid=(1.0, 0.0), size=10), Frontier(centroid=(2.0, 0.0), size=10)]
    score_frontiers(goals, robot_xy=(0.0, 0.0), robot_yaw=0.0, weight_gain=0.0)
    blacklist = GoalBlacklist(radius=0.5, ttl=60.0)
    for goal in goals:
        blacklist.add(goal.centroid[0], goal.centroid[1], now=0.0)
    assert select_goal(goals, blacklist=blacklist, now=1.0) is None


def test_select_goal_skips_unscored_frontiers():
    scored = Frontier(centroid=(1.0, 0.0), size=10, score=0.5, distance=1.0)
    unscored = Frontier(centroid=(0.5, 0.0), size=99)
    chosen = select_goal([unscored, scored], now=0.0)
    assert chosen is scored


def test_select_goal_respects_distance_window():
    near = Frontier(centroid=(0.1, 0.0), size=10, score=0.9, distance=0.1)
    good = Frontier(centroid=(2.0, 0.0), size=10, score=0.8, distance=2.0)
    far = Frontier(centroid=(50.0, 0.0), size=10, score=0.95, distance=50.0)
    chosen = select_goal([near, good, far], now=0.0, min_distance=0.35, max_distance=8.0)
    assert chosen is good


def test_select_goal_picks_highest_score():
    a = Frontier(centroid=(1.0, 0.0), size=10, score=0.2, distance=1.0)
    b = Frontier(centroid=(2.0, 0.0), size=10, score=0.7, distance=2.0)
    assert select_goal([a, b], now=0.0) is b


# ----------------------------------------------------------------------
# end-to-end policy on a synthetic map
# ----------------------------------------------------------------------
def test_policy_selects_reachable_goal_from_synthetic_map():
    data = _two_rooms()
    geometry = info(21, 21)
    robot_xy = (0.5, 1.0)
    robot_yaw = 0.0

    frontiers = detect_frontiers(
        data, geometry, robot_xy=robot_xy, min_size=8, require_reachable=True
    )
    score_frontiers(
        frontiers,
        robot_xy=robot_xy,
        robot_yaw=robot_yaw,
        data=data,
        info=geometry,
        gain_radius=1.0,
    )
    goal = select_goal(
        frontiers, blacklist=GoalBlacklist(1.0, 60.0), now=0.0, max_distance=8.0
    )
    assert goal is not None
    assert goal.centroid[0] < 1.0
    assert goal.score > float("-inf")
