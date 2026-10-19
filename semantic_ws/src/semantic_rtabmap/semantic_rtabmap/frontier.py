"""Frontier detection, scoring and goal selection for grid exploration.

This module is deliberately free of any ROS dependency so the exploration
policy can be unit tested on synthetic occupancy grids without a robot,
a camera or a running Nav2 stack.

Coordinate conventions follow ``nav_msgs/OccupancyGrid``:

* ``data`` is a row-major array of ``height`` rows by ``width`` columns;
* cell ``(cx, cy)`` covers ``[origin + c * resolution, origin + (c+1) * resolution)``;
* the world position of a cell is therefore its centre, i.e. ``+ 0.5 * resolution``;
* ``-1`` means unknown, ``0`` means free and ``>= occupied_threshold`` means occupied.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

UNKNOWN = -1

# Standard logging, not rclpy: this module is deliberately ROS-free so the
# exploration policy stays unit-testable without a robot or a running stack.
logger = logging.getLogger(__name__)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class GridInfo:
    """Geometry of an occupancy grid."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    @property
    def valid(self) -> bool:
        return self.resolution > 0.0 and self.width > 0 and self.height > 0

    def cell_to_world(self, cx: float, cy: float) -> Tuple[float, float]:
        """Return the world position of the centre of cell ``(cx, cy)``."""
        return (
            self.origin_x + (float(cx) + 0.5) * self.resolution,
            self.origin_y + (float(cy) + 0.5) * self.resolution,
        )

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        """Return the cell containing world position ``(wx, wy)``."""
        return (
            int(math.floor((wx - self.origin_x) / self.resolution)),
            int(math.floor((wy - self.origin_y) / self.resolution)),
        )

    def contains_cell(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height


@dataclass
class Frontier:
    """One connected cluster of frontier cells together with its score."""

    centroid: Tuple[float, float]
    size: int
    heading_error: float = math.pi
    distance: float = float("inf")
    gain: float = 0.0
    score: float = float("-inf")
    cells: List[Tuple[int, int]] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "x": self.centroid[0],
            "y": self.centroid[1],
            "size": self.size,
            "distance": self.distance,
            "heading_error": self.heading_error,
            "gain": self.gain,
            "score": self.score,
        }


def occupancy_arrays(
    data: np.ndarray, occupied_threshold: int = 50
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Split raw grid data into ``(free, occupied, unknown)`` boolean masks."""
    grid = np.asarray(data)
    unknown = grid < 0
    occupied = grid >= int(occupied_threshold)
    free = ~unknown & ~occupied
    return free, occupied, unknown


def frontier_mask(data: np.ndarray, occupied_threshold: int = 50) -> np.ndarray:
    """Mark free cells that touch at least one unknown cell (4-connectivity)."""
    free, _, unknown = occupancy_arrays(data, occupied_threshold)
    touching = np.zeros_like(unknown)
    touching[1:, :] |= unknown[:-1, :]
    touching[:-1, :] |= unknown[1:, :]
    touching[:, 1:] |= unknown[:, :-1]
    touching[:, :-1] |= unknown[:, 1:]
    return free & touching


def inflate(
    occupied: np.ndarray, radius_m: float, resolution: float, minimum_cells: int = 1
) -> np.ndarray:
    """Dilate the occupied mask by ``radius_m`` (used for goal clearance)."""
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    cells = max(int(minimum_cells), int(math.ceil(float(radius_m) / resolution)))
    return ndimage.binary_dilation(occupied, iterations=cells)


def _nearest_free_label(
    free_labels: np.ndarray,
    cx: int,
    cy: int,
    info: "GridInfo",
    max_radius_m: float = 1.0,
) -> int:
    """Label of the free-space component nearest to cell ``(cx, cy)``, or 0.

    Used when the robot's own cell carries label 0 because the map has not
    observed it (the floor directly beneath the base is never seen by a
    forward-facing depth camera). The robot is physically in free space, so the
    closest free cell is the honest proxy for which component it occupies.

    Rings are searched outward so the answer is the genuinely nearest cell, not
    merely the first one found by a row-major scan. Returns 0 if nothing free
    lies within ``max_radius_m``.
    """
    if free_labels.size == 0 or not info.contains_cell(cx, cy):
        return 0
    max_cells = int(math.ceil(max_radius_m / info.resolution)) if info.resolution > 0 else 0
    height, width = free_labels.shape
    for radius in range(0, max_cells + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                # Only the ring at exactly this Chebyshev radius, so the first
                # radius that yields anything is the nearest by construction.
                if max(abs(dx), abs(dy)) != radius:
                    continue
                x, y = cx + dx, cy + dy
                if x < 0 or y < 0 or x >= width or y >= height:
                    continue
                label = int(free_labels[y, x])
                if label > 0:
                    return label
    return 0


def detect_frontiers(
    data: np.ndarray,
    info: GridInfo,
    robot_xy: Optional[Tuple[float, float]] = None,
    min_size: int = 8,
    occupied_threshold: int = 50,
    goal_clearance: float = 0.0,
    require_reachable: bool = True,
    max_frontiers: int = 64,
    stats: Optional[dict] = None,
) -> List[Frontier]:
    """Return the frontier clusters of a grid, largest first.

    ``min_size`` drops clusters smaller than that many cells, which removes
    sensor-noise slivers that would otherwise generate jittery goals.
    ``goal_clearance`` rejects goals whose centroid falls within that distance
    of an occupied cell, so the robot is never sent into a wall.
    ``require_reachable`` keeps only frontiers reachable through the same
    free-space component that currently contains the robot.

    ``stats``, when given, is cleared and filled with the per-stage counts
    (``mask_cells``, ``clusters``, ``dropped_small``, ``dropped_unreachable``,
    ``dropped_no_goal_cell``, ``kept``, ``robot_label``). The caller cannot
    otherwise tell "the map has no frontier" from "every frontier was filtered
    out", and an empty goal list looks identical in both cases -- which is
    exactly the ambiguity that made the 2026-10-18 run unfalsifiable from its
    logs (one goal driven, then COMPLETE, with no record of why).
    """
    if stats is not None:
        stats.clear()
        stats.update(
            mask_cells=0,
            clusters=0,
            dropped_small=0,
            dropped_unreachable=0,
            dropped_no_goal_cell=0,
            kept=0,
            robot_label=0,
        )

    if not info.valid:
        return []
    grid = np.asarray(data)
    if grid.shape != (info.height, info.width):
        return []

    mask = frontier_mask(grid, occupied_threshold)
    if stats is not None:
        stats["mask_cells"] = int(mask.sum())
    if not mask.any():
        return []

    free, occupied, _ = occupancy_arrays(grid, occupied_threshold)

    blocked = None
    if goal_clearance > 0.0:
        blocked = inflate(occupied, goal_clearance, info.resolution)

    component_labels = None
    robot_label = 0
    if require_reachable:
        free_labels, count = ndimage.label(free, structure=np.ones((3, 3), dtype=int))
        component_labels = free_labels
        if robot_xy is not None:
            rcx, rcy = info.world_to_cell(*robot_xy)
            if info.contains_cell(rcx, rcy):
                robot_label = int(component_labels[rcy, rcx])
            if robot_label <= 0:
                # The robot's own cell is often UNKNOWN rather than free: the
                # depth camera looks forward and down, so the floor directly
                # under the base is never observed. ndimage.label gives such a
                # cell label 0, and the caller used to treat that as "cannot
                # determine reachability" and silently skip the whole filter --
                # which made `require_reachable: true` a no-op on a fresh map
                # (measured: robot cell unknown, robot_label 0).
                #
                # The robot is physically standing in free space, so the nearest
                # free cell is the right proxy for "which component am I in".
                robot_label = _nearest_free_label(
                    free_labels, rcx, rcy, info, max_radius_m=1.0
                )
                if robot_label > 0:
                    logger.debug(
                        "robot cell (%d, %d) not free; using nearest free "
                        "component %d as the reachability reference",
                        rcx, rcy, robot_label,
                    )
                else:
                    # No free space anywhere near the robot. Falling back to
                    # permissive (as the old code did inadvertently) rather than
                    # rejecting every frontier, but SAYING SO -- silently
                    # disabling a check the config asked for is what caused this
                    # to go unnoticed. Motion safety does not depend on this
                    # filter; cmd_vel_safety and obstacle_monitor are the guards.
                    logger.warning(
                        "require_reachable is set but no free cell was found "
                        "within 1.0 m of the robot; skipping the reachability "
                        "filter this cycle"
                    )

    if stats is not None:
        stats["robot_label"] = int(robot_label)

    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    if stats is not None:
        stats["clusters"] = int(count)
    candidates: List[Frontier] = []
    for label_id in range(1, count + 1):
        ys, xs = np.nonzero(labels == label_id)
        size = int(xs.size)
        if size < int(min_size):
            if stats is not None:
                stats["dropped_small"] += 1
            continue

        if require_reachable and component_labels is not None:
            if robot_label > 0:
                keep = component_labels[ys, xs] == robot_label
                if not keep.any():
                    if stats is not None:
                        stats["dropped_unreachable"] += 1
                    continue
                ys, xs = ys[keep], xs[keep]
                size = int(xs.size)

        # Goal cell: prefer the cluster's mean position, because that is the
        # most central (and therefore safest) point of the frontier. Only when
        # the mean cell is not usable -- unknown, occupied, inflated or off the
        # grid -- fall back to the nearest usable frontier cell.
        mean_x = float(xs.mean())
        mean_y = float(ys.mean())
        gx, gy = int(round(mean_x)), int(round(mean_y))

        def _usable(x: int, y: int) -> bool:
            if not info.contains_cell(x, y):
                return False
            if not bool(free[y, x]):
                return False
            return not (blocked is not None and bool(blocked[y, x]))

        if not _usable(gx, gy):
            best = None
            best_d = float("inf")
            for x, y in zip(xs.tolist(), ys.tolist()):
                if not _usable(x, y):
                    continue
                d = (x - mean_x) ** 2 + (y - mean_y) ** 2
                if d < best_d:
                    best_d = d
                    best = (x, y)
            if best is None:
                if stats is not None:
                    stats["dropped_no_goal_cell"] += 1
                continue
            gx, gy = best

        wx, wy = info.cell_to_world(gx, gy)
        cells = list(zip(xs.tolist(), ys.tolist()))
        candidates.append(Frontier(centroid=(wx, wy), size=size, cells=cells))

    candidates.sort(key=lambda f: f.size, reverse=True)
    kept = candidates[: int(max_frontiers)]
    if stats is not None:
        stats["kept"] = len(kept)
    return kept


def estimate_gain(
    data: np.ndarray,
    info: GridInfo,
    frontier: Frontier,
    radius_m: float,
    occupied_threshold: int = 50,
) -> float:
    """Count unknown cells within ``radius_m`` of a frontier centroid.

    This approximates the information a robot would gain by driving there and
    is a better predictor of useful exploration than cluster size alone.
    """
    _, _, unknown = occupancy_arrays(data, occupied_threshold)
    gx, gy = info.world_to_cell(*frontier.centroid)
    r = max(1, int(math.ceil(radius_m / info.resolution)))
    x0, x1 = max(0, gx - r), min(info.width, gx + r + 1)
    y0, y1 = max(0, gy - r), min(info.height, gy + r + 1)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    return float(unknown[y0:y1, x0:x1].sum())


def _min_max_norm(values: np.ndarray) -> np.ndarray:
    """Scale values into ``[0, 1]``; a constant array maps to all ones."""
    lo, hi = float(values.min()), float(values.max())
    if hi - lo < 1e-9:
        return np.ones_like(values)
    return (values - lo) / (hi - lo)


def score_frontiers(
    frontiers: Sequence[Frontier],
    robot_xy: Tuple[float, float],
    robot_yaw: float,
    weight_size: float = 1.0,
    weight_distance: float = 1.0,
    weight_heading: float = 0.6,
    weight_gain: float = 1.5,
    data: Optional[np.ndarray] = None,
    info: Optional[GridInfo] = None,
    gain_radius: float = 0.0,
) -> None:
    """Score frontiers in place, higher is better.

    ``score = w_size * size_norm + w_gain * gain_norm - w_distance * dist_norm
    - w_heading * |heading error| / pi``

    Every term is normalised so the weights stay comparable when the map
    resolution or the room size changes. The heading term is what keeps the
    robot from repeatedly turning around to chase a frontier behind it.
    The gain term is only added when a gain radius and grid are supplied;
    otherwise it contributes nothing rather than biasing every goal equally.
    """
    if not frontiers:
        return

    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    sizes = np.array([max(1, f.size) for f in frontiers], dtype=float)
    distances = np.array(
        [math.hypot(f.centroid[0] - rx, f.centroid[1] - ry) for f in frontiers],
        dtype=float,
    )

    use_gain = data is not None and info is not None and gain_radius > 0.0
    if use_gain:
        gains = np.array(
            [estimate_gain(data, info, f, gain_radius) for f in frontiers], dtype=float
        )
    else:
        gains = np.zeros(len(frontiers), dtype=float)

    size_norm = _min_max_norm(sizes)
    dist_norm = _min_max_norm(distances)
    gain_norm = _min_max_norm(gains) if use_gain else np.zeros_like(gains)

    for i, frontier in enumerate(frontiers):
        bearing = math.atan2(frontier.centroid[1] - ry, frontier.centroid[0] - rx)
        heading_error = abs(wrap_angle(bearing - robot_yaw))
        frontier.distance = float(distances[i])
        frontier.gain = float(gains[i])
        frontier.heading_error = float(heading_error)
        frontier.score = float(
            weight_size * size_norm[i]
            + (weight_gain * gain_norm[i] if use_gain else 0.0)
            - weight_distance * dist_norm[i]
            - weight_heading * (heading_error / math.pi)
        )


class GoalBlacklist:
    """Ring of rejected goals with a time-to-live.

    A frontier that Nav2 could not reach must not be retried forever, but the
    map keeps changing, so the rejection expires.
    """

    def __init__(self, radius: float = 1.0, ttl: float = 60.0) -> None:
        self.radius = float(radius)
        self.ttl = float(ttl)
        self._entries: List[Tuple[float, float, float]] = []

    def add(self, x: float, y: float, now: float) -> None:
        self._entries.append((float(x), float(y), float(now) + self.ttl))

    def prune(self, now: float) -> None:
        self._entries = [e for e in self._entries if e[2] > float(now)]

    def blocked(self, x: float, y: float, now: float) -> bool:
        self.prune(now)
        r2 = self.radius * self.radius
        for ex, ey, _ in self._entries:
            if (ex - x) ** 2 + (ey - y) ** 2 <= r2:
                return True
        return False

    def clear(self) -> None:
        self._entries.clear()

    def __len__(self) -> int:
        return len(self._entries)


class UnexplorableRegions:
    """Areas that repeated attempts proved unreachable, for the whole run.

    Why this is a separate structure and not just a long ``GoalBlacklist`` TTL:
    the two make different statements about the world, and the difference is
    exactly what the operator needs to see.

    * A blacklist entry says "not right now" -- Nav2 just failed there, the map
      is still changing, so a retry later is reasonable. It expires.
    * An entry here says "we tried this area repeatedly and it does not work",
      e.g. the frontier sits behind furniture the rover cannot pass. Retrying it
      is precisely what converts one failure into a run full of failures: with
      the goal timeout at 60 s, three unreachable frontiers in a row are three
      minutes of the budget spent re-proving the same thing, and they end the
      run via the consecutive-failure abort.

    So these entries never expire, and they are published (``as_list``) as the
    operator-visible answer to "where can this robot not go?" -- which is also
    what keeps the abort message honest: it can now distinguish "no reachable
    frontier left" from "the places we could not reach have been excluded".
    """

    def __init__(self, radius: float = 1.5) -> None:
        self.radius = float(radius)
        self._entries: List[Tuple[float, float]] = []

    def add(self, x: float, y: float) -> None:
        x, y = float(x), float(y)
        if not self.blocked(x, y):
            self._entries.append((x, y))

    def blocked(self, x: float, y: float) -> bool:
        r2 = self.radius * self.radius
        for ex, ey in self._entries:
            if (ex - x) ** 2 + (ey - y) ** 2 <= r2:
                return True
        return False

    def as_list(self) -> List[dict]:
        return [{"x": x, "y": y} for x, y in self._entries]

    def clear(self) -> None:
        self._entries.clear()

    def __len__(self) -> int:
        return len(self._entries)


def select_goal(
    frontiers: Sequence[Frontier],
    blacklist: Optional[GoalBlacklist] = None,
    now: float = 0.0,
    min_distance: float = 0.0,
    max_distance: float = float("inf"),
    robot_xy: Optional[Tuple[float, float]] = None,
    unexplorable: Optional[UnexplorableRegions] = None,
    stats: Optional[dict] = None,
) -> Optional[Frontier]:
    """Return the highest scoring frontier that passes all filters.

    Only a frontier already scored by :func:`score_frontiers` (or one with a
    manually assigned ``score``) can be selected. ``None`` means the caller
    should treat exploration as finished for this cycle.

    ``stats``, when given, accumulates one count per rejection reason
    (``unscored``, ``out_of_range``, ``blacklisted``, ``unexplorable``) plus
    ``selected`` (0/1). It is *not* cleared here: the caller passes the same
    dict to :func:`detect_frontiers`, and the two halves answer different
    questions -- "were there any frontiers" versus "why was none of them
    pickable".
    """
    if stats is not None:
        for key in ("unscored", "out_of_range", "blacklisted", "unexplorable"):
            stats.setdefault(key, 0)
        stats["selected"] = 0

    best: Optional[Frontier] = None
    for frontier in frontiers:
        if frontier.score == float("-inf"):
            if stats is not None:
                stats["unscored"] += 1
            continue
        if not (min_distance <= frontier.distance <= max_distance):
            if stats is not None:
                stats["out_of_range"] += 1
            continue
        if blacklist is not None and blacklist.blocked(
            frontier.centroid[0], frontier.centroid[1], now
        ):
            if stats is not None:
                stats["blacklisted"] += 1
            continue
        if unexplorable is not None and unexplorable.blocked(
            frontier.centroid[0], frontier.centroid[1]
        ):
            if stats is not None:
                stats["unexplorable"] += 1
            continue
        if best is None or frontier.score > best.score:
            best = frontier
    if stats is not None and best is not None:
        stats["selected"] = 1
    return best
