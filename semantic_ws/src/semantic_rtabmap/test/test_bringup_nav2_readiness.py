#!/usr/bin/env python3
"""The bring-up scripts must not reproduce two silent, stack-breaking mistakes.

Both were found the first time Nav2 was actually started (2026-10-15), and both
share the shape this project keeps meeting: **nothing reports an error**, and the
visible symptom points somewhere other than the cause.

1. `odom_frame_id` must be passed explicitly to RTAB-Map.
   Leaving it unset does NOT mean "sensible default" -- it means the empty
   string, and RTAB-Map then publishes NO `map -> *` transform at all. `map`
   stops existing as a frame, global_costmap waits for it forever, planner_server
   never finishes activating, and lifecycle_manager's ordering blocks the rest:
   bt_navigator, behavior_server and waypoint_follower all stay `inactive` while
   controller_server alone is `active`.

   The log line that names the cause is an INFO, not an ERROR:

       Invalid frame ID "map" passed to canTransform argument target_frame
       - frame does not exist

   Phase D never caught this because Phase D does not start Nav2.

2. The Orin side must be cleared before a bring-up.
   Only clearing TX1 means a second run stacks a whole second stack on top of the
   first: measured 22 -> 43 nodes, one relay -> three. Two publishers then contend
   on /cmd_vel_safe, two publish localization_valid, two publish /scan. No error
   is raised anywhere; the only sign is `Publisher count: 2`, which reads as
   normal. It is the same root cause as the three spurious failures in
   test_safety_monitors.py, one level down.

Both are checked structurally, against the scripts that actually run, because a
text search is exactly the kind of check that has passed while the guarded code
was gone -- four times in this repo already.
"""
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOLS = REPO_ROOT / "tools"

BRINGUP_SCRIPTS = ["r680_explore_up.sh", "r680_motion_up.sh"]
SCRIPT_PATHS = [TOOLS / n for n in BRINGUP_SCRIPTS]


def _read(p):
    assert p.exists(), "%s is missing" % p.name
    return p.read_text()


def _rtabmap_invocation(text):
    """Return the text of the rtabmap_launch invocation, args included.

    Joined continuation lines first, so a multi-line launch command is inspected
    as the single command it is. A per-line scan silently sees fragments.
    """
    joined = re.sub(r"\\\s*\n\s*", " ", text)
    for line in joined.splitlines():
        if "rtabmap.launch.py" in line:
            return line
    return ""


# --------------------------------------------------------------------------
# 1. odom_frame_id
# --------------------------------------------------------------------------
@pytest.mark.parametrize("path", SCRIPT_PATHS, ids=[p.name for p in SCRIPT_PATHS])
def test_bringup_passes_odom_frame_id_to_rtabmap(path):
    """Without it RTAB-Map publishes no map->* transform and Nav2 cannot start."""
    inv = _rtabmap_invocation(_read(path))
    assert inv, "%s: no rtabmap.launch.py invocation found" % path.name
    assert "odom_frame_id:=" in inv, (
        "%s starts RTAB-Map without odom_frame_id.\n"
        "An unset odom_frame_id is the EMPTY STRING, not a sensible default:\n"
        "RTAB-Map then publishes no map->* transform, `map` does not exist as a\n"
        "frame, global_costmap blocks forever, and Nav2 comes up with only\n"
        "controller_server active. Nothing logs an error."
        % path.name
    )


@pytest.mark.parametrize("path", SCRIPT_PATHS, ids=[p.name for p in SCRIPT_PATHS])
def test_odom_frame_id_matches_what_the_relay_actually_publishes(path):
    """It must name the frame the wheel odometry really uses.

    TX1's /odom was measured as frame_id `odom_combined`, child `base_footprint`.
    Nav2's costmaps use `global_frame: odom_combined` and the explorer config
    uses `odom_frame: odom_combined`, so all four must agree -- a mismatch would
    put the robot's own odometry in a frame nothing else references.
    """
    inv = _rtabmap_invocation(_read(path))
    m = re.search(r"odom_frame_id:=(\S+)", inv)
    assert m, "%s: odom_frame_id present but unparsable" % path.name
    frame = m.group(1)
    assert frame == "odom_combined", (
        "%s passes odom_frame_id:=%s, but the relay publishes /odom with "
        "frame_id 'odom_combined'. RTAB-Map would hang map off a frame that "
        "does not exist." % (path.name, frame)
    )


@pytest.mark.parametrize("path", SCRIPT_PATHS, ids=[p.name for p in SCRIPT_PATHS])
def test_visual_odometry_stays_off(path):
    """The other half of the pair, and easy to confuse with the fix above.

    `odom_frame_id` says which frame RTAB-Map TRUSTS as odometry.
    `visual_odometry:=true` makes rgbd_odometry COMPUTE one and publish /odom,
    which then collides with the relayed wheel odometry. The two settings are
    independent, so fixing one must not disturb the other.
    """
    inv = _rtabmap_invocation(_read(path))
    assert "visual_odometry:=false" in inv, (
        "%s must pass visual_odometry:=false; rgbd_odometry would publish a "
        "second /odom and fight the relayed wheel odometry" % path.name
    )
    assert "icp_odometry:=false" in inv, (
        "%s must pass icp_odometry:=false for the same reason" % path.name
    )


def test_the_nav2_config_agrees_on_the_odom_frame():
    """The two costmaps must use DIFFERENT frames, and both must exist.

    This is the other end of the odom_frame_id defect, and it is what made the
    symptom so confusing: global_costmap wants `map`, and the only thing that
    can produce a `map` frame here is RTAB-Map's `map -> odom_combined`
    transform -- which RTAB-Map never published while odom_frame_id was empty.
    So global_costmap waited forever on a frame whose producer was silently
    inert, and Nav2 came up with a single active node.

    The split is deliberate, not an inconsistency:
      * global_costmap in `map`       -- the global plan must live in the fixed
                                         map frame, and must therefore exist
      * local_costmap in `odom_combined` -- the rolling local window belongs in
                                         the continuous odometry frame, so map
                                         updates do not jitter it
    """
    nav2 = (REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap"
            / "config" / "r680_nav2.yaml")
    text = nav2.read_text()

    # Split per costmap section rather than grepping every global_frame at once:
    # the two values are meant to differ, so a blanket equality check is wrong.
    sections = {}
    current = None
    for line in text.splitlines():
        if re.match(r"\s*(global_costmap|local_costmap)\s*:", line):
            current = re.match(r"\s*(\w+)\s*:", line).group(1)
            sections[current] = []
        elif current and "global_frame:" in line:
            sections[current].append(line.split("global_frame:")[1].strip())

    assert "global_costmap" in sections, "no global_costmap section found"
    assert "local_costmap" in sections, "no local_costmap section found"
    assert sections["global_costmap"], "global_costmap has no global_frame"
    assert sections["local_costmap"], "local_costmap has no global_frame"

    assert sections["global_costmap"][0] == "map", (
        "global_costmap.global_frame is %s, expected 'map'. In 'odom_combined' "
        "the global plan would drift with odometry and never be anchored to the "
        "map." % sections["global_costmap"][0]
    )
    assert sections["local_costmap"][0] == "odom_combined", (
        "local_costmap.global_frame is %s, expected 'odom_combined' (the frame "
        "TX1's /odom actually uses)" % sections["local_costmap"][0]
    )


def test_explorer_uses_the_same_odom_frame_as_the_relay():
    """The explorer TF-checks localization against this frame; it must match."""
    cfg = (REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap"
           / "config" / "r680_exploration.yaml")
    text = cfg.read_text()
    m = re.search(r"^\s*odom_frame:\s*(\S+)", text, re.M)
    assert m, "r680_exploration.yaml has no odom_frame"
    assert m.group(1) == "odom_combined", (
        "the explorer expects odom_frame: %s, but the relay publishes /odom in "
        "'odom_combined'; its localization check would fail" % m.group(1)
    )


# --------------------------------------------------------------------------
# 2. Orin-side cleanup
# --------------------------------------------------------------------------
@pytest.mark.parametrize("path", SCRIPT_PATHS, ids=[p.name for p in SCRIPT_PATHS])
def test_bringup_clears_the_orin_side(path):
    """Clearing only TX1 stacks a second full stack on the first.

    22 -> 43 nodes, one relay -> three, two publishers on /cmd_vel_safe. Nothing
    errors; the only symptom is `Publisher count: 2`.
    """
    text = _read(path)
    # safe_kill must be invoked somewhere after the Orin ROS environment is set up
    assert "safe_kill.py" in text, (
        "%s never calls tools/safe_kill.py; the Orin side would accumulate a "
        "second copy of every node on every run" % path.name
    )
    # It has to happen on the Orin, i.e. after ROS_DOMAIN_ID is exported, since
    # `ros2 node list` needs the environment.
    orin = text.find("export ROS_DOMAIN_ID=99")
    assert orin != -1, "%s never sets ROS_DOMAIN_ID" % path.name
    assert text.find("safe_kill.py", orin) != -1, (
        "%s sets up the Orin ROS environment but never clears the Orin side "
        "afterwards" % path.name
    )


@pytest.mark.parametrize("path", SCRIPT_PATHS, ids=[p.name for p in SCRIPT_PATHS])
def test_bringup_asserts_the_orin_is_actually_empty(path):
    """Cleaning without checking is how 'I cleaned it' becomes a false belief.

    The failure this guards is not a crash -- it is a run that looks clean while
    two stacks are alive. Count the nodes and print the number.
    """
    text = _read(path)
    joined = re.sub(r"\\\s*\n\s*", " ", text)
    assert re.search(r"ros2 node list.{0,80}wc -l", joined, re.S), (
        "%s clears the Orin but never counts the remaining nodes; a partial "
        "cleanup would go unnoticed" % path.name
    )


def test_cleanup_uses_term_then_kill():
    """TERM first, so nodes can shut down cleanly; KILL as the backstop.

    KILL alone can leave the ROS 2 daemon holding stale registrations, which
    makes a clean Orin still report nodes.
    """
    for path in SCRIPT_PATHS:
        text = _read(path)
        assert "--signal TERM" in text, "%s: no TERM pass" % path.name
        assert "--signal KILL" in text, (
            "%s: no KILL backstop; a node ignoring TERM would survive and "
            "stack up" % path.name
        )
