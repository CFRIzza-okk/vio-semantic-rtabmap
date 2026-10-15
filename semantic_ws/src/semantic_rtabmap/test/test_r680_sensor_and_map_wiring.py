"""Guards for the two silent mis-wirings that made Phase C fail.

Why this file exists
--------------------
Both defects below had the same signature: **nothing errored**.  The launch
succeeded, the node started, `ros2 node list` showed it, and a topic of the
expected name appeared in `ros2 topic list` — but no data ever flowed.  They
were found only by asking "is the sensor actually producing messages?", and the
second one produced a *misleading* symptom that pointed at the wrong subsystem.

1. **`depthimage_to_laserscan` never received ``camera_info``.**  The launch
   file remapped ``depth`` and ``scan`` but not the camera-info input, so the
   node stayed subscribed to its built-in default ``/depth_camera_info`` — a
   topic with **zero publishers**.  Every frame it printed::

       [depthimage_to_laserscan]: No camera info, skipping point cloud squash

   and ``/scan`` was empty forever (measured: 0 messages).  The dangerous part
   is the *downstream* symptom: `r680_exploration.yaml` deliberately treats a
   missing scan as "obstacle present", so ``/safety/obstacle_stop`` read
   ``true`` and looked like a real obstacle detection.  Anyone debugging that
   would go looking at the room, not at a remap.

   The trap within the trap: the remap **source name is
   ``depth_camera_info``, not ``camera_info``**.  Writing
   ``("camera_info", ...)`` is accepted silently and changes nothing, because
   the node builds its subscription on ``depth_camera_info``.  Upstream's own
   ``depthimage_to_laserscan-launch.py`` remaps
   ``('depth_camera_info', '/camera/depth/camera_info')``, which is the
   authoritative spelling this test pins.

2. **The map topic was ``/map`` in three places, but RTAB-Map publishes
   ``/rtabmap/map``.**  ``rtabmap.launch.py`` defaults its namespace to
   ``rtabmap``.  A ``nav2_costmap_2d::StaticLayer`` with no explicit
   ``map_topic`` subscribes to ``/map``, so the global costmap would sit empty
   and the planner would happily route through unmapped space.  The repo
   already knew about the namespace — ``mapper.yaml`` documents it and
   ``mapper_node.py`` dual-publishes landmarks to paper over it — but the Nav2
   and exploration configs had been written against the assumed ``/map``.

   Both costmap-related values are also *fail-silent*: an empty static layer
   throws no error, it just makes every unknown cell free.

These guards read the files as text/AST rather than importing the launch
modules, so they run without a ROS installation or a live node graph.
"""

import ast
import re
from pathlib import Path

import pytest
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PKG_ROOT / "config"
LAUNCH_DIR = PKG_ROOT / "launch"

EXPLORATION_LAUNCH = LAUNCH_DIR / "r680_exploration.launch.py"
NAV2_YAML = CONFIG_DIR / "r680_nav2.yaml"
EXPLORATION_YAML = CONFIG_DIR / "r680_exploration.yaml"

# The frame a LaserScan must be expressed in.
#
# This was originally written the other way round, and that was WRONG. The
# reasoning then was "the aligned-depth image and its camera_info both carry
# camera_color_optical_frame, so the scan should too" -- which conflates two
# different questions:
#
#   * which frame the DEPTH IMAGE data is expressed in  -> color optical frame
#   * which frame a LASERSCAN's angle semantics require -> x-y plane horizontal
#
# REP-103 puts a LaserScan in the x-y plane of its frame with z up. Measured on
# the live TF tree (2026-10-14):
#
#   base_link -> camera_depth_frame          RPY [0.000, 1.660, 0.000] deg
#   base_link -> camera_color_optical_frame  RPY [-91.653, 0.124, -90.678] deg
#   camera_depth_frame -> camera_color_optical_frame  ~90 deg apart
#
# So camera_depth_frame shares camera_link's orientation (x fwd, y left, z up)
# and its x-y plane IS horizontal, while an optical frame (z fwd, x right, y
# down) has a VERTICAL x-y plane -- the image plane.
#
# Concretely: the scan's centre beam (angle 0, range r) is the point (r, 0, 0)
# in the scan frame. In camera_depth_frame that transforms to r metres straight
# ahead of the robot; in camera_color_optical_frame it lands off to the side.
#
# The safety monitor never noticed because it reads msg.angle_min/increment
# directly with no TF lookup -- which is exactly why this stayed hidden. Nav2's
# costmap obstacle layer DOES transform the scan into the global frame, so the
# optical frame would scatter phantom obstacles vertically, off by 90 degrees.
#
# The 15 mm lateral offset that motivated the original choice is real but tiny;
# a 90 degree orientation error is not. Orientation wins.
EXPECTED_SCAN_FRAME = "camera_depth_frame"
WRONG_SCAN_FRAME = "camera_color_optical_frame"

# The aligned-depth camera_info is the one that matches the depth image.
ALIGNED_CAMERA_INFO = "/camera/camera/aligned_depth_to_color/camera_info"

MAP_TOPIC = "/rtabmap/map"


def _node_call(executable: str) -> ast.Call:
    """The ``Node(...)`` call in the exploration launch whose executable matches."""
    tree = ast.parse(EXPLORATION_LAUNCH.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not (isinstance(node.func, ast.Name) and node.func.id == "Node"):
            continue
        for kw in node.keywords:
            if (kw.arg == "executable"
                    and isinstance(kw.value, ast.Constant)
                    and kw.value.value == executable):
                return node
    pytest.fail("no Node(executable=%r) in %s" % (executable, EXPLORATION_LAUNCH.name))


def _literal(node):
    """Best-effort value of a remap endpoint.

    The launch file assigns its topics to module-level names first
    (``depth_topic = LaunchConfiguration("depth_topic")``) and then refers to
    the *bare name* inside ``remappings``.  So an endpoint is usually an
    ``ast.Name``, not an inline ``LaunchConfiguration(...)`` call - handling
    only the call form silently yields ``None`` for every target.
    """
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        return "<var:%s>" % node.id
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        if node.func.id == "LaunchConfiguration" and node.args:
            return "<launch-config:%s>" % getattr(node.args[0], "value", "?")
    return None


def _remappings(executable: str) -> dict:
    """``{source: target-literal}`` from a node's ``remappings=[(a, b), ...]``."""
    node = _node_call(executable)
    for kw in node.keywords:
        if kw.arg != "remappings":
            continue
        pairs = {}
        for element in kw.value.elts:
            assert isinstance(element, ast.Tuple) and len(element.elts) == 2, (
                "each remapping must be a 2-tuple"
            )
            pairs[_literal(element.elts[0])] = _literal(element.elts[1])
        return pairs
    pytest.fail("Node(executable=%r) has no remappings list" % executable)


def _launch_default(name: str) -> str:
    """The ``default_value`` of a ``DeclareLaunchArgument`` in the exploration launch."""
    text = EXPLORATION_LAUNCH.read_text(encoding="utf-8")
    match = re.search(
        r'DeclareLaunchArgument\(\s*"%s"\s*,\s*default_value\s*=\s*"([^"]+)"'
        % re.escape(name),
        text,
    )
    assert match, "DeclareLaunchArgument %r not found" % name
    return match.group(1)


# --------------------------------------------------------------------------
# 1. depthimage_to_laserscan must receive camera_info
# --------------------------------------------------------------------------

def test_camera_info_remap_uses_the_nodes_real_topic_name():
    """The remap source must be ``depth_camera_info``, not ``camera_info``.

    This is the whole defect in one assertion.  ``camera_info`` looks right and
    is silently ignored; ``depth_camera_info`` is the name the node actually
    subscribes to.
    """
    pairs = _remappings("depthimage_to_laserscan_node")
    assert "depth_camera_info" in pairs, (
        "the camera-info input must be remapped from 'depth_camera_info' - that is "
        "the topic name this node subscribes to. Remapping 'camera_info' instead is "
        "accepted silently and leaves the node waiting on a topic with zero "
        "publishers, so /scan never produces a single message. Remappings found: %r"
        % sorted(pairs)
    )
    assert "camera_info" not in pairs, (
        "remapping the source name 'camera_info' does nothing for this node - the "
        "real subscription is 'depth_camera_info'"
    )


def test_camera_info_points_at_the_aligned_depth_stream():
    """The remap target must be the camera_info that matches the depth image.

    Feeding the *colour* camera_info (or the raw depth one) to a scan built from
    the aligned-depth image would put the beams in the wrong optical frame.
    """
    pairs = _remappings("depthimage_to_laserscan_node")
    target = pairs["depth_camera_info"]
    assert target == "<var:camera_info_topic>", (
        "camera_info should be remapped to the camera_info_topic launch argument "
        "so it stays overridable; got %r" % (target,)
    )
    assert _launch_default("camera_info_topic") == ALIGNED_CAMERA_INFO, (
        "camera_info_topic must default to the aligned-depth camera_info %r; "
        "the depth image being converted is aligned_depth_to_color/image_raw"
        % ALIGNED_CAMERA_INFO
    )


def test_scan_output_frame_has_a_horizontal_xy_plane():
    """``output_frame`` must be a frame whose x-y plane is horizontal.

    A LaserScan's angles are defined in its frame's x-y plane (REP-103), so the
    frame must carry an x-forward / z-up orientation. ``camera_depth_frame`` does
    (it is identity-rotated w.r.t. camera_link); ``camera_color_optical_frame``
    does not -- it is ~90 deg away, putting the scan in the vertical image plane.

    The safety monitor would not notice either way because it reads the angles
    directly without a TF lookup, but Nav2's costmap transforms the scan and
    would place the obstacles 90 deg off.
    """
    data = yaml.safe_load(EXPLORATION_YAML.read_text(encoding="utf-8"))
    params = data["depthimage_to_laserscan"]["ros__parameters"]
    frame = params["output_frame"]
    assert frame == EXPECTED_SCAN_FRAME, (
        "output_frame is %r, but a LaserScan needs a frame with a horizontal "
        "x-y plane; expected %r. An optical frame (z forward, x right, y down) "
        "would make the scan vertical for any consumer that transforms it, such "
        "as the Nav2 costmap obstacle layer." % (frame, EXPECTED_SCAN_FRAME)
    )
    assert frame != WRONG_SCAN_FRAME, (
        "output_frame must not be an optical frame -- it puts the scan in the "
        "image plane, 90 deg away from where the beams actually point"
    )
    assert not frame.endswith("_optical_frame"), (
        "%r looks like an optical frame; those have z forward and a vertical "
        "x-y plane, which is incompatible with LaserScan angle semantics" % frame
    )


def test_scan_is_remapped_and_uses_the_depth_image():
    """Sanity: the two remaps that *were* already right must stay right."""
    pairs = _remappings("depthimage_to_laserscan_node")
    assert pairs.get("depth") == "<var:depth_topic>"
    assert pairs.get("scan") == "<var:scan_topic>"
    assert _launch_default("depth_topic") == (
        "/camera/camera/aligned_depth_to_color/image_raw"
    )


# --------------------------------------------------------------------------
# 2. every consumer must agree with RTAB-Map on the map topic
# --------------------------------------------------------------------------

def test_nav2_static_layer_names_the_rtabmap_map_topic():
    """The global costmap must subscribe to where RTAB-Map actually publishes.

    Without an explicit ``map_topic`` the StaticLayer falls back to ``/map`` and
    silently builds an empty map - no error, every unknown cell treated as free.
    """
    text = NAV2_YAML.read_text(encoding="utf-8")
    match = re.search(r"^\s*map_topic:\s*(\S+)\s*$", text, re.M)
    assert match, (
        "nav2 static_layer has no explicit map_topic. RTAB-Map publishes %r "
        "(namespace defaults to 'rtabmap'), so relying on the '/map' default "
        "leaves the global costmap empty" % MAP_TOPIC
    )
    assert match.group(1) == MAP_TOPIC, (
        "static_layer map_topic is %r, expected %r" % (match.group(1), MAP_TOPIC)
    )


def test_exploration_node_names_the_rtabmap_map_topic():
    """The frontier explorer reads the same map, and had the same wrong default."""
    data = yaml.safe_load(EXPLORATION_YAML.read_text(encoding="utf-8"))
    topic = data["r680_explorer"]["ros__parameters"]["map_topic"]
    assert topic == MAP_TOPIC, (
        "r680_explorer map_topic is %r, expected %r - subscribed to a topic nobody "
        "publishes, the explorer can never select a frontier" % (topic, MAP_TOPIC)
    )


def test_exploration_launch_does_not_override_the_map_topic_with_map():
    """The launch file sets map_topic in code, so it can silently undo the YAML."""
    text = EXPLORATION_LAUNCH.read_text(encoding="utf-8")
    literals = re.findall(r'"map_topic"\s*:\s*"([^"]+)"', text)
    assert literals, "expected the explorer's map_topic to be set in the launch file"
    for value in literals:
        assert value == MAP_TOPIC, (
            "the exploration launch hard-codes map_topic=%r; it must be %r or it "
            "overrides the config file" % (value, MAP_TOPIC)
        )


def test_map_topic_consumers_all_agree():
    """Every mention of a map topic across these files must be the same string.

    Guards against fixing one consumer and leaving another behind - which is
    exactly how this defect survived: the semantic layer knew about the
    namespace and the navigation layer did not.
    """
    seen = {}
    seen["nav2 static_layer"] = re.search(
        r"^\s*map_topic:\s*(\S+)\s*$", NAV2_YAML.read_text(encoding="utf-8"), re.M
    ).group(1)
    seen["exploration yaml"] = yaml.safe_load(
        EXPLORATION_YAML.read_text(encoding="utf-8")
    )["r680_explorer"]["ros__parameters"]["map_topic"]
    seen["exploration launch"] = re.findall(
        r'"map_topic"\s*:\s*"([^"]+)"',
        EXPLORATION_LAUNCH.read_text(encoding="utf-8"),
    )[0]

    assert set(seen.values()) == {MAP_TOPIC}, (
        "map_topic consumers disagree: %r - they must all be %r" % (seen, MAP_TOPIC)
    )
