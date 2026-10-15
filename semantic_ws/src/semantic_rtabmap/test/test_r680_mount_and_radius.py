"""Regression guards for the R680 mount geometry and the collision radius.

Why this file exists
--------------------
Two numbers in this package were, for a long time, placeholders that *looked*
configured and silently produced wrong behaviour.  Neither would fail a build,
a launch, or a log line — they only show up as a robot that maps badly or clips
a wall corner.

1. **The camera mount was all zeros.**  ``r680_navigation.launch.py`` shipped
   ``camera_x/y/z/roll/pitch/yaw = 0.0``.  A D435 is not at the centre of the
   robot: it sits ~15 cm forward and ~17 cm up, pitched ~2 deg down.  Telling
   the TF tree otherwise makes RTAB-Map place everything the camera sees at the
   wrong spot, and the error grows with rotation — the map smears and obstacles
   drift.  All zeros is also exactly what an *unfilled* placeholder looks like,
   which is precisely why it survived so long.

2. **``robot_radius`` disagreed between the two costmaps.**  The local and
   global costmaps each carry their own copy.  Nothing checks they match, and
   Nav2 will happily run with local=0.18 / global=0.20 — planning through gaps
   the local layer thinks are too narrow, or vice versa.  It also drifted from
   the real footprint.

The guards below assert *properties*, not the specific measured values, so a
legitimate re-measurement does not break them:

* the camera mount is not left at the placeholder origin, and the pitch is
  positive (in both conventions used here, positive pitch = looking down);
* the mount height is plausible rather than absurd;
* both costmaps agree on ``robot_radius``, and it is at least the circumscribed
  radius of the robot's real footprint — a radius smaller than the half-diagonal
  means the wheel corners can clip an obstacle the planner thought was clear.

Note on ``camera_z``: the measured 0.165 m is the lens height **above the floor**,
and it is used directly as ``camera_z`` because ``base_link`` z=0 *is* the
ground plane.  That comes from TX1's ``mini_mec_robot.urdf``, where the wheels
have r=0.0375 and are centred at z=0.0375 — so the wheel contact point sits at
z=0.  That URDF lives on TX1 and is not in this repo, so it is documented here
rather than asserted.  If the car's model ever changes, re-check it before
trusting ``camera_z``.

The value is asserted by *plausibility range*, never by literal, on purpose: it
is a physical measurement that will be redone, and a test pinned to today's
number would fail on the next honest re-measurement while catching nothing.  Two
independent live collections put it at 0.1639 +- 0.0027 and 0.1648 +- 0.0027 m,
about 5 mm below the first, biased-high estimate of 0.17 m.

Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_r680_mount_and_radius.py -v
"""

import math
import re
from pathlib import Path

import pytest
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PKG_ROOT / "config"
LAUNCH_DIR = PKG_ROOT / "launch"

NAV2_YAML = CONFIG_DIR / "r680_nav2.yaml"
EXPLORATION_YAML = CONFIG_DIR / "r680_exploration.yaml"
NAVIGATION_LAUNCH = LAUNCH_DIR / "r680_navigation.launch.py"

# The footprint the user measured by tape, in metres (30 x 25 cm).  This is the
# conservative choice: the URDF's chassis is slightly smaller, but the radius
# must cover whatever actually sticks out.
FOOTPRINT_LENGTH = 0.30
FOOTPRINT_WIDTH = 0.25

CAMERA_ARGS = ("camera_x", "camera_y", "camera_z",
               "camera_roll", "camera_pitch", "camera_yaw")


def _launch_camera_args() -> dict:
    """Extract ``camera_*`` DeclareLaunchArgument defaults from the launch file.

    Parsed from source text rather than by importing the launch module: the
    module needs a live rclpy/launch context to import cleanly, and this guard
    must run without ROS.
    """
    text = NAVIGATION_LAUNCH.read_text(encoding="utf-8")
    found = {}
    for name in CAMERA_ARGS:
        match = re.search(
            r'DeclareLaunchArgument\(\s*"%s"\s*,\s*default_value\s*=\s*"([^"]+)"'
            % re.escape(name),
            text,
        )
        assert match, "camera arg %r not found in %s" % (name, NAVIGATION_LAUNCH.name)
        found[name] = float(match.group(1))
    return found


def _nav2_robot_radii() -> list:
    """Every ``robot_radius`` in the Nav2 config, in file order."""
    text = NAV2_YAML.read_text(encoding="utf-8")
    return [float(v) for v in re.findall(r"^\s*robot_radius:\s*([0-9.]+)", text, re.M)]


def test_camera_mount_is_not_left_at_the_placeholder_origin():
    """All-zero is the unfilled placeholder, not a mount.

    This is the specific state this package shipped in for a long time.
    """
    args = _launch_camera_args()
    assert any(abs(v) > 1e-9 for v in args.values()), (
        "every camera_* default is 0.0 -- the measured mount was never applied, "
        "so the TF tree claims the D435 sits at the robot's rotation centre"
    )
    # The two we actually measured must not be zero.
    assert args["camera_z"] > 0.0, "camera_z is the measured mount height"
    assert args["camera_pitch"] != 0.0, "camera_pitch is the measured tilt"


def test_camera_mount_is_physically_plausible():
    """Catch a units slip or a digit dropped from a measured value."""
    args = _launch_camera_args()

    assert 0.05 < args["camera_z"] < 0.60, (
        "camera_z=%r m is not a plausible lens height for a %g m tall robot"
        % (args["camera_z"], 0.20)
    )
    # The lens must be somewhere on the chassis, not a metre off the front.
    assert abs(args["camera_x"]) < FOOTPRINT_LENGTH, (
        "camera_x=%r m is outside the %g m chassis" % (args["camera_x"], FOOTPRINT_LENGTH)
    )
    assert abs(args["camera_y"]) < FOOTPRINT_WIDTH, (
        "camera_y=%r m is outside the %g m chassis" % (args["camera_y"], FOOTPRINT_WIDTH)
    )


def test_camera_pitch_sign_means_looking_down():
    """Positive pitch must keep meaning 'down' on both sides of the link.

    ``static_transform_publisher`` builds the rotation with REP-103 axes, where
    R_y(+theta) tilts the forward axis toward -z (downward).  The point cloud
    inspector reports ``estimated_pitch_deg_down`` as positive for a downward
    camera.  If someone "fixes" the sign here to match a different mental model,
    the camera ends up tilted 4 deg the wrong way instead of 2 deg the right way.
    """
    args = _launch_camera_args()
    assert args["camera_pitch"] > 0.0, (
        "the camera is mounted looking down; positive pitch encodes that. "
        "A negative value here points it up by the same angle."
    )
    assert args["camera_pitch"] < math.radians(15.0), (
        "camera_pitch=%r rad (%.1f deg) is far more than the measured ~1.7 deg"
        % (args["camera_pitch"], math.degrees(args["camera_pitch"]))
    )


def test_both_costmaps_agree_on_robot_radius():
    """Local and global costmaps each carry their own copy; nothing links them."""
    radii = _nav2_robot_radii()
    assert len(radii) >= 2, (
        "expected a robot_radius in both the local and global costmap, found %d"
        % len(radii)
    )
    assert len(set(radii)) == 1, (
        "costmaps disagree on robot_radius: %r. Nav2 does not check this, so the "
        "robot can be planned through gaps one layer thinks are too narrow." % (radii,)
    )


def test_robot_radius_covers_the_real_footprint():
    """The radius must circumscribe the footprint, not inscribe it.

    Treating the robot as a circle of the *inscribed* radius lets the corners
    (the wheel edges) clip obstacles the planner believes are clear.
    """
    radius = _nav2_robot_radii()[0]
    circumscribed = 0.5 * math.hypot(FOOTPRINT_LENGTH, FOOTPRINT_WIDTH)
    assert radius >= circumscribed - 1e-9, (
        "robot_radius=%.3f m is smaller than the circumscribed radius %.3f m of a "
        "%g x %g m footprint -- wheel corners can clip"
        % (radius, circumscribed, FOOTPRINT_LENGTH, FOOTPRINT_WIDTH)
    )


def test_exploration_robot_radius_does_not_contradict_nav2():
    """The explorer's copy is unused; it must not advertise a different number.

    ``frontier.py`` never reads it and ``exploration_node.py`` only declares it,
    so it has no effect on behaviour -- which is exactly why a stale value there
    is dangerous: it reads as the robot's radius while the costmap uses another.
    """
    nav2_radius = _nav2_robot_radii()[0]
    data = yaml.safe_load(EXPLORATION_YAML.read_text(encoding="utf-8"))
    explorer = data["r680_explorer"]["ros__parameters"]
    assert "robot_radius" in explorer, (
        "if the dead parameter is ever removed, delete this guard too"
    )
    assert float(explorer["robot_radius"]) == pytest.approx(nav2_radius), (
        "r680_exploration.yaml says robot_radius=%r but the costmap uses %r; the "
        "explorer's value is inert, so this only misleads a reader"
        % (explorer["robot_radius"], nav2_radius)
    )
