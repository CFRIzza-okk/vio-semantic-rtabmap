"""Tests for the read-only D435 point cloud inspector.

Why synthetic data matters here
-------------------------------
The estimator's whole job is to turn a point cloud into a *physical* answer
(camera mount height and pitch) that will later decide
``min_obstacle_height`` / ``max_obstacle_height`` for the Nav2 costmap.
Getting it wrong is not a cosmetic bug: too low and the floor is marked as a
lethal obstacle (the robot thinks it is boxed in and refuses to move), too
high and real obstacles are filtered away (the robot drives into them).

On the real robot the floor happened to be out of view, so the hardware run
could not validate the maths. These tests therefore feed the estimator
**synthetic clouds with a known ground truth** so the geometry is verified
independently of whatever the camera is pointed at.

The geometry under test (camera optical frame: x right, y **down**, z forward):
a level camera sees the floor as a plane at constant ``y = h`` (the mount
height). Pitched down by ``theta`` the floor line becomes ``y = h - z*theta``,
so the fitted slope *is* the pitch, and the intercept *is* the height.

Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_point_cloud_inspector.py -v
"""

import math

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from semantic_rtabmap.point_cloud_inspector_node import (
    PointCloudInspector,
    pitch_from_quaternion,
)


@pytest.fixture(scope="module")
def inspector():
    rclpy.init()
    node = PointCloudInspector()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def make_cloud(points):
    """Pack an ``(N, 3)`` array into a minimal PointCloud2."""
    arr = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.height = 1
    msg.width = int(arr.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * int(arr.shape[0])
    msg.is_dense = True
    msg.data = arr.tobytes()
    return msg


def floor_points(height, pitch_deg, z_lo=0.5, z_max=3.0, per_bin=400, noise=0.004,
                 seed=7):
    """A floor seen by a camera at ``height`` pitched down by ``pitch_deg``."""
    rng = np.random.default_rng(seed)
    theta = math.radians(pitch_deg)
    zs = np.linspace(z_lo, z_max, int((z_max - z_lo) / 0.3))
    pts = []
    for z in zs:
        y = height - z * math.tan(theta)
        x = rng.uniform(-0.3, 0.3, per_bin)
        zz = z + rng.normal(0.0, noise, per_bin)
        yy = y + rng.normal(0.0, noise, per_bin)
        pts.append(np.column_stack([x, yy, zz]))
    return np.vstack(pts)


def wall_only_points():
    """A far wall straight ahead and nothing else — no floor in view.

    This mirrors what the real D435 actually saw on the robot: a dense
    near-field blob plus a distant wall, with a hard empty gap between them.
    """
    rng = np.random.default_rng(11)
    near = np.column_stack([
        rng.uniform(-0.4, 0.4, 4000),
        rng.normal(0.0, 0.15, 4000),
        rng.uniform(0.30, 0.70, 4000),
    ])
    far = np.column_stack([
        rng.uniform(-1.5, 1.5, 4000),
        rng.normal(0.0, 0.30, 4000),
        rng.uniform(3.8, 5.2, 4000),
    ])
    return np.vstack([near, far])


# ----------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------
def test_floor_fit_recovers_level_mount_height(inspector):
    """A level camera's floor is a constant y => slope ~0, height = intercept."""
    cloud = floor_points(height=0.35, pitch_deg=0.0)
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    assert fit["estimated_mount_height_m"] == pytest.approx(0.35, abs=0.03)
    assert fit["estimated_pitch_deg_down"] == pytest.approx(0.0, abs=1.0)
    # 注意：这时不要去断言 r2。相机水平时地面的 y 是常数，拟合的因变量几乎
    # 没有方差，r2 会退化成噪声（实测约 0.17）——它只在有俯仰、y 确实随 z
    # 变化时才是有效的好坏指标。这里改用斜率来判水平。
    assert abs(fit["slope_dy_dz"]) < 0.02


def test_floor_fit_recovers_pitch_and_height_together(inspector):
    """A pitched camera must yield both the tilt and the height."""
    cloud = floor_points(height=0.42, pitch_deg=12.0)
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    assert fit["estimated_pitch_deg_down"] == pytest.approx(12.0, abs=1.0)
    assert fit["estimated_mount_height_m"] == pytest.approx(0.42, abs=0.04)
    # 有俯仰时 y 随 z 明显变化，r2 才是有效指标。
    assert fit["r2"] > 0.95


def test_floor_fit_reports_reason_when_floor_absent(inspector):
    """With no floor in range the fit must fail loudly, not invent a number."""
    cloud = wall_only_points()
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "not_enough_z_slices"
    assert "floor" in fit["reason"]
    assert "estimated_mount_height_m" not in fit
    # The empty middle is the evidence, so it must be reported.
    assert fit["bin_point_counts"][0] > 0
    assert sum(1 for c in fit["bin_point_counts"] if c >= 40) < 3


# ----------------------------------------------------------------------
# the report as a whole
# ----------------------------------------------------------------------
def test_analyze_withholds_bands_when_floor_is_not_identified(inspector):
    """No floor => no height bands. Otherwise stale percentages look calibrated."""
    report = inspector._analyze(make_cloud(wall_only_points()))

    assert report["floor_fit"]["status"] != "ok"
    assert report["candidate_bands"] == {}
    assert "withheld" in report["candidate_bands_note"]


def test_configured_mount_is_read_from_tf_not_hardcoded(inspector):
    """The mount report must reflect TF, never a baked-in claim.

    It used to be the literal string ``"configured base_link->camera_link = 0
    (default; measurement not applied)"``.  That was true when it was written
    and turned into a lie the moment the measured mount was applied to
    ``r680_navigation.launch.py`` -- while the inspector kept asserting the old
    state with full confidence.  A diagnostic that is confidently wrong is worse
    than one that admits ignorance.

    This node cannot see the launch file, so with no TF published the only
    honest answer is "unavailable".
    """
    report = inspector._analyze(make_cloud(wall_only_points()))

    mount = report["configured_mount"]
    assert isinstance(mount, dict), "the mount report must be structured, not prose"
    assert mount["status"] in ("ok", "unavailable")

    if mount["status"] == "unavailable":
        # It must say why, and what that means for the rest of the report.
        assert "note" in mount
        assert "base_link" in mount["note"]
        assert "camera-relative" in mount["note"]
    else:
        # If TF did resolve, the values must be numbers, not a description.
        assert set(mount["translation_m"]) == {"x", "y", "z"}
        for axis in ("x", "y", "z"):
            assert isinstance(mount["translation_m"][axis], float)
        assert isinstance(mount["pitch_deg"], float)
        assert mount["pitch_sign_note"] == "positive = camera looking down"


def test_pitch_from_quaternion_sign_means_looking_down():
    """Pin the pitch sign convention this whole TF depends on.

    Positive pitch must mean the camera is tilted *down*.  Both conventions
    involved agree on this -- REP-103's R_y(+theta) tilts the forward axis
    toward -z, and the floor fit reports ``estimated_pitch_deg_down`` as positive
    for a downward camera -- so an accidental sign flip anywhere would silently
    tilt the camera 4 degrees the wrong way rather than 2 degrees the right way.
    """
    from types import SimpleNamespace

    def quat_from_pitch(theta):
        """R_y(theta) as a quaternion."""
        return SimpleNamespace(x=0.0, y=math.sin(theta / 2.0),
                               z=0.0, w=math.cos(theta / 2.0))

    assert pitch_from_quaternion(quat_from_pitch(0.0)) == pytest.approx(0.0)

    # The value the launch file actually ships.
    assert pitch_from_quaternion(quat_from_pitch(0.0349)) == pytest.approx(
        0.0349, abs=1e-6
    )

    # A downward-tilted camera must come out positive, not negative.
    down = pitch_from_quaternion(quat_from_pitch(math.radians(2.0)))
    assert down > 0.0
    assert math.degrees(down) == pytest.approx(2.0, abs=1e-6)

    # And the mirror case must be negative, so the sign is not simply dropped.
    assert pitch_from_quaternion(quat_from_pitch(math.radians(-2.0))) < 0.0


def test_analyze_gives_bands_when_floor_is_identified(inspector):
    report = inspector._analyze(make_cloud(floor_points(0.35, 0.0)))

    assert report["floor_fit"]["status"] == "ok"
    bands = report["candidate_bands"]
    assert "min_height_0.00" in bands
    # With a floor at 0.35 m, points above a 0.40 m band must be the minority.
    assert bands["min_height_0.00"] > bands["min_height_0.20"]


def test_analyze_reports_rate_fields_and_z_histogram(inspector):
    report = inspector._analyze(make_cloud(wall_only_points()))

    assert report["frame_id"] == "camera_depth_optical_frame"
    assert report["sampled_points"] > 0
    assert 0.0 <= report["valid_ratio_after_filter"] <= 1.0
    # The distant wall is the dominant far cluster and must show up in the histogram.
    bins = report["z_histogram"]["bins"]
    assert bins["4.0-4.5"] > 0


def test_near_surface_flags_planarity(inspector):
    """A flat surface reads as planar; a cluttered blob does not."""
    flat = floor_points(height=0.30, pitch_deg=0.0, z_lo=0.3, z_max=0.9)
    near_flat = inspector._near_surface(flat[:, 1], flat[:, 2])
    assert near_flat["status"] == "ok"
    assert near_flat["planar"] is True

    clutter = wall_only_points()
    near_clutter = inspector._near_surface(clutter[:, 1], clutter[:, 2])
    assert near_clutter["status"] == "ok"
    assert near_clutter["planar"] is False


# ----------------------------------------------------------------------
# robustness: the three ways the fit was found to lie
# ----------------------------------------------------------------------
def continuous_floor(height, pitch_deg, z_lo=0.4, z_hi=3.0, n=40000,
                     noise=0.004, seed=3):
    """A floor sampled *continuously* in z, as a depth camera actually sees it.

    ``floor_points`` puts 400 points at each of 8 discrete z values.  A real
    floor fills every z inside a bin, and that difference matters: it is what
    exposes the "attribute each slice to the bin centre" bug, because the floor
    points in a bin actually sit at the near edge, not the middle.
    """
    rng = np.random.default_rng(seed)
    theta = math.radians(pitch_deg)
    z = rng.uniform(z_lo, z_hi, n)
    y = height - z * math.tan(theta) + rng.normal(0.0, noise, n)
    x = rng.uniform(-0.6, 0.6, n)
    return np.column_stack([x, y, z])


@pytest.mark.parametrize("height", [0.16, 0.17, 0.18, 0.25])
def test_floor_fit_is_unbiased_on_a_continuous_floor(inspector, height):
    """The reported height must be the true height, not true + an offset.

    The first implementation averaged the top decile of y in each slice, which
    is the mean of a truncated Gaussian: it sits 1.755*sigma_y above the real
    floor.  With sigma_y = 4 mm that is +7.0 mm, measured -- the estimate was
    simply high, always.  Re-selecting the points with a *symmetric* band around
    the line and refitting removes it, because a symmetric selection has no
    preferred side.

    This offset is invisible in the pitch (a constant y shift cannot change a
    slope) and in r2, so nothing else in the report would have caught it.  It
    matters because every height-above-floor threshold comes from this number.
    """
    cloud = continuous_floor(height=height, pitch_deg=2.0)
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    assert fit["estimated_mount_height_m"] == pytest.approx(height, abs=0.003)
    assert fit["estimated_pitch_deg_down"] == pytest.approx(2.0, abs=0.2)


@pytest.mark.parametrize("pitch_deg", [0.0, 2.0, 5.0, 12.0])
def test_floor_fit_survives_a_steep_pitch(inspector, pitch_deg):
    """A steeply pitched floor must still be recovered.

    At 12 deg one 0.3 m bin spans 6 cm of floor height, so attributing a
    slice's floor candidate to the bin *centre* instead of to the points' own z
    scatters the slices well beyond the inlier tolerance.  Measured before the
    fix: only 4 of 8 slices accepted, and 0.27 m reported for a true 0.17 m --
    nearly double, with the pitch at 14.5 deg instead of 12.
    """
    cloud = continuous_floor(height=0.17, pitch_deg=pitch_deg)
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    assert fit["estimated_pitch_deg_down"] == pytest.approx(pitch_deg, abs=0.5)
    assert fit["estimated_mount_height_m"] == pytest.approx(0.17, abs=0.01)


def test_floor_fit_ignores_a_planar_slab_at_the_wrong_height(inspector):
    """Being locally flat is not enough; the slices must agree on one line.

    This is what the real scene did: a slice at z = 2.95 m was planar (y spread
    0.043 m) yet sat 10 cm below where the floor has to be, and dragged the
    fitted pitch to -0.56 deg -- the camera apparently looking *up*.  A slab
    counts as an outlier here for the same reason: it is flat, but not the
    floor.
    """
    rng = np.random.default_rng(5)
    floor = continuous_floor(height=0.17, pitch_deg=2.0)
    n = 8000
    z = rng.uniform(2.6, 3.0, n)
    slab = np.column_stack([
        rng.uniform(-0.6, 0.6, n),
        np.full(n, 0.170),   # constant y => flat, and impossible for a floor
        z,
    ])
    cloud = np.vstack([floor, slab])
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    assert fit["estimated_pitch_deg_down"] == pytest.approx(2.0, abs=0.5)
    assert fit["estimated_mount_height_m"] == pytest.approx(0.17, abs=0.01)


def test_floor_fit_reports_the_points_own_z_not_the_bin_centre(inspector):
    """The reported fit must be anchored where the floor points really are.

    If the abscissa were the bin centre, ``bin_z_eff_m`` would equal
    ``bin_centers_m``.  They are deliberately different, and on a continuous
    floor the real ones must sit nearer the camera (larger y = nearer).
    """
    cloud = continuous_floor(height=0.17, pitch_deg=5.0)
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    assert "bin_z_eff_m" in fit
    for centre, real in zip(fit["bin_centers_m"], fit["bin_z_eff_m"]):
        if real is None:
            continue
        assert real <= centre + 1e-9, "floor points must sit at or before the centre"


def test_floor_fit_survives_clutter_that_outnumbers_the_floor(inspector):
    """The scene that actually broke this on the robot.

    On the real rover the floor was only visible out to about 1.3 m; everything
    beyond was furniture, and *most* z slices were clutter rather than floor.
    That matters because the final symmetric refit trusts the preliminary line
    to choose its band: measured on the robot, the nine-slice preliminary fit
    gave y = 0.122 + 0.0098z while the true floor sits at y = 0.155 at z = 0.55
    -- off by 0.028 m, i.e. outside the 0.02 m band.  Every genuine floor point
    would have been excluded and the refit would have locked onto the clutter.

    So selecting the mutually-consistent slices is not decoration: it is what
    makes the preliminary line land on the floor in the first place, and one
    isolated slab (tested above) is far too easy a case to show that.
    """
    floor = continuous_floor(height=0.17, pitch_deg=2.0, z_hi=1.3, n=20000)

    # Five flat but unrelated surfaces beyond the floor's visible extent,
    # outnumbering the three floor slices.  Their heights are chosen to sit
    # well clear of the extended floor line (y = 0.17 - 0.0349z), because a slab
    # that happens to land on that line is genuinely indistinguishable from
    # floor and would legitimately be kept.
    slabs = []
    rng = np.random.default_rng(17)
    for lo, y_const in ((1.45, 0.020), (1.75, -0.080), (2.05, -0.030),
                        (2.35, -0.150), (2.65, -0.020)):
        n = 3000
        slabs.append(np.column_stack([
            rng.uniform(-0.6, 0.6, n),
            np.full(n, y_const) + rng.normal(0.0, 0.002, n),
            rng.uniform(lo, lo + 0.25, n),
        ]))
    cloud = np.vstack([floor] + slabs)
    fit = inspector._floor_fit(cloud[:, 0], cloud[:, 1], cloud[:, 2])

    assert fit["status"] == "ok"
    # The floor must win despite being the minority of slices.
    assert fit["slices"] == 3
    assert fit["slices_rejected_inconsistent"] >= 4
    assert fit["estimated_pitch_deg_down"] == pytest.approx(2.0, abs=0.5)
    assert fit["estimated_mount_height_m"] == pytest.approx(0.17, abs=0.01)
