#!/usr/bin/env python3
"""Guard the optional planar-covariance adaptation of relayed /odom.

Background
----------
TX1's driver publishes /odom with the variance of z, roll and pitch set to 1e6 --
the ROS convention for "this degree of freedom is not estimated". For a planar
wheeled base that is *correct*: wheel encoders cannot observe those three DOF.
Measured on TX1, the 6x6 diagonal is

    x=1e-09   y=0.001   z=1e6
    roll=1e6  pitch=1e6  yaw=1e-09

i.e. 1e6 lands exactly on the three unobservable DOF while yaw is already tiny.

RTAB-Map reads the angular covariance when weighting its pose graph and responds:

    Memory.cpp:1168::addSignatureToStm() Very large angular variance
    (1000000.000000) detected! Please fix odometry covariance, otherwise poor
    graph optimizations are expected and wrong loop closure detections creating
    a lot of errors in the map could be accepted.

This robot only drives on flat ground (operator, 2026-10-14), so "the base stays
level" is true and can be asserted. That is what the switch does.

What these tests protect
------------------------
1. The switch is OFF by default -- the relay carries the safety path and must not
   silently rewrite TX1's data unless a config explicitly asks for it.
2. config/r680_relay.yaml turns it ON, so the mapping path actually gets it.
3. Only the 1e6 sentinel is replaced. If TX1 ever reports real values for those
   DOF the adaptation must become a no-op rather than overwrite them.
4. x, y and yaw must never be touched -- they carry the driver's real estimates,
   and yaw in particular is the one angular value that IS observable.
5. The 36-element length invariant holds even for a short input.
"""
import ast
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
NODE = REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap" / \
    "semantic_rtabmap" / "ros2_relay_node.py"
CONFIG = REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap" / \
    "config" / "r680_relay.yaml"

# The real TX1 /odom diagonal, measured 2026-10-14 (row-major 6x6).
TX1_DIAGONAL = [1e-09, 0.001, 1e6, 1e6, 1e6, 1e-09]
DIAG_INDEX = (0, 7, 14, 21, 28, 35)          # x, y, z, roll, pitch, yaw
PLANAR_INDICES = (14, 21, 28)                 # z, roll, pitch
OBSERVABLE_INDICES = (0, 7, 35)               # x, y, yaw


def _tx1_covariance():
    """Build a 36-element covariance with the measured TX1 diagonal."""
    cov = [0.0] * 36
    for idx, value in zip(DIAG_INDEX, TX1_DIAGONAL):
        cov[idx] = value
    return cov


@pytest.fixture(scope="module")
def module():
    """Import the relay node module without needing a live rclpy context."""
    sys.path.insert(0, str(NODE.parent))
    try:
        import ros2_relay_node
    except ImportError as exc:  # pragma: no cover - environment problem
        pytest.skip("cannot import relay node module: %s" % exc)
    return ros2_relay_node


def test_sentinel_is_replaced_on_exactly_the_unobservable_dof(module):
    cov = _tx1_covariance()
    out = module.apply_planar_covariance(cov)

    for idx in PLANAR_INDICES:
        assert out[idx] == pytest.approx(module.PLANAR_COVARIANCE_VALUE), (
            "index %d (z/roll/pitch) should have been given a finite value; "
            "leaving 1e6 is what makes RTAB-Map warn" % idx
        )
        assert out[idx] < module.UNKNOWN_COVARIANCE_MIN


def test_observable_dof_are_never_touched(module):
    """x, y and yaw carry real estimates; rewriting them would corrupt the map.

    The sentinel values here are deliberately set ABOVE the replacement
    threshold. An earlier version of this test kept TX1's real values (yaw was
    1e-09, far below UNKNOWN_COVARIANCE_MIN) and therefore passed even when yaw
    was wrongly added to PLANAR_COVARIANCE_INDICES -- it was relying on the
    number being small rather than on yaw being excluded. Driving every
    "must not change" index with a value that WOULD be replaced removes that
    coincidence.
    """
    cov = _tx1_covariance()
    for idx in OBSERVABLE_INDICES:
        cov[idx] = 1e6          # would be replaced if it were in the list

    out = module.apply_planar_covariance(cov)

    for idx in OBSERVABLE_INDICES:
        assert out[idx] == 1e6, (
            "index %d (x/y/yaw) must be forwarded verbatim even when it holds a "
            "sentinel-sized value, got %r" % (idx, out[idx])
        )


def test_index_list_names_exactly_z_roll_pitch(module):
    """Structural guard: the exclusion of x/y/yaw must not rest on magnitudes."""
    assert tuple(module.PLANAR_COVARIANCE_INDICES) == (14, 21, 28), (
        "PLANAR_COVARIANCE_INDICES must be exactly z/roll/pitch (14/21/28); "
        "index 0/7/35 are x/y/yaw and index 35 in particular is the one angular "
        "DOF the driver genuinely estimates"
    )
    assert 35 not in module.PLANAR_COVARIANCE_INDICES, "yaw must never be touched"


def test_real_values_are_not_overwritten(module):
    """If TX1 ever reports a real z/roll/pitch variance, leave it alone.

    This is why the switch is safe to leave enabled: it degrades to a no-op once
    the driver stops using the sentinel.
    """
    cov = _tx1_covariance()
    real = 0.0042
    for idx in PLANAR_INDICES:
        cov[idx] = real

    out = module.apply_planar_covariance(cov)

    for idx in PLANAR_INDICES:
        assert out[idx] == pytest.approx(real), (
            "index %d held a real value (%r) and must not be replaced by the "
            "planar default" % (idx, real)
        )


def test_result_is_always_thirty_six_elements(module):
    """nav_msgs/Odometry.covariance is a fixed float64[36]; a short list would
    make the assignment raise at runtime."""
    assert len(module.apply_planar_covariance([])) == 36
    assert len(module.apply_planar_covariance([0.0] * 3)) == 36
    assert len(module.apply_planar_covariance(_tx1_covariance())) == 36


def test_off_diagonal_terms_are_preserved(module):
    """Only the diagonal DOF we mean to change may move."""
    cov = _tx1_covariance()
    cov[1] = 0.25          # a deliberate off-diagonal term
    out = module.apply_planar_covariance(cov)
    assert out[1] == pytest.approx(0.25)


def test_node_default_is_off(module):
    """The node must default to faithful forwarding.

    It carries the safety path (cmd_vel down to TX1), so an implicit rewrite of
    TX1's data would be wrong. Only an explicit config may turn it on -- same
    convention as peer_host, whose node default also differs from the yaml.
    """
    source = NODE.read_text()
    assert 'declare_parameter("planar_odom_covariance", False)' in source, (
        "planar_odom_covariance must be declared with a default of False"
    )


def test_shipped_config_turns_it_on():
    """Mapping runs from r680_relay.yaml, so that is where it must be enabled."""
    cfg = yaml.safe_load(CONFIG.read_text())
    params = cfg["/**"]["ros__parameters"]
    assert params.get("planar_odom_covariance") is True, (
        "config/r680_relay.yaml must set planar_odom_covariance: true, otherwise "
        "the adaptation never runs in the mapping path"
    )


def test_publish_odom_actually_uses_the_switch():
    """Guard the wiring, not just the helper.

    A correct helper that publish_odom never actually invokes passes every other
    test in this file while changing nothing -- the same "silently ineffective"
    failure mode this project already hit with the depthimage_to_laserscan remap,
    where a wrong source name was accepted without any error.

    This must be a *structural* check. Searching the function's text for the name
    "apply_planar_covariance" is not enough: mutating the guard to ``if False:``
    leaves the call sitting inside a dead branch and the text search still hits.
    So walk the AST and require the call to appear inside the body of an
    ``if`` whose test is the planar_covariance switch.
    """
    tree = ast.parse(NODE.read_text())
    publish = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "publish_odom":
            publish = node
    assert publish is not None, "publish_odom not found in the relay node"

    def calls_helper(stmts):
        """True if any statement in this subtree calls apply_planar_covariance."""
        for stmt in stmts:
            for inner in ast.walk(stmt):
                if isinstance(inner, ast.Call):
                    f = inner.func
                    if isinstance(f, ast.Name) and f.id == "apply_planar_covariance":
                        return True
        return False

    guarded = False
    for stmt in ast.walk(publish):
        if not isinstance(stmt, ast.If):
            continue
        # the condition must reference self.planar_covariance, not a constant
        test = ast.dump(stmt.test)
        if "planar_covariance" not in test:
            continue
        if isinstance(stmt.test, ast.Constant):
            continue
        if calls_helper(stmt.body):
            guarded = True

    assert guarded, (
        "publish_odom must call apply_planar_covariance inside a branch guarded "
        "by self.planar_covariance; otherwise the switch does nothing"
    )

    # And it must not be reachable from any unguarded top-level statement, or the
    # switch would be bypassed entirely.
    for stmt in publish.body:
        if isinstance(stmt, ast.If):
            continue
        assert not calls_helper([stmt]), (
            "apply_planar_covariance is called outside the planar_covariance "
            "guard, so the switch would not actually control it"
        )
