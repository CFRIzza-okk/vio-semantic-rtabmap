#!/usr/bin/env python3
"""Guard the battery-voltage path from TX1 to the explorer's low-voltage guard.

Why this exists
---------------
The exploration node's battery guard was inert: `battery_topic` was empty, so
`min_battery` (10.5 V) could never trip and a long autonomous run would drive
until the pack died. That is not hypothetical on this robot -- TX1 already lost
power mid-session once.

Wiring it up exposes a second, subtler problem. The guard read:

    if self._battery is not None:
        if self._battery < minimum: ...

`_battery_stamp` was recorded on every sample but never read by anything. So a
reading that stopped arriving was still treated as current: kill the relay
mid-run and the guard keeps comparing against the last voltage it ever saw and
reports "fine". That is the same silent-failure shape as the rest of this
project's defects -- the sensor is dead and the system looks healthy.

These tests pin down both halves:
  A. the plumbing (TX1 publishes, relay forwards, node subscribes)
  B. staleness is treated as a fault, not as "no news is good news"
"""
import ast
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
PKG = REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap"
EXPLORER = PKG / "semantic_rtabmap" / "exploration_node.py"
RELAY_NODE = PKG / "semantic_rtabmap" / "ros2_relay_node.py"
TX1_RELAY = REPO_ROOT / "tools" / "ros1_relay.py"
RELAY_YAML = PKG / "config" / "r680_relay.yaml"
EXPLORATION_YAML = PKG / "config" / "r680_exploration.yaml"

TX1_TOPIC = "/PowerVoltage"          # what TX1's driver actually publishes
BRIDGE_TOPIC = "/battery_voltage"    # what the relay exposes on ROS 2


def _params(path):
    """Read a ROS 2 params file.

    The two files are NOT shaped the same, which cost me a round of KeyError:
    r680_relay.yaml uses the wildcard node name `/**`, while
    r680_exploration.yaml nests its parameters under the concrete node name
    `r680_explorer`. Assuming `/**` everywhere is wrong.
    """
    data = yaml.safe_load(path.read_text())
    if "/**" in data:
        return data["/**"]["ros__parameters"]
    # concrete node names: take the first block that has ros__parameters
    for key, value in data.items():
        if isinstance(value, dict) and "ros__parameters" in value:
            return value["ros__parameters"]
    raise AssertionError("no ros__parameters block found in %s" % path)


# --------------------------------------------------------------------------
# A. plumbing
# --------------------------------------------------------------------------
def test_tx1_relay_subscribes_to_power_voltage():
    """TX1's driver publishes /PowerVoltage (std_msgs/Float32, measured 11.90 V).

    If the relay does not subscribe to it, nothing downstream can ever work --
    and the failure is invisible, because the ROS 2 topic simply stays silent.

    This must be a structural check. A substring search for "/PowerVoltage"
    passes even after the subscription line is deleted, because the explanatory
    comment above it still contains the topic name -- which is exactly how the
    first version of this test missed that mutation.
    """
    tree = ast.parse(TX1_RELAY.read_text())

    def calls_subscriber(node):
        """True if this subtree calls rospy.Subscriber(...) on the battery topic."""
        if not isinstance(node, ast.Call):
            return False
        f = node.func
        if not (isinstance(f, ast.Attribute) and f.attr == "Subscriber"):
            return False
        if not node.args:
            return False
        first = node.args[0]
        return isinstance(first, ast.Constant) and first.value == TX1_TOPIC

    assert any(calls_subscriber(n) for n in ast.walk(tree)), (
        "tools/ros1_relay.py must actually CALL rospy.Subscriber(%r, ...); "
        "mentioning the topic in a comment is not a subscription" % TX1_TOPIC
    )
    assert "Float32" in TX1_RELAY.read_text(), (
        "std_msgs/Float32 must be imported on the TX1 side"
    )


def test_relay_node_declares_and_publishes_battery():
    src = RELAY_NODE.read_text()
    assert 'declare_parameter("output_battery"' in src, (
        "the ROS 2 relay must declare output_battery"
    )
    assert "publish_battery" in src, "the ROS 2 relay must implement publish_battery"
    assert 'kind == "battery"' in src, (
        "the receive() dispatch must route the 'battery' packet kind"
    )


def test_relay_node_imports_float32():
    """A name used but never imported passes py_compile and fails at runtime.

    This actually happened while wiring this feature: Float32 was used in
    publish_battery and at the publisher, py_compile was clean, and the node
    would have raised NameError on the first battery packet.
    """
    src = RELAY_NODE.read_text()
    assert "from std_msgs.msg import Float32" in src, (
        "Float32 is referenced in ros2_relay_node.py but not imported"
    )


def test_relay_yaml_publishes_on_the_topic_the_explorer_reads():
    """The two config files must agree, or the guard silently never fires."""
    relay = _params(RELAY_YAML)
    explorer = _params(EXPLORATION_YAML)

    published = relay.get("output_battery")
    assert published, "config/r680_relay.yaml must set output_battery"
    assert explorer.get("battery_topic") == published, (
        "explorer battery_topic is %r but relay publishes %r -- the guard would "
        "subscribe to a topic nothing publishes and stay permanently inert"
        % (explorer.get("battery_topic"), published)
    )
    assert explorer.get("battery_topic"), (
        "battery_topic must not be empty; an empty battery_topic means the "
        "low-voltage abort can never trigger"
    )


def test_min_battery_is_a_sane_3s_threshold():
    explorer = _params(EXPLORATION_YAML)
    threshold = float(explorer["min_battery"])
    # A 3S pack is 12.6 V full and wants to stop well before damage.
    assert 9.0 <= threshold <= 11.5, (
        "min_battery %r is outside a plausible 3S cutoff; the measured pack "
        "voltage on this robot is ~11.9 V" % threshold
    )


# --------------------------------------------------------------------------
# B. staleness
# --------------------------------------------------------------------------
def _find_method(name):
    tree = ast.parse(EXPLORER.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


def test_battery_timeout_parameter_exists():
    src = EXPLORER.read_text()
    assert 'declare_parameter("battery_timeout"' in src, (
        "a battery sample needs an expiry; without one a dead relay leaves the "
        "guard comparing against a stale voltage"
    )
    explorer = _params(EXPLORATION_YAML)
    assert "battery_timeout" in explorer, (
        "config/r680_exploration.yaml must set battery_timeout explicitly"
    )


def test_battery_stamp_is_actually_read():
    """`_battery_stamp` was write-only, which is how staleness went unnoticed.

    Asserting the attribute merely *exists* is not enough -- it always existed.
    The requirement is that a recorded sample can EXPIRE, i.e. the stamp feeds a
    decision.

    Note that the comparison is indirect in the real code::

        age = now - self._battery_stamp     # a BinOp, not a Compare
        if age > battery_timeout: ...

    so searching only for `_battery_stamp` inside an ast.Compare would wrongly
    fail correct code. What matters is that the stamp is stored AND that the age
    derived from it is measured against a timeout.
    """
    tree = ast.parse(EXPLORER.read_text())

    stored = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr == "_battery_stamp" \
                and isinstance(getattr(node, "ctx", None), ast.Store):
            stored = True
    assert stored, "_battery_stamp is never assigned"

    assert "battery_timeout" in EXPLORER.read_text(), "no battery_timeout parameter"

    compared = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Compare):
            names = {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
            if "age" in names:
                compared = True
    assert compared, (
        "the age of the battery sample is never compared against a timeout, so a "
        "stale reading cannot be told apart from a fresh one"
    )


def test_stale_battery_aborts_rather_than_passing():
    """Stale must abort AND cancel the goal -- not merely log a new state.

    Setting ABORTED without cancelling leaves an in-flight NavigateToPose running,
    so the robot keeps driving to a goal after the exploration node has declared
    itself finished.

    Checked structurally. Grepping a character window around the message text is
    too loose: the surrounding comments mention cancellation in prose, so the
    window still matches after the actual call is replaced with `pass`.
    """
    tree = ast.parse(EXPLORER.read_text())

    # Find the SMALLEST `if` branch that reports staleness. Picking the first
    # match via ast.walk() selects an OUTER if (the enclosing
    # `if self._battery is not None:` also contains the string in its dump), and
    # the outer branch also holds the sibling low-voltage check with its own
    # _cancel_goal -- which made this test pass even when the stale branch had
    # its cancellation deleted.
    candidates = []
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and "battery reading stale" in ast.dump(node):
            candidates.append(node)
    assert candidates, (
        "no `if` branch reports a stale battery reading; the explorer must "
        "distinguish stale from low so the cause is diagnosable"
    )
    target = min(candidates, key=lambda n: len(ast.dump(n)))

    # within that branch: an abort transition and a goal cancellation
    calls = [n for n in ast.walk(target) if isinstance(n, ast.Call)]
    names = set()
    for c in calls:
        f = c.func
        if isinstance(f, ast.Attribute):
            names.add(f.attr)
        elif isinstance(f, ast.Name):
            names.add(f.id)
    assert "_set_state" in names, "the stale branch must transition state"
    assert "_cancel_goal" in names, (
        "the stale branch must cancel the in-flight goal -- otherwise the robot "
        "keeps driving after exploration has given up"
    )

    # and the state it sets must be the abort state
    state_ok = False
    for c in calls:
        if isinstance(c.func, ast.Attribute) and c.func.attr == "_set_state":
            for sub in ast.walk(c):
                if isinstance(sub, ast.Name) and sub.id == "STATE_ABORTED":
                    state_ok = True
    assert state_ok, "a stale battery reading must set STATE_ABORTED"


def test_guard_only_runs_when_a_reading_exists():
    """The guard must stay conditional on a reading having ever arrived, so a
    relay with no battery source is still usable.

    Checked structurally: ast.dump() renders `self._battery is not None` as
    Compare(left=Attribute(attr='_battery'), ops=[IsNot()]), so a substring
    search for the source text can never match.
    """
    method = _find_method("_tick")
    assert method is not None, "_tick not found; the guard may have moved"
    for node in ast.walk(method):
        if not isinstance(node, ast.Compare):
            continue
        left = node.left
        if isinstance(left, ast.Attribute) and left.attr == "_battery" \
                and any(isinstance(op, ast.IsNot) for op in node.ops):
            return
    raise AssertionError(
        "the battery guard must remain conditional on a reading having ever "
        "arrived (`if self._battery is not None:`)"
    )
