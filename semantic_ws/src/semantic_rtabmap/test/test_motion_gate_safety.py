#!/usr/bin/env python3
"""The motion gate must be closable, and closing it must be *verifiable*.

The defect
----------
The gate was controlled by "pass `_enable_cmd_vel:=true`, or omit it for off".
Omitting it does not mean off. **ROS 1's master persists a node's private
parameters after the process exits, SIGKILL included.** So:

    relay started with _enable_cmd_vel:=true   -> master stores true
    relay killed
    relay started with NO flag                 -> reads the STALE true
    -> the gate is OPEN, and nothing says so

This was observed while parking the robot to charge. The relay was restarted
without the flag on the assumption that this closes the gate; `/cmd_vel` still
had a publisher. The plan's rollback table asserted "restart without
`_enable_cmd_vel`, the default is off" -- that sentence was false, and it is the
kind of falsehood that ends with a robot driving into something while its
operator believes it cannot move.

The fix has three parts, and all three are guarded here:
  1. `tools/r680_gate.py` deletes the stale params before starting, passes the
     flag explicitly, and then verifies against /cmd_vel -- the publisher list is
     ground truth, not the parameter and not a log line.
  2. Every script that starts the relay passes `_enable_cmd_vel` explicitly,
     either way, so no path depends on a default.
  3. The relay announces the gate state at startup, because the failure mode was
     silence.
"""
import ast
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOLS = REPO_ROOT / "tools"
TX1_RELAY = TOOLS / "ros1_relay.py"
GATE_PY = TOOLS / "r680_gate.py"
GATE_SH = TOOLS / "r680_gate.sh"
PLAN = REPO_ROOT / "R680_FIRST_MOTION_PLAN.md"


def _relay_start_lines():
    """Every line in the repo that launches the TX1 relay, with its filename."""
    hits = []
    for path in list(TOOLS.glob("*.sh")) + list(TOOLS.glob("*.py")):
        if path.name in ("ros1_relay.py", "r680_gate.py"):
            continue  # the relay itself, and the gate tool, are checked separately
        for i, line in enumerate(path.read_text().splitlines(), 1):
            if "ros1_relay.py" in line and "nohup" in line:
                hits.append((path, i, line))
    return hits


# --------------------------------------------------------------------------
# 1. no launch path may rely on the default
# --------------------------------------------------------------------------
def test_every_relay_launch_passes_the_gate_flag_explicitly():
    """Omitting the flag is the bug. Every launch site must state its intent.

    A script that omits the flag does not get "off" -- it gets whatever the last
    process left in the master. So the omission is not a neutral default; it is
    an uncontrolled value.
    """
    hits = _relay_start_lines()
    assert hits, "found no relay launch sites at all; this check has gone stale"
    for path, lineno, line in hits:
        assert "_enable_cmd_vel:=" in line, (
            "%s:%d starts the relay without an explicit _enable_cmd_vel.\n"
            "ROS 1 masters keep private params after a kill, so this inherits a "
            "stale value -- possibly `true`.\n  %s"
            % (path.name, lineno, line.strip())
        )


def test_phase_d_can_never_open_the_gate():
    """Phase D is explicitly 'pick goals, do not move'.

    Its bring-up passes false rather than omitting the flag, so a stale `true`
    cannot leak motion into a phase that is defined by not moving.
    """
    text = (TOOLS / "r680_phase_d_up.sh").read_text()
    assert "_enable_cmd_vel:=false" in text, (
        "the Phase D bring-up must pass _enable_cmd_vel:=false explicitly"
    )
    assert "_enable_cmd_vel:=true" not in text, (
        "the Phase D bring-up must never enable motion"
    )


def test_relay_docstring_does_not_show_the_broken_invocation():
    """The usage example is what people copy."""
    text = TX1_RELAY.read_text()
    assert "_enable_cmd_vel:=false" in text, (
        "the relay's own usage example must include the explicit gate flag"
    )


# --------------------------------------------------------------------------
# 2. the gate tool must verify reality, not intent
# --------------------------------------------------------------------------
def test_gate_tool_verifies_against_the_publisher_list():
    """The parameter can lie (that is the whole bug); /cmd_vel cannot.

    Checked structurally: the verification must actually consult
    `rostopic info /cmd_vel`.
    """
    src = GATE_PY.read_text()
    assert "/cmd_vel" in src, "the gate tool must inspect /cmd_vel"
    assert "rostopic info" in src, (
        "the gate tool must ask the master what publishes /cmd_vel -- trusting "
        "the parameter is exactly what failed"
    )
    tree = ast.parse(src)
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "cmd_vel_publishers" in names, (
        "the gate tool needs a function that reads the real publisher list"
    )


def test_gate_tool_clears_stale_params_before_starting():
    """Deleting the params is what makes the explicit flag authoritative."""
    src = GATE_PY.read_text()
    assert "rosparam delete" in src, (
        "the gate tool must delete the relay's private params before starting, "
        "or a stale value survives and the explicit flag is not authoritative"
    )
    assert "enable_cmd_vel" in src


def test_gate_tool_fails_closed():
    """If verification fails, the gate must end up CLOSED, not ambiguous.

    A half-verified gate is the dangerous state: the operator cannot tell
    whether the robot can move.
    """
    src = GATE_PY.read_text()
    # the failure branch must drive the gate closed rather than returning early
    idx = src.index("Verification failed")
    tail = src[idx:idx + 600]
    assert "start_relay(client, False" in tail or "close" in tail, (
        "on verification failure the gate tool must force the gate closed"
    )


def test_gate_shell_wrapper_exists_and_delegates():
    assert GATE_SH.exists(), "tools/r680_gate.sh must exist as the operator entry point"
    text = GATE_SH.read_text()
    assert "r680_gate.py" in text, "the shell wrapper must call the python tool"
    # A real `set -u` statement, not the word appearing inside a comment that
    # warns against it. Match it at the start of a line, ignoring comments.
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        assert not re.match(r"^set\s+-\w*u", stripped), (
            "do not use set -u; ROS setup.bash references unset variables"
        )


# --------------------------------------------------------------------------
# 3. the state must be announced
# --------------------------------------------------------------------------
def test_relay_announces_the_gate_state_at_startup():
    """The original failure was silent -- motion was enabled and nothing said so."""
    src = TX1_RELAY.read_text()
    assert "MOTION GATE OPEN" in src, (
        "the relay must announce an open gate loudly at startup"
    )
    assert "MOTION GATE CLOSED" in src, (
        "the relay must announce a closed gate too, so the operator can confirm "
        "the state they intended rather than infer it from absence"
    )


# --------------------------------------------------------------------------
# 4. the docs must not repeat the false claim
# --------------------------------------------------------------------------
def test_plan_does_not_claim_omitting_the_flag_closes_the_gate():
    """The rollback table said the default is off. It is not.

    This is a documentation guard on purpose: acting on that sentence is what put
    the robot in an unknown gate state, and a stale doc is indistinguishable from
    a wrong one at the moment someone is trying to stop the robot.
    """
    text = PLAN.read_text()
    if "不带 `_enable_cmd_vel` 重启 relay" in text:
        # if the sentence survives, it must be corrected in place
        idx = text.index("不带 `_enable_cmd_vel` 重启 relay")
        window = text[idx:idx + 400]
        assert ("不成立" in window or "❌" in window or "残留" in window
                or "显式" in window), (
            "the plan still says restarting the relay without _enable_cmd_vel "
            "closes the gate, with no correction next to it. ROS 1 keeps the "
            "private param, so that is false."
        )
