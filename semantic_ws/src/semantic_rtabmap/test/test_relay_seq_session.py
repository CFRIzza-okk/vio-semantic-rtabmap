#!/usr/bin/env python3
"""A restarted sender must not be locked out by the previous sender's counter.

The bug this guards
-------------------
TX1's relay accepted a cmd_vel packet only when ``seq > last_seq``, and it
re-baselined ``last_seq`` only when the TX1 relay itself restarted. The Orin's
relay node, meanwhile, starts its counter at 0 on every process start. Restart
them in that order -- which the startup scripts did, because the gate is opened
(restarting the TX1 relay) while the previous Orin stack is still sending -- and
the new sender's counter sits BELOW the stale water mark:

    old sender reached seq 13800  ->  TX1 last_seq = 13800
    new sender starts at seq 0    ->  seq <= last_seq for the next 13800 packets
    at 20 Hz that is ~11.5 minutes

Every velocity command in that window is dropped, and the drop is invisible: the
``continue`` logs nothing, the watchdog keeps publishing zeros, the uplink
(odom/tf/battery) is untouched, so ``localization_valid`` stays true and the
safety chain reports healthy while the robot refuses to move.

Measured on the real robot 2026-10-17: a 90 s autonomous run drove 0.00 m with
``goals_completed: 0``, the log showing only "Failed to make progress" from
Nav2; and the same stack, driven by two recorders, showed 595 non-zero
``/cmd_vel_safe`` samples on the Orin against 0 non-zero ``/cmd_vel`` samples on
TX1. Ten minutes later, with the sender's counter finally past the old water
mark, the identical test forwarded 226 of 240 packets.

What must hold
--------------
1. A packet from a DIFFERENT sender instance re-baselines: it is accepted at
   once, however far below the previous counter it starts.
2. Within one sender instance the guard keeps its real job -- duplicates and
   out-of-order stragglers are still dropped.
3. The two sides agree on the field name that carries the session.
"""
import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
TX1_RELAY = REPO_ROOT / "tools" / "ros1_relay.py"
ORIN_RELAY = (REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap"
              / "semantic_rtabmap" / "ros2_relay_node.py")


def _function(path, name):
    """Return the FUNCTION object itself, so tests run the shipped code."""
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            namespace = {}
            module = ast.Module(body=[node], type_ignores=[])
            exec(compile(module, str(path), "exec"), namespace)  # noqa: S102
            return namespace[name]
    raise AssertionError("%s has no top-level %s()" % (path, name))


def _old_rule(last_seq, seq):
    """The pre-fix guard, verbatim in behaviour, to show what it did."""
    return seq > last_seq


# --------------------------------------------------------------------------
# 1. the guard itself, executed
# --------------------------------------------------------------------------
def test_increasing_sequence_within_a_session_is_accepted():
    accept = _function(TX1_RELAY, "_accept_cmd_vel")
    last_seq, session = -1, None
    for seq in range(1, 6):
        ok, last_seq, session = accept(last_seq, session, "aaaa", seq)
        assert ok, "seq %d of a live session was rejected" % seq
    assert (last_seq, session) == (5, "aaaa")


def test_duplicates_and_stragglers_are_still_dropped():
    """The guard's real job must survive the fix."""
    accept = _function(TX1_RELAY, "_accept_cmd_vel")
    last_seq, session = -1, None
    for seq in (10, 11, 12):
        ok, last_seq, session = accept(last_seq, session, "aaaa", seq)
        assert ok
    for stale in (12, 11, 5):
        ok, last_seq, session = accept(last_seq, session, "aaaa", stale)
        assert not ok, "seq %d replayed inside one session was accepted" % stale
    assert last_seq == 12, "a rejected packet must not move the water mark"


def test_a_restarted_sender_is_accepted_immediately():
    """The whole point: a new sender instance re-baselines instead of waiting."""
    accept = _function(TX1_RELAY, "_accept_cmd_vel")

    # Old instance ran up to a large counter...
    last_seq, session = -1, None
    for seq in range(1, 1201):
        ok, last_seq, session = accept(last_seq, session, "old-session", seq)
        assert ok
    assert last_seq == 1200

    # ...and the sender restarted. Its first packet is seq=1, far below 1200.
    ok, last_seq, session = accept(last_seq, session, "new-session", 1)
    assert ok, (
        "a restarted sender was locked out by the previous instance's counter; "
        "on the robot this is a silent multi-minute downlink blackout"
    )
    assert (last_seq, session) == (1, "new-session")


def test_the_old_rule_would_have_dropped_the_restarted_sender():
    """Proves the hazard is real, and that (1) is testing the fix, not a no-op.

    If someone reverts to a bare `seq > last_seq`, the count below is how many
    packets -- and how many seconds at the measured 20 Hz -- the robot spends
    unable to move.
    """
    stale_water_mark = 1200
    dropped = [seq for seq in range(1, 1201) if not _old_rule(stale_water_mark, seq)]
    assert len(dropped) == 1200, "the old rule no longer reproduces the blackout"
    assert len(dropped) / 20.0 == 60.0, "expected 60 s of blackout at 20 Hz"


def test_an_absent_session_field_still_re_baselines_once():
    """An older sender (no session key) must not flip the counter every packet."""
    accept = _function(TX1_RELAY, "_accept_cmd_vel")
    last_seq, session = -1, None
    ok, last_seq, session = accept(last_seq, session, "", 7)
    assert ok and (last_seq, session) == (7, "")
    # Same (absent) session from here on: normal ascending rule.
    ok, last_seq, session = accept(last_seq, session, "", 6)
    assert not ok, "an absent session must not disable the guard"


# --------------------------------------------------------------------------
# 2. the receive loop actually uses it
# --------------------------------------------------------------------------
def test_receive_loop_delegates_to_the_guard():
    """A guard nobody calls protects nothing."""
    tree = ast.parse(TX1_RELAY.read_text())
    body = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "receive":
            body = node
    assert body is not None, "tools/ros1_relay.py lost its receive()"

    calls = {n.func.id for n in ast.walk(body)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert "_accept_cmd_vel" in calls, (
        "receive() no longer calls _accept_cmd_vel(); the shipped guard is "
        "unused and the lock-out is back"
    )


def test_no_bare_sequence_comparison_remains_in_the_receive_path():
    """The pre-fix `if seq <= self.last_seq: continue` must not come back."""
    source = TX1_RELAY.read_text()
    assert "if seq <= self.last_seq: continue" not in source, (
        "the unguarded sequence comparison is back in the downlink path"
    )


# --------------------------------------------------------------------------
# 3. both sides agree on the field that carries the session
# --------------------------------------------------------------------------
def _cmd_vel_body_keys(path):
    """Keys of the dict literal that becomes the signed cmd_vel body."""
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == "body" for t in node.targets):
            continue
        if isinstance(node.value, ast.Dict):
            keys = [k.value for k in node.value.keys
                    if isinstance(k, ast.Constant) and isinstance(k.value, str)]
            if "kind" in keys:
                return set(keys)
    raise AssertionError("%s: no cmd_vel body dict literal found" % path)


def test_sender_stamps_a_session_the_receiver_reads():
    sent = _cmd_vel_body_keys(ORIN_RELAY)
    assert "session" in sent, (
        "the Orin no longer stamps a session on cmd_vel; TX1 cannot tell a "
        "restarted sender from a replay and the blackout returns"
    )
    read = TX1_RELAY.read_text()
    assert 'body.get("session"' in read, (
        "TX1's receive() no longer reads the session field the Orin sends"
    )


def test_session_is_per_process_not_a_constant():
    """A constant id would make every restart look like the same sender."""
    source = ORIN_RELAY.read_text()
    assert "getrandbits" in source, (
        "the session id is no longer random per process; a fixed value would "
        "never trigger the re-baseline"
    )
