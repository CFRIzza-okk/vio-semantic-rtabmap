#!/usr/bin/env python3
"""The relay's HMAC must survive being signed on one Python and verified on another.

The bug this guards
-------------------
The relay spans two interpreters: TX1 runs ROS Melodic on Python 2.7, the Orin
runs ROS 2 Humble on Python 3.10. Both sides signed with

    json.dumps(body, separators=(",", ":"))

and then the receiver re-serialised the parsed body to check the signature.

Python 3 dicts preserve insertion order; Python 2.7 dicts are hash-ordered. So
the same body produced DIFFERENT bytes on each side, and the HMAC computed at
the far end never matched.

Why it stayed hidden
--------------------
The uplink was accidentally immune. TX1 serialised, TX1 signed, and the Orin --
on receipt -- re-serialised a dict whose key order came from the JSON text, so
it reproduced TX1's bytes exactly. odom, imu, tf and battery all worked.

Only the downlink broke: cmd_vel was built fresh as a Python 3 dict, signed, and
then verified by a Python 2 process that re-ordered the keys. Every velocity
packet was rejected. TX1 logged it as `invalid relay packet`, rate-limited to one
line per five seconds -- noise that looked like unrelated junk traffic.

The symptom was therefore maximally misleading: mapping and odometry were
flawless, `localization_valid` was true, `obstacle_stop` was false, the safety
chain reported healthy, and the robot never moved. Nothing said "command
rejected"; it said "invalid relay packet", five seconds apart, forever.

What must hold
--------------
1. Both sides use the same canonical form (sort_keys=True).
2. The canonical form is a pure function of the data: its bytes do not depend on
   dict insertion order.
3. The two implementations are byte-identical -- they are separate copies in
   different files and will drift unless something checks.
"""
import ast
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TX1_RELAY = REPO_ROOT / "tools" / "ros1_relay.py"
ORIN_RELAY = (REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap"
              / "semantic_rtabmap" / "ros2_relay_node.py")

# A body with the exact shape the Orin's send_cmd() builds. Key order is
# deliberately NOT alphabetical, so any implementation that forgets sort_keys
# will emit these in this order rather than sorted order.
SAMPLE = {
    "kind": "cmd_vel",
    "seq": 1,
    "payload": {
        "linear": {"x": 0.06, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    },
    "sent_at": 123456789,
}


# --------------------------------------------------------------------------
# 1. the canonical form itself
# --------------------------------------------------------------------------
def test_canonical_form_is_order_independent():
    """The bytes must be a function of the data, not of how the dict was built.

    Building the same logical body in two different insertion orders must give
    identical bytes. This is precisely what a plain json.dumps() fails.
    """
    reordered = {}
    for key in ("sent_at", "payload", "seq", "kind"):
        reordered[key] = SAMPLE[key]

    a = json.dumps(SAMPLE, separators=(",", ":"), sort_keys=True).encode()
    b = json.dumps(reordered, separators=(",", ":"), sort_keys=True).encode()
    assert a == b, "canonical form still depends on insertion order"

    # and the naive form genuinely differs, or this test proves nothing
    naive_a = json.dumps(SAMPLE, separators=(",", ":")).encode()
    naive_b = json.dumps(reordered, separators=(",", ":")).encode()
    assert naive_a != naive_b, (
        "the naive serialisation no longer differs by key order; this test is "
        "no longer demonstrating the hazard it exists for"
    )


# --------------------------------------------------------------------------
# 2. both implementations agree, literally
# --------------------------------------------------------------------------
def _canonical_source(path, func_name):
    """Extract the function body so the two copies can be compared."""
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == func_name:
            return node
    return None


def test_both_sides_use_sort_keys():
    tx1 = TX1_RELAY.read_text()
    orin = ORIN_RELAY.read_text()
    assert "sort_keys=True" in tx1, "TX1 relay must canonicalise with sort_keys"
    assert "sort_keys=True" in orin, "Orin relay must canonicalise with sort_keys"


def test_no_signing_path_still_uses_the_naive_dumps():
    """A single leftover naive json.dumps in a sign/verify path re-breaks it.

    This is the test that has to catch the ORIGINAL bug, so its direction
    matters. The code shape is::

        raw = json.dumps(body, separators=(",", ":")).encode()

    The `.encode()` is the OUTER call and `json.dumps(...)` is its `func.value`.
    Walking down from the `json.dumps` node therefore never reaches the
    `.encode()` -- an earlier version of this test did exactly that and missed
    both real regression sites. The walk must instead start at every `.encode()`
    call and look INSIDE it for a bare `json.dumps`.
    """
    for path, label in ((TX1_RELAY, "TX1"), (ORIN_RELAY, "Orin")):
        tree = ast.parse(path.read_text())
        checked = 0
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            if not (isinstance(f, ast.Attribute) and f.attr == "encode"):
                continue
            # find json.dumps(...) calls inside the thing being encoded
            for sub in ast.walk(node):
                if not isinstance(sub, ast.Call):
                    continue
                sf = sub.func
                if not (isinstance(sf, ast.Attribute) and sf.attr == "dumps"
                        and isinstance(sf.value, ast.Name) and sf.value.id == "json"):
                    continue
                checked += 1
                has_sort = any(kw.arg == "sort_keys" for kw in sub.keywords)
                assert has_sort, (
                    "%s relay line %d: json.dumps(...).encode() without "
                    "sort_keys -- if this feeds a signature, the far end "
                    "rejects the packet" % (label, sub.lineno)
                )
        assert checked > 0, (
            "%s relay: no json.dumps(...).encode() found at all -- either the "
            "signing path was removed or this check has stopped looking in the "
            "right place" % label
        )


def test_the_two_implementations_are_byte_identical():
    """They are two copies in two files; drift silently breaks one direction."""
    tx1_fn = _canonical_source(TX1_RELAY, "_canonical")
    orin_fn = _canonical_source(ORIN_RELAY, "canonical")
    assert tx1_fn is not None, "tools/ros1_relay.py lost its _canonical()"
    assert orin_fn is not None, "ros2_relay_node.py lost its canonical()"

    def only_the_expression(fn):
        for node in ast.walk(fn):
            if isinstance(node, ast.Return) and node.value is not None:
                return ast.dump(node.value)
        return None

    a = only_the_expression(tx1_fn)
    b = only_the_expression(orin_fn)
    assert a == b, (
        "the two canonical() implementations have diverged:\n  TX1 : %s\n  Orin: %s"
        % (a, b)
    )


# --------------------------------------------------------------------------
# 3. the real cross-version check
# --------------------------------------------------------------------------
def _python2():
    for exe in ("python2", "python2.7"):
        try:
            subprocess.run([exe, "-c", "print(1)"], capture_output=True, timeout=10)
            return exe
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
    return None


def test_python2_and_python3_produce_the_same_signed_bytes():
    """The decisive test: sign here (Python 3), verify with Python 2.

    A test that only exercises Python 3 cannot catch this class of bug at all --
    both sides must actually be run. Python 2 is skipped with a clear reason if
    absent, so the suite stays runnable on a machine without it, but the skip is
    explicit rather than silent.
    """
    py2 = _python2()
    if py2 is None:
        pytest.skip("python2 not available; cannot exercise the cross-version path")

    payload = json.dumps(SAMPLE, separators=(",", ":"), sort_keys=True)
    script = (
        "import json,hmac,hashlib,sys\n"
        "body=json.loads(sys.argv[1])\n"
        "raw=json.dumps(body,separators=(',',':'),sort_keys=True).encode()\n"
        "print(raw.decode())\n"
        "print(hmac.new(b'change-me-r680',raw,hashlib.sha256).hexdigest())\n"
    )
    proc = subprocess.run([py2, "-c", script, payload],
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, "python2 side failed: %s" % proc.stderr

    py2_raw, py2_sig = proc.stdout.strip().split("\n")

    import hashlib
    import hmac as _hmac
    py3_raw = json.dumps(SAMPLE, separators=(",", ":"), sort_keys=True)
    py3_sig = _hmac.new(b"change-me-r680", py3_raw.encode(), hashlib.sha256).hexdigest()

    assert py2_raw == py3_raw, (
        "raw bytes differ between interpreters:\n  py3: %s\n  py2: %s"
        % (py3_raw, py2_raw)
    )
    assert py2_sig == py3_sig, (
        "HMAC differs between interpreters -- the downlink would be rejected"
    )


def test_naive_form_really_does_differ_across_versions():
    """Proof that the hazard is real, so the guard is not superstition.

    If Python 2 and Python 3 happened to agree on this body, the whole bug class
    would be moot and the sort_keys requirement would be cargo cult. They do not
    agree, and this pins that down using the same two interpreters.
    """
    py2 = _python2()
    if py2 is None:
        pytest.skip("python2 not available")

    # Build the payload so that JSON key order is NOT already sorted, then have
    # python2 re-serialise it the naive way -- this is what the receiver did.
    payload = json.dumps(SAMPLE, separators=(",", ":"), sort_keys=True)
    script = (
        "import json,sys\n"
        "body=json.loads(sys.argv[1])\n"
        "print(json.dumps(body,separators=(',',':')).encode().decode())\n"
    )
    proc = subprocess.run([py2, "-c", script, payload],
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, proc.stderr
    py2_naive = proc.stdout.strip()
    py3_naive = json.dumps(SAMPLE, separators=(",", ":"))

    assert py2_naive != py3_naive, (
        "python2 and python3 agreed on the naive serialisation for this sample, "
        "so this test no longer demonstrates the cross-version hazard -- pick a "
        "sample whose key order differs"
    )
