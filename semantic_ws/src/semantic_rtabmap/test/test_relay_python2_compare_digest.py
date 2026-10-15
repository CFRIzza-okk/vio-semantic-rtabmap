#!/usr/bin/env python3
"""A valid cmd_vel packet must survive verification on Python 2.

The second bug in this path
---------------------------
Fixing the byte canonicalisation was necessary but NOT sufficient. Even with
identical bytes and a correct HMAC, TX1 discarded every velocity packet:

    TypeError: 'unicode' does not have the buffer interface

`json.loads()` returns `unicode` on Python 2. `sign()` returns a `str` hexdigest.
Python 2's `hmac.compare_digest()` requires two byte strings and raises for a
unicode argument -- for EVERY packet, however valid it is.

And the raise was invisible. The handler caught `(ValueError, KeyError,
TypeError)` and logged one generic "invalid relay packet", so a perfectly valid
cmd_vel with a growing sequence number was reported as though it were malformed
junk from somewhere on the network, rate-limited to one line per five seconds.

The symptom was therefore actively misleading
---------------------------------------------
While this was broken, ALL of the following looked healthy:
  * mapping and odometry, because the uplink verifier runs on Python 3, whose
    compare_digest() accepts str -- so only the downlink broke
  * `localization_valid` true, `obstacle_stop` false, safety chain nominally fine
  * TX1 receiving packets and seeing correct cmd_vel payloads in its own logs
  * the robot simply never moving

What is worth pinning down
--------------------------
1. compare_digest on Python 2 must be given bytes.
2. Both sides must do it, so the asymmetry cannot silently return.
3. A valid packet must actually be ACCEPTED, not merely "not crash" -- the test
   drives the real verification expression on real Python 2.
"""
import ast
import hashlib
import hmac
import json
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TX1_RELAY = REPO_ROOT / "tools" / "ros1_relay.py"
ORIN_RELAY = (REPO_ROOT / "semantic_ws" / "src" / "semantic_rtabmap"
              / "semantic_rtabmap" / "ros2_relay_node.py")
SECRET = "change-me-r680"

SAMPLE = {
    "kind": "cmd_vel",
    "seq": 7,
    "payload": {
        "linear": {"x": 0.06, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    },
    "sent_at": 123456789,
}


def _python2():
    for exe in ("python2", "python2.7"):
        try:
            r = subprocess.run([exe, "-c", "print(1)"], capture_output=True, timeout=10)
            if r.returncode == 0:
                return exe
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
    return None


def _sign(raw):
    return hmac.new(SECRET.encode(), raw, hashlib.sha256).hexdigest()


def _canonical(obj):
    return json.dumps(obj, separators=(",", ":"), sort_keys=True).encode()


def _extract_compare_digest_call(src, label):
    """Pull the real `hmac.compare_digest(...)` call text out of a relay source.

    Located via the AST, then sliced by the node's own line/column span. Two
    traps this avoids, both of which this test fell into first:

      * `src.find("compare_digest")` hits the occurrence inside the COMMENT that
        documents the type rules -- extracting `compare_digest(bytes, str)`,
        which then fails with "unsupported operand type(s): 'type' and 'type'".
        A plain text search cannot tell code from prose about code.
      * The call is wrapped across two lines, so a line-based split silently
        yields half of it and reads as though the relay were broken.
    """
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "compare_digest"):
            lines = src.splitlines(keepends=True)
            start = sum(len(l) for l in lines[:node.lineno - 1]) + node.col_offset
            end = sum(len(l) for l in lines[:node.end_lineno - 1]) + node.end_col_offset
            return src[start:end]
    raise AssertionError("%s relay: no compare_digest call found" % label)


# --------------------------------------------------------------------------
# 1. the failure mode itself, demonstrated on real Python 2
# --------------------------------------------------------------------------
def test_python2_compare_digest_rejects_unicode_but_accepts_bytes():
    """This is the bug, reproduced against a real Python 2.

    If this ever stops raising, the interpreter behaviour changed and the
    explicit .encode('ascii') is no longer load-bearing -- better to know that
    than to keep a ritual.
    """
    py2 = _python2()
    if py2 is None:
        pytest.skip("python2 not available; cannot exercise the Python 2 path")

    # NOTE: built by replace, not %-formatting -- the script itself contains
    # '%s' ('bytes:%s'), so %-formatting would try to consume it.
    script = (
        "import json,hmac,hashlib,sys\n"
        "body=json.loads(sys.argv[1])\n"
        "raw=json.dumps(body,separators=(',',':'),sort_keys=True).encode()\n"
        "sig=hmac.new(b'@SECRET@',raw,hashlib.sha256).hexdigest()\n"
        "outer=json.loads(sys.argv[2])\n"
        "print(type(outer['sig']).__name__)\n"
        "try:\n"
        "    hmac.compare_digest(outer['sig'], sig)\n"
        "    print('unicode:accepted')\n"
        "except TypeError:\n"
        "    print('unicode:TypeError')\n"
        "print('bytes:' + str(hmac.compare_digest(outer['sig'].encode('ascii'), sig)))\n"
    ).replace("@SECRET@", SECRET)
    body_json = json.dumps(SAMPLE, separators=(",", ":"), sort_keys=True)
    packet = json.dumps({"body": SAMPLE, "sig": _sign(_canonical(SAMPLE))},
                        separators=(",", ":"), sort_keys=True)

    r = subprocess.run([py2, "-c", script, body_json, packet],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, "python2 probe failed: %s" % r.stderr
    lines = r.stdout.strip().split("\n")
    assert lines[0] == "unicode", "expected json.loads to yield unicode on py2"
    assert "unicode:TypeError" in r.stdout, (
        "python2's compare_digest no longer rejects unicode; the .encode('ascii') "
        "call may no longer be required (re-examine before removing it)"
    )
    assert "bytes:True" in r.stdout, (
        "the encoded comparison must ACCEPT the valid signature"
    )


# --------------------------------------------------------------------------
# 2. the real verification expression, driven on both interpreters
# --------------------------------------------------------------------------
def test_valid_cmd_vel_packet_is_accepted_on_python2():
    """End-to-end: build the packet the Orin builds, verify it the way TX1 does.

    Runs the actual expression from the TX1 relay source, so a regression in the
    relay is caught here rather than only on the robot. Uses whatever the file
    currently contains for the comparison, which is the point -- this test fails
    if someone reverts the .encode().
    """
    py2 = _python2()
    if py2 is None:
        pytest.skip("python2 not available")

    src = TX1_RELAY.read_text()
    expr = _extract_compare_digest_call(src, "TX1")

    script = (
        "import json,hmac,hashlib,sys\n"
        "SECRET='@SECRET@'\n"
        "def sign(b): return hmac.new(SECRET.encode(),b,hashlib.sha256).hexdigest()\n"
        "def _canonical(o): return json.dumps(o,separators=(',',':'),sort_keys=True).encode()\n"
        "class Self:\n"
        "    sign=staticmethod(sign)\n"
        "self=Self()\n"
        "packet=sys.argv[2]\n"
        "outer=json.loads(packet); body=outer['body']; raw=_canonical(body)\n"
        "print('ACCEPTED' if @EXPR@ else 'REJECTED')\n"
    ).replace("@SECRET@", SECRET).replace("@EXPR@", expr)
    packet = json.dumps({"body": SAMPLE, "sig": _sign(_canonical(SAMPLE))},
                        separators=(",", ":"), sort_keys=True)
    body_json = json.dumps(SAMPLE, separators=(",", ":"), sort_keys=True)

    r = subprocess.run([py2, "-c", script, body_json, packet],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, (
        "the TX1 verification expression blew up on Python 2 -- this is exactly "
        "the bug that silently discarded every cmd_vel:\n%s" % r.stderr
    )
    assert "ACCEPTED" in r.stdout, (
        "a valid cmd_vel packet was REJECTED on Python 2:\n%s" % r.stdout
    )


# --------------------------------------------------------------------------
# 3. both sides stay explicit
# --------------------------------------------------------------------------
def test_both_sides_encode_BOTH_signatures_before_comparing():
    """A unicode/bytes asymmetry between the two ends is what hid this.

    BOTH operands, not just the received one. The type rules differ by
    interpreter:

        compare_digest(str,   str)    py2 OK   py3 OK
        compare_digest(bytes, str)    py2 OK   py3 TypeError
        compare_digest(bytes, bytes)  py2 OK   py3 OK

    Only (bytes, bytes) works on both, so encoding a single argument is WORSE
    than encoding neither -- it moves the failure to the other machine. Not
    theoretical: encoding only the received signature fixed TX1 and broke the
    Python 3 relay's round trip, which the full suite caught and the first
    version of this very test had let through.

    Structural, not textual: find each compare_digest call and require .encode()
    on EVERY argument.
    """
    for path, label in ((TX1_RELAY, "TX1"), (ORIN_RELAY, "Orin")):
        tree = ast.parse(path.read_text())
        found = 0
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            if not (isinstance(f, ast.Attribute) and f.attr == "compare_digest"):
                continue
            found += 1
            assert len(node.args) >= 2, (
                "%s relay line %d: compare_digest needs two arguments"
                % (label, node.lineno))
            for arg in node.args:
                ok = (isinstance(arg, ast.Call)
                      and isinstance(arg.func, ast.Attribute)
                      and arg.func.attr == "encode")
                assert ok, (
                    "%s relay line %d passes a non-bytes operand to "
                    "compare_digest. Python 2 raises TypeError for a unicode "
                    "argument (discarding every packet); Python 3 raises "
                    "TypeError when the operands are different types. Only "
                    "(bytes, bytes) works on both, so BOTH must be encoded."
                    % (label, node.lineno)
                )
        assert found > 0, "%s relay: no compare_digest call found" % label


def test_the_expression_is_accepted_on_python3():
    """Drive the real Orin expression on Python 3 and require it to ACCEPT.

    The TX1-side test proves Python 2 accepts; this proves Python 3 still does.
    Between them, a fix that satisfies only one interpreter cannot pass -- which
    is the exact shape of the mistake made while fixing the original bug.
    """
    src = ORIN_RELAY.read_text()
    expr = _extract_compare_digest_call(src, "Orin")

    script = (
        "import hmac, hashlib, json, sys\n"
        "SECRET = '@SECRET@'\n"
        "def sign(b): return hmac.new(SECRET.encode(), b, hashlib.sha256).hexdigest()\n"
        "def canonical(o): return json.dumps(o, separators=(',', ':'), sort_keys=True).encode()\n"
        "class Self:\n"
        "    sign = staticmethod(sign)\n"
        "self = Self()\n"
        "outer = json.loads(sys.argv[1])\n"
        "body = outer['body']\n"
        "raw = canonical(body)\n"
        "print('OK' if @EXPR@ else 'MISMATCH')\n"
    ).replace("@SECRET@", SECRET).replace("@EXPR@", expr)

    raw = _canonical(SAMPLE)
    packet = json.dumps({"body": SAMPLE, "sig": _sign(raw)},
                        separators=(",", ":"), sort_keys=True)

    r = subprocess.run(["python3", "-c", script, packet],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, (
        "the Orin verification expression blew up on Python 3 -- mixing bytes "
        "and str in compare_digest raises TypeError there:\n%s" % r.stderr)
    assert "OK" in r.stdout, (
        "the Orin side REJECTED a valid packet on Python 3:\n%s" % r.stdout)


def test_generic_handler_does_not_hide_the_cause():
    """The exception message must name the type.

    The original logged a bare "invalid relay packet", which is what turned a
    total downlink outage into apparent background noise.
    """
    src = TX1_RELAY.read_text()
    assert "type(exc).__name__" in src, (
        "the packet-rejection warning must name the exception type; a single "
        "generic message cannot distinguish a bad signature from a TypeError"
    )
