"""Guards that the TX1-side relay stays runnable on Python 2.7.

Why this file exists
--------------------
TX1 is Ubuntu 18.04 / ROS Melodic, whose ``rospy`` is built for **Python 2.7**.
The relay script lives on the Orin side of the repo (in ``tools/``), where it
is edited and casually syntax-checked under Python 3 -- so it is very easy to
write Python-3-only code that runs fine in a local smoke test and then dies on
TX1. That is exactly what happened: the shipped script crashed on TX1 with

    TypeError: __init__() got an unexpected keyword argument 'daemon'

before it ever bound a socket, so the relay had never actually run on the
robot. A second audit found four more Python-3-only APIs, two of which would
only have fired once motion was enabled -- i.e. in the safety watchdog.

The checks below parse the script with ``ast`` rather than regex, so prose in
comments and docstrings explaining these very traps does not trip them.

Verified on TX1's Python 2.7.17 (all False = absent):
    json.JSONDecodeError, time.monotonic, math.isfinite,
    threading.Thread(daemon=...)
and ``issubclass(socket.error, OSError)`` is False there (it subclasses
IOError), which is why catching plain ``OSError`` loses socket errors on 2.7.
"""

import ast
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TX1_SCRIPT = REPO_ROOT / "tools" / "ros1_relay.py"


@pytest.fixture(scope="module")
def tree():
    assert TX1_SCRIPT.exists(), "tools/ros1_relay.py is missing"
    return ast.parse(TX1_SCRIPT.read_text()), TX1_SCRIPT.read_text()


def _attribute_names(tree):
    """Every dotted attribute access in the module, e.g. {'json.JSONDecodeError'}."""
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            found.add("%s.%s" % (node.value.id, node.attr))
    return found


# Each entry: (dotted name, the Python version that introduced it).
PY3_ONLY_ATTRIBUTES = [
    ("json.JSONDecodeError", "3.5"),
    ("time.monotonic", "3.3"),
    ("time.monotonic_ns", "3.7"),
    ("math.isfinite", "3.2"),
]


@pytest.mark.parametrize("name,introduced", PY3_ONLY_ATTRIBUTES)
def test_avoids_python3_only_apis(tree, name, introduced):
    """These are absent on 2.7 and raise AttributeError at run time there."""
    module, _source = tree
    assert name not in _attribute_names(module), (
        "%s is Python %s+ and does not exist on TX1's Python 2.7" % (name, introduced)
    )


def test_no_thread_daemon_keyword(tree):
    """threading.Thread(daemon=...) is Python 3.3+; on 2.7 it is a TypeError.

    This was the crash that stopped the relay from starting at all.
    """
    module, _source = tree
    for node in ast.walk(module):
        if not isinstance(node, ast.Call):
            continue
        if not (isinstance(node.func, ast.Attribute) and node.func.attr == "Thread"):
            continue
        kwargs = {kw.arg for kw in node.keywords}
        assert "daemon" not in kwargs, (
            "passing daemon= to Thread() raises TypeError on Python 2.7; "
            "set thread.daemon = True after construction instead"
        )


def test_monotonic_has_a_python2_fallback(tree):
    """time.monotonic must be imported defensively, not used as time.monotonic."""
    module, source = tree
    imported = any(
        isinstance(node, ast.ImportFrom) and node.module == "time"
        for node in ast.walk(module)
    )
    assert imported, "expected a guarded 'from time import monotonic' fallback"
    assert "ImportError" in source, "the monotonic fallback must catch ImportError"


def test_socket_error_alias_covers_both_interpreters(tree):
    """The shared alias must catch socket errors on 2.7 *and* 3.x.

    On Python 2.7 ``socket.error`` subclasses ``IOError``, NOT ``OSError``, so
    catching only ``OSError`` silently lets every socket send/recv failure
    escape. On Python 3 the two names are the same class, so listing both is
    correct on either interpreter.
    """
    module, _source = tree

    alias = None
    for node in ast.walk(module):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "_SOCKET_ERRORS":
                alias = node.value
    assert alias is not None, "expected a _SOCKET_ERRORS alias"

    caught = set()
    for element in getattr(alias, "elts", []):
        if isinstance(element, ast.Name):
            caught.add(element.id)
        elif isinstance(element, ast.Attribute) and isinstance(element.value, ast.Name):
            caught.add("%s.%s" % (element.value.id, element.attr))

    assert "OSError" in caught, "the alias must still catch OSError (Python 3)"
    assert "socket.error" in caught, (
        "the alias must catch socket.error explicitly - on Python 2.7 it does "
        "not inherit from OSError"
    )


def test_no_bare_except_oserror(tree):
    """A bare ``except OSError`` is the 2.7 trap this alias exists to avoid."""
    module, _source = tree
    for node in ast.walk(module):
        if isinstance(node, ast.ExceptHandler) and isinstance(node.type, ast.Name):
            assert node.type.id != "OSError", (
                "bare 'except OSError' misses socket.error on Python 2.7; "
                "use the _SOCKET_ERRORS alias"
            )


def test_shebang_targets_python2(tree):
    """The shebang must not advertise python3 - that is how it got run wrong."""
    _module, source = tree
    first = source.splitlines()[0]
    assert "python3" not in first, (
        "shebang says python3 but this script only runs under python2 on TX1"
    )


def test_compiles_under_python2_when_available():
    """Compile with a real python2 interpreter if this host happens to have one."""
    python2 = shutil.which("python2") or shutil.which("python2.7")
    if python2 is None:
        pytest.skip("no python2 interpreter on this host (TX1 has 2.7.17)")
    result = subprocess.run(
        [python2, "-m", "py_compile", str(TX1_SCRIPT)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, "python2 rejected the script:\n%s" % result.stderr


# ===========================================================================
# The second script deployed to TX1: tools/time_sync_client.py
# ===========================================================================
# It shipped with the *same class* of bug this file was written for, and the
# checks above missed it because they only ever looked at ros1_relay.py.  Two
# distinct defects, both of which pass py_compile:
#
#   1. ``print(..., flush=True)`` - the ``flush`` keyword is Python 3.3+.  On
#      TX1's 2.7 it raises ``TypeError: 'flush' is an invalid keyword argument
#      for this function``.  Because this was the script's *logging* call, it
#      died on its first print, before sending a single request - so the clock
#      correction it exists to perform had never once run on the robot.
#      ``py_compile`` cannot catch this: it is a runtime error, not a syntax one.
#
#   2. A ``#!/usr/bin/env python3`` shebang on a script deployed to TX1.  The
#      body is 2/3 compatible, so this is not fatal on its own - but it selects
#      a different interpreter than the intended one, and TX1 may have no
#      python3 at all.  It only appeared to work because it was invoked as
#      ``python2 time_sync_client.py`` explicitly.  ``ros1_relay.py``, the other
#      TX1 script, had already been fixed to python2.
#
# These tests parse with ``ast`` rather than searching text, because both files
# *document* these traps in their docstrings - a regex for "flush=True" matches
# the explanation of why not to write it.

TIME_SYNC_CLIENT = REPO_ROOT / "tools" / "time_sync_client.py"

TX1_SCRIPTS = [TX1_SCRIPT, TIME_SYNC_CLIENT]


@pytest.fixture(scope="module", params=TX1_SCRIPTS, ids=lambda path: path.name)
def tx1_tree(request):
    """Parse each script that gets deployed to TX1."""
    assert request.param.exists(), "%s is missing" % request.param
    return (
        ast.parse(request.param.read_text()),
        request.param.read_text(),
        request.param,
    )


def test_all_tx1_scripts_avoid_python3_only_apis(tx1_tree):
    """Every script deployed to TX1 must avoid APIs 2.7 does not have."""
    module, _source, path = tx1_tree
    names = _attribute_names(module)
    for name, introduced in PY3_ONLY_ATTRIBUTES:
        assert name not in names, (
            "%s uses %s (Python %s+), which is absent on TX1's Python 2.7"
            % (path.name, name, introduced)
        )


def test_no_print_flush_keyword(tx1_tree):
    """``print(..., flush=True)`` is Python 3.3+; on 2.7 it is a TypeError.

    Checked on the AST so the docstring prose warning about this very trap does
    not trip it.  ``py_compile`` passes for this bug, which is why it shipped.
    """
    module, _source, path = tx1_tree
    for node in ast.walk(module):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Name) and func.id == "print"):
            continue
        kwargs = {kw.arg for kw in node.keywords}
        assert "flush" not in kwargs, (
            "%s calls print(..., flush=True). On Python 2.7 that raises "
            "'TypeError: flush is an invalid keyword argument for this "
            "function' - and py_compile will NOT catch it. Use print() followed "
            "by sys.stdout.flush()." % path.name
        )


def test_tx1_scripts_do_not_advertise_python3(tx1_tree):
    """A python3 shebang sends the script to the wrong interpreter on TX1."""
    _module, source, path = tx1_tree
    first = source.splitlines()[0]
    assert "python3" not in first, (
        "%s has a python3 shebang but is deployed to TX1, whose interpreter for "
        "these scripts is python2 (see tools/ros1_relay.py)" % path.name
    )


def test_time_sync_client_is_syntactically_valid_python2():
    """py_compile catches syntax errors only - kept for that, not for the above."""
    python2 = shutil.which("python2") or shutil.which("python2.7")
    if python2 is None:
        pytest.skip("no python2 interpreter on this host (TX1 has 2.7.17)")
    result = subprocess.run(
        [python2, "-m", "py_compile", str(TIME_SYNC_CLIENT)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, (
        "python2 rejected %s:\n%s" % (TIME_SYNC_CLIENT.name, result.stderr)
    )
