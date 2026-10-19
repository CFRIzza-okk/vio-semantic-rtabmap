#!/usr/bin/env python3
"""Remote command lines must never match the shell that runs them.

The defect
----------
`pkill -f PATTERN` and `pgrep -f PATTERN` match the pattern against the FULL
command line of every process -- including the shell whose command line IS the
command being run (paramiko/ssh runs it as `bash -c "<the whole thing>"`).

So a remote command that both (a) signals or queries a name and (b) contains
that name plainly SOMEWHERE ELSE on the same command line is matching itself.
`safe_kill.py` documents this and cost this project seven self-kills in one
session: pkill killed its own shell, so every later line in that call silently
never ran and the bring-up looked like "no output".

The bracket trick (`"[r]os1_relay"`) is NOT a general fix. It only holds while
no plain occurrence of the name sits in the same command line -- and that
second occurrence is easy to add without noticing: a relay path in a `nohup`
line, a message, a `grep`.

Measured on TX1 (2026-10-17), in exactly the shape of `r680_gate.start_relay`:

    $ pgrep -af "[r]os1_relay"        # same command also has /home/wheeltec/ros1_relay.py
    29486 python2 /home/wheeltec/ros1_relay.py _peer_host:=192.168.2.15 ...
    31232 bash -c echo ... /home/wheeltec/ros1_relay.py ... pgrep -af "[r]os1_relay"

31232 is the shell running the query. Any `pgrep ... && break` readiness loop
therefore breaks on its FIRST iteration, before the process it waits for has
even forked -- a wait that silently waits for nothing. Nothing fails; the loop
just stops being a loop.

What this test enforces
-----------------------
For every remote command this repo builds:
  * tools/*.sh  -- each quoted argument handed to `tx1.py`
  * tools/*.py  -- each function that assembles one, approximated by the string
    literals it contains PLUS the module-level string constants it reads. That
    resolution matters: `start_relay` never spells the relay path in its own
    literal, it interpolates the module constant `RELAY` into the launch line
    and then queries the same name with `pgrep -f "[r]os1_relay"`. Checking
    literals alone would miss exactly the defect this file exists for, and
    checking the whole file would blame `stop_relay`, whose command line really
    is free of the plain name.

  TRAP 1  a bracketed `pkill`/`pgrep -f "[x]name"` plus a PLAIN occurrence of
          `name` in the same scope -> the call matches its own shell.
  TRAP 2  (shell scripts only) a line inside the quoted argument that merely
          LOOKS like a comment: `#` does not start a comment there, so an
          apostrophe in it terminates the string. With an even number of stray
          apostrophes the file still passes `bash -n`, the argument is split
          into extra words, and the remote command is silently mangled -- which
          is worse than a syntax error.

Queries that do NOT use pkill/pgrep are out of scope here; the point is the
pattern-matching family that matches command lines, not process inspection in
general.
"""
import ast
import re
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOLS = REPO_ROOT / "tools"

# `pkill -f "[r]os1_relay"` / `pgrep -af '[w]heeltec_robot'`, with the bracket
# trick. Capture the real name (group 2 + group 3) so it can be compared with
# plain occurrences elsewhere.
BRACKET_PATTERN = re.compile(
    r"""(?:pkill|pgrep)\s+(?:-[\w-]+\s+)*-f\s+(['"])\[(\w)\](\w+)\1"""
)


def _strip_bracket_forms(text):
    """Remove `[x]name` so only PLAIN occurrences of a name remain findable."""
    return re.sub(r"\[(\w)\](\w+)", r"", text)


def _tx1_arguments(text):
    """Yield (argument, first line number, quote char) for each `tx1.py` call.

    The argument ends at the closing quote that is followed by a redirection or
    a pipe -- which is how every call site in this repo is written.
    """
    for m in re.finditer(r"tx1\.py\s+(['\"])", text):
        quote = m.group(1)
        start = m.end()
        lineno = text.count("\n", 0, start) + 1
        i = start
        while True:
            i = text.find(quote, i)
            if i == -1:
                break
            # The call ends at the closing quote followed by a redirection or a
            # pipe. `2>/dev/null` must be accepted as well as `2>&1`: matching
            # only the latter made this scan run PAST the closing quote of
            # r680_explore_watch.sh and report a stray apostrophe that is not
            # there.
            if re.match(r"\s*(?:[0-9]?>|\|)", text[i + 1:i + 15]):
                yield text[start:i], lineno, quote
                break
            i += 1


def _string_literals(path):
    """Every non-docstring str constant in a Python module."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            body = getattr(node, "body", [])
            if body and isinstance(body[0], ast.Expr) \
                    and isinstance(body[0].value, ast.Constant) \
                    and isinstance(body[0].value.value, str):
                docstrings.add(id(body[0].value))
    return [n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and id(n) not in docstrings]


def _is_docstring(node):
    return (isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str))


def _command_scopes(path):
    """Yield (label, assembled-text) for each place a remote command is built.

    One scope per function, plus one for module-level code. A scope contains the
    function's own string literals AND the value of every module-level string
    constant it reads -- because that is how the relay path reaches the command
    line that also queries the relay by name.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))

    module_consts = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Constant) \
                and isinstance(node.value.value, str):
            module_consts[node.targets[0].id] = node.value.value

    def scope_text(nodes):
        parts = []
        for node in nodes:
            for sub in ast.walk(node):
                if _is_docstring(sub):
                    continue
                if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
                    parts.append(sub.value)
                elif isinstance(sub, ast.Name) and sub.id in module_consts:
                    parts.append(module_consts[sub.id])
        return "\n".join(parts)

    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield "%s()" % node.name, scope_text([node])
    # Module-level code only -- walking the whole tree here would fold every
    # function into one scope and blame the module for a function's command.
    yield "<module>", scope_text(
        [n for n in tree.body
         if not isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef,
                               ast.ClassDef))])


def _trap1(scope_text, label, violations):
    """Report bracketed patterns whose name also appears plainly in scope."""
    names = {a + b for _, a, b in BRACKET_PATTERN.findall(scope_text)}
    if not names:
        return
    plain = _strip_bracket_forms(scope_text)
    for name in sorted(names):
        if name in plain:
            violations.append(
                "TRAP1 self-match in %s: `%s` is used with the bracket trick, "
                "but the same command line also contains the plain name "
                "`%s`. pgrep/pkill will match the shell running the command "
                "(measured: pid equals that shell), so a `pgrep ... && break` "
                "readiness loop returns immediately and the call is a no-op."
                % (label, name, name))


def test_shell_scripts_do_not_match_their_own_shell():
    """Trap 1, per tx1.py argument -- the scope that shares a command line."""
    violations = []
    checked = 0
    for path in sorted(TOOLS.glob("*.sh")):
        text = path.read_text(encoding="utf-8")
        for n, (arg, lineno, _q) in enumerate(_tx1_arguments(text), 1):
            checked += 1
            _trap1(arg, "%s tx1.py call#%d (line %d)" % (path.name, n, lineno),
                   violations)

    assert checked >= 5, (
        "found only %d tx1.py arguments; the extractor has gone stale and this "
        "check is no longer looking at the real call sites" % checked
    )
    assert not violations, "\n\n".join(violations)


def test_python_tools_do_not_match_their_own_shell():
    """Trap 1, per assembled remote command, for the paramiko-based tools.

    See `_command_scopes`: the scope is the function, with module-level string
    constants resolved, because a tool reaches the trap by interpolating such a
    constant into a command line that also queries the name.
    """
    violations = []
    checked = 0
    for path in sorted(TOOLS.glob("*.py")):
        for label, text in _command_scopes(path):
            if not BRACKET_PATTERN.search(text):
                continue
            checked += 1
            _trap1(text, "%s %s" % (path.name, label), violations)

    assert checked >= 1, (
        "found no scope building a bracketed pkill/pgrep command (the gate "
        "tool's stop path is the one that legitimately still does); the "
        "extractor has gone stale and this check is vacuous"
    )
    assert not violations, "\n\n".join(violations)


def test_no_fake_comments_inside_tx1_arguments():
    """Trap 2: `#` is not a comment inside a quoted tx1.py argument.

    An apostrophe in what reads like a comment ends the string early; with an
    even number of them the script still passes `bash -n` and the remote
    command is silently split into extra words.
    """
    violations = []
    for path in sorted(TOOLS.glob("*.sh")):
        text = path.read_text(encoding="utf-8")
        for n, (arg, lineno, _q) in enumerate(_tx1_arguments(text), 1):
            for offset, line in enumerate(arg.splitlines()):
                if line.lstrip().startswith("#"):
                    violations.append(
                        "%s tx1.py call#%d (file line %d): the argument contains "
                        "what looks like a comment -- `#` has no special meaning "
                        "there, and an apostrophe in it truncates the argument:\n"
                        "  %s" % (path.name, n, lineno + offset, line.strip())
                    )
    assert not violations, "\n\n".join(violations)


def test_single_quoted_arguments_contain_no_apostrophe():
    """Trap 2's actual mechanism: the quote that closes the argument.

    A single-quoted argument CANNOT contain an apostrophe -- the first one ends
    the string. With an ODD number the file fails `bash -n` (annoying but loud,
    and the error is reported many lines later, on the next plain quote). With
    an EVEN number the file still parses and the argument is quietly split into
    extra words, which is the dangerous case: the remote command is mangled and
    nothing says so.

    This test was written because the fix for the roscore step tripped it: a
    sentence containing `driver's` went inside the argument, and bash reported
    the resulting break 20 lines further down. Checking for the character is
    the only way to catch the even-number case statically.
    """
    violations = []
    for path in sorted(TOOLS.glob("*.sh")):
        text = path.read_text(encoding="utf-8")
        for n, (arg, lineno, quote) in enumerate(_tx1_arguments(text), 1):
            if quote != "'":
                continue
            if "'" in arg:
                line = arg[:arg.index("'")].count("\n") + lineno
                violations.append(
                    "%s tx1.py call#%d: the argument is single-quoted but "
                    "contains an apostrophe at file line %d -- that apostrophe "
                    "ends the string, and the remote command is truncated there "
                    "(or, with an even count, silently split into extra words).\n"
                    "  %s" % (path.name, n, line,
                             arg.splitlines()[
                                 arg[:arg.index("'")].count("\n")].strip())
                )
    assert not violations, "\n\n".join(violations)


@pytest.mark.parametrize("script", ["r680_phase_d_up.sh", "r680_motion_up.sh",
                                    "r680_explore_up.sh"])
def test_master_ownership_check_is_pinned_to_the_stack_port(script):
    """The check must inspect the master THIS stack will use.

    It used to take `head -1` of every rosmaster line. Measured with two masters
    alive (2026-10-17), that returns the LOWEST pid = the OLDEST master, so a
    leftover master from a previous session wins over the live one -- and the
    check reports the leftover as healthy while the master the driver will
    actually attach to is the landmine. Backwards for its purpose, so it is
    pinned to port 11311, the port the stack uses.
    """
    args = [a for a, _l, _q in _tx1_arguments((TOOLS / script).read_text())
            if "rosmaster" in a]
    assert args, "%s no longer inspects the master at all" % script
    arg = args[0]
    assert 'grep -- "-p 11311"' in arg, (
        "%s picks a rosmaster without pinning the port; `head -1` selects the "
        "oldest master, not this stack's" % script
    )
    assert "master pid=" in arg, (
        "%s must still report which master it inspected" % script
    )
    assert "-z \"$MPID\"" in arg, (
        "%s must distinguish 'no master on this port' (the roslaunch will "
        "create its own = the landmine) from a master owned by something else"
        % script
    )


def test_relay_readiness_poll_filters_on_process_name():
    """The fix must inspect processes by NAME, not by matching a command line.

    `start_relay` polls for the relay before returning, and the very same
    command line has to contain the relay's path in order to launch it. So the
    poll must filter on the process name (`ps -o comm`) -- the one property a
    shell running the command cannot fake. This is a structural guard: it fails
    if someone reintroduces a `pgrep -f` poll there.
    """
    scopes = dict(_command_scopes(TOOLS / "r680_gate.py"))
    assert "start_relay()" in scopes, "r680_gate.py must still start the relay"
    start = scopes["start_relay()"]

    assert "pgrep" not in start, (
        "start_relay's command line contains the relay's path, so a `pgrep -f "
        "ros1_relay` there matches the shell running the command and the "
        "readiness loop breaks on its first iteration"
    )
    assert "ps -eo" in start or "ps -o" in start, (
        "the relay readiness poll must read the process table with ps"
    )
    assert "comm" in start, (
        "the readiness poll must filter on the process NAME (ps comm)"
    )
    assert "ros1_relay" in start, "the poll must still look for the relay"
    assert "seq 1 20" in start or "sleep" in start, (
        "the poll must stay bounded in time"
    )


@pytest.mark.parametrize("script", ["r680_phase_d_up.sh", "r680_motion_up.sh",
                                    "r680_explore_up.sh", "r680_down.sh"])
def test_shell_scripts_still_parse(script):
    """Every fix here edits multi-line quoted arguments; keep them parseable.

    `bash -n` does not catch trap 2 (a truncated argument is still valid bash),
    so this is a floor, not the guard -- the guards are the tests above.
    """
    result = subprocess.run(["bash", "-n", str(TOOLS / script)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
