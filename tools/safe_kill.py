#!/usr/bin/env python3
"""Kill processes by command-line substring, WITHOUT ever killing the caller.

Why this exists
---------------
`pkill -f` / `pgrep -f` match against the full command line of every process,
INCLUDING the shell that is running the pkill. So

    pkill -f "rviz2 -d"

kills the shell whose command line contains the literal text `rviz2 -d` -- which
is the very shell that typed it. The bracket trick (`"[r]viz2"`) does not
generalise: it only works while the pattern is the *only* place the plain string
appears. Put the plain string anywhere else in the same command --
`roslaunch turn_on_wheeltec_robot ...`, a variable assignment, a later `grep` --
and the match comes back.

That cost seven self-kills in one session, several of which silently truncated a
bring-up script so that later steps never ran and looked like "no output".

This script instead reads /proc/<pid>/cmdline directly and refuses to signal:
  * itself,
  * any of its ancestors (its shell, the harness, up to init),
  * anything whose cmdline is this script.

Usage:
    python3 tools/safe_kill.py [--signal INT|TERM|KILL] [--dry-run] PATTERN...

Exit code is 0 even when nothing matched; matched PIDs are printed.
"""
import argparse
import os
import signal
import sys


def ancestors(pid, depth=12):
    """Walk up the process tree so we never signal our own shell."""
    out = set()
    cur = pid
    for _ in range(depth):
        out.add(cur)
        try:
            with open("/proc/%d/stat" % cur) as fh:
                # comm can contain spaces and parentheses; everything after the
                # final ')' is "state ppid ..."
                cur = int(fh.read().split(") ", 1)[1].split()[1])
        except Exception:
            break
    out.add(cur)
    return out


def cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            return fh.read().decode("utf-8", "replace").replace("\x00", " ").strip()
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--signal", default="INT", choices=["INT", "TERM", "KILL"])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("patterns", nargs="+")
    args = ap.parse_args()

    sig = {"INT": signal.SIGINT, "TERM": signal.SIGTERM, "KILL": signal.SIGKILL}[args.signal]
    me = os.getpid()
    protected = ancestors(me)
    self_text = os.path.basename(__file__)

    hits = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in protected:
            continue
        cmd = cmdline(pid)
        if not cmd:
            continue
        # never match the tool itself or an obvious editor/grep invocation
        if self_text in cmd or cmd.startswith("grep "):
            continue
        if any(p in cmd for p in args.patterns):
            hits.append((pid, cmd))

    if not hits:
        print("no matches")
        return 0

    for pid, cmd in hits:
        label = "would kill" if args.dry_run else "kill -%s" % args.signal
        print("%-9s %6d  %s" % (label, pid, cmd[:100]))
        if not args.dry_run:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                print("           (permission denied)")
    print("matched %d process(es); protected %d ancestor(s) incl. self"
          % (len(hits), len(protected)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
