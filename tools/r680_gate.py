#!/usr/bin/env python3
"""Open, close or inspect the TX1 motion gate -- and verify it actually took.

Why this exists
---------------
The gate used to be controlled by "pass _enable_cmd_vel:=true or don't". That
is not reliable, because **ROS 1's master keeps a node's private parameters after
the process is gone**, SIGKILL included. So:

  * run the relay with _enable_cmd_vel:=true   -> master stores true
  * kill it (even uncleanly)
  * run the relay with NO flag                 -> it reads the STALE true
  * the gate is OPEN, and the operator believes it is closed

That is not hypothetical. It happened while parking the robot to charge: the
relay was restarted without the flag, and /cmd_vel still had a publisher. The
plan's rollback table said "restart without the flag, the default is off" -- that
sentence was false, and this is the kind of falsehood that gets a robot driven
into something.

What this script does about it
------------------------------
  1. deletes the relay's private params on the master, so nothing can be
     inherited from a previous process
  2. starts the relay with the flag passed EXPLICITLY, either way
  3. checks the EFFECTIVE parameter value
  4. checks /cmd_vel has a publisher if and only if the gate should be open
  5. prints one unambiguous verdict line

Step 4 is the one that matters. It does not trust the parameter, the log, or
this script's own intent -- it asks the master what is actually publishing.

Usage:
    python3 tools/r680_gate.sh open|close|status        (see r680_gate.sh)
or directly:
    python3 tools/r680_gate.py open --limits 0.08 0.0 0.3
"""
import argparse
import os
import sys
import time

import paramiko

TX1_HOST = "192.168.2.102"
TX1_USER = "wheeltec"
RELAY = "/home/wheeltec/ros1_relay.py"
LOG = "/home/wheeltec/relay.log"
NODE_NS = "/r680_ros1_relay"

# Params the relay reads privately. All of them are cleared before a start so a
# previous run cannot leak into this one.
PRIVATE_PARAMS = (
    "enable_cmd_vel",
    "max_linear_x",
    "max_linear_y",
    "max_angular_z",
    "cmd_timeout",
    "peer_host",
)

PROLOGUE = (
    "source /opt/ros/melodic/setup.bash\n"
    "export ROS_MASTER_URI=http://192.168.2.102:11311\n"
    "export ROS_IP=192.168.2.102\n"
)


def _password():
    path = os.path.expanduser("~/.tx1_pass")
    with open(path) as fh:
        return fh.read().strip()


def run(client, command, timeout=120):
    _, out, err = client.exec_command(command, timeout=timeout)
    o = out.read().decode(errors="replace")
    e = err.read().decode(errors="replace")
    return o + e


def connect():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(TX1_HOST, username=TX1_USER, password=_password(), timeout=20)
    return client


def clear_params(client):
    """Delete the relay's private params so a stale value cannot be inherited.

    ONE remote process for all of them. Six separate `rosparam delete` calls
    meant six python2 interpreter startups plus six master round-trips --
    measured at 1852 ms. A single rospy/mrosgraph script does all six over
    loopback in a few hundred ms. The effect on the master is identical.
    """
    cmd = (PROLOGUE
           + "python2 - <<'PYEOF'\n"
           + "import rosgraph\n"
           + "master = rosgraph.Master('/r680_gate')\n"
           + "for name in %r:\n" % (list(PRIVATE_PARAMS),)
           + "    try:\n"
           + "        master.deleteParam('%s/' + name)\n" % NODE_NS
           + "    except Exception:\n"
           + "        pass\n"
           + "print('cleared')\n"
           + "PYEOF\n")
    return run(client, cmd)


def stop_relay(client):
    # NOTE: the kill and the start must be SEPARATE ssh calls. A single command
    # line containing both the pkill pattern and the relay's path has the pattern
    # matching its own shell, which then dies before the launch line runs.
    #
    # And poll for the process to actually be gone rather than `sleep 3`: the
    # relay exits in well under a second, so the fixed sleep was pure waiting.
    # The bracket trick is used in BOTH the pkill and the pgrep so neither
    # matches this shell's own command line.
    cmd = ("pkill -f \"[r]os1_relay\" 2>/dev/null\n"
           "for _i in $(seq 1 24); do\n"
           "    pgrep -f \"[r]os1_relay\" >/dev/null || break\n"
           "    sleep 0.25\n"
           "done\n"
           "echo stopped\n")
    return run(client, cmd)


def start_relay(client, enabled, limits):
    args = ["_peer_host:=192.168.2.15", "_enable_cmd_vel:=%s" % ("true" if enabled else "false")]
    if enabled and limits:
        args += ["_max_linear_x:=%s" % limits[0],
                 "_max_linear_y:=%s" % limits[1],
                 "_max_angular_z:=%s" % limits[2]]
    # No `sleep 8` here. Waiting for the PROCESS is only a weak signal; what
    # actually matters is that the master sees the /cmd_vel publisher, and that
    # is what wait_gate() polls below. This loop just avoids racing the fork --
    # but it has to be able to TELL whether the relay is up, and matching the
    # command line cannot do that here: this command line must contain the
    # relay's path in order to launch it, so `pgrep -f "[r]os1_relay"` matches
    # the shell running the command. Measured on TX1 (2026-10-17):
    #
    #   $ pgrep -af "[r]os1_relay"          # same command, plain path above
    #   29486 python2 /home/wheeltec/ros1_relay.py ..._enable_cmd_vel:=false
    #   31232 bash -c ... /home/wheeltec/ros1_relay.py ... pgrep -af "[r]os1_relay"
    #
    # 31232 IS the querying shell, so the old loop broke on its first iteration
    # and waited for nothing at all -- a no-op dressed as a safeguard. The
    # bracket trick does not help; the second, plain occurrence is the launch
    # line itself.
    #
    # Filter on the process NAME instead: `ps -o comm` is the one property a
    # shell whose argument mentions the name cannot fake. (The awk process and
    # the shell both carry the name in their command lines, and both are
    # excluded by the comm test.)
    cmd = (PROLOGUE
           + "nohup python2 %s %s > %s 2>&1 &\n" % (RELAY, " ".join(args), LOG)
           + "for _i in $(seq 1 20); do\n"
           + "    ps -eo comm,args | awk '$1 == \"python2\" && /ros1_relay/ {found=1} END {exit !found}' && break\n"
           + "    sleep 0.2\n"
           + "done\n"
           + "ps -eo pid,comm,args | awk '$2 == \"python2\" && /ros1_relay/ {print $1, $3; exit}'\n")
    return run(client, cmd)


def wait_gate(client, want_open, limit=15.0):
    """Poll until /cmd_vel's publisher state actually matches the intent.

    This replaces the old fixed `sleep 8` + immediate check: if the relay needed
    slightly longer than 8 s the old code would report a spurious verification
    failure and force the gate closed. Polling on the real condition is both
    faster in the normal case and more correct in the slow one.
    """
    t0 = time.time()
    while time.time() - t0 < limit:
        if bool(cmd_vel_publishers(client)) == want_open:
            return True
        time.sleep(0.3)
    return False


def effective_param(client, name):
    out = run(client, PROLOGUE + "rosparam get %s/%s 2>&1 | tail -1" % (NODE_NS, name))
    return out.strip().splitlines()[-1].strip() if out.strip() else ""


def cmd_vel_publishers(client):
    out = run(client, PROLOGUE + "rostopic info /cmd_vel 2>&1 | head -6")
    if "Publishers: None" in out:
        return []
    found = []
    in_pub = False
    for line in out.splitlines():
        if line.startswith("Publishers:"):
            in_pub = True
            continue
        if line.startswith("Subscribers:"):
            break
        if in_pub and line.strip().startswith("*"):
            found.append(line.strip())
    return found


def report(client, want_open):
    """Verify the gate's REAL state, not the intent."""
    param = effective_param(client, "enable_cmd_vel")
    pubs = cmd_vel_publishers(client)

    print("  effective %s/enable_cmd_vel = %s" % (NODE_NS, param))
    print("  /cmd_vel publishers: %s" % (pubs if pubs else "None"))

    problems = []
    # The publisher list is ground truth: it is what actually drives the base.
    actual_open = bool(pubs)
    if want_open is not None and actual_open != want_open:
        problems.append(
            "gate is %s but was asked to be %s"
            % ("OPEN" if actual_open else "CLOSED", "OPEN" if want_open else "CLOSED"))
    # And the parameter must agree with reality, or a restart would surprise us.
    param_open = param == "true"
    if param_open != actual_open:
        problems.append(
            "parameter says enable_cmd_vel=%s but /cmd_vel publisher state says %s"
            % (param, "open" if actual_open else "closed"))

    if problems:
        for p in problems:
            print("  !! %s" % p)
        print("  VERDICT: UNVERIFIED -- do not trust this gate")
        return False

    print("  VERDICT: GATE %s (verified against /cmd_vel)" % ("OPEN" if actual_open else "CLOSED"))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("open", "close", "status"))
    parser.add_argument("--limits", nargs=3, metavar=("LIN_X", "LIN_Y", "ANG_Z"),
                        default=["0.08", "0.0", "0.3"],
                        help="clamps applied when opening (default 0.08 0.0 0.3)")
    args = parser.parse_args()

    client = connect()
    try:
        if args.action == "status":
            print("Motion gate status:")
            ok = report(client, want_open=None)
            return 0 if ok else 1

        want_open = args.action == "open"
        print("Motion gate -> %s" % ("OPEN" if want_open else "CLOSED"))
        print("  [1/4] clearing stale private params (ROS 1 masters keep them)")
        clear_params(client)
        print("  [2/4] stopping the relay")
        stop_relay(client)
        print("  [3/4] starting the relay with an EXPLICIT flag")
        start_relay(client, want_open, args.limits if want_open else None)
        print("  [4/4] verifying")
        if not wait_gate(client, want_open):
            print("  !! /cmd_vel did not reach the expected state within the timeout")
        ok = report(client, want_open=want_open)
        if not ok:
            # Fail loudly and leave the robot unable to move rather than
            # ambiguous. A half-open gate is the dangerous state.
            print("\nVerification failed -- forcing the gate closed")
            clear_params(client)
            stop_relay(client)
            start_relay(client, False, None)
            wait_gate(client, False)
            report(client, want_open=False)
            return 1
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
