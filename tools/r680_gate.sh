#!/usr/bin/env bash
# Open, close or inspect the TX1 motion gate.
#
#   bash tools/r680_gate.sh status      # what is the gate doing RIGHT NOW
#   bash tools/r680_gate.sh close       # stop the robot being able to move
#   bash tools/r680_gate.sh open        # allow motion (limits 0.08 / 0.0 / 0.3)
#
# Use this instead of restarting the relay by hand. It deletes the relay's stale
# private params before starting, passes the flag explicitly, and then VERIFIES
# the outcome against /cmd_vel -- because ROS 1's master remembers a node's
# private parameters after the process dies, so "start it without the flag" does
# NOT reliably close the gate (it once came back open, silently).
#
# Do not add `set -u`: ROS setup.bash references unset variables.

exec python3 "$(dirname "$0")/r680_gate.py" "$@"
