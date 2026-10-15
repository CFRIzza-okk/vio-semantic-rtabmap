#!/usr/bin/env bash
# Bring up the full R680 stack for Phase D (goal selection, NO motion).
#
# Safety posture of this script:
#   * enable_cmd_vel is NOT passed as true anywhere, so the TX1-side relay never
#     creates a /cmd_vel publisher.  Verify with:
#       rostopic info /cmd_vel   ->  "Publishers: None"
#   * the explorer runs with dry_run:=true, so it never calls Nav2.
#   * include_nav2:=false, so there is no controller that could move the base.
#
# Usage:  bash tools/r680_phase_d_up.sh [--with-rviz]
#
# Start order matters and is not arbitrary:
#   1. TX1 driver first  -- /odom, /imu and /tf must exist before anything
#      downstream, and the relay's uplink subscriptions bind to those topics.
#   2. TX1 relay second  -- it starts pushing UDP as soon as it has data.
#   3. Orin relay third  -- binds 19001 and starts republishing.
#   4. camera, then static TF / safety, then RTAB-Map.
#      RTAB-Map last of the perception chain because it needs /odom, /tf and
#      both camera streams already flowing; starting it earlier just produces a
#      burst of "Did not receive data" warnings.
#   5. explorer last.
#
# NOTE: never add `set -u` here.  ROS's setup.bash dereferences unbound variables
# and nounset kills the script before it prints anything.
set -o pipefail

WS=/home/jetson/VIO/semantic_ws
TMP=/tmp/r680_phase_d
mkdir -p "$TMP"

say() { printf '\n=== %s ===\n' "$1"; }

# ---------------------------------------------------------------- TX1 side
# IMPORTANT: the cleanup and the launch are deliberately SEPARATE ssh calls.
#
# Doing them in one call is a self-match trap that cost real debugging time: the
# single bash command line then contains both the pkill PATTERN and the literal
# string it matches (e.g. `pkill -f "[t]urn_on_wheeltec"` next to
# `roslaunch turn_on_wheeltec_robot ...`). The bracket trick does not help,
# because the *roslaunch* argument supplies the plain text. pkill therefore
# matched its own shell, killed it, and every later command in that call -- the
# launch itself, and the verification -- silently never ran. The symptom was an
# empty step that looked like "no output to report" rather than a failure.
say "1/6 TX1: stop stale processes"
# Bracket patterns, and they are load-bearing here.
#
# `pkill -f PATTERN` matches the pattern against every process's full command
# line -- INCLUDING the shell that is running this cleanup, whose command line is
# the whole script text below. With a plain pattern ("ros1_relay") the shell
# matches itself, dies at the first pkill, and every later line in this block is
# silently skipped: the remaining pkills never run and `echo cleaned` never
# prints. That is exactly what happened, and it is why this step reported no
# output at all.
#
# The bracket form survives that because the shell's command line contains
# "[r]os1_relay" -- after the `r` comes `]`, so the regex `[r]os1_relay` does not
# match its own source text. This trick is ONLY safe while no plain occurrence of
# the name appears in the same command; that is why the driver launch is a
# separate ssh call (see the note above), where the plain text does appear.
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
pkill -f "[r]os1_relay" 2>/dev/null
pkill -f "[w]heeltec_robot" 2>/dev/null
pkill -f "[r]oscore" 2>/dev/null
sleep 3
echo "cleaned"' 2>&1 | tail -2

say "1/6 TX1 driver"
timeout 120 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
source /home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
cd /home/wheeltec
nohup roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch > /home/wheeltec/driver.log 2>&1 &
sleep 28
echo "--- nodes ---"
timeout 10 rosnode list 2>&1 | head -10
echo "--- driver log tail ---"
tail -3 /home/wheeltec/driver.log 2>/dev/null' 2>&1 | tail -14

say "2/6 TX1 relay"
# _enable_cmd_vel:=false is passed EXPLICITLY, not omitted.
#
# ROS 1's master persists a node's private parameters after the process dies, so
# a relay started with no flag inherits whatever the previous run set -- including
# a leftover `true`. Omitting the flag therefore does NOT close the gate; it only
# hides which state you are in. Phase D must never be able to move the robot, so
# say it explicitly. Use tools/r680_gate.sh status to verify the real state.
timeout 80 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
nohup python2 /home/wheeltec/ros1_relay.py _peer_host:=192.168.2.15 _enable_cmd_vel:=false > /home/wheeltec/relay.log 2>&1 &
sleep 6
echo "--- relay pid ---"
ps -eo pid,args | grep "ros1_relay" | grep -v grep | head -2
echo "--- relay log tail ---"
tail -4 /home/wheeltec/relay.log 2>/dev/null' 2>&1 | tail -9

# ---------------------------------------------------------------- Orin side
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

say "3/6 Orin relay"
nohup ros2 launch semantic_rtabmap r680_relay.launch.py > "$TMP/relay.log" 2>&1 &
sleep 10
timeout 10 ros2 topic info /odom 2>&1 | head -2

say "4/6 camera"
nohup ros2 launch semantic_rtabmap r680_depth_cloud.launch.py \
    start_inspector:=false > "$TMP/cam.log" 2>&1 &
sleep 22
timeout 10 ros2 topic info /camera/camera/aligned_depth_to_color/image_raw \
    2>&1 | head -2

say "5/6 static TF + safety monitors"
nohup ros2 launch semantic_rtabmap r680_navigation.launch.py \
    publish_base_tf:=false > "$TMP/nav.log" 2>&1 &
sleep 12
# `ros2 topic hz | grep` is unreliable here: hz repaints its line with \r and grep
# buffers, so a working topic can report nothing. Publisher/subscriber counts are
# deterministic and instant, so use those for the bring-up checks.
timeout 10 ros2 topic info /camera/camera/aligned_depth_to_color/image_raw 2>&1 | head -2
# NOTE: /scan is deliberately NOT checked here. depthimage_to_laserscan belongs to
# the EXPLORATION launch, not to r680_navigation, so at this point the topic does
# not exist yet and the check prints "Unknown topic '/scan'" -- which reads like a
# failure but is only a sequencing mistake. It is verified after step 6, below.

say "6/6 RTAB-Map"
rm -f "$HOME/.ros/rtabmap_phaseD.db"
nohup ros2 launch rtabmap_launch rtabmap.launch.py \
    visual_odometry:=false icp_odometry:=false \
    odom_topic:=/odom frame_id:=base_footprint \
    depth:=true approx_sync:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    rtabmap_viz:=false rviz:=false publish_tf_map:=true \
    "args:=--Rtabmap/DetectionRate 2.0 --Grid/CellSize 0.05 --Mem/NotLinkedNodesKept false" \
    "database_path:=$HOME/.ros/rtabmap_phaseD.db" > "$TMP/rtab.log" 2>&1 &
sleep 38
echo -n "  angular-variance warnings: "
grep -ci "large angular variance" "$TMP/rtab.log"
echo -n "  db size: "
ls -lh "$HOME/.ros/rtabmap_phaseD.db" 2>/dev/null | awk '{print $5}'

say "explorer (dry_run, no Nav2)"
nohup ros2 launch semantic_rtabmap r680_exploration.launch.py \
    include_nav2:=false include_safety:=false dry_run:=true \
    > "$TMP/explore.log" 2>&1 &
sleep 18
grep -E "explorer ready|localization_valid" "$TMP/explore.log" | tail -3

# /scan and /battery_voltage both come from launches started above, so they can
# only be checked now. Each MUST show a publisher.
say "scan + battery (both must have a publisher)"
echo -n "  /scan:             "; timeout 10 ros2 topic info /scan 2>&1 | grep -i publisher | head -1
echo -n "  /battery_voltage:  "; timeout 10 ros2 topic info /battery_voltage 2>&1 | grep -i publisher | head -1
# A live voltage proves the whole chain (TX1 driver -> TX1 relay -> UDP -> ROS 2
# relay). Without it the exploration node's low-voltage guard is inert again.
timeout 15 ros2 topic echo --once /battery_voltage 2>&1 | head -2

say "safety gate"
echo "  TX1 /cmd_vel publishers (MUST be None):"
timeout 90 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
timeout 10 rostopic info /cmd_vel 2>&1 | head -4
echo "--- odom position (should stay near origin) ---"
timeout 10 rostopic echo -n 1 /odom/pose/pose/position 2>&1 | head -4' 2>&1 | tail -8

say "done -- logs in $TMP"
