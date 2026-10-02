#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/local_setup.bash"
exec ros2 launch "$HOME/VIO/px4_tf.launch.xml" fcu_url:=serial:///dev/ttyACM0:115200
