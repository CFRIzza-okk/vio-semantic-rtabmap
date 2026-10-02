#!/usr/bin/env bash
set -eo pipefail

WS="$HOME/ros2_ws"
SRC="$WS/src/mavros"

if [[ ! -d "$SRC/.git" ]]; then
  mkdir -p "$WS/src"
  git clone --depth 1 --branch ros2 https://github.com/mavlink/mavros.git "$SRC"
fi

source /opt/ros/humble/setup.bash

sudo apt-get update
sudo apt-get install -y python3-rosdep python3-vcstool python3-colcon-common-extensions geographiclib-tools libgeographic-dev

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$SRC" --ignore-src -r -y --rosdistro humble

cd "$WS"
colcon build --symlink-install --packages-up-to mavros

# Install GeographicLib EGM96 geoid dataset (skip if already present).
if [[ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  sudo "$SRC/mavros/scripts/install_geographiclib_datasets.sh"
fi

sudo install -m 0755 "$HOME/VIO/launch_px4_mavros.sh" /usr/local/bin/launch_px4_mavros

echo
echo 'Build complete. In each new shell run:'
echo "source /opt/ros/humble/setup.bash && source $WS/install/local_setup.bash"
echo 'Verify with: ros2 topic hz /mavros/imu/data'
