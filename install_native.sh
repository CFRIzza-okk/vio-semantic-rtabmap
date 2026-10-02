#!/usr/bin/env bash
set -euo pipefail

# Native ROS 2 Humble + D435 driver + RTAB-Map install for Jetson/Ubuntu 22.04.
if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "22.04" ]]; then
  echo "This installer expects Ubuntu 22.04 (Jammy)." >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != "arm64" ]]; then
  echo "This installer is intended for Jetson ARM64." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y ros-humble-rtabmap-ros ros-humble-realsense2-camera ros-humble-realsense2-description ros-humble-rviz2 ros-humble-rtabmap-rviz-plugins

# Install a convenient native launcher (workspace RViz layout ships with the launcher).
sudo install -m 0755 launch_d435_rtabmap.sh /usr/local/bin/launch_d435_rtabmap
sudo install -D -m 0644 rtabmap_d435.rviz /usr/local/share/rtabmap_d435.rviz

# Set permissions for Intel RealSense USB devices, including D435 (VID 8086).
sudo install -m 0644 99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb

echo "Native ROS packages and RealSense udev permissions installed."
echo "Unplug/replug the D435, then source /opt/ros/humble/setup.bash and run launch_d435_rtabmap."
