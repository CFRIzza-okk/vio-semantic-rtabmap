#!/usr/bin/env bash
set -eo pipefail
# ROS setup scripts rely on unset variables while tracing their setup chain.
set +u
source /opt/ros/humble/setup.bash

# Start D435 driver with aligned depth, then RGB-D SLAM.
ros2 run tf2_ros static_transform_publisher --x 0.045 --y -0.075 --z -0.005 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id camera_link &
TF_PID=$!
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true &
CAMERA_PID=$!
cleanup() {
  kill "$CAMERA_PID" "$TF_PID" 2>/dev/null || true
  wait "$CAMERA_PID" "$TF_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_viz:=false \
  rviz:=true \
  localization:=false \
  subscribe_depth:=true \
  approx_sync:=true \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=base_link \
  publish_tf_map:=false \
  odom_topic:=/mavros/local_position/odom \
  visual_odometry:=false \
  database_path:=$HOME/.ros/rtabmap.db \
  rviz_cfg:=$HOME/VIO/rtabmap_d435.rviz
