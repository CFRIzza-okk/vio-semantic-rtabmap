#!/usr/bin/env bash
# launch_semantic_demo.sh — D435 + 语义感知层 一键启动
#
# 用法:
#   ./launch_semantic_demo.sh              # 离线自测: 发布静态 map->base_link
#   ./launch_semantic_demo.sh px4          # 实机: map 由 MAVROS 提供, 需先跑 launch_px4_mavros.sh
#   ./launch_semantic_demo.sh offline rtab # 额外再启动 RTAB-Map 建图
#
# 与 ~/VIO/launch_d435_rtabmap.sh 的分工:
#   本脚本只跑「相机 + 语义感知层」, 不启动 RTAB-Map。
#   好处是能先单独验证语义检测是否正常, 再叠加建图, 出问题时好定位。
set -eo pipefail
# ROS setup 脚本在 tracing 时会引用未定义变量
set +u

MODE="${1:-offline}"
WITH_RTABMAP="${2:-}"

source /opt/ros/humble/setup.bash
# MAVROS 来自源码工作空间 (注意用 local_setup.bash, install/setup.bash 前缀是旧的)
if [ -f "$HOME/ros2_ws/install/local_setup.bash" ]; then
  source "$HOME/ros2_ws/install/local_setup.bash"
fi
source "$HOME/VIO/semantic_ws/install/setup.bash"

if [ "$MODE" = "px4" ]; then
  PUB_STATIC_TF=false
  echo "[信息] PX4 模式: map->base_link 应由 MAVROS 发布"
  echo "[信息] 若还没启动 MAVROS, 请在另一个终端先跑: ~/VIO/launch_px4_mavros.sh"
else
  PUB_STATIC_TF=true
  echo "[信息] 离线模式: 发布静态 map->base_link 作为里程计替身"
fi

# ---------------- RealSense D435 ----------------
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true &
CAMERA_PID=$!

PIDS="$CAMERA_PID"

if [ -n "$WITH_RTABMAP" ]; then
  # 可选: 叠加上 RTAB-Map, 验证语义层与建图能共存 (Orin NX 8GB 上要注意负载)
  ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_viz:=false rviz:=false \
    localization:=false \
    subscribe_depth:=true approx_sync:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=base_link publish_tf_map:=false \
    odom_topic:=/mavros/local_position/odom \
    visual_odometry:=false \
    database_path:=$HOME/.ros/rtabmap.db &
  RTABMAP_PID=$!
  PIDS="$PIDS $RTABMAP_PID"
fi

cleanup() {
  echo ""
  echo "[信息] 正在关闭..."
  # shellcheck disable=SC2086
  kill $PIDS 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 等相机把 USB 流和 TF 都建立起来再放语义节点
sleep 5

echo "[信息] 启动语义感知层..."
ros2 launch semantic_rtabmap semantic.launch.py \
  publish_static_map_tf:="$PUB_STATIC_TF" \
  target_frame:=map
