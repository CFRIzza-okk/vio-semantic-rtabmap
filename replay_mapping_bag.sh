#!/usr/bin/env bash
# ============================================================================
# replay_mapping_bag.sh —— 离线回放 rosbag 并重建 RTAB-Map 地图
#
# 用法：
#   ./replay_mapping_bag.sh data/bags/mapping_20260101_120000
#   ./replay_mapping_bag.sh data/bags/mapping_20260101_120000 --no-rviz
#   ./replay_mapping_bag.sh data/bags/mapping_20260101_120000 --rate=0.5
#
# 注意：回放时不启动 RealSense 和实时视觉里程计；相机图像、深度、内参、
# 录制时的 /odom 和 /tf_static 全部来自 rosbag，RTAB-Map 使用录制的 /odom 重建地图。
# ============================================================================

set -Ee
set +u

VIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEMANTIC_WS="$VIO_DIR/semantic_ws"
BAG=""
USE_RVIZ=true
RATE="1.0"
DB_PATH=""

if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  sed -n '2,15p' "$0"
  exit 0
fi
BAG="$1"
shift
for arg in "$@"; do
  case "$arg" in
    --no-rviz) USE_RVIZ=false ;;
    --rate=*) RATE="${arg#--rate=}" ;;
    --db=*) DB_PATH="${arg#--db=}" ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "错误：未知参数 $arg" >&2; exit 2 ;;
  esac
done

source /opt/ros/humble/setup.bash
[ -f "$SEMANTIC_WS/install/setup.bash" ] && source "$SEMANTIC_WS/install/setup.bash"

if [ ! -d "$BAG" ]; then
  echo "错误：找不到 rosbag 目录：$BAG" >&2
  exit 1
fi
if ! ros2 bag info "$BAG" >/dev/null 2>&1; then
  echo "错误：$BAG 不是可读取的 rosbag。" >&2
  exit 1
fi

if [ -z "$DB_PATH" ]; then
  DB_PATH="$HOME/.ros/rtabmap_replay_$(date +%Y%m%d_%H%M%S).db"
fi
RVIZ_CONFIG="$VIO_DIR/rtabmap_d435.rviz"
PIDS=()
CLEANED=false

cleanup() {
  [ "$CLEANED" = true ] && return
  CLEANED=true
  echo ""
  echo "[停止] 正在停止回放和 RTAB-Map..."
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 5
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
  echo "[完成] 回放数据库：$DB_PATH"
}
trap cleanup INT TERM EXIT

if pgrep -f 'realsense2_camera_node|rtabmap_slam.*rtabmap|semantic_detector|semantic_mapper|ros2 bag play' >/dev/null 2>&1; then
  echo "错误：检测到已有相机、RTAB-Map、语义节点或 rosbag 回放进程。" >&2
  exit 1
fi

if [ "$USE_RVIZ" = true ]; then
  RVIZ_ARGS=(rviz:=true "rviz_cfg:=$RVIZ_CONFIG")
else
  RVIZ_ARGS=(rviz:=false)
fi

LOG="$VIO_DIR/data/replay_rtabmap_$(date +%Y%m%d_%H%M%S).log"
# RTAB-Map 使用录制时的 /odom；不重新运行 visual_odometry，避免离线
# 再次受到白墙、帧率变化或图像同步差异影响。
setsid ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=false \
  icp_odometry:=false \
  use_sim_time:=false \
  odom_topic:=/odom \
  publish_tf_odom:=false \
  publish_tf_map:=true \
  frame_id:=base_link \
  subscribe_depth:=true \
  subscribe_rgbd:=false \
  approx_sync:=true \
  approx_sync_max_interval:=0.20 \
  odom_sensor_sync:=true \
  queue_size:=100 \
  qos_image:=1 \
  qos_camera_info:=1 \
  qos_odom:=1 \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  rtabmap_viz:=false \
  "${RVIZ_ARGS[@]}" \
  "database_path:=$DB_PATH" \
  >"$LOG" 2>&1 &
PIDS+=("$!")

sleep 8

echo "[回放] $BAG"
# 回放图像、深度、相机内参、录制时的 /odom 和静态 TF；排除旧动态 /tf。
setsid ros2 bag play "$BAG" --clock --rate "$RATE" \
  --topics \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/color/camera_info \
  /odom \
  /tf_static \
  &
PIDS+=("$!")

cat <<EOF

============================================================
  rosbag 离线回放中
============================================================
  rosbag：$BAG
  数据库：$DB_PATH
  速率：$RATE
  RViz：$USE_RVIZ

  回放结束后，地图数据库将保存在上述路径。
============================================================
EOF

wait "${PIDS[1]}"
echo "[完成] rosbag 回放结束。"
