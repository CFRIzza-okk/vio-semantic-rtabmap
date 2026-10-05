#!/usr/bin/env bash
# ============================================================================
# launch_showcase.sh —— 参观展示：稳定的 D435 + RTAB-Map + 语义建图
#
# 目标：只追求"能稳定建图 + 能稳定显示语义标注"，不启用未经系统对照验证的
# 质量自适应约束。默认使用 uniform 基线，和此前已经跑通的建图行为一致。
#
# 用法：
#   ./launch_showcase.sh                         默认启动 RViz、全新展示数据库
#   ./launch_showcase.sh --keep-db               继续使用本次展示数据库
#   ./launch_showcase.sh --no-rviz               不启动 RViz（故障排查用）
#   ./launch_showcase.sh --fast                  降低 VO 负载，CPU 紧张时使用
#   ./launch_showcase.sh --classes=chair,bottle  限制语义类别
#   ./launch_showcase.sh --help
#
# 展示建议：
#   1. 相机缓慢移动，保持画面有纹理；
#   2. 不要快速原地旋转，不要长时间对着白墙/窗户；
#   3. 绕一圈回到起点附近，触发回环；
#   4. RViz 中打开 Semantic Map 和 Semantic Labels 两个显示项。
#
# 退出：只在本终端按 Ctrl+C。脚本会按进程组清理相机、TF、RTAB-Map、
#       语义节点和 RViz，不留下孤儿 python3/ros2 子进程。
# ============================================================================

set -Ee
set +u  # ROS setup 脚本会引用未定义变量

VIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEMANTIC_WS="$VIO_DIR/semantic_ws"
DB_PATH="$HOME/.ros/rtabmap_showcase.db"
SEMANTIC_MAP="$VIO_DIR/data/showcase_semantic_map.json"
LOG_DIR="$VIO_DIR/data/showcase_logs"
RVIZ_CONFIG="$VIO_DIR/rtabmap_d435.rviz"

USE_RVIZ=true
KEEP_DB=false
FAST=false
# 默认只保留现场常见、稳定且有足够像素的类别；不包含 person，避免移动人被建成移动地标。
CLASS_FILTER="chair,bottle,backpack,suitcase,laptop"

for arg in "$@"; do
  case "$arg" in
    --no-rviz) USE_RVIZ=false ;;
    --keep-db) KEEP_DB=true ;;
    --fast) FAST=true ;;
    --classes=*) CLASS_FILTER="${arg#--classes=}" ;;
    -h|--help) sed -n '2,38p' "$0"; exit 0 ;;
    *) echo "错误：未知参数 $arg（使用 --help 查看用法）" >&2; exit 2 ;;
  esac
done

source /opt/ros/humble/setup.bash
if [ ! -f "$SEMANTIC_WS/install/setup.bash" ]; then
  echo "错误：语义工作空间尚未编译：$SEMANTIC_WS" >&2
  echo "请先执行：cd $SEMANTIC_WS && colcon build --symlink-install --parallel-workers 1" >&2
  exit 1
fi
source "$SEMANTIC_WS/install/setup.bash"

# ------------------------- 基础检查 -------------------------
if ! lsusb 2>/dev/null | grep -qi "Intel.*0b07\|RealSense"; then
  echo "错误：没有检测到 Intel RealSense D435。请检查 USB 3.0 接口和连接。" >&2
  exit 1
fi

if pgrep -f "realsense2_camera_node|rtabmap_slam.*rtabmap|semantic_detector|semantic_mapper" \
    >/dev/null 2>&1; then
  echo "错误：检测到已有相机/RTAB-Map/语义节点正在运行。" >&2
  echo "请先在原终端按 Ctrl+C 停止旧流程，再启动本脚本。" >&2
  pgrep -af "realsense2_camera_node|rtabmap_slam.*rtabmap|semantic_detector|semantic_mapper" >&2 || true
  exit 1
fi

mkdir -p "$LOG_DIR" "$HOME/.ros" "$VIO_DIR/data"

if [ "$KEEP_DB" = false ]; then
  STAMP="$(date +%Y%m%d_%H%M%S)"
  if [ -f "$DB_PATH" ]; then
    mv "$DB_PATH" "${DB_PATH}.bak.${STAMP}"
    echo "[准备] 旧展示数据库已备份为 ${DB_PATH}.bak.${STAMP}"
  fi
  if [ -f "$SEMANTIC_MAP" ]; then
    mv "$SEMANTIC_MAP" "${SEMANTIC_MAP}.bak.${STAMP}"
    echo "[准备] 旧语义地图已备份为 ${SEMANTIC_MAP}.bak.${STAMP}"
  fi
fi

# 每个 ros2 launch / ros2 run 都放入独立 session/process group。
# 退出时 kill -- -PID 会杀掉整个进程组，而不只是 ros2 launch 外壳。
GROUP_PIDS=()
CLEANED=false

start_group() {
  local name="$1"
  shift
  local log="$LOG_DIR/${name}_$(date +%Y%m%d_%H%M%S).log"
  echo "[启动] $name"
  echo "        日志：$log"
  setsid "$@" >"$log" 2>&1 &
  local pid=$!
  GROUP_PIDS+=("$pid")
  echo "$pid" >"$LOG_DIR/${name}.pid"
}

stop_groups() {
  [ "$CLEANED" = true ] && return
  CLEANED=true
  echo ""
  echo "[退出] 正在停止展示流程..."
  # 先发 SIGINT，让 ROS 节点保存数据库/语义地图并优雅退出。
  # 只在仍未退出时再发 TERM，最后才 KILL。
  for pid in "${GROUP_PIDS[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  sleep 5
  for pid in "${GROUP_PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${GROUP_PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
  done
  for pid in "${GROUP_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  echo "[退出] 已清理所有展示进程。"
  echo "[保存] RTAB-Map 数据库：$DB_PATH"
  echo "[保存] 语义地图：$SEMANTIC_MAP"
}

on_signal() {
  stop_groups
  exit 0
}
trap on_signal INT TERM
trap stop_groups EXIT

wait_node() {
  local node="$1"
  local timeout_s="${2:-45}"
  local start="$(date +%s)"
  while [ $(( $(date +%s) - start )) -lt "$timeout_s" ]; do
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
      echo "[就绪] 节点 $node"
      return 0
    fi
    sleep 1
  done
  echo "错误：等待节点 $node 超时。请查看 $LOG_DIR" >&2
  return 1
}

wait_topic() {
  local topic="$1"
  local timeout_s="${2:-45}"
  local start="$(date +%s)"
  while [ $(( $(date +%s) - start )) -lt "$timeout_s" ]; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "[就绪] 话题 $topic"
      return 0
    fi
    sleep 1
  done
  echo "错误：等待话题 $topic 超时。请查看 $LOG_DIR" >&2
  return 1
}

# ------------------------- 1. 相机 -------------------------
start_group camera ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
wait_node /camera/camera 45
wait_topic /camera/camera/color/image_raw 30
wait_topic /camera/camera/aligned_depth_to_color/image_raw 30
wait_topic /camera/camera/color/camera_info 30

# ------------------------- 2. 相机外参 -------------------------
start_group camera_tf ros2 run tf2_ros static_transform_publisher \
  --x 0.045 --y -0.075 --z -0.005 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
sleep 2

# ------------------------- 3. RTAB-Map + 视觉里程计 + RViz -------------------------
if [ "$FAST" = true ]; then
  ODOM_ARGS="--Vis/MaxFeatures 500 --Odom/ImageDecimation 2"
  echo "[配置] fast：VO 500 特征点 + 2 倍图像降采样"
else
  ODOM_ARGS="--Vis/MaxFeatures 800"
fi

if [ "$USE_RVIZ" = true ]; then
  RVIZ_ARGS=(rviz:=true "rviz_cfg:=$RVIZ_CONFIG")
else
  RVIZ_ARGS=(rviz:=false)
fi

start_group rtabmap ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=true \
  tag_linear_variance:=0.04 \
  tag_angular_variance:=9999.0 \
  icp_odometry:=false \
  odom_topic:=/odom \
  vo_frame_id:=odom \
  publish_tf_odom:=true \
  publish_tf_map:=true \
  frame_id:=base_link \
  subscribe_depth:=true \
  subscribe_rgbd:=false \
  approx_sync:=true \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  rtabmap_viz:=false \
  "${RVIZ_ARGS[@]}" \
  "odom_args:=$ODOM_ARGS" \
  "args:=--Rtabmap/DetectionRate 2.0 --Grid/CellSize 0.05" \
  "database_path:=$DB_PATH"
wait_node /rtabmap/rtabmap 60
wait_topic /odom 60

# ------------------------- 4. 语义检测 + 语义地图 -------------------------
# 关键：publish_static_map_tf=false。map->odom->base_link 由 RTAB-Map 独占，
# 不能再额外发布静态 map->base_link，否则 TF 会出现两个父节点。
start_group semantic ros2 launch semantic_rtabmap semantic.launch.py \
  publish_static_map_tf:=false \
  publish_static_camera_tf:=false \
  target_frame:=map \
  process_rate:=5.0 \
  "class_filter:=$CLASS_FILTER" \
  "persistence_path:=$SEMANTIC_MAP"
wait_node /semantic_detector 60
wait_node /semantic_mapper 60
wait_topic /semantics/objects 30
wait_topic /semantics/objects_tracked 30
wait_topic /semantics/map_markers 30

cat <<EOF

============================================================
  展示流程已稳定启动
============================================================
  RViz：$USE_RVIZ
  语义类别：$CLASS_FILTER
  数据库：$DB_PATH
  语义地图：$SEMANTIC_MAP
  日志目录：$LOG_DIR

  展示动作：
    1. 相机缓慢移动，画面保持有纹理；
    2. 让瓶子、椅子、背包等目标进入画面；
    3. 绕一圈回到起点附近，触发回环；
    4. RViz 中勾选 Semantic Map 和 Semantic Labels。

  退出方式：在本终端按 Ctrl+C（不要直接关终端）。
============================================================
EOF

wait
