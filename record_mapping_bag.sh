#!/usr/bin/env bash
# ============================================================================
# record_mapping_bag.sh —— 录制 D435 RGB-D + 视觉里程计 rosbag
#
# 用法：
#   ./record_mapping_bag.sh
#   ./record_mapping_bag.sh --name=消防走廊
#   ./record_mapping_bag.sh --fast
#   ./record_mapping_bag.sh --no-monitor
#
# 录制期间：
#   1. 脚本启动 launch_showcase.sh --no-rviz；
#   2. 脚本同时启动 VIO 发散监测，一旦视觉里程计持续失效就直接在本终端
#      打印 “VIO发散”，避免录完才发现 bag 里的 /odom 全程无效；
#   3. 携带相机走完整路线；
#   4. 回到起点后在本终端按 Ctrl+C；
#   5. rosbag 保存到 ~/VIO/data/bags/。
#
# 录制内容用于之后离线重建 RTAB-Map 地图，不依赖外网。
# ============================================================================

set -Ee
set +u

VIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="$VIO_DIR/data/bags"
SHOWCASE="$VIO_DIR/launch_showcase.sh"
NAME="mapping_$(date +%Y%m%d_%H%M%S)"
FAST=false
MONITOR=true

for arg in "$@"; do
  case "$arg" in
    --name=*) NAME="${arg#--name=}" ;;
    --fast) FAST=true ;;
    --no-monitor) MONITOR=false ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "错误：未知参数 $arg（使用 --help 查看用法）" >&2; exit 2 ;;
  esac
done

source /opt/ros/humble/setup.bash
if [ ! -f "$SHOWCASE" ]; then
  echo "错误：找不到 $SHOWCASE" >&2
  exit 1
fi
mkdir -p "$BAG_DIR"

LAUNCH_PID=""
BAG_PID=""
MON_PID=""
CLEANED=false
BAG_PATH="$BAG_DIR/$NAME"
MONITOR_PY="$VIO_DIR/tools/vio_divergence_monitor.py"

cleanup() {
  [ "$CLEANED" = true ] && return
  CLEANED=true
  echo ""
  echo "[停止] 正在停止 rosbag 和实时建图..."

  # rosbag 可能正在把缓存写入 SQLite 并执行 zstd 压缩；不能固定几秒后
  # KILL，否则数据库虽然很大，但 metadata.yaml 还没生成，无法回放。
  if [ -n "$BAG_PID" ]; then
    kill -INT -- "-$BAG_PID" 2>/dev/null || true
    echo "[等待] rosbag 正在写盘和压缩，可能需要几分钟..."
    for _ in $(seq 1 600); do
      if ! kill -0 "$BAG_PID" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    if kill -0 "$BAG_PID" 2>/dev/null; then
      echo "[警告] rosbag 仍未退出，发送 TERM。" >&2
      kill -TERM -- "-$BAG_PID" 2>/dev/null || true
      sleep 10
    fi
  fi

  # 先停监测，避免相机和 RTAB-Map 被关闭时误报 “VIO无数据”。
  if [ -n "$MON_PID" ]; then
    kill -INT -- "-$MON_PID" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-$MON_PID" 2>/dev/null || true
    wait "$MON_PID" 2>/dev/null || true
    MON_PID=""
  fi

  # rosbag 完成写盘后再停止相机和建图节点。
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    sleep 8
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 3
    fi
  fi
  if [ -n "$BAG_PID" ]; then kill -KILL -- "-$BAG_PID" 2>/dev/null || true; fi
  if [ -n "$LAUNCH_PID" ]; then kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true; fi
  wait "$BAG_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true

  if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
    echo "[恢复] metadata.yaml 未生成，尝试重建 rosbag 索引..."
    ros2 bag reindex "$BAG_PATH" >/dev/null 2>&1 || true
  fi
  if [ -f "$BAG_PATH/metadata.yaml" ]; then
    echo "[完成] rosbag：$BAG_PATH"
  else
    echo "[错误] rosbag 未生成 metadata.yaml：$BAG_PATH" >&2
  fi
}
trap cleanup INT TERM EXIT

if pgrep -f 'realsense2_camera_node|rtabmap_slam.*rtabmap|semantic_detector|semantic_mapper' >/dev/null 2>&1; then
  echo "错误：检测到已有相机/RTAB-Map/语义节点运行，请先停止现有流程。" >&2
  exit 1
fi

LAUNCH_ARGS=("--no-rviz")
[ "$FAST" = true ] && LAUNCH_ARGS+=("--fast")

echo "[启动] 实时 RGB-D 视觉里程计和语义建图"
setsid "$SHOWCASE" "${LAUNCH_ARGS[@]}" >"$BAG_DIR/${NAME}_live.log" 2>&1 &
LAUNCH_PID=$!

wait_topic() {
  local topic="$1" timeout_s="${2:-60}" start now
  start="$(date +%s)"
  while true; do
    now="$(date +%s)"
    [ $((now - start)) -ge "$timeout_s" ] && return 1
    ros2 topic list 2>/dev/null | grep -Fxq "$topic" && return 0
    sleep 1
  done
}

for topic in /camera/camera/color/image_raw \
             /camera/camera/aligned_depth_to_color/image_raw \
             /camera/camera/color/camera_info /odom /tf_static; do
  if ! wait_topic "$topic" 90; then
    echo "错误：等待话题 $topic 超时。" >&2
    exit 1
  fi
done

# 只录回放建图必需的数据，避免录制语义图像和大体积调试话题。
echo "[录制] $BAG_PATH"
setsid ros2 bag record \
  -o "$BAG_PATH" \
  --compression-mode file \
  --compression-format zstd \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/color/camera_info \
  /odom \
  /tf_static \
  /rtabmap/odom_info \
  >"$BAG_DIR/${NAME}_record.log" 2>&1 &
BAG_PID=$!

sleep 3
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "错误：rosbag 未能启动，请查看 $BAG_DIR/${NAME}_record.log" >&2
  exit 1
fi

# VIO 发散监测：输出直接留在本终端，方便边走边看状态。
# 日志另外留一份，便于录制结束后复查。
if [ "$MONITOR" = true ]; then
  if [ -f "$MONITOR_PY" ]; then
    setsid python3 -u "$MONITOR_PY" \
      --log "$BAG_DIR/${NAME}_vio_monitor.log" &
    MON_PID=$!
  else
    echo "[警告] 找不到 $MONITOR_PY，跳过 VIO 发散监测。" >&2
  fi
fi

cat <<EOF

============================================================
  rosbag 录制中
============================================================
  路径：$BAG_PATH
  日志：$BAG_DIR/${NAME}_record.log
  监测：$BAG_DIR/${NAME}_vio_monitor.log

  现在携带相机完成一圈，结束后在本终端按 Ctrl+C。
  监测会在视觉里程计持续失效时直接提示 “VIO发散”。
============================================================
EOF

wait "$BAG_PID"
