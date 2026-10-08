#!/usr/bin/env bash
# ============================================================================
#  launch_mapping_only.sh —— 纯建图，不含任何语义层
#
#  用途
#  ----
#  只跑「相机 + 里程计 + RTAB-Map + 可视化」，不启动 semantic_detector /
#  semantic_mapper。用来回答一个具体问题：
#
#      地图建得不好，是 SLAM 的问题，还是语义层拖累的？
#
#  语义层开着的时候，YOLO 推理、深度反投影、点云发布都在抢同一块 CPU。
#  这个脚本把语义层整个拿掉，留下的是建图链路的「干净基线」。
#
#  ============================================================================
#  必须说清楚的一件事：建图无法脱离里程计
#  ============================================================================
#  RTAB-Map 不是一个「不需要里程计」的 SLAM —— 它必须有一个里程计来源，
#  用来给出相邻两帧之间的运动初值。所以：
#
#      「无论 VIO 是否发散，都能建出正确的图」这件事做不到。
#
#  里程计一旦发散，地图就跟着发散，这是原理限制，不是参数问题。回环检测
#  能把「有界的累积漂移」拉回来，但救不了一条已经发散的轨迹。
#
#  能做的是换一个【失效模式不同】的里程计来源，让它在 VIO 失效的场合仍可用：
#
#    --odom=vo   （默认）RGB-D 视觉里程计 rgbd_odometry
#                靠图像特征点。纹理丰富、光照稳定时精度最好；
#                ⚠ 白墙、暗光、烟雾、运动模糊下特征不足 → 容易发散。
#
#    --odom=icp  深度点云配准 icp_odometry
#                靠深度点云的几何结构，**不用图像特征**。
#                在无纹理/暗光环境下比 VO 稳，正好对应消防救援场景；
#                ⚠ 但走廊、空旷大厅这类几何退化（平面/圆柱）场合也会失效，
#                  且需要深度质量好、需要相机运动有足够几何变化。
#
#  两种里程计失效的场合不一样，这才是「多一层保险」的真实含义。
#  它们不是「随便哪个发散都没关系」。
#
#  ============================================================================
#  用法
#  ============================================================================
#    ./launch_mapping_only.sh                  默认：视觉里程计 + RViz
#    ./launch_mapping_only.sh --odom=icp       改用深度 ICP 里程计
#    ./launch_mapping_only.sh --no-rviz        不启动 RViz（配合 Foxglove 用）
#    ./launch_mapping_only.sh --fast           CPU 紧张时降低 VO 计算量
#    ./launch_mapping_only.sh --keep-db        接着上次的数据库继续建
#    ./launch_mapping_only.sh --help
#
#  退出：在本终端按 Ctrl+C。
# ============================================================================

set -Ee
set +u
# 启用作业控制：每个后台任务拿到**独立的进程组**，且 $! 就是该组的领头 PID。
#
# 为什么不用 setsid（实测踩过的坑，别改回去）：
#   setsid 在当前进程已经是会话/进程组领头时会 fork，于是 $! 拿到的是那个
#   【立刻退出】的中间进程 PID。后续 kill -- -PID 全部打空，节点全部残留 ——
#   实测现象：脚本退出后 realsense2_camera_node / rgbd_odometry / rtabmap
#   三个进程还活着，第二次启动就会和它们抢相机。
#   set -m 没有这个问题：实测 $! 与 PGID 相等，kill -- -$! 返回 0。
set -m

VIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEMANTIC_WS="$VIO_DIR/semantic_ws"
RVIZ_CONFIG="$VIO_DIR/rtabmap_d435.rviz"

ODOM_MODE="vo"
USE_RVIZ=true
KEEP_DB=false
FAST=false

for arg in "$@"; do
  case "$arg" in
    --odom=vo)   ODOM_MODE="vo" ;;
    --odom=icp)  ODOM_MODE="icp" ;;
    --no-rviz)   USE_RVIZ=false ;;
    --keep-db)   KEEP_DB=true ;;
    --fast)      FAST=true ;;
    -h|--help)   sed -n '2,58p' "$0"; exit 0 ;;
    *) echo "错误：未知参数 $arg（使用 --help 查看用法）" >&2; exit 2 ;;
  esac
done

case "$ODOM_MODE" in
  vo)  DB_PATH="$HOME/.ros/rtabmap_mapping_vo.db";  LOG_TAG="vo" ;;
  icp) DB_PATH="$HOME/.ros/rtabmap_mapping_icp.db"; LOG_TAG="icp" ;;
esac
LOG_DIR="$VIO_DIR/data/mapping_logs"

source /opt/ros/humble/setup.bash
if [ -f "$SEMANTIC_WS/install/setup.bash" ]; then
  source "$SEMANTIC_WS/install/setup.bash"
fi

# ------------------------- 前置检查 -------------------------
if ! lsusb 2>/dev/null | grep -qi "Intel.*0b07\|RealSense"; then
  echo "错误：没有检测到 Intel RealSense D435。" >&2
  exit 1
fi

# 只匹配真实节点的可执行文件路径，并**排除自身及其祖先进程**。
#
# 为什么必须排除祖先进程（实测踩过的坑，别删）：
#   pgrep -f 匹配的是「命令行里含有该字符串的任何进程」，不限于节点本身。
#   结果调用本脚本的包装脚本/测试脚本，只要命令行里出现过这些名字，
#   就会被当成"已有节点在运行"，脚本直接拒绝启动 —— 纯误报。
#   实测现象：把本脚本包在一个测试 harness 里跑，harness 自己把脚本挡掉了。
NODE_PATTERN='realsense2_camera_node|rtabmap_slam/rtabmap|rtabmap_odom/|lib/semantic_rtabmap/'

_self_and_ancestors() {
  local p=$$
  while [ "${p:-0}" -gt 1 ]; do
    echo "$p"
    p="$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null)" || break
    case "$p" in ''|*[!0-9]*) break ;; esac
  done
}

running_nodes() {
  local skip; skip=" $(_self_and_ancestors | tr '\n' ' ') "
  local pid
  for pid in $(pgrep -f "$NODE_PATTERN" 2>/dev/null || true); do
    case "$skip" in *" $pid "*) continue ;; esac
    tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | cut -c1-110
    echo
  done
}

if [ -n "$(running_nodes)" ]; then
  echo "错误：检测到已有建图/相机/语义节点在运行，请先停止。" >&2
  running_nodes >&2
  exit 1
fi

mkdir -p "$LOG_DIR" "$HOME/.ros"

if [ "$KEEP_DB" = false ] && [ -f "$DB_PATH" ]; then
  STAMP="$(date +%Y%m%d_%H%M%S)"
  mv "$DB_PATH" "${DB_PATH}.bak.${STAMP}"
  echo "[准备] 旧数据库已备份为 ${DB_PATH}.bak.${STAMP}"
fi

GROUP_PIDS=()
CLEANED=false

start_group() {
  local name="$1"; shift
  local log="$LOG_DIR/${LOG_TAG}_${name}_$(date +%Y%m%d_%H%M%S).log"
  echo "[启动] $name   日志：$log"
  # 不加 setsid：靠上面的 set -m 让这里成为独立进程组，$! 即 PGID。
  "$@" >"$log" 2>&1 &
  local pid=$!
  GROUP_PIDS+=("$pid")
  echo "$pid" >"$LOG_DIR/${LOG_TAG}_${name}.pid"
}

stop_groups() {
  [ "$CLEANED" = true ] && return
  CLEANED=true
  echo ""
  echo "[退出] 正在停止建图流程..."
  # 先 SIGINT 让 ROS 节点自己收尾（RTAB-Map 要把数据库落盘）
  for pid in "${GROUP_PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 5
  for pid in "${GROUP_PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${GROUP_PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done

  # 兜底清扫：按进程组再扫一遍，捞掉任何没被上面带走的孤儿。
  # ros2 launch 的子节点偶尔会换组活下来；不清掉的话下次启动会抢相机。
  local pg
  for pg in $(ps -eo pgid=,args= 2>/dev/null | grep -E "$NODE_PATTERN" | grep -v grep | awk '{print $1}' | sort -u); do
    kill -TERM -- "-$pg" 2>/dev/null || true
  done
  sleep 2
  for pg in $(ps -eo pgid=,args= 2>/dev/null | grep -E "$NODE_PATTERN" | grep -v grep | awk '{print $1}' | sort -u); do
    kill -KILL -- "-$pg" 2>/dev/null || true
  done

  for pid in "${GROUP_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done

  if [ -n "$(running_nodes 2>/dev/null)" ]; then
    echo "[警告] 仍有节点未退出：" >&2
    running_nodes >&2
    echo "        请手动执行： pkill -KILL -f 'realsense2_camera_node|rtabmap_slam/rtabmap|rtabmap_odom/'" >&2
  else
    echo "[退出] 已清理干净。数据库：$DB_PATH"
  fi
}

on_signal() { stop_groups; exit 0; }
# ⚠ 信号注意事项（实测）：
#   · 前台运行 + Ctrl+C → SIGINT，trap 正常生效。
#   · 用 `&` 放到后台运行 → POSIX 规定后台进程在【入口处就把 SIGINT 设为忽略】，
#     而 bash 无法为"入口即被忽略"的信号设置 trap，所以此时 Ctrl+C 式的 INT
#     清理【不会触发】。这种用法请改发 SIGTERM（后台进程不忽略 TERM）：
#         kill -TERM <脚本PID>
#   · 无论走哪条路径，EXIT trap 都会兜底，脚本退出时必定执行 stop_groups。
trap on_signal INT TERM
trap stop_groups EXIT

wait_node() {
  local node="$1" timeout_s="${2:-45}" start="$(date +%s)"
  while [ $(( $(date +%s) - start )) -lt "$timeout_s" ]; do
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
      echo "[就绪] 节点 $node"; return 0
    fi
    sleep 1
  done
  echo "错误：等待节点 $node 超时。见 $LOG_DIR" >&2; return 1
}

wait_topic() {
  local topic="$1" timeout_s="${2:-45}" start="$(date +%s)"
  while [ $(( $(date +%s) - start )) -lt "$timeout_s" ]; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "[就绪] 话题 $topic"; return 0
    fi
    sleep 1
  done
  echo "错误：等待话题 $topic 超时。见 $LOG_DIR" >&2; return 1
}

# ------------------------- 1. 相机 -------------------------
# ⚠ 不要加 pointcloud.enable:=true。
#   实测本机 realsense 驱动【根本不发布点云话题】：传标准的
#   pointcloud.enable:=true 没有任何效果，日志里连 pointcloud 字样都没有；
#   该节点的点云滤波参数实际叫 pointcloud__neon_.enable（ros2 param list 可见），
#   但设了也不出 /camera/camera/depth/color/points。
#   所以 ICP 模式改为在下面用 rtabmap_util/point_cloud_xyzrgb 自己合成点云。
start_group camera ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30
wait_node /camera/camera 45
wait_topic /camera/camera/color/image_raw 30
wait_topic /camera/camera/aligned_depth_to_color/image_raw 30
wait_topic /camera/camera/color/camera_info 30

# ------------------------- 2. 相机外参 -------------------------
start_group camera_tf ros2 run tf2_ros static_transform_publisher \
  --x 0.045 --y -0.075 --z -0.005 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
sleep 2

# ------------------------- 3. 里程计 + RTAB-Map + RViz -------------------------
if [ "$USE_RVIZ" = true ]; then
  RVIZ_ARGS=(rviz:=true "rviz_cfg:=$RVIZ_CONFIG")
else
  RVIZ_ARGS=(rviz:=false)
fi

if [ "$FAST" = true ]; then
  ODOM_ARGS="--Vis/MaxFeatures 500 --Odom/ImageDecimation 2"
else
  ODOM_ARGS="--Vis/MaxFeatures 800"
fi

if [ "$ODOM_MODE" = "vo" ]; then
  # 视觉里程计：rtabmap.launch.py 内部直接起 rgbd_odometry
  start_group rtabmap ros2 launch rtabmap_launch rtabmap.launch.py \
    visual_odometry:=true \
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
else
  # 深度 ICP 里程计。
  #
  # ⚠ 实测要点（踩过的坑，别改回去）：
  #   1) 本机相机【不发布点云话题】（见上面相机段的说明），所以这里用
  #      rtabmap_util/point_cloud_xyzrgb 自己从 RGB-D 合成点云 /cloud，
  #      完全不依赖相机点云输出。
  #   2) 合成点云实测只有约 2.4 Hz（640x480 在 Orin NX 上很贵），
  #      比视觉里程计低不少，这是 ICP 模式的固有代价。
  #   3) ICP 确实在跑、确实在做配准 —— 实测日志里能看到
  #      "IcpOdometry: Transferring value ... Icp/VoxelSize" 以及
  #      "Registration failed: ICP correction too large"。
  #      ⚠ 后者在【相机静止不动】时反而容易出现：噪声使配准修正量超限。
  #        走动起来才会正常输出。所以静置测试看不到 /icp_odom 是正常的，
  #        不代表 ICP 坏了。
  #   4) libpointmatcher 在本机是【静态链接】进 librtabmap_odom_plugins.so 的，
  #      ldd 看不到它 —— 不能据此判定 ICP 不可用。
  #
  # ⚠ 未做精度验证：ICP 模式只验证了「能跑起来并在做配准」，
  #   没有和 VO 做过 ATE/RPE 对照。正式实验前不要假定它比 VO 准。

  start_group pointcloud ros2 run rtabmap_util point_cloud_xyzrgb --ros-args \
    -p frame_id:=camera_color_optical_frame \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r cloud:=/cloud
  wait_topic /cloud 60

  start_group icp_odometry ros2 run rtabmap_odom icp_odometry --ros-args \
    -p subscribe_scan_cloud:=true \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p wait_for_transform:=0.2 \
    -p scan_voxel_size:=0.05 \
    -p scan_normal_k:=5 \
    -p "Icp/MaxTranslation:=0.5" \
    -p "Icp/MaxRotation:=1.0" \
    -r scan_cloud:=/cloud \
    -r odom:=/icp_odom
  # ⚠ 实测：Icp/MaxTranslation 的默认值是 0.2 m，在 2.4 Hz 稀疏点云下
  #   【偏紧到不可用】。实测日志反复出现：
  #     "libpointmatcher has failed: limit out of bounds: tr: 0.205918/0.2"
  #     "Cannot compute transform (ICP correction too large -> 0.217790 m ...)"
  #   即每帧需要约 0.206 m 修正、限幅 0.2 m，只差 3% 就被判为修正量过大，
  #   于是永久卡死：ICP 输出协方差全 9999 的【失效里程计】→ RTAB-Map 报
  #     "no odometry is provided. Image 0 is ignored!" → 建不出任何地图。
  #   放宽到 0.5 m 后留出余量。JSON 输入的 scan 帧率低，这一步是必要的。
  #
  # ⚠ 但放宽限幅只是【解除卡死】，不等于 ICP 精度够用。仍未与 VO 做
  #   ATE/RPE 对照。做正式展示请优先用默认的 --odom=vo（已实测可建出图）。
  wait_node /icp_odometry 45
  wait_topic /icp_odom 60

  start_group rtabmap ros2 launch rtabmap_launch rtabmap.launch.py \
    visual_odometry:=false \
    icp_odometry:=false \
    odom_topic:=/icp_odom \
    publish_tf_odom:=false \
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
    "args:=--Rtabmap/DetectionRate 2.0 --Grid/CellSize 0.05" \
    "database_path:=$DB_PATH"
fi

wait_node /rtabmap/rtabmap 60
wait_topic /rtabmap/cloud_map 90

cat <<EOF

============================================================
  纯建图已启动（无语义层）
============================================================
  里程计来源：$ODOM_MODE
  数据库：$DB_PATH
  日志目录：$LOG_DIR
  RViz：$USE_RVIZ

  注意：RTAB-Map 必须有里程计。里程计发散，地图就会发散。
        vo  靠图像特征 —— 纹理丰富时最好，白墙/暗光/烟雾下易失效
        icp 靠深度几何 —— 无纹理时更稳，几何退化场合也会失效

  建议观察：
    ros2 topic hz /odom            （vo 模式）
    ros2 topic hz /icp_odom        （icp 模式）
    ros2 topic hz /rtabmap/cloud_map

  退出：在本终端按 Ctrl+C
============================================================
EOF

wait
