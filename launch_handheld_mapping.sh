#!/usr/bin/env bash
# ============================================================================
#  手持建图：视觉里程计 + RTAB-Map + 语义层
#
#  用途：不接飞控、手持 D435 走一圈，验证 SLAM 建图与语义建图的完整流程。
#        这是接无人机之前最划算的验证方式 —— 能拿到真实的运动、多视角观测
#        和回环，而不需要任何飞控硬件。
#
#  用法：
#    ./launch_handheld_mapping.sh              完整流程（SLAM + 语义）
#    ./launch_handheld_mapping.sh --slam-only  只跑 SLAM，先确认能建出图
#    ./launch_handheld_mapping.sh --no-rviz    不启动 RViz（省约 28% CPU）
#    ./launch_handheld_mapping.sh --fast       额外降低 VO 计算量（CPU 紧张时用）
#    ./launch_handheld_mapping.sh --record     同时录 bag（便于复现和回放调参）
#    ./launch_handheld_mapping.sh --fresh-db   把旧数据库挪走，从零开始建图
#
#  建议的验证顺序（很重要，别一步到位）：
#    第 1 步：--slam-only 走一圈，确认里程计不飘、能建出图、能触发回环
#    第 2 步：完整流程再走一圈，看语义物体有没有正确落到地图上
#    第 3 步：换参数重复，对比效果（论文实验章节的数据来源）
#
#  ============================================================================
#  性能：为什么默认这么配参数
#  ============================================================================
#  这台 Orin NX 的 CPU 非常紧张，而且紧张是结构性的：本体通过浏览器和图形
#  远程桌面操作，这些进程（Chrome / NoMachine / Xorg）实测常驻占用约 255%
#  的 CPU，而整机只有 600%。而且它们不能关 —— 关了就没法操作这台机器。
#
#  实测基线（什么都不跑时）load average 已经 5.0（6 核），留给我们的只有
#  约 350%。所以本脚本默认做了三件降低自身开销的事：
#
#   1. 深度流用 640x480 而不是 848x480
#      实测彩色帧率 21.8 Hz → 28.1 Hz。深度降分辨率后对齐的运算量小了，
#      相机节点不再被 CPU 饿着，反而让彩色流（VO 的输入）跑满帧。
#      代价是深度细节略降，对语义建图（我们用 20 分位鲁棒深度）影响很小。
#
#   2. 语义推理限速 5 Hz（process_rate）
#      相机 30 Hz 时检测线程会跑满，实测占 38% CPU。但物体是静止的，
#      RTAB-Map 自己的回环检测也只有 2 Hz，5 Hz 完全够用。
#
#   3. 可用 --no-rviz 省下 RViz 的约 28% CPU
#
#  【真正的解法是把视觉里程计搬到 GPU】，但那需要重编 RTAB-Map（以及
#  cv_bridge）来对齐 /usr/local 那套带 CUDA 的 OpenCV 4.10 —— 系统中
#  确实存在这套库，只是 ROS 用的是 apt 的无 CUDA 版 4.5.4。属于后续工作，
#  详见 ~/VIO/SEMANTIC_RTABMAP.md 的性能章节。
#
#  走动前自检：ros2 topic hz /camera/camera/color/image_raw
#              正常应接近 28~30 Hz
# ============================================================================
#  为什么必须用视觉里程计
#  ============================================================================
#  原来的 launch_d435_rtabmap.sh 用的是 /mavros/local_position/odom，
#  并且 visual_odometry:=false。没有飞控时没有里程计，相机在 TF 里就是
#  静止的，手持走动也建不出任何地图。
#
#  这里改用 rtabmap_odom/rgbd_odometry 做视觉里程计，它从 RGB-D 图像直接
#  估计相机运动，不依赖任何外部传感器。TF 链变成：
#
#      map ──(rtabmap 发布)──▶ odom ──(VO 发布)──▶ base_link ──(静态)──▶ camera_link
#
#  ⚠ 因此【绝对不能】再发布静态 map->base_link：
#     base_link 会同时有两个父节点（odom 和 map），TF 树直接报错。
#     所以下面调用语义 launch 时显式传 publish_static_map_tf:=false。
# ============================================================================

set -eo pipefail
# ROS 的 setup 脚本里有未定义变量，遇到 set -u 会中断
set +u

source /opt/ros/humble/setup.bash

if [ ! -f "$HOME/VIO/semantic_ws/install/setup.bash" ]; then
  echo "错误: 语义工作空间尚未编译。请先执行："
  echo "  cd ~/VIO/semantic_ws && colcon build --symlink-install --parallel-workers 2"
  exit 1
fi
source "$HOME/VIO/semantic_ws/install/setup.bash"

# ---------------- 参数解析 ----------------
SLAM_ONLY=false
RECORD=false
FRESH_DB=false
USE_RVIZ=true
FAST=false
# 语义类别白名单（逗号分隔的 COCO 类名）。空 = 不过滤，全部 80 类都收。
# 为什么要能过滤：person 也是 COCO 类，走动的人会被当成地标跟踪
# （实测：只观测 4 次就被"确认"成一个语义物体），产生会自己移动的假地标。
# 做正式实验时建议收窄到场地里确实存在的大件物体，让两次运行的输入完全一致。
CLASS_FILTER=""
for arg in "$@"; do
  case "$arg" in
    --slam-only) SLAM_ONLY=true ;;
    --record)    RECORD=true ;;
    --fresh-db)  FRESH_DB=true ;;
    --no-rviz)   USE_RVIZ=false ;;
    --fast)      FAST=true ;;
    --classes=*) CLASS_FILTER="${arg#--classes=}" ;;
    -h|--help)   sed -n '2,45p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（用 --help 查看用法）"; exit 1 ;;
  esac
done

DB_PATH="$HOME/.ros/rtabmap_handheld.db"
SEMANTIC_MAP="$HOME/VIO/data/semantic_map.json"
BAG_DIR="$HOME/VIO/data/bags"
# 语义推理限速：物体是静止的，5 Hz 足够，省下的 CPU 留给视觉里程计
SEMANTIC_RATE=5.0

if [ "$FRESH_DB" = true ] && [ -f "$DB_PATH" ]; then
  STAMP=$(date +%s)
  mv "$DB_PATH" "${DB_PATH}.${STAMP}"
  echo "[信息] 旧数据库已挪到 ${DB_PATH}.${STAMP}"
fi

PIDS=()
cleanup() {
  echo ""
  echo "[信息] 正在停止所有节点..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  # 给 semantic_mapper 留出写盘时间（它每 30 秒也会自动存一次）
  sleep 2
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  echo "[信息] 已停止。语义地图: $SEMANTIC_MAP"
}
trap cleanup EXIT INT TERM

# ---------------- 0. 起飞前体检 ----------------
echo "============================================================"
echo "  CPU 体检"
echo "============================================================"
echo "  当前 load average: $(cut -d' ' -f1 /proc/loadavg)   (6 核机器上超过 4 就会拖累视觉里程计)"
# 注意：这里不建议关掉 Chrome/NoMachine —— 本机是通过浏览器和图形远程桌面
# 操作的，关掉它们等于失去对这台机器的控制。它们占的 CPU 是操作本机的必要
# 成本，只能靠降低我们自己的开销来适应（见文件头的性能说明）。
for proc in chrome nxnode Xorg; do
  if pgrep -x "$proc" >/dev/null 2>&1; then
    echo "  · $proc 在运行（桌面/远程操作进程，属于必要开销，不用关）"
  fi
done
echo "  已知基线: 桌面环境常驻约 255% CPU / 整机 600%"
echo ""

# ---------------- 1. 相机 ----------------
# 深度用 640x480 与彩色一致，而不是 848x480：实测彩色帧率 21.8 → 28.1 Hz。
# 相机节点整体被 CPU 饿着，减小深度对齐的运算量能把彩色流（VO 的输入）
# 喂饱，这比调任何 RTAB-Map 参数都直接。
echo "[1/4] 启动 RealSense D435（彩色 640x480x30 + 深度 640x480x30 对齐）..."
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30 &
PIDS+=($!)

# ---------------- 2. 静态外参 TF ----------------
# 手持时 base_link 就是相机的参考点，这个外参没有物理意义，
# 但 RTAB-Map 需要一个 base_link 才能建立 map->odom->base_link 链。
# 数值沿用装机时的外参，便于和装上无人机后的结果对比。
echo "[2/4] 发布 base_link -> camera_link 静态外参..."
ros2 run tf2_ros static_transform_publisher \
  --x 0.045 --y -0.075 --z -0.005 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link &
PIDS+=($!)

# 等相机把 camera_info 和 TF 发出来，否则 RTAB-Map 启动时会刷一堆警告
echo "      等待相机就绪..."
sleep 6

# ---------------- 3. RTAB-Map（视觉里程计 + 建图）----------------
# 注意两处容易踩的坑：
#
# ① vo_frame_id：launch 文件里它的默认值取自 odom_topic。如果只写
#    odom_topic:=/odom，vo_frame_id 会变成 "/odom" —— 带前导斜杠的不是
#    合法 TF frame 名，必须显式写成 odom。
#
# ② args 的格式：RTAB-Map 参数要写成 "--Param/Name value"，
#    写成 "--Param/Name=value" 不会生效。
#
# VO 的 CPU 开销可以通过 odom_args 单独调整：
#    Vis/MaxFeatures      特征点上限（默认 1000）
#    Odom/ImageDecimation 图像降采样倍数（默认 1；设 2 = 用 320x240 做配准）
# --fast 会同时收紧这两项，CPU 紧张时用，代价是精度略降。
if [ "$FAST" = true ]; then
  ODOM_ARGS="--Vis/MaxFeatures 500 --Odom/ImageDecimation 2"
  echo "      (--fast: VO 使用 500 特征点 + 2 倍降采样)"
else
  ODOM_ARGS="--Vis/MaxFeatures 800"
fi

if [ "$USE_RVIZ" = true ]; then
  RVIZ_ARGS=(rviz:=true "rviz_cfg:=$HOME/VIO/rtabmap_d435.rviz")
  echo "[3/4] 启动 RTAB-Map（视觉里程计模式 + RViz）..."
else
  RVIZ_ARGS=(rviz:=false)
  echo "[3/4] 启动 RTAB-Map（视觉里程计模式，无 RViz）..."
fi

# landmark_linear/angular_variance 由 rtabmap.launch.py 的 tag_linear_variance /
# tag_angular_variance 两个参数传入，作用于我们发布到
# /rtabmap/landmark_detections 与 /landmark_detections 的语义地标：
#   - tag_linear_variance 默认 0.0001 (σ=1cm)，那是给 AprilTag 的精度；YOLO 检测框
#     中心 + 深度反投影出来的语义地标误差远大于 1cm，沿用默认会把地标当成硬约束、
#     把整张地图拉歪。改成 0.04 (σ=20cm)，让地标作为软约束参与图优化。
#   - tag_angular_variance 保持默认 9999.0，即优化时忽略地标朝向（YOLO 估的 yaw 不可靠）。
ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=true \
  tag_linear_variance:=0.04 \
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
  "database_path:=$DB_PATH" &
PIDS+=($!)

sleep 4

# ---------------- 4. 语义层 ----------------
if [ "$SLAM_ONLY" = true ]; then
  echo "[4/4] 跳过语义层（--slam-only）"
else
  echo "[4/4] 启动语义检测 + 语义地图（推理限速 5 Hz）..."
  # 只有当用户显式指定了 --classes 时才向 launch 传 class_filter。
  # 之前无论是否指定都写 "class_filter:=$CLASS_FILTER"，导致空字符串
  # 时 launch 报错 "malformed launch argument 'class_filter:='"，语义层直接退出。
  if [ -n "$CLASS_FILTER" ]; then
    ros2 launch semantic_rtabmap semantic.launch.py \
      publish_static_map_tf:=false \
      publish_static_camera_tf:=false \
      target_frame:=map \
      process_rate:=$SEMANTIC_RATE \
      "class_filter:=$CLASS_FILTER" \
      "persistence_path:=$SEMANTIC_MAP" &
  else
    ros2 launch semantic_rtabmap semantic.launch.py \
      publish_static_map_tf:=false \
      publish_static_camera_tf:=false \
      target_frame:=map \
      process_rate:=$SEMANTIC_RATE \
      "persistence_path:=$SEMANTIC_MAP" &
  fi
  PIDS+=($!)
fi

# ---------------- 可选：录包 ----------------
if [ "$RECORD" = true ]; then
  mkdir -p "$BAG_DIR"
  BAG_NAME="handheld_$(date +%Y%m%d_%H%M%S)"
  echo "[附加] 录制 rosbag 到 $BAG_DIR/$BAG_NAME（zstd 压缩）..."
  ros2 bag record \
    -o "$BAG_DIR/$BAG_NAME" \
    --compression-mode file \
    --compression-format zstd \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/color/camera_info \
    /tf /tf_static /odom \
    /semantics/objects \
    /semantics/objects_tracked &
  PIDS+=($!)
fi

cat <<'EOF'

============================================================
  手持建图已启动
============================================================

【走动要领】
  · 慢。比你觉得舒服的速度再慢一半。视觉里程计靠帧间特征匹配，
    走快了特征移出画面就丢失跟踪。
  · 转要慢。原地快速转身是 VO 最常见的死法。
  · 画面里始终有纹理。避免长时间对着白墙、地板、天空 ——
    这些地方提不出特征点，VO 会失效。
  · 光照稳定。别对着窗户或灯光，自动曝光突变会打乱特征匹配。
  · 走一个【闭环】：绕一圈回到起点附近，并且朝向也和出发时接近。
    这样才能触发回环检测，是建图质量的关键验证点。

【同时可以做的验证】
  · 让同一个物体在不同角度、不同距离反复进入画面 ——
    这是检验语义数据关联会不会"过度分裂"的最好方式。
  · 纸箱、瓶子、椅子、背包这类 COCO 类别里的日常物品效果最好。

【实时观察】
  · RViz 里看建出的图有没有闭合、有没有明显错层
  · 终端里看有没有 "Loop closure detected"（触发回环）

【走完后检查】另开一个终端执行：
  source ~/VIO/semantic_ws/install/setup.bash

  # 1) 里程计是否流畅（正常应接近相机帧率）
  ros2 topic hz /odom

  # 2) 是否触发过回环
  ros2 topic echo /rtabmap/info --field loop_closure_id

  # 3) 语义地图里有什么
  ros2 topic echo --once /semantics/objects_tracked

  # 4) 二维占据栅格（后续 Nav2 用这个）
  ros2 topic echo --once /rtabmap/map --field info

  # 5) 语义地图文件（每 30 秒自动存一次，Ctrl+C 退出时也会存）
  cat ~/VIO/data/semantic_map.json

【退出】在本终端按 Ctrl+C
============================================================
EOF

wait
