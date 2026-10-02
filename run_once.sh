#!/usr/bin/env bash
# ============================================================================
#  run_once.sh —— 一键跑一次「手持语义建图」
#
#  这是 launch_handheld_mapping.sh 的封装，只补三件它没做的事：
#
#   1) 起飞前把上一次残留的节点清干净
#      尤其是 ros2 launch 退出后**被孤立的子节点**：它们是用 python3 起的，
#      进程名就叫 python3，用 `ps -eo comm | grep rtabmap` 这类按进程名过滤的
#      清理方式**根本抓不到**。留着会继续抢 CPU/GPU、占着相机设备，
#      还会让话题上出现重复的 publisher，导致下一次跑的结果不可信。
#
#   2) 全程日志留档到 data/runs/
#      论文里要复现某一次实验、对比参数改动，靠的就是这个日志。
#      日志里同时含 RTAB-Map 的 Rate/delay 和语义层的统计，是实验数据的来源。
#
#   3) Ctrl+C 时先优雅关闭、再兜底清扫，最后打印本次结果摘要
#      优雅关闭是必须的：semantic_mapper 要把语义地图写盘，
#      RTAB-Map 要把数据库落盘。直接 kill -9 会丢数据。
#
#  用法：
#    ./run_once.sh                 默认：清空数据库与语义地图，从零开始 + 启动 RViz
#    ./run_once.sh --no-rviz       不启动 RViz（CPU 紧张时用，省约 28% CPU）
#    ./run_once.sh --record        同时录 rosbag（复现问题用）
#    ./run_once.sh --keep-db       保留旧数据库**和旧语义地图**（默认是清空重建）
#    ./run_once.sh --classes=chair,dining table,tv
#                                  只跟踪指定的 COCO 类别（默认全部 80 类）
#                                  建议正式实验时收窄，尤其要排除 person ——
#                                  实测走动的人只观测 4 次就会被确认成地标
# ============================================================================

set -uo pipefail
set +u   # ROS 的 setup.bash 里有未定义变量，开 set -u 会中途退出

VIO_DIR="$HOME/VIO"
LAUNCH_SH="$VIO_DIR/launch_handheld_mapping.sh"
SEMANTIC_MAP="$VIO_DIR/data/semantic_map.json"
DB_PATH="$HOME/.ros/rtabmap_handheld.db"
RUN_DIR="$VIO_DIR/data/runs"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$RUN_DIR/run_${STAMP}.log"

# ---------------- 参数解析 ----------------
USE_RVIZ=true
RECORD=false
FRESH_DB=true
CLASSES=""
for arg in "$@"; do
  case "$arg" in
    --no-rviz)   USE_RVIZ=false ;;
    --record)    RECORD=true ;;
    --keep-db)   FRESH_DB=false ;;
    --classes=*) CLASSES="${arg#--classes=}" ;;
    -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（用 --help 查看用法）"; exit 1 ;;
  esac
done

mkdir -p "$RUN_DIR"

# ---------------- 残留清扫 ----------------
# 匹配串特意写成"可执行文件的完整路径片段"，避免误杀无关进程。
# ⚠ rtabmap_util/ 这一类必须包含：实测有 point_cloud_xyzrgb 节点从更早的运行里
#   活了下来，躲过了以前所有按名字过滤的清理，一直在后台占 CPU/GPU。
NODE_PATTERN='realsense2_camera_node|rtabmap_slam/rtabmap|rtabmap_odom/|rtabmap_util/|lib/semantic_rtabmap/(mapper_node|detector_node)|ros2 launch|rviz2|foxglove_bridge|static_transform_publisher|robot_state_publisher'

find_stragglers() {
  ps -eo pid,args 2>/dev/null | grep -E "$NODE_PATTERN" | grep -v grep | awk '{print $1}'
}

sweep() {
  local pids p
  pids="$(find_stragglers)"
  [ -z "$pids" ] && return 0
  for p in $pids; do kill -TERM "$p" 2>/dev/null || true; done
  # 先给 TERM 一点时间（节点要关设备、写盘）
  for _ in $(seq 1 12); do
    [ -z "$(find_stragglers)" ] && return 0
    sleep 0.5
  done
  # 还赖着不走的才上 KILL
  pids="$(find_stragglers)"
  for p in $pids; do kill -KILL "$p" 2>/dev/null || true; done
  sleep 1
  return 0
}

# ---------------- 0. 起飞前体检 ----------------
echo "============================================================"
echo "  起飞前体检"
echo "============================================================"

FAIL=0

# 0.1 工作空间
if [ -f "$VIO_DIR/semantic_ws/install/setup.bash" ]; then
  echo "  [OK]   语义工作空间已编译"
else
  echo "  [错误] 语义工作空间未编译，请先执行："
  echo "           cd ~/VIO/semantic_ws && colcon build --symlink-install --parallel-workers 1"
  FAIL=1
fi

# 0.2 相机
if lsusb 2>/dev/null | grep -q "8086:0b07"; then
  echo "  [OK]   RealSense D435 已连接 ($(ls /dev/video* 2>/dev/null | wc -l) 个 video 设备)"
elif ls /dev/video* >/dev/null 2>&1; then
  echo "  [警告] 没在 lsusb 里看到 8086:0b07，但有 /dev/video* 设备，先继续"
else
  echo "  [错误] 没有检测到相机设备，请插好 D435"
  FAIL=1
fi

# 0.3 残留进程
STRAY="$(find_stragglers)"
if [ -n "$STRAY" ]; then
  echo "  [清理] 发现上次残留的节点，正在清扫："
  echo "$STRAY" | while read -r p; do
    ps -o pid=,args= -p "$p" 2>/dev/null | cut -c1-110 | sed 's/^/           /'
  done
  sweep
  echo "  [OK]   残留已清理"
else
  echo "  [OK]   无残留节点"
fi

# 0.4 资源
LOAD1="$(cut -d' ' -f1 /proc/loadavg)"
MEM_AVAIL="$(awk '/MemAvailable/{printf "%.0f", $2/1024}' /proc/meminfo)"
echo "  [信息] load average = $LOAD1 (6 核，超过 4 会拖累视觉里程计)"
echo "  [信息] 可用内存 = ${MEM_AVAIL} MB"
if [ -n "$CLASSES" ]; then
  echo "  [信息] 语义类别白名单: $CLASSES"
else
  echo "  [信息] 语义类别: 全部 80 类（含 person —— 走动的人会被当成地标）"
fi
# 这台机器是通过浏览器/图形远程桌面操作的，Chrome 和 NoMachine 常驻占约 255%
# CPU（整机 600%），但它们**不能关** —— 关了就没法操作这台机器了。
# 只能靠降低我们自己的开销来适应。

if [ "$FAIL" = 1 ]; then
  echo ""
  echo "  体检未通过，已中止。"
  exit 1
fi

echo ""
echo "  日志将写入: $LOG"
echo ""

# ---------------- 0.5 清空语义地图 ----------------
# ⚠ 这一步不能省：launch_handheld_mapping.sh 的 --fresh-db 只把 **RTAB-Map 数据库**
#   挪走，而 semantic_mapper 启动时还会自己加载 data/semantic_map.json。
#   如果只清数据库不清这个文件，上一轮的语义物体（连同它们的 instance_id）
#   会被直接加载进来，新一轮的"新建 0 个物体"看着像是检测失效，
#   实际是继承了旧轨迹 —— 结果不可信。
#   所以默认一起挪走；要接着上一轮继续建图就用 --keep-db。
if [ "$FRESH_DB" = true ] && [ -f "$SEMANTIC_MAP" ]; then
  mv "$SEMANTIC_MAP" "${SEMANTIC_MAP}.${STAMP}"
  echo "[信息] 旧语义地图已挪到 ${SEMANTIC_MAP}.${STAMP}"
  echo ""
fi

# ---------------- 1. 启动 ----------------
LAUNCH_ARGS=()
[ "$USE_RVIZ" = true ] || LAUNCH_ARGS+=(--no-rviz)
[ "$RECORD" = true ]  && LAUNCH_ARGS+=(--record)
[ "$FRESH_DB" = true ] && LAUNCH_ARGS+=(--fresh-db)
[ -n "$CLASSES" ]     && LAUNCH_ARGS+=("--classes=$CLASSES")

bash "$LAUNCH_SH" "${LAUNCH_ARGS[@]}" > "$LOG" 2>&1 &
LAUNCH_PID=$!

# ---------------- 2. 等待就绪 ----------------
echo "============================================================"
echo "  正在启动（相机预热 + TensorRT 加载引擎，约 20~40 秒）"
echo "============================================================"

wait_for_ready() {
  local deadline=$((SECONDS + 100))
  local ok_det=0 ok_odom=0
  while [ $SECONDS -lt $deadline ]; do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "  [错误] 启动进程已退出，请看日志：$LOG"
      return 1
    fi
    grep -aq "semantic_detector 就绪" "$LOG" 2>/dev/null && ok_det=1
    grep -aq "Odom: quality"          "$LOG" 2>/dev/null && ok_odom=1
    if [ $ok_det = 1 ] && [ $ok_odom = 1 ]; then
      return 0
    fi
    printf '  ... 视觉里程计 %s | 语义检测 %s\n' \
      "$([ $ok_odom = 1 ] && echo 就绪 || echo 等待中)" \
      "$([ $ok_det  = 1 ] && echo 就绪 || echo 等待中)"
    sleep 5
  done
  echo "  [警告] 等待超时，但流程可能仍在启动，继续并开始监控"
  return 0
}

# ---------------- 3. 结果摘要 ----------------
print_summary() {
  echo ""
  echo "============================================================"
  echo "  本次运行结果"
  echo "============================================================"

  # 3.1 语义物体
  # ⚠ 注意：semantic_map.json 里的 tracks 是**全部**轨迹，不是只有确认的
  #   （AssociationMap.to_dict() 保存的是 self.tracks.values()）。
  #   确认条件 = 观测数 >= min_observations 且 置信度 >= min_confidence。
  #   这里必须自己复现这个判定，否则会把只观测过 1 次的临时轨迹
  #   也报成"确认物体"，摘要就是错的。门限直接从 mapper.yaml 读，避免写死。
  if [ -f "$SEMANTIC_MAP" ]; then
    python3 - "$SEMANTIC_MAP" "$VIO_DIR/semantic_ws/src/semantic_rtabmap/config/mapper.yaml" <<'PY'
import json, sys

path, cfg_path = sys.argv[1], sys.argv[2]

min_obs, min_conf = 3, 0.3          # 与 mapper.yaml 默认值一致，读不到时兜底
try:
    import yaml
    cfg = yaml.safe_load(open(cfg_path))
    # 结构是 {节点名: {ros__parameters: {...}}}，逐层剥到参数字典
    for _ in range(3):
        if not isinstance(cfg, dict):
            break
        if "ros__parameters" in cfg:
            cfg = cfg["ros__parameters"]
            break
        if len(cfg) == 1:
            cfg = next(iter(cfg.values()))
        else:
            break
    if isinstance(cfg, dict):
        min_obs  = int(cfg.get("min_observations", min_obs))
        min_conf = float(cfg.get("min_confidence", min_conf))
except Exception:
    pass

try:
    d = json.load(open(path))
except Exception as e:
    print(f"  (语义地图读取失败: {e})")
    sys.exit(0)

st = d.get("stats", {})
tracks = d.get("tracks", [])

def is_confirmed(t):
    return (t.get("observation_count", 0) >= min_obs
            and t.get("confidence", 0.0) >= min_conf)

conf    = [t for t in tracks if is_confirmed(t)]
pending = [t for t in tracks if not is_confirmed(t)]

print(f"  已确认物体: {len(conf)} 个    (判定条件: 观测 >= {min_obs} 次 且 置信度 >= {min_conf})")
if conf:
    print(f"  {'类别':<16}{'实例':>4}  {'位置 (x, y, z) / m':<26}{'观测':>6}{'置信':>8}")
    for t in sorted(conf, key=lambda x: -x.get("observation_count", 0)):
        p = t.get("position", [0, 0, 0])
        pos = f"({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:+.2f})"
        print(f"  {t.get('class_name','?'):<16}{t.get('instance_id',0):>4}  "
              f"{pos:<26}{t.get('observation_count',0):>6}"
              f"{t.get('confidence',0):>8.2f}")
else:
    print("  （没有物体达到确认条件——要么没看到目标，要么每个都只瞥到一两帧）")

if pending:
    print()
    print(f"  观测中（尚未确认）: {len(pending)} 个  —— 只瞥到几帧的临时轨迹，不会进地图")
    for t in sorted(pending, key=lambda x: -x.get("observation_count", 0))[:8]:
        p = t.get("position", [0, 0, 0])
        pos = f"({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:+.2f})"
        print(f"    {t.get('class_name','?'):<16}{t.get('instance_id',0):>4}  {pos:<26}"
              f"{t.get('observation_count',0):>4} 次")
    if len(pending) > 8:
        print(f"    ... 另有 {len(pending) - 8} 个")

print()
print(f"  处理帧数 {st.get('frames',0)} | 检测 {st.get('detections',0)} "
      f"| 关联匹配 {st.get('matched',0)} | 新建 {st.get('created',0)} "
      f"| 清理 {st.get('pruned',0)}")
PY
  else
    echo "  (没有语义地图文件，语义层可能没起来)"
  fi

  # 3.2 回环
  LOOPS="$(grep -aci "loop closure detected" "$LOG" 2>/dev/null || echo 0)"
  echo ""
  echo "  回环检测: $LOOPS 次  $([ "$LOOPS" -gt 0 ] && echo '(闭环成功，建图质量的关键验证点)' || echo '(没触发回环——注意是否真的走回了起点、朝向是否接近)')"

  # 3.3 实时性
  local rl
  rl="$(grep -a "rtabmap (" "$LOG" 2>/dev/null | tail -1 | sed 's/.*\[rtabmap.rtabmap\]: //')"
  [ -n "$rl" ] && echo "  RTAB-Map: $rl"

  # 3.4 文件
  echo ""
  if [ -f "$DB_PATH" ]; then
    echo "  RTAB-Map 数据库: $DB_PATH ($(du -h "$DB_PATH" 2>/dev/null | cut -f1))"
  fi
  [ -f "$SEMANTIC_MAP" ] && echo "  语义地图 JSON : $SEMANTIC_MAP"
  echo "  本次日志      : $LOG"
  echo ""
  echo "  提示：RTAB-Map 数据库可以在下次运行时用 --keep-db 继续加载，"
  echo "        也可以在 rtabmapviz 里打开做可视化回顾。"
  echo "============================================================"
}

# ---------------- 4. 信号处理 ----------------
STOPPING=0
on_stop() {
  [ "$STOPPING" = 1 ] && return 0
  STOPPING=1
  echo ""
  echo "[信息] 收到停止信号，正在优雅关闭..."
  echo "[信息] 先让语义层把地图写盘、RTAB-Map 把数据库落盘（约 3~5 秒）..."
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 24); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.5
    done
  fi
  # 兜底：清扫一切还活着的节点（含被孤立的 python3 节点）
  sweep
  echo "[信息] 关闭完成。"
  print_summary
  exit 0
}
trap on_stop INT TERM
trap '[ "$STOPPING" = 1 ] || on_stop' EXIT

# ---------------- 5. 走动指引 ----------------
if ! wait_for_ready; then
  sweep
  exit 1
fi

cat <<'EOF'

============================================================
  一切就绪，可以开始走动了
============================================================

【场地上要有的大件物体】（yolo11n 认大件准，小物件又漏又抖）
    椅子 chair / 餐桌 dining table / 沙发 couch / 显示器 tv
    冰箱 refrigerator / 微波炉 microwave / 盆栽 potted plant

【走动要领】
  · 慢。比你觉得舒服的速度再慢一半。
  · 转要慢。原地快速转身是视觉里程计最常见的死法。
  · 画面里始终有纹理，避免长时间对着白墙。
  · 每个目标物体在画面里保持约 1 秒（检测跑 3.4 Hz，确认要 3 次观测）。
  · 物体保持在 1~3 米内，深度精度最好。
  · 走一个【闭环】绕回起点，朝向也接近 —— 这样才能触发回环检测。

【场地禁忌】
  · 别移动家具 —— 关联算法假设物体静止，移动会产生重复轨迹。
  · 避开镜子/玻璃 —— 深度会返回反射像，产生幻影物体。
  · 别让人在画面里穿行 —— person 也是 COCO 类，会被当成地标跟踪。

【实时观察】
  · 下面每 10 秒打一行状态：语义物体数 / 里程计质量 / 回环次数
  · RViz 里看建出的图有没有闭合、有没有明显错层

【结束】在本终端按 Ctrl+C，会自动关闭并打印本次结果摘要
============================================================

EOF

# ---------------- 6. 监控循环 ----------------
LAST_SEM=""
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  sleep 10
  [ "$STOPPING" = 1 ] && break
  kill -0 "$LAUNCH_PID" 2>/dev/null || break

  TS="$(date +%H:%M:%S)"
  SEM="$(grep -a "语义地图:" "$LOG" 2>/dev/null | tail -1 | sed 's/.*语义地图: //; s/ |.*//')"
  ODOM="$(grep -a "Odom: quality" "$LOG" 2>/dev/null | tail -1 \
          | sed 's/.*Odom: //; s/, std dev=[^,]*, update time=/ 更新/; s/delay=/ 延迟/')"
  LOOPS="$(grep -aci "loop closure detected" "$LOG" 2>/dev/null || echo 0)"

  [ -z "$SEM" ] && SEM="(尚未确认物体)"
  printf '[%s] 语义: %s\n' "$TS" "$SEM"
  [ -n "$ODOM" ] && printf '           VO: %s\n' "$ODOM"
  printf '           回环: %s 次\n' "$LOOPS"
done

# launch 进程自己退出了（比如崩溃）
if [ "$STOPPING" = 0 ]; then
  echo ""
  echo "[警告] 启动进程意外退出，正在清扫残留..."
  sweep
  echo "[信息] 请看日志末尾找原因：$LOG"
  tail -20 "$LOG" | sed 's/^/    /'
  print_summary
fi
