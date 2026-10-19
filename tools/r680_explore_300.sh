#!/usr/bin/env bash
# 一键:整栈启动 -> 300 s 真实自主探索 -> 自动收尾(关闸门)。
#
#   bash tools/r680_explore_300.sh                 # 运动前要输 GO
#   bash tools/r680_explore_300.sh --yes           # 不打断(倒计时 5 s 后开始)
#   bash tools/r680_explore_300.sh --budget 600    # 换预算(整数会自动补成 600.0)
#   bash tools/r680_explore_300.sh --rehearsal     # 只选点,不驱动底盘
#   bash tools/r680_explore_300.sh --no-wait       # 起了就走,不等待也不关闸门
#   bash tools/r680_explore_300.sh --fast          # 栈已就绪时跳过整栈重建,直接开闸起跑
#   bash tools/go.sh                               # = --fast --yes 600,一句就动
#
# 为什么默认 300 s 而不是 120 s:
#   实测 120 s 那轮完成 3/3 目标、0 失败,但**结束时地图仍有 74.4% 未知、
#   还剩 3 个可达目标** —— 预算是"地图还大有可为时"用完的,不是走完了。
#
# 演示前必读(同一个 tag v0.2-exploration-demo-ready 的注释里也记着):
#   * 曾经的问题:cmd_vel_safety 的 stop_on_obstacle 一命中就把整个 Twist 置零,
#     连倒车和转向也一起清零。车头进入 0.45 m 死角就**永久锁死**,自己出不来。
#     实测第二轮 66 s 卡死 -> 连续 5 次目标超时 -> ABORTED,
#     当时的恢复办法是人工把车挪开。
#     **已修**:障碍锁存时只封平移(linear.x/y 恒为 0),允许以
#     obstacle_escape_angular_z(0.3 rad/s)为上限的原地旋转脱困 —— 旋转不产生
#     位移,不会把车带进未测量的区域,却是唯一能自己走出死角的动作。
#     真机验证(闸门关闭、由真实 /scan 触发真实锁存):
#       障碍+脱困开 -> linear.x=0.0 angular.z=0.3 reason=obstacle_stop_escape_rotation
#       障碍+脱困关 -> linear.x=0.0 angular.z=0.0 reason=obstacle_stop
#       障碍+急停   -> 全 0        reason=emergency_stop(急停优先于脱困)
#       前方空旷    -> linear.x=0.1 angular.z=0.35 reason=ok
#     残余风险:死角内旋转时,车体角点扫过的半径可能大于车头到障碍的距离。
#     所以脱困转速刻意压到 0.3 rad/s,且现场仍要有人、手能碰到断电开关。
#   * 反复失败的区域现在会被标注为"无法探索"并永久排除(本轮内),
#     发布在 /exploration/unexplorable 上;监视脚本会显示处数。
#     它与 blacklist 的区别:黑名单会过期(等会再试),这里不会(试过了,去不了)。
#   * 场地仍需留开阔区(起步至少 2 m 净空);本脚本检测到锁死会高声提醒。
#
# 本脚本只是编排层,不复制任何逻辑:调用 tools/r680_explore_up.sh 起栈、
# tools/r680_explore_start.sh 开动、tools/r680_gate.sh 收尾。
#
# 不要加 `set -u`(ROS 的 setup.bash 会踩未定义变量)。

VIO=/home/jetson/VIO
WS=$VIO/semantic_ws
BUDGET=300.0
YES=0
REHEARSE=0
WAIT=1
# --fast: the stack is already up (someone ran r680_explore_up.sh and it passed).
# Skipping the 3-4 min bring-up is what makes "type go and the car moves" true;
# the bring-up stays the default because a stale/partial stack is exactly what
# the up script's checks exist to catch.
FAST=0

while [ $# -gt 0 ]; do
    case "$1" in
        --yes|-y)    YES=1 ;;
        --rehearsal) REHEARSE=1 ;;
        --no-wait)   WAIT=0 ;;
        --fast)      FAST=1 ;;
        --budget)    shift; BUDGET="${1:-300.0}" ;;        -h|--help)   awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
        *) echo "未知参数: $1  (用法见 $0 --help)"; exit 2 ;;
    esac
    shift
done

TOOLS_DIR="$(cd "$(dirname "$0")" && pwd)"
[ -f "$TOOLS_DIR/r680_budget.sh" ] || {
    echo "缺少 $TOOLS_DIR/r680_budget.sh —— 预算无法规范化,拒绝启动"; exit 1; }
source "$TOOLS_DIR/r680_budget.sh"
command -v r680_float_arg >/dev/null || {
    echo "$TOOLS_DIR/r680_budget.sh 里没有 r680_float_arg,拒绝启动"; exit 1; }
# `--budget 600` 这种整数写法会被 DOUBLE 参数拒掉,而拒绝只有一行警告:预算
# 会静默停在节点默认的 120 s 上。规范成 600.0,并由 start 脚本回读校验。
# 见 tools/r680_budget.sh 与 test/test_r680_budget_arg.py。
BUDGET="$(r680_float_arg "$BUDGET")"
case "$BUDGET" in
    ''|*[!0-9.eE+-]*) echo "预算值非法: '${BUDGET}' (要秒数,例如 300 或 300.0)"; exit 1 ;;
esac

echo "════════════════════════════════════════════════════════════"
echo " R680 一键自主探索      预算 ${BUDGET} s      限速 0.10 / 0.0 / 0.35"
echo "════════════════════════════════════════════════════════════"

# 让 up 脚本的横幅说出真实预算;EXPLORE_TIME 才是真正生效的那个。
export MAX_EXPLORE_TIME="$BUDGET"
export GOAL_TIMEOUT=60.0
export EXPLORE_TIME="$BUDGET"

# ---------- 1/3 整栈(--fast 时只校验,不重建)
echo
if [ "$FAST" = "1" ]; then
    echo ">>> 1/3 跳过整栈重建(--fast):只校验现有栈 + 重新开闸"
    # 校验而不是假设:--fast 的价值在于省下 3~4 分钟,不在于跳过安全检查。
    # 缺任何一个必需节点就直接退出,绝不"大概能跑"就开始运动。
    MISSING=""
    for pat in rtabmap controller_server bt_navigator exploration_node ros2_relay_node; do
        # 方括号技巧:避免 pgrep 匹配到本脚本自己的命令行(踩过这个坑)。
        pgrep -f "[${pat:0:1}]${pat:1}" >/dev/null 2>&1 || MISSING="$MISSING $pat"
    done
    if [ -n "$MISSING" ]; then
        echo "  ❌ 栈不完整,缺少:$MISSING"
        echo "     要么先跑: bash tools/r680_explore_up.sh"
        echo "     要么去掉 --fast 让本脚本自己重建"
        exit 1
    fi
    echo "  ✅ 必需节点齐全"
    # 上一轮收尾会关闸门,所以 --fast 必须自己重新开闸并校验;
    # 直接跳过这步会让车"起了但一步不动"(闸门关着时 /cmd_vel 没有发布者)。
    echo ">>> 重新打开运动闸门"
    bash "$VIO/tools/r680_gate.sh" open 2>&1 | tail -5
    case "$(bash "$VIO/tools/r680_gate.sh" status 2>&1 | tail -3)" in
        *"GATE OPEN"*) echo "  ✅ 闸门已开" ;;
        *) echo "  ❌ 闸门未能打开,拒绝开始运动"; exit 1 ;;
    esac
else
    echo ">>> 1/3 整栈启动(约 3~4 分钟,车不会动)"
    bash "$VIO/tools/r680_explore_up.sh" || { echo "❌ 整栈启动失败,中止"; exit 1; }
fi

# ---------- 2/3 开始探索
echo
echo ">>> 2/3 开始探索"
if [ "$REHEARSE" = "1" ]; then
    echo "    (演练模式:dry_run,只选点不驱动底盘)"
    bash "$VIO/tools/r680_explore_start.sh" --dry-run || { echo "❌ 启动失败"; exit 1; }
else
    SARG=(--go)
    [ "$YES" = "1" ] && SARG+=(--yes)
    bash "$VIO/tools/r680_explore_start.sh" "${SARG[@]}" || { echo "❌ 启动失败"; exit 1; }
fi

if [ "$WAIT" = "0" ]; then
    echo
    echo "  --no-wait:不等待、不自动关闸门。别忘了收尾:"
    echo "    监视: bash tools/r680_explore_watch.sh"
    echo "    急停: bash tools/r680_estop.sh"
    echo "    关闸: bash tools/r680_gate.sh close"
    exit 0
fi

# ---------- 3/3 等待结束,再关闸门
echo
echo ">>> 3/3 等待探索结束(最多 ${BUDGET%.*} + 110 s)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

timeout $(( ${BUDGET%.*} + 130 )) python3 - "$BUDGET" <<'PY'
import sys, time, math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

budget = float(sys.argv[1])
rclpy.init()
n = Node('onekey_watch')
d = {}
n.create_subscription(String, '/exploration/state', lambda m: d.update(s=m.data), 10)
n.create_subscription(Bool, '/safety/obstacle_stop', lambda m: d.update(k=m.data), 10)
n.create_subscription(Odometry, '/odom', lambda m: d.update(o=m), 10)

t0 = time.time()
last = 0.0
warned = False
hist = []
while time.time() - t0 < budget + 110:
    rclpy.spin_once(n, timeout_sec=0.2)
    now = time.time()
    st = d.get('s')
    if st and st != 'EXPLORING':
        print('\n  探索结束:状态 = %s' % st)
        break
    o = d.get('o')
    if o:
        p = o.pose.pose.position
        hist.append((now, p.x, p.y))
    if now - last < 15.0:
        continue
    last = now
    pos = ('x=%+.3f y=%+.3f' % (p.x, p.y)) if o else '?'
    stop = d.get('k')
    print('  [%5.1fs] %-10s %s  stop=%s' % (now - t0, st or '?', pos, stop), flush=True)
    # 锁死判定:最近 20 s 位移 < 2 cm 且 obstacle_stop 为真
    if stop is True and len(hist) >= 2:
        recent = [h for h in hist if now - h[0] <= 20.0]
        if len(recent) >= 2:
            moved = max(math.hypot(h[1] - recent[0][1], h[2] - recent[0][2]) for h in recent)
            if moved < 0.02 and not warned:
                warned = True
                print('\n  ⚠️  检测到锁死:20 s 内位移仅 %.3f m,且 obstacle_stop=True' % moved)
                print('      原因:cmd_vel_safety 命中避障后把 Twist 全零,连倒车/转向也清零,')
                print('            车头进入 0.45 m 死角后自己出不来。')
                print('      处理:人工把车挪开(或短距离倒车),然后重跑本脚本。\n')
rclpy.shutdown()
PY

echo
echo ">>> 收尾:关闭运动闸门"
bash "$VIO/tools/r680_gate.sh" close 2>&1 | tail -4

echo
echo ">>> 最终状态"
cd "$WS" || exit 1
printf '  探索状态      = '; timeout 10 ros2 topic echo --once /exploration/state --field data 2>&1 | head -1
printf '  /cmd_vel_safe = '; timeout 10 ros2 topic echo --once /cmd_vel_safe --field linear.x 2>&1 | head -1
timeout 20 ros2 topic echo --once /exploration/status --field data 2>&1 | head -2 | sed 's/^/  /'
echo
echo "  日志: /tmp/r680_explore/{relay,cam,nav,rtab,explore}.log"
echo "  再跑一轮:直接重跑本脚本(它会自己清理上一次的整栈)"
echo
echo "  ⚠ 本脚本**只关闸门**,整栈仍在运行(Nav2 / RTAB-Map / 相机 / 中继)。"
echo "    车已经不可能动,但 rtabmap 还占着相机、还开着 .db —— 这是刻意的:"
echo "    留着现场才能看日志和地图。要退干净(含 rtabmap 存库)请跑:"
echo "        bash tools/r680_down.sh"
echo "    (它会先 TERM 等 20 s 让 rtabmap 优雅退出并存数据库,再清残留)"
