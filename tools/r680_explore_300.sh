#!/usr/bin/env bash
# 一键:整栈启动 -> 300 s 真实自主探索 -> 自动收尾(关闸门)。
#
#   bash tools/r680_explore_300.sh                 # 运动前要输 GO
#   bash tools/r680_explore_300.sh --yes           # 不打断(倒计时 5 s 后开始)
#   bash tools/r680_explore_300.sh --budget 600    # 换预算
#   bash tools/r680_explore_300.sh --rehearsal     # 只选点,不驱动底盘
#   bash tools/r680_explore_300.sh --no-wait       # 起了就走,不等待也不关闸门
#
# 为什么默认 300 s 而不是 120 s:
#   实测 120 s 那轮完成 3/3 目标、0 失败,但**结束时地图仍有 74.4% 未知、
#   还剩 3 个可达目标** —— 预算是"地图还大有可为时"用完的,不是走完了。
#
# 演示前必读(同一个 tag v0.2-exploration-demo-ready 的注释里也记着):
#   * cmd_vel_safety 的 stop_on_obstacle 一命中就把整个 Twist 置零,连倒车和
#     转向也一起清零。车头一旦进入 0.45 m 死角就**永久锁死**,自己出不来。
#     实测第二轮 66 s 卡死 -> 连续 5 次目标超时 -> ABORTED。
#     => 场地必须留开阔区(起步至少 2 m 净空);本脚本检测到锁死会高声提醒。
#   * 恢复办法:人工把车挪开(或短距离倒车),然后重跑本脚本。
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

while [ $# -gt 0 ]; do
    case "$1" in
        --yes|-y)    YES=1 ;;
        --rehearsal) REHEARSE=1 ;;
        --no-wait)   WAIT=0 ;;
        --budget)    shift; BUDGET="${1:-300.0}" ;;
        -h|--help)   sed -n '2,27p' "$0"; exit 0 ;;
        *) echo "未知参数: $1  (用法见 $0 --help)"; exit 2 ;;
    esac
    shift
done

echo "════════════════════════════════════════════════════════════"
echo " R680 一键自主探索      预算 ${BUDGET} s      限速 0.10 / 0.0 / 0.35"
echo "════════════════════════════════════════════════════════════"

# 让 up 脚本的横幅说出真实预算;EXPLORE_TIME 才是真正生效的那个。
export MAX_EXPLORE_TIME="$BUDGET"
export GOAL_TIMEOUT=60.0
export EXPLORE_TIME="$BUDGET"

# ---------- 1/3 整栈启动(此步会打开闸门,但探索器 dry_run=true autostart=false,车不会动)
echo
echo ">>> 1/3 整栈启动(约 3~4 分钟,车不会动)"
bash "$VIO/tools/r680_explore_up.sh" || { echo "❌ 整栈启动失败,中止"; exit 1; }

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
