#!/usr/bin/env bash
# 起自主探索。这是**唯一的"开始运动"动作**,所以要显式确认。
#
#   bash tools/r680_explore_start.sh --dry-run     # 只选点,不驱动底盘
#   bash tools/r680_explore_start.sh --go          # 真运动(需要 --go 明说)
#
# 为什么默认不是 --go:自主探索是这整套系统里唯一会**自己决定去哪**的模式。
# E1/E2 的速度是人给的,出问题人知道;探索器出问题时,车会自己一直走下去。
# 所以这里要求一个字面量参数,而不是"直接跑"。
#
# 停顿条件(任一触发即停):
#   - /battery_voltage 陈旧超过 battery_timeout,或低于 min_battery
#   - /safety/obstacle_stop 变 true
#   - /safety/localization_valid 变 false
#   - 超过 max_exploration_time
#   - bash tools/r680_estop.sh
#
# 不要加 `set -u`。

WS=/home/jetson/VIO/semantic_ws
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

# 预算默认 120 s(实测跑通的那一档),可由环境变量覆盖。
# 300 s 的一键用法见 tools/r680_explore_300.sh —— 它 export 这个变量再调本脚本。
# 注意:launch 文件并不声明 max_exploration_time,所以预算**只能**这样设,
# 在 r680_explore_up.sh 里改那个同名变量是没有用的(它只用于打印横幅)。
EXPLORE_TIME="${EXPLORE_TIME:-120.0}"

MODE=""
ASSUME_YES=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) MODE=dry ;;
        --go)      MODE=go ;;
        --yes|-y)  ASSUME_YES=1 ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "未知参数: $arg"; echo "用法: $0 --dry-run | --go [--yes]"; exit 2 ;;
    esac
done
[ -n "$MODE" ] || { echo "用法: $0 --dry-run | --go [--yes]"; exit 2; }

echo "=== 起探索前的前置检查 ==="
LV=$(timeout 10 ros2 topic echo --once /safety/localization_valid 2>&1 | grep -o 'true\|false' | head -1)
OS=$(timeout 10 ros2 topic echo --once /safety/obstacle_stop 2>&1 | grep -o 'true\|false' | head -1)
BV=$(timeout 10 ros2 topic echo --once /battery_voltage 2>&1 | grep data | head -1)
echo "  localization_valid: $LV  (需 true)"
echo "  obstacle_stop:      $OS  (需 false)"
echo "  battery_voltage:    $BV"

if [ "$MODE" = "go" ]; then
    if [ "$LV" != "true" ] || [ "$OS" != "false" ]; then
        echo "  ❌ 前置条件不满足,拒绝开始运动"
        exit 1
    fi
    echo
    echo "  ⚠️  即将开始**真实自主运动**"
    echo "      限速 0.10 / 0.0 / 0.35,预算 ${EXPLORE_TIME} s"
    echo "      只有满足以下全部条件才继续:"
    echo "        - 现场有人,手能碰到物理断电开关"
    echo "        - 前方至少 2 m 净空"
    echo "        - 另一个终端已经开着: bash tools/r680_explore_watch.sh"
    echo "      急停随时可用: bash tools/r680_estop.sh"
    echo
    if [ "$ASSUME_YES" = "1" ]; then
        echo "  --yes 已给出,跳过人工确认(5 s 后开始,可 Ctrl-C 中止)"
        sleep 5
    else
        read -r -p "  确认继续?输入 GO 开始: " ans
        if [ "$ans" != "GO" ]; then echo "  已取消(车未动)"; exit 0; fi
    fi
fi

echo
echo "=== 设置探索参数 ==="
if [ "$MODE" = "go" ]; then
    timeout 15 ros2 param set /r680_explorer dry_run false 2>&1 | tail -1
else
    timeout 15 ros2 param set /r680_explorer dry_run true 2>&1 | tail -1
fi
timeout 15 ros2 param set /r680_explorer max_exploration_time "$EXPLORE_TIME" 2>&1 | tail -1
timeout 15 ros2 param set /r680_explorer goal_timeout 60.0 2>&1 | tail -1
timeout 15 ros2 param set /r680_explorer min_battery 10.5 2>&1 | tail -1
timeout 15 ros2 param set /r680_explorer battery_timeout 5.0 2>&1 | tail -1
echo "  生效值:"
for p in dry_run max_exploration_time goal_timeout min_battery battery_timeout; do
    printf '    %-22s ' "$p"; timeout 8 ros2 param get /r680_explorer "$p" 2>&1 | tail -1
done

echo
echo "=== 启动探索 ==="
timeout 20 ros2 service call /exploration/start std_srvs/srv/Trigger 2>&1 | tail -3
echo
echo "  现在开始监视: bash tools/r680_explore_watch.sh"
echo "  急停:         bash tools/r680_estop.sh"
