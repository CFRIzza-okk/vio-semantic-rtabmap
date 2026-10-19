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
# 必须在 `cd` **之前**算出本脚本所在的绝对目录:cd 之后 $(dirname "$0") 这个
# 相对路径会被解释成新工作目录下的路径,于是 source 静默失败(本脚本没有 set -e),
# 依赖的函数变成空,预算变成空字符串 —— 而 `case "$APPLIED" in *"$EMPTY"*)`
# 是**任何字符串都匹配**,校验会把自己一起绕过。实测就是这么漏的。
TOOLS_DIR="$(cd "$(dirname "$0")" && pwd)"
[ -f "$TOOLS_DIR/r680_budget.sh" ] || {
    echo "缺少 $TOOLS_DIR/r680_budget.sh —— 预算无法规范化,拒绝启动"; exit 1; }
source "$TOOLS_DIR/r680_budget.sh"
command -v r680_float_arg >/dev/null || {
    echo "$TOOLS_DIR/r680_budget.sh 里没有 r680_float_arg,拒绝启动"; exit 1; }
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

# 预算默认 120 s(实测跑通的那一档),可由环境变量覆盖。
# 300 s 的一键用法见 tools/r680_explore_300.sh —— 它 export 这个变量再调本脚本。
# 注意:launch 文件并不声明 max_exploration_time,所以预算**只能**这样设,
# 在 r680_explore_up.sh 里改那个同名变量是没有用的(它只用于打印横幅)。
source "$TOOLS_DIR/r680_budget.sh"
# 见 tools/r680_budget.sh:整数字面量会被 DOUBLE 参数拒掉,失败只有一行警告,
# 预算就静默停在节点默认值上。这里先规范成浮点形态,设完再回读校验。
EXPLORE_TIME="$(r680_float_arg "${EXPLORE_TIME:-120.0}")"
# 空值/垃圾值必须在这里就死掉:空值传下去会让下面的回读校验变成永远通过。
case "$EXPLORE_TIME" in
    ''|*[!0-9.eE+-]*)
        echo "预算值非法: '${EXPLORE_TIME}' (要秒数,例如 300 或 300.0)"; exit 1 ;;
esac

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

# 从这里往下全部交给 tools/r680_start_explore.py:检查 → 确认 → 设参数 → 启动
# 合成**一个进程**。原来这一段要发起 12 次 `ros2` 命令行(5 次 param set、
# 5 次 param get、1 次预算回读、1 次 service call),每次冷启动实测 1.0~1.2 s,
# 叠起来十几秒 —— 操作员敲了 GO,车却纹丝不动。等待会让人以为脚本坏了,
# 转去手敲 ros2 param set 绕过它,而绕过的正是带校验的那条路。
#
# 校验没有减少,只是挪到了一个进程里:定位/障碍前置检查、参数回读比对预算、
# dry_run 实际生效与否、/exploration/start 的成功与否,一条不少。
ARGS=(--budget "$EXPLORE_TIME")
if [ "$MODE" = "go" ]; then ARGS+=(--go); else ARGS+=(--dry-run); fi
if [ "$ASSUME_YES" = "1" ]; then ARGS+=(--yes); fi
exec python3 "$TOOLS_DIR/r680_start_explore.py" "${ARGS[@]}"
