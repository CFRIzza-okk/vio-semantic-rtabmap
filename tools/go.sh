#!/usr/bin/env bash
# 一句话起跑:栈已经在跑的时候,让车立刻开始自主探索。
#
#   bash tools/go.sh                 # 默认 600 s
#   bash tools/go.sh 300             # 换预算(秒)
#   bash tools/go.sh 600 --rehearsal # 只选点不驱动底盘
#
# 它等价于:
#   bash tools/r680_explore_300.sh --fast --yes --budget <N>
#
# 为什么单独有这个入口:
#   r680_explore_300.sh 默认先花 3~4 分钟重建整栈(那是给"从零开始"用的)。
#   现场调试的真实节奏是:栈已经起好了,只想让车动起来。每次都要等整栈重建,
#   就会逼着人绕过脚本手敲 ros2 param set —— 那才是真正的危险动作。
#   所以这里把"栈已就绪"当作前提,并且**校验它,而不是假设它**:
#   缺节点 → 直接退出;闸门没开 → 直接退出。省时间,但不省安全检查。
#
# 不要加 `set -u`(ROS 的 setup.bash 会踩未定义变量)。

VIO=/home/jetson/VIO

BUDGET=600
ARGS=()
for arg in "$@"; do
    case "$arg" in
        -h|--help)
            # 打到第一行非注释为止:硬编码行号会随注释增长而截断,
            # 曾经就把 --fast 的说明挡掉了。
            awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"
            exit 0
            ;;
        --rehearsal|--no-wait|-y|--yes)
            ARGS+=("$arg")
            ;;
        *)
            case "$arg" in
                ''|*[!0-9.]*)
                    echo "未知参数: $arg  (预算是秒数,例如 300;其他参数见 --help)"
                    exit 2
                    ;;
            esac
            BUDGET="$arg"
            ;;
    esac
done

# --yes 是这里的默认值:这个入口存在的意义就是"敲了就动",
# 再问一次"确认继续?"只是让人多敲一次键盘,不会多一分安全 ——
# 真正的安全网是 r680_explore_start.sh 的前置检查(定位/障碍/电压)
# 和它那 5 秒的 Ctrl-C 窗口。
exec bash "$VIO/tools/r680_explore_300.sh" --fast --yes --budget "$BUDGET" "${ARGS[@]}"
