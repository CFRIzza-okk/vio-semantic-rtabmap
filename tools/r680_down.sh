#!/usr/bin/env bash
# R680 完全收尾:把 r680_explore_up.sh 起来的一切都关掉,并且逐项验证。
#
#   bash tools/r680_down.sh              # 全收:先确保停住,再拆 Orin + TX1
#   bash tools/r680_down.sh --keep-tx1   # 只拆 Orin 侧,保留底盘驱动与中继
#   bash tools/r680_down.sh --check      # 只报告当前状态,什么都不动
#
# 为什么之前"跑完没退干净":r680_explore_300.sh 的第 3/3 步只做
# `r680_gate.sh close`。闸门关了确实意味着车不可能动,但整栈还活着 ——
# Orin 上 Nav2/RTM-Map/相机/中继二十多个节点、TX1 上驱动与 rosmaster 全在。
# 下一次再跑 r680_explore_up.sh 时那些残留就靠它自己的清理步骤去收,
# 而清理一旦有漏,就变成"两套栈静静地互相打架"。
#
# 顺序是刻意的,不能颠倒:
#   1. 先让车不可能动(锁存急停 + 关闸 + 验证)
#   2. 再拆 Orin
#   3. 最后拆 TX1
# 反过来做的话,拆栈过程中会出现一个"速度发布者还在、但安全节点已经死了"
# 的窗口 —— 那正是最危险的中间态。
#
# 不要加 `set -u`(ROS 的 setup.bash 会踩到未定义变量)。

WS=/home/jetson/VIO/semantic_ws
KEEP_TX1=0
CHECK_ONLY=0
for arg in "$@"; do
    case "$arg" in
        --keep-tx1) KEEP_TX1=1 ;;
        --check)    CHECK_ONLY=1 ;;
        -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数: $arg"; exit 2 ;;
    esac
done

say() { printf '\n=== %s ===\n' "$1"; }
fail=0

need() {
    if [ "$2" = "$3" ]; then printf '  ✅ %-38s %s\n' "$1" "$2"
    else printf '  ❌ %-38s %s (期望 %s)\n' "$1" "$2" "$3"; fail=1; fi
}

# 与 r680_explore_up.sh 同一套轮询:条件成立就继续,超时才放弃。
wait_for() {
    local desc="$1" limit="$2"; shift 2
    local t0=$SECONDS
    while :; do
        if "$@" >/dev/null 2>&1; then
            printf '  ✅ %-38s %s s\n' "$desc" "$((SECONDS - t0))"
            return 0
        fi
        if [ $((SECONDS - t0)) -ge "$limit" ]; then
            printf '  ⏱️  %-38s 超时 %s s\n' "$desc" "$limit"
            return 1
        fi
        sleep 0.5
    done
}

TX1_PROLOGUE='source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102'

# Orin 侧需要拆掉的进程名 —— 与 r680_explore_up.sh 第 5/7 步保持同一份清单。
KILL_PATTERNS="ros2_relay_node exploration_node obstacle_monitor_node localization_monitor_node \
depthimage_to_laserscan cmd_vel_safety_node rtabmap nav2 realsense lifecycle_manager \
controller_server planner_server bt_navigator behavior_server waypoint_follower \
costmap static_transform rviz"

orin_clean() {
    # shellcheck disable=SC2086
    python3 /home/jetson/VIO/tools/safe_kill.py --dry-run $KILL_PATTERNS 2>/dev/null \
        | grep -q "no matches"
}

tx1_gate_closed() {
    timeout 40 python3 /home/jetson/tx1.py "$TX1_PROLOGUE
timeout 8 rostopic info /cmd_vel 2>/dev/null | grep -q 'Publishers: None'" >/dev/null 2>&1
}

# TX1 上 roslaunch 起的整棵进程树都带 wheeltec_robot 字样;
# ros1_relay / tx1_power_log 各自单列。
#
# roscore / rosmaster / rosout 是**另一棵独立的 master 树** —— 由各 up 脚本的
# "1b" 步刻意起成 detached,好让 master 不成为某个 roslaunch 的子进程(理由见
# 那些脚本里的说明)。正因为独立,它们**不会**随 wheeltec_robot 一起消失,
# 必须单独列出来。漏掉它们的后果是:验收会显示"TX1 进程 = 清零",而 master
# 其实还在跑 —— 一个假绿灯,而且下次起栈时新 roscore 会撞上它。
TX1_STACK_PATTERN='[w]heeltec_robot|[r]os1_relay|[t]x1_power_log|[r]oscore|[r]osmaster|[r]osout'

tx1_clean() {
    # 注意 pkill 的模式里**不能**出现待匹配进程的明文路径,
    # 否则会匹配到执行它的这个 shell 自己(本项目踩过七次)。
    timeout 40 python3 /home/jetson/tx1.py "pgrep -f '$TX1_STACK_PATTERN' >/dev/null 2>&1 && exit 1 || exit 0" >/dev/null 2>&1
}

# ------------------------------------------------------------------ 状态报告
report() {
    say "当前状态"
    printf '  Orin 匹配进程数: '
    # shellcheck disable=SC2086
    python3 /home/jetson/VIO/tools/safe_kill.py --dry-run $KILL_PATTERNS 2>/dev/null | tail -1
    printf '  Orin 节点数:     '
    timeout 20 ros2 node list 2>/dev/null | wc -l
    printf '  TX1 残留进程:    '
    timeout 40 python3 /home/jetson/tx1.py "pgrep -af '$TX1_STACK_PATTERN' 2>/dev/null | wc -l" 2>&1 | tail -1
    echo '  闸门:'
    bash /home/jetson/VIO/tools/r680_gate.sh status 2>&1 | tail -2 | sed 's/^/    /'
}

cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

if [ "$CHECK_ONLY" = "1" ]; then
    report
    exit 0
fi

# ---------------------------------------------------------------- 1. 先停住
# 这一步在任何拆卸之前。拆卸过程中节点会一个个死掉,而只要 TX1 闸门
# 还是开的,残留的速度发布者就可能把最后一条指令留在底盘里。
say "1/4 先让车不可能动(锁存急停 + 关闸)"
for _i in 1 2 3; do
    timeout 12 ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
done
echo "      已发 3 次 /safety/estop = true"
bash /home/jetson/VIO/tools/r680_gate.sh close 2>&1 | tail -3
wait_for "闸门确认关闭(/cmd_vel 无发布者)" 20 tx1_gate_closed

# ---------------------------------------------------------------- 2. 拆 Orin
say "2/4 拆 Orin 栈"
# shellcheck disable=SC2086
python3 /home/jetson/VIO/tools/safe_kill.py --signal TERM $KILL_PATTERNS 2>&1 | tail -1
wait_for "Orin 节点优雅退出(TERM)" 20 orin_clean
# shellcheck disable=SC2086
python3 /home/jetson/VIO/tools/safe_kill.py --signal KILL $KILL_PATTERNS 2>&1 | tail -1
wait_for "Orin 残留进程清零(KILL)" 12 orin_clean

# ---------------------------------------------------------------- 3. 拆 TX1
if [ "$KEEP_TX1" = "1" ]; then
    say "3/4 保留 TX1(--keep-tx1):底盘驱动与中继继续运行"
else
    say "3/4 拆 TX1 栈(中继 / 电压记录 / 底盘驱动 / roslaunch / master 树)"
    # 分两次:先 TERM 让它有机会正常收尾(驱动析构时会向单片机发停止帧),
    # 再对顽固残留用 KILL。
    #
    # master 树按**从上到下**的顺序拆:先 roscore(它会带走 rosmaster 和
    # rosout),再对可能残留的 rosmaster / rosout 补刀。反过来的话 roscore
    # 会失去子进程却仍然活着占着 11311。
    timeout 60 python3 /home/jetson/tx1.py "$TX1_PROLOGUE
pkill -f '[r]os1_relay' 2>/dev/null
pkill -f '[t]x1_power_log' 2>/dev/null
pkill -f '[w]heeltec_robot' 2>/dev/null
pkill -f '[r]oscore' 2>/dev/null
pkill -f '[r]osmaster' 2>/dev/null
pkill -f '[r]osout' 2>/dev/null
sleep 1
echo term-sent" 2>&1 | tail -1
    wait_for "TX1 进程优雅退出(TERM)" 15 tx1_clean
    timeout 60 python3 /home/jetson/tx1.py "$TX1_PROLOGUE
pkill -9 -f '[r]os1_relay' 2>/dev/null
pkill -9 -f '[t]x1_power_log' 2>/dev/null
pkill -9 -f '[w]heeltec_robot' 2>/dev/null
pkill -9 -f '[r]oscore' 2>/dev/null
pkill -9 -f '[r]osmaster' 2>/dev/null
pkill -9 -f '[r]osout' 2>/dev/null
sleep 1
echo kill-sent" 2>&1 | tail -1
    wait_for "TX1 残留进程清零(KILL)" 10 tx1_clean
fi

# ---------------------------------------------------------------- 4. 验收
say "4/4 验收"
GATE_LINE=$(bash /home/jetson/VIO/tools/r680_gate.sh status 2>&1 | tail -1)
need "闸门 = CLOSED" "$(echo "$GATE_LINE" | grep -o 'GATE CLOSED')" "GATE CLOSED"

if orin_clean; then ORIN_STATE="no matches"; else ORIN_STATE="仍有残留"; fi
need "Orin 进程 = 清零" "$ORIN_STATE" "no matches"

if [ "$KEEP_TX1" = "1" ]; then
    printf '  ⏭️  %-38s %s\n' "TX1 进程(--keep-tx1 跳过)" "保留"
else
    if tx1_clean; then TX1_STATE="no matches"; else TX1_STATE="仍有残留"; fi
    need "TX1 进程 = 清零" "$TX1_STATE" "no matches"
fi

echo
if [ "$fail" = "0" ]; then
    echo "════════════════════════════════════════════════════════"
    echo " DOWN —— 已完全收尾,车不可能动"
    echo "════════════════════════════════════════════════════════"
    echo " 重新起栈:  bash tools/r680_explore_up.sh"
else
    echo "════════════════════════════════════════════════════════"
    echo " 有 ❌ —— 收尾不完整,请人工确认"
    echo "════════════════════════════════════════════════════════"
    echo " 最快且最可靠的兜底:物理断电。手放在开关上。"
fi
exit "$fail"
