#!/usr/bin/env bash
# 自主探索的实时监视。一屏看清"车在去哪、走了多远、有没有异常"。
#
#   bash tools/r680_explore_watch.sh          # 每 2 s 刷新
#   bash tools/r680_explore_watch.sh 1        # 每 1 s 刷新
#
# 为什么需要它:探索器出问题时的表现是**安静**的 —— 它可能安静地卡在同一个
# 目标上、安静地反复规划失败、或者安静地因为电池样本陈旧而中止。这些都是
# "没有报错但已经不对"的形态,本项目反复遇到。
#
# 所以这一屏刻意把三类量放在一起看:
#   * 本该推进的量  —— goals_completed 在涨、位置在变
#   * 本该不变的量  —— dry_run=False、obstacle_stop=False、定位有效
#   * 本该为零的量  —— consecutive_failures
# 单看任何一类都可能读成"正常"。
#
# 特别是 consecutive_failures:它连续不为零就是"在努力但一直失败"的信号,
# 而那时 /exploration/state 很可能**仍然写着 EXPLORING**。只看 state 会
# 误判成一切正常。
#
# Ctrl-C 退出监视**不会**让车停下 —— 停车用 r680_estop.sh。
#
# 不要加 `set -u`。

WS=/home/jetson/VIO/semantic_ws
PERIOD="${1:-2}"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

last_px=""
stall=0

while true; do
    raw=$(timeout 8 ros2 topic echo --once /exploration/status 2>/dev/null \
          | grep '^data:' | head -1 | sed 's/^data: //')
    st=$(timeout 6 ros2 topic echo --once /exploration/state 2>&1 | grep '^data:' | head -1)

    lv=$(timeout 6 ros2 topic echo --once /safety/localization_valid 2>&1 | grep -o 'true\|false' | head -1)
    os_=$(timeout 6 ros2 topic echo --once /safety/obstacle_stop 2>&1 | grep -o 'true\|false' | head -1)
    bv=$(timeout 6 ros2 topic echo --once /battery_voltage 2>&1 | grep data | head -1)

    # 速度与位置从 TX1 读 —— 那才是底盘真正收到的
    tx1=$(timeout 40 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
V=$(timeout 5 rostopic echo -n 1 /cmd_vel 2>/dev/null | tr "\n" " ")
P=$(timeout 5 rostopic echo -n 1 /odom/pose/pose/position 2>/dev/null | tr "\n" " ")
echo "$V | $P"' 2>/dev/null | tail -1)
    vx=$(echo "$tx1" | grep -o 'x: [-0-9.]*' | head -1 | cut -d' ' -f2)
    px=$(echo "$tx1" | grep -o 'x: [-0-9.]*' | tail -1 | cut -d' ' -f2)

    # 轮子在转但车没动 = 堵转 / 被卡 / 打滑
    if [ -n "$px" ] && [ "$px" = "$last_px" ] && [ -n "$vx" ] && [ "$vx" != "0.0" ] && [ "$vx" != "0" ]; then
        stall=$((stall + 1))
    else
        stall=0
    fi
    last_px="$px"

    vals=$(printf '%s' "$raw" | python3 -c "
import sys, json
line = sys.stdin.read().strip()
keys = ('dry_run', 'consecutive_failures', 'goals_completed',
        'goals_failed', 'blacklisted', 'unexplorable', 'elapsed')
try:
    d = json.loads(line)
except Exception:
    print('|'.join(['-'] * len(keys))); raise SystemExit
print('|'.join(str(d.get(k, '-')) for k in keys))
" 2>/dev/null)
    IFS='|' read -r dry fail done_ failed bl unexp el <<< "$vals"
    [ -z "$dry" ] && { dry="-"; fail="-"; done_="-"; failed="-"; bl="-"; unexp="-"; el="-"; }

    printf '\033[2J\033[H'
    echo "════════ R680 自主探索监视 ════════  $(date +%H:%M:%S)"
    echo
    printf '  状态:      %s\n' "$(echo "$st" | sed 's/^data: //')"
    if [ "$dry" = "False" ]; then
        printf '  模式:      ⚠️  真实运动 (dry_run=False)\n'
    else
        printf '  模式:      演练,不驱动底盘 (dry_run=%s)\n' "$dry"
    fi
    printf '  已用时间:  %s s / 120 s 预算\n' "$el"
    printf '  目标:      完成 %s,失败 %s,黑名单 %s\n' "$done_" "$failed" "$bl"
    printf '  无法探索:  %s 处' "$unexp"
    if [ "$unexp" != "0" ] && [ "$unexp" != "-" ] && [ -n "$unexp" ]; then
        printf '   (反复失败,本轮不再派车去;明细: ros2 topic echo /exploration/unexplorable --once)'
    fi
    echo
    printf '  连续失败:  %s' "$fail"
    if [ "$fail" != "0" ] && [ "$fail" != "-" ] && [ -n "$fail" ]; then
        printf '   ⚠️  一直在失败,但状态可能仍写着 EXPLORING'
    fi
    echo
    echo
    echo "  ── 安全标志 ──"
    printf '  定位有效:  %s' "$lv"
    [ "$lv" = "false" ] && printf '   ⚠️  cmd_vel_safety 会拒绝转发'
    echo
    printf '  障碍停车:  %s' "$os_"
    if [ "$os_" = "true" ]; then
        # 这里不再只报"有障碍":障碍锁存时安全节点只封平移、放开原地旋转,
        # 所以车仍然在动是**预期**的,不是故障。
        printf '   (0.45 m 内有障碍:只封前进/横移,允许原地旋转脱困)'
    fi
    echo
    printf '  电池:      %s\n' "${bv#data: }"
    echo
    echo "  ── 底盘(TX1 侧,车真正收到的) ──"
    printf '  速度 x:    %s   (限速 0.10)\n' "$vx"
    printf '  位置 x:    %s\n' "$px"
    if [ "$stall" -ge 3 ]; then
        printf '  ⚠️  速度非零但位置不变 —— 可能堵转或被卡住(%s 次)\n' "$stall"
    fi
    echo
    echo "  Ctrl-C 退出监视(不会让车停下)"
    echo "  急停: bash tools/r680_estop.sh"

    sleep "$PERIOD"
done
