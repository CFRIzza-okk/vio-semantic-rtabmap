#!/usr/bin/env bash
# R680 自主探索:整栈启动到"武装但不动"为止。
#
# 本脚本**不会让车动**。它把一切准备好,然后打印 GO/NO-GO,由人来决定
# 是否执行最后一步(见 r680_explore_start.sh)。
#
# 三层安全阶梯(探索启动文件的三个开关,缺一不可):
#
#   dry_run  autostart   结果
#   -------- ---------   --------------------------------------------
#   true     false       只起 Nav2 与感知,探索器完全不选点   ← 本脚本默认
#   true     true        探索器发目标,但不驱动底盘(验证选点)
#   false    true        真正运动
#
# 用法:
#   bash tools/r680_explore_up.sh                  # 起栈,不开探索器
#   bash tools/r680_explore_up.sh --rehearsal      # 起栈 + dry_run 选点演练
#   bash tools/r680_explore_up.sh --no-gate        # 起点不动闸门(纯感知调试)
#
# 不要加 `set -u`(ROS 的 setup.bash 会踩到未定义变量)。

WS=/home/jetson/VIO/semantic_ws
TMP=/tmp/r680_explore
mkdir -p "$TMP"

# 探索限速。比 E1/E2 的 0.08/0.0/0.3 略高一点点,因为自主探索需要在
# 合理时间内走完一段距离;但**远低于** Nav2 默认的 0.20 —— 第一次自主
# 运动应该先确认它在真环境里的表现,再考虑放开。
MAX_LIN=0.10
MAX_LAT=0.0
MAX_ANG=0.35

# 预算收紧。原默认是 1800 s / 120 s,对第一次探索太长:
# 一旦选点或规划有问题,车会一直试下去。120 s 足够看出行为是否正确。
# 预算**只用于横幅显示**。r680_exploration.launch.py 并不声明
# max_exploration_time / goal_timeout,所以在这里改这两个数**不会**改变实际
# 预算 —— 真正设预算的唯一位置是 r680_explore_start.sh。允许环境覆盖,是为了
# 让横幅和实际值一致(r680_explore_300.sh 会 export 它们),否则横幅会撒谎。
MAX_EXPLORE_TIME="${MAX_EXPLORE_TIME:-120.0}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-60.0}"

MODE=up            # up | rehearsal
USE_GATE=1
for arg in "$@"; do
    case "$arg" in
        --rehearsal) MODE=rehearsal ;;
        --no-gate)   USE_GATE=0 ;;
        -h|--help)   sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数: $arg"; exit 2 ;;
    esac
done

say() { printf '\n=== %s ===\n' "$1"; }
fail=0
need() { # need <描述> <实际> <期望>
    if [ "$2" = "$3" ]; then printf '  ✅ %-38s %s\n' "$1" "$2"
    else printf '  ❌ %-38s %s (期望 %s)\n' "$1" "$2" "$3"; fail=1; fi
}

# ---------------------------------------------------------------- 就绪轮询
# 2026-10-16:原来整脚本用固定 sleep 盲等(合计 184 s,占整场 271.7 s 的 68%),
# 而实测各环节通常几秒就绪 —— 例如底盘驱动 5 s 内出自 /odom,却固定睡 30 s。
#
# 这里改成"条件成立就立刻继续,超时才放弃"。语义与原版一致:超时**不**中止,
# 仍然交给脚本后面原有的检查去判定成败(所以不会把"慢"误判成"坏")。
#
# 超时上限一律取原 sleep 的值或更大,因此最坏情况不会比原来更慢。
wait_for() { # wait_for <描述> <超时秒> <探测命令...>
    local desc="$1" limit="$2"; shift 2
    local t0=$SECONDS
    while :; do
        if "$@" >/dev/null 2>&1; then
            printf '  ✅ %-38s %s s\n' "$desc" "$((SECONDS - t0))"
            return 0
        fi
        if [ $((SECONDS - t0)) -ge "$limit" ]; then
            printf '  ⏱️  %-38s 超时 %s s(继续,由后续检查判定)\n' "$desc" "$limit"
            return 1
        fi
        sleep 0.5
    done
}

TX1_PROLOGUE='source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102'

tx1_odom_ready() { # /odom 真的出帧了才算驱动就绪
    timeout 25 python3 /home/jetson/tx1.py "$TX1_PROLOGUE
timeout 4 rostopic echo -n1 /odom 2>/dev/null | grep -q frame_id" >/dev/null 2>&1
}

tx1_node_ready() { # 中继等节点已注册
    timeout 25 python3 /home/jetson/tx1.py "$TX1_PROLOGUE
timeout 8 rosnode list 2>/dev/null | grep -qx '/r680_ros1_relay'" >/dev/null 2>&1
}

# Orin 侧需要清掉的进程名,清理与校验共用同一份,避免两处写岔。
KILL_PATTERNS="ros2_relay_node exploration_node obstacle_monitor_node localization_monitor_node \
depthimage_to_laserscan cmd_vel_safety_node rtabmap nav2 realsense lifecycle_manager \
controller_server planner_server bt_navigator behavior_server waypoint_follower \
costmap static_transform rviz"

orin_clean() { # 进程表里再无匹配项(这才是"清理干净"的正确判据)
    # shellcheck disable=SC2086
    python3 /home/jetson/VIO/tools/safe_kill.py --dry-run $KILL_PATTERNS 2>/dev/null \
        | grep -q "no matches"
}

orin_topic_live() { # 某话题已经有发布者
    timeout 12 ros2 topic info "$1" 2>/dev/null | grep -qi "Publisher count: [1-9]"
}

orin_all_active() { # 5 个 Nav2 生命周期节点全部 active
    local n
    for n in controller_server planner_server bt_navigator behavior_server waypoint_follower; do
        timeout 20 ros2 lifecycle get "/$n" 2>/dev/null | grep -q "active" || return 1
    done
    return 0
}

orin_bool_settled() { # 布尔话题连续两次读数一致 = 监视器已经稳定
    # 安全监视器刚启动时会先发布一个保守的"停机"值,之后才依据真实的 /scan
    # 判定。只读一次很可能正好读到那个瞬间值 —— 实测会误报 obstacle_stop=true,
    # 从而给出假 NO-GO(提速前靠 sleep 45 掩盖了这个问题)。
    # 这里等的是"稳定",**不预设它该是什么值**:真有障碍物时它会稳定在 true,
    # 后面的 need 判定照样抓得住。
    local a b
    a=$(timeout 12 ros2 topic echo --once "$1" 2>/dev/null | grep -o "data: .*")
    [ -z "$a" ] && return 1
    sleep 1.5
    b=$(timeout 12 ros2 topic echo --once "$1" 2>/dev/null | grep -o "data: .*")
    [ -n "$b" ] && [ "$a" = "$b" ]
}

printf '模式: %s   限速: %.2f / %.2f / %.2f   预算: %.0fs / %.0fs\n' \
    "$MODE" "$MAX_LIN" "$MAX_LAT" "$MAX_ANG" "$MAX_EXPLORE_TIME" "$GOAL_TIMEOUT"

# ---------------------------------------------------------------- 时钟闸
# 2026-10-16 复盘:Orin 的 RTC 停在 1970-01-01 且按**本地时区**解读,开机后系统
# 时钟整整落后真实时间 12 小时(实测两次读数相差 44359 s)。栈跑到一半 NTP 把它
# 拽回来 —— 一次 +44359 s 的跳变。后果:
#   * tf2 缓冲里的历史变换全部作废
#   * 传感器消息被时间过滤器丢弃,反复打印:
#       Message Filter dropping message: frame 'camera_depth_frame'
#       ... 'the timestamp on the message is earlier than all the data in the
#       transform cache'
#   * localization_valid 在 False/True 之间抖动 -> cmd_vel_safety 反复清零速度
#   * 控制器永远闭不上环 -> controller_server "Failed to make progress" ×3
#     (间隔 15.0 / 15.1 / 15.0 s,正是进度检查器的窗口)
#   * 探索器 60 s 目标超时 -> "goal timed out; rejecting it"
#     -> 0 个目标就宣告 COMPLETE
# 整场故障**表现成 Nav2 的问题,根子是时钟**。所以动任何东西之前先验时钟。
#
# 一次性修法:
#   sudo timedatectl set-local-rtc 0
#   sudo hwclock -w
#   sudo systemctl enable --now systemd-time-wait-sync
#   sudo reboot                     # set-local-rtc 必须重启才生效
CLOCK_SETTLE="${CLOCK_SETTLE:-3}"
if [ "${SKIP_CLOCK_CHECK:-0}" = "1" ]; then
    printf '\n=== 前置闸门:时钟检查 === (SKIP_CLOCK_CHECK=1,已跳过)\n'
else
    say "前置闸门:时钟检查"
    clock_bad=0

    _sync=$(timedatectl show -p NTPSynchronized --value 2>/dev/null)
    if [ "$_sync" = "yes" ]; then
        printf '  ✅ %-38s %s\n' "NTP 已同步" "$_sync"
    else
        printf '  ❌ %-38s %s\n' "NTP 已同步" "${_sync:-读取失败}"; clock_bad=1
    fi

    _lrtc=$(timedatectl show -p LocalRTC --value 2>/dev/null)
    if [ "$_lrtc" = "no" ]; then
        printf '  ✅ %-38s %s\n' "RTC 按 UTC 解读" "$_lrtc"
    else
        printf '  ❌ %-38s %s  <- 需 set-local-rtc 0 并重启\n' "RTC 按 UTC 解读" "${_lrtc:-读取失败}"
        clock_bad=1
    fi

    # 墙钟 vs 单调钟:时钟被 NTP 步进时,墙钟会跑得比单调钟快/慢。
    _driftout=$(python3 -c "
import time
a_w, a_m = time.time(), time.monotonic()
time.sleep($CLOCK_SETTLE)
b_w, b_m = time.time(), time.monotonic()
d = (b_w - a_w) - (b_m - a_m)
print('%s %.3f' % ('OK' if abs(d) < 0.05 else 'BAD', d))" 2>/dev/null)
    case "$_driftout" in
        OK*)  printf '  ✅ %-38s %s s (%s s 采样)\n' "时钟未跳变" "${_driftout#OK }" "$CLOCK_SETTLE" ;;
        BAD*) printf '  ❌ %-38s %s s  <- 正在跳变!\n' "时钟未跳变" "${_driftout#BAD }"; clock_bad=1 ;;
        *)    printf '  ❌ %-38s %s\n' "时钟未跳变" "检测失败"; clock_bad=1 ;;
    esac

    if [ "$clock_bad" = "1" ]; then
        cat <<'MSG'

  ❌ 时钟闸未通过 —— 拒绝启动,不碰 TX1。

     时钟不稳时启动机器人栈,后果是:TF 缓冲被作废 -> 传感器消息被时间过滤器
     丢光 -> 定位抖动 -> 速度被反复清零 -> 车"收到指令但不动",而且会被误诊成
     Nav2 的问题。先修时钟,再跑探索。

     确认无碍时可显式跳过:
       SKIP_CLOCK_CHECK=1 bash tools/r680_explore_up.sh
MSG
        exit 1
    fi
fi

# ---------------------------------------------------------------- TX1
# 清理与启动**必须**是分开的 ssh 调用。同一条命令里既有 pkill 的模式又有
# 待启动脚本的明文路径时,pkill 会匹配到执行它的那个 shell 自己,shell 在
# 启动行执行前就死了。这个坑在本项目已经踩过多次。
say "1/7 TX1: 清理"
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
pkill -f "[r]os1_relay" 2>/dev/null
pkill -f "[t]x1_power_log" 2>/dev/null
pkill -f "[w]heeltec_robot" 2>/dev/null
pkill -f "[r]oscore" 2>/dev/null
sleep 3
echo cleaned' 2>&1 | tail -1

say "1b/7 TX1: 独立 roscore —— master 不能挂在 roslaunch 下"
# roslaunch 发现没有 master 时会**自己拉起一个**,而那个 rosmaster 会成为
# roslaunch 进程的**子进程**。于是两者同生共死:在那个 roslaunch 上按 Ctrl-C,
# 或者上面清理步骤里的 `pkill -f "[w]heeltec_robot"`,都会顺手把 master 带走。
#
# 受害的不只是驱动。master 重启后 rospy 节点**不会重新注册**,所以中继和
# tx1_power_log.py 会继续跑着,却从 `rosnode list` 里消失、什么也收不到 ——
# 没有任何报错,它们只是安静地废掉了。只有进程表还能看出它们。
#
# 先把 roscore 单独起成 detached 进程,这个从属关系就断了:驱动的 roslaunch
# 会挂到已有的 master 上,而不是自己再造一个,以后重启驱动不必再重启 master。
#
# 这一步**必须**与上面的清理分成两次 ssh 调用:清理里有 `pkill -f "[r]oscore"`,
# 而这一行含明文 `roscore`,合在一起就是自匹配陷阱。
#
# 下面那段单引号参数里有两条规则,写在这里而不是写在里面 —— 里面看着像注释的
# 东西其实不是注释:
#   * 参数内部的 `#` **不构成注释**。对外层来说整段只是一个 bash 字符串,
#     所以里面哪怕写在"注释"里的撇号也照样会终止字符串(phase_d 脚本就是这么
#     被写坏的,而且报错出现在 20 行之后的那个引号上)。
#   * 查 master 用 ps + grep,**不要用 pgrep**:pgrep 会拿它自己的命令行去匹配,
#     而那个命令行就是这段参数本身、里面含有明文的进程名,于是它会匹配到
#     执行它的那个 shell。
#
# 启动之后用**有界轮询**等端口 11311,而不是 `sleep 10`。实测 master 在 fork
# 之后约 2 s 就在监听了,固定睡 10 s 等于每次启动白等 8 s;而机器慢的时候,
# 同一个固定值又会变成"睡不够"。轮询上限 15 s(比原来的 10 s 更长,最坏情况
# 不会更糟),下面的检查随后报出真实状态。
#
# 归属检查按**端口**挑 master,不用 `head -1`。2026-10-17 在两个 master 同时
# 活着时实测:`head -1` 取的是 pid 最小=**最老**的那个,所以上一次会话残留的
# master 会盖过本次栈真正在用的那个 —— 检查会把残留报成健康,而真正的地雷
# 正是本次的 master。对它要解决的问题来说,这个方向刚好是反的。
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
cd /home/wheeltec
nohup roscore > /home/wheeltec/roscore.log 2>&1 &
for _i in $(seq 1 30); do
    ss -ltn 2>/dev/null | grep -q 11311 && break
    sleep 0.5
done
echo "--- 11311 监听 ---"
ss -ltn 2>/dev/null | grep 11311 || echo "!! 11311 没有监听(master 没起来,下面的 roslaunch 会自己造一个 —— 地雷复活)"
echo "--- master 的父进程必须是 roscore,不是 roslaunch ---"
MPID=$(ps -eo pid,args | grep "[r]osmaster" | grep -- "-p 11311" | awk "{print \$1}" | head -1)
if [ -z "$MPID" ]; then
    echo "  !! 11311 上没有 rosmaster —— 驱动的 roslaunch 会自己造一个,那正是地雷"
else
    MPPID=$(ps -o ppid= -p "$MPID" 2>/dev/null | tr -d " ")
    MPARENT=$(ps -o args= -p "$MPPID" 2>/dev/null)
    echo "  master pid=$MPID  父进程 pid=$MPPID  ->  $MPARENT"
    case "$MPARENT" in
        *roscore*) echo "  OK: master 属于独立 roscore" ;;
        *)         echo "  !! master 不属于 roscore —— 地雷还在" ;;
    esac
fi' 2>&1 | tail -8

say "2/7 TX1: 底盘驱动"
# 启动与等待分开:启动只负责发命令,不要在同一个 ssh 调用里盲等 30 s。
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
source /home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
cd /home/wheeltec
nohup roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch > /home/wheeltec/driver.log 2>&1 &
sleep 1
echo launched' 2>&1 | tail -1
wait_for "TX1 驱动就绪(/odom 出帧)" 45 tx1_odom_ready

say "3/7 TX1: 电压记录(充电/放电曲线,用于定 min_battery)"
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
nohup python2 /home/wheeltec/tx1_power_log.py > /home/wheeltec/power_log.out 2>&1 &
sleep 1; echo "  已启动(每 20 s 记一次到 /home/wheeltec/power_log.csv)"' 2>&1 | tail -1

if [ "$USE_GATE" = "1" ]; then
    say "4/7 TX1: 打开运动闸门(经 r680_gate.sh,带校验)"
    bash /home/jetson/VIO/tools/r680_gate.sh open --limits "$MAX_LIN" "$MAX_LAT" "$MAX_ANG" 2>&1 | tail -6
    if ! bash /home/jetson/VIO/tools/r680_gate.sh status 2>&1 | grep -q "GATE OPEN"; then
        echo "  ❌ 闸门未确认打开 —— 中止"
        exit 1
    fi
else
    say "4/7 TX1: 跳过闸门(--no-gate),车不可能动"
    bash /home/jetson/VIO/tools/r680_gate.sh close 2>&1 | tail -2
fi

# ---------------------------------------------------------------- Orin
# Orin 侧**也必须**先清理,而且必须在 source ROS 之后(source 之前拿不到
# ros2 node list)。漏掉这一步的后果不是"多几个进程",而是**每个节点都变成
# 两个,静静地互相打架**:
#
#   * 两条 /cmd_vel_safe 发布者竞争 —— 速度指令交替被覆盖
#   * 两个 localization_valid 发布者 —— 标志可能一真一假
#   * 两份 /scan / /rtabmap/map —— costmap 收到重复输入
#   * 两套 Nav2 —— 两套 costmap 各自维护,行为不可预测
#
# 而**没有任何一处会报错**。验收检查会显示"Publisher count: 2",那是最容易
# 被当成正常的一行。本项目的 test_safety_monitors.py 里那 3 个"假失败"就是
# 同一个根因(两个发布者同话题打架),只不过那次在软件里,这次在真机上。
#
# 实测:连跑两次 r680_explore_up.sh,节点数从 22 涨到 43,relay 从 1 个变 3 个。
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

say "5/7 Orin: 清理上一次整栈(必须先做,否则会叠加成两套)"
# shellcheck disable=SC2086
python3 /home/jetson/VIO/tools/safe_kill.py --signal TERM $KILL_PATTERNS 2>&1 | tail -1
wait_for "Orin 节点优雅退出(TERM)" 20 orin_clean
# shellcheck disable=SC2086
python3 /home/jetson/VIO/tools/safe_kill.py --signal KILL $KILL_PATTERNS 2>&1 | tail -1
wait_for "Orin 残留进程清零(KILL)" 12 orin_clean

# 这里**不能**用 `ros2 node list | wc -l` 判定清理是否干净 —— 它测的是 DDS
# 发现图而不是进程表,两种情况下都会非零:
#   * 刚被杀死、发现租约未到期的节点仍会被列出(实测 KILL 之后仍显示 15 个)
#   * 行为树内部节点、costmap 组件节点、tf2 监听器节点**根本不是进程**,
#     却一直在列表里(实测健康栈 23 个节点中有 8 个属于这一类)
# 所以"应为 0"这个断言永远不可能通过,只会制造假警报,掩盖真正的残留。
# 正确判据是进程表 —— 也就是上面 safe_kill.py --dry-run 的 no matches。
printf '  清理后残留进程(应为 no matches): '
# shellcheck disable=SC2086
python3 /home/jetson/VIO/tools/safe_kill.py --dry-run $KILL_PATTERNS 2>&1 | tail -1

say "5b/7 Orin: relay + 相机 + 静态TF/安全闸门"
setsid nohup ros2 launch semantic_rtabmap r680_relay.launch.py > "$TMP/relay.log" 2>&1 &
wait_for "relay 就绪(/odom 有发布者)" 25 orin_topic_live /odom
setsid nohup ros2 launch semantic_rtabmap r680_depth_cloud.launch.py start_inspector:=false > "$TMP/cam.log" 2>&1 &
# 等相机**自己**的话题。不要等 /scan —— 那不是相机 launch 产的(它来自更晚
# 启动的 depthimage_to_laserscan),在这里等它必然等到超时。
wait_for "相机就绪(color/camera_info 有发布者)" 45 orin_topic_live /camera/camera/color/camera_info
setsid nohup ros2 launch semantic_rtabmap r680_navigation.launch.py \
    publish_base_tf:=false \
    max_linear_x:=$MAX_LIN max_linear_y:=$MAX_LAT max_angular_z:=$MAX_ANG \
    > "$TMP/nav.log" 2>&1 &
wait_for "安全闸门就绪(/cmd_vel_safe 有发布者)" 40 orin_topic_live /cmd_vel_safe

# 6 个参数并行查:串行每个要 1.3~4.3 s,并行后约等于最慢的那一次。
# 注意 wait 必须指名 PID —— 不带参数的 wait 会等上面那些永不退出的 ros2 launch。
SAFETY_PARAMS="max_linear_x max_linear_y max_angular_z command_timeout require_localization stop_on_obstacle obstacle_escape_rotation obstacle_escape_angular_z"
_pdir=$(mktemp -d); _pids=""
_i=0
for p in $SAFETY_PARAMS; do
    _i=$((_i + 1))
    ( timeout 20 ros2 param get /cmd_vel_safety "$p" 2>&1 | tail -1 > "$_pdir/$_i" ) &
    _pids="$_pids $!"
done
for _pid in $_pids; do wait "$_pid"; done
_i=0
for p in $SAFETY_PARAMS; do
    _i=$((_i + 1))
    printf '  cmd_vel_safety %-20s %s\n' "$p" "$(cat "$_pdir/$_i" 2>/dev/null)"
done
rm -rf "$_pdir"

say "6/7 Orin: RTAB-Map(建图与定位)"
rm -f "$HOME/.ros/rtabmap_explore.db"
setsid nohup ros2 launch rtabmap_launch rtabmap.launch.py \
    visual_odometry:=false icp_odometry:=false \
    odom_topic:=/odom frame_id:=base_footprint \
    odom_frame_id:=odom_combined \
    depth:=true approx_sync:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    rtabmap_viz:=false rviz:=false publish_tf_map:=true \
    "args:=--Rtabmap/DetectionRate 2.0 --Grid/CellSize 0.05 --Mem/NotLinkedNodesKept false" \
    "database_path:=$HOME/.ros/rtabmap_explore.db" > "$TMP/rtab.log" 2>&1 &
wait_for "RTAB-Map 就绪(/rtabmap/map 有发布者)" 55 orin_topic_live /rtabmap/map
printf '  RTAB-Map 角方差告警(应为 0): '
grep -ci "large angular variance" "$TMP/rtab.log"
printf '  里程计话题的发布者(应为 1,即 relay;>1 说明 rtabmap 的视觉里程计误开): '
timeout 20 ros2 topic info /odom 2>&1 | grep -i "publisher count" | tail -1

say "7/7 Orin: /scan + 安全监视器 + Nav2"
if [ "$MODE" = "rehearsal" ]; then
    DRY=true; AUTO=true;  echo "  (演练模式:dry_run=true autostart=true —— 会选点但不驱动底盘)"
else
    DRY=true; AUTO=false; echo "  (默认:dry_run=true autostart=false —— 完全不选点)"
fi
setsid nohup ros2 launch semantic_rtabmap r680_exploration.launch.py \
    include_nav2:=true include_safety:=false \
    dry_run:=$DRY autostart:=$AUTO \
    max_linear_x:=$MAX_LIN max_linear_y:=$MAX_LAT max_angular_z:=$MAX_ANG \
    > "$TMP/explore.log" 2>&1 &
wait_for "Nav2 生命周期全部 active" 60 orin_all_active
# 等安全监视器的输出稳定下来再判定 —— 否则会读到启动瞬间的保守值。
wait_for "安全监视器数值稳定" 25 orin_bool_settled /safety/obstacle_stop

say "Nav2 生命周期(必须全部 active)"
# 5 个节点并行查(串行实测 21.5 s,每个 lifecycle get 约 4.3 s)。
NAV2_NODES="controller_server planner_server bt_navigator behavior_server waypoint_follower"
_ldir=$(mktemp -d); _pids=""; _i=0
for n in $NAV2_NODES; do
    _i=$((_i + 1))
    ( timeout 12 ros2 lifecycle get "/$n" 2>&1 | tail -1 > "$_ldir/$_i" ) &
    _pids="$_pids $!"
done
for _pid in $_pids; do wait "$_pid"; done
_i=0
for n in $NAV2_NODES; do
    _i=$((_i + 1))
    printf '  %-20s %s\n' "$n" "$(cat "$_ldir/$_i" 2>/dev/null)"
done
rm -rf "$_ldir"

say "前置条件检查"
# 9 个 topic info + 5 个探测一次性并行(串行实测 26.7 s,并行后约等于最慢的一个)。
PREREQ_TOPICS="/scan /map /rtabmap/map /cmd_vel_nav /cmd_vel_safe /safety/localization_valid /safety/obstacle_stop /exploration/goal /battery_voltage"
_qdir=$(mktemp -d); _pids=""; _i=0
for t in $PREREQ_TOPICS; do
    _i=$((_i + 1))
    ( timeout 20 ros2 topic info "$t" 2>&1 | tr '\n' ' ' | sed 's/Type: //' | head -c 76 > "$_qdir/$_i" ) &
    _pids="$_pids $!"
done
( timeout 20 ros2 topic echo --once /safety/localization_valid 2>&1 | grep data | head -1 > "$_qdir/lv" ) & _pids="$_pids $!"
( timeout 20 ros2 topic echo --once /safety/obstacle_stop        2>&1 | grep data | head -1 > "$_qdir/os" ) & _pids="$_pids $!"
( timeout 20 ros2 topic echo --once /battery_voltage             2>&1 | grep data | head -1 > "$_qdir/bv" ) & _pids="$_pids $!"
( timeout 20 ros2 param get /r680_explorer min_battery 2>&1 | tail -1 > "$_qdir/mb" ) & _pids="$_pids $!"
( timeout 20 ros2 param get /r680_explorer dry_run     2>&1 | tail -1 > "$_qdir/dr" ) & _pids="$_pids $!"
for _pid in $_pids; do wait "$_pid"; done

echo "  话题:"
_i=0
for t in $PREREQ_TOPICS; do
    _i=$((_i + 1))
    printf '    %-30s %s\n' "$t" "$(cat "$_qdir/$_i" 2>/dev/null)"
done
echo
LV=$(cat "$_qdir/lv" 2>/dev/null); OS=$(cat "$_qdir/os" 2>/dev/null); BV=$(cat "$_qdir/bv" 2>/dev/null)
printf '  localization_valid %s\n' "$LV"
printf '  obstacle_stop      %s\n' "$OS"
printf '  battery_voltage    %s\n' "$BV"
printf '  min_battery(阈值)  %s\n' "$(cat "$_qdir/mb" 2>/dev/null)"
printf '  dry_run            %s\n' "$(cat "$_qdir/dr" 2>/dev/null)"
rm -rf "$_qdir"

echo
echo "  判定:"
need "localization_valid = True" "$(echo "$LV" | grep -o 'true\|false')" "true"
need "obstacle_stop = False"     "$(echo "$OS" | grep -o 'true\|false')" "false"

say "闸门最终状态(TX1 侧,以 /cmd_vel 发布者为准)"
bash /home/jetson/VIO/tools/r680_gate.sh status 2>&1 | tail -3

echo
if [ "$fail" = "0" ]; then
    echo "════════════════════════════════════════════════════════"
    echo " GO —— 栈已就绪,车尚未收到任何速度目标"
    echo "════════════════════════════════════════════════════════"
    echo " 开始自主探索:  bash tools/r680_explore_start.sh"
    echo " 监视:          bash tools/r680_explore_watch.sh"
    echo " 急停:          bash tools/r680_estop.sh"
else
    echo "════════════════════════════════════════════════════════"
    echo " NO-GO —— 上面有 ❌,不要开始探索"
    echo "════════════════════════════════════════════════════════"
fi
echo "日志在 $TMP/"
