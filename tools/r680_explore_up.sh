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
MAX_EXPLORE_TIME=120.0
GOAL_TIMEOUT=60.0

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

printf '模式: %s   限速: %.2f / %.2f / %.2f   预算: %.0fs / %.0fs\n' \
    "$MODE" "$MAX_LIN" "$MAX_LAT" "$MAX_ANG" "$MAX_EXPLORE_TIME" "$GOAL_TIMEOUT"

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

say "2/7 TX1: 底盘驱动"
timeout 150 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
source /home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
cd /home/wheeltec
nohup roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch > /home/wheeltec/driver.log 2>&1 &
sleep 30
timeout 10 rosnode list 2>&1 | tr "\n" " " | head -c 300; echo' 2>&1 | tail -3

say "3/7 TX1: 电压记录(充电/放电曲线,用于定 min_battery)"
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
nohup python2 /home/wheeltec/tx1_power_log.py > /home/wheeltec/power_log.out 2>&1 &
sleep 3; echo "  已启动(每 20 s 记一次到 /home/wheeltec/power_log.csv)"' 2>&1 | tail -1

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
python3 /home/jetson/VIO/tools/safe_kill.py --signal TERM \
    ros2_relay_node exploration_node obstacle_monitor_node localization_monitor_node \
    depthimage_to_laserscan cmd_vel_safety_node rtabmap nav2 realsense lifecycle_manager \
    controller_server planner_server bt_navigator behavior_server waypoint_follower \
    costmap static_transform rviz 2>&1 | tail -2
sleep 8
python3 /home/jetson/VIO/tools/safe_kill.py --signal KILL \
    ros2_relay_node exploration_node obstacle_monitor_node localization_monitor_node \
    depthimage_to_laserscan cmd_vel_safety_node rtabmap nav2 realsense lifecycle_manager \
    controller_server planner_server bt_navigator behavior_server waypoint_follower \
    costmap static_transform rviz 2>&1 | tail -2
sleep 6
printf '  清理后残留节点数(应为 0): '
timeout 15 ros2 node list 2>/dev/null | wc -l

say "5b/7 Orin: relay + 相机 + 静态TF/安全闸门"
nohup ros2 launch semantic_rtabmap r680_relay.launch.py > "$TMP/relay.log" 2>&1 &
sleep 10
nohup ros2 launch semantic_rtabmap r680_depth_cloud.launch.py start_inspector:=false > "$TMP/cam.log" 2>&1 &
sleep 24
nohup ros2 launch semantic_rtabmap r680_navigation.launch.py \
    publish_base_tf:=false \
    max_linear_x:=$MAX_LIN max_linear_y:=$MAX_LAT max_angular_z:=$MAX_ANG \
    > "$TMP/nav.log" 2>&1 &
sleep 12
for p in max_linear_x max_linear_y max_angular_z command_timeout require_localization stop_on_obstacle; do
    printf '  cmd_vel_safety %-20s ' "$p"
    timeout 8 ros2 param get /cmd_vel_safety "$p" 2>&1 | tail -1
done

say "6/7 Orin: RTAB-Map(建图与定位)"
rm -f "$HOME/.ros/rtabmap_explore.db"
nohup ros2 launch rtabmap_launch rtabmap.launch.py \
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
sleep 40
printf '  RTAB-Map 角方差告警(应为 0): '
grep -ci "large angular variance" "$TMP/rtab.log"
printf '  里程计话题的发布者(应为 1,即 relay;>1 说明 rtabmap 的视觉里程计误开): '
timeout 10 ros2 topic info /odom 2>&1 | grep -i "publisher count" | tail -1

say "7/7 Orin: /scan + 安全监视器 + Nav2"
if [ "$MODE" = "rehearsal" ]; then
    DRY=true; AUTO=true;  echo "  (演练模式:dry_run=true autostart=true —— 会选点但不驱动底盘)"
else
    DRY=true; AUTO=false; echo "  (默认:dry_run=true autostart=false —— 完全不选点)"
fi
nohup ros2 launch semantic_rtabmap r680_exploration.launch.py \
    include_nav2:=true include_safety:=false \
    dry_run:=$DRY autostart:=$AUTO \
    max_linear_x:=$MAX_LIN max_linear_y:=$MAX_LAT max_angular_z:=$MAX_ANG \
    > "$TMP/explore.log" 2>&1 &
sleep 45

say "Nav2 生命周期(必须全部 active)"
for n in controller_server planner_server bt_navigator behavior_server waypoint_follower; do
    printf '  %-20s ' "$n"
    timeout 10 ros2 lifecycle get "/$n" 2>&1 | tail -1
done

say "前置条件检查"
echo "  话题:"
for t in /scan /map /rtabmap/map /cmd_vel_nav /cmd_vel_safe \
         /safety/localization_valid /safety/obstacle_stop /exploration/goal /battery_voltage; do
    printf '    %-30s ' "$t"
    timeout 8 ros2 topic info "$t" 2>&1 | tr '\n' ' ' | sed 's/Type: //' | head -c 76; echo
done
echo
printf '  localization_valid '; LV=$(timeout 10 ros2 topic echo --once /safety/localization_valid 2>&1 | grep data | head -1)
echo "$LV"
printf '  obstacle_stop      '; OS=$(timeout 10 ros2 topic echo --once /safety/obstacle_stop 2>&1 | grep data | head -1)
echo "$OS"
printf '  battery_voltage    '; BV=$(timeout 10 ros2 topic echo --once /battery_voltage 2>&1 | grep data | head -1)
echo "$BV"
printf '  min_battery(阈值)  '; timeout 8 ros2 param get /r680_explorer min_battery 2>&1 | tail -1
printf '  dry_run            '; timeout 8 ros2 param get /r680_explorer dry_run 2>&1 | tail -1

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
