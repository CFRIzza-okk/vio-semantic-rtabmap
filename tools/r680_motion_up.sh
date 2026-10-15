#!/usr/bin/env bash
# R680 第一次运动（阶段 E1）的整栈启动 —— 地面直行测试用。
#
# 与 r680_phase_d_up.sh 的差别只有三处，其余步骤逐字相同（相机、RTAB-Map、
# 探索/安全检查节点的调用方式都是已在阶段 D 验证过的）：
#   1. TX1 relay 带 _enable_cmd_vel:=true —— 这是**唯一**打开运动闸门的地方
#   2. TX1 relay 与 cmd_vel_safety 的限速都收到 0.08 / 0.0 / 0.3
#   3. 完事后打印闸门状态，确认它确实是开着的（因为这脚本的全部意义就是打开它）
#
# 为什么需要整栈：cmd_vel_safety 有 require_localization=true 与
# stop_on_obstacle=true，定位无效或有障碍时它**拒绝转发**速度。所以哪怕只是
# 走一条直线，也得让相机、RTAB-Map、obstacle_monitor、localization_monitor 全在线。
#
# 本脚本**不发任何速度**。发指令是单独的一步，见 CLAUDE 的说明与铁律：
#   每次用 Ctrl-C 停掉连续发布后，必须立刻补一条零速。
#
# 用法: bash tools/r680_motion_up.sh
# 不要加 `set -u`（ROS 的 setup.bash 会踩到未定义变量）。

WS=/home/jetson/VIO/semantic_ws
TMP=/tmp/r680_motion
mkdir -p "$TMP"

MAX_LIN=0.08
MAX_LAT=0.0
MAX_ANG=0.3

say() { printf '\n=== %s ===\n' "$1"; }

say "限速（本脚本会强制成这一组）"
printf '  max_linear_x=%s  max_linear_y=%s  max_angular_z=%s\n' "$MAX_LIN" "$MAX_LAT" "$MAX_ANG"

# ---------------------------------------------------------------- TX1 side
# 清理与启动**必须**是分开的 ssh 调用：同一条命令里既有 pkill 的模式又有
# roslaunch 的明文时，pkill 会匹配到执行它的那个 shell 自己。见 phase_d 脚本里
# 更长的说明。
say "1/6 TX1: 清理旧进程"
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
pkill -f "[r]os1_relay" 2>/dev/null
pkill -f "[w]heeltec_robot" 2>/dev/null
pkill -f "[r]oscore" 2>/dev/null
sleep 3
echo "cleaned"' 2>&1 | tail -2

say "1/6 TX1 驱动"
timeout 120 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
source /home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
cd /home/wheeltec
nohup roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch > /home/wheeltec/driver.log 2>&1 &
sleep 28
timeout 10 rosnode list 2>&1 | head -10' 2>&1 | tail -12

say "2/6 TX1 relay —— 打开运动闸门 (enable_cmd_vel=true)"
timeout 80 python3 /home/jetson/tx1.py "source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
nohup python2 /home/wheeltec/ros1_relay.py _peer_host:=192.168.2.15 _enable_cmd_vel:=true _max_linear_x:=$MAX_LIN _max_linear_y:=$MAX_LAT _max_angular_z:=$MAX_ANG > /home/wheeltec/relay.log 2>&1 &
sleep 6
echo '--- relay pid ---'
ps -eo pid,args | grep '[r]os1_relay' | head -2
echo '--- relay log ---'
tail -4 /home/wheeltec/relay.log 2>/dev/null
echo '--- /cmd_vel 现在应该**有**发布者了 ---'
timeout 8 rostopic info /cmd_vel 2>&1 | head -5" 2>&1 | tail -14

    # odom_frame_id 必须显式设成 relay 实际发布的那个 frame(odom_combined)。
    #
    # 留空不是"安全默认",而是致命的:RTAB-Map 要发布 map->odom 这个 TF,
    # 就必须知道 odom frame 叫什么。空值时它**一个 map->* 的 TF 都不发**,
    # 于是 map 这个 frame 在整棵 TF 树里根本不存在。
    #
    # 后果不是"定位差一点",而是**Nav2 完全起不来**:global_costmap 会一直打印
    #   Invalid frame ID "map" passed to canTransform argument target_frame
    # 然后卡住无法激活,lifecycle_manager 的启动顺序被阻塞,于是 bt_navigator、
    # behavior_server、waypoint_follower 全部停在 inactive —— 只剩
    # controller_server 是 active。现象是"Nav2 起了一半",根因在一个空字符串。
    #
    # Phase D 从未暴露它,因为 Phase D 不启动 Nav2。
    #
    # 注意这与"不要开 visual_odometry"是两件不同的事:这里设的是 RTAB-Map
    # **相信**哪个 frame 是里程计坐标系,不是让它自己算里程计。visual_odometry
    # 仍然必须是 false,否则 rgbd_odometry 会再发一个 /odom,与 relay 打架。

# ---------------------------------------------------------------- Orin side
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

say "3/6 Orin relay"
nohup ros2 launch semantic_rtabmap r680_relay.launch.py > "$TMP/relay.log" 2>&1 &
sleep 10
timeout 10 ros2 topic info /odom 2>&1 | head -2

say "4/6 camera"
nohup ros2 launch semantic_rtabmap r680_depth_cloud.launch.py \
    start_inspector:=false > "$TMP/cam.log" 2>&1 &
sleep 22
timeout 10 ros2 topic info /camera/camera/aligned_depth_to_color/image_raw 2>&1 | head -2

say "5/6 static TF + cmd_vel_safety（放宽到极限 0.08/0.0/0.3）"
nohup ros2 launch semantic_rtabmap r680_navigation.launch.py \
    publish_base_tf:=false \
    max_linear_x:=$MAX_LIN max_linear_y:=$MAX_LAT max_angular_z:=$MAX_ANG \
    > "$TMP/nav.log" 2>&1 &
sleep 12
for p in max_linear_x max_linear_y max_angular_z command_timeout; do
  printf '  cmd_vel_safety %-16s ' "$p"
  timeout 8 ros2 param get /cmd_vel_safety "$p" 2>&1 | tail -1
done

say "6/6 RTAB-Map"
rm -f "$HOME/.ros/rtabmap_motion.db"
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
    "database_path:=$HOME/.ros/rtabmap_motion.db" > "$TMP/rtab.log" 2>&1 &
sleep 38
echo -n "  angular-variance warnings: "
grep -ci "large angular variance" "$TMP/rtab.log"

say "scan + safety monitors（探索启动文件，dry_run，不发目标）"
nohup ros2 launch semantic_rtabmap r680_exploration.launch.py \
    include_nav2:=false include_safety:=false dry_run:=true \
    > "$TMP/explore.log" 2>&1 &
sleep 20

say "运动前的必要条件（缺一不可）"
for t in /scan /cmd_vel_nav /cmd_vel_safe /safety/localization_valid /safety/obstacle_stop; do
  printf '  %-32s ' "$t"
  timeout 8 ros2 topic info "$t" 2>&1 | tr '\n' ' ' | sed 's/Type: //' | head -c 88; echo
done
echo
echo "  localization_valid（必须是 True，否则 cmd_vel_safety 不转发）:"
timeout 10 ros2 topic echo --once /safety/localization_valid 2>&1 | head -2
echo "  obstacle_stop（必须是 False）:"
timeout 10 ros2 topic echo --once /safety/obstacle_stop 2>&1 | head -2
echo "  电池（守卫在线）:"
timeout 10 ros2 topic echo --once /battery_voltage 2>&1 | head -2

say "闸门最终状态"
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
echo "  TX1 /cmd_vel:"
timeout 8 rostopic info /cmd_vel 2>&1 | head -4
echo "  /odom 位置（应仍在原点附近）:"
timeout 8 rostopic echo -n 1 /odom/pose/pose/position 2>&1 | head -4' 2>&1 | tail -12

echo
echo "日志在 $TMP/"
echo "下一步由人来发速度指令；停掉连续发布后必须立刻补零速。"
