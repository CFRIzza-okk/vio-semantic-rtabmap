#!/usr/bin/env bash
# 阶段 E1：一次受监督的直行脉冲。
#
# 设计要点（每一条都是为了“出问题时能立刻停、并且事后能查”）：
#   * 脉冲很短（默认 2 s）且很慢（默认 0.06 m/s）—— 约 12 cm
#   * **补零写在脚本里**，不依赖人的手速：脉冲结束的同一条命令流里立刻发零速
#   * 前后各录一次 /odom，用来判断“动了多少、方向对不对”
#   * 同时录 /cmd_vel_safe（Orin 侧闸门输出）与 TX1 /cmd_vel（最终到达底盘的值），
#     这样即使车没动，也能看出信号卡在哪一级
#
# 用法: bash tools/r680_motion_pulse.sh [linear_x] [seconds] [angular_z]
#   默认  0.06 2 0.0
# 不要加 `set -u`。

WS=/home/jetson/VIO/semantic_ws
TMP=/tmp/r680_motion
LIN=${1:-0.06}
SECS=${2:-2}
ANG=${3:-0.0}

mkdir -p "$TMP"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99

ZERO='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
CMD="{linear: {x: $LIN, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANG}}"

echo "=== 本次脉冲: linear.x=$LIN  angular.z=$ANG  时长=${SECS}s ==="
echo "    预计位移约 $(python3 -c "print('%.3f' % ($LIN*$SECS))") m"

# 安全前置：定位必须有效、前方必须无障碍，否则不发
LOC=$(timeout 8 ros2 topic echo --once /safety/localization_valid 2>/dev/null | grep -c "data: true")
OBS=$(timeout 8 ros2 topic echo --once /safety/obstacle_stop 2>/dev/null | grep -c "data: true")
if [ "$LOC" != "1" ]; then echo "!! localization_valid 不是 true，拒绝发指令"; exit 1; fi
if [ "$OBS" != "0" ]; then echo "!! obstacle_stop 是 true，拒绝发指令"; exit 1; fi
echo "    前置检查通过（定位有效、无障碍）"

# ---- 开始记录 ----
echo "--- /odom 之前 ---"
timeout 10 ros2 topic echo --once /odom/pose/pose/position 2>/dev/null | head -3

nohup timeout $((SECS+6)) ros2 topic echo /cmd_vel_safe > "$TMP/safe.log" 2>&1 &
SAFE_PID=$!
timeout 90 python3 /home/jetson/tx1.py "source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
timeout $((SECS+6)) rostopic echo /cmd_vel > /tmp/tx1_cmdvel.log 2>&1
echo recorded" > /dev/null 2>&1 &
sleep 4

# ---- 发脉冲 ----
echo "=== 发脉冲 ==="
timeout "$SECS" ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist "$CMD" > /dev/null 2>&1
echo "  脉冲结束，立刻补零速"
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist "$ZERO" > /dev/null 2>&1
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist "$ZERO" > /dev/null 2>&1
echo "  已补零速两次"

sleep 6
wait $SAFE_PID 2>/dev/null

# ---- 结果 ----
echo
echo "--- Orin /cmd_vel_safe 采样（非零的行）---"
grep -E "x: -?[0-9]" "$TMP/safe.log" 2>/dev/null | grep -vE "x: 0\.0$" | head -8
echo "  (总行数: $(wc -l < "$TMP/safe.log" 2>/dev/null))"

echo
echo "--- TX1 /cmd_vel 采样 ---"
timeout 60 python3 /home/jetson/tx1.py 'cat /tmp/tx1_cmdvel.log 2>/dev/null | head -40' 2>&1 | tail -30

echo
echo "--- /odom 之后 ---"
timeout 10 ros2 topic echo --once /odom/pose/pose/position 2>/dev/null | head -3

echo
echo "=== 兜底：确保最后一条指令是零速 ==="
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "$ZERO" > /dev/null 2>&1
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist "$ZERO" > /dev/null 2>&1
echo "done"
