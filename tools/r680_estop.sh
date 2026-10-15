#!/usr/bin/env bash
# 急停。一条命令,按"最快且最可靠"的顺序做三件事。
#
#   bash tools/r680_estop.sh            # 停(锁存急停 + 关 TX1 闸门)
#   bash tools/r680_estop.sh --resume   # 解除急停(不自动重新开闸)
#
# 为什么是三件事而不是一件:每一件都有它单独挡不住的失效情形。
#
#   1. /safety/estop 锁存  —— 让 cmd_vel_safety 每 tick 都输出零速。快,但只要
#      Orin 还活着才有效。
#   2. 关 TX1 闸门        —— 让 TX1 relay 不再有 /cmd_vel 发布者。即使 Orin 或
#      网络彻底失控也有效,因为这一层在 TX1 本地。
#   3. 确认              —— 查 /cmd_vel 真实发布者列表。**不查就不知道有没有
#      真停下来**,而"以为停了"和"停了"是两件不同的事(见本项目缺陷 9)。
#
# 物理断电永远是最快的一层,手应该一直在开关附近。
#
# 不要加 `set -u`。

WS=/home/jetson/VIO/semantic_ws
RESUME=0
[ "$1" = "--resume" ] && RESUME=1

if [ "$RESUME" = "1" ]; then
    cd "$WS" || exit 1
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source install/setup.bash >/dev/null 2>&1
    export ROS_DOMAIN_ID=99
    echo "=== 解除急停锁存 ==="
    timeout 20 ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: false}" 2>&1 | tail -1
    echo "  ⚠️  锁存已解除,但 TX1 闸门**仍然是关的**。"
    echo "      要恢复运动: bash tools/r680_gate.sh open"
    exit 0
fi

echo "=== 急停 ==="

echo "[1/3] 锁存 /safety/estop = true(让 cmd_vel_safety 持续输出零速)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=99
# 发三次:发现可能还没完成,一次可能丢。
for i in 1 2 3; do
    timeout 12 ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
done
echo "      已发 3 次"

echo "[2/3] 关闭 TX1 运动闸门(即使 Orin 或网络失控也有效)"
bash /home/jetson/VIO/tools/r680_gate.sh close 2>&1 | tail -3

echo "[3/3] 确认底盘当前收到的速度"
timeout 60 python3 /home/jetson/tx1.py 'source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311; export ROS_IP=192.168.2.102
echo "  /cmd_vel 发布者(应为 None):"
timeout 8 rostopic info /cmd_vel 2>&1 | sed -n "/Publishers/,/Subscribers/p" | head -4
echo "  最近一条速度指令:"
timeout 8 rostopic echo -n 1 /cmd_vel 2>&1 | grep -E "x:|z:" | head -4
echo "  当前里程计(若车没动,应和急停前一致):"
timeout 8 rostopic echo -n 1 /odom/pose/pose/position 2>&1 | head -4' 2>&1 | tail -14

echo
echo "=== 急停完成 ==="
echo "  恢复: bash tools/r680_estop.sh --resume && bash tools/r680_gate.sh open"
echo "  若车仍在动,立刻物理断电 —— 手放在开关上。"
