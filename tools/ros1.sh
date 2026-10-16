#!/usr/bin/env bash
# 在 Orin 上跑 ROS 1 命令,直连 TX1 的机器人。
#
#   bash tools/ros1.sh rostopic list
#   bash tools/ros1.sh rosnode list
#   bash tools/ros1.sh rostopic echo -n 1 /PowerVoltage
#   bash tools/ros1.sh rostopic info /cmd_vel
#
# Orin 主机上没有 ROS 1,ROS 1 只在 ros:noetic 容器(xtkz)里,所以这个脚本
# 做三件事:确保容器在跑、设好正确的 ROS 环境、执行你的命令。
#
# 手工做的话等价于:
#   docker start xtkz
#   docker exec -it xtkz bash
#   source /opt/ros/noetic/setup.bash
#   source /home/jetson/VIO/tools/ros1_env.sh
#   <你的命令>
#
# ---------------------------------------------------------------------------
# ⚠️ 安全提醒
#
# TX1 的底盘驱动 /wheeltec_robot **直接订阅** /cmd_vel,不经过 relay,也不
# 经过 tools/r680_gate.sh 那个闸门(实测 Publishers: None / Subscribers:
# /wheeltec_robot)。所以在这个环境里发布 /cmd_vel 会直接驱动底盘。
#
# 本脚本不阻止你这么做 —— 它是个通用通道。要让它不动,就别发 /cmd_vel。
#
# 不要加 `set -u`。

CONTAINER="${ROS1_CONTAINER:-xtkz}"

if [ $# -eq 0 ]; then
    echo "用法: bash tools/ros1.sh <ROS 1 命令> [参数...]" >&2
    echo "例如: bash tools/ros1.sh rostopic list" >&2
    exit 2
fi

# 容器没跑就起来。已在跑则不动它(可能是你自己起的)。
if [ "$(docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null)" != "running" ]; then
    echo "  启动容器 $CONTAINER ..." >&2
    docker start "$CONTAINER" >/dev/null 2>&1 || {
        echo "✗ 无法启动容器 $CONTAINER" >&2
        docker ps -a 2>/dev/null | sed 's/^/    /' >&2
        exit 1
    }
    sleep 3
fi

# 在容器里:source ROS 1 + 我的环境脚本,再执行用户的命令。
# 环境脚本失败就让整条命令失败 —— 不要在错的环境里跑命令,那正是本次故障的成因。
docker exec "$CONTAINER" bash -c '
source /opt/ros/noetic/setup.bash 2>/dev/null || { echo "✗ 容器里没有 /opt/ros/noetic/setup.bash" >&2; exit 1; }
source /home/jetson/VIO/tools/ros1_env.sh >/dev/null 2>&1 || {
    echo "✗ ROS 环境建立失败 —— 拒绝在错误的环境里执行命令:" >&2
    source /home/jetson/VIO/tools/ros1_env.sh
    exit 1
}
exec "$@"
' _ "$@"
