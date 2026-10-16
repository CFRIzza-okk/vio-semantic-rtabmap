#!/usr/bin/env bash
# 在 Orin 上建立正确的 ROS 1 环境(the robot's ROS 1 master)。
#
#   source tools/ros1_env.sh
#
# 为什么需要它:本项目已经在这件事上栽过一次。2026-10-15 的报错是
#
#   RLException: Unable to contact my own server at [http://192.168.1.102:40071/]
#
# 根因不是网络故障,而是 **shell 里留着旧 WiFi 网段的 ROS 环境变量**:
#
#   export ROS_MASTER_URI=http://192.168.1.102:11311
#   export ROS_IP=192.168.1.102
#
# `192.168.1.102` 是 Orin 在旧 WiFi(XTKZ 网段)上的地址。WiFi 换成了
# Innovation_Lab(192.168.5.x)之后,这个地址不再存在,于是 roslaunch 把
# 自己宣传成一个自己没有的地址,再去联系"自己"时必然失败。
#
# 报错里那句 `ping 192.168.1.102` 就是 ROS 在提示你:去 ping 一下它认为
# 自己应该在的地址 —— 而它会 100% 丢包。
#
# **这类故障的形态值得记住:ROS 的 IP 配置是"环境变量",不是"配置检查"。**
# 地址错了不会在启动时被拦住,只会在一连串后续操作里表现成各种奇怪症状。
#
# 本脚本用当前实际地址重新导出,并在设置后**主动验证**,而不是假设。
#
# ---------------------------------------------------------------------------
# ⚠️ 安全提醒 —— 在拿到车之前必读
#
# TX1 的底盘驱动 /wheeltec_robot **直接订阅** /cmd_vel,不经过 relay,
# 也不经过 tools/r680_gate.sh 那个闸门。
#
#   实测 rostopic info /cmd_vel:
#     Publishers:  None            <- 闸门关闭时确实没人发布
#     Subscribers: /wheeltec_robot <- 但驱动一直在听
#
# 所以:**任何在 TX1 的 ROS 1 master 上发布 /cmd_vel 的节点都会驱动底盘。**
# 闸门只挡住"Orin 的 relay 发布",挡不住 teleop、挡不住 TX1 自带的导航栈、
# 挡不住任何 ROS 1 脚本。
#
# 车在充电、或你不打算让它动的时候,不要在 ROS 1 侧发布 /cmd_vel。
#
# 不要加 `set -u`。

ROBOT_IP=192.168.2.102          # TX1,ROS 1 master
ROBOT_PORT=11311
MY_IFACE=eno1                   # 接到机器人的那条网线

# 取本机在这条链路上的实际地址。不用 hostname -I:它会返回所有接口,
# 可能挑中 WiFi 地址,而那条路 TX1 走不通。
MY_IP=$(ip -4 addr show "$MY_IFACE" 2>/dev/null | grep -oP 'inet \K[0-9.]+' | head -1)

if [ -z "$MY_IP" ]; then
    echo "✗ 取不到 $MY_IFACE 的地址 —— 网线没插?" >&2
    echo "  当前接口:" >&2
    ip -4 addr show 2>/dev/null | grep -E "^[0-9]+:|inet " | sed 's/^/    /' >&2
    echo "  (没有 source 任何东西,环境保持原样)" >&2
    return 1 2>/dev/null || exit 1
fi

# 顺手指出旧的残留变量,因为它们正是上一次故障的根源
STALE=$(env | grep -E "^ROS_(IP|HOSTNAME|MASTER_URI)=" | grep -v "192.168.2.102\|$MY_IP")
if [ -n "$STALE" ]; then
    echo "  注意:覆盖掉旧的 ROS 变量(它们指向已失效的网段):" >&2
    echo "$STALE" | sed 's/^/    /' >&2
fi

unset ROS_HOSTNAME            # 与 ROS_IP 同时存在时行为依赖实现,只留一个
export ROS_MASTER_URI="http://${ROBOT_IP}:${ROBOT_PORT}"
export ROS_IP="$MY_IP"

echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  ROS_IP         = $ROS_IP"

# 验证而不是假设:master 端口必须真的通
if timeout 5 bash -c "cat < /dev/null > /dev/tcp/${ROBOT_IP}/${ROBOT_PORT}" 2>/dev/null; then
    echo "  ✓ master 可达"
else
    echo "  ✗ ${ROBOT_IP}:${ROBOT_PORT} 连不上 —— TX1 上的 roscore 没起?" >&2
    echo "    在 TX1 上启动: roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch" >&2
    return 1 2>/dev/null || exit 1
fi
