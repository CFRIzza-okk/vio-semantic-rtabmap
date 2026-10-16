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
# 取本机地址的方式,由内到外三层,都不假设有某个特定命令
#
# 首选是问内核:"要到 ROBOT_IP 那个地址,我会用哪个本机地址?" —— 一个连向
# 目标地址的 UDP socket 的 getsockname() 就是答案。这比读接口列表**更准**:
# 它自动选中能路由到机器人的那条路。实测在 Orin 上它给出 eno1 的地址而不是
# WiFi 的 —— 而 WiFi 那条路 TX1 是走不通的。用 `hostname -I` 或"取第一个
# 非 lo 接口"都可能挑中 WiFi 地址,那是错的。
#
# 之所以要这么绕:ros:noetic 容器里**没有 `ip` 命令**(实测),
# 而那个容器正是本项目使用 ROS 1 的地方。脚本必须在容器里也能用。
# 我第一版写死了 `ip -4 addr show eno1`,在容器里直接失败。
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

ROBOT_IP="${ROBOT_IP:-192.168.2.102}"   # TX1,ROS 1 master
ROBOT_PORT="${ROBOT_PORT:-11311}"

_find_my_ip() {
    # 第 1 层:让内核回答"去机器人我用哪个地址"。选中的一定是能路由过去的那条路。
    local ip
    ip=$(python3 -c "
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.connect(('${ROBOT_IP}', ${ROBOT_PORT}))
    print(s.getsockname()[0])
except Exception:
    sys.exit(1)
finally:
    s.close()
" 2>/dev/null)
    if [ -n "$ip" ] && [ "$ip" != "0.0.0.0" ]; then echo "$ip"; return 0; fi

    # 第 2 层:ip 命令(宿主机上有,容器里没有)
    if command -v ip >/dev/null 2>&1; then
        ip=$(ip route get "$ROBOT_IP" 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1)
        if [ -n "$ip" ]; then echo "$ip"; return 0; fi
    fi

    # 第 3 层:ifconfig(更老的系统)
    if command -v ifconfig >/dev/null 2>&1; then
        ip=$(ifconfig 2>/dev/null | grep -B1 -A1 "inet " | grep -oP 'inet \K192\.168\.[0-9.]+' | head -1)
        if [ -n "$ip" ]; then echo "$ip"; return 0; fi
    fi

    return 1
}

MY_IP=$(_find_my_ip)

if [ -z "$MY_IP" ]; then
    echo "✗ 取不到通往 ${ROBOT_IP} 的本机地址 —— 网线没插?" >&2
    echo "  三种探测都试过了(python3 socket / ip route / ifconfig)。" >&2
    echo "  可以手工指定: export ROS_IP=<本机地址>; source tools/ros1_env.sh" >&2
    echo "  (没有 source 任何东西,环境保持原样)" >&2
    return 1 2>/dev/null || exit 1
fi

# 顺手指出旧的残留变量,因为它们正是上一次故障的根源
STALE=$(env | grep -E "^ROS_(IP|HOSTNAME|MASTER_URI)=" | grep -v "${ROBOT_IP}\|^ROS_IP=${MY_IP}$")
if [ -n "$STALE" ]; then
    echo "  注意:覆盖掉旧的 ROS 变量(它们可能指向已失效的网段):" >&2
    echo "$STALE" | sed 's/^/    /' >&2
fi

unset ROS_HOSTNAME            # 与 ROS_IP 同时存在时行为依赖实现,只留一个
export ROS_MASTER_URI="http://${ROBOT_IP}:${ROBOT_PORT}"
export ROS_IP="$MY_IP"

echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  ROS_IP         = $ROS_IP"

# 验证而不是假设:master 端口必须真的通
_probe() {
    if command -v python3 >/dev/null 2>&1; then
        python3 -c "
import socket, sys
s = socket.socket()
s.settimeout(5)
try:
    s.connect(('${ROBOT_IP}', ${ROBOT_PORT}))
except Exception:
    sys.exit(1)
finally:
    s.close()
" 2>/dev/null && return 0
    fi
    timeout 5 bash -c "cat < /dev/null > /dev/tcp/${ROBOT_IP}/${ROBOT_PORT}" 2>/dev/null
}

if _probe; then
    echo "  ✓ master 可达"
else
    echo "  ✗ ${ROBOT_IP}:${ROBOT_PORT} 连不上 —— TX1 上的 roscore 没起?" >&2
    echo "    在 TX1 上启动: roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch" >&2
    return 1 2>/dev/null || exit 1
fi
