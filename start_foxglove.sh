#!/usr/bin/env bash
# ============================================================================
#  start_foxglove.sh —— 把 ROS 2 话题推给另一台电脑上的 Foxglove Studio
#
#  ============================================================================
#  为什么用它，而不是 RViz
#  ============================================================================
#  实测：在这台 Orin NX 上跑 RViz 看建图，RViz 自己要吃掉一整个 CPU 核
#  （实测 84% CPU，整机只有 6 核），**但还是追不上点云的发布速度**，
#  消息过滤器不停地把数据丢掉 —— 一次运行里丢了 7673 条消息。
#  结果就是界面看着像卡死/定格：不是它不画，是它把数据全扔了。
#
#  Foxglove 把「渲染」放到另一台电脑上，Jetson 这边只负责把消息序列化后
#  经 WebSocket 发出去，不画一个像素。这是这台算力紧张的机器上最合适的
#  可视化方案，也是答辩演示时最稳的：观众电脑上看，Jetson 专心建图。
#
#  ============================================================================
#  用法
#  ============================================================================
#    第 1 步（只需做一次）：装 foxglove_bridge
#        sudo apt update && sudo apt install -y ros-humble-foxglove-bridge
#
#    第 2 步：正常启动建图（建议 --no-rviz）
#        cd ~/VIO && ./run_once.sh --no-rviz
#
#    第 3 步：另开一个终端，启动本脚本
#        cd ~/VIO && ./start_foxglove.sh
#
#    第 4 步：在【另一台电脑】上打开 Foxglove Studio，连接
#        ws://192.168.5.4:8765
#      （IP 以本脚本启动时打印的为准，换网络会变）
#
#  ⚠ 强烈建议用 Foxglove 的**桌面版**，不要用网页版。
#    网页版是 https 页面，浏览器会以"混合内容"为由拦掉 ws:// 连接
#    （只有连 localhost 才放行）。桌面版没有这个限制。
#
#  ============================================================================
#  连上之后看什么
#  ============================================================================
#  在 Foxglove 里加一个 3D 面板，Fixed Frame 设成 `map`，然后加这些话题：
#
#    /rtabmap/cloud_map        点云地图 —— ⭐ 主角，看建图效果
#    /rtabmap/map              二维占据栅格（Nav2 用这个）
#    /tf  /tf_static           坐标轴
#
#  ⚠ 这里原先还列了两个话题，实测**根本不存在**，照着找会白费功夫：
#      /rtabmap/odom_local_map  —— 不在当前话题表里
#      /semantics/map_markers   —— 不在当前话题表里
#    第二个尤其要注意：本包叫 semantic_rtabmap，但这个语义标记话题**没有产出**，
#    也就是说语义建图这一路当前没有输出。这是产品问题，不是可视化配置问题。
#    要用 2026-10-15 实测存在的话题清单，见下面的分组。
#
#  ⚠ 先澄清一个常见误解：本脚本**不需要**为任何话题做“加入转发”的配置。
#    foxglove_bridge 的默认 topic_whitelist 就是 [".*"]，即**所有话题一律转发**，
#    包括后加的 /battery_voltage、/safety/*、/cmd_vel_*、/scan、/exploration/*。
#    所以新话题在 Foxglove 里看不见，原因一定是“面板里没有添加这个频道”，
#    而不是“bridge 没转发它”。真正要做的事是在下面这份清单里挑，然后在
#    Foxglove Studio 的 3D / Plot 面板里手动加频道。
#
#  R680 麦轮车 / 自主探索（Fixed Frame 建议设 map）：
#    /scan                      深度转出的激光扫描 —— 看障碍点是否贴住墙线
#    /odom                      轮式里程计（与 /tf 里 map->odom 对比看漂移）
#    /battery_voltage           TX1 电池电压，std_msgs/Float32 ⚠ 见下
#    /cmd_vel_nav               Nav2 控制器/行为树的原始输出（未经限速）
#    /cmd_vel_safe              安全节点限速后的速度 —— 真正发给底盘的那个
#    /safety/obstacle_stop      障碍停机（bool，true=停）
#    /safety/localization_valid 定位是否有效（bool）
#    /safety/stop_reason        停机原因字符串 —— **车不动时先看这个**
#    /exploration/status        探索状态，含被拒绝目标的原因
#    /exploration/goal          当前目标点（geometry_msgs/PoseStamped）
#
#  完整清单（2026-10-15 用 `ros2 topic info` 实测，括号内是发布者数量；
#  发布者为 0 的话题加了也看不到数据，已单独标出）：
#
#  建图 / 地图（3D 面板）
#    /rtabmap/cloud_map                  ⭐ 点云地图（1）
#    /rtabmap/cloud_obstacles               障碍点云，已与地面分离（1）
#    /rtabmap/cloud_ground                  地面点云（1）
#    /rtabmap/map                           二维占据栅格（1）
#    /rtabmap/octomap_occupied_space        3D 占据体素（1）
#    /rtabmap/octomap_grid                  octomap 二维投影（1）
#    /rtabmap/local_grid_obstacle           局部障碍栅格（1）
#    /rtabmap/labels                        RTAB-Map 标签 MarkerArray（1）
#
#  导航路径（3D 面板）—— 排查“车为什么不动/怎么绕”的关键
#    /rtabmap/global_path               ⭐ Nav2 规划的全局路径（1）
#    /rtabmap/local_path                    局部路径（1）
#    /rtabmap/goal                          当前 Nav2 目标（1）
#    /rtabmap/goal_reached                  是否到达目标，bool（1）
#
#  定位（3D 面板）
#    /rtabmap/localization_pose         ⭐ RTAB-Map 认为车在哪（1）
#    /odom                                  轮式里程计，与 map->odom 对比看漂移（1）
#    /rtabmap/global_pose                   ⚠ 发布者为 0，别加（用 localization_pose）
#
#  速度与安全（Plot / Raw Message 面板）
#    /cmd_vel_nav                           Nav2 原始输出，未经限速（1）
#    /cmd_vel_safe                      ⭐ 限速后真正发给底盘的速度（1）
#    /safety/obstacle_stop                  障碍停机，bool（1）
#    /safety/localization_valid             定位是否有效，bool（1）
#    /safety/stop_reason                 ⭐ 停机原因字符串 —— 车不动时先看这个（1）
#    /safety/estop                      ⚠ 发布者为 0。它是**输入**话题（紧急停止的
#                                           接收端），不是状态显示，加了看不到东西
#
#  自主探索
#    /exploration/status                    状态 + 被拒目标的原因（1）
#    /exploration/state                     状态机原始状态字符串（1）
#    /exploration/goal                      当前目标点 PoseStamped（1）
#
#  传感器
#    /scan                                  深度转出的激光扫描，看障碍点是否贴住墙线（1）
#    /camera/camera/color/image_raw     ⭐ 彩色画面，Image 面板（1）
#    /camera/camera/depth/color/points      相机原始点云（1）
#    /battery_voltage                       电池电压，见下方警告（1）
#    /imu/data                          ⚠ 发布者为 0
#
#  系统健康
#    /diagnostics                       ⭐ 各节点健康状态（1）
#    /rtabmap/info                          建图统计：节点数、回环次数（1）
#
#  【反向】在 Foxglove 里给车下指令（3D 面板的 Publish 按钮）
#    /goal_pose                         ⭐ Nav2 标准目标点入口，发 PoseStamped
#    /move_base_simple/goal                 RViz 风格目标点
#    /initialpose                           手动重设初始位姿
#    /clicked_point                         点选一个坐标
#
#  ⚠ /battery_voltage 是裸 std_msgs/Float32，**没有 header、没有时间戳**。
#    Foxglove 只能拿“收到消息的时刻”当横轴，所以曲线在时间轴上会有抖动，
#    数值本身是准的（实测 11.86 V）。它请用 Plot 面板看，不要去 3D 面板里找 ——
#    没有 frame 的消息在 3D 视图里无处安放。
#
#  ⚠ 注意：RViz 配置里那几个 rtabmap 专用显示（MapCloud / MapGraph）在
#    Foxglove 里**显示不出来** —— 它们用的是 rtabmap 自己的插件和自定义
#    消息类型（rtabmap_msgs/MapData、MapGraph），Foxglove 不认识。
#    要看点云必须用上面那个标准的 `/rtabmap/cloud_map`（sensor_msgs/PointCloud2）。
#    MapGraph（回环连线）在 Foxglove 里没有对应显示，看回环请直接看日志里的
#    "回环检测: N 次"，或者用 RViz 事后回看。
# ============================================================================

set -uo pipefail
set +u   # ROS 的 setup.bash 里有未定义变量

source /opt/ros/humble/setup.bash

# ⚠ 必须再 source 语义工作空间，否则 bridge 不认识我们自定义的消息类型。
#   漏了这一步的典型报错（实测踩过）：
#     Failed to load schemaDefinition for topic "/semantics/objects_tracked"
#     (semantic_rtabmap_msgs/msg/SemanticObjectArray): package
#      'semantic_rtabmap_msgs' not found, searching: [...]
#   原因是 foxglove_bridge 通过 AMENT_PREFIX_PATH 去找消息定义，而
#   /opt/ros/humble 里没有我们自己的包。后果是 Foxglove 里那几个语义话题
#   显示不出来（**点云不受影响** —— /rtabmap/cloud_map 是标准
#   sensor_msgs/PointCloud2，不需要任何自定义定义）。
#   顺序很重要：/opt/ros/humble 必须在前，语义工作空间在后。
SEMANTIC_WS="$HOME/VIO/semantic_ws/install/setup.bash"
if [ -f "$SEMANTIC_WS" ]; then
  source "$SEMANTIC_WS"
else
  echo "[警告] 未找到 $SEMANTIC_WS"
  echo "       语义话题（/semantics/*）在 Foxglove 里会解码失败，点云仍可用。"
  echo "       若尚未编译：cd ~/VIO/semantic_ws && colcon build --symlink-install --parallel-workers 1"
fi

PORT="${PORT:-8765}"

# ---------------- 前置检查 ----------------
if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "============================================================"
  echo "  foxglove_bridge 尚未安装"
  echo "============================================================"
  echo ""
  echo "  请先执行（需要 sudo 密码，所以只能你自己来）："
  echo ""
  echo "      sudo apt update && sudo apt install -y ros-humble-foxglove-bridge"
  echo ""
  echo "  装完再运行本脚本。"
  echo ""
  echo "  说明：这个包在 apt 源里是现成的（ros-humble-foxglove-bridge 3.5.0，"
  echo "        约 12MB，依赖都是标准 ROS 2 Humble 组件），不需要编译。"
  exit 1
fi

# 消息定义检查：没有它，语义话题在 Foxglove 里解码不出来。
# 这里显式检查一次，而不是等 bridge 启动后刷一屏 WARN/ERROR —— 那时人往往
# 会以为是 bridge 坏了，其实是环境没 source 对。
if ros2 pkg prefix semantic_rtabmap_msgs >/dev/null 2>&1; then
  echo "[OK] 语义消息包已找到: $(ros2 pkg prefix semantic_rtabmap_msgs)"
else
  echo "[警告] 找不到 semantic_rtabmap_msgs！语义话题将无法在 Foxglove 里显示。"
  echo "       （点云 /rtabmap/cloud_map 不受影响，它是标准类型，仍可正常显示）"
  echo "       请确认已 source ~/VIO/semantic_ws/install/setup.bash 且工作空间已编译。"
fi
echo ""

# ---------------- 打印本机地址，方便对端连接 ----------------
echo "============================================================"
echo "  Foxglove Bridge 启动中"
echo "============================================================"
echo ""
echo "  端口: $PORT"
echo ""
echo "  在另一台电脑的 Foxglove Studio（桌面版）里连接以下地址："
echo ""
# 只列出真实网卡的地址，跳过 docker0 之类的虚拟网卡
ip -4 addr show 2>/dev/null | awk '/inet /{print $2}' | grep -v '^127\.' | grep -v '^172\.17\.' | while read -r cidr; do
  ip_only="${cidr%%/*}"
  echo "      ws://${ip_only}:${PORT}"
done
echo ""
echo "  提示：对端电脑要和这台机器在【同一个局域网】。"
echo "        本脚本启动时会打印实际可用的地址，以那个为准。"
echo ""
echo "  按 Ctrl+C 停止"
echo "============================================================"
echo ""

# ---------------- 启动 ----------------
# 故意只用 port / address 两个参数：
#   这两个是 foxglove_bridge 最稳定的接口。其它调优参数（topic_whitelist、
#   max_qos_depth、send_buffer_limit 等）不同版本名字有差异，写错了会导致
#   节点直接起不来。等确认能跑通之后，再按需要加上去调优。
#
# address 用 0.0.0.0 是必须的：默认只监听本地的话，另一台电脑连不上。
exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p port:="$PORT" \
  -p address:=0.0.0.0
