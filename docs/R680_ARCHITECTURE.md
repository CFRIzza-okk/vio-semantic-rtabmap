# R680 自主探索小车 · 系统架构

本文是分层架构的**文字说明**，配套框图见 `docs/r680_architecture.svg`（矢量）与
`docs/r680_architecture.png`（位图）。两者由同一份几何数据生成：

```bash
python3 tools/make_architecture_diagram.py docs/
```

改图请改 `tools/make_architecture_diagram.py` 里的构图段，不要手工编辑 SVG ——
脚本会在渲染前做几何自检（方块不重叠、不越出泳道与机器分区、连线不穿框、
连线全为直角、标注不压框、文本不溢出），任何一项不过就以非零码退出。

---

## 一、两机异构：为什么这样切

| | TX1 | Orin NX |
|---|---|---|
| ROS | 1 Melodic | 2 Humble（`ROS_DOMAIN_ID=99`） |
| 角色 | 底盘执行 | 感知与决策 |
| 运行 | `wheeltec_robot` 驱动、`ros1_relay.py`、`tx1_power_log.py`、`roscore` | 相机、RTAB-Map、`depthimage_to_laserscan`、Nav2、探索器、两类监视器、`ros2_relay_node` |
| 不跑 | 任何感知节点 | 任何执行器 |

切分依据是**算力**，不是功能。TX1 只负责电机闭环、里程计、IMU、电池采样；凡是
需要图像/点云/图优化的都在 Orin。跨机不是 DDS，而是自研 UDP 中继，端口互补：
Orin 绑 19001 发 19000，TX1 绑 19000 发 Orin 19001，两边配成互补的一对才通。

---

## 二、感知层

**传感器**：RealSense D435（彩色图 + 对齐深度 + 原始点云），USB3 直连 Orin；
可靠深度约 3 m。轮式编码器、IMU、电池采样在 TX1，经中继上行。

**里程计与定位**：视觉里程计**刻意关闭**（`visual_odometry:=false
icp_odometry:=false`），唯一里程计源是 TX1 的轮式 `/odom`。开着 RTAB-Map 自己的
RGB-D 里程计会再发一个 `/odom`，与中继打架。RTAB-Map 负责 SLAM 与定位，发布
`map→odom_combined`、`/rtabmap/map`、`/rtabmap/cloud_map`、
`/rtabmap/localization_pose`。

`odom_frame_id` 必须显式设成 `odom_combined`。留空时 RTAB-Map 一个 `map→*` 都
不发，`map` 帧不存在，Nav2 的 `global_costmap` 卡在 `Invalid frame ID "map"`，
`lifecycle_manager` 被阻塞，`bt_navigator`/`behavior_server`/`waypoint_follower`
全部停在 inactive —— 现象是「Nav2 起了一半」，根因是一个空字符串。

**深度转激光**：`depthimage_to_laserscan` 输出 `/scan`，`scan_height: 5`、
`range_min: 0.25`、`range_max: 6.0`、`output_frame: camera_depth_frame`。

> `output_frame` 不能用 `camera_color_optical_frame`。深度图的数据**表达**在光学系，
> 但 LaserScan 的角度语义要求 **x-y 平面水平**（REP-103）—— 这是两件不同的事。
> 实测 TF：`camera_depth_frame` 与 `base_link` 姿态一致，光学系差约 90°。
> `obstacle_monitor` 直接读角度不做 TF 变换，所以不受影响（这也让错误长期藏着）；
> 但 Nav2 的 costmap 会把 scan 变换到全局系，用光学系就会把障碍摆到竖直方向，
> 在代价地图里造出幽灵障碍，一开 Nav2 就中招。

**语义感知存在，但不在探索回路里**：`detector_node`（YOLO11n，TensorRT，
实测 19 ms/帧 ≈ 52 FPS，`conf 0.35`/`iou 0.45`）做 2D 检测 → ROI 反投影成 3D
语义物体（`roi_ratio 0.6` 回避 depth bleeding，`depth_percentile 20` 偏向前景，
`depth_tolerance 0.35` 做深度一致性聚类）；`mapper_node` 用匈牙利匹配关联，
`min_observations: 3` 才确认。这条链只在 `semantic.launch.py` 里启动，
`r680_exploration.launch.py` 起的 5 个节点里没有它 —— 自主探索当前是纯几何的。

---

## 三、规划层

**前端：frontier 探索**（`r680_explorer`）。状态机
`IDLE → EXPLORING ⇄ PAUSED → COMPLETE / ABORTED`，另有 `DRY_RUN_DONE`
（dry_run 下的专有终态：它表达「候选遍历完了」，不是「世界上没有 frontier 了」，
把两件事混成一个状态等于说假话）。启停是 4 个 `std_srvs/Trigger`：
`/exploration/{start,stop,pause,resume}`，且 `autostart: false`。

```
score = 1.0·size + 1.5·gain − 1.0·dist − 0.6·heading_err/π
```

`gain` 权重最高，即优先「能新看到最多未知区域」而不是最近的。约束：
`min_frontier_size 10`（滤掉 D435 噪声碎片）、`goal_clearance 0.25`、
`require_reachable true`（只考虑与当前自由空间连通的，防隔墙目标）、
`min_goal_distance 0.4` / `max_goal_distance 6.0`（受 3 m 可靠深度限制）。

两类失败记忆，语义不同：`blacklist`（半径 0.8 m，TTL 90 s，「现在不行，等会再试」）
与 `/exploration/unexplorable`（半径 1.5 m，失败 2 次即永久排除，「试过了，去不了」）。
阈值按区域而非坐标精确匹配，因为 frontier 质心每次差几厘米。

护栏：`max_exploration_time 120 s`、`goal_timeout 60 s`、
`max_consecutive_failures 5`、`complete_confirmations 3`、`min_battery 10.5 V` +
`battery_timeout 5 s`（覆盖 relay 挂掉后电压不再更新、却仍拿旧值报「正常」的情形）。

> `r680_exploration.yaml` 里的 `robot_radius` 标注为**当前未被使用**：
> `frontier.py` 与 `exploration_node.py` 都不读它，真正决定碰撞半径的是
> `r680_nav2.yaml` 的 costmap `robot_radius`。

**后端：Nav2**（`controller_server`/`planner_server`/`bt_navigator`/
`behavior_server`/`waypoint_follower`/`lifecycle_manager`）：

- 全局代价地图：`static_layer`（`map_topic: /rtabmap/map`）+ `obstacle_layer`
  （`/scan`）+ `inflation_layer`（`inflation_radius 0.35`），`global_frame: map`
- 局部代价地图：滚动窗口 4×4 m，`global_frame: odom_combined`，观测源**仅 `/scan`**
- 规划器 `NavfnPlanner`（`use_astar: true`）；控制器 `MPPIController`，
  `xy_goal_tolerance 0.20`
- 恢复行为：`Spin`、`BackUp`、`DriveOnHeading`、`AssistedTeleop`、`Wait`

`map_topic` 必须显式写成 `/rtabmap/map`：StaticLayer 不设时默认订阅 `/map`，
而 `/map` 无人发布，代价地图永远是空的。

`robot_radius 0.20` 的由来：车体外缘实测 30×25 cm，外接圆
`0.5·√(0.30²+0.25²) = 0.195 m`，向上取 0.20 留余量 —— 宁可进不去窄缝，
也不能让轮角蹭到。两处 costmap 必须同值。

局部代价地图**没有 voxel 层**，所以 `local_costmap` 是二维的，不存在
`/local_costmap/voxel_grid` 这个话题。

---

## 四、控制层

速度链路单向且逐级收紧：

```
Nav2 controller → /cmd_vel_nav → cmd_vel_safety → /cmd_vel_safe
  → ROS 2 中继 → UDP → ROS 1 中继 → /cmd_vel → wheeltec_robot → MCU
```

`cmd_vel_safety` 逐条判定，任一不满足就降级/置零，原因写进 `/safety/stop_reason`：

| 条件 | 参数 | 行为 |
|---|---|---|
| 限速 | `max_linear_x 0.35` / `linear_y 0.35` / `angular_z 0.8` | 削顶 |
| 指令超时 | `command_timeout 0.5 s` | 停止 |
| 定位失效 | `require_localization` + `localization_timeout 0.5 s` | 停止 |
| 障碍 | `stop_on_obstacle` + `/safety/obstacle_stop` | **只封平移**，保留原地旋转脱困 |
| 急停 | `/safety/estop` | 全零，优先级最高 |

**「只封平移」是踩坑换来的**：早期实现障碍命中就把整个 Twist 置零，车头进入
0.45 m 死角后**永久锁死**，自己出不来 —— 实测第二轮 66 s 卡死 → 连续 5 次目标
超时 → ABORTED，只能人工挪车。现在障碍锁存时 `linear.x/y` 恒为 0、允许以
`obstacle_escape_angular_z 0.30` 原地旋转：旋转不产生位移，不会把车带进未测量
区域，却是唯一能自己走出死角的动作。残余风险：死角内旋转时车体角点扫过半径可能
大于车头到障碍的距离，所以转速压到 0.3 rad/s，且现场必须有人、手能碰到断电开关。

**两级监视器**：

- `obstacle_monitor`：`/scan` 前向扇区（`forward_sector_deg 50`，D435 水平 FOV
  约 87°，取保守走廊），`stop_distance 0.45`、`slow_distance 0.80`、
  `scan_timeout 1.0`。**没有 `/scan` 数据时按「障碍存在」处理** —— 深度相机是
  唯一避障传感器，宁可不动。
- `localization_monitor`：`map→odom_combined` 或 `odom_combined→base_footprint`
  任一段超时（0.5 s，`require_map_to_odom true`）即判定位失效。

---

## 五、执行层

`wheeltec_robot` 订阅 `/cmd_vel`，经 CP2102（`/dev/ttyUSB0`）与 MCU 串口通信。
**它只在收到消息时才写串口帧**，不收就保持上一次速度 —— 这条路径上没有看门狗。
所以所有运动脚本结束都必须补零速，绕过安全链直发 `/cmd_vel` 是危险动作。

运动闸门 `_enable_cmd_vel` 在 ROS 1 中继里：为 `false` 时干脆不创建 `/cmd_vel`
发布者，于是 `rostopic info /cmd_vel` 显示 `Publishers: None` —— 这就是
「车不可能动」的可验证判据。

中继的 `planar_odom_covariance: true` 把 `/odom` 里 z/roll/pitch 的 `1e6` 哨兵
换成有限值。不这么做，RTAB-Map 会报 `Very large angular variance`，图优化变差、
错误回环更易被接受。前提是「车只跑平地」，操作者已确认。

---

## 六、编排层

- `r680_explore_up.sh`：整栈启动（TX1：清理 → 独立 roscore → 驱动 → 电压记录 →
  闸门；Orin：清理 → 中继 → 相机 → 静态TF/安全 → RTAB-Map → 探索/Nav2）
- `r680_explore_300.sh`：一键「整栈 → 300 s 真实探索 → 自动关闸收尾」，默认要输 `GO`
- `r680_down.sh`：先锁存急停 + 关闸 + 验证，再拆 Orin，最后拆 TX1
- `r680_gate.sh`：闸门开关与状态校验
- `start_foxglove.sh`：`foxglove_bridge`（8765，`address:=0.0.0.0`）

两个反复踩到的工程约束，已写进脚本注释：

1. **清理与启动必须是两次独立的 ssh 调用**。否则 `pkill -f` 的模式与待启动命令的
   明文出现在同一命令行里，pkill 会匹配到执行它的 shell 自己（本项目踩过七次）。
2. **master（`roscore`）必须独立于 `roslaunch`**。roslaunch 在没有 master 时会
   自己起一个 `rosmaster`，于是 master 成了它的**子进程**；`roslaunch` 一退或
   Ctrl-C，master 跟着死，而 rospy 节点**不会重新注册**，表现为静默失去响应、
   没有任何报错。现在 up 脚本里有一个 `1b` 步骤专门先起独立 `roscore`，并校验
   master 的父进程确实是 `roscore`。

---

## 七、三个防御性设计（框图里用琥珀色标出）

1. **运动闸门**：`_enable_cmd_vel=false` 时根本不创建 `/cmd_vel` 发布者。
   判据不是「看日志」，而是 `rostopic info /cmd_vel` 显示 `Publishers: None`。
2. **障碍锁存只封平移、保留原地旋转**：否则车头进入 0.45 m 死角会永久锁死。
3. **master 独立于 roslaunch**：否则 roslaunch 一退 master 跟着死，rospy 节点
   静默失效。

---

## 八、框图生成说明

`tools/make_architecture_diagram.py` 用 PIL 的真实字体度量排版，同时输出 SVG 与
PNG（同一份 `BOXES`/`EDGES` 数据，保证两张图完全一致）。渲染前做六项自检，
任一不过即非零退出：

1. 文本不溢出方块（用 `font.getlength()` 实测，不用估算）
2. 方块不越出所在泳道（`lane=` 参数）
3. 方块互不重叠
4. 方块不横跨跨机通道、不越出所属机器分区
5. 连线内部采样点不穿过任何方块
6. 连线标注不压方块

另外两个不写进脚本、但在验收时跑过的检查：所有线段必须轴对齐（斜线数 = 0）、
箭头终点必须落在某个方块边上（悬空箭头 = 0）。`H_edge()` 就是为此存在 ——
相邻方块行数不同时中线高度不等，直接连 `右中点 → 左中点` 会得到一段斜线，
所以它统一取源块中线，并断言目标块纵向覆盖该高度，覆盖不到就直接报错，
不静默画歪。

字体用系统已装的 Noto Sans CJK SC（`fc-match` 可确认）。若换机器渲染，
先确认 `fc-list :lang=zh` 有中文字体，否则中文会变成空白。
