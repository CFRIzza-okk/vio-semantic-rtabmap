# R680 首次实机运动计划（分阶段、受监督）

> 目标：**第一次让这台车在自主探索栈的控制下真正动起来**，且每一步都能随时退回不动。
>
> 背景：此前从未有真车运动验证。而这条链路此前**实际上是不通的**，迄今已发现**六个**
> 静默缺陷（relay 端口配反；TX1 侧 relay 是 Python 3 代码；`/tf_static` QoS 违约；
> `time_sync_client.py` 同样是 Python 3 代码且 shebang 写错；`/scan` 从未产出过一帧；
> 地图话题名与 RTAB-Map 实际发布的 `/rtabmap/map` 对不上）。**六个均已修复并加了回归测试**
> （共 132 项全过），其中四个是 2026-10-14 执行阶段 C 时才暴露出来的。
> 也就是说从"桥通了"到"车能动"这件事，现在才第一次具备条件。
>
> 这六个缺陷有个共同点，值得记住：**它们全都不报错**。进程照起、话题照现、
> `ros2 topic list` 看着一切正常，只是永远没有数据。排查这类问题的唯一有效手段
> 是问"这个传感器**真的**在出数据吗"，而不是看话题存不存在。
>
> 执行原则：**任一阶段不通过就停下，绝不跳到下一阶段。** 每阶段都有明确的中止条件。

---

## 0. 绝对不能做的事（先看这条）

1. **不要直接往 `/cmd_vel_safe` 发指令。**
   `cmd_vel_safety` 是读 `/cmd_vel_nav`、写 `/cmd_vel_safe`。直接发 `/cmd_vel_safe`
   等于**绕过全部安全闸门**直接命令底盘。测试运动一律发 `/cmd_vel_nav`，让闸门把关。
2. **不要在 `enable_cmd_vel` 打开的情况下离开现场。**
3. **每次连续发布速度被打断（Ctrl-C / timeout 结束）后，立刻补一条零速度**：
   ```bash
   # Orin 侧（经安全节点）
   ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
     '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   # TX1 侧（直接写底盘，零速度）
   rostopic pub -1 /cmd_vel geometry_msgs/Twist \
     '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```
   实际上 `cmd_vel_safety` 有 0.5 s 超时、TX1 有 0.25 s watchdog，停止发布本身就会归零；
   但依然照发，不依赖超时。
4. **手始终放在机器人电源开关上。**

---

## 1. 着手前必须先补的三个数

这三个都是"会把车送进墙里"的量，且目前全是估值：

| 项 | 现状 | 为什么必须补 |
|---|---|---|
| 相机外参 | ✅ **已完成**。写入 `x=0.15 / y=0.0 / z=0.165 / pitch=1.66°` | RTAB-Map 尺度与障碍高度都会错 |
| 车体半径 | ✅ **已完成**。两处 costmap 统一 `0.20`（外接圆 0.1953 向上取整） | 膨胀层直接决定车会不会蹭墙 |
| 横移能力 | ⚠️ **未标定**。`vy_max` 已按计划压到 `0.0`（原 0.08） | 麦轮打滑，标定前不许横移 |

> `camera_x=0.15` 是卷尺量的（镜头在轮子矩形中心前方约 15 cm），**不是**拟合出来的。
> `camera_y` / `roll` / `yaw` 仍**假设为零**（相机在中线上、与车体正交），从未测量。
> 地面平面对前后/左右偏移不敏感，inspector 也没有对应输出，所以这几个量只能靠
> 阶段 D 选点时肉眼核对"RViz 里墙的位置是否与真实墙重合"。


---

## 2. 阶段 A：写入实测外参（不动）——✅ 已完成

**目的**：让 TF 树和建图尺度正确。此阶段无任何运动可能。

已写入 `r680_navigation.launch.py` 的默认值：

```
camera_z     = 0.165     # 实测。两组独立采样：0.1639±0.0027 / 0.1648±0.0027
camera_pitch = 0.02897   # ≈1.66° 朝下，弧度（两组：1.60±0.29 / 1.66±0.26）
camera_x     = 0.15      # 卷尺量的：镜头在轮子矩形中心前方约 15 cm
camera_y     = 0.0       # ⚠️ 未实测，见下方说明
```

> **为什么从 0.17 改成 0.165**：最早那次 0.17 是**有偏的**。旧估计器每个 z 切片取
> "y 最高的 10%" 再求均值，而"取最高分位"等于在噪声里挑极大值，均值落在高斯的
> `1.755·σ_y` 处——σ=4 mm 时**实测偏 +7.0 mm**，正好等于 1.755×4。换成对称带
> 重拟合后偏置降到 0.0000（合成数据地面真值验证）。两组独立采样的重复性达
> 0.9 mm / 0.06°，远好于逐帧散布，所以这个 5 mm 的差是可信的方向性修正。
>
> **代价很低**：`camera_z` 偏低 5 mm，只会让地面估计略低、障碍过滤略保守。

**验证**：
```bash
cd ~/VIO/semantic_ws && colcon build --packages-select semantic_rtabmap
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/semantic_rtabmap/test/ -q     # 应 132 passed
ros2 run tf2_ros tf2_echo base_link camera_link     # 确认 z≈0.165
```

**中止条件**：测试数不是 132 passed → 停下来查。

---

## 3. 阶段 B：底盘驱动上线，但**没有运动权限**

**目的**：确认遥测真实、定位有效，且车绝对不动。

**TX1**（终端 1，保持运行）：
```bash
. /opt/ros/melodic/setup.bash
. /home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
unset ROS_HOSTNAME
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch \
  car_mode:=mini_mec navigation:=false pure3d_nav:=false repeat:=false
```

**TX1**（终端 2）：relay，**注意 `enable_cmd_vel` 保持默认关闭**：
```bash
. /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
python2 /home/wheeltec/ros1_relay.py          # 不带 _enable_cmd_vel
```

**Orin**（终端）：
```bash
cd ~/VIO/semantic_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=99
ros2 launch semantic_rtabmap r680_relay.launch.py
```

**验证清单**：

| 检查 | 命令 | 预期 |
|---|---|---|
| 遥测到达 | `ros2 topic hz /odom` | ~20 Hz |
| TF 到达 | `ros2 run tf2_ros tf2_echo odom_combined base_footprint` | 能解析 |
| Orin 绑定端口 | `ss -lunp \| grep 1900` | **19001** |
| TX1 无速度发布者 | 在 TX1：`rostopic info /cmd_vel` | **Unknown topic**（闸门关闭） |

**中止条件**：TX1 上 `/cmd_vel` 竟然存在发布者 → 立刻停，说明 `enable_cmd_vel` 被打开了。
**此阶段机器人不应有任何动作。**

---

## 4. 阶段 C：建图与定位 —— ✅ 已完成

**目的**：让 `/map` 和 `map→odom` 出现，并确认定位监视器稳定。

### 实际启动方式（与本节原来的写法不同，务必照这个来）

计划原来写的是 `ros2 launch semantic_rtabmap semantic.launch.py`，**那是错的**。
`semantic.launch.py` 是更早期 PX4/无人机项目留下的文件：它的说明文字整篇在讲
MAVROS，而且会用硬编码的 `x=0.045 y=-0.075 z=-0.005` 发布 `base_link→camera_link`
——那是**飞行器的旧外参**，和本车实测的 0.165 / 1.66° 直接冲突。两个发布者同时
存在会让这条 TF 在两个值之间跳。同理 `launch_mapping_only.sh` 也是手持/无人机期
的脚本，它会用 `visual_odometry:=true` 让 RTAB-Map 自己的 `rgbd_odometry` 往
`/odom` 发里程计，而 `/odom` 已经被 relay 的轮式里程计占着。

R680 正确的三段式启动（相机 / 静态 TF+安全 / RTAB-Map 分开起，因为本包内**没有任何
launch 会启动 RTAB-Map**）：

```bash
# 1) 相机（省 CPU 关掉点云观测节点）
ros2 launch semantic_rtabmap r680_depth_cloud.launch.py start_inspector:=false

# 2) 静态 TF + 安全闸门。注意 publish_base_tf:=false
ros2 launch semantic_rtabmap r680_navigation.launch.py publish_base_tf:=false

# 3) RTAB-Map：用轮式里程计，绝不开它自己的 VO
ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=false icp_odometry:=false \
  odom_topic:=/odom frame_id:=base_footprint \
  depth:=true approx_sync:=true \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  rtabmap_viz:=false rviz:=false publish_tf_map:=true \
  "args:=--Rtabmap/DetectionRate 2.0 --Grid/CellSize 0.05 --Mem/NotLinkedNodesKept false" \
  "database_path:=$HOME/.ros/rtabmap_r680.db"
```

两个容易踩的参数细节：

* **`publish_base_tf:=false` 是必须的。** 实测 TX1 的 `/tf`（经 relay 转发）**已经**
  提供 `base_footprint→base_link`，本启动文件再发一份就是同一段 TF 两个发布者。
  README 里对这一点有明确警告。`camera_link` 则相反 —— relay 的
  `filter_tf_child: camera_link` 把它滤掉了，所以必须由 Orin 发。
* **launch 参数叫 `depth`，不是 `subscribe_depth`。** `subscribe_depth` 是**节点**
  参数；在 launch 里传它不会生效。`launch_mapping_only.sh` 里写的就是错的。
* **`--Mem/NotLinkedNodesKept false` 必须加上**，否则数据库会爆炸（见下）。

### ⚠️ 数据库膨胀：一条会毁掉长时间建图的默认值

**RTAB-Map 默认把"没被链接到图里的节点"也存进数据库，连 RGB/深度图一起存。**
静止的车每 0.5 秒仍会产生一个节点（`RGBD/LinearUpdate` 拦不住这种"无运动帧"），
于是这些节点**全部落盘**，而它们对地图毫无贡献。

实测（静止 60 秒，DetectionRate 2.0）：

| 配置 | Node | Data 图像 | 库大小 |
|---|---|---|---|
| 默认 `Mem/NotLinkedNodesKept=true` | 134 | 134 | **4.5 MB** |
| `Mem/NotLinkedNodesKept=false` | **1** | **1** | **272 KB** |

**134 个节点里只有 1 个真正被链接**，其余 133 个是纯垃圾。按这个速率，Phase C 那种
二十多分钟的静止运行就能堆出 1 GB 的库——实测遗留库 `~/.ros/rtabmap_r680.db` 正是
**1001 MB / 3280 节点 / 只有 38 条 Link**。这不是 Phase C 才有的问题：10月8日的
`rtabmap_mapping_vo.db`（130 MB）和 10月10日的 `rtabmap_showcase.db`（593 MB）
是同样的模式。

RTAB-Map 作者的原话（[introlab/rtabmap_ros#671](https://github.com/introlab/rtabmap_ros/issues/671)）：
> For debugging purpose, by default rtabmap saves all data. You can set
> `Mem/NotLinkedNodesKept` to false to avoid saving data when the robot is not moving.
> …I guess `--Mem/NotLinkedNodesKept false` could be always set, even in your original
> mapping session.

也就是说**默认值是给调试用的，长期建图必须关掉**。

另外两个可选项：

* `--Mem/BinDataKept false` —— 连已链接节点的 RGB/深度图也不存。库能更小，代价是
  事后没法用 `rtabmap-databaseViewer` 回看图像。**排查阶段建议先留着**。
* 已有的大库不必删，可以用官方工具瘦身（`Mem/BinDataKept` 那步可选）：
  ```bash
  rtabmap-reprocess --Mem/NotLinkedNodesKept false --Mem/BinDataKept false \
      rtabmap.db output.db
  ```
  作者提醒：重新处理后回环检测结果可能与原图略有不同。

> **排查陷阱（我自己踩的）**：不要在 RTAB-Map 运行时用 `sqlite3` 打开它的库。
> 读连接会持锁，RTAB-Map 的写随即以
> `[FATAL] DBDriverSqlite3.cpp:5034::addStatisticsQuery() … database is locked`
> 崩溃（退出码 -6）。**先停 RTAB-Map，再查库。**

### 实测结果（2026-10-14）

| 检查项 | 期望 | 实测 | |
|---|---|---|---|
| `map → odom` 出现 | 出现 | `map→base_footprint` = `[-0.002, 0.002, 0.000]`，RPY `[0.004, 0.007, 0.102]` | ✅ |
| 地图有内容 | 有 | `/rtabmap/map` = `nav_msgs/OccupancyGrid`，**127×86 @ 0.05 m**（6.35×4.3 m） | ✅ |
| `localization_valid` 稳定 | 不翻转 | 连采 183 个样本：**true=183，false=0，翻转=0** | ✅ |
| `/scan` | 有数据 | **17.3 Hz**，614/640 有效波束，张角 55.4°，量程 1.04 / 2.86 / 4.52 m | ✅ |
| `obstacle_stop` | 空场应为 false | **false** | ✅ |

启动位置姿：`camera_link` 实测 `[0.150, 0.000, 0.165]`、pitch `1.660°` —— 与实测值
逐位一致，说明外参确实生效。

**中止条件未触发。** 但要注意：这条检查真正被证明有意义，是在 TX1 断电那一刻 ——
`localization_valid` 当场从 `true` 翻成 **`false`**。也就是说它并非恒真，两端都工作。
只测"不翻转"是不够的，必须在 TX1 掉线时确认它会变 false。

### 阶段 C 期间修掉的两个缺陷（都会静默失败）

**缺陷 1：`/scan` 从来没有输出过一帧。**

`depthimage_to_laserscan` 一直订阅 `/depth_camera_info` —— 一个 **Publisher count: 0**
的话题，D435 从不发布这个名字。它每帧打印 `No camera info, skipping point cloud
squash`，`/scan` 一帧不出。

危险之处在**下游症状**：`r680_exploration.yaml` 里明确写着"没有 /scan 数据时按
'障碍存在'处理"，所以这个故障的表现是 `obstacle_stop: true` —— **看起来像检测到了
障碍，实际是传感器全瞎**。照着"场地里有障碍"去排查会完全走错方向。

修复中有个二次陷阱：remap 的**源名字必须是 `depth_camera_info`，不是 `camera_info`**。
写成 `("camera_info", ...)` **不报错、不警告**，节点照旧订阅那个空话题，症状一模一样。
正确写法见官方 `depthimage_to_laserscan-launch.py` 里的
`('depth_camera_info', '/camera/depth/camera_info')`。

同时修了 `output_frame`：原为 `camera_depth_frame`，但实测对齐深度图像和它的
camera_info 的 frame_id 都是 `camera_color_optical_frame`（深度对齐到了彩色）。
这两个 frame 在 TF 里差 15 mm 横向：`camera_depth_frame` 是 `[0,0,0]`，
`camera_color_optical_frame` 是 `[-0.002, 0.015, 0.000]`。scan 直接喂给 0.45 m 急停，
这 15 mm 不该留着。

**缺陷 2：地图话题名对不上，costmap 会永远是空的。**

`r680_nav2.yaml` 与 `r680_exploration.yaml` 都按 `/map` 写，但 RTAB-Map 的 namespace
默认是 `rtabmap`，**实际发布 `/rtabmap/map`**。nav2 的 `static_layer` 没有设
`map_topic` 时默认订阅 `/map` —— 于是 global_costmap 一直等一个不存在的话题，
规划器会把未探索区域当作可通行。

仓库里其实**知道**这个 namespace 问题：`mapper.yaml` 的注释写明了，`mapper_node.py`
还为此往两个话题双发。只有 nav2 和探索器这两处漏了。现在三处都显式写了
`/rtabmap/map`（改动 costmap 配置前已获操作者明确同意）。

顺带更正了该文件头部两处过期注释：一处仍写"直接用 RTAB-Map 的 /map"，另一处仍写
"vy 上限 0.08 m/s"（实际已压到 0.0，即完全禁用横移）。

### 关于 TX1 时钟：一条被推翻的结论，留档以免重蹈

阶段 C 期间 RTAB-Map 报过一次 `cannot get the corresponding TF
odom_combined->base_footprint`。当时我用"跨 SSH 会话括起 `date`"的方式估算 TX1 与
Orin 的时钟差，得到 **+240 ms**，并据此把这条警告归因于时钟偏移。**这个数字是错的。**
SSH 连接、认证、解释器启动的几百毫秒被整个算进了"偏移"里，而且它并不对称地落在
括起中点。

用协议方式（t1/t2/t3/t0 四个时间戳，能把 RTT 和服务器处理时间都减掉）在**同一个 SSH
会话内**连测 6 次，结果是 **+2.483 ~ +2.514 ms，散布仅 46 µs，RTT ~0.5 ms** —— 真实
偏移是 **+2.5 ms**，比 RTAB-Map 的容差（`wait_for_transform` 0.2 s）小两个数量级，
**不可能是那条警告的原因**。旁证：`localization_valid` 183/183 稳定；若真偏 240 ms，
`localization_monitor` 的 0.5 s 超时早就会周期性把它打成 false。

那条 TF 警告的**真正原因尚未确定**（已排除时钟；怀疑 `/odom` 与 `/tf` 是两条独立的
UDP 路径、到达顺序无保证，而 RTAB-Map 拿到 odom 后立刻查 TF）。下次上电应继续查。

另外记录：TX1 的 RTC 停在 **2000 年**且不可写，`ntpd` 已安装但未运行，
`systemd-timesyncd` 在跑但 `System clock synchronized: no`（即从未同步成功）。
时钟校正实际由 **`r680-time-sync.service`**（systemd，`Restart=always`，以 root 运行）
承担，它用 `/usr/bin/python3` 拉起 `time_sync_client.py`，配合 Orin 上的
`tools/time_sync_server.py`。**这套机制是工作的**：2026-10-14 01:06:54 日志记录它把
时钟一步校正了 **+98787286 s（+1143 天）**，正是 TX1 断电重启后 RTC 回到 2000 年所需
的那一跳。

实测 TX1 晶振漂移约 **23 ppm**（连测 8 次，每 10 s 单调增长约 0.228 ms：
-0.065 → -1.664 ms）。该服务阈值是 5 ms，所以约每 **219 秒**步进一次，
时钟呈**锯齿**状。这解释了为什么在不同时刻测偏移会得到 +2.5 / +4.4 / +5.1 ms 等不同值——
**一部分是锯齿，不全是测量误差**。5 ms 仍远小于 RTAB-Map 的 200 ms 容差，
所以它不是那条 TF 警告的原因，但排查时要知道锯齿存在。

（`time_sync_client.py` 里的 7 处 `print(..., flush=True)` 与 `python3` shebang 确实是
py2 不兼容写法，但在这条 python3 服务路径上**不触发**；修复的价值是消除脆弱性，
不是"救活了一个死功能"。早先"从未成功运行过"的说法有误，已更正，详见 README 缺陷 4。）

---

## 5. 阶段 D：只选点，不运动（三重保护）

**目的**：验证选点质量。此时 `dry_run:=true`（探索器不调 Nav2）
**且** `enable_cmd_vel` 仍关闭（TX1 无速度发布者）——**两层独立保护**。

```bash
ros2 launch semantic_rtabmap r680_exploration.launch.py include_nav2:=false dry_run:=true
# 另开终端
ros2 service call /exploration/start std_srvs/srv/Trigger
ros2 topic echo /exploration/goal
ros2 topic echo /exploration/status
```

**在 RViz 里逐条核对**：

- 目标点是否落在**合理、可达、离墙有距离**的位置？
- 是否出现"隔着墙"的目标？（应为 0，`require_reachable: true` 会滤掉）
- 目标是否总贴着墙？→ 调大 `goal_clearance`（现 0.25）
- **墙的位置是否与真实墙重合？** 这条同时验证 `camera_x/y`

**中止条件**：目标频繁贴墙或落在未知区 → 先调参数，不要进入运动阶段。

---

## 6. 阶段 E：**第一次运动**（最危险的一步，必须两人在场）

### E0（强烈建议先做）：架空测试

**把四个轮子垫离地面**（木块/砖），然后：

1. 打开 TX1 运动权限：重启 relay，加 `_enable_cmd_vel:=true`
2. Orin 侧把上限收到极低：
   ```bash
   ros2 launch semantic_rtabmap r680_navigation.launch.py \
     max_linear_x:=0.08 max_linear_y:=0.0 max_angular_z:=0.3
   ```
3. 发一条极短的前进指令，**走 `/cmd_vel_nav`，不绕闸门**：
   ```bash
   timeout 3 ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
     '{linear: {x: 0.06}, angular: {z: 0.0}}'
   # 结束后立刻补零（铁律）
   ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
     '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

**架空阶段要确认的**：

- 轮子**转的方向对不对**（前进指令 → 四轮同向向前）
- `/odom` 的 x 是否随之增加（方向正确性）
- 反应是否**远远快于**指令（说明有别的速度源在发）
- 松开后轮子是否**立刻停**

**中止条件**：方向不对、有一轮反转、松手后不停 → 立即断电查原因。

### E1：落地，直线爬行

场地空旷 ≥3×3 m，两人：一人手持电源开关，一人看屏幕。

- 上限维持 `0.08 / 0.0 / 0.3`
- 指令 `x: 0.06`，`timeout 3`，走 `/cmd_vel_nav`
- 观察：车是否**缓慢直行**、是否**及时停下**、`/cmd_vel_safe` 是否 20 Hz

### E2：原地旋转

- 指令 `angular: {z: 0.25}`，`timeout 3`
- 确认转向符号与预期一致

### E3：经 Nav2 走一个点

在 RViz 里点一个 **1 m 外、空旷**的 goal（或 `NavigateToPose`），看它是否
平滑起步、到位停下。

**中止条件（任一即停并断电）**：蹭到任何东西、速度明显超过预期、
`obstacle_stop` 变 true 但车没停、定位跳变。

---

## 7. 阶段 F：短程自主探索

E 全部通过后：

```bash
ros2 launch semantic_rtabmap r680_exploration.launch.py dry_run:=false
ros2 service call /exploration/start std_srvs/srv/Trigger
```

先把预算收短：`max_exploration_time:=120`（现 1800）、`goal_timeout:=60`（现 120）、
`max_linear_x:=0.10`（现 0.35）。人守着，观察前几个目标点是否合理。

**阶段 G**：连续几轮无异常后，再逐步放开时间与速度。

---

## 8. 随时可用的紧急停止

按优先级从快到慢：

```bash
# 1. 最快：物理断电（手放在开关上）

# 2. 锁存急停（发一次就一直停，不会自己过期）
ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: true}"
# 恢复：                                          "{data: false}"

# 3. 杀掉 TX1 relay —— 0.25 s 内 TX1 watchdog 归零
#    （在 TX1 上）kill <relay pid>

# 4. 暂停探索
ros2 service call /exploration/pause std_srvs/srv/Trigger
```

**兜底逻辑**：`cmd_vel_safety` 0.5 s 无指令归零 → TX1 watchdog 0.25 s 无**通过
HMAC 校验**的指令归零。两层都在 Orin 之外/之后，即使 Orin 或网络整个挂掉，
底盘也会停。

---

## 9. 回滚

| 改动 | 回滚方式 |
|---|---|
| TX1 relay 脚本 | `cp /home/wheeltec/ros1_relay.py.bak-20261013-153438 /home/wheeltec/ros1_relay.py` |
| 运动权限 | 不带 `_enable_cmd_vel` 重启 relay（默认就是关） |
| 外参 | 启动参数传回 `camera_z:=0.0 camera_pitch:=0.0` |

---

## 10. 仍未完成、且会影响长时间运行的项

1. ~~**电压守卫未启用**~~ ✅ **已接通（2026-10-14）**。
   `/PowerVoltage`（`std_msgs/Float32`，TX1 实测 11.90 V）现在经 relay 转发到
   `/battery_voltage`，探索配置的 `battery_topic` 已指向它。

   **接通时发现一个更严重的问题**：守卫的写法是

   ```python
   if self._battery is not None:          # ← “没数据”被当作“没问题”
       if self._battery < minimum: ...
   ```

   `_battery_stamp` 每次采样都被记录，但**从来没有被任何判断读过**（只写不读）。
   后果：relay 中途挂掉、TX1 掉线或网络丢包时，`/battery_voltage` 不再更新，
   而守卫继续拿**最后一次已知值**比较并报“正常”——**传感器瞎了，系统看起来健康**。
   这与本项目其它缺陷是同一个模式。

   已补上 `battery_timeout`（默认 5 s）：样本超时即转 `ABORTED` 并取消目标，
   而不是沿用陈旧值。**“陈旧”不等于“低电压”，所以按故障处理**——一次看不见
   自己电量的自主探索，等于失去了它所依赖的保护。

   守卫：`test/test_battery_guard.py`（9 项）。变异测试 7 个变异全部被抓到；
   其中 2 个最初漏掉（TX1 侧订阅被删仍能通过——因为注释里也写着话题名；
   陈旧分支删掉 `_cancel_goal` 仍能通过——因为外层 if 的兄弟分支里也有一个），
   两处都已改为真正的 AST 结构断言。
2. ~~**`/tf_static` 的 QoS**~~ ✅ **已修复**（见下方"修复记录"）。
3. **relay 密钥仍是 `change-me-r680`**。
4. **横移未标定**。`vy_max` 已压到 0，直到单独标定过。
5. ~~**`robot_radius` 两处不一致**~~ ✅ **已统一为 0.20**（两处 costmap + 探索配置）。
6. **测试不能在实时栈运行时跑**。`test_safety_monitors.py` 等用绝对话题名，
   与实时节点互相干扰，会产生 3 个假失败（表现为 bool 值 True/False 交替——
   两个发布者在同一话题上打架的典型特征）。**跑测试前先停实时栈。**

### 10.1 `/tf_static` 修复记录（2026-10-13）

**症状**：`tf2_echo` 每次都报

```
New publisher discovered on topic '/tf_static', offering incompatible QoS.
No messages will be sent to it. Last incompatible policy: DURABILITY_QOS_POLICY
```

**根因（两个，独立）**：

1. **relay 的 QoS 违约**。`ros2_relay_node.py` 里是
   `create_publisher(TFMessage, "/tf_static", 10)` —— 传裸整数时默认 durability 是
   **VOLATILE**，而 `tf2_ros` 的监听者和本包 `safety_monitor_node` 都按
   `TRANSIENT_LOCAL` 订阅。ROS 2 里这是一对**不兼容**的 QoS，连接直接被拒，
   **一个静态变换都收不到**——不是"收晚了"，是"收不到"。已改为显式
   `TF_STATIC_QOS`（`TRANSIENT_LOCAL` + `RELIABLE`），回归测试
   `test/test_relay_tf_static_qos.py`（4 项，含"护栏接线而不只是常量"那一条：
   把 `create_publisher` 改回裸整数会让第 4 项失败）。
   **注意 `/tf`（动态）必须保持 VOLATILE，不能一起改**，测试里也钉住了。

2. **TX1 根本没往 `/tf_static` 发东西**。实测 TX1 侧：
   - `/tf_static` 唯一的发布者是 `/robot_state_publisher`，它只发了**一条、含 0 个变换**
     的消息（URDF 里没有 static joint）；
   - TX1 那四个 `static_transform_publisher`（`/base_to_camera`、`/base_to_link`、
     `/base_to_gyro`、`/base_to_laser`）**发的是 `/tf`，不是 `/tf_static`**。

   所以 relay 的 `publish_tf` 走到 `if out:` 时 `out` 为空，什么都不发 ——
   静态 TF 这条通路**目前是空转的**。QoS 修好只是把它变成"正确的空转"。

**因此 Orin 的静态 TF 必须由 Orin 自己提供**（`r680_navigation.launch.py` 里的
`base_link→base_footprint` 与 `base_link→camera_link`）。`/tf_static` 这条 relay
通路可留作将来使用，但**不要指望它**。

**顺带确认**：relay 的 `filter_tf_child: camera_link` + `drop_tf_parent: base_footprint`
过滤器**确实在工作** —— 虽然在 `/tf_static` 上无物可滤，但 TX1 的相机 TF 走 `/tf`，
同一段过滤逻辑对两条通路都生效；实测 Orin 的 `/tf` 帧表里**没有 `camera_link`**。

