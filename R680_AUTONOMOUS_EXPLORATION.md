# R680 麦轮小车自主导航探索策略

本文档说明 R680 自主探索的整体策略、代码结构、参数含义，以及**分阶段上电验证流程**。

配套代码：

| 文件 | 作用 |
|---|---|
| `semantic_ws/src/semantic_rtabmap/semantic_rtabmap/frontier.py` | 纯算法层：frontier 检测、连通性过滤、评分、黑名单（无 ROS 依赖，可单测） |
| `semantic_ws/src/semantic_rtabmap/semantic_rtabmap/exploration_node.py` | 探索状态机：选点、下发 Nav2 目标、失败剔除、安全闸门 |
| `semantic_ws/src/semantic_rtabmap/semantic_rtabmap/safety_monitor_node.py` | 障碍监视器 + 定位监视器，给安全节点提供输入 |
| `semantic_ws/src/semantic_rtabmap/config/r680_exploration.yaml` | 探索与安全监视器参数 |
| `semantic_ws/src/semantic_rtabmap/config/r680_nav2.yaml` | Nav2 参数（麦轮、仅深度相机） |
| `semantic_ws/src/semantic_rtabmap/launch/r680_exploration.launch.py` | 启动文件 |
| `semantic_ws/src/semantic_rtabmap/launch/r680_nav2_bringup.launch.py` | 本工程专用 Nav2 启动（修正速度通路，见 §5） |
| `semantic_ws/src/semantic_rtabmap/test/test_frontier_policy.py` | 算法单测（合成栅格，29 例） |
| `semantic_ws/src/semantic_rtabmap/test/test_exploration_node.py` | 节点级测试（状态机 + 安全闸门，15 例） |
| `semantic_ws/src/semantic_rtabmap/test/test_safety_monitors.py` | 安全监视器测试（含时钟偏移回归，14 例） |

---

## 0. 实测发现并修复的问题（重要）

这些是**实际把整套系统跑起来**才暴露出来的问题，不是纸面设计。

### 0.1 官方 Nav2 bringup 会让速度绕过安全节点（安全漏洞）

用 `ros2 node info` 实测官方 `nav2_bringup/launch/navigation_launch.py` 的接线：

| 节点 | 发布话题 |
|---|---|
| `controller_server` | `/cmd_vel_nav` |
| `behavior_server` | **`/cmd_vel`** |
| `velocity_smoother` | **`/cmd_vel`** |

也就是说 Nav2 的**最终**速度输出落在 `/cmd_vel`，而 `cmd_vel_safety` 只拦
`/cmd_vel_nav`。恢复动作（spin / backup / drive_on_heading）和 Smoother 之后的
速度都属于**无人把关**的状态。

修复：本工程不再使用官方 bringup，改用 `r680_nav2_bringup.launch.py`，
把 `controller_server` 和 `behavior_server` 的输出**都**重映射到 `/cmd_vel_nav`，
并且**不启动 `velocity_smoother`**（多一个速度发布者就多一条绕过安全节点的路径）。

### 0.2 `cmd_vel_safety_node` 第一拍就崩溃（致命）

原代码：

```python
self.get_logger().info("motion output: %s", reason)
```

rclpy 的 logger **不做 printf 风格替换**，传第二个参数会抛
`TypeError: RcutilsLogger.info() takes 2 positional arguments but 3 were given`，
安全节点在第一次定时器回调时就退出。也就是说**这道安全闸门此前从未真正工作过**。
`ros2_relay_node.py` 的发送失败分支里有同样写法，同样已修。

### 0.3 MPPI `CostCritic` 与 `robot_radius` 不兼容

`consider_footprint: true` 要求代价地图里有多边形 footprint，而本工程只用
`robot_radius` 描述圆形车体。实测 `controller_server` 激活阶段直接报
`Considering footprint in collision checking but no robot footprint provided`
并导致整个 bringup 失败。已改为 `consider_footprint: false`。

### 0.4 定位监视器会被 TX1 时钟偏移和静态 TF 误判

两个独立缺陷：

* 早期实现把 TF 的 `header.stamp` 与本机时钟比较算“年龄”。TX1 时钟与 Jetson
  **未同步**（实测偏差可达数年），这会让每一帧转发过来的 TF 都被判为过期，
  `localization_valid` 永远为 false，机器人**永远不动**。
* 改为按**本机单调时钟的消息到达时间**判断后，静态 TF 又有问题：
  `/tf_static` 是 latch 的、只发一次，按到达时间判断会在启动一个 timeout 周期后
  翻成 false。实测确实观察到 `true → false` 翻转。

修复：动态 TF 用到达时间判新鲜度，静态 TF 一旦收到就永久有效；
另外用 TF buffer 复核整条链是否真的能解析。

### 0.5 `/safety/stop_reason` 只在变化时发布

边沿触发意味着**之后才连上的操作员看到的是空话题**，无法判断为什么车停着。
已改为每拍（20 Hz）持续发布。

### 0.6 障碍锁存把整个 Twist 置零 -> 车头进死角后**永久锁死**

`cmd_vel_safety` 原本在 `stop_on_obstacle` 命中时输出一个全零 Twist，**包括
`angular.z`**。而 Nav2 自己的恢复行为（spin / backup）也是经这个节点出去的，
所以连"原地转开"这条唯一的自救路径也被一起掐断：

```
前向 50° 扇区内 < 0.45 m  ->  obstacle_stop = true
                          ->  cmd_vel_safety 输出全零(连 rotation 也是 0)
                          ->  Nav2 的 spin 恢复同样被清零
                          ->  车永远出不来;连续 5 次目标超时 -> ABORTED
```

实测：第二轮自主探索 **66 s 卡死 -> 连续 5 次目标超时 -> ABORTED**，
当时的"恢复办法"是人工把车挪开。

修复：障碍锁存时**只封平移，放开原地旋转**。

* 硬不变量：`linear.x` 与 `linear.y` **恒为 0**。障碍锁存期间车一步都不会朝前
  或朝侧面移动，因此不可能开进前向扇区看不见的地方。
* 允许 `angular.z`，上限由 `obstacle_escape_angular_z`（默认 0.3 rad/s）给出。
  选旋转而不是倒车，是因为**旋转不产生位移**，不会把车带进任何未测量的区域
  —— 这也正是本工程一贯拒绝倒车的原因。
* 参数 `obstacle_escape_rotation`（默认 true）可以整体关掉，回到旧行为。
* **优先级**：急停 / 定位失效 / 指令过期 一律全零，且**优先于**障碍分支。
  原来的分支顺序把障碍放在最前；在加入脱困旋转之后，那个顺序会让"已按急停"
  或"定位已失效"的车照样原地打转，所以这次把三个硬停机提到了前面。

真机验证（闸门关闭、由真实 `/scan` 触发真实锁存，验证的是放行逻辑）：

| 场景 | stop_reason | linear.x | angular.z |
|---|---|---|---|
| 障碍锁存 + 脱困开 | `obstacle_stop_escape_rotation` | 0.0 | **0.3** |
| 障碍锁存 + 脱困关 | `obstacle_stop` | 0.0 | 0.0 |
| 障碍锁存 + 急停 | `emergency_stop` | 0.0 | 0.0 |
| 前方空旷 | `ok` | 0.1 | 0.35 |

**残余风险（必须说清）**：在死角内旋转时，车体角点扫过的半径可能大于车头到
障碍物的距离。所以脱困转速刻意压得很低（0.3 rad/s），且现场仍需有人、手能碰到
断电开关。

### 0.7 同一个去不了的地方被反复重试，直到整轮探索被拖死

`GoalBlacklist` 是**会过期**的（ttl 90 s），语义是"现在不行，等会再试"。
但一个根本到不了的 frontier（例如被家具挡住）会被反复重试：按 `goal_timeout`
60 s 算，连续三次重试就是三分钟预算花在重复证明同一件事，最后以"连续失败"
中止整轮 —— 而真正的原因是**那一片地方去不了**，不是探索策略坏了。

修复：反复失败（`unexplorable_failures`，默认 2 次，按 `unexplorable_radius`
1.5 m 的半径聚类 —— frontier 质心每次差几厘米，精确匹配等于永不触发）的区域
被永久标注为**无法探索**：

* `select_goal(..., unexplorable=...)` 从此跳过它们，本轮不再派车去；
* 明细发布在 `/exploration/unexplorable`（JSON `[{"x":..,"y":..}]`），
  `/exploration/status` 里带 `unexplorable` 处数，监视脚本会显示；
* ABORTED 的消息现在带上"多少处被标注无法探索"，因为同一句"连续失败"
  可能对应两种完全不同的处境：参数该调，还是障碍物该搬。

它和黑名单的区别是本功能的全部要点：**黑名单是"等会再试"，这里是"试过了，
去不了"**。

---

### 0.8 重启顺序会让下行 cmd_vel 被静默丢弃十几分钟（"车不动"的真凶）

**这是 2026-10-17 自主探索跑不动 0 米的真正原因**，不是 0.6 的障碍锁死（那一轮
`obstacle_stop` 全程为 false），也不是底盘、驱动或中继本身的问题。

TX1 侧中继只接受**严格递增**的序号，而它只在**自己**重启时才重置水位：

```python
seq = int(body.get("seq", -1))
if seq <= self.last_seq: continue      # 静默丢弃,不打任何日志
```

Orin 侧 `cmd_seq` 每次进程启动都从 0 重新计数。原启动顺序是"4/7 开闸门（重启 TX1
中继）→ 5b/7 起 Orin 栈"，也就是说**开闸门时上一套 Orin 节点还在发**：

```
旧 Orin 节点已数到 seq ≈ 13800   ->  TX1 last_seq = 13800
新 Orin 节点从 seq = 0 开始      ->  之后 13800 个包全部 seq <= last_seq
20 Hz 下 ≈ 11.5 分钟
```

这段静默期的欺骗性在于**每一层看起来都正常**：`continue` 不打日志、看门狗照常以
20 Hz 发零、上行 odom/tf/电压完全不受影响（所以 `localization_valid` 一直是
true）、安全链报告健康 —— 只有车一步不动，Nav2 反复报 "Failed to make progress"。
90 秒的探索预算整个泡在这段黑屏里。

修复：发送端在**被签名的 body 里**带上每进程唯一的 `session`
（`random.getrandbits(64)`），接收端发现 session 变化就**重设水位**而不是拿新
进程的 0 去比旧进程的 13800；同一 session 内的重复/乱序包仍然照丢（守卫的本职，
它本来也防不住能伪造 HMAC 的人）。

真机 A/B（同一条指令 `x=0.04` 经安全节点发 8 秒，两次都发了 160 条）：

| | TX1 收到的非零包 | 车动了吗 |
|---|---|---|
| A：旧中继 + 重启 Orin 节点 | **0 / 258** | 没动 |
| B：修复后中继 + 重启 Orin 节点 | **120 / 258** | **odom x = 0.332 m** |

排查要点（下次别再走弯路）：定位这个问题时，**短命的 `ros2 topic pub` 不能用来
判断下行通不通**，它发出的大部分包会在 DDS 发现完成之前就发完了 —— 我因此一度
误判中继损坏。要用常驻发布器（每 50 ms 一条、持续十几秒）。另外**杀掉 TX1 中继
会让上行一起断**，安全节点随即因 `localization_invalid_or_timeout` 清零所有速度，
那也会伪装成"下行不通"。

回归防护：`test/test_relay_seq_session.py`（9 个测试，含"旧规则必然锁死多少个包"
这一条，变异测试三个方向均被抓）。

---

## 1. 系统分层与数据流

```
TX1 (ROS 1 Melodic)                     Jetson Orin NX (ROS 2 Humble)
┌────────────────────────┐              ┌──────────────────────────────────────┐
│ Wheeltec 麦轮底盘       │              │ D435 ──┬─→ RTAB-Map ─→ /map (2D 栅格) │
│  /odom  /imu  /tf      │              │        │        └─→ map→odom TF        │
│  /cmd_vel              │              │        └─→ depthimage_to_laserscan     │
└───────────┬────────────┘              │                     └─→ /scan         │
            │                           │                                      │
            │  UDP relay（已实测 20Hz）  │  r680_explorer ─→ Nav2 (BT)          │
            └───────────────────────────┴──────────────────────────────────────┘
                                                            │
                        controller_server ─┐                │ 目标
                        behavior_server  ─┴─→ /cmd_vel_nav ─┘
                                                 │
                                          cmd_vel_safety  ←── obstacle_monitor
                                                 │        ←── localization_monitor
                                            /cmd_vel_safe
                                                 │
                                          relay → TX1 /cmd_vel
```

关键约束（现场已确认）：

* TX1 的 ROS master 在 TX1 本机，Jetson 通过 **有线** `192.168.2.15 ↔ 192.168.2.102` 访问；
  NoMachine 走 WiFi `192.168.5.4`（SSID `Innovation_Lab`），两条路互不影响。
* TX1 没有 2D 激光雷达在跑，`/scan` 不存在。**障碍感知完全依赖 D435 深度**。
* D435 水平视场约 87°，且朝前安装 —— 地图永远是“前方一条”，不是 360°。
* `/odom` 约 20 Hz，`/tf` 约 50–60 Hz，relay 转发后 ROS 2 侧实测同样频率。
* 麦轮横移存在打滑与 x/y 耦合，**不能把精确横移当成可控能力**。
* TX1 时钟与 Jetson 不同步：任何跨机的时间比较都不可信。


---

## 2. 为什么用 frontier 探索

在“视场窄、无雷达、地图始终不完整”的条件下，覆盖率只有一种可行来源：
**反复走向“已知自由空间与未知空间的交界”，看一眼，再走向下一个交界。**

这个交界就是 frontier。策略要点：

1. **只看连通区域**：只保留与机器人当前自由空间同一连通分量的 frontier。
   否则会生成“隔着墙的目标点”，Nav2 规划必然失败，白白消耗时间预算。
2. **目标点必须离障碍物足够远**（`goal_clearance`）：目标落在膨胀障碍内时，
   退回到离簇中心最近的合法 frontier 单元；若整簇都在膨胀区内，整簇丢弃。
3. **优先“信息增益”而不是“最近”**：`info_gain_radius` 内的未知单元数量作为增益估计，
   权重最高。否则机器人会在门口反复打转，而不去真正未知的房间。
4. **朝向惩罚**：`score_weight_heading` 让正前方的 frontier 更优，
   减少“到达后又掉头”的无效旋转 —— 对打滑底盘尤其重要。
5. **失败目标黑名单 + TTL**：Nav2 到不了的点 90 秒内不再尝试，但地图会变，所以会过期。
6. **完成判定要连续多次确认**：连续 `complete_confirmations` 次没有可用 frontier
   才判定探索完成，避免瞬时误判。

评分公式（各项都做了 min-max 归一化，换分辨率或换房间大小都不用重调权重）：

```
score = w_size * size_norm
      + w_gain * gain_norm
      - w_dist * dist_norm
      - w_heading * |heading_error| / pi
```

---

## 3. 探索状态机

```
        /exploration/start            /exploration/pause
IDLE ────────────────────→ EXPLORING ⇄ PAUSED
  ↑                          │  │        │
  │ /exploration/stop        │  │        │ /exploration/resume
  └──────────────────────────┘  │        └──→ EXPLORING
                                │
              无可用 frontier（连续 N 次）→ COMPLETE
              超出时间预算 / 连续失败 / 低电压 → ABORTED
```

进入 `EXPLORING` 后每 `replan_period` 秒执行一次决策：

1. 检查全局时间预算 → 超时转 `ABORTED`；
2. 检查连续失败数 → 超限转 `ABORTED`；
3. 检查电压（若配置了 `battery_topic`）→ 低于阈值转 `ABORTED`；
4. 若已有进行中的 Nav2 目标，检查是否超 `goal_timeout` → 超时则剔除该目标；
5. 否则取位姿 → 检测 frontier → 评分 → 选择 → 下发。

`dry_run: true` 时第 5 步只发布 `/exploration/goal` 并把该点加入黑名单（以便下一轮评估别处），
**永远不会调用 Nav2 动作服务器**，所以机器人不可能动。

---

## 4. 安全设计（分层，任一层失效都会停车）

| 层 | 组件 | 触发条件 | 结果 |
|---|---|---|---|
| L1 | `obstacle_monitor` | 前向 50° 扇区内最近距离 < 0.45 m，**或 /scan 超时 1 s 无数据** | `/safety/obstacle_stop = true` |
| L2 | `localization_monitor` | `map→odom` 或 `odom→base_footprint` 任一段超过 0.5 s 未更新（静态 TF 不受此限） | `/safety/localization_valid = false` |
| L3 | `cmd_vel_safety` | 上述任一为真、或 0.5 s 没有新的速度指令、或急停 | 输出零速度 |
| L4 | 探索器 | 时间预算、连续失败、低电压 | 停止下发新目标 |
| L5 | TX1 relay watchdog | 0.25 s 没有收到**通过 HMAC 校验**的合法指令 | TX1 本地持续发布零 Twist |

设计原则：

* **探索器永远不发布速度**，只发布 Nav2 目标；唯一的放行点是 `cmd_vel_safety`。
* `/scan` 缺失时按“有障碍”处理 —— 深度相机是唯一避障传感器，没有数据就等于没有安全依据。
* TX1 侧的零速度 watchdog 独立于 Jetson，即使 Jetson 或网络整体挂掉，底盘也会停。

---

## 5. Nav2 取向（麦轮 + 仅深度）

* 控制器：MPPI，`motion_model` 选 `Omni` 以表达麦轮的横向速度；但
  **`vy` 上限刻意压到 0.08 m/s**，因为实测横移会打滑，并保留
  `PreferForwardCritic` 让车宁可转向也少横移。初期可以直接把 vy 限到 0，
  先验证“前进 + 旋转”是否稳定，再逐步放开横移。
* 全局规划器：`NavfnPlanner`（全向底盘不需要最小转弯半径约束，栅格规划器足够）。
* 代价地图观测源：`/scan`（来自 D435 对齐深度），不使用点云，降低算力占用。
* **不使用 AMCL、不使用 map_server**：定位与地图由 RTAB-Map 提供，
  `map→odom` 只能有一个发布者，否则 TF 会打架。
* 坐标约定：`map`（全局）/ `odom_combined`（局部，来自 TX1）/ `base_footprint`。
* 速度通路唯一化：见 §0.1，使用 `r680_nav2_bringup.launch.py` 而不是官方
  `navigation_launch.py`，并且不启动 `velocity_smoother`。
  将来若确实需要速度平滑，必须插在 `cmd_vel_safety` **之前**，
  而不是接在它后面。

---

## 6. 分阶段验证流程（务必按顺序）

### 阶段 0：只读链路（已完成）

TX1 relay 只发传感器，ROS 2 实测 `/odom` 20 Hz、`/imu` 20 Hz、`/tf` 约 60 Hz。

### 阶段 1：不启动 Nav2，只验证感知与安全输入

```bash
# TX1：驱动 + 只读 relay（_enable_cmd_vel:=false）
# Jetson：
cd ~/VIO/semantic_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch semantic_rtabmap r680_exploration.launch.py include_nav2:=false dry_run:=true

# 另开终端检查
ros2 topic hz /scan
ros2 topic echo /safety/obstacle_stop
ros2 topic echo /safety/localization_valid
```

预期：`/scan` 有稳定频率；手放在相机前 `obstacle_stop` 变 `true`；离开后变 `false`。
**此时机器人不应有任何动作。**

### 阶段 2：只选点，不运动

```bash
ros2 launch semantic_rtabmap r680_exploration.launch.py dry_run:=true
# 另开终端
ros2 service call /exploration/start std_srvs/srv/Trigger
ros2 topic echo /exploration/goal
ros2 topic echo /exploration/status
```

在 RViz 里观察：目标点是否落在**合理、可达、离墙有距离**的位置？
如果目标总是贴墙或落在未知区，先调 `goal_clearance` / `min_frontier_size`。

### 阶段 3：低速真实运动（场地空旷、有人手持急停）

```bash
ros2 launch semantic_rtabmap r680_exploration.launch.py dry_run:=false
ros2 service call /exploration/start std_srvs/srv/Trigger
```

同时把 `cmd_vel_safety` 的 `max_linear_x` 收到 0.10 左右，
`max_exploration_time` 收到 120 秒，先做短程验证。

随时可以：

```bash
ros2 service call /exploration/pause  std_srvs/srv/Trigger   # 暂停并取消当前目标
ros2 service call /exploration/resume std_srvs/srv/Trigger
ros2 service call /exploration/stop   std_srvs/srv/Trigger   # 回到 IDLE
ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: true}"   # 急停
```

### 阶段 4：长时间自主探索

确认阶段 3 稳定后，再放开时间预算与速度上限。

---

## 7. 操作速查

```bash
# 环境
source /opt/ros/humble/setup.bash
source ~/VIO/semantic_ws/install/setup.bash
export ROS_DOMAIN_ID=99

# TX1 环境（有线链路）
source /opt/ros/melodic/setup.bash
source /home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
unset ROS_HOSTNAME

# 单元测试
cd ~/VIO/semantic_ws
python3 -m pytest src/semantic_rtabmap/test/test_frontier_policy.py -v
python3 -m pytest src/semantic_rtabmap/test/test_exploration_node.py -v
```

---

## 8. 已知限制与后续工作

1. **电压守卫未启用**：TX1 的 `/PowerVoltage` 尚未经 relay 转发，
   `battery_topic` 默认留空。需要先在 TX1 确认该话题的消息类型，
   再在 relay 里转发，然后配置 `battery_topic` 与 `min_battery`。
2. **`/tf_static` 的 QoS**：relay 目前用默认 QoS 发布静态 TF，
   而 ROS 2 侧 TF 静态消费者期望 `TRANSIENT_LOCAL`。需要修正后再依赖
   relay 转发的静态 TF。
3. **相机外参未标定**：`r680_navigation.launch.py` 里 `base_link → camera_link`
   默认仍是零外参，必须现场测量后填入，否则 RTAB-Map 的尺度与障碍高度都会错。
4. **车体半径未实测**：`robot_radius: 0.18` 与 Nav2 膨胀半径都是估值，
   现场量完车体后要一起修正。
5. **横移未标定**：麦轮 x/y 耦合与打滑未做标定，因此 vy 上限压到 0.08 m/s。
6. **单目深度盲区**：D435 对玻璃、纯黑、强反光表面测距不可靠，
   这些区域会表现为“未知”，探索器会反复尝试。必要时加入黑名单人工排除。
7. **未做实机运动验证**：阶段 3/4（真正驱动底盘）尚未执行。
   代码与接线层面已完成下列自动化验证。

---

## 9. 实测验证证据

### 自动化测试：74 例全部通过

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/semantic_rtabmap/test/ -q
# 74 passed
```

* `test_frontier_policy.py`（29 例）：frontier 检测、min_size 过滤、连通性、
  障碍膨胀、信息增益、评分与朝向惩罚、黑名单 TTL、目标选择 —— 全部在合成栅格上验证。
* `test_exploration_node.py`（15 例）：真实 `rclpy` 节点 + 合成地图 + 注入位姿，
  验证选点、dry_run 不调 Nav2、无 frontier 转 COMPLETE、时间预算/连续失败/低电压转
  ABORTED、服务调用，以及**被取消目标的结果不会复活状态机**（回归测试）。
* `test_safety_monitors.py`（14 例）：无 `/scan` 判为有障碍、前向扇区判定、
  无限远滤除、扫描超时；定位监视器的**时钟偏移回归**与**静态 TF 不过期**回归。

### 全链路接线（假 TF + 真实 Nav2，未接底盘）

```bash
ros2 launch semantic_rtabmap r680_exploration.launch.py include_nav2:=true
```

实测结果：

| 检查项 | 结果 |
|---|---|
| Nav2 生命周期 | `Managed nodes are active`（controller/planner/behavior/bt_navigator/waypoint_follower 全部激活） |
| `/cmd_vel` 的发布者 | **无**（安全漏洞已封堵） |
| `/cmd_vel_safe` 的发布者 | 只有 `cmd_vel_safety` |
| `/cmd_vel_nav` 的发布者 | `controller_server` 与 `behavior_server` |
| `/safety/obstacle_stop` | `true`（无相机时判为有障碍，fail-safe 正确） |
| `/safety/localization_valid` | 有 TF 时 `true`，稳定不翻转 |
| `/safety/stop_reason` | `obstacle_stop`，20 Hz 持续发布 |
| `/cmd_vel_safe` 频率 | 20 Hz（零速度） |
| 节点崩溃 | 0 |

注意：这些验证都在**没有连接底盘**的情况下完成，`/cmd_vel_safe` 上虽然以 20 Hz
发布零速度，但 relay 未运行、TX1 未收到任何指令，机器人不可能运动。

