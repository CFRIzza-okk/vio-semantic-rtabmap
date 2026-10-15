# Vio-Semantic-RTABMAP

[English README](README_EN.md)

基于 RTAB-Map 的实时视觉里程计、三维建图与语义标注系统：D435 同步采集彩色图像和对齐深度图，RTAB-Map 从 RGB-D 数据估计相机运动、生成地图并通过回环降低漂移；语义检测节点使用 YOLO 识别目标，再结合目标框内的有效深度将二维检测反投影为相机坐标系下的三维地标，语义地图节点通过多帧关联与置信度筛选形成稳定目标并发布标记。


## 运行模式

本项目有两种运行模式（PX4 模式和手持模式），按使用场景选择，**不要同时启动**：

| 场景 | 启动命令 | 里程计来源 | 展示 |
|---|---|---|---|
| PX4  | `~/VIO/launch_px4_mavros.sh` 后运行 `~/VIO/launch_d435_rtabmap.sh` | MAVROS `/mavros/local_position/odom` | RViz |
| 手持 D435  | `cd ~/VIO && ./launch_showcase.sh` | RTAB-Map RGB-D 视觉里程计 | RViz |
| 手持建图 + 远程 Foxglove | `cd ~/VIO && ./launch_showcase.sh --no-rviz`，另开终端运行 `./start_foxglove.sh` | RTAB-Map RGB-D 视觉里程计 | 另一台电脑上的 Foxglove |

手持展示使用 D435 的 RGB-D 视觉里程计；PX4 模式使用飞控里程计。这两种模式的里程计来源和 TF 发布方式不同，不能混用。切换模式前，先在原终端按 Ctrl-C 退出上一套流程，避免重复启动相机或 RTAB-Map 节点。

## R680 麦轮小车自主探索（ROS 2 + TX1 ROS 1）

项目已增加 R680 接入基础：

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select semantic_rtabmap
source install/setup.bash
ros2 launch semantic_rtabmap r680_navigation.launch.py \
  camera_x:=<实测x> camera_y:=<实测y> camera_z:=<实测z> \
  camera_roll:=<实测roll> camera_pitch:=<实测pitch> camera_yaw:=<实测yaw>
```

TX1 端仍由 ROS 1 Wheeltec 驱动负责底盘和底盘里程计；ROS 1–ROS 2 bridge 应只建立下面的接口：

- TX1 `/odom` → ROS 2 `/odom`（`odom_combined → base_footprint` 的动态 TF 只能由一个节点发布）；
- TX1 `/imu` → ROS 2 `/imu`；
- ROS 2 `/cmd_vel_safe` → ROS 1 `/cmd_vel`。

Nav2 的规划输出接 `/cmd_vel_nav`，安全节点再输出 `/cmd_vel_safe`。急停 `/safety/estop`、雷达障碍 `/safety/obstacle_stop` 和定位健康 `/safety/localization_valid` 任一不满足条件时输出零速度，并在 `/safety/stop_reason` 记录停车原因。不要让 Nav2 或 bridge 之外的节点直接连接底盘，也不要同时启动 PX4 的飞行器 TF。

`base_footprint → base_link` 和 `base_link → camera_link` 默认由本启动文件发布，但如果 TX1 驱动或 URDF 已发布其中一段，应将对应参数设为 `false`，避免重复 TF。D435 驱动负责 `camera_link → camera_color_optical_frame` 等相机内部 TF；相机安装位姿必须现场测量后再填入，不能复用 PX4 外参。

TX1 侧的驱动与 UDP relay 已完成**实机端到端验证**（2026-10-13）：在 TX1 上用合成 `/odom`
验证上行（值原样到达 ROS 2，实测 10.01 Hz），在 Orin 上向 `/cmd_vel_safe` 发指令验证下行
（包到达 TX1:19000，`sig ok`，且 `enable_cmd_vel` 保持关闭）。全程**未启动底盘驱动、未发送运动指令**。
relay 脚本在 `tools/ros1_relay.py`，部署与排障说明见 [tools/README_R680_RELAY.md](tools/README_R680_RELAY.md)。

⚠️ 此前这条链路**实际上是不通的**，有若干静默缺陷，均已修复并加了回归测试：

1. **端口配反**：两侧对 `rx_port`/`tx_port` 的语义相反（ROS 2 侧 `rx_port` 是本机绑定口，
   TX1 侧 `tx_port` 才是），照抄对端就会变成两端都绑 19000、都往对端 19001 发——无人监听，
   双向零包。现已配成互补的 19001/19000。守卫：`test/test_relay_ports.py`。
2. **TX1 脚本是 Python 3 代码**：TX1 是 Melodic + Python 2.7，脚本在
   `threading.Thread(daemon=True)` 处直接 `TypeError` 崩溃，**从未在车上成功运行过**。
   另有 `json.JSONDecodeError`、`time.monotonic`、`math.isfinite`、`except OSError`
   共五处 py3-only 写法，其中两处只在**开启运动权限后才触发**（正好是安全看门狗）。
   守卫：`test/test_tx1_relay_python2.py`。
3. **`/tf_static` QoS 违约**：relay 用 `create_publisher(TFMessage, "/tf_static", 10)`
   发静态 TF，而裸整数深度默认是 **VOLATILE**；`tf2_ros` 与本包 `safety_monitor_node`
   都按 `TRANSIENT_LOCAL` 订阅。ROS 2 里这是**不兼容**的一对，连接被直接拒绝，
   静态变换**一个也收不到**（`tf2_echo` 报 `DURABILITY_QOS_POLICY`）。已改为显式
   `TRANSIENT_LOCAL`。守卫：`test/test_relay_tf_static_qos.py`，其中一条专门护栏
   "接线"而非"常量"——把 `create_publisher` 改回裸整数即失败。
4. **`time_sync_client.py` 的 Python 3 写法与 shebang**（2026-10-14）。
   这条要**准确陈述，不要夸大**：脚本里有 7 处 `print(..., flush=True)`
   （`flush` 关键字是 Python 3.3+，在 2.7 上抛
   `TypeError: 'flush' is an invalid keyword argument for this function`），
   shebang 也写着 `python3`。
   ⚠️ **但它一直工作正常，时钟同步从未因此失效。** 实测 TX1 上是
   `r680-time-sync.service`（systemd，`Restart=always`，以 root 运行）用它自己的
   **`/usr/bin/python3`** 和 `--interval 10 --threshold 0.005` 拉起这个脚本——
   在 python3 下 `flush=` 完全合法，所以这条路径根本不触发该 bug。
   2026-10-14 01:06:54 的日志可见它把时钟一步校正了
   **+98787286 s（+1143 天）**，正是 TX1 断电后 RTC 回到 2000 年所需的那一跳。
   修复的价值在于**消除脆弱性**（任何人用 `python2` 调用、或让 shebang 生效、
   或把它放到只有 python2 的环境里，就会立刻崩在第一个 print 上），而不是"救活了
   一个死掉的功能"——**当初把它写成"从未成功运行过"是错的，此处更正。**
   ⚠️ **`py_compile` 抓不到这两处**：一个是运行时错误，一个只在直接执行时才生效。
   守卫：`test/test_tx1_relay_python2.py`，现按脚本参数化，并用 AST 检查
   `print(..., flush=...)`（用 AST 而非正则，是因为文件**文档字符串里本身就写着**
   这个陷阱的说明文字）。

   > 实测补充（2026-10-14）：TX1 晶振漂移约 **23 ppm**（连测 8 次，每 10 s 偏差
   > 单调增长约 0.228 ms，即 -0.065 → -1.664 ms）。因为服务阈值是 5 ms，它约每
   > **219 秒**才步进校正一次，于是时钟呈**锯齿**状：漂移到 5 ms → 阶跃 → 再漂移。
   > 这可以解释为什么在不同时刻测同一个偏移会得到 +2.5 / +4.4 / +5.1 ms 等不同值——
   > **一部分是锯齿，不全是测量误差**。5 ms 远小于 RTAB-Map 的
   > `wait_for_transform`（200 ms），所以它不足以解释那条 TF 警告，但排查时要知道
   > 这个锯齿存在，否则会把它误当作噪声或测量错误。
5. **`/scan` 从未产出过一帧**（2026-10-14 发现）：`depthimage_to_laserscan` 一直订阅
   `/depth_camera_info` —— 一个 **Publisher count: 0** 的话题，D435 从不发布这个名字，
   所以它每帧打印 `No camera info, skipping point cloud squash`。**危险的是下游症状**：
   探索配置里写明"没有 /scan 数据时按'障碍存在'处理"，于是 `obstacle_stop` 读作
   `true`，看起来像检测到了障碍，**实际是传感器全瞎**。
   修复的二次陷阱：remap 的**源名字必须是 `depth_camera_info`，不是 `camera_info`**——
   写成后者**不报错也不警告**，症状完全一样。
   > ⚠️ **同一次修复里我把 `output_frame` 改错了**（详见第 7 条）：当时改成
   > `camera_color_optical_frame`，理由是对齐深度图像及其 camera_info——**那个理由
   > 混淆了"数据表达在哪个 frame"和"LaserScan 角度语义要求哪种 frame"**，
   > 已回退为 `camera_depth_frame`。
   守卫：`test/test_r680_sensor_and_map_wiring.py`。
6. **地图话题名对不上**（2026-10-14 发现）：配置按 `/map` 写，但 RTAB-Map 的 namespace
   默认是 `rtabmap`，**实际发布 `/rtabmap/map`**。nav2 的 `static_layer` 未设
   `map_topic` 时默认订阅 `/map`，于是 **global_costmap 永远是空的**，规划器把未探索
   区域当作可通行——同样不报任何错。已在 nav2 static_layer、探索配置与探索启动文件三处
   显式写成 `/rtabmap/map`。守卫：`test/test_r680_sensor_and_map_wiring.py`，
   其中一条断言三处取值必须一致（当初正是"语义层知道 namespace、导航层不知道"才漏掉的）。
7. **`/scan` 的 `output_frame` 用错，差 90 度**（2026-10-14 发现并修复）。
   `depthimage_to_laserscan` 的 `output_frame` 曾被"修正"为
   `camera_color_optical_frame`，理由是"对齐深度图像和它的 camera_info 的 frame_id
   都是它"。**那个理由混淆了两件不同的事**：
   - 深度图像**数据**表达在哪个 frame —— 确实是 `camera_color_optical_frame`；
   - `LaserScan` 的**角度语义**要求哪种 frame —— **x-y 平面必须水平**（REP-103）。

   实测 TF（2026-10-14）：
   ```
   camera_link -> camera_depth_frame          RPY [0.000, 0.000, 0.000]（姿态同 camera_link）
   base_link   -> camera_depth_frame          RPY [0.000, 1.660, 0.000] 度
   base_link   -> camera_color_optical_frame  RPY [-91.653, 0.124, -90.678] 度
   camera_depth_frame -> camera_color_optical_frame  平移 15 mm，姿态差约 90 度
   ```
   两个 frame **差约 90 度姿态**（不只是当初以为的 15 mm 平移——**只看平移是漏看**）。
   `camera_depth_frame` 与 `camera_link` 姿态一致（x 前、y 左、z 上），x-y 平面是
   **水平的**；光学坐标系（z 前、x 右、y 下）的 x-y 平面是**像平面，竖直的**。
   具体判据：中心波束（angle 0、距离 r）即该 frame 下的点 `(r, 0, 0)` ——
   在 `camera_depth_frame` 里变换后是**正前方 r 米**，在光学坐标系里则**偏到侧下方**。

   **为什么这个错误能藏住**：安全节点 `obstacle_monitor_node` 直接读
   `msg.angle_min/increment`、**不做 TF 变换**，所以它不受影响（`obstacle_stop`
   一直表现正常，这才是它没被发现的真正原因）。但 **Nav2 的 costmap 障碍层会把
   scan 变换到全局系**，用光学坐标系就会把障碍摆到竖直方向、错 90 度，
   **在 costmap 里造出幽灵障碍**——一开 Nav2（Phase F 自主探索）就会中招。

   已回退为 `camera_depth_frame`。15 mm 的位置误差是真的，但**姿态错 90 度远比
   位置差 15 mm 严重**。守卫：`test/test_r680_sensor_and_map_wiring.py` 中断言
   frame 名不得以 `_optical_frame` 结尾。

8. **下行 `cmd_vel` 被 100% 丢弃：Python 2 的 `compare_digest` 拒绝 unicode**
   （2026-10-15 发现并修复）。第一次真车运动时**车完全不动**，而所有监控指标都
   健康：建图正常、里程计正常、`localization_valid=true`、`obstacle_stop=false`、
   TX1 侧甚至能看到**内容完全正确**的 `cmd_vel` 包（`seq` 一路涨到 5875）。

   根因是一行看起来毫无问题的代码：

   ```python
   hmac.compare_digest(outer["sig"], self.sign(raw))
   ```

   ```
   TypeError: 'unicode' does not have the buffer interface
   ```

   在 Python 2 上，`json.loads` 返回的 `outer["sig"]` **永远是 `unicode`**，而
   `sign()` 返回 `str` hexdigest，而 **Python 2 的 `hmac.compare_digest` 要求两个
   参数都是字节串**。于是**每一个**下行包都在校验这一步抛错。

   而它被 `except (ValueError, KeyError, TypeError)` 吞成一句泛化的
   `invalid relay packet`，与"JSON 畸形""签名错误"无法区分 —— 一次**完整的通信
   中断**被伪装成了网络背景噪声。这个诊断句本身就是缺陷的一部分，现已改为带
   **异常类型 + 报文前 200 字节**。

   **为什么能藏这么久：上下行不对称。** 上行（TX1→Orin）的验证方是 Python 3，
   `compare_digest` 接受 `str`，一直正常；只有下行 100% 失败。这与缺陷 3、7 是
   同一个形状 —— **两个解释器/两个坐标系之间的不对称，是这类缺陷的藏身之处。**

   而它此前从未被发现，是因为**这条路径从未被执行过**：Phase C/D 全是只读上行，
   Phase D 的定义就是"只选点不运动"。

   修法：两侧都显式 `.encode("ascii")` 后再比较。Orin 侧虽然能工作，但它依赖
   "sig 恰好是纯 ASCII"的隐含前提，且不对称本身就是 bug 的温床，故一并显式化。

   守卫：`test/test_relay_python2_compare_digest.py`（4 项）**在真实的 `python2`
   上驱动 relay 源码里的校验表达式**，而不是只做文本检查。变异测试 4/4 抓到。

9. **运动闸门"关不掉"：ROS 1 master 保留残留私有参数**（2026-10-15 发现并修复）。
   给车充电时按文档"不带 `_enable_cmd_vel` 重启 relay（默认就是关）"操作，**结果
   `/cmd_vel` 仍然有发布者** —— 闸门实际是**开的**。

   **ROS 1 的 master 会保留节点退出后的私有参数**（`SIGKILL` 也一样）。所以：

   1. 开闸启动 relay → master 存下 `enable_cmd_vel = true`
   2. 杀掉 relay
   3. **不带参数**重启 → 新进程同名节点读到**残留的 `true`**
   4. **闸门是开的，而操作者以为它是关的** —— 没有任何提示

   **这条已写进计划文档的安全回滚是假的。** 它的危险性在于：真出事时有人照着它去
   关闸门，会以为关了。

   修法：`tools/r680_gate.sh open|close|status` —— 先**删掉** relay 的全部私有参数
   （让残留值不可能被继承），启动时**显式**传标志，然后**校验 `/cmd_vel` 的真实
   发布者列表**。最后一步才是判决性的：它问 master"现在到底是什么在驱动底盘"，
   而不是相信参数、日志或脚本的意图。任何不符即**强制关闸**，宁可让车不能动，
   也不留一个说不清的状态。relay 启动时也会**大声打印**闸门状态（开/关都打印），
   因为原故障的表现形式就是沉默。

   守卫：`test/test_motion_gate_safety.py`（9 项），含"所有启动路径必须显式传
   闸门标志"与"Phase D 绝不能开闸"。

   > **顺带记两个会导致误判的测量陷阱**（不影响产品代码，但会浪费排查时间）：
   >
   > - **`ros2 topic pub -r 10` 会和 ROS 2 发现过程竞争**。2 秒的短脉冲可能整段在
   >   发现完成前结束，指令**根本没发出去**，现象却像"发了但没效果"（表现为
   >   `cmd_vel_safety` 只有孤立的 `motion output: ok`，0.5 秒后就是
   >   `cmd_vel_timeout`）。**必须加 `-w 1`** 等待匹配订阅者。
   > - **TX1 侧的清理与启动必须分成两次 ssh 调用**。写在同一条命令里时，`pkill`
   >   的模式会匹配到承载它的那个 shell 自己，shell 在启动行执行前就死了（现象：

10. **RTAB-Map 的 `odom_frame_id` 为空 —— 导致 Nav2 完全起不来**（2026-10-15 发现并修复）。
    第一次真正启动 Nav2 时，`controller_server` 是 `active`，而
    `planner_server` / `bt_navigator` / `behavior_server` / `waypoint_follower`
    全部停在 `inactive`。现象是"Nav2 起了一半"。

    日志里的根因是一句反复刷屏的 INFO（不是 ERROR，所以很容易被跳过）：

    ```
    [global_costmap] Timed out waiting for transform from base_footprint to map
                     to become available, tf error: Invalid frame ID "map"
                     passed to canTransform argument target_frame - frame does not exist
    ```

    **`map` 这个 frame 在整棵 TF 树里根本不存在。** 而 RTAB-Map 明明在跑、
    `/rtabmap/map` 也正常发布。原因是一个**空字符串**：

    ```
    rtabmap: odom_frame_id = ""       ← 空
    rtabmap: map_frame_id  = "map"
    ```

    RTAB-Map 要发布 `map → odom` 这个变换，就必须知道 odom frame 叫什么。
    `odom_frame_id` 为空时它**一个 `map→*` 的 TF 都不发**，于是：

    `map` 不存在 → global_costmap 永远等不到 → `planner_server` 卡在初始化 →
    `lifecycle_manager` 的启动顺序被阻塞 → 后面三个节点全部 inactive。

    修法：`odom_frame_id:=odom_combined`，即 **relay 实际发布的那个 frame**
    （实测 TX1 `/odom` 是 `frame_id: "odom_combined"`、`child_frame_id:
    "base_footprint"`）。这与 Nav2 配置里的 `global_frame: odom_combined` 和
    探索配置里的 `odom_frame: odom_combined` 也对上了。

    **⚠️ 这条修法与"不要开 `visual_odometry`"不是一回事，容易混淆：**
    这里设的是 RTAB-Map **相信**哪个 frame 是里程计坐标系；而
    `visual_odometry:=true` 会让 `rgbd_odometry` **自己再算一个里程计并发布
    `/odom`**，与 relay 转发的轮式里程计打架。后者仍然必须保持 `false`。
    实测两者独立：设了 `odom_frame_id` 之后 `/odom` 的发布者仍然是 1。

    **为什么 Phase D 从未暴露它：Phase D 不启动 Nav2。** 而探索器与
    `localization_monitor` 各有自己的判据，都不依赖 `map` 这个 TF 是否存在。
    这正是"一条从未被走过的路径"——与缺陷 8 是同一个模式。

    **排查提示**：`Invalid frame ID "map" ... frame does not exist` 是一句 INFO
    级别的日志，夹在生命周期启动的一堆输出里。它才是根因，而它长得像背景噪声。

11. **整栈启动脚本只清 TX1、不清 Orin —— 连跑两次会叠成两套**（2026-10-15 修复）。
    `r680_motion_up.sh` / 初版 `r680_explore_up.sh` 都只在 TX1 侧做清理，
    Orin 侧直接启动。连跑两次的实测结果：

    | 量 | 单次 | 连跑两次 |
    |---|---|---|
    | 节点总数 | 22 | **43** |
    | `/r680_ros2_relay` | 1 | **3** |
    | 每个 Nav2 节点 | 1 | **2** |

    后果不是"多几个进程"，而是**两套栈静静地互相打架**：两条 `/cmd_vel_safe`
    发布者竞争（速度指令交替被覆盖）、两个 `localization_valid` 发布者（标志
    可能一真一假）、两份 `/scan` 与 `/rtabmap/map`（costmap 收到重复输入）。

    **而没有任何一处会报错。** 验收检查只会显示一行
    `Publisher count: 2` —— 那是最容易被当成正常的一行。

    这与 `test_safety_monitors.py` 里那 3 个"假失败"是**同一个根因**
    （两个发布者在同一话题上打架），只是一个在软件测试里，一个在真机上。

    修法：Orin 侧在 source ROS 之后（`ros2 node list` 需要它）先做两轮
    `safe_kill`（TERM 再 KILL），并**断言清理后节点数为 0**。
   >  `ps` 无输出、日志是旧格式）。括号技巧 `"[r]os1_relay"` 只在**同一条命令里
   >   没有该名字的普通出现**时有效。

> 实测补充（2026-10-13）：TX1 的 `static_transform_publisher` 节点发的是 **`/tf`**，
> 不是 `/tf_static`；TX1 的 `/tf_static` 只有 `robot_state_publisher` 发出的一条**空**消息
> （URDF 里没有 static joint）。所以 relay 的静态通路上**无物可转**——QoS 修好只是让它
> 变成"正确的空转"。**Orin 的静态 TF 必须由 `r680_navigation.launch.py` 自备，不要依赖 relay。**
> 同一段过滤逻辑对 `/tf` 也生效，实测 Orin 的 `/tf` 帧表里确实没有 TX1 那个旧 `camera_link`。

> 实测补充（2026-10-14）：**阶段 C（建图与定位）已在真机上跑通**，且是这台车第一次
> 真正建立地图与定位链路。四个检查项全部通过：
>
> - `map→base_footprint` = `[-0.002, 0.002, 0.000]`，RPY `[0.004, 0.007, 0.102]`；
> - `/rtabmap/map` = `nav_msgs/OccupancyGrid`，**127×86 @ 0.05 m**（6.35×4.3 m）；
> - `localization_valid`：连采 183 个样本 **183 true / 0 false / 0 翻转**；
> - `/scan` **17.3 Hz**，614/640 有效波束，张角 55.4°，量程 1.04 / 2.86 / 4.52 m；
>   `obstacle_stop` 在空场为 **false**（手挡相机前应变 true）。
>
> 启动时 `camera_link` 实测 `[0.150, 0.000, 0.165]`、pitch `1.660°`，与实测外参逐位一致。
>
> **`localization_valid` 的"稳定"必须两端都验。** 只证明它不翻转是不够的：TX1 断电
> 那一刻它正确地从 `true` 翻成 **`false`**，这才证明监视器不是恒真、确实在跟随真实状态。
>
> 另有两条容易误判的观察：RTAB-Map 的 `/rtabmap/map` 是 **latch（`TRANSIENT_LOCAL`）**
> 发布的，`ros2 topic hz` 用的是 VOLATILE 订阅、**收不到** latch 消息，会误报
> "does not appear to be published yet"——用
> `ros2 topic echo /rtabmap/map --qos-durability transient_local` 才看得到。
> 以及静止的车会一直停在 `local map=2, WM=2`：RTAB-Map 只在地图移动超过
> `RGBD/LinearUpdate`/`AngularUpdate` 阈值时才新增节点，这是正常行为，不是故障。

> ⚠️ **数据库膨胀（2026-10-14 查清）。** 上面那条"静止不新增节点"只对**工作内存**
> 成立，对**数据库**不成立，两者必须分开看：静止的车每 0.5 秒仍会写入一个节点，
> 只是它没被链接进图、因此不进 WM（所以 `local map=1` 与 `Node=134` 同时为真）。
>
> **RTAB-Map 默认把这些未链接的节点连同 RGB/深度图一起存盘，这是给调试用的默认值。**
> 实测静止 60 秒（DetectionRate 2.0）：
>
> | 配置 | Node | Data | 库大小 |
> |---|---|---|---|
> | 默认 `Mem/NotLinkedNodesKept=true` | 134 | 134 | **4.5 MB** |
> | `Mem/NotLinkedNodesKept=false` | **1** | **1** | **272 KB** |
>
> **134 个节点里只有 1 个是真正被链接的。** 遗留库 `~/.ros/rtabmap_r680.db` 正是
> **1001 MB / 3280 节点 / 仅 38 条 Link**；同样的模式在 10月8日（130 MB）和
> 10月10日（593 MB）的库上已经出现过，所以这是长期问题，不是 Phase C 引入的。
>
> 修法：启动时加 `--Mem/NotLinkedNodesKept false`。RTAB-Map 作者
> （[introlab/rtabmap_ros#671](https://github.com/introlab/rtabmap_ros/issues/671)）
> 明确说这个默认值是给调试的，并建议**一直设成 false**，包括正式建图时。
> 可选再加 `--Mem/BinDataKept false` 不存图（库更小，代价是事后不能用
> `rtabmap-databaseViewer` 回看）。已有的大库可用官方工具瘦身：
> `rtabmap-reprocess --Mem/NotLinkedNodesKept false --Mem/BinDataKept false in.db out.db`。
>
> **排查陷阱**：不要在 RTAB-Map 运行时用 `sqlite3` 打开它的库——读连接持锁会让
> RTAB-Map 以 `DBDriverSqlite3.cpp:5034 addStatisticsQuery() … database is locked`
> 致命崩溃（退出码 -6）。**先停 RTAB-Map，再查库。**
>
> 另外：那条 `cannot get the corresponding TF odom_combined->base_footprint` 警告
> 在 2026-10-14 的复现中**没有再次出现**（计数 0），所以此前"`/odom` 与 `/tf` 走两条
> UDP 包导致结构性乱序"的猜测**缺乏证据支持**，已撤回。它更像是启动瞬间的一次性竞争。
> 实测 `odom_combined→base_footprint` 的 TF 更新率是 **60 Hz**，远高于 `/odom` 的
> 20 Hz——若真是结构性落后，不该只出现一次。

> `/odom` 协方差 1e6 的来源已定位（2026-10-14）：**在 TX1 的驱动，不在 relay。**
> `/odom` 由 `/wheeltec_robot` 发布（`/robot_pose_ekf` 只是它的订阅者，另行发布
> `/robot_pose_ekf/odom_combined`），relay 是忠实转发的。实测对角项
> （x/y/z/roll/pitch/yaw）：
>
> | x | y | z | roll | pitch | yaw |
> |---|---|---|---|---|---|
> | 1e-09 | 0.001 | **1e6** | **1e6** | **1e6** | **1e-09** |
>
> **1e6 精确地落在 z/roll/pitch 上——正是平面轮式底盘真正无法观测的三个自由度；
> yaw 反而是 1e-09（极度自信）。** 所以这是语义上**正确**的表达，不是驱动偷懒。
> RTAB-Map 的 `Very large angular variance (1000000.000000)` 因此指的是 roll/pitch，
> 不是 yaw。
>
> 是否要改尚未决定。可选的思路是给这三个自由度填一个有限值（等价于断言"车始终贴地"，
> 这是常见建模假设，但车若真的倾斜就会失真），而不是保留"完全未知"。
> 在决定之前，这个警告的实际影响仅限于 RTAB-Map 对 roll/pitch 的置信度处理——
> Phase C 期间 `localization_valid` 183/183 稳定，未见实际损害。

> 时钟更正（2026-10-14，留档以免重蹈）：阶段 C 期间曾用"跨 SSH 会话括起 `date`"估出
> TX1 偏 **+240 ms**，并据此把 RTAB-Map 的一条 TF 查询警告归因于时钟。**该数字是错的。**
> SSH 连接/认证/解释器启动的开销被整个算进了"偏移"，且它并不对称落在括起中点。
> 改用协议方式（四个时间戳，可减掉 RTT 与服务器处理时间）在**同一 SSH 会话内**连测 6 次，
> 得 **+2.483 ~ +2.514 ms，散布 46 µs，RTT ~0.5 ms**——真实偏移 **+2.5 ms**，比
> RTAB-Map 的容差小两个数量级，**不可能是那条警告的原因**。该警告的真因**尚未确定**
> （已排除时钟；怀疑 `/odom` 与 `/tf` 走两条独立 UDP 路径、到达顺序无保证）。
>
> 旁证：`localization_valid` 183/183 稳定；若真偏 240 ms，`localization_monitor`
> 的 0.5 s 超时早就会周期性把它打成 false。
>
> 时钟现状：TX1 的 RTC 停在 **2000 年**且不可写；`ntpd` 已装但未运行；
> `systemd-timesyncd` 在跑但 `System clock synchronized: no`（从未同步成功）。
> 因此 `tools/time_sync_client.py` 是**唯一**的校正途径——而它此前因
> `print(..., flush=True)` 在 2.7 上崩溃，从未成功运行过（见缺陷 4）。

### 自主导航探索

在 relay 之上增加了基于 frontier 的自主探索能力，完整策略、参数与分阶段验证流程见 [R680_AUTONOMOUS_EXPLORATION.md](R680_AUTONOMOUS_EXPLORATION.md)：

- `frontier.py`：frontier 检测、连通性过滤、评分、失败黑名单（纯算法，无 ROS 依赖，可单测）；
- `exploration_node.py`：探索状态机，只下发 Nav2 目标，**从不直接发布速度**；
- `safety_monitor_node.py`：障碍监视器与定位监视器，为安全节点提供 `/safety/obstacle_stop` 与 `/safety/localization_valid`；
- `r680_exploration.launch.py`：把 D435 对齐深度转成 `/scan`，拉起安全节点，并可选拉起 Nav2；
- `r680_nav2_bringup.launch.py`：本工程专用 Nav2 启动。

**为什么不用官方 `nav2_bringup/navigation_launch.py`**：实测它的速度接线是
`controller_server → /cmd_vel_nav`，但 `behavior_server` 和 `velocity_smoother`
都直接发布 `/cmd_vel`，会绕过 `cmd_vel_safety`。本工程改为让
`controller_server` 与 `behavior_server` 的输出**都**汇入 `/cmd_vel_nav`，
且不启动 `velocity_smoother`，保证整条链路里唯一发布 `/cmd_vel_safe` 的节点是
`cmd_vel_safety`。

默认最保守：`dry_run:=true` 且 `autostart:=false`，启动后只选点、不驱动底盘。

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 先只验证选点是否合理，机器人不会动
ros2 launch semantic_rtabmap r680_exploration.launch.py dry_run:=true
ros2 service call /exploration/start std_srvs/srv/Trigger

# 确认选点合理、场地空旷且有人守候后，再放开运动
ros2 launch semantic_rtabmap r680_exploration.launch.py dry_run:=false
```

### D435 点云避障（观测阶段）

D435 目前是整车唯一的避障传感器。除了现有的"对齐深度 → `/scan`"，还可以让相机直出
`PointCloud2` 喂给 Nav2 的 `VoxelLayer`，用**高度阈值**（而不是猜 `scan_height`）剔除地面，
从而看见低矮障碍和桌沿。实测结论、两个静默失败的坑和待应用配置见
[R680_DEPTH_CLOUD_OBSTACLES.md](R680_DEPTH_CLOUD_OBSTACLES.md)。

两个会**静默失败**的坑（都已实测确认）：

- 点云开关**不叫** `pointcloud.enable`。realsense2_camera 的前缀取自 librealsense 处理块名，
  本机是 `Pointcloud (NEON)` → 真实参数名 **`pointcloud__neon_.enable`**。传错的后果是节点照常启动、
  话题照常出现在 `topic list` 里，但**一个发布者都不建**且不报错。查证：
  `ros2 param list /camera/camera | grep -i point`。
- **强制指定 profile**（如 `640,480,30`）会让相机帧超时（`Frames didn't arrived within 5 seconds`），
  点云一帧都出不来；用相机默认的深度 848×480 才正常。

只读观测工具（不发速度、不写 costmap）：

```bash
ros2 launch semantic_rtabmap r680_depth_cloud.launch.py
ros2 topic echo /depth_cloud/report --once --truncate-length 100000
```

`point_cloud_inspector_node` 会报告频率/点数/高度分布，并由地面平面反推**相机离地高度与俯仰角**
（即 `base_link -> camera_link` 缺的外参）。注意外参目前仍是 `0`（未标定），
在填入实测值之前 `min_obstacle_height` 只能是占位符。

测试（不需要机器人、不需要相机）：

```bash
python3 -m pytest src/semantic_rtabmap/test/ -q      # 94 passed
```

## 安装

在项目目录打开终端并运行：

```bash
cd ~/VIO
./install_native.sh
```

脚本会调用 `sudo apt-get`，将启动脚本安装到 `/usr/local/bin/launch_d435_rtabmap`，安装 `rtabmap_d435.rviz` 布局，并为 Intel RealSense USB 设备安装 udev 权限规则。完成后拔下并重新连接相机，使新规则生效。

语义工作空间如尚未编译：

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
```

Foxglove 远程展示需要安装 bridge（只需安装一次）：

```bash
sudo apt update
sudo apt install -y ros-humble-foxglove-bridge
```

## 手持 D435 实时建图与语义展示

```bash
cd ~/VIO
./launch_showcase.sh
```

脚本会依次检查并启动 RealSense D435、相机外参、RTAB-Map RGB-D 视觉里程计与建图、语义检测和语义地图节点，并等待节点和话题就绪后才报告启动完成。默认启动 RViz、创建新的展示数据库，并采用保守的 `uniform` 地标协方差模式；默认语义类别为 `chair,bottle,backpack,suitcase,laptop`，排除了会移动的人体，降低动态目标被当作静态地标的风险。

常用参数：

```bash
./launch_showcase.sh --help
./launch_showcase.sh --no-rviz       # 不在 Jetson 本机启动 RViz
./launch_showcase.sh --fast          # 降低视觉里程计计算量，CPU 紧张时尝试
./launch_showcase.sh --keep-db       # 继续使用上次展示数据库
./launch_showcase.sh --classes=chair,bottle  # 自定义 COCO 类别白名单
```

默认数据库位于 `~/.ros/rtabmap_showcase.db`，语义地图位于 `~/VIO/data/showcase_semantic_map.json`，日志位于 `~/VIO/data/showcase_logs/`。默认启动会将已有展示数据库和语义地图备份后从零开始，避免不同场次的地图数据混合。退出时在启动终端按 Ctrl-C，节点将保存数据库和语义地图。

### 使用 Foxglove 远程显示

在 终端运行：

```bash
cd ~/VIO
./launch_showcase.sh --no-rviz
```

另开 终端启动 Foxglove Bridge：

```bash
cd ~/VIO
./start_foxglove.sh
```

在同一局域网内的另一台电脑打开 Foxglove Studio 桌面版，连接脚本打印的地址，例如 `ws://192.168.5.4:8765`。Foxglove 的 3D 面板 Fixed Frame 设为 `map`，可添加：

- `/rtabmap/cloud_map`：全局三维点云地图（主要显示）；
- `/rtabmap/map`：二维占据栅格；
- `/rtabmap/odom_local_map`：局部里程计点云；
- `/tf`、`/tf_static`：动态和静态坐标变换。

要显示语义标注，再添加 `/semantics/map_markers`、`/semantics/label_markers` 和 `/semantics/markers`。只想看原始建图时，不添加或隐藏这些语义话题。Foxglove 不支持 RViz 的 RTAB-Map 专用插件（如 MapCloud/MapGraph）；全局点云应使用标准消息话题 `/rtabmap/cloud_map`。按 Ctrl-C 分别停止 Bridge 和建图脚本。

### 图像采集建议

缓慢、平稳移动相机，保持画面有纹理和稳定光照；避免快速旋转、运动模糊、长时间对着白墙或窗户。尽量走闭环并回到起点附近，帮助触发回环检测。D435 在强光、反光、透明物体和远距离区域的深度可能不可靠。若使用 `--fast` 后里程计或地图细节变差，应恢复默认设置。

### 录制 rosbag 后离线建图

如果携带相机离开局域网，可以先录制 RGB-D 数据，回来后在本机离线回放建图：

```bash
cd ~/VIO
./record_mapping_bag.sh --name=消防走廊
```

录制脚本会启动手持模式，保存彩色图像、对齐深度图、相机内参、`/odom` 和静态 TF 到 `~/VIO/data/bags/`。回放时由 RTAB-Map 重新计算地图坐标系，避免录入旧的动态 TF。完成一圈后在录制终端按 Ctrl-C，等待脚本提示 rosbag 已保存。

名称对应的目录已存在时，脚本会在启动相机之前直接退出，并列出已有 rosbag 的信息：

```text
错误：/home/jetson/VIO/data/bags/消防走廊 已存在，ros2 bag record 不会写入已存在的目录。

  已有 rosbag：6.7G
    Bag size:          6.7 GiB
    Duration:          161.137383678s
    Messages:          16250

  换一个名称：./record_mapping_bag.sh --name=消防走廊2
  或覆盖它：  ./record_mapping_bag.sh --name=消防走廊 --overwrite
```

这是 `ros2 bag record` 自身的行为：`-o` 指定的目录已存在时会报 `Output folder ... already exists` 并立即退出。要覆盖旧数据必须显式加 `--overwrite`，脚本不会自动删除已有 rosbag。

### 录制期间的 VIO 发散监测

录制脚本会同时启动视觉里程计监测，输出直接显示在录制终端里。一旦视觉里程计持续失效，终端会立即打印：

```text
⚠️  VIO发散
```

默认判定条件是「连续 3 秒没有有效位姿」且「累计 10 个坏帧」，两者同时满足才算发散，避免快速转身或短暂丢帧造成误报。发散恢复后会打印 `VIO已恢复`，录制结束时会打印统计汇总，包括收到帧数、坏帧数、发散次数和累计里程。

出现 `VIO发散` 时应当放慢移动、把相机移回有纹理的区域；若整段录制持续发散，该 rosbag 中的 `/odom` 会全程是无效零位姿，无法用于离线建图。监测日志同时保存在 `~/VIO/data/bags/<名称>_vio_monitor.log`。

不需要监测时：

```bash
./record_mapping_bag.sh --name=消防走廊 --no-monitor
```

也可以单独运行监测节点调整灵敏度：

```bash
python3 ~/VIO/tools/vio_divergence_monitor.py --grace 20 --sustain 3 --min-bad-frames 10
```

回到本机后，不启动 RealSense，直接回放：

```bash
cd ~/VIO
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊
```

不启动本机 RViz 或降低回放速度：

```bash
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊 --no-rviz
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊 --rate=0.5
```

回放脚本使用 rosbag 中的图像、深度、相机内参、录制时的 `/odom` 和静态 TF，由 RTAB-Map 重建地图；每次回放默认生成新的数据库。回放时使用 rosbag 的时钟，不回放旧的动态 `/tf`。录制和回放时不要同时运行另一套 RealSense、RTAB-Map 或 rosbag 进程。

### 查看 rosbag 和 database

用 `rtabmapviz` 在回放时实时观察：

```bash
cd ~/VIO
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊 --viz --no-rviz
```

`rtabmapviz` 会显示当前帧、特征点、局部地图和位姿图；不需要 RViz 时可以只开它。

用 RTAB-Map 自带的 database 查看器打开已保存的数据库：

```bash
source /opt/ros/humble/setup.bash
rtabmap-databaseViewer ~/.ros/rtabmap_showcase.db
```

界面上主要看这几项：`Graph View`（位姿图和回环）、`3D Map`（点云/栅格）、`Statistics`（节点数、处理耗时）、以及数据库里的节点和词袋信息。

不开图形界面时，可以用下面两条命令先确认数据库里有没有内容：

```bash
source /opt/ros/humble/setup.bash
rtabmap-info ~/.ros/rtabmap_showcase.db
```

```bash
python3 -c "
import sqlite3
c = sqlite3.connect('file:$HOME/.ros/rtabmap_showcase.db?mode=ro', uri=True)
print('Node =', c.execute('select count(*) from Node').fetchone()[0])
"
```

注意：如果 `Node = 0`，说明这份数据库里没有任何关键帧，`databaseViewer` 只会显示空图。视觉里程计发散时录制的 bag 在回放后就会得到这种空库，此时应先按上面的 VIO 监测提示重新录制，而不是反复回放。

### 盘点与抢救历史数据库

RTAB-Map 的数据库会散落在 `~/.ros/`、备份目录，以及 `rtabmap_*.db.bak.*` 这类自动备份里。注意 **`ls ~/.ros/*.db` 匹配不到 `.db.bak.*`**，很容易误判成“没有数据”。用盘点工具一次列全：

```bash
cd ~/VIO
python3 tools/scan_rtabmap_dbs.py
```

它会逐个检查大小、节点数、轨迹范围、里程、回环数、地标数、`Data`/`Feature` 是否完好，以及 SQLite 完整性，并给出可直接运行的 `rtabmap-databaseViewer` 命令。

数据库如果在建图还没结束时被 `cp`/`rsync`，会出现 `database disk image is malformed`，`databaseViewer` 会直接崩掉。通常只有 `Data`/`Feature`/`Statistics` 这几张大表损坏，而 `Node`（位姿）、`Link`（回环与地标）、`Word`（词袋）仍然完好。用恢复工具把可读的表搬到一个新库：

```bash
python3 tools/recover_rtabmap_db.py 损坏的.db 恢复后的.db
```

恢复后的库能看轨迹、回环和地标，但**没有节点图像和 3D 点云**（那部分数据在损坏的表里）。

不用打开图形界面也可以导出轨迹、回环和语义地标并画图：

```bash
python3 tools/export_rtabmap_map.py ~/.ros/rtabmap_showcase.db.bak.20261007_193738 \
  --out 轨迹.png --csv 轨迹.csv
```

输出的图包含俯视轨迹（按时间着色，叠加回环连线）、高度随时间变化、语义地标位置和统计信息。地标位置按 `世界坐标 = R_node × t_landmark + t_node` 计算；同一地标的多帧观测离散度超过 `--landmark-max-spread`（默认 0.25 m）的会被标为不可信。

还可以直接用命令行读取数据库内容：

```bash
source /opt/ros/humble/setup.bash
rtabmap-info ~/.ros/rtabmap_showcase.db.bak.20261007_193738
```

## PX4 飞行器模式：启动 MAVROS

本项目采用 PX4 飞控，使用 CUAV Pixhawk v6X V2（USB ID `3163:004c`），连接设备名为 `/dev/ttyACM0`。
启动 MAVROS ：

```bash
~/VIO/launch_px4_mavros.sh
```

脚本启动 `px4_tf.launch.xml`，加载 PX4 默认插件列表、`px4_config.yaml` 和 `px4_tf_config.yaml`。MAVROS 默认的 `local_position.tf.send` 为 `false`，因此这里的 TF 参数覆盖用于启用 world→body 动态变换。

在另一个终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 topic hz /mavros/imu/data                # 预期约 50 Hz，取决于飞控设置
ros2 topic hz /mavros/local_position/odom     # 预期约 26 Hz，取决于飞控设置
```

## PX4 模式：启动 D435 + RTAB-Map

将 D435 接入 USB 3.0 端口，使用 `lsusb` 确认设备已识别，然后运行：

```bash
~/VIO/launch_d435_rtabmap.sh
```

该脚本依次：

1. 发布静态 `base_link`→`camera_link` 外参，平移为 `(0.045, -0.075, -0.005)` m，旋转为零；
2. 启动 RealSense，输出彩色、深度及对齐后的深度图；
3. 启动 `rtabmap_launch/rtabmap.launch.py`，使用 `frame_id:=base_link`、`odom_topic:=/mavros/local_position/odom`、`visual_odometry:=false`、`publish_tf_map:=false`，并加载 `~/VIO/rtabmap_d435.rviz`。

## TF 结构与地图不可见排查

world→body 动态变换只能由一个节点负责。本项目 PX4 模式的 TF 树为：

```text
map ──(MAVROS local_position.tf.send:=true)──> base_link
    ──(static)──> camera_link ──> camera_*_frame ──> camera_color_optical_frame
```

- MAVROS 发布 `map`→`base_link` 动态变换；
- RTAB-Map 使用飞控提供的 `/mavros/local_position/odom`，并设置 `publish_tf_map:=false`，避免同时发布 `map` TF；
- 启动脚本发布 `base_link`→`camera_link`，RealSense 驱动发布相机内部 TF。

不要给 `rtabmap.launch.py` 传入 `odom_frame_id:=map`。它不是选择里程计来源的参数；错误设置会导致 RTAB-Map 尝试发布 `map`→`map` 自变换，tf2 以 `TF_SELF_TRANSFORM` 拒绝，RViz 中相关显示会因缺少 TF 而报错。

检查 PX4 模式 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```


## RViz 布局

`rtabmap_d435.rviz` 基于 RTAB-Map RGB-D 布局，并添加了 `/rtabmap/cloud_map` 全局点云显示。PX4 模式下 Fixed Frame 为 `map`，Orbit 视图的 Target Frame 使用 `base_link`：PX4 局部原点可能离飞行器较远，若视图围绕 `map` 原点旋转，点云可能落在视野之外。布局包含占据栅格 Map、RTAB-Map 插件 MapCloud/MapGraph/Info、Odom Local Map、Camera Cloud、RTAB-Map Cloud、TF、Grid 和语义标记显示。

只看原始建图时，在 Displays 面板关闭 `Semantic Map`、`Semantic Detections`、`Semantic Labels` 和 `Semantic Colored Cloud`；保留 `RTAB-Map Cloud`、`MapCloud`、`Odom Local Map` 等建图显示项。

## 常见问题

- **没有点云或 TF 显示报错**：检查 `ros2 run tf2_ros tf2_echo map base_link`，确认启动了正确模式，且没有同时启动相互冲突的 TF 发布节点。
- **`TF_SELF_TRANSFORM` 且 frame_id 与 child_frame_id 都是 `map`**：检查 RTAB-Map 启动参数，不要设置 `odom_frame_id:=map`；PX4 模式还应确认 `publish_tf_map:=false`。
- **出现重复节点或两个 `/mavros`**：先停止旧实例；多个 MAVROS 实例会争用飞控串口。
- **地图倾斜、偏移或闭环异常**：PX4 本地原点会随会话变化，不要直接混用旧数据库。可先备份再从零开始：

  ```bash
  mv ~/.ros/rtabmap.db ~/.ros/rtabmap.db.bak.$(date +%s)
  ```

- **点云已发布但 Foxglove/RViz 中不可见**：确认 Fixed Frame 为 `map`、目标视角在地图附近，并检查对应话题是否有消息。
- **语义标注没有出现**：检查 YOLO 模型文件路径、语义节点日志和 `/semantics/objects`、`/semantics/map_markers` 话题。

## 使用的相机外参

当前配置采用的坐标约定为：x 向右、y 向前、z 向上；IMU 位于相机光学中心左侧 7.5 cm、后方 4.5 cm、上方 0.5 cm，且假设两者坐标轴平行（零旋转）。转换到 ROS 机体坐标轴（x 向前、y 向左、z 向上）后，相机在 `base_link` 中的位置为 `(+0.045, -0.075, -0.005)` m。该值应视为当前标定配置，正式测量或飞行前需在 RViz 中核验方向和几何关系。

## 常用文件

- `install_native.sh`：原生主机依赖与 udev 安装脚本；
- `launch_showcase.sh`：手持 D435 实时语义建图展示脚本；
- `launch_handheld_mapping.sh`、`run_once.sh`：手持建图及实验运行脚本；
- `launch_d435_rtabmap.sh`：PX4 里程计模式下的相机、RTAB-Map 和 RViz 启动脚本；
- `start_foxglove.sh`：Foxglove Bridge 启动脚本；
- `rtabmap_d435.rviz`：RViz 配置；
- `launch_px4_mavros.sh`、`px4_tf.launch.xml`、`px4_tf_config.yaml`：MAVROS 启动及 TF 参数覆盖。

## 参考资料

- [MAVROS Humble 文档](https://docs.ros.org/en/ros2_packages/humble/api/mavros/)及[上游源码](https://github.com/mavlink/mavros)
- [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros)
- GeographicLib 大地水准面数据集：`/usr/share/GeographicLib/geoids/egm96-5.pgm`
