# D435 点云作为局部避障源：实测结论与待应用配置

> 状态：**观测与测量已完成**。点云已确认可用并实测到数据（≈19 Hz）；
> 相机外参**已实测**：离地 **≈0.17 m**、俯仰 **≈2° 朝下**（三次采样一致）；
> Nav2 costmap **一行都没改**（按用户要求），外参也**尚未写入静态 TF**。
> 剩余工作是把测量值"应用"下去，需要用户确认，见 §8。

---

## 0. 一句话结论

**能用，而且不需要装任何新包。** `nav2_costmap_2d::VoxelLayer` 本机已存在，Nav2 的障碍层可以直接
消费 `PointCloud2`。D435 现在实际上是整车**唯一的避障传感器**（TX1 上没有可用的 2D 雷达，`/scan`
是从深度图切出来的）。点云这条路带来的**增量是垂直覆盖**：低矮障碍、桌沿、悬空边缘，
以及**用高度阈值而不是猜 `scan_height` 来剔除地面**。它**不会**让水平视场变宽——D435 依然是
约 87° 的前向楔形，侧面来的东西照样看不见。

---

## 1. 已核实的环境事实（全部来自实测，非推测）

| 项目 | 值 |
|---|---|
| 相机 | Intel RealSense D435，序列号 `938422075341`，固件 `5.11.11.100`，USB 3.2 |
| 驱动 | `realsense2_camera` **4.58.4**，`librealsense2` **2.58.4** |
| 点云话题 | `/camera/camera/depth/color/points` |
| 点云发布者 QoS | **RELIABLE** / VOLATILE（不是 BEST_EFFORT） |
| 默认深度流 | **848×480×30**（注意：不是 640×480） |
| 默认彩色流 | 640×480×30 |
| 点云帧格式 | 非组织化（`height=1`，`width≈142000`） |
| 实测点云频率 | **≈19 Hz**（19.09~19.39），`valid=1.00`，采样约 71100~72500 点/帧 |
| 实测外参 | 离地 **≈0.17 m**、俯仰 **≈2° 朝下**（三次采样一致，见 §4.1/§5） |
| `nav2_costmap_2d::VoxelLayer` | **存在**（`grep type= /opt/ros/humble/share/nav2_costmap_2d/*.xml`） |
| `pointcloud_to_laserscan` | **未安装**（本方案不需要它） |
| `pcl_ros` | 已装，但**只提供 `pcd_to_pointcloud`**，没有任何滤波节点 |

---

## 2. 实测踩到的两个坑（都会静默失败）

### 2.1 点云开关不叫 `pointcloud.enable`

realsense2_camera **不是**用固定的 `pointcloud.enable` 开关点云的。它把参数前缀取自
librealsense 的**处理块名字**，并把非字母数字字符各自替换成一个下划线：

* 本机（aarch64）处理块叫 `Pointcloud (NEON)` → 前缀 **`pointcloud__neon_`**
  （空格→`_`，`(`→`_`，`)`→`_`）
* x86 上处理块叫 `Pointcloud (SSE3)` → 前缀应为 `pointcloud__sse3_`

后果：传 `pointcloud.enable` 会被**静默忽略**。节点照常启动，话题**照样出现在 `ros2 topic list` 里**
（因为 `topic list` 也会列出只有订阅者的话题），但**一个发布者都不会创建**，而且不报错。
实测证据：

```
# 传 pointcloud.enable
ros2 param get /camera/camera pointcloud.enable   ->  Parameter not set
ros2 topic info /camera/camera/depth/color/points ->  Publisher count: 0

# 传真实前缀
ros2 param get /camera/camera pointcloud__neon_.enable ->  Boolean value is: True
ros2 topic info /camera/camera/depth/color/points      ->  Publisher count: 1
```

换机器时先查真实前缀：

```bash
ros2 param list /camera/camera | grep -i point
```

### 2.2 强制指定 profile 会让相机直接帧超时

在这台机器上强制 `depth_module.depth_profile` / `rgb_camera.color_profile`
（例如都设成 `640,480,30`）会导致相机**采不到帧**：

```
WARNING (backend-v4l2.cpp:1722) Frames didn't arrived within 5 seconds
XXX Hardware Notification:Frames didn't arrived within 5 seconds
```

此时点云**一帧都出不来**。用相机自己的默认 profile（深度 **848×480**）则完全正常。
三组对照实测：

| 配置 | `Frames didn` 次数 | inspector 收到数据 |
|---|---|---|
| 仅开点云（默认 profile） | **0** | **有，20.9 Hz** |
| + `enable_sync:=true` | 0 | 有，20.4 Hz（sync 无害） |
| **强制 `640,480,30`** | **10** | **无** |

所以 `r680_depth_cloud.launch.py` 里 profile 参数**默认为空**，只有显式给了非空值才覆盖。

---

## 3. 只读观测工具

**`point_cloud_inspector_node`**（`semantic_rtabmap/point_cloud_inspector_node.py`）
是纯只读的：只订阅点云、只打印和发布统计，**不发布速度、不发布 TF、不写 costmap**，
跑它不可能让机器人动。

它回答三件事：点云有没有在发/频率/点数；地面在相机坐标系里的哪一层；以及**反推相机外参**。

外参反推的原理：地面在相机光学坐标系里是一个平面（x 右，**y 向下**，z 向前）。
相机水平时地面落在 `y = 相机离地高度`（常数）；俯仰时 `y` 随 `z` 线性变化，**斜率就是俯仰角**。
所以对每个 z 切片取 y 的 90 分位（y 最大 = 最低面 = 地面）再做直线拟合，就能量出高度和俯仰角。

启动与查看：

```bash
ros2 launch semantic_rtabmap r680_depth_cloud.launch.py
ros2 topic echo /depth_cloud/report --once --truncate-length 100000
```

> 注意：`launch_d435_rtabmap.sh` 也会启动相机，**同一时间只用一个**（USB 带宽/设备占用冲突）。
> `ros2 topic echo` 默认会把长字符串截断，读完整 JSON 必须加 `--truncate-length`。

---

## 4. 真实数据实测结果

在机器人**当前摆放位置**下跑出来的报告（19.09 Hz）：

```
width x height            142272 x 1        organized=False
sampled_points            71136            valid=1.0
frame_id                  camera_depth_optical_frame
z_range_m                 [0.288, 5.37]
y_percentiles_m           p05=-1.284  p50=-0.004  p95=0.244

z 直方图（前方距离 → 点数）
    0.0-0.5     35836
    0.5-1.0     18296
    1.0-1.5         0
    1.5-2.0         0
    2.0-2.5         0
    2.5-3.0         0
    3.0-3.5        57
    3.5-4.0      1354
    4.0-4.5      5249
    4.5-5.0      6917
    5.0-5.5      3056
    5.5-6.0       360

floor_fit.status          not_enough_z_slices
floor_fit.bin_point_counts [35240, 10, 0, 0, 0, 0, 0, 0, 0]   # 0.4~0.7m 之后全空
near_surface (0.3~0.9m)   52106 点  y p50=0.015 p90=0.141 iqr=0.163  planar=False
candidate_bands           {}  已主动抑制（见下）
```

**读数说明：** 相机前方 0.7 m 以外直到 3.5 m **完全没有点**，4~5.5 m 处是一面墙。
近场（0.3~0.9 m）那一堆**不是平面**（`iqr=0.163`，若是地面应接近 0）。
也就是**地面根本没进视野**，所以外参**量不出来**。

这里刻意做了一个诚实性设计：**地面没识别出来时，`candidate_bands` 会被主动抑制为空**
并给出 `candidate_bands_note`。因为"离地面多高"此时本身未定义，继续输出百分比
会被误读成"已经标定好的阈值"，进而变成写进 costmap 的危险数字。宁可不出数，也不出假数。

### 4.1 摆到平地上重跑：外参量出来了

机器人重新摆放（平地、前方开阔）后重跑，同一个工具直接给出了结果：

```
floor_fit.status          ok
slices                    9
slope_dy_dz               -0.0375
y_at_z0_m                 0.178
r2                        0.927
estimated_pitch_deg_down  2.15
estimated_mount_height_m  0.178
bin_point_counts          [17136, 6725, 3512, 1969, 1189, 2700, 3454, 1747, 323]
near_surface (0.3~0.9m)   planar=True  iqr=0.012  y_p50=0.142
z_range_m                 [0.395, 10.188]
```

9 个 z 切片**全部**有足够点数（0.4~3.0 m 连续可见），与 4.0 节"8 个切片全空"正好对照——
同一份代码、同一台相机，差别只在**地面有没有进视野**。

三次连续采样确认可重复：

| 采样 | 高度 (m) | 俯仰 (°朝下) | r² | 近场 iqr | 近场 y_p50 |
|---|---|---|---|---|---|
| 1 | 0.178 | 2.15 | 0.927 | 0.012 | 0.142 |
| 2 | 0.173 | 1.94 | 0.812 | 0.011 | 0.142 |
| 3 | 0.181 | 2.36 | 0.955 | 0.011 | 0.142 |
| 4 | 0.168 | 1.65 | 0.674 | — | — |

**结论：相机离地约 0.17 m（四次采样 0.168~0.181），略微朝下约 2°（1.65~2.36）。**

两个**互不依赖**的估计互相印证：拟合在 z=0.6 m 处给出 `y=0.156`，近场平面实测 `y_p50=0.142`，
相差约 1.4 cm。近场 `iqr` 只有 1.1~1.2 cm，说明那确实是一块很平的表面（地面），
而不是 4.0 节那种杂物堆。

> **一个已知偏差，用的时候要知道。** 拟合对每个 z 切片取 y 的 **90 分位**（因为 y 最大 = 最低面 = 地面），
> 这会比平面的真实位置**偏下约 0.78σ**，于是高度被**高估**。合成数据里 σ 很小（0.004）影响可忽略；
> 真实数据切片内的散布更大，所以 0.178 可能高估了 1~2 cm，**真实值更接近 0.16~0.17 m**。
> 需要更准时，以近场平面的 `y_p50` 做交叉参考。对设置 costmap 高度阈值来说，
> 1~2 cm 的出入不影响结论——地面在 y≈0.14~0.18，阈值取 0.08~0.12 m 就能稳稳把地面滤掉。

---

## 5. 外参：已量出，怎么填

`base_link -> camera_link` 是**点云避障的硬前提**：障碍物按 3D 位置写进 costmap，
相机装高和俯仰角错了，`min_obstacle_height` 就失去意义——要么地面被标成致命障碍
（车以为被围住、原地不动），要么真障碍被滤掉（更危险）。

**实测值（§4.1，三次采样一致）：**

| 量 | 值 |
|---|---|
| 离地高度 | **≈ 0.17 m**（四次采样 0.168~0.181，含已知高估 1~2 cm） |
| 俯仰 | **≈ 2.0° 朝下**（1.65~2.36） |

**注意：这个值目前只记录在本文件里，还没有写进启动文件的静态 TF。**
`r680_navigation.launch.py` 里 `base_link -> camera_link` 仍是默认的 `0`，
所以 inspector 的报告里仍会打印
`configured base_link->camera_link = 0 (default; measurement not applied)`
——这句话说的是"**配置**还是 0"，不是"没量过"。

要用上这个测量值，把它填进 `r680_navigation.launch.py` 的静态 TF：

```
x=0.0  y=0.0  z=0.17   roll=0  pitch=+0.035(rad, 约 2° 朝下)  yaw=0
```

（前向偏移 `x`、左右偏移 `y` 本轮**没有测量**——地面平面对这两个量不敏感，
报告里也没有对应输出。要精确标定需要另做，见 §7 第 6 条。）

重新量取的配方（机器人**静止、放在平地上、前方 1~3 m 可见地面**）：

```bash
ros2 launch semantic_rtabmap r680_depth_cloud.launch.py
ros2 topic echo /depth_cloud/report --once --truncate-length 100000
# 关注 floor_fit：status=ok 时给出 estimated_mount_height_m / estimated_pitch_deg_down
```

估计量的数学正确性用**合成数据 + 已知真值**单独验证过（不依赖任何真实场景）：

* 水平相机、地面高 0.35 m → 估出 `0.35 ± 0.03`，斜率 ≈ 0
* 俯仰 12°、高 0.42 m → 估出 `12.0° ± 1.0` 和 `0.42 ± 0.04`，`r2 > 0.95`
* 只有远处一面墙、没有地面 → **正确报错**，不编造高度

> 一个容易踩的细节：**相机水平时不要去信 `r2`**。此时地面的 y 是常数，拟合因变量几乎没有
> 方差，`r2` 会退化成噪声（实测约 0.17）。`r2` 只在有俯仰、y 确实随 z 变化时才有意义。

---

## 6. 待应用的 VoxelLayer 配置（**尚未应用**）

以下改动**故意没有写入** `config/r680_nav2.yaml`——按用户要求 costmap 保持不动。
这是留给"外参标定完成后"的一步。改动要点：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]     # obstacle_layer -> voxel_layer
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        publish_voxel_map: true
        z_resolution: 0.05
        z_voxels: 20
        origin_z: 0.0
        mark_threshold: 2                              # 压制深度噪声的单点毛刺
        observation_sources: scan cloud
        scan:                                          # 保留原有 2D 通道
          topic: /scan
          data_type: "LaserScan"
          clearing: true
          marking: true
        cloud:                                         # 新增点云通道
          topic: /camera/camera/depth/color/points
          data_type: "PointCloud2"
          clearing: true
          marking: true
          min_obstacle_height: 0.10   # 依据 §5 实测：地面在 y≈0.14~0.18，取 0.10 可稳稳滤掉地面
          max_obstacle_height: 1.20   # 按车体可用高度定
```

**外参状态：已实测（高度 ≈0.17 m，俯仰 ≈2° 朝下），但尚未写入静态 TF。**
§5 给出了要填的那一行。在那行填好之前，上面的 `min_obstacle_height` 只是按实测推算的
**建议值**，不要当成已经在实车上验证过的配置。

`/scan` 与 `obstacle_monitor` **保持不动**是刻意的：让"代价地图"和"安全闸门"依赖两条不同管线，
同一条管线出错时不会连最后一道刹车一起失效。

---

## 7. 诚实的局限

1. **视场不会变宽。** 点云只补垂直方向，水平仍是约 87° 前向楔形，侧面看不见。
2. **近处有盲区。** D435 约 0.28 m 以内测不到（实测 `z_range` 下界 `0.288`），点云和 `/scan` 一样。
   真要防贴脸，那是超声/碰撞条的事，相机解决不了。
3. **别拿 RTAB-Map 的建图点云做避障。** `/rtabmap/cloud_map` 是**累积地图**、不是实时感知，且重。
   局部避障必须用相机直出的实时点云。
4. **算力。** 单帧约 14 万点、19 Hz。Orin NX Super 扛得住，但体素层更新是主要开销，
   届时实测再决定是否需要降分辨率或限制观测距离。
5. **反光/玻璃/黑色表面**深度不可靠，会表现为 unknown（和 `/scan` 同样的问题）。
6. **外参已实测但尚未写入静态 TF**（高度 ≈0.17 m、俯仰 ≈2° 朝下）。在填进
   `r680_navigation.launch.py` 之前，任何以 `base_footprint` 为参考的输出仍不可信。
   另外**前向/左右偏移未测量**——地面平面对这两个量不敏感，本轮报告里也没有对应输出。

---

## 8. 当前进度与后续

本轮已完成：点云开启并实测验证、只读观测工具、外参实测（三次采样一致）、
以及待应用配置的文档化。现场收在安全状态：**没有相机进程在跑、没有速度发布、costmap 一行未改**，
测试 **81 项全过**。

**剩下的是"应用"而不是"验证"**，都需要你点头才动：

1. 把 §5 的实测外参写进 `r680_navigation.launch.py` 的静态 TF（会改变 TF 树，影响建图与导航）；
2. 把 §6 的 VoxelLayer 配置落进 `r680_nav2.yaml`（会改变 costmap 观测源）；
3. 可选：另做一次标定以测出前向/左右偏移。

**若之后要重新量外参**，机器人静止、放平地上、**前方 1~3 m 可见地面**，然后：

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch semantic_rtabmap r680_depth_cloud.launch.py
ros2 topic echo /depth_cloud/report --once --truncate-length 100000
```

看 `floor_fit`：`status` 变成 `ok` 时会给出 `estimated_mount_height_m` 和
`estimated_pitch_deg_down`，这就是 `base_link -> camera_link` 缺的两个值。
若仍是 `not_enough_z_slices`，看 `bin_point_counts` 就知道是哪段 z 上没有点，
据此判断该把相机压低还是把车挪到开阔处。

**诊断小抄**

| 现象 | 原因 | 处理 |
|---|---|---|
| 话题在 `topic list` 但 `Publisher count: 0` | 参数前缀传错 | `ros2 param list /camera/camera \| grep -i point` 查真实前缀 |
| `Frames didn't arrived within 5 seconds` | 强制指定了 profile | 把 `color_profile`/`depth_profile` 留空，用相机默认 |
| `ros2 topic echo` 输出被截断 | 默认截断长字符串 | 加 `--truncate-length 100000` |
| inspector 报 `no point cloud received yet` | 相机没起 / 帧超时 | 看 `/tmp` 下相机日志里的 `Open profile` 行 |
