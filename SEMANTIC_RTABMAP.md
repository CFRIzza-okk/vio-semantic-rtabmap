# 语义 RTAB-Map：给 SLAM 地图加上物体语义

面向毕业设计的工程实现。在 Jetson Orin NX + ROS 2 Humble + PX4/MAVROS +
RealSense D435 平台上，为 RTAB-Map 增加语义感知、语义建图和语义导航能力。

**当前进度：阶段 1、2 已完成并在真实相机上验证；阶段 3、4 待实现。**

---

## 1. 技术路线

采用「先外挂、后改核」的两步走策略：

| 阶段 | 内容 | 状态 |
|---|---|---|
| 1 | 外挂语义感知层：YOLO 2D 检测 → 深度反投影 → 3D 语义物体 | ✅ 完成，22 Hz 实测 |
| 2 | 外挂语义建图层：多帧数据关联 → 持久语义地图 → 接入 RTAB-Map 地标 | ✅ 完成并验证 |
| 3 | 语义导航：语义目标点查询 → 对接 PX4/MAVROS 飞控 | ⬜ 待实现 |
| 4 | 改 RTAB-Map 核心：语义辅助回环检测等增量创新 | ⬜ 待实现 |

**为什么先外挂**：外挂层可以独立调试、独立验证，出问题能快速定位到是感知、
关联还是 SLAM。等外挂层稳定后，再动 RTAB-Map 源码，此时任何退化都能确定
是核心改动引入的。这个"先建立可信基线，再引入变量"的思路本身也是论文里
实验设计章节可以写的方法论。

---

## 2. 系统架构

```
                      ┌──────────────────────────────────────┐
   RealSense D435     │  detector_node  (Phase 1, Python)     │
   ┌────────────┐     │                                      │
   │ color      │────▶│  1. YOLO11n TensorRT 推理  19 ms     │
   │ aligned    │────▶│  2. 深度 ROI 鲁棒估计 + 一致性聚类    │
   │ depth      │────▶│  3. 反投影得 3D 质心/尺寸/朝向        │
   │ camera_info│────▶│  4. TF 变换到 map 帧                  │
   └────────────┘     └──────────────┬───────────────────────┘
                                     │ /semantics/objects
                                     │ (SemanticObjectArray, instance_id = -1)
                                     ▼
                      ┌──────────────────────────────────────┐
                      │  mapper_node  (Phase 2, Python)       │
                      │                                      │
                      │  1. 类别门控 + 尺寸自适应门限          │
                      │  2. 匈牙利算法全局最优分配             │
                      │  3. 位置/尺寸置信度加权融合            │
                      │  4. 朝向圆均值 (处理 180° 歧义)        │
                      └───┬──────────┬──────────┬────────────┘
                          │          │          │
     /semantics/landmarks │          │          │ /semantic_mapper/query
     (rtabmap_msgs)       │          │          │ (QuerySemanticMap)
                          ▼          ▼          ▼
                   ┌──────────┐  ┌────────┐  ┌──────────────┐
                   │ RTAB-Map │  │  RViz  │  │ navigator    │
                   │ 图优化    │  │ 可视化  │  │ (Phase 3)    │
                   └──────────┘  └────────┘  └──────┬───────┘
                                                   │ MAVROS setpoint
                                                   ▼
                                                 PX4
```

### 目录结构

```
~/VIO/
├── semantic_ws/                          # 语义层工作空间（与 ros2_ws 独立，避免重编 mavros）
│   ├── src/semantic_rtabmap_msgs/        # 自定义消息与服务
│   │   ├── msg/SemanticObject.msg
│   │   ├── msg/SemanticObjectArray.msg
│   │   └── srv/QuerySemanticMap.srv
│   └── src/semantic_rtabmap/             # 节点实现
│       ├── semantic_rtabmap/
│       │   ├── backprojection.py         # 2D→3D 几何反投影
│       │   ├── association.py            # 数据关联 + 置信度融合
│       │   ├── detector_node.py          # Phase 1
│       │   └── mapper_node.py            # Phase 2
│       ├── config/{semantic,mapper}.yaml
│       └── launch/semantic.launch.py
├── launch_semantic_demo.sh               # 一键启动（相机 + 语义层）
├── tools/                                # 验证与测试脚本
│   ├── test_backprojection.py            # 几何单元测试（22 项）
│   ├── test_association.py               # 关联单元测试（40 项）
│   ├── bench_yolo.py                     # 模型速度基准
│   ├── verify_engine.py                  # TensorRT vs PyTorch 一致性验证
│   ├── capture_frame.py                  # 抓取测试帧
│   └── snapshot_debug.py                 # 抓取调试图
├── rtabmap_landmark_label_interfaces.md  # RTAB-Map 语义接口源码级调研报告
└── SEMANTIC_RTABMAP.md                   # 本文件
```

---

## 3. 快速开始

### 编译

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2
```

> 用 `--parallel-workers 2` 是因为 Orin NX 只有 8GB 内存，默认并行度会 OOM。

### 运行

```bash
# 离线自测：发布静态 map->base_link 代替 PX4 里程计
~/VIO/launch_semantic_demo.sh

# 接 PX4 实机：先在另一终端跑 MAVROS，再执行
~/VIO/launch_px4_mavros.sh
~/VIO/launch_semantic_demo.sh px4
```

### 验证

```bash
source ~/VIO/semantic_ws/install/setup.bash

# 语义物体（单帧，instance_id = -1）
ros2 topic hz /semantics/objects

# 关联后的物体（instance_id 已填）
ros2 topic echo /semantics/objects_tracked

# 语义地图可视化
ros2 topic echo /semantics/map_markers

# 发给 RTAB-Map 的地标
ros2 topic echo /semantics/landmarks

# 语义查询（导航节点的接口）
ros2 service call /semantic_mapper/query semantic_rtabmap_msgs/srv/QuerySemanticMap \
  "{class_name: 'apple', instance_id: 0, max_distance: 0.0, use_query_origin: false}"
```

### RViz

需要手动加三个 Display（现有 `rtabmap_d435.rviz` 里没有）：

| Topic | Type | 说明 |
|---|---|---|
| `/semantics/map_markers` | MarkerArray | 语义地图：立方体 + 标签 + 连线 |
| `/semantics/debug_image` | Image | 带检测框和距离的调试图 |
| `/semantics/objects` | SemanticObjectArray | 原始检测（可选） |

---

## 4. 关键技术点（可直接写进论文）

### 4.1 深度反投影：为什么不能只用框中心像素

见 `backprojection.py`。三步策略：

1. **ROI 收缩**：只在检测框中心 60% 区域采样。RealSense 深度图在物体边缘会
   发生 depth bleeding（前景深度"渗出"到背景像素），边界像素不可信。
2. **鲁棒深度**：取 ROI 内有效深度的 **20 分位数**而不是均值。背景总是更远，
   低分位数等价于"偏向物体前景"。
3. **深度一致性聚类**：以鲁棒深度 z₀ 为中心，保留 |z − z₀| ≤ 0.35 m 的像素，
   认为它们属于同一物体表面，再求 3D 质心。比单像素反投影稳定得多，且顺带
   给出可见尺寸。

**已知局限**（写进"局限性"一节）：单视角深度只能观测物体朝向相机的一面，
`size.z` 是"可见表面厚度"而非真实深度；`yaw` 单帧不可靠，需多视角融合。

### 4.2 数据关联：避免过度分裂与错误合并

见 `association.py`。语义 SLAM 最核心的一环，两类典型故障：

- **过度分裂**：同一物体被当成多个 → 地图上出现 5 个杯子
- **错误合并**：多个物体被当成一个 → "去左边的杯子"导航到错误目标

采用「类别门控 + 尺寸自适应门限 + 全局最优分配」：

| 机制 | 作用 |
|---|---|
| 类别门控 | 只有同名类别才可能关联，排除绝大部分误关联 |
| 尺寸自适应门限 | `gate = 0.6 + 0.75 × 物体尺度`；杯子(0.15m)→0.71m，汽车(3.5m)→3.0m(截断) |
| 匈牙利算法 | 全局最优一一匹配，物体密集时优于贪心 |
| 圆均值处理朝向 | 朝向是**轴向量**（180° 歧义），用加倍角法求均值 |

### 4.3 置信度融合：一个必须记录的失败教训

**这是本项目最有价值的踩坑记录，建议直接写进论文的"问题与解决"章节。**

最初用朴素贝叶斯**对数几率(log-odds)累加**融合多帧置信度：

```python
log_odds += log(p / (1 - p))
```

**为什么它是错的**：对数几率累加要求各次观测**相互独立**。但相机以 20 Hz
连续观测同一个静止物体，相邻帧高度相关——本质上是同一份证据被重复计入
几百次。

**实测后果**：一个检测置信度稳定在 0.35 的真物体，因为 0.35 < 0.5 对应的
对数几率为负，每帧都在"投反对票"。600 帧后融合置信度被压到 **6.1×10⁻⁶**，
物体直接从语义地图上消失（`confirmed_tracks()` 返回空，地标话题也是空的）。
反向的问题同样存在：单帧 0.7 的偶然误检会被推到 ≈1.0，变成"铁证"。

**正确做法**：置信度加权平均 + 轻度收缩，并明确其语义：

```
conf = (Σ cᵢ² + c_prior · k) / (Σ cᵢ + k)
```

- `k = 1.0`、`c_prior = 0.35`：样本少时被先验拉住，样本多时收敛到观测均值
- **这个量的语义是"平均检测质量"，不是"物体存在的概率"**。重复观测不会把
  它推向 1.0。物体"是否存在"由最少观测次数门槛负责。把两件事分开是有意
  为之：否则"看到 600 帧但每帧只有 0.35"和"看到 3 帧但每帧 0.95"会得到
  同一种结果，而这两种情况该有完全不同的处理。

回归测试 `test_association.py::1b` 专门锁死了这个 bug：600 帧 0.35 置信度
的观测，融合置信度必须 > 0.3。

### 4.4 实时性设计：邮箱模式解耦推理

见 `detector_node.py`。在 Orin NX 8GB 上 RTAB-Map 本身要吃 CPU/GPU。如果在
同步回调里直接跑 YOLO，订阅队列会堆积、延迟越滚越大，最终拖垮整个 SLAM。

采用**最新帧邮箱 + 单推理线程**：同步回调只把最新一帧塞进邮箱，推理线程取走
处理。处理不过来时旧帧直接丢弃，延迟恒定，不会雪崩。实测效果：

```
处理 22.5 Hz | 丢帧 3 | 推理 19.0 ms | 总耗时 40.0 ms
```

### 4.5 实测性能数据

**检测精度与推理速度**

| 项目 | 数值 |
|---|---|
| YOLO11n TensorRT FP16 | **19–20 ms/帧（约 50 FPS）** |
| YOLO11n PyTorch FP32 | 31–34 ms/帧（约 30 FPS） |
| TensorRT 加速比 | **1.53–1.79×** |
| engine vs pt 框偏差 | 1.87 像素（IoU 0.968，类别一致） |
| engine vs pt 置信度偏差 | 0.064（**FP16 量化的固有现象**，非引擎损坏） |

### 4.5.1 全链路帧率：先测对指标，再谈优化

这一节记录一次**方法论错误**，因为它的教训比结论本身更重要。

**踩的坑**：用 `ros2 topic hz` 测相机帧率，得到"只有 10~18 Hz，远低于 30 Hz"
的结论，于是开始怀疑算力不足、CPU 饱和、socket 缓冲区太小，甚至打算为此换用
GPU 方案。这些都是错的。

**原因**：`ros2 topic hz` 是一个 **Python 订阅者**。640×480×3 的彩色图是 921 KB，
30 Hz 时它需要以 27.6 MB/s 的速度反序列化成 Python 对象。它自己就是整条链路上
最慢的环节，测出来的数字是"我的探针收不过来"，不是"相机发不出来"。

**正确做法**：把「产出率」和「投递率」分开测。

1. **产出率** = 用消息头时间戳的间隔反推。时间戳是随帧生成的，只要收到任意几条
   就能算出真实产出频率，完全不受丢帧影响。工具：`tools/measure_topic_rate.py`
2. **投递率** = 用一个 **C++ 订阅者**（`ros2 bag record` 就是）计数，它在 30 Hz
   下毫无压力。

**修正后的真实数据**（全栈运行中，C++ 记录 20.7 秒）：

| 话题 | 计数 | 速率 | 结论 |
|---|---|---|---|
| 彩色（相机产出） | 618 / 20.7 s | **29.9 Hz** | 相机满帧，完全健康 |
| 对齐深度 | 590 / 20.7 s | **28.5 Hz** | 健康 |
| `/odom`（VO 输出） | 165 / 20.7 s | **8.0 Hz** | ← 真正的瓶颈 |

用时间戳法单独验证相机：产出率中位数 **30.0 Hz**（相邻帧时间戳差 33.4 ms，
精确等于 1/30 秒）。**传输层没有问题**——FastRTPS 同机通信走的是共享内存
（`/dev/shm` 下可见 `fastrtps_*` 段），根本不经过 UDP，所以 socket 缓冲区
大小与丢帧无关。

### 4.5.2 真正的瓶颈是视觉里程计

RTAB-Map 自己会打印每帧耗时，这是最直接的证据：

```
Odom: quality=406, std dev=0.000360m|0.012320rad, update time=0.064149s delay=0.105484s
Odom: quality=423, std dev=0.000200m|0.010360rad, update time=0.070793s delay=0.131632s
```

| VO 配置 | 单帧耗时 | 备注 |
|---|---|---|
| 默认（`Vis/MaxFeatures 800`，无降采样） | **60–84 ms** | 理论上限约 14 Hz |
| `--fast`（500 特征 + `Odom/ImageDecimation 2`） | 启动初期 **15–52 ms** | 降采样确实有效 |
| `--fast`，运行约 1 分钟后 | **150–340 ms** | ⚠ 严重退化，待查 |

**尚未解决：VO 输出率异常偏低**。已排除的可能原因（每一个都做过对照实验）：

| 假设 | 验证方法 | 结论 |
|---|---|---|
| 相机帧率不足 | 时间戳反推产出率 | ❌ 产出恒为 **30.0 Hz**，从未变过 |
| 传输层丢帧 | C++ bag 记录对照 | ❌ 投递 29.8 Hz，健康 |
| 检测节点抢资源 | 杀掉 `detector_node` 后复测 | ❌ VO 耗时未恢复 |
| 累积的 360MB 数据库 | 移走数据库用空库跑 | ❌ 反而更差（3.6 Hz） |
| 彩色/深度未同步 | 时间戳对齐测量 | ❌ 83% 帧在 5ms 内对齐，`enable_sync` 无改善 |
| **编译进程抢占资源** | 停掉编译后重测 | ⏳ 待验证（最可疑） |

实测对照（相机产出率在所有条件下都稳定为 30.0 Hz）：

| 条件 | 相机产出 | VO 输出 |
|---|---|---|
| 360 MB 数据库，`--fast` | 30.0 Hz | 8.0 Hz |
| 空数据库，`--fast` | 30.0 Hz | **3.6 Hz** |

> ⚠ **一条方法论教训**：上面这些实验里有几次是在子代理**正在编译 RTAB-Map**
> 的时候做的（`cc1plus` 常驻 700MB+、可用内存一度只剩 1 GB）。这种条件下测出的
> 绝对数值没有意义——这和我前面用 `ros2 topic hz` 测出假数据是同一类错误：
> **在受干扰的环境里测量，然后为假数据找原因**。
>
> 所有性能数字都必须在**无编译、无其它重负载**的条件下重测一遍才算数。


### 4.5.3 内存占用（8GB 机型的关键约束）

| 进程 | 常驻内存 | 说明 |
|---|---|---|
| `detector_node` | **1058 MB** | 最大户。模型权重只有 5 MB，其余几乎全是 PyTorch + ultralytics 的框架开销 |
| `rtabmap` | 353 MB | |
| `rgbd_odometry` | 181 MB | |
| `mapper_node` | 157 MB | |
| `realsense2_camera` | 91 MB | |

整机 7619 MB，这套流水线合计约 1.84 GB，加上桌面环境（Chrome/NoMachine/Xorg
约 1.5 GB）后余量很小，实测已出现 **swap 使用 1.7 GB** 的换页。

> **优化方向**：`detector_node` 的 1 GB 是明显的浪费。YOLO11n 的 TensorRT engine
> 已经构建好了（`~/ultralytics/ultralytics/yolo11n.engine`），如果直接用
> `tensorrt` Python 绑定加载 engine、自己实现 letterbox 前后处理，就可以完全
> 不 import torch，预计能省下 600–700 MB。这是当前性价比最高的优化，而且是
> **纯 GPU 路径**（推理本来就在 GPU 上，去掉的只是 CPU 侧的框架负担）。

> 关于置信度偏差：`half=True` 导出的 engine 是 FP16，`.pt` 跑 FP32，同一目标
> 置信度相差 5–8% 属正常。**实测验证**：重建引擎前后给出完全相同的 0.709，
> 与 FP32 的 0.773 稳定相差 0.064，证明这是量化效应。实际影响是：如果
> `conf_threshold` 恰好卡在某个目标的置信度附近，FP16 可能判为漏检。建议
> 阈值设在 0.35–0.45，避开常见置信度密集区。

---

## 5. RTAB-Map 语义接口：源码级核查结论

完整报告见 [`rtabmap_landmark_label_interfaces.md`](rtabmap_landmark_label_interfaces.md)（1124 行，每条结论带源码行号）。
以下是接入时**必须知道**的几条，每一条都踩过坑：

### ⚠️ 五个关键事实

1. **没有 `landmark_topic` 参数**，订阅名是硬编码的**相对名**
   `landmark_detections`。实测该节点的 landmark 相关参数只有三个：
   `landmark_linear_variance` / `landmark_angular_variance` /
   `Optimizer/LandmarksIgnored`。

   ⚠️ **`rtabmap.launch.py` 的 `tag_topic` 不能用来接它**。那个参数 remap 的是
   apriltag 的 `tag_detections`（`apriltag_msgs/AprilTagDetectionArray`），
   与 `rtabmap_msgs/LandmarkDetections` 没有任何关系。本项目一度按
   `tag_topic:=/semantics/landmarks` 接，实测**完全接不上**。

   ⚠️ **因为是相对名，实际话题名 = RTAB-Map 节点命名空间 + `/landmark_detections`**。
   `rtabmap.launch.py` 的 `namespace` 默认值是 `'rtabmap'`（该文件第 451 行），
   所以走官方 launch 起时真正被订阅的是 **`/rtabmap/landmark_detections`**；
   只有直接 `ros2 run rtabmap_slam rtabmap`（无命名空间）才是
   `/landmark_detections`。

   本项目做法：`mapper_node` 的 `rtabmap_landmarks_topics` 参数（逗号分隔）
   默认同时发这两个话题，两种启动方式都能接上，不必依赖别人的启动脚本。
   实测 `/rtabmap/landmark_detections`：publisher 1（`semantic_mapper`）
   / subscription 1（`rtabmap` @ namespace `/rtabmap`），QoS 双方都是
   RELIABLE + VOLATILE，**已连通**。

2. **订阅端 QoS 是 RELIABLE，不是 SensorDataQoS**。如果用 BEST_EFFORT 发布，
   RTAB-Map 会**完全收不到消息且不报任何错**。本项目的
   `LANDMARK_QOS` 已显式指定 `KeepLast(1) + RELIABLE + VOLATILE`。

3. **`header.frame_id` 是位姿的参考系**，RTAB-Map 会用
   `lookupTransform(frame_id 参数(默认 base_link), header.frame_id, stamp)`
   把它变换到 base_link，再做一次 base→odom 运动补偿。
   **TF 查询失败时该 landmark 被静默丢弃**（只有日志里的 UERROR）。
   本项目默认按官方约定发布在**相机光学系**（与 apriltag_ros 一致），
   让 RTAB-Map 走它自己的标准变换链；直接发 map 系坐标会让运动补偿
   被重复施加一次。可用 `landmarks_frame_mode` 参数切换。

   ⚠️ **实测踩坑：按图像时间戳精确查 `map→相机光学系` 必然失败。**
   RTAB-Map 的 `map→odom` 是按它自己的处理节奏发布的（实测延迟
   0.3~0.45 s，`tf_delay=0.05`），而地标的 `header.stamp` 是图像时间戳——
   检测刚算完的那一刻，那个时刻的 `map→odom` 根本还没发布，于是每次
   都抛 `ExtrapolationException`，地标**全部**退化成 map 系发布（正是上面
   说的"运动补偿被重复施加一次"）。修法是两步查询：先按时间戳精确查，
   失败再退化成"取缓冲区里最新的那个"（`Time()` 无参）。
   退化带来的误差 ≈ 相机在这段延迟内的位移（手持 ~0.5 m/s × 0.4 s ≈ 0.2 m），
   与 `tag_linear_variance:=0.04`（σ=0.2 m）同量级，可以接受。
   实测修完后 `frame_id` 稳定为 `camera_color_optical_frame`，退化警告 0 次。

4. **`id` 必须 >0 且全局稳定唯一**。RTAB-Map 内部把它当作"负数节点 id"使用，
   **没有任何外观/几何重识别或合并逻辑**。同一物体换个 id 就会变成两个互不
   关联的地标顶点，地图重复且永远拿不到回环收益。id 还会被写进数据库，
   跨 session 必须一致。本项目用 `class_id × 100000 + instance_id` 编码
   （实测：apple 类 47，实例 1 → id = 4700001），既全局唯一，也能从 id
   反解类别。

5. **`Marker/Variance*` 参数对话题接入方案完全无效**。一旦输入里已经带了
   landmark，RTAB-Map 内部的 MarkerDetector 会被整段跳过。实际生效的是
   `landmark_linear_variance` / `landmark_angular_variance`，而
   `rtabmap.launch.py` 把这两个参数暴露成 `tag_linear_variance` /
   `tag_angular_variance`，默认值 **0.0001 / 9999.0**。

   ⚠️ **默认的 0.0001（σ=1 cm）是给 AprilTag 的精度，对语义地标太小**：
   YOLO 检测框中心 + 深度反投影出来的位置误差远大于 1 cm，沿用默认等于
   把地标当硬约束，会把整张地图拉歪。本项目在
   `launch_handheld_mapping.sh` 里显式传 `tag_linear_variance:=0.04`
   （σ=20 cm），让地标作为软约束参与图优化；
   `tag_angular_variance` 保持默认 9999.0，即优化时忽略地标朝向
   （YOLO 估的 yaw 不可靠）。

### 节点标签（SetLabel）

- 服务名 `/<ns>/rtabmap/set_label`，**响应为空**，成败只能看日志
- `node_id=0`：建图模式下指最后一个工作节点；定位模式下指半径内最近节点
- 标签**图内唯一**，`label=""` 即删除
- **会持久化**到数据库 `Node.label` 列
- `Mem/MapLabelsAdded`（默认 true）**与 SetLabel 无关**，它只管自动给每个
  地图首节点打 `"map0"/"map1"`。想要干净的标签列表就设成 false

### 0.23.7 没有第一类语义物体抽象

`rtabmap_msgs` 中**没有 `Objects.msg` / `add_object`**；核心 `Landmark` 类
只有 `id / size / pose / covariance`，**没有类别名字段**。
这正是"外挂语义层"存在的价值：类别信息由我们维护。
替代方案是把类别编码进 id（本项目采用），或用节点标签（粒度是"地点"而非"物体"）。

---

## 6. 测试

```bash
cd ~/VIO

# 几何反投影（22 项）
python3 tools/test_backprojection.py

# 数据关联与置信度融合（40 项）
python3 tools/test_association.py

# 模型速度基准
python3 tools/bench_yolo.py

# TensorRT vs PyTorch 一致性（需先 python3 tools/capture_frame.py）
python3 tools/capture_frame.py
python3 tools/verify_engine.py
```

两个测试脚本都不依赖 ROS 运行时，可以直接跑，适合做回归。

---

## 7. 已修复的问题清单

开发过程中遇到并修复的问题，累计 10 个：

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | `colcon build` 报 `Invalid email` | `jetson@localhost` 缺顶级域名 | 改为 `jetson@example.com` |
| 2 | 话题永远无数据，`ros2 node info` 里没有订阅者 | `message_filters.Subscriber` 第一个参数必须是节点，漏传导致内部 `args[2]` 抛 IndexError | 改为 `Subscriber(self, Image, topic, ...)` |
| 3 | 相机节点刷一屏 "Parameter not supported" | realsense 的 `rs_launch.py:124` 会把上下文里**所有** launch 参数塞给相机节点 | 把相机拆到独立 launch 调用 |
| 4 | 指定了 `.engine` 却悄悄用 `.pt`，还从 GitHub 下载 5.4MB | ultralytics 对不存在的路径会"猜模型名并下载"而不报错 | 加载前显式 `os.path.isfile` 检查，失败则明确报错 |
| 5 | `KeyError: 0` | 把轨迹列表下标当字典键用 (`self.tracks[ti]`) | 改用 `candidates[ti]` |
| 6 | 语义地图里物体消失，地标话题为空 | 朴素对数几率累加把 0.35 置信度的真物体压到 6×10⁻⁶ | 改为置信度加权平均 + 轻度收缩（详见 4.3） |
| 7 | `size`/`yaw` 与 `position` 坐标系不一致 | `position` 变换到 map 帧，但 `size`/`yaw` 留在相机系 | 三者一起变换（尺寸用 `abs(R) @ size`） |
| 8 | 地标发出去 RTAB-Map 完全收不到，且**没有任何报错** | 订阅名是**相对名** `landmark_detections`，而 `rtabmap.launch.py` 的 `namespace` 默认 `'rtabmap'`，实际话题是 `/rtabmap/landmark_detections` | `mapper_node` 新增 `rtabmap_landmarks_topics` 参数（逗号分隔），默认同时发 `/rtabmap/landmark_detections` 与 `/landmark_detections` |
| 9 | 按 `tag_topic:=/semantics/landmarks` 接，接不上 | `tag_topic` remap 的是 apriltag 的 `tag_detections`，与 `rtabmap_msgs/LandmarkDetections` 无关 | 不再走 `tag_topic`，改为直接发到 RTAB-Map 硬编码订阅的那个话题（见 #8） |
| 10 | 每次发布地标都告警 `TF ... 不可用 (ExtrapolationException)`，地标全部退化成 map 系，被重复施加一次运动补偿 | 按**图像时间戳**精确查 `map→相机光学系`，而 RTAB-Map 的 `map→odom` 滞后 0.3~0.45 s，那一刻还没发布 | 两步查询：先按时间戳，失败退化成"取缓冲区最新的"（`Time()` 无参）；配合 `tag_linear_variance:=0.04` 让误差量级可控 |

---

## 8. 下一步计划

### 阶段 3：语义导航（`navigator_node`）

1. 订阅 `QuerySemanticMap` 服务，实现 `{"class": "apple", "action": "goto"}` 目标解析
2. 从语义地图取最近实例，生成期望位置（保持安全距离 + 飞行高度）
3. 通过 MAVROS 发布 `setpoint_position/local` 或走 `rtabmap_msgs/Goal`
4. 关键点：PX4 的 local_position 原点和 map 帧的一致性；到点判断与悬停

### 阶段 4：RTAB-Map 核心改动

需要先编译源码（`introlab/rtabmap` + `introlab/rtabmap_ros`，**Humble 对应
`humble-devel` 分支而非 `ros2` 分支**）。候选增量创新：

- **语义辅助回环检测**：在回环候选筛选阶段引入节点间"物体类别直方图"相似度
  作为预过滤，减少外观相似但语义不同的误回环（例如两条相似的走廊）
- **语义地标协方差建模**：根据等效独立观测数动态设置
  `landmark_linear_variance`，观测越充分权重越高

### 待验证

- ~~在 RTAB-Map 实际运行时验证地标接入~~ ✅ **已完成**（2026-09-29 实测）
  - `/rtabmap/landmark_detections`：publisher 1（`semantic_mapper`）
    / subscription 1（`rtabmap` @ namespace `/rtabmap`），**已连通**
  - 地标 `frame_id` = `camera_color_optical_frame`（相机光学系，符合官方约定）
  - TF 退化警告 0 次（修好了按图像时间戳精确查必然失败的问题）
  - 单次 95 s 运行：3 个确认物体、336 次关联匹配；
    TensorRT 在 GPU 上推理 27~44 ms/帧
- 地标进入 RTAB-Map 图优化后**对轨迹精度的定量影响**
  （论文实验：同一段手持轨迹，开/关 `rtabmap_landmarks_topics` 各跑一遍比 ATE）
- 用 `SetLabel` 给地图节点打语义标签，验证数据库持久化
- 长时间运行的内存与热稳定性测试
- 多物体场景下的关联准确率定量评估（论文实验章节）
