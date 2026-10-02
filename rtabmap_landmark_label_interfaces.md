# RTAB-Map 0.23.7 (ROS 2 Humble) Landmark / Label 接口源码调研报告

面向「把外部 YOLO 物体检测器接入 RTAB-Map」的接口细节核查。

## 0. 调研所用源码版本（重要前提）

| 仓库 | 引用 | commit | 说明 |
|---|---|---|---|
| introlab/rtabmap_ros | **branch `humble-devel`** | `c25a091` | **本机安装的 `ros-humble-rtabmap-* 0.23.7-1jammy` 就是从这个分支发布的** |
| introlab/rtabmap_ros | branch `ros2` | `5207dab` | 你提到的分支，作为交叉参照 |
| introlab/rtabmap (核心库) | **tag `0.23.7`** | `bc724a2` | `/opt/ros/humble/include/rtabmap-0.23/.../Version.h:32` = `"0.23.7"` |

**⚠️ 关于分支的重要更正**：你说「0.23.x 对应 `ros2` 分支」——对 ROS 2 Humble 而言这不成立。
`ros/rosdistro` 的 `humble/distribution.yaml` 中：

```yaml
  rtabmap_ros:
    release:
      ...
    source:
      type: git
      url: https://github.com/introlab/rtabmap_ros.git
      version: humble-devel        # ← Humble 的发布分支
```

所以**下面所有 rtabmap_ros 行号都以 `humble-devel` 为准**，并给出 `ros2` 分支的对应行号。
两个分支的 landmark 逻辑**完全等价**，只是行号整体偏移约 10 行（`humble-devel` 少 26 行）。

**实证校验**（确认安装的二进制与源码一致）：

```
$ strings /opt/ros/humble/lib/librtabmap_slam_plugins.so | grep -Fx ...
landmark_detection         : landmark_detection
landmark_detections        : landmark_detections
landmark_linear_variance   : landmark_linear_variance
landmark_angular_variance  : landmark_angular_variance
set_label / list_labels / remove_label : 均存在
landmarks / labels         : 均存在
```

---

## 1. `landmark_topic` 订阅：参数名、默认值、QoS

### 结论：**不存在任何 landmark 话题参数名** —— 话题名是硬编码的相对名，只能用 ROS 2 remap 接入

`rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp`（humble-devel）**第 903–904 行**：

```cpp
903: landmarkDetectionSub_ = this->create_subscription<rtabmap_msgs::msg::LandmarkDetection>(
         "landmark_detection", 1,
         std::bind(&CoreWrapper::landmarkDetectionAsyncCallback, this, std::placeholders::_1),
         landmarkSubOptions);
904: landmarkDetectionsSub_ = this->create_subscription<rtabmap_msgs::msg::LandmarkDetections>(
         "landmark_detections", 1,
         std::bind(&CoreWrapper::landmarkDetectionsAsyncCallback, this, std::placeholders::_1),
         landmarkSubOptions);
```

> `ros2` 分支对应行号：**913 / 914**（内容逐字相同）。

旁证 —— `CoreWrapper.cpp` 中**全部** `declare_parameter` 调用里，唯一与 landmark 相关的参数只有两个方差参数：

```
206: landmarkDefaultAngVariance_ = this->declare_parameter("landmark_angular_variance", landmarkDefaultAngVariance_);
207: landmarkDefaultLinVariance_ = this->declare_parameter("landmark_linear_variance", landmarkDefaultLinVariance_);
```

launch 文件里也没有 landmark 话题参数（`rtabmap_launch/launch/rtabmap.launch.py:531` 只有 AprilTag 的 `tag_topic`，默认 `/detections`，且是 remap 到 `tag_detections`，不涉及 `landmark_detections`）。

**所以接入 YOLO 的正确做法是 remap**，例如：

```bash
ros2 run rtabmap_slam rtabmap --ros-args -r landmark_detections:=/yolo/landmark_detections
# 或 launch 中 {"landmark_detections": "/yolo/landmark_detections"}
```

### QoS：`rclcpp::QoS(1)` → KeepLast(1) + **RELIABLE** + **VOLATILE**（**不是** SensorDataQoS！）

`create_subscription(topic, 1, ...)` 走的是「只给 depth」的转换构造函数：

`/opt/ros/humble/include/rclcpp/rclcpp/qos.hpp`
```cpp
128: explicit
129: QoS(
130:   const QoSInitialization & qos_initialization,
131:   const rmw_qos_profile_t & initial_profile = rmw_qos_profile_default);
...
134:  * This is a convenience constructor that calls QoS(KeepLast(history_depth)).
...
141:  QoS(size_t history_depth);  // NOLINT(runtime/explicit): conversion constructor
```

`/opt/ros/humble/include/rmw/rmw/qos_profiles.h:51`
```c
51: static const rmw_qos_profile_t rmw_qos_profile_default =
52: {
53:   RMW_QOS_POLICY_HISTORY_KEEP_LAST,
54:   10,
55:   RMW_QOS_POLICY_RELIABILITY_RELIABLE,     // ← depth 被覆盖为 1，reliability 保持 RELIABLE
56:   RMW_QOS_POLICY_DURABILITY_VOLATILE,
...
```

**实际生效 QoS：`history=KEEP_LAST(1)`, `reliability=RELIABLE`, `durability=VOLATILE`。**

> ⚠️ **接入陷阱**：`sensor_msgs`/图像类节点常用 `SensorDataQoS()`（BEST_EFFORT）。如果你的 YOLO 发布者用 BEST_EFFORT，与 rtabmap 的 RELIABLE 订阅端 **QoS 不兼容、完全收不到消息**。YOLO 侧必须用默认 RELIABLE（或至少 reliability 兼容）。

### 附加发现：回调组「串了」（0.23.7 的小 bug）

```cpp
877: landmarkCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
...
889: landmarkSubOptions.callback_group = imuCallbackGroup_;   // ← 用的是 IMU 的组！
890: imuSubOptions.callback_group = imuCallbackGroup_;
```

`landmarkCallbackGroup_` 在 `CoreWrapper.h:457` 声明、在 877 行创建，**除此之外再无任何使用点**（全文件 grep 只有这两处）。
后果：landmark 回调与 IMU 回调共享同一个 `MutuallyExclusive` 组，二者互相串行；而预留的 landmark 组是死的。对功能无影响，但高频率 IMU 会拖慢 landmark 回调。

### 数据流：异步回调 → `landmarks_` 缓存 → 主处理线程消费

```cpp
2728: void CoreWrapper::landmarkDetectionAsyncCallback(const rtabmap_msgs::msg::LandmarkDetection::SharedPtr landmarkDetection)
2729: {
2730:     if(!paused_)
2731:     {
2732:         geometry_msgs::msg::PoseWithCovarianceStamped p;
2733:         p.header = landmarkDetection->header;
2734:         p.pose = landmarkDetection->pose;
2735:         UScopeMutex lock(landmarksMutex_);
2736:         uInsert(landmarks_,
2737:             std::make_pair(landmarkDetection->id,
2738:                 std::make_pair(p, landmarkDetection->size)));
2739:     }
2740: }
```

```cpp
2742: void CoreWrapper::landmarkDetectionsAsyncCallback(const rtabmap_msgs::msg::LandmarkDetections::SharedPtr landmarkDetections)
2743: {
2744:     if(!paused_)
2745:     {
2746:         UScopeMutex lock(landmarksMutex_);
2747:         for(unsigned int i=0; i<landmarkDetections->landmarks.size(); ++i)
2748:         {
2749:             geometry_msgs::msg::PoseWithCovarianceStamped p;
2750:             p.header = landmarkDetections->landmarks[i].header;
2751:             p.pose = landmarkDetections->landmarks[i].pose;
2752:             uInsert(landmarks_,
2753:                 std::make_pair(landmarkDetections->landmarks[i].id,
2754:                     std::make_pair(p, landmarkDetections->landmarks[i].size)));
2755:         }
2756:     }
2757: }
```

关键语义：
* 缓存是 `std::map<int, pair<PoseWithCovarianceStamped, float>>`（`CoreWrapper.h:479`：`// id, <pose, size>`）。
* **`uInsert` 是按 id 覆盖**：同一个 id 在两帧之间发多次，只有最后一次生效。
* `paused_ == true` 时检测被直接丢弃。
* **只在下一个 RGBD/传感器帧被处理时才被消费**：消费点在 `CoreWrapper::process()`（起始行 **2044**，`ros2` 分支：**2070**）的 **2250–2266 行**（`ros2` 分支：**2276–2292**），随后 `landmarks_.clear()`。即 landmark 是被「挂到」下一个同步帧上的，不是独立事件。

---

## 2. 坐标系约定：`header.frame_id` 怎么用？TF 失败会怎样？

### 结论：**当作位姿的参考系，必须通过 TF 变换到 `frame_id` 参数指定的 base frame。绝不是「假定已经是 base frame」。**

调用链：

`rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp:2250`（在 `process()` 内，`ros2` 分支：2278）
```cpp
2250: //tag detections
2251: landmarksMutex_.lock();
2252: Landmarks landmarks = rtabmap_conversions::landmarksFromROS(
2253:         landmarks_,
2254:         frameId_,              // ← 目标参考系 = "frame_id" 参数（默认 base_link）
2255:         odomFrameId,           // ← 用于运动补偿
2256:         stamp,                 // ← 当前帧时间戳（odom 时间戳）
2257:         *tfBuffer_,
2258:         waitForTransform_,
2259:         landmarkDefaultLinVariance_,
2260:         landmarkDefaultAngVariance_);
2261: landmarks_.clear();
2262: landmarksMutex_.unlock();
2263: if(!landmarks.empty())
2264: {
2265:     data.setLandmarks(landmarks);
2266: }
```

`rtabmap_ros/rtabmap_conversions/src/MsgConversion.cpp:1941`（`ros2` 分支：2008）
```cpp
1960: rtabmap::Transform baseToCamera = rtabmap_conversions::getTransform(
1961:         frameId,                            // from = "frame_id" 参数（默认 base_link）
1962:         iter->second.first.header.frame_id, // to   = 消息里的 header.frame_id
1963:         iter->second.first.header.stamp,    // 用「检测消息自己的时间戳」
1964:         tfBuffer,
1965:         waitForTransform);
1967: if(baseToCamera.isNull())
1968: {
1969:     UERROR("Cannot transform tag pose from \"%s\" frame to \"%s\" frame!",
1970:             iter->second.first.header.frame_id.c_str(), frameId.c_str());
1971:     continue;                               // ← 该 landmark 被丢弃
1972: }
1974: rtabmap::Transform baseToTag = baseToCamera * transformFromPoseMsg(iter->second.first.pose.pose);
```

### 具体是哪种 TF 调用

`MsgConversion.cpp:2009`（`getTransform`）：
```cpp
2016:  // TF ready?
2017:  rtabmap::Transform transform;
2018:  try
2019:  {
2020:      geometry_msgs::msg::TransformStamped tmp;
2021:      tmp = tfBuffer.lookupTransform(fromFrameId, toFrameId, tf2_ros::fromMsg(stamp), tf2::durationFromSec(waitForTransform));
2022:      transform = rtabmap_conversions::transformFromGeometryMsg(tmp.transform);
2023:  }
2024:  catch(tf2::TransformException & ex)
2025:  {
2026:      UWARN("(getting transform \"%s\" -> \"%s\") %s (wait_for_transform=%f)", fromFrameId.c_str(), toFrameId.c_str(), ex.what(), waitForTransform);
2027:  }
2029:  return transform;
```

即 **`tf2_ros::Buffer::lookupTransform(target=frame_id参数, source=header.frame_id, time=header.stamp, timeout=wait_for_transform)`**，超时参数默认 0.2 s：

```cpp
CoreWrapper.cpp:113: frameId_("base_link"),
CoreWrapper.cpp:123: waitForTransform_(0.2),// 200 ms
CoreWrapper.cpp:210: waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
```
（`ros2` 分支同为 113 / 123 / 210）

### 还有一层「里程计运动补偿」（容易被忽略）

```cpp
1978: // Correction of the global pose accounting the odometry movement since we received it
1979: rtabmap::Transform correction = rtabmap_conversions::getMovingTransform(
1980:         frameId,                 // movingFrame  = base frame
1981:         odomFrameId,             // fixedFrame   = odom frame
1982:         odomStamp,               // 当前帧 odom 时间戳
1983:         iter->second.first.header.stamp,  // 检测时间戳
1984:         tfBuffer,
1985:         waitForTransform);
1986: if(!correction.isNull())
1987: {
1988:     baseToTag = correction * baseToTag;
1989: }
```

`getMovingTransform`（`MsgConversion.cpp:2034`）：
```cpp
2047: tmp = tfBuffer.lookupTransform(movingFrame, tf2_ros::fromMsg(stampFrom), movingFrame, tf2_ros::fromMsg(stampTo), fixedFrame, tf2::durationFromSec(waitForTransform));
```

即查询 **base frame 在 odom_frame 下从「检测时刻」到「当前帧时刻」的位移**，把老检测位姿推到当前时刻。
`odomFrameId` 来自 `syncData_.odomFrameId`（`CoreWrapper.cpp:2035`），它由 odom 消息的 `header.frame_id`（`CoreWrapper.cpp:1345`）或 `odom_frame_id` 参数（`CoreWrapper.cpp:1368`）决定。

### TF 失败的行为汇总

| 失败的 TF | 代码位置 | 后果 |
|---|---|---|
| `base_link ← header.frame_id` 查不到 | `MsgConversion.cpp:2024` → `UWARN`；`:1967` → `UERROR` + `continue` | **该 landmark 被静默丢弃**（只打日志，帧照常处理，不报异常） |
| `base→odom` 运动补偿查不到 | `MsgConversion.cpp:2050` → `UWARN`；`:1992` → `UWARN` | 位姿**保留不补偿**，仍然使用（"If odometry is small ... should not be a problem"） |

对 YOLO 接入的现实含义：
1. 你的检测消息 **必须** 带一个在 TF 树里可查到的 `header.frame_id`（例如 `camera_color_optical_frame`），且该 frame 到 `base_link` 的 TF 必须已发布。
2. `header.stamp` 必须是**那个 frame 的 TF 有历史记录的时刻**——用检测图像的时间戳。如果用 `now()` 而 TF 有延迟，会查不到。
3. TF 树静止（相机刚性固定在 base_link 上）也必须真的发布 TF（如 `static_transform_publisher`），否则每个 landmark 都会被丢弃且只有 `UERROR` 日志。
4. 若 `odom_frame_id` 为空且没有 odom 输入，运动补偿会持续 `UWARN`（但不致命）。

---

## 3. `LandmarkDetection.id` 的要求与内部用法

### 3.1 必须 `> 0`，否则直接丢弃

`rtabmap_conversions/src/MsgConversion.cpp:1953`
```cpp
1953: for(... iter=tags.begin(); iter!=tags.end(); ++iter)
1954: {
1955:     if(iter->first <=0)
1956:     {
1957:         UERROR("Invalid landmark received! IDs should be > 0 (it is %d). Ignoring this landmark.", iter->first);
1958:         continue;
1959:     }
```

核心库同样断言：`rtabmap/corelib/include/rtabmap/core/Landmark.h:52`
```cpp
52: UASSERT(id_>0);
```

### 3.2 id 被当作 **负数节点 id** 的 landmark 节点身份

`rtabmap/corelib/src/Memory.cpp:6737`（`Memory::createSignature`）
```cpp
6737: //landmarks
6738: for(Landmarks::const_iterator iter = landmarks.begin(); iter!=landmarks.end(); ++iter)
6739: {
6740:     if(iter->second.id() > 0)
6741:     {
6742:         int landmarkId = -iter->first;          // ← 取负！
...
6778:         Link landmark(s->id(), landmarkId, Link::kLandmark, landmarkPose, iter->second.covariance().inv(), landmarkSize);
6779:         s->addLandmark(landmark);
6780:
6781:         // Update landmark index
6782:         if(!isIntermediateNode)
6783:         {
6784:             std::map<int, std::set<int> >::iterator nter = _landmarksIndex.find(landmarkId);
6785:             if(nter!=_landmarksIndex.end())
6786:             {
6787:                 nter->second.insert(s->id());
6788:             }
...
6793:                 _landmarksIndex.insert(std::make_pair(landmarkId, tmp));
6795:         }
6796:     }
6797:     else
6798:     {
6799:         UERROR("Invalid landmark received! IDs should be > 0 (it is %d). Ignoring this landmark.", iter->second.id());
6800:     }
6801: }
```

* `Link` 的 `to` = **`-id`**，`from` = 观测节点 id（正）。
* `Link.h:50` 的注释把语义写死了：
```cpp
50: kLandmark,  // Transform /base_link -> /landmark, "From" is node observing the landmark "To" (landmark is negative id)
```
* `Memory.h:375`：
```cpp
375: std::map<int, std::set<int> > _landmarksIndex; // < -landmarkId, nodeIds >
```

### 3.3 所以：**id 必须是全局稳定且唯一的身份**

id 就是图优化中 landmark 顶点的身份。同一个 id 出现在两个不同节点里 ⇒ **同一个物理 landmark**。
RTAB-Map **没有任何基于外观/几何的 landmark 重识别或合并逻辑**（grep 全核心库无 landmark 匹配/合并函数）；身份 100% 由 id 决定。

### 3.4 同一个物体在两个时刻用了不同 id 会发生什么

| 方面 | 结果 |
|---|---|
| 图结构 | 生成**两个互相独立的 landmark 顶点**，各自只与观测它的节点相连 |
| 位姿 | 各自独立优化，彼此无约束、无关联 |
| 地图内容 | 地图里出现**重复的 landmark**（同一个物理物体两个点） |
| 重定位/回环 | 只有当**同一个 id 被 ≥2 个节点观测到**才会触发 landmark 回环（下文 Q8 的第 3196 行条件）。新 id 在第一次观测时**不会**触发任何回环/重定位 |
| 数据库 | 两条独立的 `Link` 记录（`to_id = -id1`、`to_id = -id2`），永久重复 |
| 导航 | `SetGoal` 按 id 指定 landmark（`CoreWrapper.cpp:3028`: `"Planning: set goal to landmark %d"`），两个 id 会变成两个不同的目标点 |

**额外注意**：id 会**写入数据库**（`Link` 表 `to_id = -id`，见 `DBDriverSqlite3.cpp:2834`）。跨 session 重载同一个 `.db` 时，id 必须保持一致，否则老 landmark 的观测记录会与新 id 脱钩。

**建议**：用 `id = 类别ID * 10000 + 实例序号`，或维护一个跨 session 稳定的实例跟踪 ID（例如基于地图坐标 + 类别哈希）。**绝对不要**用「帧内序号」。

---

## 4. `landmark_frame_id` 字段的用途

### 结论：**0.23.7 中完全未被使用 —— 「未找到」任何运行期读取点**

在 `rtabmap_ros`（`humble-devel` + `ros2` 两个分支）与 `rtabmap` 核心库中，全仓库 `grep -rn "landmark_frame_id"` 的结果只有 2 处：

```
rtabmap_ros_humble/rtabmap_msgs/msg/LandmarkDetection.msg:7:string landmark_frame_id   ← 定义
rtabmap_ros/rtabmap_slam/test/msg_builders.hpp:316: msg.landmark_frame_id = "tag_" + std::to_string(id);  ← 仅测试构造器
```

即：**没有**任何生产代码读取它。

具体否定的三件事：

1. **不用于发布 TF** —— `CoreWrapper.cpp` 中发布 TF 的地方（`sendTransform`）只用在 `map→odom`（`719`、`723`），与 landmark 无关。
2. **不进入 `MapData`** —— 回调里只拷贝了 `header` 和 `pose`（`CoreWrapper.cpp:2750–2751`），`landmark_frame_id` 被丢弃；核心 `Landmark` 类里也**没有** frame 字段：
```cpp
// rtabmap/corelib/include/rtabmap/core/Landmark.h:72
72: private:
73:     int id_;
74:     float size_;
75:     Transform pose_;
76:     cv::Mat covariance_;
```
3. **不参与 TF 变换** —— 变换用的是 `header.frame_id`（见第 2 节）。

**实际有意义的 frame 字段是 `header.frame_id`**。`LandmarkDetection.msg` 自带的注释也这么说：
```
2: # header.stamp: the timestamp of the detection (e.g. image timestamp)
3: # header.frame_id: the base frame of pose (e.g., camera optical frame)
6: # Landmark's frame id
7: string landmark_frame_id
```

→ **给 `landmark_frame_id` 赋值不会有任何效果**，可以留空。

> 对照：真正会用到「frame 名字符串」的是 **AprilTag 输入路径**：
> `CoreWrapper.cpp:2779`：`std::string tagFrameId = msg->detections[i].family+":"+uNumber2Str(msg->detections[i].id);`
> 那条路径是自己造 frame 名再去查 TF。`landmark_detection(s)` 路径**不这样做**。

---

## 5. 协方差：全 0 时 `landmark_linear_variance` / `landmark_angular_variance` 如何生效

### 默认值

`rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp:121`（`ros2` 分支同为 121/122）
```cpp
121: landmarkDefaultAngVariance_(0.001),
122: landmarkDefaultLinVariance_(0.001),
...
206: landmarkDefaultAngVariance_ = this->declare_parameter("landmark_angular_variance", landmarkDefaultAngVariance_);
207: landmarkDefaultLinVariance_ = this->declare_parameter("landmark_linear_variance", landmarkDefaultLinVariance_);
```

→ **`landmark_linear_variance` 默认 `0.001`，`landmark_angular_variance` 默认 `0.001`。**

### 生效逻辑（关键：只检查 `covariance(0,0)`）

`rtabmap_conversions/src/MsgConversion.cpp:1996`
```cpp
1996: cv::Mat covariance = cv::Mat(6,6, CV_64FC1, (void*)iter->second.first.pose.covariance.data()).clone();
1997: if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
1998: {
1999:     covariance = cv::Mat::eye(6,6,CV_64FC1);
2000:     covariance(cv::Range(0,3), cv::Range(0,3)) *= defaultLinVariance;    // ← landmark_linear_variance
2001:     covariance(cv::Range(3,6), cv::Range(3,6)) *= defaultAngVariance;    // ← landmark_angular_variance
2002: }
2003: landmarks.insert(std::make_pair(iter->first, rtabmap::Landmark(iter->first, iter->second.second, baseToTag, covariance)));
```

**全 0 协方差的行为**：`covariance.at<double>(0,0) == 0.0` → `0.0 <= 0.0` 为真 → **整个协方差被替换为 `diag(lin,lin,lin,ang,ang,ang)`**。所以：

```yaml
landmark_detections:
  ...
# 例：covariance 全 0 时
landmark_linear_variance: 0.001
landmark_angular_variance: 0.001
```

正是 `LandmarkDetection.msg` 注释推荐的用法：
```
16: # If covariance is unknown, keep it as null matrix.
17: # rtabmap_slam/rtabmap's landmark_angular_variance and
18: # landmark_linear_variance parameters can be used
19: # for convenience if covariance is null.
```

### ⚠️ 两个重要陷阱

**陷阱 1：只检查了 `(0,0)` 一个元素。**
如果你的协方差矩阵 `(0,0) > 0` 但其余对角（尤其角速度项 `(3,3)`…`(5,5)`）为 0，则**不会**使用默认值，整个矩阵原样传入，而 `Landmark` 构造函数有一串断言：

`rtabmap/corelib/include/rtabmap/core/Landmark.h:55`
```cpp
55: UASSERT_MSG(uIsFinite(covariance_.at<double>(0,0)) && covariance_.at<double>(0,0)>0, uFormat("Linear covariance should not be null! Value=%f.", ...).c_str());
56: UASSERT_MSG(... covariance_.at<double>(1,1)>0, ...);
57: UASSERT_MSG(... covariance_.at<double>(2,2)>0, ...);
58: UASSERT_MSG(... covariance_.at<double>(3,3)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", ...).c_str());
59: UASSERT_MSG(... covariance_.at<double>(4,4)>0, ...);
60: UASSERT_MSG(... covariance_.at<double>(5,5)>0, ...);
```
`UASSERT_MSG` 在 `ULogger.h:67` 定义为 `ULogger::write(ULogger::kFatal, ...)`。
→ **要么全 0（安全走默认方差），要么给满 6 个正的对角元素**；不要给「半 0」矩阵。

**陷阱 2：用官方 launch 文件时默认值是 0.0001 / 9999.0，不是 0.001 / 0.001。**

`rtabmap_launch/launch/rtabmap.launch.py`
```python
323: "landmark_linear_variance": LaunchConfiguration('tag_linear_variance'),
324: "landmark_angular_variance": LaunchConfiguration('tag_angular_variance'),
...
532: DeclareLaunchArgument('tag_linear_variance',  default_value='0.0001', description=''),
533: DeclareLaunchArgument('tag_angular_variance', default_value='9999.0', description='>=9999 means rotation is ignored in optimization, when rotation estimation of the tag is not reliable or not computed.'),
```

`9999` 是**魔法值：表示「忽略 landmark 的朝向，只优化位置」**（对应的优化器判定见 Q8）。

### 协方差最终去哪了

`Memory.cpp:6778`：
```cpp
Link landmark(s->id(), landmarkId, Link::kLandmark, landmarkPose, iter->second.covariance().inv(), landmarkSize);
```
信息矩阵 = **协方差的矩阵求逆**（不是逐元素倒数）。所以你在 ROS 侧给的方差**直接决定了 landmark 约束在 g2o/gtsam 里的权重**：方差越小，landmark 位姿越"硬"地拉住观测节点。

对 YOLO 的建议：YOLO 没有可靠的 6DoF 位姿，通常应把 `landmark_angular_variance` 设为 `9999`（忽略朝向），线性方差按深度估计精度设置（如 0.05–1.0），并把 `pose.covariance` 留全 0。

---

## 6. 节点标签：`SetLabel` 服务语义、`node_id=0`、持久化、`Mem/MapLabelsAdded`

### 6.1 服务定义与完整话题名

`rtabmap_msgs/srv/SetLabel.srv`（全文）
```
1: #request
2: # Set node_id = 0 to set label to last node
3: int32 node_id
4: string node_label
5: ---
6: #response
```
（响应为空）

`rtabmap_msgs/srv/ListLabels.srv`
```
1: #request
2: ---
3: #response
4: int32[] ids
5: string[] labels
```

`rtabmap_msgs/srv/RemoveLabel.srv`
```
1: #request
2: string label
3: ---
4: #response
```

**服务名构造**：`CoreWrapper.cpp:666`
```cpp
666: const std::string servicePrefix = get_name() + std::string("/");
...
689: setLabelSrv_    = this->create_service<rtabmap_msgs::srv::SetLabel>(servicePrefix + "set_label", std::bind(&CoreWrapper::setLabelCallback, ...), rclcpp::ServicesQoS(), processingCallbackGroup_);
690: listLabelsSrv_  = this->create_service<rtabmap_msgs::srv::ListLabels>(servicePrefix + "list_labels", ...);
691: removeLabelSrv_ = this->create_service<rtabmap_msgs::srv::RemoveLabel>(servicePrefix + "remove_label", ...);
```

节点名固定为 `"rtabmap"`（`CoreWrapper.cpp:105`：`Node("rtabmap", options)`），所以 `get_name()` = `rtabmap`，服务是**相对名**，会带上节点的 namespace：

```
/<namespace>/rtabmap/set_label
/<namespace>/rtabmap/list_labels
/<namespace>/rtabmap/remove_label
```
（默认 namespace 下即 `/rtabmap/set_label`。QoS = `rclcpp::ServicesQoS()`；回调组 = `processingCallbackGroup_`，即与主处理线程串行。）

> 注意：`get_name()` **不含** namespace 前缀，所以不要写成 `/<ns>/set_label`。

### 6.2 回调实现

`CoreWrapper.cpp:4517`（`ros2` 分支：4543）
```cpp
4517: void CoreWrapper::setLabelCallback(
4518:         const std::shared_ptr<rmw_request_id_t>,
4519:         const std::shared_ptr<rtabmap_msgs::srv::SetLabel::Request> req,
4520:         std::shared_ptr<rtabmap_msgs::srv::SetLabel::Response>)
4521: {
4522:     if(rtabmap_.labelLocation(req->node_id, req->node_label))
4523:     {
4524:         if(req->node_id > 0) { RCLCPP_INFO(..., "Set label \"%s\" to node %d", ...); }
4528:         else                 { RCLCPP_INFO(..., "Set label \"%s\" to last node", ...); }
4533:     else
4534:     {
4535:         if(req->node_id > 0) { RCLCPP_ERROR(..., "Could not set label \"%s\" to node %d", ...); }
4539:         else                 { RCLCPP_ERROR(..., "Could not set label \"%s\" to last node", ...); }
4543:     }
4544: }
```
**注意语义：响应是空的 —— 成功/失败只体现在日志里，调用方从服务返回值无法判断成败。** 需要用 `/rtabmap/list_labels` 复核。

### 6.3 `node_id = 0` 的含义：给「最后一个节点」

`rtabmap/corelib/src/Rtabmap.cpp:966`
```cpp
966: bool Rtabmap::labelLocation(int id, const std::string & label)
967: {
968:     if(_memory)
969:     {
970:         if(id > 0)
971:         {
972:             return _memory->labelSignature(id, label);
973:         }
974:         else if(_memory->isIncremental() && _memory->getLastWorkingSignature(true))
975:         {
976:             return _memory->labelSignature(_memory->getLastWorkingSignature(true)->id(), label);   // ← 建图模式：最后一个工作节点
977:         }
978:         else if(!_memory->isIncremental() && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
979:         {
980:             std::map<int, Transform> nearestNodes = getNodesInRadius(_lastLocalizationPose, _localRadius, 1);
981:             if(!nearestNodes.empty())
982:             {
983:                 return _memory->labelSignature(nearestNodes.begin()->first, label);               // ← 定位模式：离当前位姿最近的节点
984:             }
985:             else
986:             {
987:                 UERROR("No nodes found inside %s=%fm of the current pose (%s). Cannot set label \"%s\"", ...);
988:             }
989:         }
990:         else
991:         {
992:             UERROR("Last signature is null! Cannot set label \"%s\"", label.c_str());
993:         }
994:     }
995:     return false;
996: }
```

**`node_id = 0` 的两种语义**：
* 建图模式（incremental）：标记**最后一个工作内存节点**（刚产生的那个节点）→ 适合「我现在站在/看到了 X」这类打点。
* 定位模式（localization）：标记**距离最近一次定位位姿 `RGBD/LocalRadius` 内最近的节点** → 适合「把当前位置命名成某个地名」。

对 YOLO 的启示：若你想「在这台机器人当前所在位置贴上 YOLO 看到的物体名」，`node_id=0` 就够用；但**注意标签是"地点"（节点级）而非"物体"（landmark 级）**，粒度完全不同。

### 6.4 标签唯一性约束

`rtabmap/corelib/src/Memory.cpp:2958`（`Memory::labelSignature`）
```cpp
2958: bool Memory::labelSignature(int id, const std::string & label)
2959: {
2960:     // verify that this label is not used
2961:     int idFound=getSignatureIdByLabel(label);
2962:     if(idFound == 0 && label.empty() && _labels.find(id)==_labels.end())
2963:     {
2964:         UWARN("Trying to remove label from node %d but it has already no label", id);
2965:         return false;
2966:     }
2967:     if(idFound == 0 || idFound == id)
2968:     {
...
2972:         if(label.empty())
2973:         {
2974:             UWARN("Label \"%s\" removed from node %d", _labels.at(id).c_str(), id);
2975:             _labels.erase(id);                          // ← 空字符串 = 删除标签
2976:         }
2977:         else
2978:         {
2987:             uInsert(_labels, std::make_pair(s->id(), label));
2988:         }
2989:         s->setLabel(label);
2990:         _linksChanged = s->isSaved(); // HACK to get label updated in Localization mode
2991:         return true;
2992:     }
...
3029:     else if(idFound)
3030:     {
3031:         UWARN("Another node %d has already label \"%s\", cannot set it to node %d", idFound, label.c_str(), id);
3032:     }
3033:     return false;
```

要点：
* **标签在整张图上唯一**：同一个 label 不能给两个节点（3029–3031，返回 `false`）。
* **`node_label = ""` 就是删除标签**（2972–2976）。
* 节点不存在 → `UERROR("Node %d not found, failed to set label \"%s\"!")`（3026）。

`RemoveLabel` 走的就是这条路：`CoreWrapper.cpp:4567`
```cpp
4567: int id = rtabmap_.getMemory()->getSignatureIdByLabel(req->label, true);
4568: if(id == 0) { RCLCPP_WARN(..., "Label \"%s\" not found in the map, cannot remove it!", ...); }
4572: else if(!rtabmap_.labelLocation(id, "")) { RCLCPP_ERROR(..., "Failed removing label \"%s\".", ...); }
```

### 6.5 持久化到数据库：**会**。存在 `Node` 表的 `label` 列（不是独立表）

**Schema**：`rtabmap/corelib/src/resources/DatabaseSchema.sql.in:16`（同样见 `.../backward_compatibility/DatabaseSchema_0_22_0.sql:16`）
```sql
16: CREATE TABLE Node (
17:     id INTEGER NOT NULL,
18:     map_id INTEGER NOT NULL,
19:     weight INTEGER,
20:     stamp FLOAT,
21:     pose BLOB,                -- 3x4 float
22:     ground_truth_pose BLOB,   -- 3x4 float
23:     velocity BLOB,            -- 6 float (vx,vy,vz,vroll,vpitch,vyaw) m/s and rad/s
24:     label TEXT,               -- ← 标签就存这里
25:     gps BLOB,
26:     env_sensors BLOB,
27:     time_enter DATE,
28:     PRIMARY KEY (id)
29: );
```

**写入**：`rtabmap/corelib/src/DBDriverSqlite3.cpp:5792`
```cpp
5792: std::string DBDriverSqlite3::queryStepNode() const
...
5796:     return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity, gps, env_sensors) VALUES(?,?,?,?,?,?,?,?,?,?);";
```
绑定（`DBDriverSqlite3.cpp:5848`）：
```cpp
5848: if(s->getLabel().empty())
5849: {
5850:     rc = sqlite3_bind_null(ppStmt, index++, ...);      // 空标签存 NULL
5851: }
5853: else
5854: {
5855:     rc = sqlite3_bind_text(ppStmt, index++, s->getLabel().c_str(), -1, SQLITE_STATIC);
5857: }
```

**读取**：`DBDriverSqlite3.cpp:2904`
```cpp
2913: query << "SELECT id FROM Node WHERE label='" << label <<"'";
```
`DBDriverSqlite3.cpp:2934`
```cpp
2943: query << "SELECT id,label FROM Node WHERE label IS NOT NULL";
```
启动时载入内存（`rtabmap/corelib/src/Memory.cpp:321`）：
```cpp
321: // Get labels
322: UDEBUG("Get labels");
323: _dbDriver->getAllLabels(_labels);
```

**⚠️ 写入时机（重要细节）**：标签并非总是立刻落盘。
* 若目标节点**已在数据库但不在工作内存**：`Memory.cpp:3019`
```cpp
3019: signatures.front()->setLabel(label);
3020: _dbDriver->asyncSave(signatures.front()); // move it again to trash     ← 立即异步保存
```
* 若目标节点**仍在工作内存**（建图模式下 `node_id=0` 通常如此，因为最后一个节点还在 WM）：只改内存 + `Signature::setLabel`（`Memory.cpp:2989`），并设 `_linksChanged = s->isSaved()`（`Memory.cpp:2990` 注释：`// HACK to get label updated in Localization mode`），**要等该 signature 后续被保存/更新时才写库**。
* `Signature::setLabel` 会置 `_modified`（`Signature.h:76`）：
```cpp
76: void setLabel(const std::string & label) {_modified=_label.compare(label)!=0;_label = label;}
```

→ 实践含义：给刚产生的节点打标签后**立刻 `rm` 掉数据库或 kill 进程，标签可能还没落盘**；想确保持久化，可用 `/rtabmap/backup` 服务（`CoreWrapper.cpp:673`）或等节点被换出 STM。

标签同时会随图数据下发：`Node.msg:7` 有 `string label` 字段，而 `MapData.msg:12` 是 `Node[] nodes`；另有可视化话题 `labels`（`MarkerArray`，`CoreWrapper.cpp:295` 创建，`4274`/`4886` 发布）。

### 6.6 `Mem/MapLabelsAdded` 的真实含义 —— **与 SetLabel 无关**

`rtabmap/corelib/include/rtabmap/core/Parameters.h:208`
```cpp
208: RTABMAP_PARAM(Mem, MapLabelsAdded, bool, true, "Create map labels. The first node of a map will be labeled as \"map#\" where # is the map ID.");
```

* **默认值：`true`**（参数全名 `"Mem/MapLabelsAdded"`，由 `RTABMAP_PARAM` 宏 `#PREFIX "/" #NAME` 拼出，`Parameters.h:66`）
* **唯一用途**：自动给**每个新地图（map session）的第一个节点**打上 `"map0"`、`"map1"` 之类的标签：

`rtabmap/corelib/src/Memory.cpp:1231`（`addSignatureToStm` 内，两个分支）
```cpp
1231: if(_mapLabelsAdded && isIncremental())
1232: {
1233:     //Tag the first node of the map
1234:     std::string tag = uFormat("map%d", signature->mapId());
1235:     if(getSignatureIdByLabel(tag, false) == 0)
1236:     {
1237:         UINFO("Tagging node %d with label \"%s\"", signature->id(), tag.c_str());
1238:         signature->setLabel(tag);
1239:         _labels.insert(std::make_pair(signature->id(), tag));
1240:     }
1241: }
...
1244: else if(_mapLabelsAdded && isIncremental())
1245: {
1246:     //Tag the first node of the map
1247:     std::string tag = uFormat("map%d", signature->mapId());
...
1252:         _labels.insert(std::make_pair(signature->id(), tag));
1254: }
```
解析点：`Memory.cpp:724`（`Parameters::parse(params, Parameters::kMemMapLabelsAdded(), _mapLabelsAdded);`）。

**对你的影响**：默认 `true` 时，`/rtabmap/list_labels` 里会出现 `map0`、`map1`… 这些自动标签。如果你只想看到 YOLO 相关的自定义标签，**设 `Mem/MapLabelsAdded: false`**（它在 ROS 侧可直接作为节点参数传入，因为所有核心参数都被自动 declare：`CoreWrapper.cpp:363`）：
```cpp
363: const std::map<std::string, rclcpp::ParameterValue> & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
364: for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end(); ++iter)
365: {
366:     std::string vStr = this->declare_parameter(iter->first, iter->second);
```

---

## 7. 0.23.7 里有没有内置的「物体 / object」概念？

### 结论：**没有。`rtabmap_msgs` 里不存在 `Objects.msg` / `Object.msg`，也没有 `add_object` 服务。**

`rtabmap_msgs` 0.23.7 的**全部**接口清单（`rtabmap_msgs/package.xml:5` = `<version>0.23.7</version>`）：

```
msg (24):  CameraModel, CameraModels, EnvSensor, GlobalDescriptor, Goal, GPS, Info,
           KeyPoint, LandmarkDetection, LandmarkDetections, Link, MapData, MapGraph,
           Node, OdomInfo, Path, Point2f, Point3f, RGBDImage, RGBDImages,
           ScanDescriptor, SensorData, UserData
srv (16):  AddLink, CleanupLocalGrids, DetectMoreLoopClosures, GetMap, GetMap2,
           GetNodeData, GetNodesInRadius, GetPlan, GlobalBundleAdjustment,
           ListLabels, LoadDatabase, PublishMap, RemoveLabel, ResetPose, SetGoal, SetLabel
```

全仓库搜索 `Objects` / `object` / `add_object` 在 `.msg`/`.srv` 中 **零命中**。

### 这说明什么

1. **RTAB-Map 0.23.7 的数据模型是「landmark 中心」的，没有第一类（first-class）的「语义物体」抽象。**
   landmark 就是「一个带位姿、可被多个节点观测的点/物体」，唯一的语义信息是**一个 int id**。

2. **YOLO 的类别名 / 置信度 / bbox 无处安放**。核心 `Landmark` 类只有 4 个成员（`Landmark.h:72`）：
```cpp
72: private:
73:     int id_;
74:     float size_;
75:     Transform pose_;
76:     cv::Mat covariance_;
```
   没有 `category` / `name` / `class_id` / bbox 字段，也没有 `setCategory()` 之类的方法。
   （对比：RTAB-Map 的 GUI/Android 版有自己的一套 object 检测逻辑，但**没有暴露为 `rtabmap_msgs` 接口**。）

3. **landmark 在图数据里是「隐式」的**：`MapData.msg` 里**没有** landmark 数组（只有 `MapGraph graph` + `Node[] nodes`），landmark 只以 `MapGraph.links` 中 `to_id < 0`、`type == 8 (kLandmark)` 的 `Link` 形式出现（`Link.msg:10-14`）。唯一直接输出 landmark 位姿的话题是 `landmarks`（`geometry_msgs/PoseArray`，`CoreWrapper.cpp:294` 创建，`4108-4151` 发布，`frame_id = map_frame_id_`）——它只发位姿，**连 id 都不在 PoseArray 里**（只在同时订阅 `labels` MarkerArray 时以文本框形式画出来）。

### 可用的替代路径（按推荐度）

| 方案 | 接口 | 能存什么 | 评价 |
|---|---|---|---|
| **a. 把类别编码进 id** | `LandmarkDetections` | `id = classId*10000 + instance`；`size` 可放尺寸 | 最省事；但类别名不可读，需自己维护映射表。**推荐** |
| **b. 用节点标签当地名** | `SetLabel` / `ListLabels` | 节点级的任意字符串（如 `"yolo_person_at_desk"`） | 粒度是「地点」不是「物体」；一个节点只能有一个标签（唯一性约束） |
| **c. 用 `UserData` 存任意 blob** | `rtabmap_msgs/UserData`（`header/rows/cols/type/data`） | 任意 `cv::Mat`（可序列化 YOLO 类别名+分数+bbox） | 最灵活，但**只能挂到节点**（每个同步帧一个），不能挂到 landmark；且 ROS 侧只有输入话题 `user_data_async`（`CoreWrapper.cpp:899`，需 `subscribe_user_data`，见 `rtabmap_sync/src/CommonDataSubscriber.cpp:381`）与 `GetNodeData`/`GetMap2` 的 `with_user_data` 读取，**没有 set_user_data 服务**。核心侧倒是存在 `Memory::setUserData`（`Memory.cpp:3036`） |

4. **另外**：`rtabmap_msgs/Goal.msg` 说明 landmark id/标签可用于导航目标：
```
4: # Set either node_id or node_label
5: int32 node_id
6: string node_label
8: # optional: if not set, the base frame of the robot is used
9: string frame_id
```
配合 `/rtabmap/set_goal`（`CoreWrapper.cpp:687`）与日志 `"Planning: set goal to landmark %d"`（`CoreWrapper.cpp:3028`）——即 **landmark 可以直接作为导航目标点**，这对「让机器人走向 YOLO 检测到的物体」很有用。

---

## 8. Landmark 参与后端图优化的条件；`Marker/Variance*` 默认值

### 8.1 landmark 进入图（g2o/gtsam）的完整路径

**第 1 步：观测 → signature 的 landmark 链接**（`Memory::createSignature`，`rtabmap/corelib/src/Memory.cpp:6737`）
```cpp
6737: //landmarks
6738: for(Landmarks::const_iterator iter = landmarks.begin(); iter!=landmarks.end(); ++iter)
6739: {
6740:     if(iter->second.id() > 0)                        // 条件 A：id > 0
6741:     {
6742:         int landmarkId = -iter->first;
...
6762:         Transform landmarkPose = iter->second.pose();
6763:         if(_registrationPipeline->force3DoF())       // 2D SLAM：强制 landmark 的 z 轴朝上
6764:         {
...
6778:         Link landmark(s->id(), landmarkId, Link::kLandmark, landmarkPose, iter->second.covariance().inv(), landmarkSize);
6779:         s->addLandmark(landmark);
6781:         // Update landmark index
6782:         if(!isIntermediateNode)                      // 条件 B：当前节点不是中间节点
6783:         {
6784:             std::map<int, std::set<int> >::iterator nter = _landmarksIndex.find(landmarkId);
6785:             if(nter!=_landmarksIndex.end()) { nter->second.insert(s->id()); }
...
```
调用入口：`Memory::update()`（`Memory.cpp:1019`）→ 第 1044 行 `Signature * signature = this->createSignature(data, pose, stats);`
→ 由 `Rtabmap::process()` 在 `Rtabmap.cpp:1472` 触发：`if(!_memory->update(data, odomPose, odomCovariance, odomVelocity, &statistics_))`

**第 2 步：landmark 位姿进入 `_optimizedPoses` 与 `_constraints`**（`rtabmap/corelib/src/Rtabmap.cpp:1736`）
```cpp
1735: _optimizedPoses.insert(std::make_pair(signature->id(), newPose));
1736: if(_memory->isIncremental() && signature->getWeight() >= 0)     // 条件 C：建图模式 + 非中间节点
1737: {
1738:     for(std::map<int, Link>::const_iterator iter = signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
1739:     {
1740:         if(_optimizedPoses.find(iter->first) == _optimizedPoses.end())
1741:         {
1742:             _optimizedPoses.insert(std::make_pair(iter->first, newPose*iter->second.transform()));   // landmark 成为图中的一个节点
1743:             UDEBUG("Added landmark %d : %s", iter->first, ...);
1744:             addedNewLandmark = true;
1745:         }
1746:         _constraints.insert(std::make_pair(iter->first, iter->second.inverse()));
1747:     }
1748: }
```

**第 3 步：从数据库/内存收集图**（`Rtabmap::optimizeGraph`，`Rtabmap.cpp:5457`）
```cpp
5457: _memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase, !_graphOptimizer->landmarksIgnored());
```
`Memory::getMetricConstraints`（`Memory.cpp:7160`）：
```cpp
7160: else if(jter->second.type() != Link::kLandmark)
7161: {
7162:     links.insert(std::make_pair(*iter, jter->second));
7163: }
7164: else if(landmarksAdded)
7165: {
7166:     if(!uContains(poses, jter->first))
7167:     {
7168:         poses.insert(std::make_pair(jter->first, poses.at(*iter) * jter->second.transform()));
7169:     }
7170:     links.insert(std::make_pair(jter->first, jter->second.inverse()));
7171: }
```

**第 4 步：优化器把 landmark 建成顶点**
`rtabmap/corelib/src/Optimizer.cpp:292`
```cpp
292: if(posesIn.find(toId) != posesIn.end() && (!landmarksIgnored() || toId>0))
```
`rtabmap/corelib/src/optimizer/OptimizerG2O.cpp:383`
```cpp
383: int landmarkVertexOffset = poses.rbegin()->first+1;
384: std::map<int, bool> isLandmarkWithRotation;
...
405: else if(!landmarksIgnored())
406: {
407:     // check if it is SE2 or only PointXY
408:     std::multimap<int, Link>::const_iterator jter=edgeConstraints.find(id);
409:     UASSERT(jter != edgeConstraints.end());
410:
411:     if (1 / static_cast<double>(jter->second.infMatrix().at<double>(5,5)) >= 9999.0)
412:     {
413:         g2o::VertexPointXY * v2 = new g2o::VertexPointXY();     // ← 只优化位置
414:         v2->setEstimate(Eigen::Vector2d(iter->second.x(), iter->second.y()));
415:         vertex = v2;
416:         isLandmarkWithRotation.insert(std::make_pair(id, false));
417:         id = landmarkVertexOffset - id;
418:     }
419:     else
420:     {
421:         g2o::VertexSE2 * v2 = new g2o::VertexSE2();             // ← 连朝向一起优化
422:         v2->setEstimate(g2o::SE2(iter->second.x(), iter->second.y(), iter->second.theta()));
...
429:         isLandmarkWithRotation.insert(std::make_pair(id, true));
430:         id = landmarkVertexOffset - id;
431:     }
432: }
```
边界构建（`OptimizerG2O.cpp:724`）：
```cpp
724: else if(id1<0 || id2 < 0)
725: {
726:     if(!landmarksIgnored())
727:     {
728:         //landmarks
729:         UASSERT((id1 < 0 && id2 > 0) || (id1 > 0 && id2 < 0));
...
739:             std::swap(id1, id2); // should be node -> landmark
...
742:         id2 = landmarkVertexOffset - id2;
```
GTSAM 侧对应 `OptimizerGTSAM.cpp:497-661`（用 `BearingRangeFactor` / `BearingFactor` 的 range-bearing 表示）。

### 8.2 条件小结

| # | 条件 | 位置 | 备注 |
|---|---|---|---|
| A | `LandmarkDetection.id > 0` | `MsgConversion.cpp:1955`；`Memory.cpp:6740` | 否则丢弃 + `UERROR` |
| B | 观测节点不是 intermediate node（`weight != -1`） | `Memory.cpp:6782`；`Rtabmap.cpp:3190`、`3267` | 中间节点不更新 `_landmarksIndex` |
| C | 建图模式（`Memory::isIncremental()`）且 `weight >= 0` | `Rtabmap.cpp:1736` | 定位模式下走 `getMetricConstraints(landmarksAdded=true)` |
| D | `Optimizer/LandmarksIgnored == false` | `Parameters.h:446`（默认 **false**）；`Optimizer.cpp:372` | 若设 true，landmark 完全退出优化 |
| E | landmark 位姿在传入优化器的 poses 集合中 | `Optimizer.cpp:292` | 由 A–D 保证 |
| F | 观测节点本身在图中（非 intermediate/无效） | `Optimizer.cpp:266` `// Fill up all nodes before landmarks` | — |

`RTABMAP_PARAM(Optimizer, LandmarksIgnored, bool, false, "Ignore landmark constraints while optimizing. Currently only g2o and gtsam optimization supports this.")`（`Parameters.h:446`）

### 8.3 `Marker/VarianceLinear` / `Marker/VarianceAngular` 默认值

`rtabmap/corelib/include/rtabmap/core/Parameters.h`
```cpp
921: RTABMAP_PARAM(Marker, Strategy,        int,   0,     "Marker detection implementation: 0=OpenCV, 1=AprilTag");
922: RTABMAP_PARAM(Marker, Dictionary,      int,   0,     "Dictionary to use: DICT_ARUCO_4X4_50=0, ...APRILTAG_36h11=20, ...");
923: RTABMAP_PARAM(Marker, Length,          float, 0,     "The length (m) of the markers' side...");
925: RTABMAP_PARAM(Marker, MaxDepthError,   float, 0.01,  ...);
926: RTABMAP_PARAM(Marker, VarianceLinear,  float, 0.001, "Linear variance to set on marker detections. ...");
927: RTABMAP_PARAM(Marker, VarianceAngular, float, 0.01,  "Angular variance to set on marker detections. ...");
928: RTABMAP_PARAM(Marker, VarianceOrientationIgnored, bool, false, "When this setting is false, the landmark's orientation is optimized during graph optimization. When this setting is true, only the position of the landmark is optimized. ...");
929: RTABMAP_PARAM(Marker, MaxRange,        float, 0.0,   "Maximum range in which markers will be detected. <=0 for unlimited range.");
930: RTABMAP_PARAM(Marker, MinRange,        float, 0.0,   "Mininimum range in which markers will be detected. <=0 for unlimited range.");
932: RTABMAP_PARAM(Marker, PriorsVarianceLinear,  float, 0.001, "Linear variance to set on marker priors.");
933: RTABMAP_PARAM(Marker, PriorsVarianceAngular, float, 0.001, "Angular variance to set on marker priors.");
```

| 参数 | 默认值 | 行号 |
|---|---|---|
| **`Marker/VarianceLinear`** | **`0.001`** | `Parameters.h:926` |
| **`Marker/VarianceAngular`** | **`0.01`** | `Parameters.h:927` |
| `Marker/VarianceOrientationIgnored` | `false` | `Parameters.h:928` |
| `Marker/PriorsVarianceLinear` | `0.001` | `Parameters.h:932` |
| `Marker/PriorsVarianceAngular` | `0.001` | `Parameters.h:933` |

### ⚠️⚠️ 对你的方案最关键的一条：`Marker/Variance*` **不作用于 ROS landmark 话题输入**

`Marker/VarianceLinear` / `VarianceAngular` 只在**内部 MarkerDetector 检测**路径使用：

`rtabmap/corelib/src/Memory.cpp:6041`
```cpp
6041: Landmarks landmarks = data.landmarks();
6042: if(_detectMarkers && !isIntermediateNode && !data.imageRaw().empty())
6043: {
6044:     UDEBUG("Detecting markers...");
6045:     if(landmarks.empty())                                  // ← 输入已有 landmarks 就跳过内部检测！
6046:     {
6056:         if(!models.empty() && models[0].isValidForProjection())
6057:         {
6058:             std::map<int, MarkerInfo> markers = _markerDetector->detect(data.imageRaw(), models, data.depthRaw(), _landmarksSize);
...
6067:                 cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
6068:                 if(_markerOrientationIgnored)
6069:                 {
6070:                     covariance(cv::Range(3,6), cv::Range(3,6)) *= 9999; // disable orientation estimation
...
6091:                 else
6092:                 {
6091:                     covariance(cv::Range(0,3), cv::Range(0,3)) *= _markerLinVariance;    // ← Marker/VarianceLinear
6092:                     covariance(cv::Range(3,6), cv::Range(3,6)) *= _markerAngVariance;    // ← Marker/VarianceAngular
6093:                 }
6094:                 landmarks.insert(std::make_pair(iter->first, Landmark(iter->first, iter->second.length(), iter->second.pose(), covariance)));
...
6103:     else
6104:     {
6105:         UWARN("Input data has already landmarks, cannot do marker detection.");
6106:     }
```
参数解析：`Memory.cpp:764`
```cpp
764: Parameters::parse(params, Parameters::kMarkerVarianceLinear(), _markerLinVariance);
765: Parameters::parse(params, Parameters::kMarkerVarianceAngular(), _markerAngVariance);
766: Parameters::parse(params, Parameters::kMarkerVarianceOrientationIgnored(), _markerOrientationIgnored);
```

**结论**：
* 一旦你通过 `landmark_detections` 话题给了 landmark，**内置 MarkerDetector 会被整段跳过**（6045 + 6105 的 `UWARN`）。此时 `Marker/VarianceLinear=0.001`、`Marker/VarianceAngular=0.01` **完全不起作用**。
* 你这套 YOLO 方案实际生效的是 **`landmark_linear_variance` / `landmark_angular_variance`（默认 0.001 / 0.001，或 launch 文件里的 0.0001 / 9999.0）** —— 见第 5 节。
* 两者默认值**并不相同**（`Marker/VarianceAngular` 是 `0.01`，而 `landmark_angular_variance` 是 `0.001`），不要混淆。

### 8.4 「只在少量节点中观测到某个 landmark，会被优化吗？」

**会被优化，但不会触发重定位/回环。** 分两层看：

**（1）图优化层面：会。没有最小观测次数门槛。**
只要 landmark 出现在 `_optimizedPoses` 里（`Rtabmap.cpp:1740-1742` 第一次观测时就插入了），优化器就会为它建立顶点（`OptimizerG2O.cpp:413/421`），并把它与观测节点的那条 `kLandmark` 边一起解算。grep 全部优化器源码，**不存在任何「landmark 观测次数 < N 就剔除」的过滤**（唯一相关注释是 `Optimizer.cpp:196` 的排序注释 `// landmark, sort by smallest to largest landmark id, after normal links`）。
即使只被 1 个节点观测，landmark 也是「1 个顶点 + 1 条相对约束」，在该节点位姿给定下其位置完全可解。

**（2）回环/重定位触发层面：不会，需要 `> 1` 个观测节点。**
`rtabmap/corelib/src/Rtabmap.cpp:3189`
```cpp
3189: std::map<int, std::set<int> > landmarksDetected; // <Landmark ID, list of nodes that saw this landmark>
3190: if(!signature->getLandmarks().empty() && !_graphOptimizer->landmarksIgnored() && signature->getWeight()!=-1)
3191: {
3192:     bool hasGlobalLoopClosuresInOdomCache = ...;
3193:     UDEBUG("hasGlobalLoopClosuresInOdomCache=%d", hasGlobalLoopClosuresInOdomCache?1:0);
3194:     for(std::map<int, Link>::const_iterator iter=signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
3195:     {
3196:         if(uContains(_memory->getLandmarksIndex(), iter->first) &&
3197:                 _memory->getLandmarksIndex().find(iter->first)->second.size()>1)      // ← 关键条件
3198:         {
3199:             if(!_memory->isIncremental() &&          // In localization mode
3200:                 !hasGlobalLoopClosuresInOdomCache && // If there are global loop closures in odom cache, we can keep far landmarks
3201:                 _localRadius>0.0 &&
3202:                 iter->second.transform().getNormSquared() > _localRadius*_localRadius)
3203:             {
3204:                 // Ignore landmark detections over local radius
3205:                 UWARN("Ignoring landmark %d for localization as it is too far (%fm > %s=%f) ...", ...);
3211:             }
3212:             else
3213:             {
3214:                 UINFO("Landmark %d observed again! Seen the first time by node %d.", -iter->first, *_memory->getLandmarksIndex().find(iter->first)->second.begin());
3215:                 landmarksDetected.insert(std::make_pair(iter->first, _memory->getLandmarksIndex().find(iter->first)->second));
3216:                 rejectedLoopClosure = false; // If it was true, it will be set back to false if landmarks are rejected on graph optimization
3217:                 loopClosureLinksAdded.push_back(std::make_pair(signature->id(), iter->first));
3218:             }
3219:         }
3220:     }
3221: }
```

**`size() > 1` 的精确含义**：`_landmarksIndex[landmarkId]` 在 `createSignature` 中**已经包含当前节点**（调用顺序：`Rtabmap.cpp:1472` `_memory->update()` → `Memory::update` → `Memory.cpp:1044` `createSignature` → `_landmarksIndex` 更新；而第 3189 行的判断在第 1472 行**之后**）。所以 `size() > 1` ≡ **「除了当前节点外，至少还有 1 个历史节点观测过这个 landmark」** ≡ **「这个 landmark 之前见过」**。

→ 也就是说：
* **第一次**看到某 id：`size()==1`，**不**产生回环/重定位事件，但 landmark 已进图并参与优化。
* **第二次**（被另一个节点）看到同一 id：`size()==2 > 1` → 触发 `landmarksDetected`（等价于一次 landmark 回环/重定位），日志 `"Landmark %d observed again! Seen the first time by node %d."`（3214）。
* 定位模式下还有额外限制：若 landmark 距离超过 `RGBD/LocalRadius`（`Parameters::kRGBDLocalRadius()`）且 odom cache 里没有全局回环，则会被忽略（3199-3211）。

**对 YOLO 的直接指导**：
1. 让同一个物件的 id 在**多个节点**中被稳定复现，才能换来 landmark 回环/重定位收益；只出现一次的自定义 landmark 对定位没有帮助（虽然它会被优化）。
2. 因此 YOLO 的检测必须是「可复现的身份」——这又回到第 3 节：**不能用帧内序号，必须做实例级 ID 跟踪**。
3. 若 YOLO 位姿朝向不可靠，把角方差设为 `9999`（launch 默认值就是这个），优化器会自动退化为 `VertexPointXY` / 只做 bearing 的位置优化（`OptimizerG2O.cpp:411`）。

---

## 9. 一页速查（给 YOLO 接入用）

| 问题 | 答案 | 依据 |
|---|---|---|
| 订阅话题参数名 | **不存在**；硬编码相对名 `landmark_detections`（单个用 `landmark_detection`） | `CoreWrapper.cpp:903-904` |
| 接入方式 | ROS 2 remap：`-r landmark_detections:=<你的话题>` | 同上 |
| 订阅 QoS | `KEEP_LAST(1)` + `RELIABLE` + `VOLATILE`（**非** BEST_EFFORT） | `qos.hpp:141,128`; `rmw/qos_profiles.h:51` |
| `header.frame_id` | 位姿的参考系，会 TF 变换到 `frame_id` 参数（默认 `base_link`） | `CoreWrapper.cpp:2254`; `MsgConversion.cpp:1960` |
| TF 调用 | `lookupTransform(frame_id参数, header.frame_id, header.stamp, timeout=0.2s)` | `MsgConversion.cpp:2021` |
| TF 失败 | `UERROR` + `continue` → **该 landmark 被丢弃**（不报异常） | `MsgConversion.cpp:1967-1972` |
| 额外 TF | 还有 base→odom 的运动补偿（失败仅 `UWARN`，不致命） | `MsgConversion.cpp:1979-1995` |
| `id` 要求 | **`> 0` 且全局稳定唯一**；内部即 landmark 节点身份（负数 id） | `MsgConversion.cpp:1955`; `Memory.cpp:6742,6778`; `Landmark.h:52` |
| 同一物体换 id | 生成两个无关联的 landmark 顶点；地图重复；不触发回环；DB 里两条独立 Link | `Memory.cpp:6784-6794`; `Rtabmap.cpp:3196` |
| `landmark_frame_id` | **完全未被使用（未找到任何读取点）**，可留空 | 全仓库 grep 仅 msg 定义 + 测试构造器 |
| 方差参数默认 | `landmark_linear_variance=0.001`、`landmark_angular_variance=0.001`（launch 里是 `0.0001`/`9999.0`） | `CoreWrapper.cpp:121-122`; `rtabmap.launch.py:532-533` |
| 方差生效条件 | `covariance(0,0) <= 0`（**全 0 即触发**）→ 用默认方差替换整个矩阵 | `MsgConversion.cpp:1997-2002` |
| 陷阱 | 只检查 `(0,0)`；「半 0」矩阵会触发 `Landmark` 断言 | `Landmark.h:55-60` |
| SetLabel 服务名 | `/<ns>/rtabmap/set_label`（另有 `list_labels`、`remove_label`） | `CoreWrapper.cpp:666,689-691,105` |
| `node_id=0` | 建图模式→最后一个工作节点；定位模式→最近节点（`RGBD/LocalRadius` 内） | `Rtabmap.cpp:974-984` |
| 标签响应 | **空响应**，成败只能看日志 | `SetLabel.srv:5-6`; `CoreWrapper.cpp:4517-4544` |
| 标签唯一性 | 图内唯一；`label=""` 表示删除 | `Memory.cpp:2967,2972-2976,3029-3031` |
| 标签持久化 | 是，`Node.label` 列；WM 内节点延迟到下次保存 | `DatabaseSchema.sql.in:24`; `DBDriverSqlite3.cpp:5796,5848-5857`; `Memory.cpp:2990,3020` |
| `Mem/MapLabelsAdded` | 默认 `true`，**只管自动给每个地图首节点打 `"map#"`**，与 SetLabel 无关 | `Parameters.h:208`; `Memory.cpp:1231-1254` |
| 有无 object 概念 | **无**：无 `Objects.msg`、无 `add_object`；`Landmark` 只有 id/size/pose/covariance | `rtabmap_msgs/msg,srv` 全清单；`Landmark.h:72-77` |
| 类别名怎么存 | 编码进 id / 用节点标签 / 用 `UserData`（仅节点级） | `LandmarkDetection.msg`; `SetLabel.srv`; `UserData.msg` |
| 参与优化的条件 | `id>0` + 非中间节点 + `Optimizer/LandmarksIgnored=false`（默认）+ landmark 在 poses 中 | `Memory.cpp:6740,6782`; `Rtabmap.cpp:1736-1748,5457`; `Parameters.h:446` |
| 单次观测是否被优化 | **会**（无最小观测数门槛），但**不触回环**（需 index `size()>1`） | `OptimizerG2O.cpp:405-431`; `Rtabmap.cpp:3196-3197` |
| `Marker/VarianceLinear` | 默认 **`0.001`** | `Parameters.h:926` |
| `Marker/VarianceAngular` | 默认 **`0.01`** | `Parameters.h:927` |
| `Marker/Variance*` 是否管你的 landmark | **不管** —— 只作用于内部 MarkerDetector；有输入 landmark 时内部检测被跳过 | `Memory.cpp:6042-6105` |

---

## 10. 无法确认 / 未找到的项目（明确声明）

1. **`landmark_topic` 参数**：**未找到**。0.23.7 中不存在任何 landmark 话题参数，只有硬编码话题名。
2. **`landmark_frame_id` 的任何使用点**：**未找到**（除 `.msg` 定义与 `rtabmap_slam/test/msg_builders.hpp:316` 的测试构造外，全仓库零引用）。
3. **`Objects.msg` / `Object.msg` / `add_object` 服务**：**未找到**（`rtabmap_msgs` 0.23.7 的 24 个 msg + 16 个 srv 中均不存在）。
4. **独立的 Landmark 数据库表**：**未找到**。landmark 只以 `Link` 表记录（`to_id = -id`, `type = 8`）持久化，没有单独的 landmark 位姿表。
5. **优化器中「landmark 最少观测次数」过滤**：**未找到**（不存在此类过滤；观测次数只在回环触发判定 `Rtabmap.cpp:3197` 里用到）。
6. **`landmark_detection`（单数）与 `landmark_detections`（复数）两个话题的 QoS 是否可配置**：**未找到**可配置项（写死 `1`，只能改 remap，不能改 QoS；若需不同 QoS 必须改源码或用 relay）。
