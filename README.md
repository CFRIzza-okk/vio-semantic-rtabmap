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

在同一局域网内的另一台电脑打开 Foxglove Studio 桌面版，连接脚本打印的地址，例如 `ws://192.168.1.8:8765`。Foxglove 的 3D 面板 Fixed Frame 设为 `map`，可添加：

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

回到本机后，不启动 RealSense 和实时视觉里程计，直接回放：

```bash
cd ~/VIO
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊
```

不启动本机 RViz 或降低回放速度：

```bash
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊 --no-rviz
./replay_mapping_bag.sh ~/VIO/data/bags/消防走廊 --rate=0.5
```

回放脚本使用 rosbag 中的图像、深度、相机内参和静态 TF，在本机重新运行 RGB-D 视觉里程计，再由 RTAB-Map 重建地图；每次回放默认生成新的数据库。录制和回放时不要同时运行另一套 RealSense、RTAB-Map 或 rosbag 进程。

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
