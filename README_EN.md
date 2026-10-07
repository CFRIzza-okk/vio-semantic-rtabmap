# Vio-Semantic-RTABMAP

[中文 README](README.md)

A real-time visual odometry, 3D mapping, and semantic annotation system based on RTAB-Map: the D435 synchronously captures color images and aligned depth images; RTAB-Map estimates camera motion from RGB-D data, builds the map, and reduces drift through loop closure; the semantic detection node uses YOLO to identify objects, then combines valid depth values within each bounding box to back-project 2D detections into 3D landmarks in the camera coordinate frame; the semantic mapping node uses multi-frame association and confidence filtering to produce stable objects and publish visualization markers.

## Operation Modes

This project provides two operation modes (PX4 mode and handheld mode). Select the appropriate mode for the use case, and **do not start both at the same time**:

| Scenario | Launch command | Odometry source | Visualization |
|---|---|---|---|
| PX4 | Run `~/VIO/launch_px4_mavros.sh`, then run `~/VIO/launch_d435_rtabmap.sh` | MAVROS `/mavros/local_position/odom` | RViz |
| Handheld D435 | `cd ~/VIO && ./launch_showcase.sh` | RTAB-Map RGB-D visual odometry | RViz |
| Handheld mapping + remote Foxglove | `cd ~/VIO && ./launch_showcase.sh --no-rviz`, then run `./start_foxglove.sh` in another terminal | RTAB-Map RGB-D visual odometry | Foxglove on another computer |

The handheld mode uses RGB-D visual odometry from the D435; PX4 mode uses flight-controller odometry. The odometry sources and TF publishing methods are different between the two modes, so they cannot be mixed. Before switching modes, press Ctrl-C in the original terminal to exit the previous process and avoid starting duplicate camera or RTAB-Map nodes.

## Installation

Open a terminal in the project directory and run:

```bash
cd ~/VIO
./install_native.sh
```

The script invokes `sudo apt-get`, installs the launch script to `/usr/local/bin/launch_d435_rtabmap`, installs the `rtabmap_d435.rviz` layout, and installs a udev permission rule for Intel RealSense USB devices. After installation, unplug and reconnect the camera so that the new rule takes effect.

If the semantic workspace has not been built:

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
```

Remote Foxglove visualization requires the bridge to be installed once:

```bash
sudo apt update
sudo apt install -y ros-humble-foxglove-bridge
```

## Handheld D435 Real-Time Mapping and Semantic Visualization

```bash
cd ~/VIO
./launch_showcase.sh
```

The script checks and starts the RealSense D435, the camera extrinsics, RTAB-Map RGB-D visual odometry and mapping, semantic detection, and semantic mapping in sequence, and reports completion only after the nodes and topics are ready. RViz is started by default, a new showcase database is created, and the conservative `uniform` landmark covariance mode is used. The default semantic classes are `chair,bottle,backpack,suitcase,laptop`, excluding people who may move and reducing the risk of dynamic objects being treated as static landmarks.

Common options:

```bash
./launch_showcase.sh --help
./launch_showcase.sh --no-rviz       # Do not start RViz on the Jetson
./launch_showcase.sh --fast          # Reduce visual odometry computation when CPU resources are limited
./launch_showcase.sh --keep-db       # Continue using the previous showcase database
./launch_showcase.sh --classes=chair,bottle  # Specify a custom COCO class whitelist
```

The default database is stored at `~/.ros/rtabmap_showcase.db`, the semantic map at `~/VIO/data/showcase_semantic_map.json`, and logs at `~/VIO/data/showcase_logs/`. By default, existing showcase databases and semantic maps are backed up before starting from a clean state, preventing data from different sessions from being mixed. Press Ctrl-C in the launch terminal when exiting; the nodes will save the database and semantic map.

### Remote Visualization with Foxglove

Run the following in a terminal:

```bash
cd ~/VIO
./launch_showcase.sh --no-rviz
```

Open another terminal and start the Foxglove Bridge:

```bash
cd ~/VIO
./start_foxglove.sh
```

On another computer on the same local network, open the Foxglove Studio desktop application and connect to the address printed by the script, for example `ws://192.168.1.8:8765`. Set the Fixed Frame of Foxglove's 3D panel to `map`. The following topics can be added:

- `/rtabmap/cloud_map`: global 3D point-cloud map (primary display);
- `/rtabmap/map`: 2D occupancy grid;
- `/rtabmap/odom_local_map`: local odometry point cloud;
- `/tf`, `/tf_static`: dynamic and static transforms.

To display semantic annotations, also add `/semantics/map_markers`, `/semantics/label_markers`, and `/semantics/markers`. To view only the original map, do not add or hide these semantic topics. Foxglove does not support RTAB-Map-specific RViz plugins such as MapCloud and MapGraph; the global point cloud should be displayed using the standard `/rtabmap/cloud_map` topic. Press Ctrl-C to stop the Bridge and the mapping script separately.

### Image Acquisition Recommendations

Move the camera slowly and smoothly, keeping texture and stable lighting in the image; avoid fast rotations, motion blur, and pointing at blank walls or windows for extended periods. Whenever possible, complete a loop and return near the starting point to help trigger loop closure. Depth from the D435 may be unreliable under strong light, on reflective or transparent objects, and at long distances. If odometry or map detail degrades after using `--fast`, return to the default settings.

### Record a rosbag and Build the Map Offline

If you are carrying the camera outside the local network, record the RGB-D data first and replay it locally to build the map later:

```bash
cd ~/VIO
./record_mapping_bag.sh --name=fire_corridor
```

The recording script starts handheld mode and saves color images, aligned depth images, camera calibration, `/odom`, and static TF to `~/VIO/data/bags/`. During replay, RTAB-Map recomputes the map coordinate frame, avoiding the old dynamic TF from the recording session. After completing the route, press Ctrl-C in the recording terminal and wait for the rosbag to be saved.

After returning, do not start RealSense or live visual odometry; replay the bag directly:

```bash
cd ~/VIO
./replay_mapping_bag.sh ~/VIO/data/bags/fire_corridor
```

To disable local RViz or reduce the replay speed:

```bash
./replay_mapping_bag.sh ~/VIO/data/bags/fire_corridor --no-rviz
./replay_mapping_bag.sh ~/VIO/data/bags/fire_corridor --rate=0.5
```

The replay script uses the images, depth, camera calibration, and static TF stored in the rosbag, reruns RGB-D visual odometry locally, and rebuilds the map with RTAB-Map. A new database is generated by default for each replay. Do not run another RealSense, RTAB-Map, or rosbag process at the same time during recording or replay.

## PX4 Mode: Start MAVROS

This project uses a PX4 flight controller, specifically a CUAV Pixhawk v6X V2 (USB ID `3163:004c`) connected as `/dev/ttyACM0`.
Start MAVROS:

```bash
~/VIO/launch_px4_mavros.sh
```

The script starts `px4_tf.launch.xml` and loads the default PX4 plugin list, `px4_config.yaml`, and `px4_tf_config.yaml`. MAVROS sets `local_position.tf.send` to `false` by default; the TF parameter override is therefore used here to enable the world-to-body dynamic transform.

Check the data in another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 topic hz /mavros/imu/data                # approximately 50 Hz, depending on flight-controller settings
ros2 topic hz /mavros/local_position/odom     # approximately 26 Hz, depending on flight-controller settings
```

## PX4 Mode: Start D435 + RTAB-Map

Connect the D435 to a USB 3.0 port, verify that it is detected with `lsusb`, and run:

```bash
~/VIO/launch_d435_rtabmap.sh
```

The script performs the following steps:

1. Publishes the static `base_link`→`camera_link` extrinsic transform, with a translation of `(0.045, -0.075, -0.005)` m and zero rotation;
2. Starts RealSense and outputs color, depth, and aligned depth images;
3. Starts `rtabmap_launch/rtabmap.launch.py` with `frame_id:=base_link`, `odom_topic:=/mavros/local_position/odom`, `visual_odometry:=false`, and `publish_tf_map:=false`, and loads `~/VIO/rtabmap_d435.rviz`.

## TF Structure and Troubleshooting an Invisible Map

Only one node may be responsible for publishing the world-to-body dynamic transform. The TF tree in PX4 mode is:

```text
map ──(MAVROS local_position.tf.send:=true)──> base_link
    ──(static)──> camera_link ──> camera_*_frame ──> camera_color_optical_frame
```

- MAVROS publishes the dynamic `map`→`base_link` transform;
- RTAB-Map uses the flight-controller-provided `/mavros/local_position/odom` and sets `publish_tf_map:=false` to avoid publishing a competing `map` TF;
- The launch script publishes `base_link`→`camera_link`, while the RealSense driver publishes the internal camera transforms.

Do not pass `odom_frame_id:=map` to `rtabmap.launch.py`. It is not the parameter used to select the odometry source; setting it incorrectly causes RTAB-Map to attempt to publish a `map`→`map` self-transform. tf2 rejects it with `TF_SELF_TRANSFORM`, and the related RViz displays report errors because the required TF is missing.

Check the TF tree in PX4 mode:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

## RViz Layout

`rtabmap_d435.rviz` is based on the RTAB-Map RGB-D layout and adds the global `/rtabmap/cloud_map` point-cloud display. In PX4 mode, the Fixed Frame is `map`, and the Orbit view uses `base_link` as its Target Frame: the PX4 local origin may be far from the aircraft, so if the view orbits around the `map` origin, the point cloud may be outside the visible area. The layout includes the occupancy-grid Map, the RTAB-Map plugins MapCloud/MapGraph/Info, Odom Local Map, Camera Cloud, RTAB-Map Cloud, TF, Grid, and semantic marker displays.

To view only the original map, disable `Semantic Map`, `Semantic Detections`, `Semantic Labels`, and `Semantic Colored Cloud` in the Displays panel; keep mapping displays such as `RTAB-Map Cloud`, `MapCloud`, and `Odom Local Map` enabled.

## Frequently Asked Questions

- **No point cloud or TF display errors:** Check `ros2 run tf2_ros tf2_echo map base_link`, confirm that the correct mode was started, and make sure that no conflicting TF publishers are running at the same time.
- **`TF_SELF_TRANSFORM` with both frame_id and child_frame_id set to `map`:** Check the RTAB-Map launch parameters and do not set `odom_frame_id:=map`; in PX4 mode, also confirm `publish_tf_map:=false`.
- **Duplicate nodes or two `/mavros` instances:** Stop the old instance first; multiple MAVROS instances will compete for the flight-controller serial port.
- **Tilted or offset map, or abnormal loop closures:** The PX4 local origin changes between sessions; do not directly mix old databases. Back up the database and start from a clean state:

  ```bash
  mv ~/.ros/rtabmap.db ~/.ros/rtabmap.db.bak.$(date +%s)
  ```

- **Point cloud is published but not visible in Foxglove/RViz:** Confirm that the Fixed Frame is `map`, that the view is near the map, and that the corresponding topic contains messages.
- **Semantic annotations do not appear:** Check the YOLO model path, semantic-node logs, and the `/semantics/objects` and `/semantics/map_markers` topics.

## Camera Extrinsics

The current configuration uses the following coordinate convention: x points right, y points forward, and z points up; the IMU is 7.5 cm to the left, 4.5 cm behind, and 0.5 cm above the camera optical center, with the two axes assumed to be parallel (zero rotation). Converted to the ROS body coordinate axes (x forward, y left, z up), the camera position in `base_link` is `(+0.045, -0.075, -0.005)` m. This value should be regarded as the current calibration configuration; the direction and geometric relationship should be verified in RViz before formal measurement or flight.

## Common Files

- `install_native.sh`: native host dependency and udev installation script;
- `launch_showcase.sh`: handheld D435 real-time semantic mapping showcase script;
- `launch_handheld_mapping.sh`, `run_once.sh`: handheld mapping and experiment scripts;
- `launch_d435_rtabmap.sh`: camera, RTAB-Map, and RViz launch script for PX4 odometry mode;
- `start_foxglove.sh`: Foxglove Bridge launch script;
- `rtabmap_d435.rviz`: RViz configuration;
- `launch_px4_mavros.sh`, `px4_tf.launch.xml`, `px4_tf_config.yaml`: MAVROS launch script and TF parameter override.

## References

- [MAVROS Humble Documentation](https://docs.ros.org/en/ros2_packages/humble/api/mavros/) and [upstream source](https://github.com/mavlink/mavros)
- [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros)
- GeographicLib geoid dataset: `/usr/share/GeographicLib/geoids/egm96-5.pgm`
