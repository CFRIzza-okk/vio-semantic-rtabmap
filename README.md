# Jetson D435 + RTAB-Map (native ROS 2 Humble)

Ubuntu 22.04 / Jetson ARM64 native packages: RTAB-Map ROS, RealSense ROS 2 driver, RealSense description, plus MAVROS built from source. No Docker is required.

## Install

In a terminal opened in this directory:

```bash
./install_native.sh
```

The script invokes `sudo apt-get`, installs the launcher to `/usr/local/bin/launch_d435_rtabmap`, installs the RViz layout from `rtabmap_d435.rviz`, and installs a udev permission rule for Intel RealSense USB devices. Enter the password in your local terminal prompt (do not send it in chat). After installation, unplug and reconnect the camera so the new rule applies.

## Start PX4 odometry (MAVROS)

PX4 is a CUAV FMU v6X (`3163:004c`) on `/dev/ttyACM0`. Start exactly **one** MAVROS instance:

```bash
~/VIO/launch_px4_mavros.sh
```

It launches `px4_tf.launch.xml`, which loads the stock PX4 plugin list and `px4_config.yaml` plus `px4_tf_config.yaml`. The extra file is what matters: MAVROS ships with `local_position.tf.send: false`, so without it nothing publishes the world→body transform and RViz cannot render anything.

Check the data in a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash     # not install/setup.bash (stale overlay prefix)
ros2 topic hz /mavros/imu/data                # ~50 Hz
ros2 topic hz /mavros/local_position/odom     # ~26 Hz
```

## Start D435 + RTAB-Map

Connect the D435 to a USB 3 port (`lsusb` shows Intel `8086`), then run:

```bash
~/VIO/launch_d435_rtabmap.sh
```

The wrapper starts, in order:

1. a static `base_link`→`camera_link` transform at `(0.045, -0.075, -0.005)` m with zero rotation,
2. `realsense2_camera` with color, depth and aligned depth,
3. `rtabmap_launch/rtabmap.launch.py` with `frame_id:=base_link`, `odom_topic:=/mavros/local_position/odom`, `visual_odometry:=false`, `publish_tf_map:=false`, and the workspace RViz layout `~/VIO/rtabmap_d435.rviz`.

## TF ownership (the reason the map was invisible)

Only one node may publish the world→body transform. The layout used here is:

```
map ──(MAVROS local_position.tf.send:=true)──> base_link ──(static)──> camera_link ──> camera_*_frame ──> camera_color_optical_frame
```

- MAVROS publishes `map`→`base_link` (dynamic, from PX4 local position).
- RTAB-Map has `publish_tf_map:=false`, so it never publishes `map`→`map` or a competing `map` transform. It consumes `/mavros/local_position/odom` and emits `/rtabmap/cloud_map` in the `map` frame.
- The launcher publishes `base_link`→`camera_link`; the RealSense driver adds the camera-internal frames.

Do **not** pass `odom_frame_id:=map` to `rtabmap.launch.py`: that argument is for RTAB-Map's own TF bookkeeping, not for choosing the odometry source. Passing it made RTAB-Map broadcast a `map`→`map` self-transform, which tf2 rejects (`TF_SELF_TRANSFORM`), leaving `map` non-existent and every RViz display in red.

Verify the tree with:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

Both must print a translation. If they report `Invalid frame ID "map"` or two unconnected trees, MAVROS was started without `px4_tf_config.yaml`.

## RViz layout

`rtabmap_d435.rviz` is based on the RTAB-Map RGB-D layout with two changes needed to actually see points:

- **Target Frame: `base_link`** for the Orbit view. PX4's local origin is tens of metres away from the vehicle (e.g. `x ≈ -100 m`), so with the stock target frame `map` the camera orbits an empty origin and the cloud is off-screen. Fixed Frame stays `map`.
- an extra **RTAB-Map Cloud** display on `/rtabmap/cloud_map` (the stock layout only shows the RTAB-Map plugin panels plus `/voxel_cloud`).

Displays: occupancy `Map`, `MapCloud`/`MapGraph`/`Info` (RTAB-Map plugins), `Odom Local Map`, `Camera Cloud` (`/voxel_cloud`), `RTAB-Map Cloud` (`/rtabmap/cloud_map`), TF and Grid.

Move the camera slowly through a textured area and turn back toward previously seen areas to trigger loop closure. Stop with Ctrl-C; the database is saved to `~/.ros/rtabmap.db` and reused on the next run.

## Troubleshooting

- **No point cloud, TF warnings in the Displays panel** — check `ros2 run tf2_ros tf2_echo map base_link`. If `map` does not exist, MAVROS is running without the TF override; restart it with `~/VIO/launch_px4_mavros.sh`.
- **`TF_SELF_TRANSFORM ... frame_id and child_frame_id "map"`** — something is launching RTAB-Map with `publish_tf_map:=true`; this launcher disables it.
- **Duplicate nodes / `ros2 node list` shows two `/mavros`** — an old MAVROS is still alive. Stop it; two instances fight over `/dev/ttyACM0`.
- **Map looks tilted, offset, or loop closures are rejected ("Not enough inliers")** — leftover nodes from an earlier session are in `~/.ros/rtabmap.db`. PX4's local origin changes between sessions, so old nodes live in a different frame. Start clean:

  ```bash
  mv ~/.ros/rtabmap.db ~/.ros/rtabmap.db.bak.$(date +%s)
  ```

- **Cloud publishes but nothing is visible** — the view is probably aimed at the PX4 origin instead of the vehicle; set the Orbit view Target Frame to `base_link` and zoom out (the map extends tens of metres).

## Extrinsic calibration used

Your convention: x right, y forward, z up; the IMU sits 7.5 cm left, 4.5 cm behind and 0.5 cm above the camera optical centre, with axes parallel (zero rotation). Mapped into ROS body axes (x forward, y left, z up) the camera sits at `(+0.045, -0.075, -0.005)` m in `base_link`. Verify orientation and geometry in RViz before relying on measurements.

## Useful files

- `install_native.sh`: host apt installer (interactive sudo prompt in your terminal)
- `launch_d435_rtabmap.sh`: camera + RTAB-Map + RViz launcher
- `rtabmap_d435.rviz`: RViz layout used by the launcher (`rviz_cfg`)
- `launch_px4_mavros.sh`, `px4_tf.launch.xml`, `px4_tf_config.yaml`: MAVROS launcher and the TF-enabling parameter override
- `docker-compose.yml`, `Dockerfile`: old container artifacts; not needed for the native deployment

## References

- [MAVROS Humble documentation](https://docs.ros.org/en/ros2_packages/humble/api/mavros/) and [upstream source](https://github.com/mavlink/mavros)
- [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros)
- GeographicLib geoid dataset is already present at `/usr/share/GeographicLib/geoids/egm96-5.pgm`.
