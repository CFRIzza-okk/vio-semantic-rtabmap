#!/usr/bin/env python3
"""抓取一对同步的「彩色 + 对齐深度」图像与相机内参，用于对齐性核查。

用途
----
配合标定板做三项核查（见 ~/VIO/BOARD_ALIGNMENT_TEST.md）：
  A. 彩色图与深度图的横向对齐
  B. 相机内参一致性
  C. 深度值精度（对标定板平面做拟合残差）

为什么不用录 rosbag
------------------
对齐核查只需要一帧静态图像，录 bag 又要几十秒又要几十 MB。
本脚本用 ApproximateTimeSynchronizer 抓一对时间对齐的 RGB-D 并落盘。

用法
----
终端 1（启动相机）：
    source /opt/ros/humble/setup.bash
    ros2 launch realsense2_camera rs_launch.py \
      align_depth.enable:=true \
      rgb_camera.color_profile:=640x480x30 \
      depth_module.depth_profile:=640x480x30

终端 2（抓图，前缀自己定，便于拍多个距离）：
    source /opt/ros/humble/setup.bash
    python3 ~/VIO/tools/capture_rgbd.py ~/VIO/data/board_150

输出文件
--------
    <前缀>_color.png    彩色图（可肉眼查看）
    <前缀>_color.npy    彩色图（供脚本读取）
    <前缀>_depth16.png  深度原图 16UC1 毫米
    <前缀>_depth.npy    深度（米，float32）
    <前缀>_info.json    相机内参 K/D 与时间戳
"""
import json
import sys
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    import message_filters
except ImportError:
    print("缺少 message_filters，请先 source /opt/ros/humble/setup.bash")
    raise

PREFIX = sys.argv[1] if len(sys.argv) > 1 else "/home/jetson/VIO/data/board_test"


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class RgbdCapture(Node):
    def __init__(self):
        super().__init__("rgbd_capture")
        self.bridge = CvBridge()
        self.done = threading.Event()
        self.info = None

        qos = qos_profile_sensor_data
        self.sub_color = message_filters.Subscriber(
            self, Image, "/camera/camera/color/image_raw", qos_profile=qos
        )
        self.sub_depth = message_filters.Subscriber(
            self, Image, "/camera/camera/aligned_depth_to_color/image_raw", qos_profile=qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth], queue_size=8, slop=0.05
        )
        self.sync.registerCallback(self.on_pair)

        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.on_info, qos_profile=qos
        )

    def on_info(self, msg):
        self.info = msg

    def on_pair(self, color_msg, depth_msg):
        if self.done.is_set():
            return
        rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        scale = 0.001 if depth_msg.encoding == "16UC1" else 1.0
        depth_m = raw.astype(np.float32) * scale

        cv2.imwrite(f"{PREFIX}_color.png", rgb)
        cv2.imwrite(f"{PREFIX}_depth16.png", raw)
        np.save(f"{PREFIX}_color.npy", rgb)
        np.save(f"{PREFIX}_depth.npy", depth_m)

        info = {}
        if self.info is not None:
            info = {
                "width": int(self.info.width),
                "height": int(self.info.height),
                "distortion_model": str(self.info.distortion_model),
                "k": [float(v) for v in self.info.k],
                "d": [float(v) for v in self.info.d],
                "fx": float(self.info.k[0]), "fy": float(self.info.k[4]),
                "cx": float(self.info.k[2]), "cy": float(self.info.k[5]),
            }
        dt_ms = abs(stamp_ns(color_msg) - stamp_ns(depth_msg)) / 1e6
        info["rgb_depth_dt_ms"] = dt_ms
        info["depth_encoding"] = str(depth_msg.encoding)
        info["depth_scale"] = scale
        with open(f"{PREFIX}_info.json", "w", encoding="utf-8") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)

        valid = (depth_m > 0.2) & (depth_m < 12.0) & np.isfinite(depth_m)
        # 标定板应落在中位数附近，报一下这个距离带的有效像素数
        if valid.sum() > 100:
            med = float(np.median(depth_m[valid]))
            band = valid & (np.abs(depth_m - med) < 0.15)
            dist_txt = f"深度中位数 {med:.3f} m，其 ±0.15 m 内 {int(band.sum())} px"
        else:
            dist_txt = "几乎没有有效深度"

        print(f"已保存 {PREFIX}_color.png / _depth16.png / _color.npy / _depth.npy / _info.json")
        print(f"分辨率 {rgb.shape[1]}x{rgb.shape[0]}  彩色-深度时间差 {dt_ms:.1f} ms")
        print(f"深度有效像素 {int(valid.sum())}/{depth_m.size} = {valid.mean() * 100:.1f}%  {dist_txt}")
        self.done.set()


def main():
    rclpy.init()
    node = RgbdCapture()
    for _ in range(300):  # 最多等 30 秒
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.done.is_set():
            break
    else:
        print("超时：没有收到同步的彩色+深度图像对，相机是否在运行？")
        print("检查： ros2 topic hz /camera/camera/color/image_raw")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
