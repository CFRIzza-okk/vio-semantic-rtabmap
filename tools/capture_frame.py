#!/usr/bin/env python3
"""抓取一帧原始彩色图保存到磁盘，供离线模型对比使用。

用法: python3 /home/jetson/VIO/tools/capture_frame.py [输出前缀]
默认输出 /home/jetson/VIO/data/frame.{png,npy}
"""

import sys
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PREFIX = sys.argv[1] if len(sys.argv) > 1 else "/home/jetson/VIO/data/frame"


class Capture(Node):
    def __init__(self):
        super().__init__("frame_capture")
        self.bridge = CvBridge()
        self.done = threading.Event()
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.cb, qos_profile=qos_profile_sensor_data
        )

    def cb(self, msg):
        if self.done.is_set():
            return
        # 统一存成 BGR（ultralytics 对 numpy 输入按 BGR 解释）
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(f"{PREFIX}.png", img)
        np.save(f"{PREFIX}.npy", img)
        print(f"已保存 {PREFIX}.png / .npy  ({img.shape[1]}x{img.shape[0]})")
        self.done.set()


def main():
    rclpy.init()
    node = Capture()
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.done.is_set():
            break
    else:
        print("超时: 没有收到 /camera/camera/color/image_raw, 相机是否在运行?")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
