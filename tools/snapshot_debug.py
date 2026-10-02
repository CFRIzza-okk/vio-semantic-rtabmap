#!/usr/bin/env python3
"""抓取一帧 /semantics/debug_image 存成 PNG，用于离线核对检测与反投影标注。

用法: python3 /home/jetson/VIO/tools/snapshot_debug.py [输出路径]
"""

import sys
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/jetson/VIO/data/semantic_debug.png"


class Snapshot(Node):
    def __init__(self):
        super().__init__("semantic_snapshot")
        self.bridge = CvBridge()
        self.done = threading.Event()
        self.create_subscription(
            Image, "/semantics/debug_image", self.cb, qos_profile_sensor_data
        )

    def cb(self, msg):
        if self.done.is_set():
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(OUT, img)
        print(f"已保存 {OUT}  ({img.shape[1]}x{img.shape[0]}, {msg.encoding})")
        self.done.set()


def main():
    rclpy.init()
    node = Snapshot()
    # 边转边等，最多 15 秒
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.done.is_set():
            break
    else:
        print("超时: 15 秒内没有收到 /semantics/debug_image")
        print("请确认 semantic_detector 正在运行, 且 publish_debug_image:=true")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
