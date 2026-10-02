#!/usr/bin/env python3
"""测量话题的"真实产出率"，而不是"投递率"。

为什么需要这个工具
------------------
`ros2 topic hz` 报告的是**订阅者收到了多少条消息**。当话题上是大图像
（640x480x3 = 921 KB）时，`ros2 topic hz` 自己就是一个相当昂贵的订阅者，
在负载较高的机器上会严重低报 —— 本机实测曾出现"只有相机在跑"却报 0.88 Hz
的荒谬结果，而同一次运行的相机节点自己认为在发 30 Hz。

关键区别：
    产出率 = 相机实际生成了多少帧  (由消息头时间戳的间隔推得)
    投递率 = 我收到了多少帧        (由计数除以时间得到)
    丢帧率 = 1 - 投递率/产出率

时间戳是随帧一起生成的，只要收到**任意几条**消息就能从相邻时间戳的差值
推出真实产出率，不受丢帧影响。这正是判断"是相机慢，还是我的订阅慢"的依据。

用法：
    python3 tools/measure_topic_rate.py /camera/camera/color/image_raw 10
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class RateProbe(Node):
    def __init__(self, topic: str, qos_depth: int = 5):
        super().__init__("rate_probe")
        self.stamps = []
        self.t0 = time.time()
        q = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, topic, self._cb, q)

    def _cb(self, msg: Image) -> None:
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)


def main() -> int:
    topic = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    rclpy.init()
    node = RateProbe(topic)
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    elapsed = time.time() - node.t0
    n = len(node.stamps)
    print(f"话题: {topic}")
    print(f"  采样时长      : {elapsed:.1f} s")
    print(f"  收到消息数    : {n}")

    if n < 3:
        print("  消息太少，无法判断（相机没启动？话题名写错？）")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"  投递率        : {n / elapsed:.1f} Hz")

    # 用时间戳差值推真实产出率
    deltas = [b - a for a, b in zip(node.stamps, node.stamps[1:]) if b > a]
    if deltas:
        deltas.sort()
        med = deltas[len(deltas) // 2]
        mean = sum(deltas) / len(deltas)
        print(f"  产出率(中位)  : {1.0 / med:.1f} Hz   (相邻帧时间戳差 {med * 1000:.1f} ms)")
        print(f"  产出率(均值)  : {1.0 / mean:.1f} Hz   (平均时间戳差 {mean * 1000:.1f} ms)")
        print(f"  最大时间戳间隔: {deltas[-1] * 1000:.1f} ms  <- 卡顿尖峰")
        dropped = 1.0 - (n / elapsed) / (1.0 / med)
        if dropped > 0.02:
            print(f"  => 估计丢帧率 : {dropped * 100:.0f}%  (相机在产出，但投递跟不上)")
        else:
            print("  => 投递完整，没有明显丢帧")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
