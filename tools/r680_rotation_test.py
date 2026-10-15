#!/usr/bin/env python3
"""E2 -- rotation in place, with an INDEPENDENT measurement of the turn angle.

Why the extra machinery: the odometry yaw comes from the wheel encoders, so it
shares the base's sign convention and cannot on its own tell you whether the
robot turned the way a positive angular.z asks for. That is the same blind spot
that made E1's direction question unanswerable from /odom alone.

The independent check uses the depth camera. When a robot rotates about its
vertical axis by theta, a fixed world feature at bearing beta appears at bearing
beta - theta in the ROBOT frame. The LaserScan is expressed in the robot's own
frame, so the whole range profile slides within the array by exactly the turn
angle. Cross-correlating the profile before and after therefore measures theta
using a sensor that never touches the encoders.

Caveat, stated up front: this assumes the scene is static and the robot
translates little. In an environment where people walk around (this one has),
a moving object can corrupt the match -- so the per-shift agreement score is
printed, not just the winning shift, and a weak score is reported as such.

Usage:  python3 tools/r680_rotation_test.py [angular_z] [seconds]
        defaults: 0.25 rad/s for 3.0 s (= 0.75 rad ~ 43 deg)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

RATE_HZ = 20.0
ZERO_HOLD_S = 1.0


def emit(msg):
    print(msg)
    sys.stdout.flush()


def yaw_of(q):
    """Yaw from a quaternion, in radians."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RotationTest(Node):
    def __init__(self, angular_z, seconds):
        super().__init__("r680_rotation_test")
        self.angular_z = angular_z
        self.seconds = seconds

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos)

        self.yaw = None
        self.scan = None
        self.scan_meta = None

    def _on_odom(self, m):
        self.yaw = yaw_of(m.pose.pose.orientation)

    def _on_scan(self, m):
        self.scan = list(m.ranges)
        self.scan_meta = (m.angle_min, m.angle_increment)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, z):
        m = Twist()
        m.angular.z = float(z)
        self.pub.publish(m)


def estimate_shift(before, after, max_shift):
    """Best integer beam shift aligning `after` with `before`, plus its score.

    Two things here are load-bearing, and both were found by testing this
    estimator on SYNTHETIC profiles before trusting it on real data:

    1. The search minimises MEAN SQUARED RESIDUAL, not "fraction of beams that
       agree within 0.15 m". A range profile is smooth, so neighbouring beams
       hold similar values and an agreement-count score produces a broad plateau
       of tied maxima -- argmax then lands wherever the window boundary happens
       to favour it. Measured on synthetic data with a known shift, the
       agreement-count version was wrong by a consistent -5 beams on EVERY case
       (0 -> -5, +60 -> +55, -45 -> -50, +120 -> +115, +15 -> +10, -8 -> -13).

    2. Shifts whose comparison window covers less than half the beams are
       rejected. Without this, a large shift "wins" by comparing only a handful
       of conveniently-similar beams near the edge.

    The returned score is still the agreement fraction at the winning shift --
    not used for the search, but it is the honest confidence signal for the
    caller: in a room where people walk around, a low score is how we find out
    that the match is a coincidence.
    """
    n = min(len(before), len(after))
    best_shift, best_cost = 0, float("inf")
    for shift in range(-max_shift, max_shift + 1):
        total = 0.0
        count = 0
        for i in range(n):
            j = i + shift
            if j < 0 or j >= n:
                continue
            a, b = before[i], after[j]
            if not (math.isfinite(a) and math.isfinite(b)):
                continue
            d = a - b
            total += d * d
            count += 1
        if count < 0.5 * n:
            continue
        cost = total / count
        if cost < best_cost:
            best_shift, best_cost = shift, cost

    # confidence = agreement fraction AT the winning shift
    good = total = 0
    for i in range(n):
        j = i + best_shift
        if j < 0 or j >= n:
            continue
        a, b = before[i], after[j]
        if not (math.isfinite(a) and math.isfinite(b)):
            continue
        total += 1
        if abs(a - b) < 0.15:
            good += 1
    score = (good / float(total)) if total else 0.0
    return best_shift, score, best_cost


def selftest():
    """Prove the shift estimator against synthetic profiles with a KNOWN shift.

    Run before trusting any real measurement. This exists because the first
    version of estimate_shift looked perfectly reasonable and was wrong by a
    consistent -5 beams on every synthetic case (see the docstring); only a
    known-answer test caught it. A tool whose whole job is to produce an
    independent measurement should not be its own only witness.
    """
    import random
    random.seed(7)
    n = 640
    prof = []
    for i in range(n):
        ang = -0.76 + i * (1.52 / n)
        prof.append(3.0 + 1.2 * math.sin(3 * ang) + 0.5 * math.cos(7 * ang)
                    + random.gauss(0, 0.02))

    emit("  合成数据自检：把已知平移施加到测距剖面上，看能否估回原值")
    failures = 0
    for truth in (0, 60, -45, 120, 15, -8, 200, -199):
        shifted = [prof[i - truth] if 0 <= i - truth < n else float("nan")
                   for i in range(n)]
        got, score, _ = estimate_shift(prof, shifted, 220)
        ok = got == truth
        failures += 0 if ok else 1
        emit("    真实 %+5d 束 -> 估计 %+5d 束  %s  (吻合率 %.0f%%)"
             % (truth, got, "✓" if ok else "✗", score * 100))
    emit("  %s" % ("自检全部通过" if failures == 0
                   else "自检有 %d 项失败 —— 不要相信实测结果" % failures))
    return failures


def main():
    args = sys.argv[1:]
    if args and args[0] == "--selftest":
        emit("=" * 62)
        emit("  转角估计算法自检（不连接机器人、不运动）")
        emit("=" * 62)
        return 1 if selftest() else 0

    angular_z = float(args[0]) if args else 0.25
    seconds = float(args[1]) if len(args) > 1 else 3.0

    rclpy.init()
    node = RotationTest(angular_z, seconds)

    emit("=" * 62)
    emit("  R680 E2 原地旋转测试")
    emit("=" * 62)
    emit("  指令: angular.z = %+.3f rad/s, 持续 %.2f s  =>  约 %+.1f 度"
         % (angular_z, seconds, math.degrees(angular_z * seconds)))
    emit("  正 angular.z 应为逆时针（从上方看，右手法则绕 +z）")
    emit("")

    emit("[1/5] 等待订阅连接 (3 s) ...")
    node.spin_for(3.0)
    if node.yaw is None:
        emit("  ✗ 没有 /odom，中止（不发任何速度）")
        node.destroy_node(); rclpy.shutdown()
        return 2
    if node.scan is None:
        emit("  ! 没有 /scan —— 将只报告里程计转角，无法做独立验证")

    before_scan = node.scan
    before_meta = node.scan_meta
    yaw0 = node.yaw
    emit("[2/5] 基准 yaw = %+.2f 度" % math.degrees(yaw0))

    emit("[3/5] 开始旋转 %+.3f rad/s ..." % angular_z)
    emit("      >>> 车现在开始转，请看着它 <<<")
    t_start = time.time()
    period = 1.0 / RATE_HZ
    sent = 0
    next_report = t_start + 0.5
    try:
        while time.time() - t_start < seconds:
            node.send(angular_z)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period)
            if time.time() >= next_report:
                next_report += 0.5
                if node.yaw is not None:
                    emit("      t=%4.1fs  yaw=%+7.2f 度   (第 %d 条)"
                         % (time.time() - t_start, math.degrees(node.yaw), sent))
    finally:
        emit("[4/5] 停止并补零 (持续 %.1f s)" % ZERO_HOLD_S)
        t_zero = time.time()
        zeros = 0
        while time.time() - t_zero < ZERO_HOLD_S:
            node.send(0.0)
            zeros += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period)

    duration = time.time() - t_start - ZERO_HOLD_S
    node.spin_for(1.5)

    emit("[5/5] 结果")
    emit("")
    yaw1 = node.yaw
    d_odom = wrap(yaw1 - yaw0) if (yaw0 is not None and yaw1 is not None) else None
    expected = angular_z * duration

    emit("  发送: %d 条角速度 + %d 条零速" % (sent, zeros))
    emit("  实际指令时长: %.2f s" % duration)
    emit("")
    emit("  ① 里程计（编码器）转角: %s"
         % ("n/a" if d_odom is None else "%+.2f 度" % math.degrees(d_odom)))
    emit("     预期: %+.2f 度" % math.degrees(expected))

    # --- the independent measurement --------------------------------------
    d_cam = None
    score = None
    if before_scan is not None and node.scan is not None and before_meta is not None:
        inc = before_meta[1]
        max_shift = int(abs(math.radians(80.0)) / abs(inc)) if inc else 200
        shift, score, _ = estimate_shift(before_scan, node.scan, max_shift)
        d_cam = shift * inc
        emit("")
        emit("  ② 深度相机（独立于编码器）转角: %+.2f 度" % math.degrees(d_cam))
        emit("     依据: 旋转 theta 后，固定世界特征在机器人系中的方位角平移 theta，")
        emit("           故扫描剖面的最佳匹配平移量 = 转角")
        emit("     匹配质量: %.0f%% 的光束对吻合（<0.15 m 视为吻合）" % (score * 100))
        if score < 0.5:
            emit("     ⚠ 匹配质量偏低 —— 场景可能有移动物体，此独立测量不可信")
    else:
        emit("")
        emit("  ② 深度相机: 无 /scan，跳过独立验证")

    emit("")
    if d_odom is None:
        emit("  判定: ? 没有里程计数据")
        verdict = 3
    elif abs(d_odom) < math.radians(3.0):
        emit("  判定: ✗ 几乎没有转动 (%+.2f 度) —— 角速度没到轮子" % math.degrees(d_odom))
        verdict = 4
    elif d_odom * angular_z < 0:
        emit("  判定: ✗ 转向与指令相反 (指令 %+.3f, 实测 %+.2f 度)"
             % (angular_z, math.degrees(d_odom)))
        verdict = 5
    else:
        emit("  判定: ✓ 转向与指令一致 (%+.3f -> %+.2f 度)"
             % (angular_z, math.degrees(d_odom)))
        if d_cam is not None and score is not None and score >= 0.5:
            if d_cam * d_odom > 0:
                emit("        且深度相机独立测得 %+.2f 度，同向 —— 两个传感器互证"
                     % math.degrees(d_cam))
            else:
                emit("        ⚠ 但深度相机测得 %+.2f 度，方向相反 —— 需查明"
                     % math.degrees(d_cam))
        verdict = 0

    emit("")
    emit("  最终 /cmd_vel_nav 已归零")
    emit("=" * 62)

    node.destroy_node()
    rclpy.shutdown()
    return verdict


if __name__ == "__main__":
    sys.exit(main())
