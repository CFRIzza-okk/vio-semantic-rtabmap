#!/usr/bin/env python3
"""First supervised motion test for the R680 -- straight crawl at 0.06 m/s.

Design constraints, all deliberate:

  * SHORT AND SLOW. 1.5 s at 0.06 m/s is about 9 cm. Even if the wheel
    direction turns out to be inverted, or a scale factor is wrong, the robot
    cannot travel far enough to hit anything in a cleared test area.

  * THE ZERO IS NOT OPTIONAL. A `finally` block publishes zero velocity at 20 Hz
    for a full second no matter how the run ends -- exception, Ctrl-C, whatever.
    This is this project's standing rule: a continuous /cmd_vel publish must be
    followed by an immediate zero. The TX1 relay watchdog (0.25 s) and the
    cmd_vel_safety watchdog (0.5 s) are independent backstops behind it, but they
    are backstops, not the plan.

  * THE COMMAND GOES TO /cmd_vel_nav, NOT /cmd_vel_safe. /cmd_vel_nav is the
    input of cmd_vel_safety, so the run exercises the real chain including the
    limiting and the stop-reason logic. Publishing straight to /cmd_vel_safe
    would bypass the very component this test is meant to validate.

  * OBSERVED, NOT ASSUMED. The script samples the odometry and the full safety
    state, then reports measured displacement rather than "the command was sent".

Usage:  python3 tools/r680_first_motion.py [linear_x] [seconds]
        defaults: 0.06 m/s for 1.5 s
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

RATE_HZ = 20.0
ZERO_HOLD_S = 1.0


def emit(msg):
    """print + flush. os._exit() elsewhere in this project skipped buffers and
    silently discarded output, which made a working script look like it did
    nothing -- so never rely on implicit flushing."""
    print(msg)
    sys.stdout.flush()


class FirstMotion(Node):
    def __init__(self, linear_x, seconds):
        super().__init__("r680_first_motion")
        self.linear_x = linear_x
        self.seconds = seconds

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_safe, qos)

        self.odom = None
        self.safe = None
        self.safe_count = 0
        self.odom_count = 0

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)
        self.odom_count += 1

    def _on_safe(self, m):
        self.safe = (m.linear.x, m.angular.z)
        self.safe_count += 1

    def spin_briefly(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, x, z=0.0):
        m = Twist()
        m.linear.x = float(x)
        m.angular.z = float(z)
        self.pub.publish(m)


def main():
    args = sys.argv[1:]
    linear_x = float(args[0]) if args else 0.06
    seconds = float(args[1]) if len(args) > 1 else 1.5

    rclpy.init()
    node = FirstMotion(linear_x, seconds)

    emit("=" * 62)
    emit("  R680 首次运动测试")
    emit("=" * 62)
    emit("  指令: linear.x = %.3f m/s, 持续 %.2f s  =>  约 %.1f cm"
         % (linear_x, seconds, linear_x * seconds * 100))
    emit("  路径: /cmd_vel_nav -> cmd_vel_safety -> /cmd_vel_safe -> relay -> TX1")
    emit("")

    # --- warm up: let subscriptions actually connect before measuring --------
    emit("[1/5] 等待订阅连接 (3 s) ...")
    node.spin_briefly(3.0)

    if node.odom is None:
        emit("  ✗ 3 s 内没有收到 /odom —— 上行链路没通，中止（不发任何速度）")
        node.destroy_node(); rclpy.shutdown()
        return 2
    if node.safe_count == 0:
        emit("  ! 尚未收到 /cmd_vel_safe（安全节点在无指令时不发布，属正常）")

    emit("[2/5] 基准里程计: x=%.4f  y=%.4f" % node.odom)
    x0, y0 = node.odom

    # --- the run ------------------------------------------------------------
    emit("[3/5] 开始发速度 %.3f m/s ..." % linear_x)
    emit("      >>> 车现在开始移动，请看着它 <<<")
    t_start = time.time()
    period = 1.0 / RATE_HZ
    sent = 0
    next_report = t_start + 0.5
    try:
        while time.time() - t_start < seconds:
            node.send(linear_x)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period)
            # Live progress, so someone watching the robot can line the motion up
            # with a number instead of guessing afterwards which movement it was.
            if time.time() >= next_report:
                next_report += 0.5
                if node.odom is not None:
                    emit("      t=%4.1fs  odom x=%+.4f  y=%+.4f   (发送第 %d 条)"
                         % (time.time() - t_start, node.odom[0], node.odom[1], sent))
    finally:
        # ---- THE IRON RULE: zero immediately, and hold it ----------------
        emit("[4/5] 停止并补零 (持续 %.1f s)" % ZERO_HOLD_S)
        t_zero = time.time()
        zeros = 0
        while time.time() - t_zero < ZERO_HOLD_S:
            node.send(0.0)
            zeros += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period)

    duration = time.time() - t_start - ZERO_HOLD_S

    # --- settle, then measure ----------------------------------------------
    node.spin_briefly(1.5)
    emit("[5/5] 结果")
    emit("")

    if node.odom is None:
        emit("  ✗ 丢失 /odom，无法测量位移")
        node.destroy_node(); rclpy.shutdown()
        return 3

    x1, y1 = node.odom
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    expected = linear_x * duration

    emit("  发送: %d 条速度 + %d 条零速" % (sent, zeros))
    emit("  实际指令时长: %.2f s" % duration)
    emit("")
    emit("  里程计  起点 (%.4f, %.4f)  ->  终点 (%.4f, %.4f)" % (x0, y0, x1, y1))
    emit("  位移    dx=%.4f m  dy=%.4f m  |d|=%.4f m" % (dx, dy, dist))
    emit("  预期    %.4f m (%.3f m/s x %.2f s)" % (expected, linear_x, duration))
    if expected > 1e-6:
        emit("  比值    %.2f" % (dist / expected))
    emit("")

    # --- verdict ------------------------------------------------------------
    # The point of this test is the wheel-direction question that E0 would have
    # answered: does the base move the way a positive x command says it should?
    #
    # The comparison must be SIGN AGAINST SIGN, not sign against zero. An earlier
    # version tested `dx < 0` and called it "wheels are inverted" -- which is
    # simply wrong when the command itself was negative. A backward command
    # producing a backward displacement is correct behaviour, and that version
    # reported it as a failure. What actually indicates an inverted base is a
    # displacement whose sign is OPPOSITE to the command's.
    commanded = linear_x
    if dist < 0.005:
        emit("  判定: ✗ 几乎没有移动 —— 指令没有到达底盘，或底盘没响应")
        emit("        查 /safety/stop_reason、TX1 relay 日志、/cmd_vel_safe 是否有数据")
        verdict = 4
    elif commanded != 0.0 and dx * commanded < 0:
        emit("  判定: ✗ 位移方向与指令相反 (指令 %+.3f, dx=%+.4f)"
             % (commanded, dx))
        emit("        —— 底盘符号相反，必须先修正，绝不能进自主探索")
        verdict = 5
    elif dist > abs(expected) * 2.5:
        emit("  判定: ! 走得比预期远得多 (比值 %.2f) —— 检查限速与编码器标定"
             % (dist / abs(expected)))
        verdict = 6
    else:
        emit("  判定: ✓ 位移方向与指令一致 (%+.3f -> dx=%+.4f)，比值 %.2f"
             % (commanded, dx, dist / abs(expected)))
        verdict = 0

    emit("")
    emit("  最终 /cmd_vel_safe = %s" % (node.safe,))
    emit("=" * 62)

    node.destroy_node()
    rclpy.shutdown()
    return verdict


if __name__ == "__main__":
    sys.exit(main())
