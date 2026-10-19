#!/usr/bin/env python3
"""Safety gate for ROS 2 navigation velocity commands.

Only this node should publish the bridged TX1 ``/cmd_vel`` output.  Any stale
or unsafe input is replaced with a zero Twist and a reason is published for
operator diagnostics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@dataclass
class TimedFlag:
    value: bool = False
    stamp_ns: int = 0


class CmdVelSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_safety")
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_linear_y", 0.35)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("localization_timeout", 0.5)
        self.declare_parameter("stop_on_obstacle", True)
        self.declare_parameter("require_localization", True)
        # 障碍锁存时是否允许"只转不走"的脱困旋转。
        #
        # 为什么需要它：障碍监视器只看**正前方 ±30°、0.45 m** 的扇区。命中后
        # 若把整个 Twist 置零（包括 angular.z），车就再也转不动了 —— 连 Nav2
        # 自己的 spin/backup 恢复行为也被清零，于是车头一旦进入近距死角就
        # **永久锁死**，只能人工把车挪开。实测：第二轮 66 s 卡死 -> 连续 5 次
        # 目标超时 -> ABORTED。
        #
        # 为什么"只转"是安全的：旋转不产生位移，不会把车带进任何**未测量**的
        # 区域（这正是它区别于倒车的地方 —— 倒车是朝未知方向平移，本工程一律
        # 不做）。而且它给出了唯一一条能自己走出死角的路径：转开 -> 前扇区变
        # 空 -> obstacle_stop 解除 -> 恢复通行。
        #
        # 仍然被严格封死的：linear.x / linear.y 恒为 0。也就是说障碍锁存期间
        # **车一步都不会朝前或朝侧面移动**，这是本参数不改变的硬性质。
        self.declare_parameter("obstacle_escape_rotation", True)
        self.declare_parameter("obstacle_escape_angular_z", 0.30)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._input: Optional[Twist] = None
        self._input_ns = 0
        self._estop = TimedFlag()
        self._obstacle = TimedFlag()
        self._localized = TimedFlag(value=True)
        self._last_reason = "startup"

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.create_subscription(Twist, input_topic, self._on_cmd, qos)
        self.create_subscription(Bool, "/safety/estop", self._on_estop, qos)
        self.create_subscription(Bool, "/safety/obstacle_stop", self._on_obstacle, qos)
        self.create_subscription(Bool, "/safety/localization_valid", self._on_localized, qos)
        self._pub = self.create_publisher(Twist, output_topic, qos)
        self._reason_pub = self.create_publisher(String, "/safety/stop_reason", qos)
        self.create_timer(0.05, self._tick)

    def _now(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_cmd(self, msg: Twist) -> None:
        self._input = msg
        self._input_ns = self._now()

    def _on_estop(self, msg: Bool) -> None:
        self._estop = TimedFlag(bool(msg.data), self._now())

    def _on_obstacle(self, msg: Bool) -> None:
        self._obstacle = TimedFlag(bool(msg.data), self._now())

    def _on_localized(self, msg: Bool) -> None:
        self._localized = TimedFlag(bool(msg.data), self._now())

    @staticmethod
    def _zero() -> Twist:
        return Twist()

    def _tick(self) -> None:
        now = self._now()
        timeout_ns = int(float(self.get_parameter("command_timeout").value) * 1e9)
        localization_timeout_ns = int(float(self.get_parameter("localization_timeout").value) * 1e9)
        source = self._input
        # A command is only usable while it is fresh. This is checked BEFORE the
        # obstacle branch on purpose: the escape rotation below forwards a
        # rotation command, and forwarding a stale one would be motion based on
        # a decision the planner may already have abandoned.
        fresh = source is not None and now - self._input_ns <= timeout_ns
        reason = "ok"
        output = self._zero()

        # Priority matters, and "stop everything" always wins. Only when none of
        # the three hard stops applies can an obstacle be answered with a
        # rotation instead of a full stop. Putting the obstacle branch first (as
        # this did before escape rotation existed) would have let a robot that
        # cannot localize, or whose e-stop is pressed, or whose planner has gone
        # silent, still spin in place.
        if self._estop.value:
            reason = "emergency_stop"
        elif bool(self.get_parameter("require_localization").value) and (
            not self._localized.value or now - self._localized.stamp_ns > localization_timeout_ns
        ):
            reason = "localization_invalid_or_timeout"
        elif not fresh:
            reason = "cmd_vel_timeout"
        elif bool(self.get_parameter("stop_on_obstacle").value) and self._obstacle.value:
            if bool(self.get_parameter("obstacle_escape_rotation").value):
                # Translation stays at zero -- that is the invariant that makes
                # this safe. Only yaw is passed, and only up to the escape cap.
                cap = abs(float(self.get_parameter("obstacle_escape_angular_z").value))
                output.angular.z = max(-cap, min(cap, source.angular.z))
                reason = "obstacle_stop_escape_rotation"
            else:
                reason = "obstacle_stop"
        else:
            output.linear.x = max(-float(self.get_parameter("max_linear_x").value), min(float(self.get_parameter("max_linear_x").value), source.linear.x))
            output.linear.y = max(-float(self.get_parameter("max_linear_y").value), min(float(self.get_parameter("max_linear_y").value), source.linear.y))
            output.angular.z = max(-float(self.get_parameter("max_angular_z").value), min(float(self.get_parameter("max_angular_z").value), source.angular.z))
            # Never pass unsupported lateral/vertical motion to a planar base.
            output.linear.z = 0.0
            output.angular.x = output.angular.y = 0.0

        self._pub.publish(output)
        if reason != self._last_reason:
            # NOTE: rclpy's logger does not do printf-style substitution, so the
            # message must be formatted before it is passed in. Passing "%s" and
            # a second argument raises TypeError and kills the node on its first
            # tick -- i.e. exactly when this safety gate is needed.
            self.get_logger().info("motion output: %s" % reason)
            self._last_reason = reason
        # Publish the reason on every tick, not only when it changes. Edge-only
        # publishing means an operator or recorder that connects later sees an
        # empty topic and cannot tell why the robot is being held still.
        self._reason_pub.publish(String(data=reason))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

