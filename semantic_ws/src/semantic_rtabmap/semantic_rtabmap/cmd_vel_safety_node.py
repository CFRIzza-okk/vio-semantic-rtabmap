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
        reason = "ok"
        source = self._input
        output = self._zero()

        if self._estop.value:
            reason = "emergency_stop"
        elif bool(self.get_parameter("stop_on_obstacle").value) and self._obstacle.value:
            reason = "obstacle_stop"
        elif bool(self.get_parameter("require_localization").value) and (
            not self._localized.value or now - self._localized.stamp_ns > localization_timeout_ns
        ):
            reason = "localization_invalid_or_timeout"
        elif source is None or now - self._input_ns > timeout_ns:
            reason = "cmd_vel_timeout"
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
