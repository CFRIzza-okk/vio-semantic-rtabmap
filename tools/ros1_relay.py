#!/usr/bin/env python2
"""ROS 1 side of the R680 UDP relay.

Run in the TX1 ROS 1 environment. It forwards read-only odom/IMU/TF data and
accepts only the authenticated UDP cmd_vel channel, with a local watchdog.

MUST RUN UNDER PYTHON 2.7
-------------------------
TX1 is Ubuntu 18.04 / ROS Melodic, whose ``rospy`` is built for Python 2.7
(``python3 -c "import rospy"`` fails there with ModuleNotFoundError). Use::

    python2 ~/ros1_relay.py _peer_host:=192.168.2.15

This file is deliberately kept runnable on **both** Python 2.7 and Python 3,
but it is only ever executed on Python 2.7. Every construct below that differs
between the two is flagged with a ``py2`` comment, because the earlier version
was written as Python 3 and therefore crashed on TX1 before it could bind a
socket at all -- see the notes on each fix.
"""
import hashlib
import hmac
import json
import math
import os
import socket
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage


def _canonical(obj):
    """Serialise a relay body into the exact bytes that get signed.

    `sort_keys=True` is not cosmetic -- it is the whole point.

    The two ends of this relay run different Python versions: TX1 is ROS Melodic
    on Python 2.7, the Orin is ROS 2 Humble on Python 3.10. A plain
    `json.dumps(body)` emits keys in each interpreter's own dict order -- and
    those orders differ. Python 3 dicts preserve insertion order; Python 2.7
    dicts are hash-ordered. So the SAME body serialises to DIFFERENT bytes
    depending on which side does it, the HMAC over it differs, and the receiver
    rejects the packet.

    That asymmetry hid the bug for a long time. TX1 always signs what TX1
    serialised, and the Orin -- on receipt -- re-serialises a dict whose key
    order came from the JSON text, so the uplink (odom, imu, tf, battery) always
    matched. Only the downlink (cmd_vel) was re-serialised from a Python 3 dict
    by a Python 2 process, and every velocity packet was silently dropped as
    "invalid relay packet". Mapping and odometry looked perfect throughout; the
    robot simply never received a single command.

    Sorting makes the bytes a pure function of the data, independent of the
    interpreter on either end. Any change here MUST be applied to both sides at
    once -- a one-sided change breaks the link in the direction that currently
    works.
    """
    return json.dumps(obj, separators=(",", ":"), sort_keys=True).encode()


# py2: time.monotonic() is Python 3.3+. On 2.7 it raises AttributeError, which
# would have crashed the safety watchdog the moment cmd_vel was enabled.
# time.time() is the only portable fallback; the watchdog tolerates wall-clock
# time because TX1's clock is not stepped while the robot is running.
try:
    from time import monotonic as _monotonic
except ImportError:                                  # pragma: no cover - py2
    _monotonic = time.time

# py2: socket.error subclasses IOError on 2.7 but IS OSError on 3.x. Catching
# both keeps identical behaviour on either interpreter -- plain "except OSError"
# silently fails to catch socket send/recv errors under Python 2.7.
_SOCKET_ERRORS = (OSError, socket.error)


def _is_finite(value):
    """Return True when ``value`` is neither NaN nor infinite.

    py2: math.isfinite() is Python 3.2+, so it raises AttributeError on 2.7.
    """
    return not (math.isnan(value) or math.isinf(value))


class Relay:
    def __init__(self):
        # Default matches the wired link (Orin NX eno1). It used to be the
        # 192.168.1.16 WiFi address, so running without _peer_host:= sent every
        # packet out of a different interface than the relay expects.
        self.peer = (rospy.get_param("~peer_host", "192.168.2.15"), int(rospy.get_param("~rx_port", 19001)))
        self.bind_port = int(rospy.get_param("~tx_port", 19000))
        self.secret = rospy.get_param("~secret", "change-me-r680")
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.25))
        self.max_linear_x = float(rospy.get_param("~max_linear_x", 0.35))
        self.max_linear_y = float(rospy.get_param("~max_linear_y", 0.35))
        self.max_angular_z = float(rospy.get_param("~max_angular_z", 0.8))
        self.enable_cmd_vel = bool(rospy.get_param("~enable_cmd_vel", False))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.bind_port))
        self.sock.settimeout(0.2)
        self.last_cmd = 0.0
        self.last_seq = -1
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) if self.enable_cmd_vel else None
        self.zero_cmd = Twist()
        rospy.Subscriber("/odom", Odometry, lambda m: self.send("odom", self.odom(m)), queue_size=10)
        rospy.Subscriber("/imu", Imu, lambda m: self.send("imu", self.imu(m)), queue_size=10)
        rospy.Subscriber("/tf", TFMessage, lambda m: self.send("tf", self.tf(m)), queue_size=10)
        rospy.Subscriber("/tf_static", TFMessage, lambda m: self.send("tf_static", self.tf(m)), queue_size=10)
        # Battery voltage. Measured on TX1 2026-10-14: /PowerVoltage is
        # std_msgs/Float32, published by /wheeltec_robot, ~11.90 V.
        #
        # Without this the exploration node's battery guard is inert: its
        # battery_topic was empty, so min_battery (10.5 V) could never trip and a
        # long autonomous run would simply drive until the pack died -- which has
        # already happened once on this robot (TX1 lost power mid-session).
        #
        # queue_size=1 is deliberate: only the latest voltage matters, and battery
        # telemetry is low-rate. A deep queue would just add latency to a value
        # whose whole purpose is to be current.
        rospy.Subscriber("/PowerVoltage", Float32,
                         lambda m: self.send("battery", self.battery(m)), queue_size=1)
        # py2: threading.Thread(daemon=True) is Python 3.3+; on 2.7 it raises
        # "TypeError: __init__() got an unexpected keyword argument 'daemon'"
        # and the node died here, before ever binding its socket.
        self.thread = threading.Thread(target=self.receive)
        self.thread.daemon = True
        self.thread.start()
        rospy.Timer(rospy.Duration(0.05), self.watchdog)

    @staticmethod
    def stamp(s): return {"sec": int(s.secs), "nanosec": int(s.nsecs)}
    @classmethod
    def header(cls, h): return {"stamp": cls.stamp(h.stamp), "frame_id": h.frame_id}
    @classmethod
    def vec(cls, v): return {"x": v.x, "y": v.y, "z": v.z}
    @classmethod
    def quat(cls, q): return {"x": q.x, "y": q.y, "z": q.z, "w": q.w}
    @classmethod
    def odom(cls, m):
        return {"header": cls.header(m.header), "child_frame_id": m.child_frame_id,
                "pose": {"position": cls.vec(m.pose.pose.position), "orientation": cls.quat(m.pose.pose.orientation), "covariance": list(m.pose.covariance)},
                "twist": {"linear": cls.vec(m.twist.twist.linear), "angular": cls.vec(m.twist.twist.angular), "covariance": list(m.twist.covariance)}}
    @classmethod
    def imu(cls, m):
        return {"header": cls.header(m.header), "orientation": cls.quat(m.orientation), "orientation_covariance": list(m.orientation_covariance),
                "angular_velocity": cls.vec(m.angular_velocity), "angular_velocity_covariance": list(m.angular_velocity_covariance),
                "linear_acceleration": cls.vec(m.linear_acceleration), "linear_acceleration_covariance": list(m.linear_acceleration_covariance)}
    @classmethod
    def battery(cls, m):
        # std_msgs/Float32 carries no header and no stamp, so the receiver cannot
        # age-check it by timestamp the way it does odom/tf. The ROS 2 side
        # therefore timestamps on arrival -- see Relay._on_battery -- and the
        # exploration node must treat a stale battery reading as unknown rather
        # than as "still fine".
        return {"voltage": float(m.data)}
    @classmethod
    def tf(cls, m):
        return [{"header": cls.header(t.header), "child_frame_id": t.child_frame_id,
                 "translation": cls.vec(t.transform.translation), "rotation": cls.quat(t.transform.rotation)} for t in m.transforms]

    def sign(self, body): return hmac.new(self.secret.encode(), body, hashlib.sha256).hexdigest()
    def send(self, kind, payload):
        self.tx_seq = getattr(self, "tx_seq", 0) + 1
        # Python 2.7 on ROS Melodic has no monotonic_ns; this field is
        # diagnostic only and does not replace the preserved ROS header stamp.
        sent_at = int(time.time() * 1000000000)
        msg = {"kind": kind, "seq": self.tx_seq, "payload": payload, "sent_at": sent_at}
        raw = _canonical(msg)
        packet = json.dumps({"body": msg, "sig": self.sign(raw)},
                            separators=(",", ":"), sort_keys=True).encode()
        try: self.sock.sendto(packet, self.peer)
        except _SOCKET_ERRORS as e: rospy.logwarn_throttle(5.0, "relay send failed: %s", e)

    def receive(self):
        while not rospy.is_shutdown():
            try: packet, _ = self.sock.recvfrom(8192)
            except socket.timeout: continue
            except _SOCKET_ERRORS: return
            try:
                outer = json.loads(packet.decode()); body = outer["body"]
                # Must use the same canonical form the sender signed. See
                # _canonical(): re-serialising with the interpreter's own dict
                # order would produce different bytes.
                raw = _canonical(body)
                # .encode("ascii") is NOT decoration -- without it every downlink
                # packet is discarded.
                #
                # json.loads() hands back `unicode` on Python 2, while
                # sign() returns a `str` hexdigest. Python 2's
                # hmac.compare_digest() requires two byte strings and raises
                # "TypeError: 'unicode' does not have the buffer interface" for a
                # unicode argument -- for EVERY packet, however valid. The
                # except below swallowed that TypeError into the same generic
                # warning as a bad signature, so the symptom was one throttled
                # "invalid relay packet" line every five seconds, with valid
                # cmd_vel payloads and growing sequence numbers right there in
                # the log, and a robot that never moved.
                #
                # The uplink never hit this: its verifier runs on Python 3, whose
                # compare_digest() accepts str. One more asymmetry that let the
                # bug live.
                if not hmac.compare_digest(outer["sig"].encode("ascii"), self.sign(raw)) \
                        or body.get("kind") != "cmd_vel": continue
                if not self.enable_cmd_vel: continue
                seq = int(body.get("seq", -1))
                if seq <= self.last_seq: continue
                t = body["payload"]; msg = Twist()
                for axis in ("x", "y", "z"):
                    setattr(msg.linear, axis, float(t["linear"].get(axis, 0.0)))
                    setattr(msg.angular, axis, float(t["angular"].get(axis, 0.0)))
                if not all(_is_finite(v) for v in (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)): continue
                msg.linear.x = max(-self.max_linear_x, min(self.max_linear_x, msg.linear.x))
                msg.linear.y = max(-self.max_linear_y, min(self.max_linear_y, msg.linear.y))
                msg.angular.z = max(-self.max_angular_z, min(self.max_angular_z, msg.angular.z))
                msg.linear.z = msg.angular.x = msg.angular.y = 0.0
                self.cmd_pub.publish(msg); self.last_cmd = _monotonic(); self.last_seq = seq
            # py2: json.JSONDecodeError is Python 3.5+. On 2.7 that name does
            # not exist, so the except clause itself raised AttributeError on
            # the first malformed packet. 2.7's json raises ValueError, and on
            # 3.5+ JSONDecodeError subclasses ValueError, so this covers both.
            #
            # The message names the exception type and quotes the offending
            # bytes. One generic "invalid relay packet" for a malformed
            # datagram, a failed HMAC and a missing key made a real outage look
            # like background noise: cmd_vel was dropped for a long time and the
            # only symptom was one throttled line every five seconds, with no
            # way to tell which of the three it was or what had arrived.
            except (ValueError, KeyError, TypeError) as exc:
                rospy.logwarn_throttle(
                    5.0, "invalid relay packet: %s: %s | first 200 bytes: %.200r",
                    type(exc).__name__, exc, packet)

    def watchdog(self, _):
        if self.enable_cmd_vel and _monotonic() - self.last_cmd > self.cmd_timeout:
            self.cmd_pub.publish(self.zero_cmd)


def main():
    rospy.init_node("r680_ros1_relay")
    Relay(); rospy.spin()

if __name__ == "__main__": main()
