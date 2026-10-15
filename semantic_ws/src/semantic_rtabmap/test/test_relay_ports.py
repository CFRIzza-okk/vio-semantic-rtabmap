"""Regression guards for the ROS 1 <-> ROS 2 relay port wiring.

Why this file exists
--------------------
The relay's port parameters do **not** mean the same thing on the two sides:

* ROS 2 ``ros2_relay_node.py``: ``rx_port`` = the port it *binds*,
  ``tx_port`` = the port it *sends to* on the peer.
* TX1 ``ros1_relay.py``: ``tx_port`` = the port it *binds*,
  ``rx_port`` = the port it *sends to* on the peer.

The two are inverted. Reading either file alone looks perfectly reasonable,
which is exactly how this shipped broken: both sides bound 19000 and both sent
to the peer's 19001, so nothing listened on 19001 and **not a single packet got
through in either direction** — silently, with no error on either side.

So there are two guards here:

1. ``test_shipped_ports_are_complementary_with_tx1`` — a static check that the
   shipped ``r680_relay.yaml`` and TX1's defaults form a complementary pair.
   This is the one that would have caught the real bug.
2. ``test_relay_round_trip`` — a live loopback that impersonates TX1 and proves
   telemetry is received and commands are transmitted, in both directions.

Run with::

    cd ~/VIO/semantic_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest src/semantic_rtabmap/test/test_relay_ports.py -v
"""

import hashlib
import hmac
import itertools
import json
import re
import socket
import time
from pathlib import Path

import pytest
import rclpy

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]

# Deliberately unusual ports so the loopback never collides with a real relay or
# with the shipped 19000/19001 pair. Each fixture invocation takes a fresh pair:
# the relay binds a real UDP port, and reusing one across tests races with the
# previous socket's teardown (EADDRINUSE).
_PORT_PAIRS = itertools.count()
TEST_SECRET = "test-secret"


def _next_ports():
    n = next(_PORT_PAIRS)
    return 19531 + n * 2, 19530 + n * 2       # (bind, tx) for the ROS 2 side


def _read_ports(yaml_path):
    """Pull rx_port/tx_port out of the shipped YAML without needing PyYAML."""
    text = yaml_path.read_text()
    rx = re.search(r"^\s*rx_port:\s*(\d+)", text, re.M)
    tx = re.search(r"^\s*tx_port:\s*(\d+)", text, re.M)
    assert rx and tx, "r680_relay.yaml must define rx_port and tx_port"
    return int(rx.group(1)), int(tx.group(1))


def _tx1_default_ports(script_path):
    """Extract TX1's bind and destination port defaults from its source.

    ``bind_port`` comes from the ``~tx_port`` parameter and the destination from
    ``~rx_port`` — the inversion documented above, so read them by expression
    rather than by parameter name.
    """
    text = script_path.read_text()
    bind = re.search(r'bind_port\s*=\s*int\(\s*rospy\.get_param\(\s*"~tx_port"\s*,\s*(\d+)',
                     text)
    dest = re.search(r'self\.peer\s*=\s*\(.*?rospy\.get_param\(\s*"~rx_port"\s*,\s*(\d+)',
                     text, re.S)
    assert bind and dest, "could not locate TX1 relay port defaults"
    return int(bind.group(1)), int(dest.group(1))


def test_shipped_ports_are_complementary_with_tx1():
    """The shipped ROS 2 ports must meet TX1's bind/destination ports exactly.

    If this fails the relay is dead in both directions, and nothing else in the
    stack will notice: the node starts, binds happily, and simply never
    exchanges a packet.
    """
    tx1_script = REPO_ROOT / "tools" / "ros1_relay.py"
    if not tx1_script.exists():
        pytest.skip("TX1 relay script not present in this checkout")

    rx_port, tx_port = _read_ports(PACKAGE_ROOT / "config" / "r680_relay.yaml")
    tx1_bind, tx1_dest = _tx1_default_ports(tx1_script)

    # ROS 2 sends where TX1 listens...
    assert tx_port == tx1_bind, (
        "ROS 2 tx_port=%d but TX1 binds %d; commands would go nowhere"
        % (tx_port, tx1_bind)
    )
    # ...and ROS 2 listens where TX1 sends.
    assert rx_port == tx1_dest, (
        "ROS 2 rx_port=%d but TX1 sends to %d; telemetry would go nowhere"
        % (rx_port, tx1_dest)
    )
    # The two roles must not collide on one port.
    assert rx_port != tx_port


def _sign(secret, body):
    """Sign exactly the way the relay does -- by CALLING the relay's own code.

    This used to re-implement the canonical form as a bare
    ``json.dumps(body, separators=(",", ":"))``. When the relay switched to
    ``sort_keys=True`` (so that Python 2 and Python 3 serialise a dict to the
    same bytes), this copy silently went stale: the test kept producing a
    signature the relay no longer computed, every packet was dropped as a
    signature mismatch, and the failure surfaced as "relay never republished
    /odom" -- a symptom that points at the network, not at the helper.

    A hand-copied canonical form is a second implementation of a security-
    relevant function, and it will drift. Import the real one instead, so this
    test fails for the right reason if the relay's signing changes.
    """
    from semantic_rtabmap.ros2_relay_node import Relay
    return hmac.new(secret.encode(), Relay.canonical(body), hashlib.sha256).hexdigest()


def _wrap(secret, kind, payload):
    body = {"kind": kind, "payload": payload}
    return json.dumps({"body": body, "sig": _sign(secret, body)},
                      separators=(",", ":"), sort_keys=True).encode()


def _odom_payload():
    return {
        "header": {"stamp": {"sec": 1234, "nanosec": 5678},
                   "frame_id": "odom_combined"},
        "child_frame_id": "base_footprint",
        "pose": {
            "position": {"x": 1.5, "y": -0.25, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "covariance": [0.0] * 36,
        },
        "twist": {
            "linear": {"x": 0.12, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.3},
            "covariance": [0.0] * 36,
        },
    }


@pytest.fixture
def relay_pair():
    """Run the real ROS 2 relay node against a socket impersonating TX1.

    Yields ``(node, fake_tx1, bind_port, topics)`` — ``bind_port`` is where the
    ROS 2 relay listens, i.e. where the fake TX1 sends telemetry.

    ``topics`` holds per-test topic names. The relay defaults to the real
    ``/odom`` / ``/imu`` / ``/cmd_vel_safe``, and using those made this file pass
    only while **no real relay was running**: the live relay publishes /odom at
    ~20 Hz in the same ROS domain, so ``test_relay_rejects_bad_signature`` saw
    those messages on its own subscription and reported "relay accepted a packet
    with an invalid signature". Measured on the robot, that is exactly how it
    failed. Remapping to unique topics keeps the test hermetic either way.
    """
    bind_port, tx_port = _next_ports()
    topics = {
        "odom": "/test_odom_%d" % bind_port,
        "imu": "/test_imu_%d" % bind_port,
        "cmd": "/test_cmd_%d" % bind_port,
    }
    rclpy.init(args=[
        "--ros-args",
        "-p", "peer_host:=127.0.0.1",
        "-p", "rx_port:=%d" % bind_port,
        "-p", "tx_port:=%d" % tx_port,
        "-p", "secret:=%s" % TEST_SECRET,
        "-p", "output_odom:=%s" % topics["odom"],
        "-p", "output_imu:=%s" % topics["imu"],
        "-p", "input_cmd_vel:=%s" % topics["cmd"],
    ])
    from semantic_rtabmap.ros2_relay_node import Relay

    node = Relay()

    fake_tx1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_tx1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    fake_tx1.bind(("127.0.0.1", tx_port))
    fake_tx1.settimeout(3.0)

    yield node, fake_tx1, bind_port, topics

    fake_tx1.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_relay_round_trip(relay_pair):
    """Telemetry in one direction, commands in the other, over real UDP."""
    node, fake_tx1, bind_port, topics = relay_pair
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    received = []
    node.create_subscription(Odometry, topics["odom"], lambda m: received.append(m), 10)

    # ---- uplink: pretend to be TX1 sending telemetry -------------------
    packet = _wrap(TEST_SECRET, "odom", _odom_payload())
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not received:
        fake_tx1.sendto(packet, ("127.0.0.1", bind_port))
        rclpy.spin_once(node, timeout_sec=0.05)

    assert received, "relay never republished /odom for a valid signed packet"
    odom = received[0]
    assert odom.header.frame_id == "odom_combined"
    assert odom.child_frame_id == "base_footprint"
    assert odom.pose.pose.position.x == pytest.approx(1.5)
    assert odom.twist.twist.angular.z == pytest.approx(0.3)

    # ---- downlink: publish a command, TX1 must receive it --------------
    cmd_pub = node.create_publisher(Twist, topics["cmd"], 10)
    msg = Twist()
    msg.linear.x = 0.05
    msg.angular.z = 0.2
    cmd_pub.publish(msg)
    for _ in range(6):
        rclpy.spin_once(node, timeout_sec=0.05)

    data, _addr = fake_tx1.recvfrom(60000)
    outer = json.loads(data.decode())
    body = outer["body"]
    # The signature must verify with the shared secret.
    assert outer["sig"] == _sign(TEST_SECRET, body)
    assert body["kind"] == "cmd_vel"
    assert body["seq"] >= 1
    assert body["payload"]["linear"]["x"] == pytest.approx(0.05)
    assert body["payload"]["angular"]["z"] == pytest.approx(0.2)


def test_relay_rejects_bad_signature(relay_pair):
    """A tampered packet must be dropped, not acted on."""
    node, fake_tx1, bind_port, topics = relay_pair
    from nav_msgs.msg import Odometry

    received = []
    node.create_subscription(Odometry, topics["odom"], lambda m: received.append(m), 10)

    body = {"kind": "odom", "payload": _odom_payload()}
    packet = json.dumps({"body": body, "sig": "0" * 64},
                        separators=(",", ":")).encode()
    for _ in range(20):
        fake_tx1.sendto(packet, ("127.0.0.1", bind_port))
        rclpy.spin_once(node, timeout_sec=0.05)

    assert not received, "relay accepted a packet with an invalid signature"
