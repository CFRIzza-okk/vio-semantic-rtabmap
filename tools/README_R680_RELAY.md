# R680 ROS 1 / ROS 2 relay

`ros1_bridge` is unavailable in both images, so this project bridges TX1
(ROS 1 Melodic) and the Orin NX (ROS 2 Humble) with a small authenticated UDP
relay: JSON payloads, HMAC-SHA256 signed, sequence-numbered.

Everything below was verified end-to-end against the real robot on 2026-10-13.
Two defects that made the relay completely non-functional are documented at the
bottom, because both were silent — the relay looked configured and did nothing.

## Topology

```
   TX1  192.168.2.102 (eth0)  <---- wired ---->  Orin NX  192.168.2.15 (eno1)
   ROS 1 Melodic / Python 2.7                    ROS 2 Humble / Python 3
```

TX1's `eth0` carries only `192.168.2.102`; the `192.168.0.100` address is its
own WiFi AP. **The relay runs on the wired `192.168.2.x` pair** — earlier
revisions of this document used the `192.168.1.x` addresses, which was a
different interface and not the intended path. (The old `192.168.1.102`
address on TX1's eth0 was removed once the WiFi moved to `192.168.5.x`.)

## Port geometry (asymmetric — read this before changing anything)

The parameter names mean **opposite things on the two sides**:

| | binds | sends to peer |
|---|---|---|
| ROS 2 `ros2_relay_node` | `rx_port` (19001) | `tx_port` (19000) |
| TX1 `ros1_relay.py` | **`tx_port`** (19000) | **`rx_port`** (19001) |

So the two sides must form a *complementary* pair, not matching values:

```
Orin binds 19001, sends to TX1:19000   <-->   TX1 binds 19000, sends to Orin:19001
```

`test/test_relay_ports.py` asserts this against both files and runs a live
loopback. Run it before touching ports.

## Uplink / downlink

* **Uplink** — TX1 `/odom`, `/imu`, `/tf`, `/tf_static` → relay → Orin
  `/odom`, `/imu`, `/tf`, `/tf_static`.
* **Downlink** — Orin `/cmd_vel_safe` only → relay → TX1 `/cmd_vel`.

Only `/cmd_vel_safe` is accepted on the ROS 2 side; never wire Nav2 straight to
the relay. The ROS 2 relay drops TX1's `base_footprint -> camera_link`
transform, because that is the old Wheeltec camera frame and it conflicts with
the live D435.

## Safety model (three independent gates)

1. **`/cmd_vel_safe` is the only input.** On the Orin, `cmd_vel_safety` is the
   sole publisher of that topic; Nav2 publishes `/cmd_vel_nav`.
2. **`enable_cmd_vel` defaults to `false` on TX1.** With it false the relay does
   not even construct a `/cmd_vel` publisher — the topic does not exist on the
   ROS graph, so nothing can reach the wheels. Verified: `rostopic info
   /cmd_vel` returns `ERROR: Unknown topic /cmd_vel`.
3. **Watchdog + clamps on TX1.** Zero Twist after 0.25 s without a valid
   authenticated command; replayed sequence numbers rejected; x/y clamped to
   0.35 m/s, z to 0.8 rad/s; non-finite values dropped; z/roll/pitch forced 0.

The relay **never** starts the base driver and never moves the robot by itself.

## TX1 side

Requires **Python 2.7**. Melodic's `rospy` has no Python 3 build here —
`python3 -c "import rospy"` fails with `ModuleNotFoundError`. (An earlier
revision of this file said `python3`, which cannot work.)

```bash
# 1. deploy (from the Orin)
python3 /tmp/tx1put.py tools/ros1_relay.py /home/wheeltec/ros1_relay.py

# 2. run on TX1
. /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.2.102:11311
export ROS_IP=192.168.2.102
python2 /home/wheeltec/ros1_relay.py _peer_host:=192.168.2.15
```

`_peer_host` is the **Orin's** address. Leave `_enable_cmd_vel` unset (false)
until you deliberately want motion, and follow the project safety rules before
enabling it.

## ROS 2 side

```bash
cd ~/VIO/semantic_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=99
ros2 launch semantic_rtabmap r680_relay.launch.py
```

Publishes `/odom`, `/imu`, `/tf`, `/tf_static`; subscribes `/cmd_vel_safe`.

## Verifying without moving the robot

This is the safe loopback used to validate the bridge. It exercises both
directions over real UDP with the base driver **stopped**:

```bash
# Orin: expect 0.0.0.0:19001
ss -lunp | grep 1900

# TX1: publish a synthetic /odom (no base driver, so no motion is possible)
rostopic pub -r 10 /odom nav_msgs/Odometry \
  '{header: {frame_id: odom_combined}, child_frame_id: base_footprint,
    pose: {pose: {position: {x: 1.5, y: -0.25, z: 0.0}, orientation: {w: 1.0}}}}'

# Orin: the values above must appear, at ~10 Hz
ros2 topic echo /odom nav_msgs/msg/Odometry --once
ros2 topic hz /odom
```

For the downlink, stop the TX1 relay, listen on 19000, and publish
`/cmd_vel_safe` from the Orin — the packet must arrive with `sig ok`.

## Two defects that made this relay dead on arrival

Both were fixed and are now regression-tested. Recorded here because each was
invisible: the node started, bound its socket, logged nothing, and simply never
exchanged a packet.

1. **Crossed ports.** The shipped YAML had `rx_port: 19000 / tx_port: 19001`.
   Because the two sides invert the meaning of these names, *both* ends bound
   19000 and *both* sent to the peer's 19001 — where nobody listened. Nothing
   got through in either direction. Fix: `rx_port: 19001 / tx_port: 19000`.
   Guard: `test/test_relay_ports.py`.

2. **The TX1 script was Python 3 code and could not start.** It died with
   `TypeError: __init__() got an unexpected keyword argument 'daemon'` before
   binding anything, so the relay had never actually run on the robot. A full
   audit found five Python-3-only constructs; two would only have fired once
   motion was enabled, i.e. inside the safety watchdog:

   | Construct | Introduced | Effect on 2.7 |
   |---|---|---|
   | `threading.Thread(daemon=True)` | 3.3 | TypeError at startup |
   | `json.JSONDecodeError` | 3.5 | AttributeError on first bad packet |
   | `time.monotonic()` | 3.3 | AttributeError in the watchdog |
   | `math.isfinite()` | 3.2 | AttributeError on first valid command |
   | `except OSError` for sockets | — | 2.7's `socket.error` subclasses `IOError`, so errors escaped |

   Guard: `test/test_tx1_relay_python2.py` (parses with `ast`, so the prose in
   these very comments does not trip it).

The relay secret is still the default `change-me-r680`. Change it on both sides
before this leaves a closed lab network.
