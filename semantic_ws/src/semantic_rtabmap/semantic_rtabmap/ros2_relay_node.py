#!/usr/bin/env python3
"""ROS 2 side of the UDP relay fallback for TX1 ROS 1."""
import hashlib
import hmac
import json
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

# /tf_static 必须是 TRANSIENT_LOCAL（latched），/tf 必须保持 VOLATILE —— 两者不能混。
#
# 静态变换按定义只在启动时发布一次，靠 TRANSIENT_LOCAL 让晚加入的节点仍能拿到。
# 这里原先是 create_publisher(..., 10)，默认 durability 是 VOLATILE，于是：
#   * tf2_ros 的监听者（tf2_echo / RTAB-Map）和本包的 safety_monitor_node 都按
#     TRANSIENT_LOCAL 订阅，QoS 不匹配 → 连接被拒，实测 tf2_echo 报
#       "New publisher discovered on topic '/tf_static', offering incompatible
#        QoS ... Last incompatible policy: DURABILITY_QOS_POLICY"
#     一个静态变换都收不到；
#   * 而“先启动 RTAB-Map、后启动 relay”这种顺序下，即使 QoS 对了，VOLATILE
#     也保证不了补发。
# 回归防护见 test/test_relay_tf_static_qos.py，改这里之前先跑它。
TF_STATIC_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# TX1 的驱动把 /odom 里 z/roll/pitch 的方差填成 1e6，即 ROS 表示"该自由度未估计"
# 的哨兵值。对平面轮式底盘这是**语义正确**的——轮式编码器确实观测不到这三个自由度，
# 而 relay 除此之外是逐字节忠实转发的。
#
# 但 RTAB-Map 不接受它，会打印：
#   Memory.cpp:1168::addSignatureToStm() Very large angular variance
#   (1000000.000000) detected! Please fix odometry covariance, otherwise poor
#   graph optimizations are expected and wrong loop closure detections ...
# 因为它会读角方差来给位姿图加权。
#
# 本车只在平地行驶（操作者 2026-10-14 确认），所以"底盘保持水平"这个断言成立；
# 把它说出来，RTAB-Map 就能利用这个约束，而不是把位姿当作完全未知。
# 实测 TX1 的 /odom 对角项：x=1e-09, y=0.001, z=1e6, roll=1e6, pitch=1e6, yaw=1e-09
# ——注意 1e6 精确落在那三个不可观测自由度上，而 yaw 本来就是 1e-9。
#
# 做成开关且**节点默认关闭**（= 逐字节忠实转发）：relay 同时承载安全通路，
# 不能悄悄改写 TX1 的数据。由 config/r680_relay.yaml 打开，也就是跑建图的那条路径。
PLANAR_COVARIANCE_INDICES = (14, 21, 28)  # 6x6 行优先里的 z / roll / pitch
PLANAR_COVARIANCE_VALUE = 1e-3            # 标准差约 3.2 cm / 1.8 度
UNKNOWN_COVARIANCE_MIN = 1e4              # >= 此值视为 1e6 哨兵


def apply_planar_covariance(covariance):
    """把 z/roll/pitch 上表示"未估计"的哨兵值换成有限值。

    只替换哨兵。若 TX1 的驱动将来开始上报这三个自由度的真实值，这里会变成空操作，
    而不是把它们悄悄覆盖掉——所以这个开关一直开着也是安全的。
    """
    out = list(covariance)
    while len(out) < 36:
        out.append(0.0)
    for i in PLANAR_COVARIANCE_INDICES:
        if out[i] >= UNKNOWN_COVARIANCE_MIN:
            out[i] = PLANAR_COVARIANCE_VALUE
    return out


class Relay(Node):
    def __init__(self):
        super().__init__("r680_ros2_relay")
        # 默认值与 config/r680_relay.yaml 保持一致（有线网段 192.168.2.x）。
        # 曾经这里是 192.168.1.102（WiFi 网段）—— 不带 yaml 直接跑就会朝错的对端发。
        self.declare_parameter("peer_host", "192.168.2.102")
        # 端口语义警告：本节点 rx_port = **本机绑定**口，tx_port = **发往对端**的口。
        # TX1 侧 ros1_relay.py 恰好相反（tx_port 是它的绑定口）。
        # 两侧必须互补，详见 config/r680_relay.yaml 的注释。
        self.declare_parameter("rx_port", 19001)
        self.declare_parameter("tx_port", 19000)
        self.declare_parameter("secret", "change-me-r680")
        self.declare_parameter("input_cmd_vel", "/cmd_vel_safe")
        self.declare_parameter("output_odom", "/odom")
        self.declare_parameter("output_imu", "/imu")
        self.declare_parameter("output_battery", "/battery_voltage")
        self.declare_parameter("filter_tf_child", "camera_link")
        self.declare_parameter("drop_tf_parent", "base_footprint")
        # 默认 False = 原样转发 TX1 的协方差（见文件上方 PLANAR_COVARIANCE_* 的说明）。
        self.declare_parameter("planar_odom_covariance", False)
        self.planar_covariance = bool(self.get_parameter("planar_odom_covariance").value)
        self.peer = (str(self.get_parameter("peer_host").value), int(self.get_parameter("tx_port").value))
        self.secret = str(self.get_parameter("secret").value)
        self.cmd_seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR：relay 崩溃/重启后能立刻重新绑定同一端口，不必等内核回收。
        # 与 tools/ros1_ros2_relay_protocol.open_socket 的做法保持一致（TX1 侧一直有）。
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", int(self.get_parameter("rx_port").value)))
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.sock.settimeout(0.2)
        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("output_odom").value), 10)
        self.imu_pub = self.create_publisher(Imu, str(self.get_parameter("output_imu").value), 10)
        # Battery voltage, forwarded from TX1's /PowerVoltage (std_msgs/Float32).
        # Empty string disables the publisher, so a relay that has no battery
        # source simply does not advertise the topic -- better than advertising it
        # and never publishing, which looks identical to a working topic whose
        # publisher died.
        self.battery_pub = None
        battery_topic = str(self.get_parameter("output_battery").value)
        if battery_topic:
            self.battery_pub = self.create_publisher(Float32, battery_topic, 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", TF_STATIC_QOS)
        self.cmd_sub = self.create_subscription(Twist, str(self.get_parameter("input_cmd_vel").value), self.send_cmd, 10)
        self.thread = threading.Thread(target=self.receive, daemon=True); self.thread.start()

    def sign(self, raw): return hmac.new(self.secret.encode(), raw, hashlib.sha256).hexdigest()
    @staticmethod
    def canonical(obj):
        """Serialise a relay body into the exact bytes that get signed.

        `sort_keys=True` is load-bearing, not cosmetic. The two ends run
        different Python versions (TX1: 2.7 on Melodic, Orin: 3.10 on Humble) and
        their dict key orders differ -- Python 3 preserves insertion order,
        Python 2.7 is hash-ordered. Signing a body serialised in each side's own
        order yields different bytes for identical data, so the HMAC never
        matches at the far end.

        That asymmetry is why this hid for so long: the uplink matched anyway,
        because the receiver re-serialised a dict whose key order had come from
        the JSON text. Only cmd_vel -- built fresh as a Python 3 dict and then
        verified by Python 2 -- was rejected, as a throttled "invalid relay
        packet" warning that read like noise. Mapping and odometry were perfect
        the entire time; the robot never received one single velocity command.

        This MUST stay byte-identical to _canonical() in tools/ros1_relay.py.
        Changing one side alone breaks the direction that currently works.
        """
        return json.dumps(obj, separators=(",", ":"), sort_keys=True).encode()

    @staticmethod
    def stamp(s):
        from builtin_interfaces.msg import Time
        t = Time(); t.sec = int(s.get("sec", 0)); t.nanosec = int(s.get("nanosec", 0)); return t
    @staticmethod
    def vec(d, target): target.x=float(d.get("x",0)); target.y=float(d.get("y",0)); target.z=float(d.get("z",0))
    @staticmethod
    def quat(d, target): target.x=float(d.get("x",0)); target.y=float(d.get("y",0)); target.z=float(d.get("z",0)); target.w=float(d.get("w",1))
    def header(self, d, h): h.stamp=self.stamp(d.get("stamp", {})); h.frame_id=d.get("frame_id", "")
    def receive(self):
        while rclpy.ok():
            try: packet, _ = self.sock.recvfrom(60000)
            except socket.timeout: continue
            except OSError: return
            try:
                outer=json.loads(packet.decode()); body=outer["body"]; raw=self.canonical(body)
                # Encode BOTH operands. Encoding only one is worse than encoding
                # neither, and the type rules are what make that true:
                #
                #   compare_digest(str,   str)    py2 OK   py3 OK
                #   compare_digest(bytes, str)    py2 OK   py3 TypeError
                #   compare_digest(bytes, bytes)  py2 OK   py3 OK
                #
                # (bytes, bytes) is the only combination valid on BOTH
                # interpreters, so both sides encode both arguments. Encoding just
                # the received signature broke this side immediately.
                #
                # The original bug was on TX1: Python 2's compare_digest raises
                # "'unicode' does not have the buffer interface" for the unicode
                # that json.loads always returns there, so every downlink cmd_vel
                # was discarded and reported as a generic "invalid relay packet".
                if not hmac.compare_digest(outer["sig"].encode("ascii"),
                                           self.sign(raw).encode("ascii")): continue
                kind, p=body["kind"], body["payload"]
                if kind == "odom": self.publish_odom(p)
                elif kind == "imu": self.publish_imu(p)
                elif kind == "battery": self.publish_battery(p)
                elif kind in ("tf", "tf_static"): self.publish_tf(p, kind == "tf_static")
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                self.get_logger().warning("invalid relay packet")
    def publish_battery(self,p):
        # std_msgs/Float32 has no header/stamp, so there is nothing to age-check
        # on the wire. The receiver timestamps on arrival; the exploration node's
        # battery guard treats a stale reading as unknown rather than as OK.
        if self.battery_pub is None: return
        m=Float32(); m.data=float(p.get("voltage",0.0)); self.battery_pub.publish(m)
    def publish_odom(self,p):
        m=Odometry(); self.header(p["header"],m.header); m.child_frame_id=p.get("child_frame_id","")
        self.vec(p["pose"]["position"],m.pose.pose.position); self.quat(p["pose"]["orientation"],m.pose.pose.orientation)
        self.vec(p["twist"]["linear"],m.twist.twist.linear); self.vec(p["twist"]["angular"],m.twist.twist.angular)
        pose_cov=p["pose"].get("covariance",[0.0]*36); twist_cov=p["twist"].get("covariance",[0.0]*36)
        if self.planar_covariance:
            pose_cov=apply_planar_covariance(pose_cov); twist_cov=apply_planar_covariance(twist_cov)
        m.pose.covariance=pose_cov; m.twist.covariance=twist_cov; self.odom_pub.publish(m)
    def publish_imu(self,p):
        m=Imu(); self.header(p["header"],m.header); self.quat(p["orientation"],m.orientation); self.vec(p["angular_velocity"],m.angular_velocity); self.vec(p["linear_acceleration"],m.linear_acceleration)
        m.orientation_covariance=p.get("orientation_covariance",[-1.0]+[0.0]*8); m.angular_velocity_covariance=p.get("angular_velocity_covariance",[0.0]*9); m.linear_acceleration_covariance=p.get("linear_acceleration_covariance",[0.0]*9); self.imu_pub.publish(m)
    def publish_tf(self,items,is_static):
        out=[]; filtered=str(self.get_parameter("filter_tf_child").value)
        dropped_parent=str(self.get_parameter("drop_tf_parent").value)
        for d in items:
            if d.get("child_frame_id")==filtered and d.get("header",{}).get("frame_id")==dropped_parent: continue
            t=TransformStamped(); self.header(d["header"],t.header); t.child_frame_id=d.get("child_frame_id",""); self.vec(d["translation"],t.transform.translation); self.quat(d["rotation"],t.transform.rotation); out.append(t)
        if out: (self.tf_static_pub if is_static else self.tf_pub).publish(TFMessage(transforms=out))
    def send_cmd(self,msg):
        p={"linear":{"x":msg.linear.x,"y":msg.linear.y,"z":msg.linear.z},"angular":{"x":msg.angular.x,"y":msg.angular.y,"z":msg.angular.z}}
        self.cmd_seq += 1
        body={"kind":"cmd_vel","seq":self.cmd_seq,"payload":p,"sent_at":int(time.time()*1000000000)}; raw=self.canonical(body); packet=json.dumps({"body":body,"sig":self.sign(raw)},separators=(",", ":"),sort_keys=True).encode()
        try: self.sock.sendto(packet,self.peer)
        except OSError as e: self.get_logger().warning("relay send failed: %s" % e)
    def destroy_node(self):
        self.sock.close(); super().destroy_node()


def main(args=None):
    rclpy.init(args=args); n=Relay()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == "__main__": main()
