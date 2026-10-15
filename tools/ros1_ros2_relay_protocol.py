"""Small JSON/UDP protocol shared by the ROS 1 and ROS 2 relay nodes."""

import json
import socket

MAX_PACKET = 60000


def send_packet(sock, address, kind, payload):
    data = json.dumps({"kind": kind, "payload": payload}, separators=(",", ":")).encode()
    if len(data) > MAX_PACKET:
        raise ValueError(f"relay packet too large: {len(data)} bytes")
    sock.sendto(data, address)


def recv_packet(sock):
    data, address = sock.recvfrom(MAX_PACKET)
    return json.loads(data.decode()), address


def open_socket(bind_host, bind_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_host, bind_port))
    return sock


def stamp_dict(stamp):
    return {"sec": int(stamp.secs), "nanosec": int(stamp.nsecs)}


def stamp_from_ros2(stamp):
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}
