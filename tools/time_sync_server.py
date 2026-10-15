#!/usr/bin/env python3
"""Minimal UDP time responder for the R680 TX1 <-> Jetson wired link.

The TX1 has a dead RTC and no internet access, so it has no trustworthy time
source of its own. This responder runs on the Jetson (the ROS 2 host) and
answers timestamp requests from the TX1 time_sync_client.py.

It deliberately avoids the privileged NTP port 123 so it can run as an
unprivileged user, and it does not implement NTP: only the four timestamps
needed for a single-shot offset estimate.

Wire format (all integers big endian):

  request   MAGIC | seq:u64
  response  MAGIC | seq:u64 | t1:u64 | t2:u64

  t1 = server receive time, t2 = server transmit time, both in nanoseconds
  since the Unix epoch on the server clock.
"""
import argparse
import socket
import struct
import time

MAGIC = b"R680TS01"
HEADER = struct.Struct(">Q")       # seq
STAMPS = struct.Struct(">QQ")      # t1, t2


def now_ns():
    """Nanoseconds since the epoch, without requiring Python 3.7 time_ns()."""
    return int(time.time() * 1e9)


def main():
    parser = argparse.ArgumentParser(description="R680 UDP time responder")
    parser.add_argument("--bind", default="0.0.0.0", help="address to bind")
    parser.add_argument("--port", type=int, default=10123, help="UDP port")
    parser.add_argument("--verbose", action="store_true", help="log each reply")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    print("r680 time responder listening on %s:%d" % (args.bind, args.port), flush=True)

    replies = 0
    while True:
        try:
            data, peer = sock.recvfrom(1024)
        except OSError as exc:                       # keep serving after a bad datagram
            print("recv failed: %s" % exc, flush=True)
            continue

        t1 = now_ns()
        if len(data) != len(MAGIC) + HEADER.size or not data.startswith(MAGIC):
            continue                                  # ignore anything that is not ours
        try:
            (seq,) = HEADER.unpack_from(data, len(MAGIC))
        except struct.error:
            continue

        t2 = now_ns()
        try:
            sock.sendto(MAGIC + HEADER.pack(seq) + STAMPS.pack(t1, t2), peer)
        except OSError as exc:
            print("send to %s failed: %s" % (peer, exc), flush=True)
            continue

        replies += 1
        if args.verbose:
            print("reply %d to %s:%d seq=%d" % (replies, peer[0], peer[1], seq), flush=True)


if __name__ == "__main__":
    main()
