#!/usr/bin/env python2
"""Set the TX1 clock from the Jetson over the direct R680 wired link.

The TX1 RTC holds a wrong date (year 2000) and is not writable, so after every
reboot the clock returns to a stale value and TX1 has no internet to fix it
with NTP. This client asks time_sync_server.py on the Jetson for the time and
steps the local clock when the offset is worth correcting.

The shebang says python2 even though the body is 2/3 compatible, for two
reasons: this file is deployed to TX1 and the sibling TX1 script
tools/ros1_relay.py uses python2, so the interpreter choice should be
consistent. TX1 *does* have python3 and currently runs this script with it (see
below), so the shebang is about consistency and about not breaking if the script
is ever invoked directly on a host where python3 is absent.

How TX1 actually runs it: as ``r680-time-sync.service`` (systemd, Restart=always,
running as root so it can call clock_settime), with

    /usr/bin/python3 /home/wheeltec/time_sync_client.py \
        --server 192.168.2.15 --port 10123 --interval 10 --threshold 0.005

Note the --threshold of 5 ms -- much tighter than this script's own 50 ms
default. TX1's oscillator drifts ~23 ppm, so the clock ramps to ~5 ms and is
stepped back roughly every 219 s: expect a sawtooth, not a constant offset.

Runs unchanged on Python 2.7 (ROS Melodic / TX1) and Python 3, so avoid
time.monotonic_ns(), f-strings, print(..., flush=True) and other newer-only APIs.
"""
from __future__ import print_function

import argparse
import ctypes
import os
import socket
import struct
import sys
import time

MAGIC = b"R680TS01"
HEADER = struct.Struct(">Q")      # seq
STAMPS = struct.Struct(">QQ")     # t1, t2
REPLY_SIZE = len(MAGIC) + HEADER.size + STAMPS.size

CLOCK_REALTIME = 0


class Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


try:
    _libc = ctypes.CDLL("libc.so.6", use_errno=True)
except OSError:                    # pragma: no cover - non-glibc fallback
    _libc = None


def now_ns():
    """Nanoseconds since the epoch; works on Python 2.7 and 3.6."""
    return int(time.time() * 1e9)


def set_clock_ns(target_ns):
    """Step CLOCK_REALTIME. Requires root."""
    if _libc is None:
        raise RuntimeError("libc unavailable")
    ts = Timespec(int(target_ns // 1000000000), int(target_ns % 1000000000))
    if _libc.clock_settime(CLOCK_REALTIME, ctypes.byref(ts)) != 0:
        errno = ctypes.get_errno()
        raise OSError(errno, os.strerror(errno))


def describe(offset_ns):
    sign = "-" if offset_ns < 0 else "+"
    total = abs(offset_ns) / 1e9
    if total >= 86400.0:
        return "%s%.3f s (%s%.2f days)" % (sign, total, sign, total / 86400.0)
    return "%s%.6f s" % (sign, total)


def say(message):
    """Print and flush, on Python 2.7 *and* 3.

    Do not "simplify" this back to ``print(message, flush=True)``.
    ``flush`` as a keyword argument to ``print`` is Python 3.3+, and on TX1's
    Python 2.7 it raises::

        TypeError: 'flush' is an invalid keyword argument for this function

    Even with ``from __future__ import print_function`` the keyword is not
    accepted there.  This is a latent fragility, not a live outage: on TX1 the
    script is started by ``r680-time-sync.service`` with **/usr/bin/python3**
    (``--interval 10 --threshold 0.005``, Restart=always, as root), and python3
    accepts ``flush=`` happily -- so the sync has been working. The 2026-10-14
    journal shows it stepping the clock by +98787286 s (+1143 days) to recover
    from TX1's dead RTC. The fix matters because any invocation that reaches
    Python 2.7 (an explicit ``python2`` call, or the shebang after it is fixed)
    would die on the first print.

    Measured offset: **+2.5 ms** (6 consecutive protocol measurements in a single
    session agreed to within 46 us, RTT ~0.5 ms). An earlier estimate of ~240 ms
    came from bracketing ``date`` across separate SSH sessions, which folds SSH
    connect/auth/interpreter startup into the apparent offset; that number was
    wrong and is recorded here only so nobody re-derives it.

    Expect a sawtooth, not a constant: TX1's oscillator drifts ~23 ppm and the
    service only steps when the offset passes ``--threshold`` (5 ms on TX1), so
    the clock ramps to ~5 ms and is jerked back roughly every 219 s. That is why
    the same offset measured at different moments gives +2.5 / +4.4 / +5.1 ms --
    it is the sawtooth, not only measurement noise.
    """
    print(message)
    sys.stdout.flush()


def measure(sock, server, seq, timeout):
    """One request/response; returns (offset_ns, rtt_ns, server_processing_ns)."""
    sock.settimeout(timeout)
    t0 = now_ns()
    sock.sendto(MAGIC + HEADER.pack(seq), server)

    deadline = time.time() + timeout
    while time.time() < deadline:
        data, _peer = sock.recvfrom(1024)
        t3 = now_ns()
        if len(data) != REPLY_SIZE or not data.startswith(MAGIC):
            continue
        (rseq,) = HEADER.unpack_from(data, len(MAGIC))
        if rseq != seq:
            continue
        t1, t2 = STAMPS.unpack_from(data, len(MAGIC) + HEADER.size)
        offset = ((t1 - t0) + (t2 - t3)) / 2.0
        return offset, (t3 - t0), (t2 - t1)
    raise socket.timeout("no usable reply within %.3fs" % timeout)


def best_measure(sock, server, seq, timeout, samples):
    """Take the sample with the lowest round trip, which is the least noisy."""
    results = []
    for index in range(samples):
        try:
            results.append(measure(sock, server, seq + index, timeout))
        except socket.timeout as exc:
            say("time sync: %s" % exc)
    if not results:
        return None
    return min(results, key=lambda item: item[1])


def main():
    parser = argparse.ArgumentParser(description="R680 TX1 clock sync client")
    parser.add_argument("--server", default="192.168.2.15", help="Jetson address")
    parser.add_argument("--port", type=int, default=10123, help="UDP port")
    parser.add_argument("--interval", type=float, default=60.0,
                        help="seconds between corrections")
    parser.add_argument("--threshold", type=float, default=0.05,
                        help="only step the clock when the offset exceeds this")
    parser.add_argument("--timeout", type=float, default=1.0, help="per-request timeout")
    parser.add_argument("--samples", type=int, default=3, help="requests per correction")
    parser.add_argument("--once", action="store_true", help="measure once then exit")
    parser.add_argument("--dry-run", action="store_true",
                        help="report offsets but never change the clock")
    args = parser.parse_args()

    server = (args.server, args.port)
    threshold_ns = int(args.threshold * 1e9)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = int(time.time() * 1000) & 0xFFFFFFFF
    say("r680 time sync: server=%s:%d interval=%.1fs threshold=%s dry_run=%s"
        % (server[0], server[1], args.interval, args.threshold, args.dry_run))

    failures = 0
    while True:
        seq = (seq + args.samples + 1) & 0xFFFFFFFF
        result = best_measure(sock, server, seq, args.timeout, args.samples)

        if result is None:
            failures += 1
            if failures == 1 or failures % 10 == 0:
                say("time sync: no reply from %s:%d (attempt %d)"
                    % (server[0], server[1], failures))
        else:
            failures = 0
            offset, rtt, processing = result
            if abs(offset) <= threshold_ns:
                say("time sync: within threshold (%s, rtt %.3f ms)"
                    % (describe(offset), rtt / 1e6))
            elif args.dry_run:
                say("time sync: DRY RUN would step clock by %s (rtt %.3f ms)"
                    % (describe(offset), rtt / 1e6))
            else:
                target = now_ns() + offset
                try:
                    set_clock_ns(target)
                    say("time sync: stepped clock by %s (rtt %.3f ms, server processing %.3f ms)"
                        % (describe(offset), rtt / 1e6, processing / 1e6))
                except OSError as exc:
                    say("time sync: cannot set clock (%s); run as root" % exc)
                    if args.once:
                        return 1

        if args.once:
            return 0
        time.sleep(args.interval)


if __name__ == "__main__":
    sys.exit(main() or 0)
