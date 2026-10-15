#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Sample /PowerVoltage forever, appending to a CSV. Runs on TX1 (Python 2).

Why: the pack is labelled "12V 9800mAh", which does not by itself say what
"full" or "empty" means. The candidates disagree by more than 2 V:

    3S Li-ion        full 12.6 V   nominal 11.1 V   empty ~9.0 V
    4S LiFePO4       full 14.6 V   nominal 12.8 V   empty ~10.0 V
    12 V lead-acid   full 12.7 V   nominal 12.0 V   empty ~11.0 V

`min_battery` cannot be set sensibly until that is known, and getting it wrong
is the failure this project keeps hitting: a guard that reports "fine" while the
pack is already too flat to move the robot. Watching one charge cycle settle
identifies the chemistry outright -- if the pack tops out near 12.6 V it is
3S Li-ion, and anything climbing past 13 V is not.

This separates two things that the single number /PowerVoltage conflates:
  * the RESTING voltage, which decays slowly with state of charge, and
  * the LOADED voltage, which sags -- badly on an aged pack.

The robot reportedly "ran out" while the resting reading was still ~11.9 V. For
3S Li-ion that is roughly 60 % charged, so either the pack sags hard under motor
load or something else stopped it. Only a loaded sample can tell those apart,
which is why this logs the whole session rather than one reading.
"""
import csv
import os
import sys
import time

import rospy
from std_msgs.msg import Float32

OUT = "/home/wheeltec/power_log.csv"
PERIOD = 20.0

state = {"v": None, "t": None, "n": 0}


def cb(msg):
    state["v"] = float(msg.data)
    state["t"] = time.time()
    state["n"] += 1


def main():
    rospy.init_node("power_logger", anonymous=True)
    rospy.Subscriber("/PowerVoltage", Float32, cb, queue_size=1)

    new = not os.path.exists(OUT)
    fh = open(OUT, "a")
    w = csv.writer(fh)
    if new:
        w.writerow(["wall_time", "voltage", "samples_seen", "age_s", "note"])
        fh.flush()

    rospy.loginfo("power_logger -> %s every %.0f s", OUT, PERIOD)
    # stderr too: rospy.loginfo alone lands only in ~/.ros/log, which is not
    # where anyone looks during a bring-up.
    sys.stderr.write("power_logger running, appending to %s\n" % OUT)
    sys.stderr.flush()

    while not rospy.is_shutdown():
        now = time.time()
        if state["v"] is None:
            note = "no sample yet"
            row = [now, "", state["n"], "", note]
        else:
            age = now - state["t"]
            note = "ok" if age < 5.0 else "STALE"
            # An empty or stale reading must never be recorded as a voltage:
            # that is how "the sensor died" becomes "the battery is fine".
            row = [now, "%.3f" % state["v"], state["n"], "%.1f" % age, note]
        w.writerow(row)
        fh.flush()
        sys.stderr.write("  %.3f V (%s)\n" % (state["v"], note)
                         if state["v"] is not None else "  no sample\n")
        sys.stderr.flush()
        time.sleep(PERIOD)


if __name__ == "__main__":
    main()
