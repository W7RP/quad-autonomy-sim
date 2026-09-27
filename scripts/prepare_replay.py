#!/usr/bin/env python3
"""Normalise a flight's raw PX4 sensor CSVs into the simple, headed CSVs that the
C++ replay tool (quad_estimation/eskf_replay) reads.

  <logdir>/{imu,flow,range,mag}.csv   (ros2 topic echo --csv, PX4 layouts)
    -> <logdir>/replay/{imu,flow,range,mag}.csv

Keeping PX4's message layouts out of the C++ tool means only one place
(scripts/lib/px4_csv.py, which asks ROS for the layout) knows them.
Range validity is filtered here exactly as eskf_node does it live.
Offline tooling only; needs a sourced ROS 2 environment.
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
import px4_csv  # noqa: E402

ROTATION_DOWNWARD_FACING = 25


def write(path, header, cols):
    with open(path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(header)
        for row in zip(*cols):
            w.writerow([int(v) if float(v).is_integer() and abs(v) > 1e3 else f"{v:.9g}" for v in row])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    d = ap.parse_args().logdir
    out = d / "replay"
    out.mkdir(exist_ok=True)

    imu = px4_csv.read(d / "imu.csv", "px4_msgs/msg/SensorCombined")
    write(out / "imu.csv", ["t_pub_us", "t_us", "dt_us", "gx", "gy", "gz", "ax", "ay", "az"],
          [imu["timestamp"], imu["timestamp"], imu["gyro_integral_dt"],
           *imu["gyro_rad"].T, *imu["accelerometer_m_s2"].T])

    fl = px4_csv.read(d / "flow.csv", "px4_msgs/msg/SensorOpticalFlow")
    write(out / "flow.csv", ["t_pub_us", "t_us", "window_us", "fx", "fy", "quality"],
          [fl["timestamp"], fl["timestamp_sample"], fl["integration_timespan_us"],
           *fl["pixel_flow"].T, fl["quality"]])

    rg = px4_csv.read(d / "range.csv", "px4_msgs/msg/DistanceSensor")
    ok = ((rg["orientation"] == ROTATION_DOWNWARD_FACING)
          & (rg["current_distance"] >= rg["min_distance"])
          & (rg["current_distance"] <= rg["max_distance"])
          & (rg["signal_quality"] != 0))
    write(out / "range.csv", ["t_pub_us", "t_us", "range"],
          [rg["timestamp"][ok], rg["timestamp"][ok], rg["current_distance"][ok]])

    mg = px4_csv.read(d / "mag.csv", "px4_msgs/msg/VehicleMagnetometer")
    write(out / "mag.csv", ["t_pub_us", "t_us", "mx", "my", "mz"],
          [mg["timestamp"], mg["timestamp_sample"], *mg["magnetometer_ga"].T])

    print(f"replay inputs in {out}: imu {len(imu['timestamp'])}, flow {len(fl['timestamp'])}, "
          f"range {int(ok.sum())}/{len(ok)}, mag {len(mg['timestamp'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
