#!/usr/bin/env python3
"""Summarise (and optionally plot) a flown track.

Offline analysis tooling only; nothing in the flight/estimation path is Python.
Input: CSV from `ros2 topic echo --csv /fmu/out/vehicle_local_position_v1`
(one row per message, fields in px4_msgs/VehicleLocalPosition order, no header).
"""
import argparse
import csv
import sys
import warnings

# Column indices in VehicleLocalPosition (px4_msgs release/1.17):
# timestamp, timestamp_sample, xy_valid, z_valid, v_xy_valid, v_z_valid, x, y, z, ...
T, X, Y, Z = 0, 6, 7, 8


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv")
    ap.add_argument("--png", help="write a top-down + altitude plot here (needs matplotlib)")
    args = ap.parse_args()

    t, x, y, alt = [], [], [], []
    dropped = 0      # samples with an absurd timestamp
    clock_steps = 0  # small backward steps from timesync re-convergence
    with open(args.csv, newline="") as f:
        for row in csv.reader(f):
            if len(row) <= Z:
                continue
            try:
                ts = int(row[T]) * 1e-6
                sample = (float(row[X]), float(row[Y]), -float(row[Z]))  # NED z -> altitude
            except ValueError:
                continue
            # When PX4's uXRCE-DDS timesync re-converges, one sample can carry raw
            # boot time (absurd jump: dropped) and the clock may then step back by
            # a fraction of a second (valid data: kept, counted).
            if t and abs(ts - t[-1]) > 5.0:
                dropped += 1
                continue
            if t and ts < t[-1]:
                clock_steps += 1
            t.append(ts)
            x.append(sample[0])
            y.append(sample[1])
            alt.append(sample[2])
    if not t:
        print(f"no samples in {args.csv}", file=sys.stderr)
        return 1

    t0 = t[0]
    print(f"samples:   {len(t)} over {t[-1] - t0:.1f} s "
          f"({dropped} dropped for bad timestamp, {clock_steps} backward clock step(s))")
    print(f"north [m]: {min(x):6.2f} .. {max(x):6.2f}  (span {max(x) - min(x):.2f})")
    print(f"east  [m]: {min(y):6.2f} .. {max(y):6.2f}  (span {max(y) - min(y):.2f})")
    print(f"alt   [m]: {min(alt):6.2f} .. {max(alt):6.2f}")
    print(f"final:     N {x[-1]:.2f}  E {y[-1]:.2f}  alt {alt[-1]:.2f}")

    if args.png:
        try:
            # The apt mpl_toolkits next to a pip matplotlib triggers a harmless
            # "Unable to import Axes3D" warning; no 3D plots are used here.
            warnings.filterwarnings("ignore", message="Unable to import Axes3D")
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:  # noqa: BLE001 - ImportError or a NumPy ABI mismatch
            # PX4's ubuntu.sh pip-installs NumPy 2 into ~/.local, which breaks
            # Ubuntu 22.04's apt matplotlib (built against NumPy 1).
            print(f"matplotlib unusable ({type(exc).__name__}), skipping plot. Fix with:\n"
                  '  pip3 install --user "matplotlib>=3.9"', file=sys.stderr)
            return 0
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        ax1.plot(y, x)
        ax1.set_xlabel("east [m]")
        ax1.set_ylabel("north [m]")
        ax1.set_title("track (top-down)")
        ax1.set_aspect("equal", adjustable="datalim")
        ax1.grid(True)
        ax2.plot([ti - t0 for ti in t], alt)
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("altitude [m]")
        ax2.set_title("altitude")
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig(args.png, dpi=120)
        print(f"plot:      {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
