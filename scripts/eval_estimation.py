#!/usr/bin/env python3
"""Evaluate the Phase 2 ESKF (and PX4's EKF2 as a baseline) against Gazebo ground truth.

Offline analysis tooling only. Inputs are CSVs recorded by scripts/demo_phase2.sh
with `ros2 topic echo --csv` (one row per message, arrays flattened, no header):

  est.csv       /eskf/odometry_ned                       px4_msgs/VehicleOdometry
  gt_pos.csv    /fmu/out/vehicle_local_position_groundtruth_v1   VehicleLocalPosition
  gt_att.csv    /fmu/out/vehicle_attitude_groundtruth             VehicleAttitude
  ekf2_pos.csv  /fmu/out/vehicle_local_position_v1                VehicleLocalPosition
  ekf2_att.csv  /fmu/out/vehicle_attitude                         VehicleAttitude

All share PX4's timesync'd time base. Positions are compared as displacement
from the moment the ESKF initialised (its origin), which is also how a flow-only
estimator is used in practice: it has no absolute horizontal reference.

Outputs metrics.json and estimation.png in the log directory, prints a summary,
and with --check exits non-zero if the ESKF misses the acceptance thresholds.
"""
import argparse
import csv
import json
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
import px4_csv  # noqa: E402  (column layouts come from `ros2 interface show`)

# Acceptance thresholds for --check (ESKF only). Deliberately explicit: see
# docs/phase2_state_estimation.md for how they were chosen.
THRESHOLDS = {
    "vel_rmse_mps": 0.35,
    "tilt_rmse_deg": 2.0,
    "yaw_rmse_deg": 5.0,
    "alt_rmse_m": 0.25,
    "horiz_drift_pct_of_path": 10.0,
}


def _select(d, keep):
    return {k: v[keep] for k, v in d.items()}


# Clock-step reference (set in main): timesync steps measured on the IMU stream.
REF_STEPS = None


def fix_time(t):
    """Undo timesync glitches and steps (see px4_csv.unwrap_clock)."""
    t_fixed, keep = px4_csv.unwrap_clock(t, REF_STEPS)
    return t_fixed, keep


def load_px4(path, msg_type):
    """PX4 CSV -> (t [s], {field: array}, dropped). Uses timestamp_sample."""
    d = px4_csv.read(path, msg_type)
    if len(d["__rows__"]) == 0:
        raise SystemExit(f"no rows in {path}")
    t, keep = fix_time(d["timestamp_sample"] * 1e-6)
    return t[keep], _select(d, keep), int((~keep).sum())


def load_estimate(path):
    """The ESKF estimate: either the live node's /eskf/odometry_ned CSV, or the
    replay tool's headed CSV. Returns (t, pos, q(wxyz), vel, pos_var, vel_var, att_var, dropped)."""
    with open(path, newline="") as f:
        first = f.readline()
    if first.startswith("t_us"):
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        col = lambda *names: np.array([[float(r[n]) for n in names] for r in rows])  # noqa: E731
        t, keep = fix_time(col("t_us")[:, 0] * 1e-6)
        out = [col("px", "py", "pz"), col("qw", "qx", "qy", "qz"), col("vx", "vy", "vz"),
               col("var_px", "var_py", "var_pz"), col("var_vx", "var_vy", "var_vz"),
               col("var_ax", "var_ay", "var_az")]
        return (t[keep], *[a[keep] for a in out], int((~keep).sum()))
    t, d, dropped = load_px4(path, "px4_msgs/msg/VehicleOdometry")
    return (t, d["position"], d["q"], d["velocity"], d["position_variance"],
            d["velocity_variance"], d["orientation_variance"], dropped)


def quat_to_rot(q):
    """(N,4) w,x,y,z -> (N,3,3) body->world."""
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def rot_error_world(R_est, R_true):
    """World-frame small-angle rotation error: R_true = Exp(e) R_est."""
    E = R_true @ np.transpose(R_est, (0, 2, 1))
    return 0.5 * np.stack([E[:, 2, 1] - E[:, 1, 2], E[:, 0, 2] - E[:, 2, 0], E[:, 1, 0] - E[:, 0, 1]], -1)


def yaw_of(R):
    return np.arctan2(R[:, 1, 0], R[:, 0, 0])


def interp(t_src, v_src, t_dst):
    # np.interp silently clamps outside [t_src[0], t_src[-1]]: that once turned a
    # corrupted time axis into a fake "5 mm" EKF2 error. Refuse instead.
    if len(t_dst) and (t_dst.min() < t_src[0] - 0.05 or t_dst.max() > t_src[-1] + 0.05):
        raise SystemExit(f"time axis mismatch: [{t_dst.min():.3f}, {t_dst.max():.3f}] outside "
                         f"[{t_src[0]:.3f}, {t_src[-1]:.3f}]")
    return np.stack([np.interp(t_dst, t_src, v_src[:, i]) for i in range(v_src.shape[1])], -1)


def interp_quat(t_src, q_src, t_dst):
    """Component-wise interpolation + renormalisation (fine at 50 Hz), sign-continuous.
    Goes through interp(), so it gets the same range check."""
    q = q_src.copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    qi = interp(t_src, q, t_dst)
    return qi / np.linalg.norm(qi, axis=1, keepdims=True)


def rmse(x):
    return float(np.sqrt(np.mean(np.square(x))))


def metrics_for(t, pos, vel, R, gt, t0, path_len):
    """Errors of an estimator vs ground truth on its own timestamps t (t >= t0)."""
    gpos = interp(gt["t_pos"], gt["pos"], t) - interp(gt["t_pos"], gt["pos"], np.array([t0]))
    gvel = interp(gt["t_pos"], gt["vel"], t)
    gR = quat_to_rot(interp_quat(gt["t_att"], gt["q"], t))
    e_pos = pos - gpos
    e_vel = vel - gvel
    e_rot = rot_error_world(R, gR)
    e_yaw = np.degrees(np.angle(np.exp(1j * (yaw_of(gR) - yaw_of(R)))))
    horiz = np.linalg.norm(e_pos[:, :2], axis=1)
    return {
        "samples": int(len(t)),
        "horiz_pos_rmse_m": rmse(horiz),
        "horiz_pos_max_m": float(horiz.max()),
        "horiz_pos_final_m": float(horiz[-1]),
        "horiz_drift_pct_of_path": float(100.0 * horiz[-1] / max(path_len, 1e-6)),
        "alt_rmse_m": rmse(e_pos[:, 2]),
        "vel_rmse_mps": rmse(np.linalg.norm(e_vel, axis=1)),
        "tilt_rmse_deg": float(np.degrees(rmse(np.linalg.norm(e_rot[:, :2], axis=1)))),
        "yaw_rmse_deg": rmse(e_yaw),
    }, dict(e_pos=e_pos, e_vel=e_vel, e_rot=e_rot, e_yaw=e_yaw, gpos=gpos)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--est", type=Path, help="estimate CSV (default: <logdir>/est.csv)")
    ap.add_argument("--check", action="store_true", help="exit 1 if the ESKF misses THRESHOLDS")
    args = ap.parse_args()
    d = args.logdir

    global REF_STEPS
    if (d / "imu.csv").exists():
        imu = px4_csv.read(d / "imu.csv", "px4_msgs/msg/SensorCombined")
        REF_STEPS = px4_csv.reference_steps(imu["timestamp"] * 1e-6)
    else:
        REF_STEPS = px4_csv.reference_steps(
            px4_csv.read(d / "gt_pos.csv", "px4_msgs/msg/VehicleLocalPosition")["timestamp_sample"] * 1e-6)
    est_path = args.est or d / "est.csv"
    t_est, e_pos, e_q, e_vel, var_p, var_v, var_a, drop_est = load_estimate(est_path)
    t_gp, gp, drop_gp = load_px4(d / "gt_pos.csv", "px4_msgs/msg/VehicleLocalPosition")
    t_ga, ga, drop_ga = load_px4(d / "gt_att.csv", "px4_msgs/msg/VehicleAttitude")
    for name, q in (("estimate", e_q), ("ground truth", ga["q"])):
        if not np.allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-3):
            raise SystemExit(f"{name} quaternions are not unit norm: wrong columns?")

    gt = dict(t_pos=t_gp, pos=np.stack([gp["x"], gp["y"], gp["z"]], -1),
              vel=np.stack([gp["vx"], gp["vy"], gp["vz"]], -1), t_att=t_ga, q=ga["q"])
    # Evaluate where both the estimate and ground truth exist.
    lo, hi = max(t_est[0], t_gp[0], t_ga[0]), min(t_est[-1], t_gp[-1], t_ga[-1])
    m = (t_est >= lo) & (t_est <= hi)
    t_est, e_pos, e_q, e_vel = t_est[m], e_pos[m], e_q[m], e_vel[m]
    var_p, var_v, var_a = var_p[m], var_v[m], var_a[m]
    t0 = t_est[0]

    gpos_all = gt["pos"][(t_gp >= lo) & (t_gp <= hi)]
    path_len = float(np.sum(np.linalg.norm(np.diff(gpos_all[:, :2], axis=0), axis=1)))

    R_est = quat_to_rot(e_q)
    eskf, e = metrics_for(t_est, e_pos, e_vel, R_est, gt, t0, path_len)

    # Consistency: NEES with the (diagonal) published variances.
    Rt = np.transpose(R_est, (0, 2, 1))
    e_rot_body = np.einsum("nij,nj->ni", Rt, e["e_rot"])
    eskf["nees_vel_mean_3dof"] = float(np.mean(np.sum(e["e_vel"] ** 2 / var_v, axis=1)))
    eskf["nees_att_mean_3dof"] = float(np.mean(np.sum(e_rot_body ** 2 / var_a, axis=1)))
    eskf["nees_alt_mean_1dof"] = float(np.mean(e["e_pos"][:, 2] ** 2 / var_p[:, 2]))

    result = {"window_s": float(t_est[-1] - t0), "path_length_m": path_len, "eskf": eskf,
              "dropped_samples": {"est": drop_est, "gt_pos": drop_gp, "gt_att": drop_ga},
              "clock_steps_s": [round(sz, 3) for _, sz in REF_STEPS]}

    ekf2 = None
    if (d / "ekf2_pos.csv").exists() and (d / "ekf2_att.csv").exists():
        t_kp, kp, _ = load_px4(d / "ekf2_pos.csv", "px4_msgs/msg/VehicleLocalPosition")
        t_ka, ka, _ = load_px4(d / "ekf2_att.csv", "px4_msgs/msg/VehicleAttitude")
        kp_pos = np.stack([kp["x"], kp["y"], kp["z"]], -1)
        kp_vel = np.stack([kp["vx"], kp["vy"], kp["vz"]], -1)
        # EKF2's stream may start later than ours (seen: 2 s). Compare its
        # displacement from its own first sample in the window.
        t0_k = max(t0, t_kp[0], t_ka[0])
        mk = (t_kp >= t0_k) & (t_kp <= min(hi, t_ka[-1]))
        t_k = t_kp[mk]
        k_pos = kp_pos[mk] - interp(t_kp, kp_pos, np.array([t0_k]))
        k_R = quat_to_rot(interp_quat(t_ka, ka["q"], t_k))
        ekf2, e2 = metrics_for(t_k, k_pos, kp_vel[mk], k_R, gt, t0_k, path_len)
        ekf2["window_start_offset_s"] = float(t0_k - t0)
        result["ekf2_baseline"] = ekf2

    # ---- report
    def row(name, key, unit):
        a = eskf[key]
        b = ekf2[key] if ekf2 else float("nan")
        lim = THRESHOLDS.get(key)
        flag = "" if lim is None else ("  ok" if a <= lim else f"  FAIL (> {lim})")
        print(f"  {name:<28}{a:9.3f}{b:12.3f}  {unit}{flag}")

    print(f"window {result['window_s']:.1f} s, ground-truth path {path_len:.1f} m, "
          f"timesync clock steps undone {result['clock_steps_s']} s, "
          f"glitch samples dropped {result['dropped_samples']}")
    print(f"  {'metric':<28}{'ESKF':>9}{'EKF2':>12}")
    row("horizontal pos RMSE", "horiz_pos_rmse_m", "m")
    row("horizontal pos final", "horiz_pos_final_m", "m")
    row("horizontal drift", "horiz_drift_pct_of_path", "% of path")
    row("altitude RMSE", "alt_rmse_m", "m")
    row("velocity RMSE", "vel_rmse_mps", "m/s")
    row("tilt RMSE", "tilt_rmse_deg", "deg")
    row("yaw RMSE", "yaw_rmse_deg", "deg")
    print(f"  NEES mean (ideal = dof): velocity {eskf['nees_vel_mean_3dof']:.2f}/3, "
          f"attitude {eskf['nees_att_mean_3dof']:.2f}/3, altitude {eskf['nees_alt_mean_1dof']:.2f}/1")

    failed = [k for k, lim in THRESHOLDS.items() if eskf[k] > lim]
    result["thresholds"] = THRESHOLDS
    result["failed"] = failed
    out_stem = "" if est_path.name == "est.csv" else f"_{est_path.stem}"
    (d / f"metrics{out_stem}.json").write_text(json.dumps(result, indent=2))

    # ---- plot
    try:
        warnings.filterwarnings("ignore", message="Unable to import Axes3D")
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib unusable ({type(exc).__name__}), skipping plot", file=sys.stderr)
    else:
        ts = t_est - t0
        fig, ax = plt.subplots(3, 2, figsize=(13, 11))
        a = ax[0, 0]
        a.plot(e["gpos"][:, 1], e["gpos"][:, 0], "k", lw=2, label="ground truth")
        a.plot(e_pos[:, 1], e_pos[:, 0], label="ESKF (ours)")
        if ekf2:
            a.plot(e2["gpos"][:, 1] + e2["e_pos"][:, 1], e2["gpos"][:, 0] + e2["e_pos"][:, 0],
                   "--", label="PX4 EKF2")
        a.set(xlabel="east [m]", ylabel="north [m]", title="track (top-down)")
        a.set_aspect("equal", adjustable="datalim")
        a.legend()
        a = ax[0, 1]
        a.plot(ts, np.linalg.norm(e["e_pos"][:, :2], axis=1), label="ESKF")
        if ekf2:
            a.plot(t_k - t0, np.linalg.norm(e2["e_pos"][:, :2], axis=1), "--", label="EKF2")
        a.set(xlabel="t [s]", ylabel="[m]", title="horizontal position error")
        a.legend()
        a = ax[1, 0]
        for i, (lbl, c) in enumerate(zip("NED", ("C0", "C1", "C2"))):
            a.plot(ts, e["e_vel"][:, i], c, lw=1, label=f"v{lbl} error")
            a.fill_between(ts, -3 * np.sqrt(var_v[:, i]), 3 * np.sqrt(var_v[:, i]), color=c, alpha=0.12)
        a.set(xlabel="t [s]", ylabel="[m/s]", title="ESKF velocity error with ±3σ")
        a.legend(ncol=3, fontsize=8)
        a = ax[1, 1]
        a.plot(ts, -e["gpos"][:, 2], "k", lw=2, label="ground truth")
        a.plot(ts, -e_pos[:, 2], label="ESKF")
        if ekf2:
            a.plot(t_k - t0, -(e2["gpos"][:, 2] + e2["e_pos"][:, 2]), "--", label="EKF2")
        a.set(xlabel="t [s]", ylabel="[m]", title="height above start")
        a.legend()
        a = ax[2, 0]
        a.plot(ts, np.degrees(np.linalg.norm(e["e_rot"][:, :2], axis=1)), label="ESKF tilt")
        if ekf2:
            a.plot(t_k - t0, np.degrees(np.linalg.norm(e2["e_rot"][:, :2], axis=1)), "--", label="EKF2 tilt")
        a.set(xlabel="t [s]", ylabel="[deg]", title="tilt error (roll/pitch)")
        a.legend()
        a = ax[2, 1]
        a.plot(ts, e["e_yaw"], label="ESKF yaw")
        a.fill_between(ts, -3 * np.degrees(np.sqrt(var_a[:, 2])), 3 * np.degrees(np.sqrt(var_a[:, 2])),
                       color="C0", alpha=0.12)
        if ekf2:
            a.plot(t_k - t0, e2["e_yaw"], "--", label="EKF2 yaw")
        a.set(xlabel="t [s]", ylabel="[deg]", title="yaw error (ESKF ±3σ shaded)")
        a.legend()
        for a in ax.flat:
            a.grid(True, alpha=0.4)
        fig.tight_layout()
        fig.savefig(d / f"estimation{out_stem}.png", dpi=110)
        print(f"plot: {d / f'estimation{out_stem}.png'}")

    if args.check and failed:
        print(f"FAILED thresholds: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
