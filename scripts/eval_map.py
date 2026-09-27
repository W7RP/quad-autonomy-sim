#!/usr/bin/env python3
"""Score an RTAB-Map point cloud against the world's true geometry (Phase 3).

Offline analysis tooling only. Inputs, in a demo log directory:
  cloud.ply     the map, exported from the RTAB-Map database (map frame, ENU)
  gt_pos.csv    /fmu/out/vehicle_local_position_groundtruth_v1   (ground truth)
  ekf2_pos.csv  /fmu/out/vehicle_local_position_v1               (PX4 EKF2)
and the world SDF (default sim/worlds/cluttered.sdf), whose static box and
cylinder obstacles plus the ground plane are the ground-truth surfaces.

Alignment is measured, not fitted. The map frame is RTAB-Map's, which starts as
the odometry frame, i.e. PX4 EKF2's local origin. Its position in the world is
(true position - EKF2 position) at the first common sample, converted NED->ENU.
Fitting the cloud to the geometry (ICP) would hide exactly the errors this is
meant to measure.

Metrics:
  accuracy   distance from each map point to the nearest true surface
             (median, mean, share within 0.10 / 0.20 m)
  coverage   per obstacle, the share of its sampled side/top surface that
             has a map point within 0.15 m (only surfaces the flight could see
             are meaningful; the report lists every obstacle)
  phantoms   map points more than 0.5 m from any true surface
Writes map_metrics.json and map.png; with --check, exits 1 on missed thresholds.
"""
import argparse
import json
import math
import sys
import warnings
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
import px4_csv  # noqa: E402

THRESHOLDS = {
    "median_error_m": 0.10,
    "within_20cm_pct": 85.0,   # share of map points within 0.20 m of a true surface
    "phantom_pct": 3.0,        # share of points > 0.5 m from any true surface (max)
    "seen_obstacle_coverage_pct": 50.0,  # mean coverage over obstacles the map touched
}


# ---------------------------------------------------------------- geometry
def load_obstacles(sdf_path):
    """Static box/cylinder models of the world: dicts with kind, centre, yaw, size."""
    root = ET.parse(sdf_path).getroot()
    obstacles = []
    for model in root.iter("model"):
        if (model.findtext("static") or "").strip() != "true":
            continue
        pose = [float(v) for v in (model.findtext("pose") or "0 0 0 0 0 0").split()]
        geom = model.find("./link/collision/geometry")
        if geom is None:
            continue
        box, cyl = geom.find("box"), geom.find("cylinder")
        if box is not None:
            size = [float(v) for v in box.findtext("size").split()]
            obstacles.append(dict(name=model.get("name"), kind="box", c=np.array(pose[:3]),
                                  yaw=pose[5], size=np.array(size)))
        elif cyl is not None:
            obstacles.append(dict(name=model.get("name"), kind="cylinder", c=np.array(pose[:3]),
                                  r=float(cyl.findtext("radius")), h=float(cyl.findtext("length"))))
    return obstacles


def surface_distance(p, o):
    """Unsigned distance from points p (N,3) to obstacle o's surface."""
    d = p - o["c"]
    if o["kind"] == "box":
        c, s = math.cos(-o["yaw"]), math.sin(-o["yaw"])
        local = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1], d[:, 2]], -1)
        q = np.abs(local) - o["size"] / 2
    else:
        q = np.stack([np.hypot(d[:, 0], d[:, 1]) - o["r"], np.abs(d[:, 2]) - o["h"] / 2], -1)
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    return np.abs(outside + inside)


def sample_surface(o, step=0.1):
    """Points on the obstacle's sides and top (the bottom sits on the ground)."""
    pts = []
    if o["kind"] == "box":
        sx, sy, sz = o["size"]
        c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
        zs = np.arange(-sz / 2 + step / 2, sz / 2, step)
        for u in np.arange(-sx / 2, sx / 2 + 1e-9, step):
            for v in (-sy / 2, sy / 2):
                pts += [(u, v, z) for z in zs]
        for v in np.arange(-sy / 2, sy / 2 + 1e-9, step):
            for u in (-sx / 2, sx / 2):
                pts += [(u, v, z) for z in zs]
        for u in np.arange(-sx / 2, sx / 2 + 1e-9, step):
            pts += [(u, v, sz / 2) for v in np.arange(-sy / 2, sy / 2 + 1e-9, step)]
        pts = np.array(pts)
        pts = np.stack([c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1], pts[:, 2]], -1)
    else:
        r, h = o["r"], o["h"]
        n = max(12, int(2 * math.pi * r / step))
        ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
        for z in np.arange(-h / 2 + step / 2, h / 2, step):
            pts += [(r * math.cos(a), r * math.sin(a), z) for a in ang]
        pts += [(0.0, 0.0, h / 2)]
        pts = np.array(pts)
    return pts + o["c"]


# ---------------------------------------------------------------- inputs
def read_ply(path):
    """xyz of a PLY point cloud (ascii or binary_little_endian, any extra properties)."""
    with open(path, "rb") as f:
        # Only the vertex element's properties describe the vertex records; PCL
        # also writes e.g. a "camera" element with its own properties after it.
        # The vertex block comes first in the data, so it is read directly.
        header, props, n, element = [], [], 0, None
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            header.append(line)
            if line.startswith("element"):
                element = line.split()[1]
                if element == "vertex":
                    n = int(line.split()[-1])
            elif line.startswith("property") and element == "vertex":
                if line.startswith("property list"):
                    raise SystemExit("PLY vertex list properties are not supported")
                _, typ, name = line.split()
                props.append((name, typ))
            elif line == "end_header":
                break
        fmt = next(h for h in header if h.startswith("format")).split()[1]
        types = {"float": "<f4", "float32": "<f4", "double": "<f8", "uchar": "u1", "uint8": "u1",
                 "char": "i1", "int8": "i1", "ushort": "<u2", "short": "<i2", "uint": "<u4",
                 "int": "<i4", "uint32": "<u4", "int32": "<i4"}
        if fmt == "ascii":
            data = np.loadtxt(f, max_rows=n, ndmin=2)
            idx = [i for i, (name, _) in enumerate(props) if name in ("x", "y", "z")]
            return data[:, idx]
        if fmt != "binary_little_endian":
            raise SystemExit(f"unsupported PLY format {fmt}")
        dt = np.dtype([(name, types[typ]) for name, typ in props])
        arr = np.frombuffer(f.read(dt.itemsize * n), dtype=dt, count=n)
        return np.stack([arr["x"], arr["y"], arr["z"]], -1).astype(float)


def world_geodetic_origin(sdf_path):
    """(lat, lon, elevation) of the world's origin from <spherical_coordinates>."""
    sc = ET.parse(sdf_path).getroot().find(".//spherical_coordinates")
    return (float(sc.findtext("latitude_deg")), float(sc.findtext("longitude_deg")),
            float(sc.findtext("elevation") or 0.0))


def map_to_world_offset(d, sdf_path, origin="ekf2"):
    """World (ENU) position of the map origin.

    PX4's ground-truth local position is NOT relative to the world origin: its
    origin is the vehicle's start point, given as ref_lat/ref_lon/ref_alt. So:
      map origin = ref point in world + truth(t0) - EKF2(t0)
    with the ref point placed via the world's geodetic origin. An earlier
    version skipped the ref point and put the map's ground 0.26 m low."""
    imu = d / "imu.csv"
    ref = None
    if imu.exists():
        ref = px4_csv.reference_steps(px4_csv.read(imu, "px4_msgs/msg/SensorCombined")["timestamp"] * 1e-6)
    gt = px4_csv.read(d / "gt_pos.csv", "px4_msgs/msg/VehicleLocalPosition")
    k2 = px4_csv.read(d / "ekf2_pos.csv", "px4_msgs/msg/VehicleLocalPosition")
    tg, kg = px4_csv.unwrap_clock(gt["timestamp_sample"] * 1e-6, ref)
    tk, kk = px4_csv.unwrap_clock(k2["timestamp_sample"] * 1e-6, ref)
    t0 = max(tg[kg][0], tk[kk][0])
    g = np.array([np.interp(t0, tg[kg], gt[a][kg]) for a in "xyz"])
    e = np.array([np.interp(t0, tk[kk], k2[a][kk]) for a in "xyz"])
    lat0, lon0, elev0 = world_geodetic_origin(sdf_path)
    # Small-offset equirectangular conversion: exact to far below a millimetre
    # over the metres involved here.
    r_earth = 6378137.0
    ref_ned = np.array([
        math.radians(gt["ref_lat"][kg][0] - lat0) * r_earth,
        math.radians(gt["ref_lon"][kg][0] - lon0) * r_earth * math.cos(math.radians(lat0)),
        -(gt["ref_alt"][kg][0] - elev0)])
    # origin "start": the map was built on Gazebo's ground-truth odometry
    # (sim-only diagnostic), whose frame starts at the vehicle's true start
    # pose, which is exactly PX4's ground-truth reference point.
    off_ned = ref_ned if origin == "start" else ref_ned + g - e
    traj_ned = ref_ned + np.stack([gt[a][kg] for a in "xyz"], -1)
    traj_enu = np.stack([traj_ned[:, 1], traj_ned[:, 0], -traj_ned[:, 2]], -1)
    return np.array([off_ned[1], off_ned[0], -off_ned[2]]), traj_enu


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--cloud", type=Path, help="default: <logdir>/cloud.ply")
    ap.add_argument("--world", type=Path,
                    default=Path(__file__).resolve().parents[1] / "sim" / "worlds" / "cluttered.sdf")
    ap.add_argument("--map-origin", choices=["ekf2", "start"], default="ekf2",
                    help="ekf2: map built on PX4 odometry (default); start: on Gazebo ground truth")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    d = args.logdir

    obstacles = load_obstacles(args.world)
    cloud_map = read_ply(args.cloud or d / "cloud.ply")
    offset, traj = map_to_world_offset(d, args.world, args.map_origin)
    pts = cloud_map + offset

    # accuracy: nearest true surface (obstacles or the ground plane z = 0)
    dist = np.abs(pts[:, 2])
    nearest = np.full(len(pts), -1)
    for i, o in enumerate(obstacles):
        di = surface_distance(pts, o)
        closer = di < dist
        dist = np.where(closer, di, dist)
        nearest = np.where(closer, i, nearest)
    on_ground = nearest < 0

    # coverage per obstacle
    coverage = {}
    from_cloud = pts[~on_ground] if (~on_ground).any() else pts
    for o in obstacles:
        s = sample_surface(o)
        near = from_cloud[np.all(np.abs(from_cloud - o["c"]) < 5.0, axis=1)]
        if len(near) == 0:
            coverage[o["name"]] = 0.0
            continue
        hit = np.zeros(len(s), bool)
        for k in range(0, len(s), 256):  # chunked nearest-point test (memory-bounded)
            dd = np.linalg.norm(s[k:k + 256, None, :] - near[None, :, :], axis=2)
            hit[k:k + 256] = dd.min(axis=1) < 0.15
        coverage[o["name"]] = float(100.0 * hit.mean())
    seen = [v for v in coverage.values() if v > 0.0]

    m = {
        "points": int(len(pts)),
        "obstacle_points": int((~on_ground).sum()),
        "median_error_m": float(np.median(dist)),
        "mean_error_m": float(dist.mean()),
        "within_10cm_pct": float(100.0 * (dist < 0.10).mean()),
        "within_20cm_pct": float(100.0 * (dist < 0.20).mean()),
        "phantom_pct": float(100.0 * (dist > 0.5).mean()),
        "obstacles_seen": f"{len(seen)}/{len(obstacles)}",
        "seen_obstacle_coverage_pct": float(np.mean(seen)) if seen else 0.0,
        "coverage_pct": coverage,
        "map_origin_in_world_enu": offset.round(3).tolist(),
    }

    print(f"map: {m['points']} points ({m['obstacle_points']} on obstacles), origin in world {m['map_origin_in_world_enu']}")
    print(f"  distance to true surface: median {m['median_error_m']:.3f} m, mean {m['mean_error_m']:.3f} m, "
          f"within 10 cm {m['within_10cm_pct']:.1f} %, within 20 cm {m['within_20cm_pct']:.1f} %")
    print(f"  phantom points (> 0.5 m from any surface): {m['phantom_pct']:.2f} %")
    print(f"  obstacles seen {m['obstacles_seen']}, mean coverage of seen ones {m['seen_obstacle_coverage_pct']:.1f} %")
    for name, c in coverage.items():
        print(f"    {name:<16} {c:5.1f} %")

    failed = []
    for k, lim in THRESHOLDS.items():
        v = m[k]
        bad = v > lim if k in ("median_error_m", "phantom_pct") else v < lim
        if bad:
            failed.append(k)
    m["thresholds"], m["failed"] = THRESHOLDS, failed
    (d / "map_metrics.json").write_text(json.dumps(m, indent=2))

    try:
        warnings.filterwarnings("ignore", message="Unable to import Axes3D")
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib unusable ({type(exc).__name__}), skipping plot", file=sys.stderr)
    else:
        fig, ax = plt.subplots(1, 2, figsize=(15, 7))
        above = pts[:, 2] > 0.15
        sc = ax[0].scatter(pts[above, 0], pts[above, 1], c=pts[above, 2], s=1, cmap="viridis")
        fig.colorbar(sc, ax=ax[0], label="height [m]")
        for o in obstacles:
            if o["kind"] == "box":
                sx, sy, _ = o["size"]
                c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
                corners = [(u, v) for u, v in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
                poly = [(o["c"][0] + c * u - s * v, o["c"][1] + s * u + c * v) for u, v in corners]
                ax[0].add_patch(Polygon(poly, fill=False, ec="red", lw=1.5))
            else:
                ax[0].add_patch(Circle(o["c"][:2], o["r"], fill=False, ec="red", lw=1.5))
        ax[0].plot(traj[:, 0], traj[:, 1], "k--", lw=1, label="flown path (truth)")
        ax[0].set(title="map above ground (dots) vs true obstacles (red)", xlabel="x east [m]", ylabel="y north [m]")
        ax[0].set_aspect("equal")
        ax[0].legend(loc="upper right")
        ax[1].hist(np.clip(dist, 0, 1.0), bins=50)
        ax[1].set(title="distance from map point to nearest true surface", xlabel="[m] (clipped at 1 m)",
                  ylabel="points")
        for a in ax:
            a.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(d / "map.png", dpi=110)
        print(f"plot: {d / 'map.png'}")

    if args.check and failed:
        print(f"FAILED thresholds: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
