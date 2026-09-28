#!/usr/bin/env python3
"""Score a Phase 4 autonomy mission (scripts/demo_phase4.sh) against ground truth.

Offline analysis tooling only. Inputs in the log directory:
  events.jsonl   the planner's events (plans with their paths, goals reached)
  intruder.json  when and where scripts/scenario_intruder.py dropped the obstacle
  gt_pos.csv, ekf2_pos.csv, imu.csv, status.csv, node.log
and the world SDF (default sim/worlds/demo_final.sdf) for the true obstacles.

Metrics:
  completion    mission_complete event, every goal reached, landed and disarmed
  safety        the vehicle's minimum distance to any TRUE obstacle surface,
                from its ground-truth track. The intruder counts at its parked
                pose before the move and at its dropped pose after.
  reaction      did the intruder block the path in force when it appeared? How
                long until the first plan whose path clears it by at least the
                hard margin?
  planning      replans by reason, planning time, failures
  efficiency    flown distance
  map           the planner's obstacle points at the end of the mission
                (final_map.xyz, written by the planner's debug_dump_dir), per
                source (RTAB-Map's map, live depth): distance to the nearest
                true surface, and the share of phantoms (> 0.5 m from any)
Writes mission_metrics.json and mission.png; with --check, exits 1 on missed thresholds.
"""
import argparse
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
import eval_map  # noqa: E402  (world geometry, map->world alignment)
import px4_csv  # noqa: E402

THRESHOLDS = {
    "min_clearance_m": 0.40,       # vehicle centre to any true surface (x500 arm radius ~0.35 m)
    "intruder_reaction_s": 1.0,    # move -> first plan that clears it
    "max_plan_ms": 400.0,          # the RRT* time budget
    "map_phantom_pct": 3.0,        # as Phase 3's acceptance (scripts/eval_map.py)
}
HARD_MARGIN = 0.55                 # planner's hard_inflation_m
INTRUDER_SIZE = np.array([1.0, 1.0, 2.6])
INTRUDER_PARKED = np.array([22.0, -8.0, 1.3])


def box(centre):
    return dict(name="intruder", kind="box", c=np.asarray(centre, float), yaw=0.0, size=INTRUDER_SIZE)


def path_clearance(path_world, obstacle, step=0.05):
    """Minimum distance from a polyline to an obstacle surface (sampled every `step`)."""
    pts = []
    for a, b in zip(path_world[:-1], path_world[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        pts.append(a + (b - a) * (np.arange(n + 1)[:, None] / n))
    return float(eval_map.surface_distance(np.vstack(pts), obstacle).min()) if pts else float("inf")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir", type=Path)
    ap.add_argument("--world", type=Path,
                    default=Path(__file__).resolve().parents[1] / "sim" / "worlds" / "demo_final.sdf")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    d = args.logdir

    events = [json.loads(line) for line in open(d / "events.jsonl") if line.strip()]
    plans = [e for e in events if e.get("event") == "plan"]
    ok_plans = [e for e in plans if e.get("ok")]
    goals_reached = sorted({e["goal"] for e in events if e.get("event") == "goal_reached"})
    n_goals = 1 + max(e["goal"] for e in plans) if plans else 0
    complete = any(e.get("event") == "mission_complete" for e in events)
    node_log = (d / "node.log").read_text() if (d / "node.log").exists() else ""
    landed = "landed and disarmed" in node_log

    # Ground-truth track in the world frame, and odom -> world offset for the plans.
    offset, traj = eval_map.map_to_world_offset(d, args.world, "ekf2")
    to_world = lambda p: np.asarray(p, float) + offset  # noqa: E731  (odom and world are both ENU)

    obstacles = eval_map.load_obstacles(args.world)  # static obstacles only
    intr = json.loads((d / "intruder.json").read_text()) if (d / "intruder.json").exists() else None
    moved_centre = np.array([intr["x"], intr["y"], 1.3]) if intr and intr.get("ok") else None

    # Safety: vehicle vs every true surface, time-aware for the intruder. The
    # scenario's time is simulation time; ground truth is PX4-stamped, so the
    # move is located on the track by position/time mapping through odometry:
    # take the ground-truth sample closest to where the vehicle was at the move.
    gt = px4_csv.read(d / "gt_pos.csv", "px4_msgs/msg/VehicleLocalPosition")
    clear_static = np.full(len(traj), np.inf)
    nearest = np.full(len(traj), "", dtype=object)
    for o in obstacles:
        dist = eval_map.surface_distance(traj, o)
        closer = dist < clear_static
        clear_static = np.where(closer, dist, clear_static)
        nearest = np.where(closer, o["name"], nearest)
    k_move = None
    if moved_centre is not None:
        veh = to_world([intr["vehicle"][0], intr["vehicle"][1], 0.0])[:2]
        k_move = int(np.argmin(np.linalg.norm(traj[:, :2] - veh, axis=1)))
    d_parked = eval_map.surface_distance(traj, box(INTRUDER_PARKED))
    d_moved = eval_map.surface_distance(traj, box(moved_centre)) if moved_centre is not None else np.full(len(traj), np.inf)
    d_intr = d_parked.copy()
    if k_move is not None:
        d_intr[k_move:] = d_moved[k_move:]
    airborne = traj[:, 2] > 0.5  # on the ground the landing gear touches the ground plane, fine
    clearance = np.minimum(clear_static, d_intr)
    k_min = int(np.argmin(np.where(airborne, clearance, np.inf)))
    min_clear = float(clearance[k_min])
    closest_to = "intruder" if d_intr[k_min] <= clear_static[k_min] else nearest[k_min]

    # Reaction: the path in force at the move, and the first plan clearing the intruder.
    reaction = {}
    if moved_centre is not None:
        intr_box = box(moved_centre)
        before = [e for e in ok_plans if e["t"] <= intr["t"]]
        after = [e for e in ok_plans if e["t"] > intr["t"]]
        in_force = before[-1] if before else None
        blocked = in_force is not None and path_clearance(
            np.array([to_world(p) for p in in_force["path"]]), intr_box) < HARD_MARGIN
        first_clear = next((e for e in after if path_clearance(
            np.array([to_world(p) for p in e["path"]]), intr_box) >= HARD_MARGIN), None)
        reaction = {
            "path_in_force_was_blocked": bool(blocked),
            "intruder_reaction_s": (first_clear["t"] - intr["t"]) if first_clear else None,
            "reaction_plan_reason": first_clear["reason"] if first_clear else None,
        }

    # Map accuracy of what the planner planned on, per source.
    map_quality = {}
    dump = d / "final_map.xyz"
    if dump.exists():
        pts = np.loadtxt(dump, ndmin=2)
        truth = obstacles + ([box(moved_centre)] if moved_centre is not None else [])
        for src, name in ((0, "rtabmap"), (1, "live_depth")):
            q = pts[pts[:, 0] == src, 1:] + offset
            if len(q):
                dist = np.min([eval_map.surface_distance(q, o) for o in truth] + [np.abs(q[:, 2])],
                              axis=0)  # obstacles and the ground plane
                map_quality[name] = {"points": int(len(q)), "median_m": float(np.median(dist)),
                                     "phantom_pct": float(100.0 * np.mean(dist > 0.5))}

    reasons = {}
    for e in plans:
        key = e["reason"] + ("" if e.get("ok") else " (failed)")
        reasons[key] = reasons.get(key, 0) + 1
    plan_ms = [e["plan_ms"] for e in plans if "plan_ms" in e]
    flown = float(np.sum(np.linalg.norm(np.diff(traj[airborne][:, :2], axis=0), axis=1)))

    m = {
        "mission_complete": complete,
        "goals_reached": f"{len(goals_reached)}/{n_goals}",
        "landed": landed,
        "min_clearance_m": min_clear,
        "min_clearance_to": closest_to,
        **reaction,
        "plans": len(plans),
        "plans_by_reason": reasons,
        "plan_failures": sum(1 for e in plans if not e.get("ok")),
        "mean_plan_ms": float(np.mean(plan_ms)) if plan_ms else 0.0,
        "max_plan_ms": float(np.max(plan_ms)) if plan_ms else 0.0,
        "flown_distance_m": flown,
        "mission_time_s": (events[-1]["t"] - plans[0]["t"]) if plans else 0.0,
        "map": map_quality,
    }

    print(f"mission complete: {complete}, goals reached {m['goals_reached']}, landed: {landed}")
    print(f"minimum clearance to any true obstacle surface: {min_clear:.2f} m ({closest_to})")
    if reaction:
        rs = reaction["intruder_reaction_s"]
        print(f"intruder: blocked the path in force: {reaction['path_in_force_was_blocked']}; "
              f"reaction {('%.2f s' % rs) if rs is not None else 'NEVER'} ({reaction['reaction_plan_reason']})")
    print(f"plans: {len(plans)} {reasons}; planning time mean {m['mean_plan_ms']:.0f} ms, max {m['max_plan_ms']:.0f} ms")
    print(f"flown {flown:.1f} m in {m['mission_time_s']:.1f} s of mission")
    for name, q in map_quality.items():
        print(f"map ({name}): {q['points']} points, median {100 * q['median_m']:.1f} cm from the true "
              f"surface, {q['phantom_pct']:.1f} % phantoms")

    failed = []
    if not (complete and landed and len(goals_reached) == n_goals):
        failed.append("completion")
    if min_clear < THRESHOLDS["min_clearance_m"]:
        failed.append("min_clearance_m")
    if reaction and (not reaction["path_in_force_was_blocked"] or reaction["intruder_reaction_s"] is None
                     or reaction["intruder_reaction_s"] > THRESHOLDS["intruder_reaction_s"]):
        failed.append("intruder_reaction_s")
    if not reaction:
        failed.append("intruder_not_moved")
    if m["max_plan_ms"] > THRESHOLDS["max_plan_ms"]:
        failed.append("max_plan_ms")
    if any(q["phantom_pct"] > THRESHOLDS["map_phantom_pct"] for q in map_quality.values()):
        failed.append("map_phantom_pct")
    m["thresholds"], m["failed"] = THRESHOLDS, failed
    (d / "mission_metrics.json").write_text(json.dumps(m, indent=2, default=str))

    try:
        warnings.filterwarnings("ignore", message="Unable to import Axes3D")
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon, Rectangle
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib unusable ({type(exc).__name__}), skipping plot", file=sys.stderr)
    else:
        fig, ax = plt.subplots(figsize=(10, 9))
        for o in obstacles:
            if o["kind"] == "box":
                sx, sy, _ = o["size"]
                c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
                cs = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
                ax.add_patch(Polygon([(o["c"][0] + c * u - s * v, o["c"][1] + s * u + c * v) for u, v in cs],
                                     fc="0.8", ec="k"))
            else:
                ax.add_patch(Circle(o["c"][:2], o["r"], fc="0.8", ec="k"))
        cmap = plt.get_cmap("viridis")
        t0, t1 = (ok_plans[0]["t"], ok_plans[-1]["t"]) if ok_plans else (0, 1)
        for e in ok_plans:
            p = np.array([to_world(q) for q in e["path"]])
            ax.plot(p[:, 0], p[:, 1], "-", lw=0.8, alpha=0.6, color=cmap((e["t"] - t0) / max(t1 - t0, 1e-6)))
        ax.plot(traj[airborne][:, 0], traj[airborne][:, 1], "k", lw=2, label="flown (ground truth)")
        if moved_centre is not None:
            ax.add_patch(Rectangle(moved_centre[:2] - 0.5, 1.0, 1.0, fc="tab:red", ec="darkred",
                                   alpha=0.8, label="intruder (dropped mid-flight)"))
            ax.plot(*traj[k_move, :2], "rx", ms=12, mew=3, label="vehicle when dropped")
        ax.plot(*traj[k_min, :2], "o", mfc="none", mec="tab:orange", ms=14, mew=2,
                label=f"min clearance {min_clear:.2f} m")
        ax.set(title="Phase 4: planned paths (colour = time) and flown track", xlabel="x east [m]",
               ylabel="y north [m]", xlim=(-5, 15), ylim=(-5, 15))
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        fig.tight_layout()
        fig.savefig(d / "mission.png", dpi=110)
        print(f"plot: {d / 'mission.png'}")

    if args.check and failed:
        print(f"FAILED: {', '.join(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
