#!/usr/bin/env python3
"""Generate sim/worlds/cluttered.sdf (Phase 3 mapping world) from one obstacle list.

The obstacle list below is the single source of truth: the world file is
generated from it, and scripts/eval_map.py reads the generated SDF back as the
ground-truth geometry the RTAB-Map map is scored against. It also writes
demo_final.sdf (Phase 4): the same field plus a movable intruder and a fixed
camera that records the flythrough.

Layout (world frame ENU, metres; the vehicle spawns at the origin facing +x):
the Phase 3 route is a 10 m square at 1.8 m altitude with corners (0,0),
(10,0), (10,10), (0,10). Obstacles sit inside and outside that loop with at
least 1.2 m clearance from the route, plus a two-pillar gate on the x=10 leg.
Everything is textured (the flow_ground texture) because RTAB-Map needs visual
features, not just geometry.

Offline asset tooling (not part of any flight path).
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE_WORLD = HERE.parent / "worlds" / "flow_field.sdf"
OUT_DIR = HERE.parent / "worlds"

# (name, kind, centre x, y, size...). Boxes: size = (sx, sy, sz), sitting on the
# ground. Cylinders: (radius, height). Yaw in degrees for boxes.
OBSTACLES = [
    # inside the loop
    ("pillar_centre", "cylinder", 5.0, 5.0, (0.4, 4.0)),
    ("box_tall", "box", 3.0, 6.5, (1.5, 1.5, 2.5), 0),
    ("box_low", "box", 7.0, 3.0, (1.0, 2.0, 1.2), 20),
    # gate on the x = 10 leg: the route passes between these
    ("gate_west", "cylinder", 8.45, 5.0, (0.3, 3.5)),
    ("gate_east", "cylinder", 11.55, 5.0, (0.3, 3.5)),
    # outside the loop
    ("wall_south", "box", 5.0, -2.5, (8.0, 0.3, 3.0), 0),
    ("box_east", "box", 13.0, 8.0, (2.0, 2.0, 3.0), 0),
    ("box_north", "box", 4.0, 12.5, (3.0, 1.0, 2.0), 0),
    ("pillar_nw", "cylinder", -2.5, 7.0, (0.5, 3.5)),
    ("pillar_sw", "cylinder", -2.5, 2.0, (0.3, 2.5)),
    ("box_ne_corner", "box", 12.8, 12.8, (1.2, 1.2, 1.8), 45),
]
ROUTE_CORNERS = [(0, 0), (10, 0), (10, 10), (0, 10)]

# demo_final only (Phase 4). The intruder starts parked outside the field;
# scripts/scenario_intruder.py moves it onto the vehicle's planned path mid-
# flight (gz set_pose). It is excluded from the route-clearance check on
# purpose: blocking the path is its job.
INTRUDER = ("intruder", "box", 22.0, -8.0, (1.0, 1.0, 2.6), 0)
# Fixed overhead camera recording the flythrough (camera-video-recorder
# system, service /flythrough/record_video). Pose: x y z roll pitch yaw.
FLYTHROUGH_CAMERA_POSE = (5.0, -9.0, 11.0, 0.0, 0.85, 1.5708)
MIN_CLEARANCE = 1.2  # metres from any route segment to any obstacle surface


def seg_point_distance(a, b, p):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def footprint_outline(o, step=0.05):
    """Points along the obstacle's footprint outline (exact up to `step`)."""
    name, kind, x, y = o[:4]
    if kind == "cylinder":
        r = o[4][0]
        n = max(16, int(2 * math.pi * r / step))
        return [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n))
                for k in range(n)]
    sx, sy, _ = o[4]
    yaw = math.radians(o[5])
    c, s_ = math.cos(yaw), math.sin(yaw)
    corners = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
    pts = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        for k in range(n):
            u = ax + (bx - ax) * k / n
            v = ay + (by - ay) * k / n
            pts.append((x + c * u - s_ * v, y + s_ * u + c * v))
    return pts


def check_clearance():
    segs = list(zip(ROUTE_CORNERS, ROUTE_CORNERS[1:] + ROUTE_CORNERS[:1]))
    bad = []
    for o in OBSTACLES:
        d = min(seg_point_distance(a, b, p) for a, b in segs for p in footprint_outline(o))
        if d < MIN_CLEARANCE:
            bad.append((o[0], round(d, 2)))
    return bad


def material():
    return """<material>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal>
              <albedo_map>model://flow_ground/materials/textures/flow_ground.png</albedo_map>
              <roughness>0.9</roughness><metalness>0.0</metalness>
            </metal></pbr>
          </material>"""


def model_sdf(o):
    name, kind, x, y = o[:4]
    if kind == "box":
        sx, sy, sz = o[4]
        yaw = math.radians(o[5])
        geom = f"<box><size>{sx} {sy} {sz}</size></box>"
        z = sz / 2
    else:
        r, h = o[4]
        yaw = 0.0
        geom = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
        z = h / 2
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          {material()}
        </visual>
      </link>
    </model>
"""


def flythrough_camera_sdf():
    x, y, z, r, p, yw = FLYTHROUGH_CAMERA_POSE
    return f"""    <model name="flythrough_camera">
      <static>true</static>
      <pose>{x} {y} {z} {r} {p} {yw}</pose>
      <link name="link">
        <sensor name="flythrough" type="camera">
          <update_rate>15</update_rate>
          <always_on>1</always_on>
          <topic>flythrough/image</topic>
          <camera>
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>960</width><height>540</height></image>
            <clip><near>0.5</near><far>100</far></clip>
          </camera>
          <!-- Must sit inside the <sensor>: attached to the model instead, the
               system finds no sensor and gz sim segfaults in Configure(). -->
          <plugin filename="gz-sim-camera-video-recorder-system"
                  name="gz::sim::systems::CameraVideoRecorder">
            <service>/flythrough/record_video</service>
            <use_sim_time>true</use_sim_time>
            <fps>15</fps>
            <bitrate>4000000</bitrate>
          </plugin>
        </sensor>
      </link>
    </model>
"""


def generate(name, extra_models):
    base = BASE_WORLD.read_text()
    base = base.replace('<world name="flow_field">', f'<world name="{name}">', 1)
    header_end = base.index("<sdf")
    header = """<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by sim/tools/generate_cluttered_world.py: edit the obstacle list
     there, not this file. Phase 3 mapping world: flow_field (textured ground,
     PX4 default physics/lighting/geodetic origin) plus textured obstacles
     around a 10 m square route. scripts/eval_map.py reads the obstacles below
     as ground truth. -->
"""
    body = base[header_end:]
    insert_at = body.index("    <light name=")
    obstacles = "".join(model_sdf(o) for o in OBSTACLES) + extra_models
    out = OUT_DIR / f"{name}.sdf"
    out.write_text(header + body[:insert_at] + obstacles + body[insert_at:])
    print(f"wrote {out} with {len(OBSTACLES)} obstacles (min route clearance {MIN_CLEARANCE} m checked)")


def main() -> int:
    bad = check_clearance()
    if bad:
        print(f"obstacles closer than {MIN_CLEARANCE} m to the route: {bad}", file=sys.stderr)
        return 1
    generate("cluttered", "")
    # Phase 4: the same field plus a movable intruder and a recording camera.
    # The intruder is not static, so set_pose also moves its collision shape.
    intruder = model_sdf(INTRUDER).replace("<static>true</static>", "<static>false</static>").replace(
        "<link name=\"link\">", "<link name=\"link\">\n        <gravity>false</gravity>\n"
        "        <inertial><mass>50</mass></inertial>", 1)
    generate("demo_final", intruder + flythrough_camera_sdf())
    return 0


if __name__ == "__main__":
    sys.exit(main())
