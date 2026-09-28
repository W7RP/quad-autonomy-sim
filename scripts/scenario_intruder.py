#!/usr/bin/env python3
"""Phase 4 scenario: drop an obstacle onto the vehicle's planned path mid-flight.

Scenario orchestration, not autonomy: it plays the role of the world changing.
Nothing in the flight or planning path depends on it.

It waits until the planner is on the `--leg` goal and the vehicle has flown at
least `--after-m` along that leg. Then it moves the world's "intruder" box onto
the current planned path, `--ahead-m` ahead of the vehicle (never closer than
`--min-gap-m`), with Gazebo's set_pose service. The move (simulation time and
position) goes to `--log` as JSON, so the evaluation can measure how long the
planner took to react.

Frames: the planner's path is in `odom` (ENU, origin = vehicle start). The
world is ENU too, and the vehicle spawns at the world origin, so world x, y =
odom x, y. The box is placed standing on the ground.

The move is a gz-transport request made in-process (python3-gz-transport13,
installed with Gazebo Harmonic). An earlier version ran the `gz service` CLI,
which takes ~1 s to start under load; rclpy does not spin meanwhile, so the
logged time was that much earlier than the real move, and the box landed
~1.5 m closer to the vehicle than intended.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Scenario(Node):
    def __init__(self, a):
        super().__init__("scenario_intruder",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.a = a
        self.goal = None
        self.path = []
        self.pos = None
        self.leg_start = None
        self.done = False
        self.gz = GzNode()  # created up front: discovery is done before the move
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/planner/events", self.on_event, 50)
        self.create_subscription(Path, "/planner/path", self.on_path, latched)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_timer(0.1, self.tick)

    def on_event(self, msg):
        e = json.loads(msg.data)
        if e.get("event") == "plan" and e.get("ok"):
            self.goal = e["goal"]
            if self.goal == self.a.leg and self.leg_start is None and self.pos:
                self.leg_start = self.pos

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_odom(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def point_ahead(self):
        """Walk the path polyline from the vehicle's closest point, ahead_m on."""
        best, best_d, best_s = None, float("inf"), 0.0
        s = 0.0
        for (x0, y0), (x1, y1) in zip(self.path, self.path[1:]):
            dx, dy = x1 - x0, y1 - y0
            seg = math.hypot(dx, dy)
            if seg < 1e-6:
                continue
            t = max(0.0, min(1.0, ((self.pos[0] - x0) * dx + (self.pos[1] - y0) * dy) / seg ** 2))
            d = math.hypot(x0 + t * dx - self.pos[0], y0 + t * dy - self.pos[1])
            if d < best_d:
                best, best_d, best_s = (x0, y0), d, s + t * seg
            s += seg
        if best is None:
            return None
        target = best_s + self.a.ahead_m
        s = 0.0
        for (x0, y0), (x1, y1) in zip(self.path, self.path[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if s + seg >= target and seg > 1e-6:
                t = (target - s) / seg
                return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            s += seg
        return None  # the path ends before that: too close to the goal

    def tick(self):
        if self.done or self.goal != self.a.leg or not self.path or not self.pos or not self.leg_start:
            return
        flown = math.hypot(self.pos[0] - self.leg_start[0], self.pos[1] - self.leg_start[1])
        if flown < self.a.after_m:
            return
        p = self.point_ahead()
        if p is None or math.hypot(p[0] - self.pos[0], p[1] - self.pos[1]) < self.a.min_gap_m:
            return
        req = Pose()
        req.name = "intruder"
        req.position.x, req.position.y, req.position.z = p[0], p[1], self.a.box_z
        req.orientation.w = 1.0
        t = self.get_clock().now().nanoseconds * 1e-9
        w0 = time.monotonic()
        ok, rep = self.gz.request(f"/world/{self.a.world}/set_pose", req, Pose, Boolean, 3000)
        call_ms = (time.monotonic() - w0) * 1e3
        rec = {"event": "intruder_moved", "ok": bool(ok and rep.data), "t": t, "call_ms": call_ms,
               "x": p[0], "y": p[1], "vehicle": list(self.pos), "flown_on_leg_m": flown}
        with open(self.a.log, "w") as f:
            json.dump(rec, f)
        self.get_logger().info(json.dumps(rec))
        self.done = True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="demo_final")
    ap.add_argument("--leg", type=int, default=4, help="goal index of the leg to disrupt")
    ap.add_argument("--after-m", type=float, default=3.0)
    ap.add_argument("--ahead-m", type=float, default=3.0)
    ap.add_argument("--min-gap-m", type=float, default=2.5)
    ap.add_argument("--box-z", type=float, default=1.3, help="world z of the box centre (2.6 m tall)")
    ap.add_argument("--log", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = Scenario(a)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
