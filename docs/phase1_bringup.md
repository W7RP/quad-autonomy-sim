# Phase 1: Bring-up design notes

## Offboard node (`quad_offboard/offboard_square`)

**State machine** (20 Hz timer):

```
WAIT_FOR_PX4 --valid local pos + status--> PRIME --1 s of setpoints--> ENGAGE
ENGAGE --OFFBOARD && ARMED--> MISSION --route done--> LAND --disarmed--> DONE
ENGAGE --timeout--> ABORTED        MISSION --left OFFBOARD--> ABORTED
```

- **PRIME.** PX4 refuses the switch to OFFBOARD unless setpoints are already
  streaming. We send zero velocity for 1 s first.
- **ENGAGE.** Mode and arm commands are re-sent every second. Right after SITL
  boots, arming is rejected until EKF2 converges and the preflight checks pass,
  which takes a few seconds. Retrying is simpler and more robust than parsing
  `vehicle_command_ack`.
- **MISSION.** `WaypointFollower::step()` gives a velocity command:
  `v = Kp · (wp − p)`. The horizontal speed limit is applied to the vector (so the
  direction is kept) and the vertical limit separately. A waypoint is accepted when
  the vehicle is within 0.3 m of it *and* moving slower than 0.3 m/s, so corners
  are actually reached rather than cut.
- **LAND.** Sends `NAV_LAND`, then waits for PX4 to auto-disarm
  (`COM_DISARM_LAND`).

**Why velocity setpoints, not position?** The task calls for velocity control, and
it keeps the path-following logic on the companion side, which is where Phases 2–4
plug in. PX4's own position controller still closes the inner velocity loop.

**Executor.** Everything runs in a single MutuallyExclusive callback group on a
SingleThreadedExecutor. The subscriptions only copy scalars into members, and the
timer reads them. Because callbacks never overlap, no locks are needed. Phase 2
introduces the multi-group design that the estimator needs.

**Allocation.** `WaypointFollower` stores the route in a fixed `std::array` and
never allocates after construction. The ROS message objects are stack-allocated
per publish. Phase 1 doesn't claim hard real-time: rclcpp's publish path can still
allocate internally. Phase 2 addresses that with loaned messages and the
estimator's own pre-allocated state.

## Parameters

PX4-side parameters are in `firmware/params/common.params`, each with its
reason. The node's parameters are in `quad_offboard/config/square_mission.yaml`.

## Verification checklist

1. `ros2 topic list | grep fmu` shows `/fmu/out/vehicle_odometry`,
   `/fmu/out/vehicle_status_v1`, `/fmu/in/trajectory_setpoint`, and so on.
2. `ros2 topic hz /fmu/out/vehicle_odometry` reports
   roughly 100 Hz.
3. `scripts/demo_phase1.sh --headless` exits 0, and `track.png` shows a closed 5 m
   square at 3 m altitude.
