# Phase 4: Autonomy loop (plan)

**Status: not started.**

## Goal

Map, then plan, then fly, then replan around an obstacle that moved or appeared.

## Plan

- **Map representation.** An OctoMap (or a voxel grid) from the Phase 3 RTAB-Map
  output, inflated by the vehicle radius plus a margin.
- **Planner.** A C++20 RRT* in 3D over the occupancy map, followed by shortcutting
  and a time parameterisation that respects the Phase 1 speed limits. Nav2 was
  considered and rejected as the primary path: it's built around 2D costmaps and
  ground-robot controllers. A flying vehicle would use it in only a 2.5D slice, and
  the fit isn't clean.
- **Execution.** The planned path feeds the Phase 1 `WaypointFollower`, or a
  trajectory-setpoint variant, over the same offboard interface.
- **Replanning.** A collision check of the remaining path against the live map runs
  at a fixed rate. If the path is blocked, the vehicle holds, replans from its
  current state, and resumes. The demo moves an obstacle through the gz service
  API partway through the flight.
- **Final demo.** `sim/worlds/demo_final.sdf`, with a recorded Gazebo flythrough
  video and bag.
