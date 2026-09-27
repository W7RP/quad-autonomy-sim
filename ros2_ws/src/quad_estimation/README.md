# quad_estimation (Phase 2)

A hand-rolled C++20 error-state EKF for a multirotor: IMU prediction, fused with
downward optical flow, a downward rangefinder and magnetometer heading. The
design, the results and the known limitations are in
[docs/phase2_state_estimation.md](../../../docs/phase2_state_estimation.md).

## Layout

| file | what |
|---|---|
| `include/quad_estimation/eskf.hpp`, `src/eskf.cpp` | Filter math and measurement models. ROS-free, fixed-size Eigen, no allocation. |
| `include/quad_estimation/pipeline.hpp`, `src/pipeline.cpp` | Per-sample logic around the filter: timeline / clock-step handling, static alignment, measurement buffering, gyro averaging for flow. Shared by the node and the replay tool. |
| `src/eskf_node.cpp` | ROS 2 node. Threads, executors and callback groups are documented at the top of the file. |
| `src/alloc_probe.cpp` | Replaced `operator new` that counts hot-path allocations (node executable only). |
| `src/eskf_replay.cpp` | Offline replay of recorded flights through the same pipeline. |
| `src/params.cpp` | One list of tunable parameters, shared by the node (ROS params) and replay (YAML). |
| `config/eskf.yaml` | SITL-tuned parameters, with notes on hardware values. |
| `test/` | Jacobians vs finite differences, a no-malloc trap, a synthetic flight with NEES, and timeline robustness. |

## Interface

| | topic | type |
|---|---|---|
| in | `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` |
| in | `/fmu/out/sensor_optical_flow` | `px4_msgs/SensorOpticalFlow` (needs `firmware/px4_patches/0001`) |
| in | `/fmu/out/distance_sensor` | `px4_msgs/DistanceSensor` (needs `0001`) |
| in | `/fmu/out/vehicle_magnetometer` | `px4_msgs/VehicleMagnetometer` (needs `0001`) |
| out | `/eskf/odometry_ned` | `px4_msgs/VehicleOdometry`: NED/FRD, PX4 time base, diagonal variances |
| out | `/eskf/odometry` | `nav_msgs/Odometry`: ENU/FLU (REP-103), frames `odom` / `base_link` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray`: callback timing, allocations, fusion counters, clock steps |

`/eskf/odometry_ned` uses the same message and conventions as PX4's
`/fmu/in/vehicle_visual_odometry`. Feeding it back to PX4 would be a small step,
but it is not done: the estimator runs in shadow mode.

## Run

```bash
ros2 run quad_estimation eskf_node --ros-args --params-file config/eskf.yaml
```

Keep the vehicle still for about 2 s after start: the filter aligns from
stationary IMU data before it publishes anything.

## Replay a recorded flight

```bash
python3 scripts/prepare_replay.py logs/phase2_<ts>     # PX4 CSVs -> normalised CSVs
ros2_ws/install/quad_estimation/lib/quad_estimation/eskf_replay \
  --in logs/phase2_<ts>/replay --params ros2_ws/src/quad_estimation/config/eskf.yaml \
  --out logs/phase2_<ts>/est_replay.csv [--set name=value ...]
python3 scripts/eval_estimation.py logs/phase2_<ts> --est logs/phase2_<ts>/est_replay.csv
```
