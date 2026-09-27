# Architecture

## Process view (simulation)

| Process | Started by | Role |
|---|---|---|
| `gz sim -s` (server) | PX4 `px4-rc.gzsim` | physics, sensor simulation |
| `gz sim -g` (GUI) | PX4 `px4-rc.gzsim` (skipped with `HEADLESS=1`) | visualisation only |
| `px4` | `scripts/sim.sh` | flight stack. `gz_bridge` exchanges sensor and actuator data with Gazebo over gz-transport, in lockstep |
| `MicroXRCEAgent udp4 -p 8888` | `scripts/sim.sh` | turns PX4's XRCE-DDS client sessions into full DDS participants |
| ROS 2 nodes | `ros2 run` / launch | autonomy. They only see `/fmu/in/*` and `/fmu/out/*` DDS topics |

For project vehicles (`sim/models/<name>/px4_airframe`, e.g. the Phase 3
`x500_mapper`), `scripts/sim.sh` starts the Gazebo server and spawns the model
itself, and PX4 attaches to it (`PX4_GZ_MODEL_NAME`, `PX4_SYS_AUTOSTART`).

Phase 3's ROS nodes (the ros_gz_bridge, the odometry bridge, RTAB-Map) run on
**simulation time** from Gazebo's `/clock`, because the camera images carry
Gazebo stamps.

PX4 and Gazebo run in lockstep: PX4's clock advances only as Gazebo steps. If
Gazebo runs slower than real time, the whole simulation slows down consistently
rather than PX4 seeing late sensor data.

## Interface contract with PX4

The autonomy stack depends only on the topics listed in
`PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` for the pinned PX4
release. Rules:

- **Versioned topics.** Since v1.16, a message with a non-zero `MESSAGE_VERSION`
  is published as `<topic>_v<N>`, e.g. `/fmu/out/vehicle_status_v1`. Nodes derive
  topic names from the `px4_msgs` constant (`quad_offboard/px4_topics.hpp`) rather
  than hard-coding them.
- **QoS.** PX4 publishes best-effort. Subscribers use `rclcpp::SensorDataQoS()`, or
  they never match.
- **Frames.** PX4 uses NED for the local frame and FRD for the body frame. Anything
  republished to the wider ROS graph uses ENU/FLU (REP-103). Conversions happen once,
  at the boundary node.
- **Timestamps.** `timestamp` fields are in microseconds. Messages sent **to** PX4
  (`/fmu/in/*`) carry `timestamp = 0`, which PX4 replaces with its own time on
  arrival. A non-zero stamp is converted with the uXRCE-DDS timesync offset.
  In SITL, where the simulation runs a few percent off real time, that offset
  goes stale between corrections: fresh offboard setpoints looked ~1 s old, and
  PX4 dropped to Hold mid-mission (docs/phase3_perception_slam.md).
- **Extra topics.** The estimator's inputs (optical flow, rangefinder,
  magnetometer) and the SITL ground truth are not exported by stock PX4. They
  are added by `firmware/px4_patches/0001-*`.
- **Timesync artefacts.** Stamps can glitch (one sample in raw PX4 boot time) or
  step (−0.3 s to +8 s; large steps when the simulator runs slower than real
  time). Consumers integrate with PX4's own intervals, and accept a stamp jump
  only once the next sample confirms it (see docs/phase2_state_estimation.md).
- **Pinning.** `px4_msgs` must come from the branch that matches the PX4 release
  (`release/1.17` for `v1.17.0`). A mismatch compiles fine but silently breaks topic
  matching.

## Control authority

PX4 always owns stabilisation, the attitude and rate loops, failsafes and arming
checks. The companion computer only sends setpoints (velocity in Phase 1, position
or trajectory in later phases) while PX4 is in OFFBOARD. Any failsafe or operator
mode change takes authority back immediately, and our nodes are written to accept
that rather than fight it.

## Workspace layout

`ros2_ws/src` holds our packages plus `external/` (px4_msgs, px4_ros_com), which
the setup script clones at pinned branches and git ignores. We don't use a
submodule because px4_msgs is large, and a clone keeps the pin in one place
(`scripts/setup/common.sh`).
