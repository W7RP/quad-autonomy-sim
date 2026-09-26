# Phase 2: State estimation (plan)

**Status: not started.** This page is the design brief. It will turn into the
design record as the phase is built.

## Goal

A hand-rolled C++20 filter (error-state EKF first, UKF as a variant if useful)
running on the companion computer. It estimates position, velocity, attitude and
IMU biases from:

- IMU at about 250 Hz (`/fmu/out/sensor_combined`)
- optical flow + range at about 30 Hz (`gz_x500_flow`, airframe 4021)
- later: visual odometry from a camera (`gz_x500_mono_cam_down` or `gz_x500_vision`)

PX4's EKF2 keeps flying the vehicle. Our filter runs in parallel ("shadow mode"),
so a filter bug can't crash the vehicle. Feeding it back to PX4 through
`/fmu/in/vehicle_visual_odometry` is an optional stretch goal at the end of the
phase.

## Validation

- Ground truth: the Gazebo pose of the model, bridged via `ros_gz_bridge`, used
  **only** by the evaluation node, never by the filter.
- Metrics: position and attitude error over time, RMSE per axis, and NEES
  (normalised estimation error squared) to check that the covariance is honest,
  not just that the mean is close.
- Artefacts: CSV per run plus plots, produced by a demo script like Phase 1's.

## Real-time discipline (to implement and document in the code)

| Rule | How |
|---|---|
| Bounded-time callbacks | fixed-size state; prediction and update are O(1) in history length; no loops over unbounded containers |
| No heap allocation in the hot path | fixed-size Eigen types (`Matrix<double,15,15>`); ring buffers as `std::array`; `EIGEN_RUNTIME_NO_MALLOC` asserted in debug builds; loaned messages where the middleware supports them |
| Explicit executor design | IMU propagation in its own MutuallyExclusive group; measurement updates in a second group; publishing in a third; MultiThreadedExecutor with exactly those threads; a lock-free SPSC queue between propagation and update |
| Out-of-order measurements | a short IMU history ring buffer, so late flow or VIO samples are fused at their own timestamp |
| Timestamp gating | Seen in Phase 1 when PX4's uXRCE-DDS timesync re-converged mid-flight: one `vehicle_local_position` sample (of about 1700) was stamped with raw boot time, and the timesync-corrected clock then stepped **back** about 0.31 s. The filter must reject absurd jumps, and treat a small backward step as a clock discontinuity (re-anchor) rather than a negative `dt`. Alternatively, run the filter on `timestamp_sample` deltas from PX4's monotonic clock |
| Watchdogs | per-callback timing statistics published as diagnostics, and a warning when a deadline is missed |

A separate `robot_localization` launch file can run on the same bag files as a
baseline for comparison.
