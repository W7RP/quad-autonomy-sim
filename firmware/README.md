# firmware/

PX4 configuration for this project. The PX4 source itself is not vendored: it's
cloned at a pinned tag (`PX4_VERSION` in `scripts/setup/common.sh`, currently
`v1.17.0`) to `~/PX4-Autopilot`, and the patches below are applied to it by
`scripts/setup/03_build_workspace.sh` (idempotently).

## Parameters

| File | Used by |
|---|---|
| `params/common.params` | SITL: applied by `scripts/sim.sh` at every boot. Hardware: apply the same values via QGC |
| `params/sitl_only.params` | SITL only: no ground station (`NAV_DLL_ACT 0`), PX4's WMM magnetometer simulator, pinned magnetometer calibration, simulated-world declination, no GPS delay compensation (`EKF2_GPS_DELAY 0`: PX4's simulated GPS has no latency) |
| `params/hardware_uxrce_dds.params.example` | hardware only: uXRCE-DDS over TELEM2 or Ethernet |

The airframes are PX4's stock `4001_gz_x500` (Phase 1) and `4021_gz_x500_flow`
(Phase 2, no GPS) in SITL. On hardware, pick the airframe that matches the
frame, then apply the params above.

## PX4 patches (`px4_patches/`)

| Patch | Why | On hardware? |
|---|---|---|
| `0001-uxrce-dds-export-estimation-topics.patch` | Adds `sensor_optical_flow`, `distance_sensor` and `vehicle_magnetometer` (the Phase 2 estimator's inputs) plus the SITL ground-truth topics to the uXRCE-DDS topic list. This is PX4's intended way to add bridge topics. | Yes, for the estimator inputs (the ground-truth topics simply never publish) |
| `0002-gz-bridge-use-wmm-mag-sim.patch` | With `SENS_EN_MAGSIM=1`, gz_bridge skips Gazebo Harmonic's magnetometer so that PX4's `sensor_mag_sim` (WMM field, ground-truth attitude) is the only magnetometer. Gazebo's is not usable in tilted flight (evidence in docs/phase2_state_estimation.md). | No (SITL-only code) |

Each patch is a plain `git diff` against the pinned tag. If a PX4 bump makes
one fail to apply, the setup script stops and names it.

The patches modify PX4, so they are covered by PX4's BSD 3-Clause license
rather than this repository's MIT license.
