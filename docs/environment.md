# Verified environment

The exact environment Phase 1 was built and flown in (2026-09-26). If something
breaks after an upgrade, diff against this first.

## Host

| Item | Version |
|---|---|
| Windows host | WSL2 with WSLg (Gazebo GUI verified rendering in a WSLg window) |
| WSL kernel | 6.18.33.2-microsoft-standard-WSL2 |
| Distro | Ubuntu 22.04.5 LTS (jammy) |
| Compiler | gcc 11.4.0 (C++20), cmake 3.22.1 |
| Python | 3.10.12; NumPy 2.2.6 (pip, `~/.local`, from PX4's `ubuntu.sh`); matplotlib 3.10.9 (pip, `~/.local`, see issue 2) |

## Stack

| Component | Version | How it was installed |
|---|---|---|
| ROS 2 | **Humble**: `ros-humble-desktop 0.10.0-1jammy.20260908`, rclcpp 16.0.21, rmw_fastrtps_cpp 6.2.10, Fast-DDS 2.6.12 | apt, via `ros2-apt-source 1.3.0` (`scripts/setup/01_install_ros2_humble.sh`) |
| colcon | `python3-colcon-common-extensions 0.3.0` | apt (same script) |
| PX4 Autopilot | **v1.17.0**, commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | git clone + `Tools/setup/ubuntu.sh` (`02_install_px4_toolchain.sh`) |
| Gazebo | **Harmonic**: `gz-harmonic 1.0.0`, gz-sim **8.15.0** | installed by PX4's `ubuntu.sh` |
| NuttX toolchain | arm-none-eabi-gcc 10.3.1 (2021.07) | installed by PX4's `ubuntu.sh` (hardware builds only) |
| px4_msgs | branch `release/1.17`, commit `86d8239e962f6939e05c3737784f60c02fa884db` | `03_build_workspace.sh` |
| px4_ros_com | branch `main`, commit `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` | `03_build_workspace.sh` |
| Micro XRCE-DDS Agent | **v2.4.3** (`7362281`), vendored Fast-DDS 2.14.7 / Fast-CDR 2.2.8 | built from source into `~/.local` (`03_build_workspace.sh`, no sudo) |

## Verification results

| Step | Result | Notes |
|---|---|---|
| 1. ROS 2 Humble (apt) | ✅ | the sudo steps were run by the user in their own terminal |
| 2. colcon | ✅ | |
| 3. PX4 clone + `ubuntu.sh` | ✅ | installed Gazebo Harmonic. Gazebo was not installed separately |
| 4. `make px4_sitl gz_x500` | ✅ | Gazebo server + GUI up, `x500_0` spawned, "Ready for takeoff!" about 13 s after launch |
| 5. uXRCE-DDS bridge | ✅ | `ros2 topic list` shows `/fmu/out/vehicle_odometry` (about 100 Hz), `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position_v1`, `/fmu/in/*` |
| Phase 1 unit tests | ✅ | `colcon test --packages-select quad_offboard`: 0 failures, 0 compiler warnings |
| Phase 1 demo | ✅ | `demo_phase1.sh --headless`: armed, climbed to 3.05 m, flew a 5.35 × 5.44 m square (5 m commanded), landed 0.26 m from the start, auto-disarmed, exit 0 |

## Issues hit during bring-up (and fixes)

1. **Arming denied: "Preflight Fail: No connection to the GCS".** In SITL,
   `NAV_DLL_ACT` ends up at 2 (data-link-loss → Return) after boot, and no GCS is
   connected, so PX4 refuses to arm. Setting it through `PX4_PARAM_NAV_DLL_ACT=0`
   alone did **not** work. rcS applies those env vars before the airframe scripts
   run. PX4 doesn't record a set to a value equal to the current default as a user
   change, so the later `set-default 2` wins. Fix: `firmware/params/sitl_only.params`,
   re-applied by `scripts/sim.sh` through `px4-param set` once boot has finished.
   Running bare `make px4_sitl gz_x500` skips this, so arming is refused unless the
   value was saved by an earlier `sim.sh` run.
2. **NumPy 2 vs apt matplotlib.** PX4's `ubuntu.sh` pip-installs NumPy 2.2.6 into
   `~/.local`, which shadows Ubuntu's NumPy 1.21 and breaks the apt `matplotlib`
   (`_ARRAY_API not found`). ROS 2 CLI tools were unaffected. Only plotting is.
   Fix (applied): `pip3 install --user "matplotlib>=3.9"`, which installed 3.10.9 and
   also pulled pyparsing 3.3.3 and contourpy 1.3.2 into `~/.local`. Neither PX4
   nor ROS 2 Humble imports pyparsing; the ROS CLI, colcon and PX4 builds were
   re-checked afterwards. `track_summary.py` still detects the broken combination
   and prints the hint, for other machines.
3. **Timesync step mid-flight.** In the verified demo run, PX4 published one
   `vehicle_local_position` sample stamped with raw boot time (36.58 s instead
   of about 1.79e9 s), and the corrected clock then stepped back about 0.31 s,
   consistent with the uXRCE-DDS timesync re-converging. It didn't affect
   Phase 1, which uses no timestamps for control. `track_summary.py` drops the
   absurd sample and keeps the rest. The Phase 2 estimator must handle this
   explicitly (docs/phase2_state_estimation.md).
4. **`ros2 topic hz` on Humble has no `--qos-reliability` flag.** It isn't needed:
   `hz` matches PX4's best-effort publishers as-is. `ros2 topic echo` does take the
   flag.
