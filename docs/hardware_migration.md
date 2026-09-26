# Moving from SITL to hardware

The project is set up so this is a transport and config change, not a rewrite.

| Concern | SITL | Hardware | What changes in this repo |
|---|---|---|---|
| Flight controller | `px4` POSIX binary | Pixhawk-class FC running the **same PX4 tag** on NuttX (`make px4_fmu-v6x_default` etc.; the NuttX toolchain comes from `ubuntu.sh`) | nothing |
| Parameters | `scripts/sim.sh` applies `firmware/params/*.params` at every boot | same values via QGroundControl or the MAVLink console | nothing: one file is the source of truth |
| FC ↔ companion link | UDP 127.0.0.1:8888 | UART (TELEM2) or Ethernet | set `UXRCE_DDS_CFG` etc. (see `firmware/params/hardware_uxrce_dds.params.example`), run `MicroXRCEAgent serial --dev … -b 921600` |
| Companion computer | this WSL2 box | Jetson, Raspberry Pi 5 or NUC running Ubuntu 22.04 + Humble | the same `scripts/setup/01` and `03` (skip the PX4 SITL build) |
| Sensors for Phase 2–4 | Gazebo plugins bridged with `ros_gz` | real drivers publishing the same message types and frames | per-sensor launch file only |
| Clock | Gazebo lockstep | wall clock | nodes use the ROS clock and never assume sim time |

Rules that keep this true:

- Nodes never talk to Gazebo directly for anything a real vehicle wouldn't have.
  Ground truth is used **only** by evaluation tooling (Phase 2 error plots), never
  by the control or estimation path.
- No node hard-codes `/fmu/...` topic names. Names come from `px4_topic<Msg>()` plus a
  `px4_namespace` parameter (multi-vehicle SITL and hardware both use namespaces).
- Safety behaviour (abort when PX4 leaves OFFBOARD, never re-engage automatically)
  is tested in SITL first, because on hardware it's what a safety pilot relies on.
