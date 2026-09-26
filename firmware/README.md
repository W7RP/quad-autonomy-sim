# firmware/

PX4 configuration for this project. The PX4 source itself is not vendored: it's
cloned at a pinned tag (`PX4_VERSION` in `scripts/setup/common.sh`, currently
`v1.17.0`) to `~/PX4-Autopilot`.

| File | Used by |
|---|---|
| `params/offboard_common.params` | SITL: applied by `scripts/sim.sh` at every boot. Hardware: apply the same values via QGC |
| `params/sitl_only.params` | SITL only: relaxes checks that assume a ground station (`NAV_DLL_ACT 0`) |
| `params/hardware_uxrce_dds.params.example` | hardware only: uXRCE-DDS over TELEM2 or Ethernet |

The airframe is PX4's stock `4001_gz_x500` in SITL. On hardware, pick the matching
airframe for the frame, then apply the params above.
