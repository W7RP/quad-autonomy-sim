# WSL2 setup notes and troubleshooting

The main instructions are in the root README. This page collects the WSL2-specific
issues.

**Gazebo GUI is black, very slow, or crashes.** WSLg uses GPU passthrough through
Mesa's D3D12 driver. Check `glxinfo -B | grep renderer` (from `mesa-utils`). If it
shows `llvmpipe`, you're on software rendering: it works, but slowly. Options:
update the Windows GPU driver, run `export LIBGL_ALWAYS_SOFTWARE=1` to force a
stable software path, or run `scripts/sim.sh --headless` and skip the GUI. The
physics server doesn't need a GPU for Phase 1–2. Phase 3's depth camera needs
rendering on the server too, so check this before starting Phase 3.

**Memory.** By default WSL2 gets 50% of host RAM. A parallel PX4 build plus
Gazebo is comfortable in 8 GB or more. If builds get OOM-killed, lower the
parallelism (`JOBS=4 ./scripts/setup/03_build_workspace.sh`) or raise `memory=` in
`%UserProfile%\.wslconfig` on the Windows side.

**Files on /mnt/c.** Don't put the repo, PX4-Autopilot or ros2_ws under `/mnt/c`.
The 9P file share makes builds 5–10× slower and breaks file permissions.

**DDS discovery.** Everything runs on one host, so the default Fast DDS
configuration works as-is. If `ros2 topic list` is empty while PX4 reports the
uXRCE-DDS client connected, check that `ROS_DOMAIN_ID` matches `UXRCE_DDS_DOM_ID`
(both 0 by default), and try `ros2 daemon stop` to clear a stale discovery cache.

**NumPy 2 vs apt matplotlib.** PX4's `ubuntu.sh` pip-installs NumPy 2 into
`~/.local`, which breaks Ubuntu 22.04's apt `python3-matplotlib`
(`AttributeError: _ARRAY_API not found`). Fix it with
`pip3 install --user "matplotlib>=3.9"`. The ROS 2 CLI isn't affected.

**Arming refused with bare `make px4_sitl gz_x500`.** Without a ground station, SITL
refuses to arm ("Preflight Fail: No connection to the GCS") unless `NAV_DLL_ACT`
is 0. `scripts/sim.sh` sets it. See docs/environment.md, issue 1.

**Stale Gazebo processes.** If a previous run was killed hard, a `gz sim` server
may still hold the world, and PX4 will attach to it. Clean up with
`pkill -f "gz sim"`.

**PX4 parameters persist** in `PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*.bson`.
`scripts/sim.sh` re-applies the values in `firmware/params` at every boot, so they
always win. To reset everything else, delete those files.
