# Running it

Setup, and how to run each phase by hand or as a scored demo. Tested on WSL2
with Ubuntu 22.04; the exact versions are in [environment.md](environment.md),
troubleshooting in [setup_wsl2.md](setup_wsl2.md).

## Setup

**WSL2 + Ubuntu 22.04** (Windows side, once, in an admin PowerShell):
`wsl --install -d Ubuntu-22.04`. Windows 11 includes WSLg, so Gazebo's GUI
works without an X server. Keep the repo in the Linux filesystem (`~/…`), not
under `/mnt/c`: builds on the Windows mount are many times slower.

Then, inside Ubuntu:

```bash
git clone <this repo> ~/projects/quad-autonomy-sim && cd ~/projects/quad-autonomy-sim

./scripts/setup/01_install_ros2_humble.sh      # ROS 2 Humble + colcon (apt)       [sudo]
./scripts/setup/02_install_px4_toolchain.sh    # PX4 v1.17.0 + ubuntu.sh (Gazebo)  [sudo]
# open a new shell, then:
./scripts/setup/03_build_workspace.sh          # PX4 patches + SITL build, XRCE agent, ros2_ws
./scripts/setup/04_install_perception_deps.sh  # ros_gz bridge + RTAB-Map (apt)   [sudo]
```

Each script is safe to re-run and skips work already done. The pinned
versions (PX4 tag, px4_msgs branch, agent tag) live in
[scripts/setup/common.sh](../scripts/setup/common.sh).

Every new shell needs the environment:

```bash
source scripts/env.sh     # ROS 2 + workspace overlay + agent on PATH
```

Unit tests (all packages):

```bash
cd ros2_ws && colcon test && colcon test-result --verbose
```

## Scored demos

Each demo starts everything, flies, records, scores the flight against
Gazebo's ground truth, and exits 0 only if it met its thresholds. Add
`--headless` to run without the Gazebo GUI. Output goes to `logs/phaseN_<ts>/`.

| demo | what it does | main outputs |
|---|---|---|
| `scripts/demo_phase1.sh` | offboard square flight | `track.csv`, `track.png` |
| `scripts/demo_phase2.sh` | flight with the ESKF in shadow mode; `SQUARE_SIDE=12 ALTITUDE=4 SPEED=3` for the held-out path | `metrics.json`, `estimation.png`, raw sensor CSVs for `eskf_replay` |
| `scripts/demo_phase3.sh` | two mapping laps; `ODOM_SOURCE=gt` maps on ground truth (diagnostic) | `rtabmap.db`, `cloud.ply`, `map.png`, `map_metrics.json` |
| `scripts/demo_phase4.sh` | map, plan, fly, replan around a dropped obstacle | `flythrough.mp4`, `mission.png`, `mission_metrics.json`, `events.jsonl` |

`rtabmap-databaseViewer logs/phase3_<ts>/rtabmap.db` browses a map.

## By hand

Use `scripts/sim.sh` rather than a bare `make px4_sitl gz_x500`: it also starts
the XRCE-DDS agent and applies `firmware/params`, without which PX4 refuses to
arm with no ground station connected.

**Phase 1, offboard square.**

```bash
./scripts/sim.sh                                   # terminal 1 (--headless for no GUI)
ros2 run quad_offboard offboard_square --ros-args \
  --params-file ros2_ws/src/quad_offboard/config/square_mission.yaml   # terminal 2
```

**Phase 2, the estimator** (keep the vehicle still ~2 s while it aligns):

```bash
./scripts/sim.sh --model x500_flow --world flow_field                  # terminal 1
ros2 run quad_estimation eskf_node --ros-args \
  --params-file ros2_ws/src/quad_estimation/config/eskf.yaml           # terminal 2
ros2 run quad_offboard offboard_square --ros-args \
  --params-file ros2_ws/src/quad_offboard/config/square_mission.yaml   # terminal 3
# watch /eskf/odometry and /diagnostics
```

**Phase 3, mapping.**

```bash
./scripts/sim.sh --model x500_mapper --world cluttered                 # terminal 1
ros2 launch quad_perception mapping.launch.py rviz:=true               # terminal 2
ros2 run quad_offboard offboard_square --ros-args \
  --params-file ros2_ws/src/quad_offboard/config/square_mission.yaml \
  -p "route_nea:=[0.0, 10.0, 1.8, 10.0, 10.0, 1.8, 10.0, 0.0, 1.8, 0.0, 0.0, 1.8]" \
  -p laps:=2 -p yaw_mode:=travel -p cruise_speed_mps:=1.5              # terminal 3
```

**Phase 4, autonomy.**

```bash
./scripts/sim.sh --model x500_mapper --world demo_final                # terminal 1
ros2 launch quad_planning autonomy.launch.py rviz:=true                # terminal 2
ros2 run quad_offboard offboard_square --ros-args \
  --params-file ros2_ws/src/quad_offboard/config/square_mission.yaml \
  -p route_source:=planner -p altitude_m:=1.8 -p cruise_speed_mps:=1.5 \
  -p yaw_mode:=travel                                                  # terminal 3
```

In RViz, add a Path display on `/planner/path` to see the plan in force, and
`ros2 topic echo /planner/events` shows every planning decision. To move the
obstacle yourself (x east, y north, metres from the start):

```bash
gz service -s /world/demo_final/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'name: "intruder", position: {x: 5, y: 5.5, z: 1.3}, orientation: {w: 1}'
```

Other missions: copy `ros2_ws/src/quad_planning/config/mission_demo.yaml`,
edit `goals_enu` (x east, y north, z up, from the start point) and pass
`mission:=<file>` to the launch.
