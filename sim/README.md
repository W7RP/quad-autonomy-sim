# sim/

Gazebo Harmonic worlds and models owned by this project.

- **Phase 1–2** use PX4's stock `default` world and the stock `x500` or
  `x500_flow` models from `PX4-Autopilot/Tools/simulation/gz`. Nothing custom is
  needed yet.
- **Phase 3** adds `worlds/cluttered.sdf`, and possibly a model variant with a
  depth camera or LiDAR, in `models/`.
- **Phase 4** adds `worlds/demo_final.sdf` with movable obstacles.

Custom worlds are started with `gz sim` first. PX4 then attaches to the running
world (`px4-rc.gzsim` detects it), so PX4's own tree stays unmodified.
