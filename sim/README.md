# sim/

Gazebo Harmonic worlds and models owned by this project. `scripts/sim.sh --world
<name>` looks in `sim/worlds` first, then in PX4's `Tools/simulation/gz/worlds`;
`sim/models` is always on Gazebo's resource path.

| World / model | Used by | What |
|---|---|---|
| PX4's `default` + `x500` | Phase 1 | stock |
| `worlds/flow_field.sdf` + `models/flow_ground` + PX4's `x500_flow` | Phase 2 | PX4's default world with the grey ground replaced by a tiled, high-contrast texture. The simulated optical-flow sensor is a real camera plus OpenCV feature tracking; over the flat grey ground PX4's own EKF2 diverged by 70 m. |
| `worlds/cluttered.sdf` + `models/x500_mapper` | Phase 3 | `flow_field` plus 11 textured obstacles around a 10 m square route, **generated** by `tools/generate_cluttered_world.py` (which also enforces 1.2 m route clearance). `eval_map.py` reads it back as ground truth. `x500_mapper` is the x500 plus a forward RGB-D camera. |
| `worlds/demo_final.sdf` (planned) | Phase 4 | movable obstacles |

**Project vehicles.** A model directory with a `px4_airframe` file (e.g.
`x500_mapper`, which contains `4001`) is spawned by `scripts/sim.sh` itself.
PX4 then attaches to it with that airframe (`PX4_GZ_MODEL_NAME`), so PX4's tree
needs no new airframe.

The ground texture is generated, not drawn: `tools/generate_ground_texture.py`
(seeded, so reproducible; seamless, multi-scale). Custom worlds keep PX4's
default physics, lighting and geodetic origin, so results stay comparable.
