# sim/

Gazebo Harmonic worlds and models owned by this project. `scripts/sim.sh --world
<name>` looks in `sim/worlds` first, then in PX4's `Tools/simulation/gz/worlds`;
`sim/models` is always on Gazebo's resource path.

| World / model | Used by | What |
|---|---|---|
| PX4's `default` + `x500` | Phase 1 | stock |
| `worlds/flow_field.sdf` + `models/flow_ground` + PX4's `x500_flow` | Phase 2 | PX4's default world with the grey ground replaced by a tiled, high-contrast texture. The simulated optical-flow sensor is a real camera plus OpenCV feature tracking; over the flat grey ground PX4's own EKF2 diverged by 70 m. |
| `worlds/cluttered.sdf` (planned) | Phase 3 | obstacles for mapping |
| `worlds/demo_final.sdf` (planned) | Phase 4 | movable obstacles |

The ground texture is generated, not drawn: `tools/generate_ground_texture.py`
(seeded, so reproducible; seamless, multi-scale). Custom worlds keep PX4's
default physics, lighting and geodetic origin, so results stay comparable.
