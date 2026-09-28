# quad_planning (Phase 4)

A C++20 3D planner that flies the vehicle between goals over the Phase 3 map:
voxel occupancy from RTAB-Map's map plus the live depth camera, RRT* with
shortcut smoothing, continuous path validation, and replanning. Paths go to
`quad_offboard` (`route_source:=planner`), which flies them through the
Phase 1 offboard interface. Design, results and limitations:
[docs/phase4_autonomy.md](../../../docs/phase4_autonomy.md).

| file | what |
|---|---|
| `include/quad_planning/voxel_grid.hpp`, `src/voxel_grid.cpp` | fixed-size 3D occupancy grid: mark, box dilation (separable, O(cells)), segment checks. Allocates once, at construction |
| `include/quad_planning/rrt_star.hpp`, `src/rrt_star.cpp` | anytime RRT* with a fixed node pool and a time budget; greedy shortcut smoothing |
| `src/planner_node.cpp` | the ROS node: inputs, grid rebuild, mission, validation, replanning, events |
| `config/planner.yaml` | grid, margins, flight band, arena, RRT* budget (each value commented) |
| `config/mission_demo.yaml` | the demo's goals (odom frame, ENU) |
| `launch/autonomy.launch.py` | Phase 3 mapping (loop closure off) + the planner |
| `test/test_planning.cpp` | grid, dilation vs brute force, RRT* through a door, determinism, budget |

Both library classes are ROS-free, so they are unit-tested without a simulator.

**Topics.**

| direction | topic | type | notes |
|---|---|---|---|
| in | `/odom` | `nav_msgs/Odometry` | vehicle position (from `px4_odometry_bridge`) |
| in | `/camera/depth/image_raw`, `/camera/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | live depth, projected with TF at the image stamp |
| in | `/rtabmap/cloud_obstacles` | `sensor_msgs/PointCloud2` | RTAB-Map's obstacle map (latched), frame `map` |
| out | `/planner/path` | `nav_msgs/Path` (latched) | the path in force, frame `odom`; **empty = hold position** |
| out | `/planner/mission_complete` | `std_msgs/Bool` (latched) | all goals reached |
| out | `/planner/events` | `std_msgs/String` | one JSON object per plan / goal (also written to `events_file`) |

```bash
# with scripts/sim.sh --model x500_mapper --world demo_final running:
ros2 launch quad_planning autonomy.launch.py rviz:=true events_file:=/tmp/events.jsonl
ros2 run quad_offboard offboard_square --ros-args -p route_source:=planner \
  -p altitude_m:=1.8 -p cruise_speed_mps:=1.5 -p yaw_mode:=travel
```

**Tests:** `cd ros2_ws && colcon test --packages-select quad_planning && colcon test-result --verbose`
