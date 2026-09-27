# quad_perception (Phase 3)

Live RGB-D mapping: Gazebo camera to ROS, PX4 odometry to TF, RTAB-Map. The
design, results and known limitations are in
[docs/phase3_perception_slam.md](../../../docs/phase3_perception_slam.md).

| file | what |
|---|---|
| `src/px4_odometry_bridge.cpp` | PX4 `/fmu/out/vehicle_odometry` (NED/FRD) to `/odom` + TF `odom -> base_link` (ENU/FLU), stamped with the ROS (sim) clock on arrival |
| `include/quad_perception/odometry_conversion.hpp` | the ROS-free conversion, unit-tested in `test/` |
| `launch/mapping.launch.py` | bridge, static camera TFs, odometry bridge, RTAB-Map, optional RViz |
| `config/gz_bridge.yaml` | Gazebo to ROS topics: `/clock`, colour, depth, camera info |
| `config/gz_bridge_gt_odom.yaml` | **sim-only** ground-truth odometry for the `odom_source:=gt` diagnostic |
| `config/rtabmap.yaml` | RTAB-Map parameters, each non-default one commented |
| `config/mapping.rviz` | map cloud, odometry, camera image |

```bash
# with scripts/sim.sh --model x500_mapper --world cluttered running:
ros2 launch quad_perception mapping.launch.py rviz:=true database:=/tmp/map.db
```

The camera mount in `launch/mapping.launch.py` (`CAMERA_XYZ`, `CAMERA_RPY`)
must match `rgbd_link`'s pose in `sim/models/x500_mapper/model.sdf`. On a real
vehicle it is the measured extrinsic calibration of the camera.
