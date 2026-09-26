# Phase 3: Perception + SLAM (plan)

**Status: not started.**

## Goal

Build a 3D map live while flying a scripted path through a cluttered world.

## Plan

1. **Sensor.** Start with PX4's `gz_x500_depth` (OAK-D-style depth camera), and
   consider `gz_x500_lidar_*` later. Check on WSLg first: depth rendering needs a
   working GPU path on the gz *server* (see docs/setup_wsl2.md).
2. **Bridge.** `ros_gz_bridge` for image, depth, camera_info and clock. On Humble,
   the Harmonic-matched packages are `ros-humble-ros-gzharmonic`, from the OSRF apt
   repo (not the default `ros-humble-ros-gz`, which targets Fortress).
3. **Odometry input to SLAM.** PX4's odometry (`/fmu/out/vehicle_odometry`,
   converted from NED/FRD to ENU/FLU) at first; the Phase 2 filter's output later.
4. **SLAM.** RTAB-Map (`ros-humble-rtabmap-ros`) in RGB-D mode. It handles 3D maps
   and loop closure out of the box, and it exports an OctoMap for Phase 4.
   Cartographer's 3D mode is more LiDAR-oriented. It's kept as an option if we move
   to a LiDAR.
5. **World.** `sim/worlds/cluttered.sdf` with pillars, boxes and a wall with a gap.
   It's launched by starting gz with the custom world first; PX4 then attaches to
   the running world (`px4-rc.gzsim` detects it).
6. **Demo.** Fly a scripted lawnmower path through the world using the Phase 1
   follower, record a bag, and save the RTAB-Map database and an exported point
   cloud.
