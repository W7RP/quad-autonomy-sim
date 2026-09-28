#!/usr/bin/env bash
# Phase 3 demo, end to end: fly the RGB-D x500 two laps through the cluttered
# world while RTAB-Map builds a map, then score the map against the world's
# true geometry.
#
#   scripts/demo_phase3.sh              # with Gazebo GUI
#   scripts/demo_phase3.sh --headless   # no GUI
#   RVIZ=true scripts/demo_phase3.sh    # also watch the map grow in RViz
#   ODOM_SOURCE=gt scripts/demo_phase3.sh --headless
#       SIMULATION-ONLY diagnostic: map on Gazebo's true pose instead of PX4
#       EKF2, to separate odometry errors from mapping errors.
#
# Output: logs/phase3_<timestamp>/ with rtabmap.db (the full map database,
# openable in rtabmap-databaseViewer), cloud.ply, map_metrics.json, map.png,
# and the logs. Exit code 0 only if the flight completed AND the map met the
# thresholds in scripts/eval_map.py.
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

sim_args=("$@")
demo_logdir phase3
logdir="$QUAD_LOG_DIR"
trap demo_cleanup EXIT
# A trapped INT/TERM would run the handler and then carry on with the script
# (seen: Ctrl-C stopped the simulator, then the demo went on to start the next
# step). Exit instead; the EXIT trap cleans up once.
trap 'exit 130' INT TERM
log "logs -> $logdir"

command -v rtabmap-export >/dev/null || die "RTAB-Map missing: run scripts/setup/04_install_perception_deps.sh"
demo_start_sim "$logdir" --model x500_mapper --world cluttered "${sim_args[@]}"
log "waiting for PX4 <-> ROS 2 bridge"
demo_wait_for_message /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus 90
ok "bridge up"

odom_source="${ODOM_SOURCE:-px4}"
demo_start_launch "$logdir/mapping.log" quad_perception mapping.launch.py \
  database:="$logdir/rtabmap.db" rviz:="${RVIZ:-false}" odom_source:="$odom_source"
demo_wait_for_message /camera/depth/image_raw sensor_msgs/msg/Image 60
ok "camera bridged"

# Ground truth and EKF2 positions, to place the map in the world; the IMU
# stream is the reference for undoing timesync clock steps.
demo_record /fmu/out/vehicle_local_position_groundtruth_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/gt_pos.csv"
demo_record /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/ekf2_pos.csv"
demo_record /fmu/out/sensor_combined px4_msgs/msg/SensorCombined "$logdir/imu.csv"
sleep 3

# Two laps of the 10 m square at 1.8 m (sim/tools/generate_cluttered_world.py
# checks 1.2 m clearance to every obstacle), facing the direction of travel so
# the forward camera looks where it is going. North, east, altitude from start.
route="[0.0, 10.0, 1.8,  10.0, 10.0, 1.8,  10.0, 0.0, 1.8,  0.0, 0.0, 1.8]"
log "flying two mapping laps"
set +e
ros2 run quad_offboard offboard_square --ros-args \
  --params-file "$(ros2 pkg prefix quad_offboard)/share/quad_offboard/config/square_mission.yaml" \
  -p "route_nea:=$route" -p laps:=2 -p yaw_mode:=travel -p cruise_speed_mps:=1.5 \
  2>&1 | tee "$logdir/node.log"
flight_rc=${PIPESTATUS[0]}
set -e

log "stopping mapping (RTAB-Map saves its database)"
demo_stop_launch 60
[[ -s "$logdir/rtabmap.db" ]] || die "no RTAB-Map database written, see $logdir/mapping.log"
grep -E "loop closure|Loop closure|Rtabmap.cpp.*loop" -i "$logdir/mapping.log" | tail -3 || true

log "exporting the map cloud"
rtabmap-export --cloud --voxel 0.05 --max_range 8 --output cloud --output_dir "$logdir" "$logdir/rtabmap.db" \
  >"$logdir/export.log" 2>&1 || die "rtabmap-export failed, see $logdir/export.log"
# --cloud appends "_cloud" to the output name.
mv "$logdir/cloud_cloud.ply" "$logdir/cloud.ply" 2>/dev/null || true
[[ -s "$logdir/cloud.ply" ]] || die "no cloud exported, see $logdir/export.log"
demo_cleanup
demo_pids=()

log "scoring the map against the world's true geometry"
set +e
map_origin=ekf2
[[ "$odom_source" == gt ]] && map_origin=start
python3 "$REPO_ROOT/scripts/eval_map.py" "$logdir" --map-origin "$map_origin" --check
eval_rc=$?
set -e
if ((flight_rc != 0)); then die "flight did not complete (offboard node exit $flight_rc)"; fi
if ((eval_rc != 0)); then die "map missed acceptance thresholds"; fi
ok "Phase 3 demo complete"
