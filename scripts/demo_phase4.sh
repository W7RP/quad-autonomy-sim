#!/usr/bin/env bash
# Phase 4 demo, end to end: map -> plan -> fly -> replan.
#
# x500_mapper flies a mission of goals through sim/worlds/demo_final.sdf. Every
# path is planned by quad_planning (RRT* over RTAB-Map's map + live depth).
# The mission laps the field (building the map), then crosses it diagonally
# around the mapped obstacles. Partway along that leg, scripts/scenario_intruder.py
# drops a new obstacle onto the planned path, and the planner has to notice and
# replan. A fixed camera records the flythrough.
#
#   scripts/demo_phase4.sh              # with Gazebo GUI
#   scripts/demo_phase4.sh --headless   # no GUI
#
# Output: logs/phase4_<ts>/ with flythrough.mp4, events.jsonl (every plan and
# goal), intruder.json, mission.png, mission_metrics.json and the logs.
# Exit code 0 only if the mission completed AND met the thresholds in
# scripts/eval_mission.py.
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

sim_args=("$@")
demo_logdir phase4
logdir="$QUAD_LOG_DIR"
trap demo_cleanup EXIT
# A trapped INT/TERM would run the handler and then carry on with the script
# (seen: Ctrl-C stopped the simulator, then the demo went on to start the next
# step). Exit instead; the EXIT trap cleans up once.
trap 'exit 130' INT TERM
log "logs -> $logdir"

demo_start_sim "$logdir" --model x500_mapper --world demo_final "${sim_args[@]}"
log "waiting for PX4 <-> ROS 2 bridge"
demo_wait_for_message /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus 120
ok "bridge up"

demo_start_launch "$logdir/autonomy.log" quad_planning autonomy.launch.py \
  database:="$logdir/rtabmap.db" events_file:="$logdir/events.jsonl" \
  debug_dump_dir:="$logdir" rviz:="${RVIZ:-false}"
demo_wait_for_message /camera/depth/image_raw sensor_msgs/msg/Image 90
ok "mapping + planner up"

demo_record /fmu/out/vehicle_local_position_groundtruth_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/gt_pos.csv"
demo_record /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/ekf2_pos.csv"
demo_record /fmu/out/sensor_combined px4_msgs/msg/SensorCombined "$logdir/imu.csv"
demo_record /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus "$logdir/status.csv"

# The world change (orchestration, not autonomy).
python3 "$REPO_ROOT/scripts/scenario_intruder.py" --world demo_final --leg 4 \
  --log "$logdir/intruder.json" >"$logdir/scenario.log" 2>&1 &
demo_pids+=($!)

record_video() {  # start|stop
  local req
  if [[ "$1" == start ]]; then
    req="start: true, format: \"mp4\", save_filename: \"$logdir/flythrough.mp4\""
  else
    req="stop: true"
  fi
  gz service -s /flythrough/record_video --reqtype gz.msgs.VideoRecord \
    --reptype gz.msgs.Boolean --timeout 5000 --req "$req" >/dev/null 2>&1 \
    || warn "video recorder did not answer ($1)"
}
sleep 3
record_video start

log "flying the planned mission"
set +e
ros2 run quad_offboard offboard_square --ros-args \
  --params-file "$(ros2 pkg prefix quad_offboard)/share/quad_offboard/config/square_mission.yaml" \
  -p route_source:=planner -p altitude_m:=1.8 -p cruise_speed_mps:=1.5 -p yaw_mode:=travel \
  -p px4_timeout_s:=180.0 2>&1 | tee "$logdir/node.log"
flight_rc=${PIPESTATUS[0]}
set -e

record_video stop
sleep 2
demo_stop_launch 60
demo_cleanup
demo_pids=()

log "scoring the mission"
set +e
python3 "$REPO_ROOT/scripts/eval_mission.py" "$logdir" --check
eval_rc=$?
set -e
if ((flight_rc != 0)); then die "flight did not complete (offboard node exit $flight_rc)"; fi
if ((eval_rc != 0)); then die "mission missed acceptance thresholds"; fi
ok "Phase 4 demo complete"
