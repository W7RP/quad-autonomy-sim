#!/usr/bin/env bash
# Phase 2 demo, end to end: fly the x500_flow over the textured world while our
# ESKF runs in shadow mode, then score it (and PX4's EKF2) against ground truth.
#
#   scripts/demo_phase2.sh              # with Gazebo GUI
#   scripts/demo_phase2.sh --headless   # no GUI
#   SQUARE_SIDE=12 ALTITUDE=4 SPEED=3 scripts/demo_phase2.sh --headless   # other path
#
# Output: logs/phase2_<timestamp>/ with px4.log, eskf.log, node.log,
# diagnostics.txt (the estimator's real-time stats at the end of the flight),
# the recorded CSVs, metrics.json and estimation.png.
# Exit code 0 only if the flight completed AND the ESKF met the acceptance
# thresholds in scripts/eval_estimation.py.
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

sim_args=("$@")
demo_logdir phase2
logdir="$QUAD_LOG_DIR"
trap demo_cleanup EXIT
# A trapped INT/TERM would run the handler and then carry on with the script
# (seen: Ctrl-C stopped the simulator, then the demo went on to start the next
# step). Exit instead; the EXIT trap cleans up once.
trap 'exit 130' INT TERM
log "logs -> $logdir"

demo_start_sim "$logdir" --model x500_flow --world flow_field "${sim_args[@]}"
log "waiting for PX4 <-> ROS 2 bridge"
demo_wait_for_message /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus 90
ok "bridge up"

# Estimator first, while the vehicle is still on the ground: it aligns from
# ~1.2 s of stationary IMU data before it starts publishing.
if pgrep -x eskf_node >/dev/null; then
  die "an eskf_node is already running (pid $(pgrep -x eskf_node | tr '\n' ' ')); stop it first"
fi
demo_start_node "$logdir/eskf.log" quad_estimation eskf_node --ros-args \
  --params-file "$(ros2 pkg prefix quad_estimation)/share/quad_estimation/config/eskf.yaml"
demo_wait_for_message /eskf/odometry_ned px4_msgs/msg/VehicleOdometry 30
ok "ESKF aligned and publishing"

demo_record /eskf/odometry_ned px4_msgs/msg/VehicleOdometry "$logdir/est.csv"
demo_record /fmu/out/vehicle_local_position_groundtruth_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/gt_pos.csv"
demo_record /fmu/out/vehicle_attitude_groundtruth px4_msgs/msg/VehicleAttitude "$logdir/gt_att.csv"
demo_record /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/ekf2_pos.csv"
demo_record /fmu/out/vehicle_attitude px4_msgs/msg/VehicleAttitude "$logdir/ekf2_att.csv"
demo_record /fmu/out/estimator_status_flags px4_msgs/msg/EstimatorStatusFlags "$logdir/ekf2_flags.csv"
# Raw estimator inputs, so the flight can be replayed offline through the same
# pipeline (scripts/prepare_replay.py + eskf_replay).
demo_record /fmu/out/sensor_combined px4_msgs/msg/SensorCombined "$logdir/imu.csv"
demo_record /fmu/out/sensor_optical_flow px4_msgs/msg/SensorOpticalFlow "$logdir/flow.csv"
demo_record /fmu/out/distance_sensor px4_msgs/msg/DistanceSensor "$logdir/range.csv"
demo_record /fmu/out/vehicle_magnetometer px4_msgs/msg/VehicleMagnetometer "$logdir/mag.csv"
sleep 2  # recorders subscribe

# printf %.2f: ROS parameters are typed, and "3" would arrive as an integer.
side="$(printf %.2f "${SQUARE_SIDE:-8}")"
alt="$(printf %.2f "${ALTITUDE:-3}")"
speed="$(printf %.2f "${SPEED:-2}")"
log "flying a ${side} m square at ${alt} m, ${speed} m/s (PX4's EKF2 flies; the ESKF only watches)"
set +e
ros2 run quad_offboard offboard_square --ros-args \
  --params-file "$(ros2 pkg prefix quad_offboard)/share/quad_offboard/config/square_mission.yaml" \
  -p square_side_m:="$side" -p altitude_m:="$alt" -p cruise_speed_mps:="$speed" \
  2>&1 | tee "$logdir/node.log"
flight_rc=${PIPESTATUS[0]}
set -e

sleep 2
# /diagnostics is published at 1 Hz; allow for CLI start-up and discovery.
timeout 15 ros2 topic echo --no-daemon --once /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  >"$logdir/diagnostics.txt" 2>/dev/null || warn "could not capture /diagnostics"
demo_cleanup
demo_pids=()

log "evaluating against ground truth"
set +e
python3 "$REPO_ROOT/scripts/eval_estimation.py" "$logdir" --check
eval_rc=$?
set -e
grep -E "key: (imu_cb_(mean|p99|max)_us|imu_cb_overruns|hot_path_allocations|sensor_thread_allocations|clock_steps|absurd_stamps|imu_gaps)" \
  -A1 "$logdir/diagnostics.txt" 2>/dev/null | grep -vE "^--" | paste - - | sed -E 's/ *key: //; s/ *value: / = /' || true

if ((flight_rc != 0)); then die "flight did not complete (offboard node exit $flight_rc)"; fi
if ((eval_rc != 0)); then die "ESKF missed acceptance thresholds"; fi
ok "Phase 2 demo complete"
