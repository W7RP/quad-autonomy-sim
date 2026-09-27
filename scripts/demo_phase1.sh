#!/usr/bin/env bash
# Phase 1 demo, end to end: start sim, fly the square, land, save the track.
#
#   scripts/demo_phase1.sh              # with Gazebo GUI
#   scripts/demo_phase1.sh --headless   # no GUI (extra args go to scripts/sim.sh)
#
# Output goes to logs/phase1_<timestamp>/ (px4.log, node.log, track.csv, track.png).
# Exit code 0 only if the node reports the full mission completed and landed.
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

demo_logdir phase1
logdir="$QUAD_LOG_DIR"
trap demo_cleanup EXIT INT TERM
log "logs -> $logdir"

demo_start_sim "$logdir" "$@"
log "waiting for PX4 <-> ROS 2 bridge"
demo_wait_for_message /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus 90
ok "bridge up"

demo_record /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition "$logdir/track.csv"

log "flying square"
set +e
ros2 run quad_offboard offboard_square --ros-args \
  --params-file "$(ros2 pkg prefix quad_offboard)/share/quad_offboard/config/square_mission.yaml" \
  2>&1 | tee "$logdir/node.log"
rc=${PIPESTATUS[0]}
set -e

sleep 1
python3 "$REPO_ROOT/scripts/track_summary.py" "$logdir/track.csv" --png "$logdir/track.png" || true

if ((rc == 0)); then ok "Phase 1 demo complete"; else warn "node exited with $rc"; fi
exit "$rc"
