#!/usr/bin/env bash
# Phase 1 demo, end to end: start sim, fly the square, land, save the track.
#
#   scripts/demo_phase1.sh              # with Gazebo GUI
#   scripts/demo_phase1.sh --headless   # no GUI
#
# Output goes to logs/phase1_<timestamp>/ (px4.log, node.log, track.csv, track.png).
# Exit code 0 only if the node reports the full mission completed and landed.
set -euo pipefail
source "$(dirname "$0")/env.sh"

sim_args=("$@")
logdir="$REPO_ROOT/logs/phase1_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$logdir"
export QUAD_LOG_DIR="$logdir"

pids=()
cleanup() {
  # SIGTERM, not SIGINT: background jobs of a non-interactive shell ignore SIGINT.
  for ((i=${#pids[@]}-1; i>=0; i--)); do kill "${pids[i]}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

log "logs -> $logdir"
# stdin from /dev/null puts PX4 in daemon mode (no pxh> console).
"$REPO_ROOT/scripts/sim.sh" "${sim_args[@]}" </dev/null >"$logdir/px4.log" 2>&1 &
pids+=($!)

status_topic="/fmu/out/vehicle_status_v1"
log "waiting for PX4 <-> ROS 2 bridge ($status_topic)"
for _ in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -qx "$status_topic"; then break; fi
  kill -0 "${pids[0]}" 2>/dev/null || die "sim exited early, see $logdir/px4.log"
  sleep 1
done
ros2 topic list 2>/dev/null | grep -qx "$status_topic" || die "bridge not up after 90 s, see $logdir/px4.log"
ok "bridge up"

ros2 topic echo --csv --qos-reliability best_effort \
  /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition >"$logdir/track.csv" &
pids+=($!)

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
