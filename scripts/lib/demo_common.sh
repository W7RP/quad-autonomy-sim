# Shared plumbing for scripts/demo_*.sh. Sourced, not executed.
# Expects scripts/env.sh to have been sourced (log/ok/warn/die, REPO_ROOT).

demo_pids=()

demo_cleanup() {
  # SIGTERM, not SIGINT: background jobs of a non-interactive shell ignore SIGINT.
  # Reverse order: recorders and nodes first, the simulator last.
  local i pid
  for ((i = ${#demo_pids[@]} - 1; i >= 0; i--)); do
    kill "${demo_pids[i]}" 2>/dev/null || true
  done
  # Give everything 5 s to exit cleanly, then make sure nothing outlives the demo:
  # a stray estimator from a previous run keeps publishing on the same topic.
  for _ in $(seq 1 50); do
    local alive=0
    for pid in "${demo_pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    ((alive)) || break
    sleep 0.1
  done
  for pid in "${demo_pids[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}

# demo_start_node <logfile> <package> <executable> [args...]   (background)
# Runs the node executable directly rather than through `ros2 run`: `ros2 run`
# is a wrapper process, and killing the wrapper can leave the node orphaned
# (seen here: an old eskf_node kept publishing into the next run's topics).
demo_start_node() {
  local logfile="$1" pkg="$2" exe="$3"
  shift 3
  "$(ros2 pkg prefix "$pkg")/lib/$pkg/$exe" "$@" >"$logfile" 2>&1 &
  demo_pids+=($!)
}

# demo_logdir <name>  -> creates logs/<name>_<timestamp>; sets and exports
# QUAD_LOG_DIR (not echoed: a $(...) subshell would lose the export).
demo_logdir() {
  QUAD_LOG_DIR="$REPO_ROOT/logs/${1}_$(date +%Y%m%d_%H%M%S)"
  export QUAD_LOG_DIR
  mkdir -p "$QUAD_LOG_DIR"
}

# demo_start_sim <logdir> [sim.sh args...]
demo_start_sim() {
  local dir="$1"
  shift
  # stdin from /dev/null puts PX4 in daemon mode (no pxh> console).
  "$REPO_ROOT/scripts/sim.sh" "$@" </dev/null >"$dir/px4.log" 2>&1 &
  demo_pids+=($!)
  demo_sim_pid=$!
}

# demo_wait_for_message <topic> <type> <timeout_s>
# Waits for an actual message, not just the topic name: `ros2 topic list` is
# answered by the ROS 2 daemon, which can still remember a previous session.
# All echo calls use --no-daemon: the shared daemon was seen stuck in a broken
# state ("!rclpy.ok()"), silently failing every CLI call that consulted it.
demo_wait_for_message() {
  local topic="$1" type="$2" timeout_s="$3" start=$SECONDS
  while ((SECONDS - start < timeout_s)); do
    if [[ -n "${demo_sim_pid:-}" ]] && ! kill -0 "$demo_sim_pid" 2>/dev/null; then
      die "simulator exited early, see $QUAD_LOG_DIR/px4.log"
    fi
    if timeout 3 ros2 topic echo --no-daemon --once --qos-reliability best_effort "$topic" "$type" \
        >/dev/null 2>&1; then
      return 0
    fi
  done
  die "no data on $topic after ${timeout_s} s (logs: $QUAD_LOG_DIR)"
}

# demo_record <topic> <type> <csv>   (background; stopped by demo_cleanup)
demo_record() {
  ros2 topic echo --no-daemon --csv --qos-reliability best_effort "$1" "$2" >"$3" 2>/dev/null &
  demo_pids+=($!)
}
