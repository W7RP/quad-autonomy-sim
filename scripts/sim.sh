#!/usr/bin/env bash
# Start the simulated vehicle: Micro XRCE-DDS agent + PX4 SITL + Gazebo.
#
#   scripts/sim.sh                 # GUI, default world, interactive pxh> console
#   scripts/sim.sh --headless      # no Gazebo GUI (CI, or a slow GPU passthrough)
#   scripts/sim.sh --world walls   # any world in PX4's Tools/simulation/gz/worlds
#
# Equivalent to `make px4_sitl gz_x500` plus the agent, but runs the already-built
# binary directly (no rebuild check) and applies firmware/params/*.params.
# Ctrl+C (or `shutdown` in pxh>) stops everything this script started.
#
# Hardware swap: on a real vehicle this whole script is replaced by the flight
# controller itself and `MicroXRCEAgent serial ...` on the companion computer.
set -euo pipefail
source "$(dirname "$0")/env.sh"

headless=0
world="default"
model="x500"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) headless=1 ;;
    --world) world="$2"; shift ;;
    --model) model="$2"; shift ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

px4_bin="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$px4_bin" ]] || die "PX4 SITL not built: cd $PX4_DIR && make px4_sitl"
command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not found: run scripts/setup/03_build_workspace.sh"

# Project parameters: shared ones first, then SITL-only relaxations.
project_params=()
for params in offboard_common.params sitl_only.params; do
  while read -r name value _; do
    [[ -z "$name" || "$name" == \#* ]] && continue
    project_params+=("$name=$value")
  done < "$REPO_ROOT/firmware/params/$params"
done

# They are applied twice, on purpose:
#  1. as PX4_PARAM_* env vars, which rcS applies early in boot, so they are in
#     effect before most modules start;
#  2. again via the px4-param client once boot has finished. rcS applies the env
#     vars BEFORE the airframe/rc.* scripts run, and PX4 does not record a
#     `param set` to a value equal to the current default as a user change. A
#     later `param set-default` then silently overrides it. This bit us with
#     NAV_DLL_ACT 0 (compiled default 0, raised to 2 later in SITL boot).
for kv in "${project_params[@]}"; do export "PX4_PARAM_${kv%%=*}=${kv#*=}"; done

apply_params_post_boot() {
  local bin="$PX4_DIR/build/px4_sitl_default/bin"
  # The logger is the last module rcS starts, so "logger running" == boot done.
  for _ in $(seq 1 240); do
    "$bin/px4-logger" status >/dev/null 2>&1 && break
    sleep 0.5
  done
  local kv
  for kv in "${project_params[@]}"; do
    "$bin/px4-param" set "${kv%%=*}" "${kv#*=}" >/dev/null 2>&1 \
      || warn "could not set ${kv%%=*}"
  done
  log "applied ${#project_params[@]} project parameters after boot"
}

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  # PX4 starts gz sim itself; make sure no server/GUI outlives the session.
  pkill -f "gz sim.*${world}.sdf" 2>/dev/null || true
  pkill -f "gz sim -g" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if pgrep -x MicroXRCEAgent >/dev/null; then
  warn "MicroXRCEAgent already running, reusing it"
else
  log "starting MicroXRCEAgent on udp4 :8888"
  MicroXRCEAgent udp4 -p 8888 >"${QUAD_LOG_DIR:-/tmp}/xrce_agent.log" 2>&1 &
  pids+=($!)
fi

export PX4_SIM_MODEL="gz_${model}"
export PX4_GZ_WORLD="$world"
export GZ_IP=127.0.0.1
((headless)) && export HEADLESS=1

log "starting PX4 SITL ($PX4_SIM_MODEL, world=$world, headless=$headless)"
cd "$PX4_DIR/build/px4_sitl_default/rootfs"
apply_params_post_boot >&2 &
pids+=($!)
if [[ -t 0 ]]; then
  "$px4_bin"          # interactive pxh> console; Ctrl+C reaches the whole group
else
  # Daemon mode when not attached to a terminal (demo scripts, CI). Run it in the
  # background and `wait`, so a SIGTERM to this script runs cleanup() immediately
  # instead of being deferred until PX4 exits on its own.
  "$px4_bin" -d &
  pids+=($!)
  wait "$!"
fi
