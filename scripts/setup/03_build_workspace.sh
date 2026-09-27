#!/usr/bin/env bash
# Steps 4+5 (no sudo): build PX4 SITL, build the Micro XRCE-DDS agent into
# $XRCE_AGENT_PREFIX (~/.local), fetch px4_msgs/px4_ros_com, colcon build the
# workspace. Safe to re-run.
set -euo pipefail
source "$(dirname "$0")/common.sh"

[[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble missing: run 01_install_ros2_humble.sh"
command -v gz >/dev/null || die "Gazebo missing: run 02_install_px4_toolchain.sh"
[[ -d "$PX4_DIR/.git" ]] || die "PX4 not cloned at $PX4_DIR: run 02_install_px4_toolchain.sh"
jobs="${JOBS:-$(nproc)}"

# --- PX4 SITL ------------------------------------------------------------------
# Project patches to PX4 (firmware/px4_patches, see its README). Idempotent:
# a patch that already reverse-applies cleanly is already in the tree.
for patch in "$REPO_ROOT"/firmware/px4_patches/*.patch; do
  name="$(basename "$patch")"
  if git -C "$PX4_DIR" apply --reverse --check "$patch" 2>/dev/null; then
    ok "PX4 patch already applied: $name"
  else
    git -C "$PX4_DIR" apply "$patch" || die "PX4 patch does not apply: $name (PX4 version changed?)"
    ok "applied PX4 patch: $name"
  fi
done

log "Building PX4 SITL ($(git -C "$PX4_DIR" describe --tags --always))"
make -C "$PX4_DIR" px4_sitl -j"$jobs"
ok "PX4 SITL built: $PX4_DIR/build/px4_sitl_default/bin/px4"

# --- Micro XRCE-DDS agent ------------------------------------------------------
# Built WITHOUT ROS sourced: its superbuild vendors its own Fast-DDS/Fast-CDR,
# and letting CMake find ROS's copies instead is a known source of build breaks.
if [[ -x "$XRCE_AGENT_PREFIX/bin/MicroXRCEAgent" ]]; then
  ok "MicroXRCEAgent already installed in $XRCE_AGENT_PREFIX"
else
  log "Building Micro-XRCE-DDS-Agent $XRCE_AGENT_VERSION into $XRCE_AGENT_PREFIX"
  [[ -d "$XRCE_AGENT_SRC/.git" ]] || git clone --branch "$XRCE_AGENT_VERSION" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$XRCE_AGENT_SRC"
  env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    cmake -S "$XRCE_AGENT_SRC" -B "$XRCE_AGENT_SRC/build" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$XRCE_AGENT_PREFIX" \
      -DCMAKE_INSTALL_RPATH="$XRCE_AGENT_PREFIX/lib"
  env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    cmake --build "$XRCE_AGENT_SRC/build" -j"$jobs"
  cmake --install "$XRCE_AGENT_SRC/build"
  ok "MicroXRCEAgent installed: $XRCE_AGENT_PREFIX/bin/MicroXRCEAgent"
fi

# --- ROS 2 workspace -------------------------------------------------------------
# External packages go in ros2_ws/src/external (git-ignored), pinned in common.sh.
ext="$REPO_ROOT/ros2_ws/src/external"
mkdir -p "$ext"
clone_pinned() {  # <dir> <url> <branch> [commit]
  if [[ -d "$ext/$1/.git" ]]; then
    ok "$1 present ($(git -C "$ext/$1" rev-parse --short HEAD))"
    return
  fi
  # Blobless clone: full commit history (so a pinned commit behind the branch
  # head is reachable) without downloading every historical file.
  git clone --branch "$3" --filter=blob:none "$2" "$ext/$1"
  if [[ -n "${4:-}" ]]; then
    git -C "$ext/$1" -c advice.detachedHead=false checkout "$4"
  fi
}
clone_pinned px4_msgs https://github.com/PX4/px4_msgs.git "$PX4_MSGS_BRANCH" "$PX4_MSGS_COMMIT"
clone_pinned px4_ros_com https://github.com/PX4/px4_ros_com.git "$PX4_ROS_COM_BRANCH" "$PX4_ROS_COM_COMMIT"

log "colcon build"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "$REPO_ROOT/ros2_ws"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --parallel-workers "$jobs"
ok "workspace built: source scripts/env.sh"
