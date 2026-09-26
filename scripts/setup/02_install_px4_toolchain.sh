#!/usr/bin/env bash
# Step 3: clone PX4-Autopilot at the pinned release and run its own Tools/setup/ubuntu.sh.
# ubuntu.sh installs the build toolchain (incl. the NuttX ARM toolchain for real
# flight-controller builds) and the matching Gazebo release (Harmonic on 22.04).
# Gazebo is deliberately NOT installed separately: a mismatched gz version is the
# most common cause of `make px4_sitl gz_x500` failing.
# ubuntu.sh calls sudo internally.
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_jammy

if [[ ! -d "$PX4_DIR/.git" ]]; then
  log "Cloning PX4-Autopilot $PX4_VERSION into $PX4_DIR"
  git clone --branch "$PX4_VERSION" --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
  ok "PX4-Autopilot already present at $PX4_DIR ($(git -C "$PX4_DIR" describe --tags --always))"
fi

log "Running PX4 Tools/setup/ubuntu.sh"
bash "$PX4_DIR/Tools/setup/ubuntu.sh"

command -v gz >/dev/null || die "gz not found after ubuntu.sh"
ok "Gazebo: $(gz sim --version 2>/dev/null | head -1)"
ok "cmake: $(cmake --version | head -1)"
warn "ubuntu.sh may have changed group membership / PATH: open a new shell before building"
