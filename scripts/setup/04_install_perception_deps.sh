#!/usr/bin/env bash
# Phase 3 dependencies (needs sudo): the ROS 2 <-> Gazebo Harmonic bridge and
# RTAB-Map. Safe to re-run.
#
# ros-humble-ros-gzharmonic comes from the Gazebo (OSRF) apt repo that PX4's
# ubuntu.sh already added. The ROS repo's ros-humble-ros-gz targets Gazebo
# Fortress, and the two cannot be installed together.
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_jammy

pkgs=(ros-humble-ros-gzharmonic ros-humble-rtabmap-ros)
missing=()
for p in "${pkgs[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
if ((${#missing[@]} == 0)); then
  ok "already installed: ${pkgs[*]}"
  exit 0
fi

if dpkg -l 'ros-humble-ros-gz*' 2>/dev/null | grep -q '^ii  ros-humble-ros-gz-'; then
  die "Fortress ros-gz packages are installed; remove them before installing ros-humble-ros-gzharmonic"
fi

log "installing: ${missing[*]}"
sudo apt-get update
sudo apt-get install -y "${missing[@]}"

for p in "${pkgs[@]}"; do
  ok "$(dpkg-query -W -f='${Package} ${Version}' "$p")"
done
