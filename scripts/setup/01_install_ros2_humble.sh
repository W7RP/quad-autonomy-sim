#!/usr/bin/env bash
# Step 1+2: ROS 2 Humble (binary apt install, per docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
# plus colcon. Needs sudo. Safe to re-run: skips work that is already done.
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_jammy

if dpkg -s ros-humble-desktop >/dev/null 2>&1 && dpkg -s python3-colcon-common-extensions >/dev/null 2>&1; then
  ok "ros-humble-desktop and colcon already installed, nothing to do"
  exit 0
fi

log "Configuring UTF-8 locale"
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

log "Enabling the Ubuntu universe repository"
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  log "Installing ros2-apt-source (ROS apt repo + signing key)"
  ros_apt_ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [[ -n "$ros_apt_ver" ]] || die "could not determine latest ros-apt-source release"
  tmp_deb="$(mktemp --suffix=.deb)"
  curl -fL -o "$tmp_deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_ver}/ros2-apt-source_${ros_apt_ver}.jammy_all.deb"
  sudo dpkg -i "$tmp_deb"
  rm -f "$tmp_deb"
fi

log "Installing ROS 2 Humble desktop, dev tools and colcon"
sudo apt-get update
# The ROS docs recommend upgrading first on 22.04: some ROS packages depend on
# newer systemd/udev builds, and installing without upgrading can remove core packages.
sudo apt-get upgrade -y
sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions

# Verify
# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u
command -v ros2 >/dev/null || die "ros2 not on PATH after install"
command -v colcon >/dev/null || die "colcon not on PATH after install"
ok "ROS_DISTRO=$ROS_DISTRO, $(dpkg-query -W -f='ros-humble-ros-base ${Version}' ros-humble-ros-base)"
ok "colcon: $(command -v colcon)"
