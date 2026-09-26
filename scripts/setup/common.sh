# Shared helpers and pinned versions for the setup scripts. Sourced, not executed.

# Pinned versions. Bump these together: px4_msgs must match the PX4 release,
# otherwise the uXRCE-DDS message definitions drift and topics silently stop matching.
export PX4_VERSION="${PX4_VERSION:-v1.17.0}"
export PX4_MSGS_BRANCH="${PX4_MSGS_BRANCH:-release/1.17}"
export PX4_ROS_COM_BRANCH="${PX4_ROS_COM_BRANCH:-main}"   # no release/1.17 branch exists upstream
# Exact commits verified working (docs/environment.md). Branch heads move; these don't.
# Set to "" to take the branch head instead.
export PX4_MSGS_COMMIT="${PX4_MSGS_COMMIT-86d8239e962f6939e05c3737784f60c02fa884db}"
export PX4_ROS_COM_COMMIT="${PX4_ROS_COM_COMMIT-86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af}"
export XRCE_AGENT_VERSION="${XRCE_AGENT_VERSION:-v2.4.3}"
export ROS_DISTRO_EXPECTED="humble"

# Locations. PX4 and the agent live outside the repo: they are large, have their
# own git history, and are toolchain rather than project source.
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export XRCE_AGENT_SRC="${XRCE_AGENT_SRC:-$HOME/Micro-XRCE-DDS-Agent}"
export XRCE_AGENT_PREFIX="${XRCE_AGENT_PREFIX:-$HOME/.local}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export REPO_ROOT

log()  { printf '\033[1;34m[setup]\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m[ ok  ]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn ]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fail ]\033[0m %s\n' "$*" >&2; exit 1; }

require_jammy() {
  # shellcheck disable=SC1091
  . /etc/os-release
  [[ "${VERSION_CODENAME:-}" == "jammy" ]] || die "Ubuntu 22.04 (jammy) required, found ${PRETTY_NAME:-unknown}"
}
