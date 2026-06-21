#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${G1PILOT_WS:-${HOME}/g1pilot_ws}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 Humble was not found at /opt/ros/${ROS_DISTRO}."
  exit 1
fi

if [ ! -f "${WS}/.venv/bin/activate" ]; then
  echo "Workspace venv not found: ${WS}/.venv"
  echo "Run scripts/setup_humble_user_workspace.sh first."
  exit 1
fi

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${WS}/.venv/bin/activate"
if [ -f "${WS}/deps/env_humble_full.sh" ]; then
  # shellcheck disable=SC1091
  source "${WS}/deps/env_humble_full.sh"
fi

rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true

cd "${WS}"
python -m colcon build --symlink-install "$@"
