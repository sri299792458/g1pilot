#!/usr/bin/env bash
set -euo pipefail

# Distro-aware workspace build for Humble/Jazzy lab machines.

detect_ros_distro() {
  for distro in jazzy humble; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
      echo "${distro}"
      return
    fi
  done
  find /opt/ros -mindepth 2 -maxdepth 2 -name setup.bash 2>/dev/null \
    | sed -n 's#^/opt/ros/\([^/]*\)/setup.bash$#\1#p' \
    | sort \
    | tail -n 1
}

ROS_DISTRO="${ROS_DISTRO:-$(detect_ros_distro)}"
WS="${G1PILOT_WS:-${HOME}/g1pilot_${ROS_DISTRO}_ws}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 ${ROS_DISTRO:-<unset>} was not found at /opt/ros/${ROS_DISTRO}/setup.bash."
  exit 1
fi

if [ ! -f "${WS}/.venv/bin/activate" ]; then
  echo "Workspace venv not found: ${WS}/.venv"
  echo "Run scripts/setup_user_workspace.sh first."
  exit 1
fi

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

# shellcheck disable=SC1091
source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${WS}/.venv/bin/activate"
if [ -f "${WS}/deps/env_${ROS_DISTRO}_full.sh" ]; then
  # shellcheck disable=SC1091
  source_setup_file "${WS}/deps/env_${ROS_DISTRO}_full.sh"
elif [ -f "${WS}/env_ros.sh" ]; then
  # shellcheck disable=SC1091
  source_setup_file "${WS}/env_ros.sh"
fi

rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true

cd "${WS}"
python -m colcon build --symlink-install "$@"
