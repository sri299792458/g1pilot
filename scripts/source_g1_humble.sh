#!/usr/bin/env bash

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "This file must be sourced, not executed:"
  echo "  source $0 <network-interface>"
  exit 1
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
IFACE="${1:-${G1_INTERFACE:-}}"
WS="${G1PILOT_WS:-${HOME}/g1pilot_ws}"

if [ -z "${IFACE}" ]; then
  echo "Usage: source ${BASH_SOURCE[0]} <network-interface>"
  echo "Example: source ${BASH_SOURCE[0]} eno1"
  echo "For offline localhost testing: source ${BASH_SOURCE[0]} lo"
  return 1
fi

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 Humble was not found at /opt/ros/${ROS_DISTRO}."
  return 1
fi

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WS}/.venv/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "${WS}/.venv/bin/activate"
fi

if [ -f "${WS}/deps/env_humble_full.sh" ]; then
  # shellcheck disable=SC1091
  source "${WS}/deps/env_humble_full.sh"
fi

if [ -f "${WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi

rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true

export G1_INTERFACE="${IFACE}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

if [ "${IFACE}" = "lo" ]; then
  if command -v ip >/dev/null 2>&1 && ! ip link show lo | grep -q MULTICAST; then
    echo "Loopback multicast is disabled. Enable it with:"
    echo "  sudo ip link set lo multicast on"
  fi
fi

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "G1_INTERFACE=${G1_INTERFACE}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
