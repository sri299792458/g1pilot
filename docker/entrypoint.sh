#!/usr/bin/env bash
set -e

USE_ROBOT="${USE_ROBOT:-true}"

if [ "${USE_ROBOT}" = "true" ] && [ -z "$G1_INTERFACE" ]; then
    echo "ERROR: G1_INTERFACE environment variable is not set."
    echo "Set it to your network interface, e.g.: G1_INTERFACE=eno2"
    echo "For offline/RViz-only testing, set USE_ROBOT=false."
    exit 1
fi

# Run as interactive shell so .bashrc is sourced (matches manual workflow exactly).
exec bash -ic '
cd /ros2_ws &&
./cbuild &&
if [ "${USE_ROBOT}" = "true" ]; then source setup_uri.sh ${G1_INTERFACE}; fi &&
source install/setup.bash &&
ros2 launch g1pilot bringup_launcher.launch.py \
  use_robot:=${USE_ROBOT} \
  enable_collision_avoidance:=${ENABLE_COLLISION_AVOIDANCE:-false}
'
#unset RMW_IMPLEMENTATION &&
