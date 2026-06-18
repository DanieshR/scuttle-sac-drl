#!/usr/bin/env bash
# start-router.sh — start the single zenoh router for the deployment.
#
# Run on the BASE STATION only. There is exactly ONE router on the network; every
# ROS node (here and on the Pi) is a zenoh session that connects to it.
#
#   ./deploy/scripts/start-router.sh
#
# The default router config already listens on tcp/[::]:7447 (all interfaces), so
# the Pi can reach it at tcp/<this-machine-ip>:7447 — no router config edit needed.
# Tell the Pi which IP that is with ./deploy/scripts/whoami-net.sh (run here).
set -euo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
  source "${HOME}/ros2_ws/install/setup.bash"
fi

if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
  echo "rmw_zenoh_cpp not installed. Install it first:" >&2
  echo "  sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp" >&2
  exit 1
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "Base station IP for the Pi to connect to: $(dirname "$0")/whoami-net.sh -> $("$(dirname "$0")/whoami-net.sh")"
echo "Starting zenoh router (rmw_zenohd) on tcp/[::]:7447, ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ..."
exec ros2 run rmw_zenoh_cpp rmw_zenohd
