#!/usr/bin/env bash
# rviz-live.sh — open RViz2 on the LIVE distributed SCUTTLE stack (laptop/base station).
#
# Sources ROS + the workspace + zenoh-env (base role) so RViz sees the Pi's topics over
# the Zenoh link, then loads the engineering view: /map, GDM heatmap (/gdm/gmrf_gas_map),
# /scan, TF, RobotModel, and the explore_lite frontier markers (/explore/frontiers —
# only populated while in frontier mode).
#
#   ~/ros2_ws/deploy/scripts/rviz-live.sh
set +u
WS="${WS:-$HOME/ros2_ws}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
source "$WS/deploy/scripts/zenoh-env.sh" base
exec rviz2 -d "$WS/deploy/rviz/scuttle_live.rviz"
