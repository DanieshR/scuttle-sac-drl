#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
export DRLNAV_BASE_PATH='/home/turtlebot3_drlnav'
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$DRLNAV_BASE_PATH/src/turtlebot3_simulations/turtlebot3_gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$DRLNAV_BASE_PATH/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib
export ROS_DOMAIN_ID=1

cd $DRLNAV_BASE_PATH

if [ ! -d "install" ]; then
    echo "Building workspace..."
    colcon build --symlink-install
fi

source $DRLNAV_BASE_PATH/install/setup.bash

# Ensure model://scuttle resolves to the real native SDF (scuttle_description),
# not the get_scan_count stub in turtlebot3_gazebo/models (which ament puts first).
export GAZEBO_MODEL_PATH=$DRLNAV_BASE_PATH/install/scuttle_description/share/scuttle_description/models:$GAZEBO_MODEL_PATH

exec "$@"
