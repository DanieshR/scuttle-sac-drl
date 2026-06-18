#!/usr/bin/env python3
"""
basestation_bringup.launch.py — the BASE-STATION half of the distributed offload (Phase 3).

RUN ON THE BASE STATION (this laptop). Brings up the non-real-time workloads moved
off the Pi: SLAM (produces map->odom + /map), frontier exploration (consumes /map,
sends navigate_to_pose goals), and optionally GDM. The robot-side stack
(sensors/motor/odom/Nav2) runs from robot_bringup.launch.py on the Pi.

  source deploy/scripts/zenoh-env.sh base        # in this shell
  ./deploy/scripts/start-router.sh               # in another shell (leave running)
  ros2 launch deploy/launch/basestation_bringup.launch.py

We launch slam_toolbox directly here (rather than including scuttle_nav/slam.launch.py)
because that stock launch HARDCODES use_sim_time:=true — wrong for a real-clock,
chrony-synced deployment. The plan is explicit: do NOT set use_sim_time. So everything
below pins use_sim_time:=False.

GDM: the live gas-mapping nodes (scuttle_base/map_throttle + gmrf_gas_mapping/gmrf_node)
currently exist only on the Pi. To run GDM here you must first port those packages to
the base station and colcon build them. Until then leave enable_gdm:=false (default).
The gas SENSOR source stays on the robot and publishes /gdm/gas_1 across the link.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('scuttle_nav')
    slam_config = os.path.join(pkg_nav, 'config', 'mapper_params_online_async.yaml')

    enable_gdm = LaunchConfiguration('enable_gdm')
    enable_gdm_arg = DeclareLaunchArgument(
        'enable_gdm', default_value='false',
        description='Run GDM here. Requires gmrf_gas_mapping + scuttle_base ported to the base station.'
    )

    # ---- SLAM (slam_toolbox async, lifecycle) — real clock ----
    slam_node = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_config, {'use_sim_time': False}],
        remappings=[('/scan', '/scan')],
    )
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_node),
        transition_id=Transition.TRANSITION_ACTIVATE))

    # ---- Frontier (explore_lite) — real clock ----
    frontier_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'frontier.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # ---- GDM (optional; needs ported packages) ----
    # map_throttle decouples GMRF from SLAM's frequent /map republishes; gmrf_node builds
    # the gas grid from /map_throttled + /gdm/gas_1 (the latter crosses from the robot).
    gdm_map_throttle = Node(
        package='scuttle_base', executable='map_throttle', name='map_throttle',
        output='screen', condition=IfCondition(enable_gdm),
        parameters=[{'input_topic': '/map', 'output_topic': '/map_throttled', 'period_sec': 30.0}],
    )
    gdm_gmrf = Node(
        package='gmrf_gas_mapping', executable='gmrf_node', name='gmrf_node',
        output='screen', condition=IfCondition(enable_gdm),
        parameters=[{
            'use_sim_time': False, 'frame_id': 'map',
            'occupancy_map_topic': '/map_throttled', 'sensor_topic': '/gdm/gas_1',
            'observation_topic': '/gdm/obs_unused', 'output_csv_folder': '',
            'exec_freq': 2.0, 'cell_size': 0.5,
            'min_sensor_val': 0.0, 'max_sensor_val': 110.0,
            'GMRF_lambdaPrior': 5.0, 'GMRF_lambdaObs': 0.5, 'GMRF_lambdaObsLoss': 0.01,
        }],
    )

    return LaunchDescription([
        enable_gdm_arg,
        slam_node, slam_configure, slam_activate,
        frontier_launch,
        gdm_map_throttle, gdm_gmrf,
    ])
